// ============================================================
//  Vacation Home Temperature Monitor — T-SIM7000G
//  Board: LilyGO T-SIM7000G (ESP32 + SIM7000G modem)
//
//  Transport priority:
//    1. WiFi  → MQTT over TLS (port 8883)
//    2. Cellular (Hologram) → MQTT over TLS (port 8883)
//
//  GPS (built-in SIM7000G GNSS):
//    - Modem always initialized so GPS works on both WiFi and cellular paths
//    - Location is only included in a payload when the device has moved
//      more than MOVE_THRESHOLD_M meters from the last reported position
//    - While stationary, fixes are averaged using Welford's running mean;
//      once BASELINE_SAMPLES good fixes accumulate the refined position is
//      sent once as a one-time accuracy update
//
//  Time:
//    - WiFi path → NTP synced (pool.ntp.org); timezone = US Central with DST
//    - Cellular path → system clock set from GPS UTC each cycle
//    - Both paths use the POSIX TZ string "CST6CDT,M3.2.0,M11.1.0" so
//      localtime_r() always returns the correct Central time automatically
//
//  Daily summary:
//    - On-device hi/lo tracked across every temperature reading
//    - Transmitted ONCE at 6:00 PM Central on topic MQTT_DAILY_TOPIC
//    - Resets at midnight Central; safe to power-cycle mid-day (sends
//      whatever readings have accumulated when 6 PM arrives)
//
//  Sensor: BMP280 breakout (GY-BMP280) via I2C
//    VCC → 3.3V  |  GND → GND  |  SCL → GPIO22  |  SDA → GPIO21
//    CSB → 3.3V (I2C mode)  |  SDO → GND (address 0x76)
//  Payload includes temperature (°F or °C) and pressure (hPa).
//  Note: BMP280 does not have a humidity sensor (use BME280 for humidity).
//
//  Data savings: only one temperature unit is transmitted per payload.
//  Toggle USE_IMPERIAL below — comment it out to switch to Metric.
//
//  OTA updates:
//    - WiFi path only (SIM7000G does not expose TinyGsmClientSecure).
//    - Device checks GitHub for a new firmware version once per hour.
//    - To release a new version: bump FW_VERSION_* in version.h,
//      commit, then: git tag v1.x.x && git push origin v1.x.x
//    - GitHub Actions builds the binary and updates version.json
//      automatically; devices detect the new version on next check.
// ============================================================

// ─── Unit toggle ─────────────────────────────────────────────────────────────
//   Defined  → sends temp_f / hi_f / lo_f  (Imperial, °F)
//   Commented → sends temp_c / hi_c / lo_c (Metric,   °C)
#define USE_IMPERIAL

// ─── TinyGSM modem selection (must come before TinyGsmClient.h) ──────────────
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 512

#include <Arduino.h>
#include <math.h>            // sin, cos, atan2, sqrt — Haversine
#include <time.h>            // time_t, struct tm, mktime, localtime_r
#include <sys/time.h>        // settimeofday
#include <TinyGsmClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

#include "secrets.h"         // WiFi + MQTT credentials (gitignored)
#include "version.h"         // FW_VERSION_STR — bump before each release
#include "ota_update.h"      // otaInit() / otaLoop() / otaCheckNow()

// ─── OTA config — set to match your GitHub repo ──────────────────────────────
// These can also be set as -D build flags in platformio.ini instead.
#define OTA_GITHUB_USER  "dditzler"
#define OTA_GITHUB_REPO  "vacation-home-temp-monitor"

// ─── WiFi credentials (from secrets.h) ───────────────────────────────────────
const char* WIFI_NETWORKS[][2] = {
  { WIFI_SSID_1, WIFI_PASS_1 },   // home
  { WIFI_SSID_2, WIFI_PASS_2 },   // vacation home
};

// ─── Cellular — Hologram network ─────────────────────────────────────────────
const char* CELLULAR_APN = "hologram";
const char* CELLULAR_PIN = "";           // SIM PIN — leave blank if none

// ─── MQTT broker (HiveMQ Cloud) ──────────────────────────────────────────────
// Production broker (WiFi path — TLS)
const char* MQTT_HOST        = "ad21434501c84aad995bc5621bf77f15.s1.eu.hivemq.cloud";
const int   MQTT_PORT_WIFI   = 8883;

// Cellular test broker — HiveMQ public broker, no TLS, no auth required.
// Confirms the cellular MQTT stack works before we add TLS.
// TODO: replace with SSLClient-wrapped HiveMQ Cloud once confirmed working.
const char* MQTT_HOST_CELL   = "broker.hivemq.com";
const int   MQTT_PORT_CELL   = 1883;
const char* MQTT_USER        = MQTT_USER_VAL;   // from secrets.h
const char* MQTT_PASS        = MQTT_PASS_VAL;   // from secrets.h
const char* MQTT_TOPIC       = "vacation/cust1/il/temp/status";   // realtime
const char* MQTT_DAILY_TOPIC = "vacation/cust1/il/temp/daily";    // hi/lo summary

// ─── Hardware pin assignments ─────────────────────────────────────────────────
// BME280 uses I2C — SDA=GPIO21, SCL=GPIO22 (ESP32 defaults, no pin defines needed)
#define MODEM_TX      27
#define MODEM_RX      26
#define MODEM_PWRKEY   4
#define MODEM_DTR     25

// ─── Timing ──────────────────────────────────────────────────────────────────
// PUBLISH_INTERVAL can be overridden per-environment via a -D build flag
// (e.g. tsim7000g_test sets it to 10000UL for faster feedback).
#ifndef PUBLISH_INTERVAL
#define PUBLISH_INTERVAL  300000UL   // ms between realtime publishes (5 minutes on cellular)
#endif
#define WIFI_TIMEOUT_MS    30000UL   // ms to try WiFi before cellular fallback

// ─── GPS tuning ──────────────────────────────────────────────────────────────
#define MOVE_THRESHOLD_M   50.0      // meters; less than this = stationary
#define BASELINE_SAMPLES   20        // fixes to Welford-average for refined pos
#define GPS_HDOP_MAX        5.0      // reject fixes with HDOP above this (loosened for faster first fix)
#define GPS_INITIAL_WAIT_S 120       // seconds to wait for first fix at startup

// ─── Time zone ───────────────────────────────────────────────────────────────
// POSIX TZ string: US Central Standard/Daylight — handles DST automatically
#define TZ_CENTRAL "CST6CDT,M3.2.0,M11.1.0"

// ─── Temperature offset (self-heating correction) ────────────────────────────
// BMP280 reads high when mounted near the ESP32/modem heat source.
// Tune this value by comparing against a known-good thermometer.
// Set to 0.0f to disable once the sensor is physically separated from the board.
#define TEMP_OFFSET_F  -8.0f

// ─── Sensor ──────────────────────────────────────────────────────────────────
// STUB_TEMP: defined by the tsim7000g_test build environment.
// When set, sensor hardware is skipped entirely and fixed values are injected
// so the modem / GPS / network / MQTT stack can be tested without wiring.
#ifndef STUB_TEMP
Adafruit_BMP280 bme;   // I2C address 0x76 (SDO → GND); chip ID 0x58
#endif

// ─── Transport objects ────────────────────────────────────────────────────────
WiFiClientSecure wifiSecure;
HardwareSerial   SerialAT(1);
TinyGsm          modem(SerialAT);
TinyGsmClient    gsmClient(modem, 0);   // plain TCP — TinyGSM SIM7000 does not
                                        // expose TinyGsmClientSecure; cellular
                                        // path uses port 1883 (unencrypted).

PubSubClient  mqttWifi(wifiSecure);
PubSubClient  mqttCell(gsmClient);
PubSubClient* mqtt = nullptr;

bool          usingCellular = false;
unsigned long lastPublish   = 0;
String        deviceId;

// ─── Time state ──────────────────────────────────────────────────────────────
bool timeSynced = false;   // true once NTP or GPS-UTC has set the system clock

// ─── GPS state ────────────────────────────────────────────────────────────────
double  baselineLat   = 0.0, baselineLon  = 0.0;
int     baselineCount = 0;
bool    baselineSent  = false;

double  lastSentLat   = 0.0, lastSentLon = 0.0;
bool    lastSentValid = false;

bool    locPending = false;
double  pendingLat = 0.0, pendingLon = 0.0;

// Last GPS UTC time (updated by readGpsFix on every successful call)
bool  gpsTimeValid = false;
int   gpsYear = 0, gpsMonth = 0, gpsDay = 0;
int   gpsHour = 0, gpsMin   = 0, gpsSec = 0;

// Last known satellite counts (updated every GPS query, fix or not)
int   gpsVisibleSats = 0, gpsUsedSats = 0;

// ─── Daily hi/lo state ───────────────────────────────────────────────────────
float dailyHi       = -999.0f;  // sentinel = no reading yet this day
float dailyLo       =  999.0f;
int   dailyReadings = 0;         // readings accumulated since last midnight reset
bool  dailySent     = false;     // true once today's 6pm summary has been sent
int   lastDay       = -1;        // day-of-month at last midnight-check (local time)

// ─── Haversine distance (meters) ─────────────────────────────────────────────
double haversineM(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = (lat2 - lat1) * DEG_TO_RAD;
  double dLon = (lon2 - lon1) * DEG_TO_RAD;
  double a    = sin(dLat / 2) * sin(dLat / 2)
              + cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD)
              * sin(dLon / 2) * sin(dLon / 2);
  return R * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

// ─── GPS time → system clock (cellular path only) ────────────────────────────
// Called after each successful GPS fix.  Temporarily sets TZ=UTC so mktime()
// interprets the GPS UTC fields correctly, converts to epoch, then restores
// Central time and calls settimeofday().
void syncTimeFromGPS() {
  if (!gpsTimeValid) return;

  setenv("TZ", "UTC0", 1);
  tzset();

  struct tm t = {};
  t.tm_year  = gpsYear  - 1900;
  t.tm_mon   = gpsMonth - 1;
  t.tm_mday  = gpsDay;
  t.tm_hour  = gpsHour;
  t.tm_min   = gpsMin;
  t.tm_sec   = gpsSec;
  t.tm_isdst = 0;
  time_t epoch = mktime(&t);

  setenv("TZ", TZ_CENTRAL, 1);   // restore Central before any time ops
  tzset();

  struct timeval tv = { epoch, 0 };
  settimeofday(&tv, nullptr);
  timeSynced = true;
}

// ─── Read one GPS fix ─────────────────────────────────────────────────────────
// Always stores the raw UTC time fields in globals (used for clock sync).
// Returns true only if position quality passes the HDOP threshold.
bool readGpsFix(double& lat, double& lon, float& hdop) {
  float fLat, fLon, speed, alt, accuracy;
  int   vsat, usat, year, month, day, hour, minute, sec;

  if (!modem.getGPS(&fLat, &fLon, &speed, &alt,
                    &vsat, &usat, &accuracy,
                    &year, &month, &day, &hour, &minute, &sec)) {
    return false;
  }

  // Always update satellite counts so "no fix" messages can report them
  gpsVisibleSats = vsat;
  gpsUsedSats    = usat;

  // Store GPS UTC time regardless of positional quality; used for clock sync
  if (year > 2020) {
    gpsYear = year; gpsMonth = month; gpsDay = day;
    gpsHour = hour; gpsMin   = minute; gpsSec = sec;
    gpsTimeValid = true;
    if (usingCellular) syncTimeFromGPS();   // keep system clock aligned to GPS
  }

  if (fLat == 0.0f && fLon == 0.0f) return false;
  if (accuracy > GPS_HDOP_MAX)       return false;

  lat  = static_cast<double>(fLat);
  lon  = static_cast<double>(fLon);
  hdop = accuracy;
  return true;
}

// ─── GPS initialisation ───────────────────────────────────────────────────────
void initGPS() {
  // GPS was already enabled early in setup() to start acquiring sats during
  // network connection. Just wait here for the first fix.
  Serial.println("Waiting for GPS fix (already acquiring)...");

  Serial.printf("Waiting up to %d s for initial GPS fix", GPS_INITIAL_WAIT_S);
  unsigned long deadline = millis() + GPS_INITIAL_WAIT_S * 1000UL;

  while (millis() < deadline) {
    double lat, lon;
    float  hdop;
    if (readGpsFix(lat, lon, hdop)) {
      Serial.printf("\nGPS fix: %.5f, %.5f  HDOP: %.1f\n", lat, lon, hdop);
      lastSentLat   = lat;  lastSentLon   = lon;  lastSentValid = true;
      baselineLat   = lat;  baselineLon   = lon;  baselineCount = 1;
      pendingLat    = lat;  pendingLon    = lon;  locPending    = true;
      return;
    }
    // Even without a position fix, readGpsFix may have synced the clock
    delay(2000);
    Serial.print(".");
  }
  Serial.println("\nGPS: no position fix yet — will retry each publish cycle.");
}

// ─── GPS update (called each publish cycle) ───────────────────────────────────
bool updateGPS() {
  double lat, lon;
  float  hdop;

  if (!readGpsFix(lat, lon, hdop)) {
    Serial.printf("GPS: no fix  (sats visible: %d, used: %d)\n",
                  gpsVisibleSats, gpsUsedSats);
    return false;
  }

  if (!lastSentValid) {
    Serial.printf("GPS: first fix %.5f, %.5f  HDOP %.1f\n", lat, lon, hdop);
    lastSentLat = lat;  lastSentLon = lon;  lastSentValid = true;
    baselineLat = lat;  baselineLon = lon;  baselineCount = 1;
    pendingLat  = lat;  pendingLon  = lon;
    return true;
  }

  double distMoved = haversineM(lastSentLat, lastSentLon, lat, lon);

  if (distMoved >= MOVE_THRESHOLD_M) {
    Serial.printf("GPS: moved %.1f m — updating position\n", distMoved);
    lastSentLat   = lat;  lastSentLon   = lon;
    baselineLat   = lat;  baselineLon   = lon;
    baselineCount = 1;    baselineSent  = false;
    pendingLat    = lat;  pendingLon    = lon;
    return true;
  }

  // Stationary — accumulate Welford running mean (reject scatter > threshold/2)
  double driftFromBase = haversineM(baselineLat, baselineLon, lat, lon);
  if (driftFromBase < MOVE_THRESHOLD_M / 2.0) {
    baselineCount++;
    baselineLat += (lat - baselineLat) / static_cast<double>(baselineCount);
    baselineLon += (lon - baselineLon) / static_cast<double>(baselineCount);
    Serial.printf("GPS: sample %d/%d  avg=(%.5f, %.5f)\n",
                  baselineCount, BASELINE_SAMPLES, baselineLat, baselineLon);
  }

  if (baselineCount >= BASELINE_SAMPLES && !baselineSent) {
    Serial.printf("GPS: baseline ready (n=%d) → refined pos (%.5f, %.5f)\n",
                  baselineCount, baselineLat, baselineLon);
    baselineSent  = true;
    lastSentLat   = baselineLat;  lastSentLon  = baselineLon;
    pendingLat    = baselineLat;  pendingLon   = baselineLon;
    return true;
  }

  return false;
}

// ─── Daily hi/lo tracking & 6pm summary ──────────────────────────────────────
// Call every publish cycle with the freshly-read temperature.
void checkDailySummary(float temp) {

  // On WiFi, NTP syncs asynchronously — check whether the clock is valid yet
  if (!timeSynced && !usingCellular) {
    time_t now = time(nullptr);
    if (now > 1000000000UL) timeSynced = true;   // epoch > ~2001: clock is set
  }
  if (!timeSynced) return;   // no time reference yet — skip until we have one

  // ── Update hi/lo with this reading ──────────────────────────────────────
  if (temp > dailyHi) dailyHi = temp;
  if (temp < dailyLo) dailyLo = temp;
  dailyReadings++;

  // ── Get current Central time ─────────────────────────────────────────────
  time_t    now = time(nullptr);
  struct tm ct;
  localtime_r(&now, &ct);     // ct is now in US Central (DST-aware via TZ env)

  // ── Midnight rollover — new day, reset accumulators ──────────────────────
  if (lastDay != -1 && ct.tm_mday != lastDay) {
    Serial.printf("Daily reset: %02d→%02d\n", lastDay, ct.tm_mday);
    dailyHi = temp;  dailyLo = temp;  dailyReadings = 1;  dailySent = false;
  }
  lastDay = ct.tm_mday;

  // ── 6 PM Central — publish today's hi/lo (once per day) ─────────────────
  // Uses >= 18 so a missed cycle during reconnect can't skip the send entirely.
  if (ct.tm_hour >= 18 && !dailySent && dailyReadings >= 1) {
    char dateBuf[12];
    strftime(dateBuf, sizeof(dateBuf), "%Y-%m-%d", &ct);

    char payload[160];
#ifdef USE_IMPERIAL
    snprintf(payload, sizeof(payload),
      "{\"customerId\":\"cust1\",\"houseId\":\"il\",\"mode\":\"daily\","
      "\"date\":\"%s\",\"hi_f\":%.1f,\"lo_f\":%.1f}",
      dateBuf, dailyHi, dailyLo);
#else
    snprintf(payload, sizeof(payload),
      "{\"customerId\":\"cust1\",\"houseId\":\"il\",\"mode\":\"daily\","
      "\"date\":\"%s\",\"hi_c\":%.1f,\"lo_c\":%.1f}",
      dateBuf, dailyHi, dailyLo);
#endif

    Serial.printf("[%s] Daily: %s\n", usingCellular ? "CELL" : "WiFi", payload);
    if (mqtt->publish(MQTT_DAILY_TOPIC, payload, true)) {   // retain=true: broker holds last daily for fresh dashboard loads
      dailySent = true;
    }
    // If publish returns false (e.g. broker dropped), dailySent stays false
    // and the next cycle will retry until it succeeds.
  }
}

// ─── Modem power-on and initialisation ───────────────────────────────────────
bool initModem() {
  Serial.println("Powering on SIM7000G...");

  // LilyGO T-SIM7000G PWRKEY sequence (per official LilyGO reference examples):
  // Drive HIGH ≥ 100 ms, then release LOW — SIM7000G boots in ~5 s.
  // (Earlier code had the polarity reversed which prevented boot.)
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(300);
  digitalWrite(MODEM_PWRKEY, LOW);

  // DTR LOW keeps the modem awake and prevents it entering sleep mode
  pinMode(MODEM_DTR, OUTPUT);
  digitalWrite(MODEM_DTR, LOW);

  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(100);

  // Poll for AT response — modem typically responds within 5-8 s of power-on.
  // Using testAT() rather than a fixed delay makes this robust across
  // board revisions and cold vs. warm boot conditions.
  Serial.print("Waiting for modem");
  bool modemUp = false;
  for (int i = 0; i < 15; i++) {
    delay(1000);
    Serial.print(".");
    if (modem.testAT(500)) { modemUp = true; break; }
  }
  Serial.println();

  if (!modemUp) {
    Serial.println("ERROR: Modem not responding after 15 s.");
    return false;
  }

  // Modem is alive — init() configures without forcing a hardware reset
  // (restart() is for when the modem is already running; here we just powered it on)
  if (!modem.init()) {
    Serial.println("ERROR: Modem init() failed.");
    return false;
  }

  Serial.printf("Modem: %s\n", modem.getModemInfo().c_str());
  return true;
}

// ─── Cellular data connect (modem already initialised) ───────────────────────
bool connectCellular() {
  modem.sendAT(GF("+CNMP=38"));   // LTE only
  modem.waitResponse(10000L);
  modem.sendAT(GF("+CMNB=3"));    // Cat-M + NB-IoT
  modem.waitResponse(10000L);

  Serial.printf("Connecting to Hologram APN: %s\n", CELLULAR_APN);
  if (!modem.gprsConnect(CELLULAR_APN, "", "")) {
    Serial.println("ERROR: Cellular connect failed.");
    return false;
  }
  Serial.printf("Cellular connected. IP: %s\n", modem.localIP().toString().c_str());
  return true;
}

// ─── WiFi connect ────────────────────────────────────────────────────────────
bool connectWiFi() {
#ifdef FORCE_CELLULAR
  Serial.println("WiFi skipped (FORCE_CELLULAR build flag set).");
  return false;
#endif
  int count = sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);
  unsigned long start = millis();

  for (int i = 0; i < count; i++) {
    Serial.printf("Trying WiFi SSID: %s\n", WIFI_NETWORKS[i][0]);
    WiFi.begin(WIFI_NETWORKS[i][0], WIFI_NETWORKS[i][1]);

    while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_TIMEOUT_MS) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("WiFi connected. IP: %s\n", WiFi.localIP().toString().c_str());
      return true;
    }
    Serial.println("Failed, trying next...");
    WiFi.disconnect();
  }
  Serial.println("No WiFi network reachable.");
  return false;
}

// ─── Time initialisation ─────────────────────────────────────────────────────
void initTime() {
  // Set timezone first — affects all subsequent localtime_r() calls
  setenv("TZ", TZ_CENTRAL, 1);
  tzset();

  if (!usingCellular) {
    // WiFi path: kick off NTP sync (async; timeSynced checked in loop)
    Serial.println("Starting NTP sync (pool.ntp.org)...");
    configTime(0, 0, "pool.ntp.org", "time.google.com");

    // Block briefly to see if NTP responds quickly (good WiFi = usually < 2s)
    struct tm t;
    if (getLocalTime(&t, 8000)) {
      char buf[32];
      strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
      Serial.printf("NTP synced: %s Central\n", buf);
      timeSynced = true;
    } else {
      Serial.println("NTP not yet available — will retry in background.");
      // configTime keeps retrying; loop() checks epoch value each cycle
    }
  }
  // Cellular path: clock will be set from GPS UTC via syncTimeFromGPS()
}

// ─── MQTT connect/reconnect ───────────────────────────────────────────────────
void connectMQTT() {
  // WiFi  → HiveMQ Cloud TLS port 8883
  // Cell  → HiveMQ public broker plain TCP port 1883 (temporary test)
  mqtt->setServer(usingCellular ? MQTT_HOST_CELL : MQTT_HOST,
                  usingCellular ? MQTT_PORT_CELL : MQTT_PORT_WIFI);
  // Cellular: 5-minute keepalive to minimise Hologram data charges.
  // WiFi: 60-second keepalive for faster broker disconnect detection.
  mqtt->setKeepAlive(usingCellular ? 300 : 60);

  while (!mqtt->connected()) {
    Serial.printf("Connecting to MQTT [%s]...", usingCellular ? "CELL" : "WiFi");
    if (mqtt->connect(deviceId.c_str(), MQTT_USER, MQTT_PASS)) {
      Serial.println(" OK");
    } else {
      Serial.printf(" failed rc=%d — retry in 5s\n", mqtt->state());
      delay(5000);
    }
  }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Vacation Home Temperature Monitor — T-SIM7000G ===");
  Serial.println("Firmware: " FW_VERSION_STR);

#ifdef USE_IMPERIAL
  Serial.println("Unit mode: Imperial (°F)");
#else
  Serial.println("Unit mode: Metric (°C)");
#endif

#ifdef STUB_TEMP
  Serial.printf("BMP280: STUB MODE — injecting %.1f°F, no sensor required\n",
                (float)STUB_TEMP);
#else
  Wire.begin();   // SDA=GPIO21, SCL=GPIO22 (ESP32 defaults)
  uint8_t bmpAddr = 0;
  if      (bme.begin(0x76)) { bmpAddr = 0x76; }
  else if (bme.begin(0x77)) { bmpAddr = 0x77; }
  else {
    Serial.println("FATAL: BMP280 not found at 0x76 or 0x77 — check wiring.");
    Serial.println("  VCC→3.3V  GND→GND  SCL→GPIO22  SDA→GPIO21");
    while (true) { delay(10000); }
  }
  Serial.printf("BMP280 ready (I2C 0x%02X) — temp + pressure only\n", bmpAddr);
#endif

  // Modem always needed — GPS requires it even on the WiFi data path
  if (!initModem()) {
    Serial.println("FATAL: Modem did not start. Halting.");
    while (true) { delay(10000); }
  }

  // Start GPS immediately after modem init so it acquires satellites
  // in parallel while the network connection is being established.
  // This can save 30-60 seconds of TTFF on a cold start.
  Serial.println("Starting GPS early (acquiring sats during network connect)...");
  modem.enableGPS();

  // Data transport
  if (connectWiFi()) {
    wifiSecure.setInsecure();
    mqtt          = &mqttWifi;
    usingCellular = false;
    deviceId      = "tsim-wifi-" + WiFi.macAddress();
    Serial.println("Transport: WiFi");
  } else {
    WiFi.mode(WIFI_OFF);
    if (!connectCellular()) {
      Serial.println("FATAL: No network available. Halting.");
      while (true) { delay(10000); }
    }
    mqtt          = &mqttCell;
    usingCellular = true;
    deviceId      = "tsim-cell-" + modem.getIMEI();
    Serial.println("Transport: Cellular (Hologram)");
  }

  connectMQTT();
  initTime();   // NTP for WiFi path; GPS-based sync will fill in cellular path
  initGPS();    // Enable GNSS and wait for first fix

  // ── OTA: WiFi path only ───────────────────────────────────────────────────
  // SIM7000G does not expose TinyGsmClientSecure, so HTTPS OTA is only
  // available when connected via WiFi. wifiSecure is already configured above.
  if (!usingCellular) {
    otaInit(wifiSecure);
    Serial.println("OTA: enabled (WiFi path, checks every hour)");
  } else {
    Serial.println("OTA: disabled on cellular path (no SSL support on SIM7000G)");
  }
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  if (!mqtt->connected()) connectMQTT();
  mqtt->loop();

  // ── OTA check (WiFi only, rate-limited to OTA_CHECK_INTERVAL_S) ──────────
  // Device will reboot automatically if a new firmware version is downloaded.
  if (!usingCellular) {
    otaLoop();
  }

  unsigned long now = millis();
  if (now - lastPublish < PUBLISH_INTERVAL) return;
  lastPublish = now;

  // ── Temperature, Humidity, Pressure ─────────────────────────────────────
#ifdef STUB_TEMP
  // Test mode: no sensor needed — stub values set by build flag
  float       temp    = (float)STUB_TEMP;
  const char* tempKey = "temp_f";
#else
  float tempC = bme.readTemperature();   // always °C from sensor

  if (isnan(tempC)) {
    Serial.println("ERROR: BMP280 read failed — check wiring.");
    return;
  }

#  ifdef USE_IMPERIAL
  float       temp    = tempC * 9.0f / 5.0f + 32.0f + TEMP_OFFSET_F;
  const char* tempKey = "temp_f";
#  else
  float       temp    = tempC + (TEMP_OFFSET_F * 5.0f / 9.0f);  // convert offset to °C
  const char* tempKey = "temp_c";
#  endif
#endif

  // ── GPS — movement check, baseline refinement, clock sync ───────────────
  bool hasNewLoc = updateGPS();
  if (locPending) { hasNewLoc = true; locPending = false; }

  // ── Daily hi/lo tracking and 6 PM summary ────────────────────────────────
  checkDailySummary(temp);

  // ── Realtime payload ─────────────────────────────────────────────────────
  char payload[180];
  if (hasNewLoc) {
    snprintf(payload, sizeof(payload),
      "{\"customerId\":\"cust1\",\"houseId\":\"il\",\"mode\":\"realtime\","
      "\"%s\":%.1f,\"sats\":%d,\"lat\":%.5f,\"lon\":%.5f}",
      tempKey, temp, gpsUsedSats, pendingLat, pendingLon);
  } else {
    snprintf(payload, sizeof(payload),
      "{\"customerId\":\"cust1\",\"houseId\":\"il\",\"mode\":\"realtime\","
      "\"%s\":%.1f,\"sats\":%d}",
      tempKey, temp, gpsVisibleSats);
  }

  Serial.printf("[%s] %s\n", usingCellular ? "CELL" : "WiFi", payload);
  mqtt->publish(MQTT_TOPIC, payload, true);   // retain=true: broker holds last reading for fresh dashboard loads
}
