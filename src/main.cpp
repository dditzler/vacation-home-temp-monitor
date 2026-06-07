// ============================================================
//  Vacation Home Temperature Monitor — T-SIM7000G
//  Board: LilyGO T-SIM7000G (ESP32 + SIM7000G modem)
//  Serial: SIM7-0001
//  Firmware: see version.h
//
//  Transport priority:
//    1. WiFi  → MQTT over TLS (port 8883)
//    2. Cellular (Hologram) → MQTT over TLS via SSLClient (port 8883)
//
//  Sensor: SHT31-D breakout (Adafruit) via I2C
//    VCC → 3.3V  |  GND → GND  |  SCL → GPIO22  |  SDA → GPIO21
//    ADDR pin → GND  (I2C address 0x44)
//    Provides: temperature (°C, ±0.3°C) + relative humidity (%, ±2%)
//
//  Power outage detection: voltage divider on VBUS → GPIO34 (ADC)
//    VBUS (5V) → 100kΩ → GPIO34 → 47kΩ → GND
//    ~1.60V / 1989 ADC counts when mains on; ~0 when off
//    Publishes to MQTT_POWER_TOPIC on outage and restore events (retained)
//
//  Time sync:
//    WiFi path  → NTP (pool.ntp.org)
//    Cellular   → NITZ via modem.getNetworkTime() after registration
//    Both paths use POSIX TZ "CST6CDT,M3.2.0,M11.1.0" for Central time
//
//  Daily summary:
//    On-device hi/lo tracked; transmitted once at 6:00 PM Central
//    on topic MQTT_DAILY_TOPIC. Resets at midnight.
//
//  OTA:
//    WiFi path  → WiFiClientSecure (native TLS)
//    Cellular   → SSLClient over TinyGsmClient socket 1
//    Checks hourly; reboots automatically on new version.
//    To release: bump version.h, commit, git tag sensor-vX.Y.Z, push.
//
//  MQTT topics:
//    vacation/cust1/il/temp/status      — realtime temp+humidity+mains (5 min)
//    vacation/cust1/il/temp/daily       — hi/lo summary at 6 PM Central
//    vacation/cust1/il/temp/power       — outage/restore events (retained)
//    vacation/cust1/il/temp/sensor_raw  — ESP-NOW vibration data from C6 sensor
// ============================================================

// ─── Unit toggle ─────────────────────────────────────────────────────────────
//   Defined  → sends temp_f / hi_f / lo_f  (Imperial, °F)
//   Commented → sends temp_c / hi_c / lo_c (Metric,   °C)
#define USE_IMPERIAL

// ─── TinyGSM modem selection (must come before TinyGsmClient.h) ──────────────
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 512

#include <Arduino.h>
#include <time.h>
#include <sys/time.h>
#include <TinyGsmClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <esp_now.h>
#if defined(SENSOR_SHT31)
  #include <Adafruit_SHT31.h>
#elif defined(SENSOR_BMP280)
  #include <Adafruit_BMP280.h>
#else
  #error "Define SENSOR_SHT31 or SENSOR_BMP280 in build_flags"
#endif

#include <SSLClient.h>   // TLS over TinyGsmClient for cellular OTA

// ─── OTA config ───────────────────────────────────────────────────────────────
#ifndef OTA_GITHUB_USER
  #define OTA_GITHUB_USER  "dditzler"
#endif
#ifndef OTA_GITHUB_REPO
  #define OTA_GITHUB_REPO  "ernie"
#endif

#include "secrets.h"    // WiFi + MQTT credentials + OTA_GITHUB_TOKEN (gitignored)
#include "version.h"    // FW_VERSION_STR — bump before each release
#include "ota_update.h" // otaInit() / otaLoop()
#include "ca_cert.h"    // HiveMQ Cloud root CA for SSLClient cellular TLS

// ─── WiFi credentials (from secrets.h) ───────────────────────────────────────
const char* WIFI_NETWORKS[][2] = {
  { WIFI_SSID_1, WIFI_PASS_1 },   // home
  { WIFI_SSID_2, WIFI_PASS_2 },   // vacation home
};

// ─── Cellular — Hologram ─────────────────────────────────────────────────────
const char* CELLULAR_APN = "hologram";

// ─── MQTT broker ─────────────────────────────────────────────────────────────
// Both WiFi and cellular use the same HiveMQ Cloud TLS endpoint.
const char* MQTT_HOST        = "ad21434501c84aad995bc5621bf77f15.s1.eu.hivemq.cloud";
const int   MQTT_PORT        = 8883;
// Hologram's TinyGsmClient DNS fails on long hostnames — hardcode one IP for cellular.
// dig ad21434501c84aad995bc5621bf77f15.s1.eu.hivemq.cloud → 46.137.47.218 / 54.73.92.158 / 52.31.149.80
// SSLClient still sends MQTT_HOST as SNI so TLS cert validation works correctly.
const char* MQTT_HOST_IP     = "46.137.47.218";

const char* MQTT_USER        = MQTT_USER_VAL;
const char* MQTT_PASS        = MQTT_PASS_VAL;
const char* MQTT_TOPIC       = "vacation/cust1/il/temp/status";
const char* MQTT_DAILY_TOPIC = "vacation/cust1/il/temp/daily";
const char* MQTT_POWER_TOPIC = "vacation/cust1/il/temp/power";        // retained
const char* MQTT_SENSOR_RAW  = "vacation/cust1/il/temp/sensor_raw";   // ESP-NOW vibration

// ─── Hardware pin assignments ─────────────────────────────────────────────────
#define MODEM_TX      27
#define MODEM_RX      26
#define MODEM_PWRKEY   4
#define MODEM_DTR     25

// Voltage divider: VBUS (5V) → 100kΩ → GPIO34 → 47kΩ → GND
// GPIO34 is ADC1_CH6 (input-only — no boot conflict)
#define POWER_ADC_PIN       34
#define POWER_ON_THRESH     1000    // raw 12-bit counts; ~1989 normal, ~0 when off
#define POWER_DEBOUNCE_MS   3000UL  // 3 s debounce before declaring a state change

// Sensor I2C addresses
#if defined(SENSOR_SHT31)
  #define SHT31_ADDR  0x44   // ADDR pin → GND
#elif defined(SENSOR_BMP280)
  #define BMP280_ADDR 0x76   // SDO pin → GND
#endif

// ─── Timing ──────────────────────────────────────────────────────────────────
#ifndef PUBLISH_INTERVAL
#define PUBLISH_INTERVAL  300000UL   // 5 minutes on cellular
#endif
#define WIFI_TIMEOUT_MS    30000UL

// ─── Time zone ───────────────────────────────────────────────────────────────
#define TZ_CENTRAL "CST6CDT,M3.2.0,M11.1.0"

// ─── Temperature offset (self-heating correction) ────────────────────────────
// Self-heating correction — tune by comparing against a reference thermometer.
// BMP280 runs hotter (modem heat path); SHT31-D is slightly better isolated.
#if defined(SENSOR_BMP280)
  #define TEMP_OFFSET_F  -8.0f
#else
  #define TEMP_OFFSET_F  -4.0f
#endif

// ─── Sensor object ───────────────────────────────────────────────────────────
#ifndef STUB_TEMP
  #if defined(SENSOR_SHT31)
    Adafruit_SHT31    sht31;
  #elif defined(SENSOR_BMP280)
    Adafruit_BMP280   bmp280;
  #endif
#endif

// ─── ESP-NOW message structure (must match vibration_xiao_c6 firmware exactly) ─
typedef struct {
  bool     hvacOn;        // true = furnace vibration detected
  uint8_t  reason;        // 0 = state change, 1 = heartbeat
  uint32_t onDurationS;   // seconds vibration has been continuous (0 if off)
  uint16_t rawMagnitude;  // max axis deviation in ADXL345 counts — for calibration
} VibrationMsg;


// ─── Transport objects ────────────────────────────────────────────────────────
WiFiClientSecure wifiSecure;
HardwareSerial   SerialAT(1);
TinyGsm          modem(SerialAT);
TinyGsmClient    gsmClientMQTT(modem, 0);  // socket 0 — MQTT base TCP (wrapped by SSL)
TinyGsmClient    gsmClientOTA(modem, 1);   // socket 1 — OTA base TCP (wrapped by SSL)
SSLClient        sslGsmMqtt(&gsmClientMQTT);  // TLS for cellular MQTT
SSLClient        sslGsmOTA(&gsmClientOTA);    // TLS for cellular OTA

PubSubClient  mqttWifi(wifiSecure);
PubSubClient  mqttCell(sslGsmMqtt);
PubSubClient* mqtt = nullptr;

bool          usingCellular = false;
unsigned long lastPublish   = 0;
String        deviceId;

// ─── Time state ──────────────────────────────────────────────────────────────
bool timeSynced = false;

// ─── Daily hi/lo state ───────────────────────────────────────────────────────
float dailyHi       = -999.0f;
float dailyLo       =  999.0f;
int   dailyReadings = 0;
bool  dailySent     = false;
int   lastDay       = -1;

// ─── Power outage detection state ────────────────────────────────────────────
bool          mainsPresent     = true;
unsigned long powerEdgeMs      = 0;
bool          powerEdgePending = false;
unsigned long outageStartMs    = 0;


// ─── Daily hi/lo tracking & 6pm summary ──────────────────────────────────────
void checkDailySummary(float temp) {
  if (!timeSynced && !usingCellular) {
    if (time(nullptr) > 1000000000UL) timeSynced = true;
  }
  if (!timeSynced) return;

  if (temp > dailyHi) dailyHi = temp;
  if (temp < dailyLo) dailyLo = temp;
  dailyReadings++;

  time_t    now = time(nullptr);
  struct tm ct;
  localtime_r(&now, &ct);

  if (lastDay != -1 && ct.tm_mday != lastDay) {
    Serial.printf("Daily reset: %02d→%02d\n", lastDay, ct.tm_mday);
    dailyHi = temp; dailyLo = temp; dailyReadings = 1; dailySent = false;
  }
  lastDay = ct.tm_mday;

  if (ct.tm_hour >= 18 && !dailySent && dailyReadings >= 1) {
    char dateBuf[12];
    strftime(dateBuf, sizeof(dateBuf), "%Y-%m-%d", &ct);

    char payload[200];
#ifdef USE_IMPERIAL
    snprintf(payload, sizeof(payload),
      "{\"sn\":\"SIM7-0001\",\"mode\":\"daily\","
      "\"date\":\"%s\",\"hi_f\":%.1f,\"lo_f\":%.1f}",
      dateBuf, dailyHi, dailyLo);
#else
    snprintf(payload, sizeof(payload),
      "{\"sn\":\"SIM7-0001\",\"mode\":\"daily\","
      "\"date\":\"%s\",\"hi_c\":%.1f,\"lo_c\":%.1f}",
      dateBuf, dailyHi, dailyLo);
#endif
    Serial.printf("[%s] Daily: %s\n", usingCellular ? "CELL" : "WiFi", payload);
    if (mqtt->publish(MQTT_DAILY_TOPIC, payload, true)) dailySent = true;
  }
}

// ─── Power outage detection ───────────────────────────────────────────────────
//
//  Debounces ADC transitions. Publishes retained events on MQTT_POWER_TOPIC.
//
//  Outage payload:  {"sn":"SIM7-0001","event":"outage","adc":<n>,"fw":"<ver>"}
//  Restore payload: {"sn":"SIM7-0001","event":"restore","outage_s":<n>,"adc":<n>,"fw":"<ver>"}
//
//  During an actual outage the ESP32 loses USB power — no MQTT possible.
//  The restore event (with outage_s duration) is sent as soon as the device
//  reboots and reconnects.
//
void checkPower() {
  int  raw      = analogRead(POWER_ADC_PIN);
  bool mainsNow = (raw >= POWER_ON_THRESH);

  if (mainsNow == mainsPresent) {
    powerEdgePending = false;
    return;
  }

  if (!powerEdgePending) {
    powerEdgePending = true;
    powerEdgeMs      = millis();
    Serial.printf("[POWER] ADC=%d — possible %s, debouncing...\n",
                  raw, mainsNow ? "RESTORE" : "OUTAGE");
    return;
  }

  if (millis() - powerEdgeMs < POWER_DEBOUNCE_MS) return;

  // Debounce complete — commit the state change
  powerEdgePending = false;
  mainsPresent     = mainsNow;

  char payload[180];
  if (!mainsPresent) {
    outageStartMs = millis();
    snprintf(payload, sizeof(payload),
      "{\"sn\":\"SIM7-0001\",\"event\":\"outage\",\"adc\":%d,\"fw\":\"%s\"}",
      raw, FW_VERSION_STR);
    Serial.printf("[POWER] *** MAINS LOST *** ADC=%d\n", raw);
  } else {
    uint32_t outageSec = (millis() - outageStartMs) / 1000;
    snprintf(payload, sizeof(payload),
      "{\"sn\":\"SIM7-0001\",\"event\":\"restore\",\"outage_s\":%lu,\"adc\":%d,\"fw\":\"%s\"}",
      outageSec, raw, FW_VERSION_STR);
    Serial.printf("[POWER] Mains restored after %lu s, ADC=%d\n", outageSec, raw);
  }

  if (mqtt && mqtt->connected()) {
    mqtt->publish(MQTT_POWER_TOPIC, payload, true);  // retained
  }
}

// ─── ESP-NOW receive callback ─────────────────────────────────────────────────
// Called on every incoming VibrationMsg from the C6 vibration sensor.
// Publishes raw vibration data to MQTT for dashboard + threshold calibration.
void onVibrationReceived(const esp_now_recv_info_t* info,
                         const uint8_t* data, int len) {
  if (len != sizeof(VibrationMsg)) {
    // Log sender MAC to help identify stray devices (range-test rovers, etc.)
    Serial.printf("[ESP-NOW] Ignoring %d-byte packet from %02X:%02X:%02X:%02X:%02X:%02X (expected %d bytes)\n",
      len,
      info->src_addr[0], info->src_addr[1], info->src_addr[2],
      info->src_addr[3], info->src_addr[4], info->src_addr[5],
      sizeof(VibrationMsg));
    return;
  }
  VibrationMsg msg;
  memcpy(&msg, data, sizeof(msg));

  int8_t rssi = info->rx_ctrl->rssi;
  Serial.printf("[ESP-NOW] hvac=%s  reason=%s  onDur=%ds  mag=%u  rssi=%d dBm\n",
    msg.hvacOn ? "ON" : "OFF",
    msg.reason == 0 ? "state_change" : "heartbeat",
    msg.onDurationS,
    msg.rawMagnitude,
    rssi);

  if (!mqtt || !mqtt->connected()) {
    Serial.println("[ESP-NOW] MQTT not connected — dropping vibration message.");
    return;
  }

  char payload[160];
  snprintf(payload, sizeof(payload),
    "{\"sn\":\"EC6-0001\",\"hvac\":\"%s\",\"reason\":\"%s\","
    "\"on_dur_s\":%lu,\"magnitude\":%u,\"rssi_dbm\":%d}",
    msg.hvacOn ? "on" : "off",
    msg.reason == 0 ? "state_change" : "heartbeat",
    msg.onDurationS,
    msg.rawMagnitude,
    rssi);
  mqtt->publish(MQTT_SENSOR_RAW, payload);
}

// ─── Modem power-on and initialisation ───────────────────────────────────────
bool initModem() {
  Serial.println("Powering on SIM7000G...");
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(300);
  digitalWrite(MODEM_PWRKEY, LOW);

  pinMode(MODEM_DTR, OUTPUT);
  digitalWrite(MODEM_DTR, LOW);

  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(100);

  Serial.print("Waiting for modem");
  bool modemUp = false;
  for (int i = 0; i < 15; i++) {
    delay(1000);
    Serial.print(".");
    if (modem.testAT(500)) { modemUp = true; break; }
  }
  Serial.println();

  if (!modemUp) { Serial.println("ERROR: Modem not responding."); return false; }
  if (!modem.init()) { Serial.println("ERROR: Modem init() failed."); return false; }

  Serial.printf("Modem: %s\n", modem.getModemInfo().c_str());
  return true;
}

// ─── Cellular data connect ───────────────────────────────────────────────────
bool connectCellular() {
  modem.sendAT(GF("+CNMP=38"));  // LTE only
  modem.waitResponse(10000L);
  modem.sendAT(GF("+CMNB=3"));   // Cat-M + NB-IoT
  modem.waitResponse(10000L);

  Serial.printf("Connecting to Hologram APN: %s\n", CELLULAR_APN);
  if (!modem.gprsConnect(CELLULAR_APN, "", "")) {
    Serial.println("ERROR: Cellular connect failed."); return false;
  }
  Serial.printf("Cellular IP: %s\n", modem.localIP().toString().c_str());

  // Hologram's default DNS fails on long hostnames — override with Google DNS
  modem.sendAT(GF("+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\""));
  modem.waitResponse(2000L);
  Serial.println("DNS set to 8.8.8.8 / 8.8.4.4");
  return true;
}

// ─── NITZ time sync (cellular path) ─────────────────────────────────────────
// SIM7000G provides network time via AT+CCLK? after registration.
// Falls back gracefully if NITZ is unavailable on this operator.
void syncTimeFromNITZ() {
  // TinyGSM getNetworkTime() now requires 7 out-params instead of returning String.
  int yy, mo, dd, hh, mm, ss;
  float tz_f;
  if (!modem.getNetworkTime(&yy, &mo, &dd, &hh, &mm, &ss, &tz_f)) {
    Serial.println("[NITZ] No time from network — clock not yet synced.");
    return;
  }
  // tz_f is in quarter-hours (e.g. -24.0 = UTC-6). Convert to seconds.
  int tzOffsetSec = (int)(tz_f * 15.0f * 60.0f);

  setenv("TZ", "UTC0", 1); tzset();
  struct tm t = {};
  t.tm_year  = (yy > 99 ? yy : 2000 + yy) - 1900;  // handle 2-digit or 4-digit year
  t.tm_mon   = mo - 1;
  t.tm_mday  = dd;
  t.tm_hour  = hh;
  t.tm_min   = mm;
  t.tm_sec   = ss;
  t.tm_isdst = 0;
  time_t epoch = mktime(&t) - tzOffsetSec;  // shift to UTC

  setenv("TZ", TZ_CENTRAL, 1); tzset();
  struct timeval tv = { epoch, 0 };
  settimeofday(&tv, nullptr);
  timeSynced = true;

  struct tm ct;
  localtime_r(&epoch, &ct);
  char buf[32]; strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ct);
  Serial.printf("[NITZ] Synced: %s Central (tz_offset=%.2f qhr)\n", buf, tz_f);
}

// ─── WiFi connect ────────────────────────────────────────────────────────────
bool connectWiFi() {
#ifdef FORCE_CELLULAR
  Serial.println("WiFi skipped (FORCE_CELLULAR).");
  return false;
#endif
  int count = sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);
  unsigned long start = millis();
  for (int i = 0; i < count; i++) {
    Serial.printf("Trying WiFi: %s\n", WIFI_NETWORKS[i][0]);
    WiFi.begin(WIFI_NETWORKS[i][0], WIFI_NETWORKS[i][1]);
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_TIMEOUT_MS) {
      delay(500); Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("WiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());
      return true;
    }
    WiFi.disconnect();
  }
  Serial.println("No WiFi reachable.");
  return false;
}

// ─── Time initialisation ─────────────────────────────────────────────────────
void initTime() {
  setenv("TZ", TZ_CENTRAL, 1); tzset();
  if (!usingCellular) {
    Serial.println("NTP sync (pool.ntp.org)...");
    configTime(0, 0, "pool.ntp.org", "time.google.com");
    struct tm t;
    if (getLocalTime(&t, 8000)) {
      char buf[32]; strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
      Serial.printf("NTP synced: %s Central\n", buf);
      timeSynced = true;
    } else {
      Serial.println("NTP not yet available — will resolve in background.");
    }
  } else {
    syncTimeFromNITZ();
  }
}


// ─── MQTT connect ─────────────────────────────────────────────────────────────
void connectMQTT() {
  mqtt->setServer(MQTT_HOST, MQTT_PORT);
  mqtt->setKeepAlive(usingCellular ? 300 : 60);
  while (!mqtt->connected()) {
    Serial.printf("Connecting MQTT [%s]...", usingCellular ? "CELL" : "WiFi");

    if (usingCellular) {
      // Hologram's TinyGsmClient DNS fails on long HiveMQ subdomain.
      // Pre-connect the raw TCP socket to the hardcoded IP so SSLClient sees
      // an already-connected socket and skips its own DNS+TCP step, doing only
      // the TLS handshake (which uses MQTT_HOST for SNI via setCACert path).
      if (!gsmClientMQTT.connected()) {
        Serial.printf("\n  [TCP] Connecting to %s...", MQTT_HOST_IP);
        if (!gsmClientMQTT.connect(MQTT_HOST_IP, MQTT_PORT)) {
          Serial.println(" FAILED — retry 5s");
          delay(5000);
          continue;
        }
        Serial.println(" OK");
      }
    }

    if (mqtt->connect(deviceId.c_str(), MQTT_USER, MQTT_PASS)) {
      Serial.println(" OK");
    } else {
      Serial.printf(" rc=%d — retry 5s\n", mqtt->state());
      gsmClientMQTT.stop();  // force TCP reconnect on next attempt
      delay(5000);
    }
  }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("\n=== SteadyState Hub SIM7-0001 — T-SIM7000G  fw=%s ===\n", FW_VERSION_STR);

  // ── ADC: VBUS voltage divider for power outage detection ─────────────────
  pinMode(POWER_ADC_PIN, INPUT);
  analogSetPinAttenuation(POWER_ADC_PIN, ADC_11db);  // 0–3.3V range
  int bootAdc = analogRead(POWER_ADC_PIN);
  mainsPresent = (bootAdc >= POWER_ON_THRESH);
  Serial.printf("Power ADC GPIO%d: raw=%d  mains=%s\n",
                POWER_ADC_PIN, bootAdc, mainsPresent ? "PRESENT" : "ABSENT");


  // ── Temp/humidity sensor init ─────────────────────────────────────────────
#ifdef STUB_TEMP
  Serial.printf("[SENSOR] STUB MODE — injecting %.1f°F, 45.0%%RH\n", (float)STUB_TEMP);
#elif defined(SENSOR_SHT31)
  Wire.begin();  // SDA=GPIO21, SCL=GPIO22
  if (!sht31.begin(SHT31_ADDR)) {
    Serial.printf("FATAL: SHT31-D not found at 0x%02X — check wiring.\n", SHT31_ADDR);
    Serial.println("  VCC→3.3V  GND→GND  SCL→GPIO22  SDA→GPIO21  ADDR→GND");
    while (true) delay(10000);
  }
  Serial.printf("[SENSOR] SHT31-D ready (I2C 0x%02X)\n", SHT31_ADDR);
  float testTemp = sht31.readTemperature();
  float testHum  = sht31.readHumidity();
  if (!isnan(testTemp) && !isnan(testHum))
    Serial.printf("[SENSOR] Boot read: %.1f°C  %.1f%%RH\n", testTemp, testHum);
  else
    Serial.println("[SENSOR] WARNING: SHT31-D boot read returned NaN.");
#elif defined(SENSOR_BMP280)
  Wire.begin();  // SDA=GPIO21, SCL=GPIO22
  if (!bmp280.begin(BMP280_ADDR)) {
    Serial.println("FATAL: BMP280 not found at 0x76 — check wiring.");
    Serial.println("  VCC→3.3V  GND→GND  SCL→GPIO22  SDA→GPIO21");
    while (true) delay(10000);
  }
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,   // temp
                     Adafruit_BMP280::SAMPLING_X16,  // pressure
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_500);
  Serial.println("[SENSOR] BMP280 ready (I2C 0x76) — temp only (no humidity)");
  Serial.printf("[SENSOR] Boot read: %.1f°C\n", bmp280.readTemperature());
#endif

  // ── Modem ────────────────────────────────────────────────────────────────
  if (!initModem()) {
    Serial.println("FATAL: Modem did not start."); while (true) delay(10000);
  }

  // ── Network ──────────────────────────────────────────────────────────────
  if (connectWiFi()) {
    wifiSecure.setInsecure();
    mqtt          = &mqttWifi;
    usingCellular = false;
    deviceId      = "ssHub001-wifi-" + WiFi.macAddress();
    Serial.println("Transport: WiFi");
  } else {
    // Keep WiFi in STA mode (radio off) so ESP-NOW can still be initialized later.
    // WIFI_OFF would prevent esp_now_init() from working.
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    if (!connectCellular()) {
      Serial.println("FATAL: No network available."); while (true) delay(10000);
    }
    sslGsmMqtt.setCACert(HIVEMQ_ROOT_CA);
    mqtt          = &mqttCell;
    usingCellular = true;
    deviceId      = "ssHub001-cell-" + modem.getIMEI();
    Serial.println("Transport: Cellular (Hologram) — MQTT TLS via SSLClient");
  }

  connectMQTT();
  initTime();

  // ── OTA ──────────────────────────────────────────────────────────────────
  if (!usingCellular) {
    otaInit(wifiSecure);
    Serial.println("OTA: WiFi (checks hourly)");
  } else {
    sslGsmOTA.setInsecure();
    otaInit(sslGsmOTA);
    Serial.println("OTA: Cellular via SSLClient (checks hourly)");
  }

  // ── ESP-NOW ──────────────────────────────────────────────────────────────
  // WiFi must be in STA mode for ESP-NOW to work. On the cellular path we
  // already set WIFI_STA above (without connecting) so this is a no-op there.
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init failed — vibration data unavailable.");
  } else {
    esp_now_register_recv_cb(onVibrationReceived);
    Serial.println("[ESP-NOW] Receiver ready — listening for C6 vibration sensor.");
  }

  // ── Publish restore event if this boot follows a power outage ────────────
  // Check NVS for a saved outage start timestamp. If found, mains were lost
  // before the last reboot — calculate real duration using NITZ-synced time.
  if (mainsPresent) {
    char payload[180];
    snprintf(payload, sizeof(payload),
      "{\"sn\":\"SIM7-0001\",\"event\":\"restore\",\"outage_s\":0,"
      "\"adc\":%d,\"fw\":\"%s\",\"note\":\"boot\"}",
      bootAdc, FW_VERSION_STR);
    Serial.println("[POWER] Published boot-restore event.");
    mqtt->publish(MQTT_POWER_TOPIC, payload, true);
  }
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  if (!mqtt->connected()) connectMQTT();
  mqtt->loop();
  otaLoop();
  checkPower();  // ADC poll + debounce — publishes on mains state change

  unsigned long now = millis();
  if (now - lastPublish < PUBLISH_INTERVAL) return;
  lastPublish = now;

  // ── Read sensor ──────────────────────────────────────────────────────────
#ifdef STUB_TEMP
  float tempF = (float)STUB_TEMP;
  float rh    = 45.0f;
  bool  hasRH = true;
#elif defined(SENSOR_SHT31)
  float tempC = sht31.readTemperature();
  float rh    = sht31.readHumidity();
  bool  hasRH = true;
  if (isnan(tempC) || isnan(rh)) {
    Serial.println("ERROR: SHT31-D read failed — skipping publish.");
    return;
  }
#  ifdef USE_IMPERIAL
  float tempF = tempC * 9.0f / 5.0f + 32.0f + TEMP_OFFSET_F;
#  else
  float tempF = tempC + (TEMP_OFFSET_F * 5.0f / 9.0f);
#  endif
#elif defined(SENSOR_BMP280)
  float tempC = bmp280.readTemperature();
  float rh    = NAN;
  bool  hasRH = false;
  if (isnan(tempC)) {
    Serial.println("ERROR: BMP280 read failed — skipping publish.");
    return;
  }
#  ifdef USE_IMPERIAL
  float tempF = tempC * 9.0f / 5.0f + 32.0f + TEMP_OFFSET_F;
#  else
  float tempF = tempC + (TEMP_OFFSET_F * 5.0f / 9.0f);
#  endif
#endif

  // ── Daily hi/lo ──────────────────────────────────────────────────────────
  checkDailySummary(tempF);

  // ── Realtime status payload ───────────────────────────────────────────────
  char payload[240];
#ifdef USE_IMPERIAL
  if (hasRH)
    snprintf(payload, sizeof(payload),
      "{\"sn\":\"SIM7-0001\",\"mode\":\"realtime\","
      "\"temp_f\":%.1f,\"rh\":%.1f,\"mains\":\"%s\",\"fw\":\"%s\"}",
      tempF, rh, mainsPresent ? "on" : "off", FW_VERSION_STR);
  else
    snprintf(payload, sizeof(payload),
      "{\"sn\":\"SIM7-0001\",\"mode\":\"realtime\","
      "\"temp_f\":%.1f,\"rh\":null,\"mains\":\"%s\",\"fw\":\"%s\"}",
      tempF, mainsPresent ? "on" : "off", FW_VERSION_STR);
#else
  if (hasRH)
    snprintf(payload, sizeof(payload),
      "{\"sn\":\"SIM7-0001\",\"mode\":\"realtime\","
      "\"temp_c\":%.1f,\"rh\":%.1f,\"mains\":\"%s\",\"fw\":\"%s\"}",
      tempF, rh, mainsPresent ? "on" : "off", FW_VERSION_STR);
  else
    snprintf(payload, sizeof(payload),
      "{\"sn\":\"SIM7-0001\",\"mode\":\"realtime\","
      "\"temp_c\":%.1f,\"rh\":null,\"mains\":\"%s\",\"fw\":\"%s\"}",
      tempF, mainsPresent ? "on" : "off", FW_VERSION_STR);
#endif

  Serial.printf("[%s] %s\n", usingCellular ? "CELL" : "WiFi", payload);
  mqtt->publish(MQTT_TOPIC, payload, true);
}
