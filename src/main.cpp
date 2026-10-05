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
//    Checks every OTA_CHECK_INTERVAL_S (daily on tsim7000g); SMS "ota" forces a check.
//    Reboots automatically on new version.
//    To release: bump version.h, commit, git tag sensor-vX.Y.Z, push.
//
//  MQTT topics:
//    vacation/cust1/il/temp/status      — realtime temp+humidity+mains (10 min; 2 min below 55°F)
//    vacation/cust1/il/temp/daily       — hi/lo summary at 6 PM Central
//    vacation/cust1/il/temp/power       — outage/restore events (retained)
//    vacation/cust1/il/temp/sensor_raw  — ESP-NOW vibration relay (only with -DENABLE_ESPNOW_RELAY)
//
//  Low-data notes (fw 1.4.1):
//    - MQTT reconnects use exponential backoff (15 s → 30 min cap). A failing
//      TLS handshake costs ~5 KB; the old fixed 5 s retry burned ~1.5 MB/hr.
//    - ESP-NOW vibration relay is compiled out unless ENABLE_ESPNOW_RELAY.
//    - Status payload carries "reconn" (MQTT reconnects since boot) so silent
//      NAT drops / handshake churn are visible from the broker side.
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
#include "ca_cert.h"    // Broker root CA bundle for SSLClient cellular TLS

// ─── WiFi credentials (from secrets.h) ───────────────────────────────────────
const char* WIFI_NETWORKS[][2] = {
  { WIFI_SSID_1, WIFI_PASS_1 },   // home
  { WIFI_SSID_2, WIFI_PASS_2 },   // vacation home
};

// ─── Cellular — Hologram ─────────────────────────────────────────────────────
const char* CELLULAR_APN = "hologram";

// ─── MQTT broker ─────────────────────────────────────────────────────────────
// EMQX Cloud Serverless (free tier), deployment "ernie" in project SteadyState,
// AWS us-east-1. Console login: Google account david@ditzco.com.
// (fw <= 1.4.2 used HiveMQ Cloud Serverless, retired 2026-12-31.)
//
// To move to another broker: change MQTT_HOST / MQTT_PORT, refresh the fallback
// IPs (dig <host>), make sure ca_cert.h holds its root CA, and update
// MQTT_USER_VAL / MQTT_PASS_VAL in secrets.h.
const char* MQTT_HOST        = "d11f1164.ala.us-east-1.emqxsl.com";
const int   MQTT_PORT        = 8883;
// Cellular connects by hostname first (modem DNS, set to 8.8.8.8 in
// connectCellular). If that fails, it tries these IPs in order — they can go
// stale, which is why they are only a fallback.
// dig d11f1164.ala.us-east-1.emqxsl.com → 23.23.126.71 / 100.51.47.111 (2026-10-04)
// SSLClient still sends MQTT_HOST as SNI so TLS cert validation works either way.
const char* MQTT_HOST_FALLBACK_IPS[] = { "23.23.126.71", "100.51.47.111" };
const int   MQTT_HOST_FALLBACK_N =
  sizeof(MQTT_HOST_FALLBACK_IPS) / sizeof(MQTT_HOST_FALLBACK_IPS[0]);

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
#define PUBLISH_INTERVAL  600000UL   // 10 minutes on cellular (normal temps)
#endif

// The sensor is read locally every SENSOR_SAMPLE_MS (no data cost). A status
// message is published when:
//   - PUBLISH_INTERVAL has elapsed (normal), or
//   - COLD_PUBLISH_INTERVAL has elapsed while temp < COLD_WATCH_F, or
//   - the temp crosses into / out of a cold band (immediate).
// Bands: 0 = normal (>= 55°F), 1 = watch (< 55°F), 2 = alert (< 50°F).
// Leaving a colder band requires COLD_HYST_F above its threshold (no flapping).
#ifndef SENSOR_SAMPLE_MS
#define SENSOR_SAMPLE_MS        60000UL    // read sensor every 1 min
#endif
#ifndef COLD_PUBLISH_INTERVAL
#define COLD_PUBLISH_INTERVAL  120000UL    // 2 min while below COLD_WATCH_F
#endif
#define COLD_WATCH_F   55.0f
#define COLD_ALERT_F   50.0f   // matches dashboard "cold" color threshold
#define COLD_HYST_F     0.5f
#define WIFI_TIMEOUT_MS    30000UL

// MQTT keepalive on cellular (seconds). Kept just above PUBLISH_INTERVAL so the
// regular publish doubles as the keepalive and PINGREQs are rare.
#ifndef MQTT_KEEPALIVE_CELL_S
#define MQTT_KEEPALIVE_CELL_S  ((uint16_t)(PUBLISH_INTERVAL / 1000UL + 120UL))
#endif

// MQTT reconnect backoff — doubles after each failure, resets on success.
#define MQTT_BACKOFF_MIN_MS    15000UL     // 15 s
#define MQTT_BACKOFF_MAX_MS  1800000UL     // 30 min cap (~240 KB/day worst case)

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

// ─── Hologram SMS downlink config ────────────────────────────────────────────
// The hub polls for incoming SMS every SMS_POLL_INTERVAL_MS.
// Recognized commands (sent via hologram_monitor.py --send-sms <cmd>):
//   ping   — publish a status payload immediately
//   reboot — reboot the ESP32
//   ota    — trigger an OTA check immediately
// Unrecognized SMS bodies are logged but ignored.
#define SMS_POLL_INTERVAL_MS  60000UL    // poll once per minute

// ─── ESP-NOW message structure (must match vibration_xiao_c6 firmware exactly) ─
// fw 1.1.0: simplified — no on/off decision in sensor firmware.
// Hub injects hub_temp_f + hub_temp_trend into MQTT payload; cloud assembles cycles.
typedef struct {
  uint8_t  wakeReason;    // 0=activity interrupt, 1=timer, 2=heartbeat
  uint16_t rawMagnitude;  // max axis range in ADXL345 counts this wake
} VibrationMsg;           // sizeof = 4 bytes


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
unsigned long lastSample    = 0;
uint8_t       lastColdBand  = 0;   // 0 normal, 1 watch (<55°F), 2 alert (<50°F)

// Cold band with hysteresis — see COLD_* defines.
uint8_t calcColdBand(float f, uint8_t prev) {
  uint8_t b = (f < COLD_ALERT_F) ? 2 : (f < COLD_WATCH_F) ? 1 : 0;
  if (b < prev) {   // warming up: only leave the colder band once clearly above it
    float thr = (prev == 2) ? COLD_ALERT_F : COLD_WATCH_F;
    if (f < thr + COLD_HYST_F) b = prev;
  }
  return b;
}

// MQTT reconnect state (see connectMQTT)
unsigned long mqttLastAttempt  = 0;
unsigned long mqttWaitMs       = 0;      // 0 → first attempt is immediate
uint32_t      mqttReconnects   = 0;      // successful connects after the first
bool          mqttEverConnected = false;

// Boot-restore event is queued and sent on the first successful connect,
// so it isn't lost if the broker is unreachable at boot.
char          bootEventPayload[180];
bool          bootEventPending = false;
unsigned long lastSmsPoll   = 0;
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

// ─── Hub temp trend — rolling 15-min window ──────────────────────────────────
// Stores the last N temp readings with timestamps. onVibrationReceived() reads
// these to inject hub_temp_f and hub_temp_trend into the sensor_raw MQTT payload.
// Trend: +1 = rose >0.5°F in window, -1 = fell >0.5°F, 0 = flat.
#define TEMP_HISTORY_SIZE  10   // at 5-min publish interval = 50-min window max
struct TempSample { unsigned long ts; float tempF; };
TempSample    tempHistory[TEMP_HISTORY_SIZE];
int           tempHistoryIdx   = 0;
int           tempHistoryCount = 0;
float         latestTempF      = NAN;  // most recent reading

void recordTempSample(float tempF) {
  latestTempF = tempF;
  tempHistory[tempHistoryIdx] = { millis(), tempF };
  tempHistoryIdx = (tempHistoryIdx + 1) % TEMP_HISTORY_SIZE;
  if (tempHistoryCount < TEMP_HISTORY_SIZE) tempHistoryCount++;
}

// Returns +1 (rising), -1 (falling), 0 (flat) based on oldest vs newest sample
// within a 15-min window. Requires at least 2 samples.
int8_t calcTempTrend() {
  if (tempHistoryCount < 2) return 0;
  unsigned long now = millis();
  unsigned long windowMs = 15UL * 60UL * 1000UL;  // 15 minutes

  // Find oldest sample within window
  float oldest = NAN;
  for (int i = 0; i < tempHistoryCount; i++) {
    int idx = (tempHistoryIdx - tempHistoryCount + i + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
    if (now - tempHistory[idx].ts <= windowMs) {
      if (isnan(oldest)) oldest = tempHistory[idx].tempF;
    }
  }
  if (isnan(oldest) || isnan(latestTempF)) return 0;

  float delta = latestTempF - oldest;
  if (delta >  0.5f) return  1;
  if (delta < -0.5f) return -1;
  return 0;
}


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
// Called on every incoming VibrationMsg from the C6 vibration sensor (fw 1.1.0+).
// Injects hub_temp_f + hub_temp_trend into the MQTT payload so the cloud can
// assemble cycles and calibrate per device without any firmware thresholds.
void onVibrationReceived(const uint8_t* mac_addr,
                         const uint8_t* data, int len) {
  if (len != sizeof(VibrationMsg)) {
    Serial.printf("[ESP-NOW] Ignoring %d-byte packet from %02X:%02X:%02X:%02X:%02X:%02X (expected %d)\n",
      len,
      mac_addr[0], mac_addr[1], mac_addr[2],
      mac_addr[3], mac_addr[4], mac_addr[5],
      sizeof(VibrationMsg));
    return;
  }
  VibrationMsg msg;
  memcpy(&msg, data, sizeof(msg));

  int8_t rssi      = 0;  // legacy cb doesn't carry RSSI; use 0 as sentinel
  float  hubTemp   = isnan(latestTempF) ? -999.0f : latestTempF;
  int8_t tempTrend = calcTempTrend();   // +1 rising, -1 falling, 0 flat

  const char* reasonStr =
    msg.wakeReason == 0 ? "activity" :
    msg.wakeReason == 2 ? "heartbeat" : "timer";

  Serial.printf("[ESP-NOW] reason=%s  mag=%u  rssi=%d dBm  hub_temp=%.1f°F  trend=%+d\n",
    reasonStr, msg.rawMagnitude, rssi, hubTemp, tempTrend);

  if (!mqtt || !mqtt->connected()) {
    Serial.println("[ESP-NOW] MQTT not connected — dropping vibration message.");
    return;
  }

  char payload[192];
  snprintf(payload, sizeof(payload),
    "{\"sn\":\"EC6-0001\",\"reason\":\"%s\","
    "\"magnitude\":%u,\"rssi_dbm\":%d,"
    "\"hub_temp_f\":%.1f,\"hub_temp_trend\":%d}",
    reasonStr,
    msg.rawMagnitude,
    rssi,
    hubTemp,
    tempTrend);
  mqtt->publish(MQTT_SENSOR_RAW, payload);
}

// ─── Hologram SMS downlink handler ───────────────────────────────────────────
// Polls the SIM7000G for incoming SMS. On cellular path only — WiFi path has
// no SIM so there's nothing to poll.
//
// Commands are matched case-insensitively against the SMS body (trimmed).
// After acting, the SMS is deleted from the SIM to prevent re-processing.
//
// To send a command:
//   python3 hologram_monitor.py --send-sms "ping"
//
// Read a single SMS by index using raw AT+CMGR.
// Returns the message body, or "" if no message at that index.
// SIM7000G response format:
//   +CMGR: "REC READ","<number>",,"<timestamp>"
//   <body text>
//   OK
static String smsReadAt(int idx) {
  modem.sendAT(GF("+CMGR="), idx);
  if (modem.waitResponse(3000L, GF("+CMGR:")) != 1) return "";
  modem.stream.readStringUntil('\n');  // skip header line (sender, timestamp)
  String body = modem.stream.readStringUntil('\n');
  modem.waitResponse(1000L);  // consume trailing OK
  body.trim();
  return body;
}

// Delete an SMS by index using raw AT+CMGD.
static void smsDeleteAt(int idx) {
  modem.sendAT(GF("+CMGD="), idx);
  modem.waitResponse(2000L);
}

void checkSMS() {
  if (!usingCellular) return;   // no SIM on WiFi path

  // List all stored SMS with AT+CMGL="ALL" and process each.
  // Simpler than iterating fixed indices — works regardless of SIM slot layout.
  // Format per entry:
  //   +CMGL: <idx>,"REC READ","<sender>",,"<ts>"
  //   <body>
  modem.sendAT(GF("+CMGL=\"ALL\""));
  if (modem.waitResponse(5000L, GF("+CMGL:")) != 1) {
    // No messages (response is just "OK") — normal, nothing to do.
    return;
  }

  // Parse each message entry
  while (true) {
    // We've already consumed "+CMGL:" for the first entry (or loop continues).
    // Read the header to extract index.
    String header = modem.stream.readStringUntil('\n');
    header.trim();
    // header looks like: <idx>,"REC UNREAD","...",,"..."
    int idx = header.toInt();   // toInt() stops at first non-digit

    String body = modem.stream.readStringUntil('\n');
    body.trim();

    if (body.length() == 0) break;

    String bodyLower = body;
    bodyLower.toLowerCase();
    Serial.printf("[SMS] idx=%d body='%s'\n", idx, body.c_str());

    if (bodyLower == "ping") {
      if (mqtt && mqtt->connected()) {
        char payload[160];
        snprintf(payload, sizeof(payload),
          "{\"sn\":\"SIM7-0001\",\"event\":\"pong\","
          "\"fw\":\"%s\",\"mains\":\"%s\"}",
          FW_VERSION_STR, mainsPresent ? "on" : "off");
        mqtt->publish(MQTT_TOPIC, payload, false);
        Serial.println("[SMS] pong published to MQTT.");
      }

    } else if (bodyLower == "reboot") {
      Serial.println("[SMS] Reboot command — rebooting in 2s...");
      if (mqtt && mqtt->connected()) {
        char payload[100];
        snprintf(payload, sizeof(payload),
          "{\"sn\":\"SIM7-0001\",\"event\":\"reboot_cmd\",\"fw\":\"%s\"}",
          FW_VERSION_STR);
        mqtt->publish(MQTT_TOPIC, payload, false);
        delay(500);
      }
      smsDeleteAt(idx);
      delay(1000);
      ESP.restart();
      return;

    } else if (bodyLower == "ota") {
      // otaLoop() has its own internal timer; we can't reset it without an
      // extern. Log the command and let the next hourly check pick it up.
      Serial.println("[SMS] OTA command — will check at next scheduled interval.");
      if (mqtt && mqtt->connected()) {
        char payload[100];
        snprintf(payload, sizeof(payload),
          "{\"sn\":\"SIM7-0001\",\"event\":\"ota_cmd\",\"fw\":\"%s\"}",
          FW_VERSION_STR);
        mqtt->publish(MQTT_TOPIC, payload, false);
      }

    } else {
      Serial.printf("[SMS] Unrecognized command: '%s' — ignoring.\n", body.c_str());
    }

    smsDeleteAt(idx);

    // Check if there's another entry
    if (modem.waitResponse(1000L, GF("+CMGL:")) != 1) break;
  }
  modem.waitResponse(1000L);  // consume final OK
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
  // TinyGSM already converts the modem's quarter-hour offset to HOURS
  // (TinyGsmTime.tpp: *timezone = itimezone / 4.0), so tz_f = -5.0 means UTC-5.
  // fw <= 1.4.1 treated it as quarter-hours (×15 min) → clock ran ~3.75 h slow.
  int tzOffsetSec = (int)(tz_f * 3600.0f);

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
  Serial.printf("[NITZ] Synced: %s Central (network tz=UTC%+.2f h)\n", buf, tz_f);
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


// ─── MQTT connect (non-blocking, exponential backoff) ────────────────────────
// Makes at most ONE connection attempt per call, and only once the current
// backoff window has elapsed. Returns true if connected.
// Never blocks the loop, so SMS commands (reboot/ota/ping) keep working
// while the broker is unreachable.
bool connectMQTT() {
  if (mqtt->connected()) return true;
  if (millis() - mqttLastAttempt < mqttWaitMs) return false;
  mqttLastAttempt = millis();

  mqtt->setServer(MQTT_HOST, MQTT_PORT);
  mqtt->setKeepAlive(usingCellular ? MQTT_KEEPALIVE_CELL_S : 60);
  Serial.printf("Connecting MQTT [%s]...", usingCellular ? "CELL" : "WiFi");

  bool ok = true;
  if (usingCellular) {
    // Re-open the data bearer if the network dropped it.
    if (!modem.isGprsConnected()) {
      Serial.print("\n  [GPRS] Bearer down — reconnecting...");
      ok = modem.gprsConnect(CELLULAR_APN, "", "");
      Serial.println(ok ? " OK" : " FAILED");
    }
    // Pre-connect the raw TCP socket so SSLClient sees an already-connected
    // socket and skips its own DNS+TCP step, doing only the TLS handshake
    // (which still uses MQTT_HOST for SNI + certificate hostname check).
    // Try the hostname first (survives broker IP changes), then fallback IPs.
    if (ok && !gsmClientMQTT.connected()) {
      Serial.printf("\n  [TCP] Connecting to %s...", MQTT_HOST);
      ok = gsmClientMQTT.connect(MQTT_HOST, MQTT_PORT);
      Serial.println(ok ? " OK" : " FAILED");
      for (int i = 0; !ok && i < MQTT_HOST_FALLBACK_N; i++) {
        gsmClientMQTT.stop();
        Serial.printf("  [TCP] Fallback IP %s...", MQTT_HOST_FALLBACK_IPS[i]);
        ok = gsmClientMQTT.connect(MQTT_HOST_FALLBACK_IPS[i], MQTT_PORT);
        Serial.println(ok ? " OK" : " FAILED");
      }
    }
  }

  if (ok && mqtt->connect(deviceId.c_str(), MQTT_USER, MQTT_PASS)) {
    Serial.println(" OK");
    if (mqttEverConnected) mqttReconnects++;
    mqttEverConnected = true;
    mqttWaitMs = MQTT_BACKOFF_MIN_MS;   // next failure starts small again
    return true;
  }

  if (ok) Serial.printf(" rc=%d", mqtt->state());
  if (usingCellular) {
    sslGsmMqtt.stop();    // reset TLS session state
    gsmClientMQTT.stop(); // force TCP reconnect on next attempt
  }
  mqttWaitMs = (mqttWaitMs == 0) ? MQTT_BACKOFF_MIN_MS
             : min(mqttWaitMs * 2UL, MQTT_BACKOFF_MAX_MS);
  Serial.printf(" — retry in %lu s\n", mqttWaitMs / 1000UL);
  return false;
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
#ifdef ENABLE_ESPNOW_RELAY
    WiFi.mode(WIFI_STA);
#else
    WiFi.mode(WIFI_OFF);   // no ESP-NOW relay in this build — radio fully off
#endif
    if (!connectCellular()) {
      Serial.println("FATAL: No network available."); while (true) delay(10000);
    }
    sslGsmMqtt.setCACert(BROKER_ROOT_CA);
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
    Serial.printf("OTA: Cellular via SSLClient (checks every %lu h)\n",
                  (unsigned long)OTA_CHECK_INTERVAL_S / 3600UL);
  }

#ifdef ENABLE_ESPNOW_RELAY
  // ── ESP-NOW ──────────────────────────────────────────────────────────────
  // WiFi must be in STA mode for ESP-NOW to work. On the cellular path we
  // already set WIFI_STA above (without connecting) so this is a no-op there.
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init failed — vibration data unavailable.");
  } else {
    esp_now_register_recv_cb(onVibrationReceived);
    Serial.println("[ESP-NOW] Receiver ready — listening for C6 vibration sensor.");
  }
#else
  Serial.println("[ESP-NOW] Relay disabled in this build (low-data mode).");
#endif

  // ── Publish restore event if this boot follows a power outage ────────────
  // Check NVS for a saved outage start timestamp. If found, mains were lost
  // before the last reboot — calculate real duration using NITZ-synced time.
  if (mainsPresent) {
    char payload[180];
    snprintf(payload, sizeof(payload),
      "{\"sn\":\"SIM7-0001\",\"event\":\"restore\",\"outage_s\":0,"
      "\"adc\":%d,\"fw\":\"%s\",\"note\":\"boot\"}",
      bootAdc, FW_VERSION_STR);
    strlcpy(bootEventPayload, payload, sizeof(bootEventPayload));
    bootEventPending = true;
    Serial.println("[POWER] Boot-restore event queued for first MQTT connect.");
  }
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  if (connectMQTT()) {
    mqtt->loop();
    if (bootEventPending &&
        mqtt->publish(MQTT_POWER_TOPIC, bootEventPayload, true)) {
      bootEventPending = false;
      Serial.println("[POWER] Published boot-restore event.");
    }
  }
  otaLoop();
  checkPower();  // ADC poll + debounce — publishes on mains state change

  // ── Hologram SMS downlink poll ────────────────────────────────────────────
  if (usingCellular && (millis() - lastSmsPoll >= SMS_POLL_INTERVAL_MS)) {
    lastSmsPoll = millis();
    checkSMS();
  }

  unsigned long now = millis();
  // Read the sensor once a minute (local only — no data used).
  if (lastSample != 0 && now - lastSample < SENSOR_SAMPLE_MS) return;
  lastSample = now;

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

  // ── Record temp for trend calculation (used in ESP-NOW callback) ─────────
  recordTempSample(tempF);

  // ── Daily hi/lo (every sample → more accurate hi/lo) ──────────────────────
  checkDailySummary(tempF);

  // ── Decide whether to publish ────────────────────────────────────────────
#ifdef USE_IMPERIAL
  float bandF = tempF;
#else
  float bandF = tempF * 9.0f / 5.0f + 32.0f;   // thresholds are in °F
#endif
  uint8_t band        = calcColdBand(bandF, lastColdBand);
  bool    bandChanged = (band != lastColdBand);
  unsigned long interval = band ? min(COLD_PUBLISH_INTERVAL, PUBLISH_INTERVAL)
                                : PUBLISH_INTERVAL;
  // First status goes out right after boot.
  if (lastPublish != 0 && !bandChanged && now - lastPublish < interval) return;
  if (bandChanged)
    Serial.printf("[TEMP] Cold band %u → %u at %.1f°F — publishing now\n",
                  lastColdBand, band, bandF);
  lastColdBand = band;
  lastPublish  = now;

  // ── Realtime status payload ───────────────────────────────────────────────
  char payload[240];
#ifdef USE_IMPERIAL
  if (hasRH)
    snprintf(payload, sizeof(payload),
      "{\"sn\":\"SIM7-0001\",\"mode\":\"realtime\","
      "\"temp_f\":%.1f,\"rh\":%.1f,\"mains\":\"%s\",\"fw\":\"%s\",\"reconn\":%lu}",
      tempF, rh, mainsPresent ? "on" : "off", FW_VERSION_STR, (unsigned long)mqttReconnects);
  else
    snprintf(payload, sizeof(payload),
      "{\"sn\":\"SIM7-0001\",\"mode\":\"realtime\","
      "\"temp_f\":%.1f,\"rh\":null,\"mains\":\"%s\",\"fw\":\"%s\",\"reconn\":%lu}",
      tempF, mainsPresent ? "on" : "off", FW_VERSION_STR, (unsigned long)mqttReconnects);
#else
  if (hasRH)
    snprintf(payload, sizeof(payload),
      "{\"sn\":\"SIM7-0001\",\"mode\":\"realtime\","
      "\"temp_c\":%.1f,\"rh\":%.1f,\"mains\":\"%s\",\"fw\":\"%s\",\"reconn\":%lu}",
      tempF, rh, mainsPresent ? "on" : "off", FW_VERSION_STR, (unsigned long)mqttReconnects);
  else
    snprintf(payload, sizeof(payload),
      "{\"sn\":\"SIM7-0001\",\"mode\":\"realtime\","
      "\"temp_c\":%.1f,\"rh\":null,\"mains\":\"%s\",\"fw\":\"%s\",\"reconn\":%lu}",
      tempF, mainsPresent ? "on" : "off", FW_VERSION_STR, (unsigned long)mqttReconnects);
#endif

  Serial.printf("[%s] %s\n", usingCellular ? "CELL" : "WiFi", payload);
  mqtt->publish(MQTT_TOPIC, payload, true);
}
