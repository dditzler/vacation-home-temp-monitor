// ============================================================
//  OTA Update Module — LilyGO T-SIM7000G
//  See ota_update.h for configuration and public API.
// ============================================================

#include "ota_update.h"
#include <ArduinoHttpClient.h>
#include <Update.h>

// ── Internal state ────────────────────────────────────────────
static Client*       _client    = nullptr;
static unsigned long _lastCheck = 0;

// ── Minimal JSON string extractor ────────────────────────────
// Pulls the value for a simple string key from a flat JSON object
// without requiring the ArduinoJson library.
// e.g. extractJsonStr(body, "version") from {"version":"1.2.0",...}
static String extractJsonStr(const String& json, const String& key) {
  String search = "\"" + key + "\"";
  int idx = json.indexOf(search);
  if (idx < 0) return "";
  idx = json.indexOf(':', idx + search.length());
  if (idx < 0) return "";
  idx = json.indexOf('"', idx + 1);
  if (idx < 0) return "";
  int end = json.indexOf('"', idx + 1);
  if (end < 0) return "";
  return json.substring(idx + 1, end);
}

// ── Parse "MAJOR.MINOR.PATCH" → uint32 (same encoding as FW_VERSION_INT) ─────
static uint32_t parseVersionInt(const String& v) {
  int d1 = v.indexOf('.');
  int d2 = v.indexOf('.', d1 + 1);
  if (d1 < 0 || d2 < 0) return 0;
  uint32_t maj = (uint32_t)v.substring(0, d1).toInt();
  uint32_t min = (uint32_t)v.substring(d1 + 1, d2).toInt();
  uint32_t pat = (uint32_t)v.substring(d2 + 1).toInt();
  return (maj << 16) | (min << 8) | pat;
}

// ── Step 1: fetch version.json ────────────────────────────────
static bool fetchManifest(String& outVersion,
                           String& outHost, int& outPort, String& outPath) {
  HttpClient http(*_client, OTA_GITHUB_HOST, 443);
  http.setTimeout(10000);

  Serial.print("[OTA] Checking " OTA_GITHUB_HOST OTA_VERSION_PATH " ... ");
  if (http.get(OTA_VERSION_PATH) != HTTP_SUCCESS) {
    Serial.println("connection failed");
    http.stop();
    return false;
  }

  int status = http.responseStatusCode();
  if (status != 200) {
    Serial.println("HTTP " + String(status));
    http.stop();
    return false;
  }

  String body = http.responseBody();
  http.stop();
  Serial.println("OK");
  Serial.println("[OTA] Manifest: " + body);

  outVersion      = extractJsonStr(body, "version");
  String binaryUrl = extractJsonStr(body, "binary_url");

  if (outVersion.isEmpty() || binaryUrl.isEmpty()) {
    Serial.println("[OTA] Manifest missing 'version' or 'binary_url'");
    return false;
  }
  if (!binaryUrl.startsWith("https://")) {
    Serial.println("[OTA] binary_url must start with https://");
    return false;
  }

  // Split "https://HOST/PATH" into components
  String rest = binaryUrl.substring(8);
  int slash   = rest.indexOf('/');
  if (slash < 0) { Serial.println("[OTA] Malformed binary_url"); return false; }

  outHost = rest.substring(0, slash);
  outPath = rest.substring(slash);
  outPort = 443;
  return true;
}

// ── Step 2: download binary and write to flash ────────────────
static bool downloadAndFlash(const String& host, int port,
                              const String& path, int depth = 0) {
  if (depth > 2) { Serial.println("[OTA] Too many redirects"); return false; }

  Serial.println("[OTA] Downloading from " + host + path);

  HttpClient http(*_client, host, port);
  http.setTimeout(60000);   // large binary over WiFi — give it time

  if (http.get(path) != HTTP_SUCCESS) {
    Serial.println("[OTA] Download connection failed");
    http.stop();
    return false;
  }

  int status = http.responseStatusCode();

  // Follow redirects (GitHub Releases redirect to objects.githubusercontent.com)
  if (status == 301 || status == 302 || status == 307 || status == 308) {
    String location = http.header("Location");
    http.stop();
    if (location.isEmpty()) {
      Serial.println("[OTA] Redirect with no Location header");
      return false;
    }
    Serial.println("[OTA] Redirect → " + location);
    if (!location.startsWith("https://")) {
      Serial.println("[OTA] Non-HTTPS redirect — aborting");
      return false;
    }
    String rest  = location.substring(8);
    int    slash = rest.indexOf('/');
    if (slash < 0) return false;
    return downloadAndFlash(rest.substring(0, slash), 443,
                            rest.substring(slash), depth + 1);
  }

  if (status != 200) {
    Serial.println("[OTA] Download HTTP " + String(status));
    http.stop();
    return false;
  }

  int contentLength = http.contentLength();
  Serial.println("[OTA] Content-Length: " + String(contentLength) + " bytes");

  if (!Update.begin(contentLength > 0 ? contentLength : UPDATE_SIZE_UNKNOWN)) {
    Serial.println("[OTA] Update.begin() failed: " + String(Update.errorString()));
    http.stop();
    return false;
  }

  uint8_t  buf[512];
  int      written   = 0;
  int      remaining = contentLength > 0 ? contentLength : INT_MAX;

  while (http.connected() && remaining > 0) {
    int avail = http.available();
    if (avail > 0) {
      int n = http.read(buf, min(avail, (int)sizeof(buf)));
      n = min(n, remaining);
      if (n > 0) {
        if ((int)Update.write(buf, n) != n) {
          Serial.println("[OTA] Flash write error: " + String(Update.errorString()));
          http.stop();
          Update.abort();
          return false;
        }
        written   += n;
        remaining -= n;
        if (written % 20480 == 0) {
          Serial.printf("[OTA] Written %d / %d bytes\n", written, contentLength);
        }
      }
    } else {
      delay(5);
    }
  }
  http.stop();

  if (!Update.end(true)) {
    Serial.println("[OTA] Update.end() error: " + String(Update.errorString()));
    return false;
  }

  Serial.printf("[OTA] Flash complete (%d bytes). Rebooting...\n", written);
  return true;
}

// ── Public API ────────────────────────────────────────────────

void otaInit(Client& sslClient) {
  _client    = &sslClient;
  _lastCheck = 0;   // check on first otaLoop() call
  Serial.println("[OTA] Initialized — firmware " FW_VERSION_STR
                 "  repo: " OTA_GITHUB_USER "/" OTA_GITHUB_REPO);
}

bool otaCheckNow() {
  if (!_client) {
    Serial.println("[OTA] Not initialized — call otaInit() first");
    return false;
  }
  _lastCheck = millis();

  // 1. Fetch manifest
  String latestVersion, binHost, binPath;
  int    binPort;
  if (!fetchManifest(latestVersion, binHost, binPort, binPath)) return false;

  Serial.println("[OTA] Latest: " + latestVersion +
                 "  Current: " FW_VERSION_STR);

  // 2. Compare
  if (parseVersionInt(latestVersion) <= FW_VERSION_INT) {
    Serial.println("[OTA] Firmware is up to date.");
    return false;
  }

  Serial.println("[OTA] Update available — flashing...");

  // 3. Download and flash
  if (!downloadAndFlash(binHost, binPort, binPath)) {
    Serial.println("[OTA] Update failed — will retry next check.");
    return false;
  }

  delay(500);
  ESP.restart();
  return true;   // unreachable; keeps compiler happy
}

bool otaLoop() {
  if (!_client) return false;

  unsigned long intervalMs = (unsigned long)OTA_CHECK_INTERVAL_S * 1000UL;
  if (millis() - _lastCheck < intervalMs) return false;

  return otaCheckNow();
}
