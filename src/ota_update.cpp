// ============================================================
//  OTA Update Module — LilyGO T-SIM7000G
//  See ota_update.h for configuration and public API.
// ============================================================

#include "secrets.h"      // OTA_GITHUB_TOKEN — must come before ota_update.h
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

// Returns true if host is a GitHub domain requiring PAT auth.
static bool isGitHubHost(const String& host) {
  return host.endsWith("github.com") || host.endsWith("githubusercontent.com");
}

// ── Step 1: fetch sensor-version.json ─────────────────────────
static bool fetchManifest(String& outVersion,
                           String& outHost, int& outPort, String& outPath) {
  HttpClient http(*_client, OTA_GITHUB_HOST, 443);
  http.setTimeout(10000);

  Serial.print("[OTA] Checking manifest ... ");
  http.beginRequest();
  http.get(OTA_VERSION_PATH);
  http.sendHeader("Authorization", "token " OTA_GITHUB_TOKEN);
  http.sendHeader("User-Agent", "ESP32-OTA-Sensor");
  http.endRequest();

  int status = http.responseStatusCode();
  if (status != 200) {
    Serial.println("HTTP " + String(status));
    http.stop();
    return false;
  }

  String body = http.responseBody();
  http.stop();
  Serial.println("OK");

  outVersion       = extractJsonStr(body, "version");
  String binaryUrl = extractJsonStr(body, "binary_url");

  if (outVersion.isEmpty()) {
    Serial.println("[OTA] Manifest missing 'version'");
    return false;
  }
  if (outVersion == "0.0.0") {
    Serial.println("[OTA] Manifest is placeholder (0.0.0) — skipping");
    return false;
  }
  if (binaryUrl.isEmpty()) {
    Serial.println("[OTA] Manifest missing 'binary_url'");
    return false;
  }
  if (!binaryUrl.startsWith("https://")) {
    Serial.println("[OTA] binary_url must start with https://");
    return false;
  }

  String rest = binaryUrl.substring(8);
  int slash   = rest.indexOf('/');
  if (slash < 0) { Serial.println("[OTA] Malformed binary_url"); return false; }

  outHost = rest.substring(0, slash);
  outPath = rest.substring(slash);
  outPort = 443;
  return true;
}

// ── Step 2: download binary and write to flash ────────────────
// ArduinoHttpClient has no response-header API, so we use raw
// Client reads to parse the status line, headers (for Location
// on redirects), and body (streamed directly into Update).
// includeAuth=true on first call (GitHub API); false after
// redirect to CDN/S3 (which uses pre-signed query params).
static bool downloadAndFlash(const String& host, int port,
                              const String& path,
                              bool includeAuth, int depth = 0) {
  if (depth > 3) { Serial.println("[OTA] Too many redirects"); return false; }

  Serial.println("[OTA] GET " + host + path);

  // Open a fresh TLS connection to this host
  _client->stop();
  if (!_client->connect(host.c_str(), (uint16_t)port)) {
    Serial.println("[OTA] Connect failed: " + host);
    return false;
  }

  // Send raw HTTP/1.0 request (avoids chunked Transfer-Encoding)
  _client->print("GET "); _client->print(path); _client->println(" HTTP/1.0");
  _client->print("Host: "); _client->println(host);
  if (includeAuth && isGitHubHost(host)) {
    _client->print("Authorization: token "); _client->println(OTA_GITHUB_TOKEN);
    _client->println("Accept: application/octet-stream");
  }
  _client->println("User-Agent: ESP32-OTA-Sensor");
  _client->println();   // blank line = end of request headers

  // Wait for first response byte (up to 30 s)
  unsigned long t0 = millis();
  while (!_client->available() && _client->connected() && millis() - t0 < 30000UL) delay(10);
  if (!_client->available()) {
    Serial.println("[OTA] Response timeout");
    _client->stop();
    return false;
  }

  // ── Parse status line: "HTTP/1.x NNN Reason\r\n" ─────────────
  _client->setTimeout(10000);
  String statusLine = _client->readStringUntil('\n');
  statusLine.trim();
  int sp = statusLine.indexOf(' ');
  int statusCode = (sp >= 0) ? statusLine.substring(sp + 1, sp + 4).toInt() : 0;

  // ── Parse response headers ────────────────────────────────────
  String location = "";
  int    contentLength = -1;
  while (_client->connected() || _client->available()) {
    String line = _client->readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) break;   // blank line = end of headers
    String lower = line; lower.toLowerCase();
    if (lower.startsWith("location:")) {
      location = line.substring(line.indexOf(':') + 1);
      location.trim();
    } else if (lower.startsWith("content-length:")) {
      contentLength = line.substring(line.indexOf(':') + 1).toInt();
    }
  }

  // ── Follow redirects ──────────────────────────────────────────
  if (statusCode >= 300 && statusCode < 400) {
    _client->stop();
    if (location.isEmpty()) {
      Serial.println("[OTA] Redirect with no Location header");
      return false;
    }
    Serial.println("[OTA] Redirect → " + location);
    if (!location.startsWith("https://")) {
      Serial.println("[OTA] Non-HTTPS redirect — aborting");
      return false;
    }
    String rest      = location.substring(8);
    int    slash     = rest.indexOf('/');
    if (slash < 0) return false;
    String redirHost = rest.substring(0, slash);
    String redirPath = rest.substring(slash);
    return downloadAndFlash(redirHost, 443, redirPath,
                            isGitHubHost(redirHost), depth + 1);
  }

  if (statusCode != 200) {
    Serial.println("[OTA] HTTP " + String(statusCode));
    _client->stop();
    return false;
  }

  // ── Stream body → OTA flash ───────────────────────────────────
  Serial.println("[OTA] Content-Length: " + String(contentLength) + " bytes");
  if (!Update.begin(contentLength > 0 ? contentLength : UPDATE_SIZE_UNKNOWN)) {
    Serial.println("[OTA] Update.begin() failed: " + String(Update.errorString()));
    _client->stop();
    return false;
  }

  uint8_t buf[512];
  int     written   = 0;
  int     remaining = (contentLength > 0) ? contentLength : INT_MAX;

  while ((_client->connected() || _client->available()) && remaining > 0) {
    int avail = _client->available();
    if (avail > 0) {
      int n = _client->read(buf, min(avail, (int)sizeof(buf)));
      n = min(n, remaining);
      if (n > 0) {
        if ((int)Update.write(buf, n) != n) {
          Serial.println("[OTA] Flash write error: " + String(Update.errorString()));
          _client->stop();
          Update.abort();
          return false;
        }
        written   += n;
        remaining -= n;
        if (written % 20480 == 0)
          Serial.printf("[OTA] Written %d / %d bytes\n", written, contentLength);
      }
    } else {
      delay(5);
    }
  }
  _client->stop();

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

  // 3. Download and flash — pass auth=true; stripped on CDN redirects
  if (!downloadAndFlash(binHost, binPort, binPath, true)) {
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
