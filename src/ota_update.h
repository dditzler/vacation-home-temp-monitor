#pragma once

// ============================================================
//  OTA Update Module — LilyGO T-SIM7000G
//
//  Supports PRIVATE GitHub repos via PAT auth header.
//  Checks sensor-version.json on raw.githubusercontent.com,
//  then downloads the binary from the GitHub Releases API.
//  Auth header is sent only to GitHub/githubusercontent hosts;
//  S3/CDN redirect targets skip it (they use signed URLs).
//
//  Works over WiFi (WiFiClientSecure) and cellular (SSLClient).
//
//  Required lib (add to platformio.ini lib_deps):
//    arduino-libraries/ArduinoHttpClient @ ^0.6.1
// ============================================================

#include <Arduino.h>
#include <Client.h>
#include "version.h"

// ──────────────────────────────────────────────────────────────
//  Configuration
//  Set via -D build flags in platformio.ini, or define before
//  including this header, or edit the defaults below.
// ──────────────────────────────────────────────────────────────

#ifndef OTA_GITHUB_HOST
  #define OTA_GITHUB_HOST  "raw.githubusercontent.com"
#endif

// OTA_GITHUB_USER, OTA_GITHUB_REPO, OTA_GITHUB_TOKEN must be
// defined in main.cpp or as -D build flags.
#ifndef OTA_GITHUB_USER
  #error "OTA_GITHUB_USER is not defined."
#endif
#ifndef OTA_GITHUB_REPO
  #error "OTA_GITHUB_REPO is not defined."
#endif
#ifndef OTA_GITHUB_TOKEN
  #error "OTA_GITHUB_TOKEN is not defined — required for private repo OTA."
#endif

// sensor-version.json is updated by GitHub Actions on each sensor release tag.
#ifndef OTA_VERSION_PATH
  #define OTA_VERSION_PATH \
    "/" OTA_GITHUB_USER "/" OTA_GITHUB_REPO "/main/sensor-version.json"
#endif

// How often to check for updates (seconds). Default: once per hour.
#ifndef OTA_CHECK_INTERVAL_S
  #define OTA_CHECK_INTERVAL_S  3600
#endif

// ──────────────────────────────────────────────────────────────
//  Public API
// ──────────────────────────────────────────────────────────────

/**
 * Call once from setup() after WiFi + SSL client are ready.
 * @param sslClient  A connected WiFiClientSecure (or any TLS Client).
 */
void otaInit(Client& sslClient);

/**
 * Call from loop() — internally rate-limited to OTA_CHECK_INTERVAL_S.
 * Returns true only if a new firmware was flashed (device reboots first).
 */
bool otaLoop();

/**
 * Force an immediate check regardless of the rate-limit timer.
 * Useful when triggered by an MQTT command.
 */
bool otaCheckNow();
