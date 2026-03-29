#pragma once

// ============================================================
//  OTA Update Module — LilyGO T-SIM7000G
//
//  Checks a version.json file on GitHub (raw.githubusercontent.com,
//  no redirect) and downloads + flashes the new firmware binary
//  from GitHub Releases if a newer version is available.
//
//  Used on the WiFi path only — SIM7000G does not expose
//  TinyGsmClientSecure, so HTTPS OTA is not available over cellular.
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

// OTA_GITHUB_USER and OTA_GITHUB_REPO must be defined in main.cpp
// (or as -D flags) before this header is included.
#ifndef OTA_GITHUB_USER
  #error "OTA_GITHUB_USER is not defined. Set it in main.cpp or platformio.ini."
#endif
#ifndef OTA_GITHUB_REPO
  #error "OTA_GITHUB_REPO is not defined. Set it in main.cpp or platformio.ini."
#endif

// Path to the version manifest on the default branch.
// GitHub Actions keeps this file current after every release.
#ifndef OTA_VERSION_PATH
  #define OTA_VERSION_PATH \
    "/" OTA_GITHUB_USER "/" OTA_GITHUB_REPO "/main/version.json"
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
