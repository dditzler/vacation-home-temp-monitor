#pragma once

// ============================================================
//  Firmware Version
//  Bump these values before tagging a release in GitHub.
//  Format: MAJOR.MINOR.PATCH  (values 0-255 each)
//
//  Release workflow:
//    1. Increment the numbers below and update FW_VERSION_STR.
//    2. Commit: git add src/version.h && git commit -m "chore: bump sensor to 1.x.x"
//    3. Tag:    git tag sensor-v1.x.x && git push origin sensor-v1.x.x
//    4. GitHub Actions builds the binary, publishes a Release,
//       and updates sensor-version.json — devices detect it on next OTA check.
// ============================================================

#define FW_VERSION_MAJOR  1
#define FW_VERSION_MINOR  3
#define FW_VERSION_PATCH  0

// Human-readable string printed to Serial on boot and sent in MQTT status.
#define FW_VERSION_STR    "1.3.0"

// Numeric value for integer comparisons during OTA version checks.
// Encoded as 0x00MMNNPP (Major / Minor / Patch).
#define FW_VERSION_INT  ((uint32_t)(FW_VERSION_MAJOR) << 16 | \
                         (uint32_t)(FW_VERSION_MINOR) <<  8 | \
                         (uint32_t)(FW_VERSION_PATCH))
