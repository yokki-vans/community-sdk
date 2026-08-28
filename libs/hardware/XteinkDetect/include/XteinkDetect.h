#pragma once

// FreeInk SDK — Xteink X3/X4 runtime detection.
//
// The Xteink X3 and X4 are two BoardProfiles compiled into one ESP32-C3 binary.
// They share a pinout but differ in panel controller (X3 = UC8253 792x528,
// X4 = SSD1677 800x480) and battery backend, so the running firmware must pick
// the right one before bringing up the display and SD card. The SDK leaves
// detection to the consumer (BoardConfig.h header note); this helper supplies
// the canonical Xteink fingerprint so dual X3/X4 apps don't each reinvent it.
//
// Detection probes the X3-only I2C peripherals on SDA=20 / SCL=0 — the BQ27220
// fuel gauge (0x55), DS3231 RTC (0x68) and QMI8658 IMU (0x6B/0x6A). The X4 has
// none of them, so at least two of five passes scoring >= 2 hits confirm an X3.
// Anything else may use X4 as a per-boot fallback but is never cached as X4.
//
// In builds without an Xteink profile (neither FREEINK_DEVICE_X4 nor
// FREEINK_DEVICE_X3) both functions compile to no-ops returning false and never
// touch a pin: the probe bus (SDA=20 / SCL=0) is only safe on the Xteink C3
// pinout — on an ESP32-S3 those are native USB D+ and the boot strap.

#include <stddef.h>
#include <stdint.h>

namespace freeink {

// Probe outcome. X3Confirmed means at least two of five passes positively saw
// two X3-only peripherals. X4Confirmed is the legacy API name for five all-zero
// passes: callers may use X4 as the geometry-safe fallback for that boot, but
// must not persist it because an unavailable X3 I2C bus looks identical.
enum class XteinkVerdict : uint8_t { X4Confirmed, X3Confirmed, Inconclusive };

// Run the X3 I2C fingerprint and return the verdict. Optionally reports the
// first two per-pass chip-hit scores (0-3) for diagnostics. Leaves the I2C bus
// released and the probe pins back in INPUT mode. Safe to call before any other
// hardware bring-up. In builds without an Xteink profile this is a no-op
// returning Inconclusive with zero scores.
XteinkVerdict detectXteinkVerdict(uint8_t* score1 = nullptr, uint8_t* score2 = nullptr);

// Run the X3 I2C fingerprint and return true if this board is an Xteink X3.
// Leaves the I2C bus released and the probe pins back in INPUT mode. Safe to
// call before any other hardware bring-up.
bool detectXteinkIsX3();

// --- X3 display-controller fingerprint ---------------------------------------
// Newer X3 production units ship a UC8279d panel controller instead of the
// UC8253 (same board, glass and pinout). The two are told apart by reading the
// UC8279's VER (0x70: reserved 0x00 + CHIP_VER + 24-bit LUT_VER) and FLG
// (0x71: status, BUSY_N=1 when idle) registers over a bit-banged half-duplex
// 4-wire SPI on the X3 display pins, after a hardware reset pulse. The UC8253
// either doesn't answer 0x70 (bus floats) or answers with a different byte
// shape, so two matching signatures across three passes confirm the new
// controller. Anything else is inconclusive; callers may use the shipping
// UC8253 profile only as a non-persistent fallback. Safe to call before
// FreeInkDisplay::begin() — the pins are released afterwards and the driver
// re-resets the panel.
enum class X3DisplayVerdict : uint8_t { Uc8253Assumed, Uc8279Confirmed, Inconclusive };

// Probe the X3 display controller. Optionally reports the raw VER bytes and
// FLG byte from the first successful pass (for bring-up logging / tuning on new
// hardware). Only meaningful on a confirmed X3; in builds without
// FREEINK_DEVICE_X3 this is a no-op returning Uc8253Assumed.
X3DisplayVerdict detectX3DisplayController(uint8_t verBytes[5] = nullptr, uint8_t* flg = nullptr);

// --- Board-agnostic display-controller fingerprint ---------------------------
// Newer production runs of several Xteink panels swap their default controller
// for an UltraChip sibling that shares the UC81xx KW-mode command set: the X3's
// UC8253 -> UC8279d, and the X4 / X4 Pro's SSD1677 -> UC8179. All UC81xx parts
// answer a VER (0x70) / FLG (0x71) read; the SSD-family and UC8253 parts do not
// answer 0x70 the same way, so two matching UC81xx signatures across three
// passes confirm the sibling silicon. Unlike detectX3DisplayController (which
// hard-codes the X3 C3 pinout), this reads the pins from BoardConfig::ACTIVE,
// so it works on any Xteink profile — including the S3 X4 Pro, where the X3 I2C
// probe would be unsafe. Bit-bangs a half-duplex 4-wire SPI after a reset pulse
// and leaves the pins released; safe to call before FreeInkDisplay::begin().
enum class DisplayControllerVerdict : uint8_t { PrimaryAssumed, Uc81xxConfirmed, Inconclusive };

// Factory calibration fallback for a live probe that could not establish a
// stable read.  Xteink uses different screenType values for the two controller
// families: 1/0x0B identify the UC8179 sibling of SSD1677, while 2/0x0C
// identify the UC8279 sibling of UC8253.  Keeping this decision pure makes it
// testable and, critically, prevents an X4 calibration value from selecting an
// X3 driver (or vice versa).
inline bool oemScreenTypeMatchesUltraChip(uint8_t screenType, bool x3Family) {
  return x3Family ? (screenType == 2 || screenType == 0x0C) : (screenType == 1 || screenType == 0x0B);
}

// Validate an RMTP read used to distinguish a blank-MTP UC8279D from a
// floating UC8253 bus. A programmed controller starts with the unambiguous
// 0xA5 enable key. Blank-MTP field panels instead expose a stable non-uniform
// dump, so the complete payload must repeat exactly on a second read.
inline bool uc81xxMtpReadbackIsValid(const uint8_t* first, const uint8_t* second, size_t length) {
  if (first == nullptr || length == 0) return false;
  if (first[0] == 0xA5) return true;

  bool uniform = true;
  for (size_t i = 1; i < length; ++i) {
    if (first[i] != first[0]) {
      uniform = false;
      break;
    }
  }
  if (uniform || second == nullptr) return false;
  for (size_t i = 0; i < length; ++i) {
    if (first[i] != second[i]) return false;
  }
  return true;
}

DisplayControllerVerdict detectXteinkDisplayController(uint8_t verBytes[5] = nullptr, uint8_t* flg = nullptr);

// Convenience: resolve which panel controller this unit carries and, when it is
// the UltraChip sibling, promote BoardConfig::ACTIVE.displayController to it
// (SSD1677 -> UC8179, UC8253 -> UC8279) so FreeInkDisplay::begin() selects the
// matching driver. A confirmed live display-bus probe is authoritative. If the
// live read is inconclusive, a family-matching OEM hw_calib/screenType value is
// accepted as a fallback; values for the other family are rejected, which
// avoids selecting the wrong driver after a foreign full-flash. Leaves the
// profile's default controller in place when neither source confirms its
// UltraChip sibling. Returns true iff the controller was promoted. Call before
// FreeInkDisplay::begin(). In
// builds without a probe-capable profile this is a no-op returning false.
bool applyXteinkDisplayController();

// Convenience: run detectXteinkIsX3(), set BoardConfig::ACTIVE to the matching
// profile via selectDevice(), and return whether an X3 was detected (so the
// caller can put FreeInkDisplay in X3 mode with setDisplayX3()). On a
// confirmed X3 the display-controller probe also runs, selecting the UC8279
// sibling profile when that controller is fingerprinted. Call this before
// SDCardManager::begin() and FreeInkDisplay::begin() so both read the correct
// profile.
bool selectXteinkDevice();

}  // namespace freeink
