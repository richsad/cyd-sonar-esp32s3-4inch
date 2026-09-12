#pragma once

// ============================================================================
//  FT6336G capacitive touch, over I2C.
//
//  Written out longhand rather than pulled from a library for two reasons: it
//  is about forty lines, and bring-up needs to *scan* the bus and report what
//  answered rather than assume 0x38 and fail silently. Most CYD boards use a
//  GT911, so copy-pasted CYD touch code will not work here.
//
//  Copyright (c) 2026 Rich Sadowsky. MIT licensed - see LICENSE.
//  Written with Claude Code (Claude Opus 5).
// ============================================================================

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

// FocalTech register map (FT6x36 family).
#define FT_REG_TD_STATUS 0x02  // low nibble = number of active touches
#define FT_REG_P1_XH     0x03  // [3:0] X high bits; [7:6] event flag
#define FT_REG_CHIP_ID   0xA3
#define FT_REG_VENDOR_ID 0xA8  // 0x11 = FocalTech

struct TouchPoint {
  bool pressed;
  int16_t x;
  int16_t y;
};

static uint8_t s_touchAddr = TOUCH_I2C_ADDR;
static uint8_t s_touchChipId = 0;
static uint8_t s_touchVendorId = 0;

// Last raw controller reading, before any rotation mapping. Exposed so the
// mapping can be checked against ground truth instead of inferred.
static int16_t s_rawX = -1, s_rawY = -1;
static uint8_t s_rawStatus = 0;

// Consecutive failed reads. The FT6336 can wedge the bus, and a silent failure
// is indistinguishable from "nobody is touching the screen" - which is exactly
// how the controller dying went unnoticed until the buttons stopped working.
static uint16_t s_failStreak = 0;
static bool s_wantRecover = false;

static bool touchWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(s_touchAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// REMOVED: a write of 0x00 to register 0xA4, added as a guess that the part
// was dropping to Monitor mode and not waking.
//
// It did not help - the controller wedged again with it in place - and 0xA4 is
// not confirmed to be the CTRL register on the FT6336G specifically. Writing a
// register whose meaning is unverified, to fix a fault it demonstrably does not
// fix, is strictly worse than not writing it: it is one more thing that has to
// be ruled out next time. If the part really does sleep, that gets established
// first and fixed second.

static bool touchReadReg(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(s_touchAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)s_touchAddr, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// Scans the bus and writes a human-readable summary into `out`. This is the
// single most useful thing the bring-up sketch prints: if the pins are wrong
// the scan finds nothing, and if the chip is not what we assumed the vendor ID
// says so.
static int touchScanBus(char *out, size_t outLen) {
  int found = 0;
  size_t used = 0;
  out[0] = '\0';
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      found++;
      if (used < outLen - 8) {
        used += snprintf(out + used, outLen - used, "%s0x%02X", used ? " " : "", addr);
      }
      // Prefer a device that answers at the expected address.
      if (addr == TOUCH_I2C_ADDR) s_touchAddr = addr;
      else if (found == 1) s_touchAddr = addr;
    }
  }
  if (!found) snprintf(out, outLen, "none - check SDA/SCL pins");
  return found;
}

// Is GPIO18 actually the reset line? Nobody has ever checked. touchRecover()
// pulses it and has been assumed to work, but a forced recover in the wedged
// state changed nothing at all - which is exactly what a no-op looks like.
//
// Held low, a real RST makes the controller stop answering I2C. If the chip ID
// still reads back while RST is asserted, the pin is not what we think it is
// and every recovery so far has been theatre.
static bool touchProveReset() {
  if (TOUCH_RST < 0) return false;
  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, LOW);
  delay(30);
  uint8_t id = 0xFF;
  const bool answeredWhileHeld = touchReadReg(FT_REG_CHIP_ID, &id, 1);
  digitalWrite(TOUCH_RST, HIGH);
  delay(200);
  return !answeredWhileHeld;  // true = the pin really does hold it in reset
}

static bool touchInit() {
  pinMode(TOUCH_INT, INPUT_PULLUP);
  if (TOUCH_RST >= 0) {
    pinMode(TOUCH_RST, OUTPUT);
    digitalWrite(TOUCH_RST, LOW);
    delay(10);
    digitalWrite(TOUCH_RST, HIGH);
    delay(150);  // FT6336 needs ~100 ms after reset before it answers
  }
  Wire.begin(TOUCH_SDA, TOUCH_SCL, TOUCH_I2C_HZ);

  touchReadReg(FT_REG_CHIP_ID, &s_touchChipId, 1);
  return touchReadReg(FT_REG_VENDOR_ID, &s_touchVendorId, 1);
}

static uint8_t touchChipId()   { return s_touchChipId; }
static uint8_t touchVendorId() { return s_touchVendorId; }
static uint8_t touchAddr()     { return s_touchAddr; }

// Pulse RST and bring the bus back up. Cheap, and the only reliable way out of
// a wedged controller short of a reboot.
static void touchRecover() {
  Wire.end();
  if (TOUCH_RST >= 0) {
    pinMode(TOUCH_RST, OUTPUT);
    digitalWrite(TOUCH_RST, LOW);
    delay(10);
    digitalWrite(TOUCH_RST, HIGH);
    delay(150);
  }
  Wire.begin(TOUCH_SDA, TOUCH_SCL, TOUCH_I2C_HZ);
  s_failStreak = 0;
  s_wantRecover = false;
}

static uint16_t touchFailStreak() { return s_failStreak; }
static bool touchWantRecover() { return s_wantRecover; }

// Is the controller actually alive? A wedged FT6336 reports "nobody is
// touching the screen", which is indistinguishable from the truth - so silence
// proves nothing and the chip ID has to be asked for directly.
static bool touchAlive() {
  uint8_t id = 0xFF;
  if (!touchReadReg(FT_REG_CHIP_ID, &id, 1)) return false;
  return id == s_touchChipId && id != 0xFF;
}

static TouchPoint touchRead() {
  TouchPoint p = {false, 0, 0};
  uint8_t buf[5];
  if (!touchReadReg(FT_REG_TD_STATUS, buf, 5)) {
    // Flag it; do not recover here. touchRecover() holds RST low and then
    // sleeps 150 ms, and in Sonar this function is called from inside the
    // capture loop - blocking there underruns the I2S DMA mid-ping. The main
    // loop services the flag between pings instead.
    if (++s_failStreak > 50) s_wantRecover = true;
    return p;
  }
  s_failStreak = 0;
  s_rawStatus = buf[0];

  // The FT6336 tracks at most two points, so a count above that is a corrupt
  // read, not a touch - st=0x56 ("6 touches") showed up once I2C traffic to the
  // codec started interleaving with touch polling. Rejecting it here stops
  // garbage coordinates from being acted on as button presses.
  uint8_t n = buf[0] & 0x0F;
  if (n == 0 || n > 2) return p;

  // buf[1]=P1_XH buf[2]=P1_XL buf[3]=P1_YH buf[4]=P1_YL
  int16_t x = ((buf[1] & 0x0F) << 8) | buf[2];
  int16_t y = ((buf[3] & 0x0F) << 8) | buf[4];
  s_rawX = x;
  s_rawY = y;

  // Native portrait -> ROT 1 landscape. These constants were measured with the
  // four-corner calibration in PanelTest, not taken from a datasheet: the long
  // axis maps straight through, the short axis is inverted.
  p.pressed = true;
#if TOUCH_SWAP_XY
  p.x = y;
  p.y = TOUCH_INVERT_Y ? (SCREEN_H - 1 - x) : x;
#else
  p.x = TOUCH_INVERT_Y ? (SCREEN_H - 1 - x) : x;
  p.y = y;
#endif
  return p;
}

static int16_t touchRawX() { return s_rawX; }
static int16_t touchRawY() { return s_rawY; }
static uint8_t touchRawStatus() { return s_rawStatus; }
