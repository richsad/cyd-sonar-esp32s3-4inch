// ============================================================================
//  CYD Sonar - an acoustic tape measure on the Hosyond ESP32-S3 4.0" module.
//
//  The board chirps out of its own speaker, listens on its own microphone, and
//  cross-correlates to find the echoes. No host, no extra hardware, no
//  ultrasonic module - the speaker's output IS the microphone's stimulus.
//
//  See sonar.h for why the direct path is the clock, which is the one idea the
//  whole thing rests on.
//
//  Build:
//    arduino-cli compile --upload -p /dev/cu.usbmodemXXXX \
//      --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,CDCOnBoot=cdc Sonar
//
//  Copyright (c) 2026 Rich Sadowsky. MIT licensed - see LICENSE.
//  Written with Claude Code (Claude Opus 5).
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>

#include "config.h"

#define FT_REG_TD_STATUS 0x02
#define FT_REG_CHIP_ID   0xA3
#define FT_REG_VENDOR_ID 0xA8

#include "touch.h"
#include "sonar.h"
#include "ui.h"

static void pollTouch();

static Peak s_peaks[3];
static int  s_nPeaks = 0;
static int  s_head = -1;
static uint32_t s_touchRecoveries = 0;
static float s_pps = 0.0f;

void setup() {
  Serial.begin(115200);

  // Never block on USB.
  //
  // This is the native USB-Serial-JTAG port. Once the host has enumerated it
  // but no application is reading, the TX buffer fills and Serial.write stalls
  // waiting for a reader that is not coming. Everything in this sketch runs in
  // loop() - including the button handler - so a stalled print is a stalled
  // instrument, and a telemetry line goes out every second.
  //
  // That fits the bench report exactly, and fits the part that kept confusing
  // me: the buttons behaved whenever a capture was attached and draining the
  // port, and went unresponsive when nothing was. I had been reading that as
  // the fault being intermittent.
  Serial.setTxTimeoutMs(0);
  delay(300);
  Serial.println("\n=== CYD Sonar ===");

  // Why did we just start?
  //
  // The touch wedge ends with "a few fast clicks and then the buttons work" -
  // which is not a recovery, it is this sketch booting: the amp-enable pop plus
  // three back-to-back warm-up pings. That also explains the one thing that
  // made no sense, that only a full board reset clears the wedge. It was never
  // that a full reset is REQUIRED; it is that a full reset is what happens.
  //
  // So the question stops being "why does touch stop" and becomes "why does the
  // board reboot", which this answers outright. A brownout points at the supply
  // - amp plus backlight on USB power - and a panic or watchdog points at the
  // firmware, and they need completely different fixes.
  const esp_reset_reason_t rr = esp_reset_reason();
  static const char *kReason[] = {"unknown", "power-on", "external", "software",
                                  "panic", "interrupt wdt", "task wdt", "other wdt",
                                  "deep sleep", "BROWNOUT", "sdio"};
  Serial.printf("[boot] reset reason %d (%s), free heap %u\n", (int)rr,
                (rr < (sizeof(kReason) / sizeof(*kReason))) ? kReason[rr] : "?",
                (unsigned)ESP.getFreeHeap());

  Wire.begin(TOUCH_SDA, TOUCH_SCL, TOUCH_I2C_HZ);
  uiInit();

  sonarSetPingHook(pollTouch);

  if (!touchInit()) Serial.println("[touch] FT6336 not answering");
  else Serial.printf("[touch] chip 0x%02X vendor 0x%02X\n",
                     touchChipId(), touchVendorId());

  if (!sonarAudioInit()) {
    Serial.println("[audio] init FAILED");
    gfx->setTextSize(2);
    gfx->setTextColor(C_AMBER);
    gfx->setCursor(20, PLOT_Y + 60);
    gfx->print("AUDIO INIT FAILED");
    return;
  }
  // Warm-up, discarded. The PA is enabled moments before this and its settling
  // transient correlates as a fat false echo out to about 2.5 ft - 8% of the
  // direct path, twice any real reflector in the room. Left in, the auto-gain
  // attacks up to it instantly and then spends six seconds releasing back down
  // with the trace squashed flat the whole time.
  for (int i = 0; i < 3; i++) sonarPing();
  s_pingCount = 0;  // and start the average clean

  uiDrawBlind();
  Serial.printf("ready - %d Hz, chirp %d samples, span %.0f in\n",
                SAMPLE_RATE, CHIRP_LEN, RANGE_MAX_IN);
}

// Touch is polled between pings rather than on a timer. A ping is ~120 ms, so
// worst-case button latency is about that - fast enough that it reads as
// instant, and it keeps the I2C traffic away from the capture, which is where
// the FT6336's bus noise showed up as phantom touches on the radio build.
// Serial mirrors of the four buttons, plus a profile dump. The panel is the
// instrument, but every button needs a finger, and the bench protocol has to be
// runnable from a machine that cannot reach the board - including from a
// different room, or by someone driving it remotely.
//
//   a  cycle AVG      r  toggle REF     b  toggle BAND
//   h  toggle HOLD    p  dump profile   ?  state
static void serialCmds() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == 'a') { sonarCycleAvg();               uiDrawButtons(); Serial.printf("[cmd] AVG %d\n", sonarAvgCount()); continue; }
    if (c == 'r') { sonarToggleRef(); uiResetPlot(); uiDrawButtons(); Serial.printf("[cmd] REF %s\n", sonarHaveRef() ? "ON" : "off"); continue; }
    if (c == 'b') { sonarSetStealth(!sonarStealth()); uiDrawButtons(); Serial.printf("[cmd] BAND %s\n", sonarStealth() ? "STEALTH" : "WIDE"); continue; }
    if (c == 'h') { s_hold = !s_hold;              uiDrawButtons(); Serial.printf("[cmd] %s\n", s_hold ? "HELD" : "running"); continue; }
    if (c == 't') {
      uint8_t id = 0xFF, ven = 0xFF;
      const bool okId  = touchReadReg(FT_REG_CHIP_ID, &id, 1);
      const bool okVen = touchReadReg(FT_REG_VENDOR_ID, &ven, 1);
      char scan[96];
      const int nDev = touchScanBus(scan, sizeof(scan));
      Serial.printf("[touch] addr 0x%02X  chip 0x%02X(%s) vendor 0x%02X(%s)  "
                    "fails %u  last raw %d,%d st 0x%02X  bus: %d [%s]\n",
                    touchAddr(), id, okId ? "ok" : "NAK", ven, okVen ? "ok" : "NAK",
                    touchFailStreak(), touchRawX(), touchRawY(), touchRawStatus(),
                    nDev, scan);
      // Full register dump. Every theory so far has been a guess about what
      // the part is doing internally; this asks it.
      //
      // 0xA5 is the one that matters. On the FT6x06 family it is PWR_MODE:
      // 0 Active, 1 Monitor, 3 Hibernate - and "answers I2C but has stopped
      // scanning" is exactly what a sleeping part looks like from outside. The
      // earlier keep-awake write targeted 0xA4, which is the interrupt mode
      // register, not the power one. That guess was simply aimed at the wrong
      // address, and removing it was still right: it fixed nothing.
      Serial.print("[touch] regs 0x00-0x06:");
      for (uint8_t r = 0x00; r <= 0x06; r++) {
        uint8_t v = 0xFF;
        Serial.printf(" %02X=%s", r,
                      touchReadReg(r, &v, 1) ? String(v, HEX).c_str() : "NAK");
      }
      Serial.print("\n[touch] regs 0x80,0x88,0xA0-0xA8:");
      const uint8_t interesting[] = {0x80, 0x88, 0xA0, 0xA1, 0xA2, 0xA3,
                                     0xA4, 0xA5, 0xA6, 0xA7, 0xA8};
      for (uint8_t r : interesting) {
        uint8_t v = 0xFF;
        Serial.printf(" %02X=%s", r,
                      touchReadReg(r, &v, 1) ? String(v, HEX).c_str() : "NAK");
      }
      Serial.println();
      Serial.printf("[touch] RST pin %d is %s\n", TOUCH_RST,
                    touchProveReset() ? "REAL - holds the part in reset"
                                      : "A NO-OP - the part answers while it is asserted");
      touchRecover();
      Serial.printf("[touch] forced recover -> alive %s\n", touchAlive() ? "yes" : "NO");
      continue;
    }
    if (c == '?') { Serial.printf("[state] n %d/%d  ref %s%s  band %s  chg %.2f base %.2f\n",
                                  sonarNeff(), sonarAvgCount(), sonarHaveRef() ? "on" : "off",
                                  sonarRefStale() ? " STALE" : "", sonarStealth() ? "stealth" : "wide",
                                  sonarChange(), s_changeBase); continue; }
    if (c != 'p') continue;
    Serial.println("\nprofile (max per bin, % of direct):");
    for (int c = 0; c < 48; c++) {
      const int a = c * PROF_LEN / 48, b = (c + 1) * PROF_LEN / 48;
      float m = 0.0f;
      for (int i = a; i < b; i++) { const float v = sonarValue(i); if (v > m) m = v; }
      int h = (int)(m * 300.0f);
      if (h > 40) h = 40;
      Serial.printf("%9s  %5.2f%%  %.*s\n", fmtDist(samplesToInches(a)),
                    m * 100.0f, h, "########################################");
    }
  }
}

// Touch is POLLED inside the capture loop but ACTED ON outside it.
//
// The poll itself is a ~1 ms I2C read and harmless there. Running the handler
// there was not: REF calls uiResetPlot(), which repaints 450x180 pixels - about
// 162 KB over SPI, some 32 ms - from inside the I2S write/read lockstep. TX
// starves, RX drops samples, that ping's capture is corrupt, and the speaker
// makes noises that are not the chirp. Logging from here cost the same way, in
// smaller pieces: the ping rate had quietly fallen from 8.2 to 7.8.
//
// So the hook only ever records. Everything with a cost happens in loop().
static volatile int  s_pendingBtn = -1;
static char s_touchLog[96];
static volatile bool s_haveTouchLog = false;

static void pollTouch() {
  static bool wasDown = false;
  static uint32_t lastUp = 0;
  static uint32_t downSince = 0;
  static uint32_t lastStill = 0;

  const TouchPoint t = touchRead();

  if (t.pressed && !wasDown) {
    downSince = millis();
    if (!s_haveTouchLog) {
      snprintf(s_touchLog, sizeof(s_touchLog), "DOWN x=%3d y=%3d st 0x%02X",
               t.x, t.y, touchRawStatus());
      s_haveTouchLog = true;
    }
  } else if (!t.pressed && wasDown) {
    if (!s_haveTouchLog) {
      snprintf(s_touchLog, sizeof(s_touchLog), "UP after %lu ms",
               (unsigned long)(millis() - downSince));
      s_haveTouchLog = true;
    }
  } else if (t.pressed && wasDown && millis() - downSince > 2000 &&
             millis() - lastStill > 2000) {
    // A touch that never lifts would block every later press, since presses are
    // only accepted on a rising edge. Rate-limited to once every two seconds -
    // the first version's window let it fire on ten consecutive polls.
    lastStill = millis();
    if (!s_haveTouchLog) {
      snprintf(s_touchLog, sizeof(s_touchLog), "STILL DOWN %lu ms st 0x%02X",
               (unsigned long)(millis() - downSince), touchRawStatus());
      s_haveTouchLog = true;
    }
  }

  if (t.pressed && !wasDown && millis() - lastUp > 180) {
    const int b = uiHitTest(t.x, t.y);
    if (b >= 0) s_pendingBtn = b;
  }
  if (!t.pressed && wasDown) lastUp = millis();
  wasDown = t.pressed;
}

// Drained from loop(), where a 32 ms repaint costs nothing but a little latency.
static void serviceTouch() {
  if (s_haveTouchLog) {
    Serial.printf("[touch] %s\n", s_touchLog);
    s_haveTouchLog = false;
  }
  const int b = s_pendingBtn;
  if (b < 0) return;
  s_pendingBtn = -1;

  switch (b) {
    case 0: sonarCycleAvg(); break;
    case 1: sonarToggleRef(); uiResetPlot(); break;
    case 2: sonarSetStealth(!sonarStealth()); uiResetPlot(); break;
    case 3: s_hold = !s_hold; break;
  }
  uiDrawButtons();
  Serial.printf("[touch] -> %s\n", s_btn[b].label);
}

// Telemetry, emitted from loop() on a timer.
//
// It used to be printed after a successful ping, which meant it went silent in
// exactly the states worth watching: HOLD stops pinging altogether, and a
// collapsed ping rate makes the lines sparse enough to miss. Reported from the
// bench as "a state between chirps where the buttons go inactive" - and there
// was no way to tell from the log whether that was HOLD, a stalled capture, or
// a slow one, because the log itself had stopped.
//
// Driven by the clock now, so the quiet states describe themselves.
static void logTelemetry() {
  static uint32_t lastLog = 0;
  if (millis() - lastLog < 1000) return;
  lastLog = millis();

  // No host session, no telemetry. Belt and braces alongside the zero TX
  // timeout: nothing to serialise, nothing to drop, no time spent formatting a
  // line that has nowhere to go.
  if (!Serial) return;

  Serial.printf("%4.1f/s  n %2d/%-2d %-8s %s%s  direct %6.0f  chg %.2f/%.2f |",
                s_pps, sonarNeff(), sonarAvgCount(),
                s_hold ? "HELD"
                       : (sonarHandled() ? "MOVED"
                       : (sonarDisturbed() ? "CHANGING"
                       : (sonarSettled() ? "LOCK" : "settling"))),
                sonarRefStale() ? "REF!" : (sonarHaveRef() ? "REF " : "    "),
                sonarStealth() ? " STL" : "",
                s_directMag, sonarChange(),
                s_changeBase * CHANGE_TRIGGER_K + CHANGE_FLOOR);
  if (s_touchRecoveries) Serial.printf(" [tch rec %u]", s_touchRecoveries);
  if (sonarStalls()) Serial.printf(" [i2s stall %u]", sonarStalls());
  Serial.printf(" tch st 0x%02X fails %u  up %lus heap %u", touchRawStatus(),
                touchFailStreak(), (unsigned long)(millis() / 1000),
                (unsigned)ESP.getFreeHeap());
  for (int i = 0; i < s_nPeaks; i++) {
    Serial.printf("  %c%-9s %4.1f%%", i == s_head ? '*' : ' ',
                  fmtDist(sonarPeakInches(s_peaks[i].idx)),
                  s_peaks[i].mag * 100.0f);
  }
  Serial.println();
}

void loop() {
  serialCmds();
  pollTouch();
  serviceTouch();
  logTelemetry();

  if (s_hold) { delay(20); serviceTouch(); return; }

  const uint32_t t0 = millis();
  if (!sonarPing()) {
    // Either no correlation at all, or a capture block timed out. Both are
    // worth one line, not a silent retry loop.
    static uint32_t lastMoan = 0;
    if (millis() - lastMoan > 2000) {
      lastMoan = millis();
      Serial.printf("[ping] aborted - stalls %u\n", sonarStalls());
    }
    delay(20);
    return;
  }

  // Peaks above 1% of the direct path. Below that it is noise, and labelling
  // noise makes the display look busier rather than more informative.
  s_nPeaks = sonarFindPeaks(s_peaks, 3, 0.01f);
  s_head = sonarHeadline(s_peaks, s_nPeaks);

  uiDrawTrace();
  uiDrawPeaks(s_peaks, s_nPeaks, s_head);
  uiDrawHead(s_peaks, s_nPeaks, s_head, s_pps);
  uiDrawState();

  // --- touch watchdog ----------------------------------------------------
  //
  // A wedged FT6336 does not report an error. It reports "nobody is touching
  // the screen", which is exactly what a working one reports most of the time,
  // so the buttons simply stop and nothing anywhere says why. The only way to
  // tell is to ask the controller who it is, which is cheap enough to do every
  // couple of seconds.
  //
  // Serviced HERE, between pings, and never from the capture hook: recovery
  // holds RST low and sleeps 150 ms, which would underrun the I2S DMA mid-ping.
  static uint32_t lastHealth = 0;
  bool recover = touchWantRecover();
  if (!recover && millis() - lastHealth >= 2000) {
    lastHealth = millis();
    if (!touchAlive()) {
      recover = true;
      Serial.println("[touch] controller not answering - recovering");
    }
  }
  if (recover) {
    touchRecover();
    s_touchRecoveries++;
    Serial.printf("[touch] recovered (#%u) -> alive %s\n",
                  s_touchRecoveries, touchAlive() ? "yes" : "NO");
    uiDrawButtons();
  }

  const uint32_t dt = millis() - t0;
  const float inst = dt > 0 ? 1000.0f / dt : 0.0f;
  s_pps += 0.1f * (inst - s_pps);


}
