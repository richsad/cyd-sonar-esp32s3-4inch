#pragma once

// ============================================================================
//  A-scope display.
//
//  The instrument is the whole profile, not one number. Three stable reflectors
//  in an ordinary room - a door, a wall, a desk edge - swap places as strongest
//  from ping to ping, so a single-target readout looks wildly jumpy while the
//  underlying measurement is rock solid. Drawing all of them shows that at a
//  glance, and the headline number is just the strongest one labelled.
//
//  Redraw is column-delta: each of the 450 plot columns remembers its height
//  and only the difference is painted. A full clear-and-redraw of the plot is
//  180 KB over SPI, about 36 ms; the delta is a few hundred bytes and the trace
//  keeps up with the ping rate.
//
//  Copyright (c) 2026 Rich Sadowsky. MIT licensed - see LICENSE.
//  Written with Claude Code (Claude Opus 5).
// ============================================================================

#include <Arduino_GFX_Library.h>

#include "config.h"
#include "sonar.h"

static Arduino_DataBus *bus =
    new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, LCD_MISO);
static Arduino_GFX *gfx =
    new Arduino_ST7796(bus, LCD_RST, LCD_ROTATION, LCD_IPS, LCD_W, LCD_H);

static uint8_t s_colH[PLOT_W];    // last drawn height per column
static float   s_fullScale = AGC_MIN_FULLSCALE;  // start zoomed in and attack
                                                // upward; the release is slow by
                                                // design and crawling down from a
                                                // high start costs six seconds of
                                                // flat trace at boot.
static char    s_lastHead[24] = "";
static bool    s_lastHeadDim = false;
static char    s_lastState[16] = "";
static bool    s_hold = false;

struct Button { const char *label; int16_t x; };
static Button s_btn[4] = {{"AVG", 0}, {"REF", 0}, {"BAND", 0}, {"HOLD", 0}};

static inline int16_t btnX(int i) {
  const int16_t total = 4 * BTN_W + 3 * BTN_GAP;
  return (SCREEN_W - total) / 2 + i * (BTN_W + BTN_GAP);
}

// A grid column? Feet marks, so 13 of them across 12 ft.
static inline bool isGridCol(int x) {
  for (int ft = 0; ft <= 12; ft++) {
    const int gx = (int)(ft * 12.0f / RANGE_MAX_IN * PLOT_W);
    if (x == gx) return true;
  }
  return false;
}

static void uiDrawButton(int i, bool active) {
  const int16_t x = btnX(i);
  gfx->fillRect(x, BTN_Y, BTN_W, BTN_H, active ? C_AMBER : C_PANEL);
  gfx->drawRect(x, BTN_Y, BTN_W, BTN_H, active ? C_AMBER : C_GRID);

  char txt[16];
  switch (i) {
    case 0: snprintf(txt, sizeof(txt), "AVG %d", sonarAvgCount()); break;
    case 1: snprintf(txt, sizeof(txt), "REF %s",
                     sonarRefStale() ? "!!" : (sonarHaveRef() ? "ON" : "--")); break;
    case 2: snprintf(txt, sizeof(txt), "%s", sonarStealth() ? "STEALTH" : "WIDE"); break;
    default: snprintf(txt, sizeof(txt), "%s", s_hold ? "HELD" : "HOLD"); break;
  }
  gfx->setTextSize(2);
  gfx->setTextColor(active ? C_BG : C_TEXT);
  const int16_t tw = strlen(txt) * 12;
  gfx->setCursor(x + (BTN_W - tw) / 2, BTN_Y + (BTN_H - 16) / 2);
  gfx->print(txt);
}

static void uiDrawButtons() {
  uiDrawButton(0, false);
  uiDrawButton(1, sonarHaveRef() || sonarRefStale());
  uiDrawButton(2, sonarStealth());
  uiDrawButton(3, s_hold);
}

static void uiInit() {
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);
  gfx->begin(LCD_SPI_HZ);
  gfx->fillScreen(C_BG);

  // Header
  gfx->fillRect(0, 0, SCREEN_W, HEAD_H, C_PANEL);
  gfx->setTextSize(2);
  gfx->setTextColor(C_DIM);
  gfx->setCursor(12, 14);
  gfx->print("SONAR");

  // Plot frame, grid and axis
  gfx->drawFastHLine(PLOT_X, AXIS_Y, PLOT_W, C_DIM);
  for (int ft = 0; ft <= 12; ft++) {
    const int gx = PLOT_X + (int)(ft * 12.0f / RANGE_MAX_IN * PLOT_W);
    gfx->drawFastVLine(gx, PLOT_Y, PLOT_H, C_GRID);
    gfx->drawFastVLine(gx, AXIS_Y, 5, C_DIM);
    if (ft % 2 == 0) {
      char l[6];
      snprintf(l, sizeof(l), "%d'", ft);
      gfx->setTextSize(1);
      gfx->setTextColor(C_DIM);
      gfx->setCursor(gx - 5, AXIS_Y + 9);
      gfx->print(l);
    }
  }

  memset(s_colH, 0, sizeof(s_colH));
  uiDrawButtons();
}

// Blind-zone shading, so the dead near field reads as "not measured" rather
// than "nothing there".
static void uiDrawBlind() {
  const int bx = (int)(sonarBlindIn() / RANGE_MAX_IN * PLOT_W);
  for (int x = 0; x < bx; x++) {
    if (x % 3 == 0) gfx->drawFastVLine(PLOT_X + x, PLOT_Y, PLOT_H, C_GRID);
  }
}

// Repaint the plot from scratch. Needed when the blind zone moves - toggling
// REF changes it - because the delta redraw only touches columns whose height
// changed, and stale hatching is not a height.
static void uiResetPlot() {
  gfx->fillRect(PLOT_X, PLOT_Y, PLOT_W, PLOT_H, C_BG);
  for (int ft = 0; ft <= 12; ft++) {
    const int gx = PLOT_X + (int)(ft * 12.0f / RANGE_MAX_IN * PLOT_W);
    gfx->drawFastVLine(gx, PLOT_Y, PLOT_H, C_GRID);
  }
  memset(s_colH, 0, sizeof(s_colH));
  uiDrawBlind();
}

static void uiDrawTrace() {
  // Auto-gain: the top of the scale tracks the strongest thing outside the
  // blind zone, so a quiet room still shows detail instead of a flat line.
  const int start = inchesToSamples(sonarBlindIn());
  float peak = AGC_MIN_FULLSCALE;
  for (int i = start; i < PROF_LEN; i++) {
    const float v = sonarValue(i);
    if (v > peak) peak = v;
  }
  // The release is deliberately slow so the trace does not pump, but after a
  // real scene change that slowness is just six seconds of wrong scale. When
  // the engine says the scene changed, snap.
  if (sonarDisturbed()) {
    s_fullScale = peak;
  } else {
    const float rate = (peak > s_fullScale) ? AGC_ATTACK : AGC_RELEASE;
    s_fullScale += rate * (peak - s_fullScale);
  }
  if (s_fullScale < AGC_MIN_FULLSCALE) s_fullScale = AGC_MIN_FULLSCALE;

  const float inv = 1.0f / s_fullScale;

  for (int x = 0; x < PLOT_W; x++) {
    // Each column covers a little over two samples; take the strongest, which
    // keeps a narrow peak visible instead of averaging it away.
    const int a = (int)((float)x / PLOT_W * PROF_LEN);
    const int b = (int)((float)(x + 1) / PLOT_W * PROF_LEN);
    float m = 0.0f;
    for (int i = a; i < b && i < PROF_LEN; i++) {
      const float v = sonarValue(i);
      if (v > m) m = v;
    }
    int h = (int)(m * inv * PLOT_H);
    if (h > PLOT_H) h = PLOT_H;
    if (h < 0) h = 0;

    const int prev = s_colH[x];
    if (h == prev) continue;

    const int sx = PLOT_X + x;
    if (h > prev) {
      gfx->drawFastVLine(sx, PLOT_Y + PLOT_H - h, h - prev,
                         h > PLOT_H * 3 / 4 ? C_TRACE_HI : C_TRACE);
    } else {
      gfx->drawFastVLine(sx, PLOT_Y + PLOT_H - prev, prev - h, C_BG);
      // The grid line lived under the erased pixels; put it back.
      if (isGridCol(x)) {
        gfx->drawFastVLine(sx, PLOT_Y + PLOT_H - prev, prev - h, C_GRID);
      }
    }
    s_colH[x] = (uint8_t)h;
  }
}

static void uiDrawPeaks(const Peak *pk, int n, int head) {
  gfx->fillRect(0, LABEL_Y, SCREEN_W, LABEL_H, C_BG);
  gfx->setTextSize(1);
  for (int i = 0; i < n && i < 3; i++) {
    const int x = PLOT_X + (int)((float)pk[i].idx / PROF_LEN * PLOT_W);
    char l[16];
    snprintf(l, sizeof(l), "%s", fmtDist(sonarPeakInches(pk[i].idx)));
    int tx = x - (int)strlen(l) * 3;
    if (tx < 2) tx = 2;
    if (tx > SCREEN_W - (int)strlen(l) * 6 - 2) tx = SCREEN_W - strlen(l) * 6 - 2;
    gfx->setTextColor(i == head ? C_AMBER : C_DIM);
    gfx->setCursor(tx, LABEL_Y + 4);
    gfx->print(l);
  }
}

static void uiDrawHead(const Peak *pk, int n, int head, float pps) {
  // While the lock is re-acquiring, the last good reading stays on screen in
  // grey rather than blanking to "--". A number that vanishes for six pings and
  // comes back reads as a fault; a number that greys out reads as "hold on".
  char txt[24];
  const bool dim = (head < 0);
  if (head >= 0) snprintf(txt, sizeof(txt), "%s", fmtDist(sonarPeakInches(pk[head].idx)));
  else if (s_lastHead[0]) snprintf(txt, sizeof(txt), "%s", s_lastHead);
  else snprintf(txt, sizeof(txt), "--");

  if (strcmp(txt, s_lastHead) != 0 || dim != s_lastHeadDim) {
    gfx->fillRect(96, 6, 232, 32, C_PANEL);
    gfx->setTextSize(3);
    gfx->setTextColor(dim ? C_DIM : C_AMBER);
    gfx->setCursor(100, 12);
    gfx->print(txt);
    strncpy(s_lastHead, txt, sizeof(s_lastHead) - 1);
    s_lastHeadDim = dim;
  }

  // Strength and ping rate, right-aligned. Redrawn every frame; it is a small
  // rectangle and the numbers change constantly anyway.
  gfx->fillRect(336, 6, SCREEN_W - 340, 32, C_PANEL);
  char s1[16], s2[16];
  snprintf(s1, sizeof(s1), "%.1f%%", head >= 0 ? pk[head].mag * 100.0f : 0.0f);
  snprintf(s2, sizeof(s2), "%.1f/s", pps);
  gfx->setTextSize(1);
  gfx->setTextColor(C_TEXT);
  gfx->setCursor(SCREEN_W - 8 - strlen(s1) * 6, 12);
  gfx->print(s1);
  gfx->setTextColor(C_DIM);
  gfx->setCursor(SCREEN_W - 8 - strlen(s2) * 6, 26);
  gfx->print(s2);
}

// The settling badge, under the SONAR wordmark. This is the whole point of the
// adaptive averaging being visible: a reading that is still converging looks
// exactly like a settled one unless something says so.
static void uiDrawState() {
  char st[16];
  uint16_t col;
  if (sonarHandled())         { snprintf(st, sizeof(st), "MOVED");    col = C_AMBER; }
  else if (sonarDisturbed())  { snprintf(st, sizeof(st), "CHANGING"); col = C_AMBER; }
  else if (sonarSettled())    { snprintf(st, sizeof(st), "LOCK");     col = C_TRACE; }
  else { snprintf(st, sizeof(st), "%d/%d", sonarNeff(), sonarAvgCount()); col = C_DIM; }

  if (strcmp(st, s_lastState) == 0) return;
  gfx->fillRect(10, 28, 80, 10, C_PANEL);
  gfx->setTextSize(1);
  gfx->setTextColor(col);
  gfx->setCursor(12, 29);
  gfx->print(st);
  strncpy(s_lastState, st, sizeof(s_lastState) - 1);
}

// Returns the button index hit, or -1.
//
// The hit zone is deliberately larger than the drawn button. Bench testing
// produced a stream of misses at y=319 - the last row of pixels, right against
// the bezel - because the buttons are drawn 276..316 and a finger aimed at the
// bottom edge of the screen lands below them. It also absorbs the 4 px gaps
// between buttons by taking the nearest, rather than dropping the press.
//
// A press that hits nothing is a press the user has to repeat while wondering
// whether the hardware works. Generous targets cost nothing here: there is
// nothing else along the bottom of the screen to hit by accident.
static int uiHitTest(int16_t x, int16_t y) {
  if (y < BTN_Y - 10 || y >= SCREEN_H) return -1;
  const int16_t x0 = btnX(0);
  const int16_t stride = BTN_W + BTN_GAP;
  if (x < x0 - BTN_GAP) return -1;
  int i = (x - x0) / stride;
  if (i < 0 || i > 3) return -1;
  return i;
}
