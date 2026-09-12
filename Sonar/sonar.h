#pragma once

// ============================================================================
//  The ranging engine: chirp out, capture in, matched filter, profile.
//
//  THE DIRECT PATH IS THE CLOCK. The first and largest correlation peak is the
//  speaker leaking straight into the microphone an inch or two away,
//  arriving in well under 0.1 ms. Every echo is timed from that peak, never
//  from when the chirp was queued.
//
//  That is not a convenience, it is the whole reason this works on this board.
//  Measured latency through the codec and the DMA rings is about 22 ms and it
//  is not constant between runs - MicTest saw the direct path land anywhere
//  from sample 1056 to 1063. Timing from the queue would put every reading tens
//  of feet out and jitter it by inches. Timing from the direct path cancels all
//  of it, exactly, on every single ping, with no calibration step and no
//  constant to trim.
//
//  Copyright (c) 2026 Rich Sadowsky. MIT licensed - see LICENSE.
//  Written with Claude Code (Claude Opus 5).
// ============================================================================

#include <Arduino.h>
#include <driver/i2s_std.h>
#include <math.h>

#include "config.h"
#include "es8311.h"

static i2s_chan_handle_t s_tx = nullptr;
static i2s_chan_handle_t s_rx = nullptr;

static float   s_chirpRef[CHIRP_LEN];   // matched filter, zero mean
static int16_t s_chirpPcm[CHIRP_LEN];
static bool    s_stealth = false;

static int16_t s_txFrame[CAP_LEN];
static int16_t s_rxFrame[CAP_LEN];
static float   s_corr[CAP_LEN];

// All three are normalised to the direct-path magnitude, so 0.04 means "4% of
// the direct path" and the numbers stay comparable as gain or volume change.
static float s_prof[PROF_LEN];   // this ping
static float s_prev[PROF_LEN];   // the one before, for the change metric
static float s_avg[PROF_LEN];    // EMA across pings - what gets drawn
static float s_ref[PROF_LEN];    // learned clutter reference, or all zero
static bool  s_haveRef = false;

// Settling state. s_neff is the averaging depth actually in use right now; it
// collapses to 1 the instant the scene changes and climbs back one ping at a
// time toward the depth the user asked for. Fast re-acquisition, then
// progressive smoothing, with no setting to fiddle with.
static float s_change     = 0.0f;  // this ping's change against the average
static float s_changeBase = 0.0f;  // what change looks like when nothing happens
static int   s_neff       = 1;
static bool  s_disturbed  = false;
static bool  s_handled    = false;  // the BOARD moved, not the room
static bool  s_refStale   = false;
static float s_directAvg  = 0.0f;
static int   s_lockIdx    = -1;  // headline target, in profile samples
static int   s_lockMiss   = 0;
static int   s_moveStreak = 0;

static uint8_t s_avgIdx = AVG_DEFAULT;
static int     s_directIdx = -1;
static float   s_directMag = 0.0f;
static uint32_t s_pingCount = 0;

// Capture blocks that timed out. Counted rather than silently absorbed: a
// stalled capture is the difference between a slow instrument and a broken one.
static uint32_t s_i2sStalls = 0;

// Called every fourth capture block so the UI can poll touch mid-ping. A
// function pointer rather than a direct call because the engine must not know
// about the display.
static void (*s_pingHook)() = nullptr;
static void sonarSetPingHook(void (*fn)()) { s_pingHook = fn; }

// The usable near limit right now: the ring's reach normally, far closer once a
// reference has subtracted the ring away.
static inline float sonarBlindIn();

static inline float samplesToInches(int s) {
  return s * SOUND_IN_S / (float)SAMPLE_RATE / 2.0f;
}
static inline int inchesToSamples(float in) {
  return (int)(in * 2.0f * (float)SAMPLE_RATE / SOUND_IN_S);
}

// ---------------------------------------------------------------------------
//  Chirp
// ---------------------------------------------------------------------------
static void sonarBuildChirp() {
  const float f0 = s_stealth ? CHIRP_F0_STEALTH : CHIRP_F0_WIDE;
  const float f1 = s_stealth ? CHIRP_F1_STEALTH : CHIRP_F1_WIDE;

  double phase = 0.0, mean = 0.0;
  for (int i = 0; i < CHIRP_LEN; i++) {
    const double t = (double)i / (double)(CHIRP_LEN - 1);
    const double f = f0 + (f1 - f0) * t;
    phase += 2.0 * M_PI * f / (double)SAMPLE_RATE;
    const double w = 0.5 - 0.5 * cos(2.0 * M_PI * i / (CHIRP_LEN - 1));  // Hann
    const double v = w * sin(phase);
    s_chirpRef[i] = (float)v;
    s_chirpPcm[i] = (int16_t)lrint(CHIRP_AMP * v * 32767.0);
    mean += v;
  }
  // Zero-mean the reference so the microphone's DC offset - a steady +60 LSB on
  // this board - cannot correlate with anything and invent a peak.
  mean /= CHIRP_LEN;
  for (int i = 0; i < CHIRP_LEN; i++) s_chirpRef[i] -= (float)mean;
}

static void sonarSetStealth(bool on) {
  if (on == s_stealth) return;
  s_stealth = on;
  sonarBuildChirp();
  // The reference was learned with the other chirp's ringing in it.
  s_haveRef = false;
  memset(s_ref, 0, sizeof(s_ref));
}
static bool sonarStealth() { return s_stealth; }

// ---------------------------------------------------------------------------
//  I2S, full duplex - one i2s_new_channel() call, both handles. See config.h.
// ---------------------------------------------------------------------------
static bool sonarAudioInit() {
  pinMode(AMP_EN_PIN, OUTPUT);
  digitalWrite(AMP_EN_PIN, !AMP_EN_ACTIVE);  // off until I2S is clocking

  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num  = 4;
  chanCfg.dma_frame_num = BLOCK;
  chanCfg.auto_clear    = true;

  if (i2s_new_channel(&chanCfg, &s_tx, &s_rx) != ESP_OK) return false;

  i2s_std_config_t cfg = {
      .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                      I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .mclk = (gpio_num_t)I2S_MCLK,
          .bclk = (gpio_num_t)I2S_BCLK,
          .ws   = (gpio_num_t)I2S_LRCLK,
          .dout = (gpio_num_t)I2S_DOUT,
          .din  = (gpio_num_t)I2S_DIN,
          .invert_flags = {false, false, false},
      },
  };
  if (i2s_channel_init_std_mode(s_tx, &cfg) != ESP_OK) return false;
  cfg.slot_cfg.slot_mask = ES8311_RX_SLOT;
  if (i2s_channel_init_std_mode(s_rx, &cfg) != ESP_OK) return false;
  if (i2s_channel_enable(s_tx) != ESP_OK) return false;
  if (i2s_channel_enable(s_rx) != ESP_OK) return false;

  // The codec needs MCLK present before its registers mean anything.
  delay(50);
  const bool ok = es8311Init();
  delay(10);
  digitalWrite(AMP_EN_PIN, AMP_EN_ACTIVE);

  sonarBuildChirp();
  memset(s_avg, 0, sizeof(s_avg));
  memset(s_prev, 0, sizeof(s_prev));
  memset(s_ref, 0, sizeof(s_ref));
  return ok;
}

// RX DMA fills whether or not anyone reads it, so anything already in there
// predates this measurement.
static void sonarRxFlush() {
  static int16_t junk[BLOCK];
  size_t got;
  for (int i = 0; i < 8; i++) {
    if (i2s_channel_read(s_rx, junk, sizeof(junk), &got, 0) != ESP_OK) break;
    if (got == 0) break;
  }
}

// ---------------------------------------------------------------------------
//  One ping
// ---------------------------------------------------------------------------
static bool sonarPing() {
  memset(s_txFrame, 0, sizeof(s_txFrame));
  memcpy(s_txFrame, s_chirpPcm, sizeof(s_chirpPcm));

  sonarRxFlush();

  // Stepped in lockstep one block at a time, so TX and RX hold a fixed offset
  // through the frame. The value of that offset never has to be known.
  //
  // The hook runs every fourth block - every 21 ms - so touch gets polled
  // DURING the capture. Polling only between pings leaves an 85 ms blind window
  // that a normal 100 ms tap can fall entirely inside, which reads to the user
  // as buttons that work only if you press and hold. The I2C read costs about a
  // millisecond and the I2S DMA rings carry on underneath it.
  const size_t want = BLOCK * sizeof(int16_t);
  size_t done;
  for (int i = 0; i < CAP_LEN; i += BLOCK) {
    if (i2s_channel_write(s_tx, &s_txFrame[i], want, &done,
                          I2S_BLOCK_TIMEOUT_MS) != ESP_OK || done != want) {
      s_i2sStalls++;
      return false;  // a frame with a hole in it correlates to nonsense
    }
    if (i2s_channel_read(s_rx, &s_rxFrame[i], want, &done,
                         I2S_BLOCK_TIMEOUT_MS) != ESP_OK || done != want) {
      s_i2sStalls++;
      return false;
    }
    if (s_pingHook && ((i / BLOCK) & 3) == 3) s_pingHook();
  }

  // Matched filter. Plain O(N*M): 3840 lags x 256 taps is about 1M
  // multiply-accumulates, which the S3 does in well under the 85 ms the capture
  // itself takes - so an FFT would shorten nothing that matters.
  const int lim = CAP_LEN - CHIRP_LEN;
  float best = 0.0f;
  int   bestI = -1;
  for (int lag = 0; lag < lim; lag++) {
    float acc = 0.0f;
    const int16_t *p = &s_rxFrame[lag];
    for (int k = 0; k < CHIRP_LEN; k++) acc += s_chirpRef[k] * (float)p[k];
    const float mag = fabsf(acc);
    s_corr[lag] = mag;
    if (mag > best) { best = mag; bestI = lag; }
  }
  if (bestI < 0 || best <= 0.0f) return false;

  s_directIdx = bestI;
  s_directMag = best;

  // Everything from here is expressed as a fraction of the direct path, which
  // is what makes readings comparable across gain and volume changes.
  const float inv = 1.0f / best;
  for (int i = 0; i < PROF_LEN; i++) {
    const int j = bestI + i;
    s_prof[i] = (j < lim) ? s_corr[j] * inv : 0.0f;
  }

  // --- how much did the scene change? ------------------------------------
  //
  // Normalised L1 between CONSECUTIVE RAW PINGS, outside the blind zone.
  //
  // Deliberately not against the running average, which was the first attempt
  // and oscillates: the average's own noise depends on how deep it currently
  // is, so the instant a disturbance collapses the depth to 1 the metric is
  // being measured against a single noisy ping, reads high, and re-triggers.
  // The depth could never climb back out. Ping-against-ping has the same
  // statistics at every depth, so the learned baseline stays meaningful.
  //
  // Raw, before REF subtraction - otherwise a learned reference would mask the
  // very changes this exists to catch.
  const int cstart = inchesToSamples(BLIND_IN);
  float num = 0.0f, den = 0.0f;
  for (int i = cstart; i < PROF_LEN; i++) {
    num += fabsf(s_prof[i] - s_prev[i]);
    den += s_prev[i];
  }
  s_change = (den > 1e-6f) ? num / den : 0.0f;
  memcpy(s_prev, s_prof, sizeof(s_prev));

  // --- did the BOARD move, as opposed to the room? ------------------------
  //
  // The deviation has to PERSIST. A single ping over the threshold is not a
  // moved board - it is a hand near the speaker, which is exactly what happens
  // every time someone reaches for the touchscreen. Bench testing had REF going
  // stale the instant it was pressed, every time, because pressing REF puts a
  // hand next to the board by definition.
  //
  // A genuine move settles the direct path at a NEW level and stays there; a
  // hand passing by is a transient. Requiring a run of pings tells them apart
  // without needing to know anything about touch.
  if (s_directAvg <= 0.0f) s_directAvg = s_directMag;
  const float dRel = fabsf(s_directMag - s_directAvg) / s_directAvg;
  if (dRel > DIRECT_MOVE_FRAC) s_moveStreak++;
  else s_moveStreak = 0;
  s_handled = (s_pingCount > SETTLE_SEED) && (s_moveStreak >= DIRECT_MOVE_PINGS);
  s_directAvg += 0.05f * (s_directMag - s_directAvg);

  // A reference learned at the old position nulls real targets and invents
  // phantom ones. It is not cleared automatically - that would silently discard
  // something the user deliberately set - but it is flagged hard.
  if (s_handled && s_haveRef) s_refStale = true;

  // --- settle ------------------------------------------------------------
  const int target = kAvgCounts[s_avgIdx];
  const float trigger = s_changeBase * CHANGE_TRIGGER_K + CHANGE_FLOOR;
  s_disturbed = (s_pingCount > SETTLE_SEED) && (s_change > trigger);

  if (s_disturbed) {
    s_neff = 1;      // dump the history; the old room is no longer evidence
    s_lockIdx = -1;  // and the old headline target may not exist any more
  } else {
    if (s_neff < target) s_neff++;
    // Asymmetric: follow the floor down quickly, concede an increase slowly.
    // See config.h for the log that made this necessary.
    const float r = (s_change < s_changeBase) ? CHANGE_BASE_DOWN : CHANGE_BASE_UP;
    s_changeBase += r * (s_change - s_changeBase);
  }
  if (s_pingCount <= SETTLE_SEED) s_changeBase += 0.25f * (s_change - s_changeBase);
  if (s_neff > target) s_neff = target;

  // EMA at the depth currently in use.
  if (s_pingCount == 0 || s_neff <= 1) {
    memcpy(s_avg, s_prof, sizeof(s_avg));
  } else {
    const float a = 1.0f / (float)s_neff;
    for (int i = 0; i < PROF_LEN; i++) s_avg[i] += a * (s_prof[i] - s_avg[i]);
  }
  s_pingCount++;
  return true;
}

// What the display should draw: the average, less the learned reference.
static inline float sonarValue(int i) {
  const float v = s_avg[i] - (s_haveRef ? s_ref[i] : 0.0f);
  return v > 0.0f ? v : 0.0f;
}

// Learn the room as it stands. Whatever is static right now - including the
// speaker's own ringing, which is why this reclaims the blind zone - drops to
// zero, and the display becomes a change detector. Press it again to clear.
static void sonarToggleRef() {
  if (s_haveRef) {
    s_haveRef = false;
    memset(s_ref, 0, sizeof(s_ref));
  } else {
    memcpy(s_ref, s_avg, sizeof(s_ref));
    s_haveRef = true;
  }
  s_refStale = false;
}
static bool sonarHaveRef() { return s_haveRef; }
static inline float sonarBlindIn() { return s_haveRef ? BLIND_IN_REF : BLIND_IN; }

static void sonarCycleAvg() {
  s_avgIdx = (s_avgIdx + 1) % AVG_STEPS;
  s_neff = 1;  // re-converge at the new depth instead of dragging the old
}
static uint8_t sonarAvgCount() { return kAvgCounts[s_avgIdx]; }

static int   sonarNeff()      { return s_neff; }
static uint32_t sonarStalls() { return s_i2sStalls; }
static bool  sonarDisturbed() { return s_disturbed; }
static bool  sonarHandled()   { return s_handled; }
static bool  sonarRefStale()  { return s_refStale; }
static float sonarChange()    { return s_change; }

// Settled means the average is at the depth the user asked for AND the scene
// has stopped moving under it.
static bool sonarSettled() {
  return !s_disturbed && s_neff >= kAvgCounts[s_avgIdx];
}

// ---------------------------------------------------------------------------
//  Peaks
// ---------------------------------------------------------------------------
struct Peak { int idx; float mag; };

// Strongest local maxima beyond the blind zone, nearest-first is not what we
// want here - strongest-first is, because that is what gets labelled.
static int sonarFindPeaks(Peak *out, int maxOut, float floorFrac) {
  const int start = inchesToSamples(sonarBlindIn());
  const int guard = CHIRP_LEN / 4;  // one peak per resolution cell, not twelve
  int n = 0;

  for (int i = start + 1; i < PROF_LEN - 1; i++) {
    const float v = sonarValue(i);
    if (v < floorFrac) continue;
    if (v < sonarValue(i - 1) || v < sonarValue(i + 1)) continue;

    bool local = true;
    for (int j = i - guard; j <= i + guard; j++) {
      if (j < 0 || j >= PROF_LEN || j == i) continue;
      if (sonarValue(j) > v) { local = false; break; }
    }
    if (!local) continue;

    if (n < maxOut) {
      out[n].idx = i;
      out[n].mag = v;
      n++;
    } else if (v > out[maxOut - 1].mag) {
      out[maxOut - 1].idx = i;
      out[maxOut - 1].mag = v;
    } else {
      i += guard;
      continue;
    }
    // Keep the array sorted strongest-first: that is the order they get
    // labelled in, and the top one is the headline reading.
    for (int k = (n < maxOut ? n : maxOut) - 1;
         k > 0 && out[k].mag > out[k - 1].mag; k--) {
      Peak t = out[k]; out[k] = out[k - 1]; out[k - 1] = t;
    }
    i += guard;  // one peak per resolution cell
  }
  return n;
}

// Which peak is THE reading? Not simply the strongest: an ordinary room holds
// several reflectors of similar strength, and taking the argmax every ping makes
// the headline hop between them while every one of them is individually stable
// to a tenth of an inch. So the previous headline gets first refusal - any peak
// still within the gate keeps the lock - and only a target that has genuinely
// gone away releases it.
//
// The lock is dropped outright when the scene changes, because after a
// disturbance the old target may not exist any more.
static int sonarHeadline(const Peak *pk, int n) {
  if (n <= 0) { s_lockIdx = -1; return -1; }

  if (s_lockIdx >= 0) {
    const int gate = inchesToSamples(LOCK_GATE_IN);
    int bestK = -1, bestD = gate + 1;
    for (int k = 0; k < n; k++) {
      const int d = abs(pk[k].idx - s_lockIdx);
      if (d <= gate && d < bestD) { bestD = d; bestK = k; }
    }
    if (bestK >= 0) {
      s_lockIdx = pk[bestK].idx;
      s_lockMiss = 0;
      return bestK;
    }
    // Give it a few pings before letting go: a return can drop below the peak
    // threshold for one ping without the surface having gone anywhere.
    if (++s_lockMiss < LOCK_MISS_MAX) return -1;
  }

  s_lockIdx = pk[0].idx;  // peaks arrive strongest-first
  s_lockMiss = 0;
  return 0;
}

// Sub-sample refined distance for a peak index.
static float sonarPeakInches(int i) {
  if (i <= 0 || i >= PROF_LEN - 1) return samplesToInches(i);
  const float ym = sonarValue(i - 1), y0 = sonarValue(i), yp = sonarValue(i + 1);
  const float denom = ym - 2.0f * y0 + yp;
  const float shift = (denom != 0.0f) ? 0.5f * (ym - yp) / denom : 0.0f;
  return samplesToInches(i) + shift * SOUND_IN_S / (float)SAMPLE_RATE / 2.0f;
}

// Feet and inches. The engine works entirely in samples; this is the only
// place that pretends distance has units.
static const char *fmtDist(float inches) {
  static char buf[24];
  // Round to tenths BEFORE splitting into feet. Splitting first lets 47.98 in
  // become 3 ft and 11.98 in, which %.1f then prints as 3' 12.0".
  int tenths = (int)lrintf(inches * 10.0f);
  if (tenths < 0) tenths = 0;
  const int ft = tenths / 120;
  const float in = (tenths - ft * 120) / 10.0f;
  if (ft > 0) snprintf(buf, sizeof(buf), "%d' %.1f\"", ft, in);
  else        snprintf(buf, sizeof(buf), "%.1f\"", in);
  return buf;
}
