// ============================================================================
//  MicTest - step 0 for CYD Sonar, on the Hosyond ESP32-S3 4.0" module.
//
//  Nothing on this board has ever read a microphone sample. The talk radio
//  project assigned IO6 to the mic from a vendor pinout and then deliberately
//  never built the RX channel, because an earlier attempt at adding RX silenced
//  the speaker. So before any sonar code gets written, three things have to be
//  true, and this sketch proves them one at a time:
//
//    1. The mic exists and produces samples.        -> 'm' level meter
//    2. Enabling RX does not break TX.              -> 'q' tone, with RX live
//    3. TX and RX are usable in the same frame.     -> 'c' chirp + matched filter
//
//  (3) is the actual sonar experiment. It emits a chirp, captures the mic
//  through the same frame, and correlates. The first peak is the speaker
//  leaking straight into the mic an inch or two away; anything after it is
//  the room. If you see a second peak move when you move a wall or a book in
//  front of the board, the whole project is proven in one command.
//
//  No display, no Wi-Fi, no libraries beyond Wire and the IDF I2S driver. This
//  is a bring-up tool - keep it boring so a failure means the hardware.
//
//  Build:
//    arduino-cli compile --upload -p /dev/cu.usbmodemXXXX \
//      --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,CDCOnBoot=cdc MicTest
//
//  Copyright (c) 2026 Rich Sadowsky. MIT licensed - see LICENSE.
//  Written with Claude Code (Claude Opus 5).
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s_std.h>
#include <math.h>

// ---------------------------------------------------------------------------
//  Hardware. Everything here is carried over from cyd-talk-radio-esp32s3-4inch,
//  where it was confirmed on this same board - except I2S_DIN, which is the
//  whole point of this sketch.
// ---------------------------------------------------------------------------
#define I2C_SDA 16
#define I2C_SCL 15
#define I2C_HZ  400000

#define I2S_BCLK   5
#define I2S_LRCLK  7
#define I2S_DOUT   8  // -> ES8311 -> PA -> speaker header   (confirmed)
#define I2S_DIN    6  // <- ES8311 <- on-board mic           (UNVERIFIED)
#define I2S_MCLK   4  // required: the codec derives its clocks from this

#define AMP_EN_PIN     1
#define AMP_EN_ACTIVE  LOW  // deduced from the amp sweep, not the datasheet

#define ES8311_ADDR 0x18

// Set to 0 to bisect: if the tone plays with FULL_DUPLEX 0 and not with 1, the
// problem is RX disturbing the shared bus, not the codec or the amp.
#define FULL_DUPLEX 1

// The ES8311 puts ADC data in one slot of the frame. Left is the usual choice
// and matches how the DAC is fed. If the meter reads dead silence but the codec
// is otherwise healthy, try I2S_STD_SLOT_RIGHT before suspecting the mic.
#define RX_SLOT I2S_STD_SLOT_LEFT

// ---------------------------------------------------------------------------
//  Audio parameters
//
//  48 kHz rather than the radio's 32 kHz. Sonar range resolution is set by the
//  sample period - one sample is 0.28 in of round trip, so 0.14 in of distance
//  - and by chirp bandwidth, and 48 kHz buys headroom on both. The codec's divider set is written for MCLK = 256*fs, which the ESP32
//  I2S driver generates at whatever rate it is told, so the rate is free to
//  change. 'r' toggles back to 32 kHz to check that assumption.
// ---------------------------------------------------------------------------
#define RATE_DEFAULT 48000
#define RATE_ALT     32000

#define BLOCK 256  // samples per write/read iteration

// Chirp. 4 kHz -> 18 kHz over 5.3 ms at 48 kHz. Wide band because the
// correlation peak width is roughly 1/bandwidth: 14 kHz gives a peak about
// 70 us wide, or just under half an inch of range resolution. It is audible -
// a short tick.
// 'n' narrows it to 14-19 kHz, which most adults cannot hear, at the cost of
// a peak about four times broader.
#define CHIRP_LEN 256
#define CHIRP_F0_WIDE   4000.0f
#define CHIRP_F1_WIDE  18000.0f
#define CHIRP_F0_NARROW 14000.0f
#define CHIRP_F1_NARROW 19000.0f
#define CHIRP_AMP 0.70f

// Capture window. Must comfortably exceed DMA latency (both directions) plus
// the longest echo of interest. 4096 samples is 85 ms at 48 kHz.
#define CAP_LEN 4096

#define TONE_HZ  440.0f
#define TONE_AMP 0.35f

// Speed of sound at 68 F, in inches/s. It moves about 1.1 in/s per degree F,
// so a room 20 F off nominal shifts a 6 ft reading by about a quarter inch -
// below this rig's noise, but it is the first thing to correct if the readings
// ever need to be trusted to better than that.
#define SOUND_IN_S 13511.8f

// Feet and inches. The ranging works entirely in samples; this is the only
// place in the sketch that pretends distance has units at all.
static const char *fmtDist(float inches) {
  static char buf[24];
  const int ft = (int)(inches / 12.0f);
  const float in = inches - ft * 12.0f;
  if (ft > 0) snprintf(buf, sizeof(buf), "%d' %4.1f\"", ft, in);
  else        snprintf(buf, sizeof(buf), "%5.1f\"", in);
  return buf;
}

// ---------------------------------------------------------------------------
//  ES8311
// ---------------------------------------------------------------------------
static bool es8311Write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ES8311_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// One register per transaction: the ES8311 does not auto-increment on reads.
static uint8_t es8311Read(uint8_t reg) {
  Wire.beginTransmission(ES8311_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom((int)ES8311_ADDR, 1) != 1) return 0xFF;
  return Wire.read();
}

static bool es8311Present() { return es8311Read(0xFD) == 0x83; }

static bool es8311Init() {
  if (!es8311Present()) return false;

  es8311Write(0x00, 0x1F);  // reset
  delay(20);
  es8311Write(0x00, 0x00);
  es8311Write(0x00, 0x80);  // slave mode, power up

  // Clock manager: every internal clock on, standard MCLK = 256*fs dividers.
  es8311Write(0x01, 0x3F);
  es8311Write(0x02, 0x00);
  es8311Write(0x03, 0x10);
  es8311Write(0x04, 0x10);
  es8311Write(0x05, 0x00);
  es8311Write(0x06, 0x03);
  es8311Write(0x07, 0x00);
  es8311Write(0x08, 0xFF);

  es8311Write(0x09, 0x0C);  // SDP in  - I2S, 16 bit (to DAC)
  es8311Write(0x0A, 0x0C);  // SDP out - I2S, 16 bit (from ADC)

  es8311Write(0x0B, 0x00);
  es8311Write(0x0C, 0x00);
  es8311Write(0x0D, 0x01);
  es8311Write(0x0E, 0x02);  // analog PGA + ADC powered up
  es8311Write(0x0F, 0x44);
  es8311Write(0x10, 0x03);
  es8311Write(0x11, 0x7F);
  es8311Write(0x12, 0x00);  // DAC powered up
  es8311Write(0x13, 0x10);
  es8311Write(0x14, 0x1A);  // analog mic selected (bit 6 clear = not digital)
  es8311Write(0x15, 0x00);
  es8311Write(0x16, 0x24);  // ADC PGA - 'g' sweeps this
  es8311Write(0x17, 0xBF);  // ADC digital volume, 0xBF = 0 dB
  es8311Write(0x18, 0x00);  // ALC OFF. Sonar needs a fixed gain: an ALC would
                            // quietly rescale the echo against the direct path
                            // and make every amplitude meaningless.

  es8311Write(0x31, 0x00);  // DAC unmuted
  es8311Write(0x32, 0xBF);  // DAC volume. LOGARITHMIC: -95.5 + 0.5*v dB.
  es8311Write(0x37, 0x08);
  es8311Write(0x44, 0x00);  // ADC->DAC loopback off. 'l' toggles to 0x08.
  es8311Write(0x45, 0x00);
  return true;
}

// ---------------------------------------------------------------------------
//  I2S, full duplex
//
//  Both handles come from ONE i2s_new_channel() call. That is what makes the
//  peripheral full duplex: TX and RX then share BCLK and WS internally. Two
//  separate calls give two independent channels fighting over the same pins,
//  which is the likeliest explanation for the silenced speaker last time.
//
//  MCLK is named in both configs; the driver emits it once.
// ---------------------------------------------------------------------------
static i2s_chan_handle_t s_tx = nullptr;
static i2s_chan_handle_t s_rx = nullptr;
static uint32_t s_rate = RATE_DEFAULT;

static void i2sTeardown() {
  if (s_tx) { i2s_channel_disable(s_tx); i2s_del_channel(s_tx); s_tx = nullptr; }
  if (s_rx) { i2s_channel_disable(s_rx); i2s_del_channel(s_rx); s_rx = nullptr; }
}

static bool i2sInit(uint32_t rate) {
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num  = 4;
  chanCfg.dma_frame_num = BLOCK;
  chanCfg.auto_clear    = true;  // silence on underrun, not the stale buffer

#if FULL_DUPLEX
  esp_err_t e = i2s_new_channel(&chanCfg, &s_tx, &s_rx);
#else
  esp_err_t e = i2s_new_channel(&chanCfg, &s_tx, nullptr);
#endif
  if (e != ESP_OK) { Serial.printf("[i2s] new_channel %d\n", e); return false; }

  i2s_std_config_t cfg = {
      .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(rate),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                      I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .mclk = (gpio_num_t)I2S_MCLK,
          .bclk = (gpio_num_t)I2S_BCLK,
          .ws   = (gpio_num_t)I2S_LRCLK,
          .dout = (gpio_num_t)I2S_DOUT,
#if FULL_DUPLEX
          .din  = (gpio_num_t)I2S_DIN,
#else
          .din  = I2S_GPIO_UNUSED,
#endif
          .invert_flags = {false, false, false},
      },
  };

  if ((e = i2s_channel_init_std_mode(s_tx, &cfg)) != ESP_OK) {
    Serial.printf("[i2s] init tx %d\n", e); return false;
  }
#if FULL_DUPLEX
  cfg.slot_cfg.slot_mask = RX_SLOT;
  if ((e = i2s_channel_init_std_mode(s_rx, &cfg)) != ESP_OK) {
    Serial.printf("[i2s] init rx %d\n", e); return false;
  }
#endif

  if ((e = i2s_channel_enable(s_tx)) != ESP_OK) {
    Serial.printf("[i2s] enable tx %d\n", e); return false;
  }
#if FULL_DUPLEX
  if ((e = i2s_channel_enable(s_rx)) != ESP_OK) {
    Serial.printf("[i2s] enable rx %d\n", e); return false;
  }
#endif
  s_rate = rate;
  return true;
}

// RX DMA keeps filling whether or not anyone reads it. Anything sitting in it
// predates the measurement, so throw it away before every capture.
static void rxFlush() {
#if FULL_DUPLEX
  static int16_t junk[BLOCK];
  size_t got;
  for (int i = 0; i < 8; i++) {
    if (i2s_channel_read(s_rx, junk, sizeof(junk), &got, 0) != ESP_OK) break;
    if (got == 0) break;
  }
#endif
}

// ---------------------------------------------------------------------------
//  Signal generation
// ---------------------------------------------------------------------------
static float s_chirpRef[CHIRP_LEN];  // matched filter, float, zero mean
static int16_t s_chirpPcm[CHIRP_LEN];
static bool s_narrow = false;

static void buildChirp() {
  const float f0 = s_narrow ? CHIRP_F0_NARROW : CHIRP_F0_WIDE;
  const float f1 = s_narrow ? CHIRP_F1_NARROW : CHIRP_F1_WIDE;

  double phase = 0.0;
  double mean  = 0.0;
  for (int i = 0; i < CHIRP_LEN; i++) {
    const double t = (double)i / (double)(CHIRP_LEN - 1);
    const double f = f0 + (f1 - f0) * t;            // linear sweep
    phase += 2.0 * M_PI * f / (double)s_rate;
    const double w = 0.5 - 0.5 * cos(2.0 * M_PI * i / (CHIRP_LEN - 1));  // Hann
    const double v = w * sin(phase);
    s_chirpRef[i] = (float)v;
    s_chirpPcm[i] = (int16_t)lrint(CHIRP_AMP * v * 32767.0);
    mean += v;
  }
  // Remove DC from the reference so a mic offset cannot masquerade as a peak.
  mean /= CHIRP_LEN;
  for (int i = 0; i < CHIRP_LEN; i++) s_chirpRef[i] -= (float)mean;
}

// ---------------------------------------------------------------------------
//  Test 1 - level meter. Writes silence so the speaker is not part of the
//  measurement, and prints what the mic hears.
// ---------------------------------------------------------------------------
static bool s_tone = false;
static float s_tonePhase = 0.0f;

static void fillTx(int16_t *dst, int n) {
  if (!s_tone) { memset(dst, 0, n * sizeof(int16_t)); return; }
  const float step = 2.0f * (float)M_PI * TONE_HZ / (float)s_rate;
  for (int i = 0; i < n; i++) {
    dst[i] = (int16_t)(TONE_AMP * 32767.0f * sinf(s_tonePhase));
    s_tonePhase += step;
    if (s_tonePhase > 2.0f * (float)M_PI) s_tonePhase -= 2.0f * (float)M_PI;
  }
}

static void meterStep() {
  static int16_t tx[BLOCK], rx[BLOCK];
  size_t done;

  fillTx(tx, BLOCK);
  i2s_channel_write(s_tx, tx, sizeof(tx), &done, 200);

#if FULL_DUPLEX
  if (i2s_channel_read(s_rx, rx, sizeof(rx), &done, 200) != ESP_OK) return;
  const int n = done / sizeof(int16_t);
  if (n == 0) return;

  double sum = 0, sq = 0;
  int16_t peak = 0;
  for (int i = 0; i < n; i++) {
    sum += rx[i];
    sq  += (double)rx[i] * rx[i];
    int16_t a = rx[i] < 0 ? -rx[i] : rx[i];
    if (a > peak) peak = a;
  }
  const double dc   = sum / n;
  const double rms  = sqrt(sq / n - dc * dc);           // AC only
  const double dbfs = rms > 0 ? 20.0 * log10(rms / 32768.0) : -120.0;

  static uint32_t last = 0;
  if (millis() - last < 100) return;
  last = millis();

  int bars = (int)((dbfs + 80.0) / 80.0 * 40.0);
  if (bars < 0) bars = 0;
  if (bars > 40) bars = 40;
  char meter[41];
  memset(meter, '-', 40);
  memset(meter, '#', bars);
  meter[40] = 0;

  Serial.printf("[%s] rms %6.0f  peak %5d  dc %+6.0f  %6.1f dBFS\n",
                meter, rms, peak, dc, dbfs);
#endif
}

// ---------------------------------------------------------------------------
//  Test 2 - chirp and matched filter. The sonar experiment.
//
//  TX and RX are stepped in lockstep, one BLOCK at a time, so the capture holds
//  a fixed and unknown offset relative to the chirp: DMA latency in both
//  directions, plus the codec's own group delay. That offset never has to be
//  measured. The direct path - the speaker leaking into the mic an inch or
//  two away, arriving in well under 0.1 ms - lands as the first and by
//  far the largest correlation peak, and every echo is timed from THAT, not
//  from when we asked for the chirp. The system calibrates itself on every
//  single ping, and any latency in the chain cancels exactly.
// ---------------------------------------------------------------------------
static int16_t s_txFrame[CAP_LEN];
static int16_t s_rxFrame[CAP_LEN];
static float   s_corr[CAP_LEN];

// Returns the index of the direct-path peak, or -1.
static int pingOnce(bool verbose) {
#if !FULL_DUPLEX
  Serial.println("needs FULL_DUPLEX 1");
  return -1;
#else
  memset(s_txFrame, 0, sizeof(s_txFrame));
  memcpy(s_txFrame, s_chirpPcm, sizeof(s_chirpPcm));

  rxFlush();

  size_t done;
  for (int i = 0; i < CAP_LEN; i += BLOCK) {
    i2s_channel_write(s_tx, &s_txFrame[i], BLOCK * sizeof(int16_t), &done, 500);
    i2s_channel_read (s_rx, &s_rxFrame[i], BLOCK * sizeof(int16_t), &done, 500);
  }

  // Matched filter. Plain O(N*M) correlation - 1M multiply-accumulates, a few
  // tens of ms. An FFT would be faster and is not needed to answer the
  // question this sketch exists to answer.
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
  if (bestI < 0 || best <= 0.0f) { Serial.println("no correlation at all"); return -1; }

  if (!verbose) return bestI;

  Serial.printf("\ndirect path at sample %d, magnitude %.0f\n", bestI, best);
  Serial.printf("  (%.2f ms of round-trip latency in the codec + DMA chain)\n",
                1000.0 * bestI / s_rate);

  // Peaks after the direct path. Suppress anything within half a chirp of an
  // already-reported peak, otherwise one echo reports as a dozen.
  Serial.println("  echoes:");
  const float floorMag = best * 0.02f;
  int shown = 0;
  for (int lag = bestI + CHIRP_LEN / 2; lag < lim - 1 && shown < 8; lag++) {
    if (s_corr[lag] < floorMag) continue;
    if (s_corr[lag] < s_corr[lag - 1] || s_corr[lag] < s_corr[lag + 1]) continue;
    bool local = true;
    for (int j = lag - CHIRP_LEN / 2; j < lag + CHIRP_LEN / 2 && j < lim; j++) {
      if (j >= 0 && s_corr[j] > s_corr[lag]) { local = false; break; }
    }
    if (!local) continue;
    const float in = (lag - bestI) * SOUND_IN_S / s_rate / 2.0f;
    Serial.printf("    +%4d samples  %9s  %5.1f%% of direct\n",
                  lag - bestI, fmtDist(in), 100.0f * s_corr[lag] / best);
    shown++;
    lag += CHIRP_LEN / 2;
  }
  if (shown == 0) Serial.println("    none above 2% of the direct path");

  // Coarse profile, 0 to 10 ft.
  const int span = (int)(2.0f * 120.0f * s_rate / SOUND_IN_S);  // samples for 120 in
  Serial.println("  profile, 0 -> 10 ft:");
  for (int col = 0; col < 60; col++) {
    const int a = bestI + (int)((float)col / 60.0f * span);
    const int b = bestI + (int)((float)(col + 1) / 60.0f * span);
    float m = 0;
    for (int j = a; j < b && j < lim; j++) if (s_corr[j] > m) m = s_corr[j];
    int h = (int)(m / best * 20.0f);
    if (h > 20) h = 20;
    Serial.printf("  %9s |%.*s\n", fmtDist(col * 120.0f / 60.0f), h,
                  "####################");
  }
  return bestI;
#endif
}

// ---------------------------------------------------------------------------
//  Command loop
// ---------------------------------------------------------------------------
enum Mode : uint8_t { MODE_IDLE, MODE_METER, MODE_TRACK };
static Mode s_mode = MODE_METER;

static void help() {
  Serial.println();
  Serial.println("  m  level meter (mic only)        q  toggle 440 Hz tone");
  Serial.println("  c  single chirp + full report    t  continuous range track");
  Serial.println("  n  toggle narrow/stealth chirp   g  cycle ADC PGA gain 0-7");
  Serial.println("  l  toggle codec ADC->DAC loopback (WILL howl - hold it away)");
  Serial.println("  r  toggle 48 kHz / 32 kHz        d  dump codec registers");
  Serial.println("  ?  this help");
  Serial.println();
}

static void handle(char c) {
  switch (c) {
    case 'm':
      s_mode = (s_mode == MODE_METER) ? MODE_IDLE : MODE_METER;
      Serial.printf("meter %s\n", s_mode == MODE_METER ? "on" : "off");
      break;

    case 'q':
      s_tone = !s_tone;
      Serial.printf("tone %s - if this is silent with FULL_DUPLEX 1 and audible "
                    "with 0, RX is the problem\n", s_tone ? "ON" : "off");
      break;

    case 'c':
      s_mode = MODE_IDLE;
      pingOnce(true);
      break;

    case 't':
      s_mode = (s_mode == MODE_TRACK) ? MODE_IDLE : MODE_TRACK;
      Serial.printf("tracking %s\n", s_mode == MODE_TRACK ? "on" : "off");
      break;

    case 'n':
      s_narrow = !s_narrow;
      buildChirp();
      Serial.printf("chirp %s\n", s_narrow ? "14-19 kHz (near-inaudible, blunter peak)"
                                           : "4-18 kHz (audible tick, sharpest peak)");
      break;

    case 'g': {
      static uint8_t gain = 0;
      gain = (gain + 1) & 0x07;
      es8311Write(0x16, gain);
      Serial.printf("ADC PGA reg 0x16 = %u\n", gain);
      break;
    }

    case 'l': {
      static bool lb = false;
      lb = !lb;
      es8311Write(0x44, lb ? 0x08 : 0x00);
      Serial.printf("codec loopback %s (reg 0x44 = 0x%02X)\n",
                    lb ? "ON - speak at it" : "off", lb ? 0x08 : 0x00);
      break;
    }

    case 'r': {
      const uint32_t next = (s_rate == RATE_DEFAULT) ? RATE_ALT : RATE_DEFAULT;
      i2sTeardown();
      if (!i2sInit(next)) { Serial.println("reinit FAILED"); break; }
      delay(50);
      es8311Init();
      buildChirp();
      Serial.printf("rate now %u Hz\n", (unsigned)s_rate);
      break;
    }

    case 'd': {
      Serial.println("codec registers:");
      const uint8_t regs[] = {0x00,0x01,0x09,0x0A,0x0E,0x12,0x14,0x16,0x17,
                              0x18,0x31,0x32,0x44,0xFD};
      for (uint8_t r : regs) Serial.printf("  0x%02X = 0x%02X\n", r, es8311Read(r));
      break;
    }

    case '?': help(); break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n=== CYD Sonar - MicTest ===");
#if FULL_DUPLEX
  Serial.println("full duplex: TX + RX");
#else
  Serial.println("TX ONLY (FULL_DUPLEX 0) - bisect build");
#endif

  Wire.begin(I2C_SDA, I2C_SCL, I2C_HZ);

  pinMode(AMP_EN_PIN, OUTPUT);
  digitalWrite(AMP_EN_PIN, !AMP_EN_ACTIVE);  // held off until I2S is clocking

  if (!i2sInit(RATE_DEFAULT)) { Serial.println("I2S init FAILED"); return; }

  // The codec needs MCLK present before its registers mean anything, so it is
  // configured after I2S is already running.
  delay(50);
  Serial.printf("[es8311] 0xFD = 0x%02X (expect 0x83)\n", es8311Read(0xFD));
  Serial.printf("[es8311] init %s\n", es8311Init() ? "ok" : "NOT FOUND");
  delay(10);
  digitalWrite(AMP_EN_PIN, AMP_EN_ACTIVE);

  buildChirp();
  Serial.printf("rate %u Hz, chirp %d samples, capture %d samples\n",
                (unsigned)s_rate, CHIRP_LEN, CAP_LEN);
  help();
  Serial.println("meter running - tap the board or talk at it");
}

void loop() {
  while (Serial.available()) handle((char)Serial.read());

  if (s_mode == MODE_METER) {
    meterStep();
  } else if (s_mode == MODE_TRACK) {
    const int d = pingOnce(false);
    if (d >= 0) {
      // Strongest echo outside the direct path's own shadow.
      const int lim = CAP_LEN - CHIRP_LEN;
      float best = 0; int bi = -1;
      for (int j = d + CHIRP_LEN; j < lim; j++) {
        if (s_corr[j] > best) { best = s_corr[j]; bi = j; }
      }
      if (bi > 0) {
        Serial.printf("range %9s   strength %5.1f%%\n",
                      fmtDist((bi - d) * SOUND_IN_S / s_rate / 2.0f),
                      100.0f * best / s_corr[d]);
      }
    }
    delay(300);
  } else {
    // Idle still has to feed TX or the DMA underruns and the codec hears a
    // discontinuity every time we come back.
    static int16_t quiet[BLOCK] = {0};
    size_t done;
    i2s_channel_write(s_tx, quiet, sizeof(quiet), &done, 100);
    rxFlush();
  }
}
