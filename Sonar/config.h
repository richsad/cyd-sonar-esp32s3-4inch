#pragma once

// ============================================================================
//  Board / project configuration for CYD Sonar on the Hosyond ESP32-S3 4.0"
//  touchscreen module.
//
//  The panel, touch and codec sections are carried over from
//  cyd-talk-radio-esp32s3-4inch, where every value was confirmed against this
//  same hardware. The audio and ranging sections are new.
//
//  Copyright (c) 2026 Rich Sadowsky. MIT licensed - see LICENSE.
//  Written with Claude Code (Claude Opus 5).
// ============================================================================

// ---------------------------------------------------------------------------
//  LCD - ST7796 over 4-line SPI, 320x480 IPS. NOT the 240x320 ILI9341 the
//  listing claims; see the talk radio README for how that was caught.
// ---------------------------------------------------------------------------
#define LCD_W 320
#define LCD_H 480

#define LCD_CS    10
#define LCD_DC    46
#define LCD_SCK   12
#define LCD_MOSI  11
#define LCD_MISO  13
#define LCD_RST   -1
#define LCD_BL    45

#define LCD_ROTATION 1
#define SCREEN_W 480
#define SCREEN_H 320
#define LCD_IPS true
#define LCD_SPI_HZ 40000000

// ---------------------------------------------------------------------------
//  Touch - FT6336G capacitive. Not the GT911 most CYD boards use, so copied
//  CYD touch code will not work here.
// ---------------------------------------------------------------------------
#define TOUCH_SDA 16
#define TOUCH_SCL 15
#define TOUCH_RST 18
#define TOUCH_INT 17
// 100 kHz, not the 400 kHz the talk radio build used.
//
// The FT6336 wedges: it answers I2C, reads back the right chip ID, and reports
// zero touches while the sonar watches a finger five inches from the panel. A
// controller reset does not clear it (and GPIO18 is confirmed to be a real
// reset line), which argues the part is not stuck so much as not detecting -
// and a marginal bus is one of the few things that fits.
//
// This bus is shared with the ES8311 at 0x18, on a board whose routing nobody
// here has seen. Standard mode is the cheapest experiment available and costs
// nothing that matters: the touch poll is five bytes, four times longer at
// 100 kHz is still well under a millisecond, and the codec is only written
// during bring-up.
//
// If the wedge stops recurring, that is the answer. If it does not, the next
// suspect is the backlight PWM sitting next to a capacitive panel.
#define TOUCH_I2C_HZ   100000
#define TOUCH_I2C_ADDR 0x38
#define TOUCH_SWAP_XY   1
#define TOUCH_INVERT_Y  1

// ---------------------------------------------------------------------------
//  Audio - ES8311 codec, full duplex on one I2S channel pair
//
//  Both I2S handles MUST come from one i2s_new_channel() call. Two separate
//  calls give two channels fighting over the same BCLK/WS pins, which is what
//  silenced the speaker during the talk radio bring-up and got RX abandoned
//  there. Proven on hardware by MicTest: TX is untouched by full duplex.
// ---------------------------------------------------------------------------
#define I2S_BCLK   5
#define I2S_LRCLK  7
#define I2S_DOUT   8
#define I2S_DIN    6
#define I2S_MCLK   4

#define AMP_EN_PIN     1
#define AMP_EN_ACTIVE  LOW

#define ES8311_RX_SLOT I2S_STD_SLOT_LEFT
#define AMP_USE_MCLK 1

// DAC volume. LOGARITHMIC: gain_dB = -95.5 + 0.5 * value, so 0xBF is unity.
// Loudness is set by CHIRP_AMP, which is linear and predictable.
#define CODEC_VOLUME 0xBF

// 48 kHz: one sample is 0.28 in of round trip, so 0.14 in of distance.
#define SAMPLE_RATE 48000
#define BLOCK 256

// ---------------------------------------------------------------------------
//  Chirp
//
//  Range resolution is roughly c/(2*bandwidth), so the 14 kHz wide sweep
//  resolves just under half an inch and the 5 kHz stealth sweep about 1.4 in.
//  Wide is the default: it is a soft tick rather than a nuisance, and the
//  speaker rolls off badly above 14 kHz so stealth also costs echo strength.
// ---------------------------------------------------------------------------
#define CHIRP_LEN 256
#define CHIRP_F0_WIDE    4000.0f
#define CHIRP_F1_WIDE   18000.0f
#define CHIRP_F0_STEALTH 14000.0f
#define CHIRP_F1_STEALTH 19000.0f
#define CHIRP_AMP 0.70f

// Capture window. Must exceed the codec+DMA latency (measured at ~1060
// samples, 22 ms) plus the full display span plus a chirp length. 4096 leaves
// generous margin for that latency drifting between runs, which it does.
#define CAP_LEN 4096

// Per-block I2S timeout. A block is 256 samples, 5.3 ms of audio, and the DMA
// rings hold 1024 samples - so any transfer that has not completed in 100 ms is
// not slow, it is stuck.
//
// This was 500 ms, applied to both the write and the read of all 16 blocks, so
// a stalled capture could spend EIGHT SECONDS inside one ping and produce a
// garbage frame at the end of it. Reported from the bench as the chirp rate
// dropping to one every few seconds with the buttons unresponsive, which is
// exactly what a multi-second blocking call in the middle of loop() looks like.
// Bounded and logged now, and a short transfer abandons the ping rather than
// correlating a frame with holes in it.
#define I2S_BLOCK_TIMEOUT_MS 100

// ---------------------------------------------------------------------------
//  Ranging
//
//  Speed of sound at 68 F, in inches/sec. About 1.1 in/s per degree F: a room
//  20 F off nominal shifts a 6 ft reading by a quarter inch, which is below
//  this rig's noise but is the first correction to make if it ever matters.
// ---------------------------------------------------------------------------
#define SOUND_IN_S 13511.8f

// Displayed span, in inches. 1024 samples at 48 kHz is 144.4 in, so 12 ft is
// almost exactly the natural window.
#define RANGE_MAX_IN 144.0f
#define PROF_LEN 1024

// Everything closer than this is the speaker ringing after the chirp, not the
// room. Measured, not guessed: the ring runs 100% of the direct path at the
// peak, is still 8% at 9 in, and only falls below 2% past 18 in. At 8 in the
// auto-gain locked onto the tail and pinned full scale at 12% while real
// echoes sat at 3.7%, squashing the trace to a third of its height.
//
// REF subtraction is what actually reclaims this range; this constant only
// keeps the ring out of the auto-gain and the peak list.
#define BLIND_IN 15.0f

// ...but only while the ring is actually there. With a reference loaded the
// ring is subtracted out, so the near field stops being blind - and searching
// for peaks from 15 in makes it impossible to SEE that, which is the single
// most convincing thing REF does. The matched filter compresses the chirp to a
// few samples regardless of its length, so nothing but the ring was ever in the
// way.
#define BLIND_IN_REF 4.0f

// Averaging. These are EMA time constants expressed as a ping count, applied to
// the correlation MAGNITUDE - incoherent averaging.
//
// Coherent averaging of the raw captures would give sqrt(N) in amplitude rather
// than in variance, which is better, but it needs sub-sample alignment: at
// 18 kHz one sample of misalignment is 135 degrees of phase, so the sum would
// cancel rather than build. Magnitude averaging has no phase to lose.
#define AVG_STEPS 4
static const uint8_t kAvgCounts[AVG_STEPS] = {1, 4, 16, 64};
#define AVG_DEFAULT 1  // index into kAvgCounts

// ---------------------------------------------------------------------------
//  Settling
//
//  The scene-change metric is self-calibrating, like the direct path is. What
//  counts as "a lot of change" depends on the room, the gain and the averaging
//  depth, so rather than hard-coding a threshold the engine learns what change
//  looks like when nothing is happening and triggers on a multiple of that.
//
//  CHANGE_FLOOR only guards the case where the learned baseline is near zero -
//  a very quiet, very static scene - which would otherwise trigger on noise.
// ---------------------------------------------------------------------------
#define CHANGE_TRIGGER_K 3.0f
#define CHANGE_FLOOR     0.08f

// The baseline tracks the QUIET FLOOR, not the mean - fast down, slow up.
//
// A symmetric rate self-deafens. Bench log, board being physically moved:
//
//     chg 1.16/1.78      chg 0.93/2.40      chg 2.29/2.81
//     chg 0.78/2.22      chg 0.53/2.40      chg 1.42/2.83
//
// every one of those pings is a disturbance, none of them crossed the trigger,
// so all of them were fed to the baseline as "quiet" - which raised the
// threshold, which made the next one even less likely to cross. The trigger
// ran away from the signal and the move was never detected at all.
//
// Measured, board genuinely still: chg 0.14-0.16. Being moved: 0.78-2.29. That
// is 5-15x, an enormous margin - it was never a threshold problem.
#define CHANGE_BASE_DOWN 0.10f   // ~1.2 s to follow the floor down
#define CHANGE_BASE_UP   0.005f  // ~25 s to concede the room got noisier
#define SETTLE_SEED      8  // pings spent learning the baseline before arming

// A direct-path magnitude this far from its running mean means the BOARD moved,
// not the room: the direct path is speaker-to-mic across a fixed PCB, so it only
// changes when the board's own acoustic coupling does - picked up, set down on a
// different surface, a hand near the speaker. Measured: moving the board took it
// from ~6700 to ~12100, an 80% jump, where a person walking through the room
// leaves it flat.
//
// This is the discriminator that decides whether a learned REF is still valid.
#define DIRECT_MOVE_FRAC 0.25f

// ...and it has to hold for this many consecutive pings, about a second. A hand
// near the speaker moves the direct path just as much as picking the board up
// does; only one of them stays moved.
#define DIRECT_MOVE_PINGS 8

// Headline target lock. Without it the big number jumps between reflectors of
// similar strength every ping and looks broken while the measurement is solid.
#define LOCK_GATE_IN   6.0f
#define LOCK_MISS_MAX  6

// Display auto-gain. Echoes run 3-5% of the direct path, so a fixed scale is
// either empty or clipped; the top of the scale tracks the strongest echo.
#define AGC_ATTACK   0.30f
#define AGC_RELEASE  0.05f
#define AGC_MIN_FULLSCALE 0.015f  // never zoom in past 1.5% of direct

// ---------------------------------------------------------------------------
//  Layout
// ---------------------------------------------------------------------------
#define HEAD_H   44
#define LABEL_Y  48
#define LABEL_H  16
#define PLOT_X   15
#define PLOT_Y   68
#define PLOT_W   450
#define PLOT_H   180
#define AXIS_Y   (PLOT_Y + PLOT_H)
#define BTN_Y    276
#define BTN_H    40
#define BTN_W    114
#define BTN_GAP  4

#define C_BG     0x0000
#define C_PANEL  0x18E3
#define C_GRID   0x2945
#define C_TRACE  0x07E4
#define C_TRACE_HI 0x9FF3
#define C_AMBER  0xFD20
#define C_TEXT   0xFFFF
#define C_DIM    0x7BEF
#define C_REF    0x780F
