# CYD Sonar — Hosyond ESP32-S3 4.0"

An acoustic tape measure. The board chirps out of its own speaker, listens on
its own microphone, and cross-correlates the two to find the echo. Distance to
a wall, in feet and inches, with no host and no extra hardware.

The speaker's output *is* the microphone's stimulus — this is one instrument,
not two features sharing a board.

Hardware is the Hosyond ESP32-S3 4.0" touchscreen module, the same board as
[cyd-talk-radio-esp32s3-4inch](../cyd-talk-radio-esp32s3-4inch), whose
corrected pinout and ES8311 findings this project builds on.

---

## Status

**Working, and verified end to end on hardware** against the bench protocol in
[TESTING.md](TESTING.md). Ranging at **8.2 pings/sec**:

```
 8.2/s  n 4/4  LOCK  direct 7540  chg 0.18/0.53 | *4' 9.8" 9.4%  6' 3.0" 5.5%  9' 9.2" 3.2%
```

| Test | Result |
|---|---|
| Settles at rest | Resting change 0.13-0.20 against a 0.50 trigger, about 3x margin |
| Depth climbs | 1 -> 64 one ping per ping, LOCK in ~8 s |
| Reacts to the room | Goes `CHANGING` and re-acquires |
| Buttons | All four, including presses on the very bottom row of pixels |
| REF | Nulls the static room from 5-6% down to ~1% |
| Target lock | Holds a weaker return rather than chasing the strongest |
| Board moved | `MOVED` fires on a real move, not on a hand reaching for the screen |
| Accuracy vs tape | **not yet done** |
| Untethered | Constant 8/s chirp, all buttons responsive, nothing attached |

The clearest single result is the near field with a reference loaded. A hand six
inches from the board - inside the 15 in zone the raw trace cannot see at all -
comes back at **34-53% of the direct path**, against room echoes at 2-3%:

```
 *4.5"  53.4%    6' 8.4"  3.2%    2' 11.8"  2.4%
```

Treat those sub-6 in numbers as "something very close" rather than as a
measurement: they sit right against the 4 in search floor and are probably
clipped by it.

[`MicTest/`](MicTest/MicTest.ino) is the bring-up sketch that got here, kept as
a regression check. It proved, in order, that the mic produces samples at all,
that enabling RX does not silence TX, and that a chirp can be emitted and
captured in a single frame.

Two findings worth carrying to any other project on this board:

- **Full duplex works.** The talk radio project recorded that adding an RX
  channel silenced the speaker and abandoned the mic over it. The cause was
  *two* `i2s_new_channel()` calls fighting over the same BCLK/WS pins. One call
  returning both handles is what actually makes the peripheral full duplex,
  and TX is untouched by it.
- **The codec latency is real and variable.** The direct path lands anywhere
  from sample 1056 to 1063 - about 22 ms through the codec and the DMA rings,
  drifting between runs. Nothing here is timed from when the chirp was queued.

## Using it

### Reading the screen

```
┌──────────────────────────────────────────────┐
│ SONAR    6' 3.7"                  3.2%  8.1/s│
│        4' 9.6"      6' 3.7"                  │
│  ░░                                          │
│  ░░     ▐█▌           ▐█▌                    │
│  ░░  ▂▃▄███▄▃▂     ▁▂▄███▄▂▁                 │
│ ─┴────┴────┴────┴────┴────┴────┴────┴────┴── │
│  0'   2'   4'   6'   8'   10'  12'           │
│   [AVG 4]  [REF --]  [WIDE]  [HOLD]          │
└──────────────────────────────────────────────┘
```

Top line: the strongest target, its echo strength as a percentage of the direct
path, and the ping rate. Below it, up to three peaks labelled in place.

Each bump is a real surface. Height is echo strength, horizontal position is
distance. The hatched strip on the left is the 15 in blind zone where the
speaker is still ringing from its own chirp.

**The whole point of the A-scope is that the targets stay put.** An ordinary
room holds three or four stable reflectors of similar strength, and they trade
places as "strongest" from one ping to the next - so a single-number readout
flickers wildly while the measurement underneath is solid to a tenth of an inch.
Drawing all of them shows that at a glance.

Nothing is ever full scale except by accident: the top of the scale tracks the
strongest return outside the blind zone, so a quiet room still shows detail
rather than a flat line. Read the percentage, not the bar height, when comparing
two moments.

### The buttons

| Button | What it does | When you would reach for it |
|---|---|---|
| **AVG** | Cycles 1 → 4 → 16 → 64 pings of averaging | A return you can almost see. AVG 64 is about eight seconds of integration and will pull it out of the noise. Costs responsiveness: anything moving smears. |
| **REF** | Learns the room as it stands and subtracts it | The interesting one - see below. Press again to clear. |
| **BAND** | WIDE 4-18 kHz ⇄ STEALTH 14-19 kHz | Stealth is near-inaudible, at about 1.4 in resolution instead of half an inch, and weaker returns because the speaker rolls off up there. Leave it on WIDE unless the ticking bothers you. |
| **HOLD** | Freezes the trace | Capturing a reading to look at. |

### The two things it does

**As a tape measure.** Point the speaker edge at a wall and read the labelled
peak. Best on a flat surface square to the board, between 2 and 10 ft. A surface
at an angle scatters the chirp away rather than back, so an oblique wall reads
weak or not at all - that is physics, not a fault.

**As a motion detector.** Press **REF** pointing at the empty room. Everything
static drops to flat black, including the speaker's own ringing, and only
*changes* appear. Walk in front of it and you are the only thing on screen.

**With a reference loaded the near field stops being blind.** The 15 in floor
exists because of the ring, and the ring is exactly what the reference
subtracts, so the search closes to 4 in while REF is on. A hand held six inches
from the board - invisible to the raw trace - appears out of black.

### Settling, and what the badge means

The board never stops pinging, but a reading that is still converging looks
exactly like a settled one unless something says so. The badge under the
wordmark says which:

| Badge | Meaning |
|---|---|
| **LOCK** | Averaging is at the depth you asked for and the scene has stopped moving under it. The number is good. |
| **12/64** | Still converging - 12 pings deep out of the 64 requested. |
| **CHANGING** | Something in the room moved. Averaging has collapsed to 1 ping and is climbing again. |
| **MOVED** | The *board* moved, which is a different thing - see below. |

Averaging depth is adaptive. The **AVG** button sets a ceiling, not a fixed
value: the moment the scene changes, depth collapses to a single ping so the
display re-acquires immediately, then climbs back one ping at a time. Fast when
something happens, smooth when nothing does, with nothing to fiddle with.

The scene-change metric is self-calibrating, like the direct path is. Rather
than a hard-coded threshold, it learns what change looks like when nothing is
happening and triggers on a multiple of that - so it adapts to a noisy room,
a different gain, or a different averaging depth on its own.

### Board moved, or room moved?

These need opposite responses, and the direct path tells them apart.

The direct path is speaker-to-mic across a fixed PCB, so its magnitude only
changes when the **board's own acoustic coupling** does - picked up, set down on
a different surface, a hand near the speaker. A person walking through the room
leaves it flat. Measured: moving the board took it from ~6700 to ~12100.

That matters because of **REF**. A reference learned at the old position nulls
real targets and invents phantom ones, and there is no way to tell from the
trace alone. So when the board is handled with a reference loaded, the button
goes to **REF !!** and the badge reads **MOVED**. It is not cleared for you -
that would silently discard something you deliberately set - but you should
press REF twice to re-learn it, or once to drop it.

A person walking in front of the board does *not* invalidate the reference.
That is REF working.

### Over serial, at 115200

One telemetry line per second, plus `p` to dump the full profile as text:

```bash
tools/cyd_serial.py -p /dev/cu.usbmodemXXXX -s p -t 8
```

```
 8.2/s  avg 4   direct 6716  fs 4.00% |  4' 9.6"  4.0%   9' 8.6"  2.7%   6' 4.0"  2.5%
```

`direct` is the raw direct-path correlation magnitude and `fs` is where the
auto-gain has put full scale. Both are diagnostics: `direct` changing by a
factor of two means the board moved or something is touching the speaker, and
`fs` far above the strongest labelled peak means something broad and unpeaked is
in the profile - which is how the speaker's ring extent got measured.

Touch presses log as they happen, hit or miss:

```
[touch] x=180 y=295 raw  25,139 -> REF
```

## How the ranging works

A short chirp — 4 kHz to 18 kHz over 5.3 ms — is emitted and the microphone is
captured through the same frame. Correlating the capture against the chirp
(a matched filter) collapses the sweep to a sharp peak wherever a copy of it
arrives. Peak width is roughly 1/bandwidth: 14 kHz of sweep gives ~70 µs, or
just under half an inch of range resolution.

**The direct path is the clock.** The first and largest peak is the speaker
leaking straight into the mic an inch or two away, arriving in well
under 0.1 ms. Every echo is timed from *that* peak, never from when the chirp
was queued. This matters more than it sounds: DMA latency in both directions
and the codec's own group delay are unknown, variable, and add up to tens of
milliseconds — and all of it cancels exactly, on every single ping. The
instrument recalibrates itself continuously and there is no constant to trim.

```
distance = (peak_index − direct_index) × c / sample_rate / 2
```

At 48 kHz one sample is 0.28 in of round trip, so 0.14 in of distance.

All distances are reported in feet and inches.

## The USB serial trap

**`Serial.setTxTimeoutMs(0)` is not optional on this board.**

The ESP32-S3 native USB port is USB-Serial-JTAG. Once the host has enumerated
it but no application is reading, the TX buffer fills and `Serial.write` stalls
waiting for a reader that is never coming. Everything here runs in `loop()` -
the button handler included - so a stalled print is a stalled instrument, and
the telemetry line goes out once a second.

The symptom is buttons that go dead for a minute at a time while the chirp
carries on, because the ranging is upstream of the print and the UI is
downstream of it.

What made this cost hours is the shape of the evidence. The fault appeared
intermittent, and it was not: it correlated with whether a serial capture was
attached. Every attempt to observe it drained the buffer and made it go away.
"It works when you are watching" got read as sporadic behaviour, and sent the
investigation through the touch controller, the I2C bus speed, the backlight,
brownout resets and the FT6336's power-mode registers - none of which were
involved. The diagnostics were the bug.

Two guards, both cheap: a zero TX timeout so a write can never wait, and a
`if (!Serial)` check so a line is not even formatted with no host session to
receive it.

Anything else on this board that prints periodically from `loop()` has the same
exposure, whether or not it has noticed yet.

## Why the room's noise does not bother it

Observed on the bench: it keeps working with audio and video playing in the
same room. That is pulse compression, borrowed from radar, and it is the reason
a tiny speaker driven at 0.7 amplitude is enough.

The matched filter collapses 256 samples of *known* waveform into one peak.
Anything uncorrelated with that waveform cannot form a peak - it smears across
every lag instead. The gain against uncorrelated noise is the time-bandwidth
product:

```
T x B  =  5.33 ms x 14 kHz  =  74.7  ->  ~18.7 dB
```

Music and speech pay that before anything else, and two more things work
against them: their energy sits mostly below 4 kHz, outside the 4-18 kHz band
the filter is sensitive to at all, and every number on screen is a fraction of
the direct path, which is re-measured on every ping - so a louder room does not
move the scale, it only adds smear.

What *would* interfere, in rough order of likelihood:

- **Broadband impulses** - a door slam, a dropped plate, a clap. Brief, and
  averaging rides over them.
- **Real energy in 4-18 kHz** - cymbals, hissing, jangling keys, sibilance.
- **A second CYD Sonar in the same room.** It would emit the *same* chirp,
  which correlates perfectly and lands as a phantom target at whatever range
  the path length implies. Two units sharing a room would need chirps coded to
  be orthogonal - a different sweep direction is the cheap version.

Stealth mode gives up most of this. A 5 kHz sweep is TB = 27, about 14 dB, and
it sits in the band where hiss and cymbals actually live.

## Known limits

- **Minimum range is 15 in**, and that number is measured rather than assumed.
  The speaker's ring after the chirp runs 100% of the direct path at its peak,
  is still 8% at 9 in, and only falls below 2% past 18 in. **REF** subtraction
  is what actually reclaims this range; the blind-zone constant only keeps the
  ring out of the auto-gain and the peak list.
- **Maximum range** is set by SNR, not by the capture window. A small speaker
  and an uncalibrated mic will run out long before the 85 ms window does.
- **Absolute levels are meaningless.** The mic has no calibration, so echo
  strength is only ever a percentage of the direct path.
- **The chirp is audible** in wide mode — a short tick. `n` switches to a
  14–19 kHz sweep most adults cannot hear, at the cost of a peak about four
  times broader.

Bench protocol for validating a build: [TESTING.md](TESTING.md).

## Building

Arduino IDE or `arduino-cli`, ESP32 core 3.x, **ESP32S3 Dev Module**, PSRAM
enabled. `MicTest` needs no libraries at all beyond the core.

```bash
arduino-cli compile --upload -p /dev/cu.usbmodemXXXX --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,CDCOnBoot=cdc MicTest
```

`CDCOnBoot=cdc` routes `Serial` over the native USB port. Open the monitor at
115200 and press `?`.

## Board notes carried over

These were measured on the talk radio build, not taken from the listing — the
listing is wrong about all three:

| | Listed | Actually |
|---|---|---|
| Panel | 240×320 | **320×480** (ROT 1 → 480×320 landscape) |
| Driver | ILI9341 | **ST7796** |
| Audio | I2S amp, `IO1` enable | **ES8311 codec**, `IO1` **active LOW** |

The ES8311's ALC is switched **off** here. Sonar needs a fixed gain — an
automatic level control would quietly rescale the echo against the direct path
and make every amplitude on screen a lie.

## License and credits

MIT — see [LICENSE](LICENSE).

Built with [Claude Code](https://claude.com/claude-code) (Claude Opus 5).

Pinout, ES8311 register set and the corrected panel specs come from
[cyd-talk-radio-esp32s3-4inch](../cyd-talk-radio-esp32s3-4inch), which derived
them by measurement after [LCD wiki's 2.8" ESP32-S3
page](https://www.lcdwiki.com/2.8inch_ESP32-S3_Display) proved to be a starting
point rather than a reference. The chirp/matched-filter ranging and the
direct-path self-calibration are original to this project.
