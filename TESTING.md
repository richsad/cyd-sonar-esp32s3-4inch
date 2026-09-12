# Bench protocol

Ordered so that a failure short-circuits the rest: each test assumes the ones
above it passed. Roughly 20 minutes end to end.

Everything here is read off the serial telemetry, which is the instrument for
testing purposes - the panel is for using it.

```bash
arduino-cli compile --upload -p /dev/cu.usbmodemXXXX --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,CDCOnBoot=cdc Sonar
```

Capture the whole session rather than watching it scroll:

```bash
tools/cyd_serial.py -p /dev/cu.usbmodemXXXX -t 600 | tee /tmp/sonar-session.log
```

The telemetry line:

```
 8.2/s  n 12/64 settling  REF   direct 6716  chg 0.21/0.68 |  4' 9.6" 4.0%  ...
  |        |       |       |        |          |    |
  |        |       |       |        |          |    live trigger threshold
  |        |       |       |        |          this ping's change
  |        |       |       |        direct-path magnitude
  |        |       |       reference state
  |        |       settling state
  |        depth in use / depth requested
  ping rate
```

---

## 1. Does it settle at all?

**The most important test, and the one most likely to fail** - `CHANGE_TRIGGER_K`
and `CHANGE_FLOOR` are reasoned guesses that have never seen real data.

Put the board on a table, hands off, and leave it alone for 30 seconds.

- **Pass:** state reaches `LOCK` within a few seconds and stays there. `chg`
  sits clearly below the threshold beside it.
- **Fail, never locks:** `chg` is at or above the threshold with nothing moving.
  Too sensitive - raise `CHANGE_TRIGGER_K`.
- **Fail, thrashing:** flips between `LOCK` and `CHANGING` with the room still.
  Same fix.

Write down the resting `chg` value. Every other threshold is judged against it.

## 2. Does the depth climb?

Press **AVG** until the log reads `/64`, then leave it alone.

- **Pass:** `n` climbs 1 → 64 one ping per line, reaching `64/64` in about eight
  seconds, then `LOCK`.
- **Fail:** `n` sticks low. The change metric is re-triggering; same fix as (1).

## 3. Does it react?

At `AVG 64`, wave a hand a couple of feet in front of the board.

- **Pass:** state goes `CHANGING`, `n` drops to 1, and it climbs back to `LOCK`
  within about ten seconds of you stopping.
- **Fail, no reaction:** too dull - lower `CHANGE_TRIGGER_K`.

This is the test that proves the adaptive averaging does what it claims. Compare
against holding still: `chg` while waving should be several times the resting
value from (1). If it is not, the metric itself is wrong, not the threshold.

## 4. Buttons

Tap each of the four once. Every press logs, hit or miss.

```
[touch] x=180 y=295 raw  25,139 -> REF
```

- **`-> AVG` / `REF` / `BAND` / `HOLD`** - working.
- **`-> MISS` with sane coordinates** - hit test or coordinate mapping is wrong;
  the logged numbers say exactly how.
- **no line at all** - the FT6336 is not reporting. Note whether it stops after
  a while rather than never starting: the talk radio build saw the controller
  wedge when codec I2C traffic interleaved with touch polling, which Sonar now
  does deliberately during every capture.

## 5. REF

Press **REF** with the room still.

- **Pass:** the trace flattens to near black within a ping or two, and the
  button reads `REF ON`.
- Then wave a hand: it should appear out of nothing, brightly.
- Then hold a hand **6 inches** from the board - inside the blind zone. It
  should show. This is the only way to see anything closer than 15 in, and it is
  the clearest demonstration of what REF is for.

Press **REF** again to clear.

## 6. MOVED

Load a reference (**REF**), let it settle, then pick the board up and set it
back down somewhere else.

- **Pass:** badge reads `MOVED`, button reads `REF !!`.
- **Fail, never fires:** `DIRECT_MOVE_FRAC` too high. The measured jump when the
  board was moved was 6700 → 12100, about 80%, against a threshold of 25% - so
  this should fire easily. If it does not, watch whether `direct` actually moves.
- **Fail, fires constantly:** too low; raise it.

Also check the negative case: walking in front of the board should **not**
produce `MOVED`, only `CHANGING`. That distinction is the whole point.

## 7. Accuracy - the real test

Board on a table facing a flat wall, square on, nothing else within a couple of
feet of the line between them. Measure with a tape from the **face of the
board** to the wall. Compare at 3, 5 and 7 ft.

Record all three. The error pattern says which constant is wrong:

| Pattern | Cause | Fix |
|---|---|---|
| Error grows with distance, same % each time | Speed of sound. The constant is set for 68 F; a 75 F room is 0.8% fast, which is 0.6 in at 6 ft. | `SOUND_IN_S` in `config.h` |
| Same offset at every distance | The direct path is not being found where it is assumed to be. | Needs investigation, not a constant |
| Random scatter | Not a calibration problem. Check the wall is square on. | - |

A flat, hard, square-on wall is the friendliest possible target. A wall at an
angle scatters the chirp away rather than back and may not register at all -
that is physics, not a fault, but do not start the accuracy test on one.

## Test it untethered

Do this before believing any UI result. With a serial capture attached, the host
drains the USB buffer and hides the single worst bug this project has had - see
the USB serial trap in the README. A UI that works while you are watching it
proves nothing on this board.

Unplug from the host, or attach nothing, and check that the chirp is steady and
every button responds.

## Touch controller reference state

The FT6336 intermittently answers I2C, reads back the right chip ID, and
reports zero touches while the sonar watches a finger five inches from the
panel. A real controller reset does not clear it - GPIO18 is confirmed to hold
the part in reset - but a full board reset does. Bus speed is not the cause
(100 kHz behaves the same) and the backlight is fixed-on, never PWM, so neither
of the obvious environmental suspects applies.

When it next wedges, **do not reset it**. Run `t` over serial and diff against
this, captured healthy:

```
regs 0x00-0x06: 00=0 01=0 02=0 03=0 04=0 05=0 06=0
regs 0x80,0x88,0xA0-0xA8: 80=f 88=a A0=2 A1=5 A2=1 A3=64 A4=1 A5=0 A6=a4 A7=0 A8=11
```

| Register | Healthy | Reads differently when wedged? |
|---|---|---|
| `A5` PWR_MODE | `0` = Active | `1` Monitor or `3` Hibernate means the part is asleep - that is the whole answer |
| `80` threshold | `0f` | A larger value means it has been desensitised rather than stopped |
| `88` report rate | `0a` | A collapsed rate would explain missed touches |
| `A4` interrupt mode | `1` | - |
| `A3`/`A8` id | `64`/`11` | Already known good, since the chip ID read succeeds while wedged |

If every register matches this baseline exactly, the controller believes it is
working normally, and the fault is the panel or its connection rather than
anything addressable in firmware.

## Tuning constants

All in [`Sonar/config.h`](Sonar/config.h).

| Constant | Now | Raise it when |
|---|---|---|
| `CHANGE_TRIGGER_K` | 3.0 | It never settles |
| `CHANGE_FLOOR` | 0.08 | It never settles in a *very* quiet room |
| `DIRECT_MOVE_FRAC` | 0.25 | `MOVED` fires when you have not touched it |
| `LOCK_GATE_IN` | 6.0 | The headline will not follow a target you are moving |
| `LOCK_MISS_MAX` | 6 | The headline greys out too eagerly |

## License and credits

MIT - see [LICENSE](LICENSE). Written with
[Claude Code](https://claude.com/claude-code) (Claude Opus 5).
