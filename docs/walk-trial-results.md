# The new walk: floor try results

Runs of [walk-trial.md](walk-trial.md). Newest first.

## 2026-11-22, boot 16: the first floor run of the new walk (from the recording)

Build 7e92703c, kernel with `KILO_REAR_UP=1 KILO_RIGHTING=1`. What the
owner saw has not been written down yet; everything here is what the
instruments recorded (the mind's log `~/kilo-logs/kilo-card-b16.log`, the
whole-boot recording `rec-card-b16.krec`, the walk recordings
`run-b16-25s-walk-1` and `run-b16-32s-walk-2`), studied with
`mind/tests/walkstudy.py floor`. Times are seconds after the Pi started.

**What was asked.** 25 s: the distance guard backed him off at 40 mm/s
(something 5 cm from his nose as he stood up). 32 s: C4, the walk at
58 mm/s; B5 three times (41, 23, then 23 backward); B7 up to the top level,
72.5 mm/s; C6 at 47 s (steering right); A4 at 56 s: the trot, straight to
201.6 mm/s with the steering kept; A5 at 63 s stopped it. Both walks
ended as planned: 0 holds, 0 catches, 0 waits, 0 FEET underruns, the FEET
frames 80 to 102 ms ahead of their tick, 3 to 4 ticks queued. He did not
fall in either: the one fall of the boot came 46 s after the trot ended,
standing still (docs/righting-results.md).

**Footfalls [measured].** Every folded cycle is exactly 48 ticks (960 ms):
one foot up at a time, in the order LB, LF, RB, RF, each up for 10 ticks
with 2 ticks on four feet between: a footfall every 240 ms, as planned.

**The rock, against the old walk** (`walkstudy.py floor`; the old walk's
numbers are §16.1 of walking-and-balance.md, 16 cycles at 80 mm/s):

| | old walk, 80 mm/s | new walk, 58 mm/s (3 cycles) | new walk, 72.5 mm/s (2 cycles) | new walk, 23 mm/s (1 cycle) |
|---|---|---|---|---|
| roll swing per cycle | 4.8° | 4.9° (sd 0.4) | 4.3° (sd 0.2) | 6.4° |
| fastest roll | 73°/s | 41°/s | 47°/s | 52°/s |
| roll and pitch rate, rms | 36°/s | 20.8°/s | 21.0°/s | 29.5°/s |
| pitch swing | 1.9° | 3.1° | 2.6° | 3.3° |

Over whole stretches (the TELEMETRY, every tick): walking forward at 58
and 72.5 mm/s the roll stayed within -2.4 to +3.1°, with steering right
-2.5 to +2.7° (fastest 63°/s). Backward it rocks more: at 23 mm/s -3.0 to
+4.7°, rate rms 38°/s; the guard's backing off at 40 mm/s -4.0 to +5.6°,
fastest 90°/s, rms 35°/s (the model said backward has the least margin,
-4 to -6 mm: §16.4).

Read:

- **Not smoother in roll, smoother in speed.** The body still rocks about
  as far from side to side each cycle as the old walk did (4.3 to 4.9°
  against 4.8°), but half as fast (the fastest roll 41 to 47°/s against
  73, the rate rms 21 against 36°/s), because the cycle is twice as long.
  Pitch rocks more (2.6 to 3.1° against 1.9°).
- **The simulators.** Of the three physics of §16.5, RockingPhysics with
  10 mm (the weight must be 10 mm inside three feet before they hold it)
  predicted this: a roll swing of 5.9° at 58 mm/s and 4.1° at 73, against
  4.9 and 4.3° measured (its rates were too high: 80 and 58°/s against
  41 and 47). The other two predicted 1.0 to 1.4°: 3 to 5 times too
  little. So on the floor the third foot does not hold the weight while
  the margin is near 0 [inferred]. That agrees with the self-measurement
  fit of the same day (docs/self-measurement-results.md): the walk's real
  margin is about -0.5 mm where about 3 mm is needed.
- **The slowest level rocks most** (6.4° at 23 mm/s, one cycle only), as
  RockingPhysics also said (5.9°).

**The trot at 201.6 mm/s, steering right** (A4 straight to the top,
7.4 s, 365 ticks): roll -2.8 to +5.4° (on average 1.3° with his right
side up, leaning to the left, the outside of the right turn), fastest roll 69°/s, rate rms
41°/s, pitch -0.9 to +3.9°. 0 holds, 0 catches, no fall. There is no
earlier floor number for this trot at its top speed with steering to set
it against.

Not known: how it looked, whether a foot scraped or slipped, whether he
walked straight. The side video of walk-trial.md was not taken (or has
not arrived).
