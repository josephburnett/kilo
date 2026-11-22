# The movement engine: what the recordings already say

For [movement-engine.md](movement-engine.md), Part one A: the seven
questions that could be answered without Kilo, from the flight recordings
on the laptop. Done 2026-11-22. Nothing new was tried on Kilo.

**What the numbers come from.** Every `.krec` in `~/kilo-logs` and
`~/kilo-logs/from-server`: 14 boots of the card, 2026-11-17 to 11-21, mind
builds fdc41a79, 21d13ba7, b684222a, f3020d5e and 0a4c7465. The same boot
is in several files (the card's copy, the server's, one file per run);
each piece of it is counted once. Movement in them: the **old** walk (the
diagonal pairs, replaced on 2026-11-22), the **old** trot, the long trot,
the crawl, 14 tries of the old rear-up designs, one self-measurement and
six falls. The new walk has never walked, so nothing here is about it
directly.

**To recompute** after the next floor runs (fetch the card first, as in
docs/walk-trial.md):

```
PYTHONPATH=mind python mind/tests/enginestudy.py all
```

(`inventory`, `link`, `margin`, `balance`, `lag`, `estimate`, `getup`,
`tricks` one at a time.)

**Confidence words.** *Measured*: read directly from the recordings.
*Inferred*: worked out from measurements with a stated assumption.
*Not determined*: the recordings cannot say, and why.

## 1. How late is the link?

**Question.** How short can the stream's look-ahead be, and how fast can a
decision in the mind reach the servos?

**Numbers** (38 walk runs, 17 926 ticks in GAIT, all five builds; old walk,
old trot, long trot, crawl):

| what | value |
|---|---|
| a FEET frame sent, to its tick's TELEMETRY back (look-ahead 5 ticks = 100 ms) | median 99 ms, 5-95 % 81-100, smallest 77 ms; none under 60 |
| the body's queue | 3 or 4 ticks in 99.8 % of ticks; 0-2 only in the first two ticks of a stream |
| underruns, late FEET (the body's STATS) | 0 and 0 in every boot (one underrun at the very end of two runs) |
| TELEMETRY arrival jitter (above the fastest arrival in its second) | median 0.6 ms, largest 2.7 ms |
| gaps between the mind's sends | median 20 ms; 29 over 40 ms, largest 80 ms (build 21d13ba7 had 60 ms stalls; f3020d5e and 0a4c7465 at most 40) |
| the mind's loop: a FEET's tick minus the newest TELEMETRY tick it had | 5 ticks in 86 %, 4-6 in 99.6 % |

**What it means.** The link and the mind's clock are very steady: a FEET
always reached the body at least 77 ms before it was needed, so about 65
ms of the 100 ms look-ahead was never used (the body reads FEET about
1.6 ms into its tick and reports about 5-9 ms in, inferred from
architecture.md §3a). A look-ahead of **3 ticks (60 ms)** would have kept
at least 25 ms in hand on every recorded tick, including the old build's
60 ms stalls; 2 ticks would have kept about 5 ms. With 3 ticks a reading
taken by the body reaches a changed command about 70-80 ms later (now
110-120), before the servos' own 50 + 70 ms.

**Confidence.** Measured (lead, queue, jitter, loop); the 25 ms and 5 ms
margins are inferred from the body's tick layout.

## 2. Does the planned support margin predict the rock?

**Question.** How far inside three feet must the weight be before the
third foot really holds it? (The two simulators disagree about the new
walk because one assumes 10 mm, the other about 0-3 mm:
walking-and-balance.md §16.5.)

**Numbers.** 681 swings of one leg alone (old walk 321, crawl 92, runs
mixing walk, crawl and trot 268). For each: the plan's smallest margin
(the model's centre of mass inside the three feet) and the measured tilt
about the tripod's diagonal toward the lifted corner, from the tick
before the lift to 60 ms after the landing (measured minus predicted):

| plan's margin | swings | tilt toward the lifted corner (deg, mean ± se) |
|---|---|---|
| 4 to 6 mm outside | 51 | +3.5 ± 0.2 |
| 2 to 4 mm outside | 86 | +3.4 ± 0.1 |
| 0 to 2 mm outside | 14 | +1.8 ± 0.2 |
| 0 to 2 mm inside | 100 | +1.7 ± 0.1 |
| 2 to 4 mm inside | 67 | +0.3 ± 0.1 |
| 4 to 6 mm inside | 76 | +0.1 ± 0.1 |
| 12 to 16 mm inside (the crawl, 14-tick swings) | 180 | +1.5 to +2.1 (a slow lean over 0.3 s, not a tip) |

**What it means.** The weight stops tipping toward the lifted foot
between about 0 and 2 mm inside the planned triangle, not at 10 mm: the
floor sides with the virtual body's own physics and RockingPhysics at 3 mm,
not at 10 mm. The self-measurement agrees (its tips came *after* the
centre of mass crossed the edge, tip delay 508 ms, never before). So the
new walk's +0.4 mm at the hand-over is at the edge: expect a small rock
there (about 1.5°), not the old walk's 4.8°. The engine should keep
**3 mm** inside three feet, not 10 (the crawl's 14 mm is far more than
needed).

**Confidence.** Inferred, medium. The swings with a margin from 2 mm
outside to 6 mm inside are the old walk's single-leg moments, 60 ms long
and followed by its two-feet phase, so their window is short and mixed;
the margin is the model's (the self-measurement put the centre of mass
3 mm further back, which moves each swing's margin by up to about 1.5 mm).
The new walk's floor try (B, item 8) settles it.

## 3. How much does the body's balance correction act?

**Question.** Does the body's own loop matter in gaits, or only standing?

**Numbers** (every TELEMETRY of the 14 boots, 98 915 frames):

| mode | frames | balance on | held (frozen) | correction when on: median, 95th pct, largest | attitude minus reference, median |
|---|---|---|---|---|---|
| IDLE (standing, sitting) | 44 514 | 22 % | 0 % | 0.75°, 4.7°, 21° | 2.8° |
| GAIT | 19 010 | 85 % | 64 % | 0.75°, 5.5°, 21° | 2.4° |
| TRICK, FALLEN, RECOVERING, LIMP | 35 391 | 0 % | | | |

Per boot the correction is mostly a steady nose-up lean of 0.3 to 1.2°
(pitch sd 0.2-0.6°) while the measured pitch stays 1.3 to 3° nose down of
the reference: that is the IMU's 2.4° mounting offset the
self-measurement found, which the body takes for a real tilt and leans
against slowly. The large values are boot 3 (Kilo in a hand) and moments
after falls.

**What it means.** In the gaits the loop is frozen two thirds of the
time (any foot up holds it) and otherwise carries a small steady lean, so
**it does not shape the gait**; standing it corrects small slow errors.
It is worth keeping on the board, as decided, as the layer that removes
slow error by itself; but today a fixed IMU offset is a large part of
what it "corrects".

**What the board reports today, against its role** (an independent layer
that takes poses, corrects error itself and reports it): it reports the
correction it carries (`corr_pitch`, `corr_roll`), the reference it holds
to (`ref_pitch`, `ref_roll`), the measured attitude and rates (so the
residual is the attitude minus the reference), whether the correction is
on or frozen (`BALANCE_ACTIVE`, `BALANCE_HELD`), which feet it corrects
through (`support`), its queue (`feet_queued`) and its tick time. It does
**not** report: whether a joint or slew limit cut what it was asked to
do; the joint angles it actually commanded (the JOINTS stream exists but
is in no recording); which part of the pose it could not reach; or its
own state of confidence (an IMU offset it has learned, a correction at
its limit). The engine needs the first two at least.

**Confidence.** Measured; that the steady lean is the IMU offset is
inferred (the same 2.4° the self-measurement fitted, on the same floor).

## 4. Does the servo lag change with the battery?

**Question.** One fixed lead for the servos, or track it live?

**Numbers.** The delay from the plan's attitude (no lag) to the measured
roll, by cross-correlation, in 10 walk and crawl runs (the trot runs'
plans hardly roll): median 123 ms, 105-127 ms, at 6644 to 7740 mV;
slope -0.4 ms per 100 mV (r -0.19): **no trend**. The model's 50 ms dead
time + 70 ms time constant gives about 120 ms on this measure; the
self-measurement fitted 70 ± 12 + 69 ± 15 ms.

**What it means.** One fixed lead is enough over the battery range seen.
Below 6.6 V nothing is recorded.

**Confidence.** Measured for 6.6-7.7 V, 10 runs, mostly the crawl; the
pitch axis gave one usable run only.

## 5. How good is the state estimate?

**Question.** Is the always-on estimate good enough to choose actions
from?

**Numbers** (`kilo.intent.replay`'s rules over every boot, TELEMETRY
counted where a posture had been asked for at least 3 s and the body was
idle):

| asked | agrees | wrong |
|---|---|---|
| stand | 99.7 % of 4 107 | 12 UNKNOWN (one 0.4 s stretch, 14° nose down) |
| sit | 100 % of 1 993 | |
| rest | 100 % of 24 906 | 8 ON_END (one 0.3 s stretch, being picked up) |
| limp | 100 % of 7 327 | |

363 changes of state in the 14 boots; only 2 were a resting state
flipping straight to another within a second. After a fall he is
almost never still before rc moves him (rc starts within 1.2 s), so only
65 frames judge the lying states: 61 agree with the attitude, 4 "on his
back" (by the righting tools' 30° rule) read ON_LEFT_SIDE.

**What it means.** Upright, the estimate is right and steady. Lying, it
is not tested: the recordings hold almost no still lying Kilo.

**Confidence.** Measured upright; not determined for lying (too few
still frames; the righting experiment's B1 falls will provide them).

## 6. How fast does he roll in a get-up, and how often did rc fail?

**Question.** The time budget of a get-up steered by the mind; what the
failures looked like.

**Numbers.** 9 falls with rc's tries in the boots (13 tries; 4 of the
falls in a hand, righting.md), plus one fall
with no try (boot 8, tail down: the body gave up at once, RECOVERY_FAILED).
On the floor (the six tries of righting-results.md, Kilo not in a hand,
plus boot 14's fall, whose hand is not known):

- 6 falls: 4 got up at the first try, 1 at the third (boot 8, on his left
  side: back onto his back, then the left side again, then up), 1 not
  tried (tail down). So **rc got him up at the first try in 4 of 6
  falls**; that is the owner's "didn't work 100 % of the time".
- Fastest roll in a try 272 to 579°/s (median 410); mean roll rate 44 to
  142°/s.
- The roll comes in bursts (rate over 60°/s): 42 bursts, median 0.16 s,
  5-95 % 0.08-0.76 s, rolling median 16°, up to 175°.
- He is still again 1.8 to 3.5 s after a try begins, which is rc's own
  1.78 s of moves plus 0.0-1.7 s.

**What it means.** One action's roll is over in 0.1 to 0.8 s, so a mind
looking every tick (now 40 ms: TELEMETRY is at 25 Hz outside a stream)
can see each action's result well within a second; switching after about
0.3 s with no progress in the roll fits the bursts.

**Confidence.** Measured, from few falls (6 on the floor); at 25 Hz the
fastest rates may be underestimated.

## 7. Do tricks play on schedule, and does the IMU follow the model?

**Numbers.** 14 trick runs, all old rear-up designs: 2 fell, 7 stopped by
a key, 5 refused (battery or busy); none ran to its end. The keyframes the
body reported starting (48) came 8 to 41 ms after the schedule (median
28), within what 25-50 Hz telemetry can resolve.

**What it means.** The body's player keeps time. Whether the IMU follows
the model off a plain stance is **not determined**: every recorded trick
is a rear-up design that has since been replaced (its library entry's
checksum no longer matches), so no prediction can be made for them, and
none is a three-foot paw trick.

**Confidence.** Measured (timing); not determined (the model check).

## What this changes in the proposal

- The stream's look-ahead can come down from 100 ms to **60 ms** (3
  ticks), which also shortens every loop through the mind to about 75 ms.
- The weight target for three feet is **3 mm inside**, not the 10 mm
  estimated for "lick a paw"; RockingPhysics at 10 mm should not be used
  to judge the new walk.
- The servo lead stays **fixed** (one number, refitted by the
  self-measurement), not tracked live.
- The get-up policy's "no progress" switch: about **0.3 s**.
- The body's correction is small and slow and mostly frozen in gaits, which
  fits it staying on the board as its own layer; the board's report must
  add what limits cut and what it commanded, and the IMU's fitted offset
  must reach the board, or it keeps leaning against a tilt that is not
  there.
- The new walk's floor try is still needed (item 2 is inferred); no trial
  is needed for the link delay or the battery lag.
