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

**Boot 16 (2026-11-22), the random mode's tricks** [measured, from the
mind's log; the random mode's tricks are not recorded as runs, so only
the log's TRICK_DONE times exist]. A5 started the behaviour runner, then
four more A5 presses asked for check_around, wash_face, stretch and sniff.
Each played to its end (TRICK_DONE DONE) 4 to 20 ms after its scheduled
length: check_around 4024 ms (4020 scheduled), wash_face 6135 (6120),
stretch 4587 (4580), sniff 3955 (3940); the runner's own sit moves alike
(1959 / 1940, 2370 / 2360, 2469 / 2460, 2474 / 2460, 2187 / 2180). The
two asked while another move was playing (stretch, then sniff 2 s later)
waited and played one after the other, as designed. They end in the
sitting pose; the state estimate said SITTING (pitch -26.5 to -27.0°)
0.3 s after wash_face and after each sit move, and had no time to settle
after check_around, stretch and sniff before the next move began. The six righting moves of the same boot played 9 to 14 ms
over their 2660 ms, their keyframes 31 to 37 ms after the schedule
(`enginestudy.py tricks`). Nothing odd, except that the distance guard
saw something 6 to 8 cm from his nose for 1.8 s during sniff (89.1 s:
eyes red, no action, as designed for a trick).

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

## Step 2: the solver judged against the tuned planner (2026-11-23)

**Question.** Is the engine's step clock and solver (kernel/mind/engine,
docs/movement-engine.md step 2) at least as good as the hand-tuned planner
it replaces, gait by gait? The owner's rule: replace a gait only where it
is; the walk and the trot "looked REALLY good" on the floor.

**How it was judged** (simulation only; nothing was tried on Kilo). Both
the old planner (the C++ of main at 0142b79, built once more for this) and
the engine planned the same walks tick by tick with no body
(`engine-judge`); each plan was played on the virtual body with the
judge of docs/model-fit-results.md (RockingPhysics: the weight held by
three feet as soon as it is inside them, the support lagging 55 ms, the
legs giving as the model says) and the body's own balance loop, over the
whole cycles from tick 150 to 550. To recompute:

```
make -C kernel/host build/engine-judge
PYTHONPATH=mind python mind/tests/enginejudge.py table      # the gaits at the remote's speeds
PYTHONPATH=mind python mind/tests/enginejudge.py changes    # speed changes, stops, quick stops, steer
```

The columns: the plan's least margin of the centre of mass inside three
feet (mm, by the model); the measured roll and pitch swing per cycle
(degrees, peak to peak: how much he rocks); the rates' rms (deg/s); the
measured attitude against the one the plan predicted (rms pitch / roll and
the largest, degrees: how well the plan predicts the body); the speed over
the ground (mm/s); the fastest joint (deg/s; the body's slew allows 500);
ticks the body's slew or reach cut; and the largest error with the model
wrong (the centre of mass 3 mm back, 5 mm ahead or 3 mm right, the feet
133 x 69 mm, the servos slower or faster) and the falls among them.

| gait | speed | | margin | roll / pitch swing | rate rms | error rms | largest | speed | fastest joint | cut | model wrong: largest / falls |
|---|---|---|---|---|---|---|---|---|---|---|---|
| walk | 23.2 | old | +0.5 | 1.52 / 0.86 | 7.8 | 0.35 / 0.60 | 0.9 | 18 | 308 | 0 | 1.8 / 0 |
| | | **new** | **+3.5** | **1.42** / 0.91 | **6.4** | 0.35 / **0.54** | 0.9 | 17 | **277** | 0 | **1.4** / 0 |
| walk | 40.6 | old | +1.3 | 1.89 / 0.76 | 7.9 | 0.32 / 0.49 | 0.8 | 31 | 356 | 0 | 1.4 / 0 |
| | | **new** | **+3.4** | **0.97** / 0.81 | **5.8** | 0.32 / **0.46** | 0.8 | 31 | **330** | 0 | **1.3** / 0 |
| walk | 58 | old | +1.8 | 2.12 / 0.66 | 8.0 | 0.28 / 0.43 | 0.7 | 44 | 405 | 0 | 1.1 / 0 |
| | | **new** | **+3.4** | **0.63** / 0.69 | **5.2** | 0.28 / **0.41** | 0.7 | 45 | 407 | 0 | 1.2 / 0 |
| walk | 66.7 | old | +1.8 | 2.21 / 0.59 | 7.9 | 0.27 / 0.43 | 0.7 | 50 | 386 | 0 | 1.1 / 0 |
| | | **new** | **+2.5** | **1.13** / 0.64 | **5.7** | 0.26 / 0.40 | 0.7 | 52 | 432 | 0 | 1.3 / 0 |
| walk | 72.5 | old | +1.8 | 2.22 / 0.57 | 7.9 | 0.26 / 0.42 | 0.7 | 53 | 444 | 0 | 1.2 / 0 |
| | | **new** | **+2.7** | **1.00** / 0.60 | **6.0** | 0.26 / 0.44 | 0.9 | 56 | 459 | 0 | 1.4 / 0 |
| trot | 57.6 | old | | 0.88 / 1.02 | 14.0 | 0.34 / 0.61 | 1.1 | 32 | 295 | 0 | 1.6 / 0 |
| | | new | | 0.88 / 1.02 | 13.9 | 0.33 / 0.61 | 1.1 | 32 | 297 | 0 | 1.6 / 0 |
| trot | 100.8 | old | | 1.15 / 0.54 | 11.5 | 0.19 / 0.34 | 0.7 | 59 | 321 | 0 | 1.8 / 0 |
| | | new | | 1.01 / 0.52 | 9.7 | 0.18 / 0.31 | 0.6 | 59 | 310 | 0 | 1.8 / 0 |
| trot | 144 | old | | 2.25 / 0.91 | 20.1 | 0.38 / 0.67 | 1.4 | 77 | 362 | 0 | 2.0 / 0 |
| | | new | | 2.26 / 0.85 | 19.5 | 0.37 / 0.65 | 1.4 | 77 | 344 | 0 | 2.0 / 0 |
| trot | 180 | old | | 4.14 / 1.40 | 29.9 | 0.74 / 1.22 | 2.1 | 81 | 380 | 0 | 3.5 / 0 |
| | | new | | 4.10 / 1.07 | 24.9 | 0.77 / 1.26 | 2.1 | 83 | 379 | 0 | 2.7 / 0 |
| trot | 207 | old | | 4.75 / 1.35 | 29.8 | 0.89 / 1.45 | 2.4 | 85 | 451 | 0 | 3.2 / 0 |
| | | new | | 4.74 / 1.34 | 29.3 | 0.89 / 1.44 | 2.4 | 86 | 427 | 0 | 3.4 / 0 |
| crawl | 8.9 | old | +13.4 | 21.5 / 0.83 | 14.3 | 0.34 / 1.01 | 2.1 | 8 | 333 | 0 | 4.0 / 0 |
| | | **new** | +7.3 | **9.6** / 1.18 | **6.6** | 0.33 / **0.43** | **0.8** | 8 | 341 | 0 | **1.6** / 0 |
| crawl | 15.5 | old | +13.5 | 19.5 / 0.72 | 16.7 | 0.36 / 1.06 | 2.2 | 14 | 406 | 0 | 4.4 / 0 |
| | | **new** | +7.3 | **8.4** / 1.02 | **6.5** | 0.30 / **0.38** | **0.7** | 14 | 422 | 0 | **1.5** / 0 |
| crawl | 22.2 | old | +13.6 | 20.7 / 0.96 | 24.6 | 0.40 / 1.17 | 2.2 | 19 | 392 | 0 | 5.8 / 0 |
| | | **new** | +7.2 | **9.4** / 0.98 | **8.6** | 0.33 / **0.46** | **0.7** | 19 | 381 | 0 | **1.6** / 0 |
| crawl | 25 | old | +13.6 | 20.9 / 0.93 | 27.6 | 0.42 / 1.30 | 2.3 | 21 | 392 | 0 | 6.4 / 0 |
| | | **new** | +7.3 | **9.4** / 0.96 | **9.2** | 0.33 / **0.49** | **0.8** | 21 | 381 | 0 | **1.5** / 0 |

The changes (`changes`; from the change to the end; the old against the new):

| gait | asked | largest tilt (old / new) | largest error | done at tick |
|---|---|---|---|---|
| walk 40.6 | to 72.5 at tick 200 | 1.1 / 0.8 | 0.8 / 0.9 | |
| walk 58 | stop at 300 | 2.8 / 1.6 | 2.8 / 1.2 | 406 / 405 |
| walk 58 | quick stop at 300 | 2.6 / 2.5 | 2.6 / 1.2 | 370 / 380 |
| walk 58 | steer 1 at 200, back at 350 | 3.4 / 3.3 | 3.4 / 1.9 | |
| walk 58 | to the trot at 200 | 3.1 / 2.1 | 3.1 / 1.2 | |
| trot 100.8 | to 201.6 at 200 | 2.5 / 2.4 | 2.5 / 2.4 | |
| trot 144 | stop at 300 | 1.4 / 1.4 | 1.4 / 1.4 | 364 / 365 |
| trot 201.6 | quick stop at 300 | 2.4 / 2.6 | 2.4 / 2.6 | 343 / 328 |
| crawl 22.2 | stop at 300 | 11.9 / 6.3 | 2.9 / 1.4 | 535 / 548 |

No fall in any run, old or new.

Also on the virtual body's own physics (kilo.vbody.physics: the weight
held 5 mm inside three feet, rigid legs; not the judge), a stop at tick 400
(largest error against the prediction, old / new): the crawl 3.2 / 0.5°,
the walk 1.5 / 1.7°, the trot 1.3 / 3.0°. The trot's 3.0° comes as he
stands at the end of the stop; its feet are the old trot's within a
quarter of a millimetre and a tick (the feet now go in quarter
millimetres), and this physics' diagonal rock is sensitive to which feet
are within 0.6 mm of the lowest. The judge (RockingPhysics) has the two
stops equal. One run each; not followed further.

**What it means.**

- **The walk** (new: the weight 3.5 mm inside the three feet, solved
  every tick, in place of the fixed 1° roll and 3 mm fore-aft sway). By
  this judge it rocks about half as much at 40 to 72 mm/s (roll swing 0.6
  to 1.1° against 1.9 to 2.2°), a little less at 23 mm/s, keeps more
  margin (2.5 to 3.5 mm against 0.5 to 1.8), predicts the body as well
  or a little better, and walks as fast (within 1 mm/s; a little faster at
  66 and 72). Worse: its fastest joint is 30 to 45°/s faster at 66 and 72
  mm/s (432 and 459°/s, under the body's 500°/s, nothing cut), and with
  the model wrong its largest error is 0.1 to 0.2° larger at 58 to 72
  mm/s (1.2 to 1.4° against 1.1 to 1.2°; the case is the centre of mass 3
  mm to the right, which the solver cannot know of). Judged at least as
  good: **the solver replaces the walk**. Its weight target was chosen
  here: 3 mm let the walk rock least (0.6 to 0.9°) but its largest error
  with the centre of mass 3 mm right was 2.0 to 2.3°; 4 mm rocked more at
  23 mm/s (1.9°) than the old walk; 3.5 mm is the least that is not worse
  anywhere but those two columns.
- **The trot** has no weight target (two feet carry it); the engine keeps
  its timing (the swing growing with the speed a tick a step) and its
  landing on the fitted pendulum, now as the trot's row of the gait table
  and its landing rule. Equal or better on every column at every speed
  but one: with the model wrong at 207 mm/s its largest error is 3.4°
  against 3.2°, and its quick stop from 201.6 mm/s tilts 2.6° against 2.4°
  (and is done 0.3 s sooner). Judged at least as good: **replaced**.
- **The crawl** keeps the weight 8 mm inside (the old crawl 14 mm, which
  item 2 above found far more than needed): it rocks less than half as much
  (roll swing 8.4 to 9.6° against 19.5 to 21.5°: the old crawl rolls
  the body up to 14° to keep its 14 mm), predicts the body two to three
  times better, tips far less with the model wrong (1.5 to 1.6° against
  4.0 to 6.4°), at the same speed. Its static margin is smaller (7.2 to
  7.3 mm against 13.4 to 13.6), by choice; with 14 mm the engine's crawl
  was slower (16 against 19 to 21 mm/s at 22 to 25) and its joints faster.
  **Replaced**, at 8 mm.
- What simulation cannot show: whether the walk's smaller rock is real on
  the floor. The judge rocks far less than the floor: the recorded walk
  of 2026-11-22 (the old walk) swung 4.9° of roll a cycle at 58 mm/s and
  4.3° at 72 (`walkstudy.py floor`, 3 and 2 cycles), the judge's old walk
  2.1 and 2.2°. So the halving is the judge's; the floor may show less of
  it. Nor can it show whether the faster joints at the walk's top speeds
  keep up, or anything about slipping.
  The floor try (C4 at each level, B7/B5, A4, B4) settles it.

## Step 4: the paw wash judged on the body model (2026-11-23)

**Question.** Is the engine's paw wash (kernel/mind/engine/paw.h: a front
paw lifted on three feet, the face washed with it) safe to play on Kilo,
and does it keep the head clear of the paw (the owner's rule after the
original paw lick bumped his head: at least 10 mm)?

**How** (simulation only; `mind/tests/test_engine_tricks.py`). The trick
as the mind makes it (`engine-judge --trick paw_wash`), on the models the
random mode's library tricks are held to (mind/tests/test_alive_tricks.py):
the rigid quasi-static model on Kilo as designed and as recorded, both
paws, tempos 0.5 to 2; and the tricks' dynamics (kilo.tricks.dynamics: the
servos' lag, the legs' give, friction) with the trick's own guard, in the
fit and at the 19 corners of `dynamics.bands()`.

**Numbers.** The lean the engine chose: the body 7.0 mm back, rolled 5.1°
right side down (the left paw; mirrored for the right); the centre of mass
7.1 mm inside the three feet by the model with the paw up and at the
bottom of a stroke. A first version with 6 mm, a short hold of the lean
and the paw going straight up rocked onto the diagonal for 40 ms as the
paw left the floor (the right hind foot unloaded) on the dynamics; with 7
mm, the lean held until the servos have it (10 ticks) and the paw first
just off the floor (10 mm over 16 ticks), each of the three feet keeps its
load, the right hind's least about 12 g, in the fit and at the corners
tested (the centre of mass 4 mm further forward is the closest).

| check (tempo 1) | left paw, as designed | as recorded | right paw, as designed | as recorded |
|---|---|---|---|---|
| least margin of the centre of mass inside the support, all along | 6.4 mm | 6.7 mm | 6.4 mm | 3.2 mm |
| largest roll | 5.1° | 5.1° | 5.1° | 5.1° |
| largest single topple | 1.2 mm | 1.2 mm | 1.2 mm | 1.2 mm |
| least gap between the head and a front leg (either way the pan turns) | 17.0 mm | 17.0 mm | 16.7 mm | 16.7 mm |
| what touches the floor | the feet | the feet | the feet | the feet |

(The right paw on Kilo as recorded keeps less: the recorded right hind
shoulder sits differently, which the engine's model does not know.) On
the dynamics: no fall and no stop by the guard, the three feet carrying
him the whole time the paw is up (over a second).

**What it means.** By the models he stays on his three feet with room to
spare and the head stays clear of the paw by 17 mm (the paw cannot come
closer to the muzzle than its leg's plane allows with the head straight:
about 16 mm by the model's head, which is an estimate). Not known: whether
Kilo holds the lean as the model says (the self-measurement's 3-foot tips
came only after the mass crossed the edge, results §2), whether a paw at
the cheek looks like washing, and whether the head model is right.
