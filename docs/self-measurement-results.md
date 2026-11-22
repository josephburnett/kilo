# Self-measurement: results

What Kilo measured about himself with the self-measurement routine
(docs/self-measurement.md): the date, the floor, what was seen, what the
fit (`kilo measure fit`) made of it, and what it would mean for the model.

Words used below:

- **the fit**: the laptop program that explains every tilt Kilo's tilt
  sensor read during the routine with a handful of numbers about his body.
- **+-**: how sure the fit says it is (the truth should be within twice
  this in about 19 runs out of 20, if the fit's picture of Kilo is right).
- **IMU offset**: what the tilt sensor reads when the body stands level.
- **held-out check**: eight poses in the routine that the fit is not
  allowed to use; afterwards each model's prediction for them is compared
  with what Kilo really read. It is the honest test of the fit.
- **the walk's margin**: how far inside the triangle of the three feet on
  the ground the centre of mass stays while one foot is in the air.

## 2026-11-22: two runs, the second turned round

**What was done.** Build 4eb35a20 (mind and body), boot 15 of the card.
Flat floor, hands off. The routine twice in one boot; for the second Kilo
was turned to face the other way on the same spot. Battery 8.0 V at the
first start, 7.97 V at the second (7857-8010 mV over the runs).

**The log** (`~/kilo-logs/kilo-card-b15.log`): `measure: done (3959 ticks,
79 s, 6 tips)` and `measure: done (4115 ticks, 82 s, 4 tips)`. Each run
ends with one `FEET_UNDERRUN` (the body ran out of queued foot positions)
40 ms before its end, as on 2026-11-21: the routine's last tick, harmless.
No fall, no watchdog stop during the runs.

Recordings (not in git): `~/kilo-logs/run-b15-32s-trick-1.krec`,
`~/kilo-logs/run-b15-124s-trick-2.krec`. Fits:
`~/kilo-logs/run-b15-32s-trick-1.fit.json`,
`~/kilo-logs/run-b15-124s-trick-2.fit.json`, and both together
(`kilo measure fit RUN1 --turned RUN2`) `~/kilo-logs/run-b15-both.fit.json`.

### The numbers

"Both" is the fit of the two runs together, which also takes the floor's
slope out (a slope tilts the body one way in run 1 and the other way in
run 2; Kilo's own offsets stay the same way round). "Hand" is the owner's
tape and scale (2026-10-15, 2026-11-14). "11-21" is the first run (old
script, boot 13) fitted again with today's fit.

| number | model now | run 1 | run 2 | both | 11-21 | hand | verdict |
|---|---|---|---|---|---|---|---|
| IMU pitch offset, nose down (deg) | 0 | 2.14 +- 0.06 | 2.61 +- 0.07 | **2.39 +- 0.04** | 2.37 +- 0.11 | | measured |
| IMU roll offset (deg) | 0 | 0.37 +- 0.06 | 0.17 +- 0.08 | **0.29 +- 0.04** | | | measured |
| floor slope, nose down / left down (deg) | | | | -0.26 / +0.09 (+- 0.04) | | flat | measured: small |
| centre of mass ahead of the shoulders' middle (mm) | 7.4 | 3.3 +- 1.4 | 7.1 +- 4.3 | **4.3 +- 1.2** | 5.0 +- 7.5 | 7.5 | probably 3 mm further back than the hand measurement |
| centre of mass to the left (mm) | 0 | -0.9 +- 0.7 | +0.6 +- 2.4 | -0.7 +- 0.6 | | 0 | matches 0 |
| centre of mass height above the shoulder axles (mm) | -2.8 | -37 (at limit) | -40 (at limit) | -39 (at limit) | -28 +- 8 | about -3 | **not measured** |
| feet, front to back (mm) | 123 | 132.4 +- 4.8 | 132.3 +- 4.4 | **132.3 +- 3.3** | 132 +- 8 | 133 | measured: 132-133 |
| feet, left to right (mm) | 65 | 72.1 +- 0.2 | 72.5 +- 1.3 | 72.3 +- 0.1 | 73 +- 2 | 70 | **not settled** (below) |
| leg give, "sag" (deg per kg cm) | 0 | 4.6 +- 0.6 | 4.0 +- 0.5 | 4.3 +- 0.4 | 4.3 | | repeatable, but fails the held-out check |
| twist: LF+RB legs longer than RF+LB (mm) | 0 | -0.07 +- 0.11 | +0.07 +- 0.11 | 0.00 +- 0.08 | 0.3 +- 0.1 | | none |
| x scale, foot travel per mm commanded | 1 | 1.12 +- 0.21 | 1.09 +- 0.20 | 1.10 +- 0.14 | | | not resolved |
| servo dead time (ms) | 50 | 74 +- 10 | 75 +- 9 | 75 +- 7 | | | differs, but see below |
| servo time constant (ms) | 70 | 51 +- 12 | 49 +- 11 | 50 +- 8 | | | differs, but see below |
| tip delay, step over the edge to half way down (ms) | 99 (servos only) | 350 +- 180 | 330 +- 250 | 380 +- 130 | | | not resolved: tips are slow |

What the data says by itself (no model needed): coming back to the home
pose 21 times per run he stood within 0.14 deg of pitch and 0.30 of roll
(sd; up to 0.55 and 0.91 apart). The same step reached from above and from
below is held 0.64 deg differently in pitch and 1.04 in roll (hysteresis:
play and friction). So no pose of his is known better than about half a
degree, and the fit's spread for a hold came out at 0.45 deg.

### Measured well enough to use

- **The IMU offset: 2.39 deg nose down, 0.29 deg of roll** (+- 0.04).
  Run 1 read 2.14, run 2 (turned) 2.61: their mean is the robot's, half
  their difference (0.24 deg) is the floor, and the joint fit agrees
  (floor -0.26 nose down, i.e. that spot slopes 0.26 deg up toward where
  his nose pointed in run 1). It is the same 2.4 deg the first run found on
  2026-11-21 and the one docs/movement-engine-results.md §3 saw the body's
  balance loop lean against in every gait. **Confirmed: it is Kilo's, not
  the floor's.** What the tilt sensor cannot say is *which* of two things
  it is: the sensor board mounted 2.4 deg nose down on a level body, or a
  body that really stands 2.4 deg nose down because the front legs come
  out about 5.5 mm shorter than the back ones (2.4 deg over 132 mm). Both
  read the same in every pose (self-measurement.md, "Not measured").
- **The feet are 132 mm front to back**, not 123: both runs, the first run
  and the owner's tape (133) agree, and the held-out poses' pitch is
  predicted better with it (0.42 -> 0.36 deg rms).

### Probably right, not certain

- **The centre of mass is about 4.3 mm ahead of the shoulders' middle**
  (+- 1.2), 3 mm behind the hand measurement of 2026-11-14 (7.5). The two
  runs differ by 3.8 mm (within their +-; run 2 had fewer tips), and the
  first run said 4.3 too. It comes only from the tips (four-feet poses do
  not depend on it), so it rests on 10 crossings, 4 of them clean.

### Not measured, and why

- **The height of the centre of mass.** All three fits run to the fit's
  lower stop (-40 mm, i.e. the centre of mass 37 mm lower than the model).
  It is read from the sideways-leaning tip tests against the flat ones, and
  those tipped only 1 to 2.3 deg of the 5 deg a clean tip falls: the
  lifted foot did not clear well or the legs gave. Even at the stop the
  model cannot put those crossings where they were, so the fit is pushing
  against something the model lacks, not finding a height; and the tip
  speed points the other way (a pendulum 190-440 mm long, i.e. higher).
  The fit used to print the combined fit's -39.0 +- 8.1 as "differs"
  because it was 1 mm from the stop; it now calls a value within one +-
  of its stop "at limit" (fixed in this change, `mind/kilo/measure/fit.py`
  `limits`, tested in `mind/tests/test_measure.py`). A kitchen-scale tilt
  test remains the way to measure the height.
- **The left-right spacing of the feet.** The roll stairs say 72 mm with a
  tiny +-, but the held-out poses roll *more* than even the model's 65 mm
  predicts (they fit best near 60 mm; with 72 their roll error grows from
  0.50 to 0.70 deg rms). Two parts of the same run disagree by 12 mm, and
  the roll hysteresis (1 deg) is the likely cause: the small roll stairs
  are partly swallowed by play, which looks like wider feet. The tape's
  70 mm is the best number there is; the +- of 0.1 mm is not real.
- **The leg give (sag 4.3).** Repeatable (both runs, the first run, and
  the tricks' own fit of 4.0 agree), but in the held-out check it makes the
  pitch prediction worse (0.42 -> 0.69 deg rms): with a 5 mm shorter left
  back leg the model with sag sinks 0.5 deg onto it, Kilo stays on the
  other three. The number is real as "how the statics best soak up the
  play", not as a spring in the legs.
- **Servo timing.** Dead time 75 and time constant 50 ms against the
  model's 50 and 70: the total to half way is about the same (110 against
  99 ms) and the two trade off against each other. These are slow stairs
  of a few mm; the model's numbers were fitted to real walking (2026-11-15)
  and fit the walks to 1.15 deg. Not a reason to change the walks' timing.
- **x scale and tip delay**: too wide to say anything.

**The fit's own verdict, the held-out check** (16 poses over both runs):
the model as it is, given only the measured IMU offset, predicts them to
0.46 deg rms, which is the hold-to-hold spread itself; the fully fitted
model does worse, 0.65 deg. Of the fitted numbers only the 132 mm length
helps (0.43); the track and the sag hurt; the centre of mass does not
matter there.

### Were the tips real?

All ten tip detections (6 in run 1, 4 in run 2, as the log counted) are
real, sudden movements: the tilt jumped 1 to 5 deg within a few ticks at
18 to 48 deg/s, not a drift. But only four were clean tips onto the
lifted foot: the right front and left back tests in both runs (3.4 to 4.8
of the 5 deg a clean tip falls). The left front test and the three
sideways-leaning ones dropped only 1 to 2.3 deg; in run 2 two of those
leaned over slowly instead of jumping, which is the whole difference
between 6 and 4. Lifting the left front or the right back foot leaned the
body 1.2 to 2.1 deg at once, before any sweep, in both runs (the other two
feet: 0 to 0.5): that diagonal holds him less firmly. The tips grow 2 to 3
times slower than a stiff inverted pendulum would (e-folding 140-210 ms
against 66 ms).

### What `kilo model apply --dry-run` would change (both runs' fit)

`kilo model apply ~/kilo-logs/run-b15-both.fit.json --dry-run` (run before
the at-limit fix; the height was refused as implausible then, as "at
limit" now):

| what | now | would be |
|---|---|---|
| centre of mass correction, forward (`com_dx_mm`) | 0 | -3.2 |
| leg give (`sag_deg_per_kg_cm`) | 0 | 4.3 |
| IMU offset (`imu_pitch_deg`, `imu_roll_deg`) | 0, 0 | 2.4, 0.3 |
| servo dead time, time constant (`servo_lag_ms`, `servo_tau_ms`) | 50, 70 | 75, 50 |
| feet front to back (`LEG_LB_MM`, also the body's parameter: a body flash) | 123 | 132 |
| feet left to right (`LEG_W_MM`, the same) | 65 | 72 |
| not applied | | sideways and height of the centre of mass, twist, x scale, tip delay, floor |

Its report: standing centre of mass 7.4 -> 4.2 mm ahead; the trot's view
of the centre of mass (3.5 mm) and the tricks' views of the forward centre
of mass and the sag become "within the measurement's error"; **the
rear-up's check FAILS** (4, 4 and 2 problems in its three stages, against 0
with today's model).

The dry run also showed that `kilo model apply` left the intent layer
stale: the state estimate's table (standing is now expected at 2.4 deg
nose down, sitting at 23.9 instead of 28.4 nose up), its moves (stand,
sit, rest; the righting experiment's) and their vectors are all made from
the model but were not rebuilt, so `kernel/host/test.sh` would have
failed after an apply. Fixed in this change (`mind/kilo/cli_model.py`
`regenerate`).

### What applying would do (simulated, not tried on Kilo)

In a scratch copy with the fit applied as above (`walkstudy.py margins`,
the planner on the model; the virtual body is a simplified robot):

| the walk, forward | margin, model now | if the true centre of mass is 3.1 mm back (the measurement) | after apply |
|---|---|---|---|
| 23 mm/s | -0.1 mm | -1.0 | +0.2 |
| 41 mm/s | +0.4 | -0.5 | +0.4 |
| 58 mm/s (default) | +0.4 | -0.5 | +0.6 |
| 67 mm/s | +0.4 | -0.0 | +0.6 |
| 73 mm/s | +0.6 | +0.0 | +0.9 |

The walk places its feet from the model's centre of mass, so applying
moves the feet with it: if the measurement is right, the real margin goes
from about -0.5 mm (today) to +0.2..+0.9 mm. That is about 1 mm gained,
**still far short of the roughly 3 mm inside three feet the floor
recordings say is needed**: applying the model does not fix the walk's
margin; the walk's own design has to. Backward stays -5.6 mm, turning in
place -2.6. The trot's least margin (four-feet phases) 32.5 -> 36.0 mm,
the crawl's 13.0 unchanged.

The intent layer's moves between postures (`python -m kilo.intent.moves`,
planned and checked on the model): stand to sit and back still ship, with
more margin (31.9 -> 35.4 mm); **stand to rest and rest to stand would no
longer ship** (their closest approach of the centre of mass to the feet's
edge drops from 10.3 to 8.7 and 8.4 mm, under the 10 mm the check asks),
so Kilo would go back to the body's own all-joints-at-once posture change
for those two. Sit to rest and back stayed unshipped.

Tests and generated files the full apply touches (scratch copy, the
mind's tests of the planner, walks, trot, intent, tricks, rear-up,
righting: 28 of 233 fail):

- expected, re-pinned numbers or regenerated files: the masses and the
  stand margin (`test_planner`), the views and `kilo model show`
  (`test_one_model`), the intent table and moves (`test_intent`), the
  rear-up's built keyframes and its photo/tail geometry tests
  (`test_rear_up`), the planner, measure and intent vectors;
- **real regressions** (on the model, simulated): the long trot in the
  virtual body exceeds its error limit (4.8 and 5.1 deg against 4.0 at
  180 and 207 mm/s; the new servo timing and wider feet); the crawl's lean
  no longer keeps the mass inside three feet everywhere
  (`test_the_lean_keeps_the_mass_inside_three_feet[crawl]`); the rear-up's
  stage 1 fails in some uncertainty bands; the righting experiment's
  verdict changes (`test_righting`).

**A partial apply** (only the numbers this write-up calls measured or
probable: the IMU offset, the centre of mass 3.2 mm back, the feet 132 mm
long; not the sag, the track or the servo timing), also in a scratch copy:
the walk's margins are the same as the full apply's (+0.2 to +0.8 mm
forward, -5.2 backward); the rear-up's own check passes (150 runs, 0
problems), but its test of stage 1 finds one uncertainty band (the centre
of mass 3 mm to the left) reaching the roll limit of 6 deg; stand to rest
and rest to stand still stop shipping (margin 9.1 and 8.8 mm: it is the
centre of mass moving back that does it); the long trot in the virtual
body still exceeds its error limit at 207 mm/s (4.7 deg), and the crawl's
lean test still fails. 21 of 231 tests fail, most of them pinned numbers.
So even the partial set is a change to plan for, not a free gain: the
REST moves, the crawl's lean and the trot's checks need re-tuning to the
new centre of mass.

### Recommendation

1. Do not apply this fit as a whole: the held-out check says the fitted
   shape is no better than today's, and the full set breaks the trot, the
   crawl's lean and the rear-up's check in simulation.
2. The IMU offset (2.4 deg) is the one number that is measured beyond
   doubt. Before correcting it, find out which of the two it is: a spirit
   level (or a phone's level app) on a flat part of the frame while he
   stands on a flat floor. Frame level: it is the sensor's mounting; frame
   nose down: it is the legs (their calibration).
   **First reading (2026-11-22, provisional).** The owner put a phone's
   level app on his back while he stood on a level floor: the back about
   1 degree LOWER AT THE REAR (tail end down, nose up), against the IMU's
   2.4 degrees nose DOWN in the same stand. If that holds, the body is not
   leaning nose down at all: the sensor is mounted about 3.4 degrees nose
   down on the frame, and the correction is the sensor's mounting (the
   first route below, `IMU_OFFSETS_SET`), not the legs. The owner doubts
   this one reading (the floor or the phone may have been off by about a
   degree) and will measure again both ways round (the phone turned end
   for end on the same spot, and Kilo turned round), so it does not settle
   the question yet. Nothing is corrected until it does.
   - **The sensor's mounting** belongs on the board, where the attitude is
     made, so that the board's balance loop, the telemetry and the mind
     all see a level body as level. Today there is no board parameter for
     it (none in `protocol/spec.toml`; `docs/calibration.md` covers only
     the joints). The one existing way is the sensor's own offset
     registers (the `IMU_OFFSETS_SET` command, saved in the board's
     EEPROM): about 86 counts of the accelerometer's x offset for 2.4 deg,
     exact enough at the tilts he walks at. Nothing in the `kilo` program
     sends it yet. `[model.correction] imu_pitch_deg` is NOT the place for
     it: only the Python reference (the state estimate's table, the
     tricks) reads it; the Pi's planner and the board never see it, so the
     balance loop would keep leaning. Correct it in one place only.
   - **The legs**: a joint calibration of the front legs against the back
     (`kilo cal`), about 5.5 mm of leg in all.
3. The 132 mm feet length and the 4.3 mm centre of mass are reasonable to
   apply together once 2 is settled, with the owner's go-ahead: they agree
   with the tape and the first run. They gain the walk about 1 mm (if they
   are right) and cost re-tuning: the REST moves, the crawl's lean, the
   trot's virtual-body check (the partial study above).
4. The walk's margin is a design problem, not a model one: no number in
   this measurement brings it near the 3 mm it needs.

### What a further run should do differently

- Lift the foot higher in the sideways-leaning tests (they tipped only a
  quarter to half way): those are the only tests that see the height.
- Approach every stair and check pose from both sides, so the play
  (1 deg of roll) averages out instead of reading as wider feet.
- Measure the height with a kitchen-scale tilt test instead.
- The turned second run worked as designed: keep it.

## 2026-11-21, first run (old script 1, boot 13)

Build mind 0a4c7465. Flat floor; recording
`~/kilo-logs/run-b13-145s-trick-4.krec`. 1759 ticks, 35 s, 6 tips,
battery 7.4-7.5 V, straight after two long-trot walks. It found the same
2.4 deg IMU offset and a centre of mass 3 mm further back; its height ran
into the fit's stop. What it taught, and how the routine changed, is in
docs/self-measurement.md, "What the first run taught"; its numbers fitted
again with today's fit are the "11-21" column above.
