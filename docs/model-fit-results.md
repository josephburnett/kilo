# The body model, judged by prediction: results

What the recordings from Kilo say the body model's numbers should be, if
the model is to **predict** what his body does. Done 2026-11-22 on the
laptop from the recordings already there; nothing was tried on Kilo.

The owner's test for the model: "If you have a model with specific values
that works for you, that allows you to program movements and *predict*
accurately what is going to happen with the body, then it's good." So the
model is judged here by one thing: replay what Kilo was told to do, and
see how close the model comes to what his tilt sensor read.

Words used below:

- **the score**: the root-mean-square (rms) difference, in degrees,
  between the pitch (nose up or down) and roll (side to side) Kilo's tilt
  sensor read and what the model predicts for the same moment. Smaller is
  better. "0.63 / 1.09" means 0.63 degrees of pitch, 1.09 of roll.
- **the replay**: every run in which the Pi streamed foot positions to the
  body (the walks, trots, crawls and the self-measurement routine): each
  20 ms tick's foot positions, played into the virtual body (a simulated
  Kilo on the laptop) with the corrections his own balance loop reported
  adding, through the servos' delay; the virtual body's tilt plus the
  sensor's offset is the prediction.
- **the judge**: the simulator's physics the replay runs on
  (RockingPhysics: a rigid body on four feet that rocks about a diagonal
  when the weight is not over three of them). It is imperfect
  (docs/righting-results.md); its own two settings were fitted with the
  model's numbers.
- **held out**: recordings the fitting never looked at. The score on them
  is the honest test.
- **+-**: how far the number moves from one boot (one session) to the
  next, divided by the square root of the number of boots: the truth is
  likely within twice it.

**To recompute** (after new floor runs are on the laptop):

```
kilo model score                        # the score of the model as it is, per recording
kilo model score --set com_x=-3         # ...of the model with one number changed
kilo model score --fit com_x,sag        # ...find the numbers that predict the fit set best (minutes)
```

(`mind/kilo/measure/score.py`; its test with a small piece of the
recordings is `mind/tests/test_model_score.py`.)

## The recordings

Every `.krec` in `~/kilo-logs` (each piece of each boot counted once): 42
runs with foot streams, 27 153 scored ticks (9 minutes), boots 2 to 17
(2026-11-17 to 11-22): the old walk, the new walk, the old trot, the long
trot, the crawl, three self-measurement runs. Boot 3 is left out (Kilo was
walked in a hand). The first 25 ticks of each run, and ticks tilted past
20 degrees, are not scored.

- **Fit set** (the fitting may look): boots 2, 6, 8, 11, 13 and the first
  self-measurement run of boot 15; 18 253 ticks.
- **Held out**: boot 14 (old trot, long trot, crawl, walk), boots 16 and
  17 (the new walk and trot of 2026-11-22) and the second self-measurement
  run of boot 15 (Kilo turned round); 8 900 ticks.

## The score, before and after

"Before" is the model as it was this morning, "after" with the numbers
below applied; both on the same judge. "Planner" is what the Pi's walk
planner itself predicted for each tick (it ignores the servos' delay and
assumes he never tips), for comparison.

| | planner, as sent | model before | before + only the IMU offset | **model after** |
|---|---|---|---|---|
| fit set (pitch / roll) | 2.21 / 1.65 | 2.37 / 1.40 | 0.74 / 1.31 | **0.66 / 1.19** |
| **held out** | 2.33 / 1.80 | 2.49 / 1.58 | 0.79 / 1.47 | **0.64 / 1.11** |

The held-out recordings, one by one (pitch / roll, degrees rms):

| recording | what he did | ticks | before | after |
|---|---|---|---|---|
| boot 14, run 1 | walk (old) | 110 | 2.66 / 0.76 | 0.82 / 1.19 |
| boot 14, run 2 | long trot | 993 | 2.07 / 1.90 | 0.64 / 1.29 |
| boot 14, run 3 | crawl, walk | 1437 | 2.20 / 2.03 | 0.61 / 1.32 |
| boot 14, run 4 | trot | 443 | 2.07 / 2.21 | 0.78 / 1.49 |
| boot 15, run 2 | self-measurement, turned | 4093 | 2.76 / 1.09 | 0.60 / 0.88 |
| boot 16, run 1 | walk (new), guard backing off | 136 | 2.30 / 2.06 | 0.52 / 1.24 |
| boot 16, run 2 | walk (new), trot 201 mm/s | 1552 | 2.35 / 1.64 | 0.67 / 1.09 |
| boot 17, run 1 | walk (new) | 136 | 2.41 / 2.32 | 0.89 / 1.81 |

Read:

- Most of the pitch gain is one number, the tilt sensor's offset (2.3
  degrees). The rest of the gain, in both pitch and roll, comes from the
  legs giving under load, the wider track and the centre of mass further
  back, and the judge's two settings.
- The roll gets better in 7 of the 8 held-out runs; the one that gets
  worse (boot 14's short old walk) is 110 ticks.
- The trot's and the crawl's runs gain most in roll; the new walk's least
  (boot 17: 2.32 to 1.81).
- What is left (about 0.6 degrees of pitch, 1.1 of roll) is near what the
  data itself allows: the self-measurement found the same pose held 0.14
  to 0.30 degrees differently each time and approached from two sides 0.6
  to 1.0 degrees differently (play in the joints), which no model here
  has.

## The numbers

Each was fitted by replaying all of the fit set again and again (a search
one number at a time, three rounds), then checked boot by boot: each
boot's own best value. Simulated, not observed on Kilo.

| number | model before | fit set | held out | boot by boot (median, +-) | applied | verdict |
|---|---|---|---|---|---|---|
| tilt sensor offset, nose down (deg) | 0 | 2.32 | 2.33 | 2.32 +- 0.07 | **2.3** | measured (the self-measurement: 2.39) |
| tilt sensor offset, roll (deg) | 0 | 0.50 | 0.67 | 0.63 +- 0.10 | **0.5** | measured (the self-measurement: 0.29; the floors differ) |
| legs' give (deg per kg cm) | 0 | 3.5 | 4.3 | 4.35 +- 0.7 | **4.0** | measured; the rear-up plays need at least about 3.8, the self-measurement 4.3 |
| left hind leg's give (times the others') | 1 | 1.27 | 1.72 | 1.6 +- 0.2 | 1 (unchanged) | probable: softer in 8 of 9 boots; not applied (below) |
| right hind leg's give | 1 | 0.36 | below 0 | 0.3 +- 0.3 | 1 (unchanged) | probable: stiffer; not applied (below) |
| left front leg's give | 1 | 0.1 | no best value | 0.1 +- 0.1 | 1 (unchanged) | **not settled**: the fit set says stiffer, the held-out runs are best at 1.0 to 1.3 |
| right front leg's give | 1 | 0.78 | 0.50 | 0.97 +- 0.1 | 1 (unchanged) | like the others |
| centre of mass forward (mm, correction) | 0 | -1.7 | -3.7 | -1.8 +- 0.6 | **-3.0** | measured: 4.4 mm ahead of the shoulders' middle standing (hand 7.5, self-measurement 4.3); the walks predict as well at -2 as at -3, the rear-up plays need -3 or further back (below) |
| centre of mass to the left (mm) | 0 | +0.3 | +0.6 | +0.4 +- 0.3 | 0 (unchanged) | matches 0 |
| centre of mass height (mm) | 0 | -19 | -15 | -20 +- 5 (spread 14) | 0 (unchanged) | **not measured**: always "lower", but it trades against the legs' give and the judge (below) |
| feet left to right (`LEG_W_MM`, mm) | 65 | 69.9 | 73.1 | 72 +- 1.3 | **70** | measured (the owner's tape: 70) |
| feet front to back (`LEG_LB_MM`, mm) | 123 | 124 | 104 | 126 +- 5 (spread 12) | 123 (unchanged) | **not measured** by these runs (below) |
| servo dead time (ms) | 50 | 56 | 56 | 58 +- 3 | 50 (unchanged) | 55 +- 5 predicts the same within the search's noise: not worth re-timing the walks |
| servo time constant (ms) | 70 | 63 | 72 | 68 | 70 (unchanged) | matches |
| left front servos' extra delay (ms) | 0 | -3 | -6 | -6 (spread 28) | 0 (unchanged) | no sign of a slow left front servo |
| each leg longer than commanded (mm) | 0 | 0, -0.1, +0.2 (LF, RF, RB; LB the reference) | | | 0 (unchanged) | none |
| the judge: weight held by three feet from mm inside | (10 or 3) | 0 | (4) | 0.1 | 0 | the floor holds him as soon as the weight is inside, as docs/movement-engine-results.md §2 found |
| the judge: support lag (ms) | (0 or 50) | 70 | 68 | 68 (spread 23) | 55 | the feet on the ground lag the plan more than the swing does |

The search is noisy: replaying the same model with a number moved by a
hair can change the fit set's total by about 2 %, because a tip a tick
earlier or later changes a whole stretch. Numbers whose gain is smaller
than that are left as they were (the servo timing).

### What it says about Kilo (simulated, from the recordings)

- **His legs give**: about 4 degrees per kg cm of load on a servo, the
  same number the rear-up plays (2026-11-21) and the self-measurement
  found. On four feet that is a soft table: the leg that carries most
  sinks most.
- **Probably, the left hind leg gives more than the right hind** (half
  as soft again against twice as stiff, in the walks' fit). That is the
  self-measurement's "lifting the left front or the right back foot
  leaned the body 1.2 to 2.1 degrees at once" and the rear-up's "the left
  side sits lower when the hind legs carry him". Put into the rear-up's
  model, it predicts the roll boot 17 read at "back" (about +4 degrees; Kilo
  +3.3 and +4.2; with the legs alike the model says +0.5), which the fit
  never saw. **Not applied**, for two reasons: with the other numbers in
  place it hardly improves the walks' score (held out 0.62 / 1.10
  with it, 0.64 / 1.11 without), and with it the rear-up's check predicts
  that today's symmetric stage 1 still tips him left (stopped by its push
  watch or the 8 degree roll limit in every band), so the trick would no
  longer pass its own check. That makes it a trick decision, not a model
  one: the next play of stage 1 (A1) is the test. If he tips left again
  with both hind legs told the same, it is this, and the values are
  `servo_give` 1.5 on joints 11 and 15, 0.5 on 10 and 14
  (`kilo model score --set give_lb=1.5 --set give_rb=0.5` shows them).
- **The left front leg is not shown to be weak.** The walk drifts left
  (docs/walk-trial-results.md) and he drops further onto his left front
  corner; the recordings here cannot tell whether that leg gives more:
  the older boots say it is stiffer than the others, the newer ones say it
  is like them. The drift is a turn (yaw), which the model does not have
  at all, so it cannot be fitted here.
- **The centre of mass is 3 mm further back than the hand measurement**
  (4.4 mm ahead of the shoulders' middle), where three kinds of data meet:
  the walks put it 1.7 (fit set) to 3.7 mm (held out) back, +- 0.6 boot by
  boot, and predict as well at 2 as at 3; the self-measurement said 3.1;
  and the rear-up plays of 2026-11-19..21 rock back onto the tail at
  "gather" in the model only from about 3 mm back (at 2 he rocks half way:
  +15 degrees nose up against Kilo's +23; at 3, +19.6, rocking back at the
  same moment as Kilo did). Before the judge's support lag was in, the
  trots wanted it 4 to 5 mm back and the walks not at all; with it the
  gaits come within about 2 mm of each other.
- **The feet stand about 70 mm apart left to right**, as the tape said,
  not the 65 the model had.
- **The righting kick is not explained by these numbers.** On his back
  (boot 16) the kick turned him 31 to 33 degrees; the simulator on the one
  model says 54 to 69 (on the tricks' old numbers 58 to 74). What decides
  it there is the outline of his back, which has never been measured
  (docs/righting-results.md), not the centre of mass or the legs.

### What the recordings cannot tell apart

- **The tilt sensor mounted 2.3 degrees nose down, or the front legs
  about 5 mm shorter than the back ones.** Every recording reads the same
  either way: the model carries it as the sensor's offset
  (`[model.correction] imu_pitch_deg`), which predicts the readings in both
  cases. A level on the frame settles it (below, 1). Until then the
  offset is NOT sent to the board (it would make the board's balance loop
  stop leaning against it, which is right only if it is the sensor); the
  board keeps leaning about a degree nose up when he stands (docs/
  movement-engine-results.md §3).
- **The height of the centre of mass against the legs' give and the
  judge's tipping.** The fit always wants it lower (10 to 50 mm), but the
  amount swings from boot to boot, and the self-measurement ran into the
  same wall: the model is missing something about how he tips (the
  self-measurement's tips are 2 to 3 times slower than a pendulum), and a
  lower centre of mass is the fit's way of imitating it. Not applied.
- **The feet's front-to-back spacing.** The walks do not care (one
  millimetre of the spacing changes the score by less than the search's
  noise); the self-measurement's pitch stairs prefer 128 to 132 mm, by
  0.02 degrees. The tape's 133 is the feet; 123 is the shoulder axles. If
  the feet stand 5 mm outside the shoulders the cause is the shoulders'
  calibration, and the model should stay at 123 with the feet's x offset;
  if not, the spacing is wrong. Below, 3.
- **The left front leg's give** (above): the two halves of the data
  disagree.
- **The servos' dead time against their time constant**: they trade (55 +
  60 ms predicts like 50 + 70).

## What changed in the model (protocol/spec.toml)

- `[model.correction]`: `com_dx_mm` -3.0, `sag_deg_per_kg_cm` 4.0,
  `imu_pitch_deg` 2.3, `imu_roll_deg` 0.5. The tilt offsets are what the
  sensor reads standing level, for everything in the mind that predicts a
  reading; the board is not told (above).
- `LEG_W_MM` 65 -> 70 (also the body's own parameter: the body's balance
  loop uses it for its roll lever; a body flash takes it, and a body that
  saved its parameters keeps 65 until `kilo param` sets it).
- **The tricks' separate model is gone** (`[model.views.tricks]`: its
  centre of mass 5.5 mm back and 5 mm to the right, its own tilt offsets
  1.9 and 2.0, the tail's droop 16). The tricks' dynamics, the rear-up, the
  intent layer's moves, the righting experiment and the state estimate
  now all read the one model (`kilo.tricks.dynamics.NOMINAL`). Two views
  remain: the trot's fitted pendulum (its centre of mass 3.5 mm is the
  trot's effective one, fitted to its floor rock, and stays), and the
  rigid fit of the falls of 2026-11-17 (`tricks_rigid`, a record of Kilo
  before his right hind shoulder was re-seated).
- The virtual body (`kilo.vbody.physics`) can now give its legs and time
  each leg's servos as the model says (`legs_give`). The walks' virtual
  body in the tests stays rigid; the score's judge turns it on.

## What will look different on the floor

Simulated and reasoned, not seen:

- **The walk**: planned for a centre of mass 3 mm further back and feet 5
  mm wider apart; its sway leans him up to 1.6 degrees instead of 0.9;
  its least margin -3.1 mm instead of -3.5.
- **The crawl**: leans a degree less (12.8 degrees of roll instead of
  13.8): the wider feet need less.
- **The trot**: its foot placement is its own fitted pendulum and does
  not change; the diagonal it rocks on is 5 mm wider in the plan.
- **Standing down to resting** (the intent layer's move): the front
  knees first, then the rest (until today: the front legs, then the hind
  legs). Its centre of mass keeps 8.7 mm inside the feet; the move's bar
  came down from 10 to 8.5 mm (`kilo.intent.moves.MARGIN_MM`), because the
  body's own all-joints-at-once change, which he would use otherwise,
  keeps only 5.8. Resting up to standing: as before (hind legs first),
  8.7 mm. Standing and sitting: as before. Sitting and resting: still not
  shipped (he goes through standing).
- **The rear-up**: the same keyframes and guards; only the pitch the model
  expects at each keyframe moved (0.4 to 1.2 degrees lower: the sensor's
  offset and the centre of mass), which is what `kilo log trick` compares
  against. Its check
  passes in all three stages.
- **The righting experiment (B1)**: the same moves; on the one model they
  no longer reach their own bar in the simulator (from a side onto his
  belly 69 of 95 runs, was 89; from his back 52 of 76, was 68: the
  centre of mass in the middle instead of 5 mm right). B1 still plays
  them as the experiment they are.
- **What the state estimate expects**: standing at 2.3 degrees nose down
  (the sensor's offset) and 0.5 of roll, sitting and resting likewise.
- **The board**: nothing until it is flashed; then its roll lever is 70.

## What the owner could do on the robot to pin down what the recordings cannot

None of this is urgent and none is timed.

1. **Level on the frame.** With Kilo standing (B6) on a floor you know is
   level, put the phone's level app flat on the top of the frame, along
   his length; read it. Turn the phone end for end on the same spot and
   read again; then turn Kilo round and do both again. The mean of the
   four is the frame's own tilt. Frame level: the 2.3 degrees is the
   sensor's mounting, and it can be corrected on the board. Frame nose
   down about 2.3: it is the front legs, a calibration job.
2. **A slow-motion video from the front, at foot height, of one walk
   (C4)**, a metre or two: whether the left front leg bends more than the
   right when it carries him. That separates "left front leg gives" from
   "left front leg is fine and something else turns him".
3. **The feet against the shoulders.** In the calibration posture (C5,
   once), a photo straight from the side with a ruler: whether the upper
   legs hang straight down. If they lean outward (front ones forward,
   back ones back) the feet's 133 mm is calibration; if straight, the
   spacing in the model is wrong.
4. **The height of the centre of mass**: the kitchen-scale tilt test in
   docs/self-measurement-results.md, if you ever have the scale out.
