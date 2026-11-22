# Self-measurement: Kilo measures himself (C5 twice)

Press **C5** (column C, row 5 from the bottom: the calibration posture) and
then **C5 again**, with no other key in between, with Kilo on a flat, hard
floor. The first press is the calibration posture, as always; the second
makes him stand up and, once he is still, starts the routine. For about a minute and a quarter he leans in small stairs, twists,
spreads his feet, and lifts one foot at a time a few millimetres while he
slides his body over the others; then he stands. Afterwards, on the
laptop, `kilo measure fit` (from the card) or `kilo measure fetch` (over
the network) reads what he felt and works out a few numbers about his
body, and says for each whether it **matches** the model the walks use,
**differs** from it, or was **not measured**. **Nothing changes by
itself**: the numbers go into a file and a table, and you decide whether
any of them replaces a number in the model.

This is the second design of the routine ("script 2"). The first
(2026-11-21) took 40 seconds; what its one run on Kilo showed, and what
was changed because of it, is at the end.

What you need to do, in short:

1. A flat, hard floor (a table top on the floor, a board, tiles; not a
   rug, not a slope), with about **30 cm clear** around him. He moves his
   body a few centimetres and does not walk.
2. Battery charged (he refuses below 7.4 V, as for the rear-up).
3. Press **C5** (he takes the calibration posture), then **C5** again
   whenever you are ready: he stands, and once he has been still for about
   a second and a half the routine starts by itself. Any other key pressed
   between the two C5s makes the next C5 a first press again (the
   calibration posture). (B1 started it until 2026-11-22.)
4. Watch. It is longer than before: **75 to 90 seconds**. Stop him
   whenever you like (below). Picking him up is fine: he stops and stands.
   **One thing to note while you watch**: each time a foot comes up
   (twelve times: the order is in the table below), does it visibly leave
   the floor? "Yes", "no" or "don't know" per foot is a real help: a foot
   that does not clear is the one thing the recording cannot tell from
   legs that give.
5. On the laptop, either:
   - by the card: switch Kilo off, card into the laptop, then
     `kilo card CARD --fetch` and `kilo measure fit FILE` with the
     `run-...krec` it names for the measurement (it needs saving on: a
     kernel built with `KILO_RECORD_SAVE=1`, or `kilo record save on`);
   - over the network: `kilo measure fetch --host kilo.local`.
   Write what you saw and the table it prints in
   `docs/self-measurement-results.md`.

No step is timed: he waits for the second C5, and the laptop waits for you.

## Words used here

- **Centre of mass**: the point where his weight acts, as if all 358 g
  sat there. Where it is decides when he tips. Given in millimetres in his
  own frame: `com_x` forward from the middle between the four shoulder
  axles, `com_y` to his left, `com_z` up from the shoulder axles.
- **IMU**: the motion sensor on the board, which says how he is tilted.
  **Pitch** is nose down or up, **roll** leaning to the left or right.
  It reads in steps of 1/16 degree; standing still it varies by about 0.03
  degrees.
- **IMU offset**: what the IMU reads when the body is level. It is never
  exactly zero: the board sits a little crooked.
- **The feet's rectangle** (`length`, `track`): how far apart the four
  feet stand, front to back and left to right, with the feet under the
  shoulders. It sets how many degrees he tilts when the front legs are
  made a millimetre shorter than the back ones. The model has 123 by 65
  mm; your tape (2026-10-15) said 133 by 70, and the first run agreed
  with the tape.
- **Servo dead time** and **time constant**: after a leg is told to move,
  nothing happens for the dead time (about 50 ms in the model), then the
  leg gets there like a lagging pointer: 63 % of the way after one time
  constant (about 70 ms in the model).
- **Sag**: the servos give a little under load, like stiff springs.
  Measured in degrees per kg cm (how many degrees a servo gives for each
  kilogram-centimetre of twisting load). The walks' model takes the legs as
  rigid (0). The first run says about 4.
- **Twist**: one diagonal pair of legs (left front and right back) a little
  longer than the other pair, like a table with one short leg.
- **Tripod**: standing on three feet with the fourth in the air.
- **Lean**: lifting a foot makes the tripod left behind settle a degree or
  two toward it by itself (the sag, the twist). That is not a tip.
- **Crossing**: during a sweep (below), the place where the centre of mass
  passes over the line between two diagonal feet, so that he tips onto the
  lifted foot.
- **Repeatability**: how differently he stands each time he comes back to
  the very same pose. The first run: up to half a degree. Nothing about a
  pose can be known better than that, however fine the IMU reads.
- **Hysteresis**: the same pose held a little differently when he comes
  to it from above than from below (play in the servos).
- **Held-out check**: six poses near the end that the fit is NOT allowed
  to use. Afterwards it has to predict them. How well today's model and
  the fitted one each do is the plain test of whether the fit learned
  something true.
- **+-**: how sure the fit is: one standard deviation. A number more than
  two +- away from the model's is marked **significant**.

## What he does, in order

All of it streams from the Pi to the body 50 times a second as foot
positions (the same way he walks), with the body's own balance
correction switched off, so that what the IMU reads is his body's own
answer to each pose. He starts from his walking stance (the legs at 76
mm) and comes back to it, and stands there a third of a second, after
each part.

| # | Part | What moves | How long | What it tells |
|---|---|---|---|---|
| 1 | level | nothing: he stands | 1.5 s | the IMU offset (with the floor's slope) |
| 2 | stairs_pitch | the front legs shorter and the back longer in stairs of 2 mm, to 6 mm (nose down 5.6 degrees), a jump back; the same nose up; then both again coming down the stairs. Each stair held half a second | 9 s | the rectangle's length (degrees per millimetre); the hysteresis (each stair is reached once from below and once from above); and in the 6 mm jumps the servos' dead time and time constant |
| 3 | stairs_roll | the same left and right, stairs of 2 mm to 4 mm (7 degrees) | 7 s | the rectangle's track; the hysteresis in roll |
| 4 | twist | his body 20 mm forward; then the left front and right back legs 1 and 2 mm longer and the other two as much shorter, and the other way | 6 s | the twist: his nose dips either way he is twisted, least where his own built-in twist is exactly undone; the sag |
| 5 | splay | nose down 5.6 degrees; the front feet 16 mm forward and the back feet 16 mm back, then 14 mm inward, twice | 5 s | whether a millimetre of foot travel forward-back is as long as a millimetre up-down (a check, to about 10 %) |
| 6-9 | edge_lf, edge_rf, edge_lb, edge_rb | one foot lifted 5 mm and held half a second; then all four feet slide slowly under him (8 mm a second), so that his body moves forward or back over them, until he tips onto the lifted foot; 3 mm more, the foot down, back to the stance. Left front, right front, left back, right back | about 5 s each | the lean of each tripod; where the centre of mass is, forward and sideways |
| 10-11 | edge_lf_away, edge_lf_toward | the left front foot again, with his body leaning 7 degrees to the right (away from that foot), then 5.3 degrees to the left (toward it) | about 4.5 s each | how high the centre of mass is |
| 12 | edge_rf_away | the right front foot, leaning 7 degrees to the left | about 4.5 s | the height, checked on another foot |
| 13 | edge_lb_toward | the left back foot, leaning 5.3 degrees to the left | about 4.5 s | the height, checked on a back foot |
| 14 | check | six poses held half a second each: nose down and leaning; nose up and leaning the other way; forward and leaning; back and nose down; the left back foot up; the right front foot up while leaning | 7 s | nothing: the fit does not use them. It is tested on them |
| 15 | level_end | nothing | 1 s | whether the IMU drifted |

So a foot comes up twelve times: left front, right front, left back,
right back; left front twice more; right front; left back; and in the
check the left back and the right front.

He stands when it is done. With the model right it takes 76 s; a sweep
that never tips goes to its end, and the whole is at most 89 s, plus
about 1.5 s to start and half a second to stand up at the end.

**The tip.** When he tips, he falls only as far as the lifted foot: 5 mm,
about 5 degrees. It looks like a small stumble onto that foot. That is
the measurement. The routine sees it and ends that sweep soon after. It
tells a tip from a lean by the jump: a tip is the tilt toward the lifted
foot 1.5 degrees above where it was at the end of the half second with
the foot up, having risen a whole degree within the last 0.16 s. A lean
that creeps does not end the sweep. (If he is already 3 degrees over
while the foot is held up, he tipped as it came up; the sweep is then
only 3 mm.)

## How to stop him

- **Any key** on the remote stops it at once and he stands. C5 again is
  the usual way (it only stops it; it does not take the calibration
  posture). B6 (stand), A7 (sleep) and C7 (freeze) then also do their
  own job; the other keys only stop it.
- **Picking him up** is fine: the tilt goes far from what the model
  expects (or past 25 degrees) and he stops and stands. Put him back down
  on his feet.
- If he falls (the body's fall reflex), the routine stops at once.
- A laptop taking the body (`kilo posture --host ...`) stops it too.
- He also stops by himself if the body leaves the stream, the link drops,
  or the Pi's loop stalls.

After the second C5 he stands (the routine starts from standing), then
the routine is asked to start every moment from 1.5 s on. It will not start
(he stays standing, nothing else moves) when: the battery is under 7.4 V
at rest; he is on USB power; he is still tilted more than 10 degrees, or
moving more than 3 degrees within a second, 6 s after the second C5
(settling, in a hand, on a slope). A key pressed while he settles cancels
it. While the behaviour runner (A5) runs, C5 is ignored: B6 first. The
remote's log line says why (`kilo log --host ...`): `remote C5 (0x90,
calibration posture): standing, measure refused: ...`.

From the calibration posture to standing is the body's own posture change,
the same as pressing B6 there; the routine's first part (75 ticks of
holding level at the walking stance) only starts once he stands still, as
when B1 started it from standing.

## Safety, in numbers

- Every pose keeps his centre of mass at least **10 mm inside** the
  rectangle of his four feet, even if the centre of mass is anywhere within
  10 mm forward or back, 7 mm to either side and 15 mm up or down of where
  the model puts it (checked for every tick of the routine:
  `kernel/host/tests/test_measure.cpp`; the tightest is 10.2 mm).
- The largest tilt any pose can give, with the centre of mass anywhere in
  those bands, is **under 10 degrees** (9.6: 5.3 degrees of lean plus a
  5 mm tip). The body's fall reflex is at 75 degrees.
- Every foot position is inside the legs' reach and every joint at least 3
  degrees inside its limits and inside the servo pulse range of Kilo's
  calibration (`mind/tests/test_measure.py`). The right hind shoulder
  stops at -57 degrees in the calibration of 2026-10-10: that is why the
  right back foot's sweep starts further back, and why the leaning sweeps
  are the left front (both ways), the right front (away) and the left
  back (toward) and not their mirror images. All four feet slide
  together, and with the right side down the right hind shoulder cannot
  follow the others far enough forward.
- The routine stops itself if the tilt passes 25 degrees, or is 10
  degrees away from what the model expects.

## Afterwards: the numbers

```
kilo measure fit ~/kilo-logs/run-...krec          (from the card)
kilo measure fetch --host kilo.local              (over the network)
```

It fits the run and prints a table like this (from a run in the virtual
body, whose centre of mass was put 4 mm back and 3 mm to the right and
its IMU 1.5 and -2 degrees off; the fit found them):

```
parameter  unit          model   fitted     +-  significant  what
imu_pitch  deg            0.00     1.50   0.00  yes          IMU pitch offset (+ floor slope, + front/back legs)
imu_roll   deg            0.00    -2.00   0.00  yes          IMU roll offset (+ floor slope, + left/right legs)
com_x      mm             7.43     3.43   0.28  yes          centre of mass, ahead of the shoulders' middle
com_y      mm             0.00    -3.00   0.15  yes          centre of mass, to the left
com_z      mm            -2.83    -3.12   2.02  no           centre of mass, above the shoulder axles
length     mm           123.00   123.20   0.27  no           the feet's rectangle, front to back (pitch per mm of leg)
track      mm            65.00    64.95   0.08  no           the feet's rectangle, left to right (roll per mm of leg)
x_scale    mm/mm          1.00     1.00   0.00  fixed        a foot's travel per mm of x commanded (against the legs' z)
sag        deg/kg cm      0.00     0.03   0.04  no           leg servos' give under load
twist      mm             0.00    -0.00   0.01  no           LF+RB legs longer than RF+LB (each pair by half)
lag_ms     ms            50.00    52.26   0.45  yes          servo dead time (with the IMU's own delay)
tau_ms     ms            70.00    73.67   0.58  yes          servo time constant
tip_ms     ms            98.52   390.74  35.51  yes          a tip's delay: the step over the edge to half way up the tip

against the model (protocol/spec.toml); the fit changes none of it:
  com_x      [model] stand_com_x_mm, *_mass   differs: 7.43 -> 3.43 +- 0.28
  com_z      [model] *_mass (z)               matches (-2.83; measured -3.12 +- 2.02)
  length     LEG_LB_MM                        matches (123.00; measured 123.20 +- 0.27)
  x_scale    LEG_L1_MM, LEG_L2_MM (the IK)    matches: the fit keeps 1.00; the splay alone says 1.00 +- 0.02
  ...
```

Under the table:

- **against the model**: for every number, where the model keeps it (a
  name in `protocol/spec.toml`) and the verdict: **matches** (within two
  +- of the model's), **differs: old -> new**, **not resolved** (within
  two +-, but the +- is too wide to be worth anything), **not measured**
  (the fit ran into the end of the range it may search, as the height did
  on the first run, or the sweeps that would tell it did not tip), or
  **new** for a number the model does not have (the IMU offsets, the sag,
  the twist).
- **repeatability** and **hysteresis**: how differently he stood on
  coming back to the same pose (words, above). The +- already allow for
  it.
- one line per sweep: where he crossed (measured, the model's, the fitted
  model's), how far the tripod **leaned as the foot came up**, and **how
  far he tipped** of the 5 degrees the lifted foot allows. A tip well
  short of that, or a sweep that says "a slow lean, not a tip", is where
  your note about the foot leaving the floor matters.
- **held-out check**: for each of the six poses the measured tilt,
  today's model's and the fitted model's, and the error of each over all
  six. If the fitted model is not clearly closer than today's, the fit
  has learned nothing worth putting in the model, whatever its table
  says.
- **note** lines: cautions, such as the height resting on one sweep only.

The same numbers, with their uncertainties and how they depend on each
other, go to a file next to the recording (`run-....fit.json`).
`kilo measure fit FILE` fits a run again; several files fit together.
Recordings of the first design still fit (the file says which script it
played).

- **Turned round.** The floor's slope looks to the IMU exactly like an
  IMU offset. Run it twice on the same spot, the second time with Kilo
  turned round (180 degrees), and fit both: `kilo measure fit FIRST.krec
  --turned SECOND.krec`. The slope then comes out on its own
  (`floor_pitch`, `floor_roll`) and the IMU offsets are the IMU's.
- **Walks.** `--walk WALK.krec` adds walk recordings: their error stream
  (what the IMU read against what the planner predicted, ticks on three or
  four feet) times the same servos under walking loads. They add to the
  dead time and time constant, nothing else: a walk's average error mixes
  the IMU offset with the planner's own lean, so it says nothing about the
  centre of mass.

## Why each part tells what it does

**Level.** Standing still on four feet, what the IMU reads is its own
offset, plus the floor's slope, plus any difference in length between the
front and back legs (or the left and right ones): those three look the
same to it.

**Stairs.** Making the front legs 2 mm shorter and the back 2 mm longer
tilts him by 4 mm over the distance between his front and back feet: the
angle is the rectangle's length, read off directly. Three stairs each way
make a line, whose slope is the length (10 mm more length is 0.4 degrees
less at the top stair) and whose bend, if any, is the legs giving more as
the weight shifts. Coming back down the same stairs shows the hysteresis.
The jump over the whole height at once, and watching the tilt follow,
gives the servos' timing: the tilt starts late by the dead time and
closes in at the time constant's pace (one millisecond more dead time
changes the reading by 0.06 degrees at the steepest part).

**Twist.** With his body forward the front feet carry him, and he stands
on both front feet and one back foot: which back foot depends on which
diagonal is longer. Making one diagonal longer in steps tips his nose
down a little either way, so the pitch against the steps is a V. Its
point is where the commanded twist exactly undoes his own: half a
millimetre of built-in twist moves it by half a millimetre, 0.4 degrees
at a step. Legs that give round the V off, which is how the sag shows
here.

**Splay.** Nose down, the tilt depends on how far apart front and back
feet stand. Moving the front feet forward and the back feet back makes
that distance 32 mm longer by the commands; if the feet really travel
10 % more or less, the tilt is 0.3 degrees off. That is small against how
he repeats, so this is a check to about 10 % and no better. The fit does
not use the splay for anything else, and keeps the scale at 1: fitted
together with the rest, a 10 % doubt about the scale would look like a
10 mm doubt about the height.

**Lifting a foot.** With one foot up he stands on the other three, and
they take the lifted foot's share: legs that give let the body settle
toward the lifted foot, and a longer diagonal settles differently from a
shorter one. Half a second of holding still with the foot up reads that
lean, on all four feet in turn. It is the best view of the sag and
the twist, and the first design threw it away.

**Sweeps (edges).** The line between the two feet of the other diagonal
is the edge he can tip over. Sliding the feet under him moves his body,
and so his centre of mass, toward that edge; the moment it passes over
it, he tips onto the lifted foot. Where the feet were then tells where
the centre of mass is. The two diagonals cross, so the four sweeps
between them pin down both forward-back and side to side (a 1 mm error
sideways moves a crossing by 2 mm, forward-back by 1 mm). Each diagonal
is swept from both sides, which also cancels the delay between the step
that crosses the edge and the tip. For where his mass was over his feet
at that moment the fit takes the tilt the IMU read just before the tip,
not a model's guess of it.

**Leaning sweeps.** Leaning the body 7 degrees moves a high centre of
mass further sideways than a low one. So the same foot's crossing, done
leaning, moves by an amount that depends on the height: 0.25 mm per mm of
height leaning away from the lifted foot, 0.19 mm the other way leaning
toward it. The first design leaned over two different feet and had
nothing to compare each with; now each leaning sweep stands against the
flat sweep of the same foot, and the left front foot is done leaning
both ways (0.44 mm between them per mm of height).

**Tip rate** (your idea: "watching the rate of tipping"). Once he tips, the
tilt grows like an inverted pendulum's, faster for a lower centre of mass.
The routine records every tip and the fit reports how fast each grew (its
e-folding time: how long the tilt takes to grow 2.7 times). **It is
reported, not used for the height**: in the virtual body, where the answer
is known (a point mass, 90 ms), it reads 68 ms, because a 5 mm tip is over
in about 0.15 s, before it is a clean pendulum, and the feet are still
moving. The leaning sweeps give the height instead.

## What it can and cannot measure

The fit was checked in simulation (`kilo measure simulate`,
`mind/kilo/measure/simulate.py`): runs made from Kilo's model with the
numbers changed at random (centre of mass up to 6 mm off forward-back, 4
sideways, 10 up or down; the rectangle 4 mm shorter to 12 mm longer and 2
narrower to 7 wider; sag 0.5 to 4; twist up to 1 mm; IMU offsets up to 3
degrees; dead time 30 to 80 ms; time constant 40 to 110 ms), the IMU's
noise and 1/16 degree steps added, the routine's own early end of each
sweep, then fitted. And once on the virtual body's physics, a different
model (rigid legs, a real tip): the table above.

24 runs each way. "Error" is the fit's miss (root mean square); "+-"
what the fit said on average; "within": how often the truth was inside
2 +-. "Backlash" is what Kilo's servos do: every hold comes to rest a
little differently, 0.3 degrees at random (the first run's returns to the
home pose differed by 0.2 degrees in pitch and 0.5 in roll).

| | IMU noise only | | | with 0.3 degrees of backlash | | |
|---|---|---|---|---|---|---|
| number | error | +- said | within | error | +- said | within |
| IMU pitch, roll offset | 0.004 deg | 0.003 | 92-96 % | 0.05 deg | 0.04 | 92-100 % |
| centre of mass forward-back (com_x) | 0.3 mm | 0.2 | 88 % | 0.8 mm | 0.6 | 96 % |
| centre of mass sideways (com_y) | 0.2 mm | 0.1 | 92 % | 0.5 mm | 0.5 | 92 % |
| centre of mass height (com_z) | 1.7 mm | 1.5 | 88 % | 6.2 mm | 5.9 | 100 % |
| rectangle, length | 0.1 mm | 0.1 | 92 % | 2.8 mm | 3.1 | 92 % |
| rectangle, track | 0.05 mm | 0.06 | 96 % | 0.6 mm | 0.9 | 92 % |
| x scale (the splay alone) | 0.02 | 0.04 | 100 % | 0.13 | 0.14 | 100 % |
| sag | 0.05 | 0.01 | 71 % | 0.2 | 0.3 | 100 % |
| twist | 0.006 mm | 0.003 | 92 % | 0.05 mm | 0.06 | 92 % |
| dead time | 0.2 ms | 0.2 | 88 % | 2.2 ms | 7 | 100 % |
| time constant | 0.3 ms | 0.3 | 100 % | 3.4 ms | 9 | 100 % |

So, with Kilo's servos as they are: expect the centre of mass to under a
millimetre forward-back and half a millimetre sideways; the rectangle's
length to 3 mm and its track to 1 mm; the twist to a twentieth of a
millimetre; the sag and the servo timing well; and the **height of the
centre of mass only to about 6 mm**. That is a weak handle and stays one:
a crossing moves a quarter of a millimetre per millimetre of height, and
he does not repeat a pose to better than a third of a degree. If the
height matters more than that, a kitchen-scale tilt test (as on
2026-10-15) is the better tool. The +- are honest with backlash (the
truth inside 2 +- in 92 to 100 % of runs); without it the sag's is too
small.

Assumed, not measured:

- **A millimetre is a millimetre.** The IMU reads only angles, so one
  length must be taken as right: the legs' (a millimetre of leg commanded
  is a millimetre). The rectangle is measured against that. The splay
  checks the forward-back scale against it, to 10 %.
- **The total mass** (358 g, your scale).

Not measured, and why:

- **The legs' other length differences.** All four longer is only a
  height; the front pair longer than the back looks exactly like an IMU
  pitch offset, the left pair like a roll offset. They are inside the
  IMU offsets.
- **The tail's mass and droop.** Swinging the tail moves the centre of
  mass under a millimetre: on four feet that changes the tilt by less than
  0.01 degrees. Its droop matters only when the tail touches the floor.
  Left out.
- **A body shift on four feet** (the body 20 mm forward and back): the
  sag tilts him only 0.18 degrees between the two (sag 3), under three of
  the IMU's steps; the tripods and the twist show the sag better. Left
  out. **Raising and lowering** the body tilts nothing at all. Left out.
- **Each leg's own servo timing**: one leg's step on four feet rocks the
  body onto a diagonal and mixes timing with the rocking; the pitch and
  roll stairs time the same servos cleanly. One dead time and one time
  constant for all.
- **Inertia** (how the mass is spread, not just where its centre is): the
  tips are too short to tell (the tip rate, above).
- **The mirror-image leaning sweeps** (right front toward, left back
  away, anything leaning on the right back): the right hind shoulder's
  reach (Safety, above).
- **Soft legs that do not clear.** If the legs give much more than the
  first run says (sag well over 4) or one diagonal is much longer, a 5 mm
  lift may leave too little for a clean tip: he leans onto the foot
  slowly. The fit then says "a slow lean, not a tip" for that sweep, uses
  no crossing from it (its lean is still used), and with fewer crossings
  the centre of mass is less sure (its +- grows, as it should; the height
  is "not measured" if no leaning sweep tipped together with its flat
  one).

## What is recorded

The run is in the flight recorder like a walk or a trick: `kilo runs
--host ...` lists it as `self-measurement (C5 twice)`, `kilo runs --fetch`,
`kilo card CARD --fetch` and `kilo measure fetch` get it. In it: which
script it played and its tip rule (`measure ... script=2 ...`), every
foot position sent and the attitude the walks' (rigid) model predicts for
it, every IMU reading (pitch, roll, their rates) and the battery, a mark
at the start of every part (`step i=.. exp=.. phase=..`), for each sweep
the tilt with all four feet down and the tilt with the foot held up
(`base`), each tip the routine saw (`tip`), the keys pressed, and how it
ended (`end reason=DONE|STOPPED|FELL|ABORTED`).

## Where it lives

- The routine on the Pi: `kernel/mind/measure/` (`script.h`: the parts and
  their poses; `measure.h`: starting, streaming, the tip, stopping), C5
  twice in `kernel/mind/remote/`.
- The same script in Python (and the first design's, for its recordings),
  the tip rule's reference, the statics (how he stands on his feet with
  springy legs), the fit and the simulation: `mind/kilo/measure/`.
- The command: `kilo measure plan|fit|fetch|simulate` (`mind/kilo/cli_measure.py`).
- Tests: `kernel/host/tests/test_measure.cpp`, `mind/tests/test_measure.py`,
  and the whole loop (the routine in `kilo-mind-host` against the virtual
  body, fetched and fitted) in `kernel/host/test.sh`.

## What the first run taught (2026-11-21), and what changed

The first design's one run on Kilo (`docs/self-measurement-results.md`)
fitted, and was wrong in ways its own table did not show:

1. **A lean was taken for a tip.** Lifting the left front or the right
   back foot leaned the tripod 1.3 to 1.8 degrees at once. The routine
   measured the tip from how he stood before the lift and called 2
   degrees a tip, so three of six sweeps ended before he reached the
   edge. Now: the tip is measured from the half second with the foot up,
   and must be a jump (The tip, above).
2. **The lean itself was thrown away.** The fit dropped the sweeps that
   did not tip, lift and all, and with them the only data that showed the
   twist; it reported a twist of 0.1 mm. Now every lifted hold is used,
   and the twist has a part of its own.
3. **The height ran into its limit** (-40 mm, the end of the range the
   fit may search) and was printed as a measurement. It came from one
   leaning sweep with nothing to compare it with. Now: four leaning
   sweeps, each against the flat sweep of the same foot; a value at a
   limit is printed as "not measured"; and the height is honest about how
   little it knows (table above).
4. **The model's rectangle is too small.** He tilted 5.1 to 5.3 degrees
   for a step the model says is 5.6, and 6.5 to 6.8 for one it says is
   7.0: feet 133 by about 69 mm apart, as your tape said, not 123 by 65.
   The fit had no number for that and bent the sag instead. Now the
   stairs measure it.
5. **He does not repeat.** Coming back to the same pose he stood up to
   0.6 degrees differently in pitch and 1.5 in roll, and the fit's +-
   (0.08 degrees) knew nothing of it. Now the returns are measured, a
   hold counts as one observation however many ticks it lasts, and the
   check poses test the result.

The first run's recording, fitted again with today's fit, gives: track
73 +- 2 mm (differs from the model's 65), length 132 +- 8 (not resolved),
twist 0.3 +- 0.1 mm, sag 4.3, and a height that still comes out low
(-28 +- 8 mm) from its single leaning sweep, whose tip went less than
half way: a hint at most.
