# Self-measurement: Kilo measures himself (B1)

Press **B1** (column B, bottom row) with Kilo standing on a flat, hard
floor. For about 40 seconds he leans, steps his body about and lifts one
foot at a time a few millimetres; then he stands. Afterwards, on the
laptop, `kilo measure fetch` downloads what he felt and works out a few
numbers about his body. **Nothing changes by itself**: the numbers go into
a file and a table, and you decide whether any of them replaces a number
in the model the walks use.

What you need to do, in short:

1. A flat, hard floor (a table top on the floor, a board, tiles; not a
   rug, not a slope), with about **30 cm clear** around him. He moves his
   body a few centimetres and does not walk.
2. Battery charged (he refuses below 7.4 V, as for the rear-up).
3. Press **B6**, let him stand still for two seconds, press **B1**.
4. Watch. Stop him whenever you like (below). Picking him up is fine: he
   stops and stands.
5. On the laptop: `kilo measure fetch --host kilo.local` (or `--host IP`).
   Write what you saw and the table it prints in
   `docs/self-measurement-results.md`.

No step is timed: he waits for B1, and the laptop waits for you.

## Words used here

- **Centre of mass**: the point where his weight acts, as if all 358 g
  sat there. Where it is decides when he tips. Given in millimetres in his
  own frame: `com_x` forward from the middle between the four shoulder
  axles, `com_y` to his left, `com_z` up from the shoulder axles.
- **IMU**: the motion sensor on the board, which says how he is tilted.
  **Pitch** is nose down or up, **roll** leaning to the left or right.
  It reads in steps of 1/16 degree; standing still it varies by about 0.03
  degrees (today's recordings).
- **IMU offset**: what the IMU reads when the body is level. It is never
  exactly zero: the board sits a little crooked.
- **Servo dead time** and **time constant**: after a leg is told to move,
  nothing happens for the dead time (about 50 ms in the model), then the
  leg gets there like a lagging pointer: 63 % of the way after one time
  constant (about 70 ms in the model).
- **Sag**: the servos give a little under load, like stiff springs.
  Measured in degrees per kg cm (how many degrees a servo gives for each
  kilogram-centimetre of twisting load). The walks' model takes the legs as
  rigid (0). The by-hand look at the rear-up recordings found about 3.
- **Twist**: one diagonal pair of legs (left front and right back) a little
  longer than the other pair, like a table with one short leg.
- **Tripod**: standing on three feet with the fourth in the air.
- **Crossing**: during a sweep (below), the place where the centre of mass
  passes over the line between two diagonal feet, so that he tips onto the
  lifted foot.
- **+-**: how sure the fit is: one standard deviation. A number more than
  two +- away from the model's is marked **significant**.

## What he does, in order

All of it streams from the Pi to the body 50 times a second as foot
positions (the same way he walks), with the body's own balance
correction switched off, so that what the IMU reads is his body's own
answer to each pose. He starts from his walking stance (the legs at 76
mm) and comes back to it after each part.

| # | Part | What moves | How long | What it tells |
|---|---|---|---|---|
| 1 | level | nothing: he stands | 1.5 s | the IMU offset (with the floor's slope) |
| 2 | steps | the front legs 6 mm shorter and the back 6 mm longer (nose down 5.6 degrees), then the other way; the left legs 4 mm shorter and the right 4 mm longer (lean 7 degrees), then the other way; each a quick move held 0.8 s | 6.4 s | the servos' dead time and time constant (how late the tilt follows), and the sag (how much less or more than the geometry he tilts) |
| 3-6 | edge_lf, edge_rf, edge_lb, edge_rb | one foot lifted 5 mm; then all four feet slide slowly under him (8 mm a second), so that his body moves forward or back over them, until he tips onto the lifted foot; 3 mm more, the foot down, back to the stance | about 5 s each | where the centre of mass is, forward and sideways |
| 7 | edge_rf_roll | the same with his body leaning 7 degrees to the left, away from the lifted right front foot | about 4.5 s | how high the centre of mass is |
| 8 | edge_rb_roll | the same leaning 5.3 degrees to the right, toward the lifted right back foot | about 4.5 s | how high the centre of mass is, checked the other way |
| 9 | level_end | nothing | 1 s | whether the IMU drifted |

He stands when it is done. In simulation, with the model right, it takes
39 s; if a sweep never tips (the centre of mass far from where the model
puts it) that sweep goes to its end and the whole is at most 49 s, plus
about 1.5 s to start and half a second to stand up at the end.

**The tip.** When he tips, he falls only as far as the lifted foot: 5 mm,
about 5 degrees. It looks like a small stumble onto that foot. That is
the measurement. The routine sees it and ends that sweep soon after.

## How to stop him

- **Any key** on the remote stops it at once and he stands. B1 again is
  the usual way. B6 (stand), A7 (sleep) and C7 (freeze) then also do their
  own job; the other keys only stop it.
- **Picking him up** is fine: the tilt goes far from what the model
  expects (or past 25 degrees) and he stops and stands. Put him back down
  on his feet.
- If he falls (the body's fall reflex), the routine stops at once.
- A laptop taking the body (`kilo posture --host ...`) stops it too.
- He also stops by himself if the body leaves the stream, the link drops,
  or the Pi's loop stalls.

He will not start (and nothing moves) when: the battery is under 7.4 V at
rest; he is on USB power; he is not standing (B6 first); he is tilted more
than 10 degrees, or has moved more than 3 degrees in the last second
(still settling, in a hand, on a slope). While he walks, B1 stops the walk
first and then usually refuses (he has not stood still for a second yet):
press it again. While the behaviour runner (A5) runs, B1 is ignored: B6
first. The remote's log line says why
(`kilo log --host ...`): `remote B1 (0x4A, measure): measure refused: ...`.

## Safety, in numbers

- Every pose keeps his centre of mass at least **10 mm inside** the
  rectangle of his four feet, even if the centre of mass is anywhere within
  10 mm forward or back, 7 mm to either side and 15 mm up or down of where
  the model puts it (checked for every tick of the routine:
  `kernel/host/tests/test_measure.cpp`).
- The largest tilt any pose can give, with the centre of mass anywhere in
  those bands, is **under 10 degrees**: 5.3 degrees of lean plus a 5 mm tip.
  The body's fall reflex is at 75 degrees.
- Every foot position is inside the legs' reach and every joint at least 3
  degrees inside its limits and inside the servo pulse range of Kilo's
  calibration (the right hind shoulder stops at -57 degrees in the
  calibration of 2026-10-10; with that foot lifted the routine keeps it at
  least 3 degrees short: `mind/tests/test_measure.py`).
- The routine stops itself if the tilt passes 25 degrees, or is 10
  degrees away from what the model expects.

## Afterwards: the numbers

```
kilo measure fetch --host kilo.local
```

downloads the last run (into `~/kilo-logs`, as `kilo runs --fetch` does),
fits it and prints a table like this (this one is from a run in the
virtual body, whose centre of mass was put 4 mm back and 3 mm to the right
and its IMU 1.5 and -2 degrees off; the fit found them):

```
parameter  unit          model   fitted     +-  significant  what
imu_pitch  deg            0.00     1.50   0.00  yes          IMU pitch offset (+ floor slope, + front/back legs)
imu_roll   deg            0.00    -2.00   0.00  yes          IMU roll offset (+ floor slope, + left/right legs)
com_x      mm             7.43     3.44   0.27  yes          centre of mass, ahead of the shoulders' middle
com_y      mm             0.00    -3.00   0.13  yes          centre of mass, to the left
com_z      mm            -2.83    -2.35   2.02  no           centre of mass, above the shoulder axles
sag        deg/kg cm      0.00     0.02   0.01  no           leg servos' give under load
twist      mm             0.00    -0.00   0.00  no           LF+RB legs longer than RF+LB (each pair by half)
lag_ms     ms            50.00    51.29   0.31  yes          servo dead time (with the IMU's own delay)
tau_ms     ms            70.00    73.36   0.39  yes          servo time constant
tip_ms     ms            85.76   370.66  31.39  yes          a tip's delay: the step over the edge to 40 % of the tip
```

The same numbers, with their uncertainties and how they depend on each
other, go to a file next to the recording (`run-...-measure-N.fit.json`).
`kilo measure fit FILE` fits a downloaded run again; several files fit
together.

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

**Level.** Standing still on four feet, the body is level whatever the
centre of mass does (four rigid legs of the same length on a flat floor),
so what the IMU reads is its own offset, plus the floor's slope, plus any
difference in length between the front and back legs (or the left and
right ones): those three look the same to it.

**Steps.** Telling the legs to tilt the body 5.6 or 7 degrees all at once
and watching the tilt follow gives the servos' timing directly: the tilt
starts late by the dead time and closes in at the time constant's pace.
It is a big, clean signal: one millisecond more dead time changes the
reading by 0.08 degrees at the steepest part, where the IMU resolves
0.06. The size of the step says how much the legs give: tilted, the weight
moves onto the lower legs, and if the servos give, he tilts a little more
than the geometry says (0.3 degrees with a sag of 3).

**Sweeps (edges).** With one foot up he stands on the other three. The
line between the two feet of the other diagonal is the edge he can tip
over. Sliding the feet under him moves his body, and so his centre of
mass, toward that edge; the moment it passes over it, he tips onto the
lifted foot. Where the feet were then tells where the centre of mass is,
to about half a millimetre across the edge. The two diagonals cross, so
the four sweeps between them pin down both forward-back and side to
side (a 1 mm error sideways moves a crossing by 2 mm, forward-back by 1
mm). Each diagonal is swept from both sides, which also cancels the
delay between the step that crosses the edge and the tip being seen.

**Rolled sweeps.** Leaning the body 7 degrees moves a high centre of mass
further sideways than a low one. So the same crossing, done leaning, moves
by an amount that depends on the height: 0.3 mm per mm of height leaning
away from the lifted foot, 0.2 mm the other way when leaning toward it.
The two lean the opposite way along the sweep, so the tip's delay does not
mix with the height.

**The legs' sag** shows in many places at once: in the steps, in how much
a lifted foot's tripod leans, and in where the crossings fall (a soft
tripod leans toward the lifted foot before he tips). **The twist** shows
when one foot is lifted: the tripod left behind leans toward or away from
the lifted foot by how much its diagonal is longer.

**Tip rate** (your idea: "watching the rate of tipping"). Once he tips, the
tilt grows like an inverted pendulum's, faster for a lower centre of mass.
The routine records every tip and the fit reports how fast each grew (its
e-folding time: how long the tilt takes to grow 2.7 times). **It is
reported, not used for the height**: in the virtual body, where the answer
is known (a point mass, 90 ms), it reads 68 ms, because a 5 mm tip is over
in about 0.15 s, before it is a clean pendulum, and the feet are still
moving. The rolled sweeps give the height instead.

## What it can and cannot measure

The fit was checked in simulation (`kilo measure simulate`,
`mind/kilo/measure/simulate.py`): runs made from Kilo's model with the
numbers changed at random (centre of mass up to 6 mm off forward-back, 4
sideways, 10 up or down; sag 0.5 to 4; twist up to 1 mm; IMU offsets up to
3 degrees; dead time 30 to 80 ms; time constant 40 to 110 ms), the IMU's
noise and 1/16 degree steps added, the routine's own early end of each
sweep, then fitted. And once on the virtual body's physics, a different
model (rigid legs, a real tip): above.

25 runs each way. "Error" is the fit's miss (root mean square); "+-"
what the fit said on average; "within": how often the truth was inside
2 +-. "Backlash" adds what a real floor and real servos do: every hold
comes to rest a little differently (0.15 degrees, at random).

| | IMU noise only | | | with backlash | | |
|---|---|---|---|---|---|---|
| number | error | +- said | within | error | +- said | within |
| IMU pitch, roll offset | 0.002 deg | 0.002 | 100 % | 0.04 deg | 0.04 | 92-100 % |
| centre of mass forward-back (com_x) | 0.13 mm | 0.15 | 92 % | 1.1 mm | 1.1 | 84 % |
| centre of mass sideways (com_y) | 0.05 mm | 0.09 | 88 % | 0.5 mm | 0.6 | 96 % |
| centre of mass height (com_z) | 0.9 mm | 0.8 | 88 % | 8.6 mm | 8.1 | 84 % |
| sag | 0.01 | 0.01 | 100 % | 0.2 | 0.25 | 100 % |
| twist | 0.001 mm | 0.002 | 100 % | 0.04 mm | 0.05 | 100 % |
| dead time | 0.2 ms | 0.2 | 96 % | 1.1 ms | 6 | 100 % |
| time constant | 0.2 ms | 0.2 | 96 % | 2.5 ms | 8 | 100 % |

On the virtual body's physics (a different model) the fit found the
centre of mass forward-back and sideways to 0.01 mm, the IMU offsets
exactly, the dead time 51 ms (50) and the time constant 73 ms (70); the
height -2.4 +- 2.0 mm (-2.8).

So: with real servos expect the centre of mass to about a millimetre
forward-back and half a millimetre sideways, its **height only to about
8 mm** (a weak handle: a crossing moves only 0.2-0.3 mm per mm of height),
the sag and the servo timing well. The +- are about right; the height's
and com_x's are a little optimistic (84 % instead of 95 % within 2 +-).

Measured: the centre of mass forward-back and sideways, the legs' sag,
the twist, the servos' dead time and time constant, the IMU offsets (with
the floor, unless a turned run takes it out). The height of the centre of
mass less well (a few millimetres).

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
  the IMU's steps; the steps and the tripods show the sag better. Left
  out. **Raising and lowering** the body tilts nothing at all. Left out.
- **Each leg's own servo timing**: one leg's step on four feet rocks the
  body onto a diagonal and mixes timing with the rocking; the pitch and
  roll steps time the same servos cleanly. One dead time and one time
  constant for all.
- **Inertia** (how the mass is spread, not just where its centre is): the
  tips are too short to tell (the tip rate, above).
- **Soft legs that do not clear.** If the legs give much more than the
  model thinks (sag well over 4) or one diagonal is much longer, a 5 mm
  lift may not take the foot off the floor: he leans onto it slowly
  instead of tipping. The fit then says "a slow lean, not a tip" for that
  sweep and uses no crossing from it; with fewer crossings the centre of
  mass is less sure (its +- grows, as it should).

## What is recorded

The run is in the flight recorder like a walk or a trick: `kilo runs
--host ...` lists it as `self-measurement (B1)`, `kilo runs --fetch` and
`kilo measure fetch` download it. In it: every foot position sent and the
attitude the walks' (rigid) model predicts for it, every IMU reading
(pitch, roll, their rates) and the battery, a mark at the start of every
part (`step i=.. exp=.. phase=..`), each tip the routine saw (`tip`), the
keys pressed, and how it ended (`end reason=DONE|STOPPED|FELL|ABORTED`).

## Where it lives

- The routine on the Pi: `kernel/mind/measure/` (`script.h`: the parts and
  their poses; `measure.h`: starting, streaming, stopping), B1 in
  `kernel/mind/remote/`.
- The same script in Python, the statics (how he stands on his feet with
  springy legs), the fit and the simulation: `mind/kilo/measure/`.
- The command: `kilo measure plan|fit|fetch|simulate` (`mind/kilo/cli_measure.py`).
- Tests: `kernel/host/tests/test_measure.cpp`, `mind/tests/test_measure.py`,
  and the whole loop (the routine in `kilo-mind-host` against the virtual
  body, fetched and fitted) in `kernel/host/test.sh`.
