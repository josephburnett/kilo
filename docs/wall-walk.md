# Walking toward a wall: how far does a step really carry him?

The plan tells Kilo's feet how far to push the body each step. Nobody knows
how far the body then really goes: feet slip, legs give, the floor is
smooth. The distance sensor in his eyes (the two silver cans: an ultrasonic
range finder, `docs/distance-guard.md`) can measure it: walk him straight at
a flat wall and the wall comes closer by exactly as far as he went.

Nothing new is needed on Kilo for this: an ordinary walk (C4), trot (A4) or
crawl (B4) toward a wall already records everything, and the distance guard
stops him by himself before the wall. The measuring is done afterwards on
the laptop, from the recording: `kilo log wall FILE.krec`.

## What the recording has, and what the laptop makes of it

While he walks the mind records every new distance reading (12.5 a second,
each stamped with the body's 20 ms tick), his body's tilt, the gyro's turn
rate, the gait and the speed it was set to, and where every foot was told
to be at every tick (the "step clock"). While the mind walks him it holds
his head straight ahead and level, so the beam points along his body.

`kilo log wall FILE.krec` then reports, for each forward walk in the file:

- **travelled**: the wall's distance while he stood still before the walk,
  less the one while he stood still after the stop (the second between the
  stop and the sit), with an error estimate that names its parts;
- **commanded**: how far the feet on the floor pushed the body, summed tick
  by tick from the plan, and the set speed times the time walked;
- **the share he really went** of the feet's push (1 = no slip; the slip
  is 1 less it), from the two still ends and, as a check, fitted over every
  reading;
- for each stretch at one speed: the measured speed against the commanded
  one, and the distance per step cycle;
- the turn (from the gyro, less its resting offset; + = left) and the
  sideways drift it implies.

The readings are corrected with the body model (`protocol/spec.toml`
`[model]`): the beam tilts with the body (the body's pitch, less what the
tilt sensor reads standing level), the sensor sits at the model's nose and
moves with the pitch, and a beam turned away from square reads the wall
further off (1 / cos of the turn). These corrections are small: about 1 mm
for the pitch, a few mm for a turn of 10 degrees. Readings with the head
turned, past 2.5 m, without an echo, of the floor, or single stray echoes
are left out, and so is everything once he has turned more than 15
degrees from where he started (a flat wall's echo is not trustworthy past
that). If the readings do not follow one flat wall (something else came
into the beam), it says **NOT A WALL MEASUREMENT** and why.

What limits it: the speed of sound the sensor assumes (343 m/s, right at
20 degrees C; 1.5 % off at about 10 degrees warmer or colder), how square
to the wall he starts (taken as up to 3 degrees off), and a gyro that is
not perfect. With a tape measure at the start and the end (step 7 below)
the speed of sound drops out: `kilo log wall FILE.krec --tape START END`.

## What the sensor is like (the recordings of boots 14 to 17)

From the recordings of 2026-11-21/22 (none of them a walk at a wall; read,
not measured for this):

- Standing still, the readings agree within 1 to 2 mm up to 1.5 m (most
  readings repeat exactly or move by 1 mm; now and then by 4 to 5 mm); at
  2 m about 9 mm.
- Walking with the head held straight, one reading is about 4 mm off (the
  walk, the crawl) to 6 mm (the trot), part of it his body really surging
  forward and back within a step. 5 to 10 % of readings are stray echoes
  more than 4 cm off for one reading (more at the trot); no missed echoes
  in these walks.
- The body's pitch and roll while walking (pitch -1 to +4 degrees, roll up
  to 12 degrees at the crawl) did not visibly disturb the readings: no
  connection between them and the reading's error was found (correlations
  under 0.12).
- With the head moving (the A5 behaviour looking around), 15 to 34 % of
  readings were stray: they are useless, and the laptop leaves them out.
- In all three gaits he turned while walking: 2 to 3.6 degrees a second to
  the left at the walk (`docs/walk-trial-results.md`). Over 40 cm at the
  walk that is 15 to 25 degrees, which is why the walks below are short.

## How he stops (the distance guard)

Walking forward he stops by himself when the wall is about 20 cm from his
eyes (21.5 cm at the trot's top speed): the quick stop, then he sits. In
the computer's simulation that leaves 15 to 19 cm between his eyes and the
wall (`docs/distance-guard.md`, "The thresholds"); not yet seen on the
floor. The second between the stop and the sit is when he stands still for
the end of the measurement, so let him stop by himself.

## The floor procedure

Take as long as you like between the steps; nothing is waiting on a timer.

**You need:** a charged battery; a hard flat floor; a **flat, hard wall** at
least 50 cm wide and 20 cm high where his eyes point (a skirting board or a
plain wall, a big cardboard box side or a door; not a curtain, a sofa, a
cushion or a corner); nothing else on the floor in front of him; a tape
measure if you like (step 7).

1. Switch Kilo on and wait until his eyes are steady green (he is ready,
   `docs/signals.md`).
2. Put him on the floor facing the wall square on, his nose pointing
   straight at it (look from above: his body at right angles to the wall).
   How far: for the **walk** about **60 cm** from the wall to the front of
   his eyes; for the **trot** about **100 cm**; for the **crawl** about
   **50 cm**. Closer than 20 cm he will not start (the guard refuses a
   forward key that close: three quick purple flashes).
3. **B6**: he stands. Check he still points square at the wall; nudge him
   straight if not. Then let him stand still for a moment (a few seconds is
   plenty): the readings he takes standing still are the start of the
   measurement.
4. Press the gait's key once and nothing else: **C4** (walk), **A4**
   (trot) or **B4** (crawl). Do not steer and do not change the speed.
5. He walks to the wall, stops by himself, and sits. Do nothing until he
   sits (a second or so after he stops). If anything looks wrong, **B6**
   or **C7** stops him at once, and picking him up is fine.
6. Note which gait it was and anything you saw (a foot slipping, a turn).
7. If you have a tape measure: before step 4 and after he has stopped,
   measure from the wall to the front of his eyes (the silver cans) and
   note both, to the nearest half centimetre. This is optional; it lets the
   laptop check the sensor's own scale.
8. Carry him back to the start (step 2) and do it again: twice for each
   gait if you can, the walk first.

Afterwards, with your OK, Claude fetches the recordings from Kilo (`kilo
runs --fetch`) and runs `kilo log wall` on each. Write what you saw in a
`docs/wall-walk-results.md` (Claude does this with you); "don't know" is a
fine answer.

## Where the result goes in the body model

`protocol/spec.toml` `[model.travel]` has one number per gait,
`walk_ground_frac`, `trot_ground_frac`, `crawl_ground_frac`: the share of
the feet's push the body really goes. They are 1 today (no slip), which is
what the planner takes; `[model.about]` says they are assumed. `kilo log
wall` prints the value a walk measured and the line it would set (`for the
model: [model.travel] trot_ground_frac = ...`); the mean of a few good walks
of a gait goes there, with `[model.about]` saying where it came from.
Nothing reads them yet besides `kilo log wall` and `BodyModel.ground_frac_of`;
the planner can use them to command the speed it wants.
