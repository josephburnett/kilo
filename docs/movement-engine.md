# One movement engine

Written 2026-11-22 as a design; **steps 1 and 2 are built** (step 1 the
same day, step 2 on 2026-11-23: "What exists now" just below), **step 4 in
part** (the paw lift and face wash, 2026-11-23); steps 3, 5 and 6 are not. The owner's four decisions
on it are at the end. Numbers marked **(est.)** are estimates; every other
number is read from the code, the docs or the recordings named next to
it, most of them from
[movement-engine-results.md](movement-engine-results.md) (the old walk,
trot, long trot and crawl of 2026-11-17 to 11-21). Nothing of it has been
tried on Kilo: everything below about steps 1 and 2 was checked on the
laptop only (the body's real program in its host tests, the mind against
it, the walks on the virtual body).

## What exists now (step 1, 2026-11-22)

- **One stream of poses.** The mind moves the body only by `POSE`
  messages (protocol 3.0, `protocol/spec.toml`): a pose and the tick by
  which to reach it. Each leg is a foot (where it is, in quarter
  millimetres) or two joint angles; the head joints are given or left to
  the body's posture; the pose carries the attitude the mind expects
  there (the body's balance reference) and which feet are down. FEET and
  the five TRICK messages are gone.
- **The board assumes the poses** (`body/src/core/stream.h`). It keeps up
  to 5 of them in tick order and moves toward the next one from wherever
  the joints are, arriving exactly on its tick, in a straight line or
  along a smooth (cosine) curve. So a walk sends a pose every tick, and a
  trick one per keyframe. It still corrects its balance itself (through
  the feet for a walk, in the joints for a trick frame that asks for it),
  clips every joint to its limits and speed, and when the poses stop it
  holds the last one (the swing feet of a walk come down). It reports,
  every TELEMETRY, the correction it carries, the reference, the feet down,
  how many poses wait, and which legs a limit or the slew cut
  (`limited`, new).
- **The board's trick player and its get-up (rc) are deleted**, as
  decided. After a fall the body holds him until he is still, lays him
  down and lets the legs go limp 2 s later (EVENT LYING). He does not get
  up by himself; B1 (in a `KILO_RIGHTING=1` build) runs the righting
  moves as poses the fall reflex hands the legs over to.
- **The mind** sends the walk's and the self-measurement's feet as before
  (whole millimetres), 3 ticks (60 ms) ahead instead of 5 (results §1).
  The trick player (`kernel/mind/tricks/player.h`) turns each library
  trick into poses: mirror and speed applied in the mind, the end posture
  sent when the body's tick reaches the last pose, the guards as before.
  While a trick plays it holds other posture requests back until the end
  (a posture ends a stream on the body), as the body's player did.
- **Room on the board**: flash 27094 of 30720 B (was 27572), RAM 1249 of
  1540 B (was 1539).
- **What the owner should see on the floor**: every key as before (the
  walks, the trot, the postures, the random mode's tricks, the rear-up,
  the self-measurement). Differences to look for: a fall no longer ends
  in a get-up attempt but lying down and going limp; a change asked of a
  walk (faster, slower, steer, stop) reaches the legs about 40 ms sooner
  (60 ms of poses queued instead of 100). Not yet
  checked: whether 60 ms of look-ahead is enough on the Pi under load (the
  recordings say yes, results §1).

## What exists now (step 2, 2026-11-23)

- **One step clock** (`kernel/mind/engine/clock.h`). A phase goes round
  once a cycle; each leg lifts when the phase passes its own point. The
  cadence (how fast the phase goes round) is a dial that can change at any
  tick. The clock can **wait**: a tick it does not advance, the feet hold.
- **The gaits are rows of a table** (`engine/gait.cpp`), not programs: the
  order the legs lift in, how the four-foot time is shared out before each
  lift, the swing, the lift, the body's height and the **weight target**
  (how far inside the feet on the ground the centre of mass must be): the
  walk 3.5 mm, the crawl 8 mm, the trot none (on two feet the landing
  balances him: each pair lands on the pendulum point fitted to the floor
  recordings, as before).
- **The tick solver** (`engine/solve.h`) replaces the hand-tuned planner.
  Every tick: the commands move toward what was asked (speed, steer,
  direction, cadence: no stop needed for any of them); the clock says which
  legs lift; a lifted foot swings to half a stride ahead of its shoulder;
  and the weight: the solver looks 40 ticks (0.8 s) ahead, where the clock
  says every foot will be, and finds the smoothest path of the body (a
  shift fore and aft over the feet and a roll) that keeps the centre of
  mass the gait's target inside the three feet down at every one of those
  ticks. This tick's body pose is that path's first point. The walk's old
  fixed sway (1° of roll, 3 mm fore and aft) and the crawl's lean solve are
  gone: one solve does both, from the model. A lift waits while the weight
  is not yet there. The gate (no lift while the body is more than 5° off
  the plan) and the catch (a swinging foot down at once when he tips 7°
  toward it) are the clock's, so every gait has them. The feet go to the
  board in quarter millimetres (they went in whole ones). Each leg's own
  servo delay (`[model.correction]` servo_lag_d_ms) moves that leg's foot
  earlier along its path; all are 0 until measured, so this changes
  nothing yet.
- **Speed and cadence are separate.** A walk goal (`WalkGoal`) has a speed
  and a cadence (0.6 to 1.6 of the gait's own: a faster cadence, shorter
  steps for the same speed); a stride past the gait's longest raises the
  cadence instead. Nothing on the remote sets the cadence yet.
- **Judged in simulation, gait by gait** (docs/movement-engine-results.md,
  "Step 2"): against the tuned planner on the virtual body's judge, the
  walk rocks about half as much at 40 to 72 mm/s with more margin, at the
  same speed and prediction; its fastest joint is up to 45°/s faster at
  its top speeds (459°/s, under the body's 500). The trot is as before
  (its timing and pendulum landing kept). The crawl rocks less than half
  as much, at the same speed. All three replaced the planner; the old
  planner and its Python twin are deleted.
- **What the owner should see on the floor** (not yet tried): C4, A4, B4
  and B7/B5 as before. The walk should sway less from side to side (by the
  judge about half; the floor may show less of the difference: the judge
  rocks less than the floor does), its body moving a little fore and aft
  instead. The crawl (B4) should roll far less (it rolled up to 14°) and
  look smoother. The trot should look the same. A change of gait still
  goes through standing feet (walk to trot without squaring up is not
  built: the gaits' different lift patterns go through home).
- **How to judge again**, after a change or with new recordings:
  `make -C kernel/host build/engine-judge`, then
  `PYTHONPATH=mind python mind/tests/enginejudge.py table` and `changes`.
  The laptop reaches the engine only through that program (`kilo.engine`);
  there is no Python copy of it.

## What exists now (step 4, the first trick, 2026-11-23)

- **The paw wash** (`kernel/mind/engine/paw.h`): the owner's first trick
  as a goal (decision 4). Standing, he moves his weight back and onto the
  other side (by the model: the body 7 mm back and rolled 5°, so that
  the centre of mass is 7 mm inside the three feet that stay down, twice
  the walk's 3.5 because he holds it; the lean is held until the servos
  have it before the paw leaves the floor), lifts a front paw, first just
  off the floor over a third of a second, then up beside his
  muzzle, tilts his head down to it, strokes down his face and back up two
  to four times, puts the paw down, and stands. It is made from the body
  model each time it is asked for, at the tempo and the number of strokes
  asked (0.5 to 2 times its own pace, 1 to 8 strokes), not played from a
  recording; either paw (the player mirrors it).
- **Safety.** A hold with the paw up is watched: if he is not standing
  still on three feet (the roll more than 6° off what the lean expects, or
  the pitch more than 6°), the trick stops and he goes back to STAND. The
  board's balance correction is off for it. Judged on the laptop
  (`mind/tests/test_engine_tricks.py`; results, "Step 4"): by the
  quasi-static model the centre of mass stays at least 3.2 mm inside the
  feet all along (6.4 mm but for the right paw on Kilo as recorded, whose
  right hind shoulder sits differently) and the head at least 16.7 mm from
  the paw (the rule: 10 mm,
  either way the pan turns); played on the tricks' dynamics with the
  servos' lag and the legs' give, it neither falls nor trips its guard in
  the fit or at any of the 19 corners of what the recordings allow.
- **Where it plays**: the random mode's face wash when he stands (A5, then
  A5 again for a trick: the trick drawn is the face wash one time in five).
  Sitting, the face wash is the library's as before. The library's standing
  face wash (it sat down first) is deleted.
- **What the owner should see** (not yet tried): standing in the random
  mode, now and then he leans back and a little to one side, lifts a front
  paw up beside his face, dips his head to it and wipes a few times, then
  stands again. It has never run on Kilo.
- **Not built in step 4**: the other tricks as goals (the stretch, with its
  shake as an overlay, the sniff, the rear-up as "nose up to N degrees").
  **Step 3** (postures as goals) is not built: sitting and lying rest the
  body on his haunches and belly, which the engine has no model of; the
  moves between postures stay the intent layer's, planned and checked in
  Python's statics and dynamics (docs/intent.md).

## Summary (half a page)

Today Kilo has three ways to move that share one body model but nothing
else: the **gaits** (the mind streams foot positions, `kernel/mind/planner/`),
the **postures and intents** (moves planned once and played,
`kernel/mind/intent/`), and the **tricks** (keyframes uploaded to and played
by the body, `kernel/mind/tricks/`, `body/src/core/keyframe.cpp`), plus the
body's own fixed get-up (**rc**). Each was built for one job, so none of them
can be interrupted, re-timed or adapted by the others.

The proposal: **one engine in the mind** that is given a goal ("be standing",
"walk there at this pace", "lick the left paw", "get up") and every 20 ms
works out the poses Kilo should take next from the model and what the
board reports right now. A movement is no longer a recording to replay but a
small set of wishes (which feet carry him, where his weight goes, where a
paw goes, at what tempo) that the engine re-solves every tick from where
Kilo actually is. Tempo is a dial, not a property of a recording. Getting
up is steered by the measured roll, not played blind.

**The board is not a puppet.** In the owner's words: "We absolutely need to
be clear on the role of the board. It's not a puppet. It's an independent
layer responsible for assuming a series of poses and automatically
correcting error and surfacing that as a stream of data for high level
planning." So the mind sends the board **poses to assume, each with the
time to reach it**; the board gets there by itself (slew and joint limits,
its 50 Hz servo loop), corrects the error it measures on its own (the
balance correction), keeps working toward the last pose and holding it if
the mind goes quiet, and reports, every tick, what it corrected, what it
could not do and the error left over. The mind plans from that report. The
board loses the trick player, rc and the FEET-only format: **one pose
message** replaces FEET and the five TRICK messages, which frees an
estimated 300 B of the board's RAM (it has almost none spare today) and
1.5-2 KB of flash **(est.)**.

The way there is six steps, each tried on the floor and each deleting what
it replaces. Part one says what the recordings already settled and what
only the floor can. Step 1 is built (above); 2 to 6 remain.

## Part one: what to learn first

**A. Learned from the recordings (done 2026-11-22;
[results](movement-engine-results.md), `mind/tests/enginestudy.py all`
recomputes it after new floor runs).** All from the old gaits and builds
of 2026-11-17 to 11-21.

1. **The link is steady and early.** Every FEET reached the body at least
   77 ms before its tick with the 100 ms look-ahead (17 852 ticks, 0
   late, 0 underruns); TELEMETRY arrives within 2.7 ms of its slot. A
   **60 ms look-ahead (3 ticks)** would have kept 25 ms in hand on every
   recorded tick. A reading reaches a changed command through the mind in
   about 75 ms then (110-120 ms now).
2. **Three feet hold the weight from about 0-2 mm inside**, not 10: over
   681 single-leg swings the tilt toward the lifted foot is +3.4° with the
   weight 2-6 mm outside, +1.7° at 0-2 mm inside, +0.1-0.3° at 2-6 mm
   inside (inferred, medium confidence: those swings are short). The
   engine keeps **3 mm**. The new walk (+0.4 mm at the hand-over) should
   rock a little there, far less than the old walk.
3. **The body's correction is small and mostly frozen in gaits** (held 64 %
   of GAIT ticks; median 0.75°), and standing it is largely a steady 0.5°
   lean against the IMU's 2.4° mounting offset, which it takes for real
   tilt. It does not shape the gait.
4. **The servo lag does not follow the battery**: 105-127 ms (median 123)
   at 6.6-7.7 V, no trend over 10 runs. One fixed lead.
5. **The state estimate is right upright** (99.7-100 % over 38 000 judged
   frames, 2 brief wrong stretches); lying it is untested (65 still frames
   after falls).
6. **Getting up**: rc got him up at the first try in 4 of 6 floor falls,
   once at the third, once not at all (tail down: no try). Rolls come in
   bursts of 0.1-0.8 s (median 0.16 s, up to 175°), at up to 580°/s.
7. **Tricks keep time** (keyframes within 8-41 ms of schedule, 48
   keyframes). Whether the IMU follows the model off a stance is not known:
   every recorded trick is a replaced rear-up design.

**B. Needs Kilo on the floor** (keys after the pending merges: C4 walk,
A4 the trot, C5 twice the self-measurement, A1 rear-up, B1 the righting
experiment in a `KILO_RIGHTING` build, A5 the random mode; no step is
timed). No trial is needed for the link delay or the battery lag (A1, A4).

8. **The new walk** (C4 at each level, B7/B5; docs/walk-trial.md, with a
   side video), then `walkstudy.py floor` and `enginestudy.py margin`.
   *Answers* item 2 for the new walk itself: the recordings say a margin
   near 0 rocks a little; this says how much.
9. **Self-measurement v2** (C5 twice), two or three runs on the same floor.
   *Answers:* the centre of mass height (the first run hit the fit's limit),
   the feet rectangle, sag under load, the IMU's offset (which must then
   reach the board, item 3), and how much these vary between runs, which
   tells how much the engine should adapt them live.
10. **The trot** (A4, B7 up to level 5) with video: slip on the long stride.
    *Answers:* whether slipping is common enough to need detecting.
11. **Tricks in the random mode** (A5, then A5 again for each trick): do
    they look right at the original timing? Let one rear-up (A1) run to
    its end. *Answers:* which moves are worth keeping as goals, and, with
    today's library in the recordings, the IMU against the model (item 7).
12. **Falls with hands off**, before step 1 deletes rc: the built-in rc (a
    build without the righting switch) and B1 (with it), on each side, the
    back and once nose or tail down, filmed. Leave him lying still for a
    few seconds once before touching anything. *Answers:* which phase of a
    get-up fails, whether a few actions chosen from the measured roll can
    do it, and the estimate's lying states (item 5).
13. **A slope and a push**: standing (B6) with a book under the two left
    feet, then a gentle push from the side; the same while walking (C4).
    *Answers:* how big the residual is that the board's correction leaves,
    and how big the disturbance must be before a step has to change.

## Part two: the architecture

### Layers

```
behaviour, remote, guard   -> GOALS        (a few per second)
engine: goal -> TASK       -> what must hold: support, weight, paw, tempo
engine: tick solver, 50 Hz -> POSES with the time to reach each, a few ticks ahead
board, 50 Hz               -> assumes the poses: slew, limits, its own balance
                              correction; reports corrections, limits hit, residual
observer (mind, every TELEMETRY) -> state, errors, contact guesses -> back to the engine
```

- **Goal** (new `kernel/mind/engine/goal.h`): a small struct, one of
  `Be{STANDING|SITTING|RESTING}`, `Go{velocity, turn, pace, cadence}`,
  `Do{trick, side, tempo, repeats}`, `Right{}`, `Measure{}`, `Hold{}`. A new
  goal replaces the old one at any tick; the engine works out the way from
  the current state to the new one. The behaviour runner, the remote and the
  distance guard only ever make goals.
- **Task**: what the goal means right now, in the model's terms. Not a list
  of poses. Its parts:
  - a **contact plan**: for each foot, on the ground or not, from a gait
    clock (below) or from the goal (three feet for a paw trick);
  - a **weight target**: where the centre of mass must be, as a margin
    inside the support the contact plan leaves (from the model,
    `model.h`): 3 mm inside three feet (results item 2);
  - **body targets**: height, pitch, roll, fore-aft shift;
  - **end-effector targets** for free limbs: a paw at a point relative to
    the head, the head and tail angles;
  - **limits**: joint speed (the board's 500°/s slew), the servos' load from
    the model (as the rear-up and righting checks do now).
- **Tick solver** (`engine/solve.cpp`): each tick, from the task and the
  observer's estimate, the body pose and the four feet, then the joint
  angles by the same planar two-link IK the model already has. It looks
  ahead over the next swing (about 0.2-0.5 s) for the weight path, as the
  planner's solve before a lift does now (§10), but redoes it whenever the
  goal, tempo or measured state change, not once per move.

### A movement is wishes plus a clock, not keyframes

- **Gait clock**: each leg has a phase 0..1 advancing at the cadence. A gait
  is four phase offsets and a duty factor: the walk (lateral sequence, LB LF
  RB RF, duty 0.77), the trot (diagonals), the crawl. The swing foot's path
  and landing point follow from the phase, the commanded velocity and the
  weight target. Today's `patterns`, `plan`, `walker` and `trot` become this
  one clock and one table of offsets.
- **Tempo is live**: cadence and stride are separate dials. A goal may set
  both, or only a speed and let the engine choose the cadence (cats lengthen
  the stride first, §16.2). Changing gait means moving the offsets toward the
  new ones over a cycle rather than walking the legs home and starting again
  (today's change between walk and trot takes 1.5-2 s, §16.6).
- **The clock can wait**: today's lift gate (no lift while more than 5° off
  the plan, up to 0.5 s) becomes "the phase stops advancing"; today's catch
  (a swinging foot comes down when he tips 7° toward it) becomes "this
  foot's contact comes early". Both already exist in `plan.cpp`; they become
  properties of the clock that every movement gets, including tricks.
- **Tricks use the same clock**: "lift a paw and wash the face" (the first
  trick rebuilt, decision 4) is: contact plan three feet, weight 3 mm inside
  their triangle, the free paw to a point under the chin, head tilted down
  to meet it, a small paw stroke at the tempo. "Stretch" is: front feet
  forward, body low in front, hold. Because these are wishes, they are
  valid from any pose the solver can reach: from sitting, a paw trick needs
  no three-foot shift; from standing it does, and the engine makes it.
- **A shake is an overlay, not a pose.** The owner asked (2026-11-22) for
  the stretch to end with a little shake of the raised rear, and how that
  is expressed here. In the engine the stretch is a held goal: front low,
  rear high, the weight where the solver keeps it over the feet, head up. The shake is not part of
  that goal: it is an overlay, a small oscillation with an amplitude (a few
  mm of the rear) and a rate (a few swings a second) added on top of
  whatever goal is being held. So the same shake can be put on any pose
  (standing, sitting, the stretch), at any tempo, and the board keeps
  correcting balance underneath it as it does for the held pose. Today,
  with tricks still recorded keyframes, the shake is only extra keyframes
  at the end of the stretch's hold (the hind feet 4 mm forward and back
  three times, the tail with them: `mind/kilo/tricks/alive_set.py`
  `SHAKES`); it cannot be put on another pose without writing it again.

### Balance: the board's layer, and what the mind does above it

The board corrects in its own tick: about 4 ms from IMU to servo command
plus up to 10 ms of IMU sample age **(est.)** (docs/architecture.md §2). A
loop through the mind is, measured: 5 ticks between the reading the mind
had and the tick its command plays (100 ms look-ahead), plus 5-15 ms
inside the board's tick; about 75 ms with the 60 ms look-ahead this
proposal takes. The servos themselves take about 120 ms (measured 105-127
ms in the walks). A tip grows with an e-folding time of about 65-112 ms
(self-measurement, one timed tip).

So:

- **On the board, as its own layer** (decided 2026-11-22): the balance
  correction (its reference is the engine's predicted attitude, so it only
  sees model error), slew and joint limits, the watchdog's hold, settle
  and rest, battery limits, fall detection. These act in the first 100 ms
  and need nothing from the mind in that time. Today the correction is
  small (median 0.75°) and held while any foot is up, so it does not shape
  a gait; it must be given the IMU's measured offset, or it keeps leaning
  against a tilt that is not there (results item 3).
- **The board reports what it did**, every tick: the correction it carries,
  the reference it held to, the residual (attitude minus reference), on or
  frozen, the feet it corrected through, and, new, which limits cut the
  pose it was given and the joint angles it commanded. The observer reads
  this as data, not as an echo of what was sent.
- **In the mind**: everything that changes the *plan*: where the next foot
  lands, waiting or hurrying a phase, leaning uphill on a slope, widening
  the stance after a push, slowing on a low battery. These act over the next
  step (200-500 ms), where the 75 ms through the mind is small against the
  servos'. The catch (a swinging foot down early) stays in the mind: its
  loop is shorter than one tip's growth plus the servos.

### Contact and support without foot sensors

Kilo has no foot switches and no servo position feedback. Contact is
therefore a belief: the plan says which feet are down; the observer checks
it against the attitude and the board's report. A measured tilt toward a
foot the plan says is down means that foot is not carrying (a slip, a
hole, a soft floor); a steady offset means a slope or a model error (the
self-measurement's 2.4° nose-down is one). The existing error estimator
(`planner/errors.cpp`: offset and lag from the telemetry) becomes the
observer's, and its results feed the engine live, within bounds: a learned
centre-of-mass offset shifts the weight target. The servo lead is one
fixed number from the self-measurement: it did not change with the battery
(results item 4). Slipping forward can also show in the distance sensor
when facing a wall (80 ms cadence), as progress smaller than commanded;
that is a later addition.

### Each servo: its own lag and strength

The servo lead above is one number for the whole robot. The owner's rule
(2026-11-22): the model must have a place for each servo's own lag and its
own strength, because one slow or weak servo is enough to tip a move that
loads it (the rear-up of 2026-11-22 rolled left as the hind legs pushed;
docs/rear-up-trick-results.md).

**Where it lives.** In the one body model, `protocol/spec.toml`
`[model.correction]`, by the body's joint id 0..15 (the same numbering as
the servo zeros): `servo_lag_d_ms` (this servo starts moving this much later
than the robot's `servo_lag_ms`), `servo_tau_d_ms` (it closes the gap this
much slower than `servo_tau_ms`) and `servo_give` (it gives under load this
many times the legs' `sag_deg_per_kg_cm`). The defaults (0, 0, 1) say every
servo is the robot's, which is all that has been measured, so nothing Kilo
does changes until a measurement says otherwise. Python `BodyModel`
(`joint_lag_ms`, `joint_tau_ms`, `joint_sag`) and C++
`kernel/mind/planner/model.h` (`joint_lag_ms`, `joint_tau_ms`) read them;
`kilo model show` lists them; `kilo model apply` writes them from a fit
file's `lag_d_ms:JOINT`, `tau_d_ms:JOINT` and `give:JOINT` numbers (JOINT a
name such as `lb_knee`). The tricks' model (`kilo.tricks.dynamics`) already
plays each servo with its own lag, time constant and give when they
differ.

**Who uses it, and how.** The knowledge stays in the model; the mind plans
with it; the board does not need it (it holds poses and corrects what is
left):

- the engine's walks send each leg its own lead (built in step 2, per leg:
  the mean of its shoulder's and its knee's): a leg 30 ms slower gets its
  foot 30 ms earlier along its path, so the legs arrive together;
- a move that loads one side (the rear-up's push, getting up) is checked in
  the tricks' model with that side's give and lag, and paced so the slower
  side keeps up: the move's duration is set by the slowest servo it loads;
- the engine's observer compares the attitude it predicts (with each
  servo's numbers) against the measured one; a residual that grows only
  when one leg is loaded points at that leg.

**How it would be measured** (not built yet). The present self-measurement
moves legs in pairs (front against back, left against right), so it sees
the robot's lag and give, not a servo's. The step that would: on four
feet, the body 10 mm forward so that each hind leg carries about a third of
the weight, one hind leg at a time lengthened 3 mm in one tick and held,
then shortened back; the IMU's roll and pitch over the next 300 ms give
that leg's dead time and time constant (the moment the tilt starts and how
fast it gets there), and the held tilt against the model's gives its give
under that load. Then the same for each front leg, the body 10 mm back.
Eight short steps, each a 3 mm move on four feet (no tip possible), about
15 s in all; comparing left with right needs no absolute model, only the
two answers. The 25-per-second telemetry limits the timing to about 20 ms,
which is enough for the 40 ms or more that would matter.

### Lying down and getting up

The lying simulator reproduces none of the four recorded rc get-ups
(docs/righting-results.md), so no open-loop sequence designed in it can be
trusted. rc itself got him up at the first try in 4 of 6 floor falls.
Proposal:

- A few **actions** the model suggests, each a task like any other: "kick
  with the legs underneath", "throw the legs on top over", "tuck the head",
  "fold", "push up to the belly" (the righting branch's kick and throw are
  the first two).
- A **policy in the mind**, ticked at 50 Hz on the measured roll and roll
  rate: which action for this roll band and side; switch when the roll has
  not improved for 0.3 s (the recorded rolls come in bursts of 0.1-0.8 s,
  median 0.16 s); give up after three tries (as the experiment does: limp
  and a meow). The board must stream TELEMETRY at 50 Hz while lying (25 Hz
  today outside a walk).
- **Learning from each try**: log (start roll band, action, outcome). After
  enough falls the policy prefers what worked on Kilo, not what worked in
  the simulator. The simulator only ranks the first candidates and screens
  servo loads.
- **No body reflex**, as decided 2026-11-21 and 2026-11-22: the board
  detects the fall, holds the legs soft and reports; getting up is a goal.
  rc is deleted at step 1.

## Part three: what it replaces and deletes

Deleted outright once its step lands (git keeps it):

- Mind, C++ (**done in step 2**): `kernel/mind/planner/{patterns,plan,trot}.*`
  became `kernel/mind/engine/{gait,clock,solve}.*`, the walker moved to
  `engine/walker.*`; `kernel/mind/intent/intent.*`, `intent.gen.*`,
  `gen_intent.py`, `table.h` (the posture graph and its moves;
  `estimate.*` moves to the observer); `kernel/mind/tricks/{player,library,library.gen}.*`,
  `gen_library.py`; the righting moves in `intent/`.
- Mind, Python references: `mind/kilo/planner/{patterns,plan,walker}.py`
  (**done in step 2**, with their tests and the walk simulations
  `walksim.py`, `stridesim.py`, `stopsim.py`),
  `mind/kilo/intent/{graph,moves}.py` and `moves/`, `mind/kilo/tricks/{library,legacy,import_legacy,player,rear_up}.py`
  and `library/`; their vector generators and vector files under
  `kernel/host/tests/`. The engine is written once, in C++, and tested in
  the laptop build of the kernel (decision 3). Kept in Python as tools:
  `dynamics.py`, `righting.py` (the simulators), `measure/` (the fit), the
  study scripts.
- Protocol (**done in step 1**): `FEET`, `TRICK_BEGIN`, `TRICK_FRAMES`,
  `TRICK_COMMIT`, `TRICK_PLAY`, `TRICK_STOP`, `FALL_RC` are gone, replaced
  by one `POSE` message of 31 B (1850 B/s at 50 Hz, 7 % of the line): per
  leg a foot (x, z in quarter mm: x ±120 mm, z 15-110 mm) or two joint
  angles, the head and tail angles or "leave them", the attitude the engine
  expects there, the feet that carry him, and flags (which legs are angles,
  balance, may-play-while-fallen, eased). TELEMETRY's `limited` says which
  legs a joint limit, the slew or the leg's reach cut; the commanded joint
  angles are the JOINTS stream, which the mind can turn on.
- Board (**done in step 1**): `core/keyframe.*` and `core/feet.*` replaced
  by `core/stream.*`; rc and its states gone from `reflex`/`body.cpp`/
  `posture.cpp`; the trick dispatch gone from `link.cpp`. Freed: 478 B of
  flash and 290 B of RAM (the estimate was 300 B and 1.5-2 KB: the new
  stream player, with its interpolation and joint mode, took back about
  1 KB of what keyframe.cpp gave). The joint-space balance gains stay
  (the built-in postures and the tricks' frames use them).

## Part four: the path

Every step is tried on the floor with keys he already has, and deletes
what it replaces.

1. **One stream (built 2026-11-22, not yet tried on the floor).** `POSE`
   replaces FEET and the TRICK messages; the board's trick player and rc
   are gone. The look-ahead is 60 ms. The walker sends POSE in foot mode;
   the tricks and intent moves go as one joint-mode pose per keyframe
   phase (the board interpolates between them). Floor try: every key looks
   as before; tricks at the same timing (compare the recordings of item
   11); after a fall he lies soft and reports, and B1 (a `KILO_RIGHTING=1`
   build) runs the righting experiment's moves through the new stream.
   *Why first*: every later step needs it, it gives the board back the RAM
   the rest needs, and it changes no movement, so a difference on the
   floor is the transport's fault and easy to find.
2. **Gait clock and solver (built 2026-11-23, not yet tried on the floor).**
   Walk, trot and crawl are rows on one clock; `planner/` (but the model
   and the error) is gone. Floor try: C4, A4, B4, B7/B5 mid-walk, judged
   by the roll numbers `walkstudy.py floor` computes. The tempo changes
   without a stop (it did before, too). Not built: walk to trot without
   squaring up (the offsets could slew from one to the other, the clock
   allows it; judged in simulation only through home so far).
3. **Postures as goals.** Stand, sit, rest from any measured state through
   the solver; `intent/` moves and the graph go. Floor try: B6, A7, the
   random mode's sitting and lying down, from odd starts.
4. **Tricks as goals (begun 2026-11-23: the paw lift on three feet and the
   face wash, standing).** The kept tricks (item 11) rewritten as tasks with
   a tempo; the keyframe library goes. Floor try: A5 twice, A1 (rear up as
   "nose up to N degrees").
5. **Closed-loop getting up.** The policy of part two; the righting
   experiment goes. Floor try: B1 from each side and the back, then the
   same as a goal the engine sets itself after a fall.
6. **Live adaptation.** The observer's offsets feed the engine within
   bounds; slope lean; a changed step after a push. Floor try: item 13
   again, before and after.

**Main risks.** The mind now plans every movement: a kernel stall becomes
the board finishing its last pose and holding it with its correction on
(the watchdog then settles him), so a stall mid-trick leaves him in that
pose rather than mid-fall; the solver must only send poses that are safe
to hold. The solver replaces tuned, floor-tested code (the walk, the quick
stop) and will at first be worse at something. Between step 1 and step 5
he does not get up by himself (decision 1).

**What the remaining steps need that the laptop cannot give.** Step 2 is
judged against RockingPhysics (results, "Step 2"); the floor (items 8 and
10) has the last word: the walk it replaced was tuned there. Steps 3 and 4 build on step 2's solver. Step 5
needs falls recorded with hands off (item 12): nothing on the laptop
predicts a get-up. Step 6 needs item 13 (a slope and a push). The
per-servo lag and give in `[model.correction]` are all at their defaults
(nothing measured): the engine reads the lag when it plans (step 2), but
until the measurement of "Each servo" above is built and run it changes
nothing.

**Honest unknowns.** Whether 3 mm inside three feet holds on the floor for
the new walk (item 8; the recordings say so only for the old gaits).
Whether a few actions and a policy can get him up at all, since nothing we
have can predict a get-up (item 12). Whether the IMU follows the model off
a plain stance (item 11). Whether hobby servos with no feedback follow
wishes smoothly enough to look cat-like, or need the tempo kept low.

## The owner's decisions (2026-11-22)

1. **rc, the board's built-in get-up, is deleted at step 1.** Until step 5
   brings the new get-up he does not get up by himself; B1 runs the
   righting experiment's moves through the new stream.
2. **The balance correction stays on the board, "for sure".** The board is
   an independent layer: it assumes the poses it is given, corrects error
   by itself and reports what it corrected as a data stream the mind plans
   from (the owner's words are in the summary).
3. **The engine is written once, in C++**, tested in the laptop build of
   the kernel; no Python reference twin. Python stays for the simulators,
   the measurement fit and the study scripts.
4. **The first trick rebuilt as a goal: lifting a front paw on three feet
   and washing the face with it**, at a tempo the random mode can vary.
