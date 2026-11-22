# One movement engine: a proposal

Written 2026-11-22. A design proposal, nothing built; the owner's four
decisions on it are at the end. Numbers marked **(est.)** are estimates;
every other number is read from the code, the docs or the recordings
named next to it, most of them from
[movement-engine-results.md](movement-engine-results.md) (the old walk,
trot, long trot and crawl of 2026-11-17 to 11-21). Nothing here has been
tried on Kilo.

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
only the floor can.

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

- Mind, C++: `kernel/mind/planner/{patterns,plan,walker,trot}.*` (become the
  gait clock and solver); `kernel/mind/intent/intent.*`, `intent.gen.*`,
  `gen_intent.py`, `table.h` (the posture graph and its moves;
  `estimate.*` moves to the observer); `kernel/mind/tricks/{player,library,library.gen}.*`,
  `gen_library.py`; the righting moves in `intent/`.
- Mind, Python references: `mind/kilo/planner/{patterns,plan,walker}.py`,
  `mind/kilo/intent/{graph,moves}.py` and `moves/`, `mind/kilo/tricks/{library,legacy,import_legacy,player,rear_up}.py`
  and `library/`; their vector generators and vector files under
  `kernel/host/tests/`. The engine is written once, in C++, and tested in
  the laptop build of the kernel (decision 3). Kept in Python as tools:
  `dynamics.py`, `righting.py` (the simulators), `measure/` (the fit), the
  study scripts.
- Protocol: `FEET`, `TRICK_BEGIN`, `TRICK_FRAMES`, `TRICK_COMMIT`,
  `TRICK_PLAY`, `TRICK_STOP`, `FALL_RC`. Replaced by one `POSE` message: a
  pose for the board to assume and **the tick by which to reach it**; per
  leg either a foot position (x, z in mm, wider ranges than FEET's ±60 and
  30-95) or two joint angles (for a paw above the body), the head and tail
  angles, the attitude the engine expects there (the board's balance
  reference) and the feet that should carry him: about 22 B payload
  **(est.)**, under 10 % of the line at 50 Hz. The board moves toward the
  newest pose by its tick, slew-limited, and once there holds it with its
  correction on; a late or missing POSE therefore costs nothing but
  freshness, and the mind may send poses further apart where nothing needs
  every tick (a hold, a slow stretch). TELEMETRY gains a "limited" flag per
  leg (a joint or slew limit cut the pose) and, while a pose stream runs,
  the commanded joint angles at a low rate.
- Board: `core/keyframe.cpp` (1176 B of flash in the 2026-10-11 size report,
  and its 24-frame store, 336 B plus header of RAM); rc and the recovery
  states in `reflex`/`body.cpp` (150-250 B of code and about 40 B of table,
  docs/righting.md's estimate); the trick dispatch in `link.cpp`; the
  joint-space balance gains for SIT if every pose is streamed. `feet.cpp`
  grows a little (joint mode, reach-by-tick). **Net (est.)**: about 300 B
  of RAM and 1.5-2 KB of flash freed, against about 1 B of RAM and 3 KB of
  flash spare reported after the righting branch.

## Part four: the path

Every step is tried on the floor with keys he already has, and deletes
what it replaces.

1. **One stream.** `POSE` replaces FEET and the TRICK messages; the board's
   trick player and rc go. The look-ahead drops to 60 ms. The mind's
   existing walker sends POSE in foot mode; the existing tricks and intent
   moves are sampled in the mind and sent in joint mode. Floor try: every
   key looks as before; tricks at the same timing (compare the recordings
   of item 11); after a fall he lies soft and reports, and B1 runs the
   righting experiment's moves through the new stream. *Why first*: every
   later step needs it, it gives the board back the RAM the rest needs, and
   it changes no movement, so a difference on the floor is the transport's
   fault and easy to find.
2. **Gait clock and solver.** Walk, trot and crawl become offsets on one
   clock; `planner/` goes. Floor try: C4, A4, B4, B7/B5 mid-walk; new: the
   tempo changes without a stop, walk to trot without squaring up. Judged
   by the roll numbers `walkstudy.py floor` already computes.
3. **Postures as goals.** Stand, sit, rest from any measured state through
   the solver; `intent/` moves and the graph go. Floor try: B6, A7, the
   random mode's sitting and lying down, from odd starts.
4. **Tricks as goals.** The kept tricks (item 11) rewritten as tasks with a
   tempo; the keyframe library goes. First the paw lift on three feet and
   the face wash. Floor try: A5 twice, A1 (rear up as "nose up to N
   degrees").
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
stop) and will at first be worse at something. RAM on the board stays
tight until step 1 lands, so nothing can be added there before it. Between
step 1 and step 5 he does not get up by himself (decision 1).

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
