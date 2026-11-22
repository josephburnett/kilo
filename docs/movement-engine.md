# One movement engine: a proposal

Written 2026-11-22. A design proposal, nothing built. Numbers marked
**(est.)** are estimates; every other number is read from the code, the
docs or the recordings named next to it. Nothing here has been tried on
Kilo.

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
works out where every foot and joint should be from the model and what the
sensors say right now. A movement is no longer a recording to replay but a
small set of wishes (which feet carry him, where his weight goes, where a
paw goes, at what tempo) that the engine re-solves every tick from where
Kilo actually is. Tempo is a dial, not a property of a recording. Getting
up is steered by the measured roll, not played blind.

The body keeps what must be fast or must work without the mind: the 50 Hz
servo loop, the small balance correction, slew and joint limits, the
watchdog, and fall *detection* (hold soft, report; no rc). It loses the
trick player, rc and the FEET-only format: **one stream message** replaces
FEET and the five TRICK messages, which frees an estimated 300 B of the
body's RAM (it has almost none spare today) and 1.5-2 KB of flash **(est.)**.

The way there is six steps, each tried on the floor and each deleting what
it replaces. **Before step 1, a short list of things to find out** with the
software as it is, most of which an agent can do today from the recordings
in `~/kilo-logs`.

## Part one: what to learn from what we have, before building

**A. Can be analysed today from the recordings (no robot).** The flight
recorder keeps every link frame both ways with the mind's clock
(`kernel/mind/record/recorder.h`), so these need only a study script in
`mind/tests/` like `walkstudy.py`.

1. **How late is everything on the link?** From each walk: when the mind
   sent each FEET, for which body tick, and when each TELEMETRY for a tick
   arrived; the body's `feet_queued` minimum. Already known: FEET reached the
   body 80-108 ms before its tick with a 100 ms target, queue 1-4 ticks, no
   underruns (docs/walking-and-balance.md §16.1). *Answers:* how short the
   look-ahead (now `feet_lookahead_ticks = 5`, 100 ms) can be, and so how
   fast a decision in the mind can reach the servos. That number decides
   what feedback may live in the mind (part 2, "balance").
2. **Does the model's support margin predict rocking?** For every recorded
   walk, crawl and trot: the planned margin each tick (the PLAN records)
   against the measured roll rate. The old walk ran 4.8-7.5 mm outside three
   feet and rocked 4.8° a cycle; the crawl plans +14 mm. *Answers:* how far
   inside three feet the weight must be before the third foot really holds
   it (the open "inset" question of §16.5: 3 mm or 10 mm), which sets the
   margin the engine must keep.
3. **How much does the body's balance loop actually do?** `corr_pitch`,
   `corr_roll` and `BALANCE_HELD` over all runs. The walk study says the
   loop never acted during the walk. *Answers:* whether the body loop
   matters only when standing and on slopes (likely) or also in gaits.
4. **Servo lag against the battery.** Predicted attitude (in FEET) against
   measured, lag fitted per run against `batt_mv`. The model has 50 ms dead
   time + 70 ms time constant; the self-measurement fitted 70 ± 12 and 69 ±
   15 ms (docs/self-measurement-results.md). *Answers:* whether the engine
   can use one fixed lead or must track it live.
5. **How good is the state estimate?** `python -m kilo.intent.replay` over
   every `.krec`, and the nine falls in `mind/tests/fixtures/falls_2026_11.json`:
   agreement, false changes, time to settle. *Answers:* whether the
   estimate is good enough to choose actions from (righting, "from any
   pose"), or needs more than attitude (e.g. what the legs were told).
6. **How fast does he roll in a get-up?** The six floor tries of rc
   (`python -m kilo.tricks.righting --phases`): roll speed per pose, how
   long until still (0.4-1.4 s seen). *Answers:* the time budget of a
   closed-loop get-up: how often the mind can look and change its action.
7. **Do tricks play on schedule, and does the IMU follow the model?** Trick
   runs: `elapsed_ms` against `expected_ticks`, and measured pitch/roll
   against `tricks/model.cpp`'s prediction. *Answers:* whether the model is
   good enough off a plain stance to check a paw-lifting goal.

**B. Needs Kilo on the floor** (with the keys as they will be after the
pending merges; no step is timed).

8. **The new walk** (C4 at each level, B7/B5; docs/walk-trial.md, with a
   side video). The model gives it +0.4 mm of margin at the hand-over.
   *Answers* item 2 on the floor: does a margin near 0 hold or rock?
9. **Self-measurement v2** (C5 twice), two or three runs on the same floor.
   *Answers:* the centre of mass height (the first run hit the fit's limit),
   the feet rectangle, sag under load, and how much these vary between runs,
   which tells how much the engine should adapt them live.
10. **The trot** (A4, B7 up to level 5) with video: slip on the long stride.
    *Answers:* whether slipping is common enough to need detecting.
11. **Tricks in the random mode** (A5, then A5 again for each trick): do
    they look right at the original timing? *Answers:* which moves are
    worth keeping as goals, and whether timing (tempo) is what makes them
    look cat-like.
12. **Falls with hands off**, both get-ups: the built-in rc (a build
    without the righting switch) and the B1 experiment (with it), on each
    side and the back, filmed. *Answers:* which phase of a get-up fails, and
    whether a few model-suggested actions chosen from the measured roll can
    do it.
13. **A slope and a push**: standing (B6) with a book under the two left
    feet, then a gentle push from the side; the same while walking (C4).
    *Answers:* how big the residual is that the body loop leaves, and how big
    the disturbance must be before a step has to change.

## Part two: the architecture

### Layers

```
behaviour, remote, guard   -> GOALS        (a few per second)
engine: goal -> TASK       -> what must hold: support, weight, paw, tempo
engine: tick solver, 50 Hz -> one POSE per tick, a few ticks ahead
body, 50 Hz                -> slew, limits, balance residual, servos
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
    `BodyModel` / `model.h`);
  - **body targets**: height, pitch, roll, fore-aft shift;
  - **end-effector targets** for free limbs: a paw at a point relative to
    the head, the head and tail angles;
  - **limits**: joint speed (the body's 500°/s slew), the servos' load from
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
- **Tricks use the same clock**: "lick a paw" is: contact plan three feet,
  weight 10 mm **(est.)** inside their triangle, the free paw to a point under
  the chin, head tilted down to meet it, a small paw stroke at the tempo.
  "Stretch" is: front feet forward, body low in front, hold. Because these
  are wishes, they are valid from any pose the solver can reach: from
  sitting, a paw trick needs no three-foot shift; from standing it does,
  and the engine makes it.

### Balance: what stays on the body, and why

The body loop is about 4 ms from IMU to servo command plus up to 10 ms of
IMU sample age **(est.)** (docs/architecture.md §2). A loop through the mind
adds: up to 20 ms of tick phase, the TELEMETRY on the wire (1.6 ms), the
mind's own pass, the FEET on the wire, and the look-ahead the stream keeps
against jitter (100 ms today) **(est. in total, 40-130 ms more; item 1
measures it)**. The servos themselves take 50 + 70 ms (model). A tip grows
with an e-folding time of about 65-112 ms (self-measurement, one timed tip).

So:

- **On the body, unchanged in kind**: the residual balance loop (its
  reference is the engine's predicted attitude, so it only sees model
  error), slew and joint limits, the watchdog's hold, settle and rest,
  battery limits, fall detection. These act in the first 100 ms.
- **In the mind**: everything that changes the *plan*: where the next foot
  lands, waiting or hurrying a phase, leaning uphill on a slope, widening
  the stance after a push, slowing on a low battery. These act over the next
  step (200-500 ms), where the link delay is small against the servos'.
- **To decide with data** (item 1): whether the catch must move to the
  body. Recommendation: leave it in the mind unless the measured mind loop
  is longer than about one tip e-folding time.

### Contact and support without foot sensors

Kilo has no foot switches and no servo position feedback. Contact is
therefore a belief: the plan says which feet are down; the observer checks
it against the attitude. A measured tilt toward a foot the plan says is
down means that foot is not carrying (a slip, a hole, a soft floor); a
steady offset means a slope or a model error (the self-measurement's 2.4°
nose-down is one). The existing error estimator (`planner/errors.cpp`:
offset and lag from the telemetry) becomes the observer's, and its
results feed the engine live, within bounds: a learned centre-of-mass
offset shifts the weight target, a learned lag shifts the lead. Slipping
forward can also show in the distance sensor when facing a wall (80 ms
cadence), as progress smaller than commanded; that is a later addition.

### Lying down and getting up

The lying simulator reproduces none of the four recorded rc get-ups
(docs/righting-results.md), so no open-loop sequence designed in it can be
trusted. Proposal:

- A few **actions** the model suggests, each a task like any other: "kick
  with the legs underneath", "throw the legs on top over", "tuck the head",
  "fold", "push up to the belly" (the righting branch's kick and throw are
  the first two).
- A **policy in the mind**, ticked at 50 Hz on the measured roll and roll
  rate: which action for this roll band and side; switch when the roll stops
  improving for a set time (from item 6, roughly 200-300 ms **(est.)**);
  give up after three tries (as the experiment does: limp and a meow).
- **Learning from each try**: log (start roll band, action, outcome). After
  enough falls the policy prefers what worked on Kilo, not what worked in
  the simulator. The simulator only ranks the first candidates and screens
  servo loads.
- **No permanent body reflex**, as decided 2026-11-21: the body detects
  the fall, holds the legs soft and reports; getting up is a goal.

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
  `kernel/host/tests/`. Kept as analysis tools: `dynamics.py`, `righting.py`
  (the simulators), `measure/` (the fit).
- Protocol: `FEET`, `TRICK_BEGIN`, `TRICK_FRAMES`, `TRICK_COMMIT`,
  `TRICK_PLAY`, `TRICK_STOP`, `FALL_RC`. Replaced by one `POSE` stream: per
  leg either a foot position (x, z in mm, wider ranges than FEET's ±60 and
  30-95) or two joint angles (for a paw above the body), the head and tail
  angles, the predicted pitch and roll, the support bits: about 20 B
  payload **(est.)**, under 10 % of the line at 50 Hz.
- Body: `core/keyframe.cpp` (1176 B of flash in the 2026-10-11 size report,
  and its 24-frame store, 336 B plus header of RAM); rc and the recovery
  states in `reflex`/`body.cpp` (150-250 B of code and about 40 B of table,
  docs/righting.md's estimate); the trick dispatch in `link.cpp`; the
  joint-space balance gains for SIT if every pose is streamed. `feet.cpp`
  grows a little (joint mode). **Net (est.)**: about 300 B of RAM and
  1.5-2 KB of flash freed, against about 1 B of RAM and 3 KB of flash
  spare reported after the righting branch.

## Part four: the path

Every step is tried on the floor with keys he already has, and deletes
what it replaces.

1. **One stream.** `POSE` replaces FEET and the TRICK messages; the body
   trick player and rc go. The mind's existing walker sends POSE in foot
   mode; the existing tricks and intent moves are sampled in the mind and
   sent in joint mode. Floor try: every key looks as before; tricks at the
   same timing (compare the recordings of item 11). *Why first*: every
   later step needs it, it gives the body back the RAM the rest needs, and
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
   tempo; the keyframe library goes. First the paw lift on three feet.
   Floor try: A5 twice, A1 (rear up as "nose up to N degrees").
5. **Closed-loop getting up.** The policy of part two; the righting
   experiment goes. Floor try: B1 from each side and the back, then the
   same as a goal the engine sets itself after a fall.
6. **Live adaptation.** The observer's offsets and lag feed the engine
   within bounds; slope lean; a changed step after a push. Floor try: item
   13 again, before and after.

**Main risks.** The mind now owns every movement: a kernel stall becomes a
held pose on the body (the watchdog handles it, but tricks would freeze
mid-air, so the look-ahead must cover stalls). The solver replaces tuned,
floor-tested code (the walk, the quick stop) and will at first be worse at
something. RAM on the body stays tight until step 1 lands, so nothing can
be added there before it.

**Honest unknowns.** How far inside three feet the weight must be (items 2,
8). The mind's real loop delay (item 1). Whether a few actions and a policy
can get him up at all, since nothing we have can predict a get-up (item
12). Whether hobby servos with no feedback follow wishes smoothly enough to
look cat-like, or need the tempo kept low.

## The owner's decisions

1. **Should he stop getting up by himself from step 1 (rc deleted), until
   step 5 brings the new get-up?** Recommendation: yes, with B1 still
   running the righting experiment's moves through the new stream. It
   matches your 2026-11-21 "no fallback" and avoids keeping rc for one more
   step.
2. **Should the small balance correction stay on the body?** Recommendation:
   yes; move the catch there only if item 1 shows the mind is too slow.
3. **Keep writing every engine piece twice (Python reference and C++,
   checked by vector files), or write it once in C++ and test it in the
   laptop build of the kernel?** Recommendation: once, in C++. The Python
   reference made sense when the mind was Python; the simulators and the fit
   stay in Python as tools.
4. **Which cat-like goal should prove the engine at step 4?**
   Recommendation: lifting a front paw on three feet and washing the face
   with it, at a tempo the random mode can vary.
