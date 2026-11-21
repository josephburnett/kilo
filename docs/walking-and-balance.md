# Walking and balance: what we have, why the trot tips, what the model should be

Written 2026-10-13 after the first floor drives (trot at 1000 ms with
balance on: "wobbles so much when walking that he tipped over"; tricks fine).
Numbers use Kilo's measured geometry: legs 45 + 50 mm, body 123 mm long,
feet 65 mm apart, body height 0.80 × 95 = 76 mm, centre of mass about 80 mm
up. Code at the time: `body/src/core/gait.cpp`, `balance.cpp`, `body.cpp`
(the wiring); design: `docs/architecture.md` §3d, §3f. The body's gait
generator is gone since protocol 2.0 (commit a38580e): the mind now plans
the feet (`kernel/mind/planner/`) and streams them to the body, which plays
them (`body/src/core/feet.cpp`).

## 1. What we have

**The gait** is open loop. Each leg follows a fixed foot path in its own
plane (back along the ground in stance, a cosine arc in swing), timed by a
global phase. Nothing about it depends on what the body is doing. It knows
the body height and a commanded pose (pitch, roll) and that is all.

**The balance** is a slow leveller. Once per 20 ms tick it takes the measured
pitch and roll, subtracts its own correction and the commanded pose to get a
"ground slope", and asks for leg-length changes (front/back, left/right)
that would level the body on that slope. Between the sensor and the legs
there is a 2° dead band, a low-pass (about 60 ms), a slew of 1° per tick
(50°/s), a clamp of 15°, and the servos themselves (about 150 ms). The loop
takes roughly 250 ms to act on what it sees.

**The coupling** is one number pair: the balance's pitch/roll corrections
are added to the gait's pose terms, so the stance feet get longer or shorter.
The balance does not know which feet are on the ground, where the gait is in
its cycle, or what the body is about to do. The gait does not know the
balance exists. The balance also cannot tell the body's own rocking (caused
by the gait) from a slope: it levels against both, late.

That design is right for standing on a slope, which is what it was built and
tested for (H13: level within the dead band at Kp 0.99). It was never a
walking stabiliser, and the numbers below say it cannot be one.

## 2. Why the trot tips, in numbers

A trot stands on a diagonal pair of feet while the other pair swings. On a
diagonal pair the body is an inverted pendulum pivoting about the line
between those two feet: nothing holds it up sideways. For a centre of mass
h ≈ 80 mm above the ground the fall grows as e^(t/τ) with

    τ = sqrt(h / g) = sqrt(0.08 / 9.81) ≈ 90 ms.

Every 90 ms on two feet multiplies the tilt by 2.7. The diagonal-only time
per step is (1 − duty) × period:

| trot setting                | two feet only | tilt growth | result (observed)           |
|-----------------------------|---------------|-------------|-----------------------------|
| legacy, ≈250 ms, duty 0.70  | ≈75 ms        | × 2.3       | shuffles, stays up          |
| 800 ms, duty 0.55 (P2 test) | 360 ms        | × 55        | fell in 3 s                 |
| 500 ms, duty 0.55 (evening) | 225 ms        | × 12        | wobbled, fell once          |
| 1000 ms, duty 0.70 (now)    | 300 ms        | × 28        | wobbles, tips over          |

A 1° lean at lift-off becomes 28° by touchdown. The next pair catches it,
over-corrects, and the rock grows: that is the wobble. The balance loop,
at ≈250 ms from seeing to acting, is three times slower than the fall; and
on a diagonal pair, changing the lengths of the two stance legs does not
produce a torque about the line they stand on. Through the gains it does
not matter: no gain on this loop stabilises a 300 ms diagonal stance. The
balance can only level the body during the four-foot phases, and with its
delay it is often pushing the wrong way by the time the legs move, which is
the "balance may feed the rocking" of the P2 results.

**So slowing the trot down made it less stable, not more.** Period is the
wrong knob. The thing that must stay short is the time on two feet.

## 3. What "speed" should change

Keep the swing short and fixed, and get speed from the stride and the
stance time:

    swing time   T_sw  fixed, 150–200 ms (short enough for τ = 90 ms, long
                       enough for the 10°/tick servo slew: a 25 mm stride
                       in 150 ms is about 6°/tick at the shoulder)
    stance time  T_st  = stride / speed
    period       = T_sw + T_st,   duty = T_st / period

At walking pace (say 50 mm/s, stride 25 mm) that is a 500 ms stance, period
650 ms, duty 0.77. At a crawl (10 mm/s) a 2.5 s stance, duty 0.94: a slow
gait with quick steps and long four-foot phases, which is what the stock
firmware's shuffle looks like. Speed then modulates continuously, and the
two-foot time never grows. The current remote maps the key to a period;
that mapping is what the mind should change (the body already accepts any
period/duty per goal, with phase continuity).

Walk and crawl (one leg at a time, three feet down) have no diagonal phase
and can be as slow as you like, if the centre of mass is inside the
triangle of the three stance feet. On a cat 65 mm wide that triangle's edge
runs right under the body's centre line, so an unmoved body sits on the
edge: marginal. That is where sway comes in (section 4).

## 4. The model that should exist

One shared state, written by the gait and read by the balance:

- **Support state**: which feet are on the ground this tick and where they
  are (the gait knows: stance/swing per leg, foot x and z). From it, the
  support polygon (four feet, a triangle, or a diagonal line).
- **Expected attitude**: what the body will do because of the gait itself
  (the planned sway and the bob), so the balance corrects deviations from
  the plan, not the plan.

And two couplings instead of one:

**Gait → balance.** The balance becomes stance-aware. It corrects only
through feet that are down. Its reference is the planned attitude, not 0.
Its "slope" estimate is frozen while on two feet and updated on four. Its
delay chain (dead band, low-pass, slew) is sized for levelling, which is
fine; it is simply not asked to catch falls.

**Balance → gait.** Three things the gait does for balance:

1. **Sway (feedforward).** Before a foot lifts, roll the body toward the
   feet that stay down, and pitch toward the end that keeps both feet. On a
   cat whose legs move only fore-aft (two servos per leg, no sideways foot
   motion) rolling the body is the only way to move the centre of mass
   sideways: 10° of roll moves it about 80 mm × sin 10° ≈ 14 mm, from the
   triangle's edge to well inside it. The gait already has the pose terms to
   do this (the same leg-length channel the balance uses); what is missing
   is a roll/pitch pattern locked to the leg phases. This is what makes a
   slow walk statically stable, and what lets "balance on" and "walking" be
   one thing.
2. **Early touchdown (feedback).** If the body starts tipping toward the
   swinging side faster than a threshold, end the swing now: put the foot
   down where it is. A foot on the ground is the only thing that stops a
   sideways fall on this cat; nothing else can. This is a reflex at tick
   rate (20 ms), inside the fall's 90 ms, and it is what the fast legacy
   trot gets for free from its 75 ms swings.
3. **Forward placement (capture point).** Fore-aft falls can be caught by
   landing the swing foot further ahead or behind, by about
   τ × forward velocity of the tilt. The legs can do this (it is the one
   axis they move in). Lower priority: pitch falls are rarer here than roll
   falls because the body is long and narrow.

**Terrain** (the goal: balance on all the time, uneven ground) falls out of
the same state. A swing foot that meets the ground early, or late, changes
the measured attitude at touchdown; the stance-aware balance then adjusts
that leg's length in the next four-foot phase, which is the slope
correction we already have, applied per step. Without foot contact sensing
(Kilo has none) the touchdown height is a guess; the IMU jolt at contact
and the attitude change after it are the available signals, and the early
touchdown reflex above doubles as "stop pushing when the foot has found
ground" if it also watches pitch.

## 5. What this means in practice, in order

*Since 2026-10-14 (protocol 2.0) the steps are planned in the mind and the
firmware's gait generator is gone (sections 9 and 10). Items 2 to 6 below
were built into that generator and left with it; what became of each in
the mind: the speed mapping (2) is `kilo.planner.patterns`; the sway (3)
became the planner's lean, a move of the whole body over the three feet
that stay down, finished a servo lag before each lift; the stance-aware
balance (4) is the body's loop with the plan's prediction as its reference,
holding while any foot is up; the early touchdown (5) is the planner's
catch from the error the body reports; forward placement (6) was not
carried over (the plant showed no benefit). The mind's tests of all this
are tests/test_planner.py and tests/test_walk_sim.py (section 10).*

1. **Parameters only, next drive** (no code): a trot with a fixed short
   swing. Period 600 ms, duty 192 (0.75): swing 150 ms, four-foot phases of
   150 ms, stride 20–25 mm. Expect much less wobble than 1000/0.70. This is
   a test of section 2, not a fix. Measure: roll amplitude per cycle from
   telemetry (`kilo monitor`), held in the hand and on the floor.
2. **Speed mapping in the mind** (small): the remote's speed keys and
   `kilo walk` set stride and stance time with a fixed swing, instead of a
   period. Continuous speed, two-foot time constant.
   **Done (2026-10-13)**: `kilo.body.client.gait_timing(pattern, speed)`,
   `kilo walk --speed MM_S`, and the remote's slower / faster keys (C5 / C1
   then, 0.5x to 2x of the default speed; today B5 / B7 and the levels in
   `kernel/README.md` "The remote's keys"). Each pattern keeps its default's swing time
   (trot 150 ms; walk 500 and crawl 297 ms, which their higher steps need
   under the slew limit), stance = stride / speed, period = swing + stance.
   Below the longest stance (duty 230/256 or period 2000 ms; the trot's is
   1327 ms) the stride shrinks to as little as 8 mm; above the default
   stride at the shortest stance (duty 0.65 trot, 0.75 walk/crawl) the speed
   is capped. Trot 6..90 mm/s, default 56 (600 / 192 / 25 mm). The trot's
   0.65 keeps three ticks on four feet, which the stance-aware balance needs
   to run at all (review 2026-10-13).
3. **Sway in the gait** (firmware, moderate): planned roll/pitch locked to
   the leg phases, amplitude from the geometry (about 8–12° roll for the
   walk, less for the trot), ramped with the stride. Makes slow walk and
   crawl statically stable with balance on. *Done 2026-10-13* (§3d "Sway",
   not yet on the floor): SET_GAIT flag `SWAY`, on by default for walk and
   crawl from the mind; 8° roll and 4° pitch away from the lifting leg at
   its mid-swing (WALK peaks at 12° between its two same-side swings), 0
   for the trot by construction. The gait also publishes the support state
   (`Gait::down`, feet on the ground) and the planned attitude
   (`plan_pitch`, `plan_roll`) for item 4.
4. **Stance-aware balance** (firmware, moderate): reference = planned
   attitude; correct through stance feet only; hold the slope estimate on
   two feet. *Done 2026-10-13* (§3f "Stance-aware under the gait", not yet
   on the floor): in a host plant that falls on a diagonal, the walk with
   sway at Kp ≈ 1 now follows its plan within 1° with corrections ≤ 1.8°
   (they were 10.8°, fighting the sway), and on a trot the corrections hold
   on two feet and 2 ticks after (measured: the lean per step stops
   improving past 2) while a 10° slope is still levelled to 2.65° at every
   lift-off.
5. **Early touchdown reflex** (firmware, small once 4 exists): tipping
   toward the swing side ends the swing. The piece that lets the trot
   survive a misstep or a bump. *Done 2026-10-13* (§3d "Early touchdown",
   SET_GAIT flag `CATCH`, on by default, `kilo walk --no-catch`): at 90°/s
   toward a swinging leg; in the plant normal walking peaks at 40°/s (walk)
   and 65°/s (trot on a 5° slope) and never triggers. Since the same day it
   waits for a second tick over the threshold and lowers the caught foot
   over a few ticks with its x held; a 60°/s shove on the trot then catches
   the leg on the falling side late in its swing, and the lean is 12.2° vs
   15.5° in the plant that counts a foot down on its command, unchanged
   (19.6°) when the foot must first reach the ground through the servo lag
   (the earlier 7.3° assumed a caught foot carried the body at once).
6. **Forward placement and terrain** later, on the same state.
   *Forward placement done 2026-10-13* (§3d "Forward placement", SET_GAIT
   flag `PLACE`, off by default until a floor run compares `kilo walk --place` with the default): at a lift-off the
   leg's touchdown moves 7 mm per rad/s of pitch rate (τ·h), forward on a
   nose-down rate, clamped to 3 mm (the trot's 150 ms swing has no more
   slew to give). In a capture plant with a rigid stance it neither helps
   nor hurts the pitch after a fore-aft shove (three or four feet stop the
   shove before any foot lifts); normal walking shifts the feet by at most
   1.25 mm. Terrain: not started.
   Also from the plant (**fixed** 2026-10-13, the "sway finding" of §3f):
   the walk's sway crossed zero at the lift-offs where it changes sides, so
   with the servo lag the body leaned up to 3.8° toward the lifting leg; the
   sway now runs 225 ms ahead (`[gait] sway_lead_ms`) and the body leans at
   least 4.0° (walk) and 7.5° (crawl) away from every leg as it lifts.

Each step is testable in the hand first (legs in the air: cadence, servo
slew, sway pattern) and then on the floor with the roll trace as the
number that must go down.

## 6. What stays as it is

The leveller itself (controller, gains, sign conventions, the geometric
pose terms) is correct and measured; it is the four-foot and standing part
of the model, and in a walk it trims what the plan did not predict. The
IK and the leg-length pose terms stay in the body. The gait's phase
machinery and foot paths moved to the mind (kilo.planner.plan) as slots of
a shift and a swing with the same cosine foot paths. The legacy firmware
never had any of this: it stayed up by being fast. The mind's test of the
whole loop is the virtual body's physics (tests/test_walk_sim.py): the
planned walk follows its plan within 2.5°, the same steps without the lean
tip over the edge every swing.

## 7. Measured on the floor, 2026-10-13 16:50 PDT (step 1 of section 5)

Remote drive, balance on, firmware 74347362 (no sway yet), the mind at
df578c4 (trot period 600, duty 192, stride 25, step 8 = 150 ms on two feet).
Roll from `kilo monitor --json` at 25 Hz (`~/kilo-logs/monitor-20261013-164821.jsonl`):

| what                          | roll (deg)       | pitch (deg)  | result                                   |
|-------------------------------|------------------|--------------|------------------------------------------|
| standing before               | 0.6 .. 1.0       | 2.5 .. 2.7   |                                          |
| trot forward, 5 s             | -1.5 .. 3.4 (sd 1.2) | 0.4 .. 2.7 | "much less wobble, stayed on his feet"   |
| trot backward, 6 s            | -3.4 .. 3.6 (sd 1.6) | 0.2 .. 4.5 | walked                                   |
| crawl, marching in place (x2) | +9.8 then -124   |              | FELL to the left within 2 s, both times  |

The trot at 150 ms on two feet holds within ±3.5° of roll where the 300 ms
trot reached -16.5° and fell (P2 results, 800 ms / 0.55). Section 2's
arithmetic holds on the robot.

The crawl fell as section 3 predicts for a three-foot gait with no sway:
the centre of mass sits on the edge of the support triangle, and a 300 ms
swing (period 2000, duty 0.85) with the body low and the step 18 mm high
tips it. That is what the sway (section 4, item 1; merged 405d139, not yet
flashed) is for. Until it is flashed, crawl and walk are expected to tip;
the trot is the gait to drive with.

## 8. Measured on the floor, 2026-10-14 (the sway, flashed as 56768228)

Full table in docs/p3-hardware-test-results.md. The short version: the walk
pattern fell within 2-3 s with the balance on, sway on or off, every time
(four runs), always with the same growing 3 Hz side-to-side swing; with the
balance **off** it rocked ±15° and stayed up for the whole 6 s. The trot
held in every run. So the balance loop is unstable on three feet (the body
tips about the far diagonal in under 0.1 s, the loop answers 0.25 s later,
after the body has turned round), and section 4's item 1 as built, a roll
reference for that loop, cannot work: it asks the unstable loop for more.

## 9. Direction agreed 2026-10-14: the mind plans, the body plays and corrects the residual

Discussed with the owner on the evidence above. Four parts:

1. **A model of the body in the mind.** Masses and where they are (body,
   battery, leg segments, later the Pi): the centre of mass for any leg
   configuration. Servo speed under load and lag (assumed 150 ms, never
   measured). From these, how fast a tip grows (today's traces: 30° in
   0.2 s) and the lead the servos need. To measure: weight per foot on a
   kitchen scale, with and without the battery; a servo step test with the
   IMU recording.
2. **The mind plans every step through the whole body.** Not "lift a leg
   and hope": the plan is a path for the centre of mass that stays inside
   the support that remains, with the body already moving before the lift
   (the way a person walks: the fall is planned and the next foot is on its
   way). The mind streams foot positions and the predicted attitude for
   each 20 ms tick, about five ticks (100 ms) ahead. It replans from the
   body's reported actual position, never from what it last asked for.
3. **The body keeps a self-contained correction loop.** Its reference is
   the predicted attitude for this tick, so it sees only model error. It
   always moves from where it is toward the next point at a limited rate
   (no snap). It keeps its own limits that no plan can override: servo
   slew, fall reflex, the link watchdog (a gap of ~100 ms slows to a stand
   on the current feet; 500 ms stops the gait as now).
4. **Error telemetry every tick**: predicted attitude, measured attitude,
   the correction applied. A steady offset = mass or IMU placement to
   learn once; a lag between predicted and measured = servo lag, which
   drifts with battery voltage (track it against batt_mv and adjust the
   lead); a spike = a bump, a slope, a slipped foot: the body has handled
   the first 0.1 s, the mind decides whether to carry on, slow down, widen
   or stop.

Consequence: the gait generator leaves the firmware (which also frees the
flash and RAM the plan buffer needs; the firmware has 4 bytes of globals
free today). The firmware becomes a pose player with reflexes and the
residual loop. Can the Pi keep up: a 50 Hz Python loop on the Pi 2 with
the daemon running fired 1500 ticks with the latest 0.44 ms late (20:49);
a 10-minute run under SD-write and Wi-Fi load had its worst tick 9.6 ms
late, none later than 20 ms (docs/p3-hardware-test-results.md section 6). The per-tick work (4 legs of IK plus the
mass model) is well under 1 ms; a frame of foot positions plus predicted
attitude is ~35 bytes, ~1.5 ms on the wire. The 100 ms lookahead in the
body covers garbage-collector pauses and SD stalls; if Python itself
becomes the limit the planner can be rewritten in Rust (owner).

Not started: the owner asked to discuss before any balance work, and did.

**Body side, 2026-10-14 (protocol 2.0, branch fw/stream).** The gait
generator left the firmware; the body plays the mind's `FEET` stream
(docs/architecture.md §3d) and its balance takes the predicted attitude as
the reference and corrects through the feet on the ground (§3f). The host
plant now tips: a point mass on the four legs, standing on the feet whose
triangle holds it and turning about the nearest edge as an inverted
pendulum when they do not (2 to 30° in 180 ms, the floor's ~0.2 s). In it a
walk that rolls the body 10° away from each lifting side stays up with
corrections of 1.4°; the same steps without the roll sink 8° toward every
lifting foot. One finding changed the body: a three-foot stance shoved
onto its lifted foot leaned 19.2° with the loop updating on three feet and
12.9° with it off, because the corrections act through the two feet the
body pivots on. The body now holds its corrections whenever a foot is up;
the loop trims on four feet only, and the plan's lean has to hold the mass
on three. The mass model, the servo step test and the planner are still
the mind's to do.
Next concrete steps, in order: weigh Kilo per foot; servo step test; put
both into the host plant (which today treats three feet as safe, which is
why the sway passed there); then the protocol change (streamed poses,
error telemetry) and the planner.

## 10. The planner, as built (the mind, 2026-10-14)

Code: `mind/kilo/planner/` (model.py, patterns.py, plan.py, errors.py,
walker.py); the virtual body it is tested on: `mind/kilo/vbody/feet.py`
and `physics.py`; the C++ port that walks Kilo: `kernel/mind/planner/`,
driven by the remote (`kernel/README.md` "The remote's keys").

**What it computes, every 20 ms tick.** Where each foot is (fore-aft x and
depth z under its shoulder), which feet are on the ground, and the
attitude the body will have. The pattern's legs take turns in slots: a
*shift* with all four feet down, then a *swing*. Feet on the ground slide
back at the stride speed; a swinging foot lifts on a cosine arc to half a
stride ahead of its shoulder. Speed changes move the stride (0.5 mm a
tick) and the ground time (2 % a tick) gradually; a pattern change takes
every leg home first (strides to 0, a step each, as a stop), then starts
the new pattern from its first lift. For the walk and the crawl, before every lift
the planner solves for the smallest move of the body (a fore-aft shift of
every foot, a roll and a little pitch by making legs longer or shorter)
that puts the centre of mass at least 14 mm inside the triangle of the
three feet that stay down, at lift-off and at touchdown, and moves the body
there during the shift: done a servo lag before the lift (since 2026-11-15:
the lift starts 60 ms before it is done, §11), held until the foot is down,
dropping a side no faster than gravity can follow. That solve
is spread over the swing before the slot, a few pieces a tick (on this
desktop Planner.next() costs at most ~0.3 ms a tick, where one tick at each
walk slot's start cost 2.7 ms, 43 ms on the Pi 2); after a change
mid-swing it is redone at the slot's start while the plan waits. The trot
stands on diagonal pairs for 150 ms and does not lean. The predicted
attitude for a tick is the plane through the support feet the plan
commanded a servo lag earlier; it goes to the body as the balance
reference (kept inside FEET's ±30°). From the body's reports: no lift
while the body is more than 5° off the plan (the plan waits with every foot
where it is, up to 0.5 s); a swinging leg lands at once
where it is when the body tips toward it by 7°. A stop takes every leg home
in one more step, levels the body and hands over to the STANCE posture.

**The parameters, and which are estimates.** Measured: leg 45 + 50 mm,
foot disc radius 6 mm, shoulders 123 mm apart, track 65 mm. Estimates
(BodyModel, one place): body 700 g with its centre 5 mm ahead of the
middle and 5 mm above the shoulder axles, battery 150 g 15 mm below,
upper leg 20 g (its centre at 0.8 of the link: the knee servo), lower leg
8 g, the Pi not carried; 962 g in all; servo lag 150 ms. Chosen: margin
14 mm (10 until the masses are measured would have left the plan with a
plausible model error, below), shift at most 18 mm, roll 14°, pitch 6°, lift gate 5°, catch 7°,
patterns and swing times as in mind/README.md. Replace the masses with the
per-foot scale readings and the lag with the servo step test.

**The stream's timing.** The walker (in the C++ mind `Walker::poll()` on a
20 ms deadline grid, `kernel/mind/planner/walker.h`; in the Python reference
a thread) wakes every 20 ms. The body's clock is estimated from the
TELEMETRY arrival times (50 Hz while walking): body time = local time + the
largest (tick x 20 ms - arrival) of the last 2 s, i.e. the least delayed
frame; the window restarts when five frames in a row are more than a tick
below that (the body's tick fell back after a stall over 40 ms). Each wake sends every planned tick from the last one sent up to the
body's tick + 5 (100 ms ahead), so a late wake (a garbage-collector pause,
an SD stall) catches up in one go; a tick already played is planned but not
sent. The body queues 8 ticks and holds the last pose on an underrun, then
settles (lifted feet onto the plane of the three support feet), and IDLE
STANCE crossfades the legs home. The thread never writes to disk (its records go to the frame
log's queue), freezes the existing objects for the garbage collector and
raises its young-generation threshold while it runs, and sends nothing
while the mind's main loop looks dead. FEET goes out at once on the
caller's thread (SerialLink.send_stream): no ack, refresh or retry, outside
the goals' epoch machinery; no SET_POSTURE goal is latched while it
streams (a refresh would end GAIT on the body).

**What the virtual body showed.** In its physics (a rigid body on point
feet, servo lag, tipping over an edge as an inverted pendulum) the planned
walk and crawl follow their plan within 2.5° with the body leaning 6-9°
on purpose; the same steps without the lean put the mass up to 4 mm past
the edge and the body tips 9-11° onto the swinging foot every swing (the
rigid catch stops it short of a fall; on the floor the balance loop of the
old firmware took it over); the trot stays within 2°; speed, direction and
pattern changes mid-walk stay within 4°; a centre of mass 10 mm ahead and
5 mm left of the model's with 50 g more battery stays up (with a 10 mm
margin the walk leaves its plan by up to ~12° in some directions and the
catch holds it; the 14 mm margin now the default keeps every direction within 3°); a 60 ms stall of the producer is
caught up with no underrun; a slower servo (200 ms) and a tilted IMU show
in the error estimates (lag +40..60 ms, the offset). The real body is the
next test.

## 11. Faster gaits (2026-11-15)

*Since 2026-11-16 the walk is a different gait, the walk and the trot have
a fixed cadence and the planner's servo model is a dead time and a lag
(§12). What follows is the state of 2026-11-15.*

The owner, after the first planned walks on the floor: "Walk and crawl are
too slow. Placing the feet super delicately." The gaits were retuned in the
virtual body (tests/walksim.py) and the defaults raised about 2.5 times:

| pattern | before: stride, swing, cycle, default (top) | now: stride, swing, cycle, default (top) |
|---|---|---|
| trot | 25 mm, 150 ms, 600 ms, 55.6 (92.6) mm/s | 35 mm, 150 ms, 393 ms, 144 (152) mm/s |
| walk | 30 mm, 500 ms, 3200 ms, 11.1 (12.0) mm/s | 40 mm, 300 ms, 2550 ms, 17.8 (21.5) mm/s |
| crawl | 25 mm, 297 ms, 2800 ms, 10.0 (13.2) mm/s | 50 mm, 300 ms, 2550 ms, 22.2 (26.9) mm/s |

The remote's levels are now 0.4, 0.7, 1, 1.1, 1.2 × the default, so level 1
is about the old default.

**What limits each gait.**

- *The servos.* The body slews a joint at most 10° per 20 ms tick (500°/s,
  `SLEW_MAX_A16`, about the servos' unloaded speed, an estimate). The
  fastest joint of a swing is set by the foot's travel over the swing
  time; the old trot asked up to 355°/s and kept up on the floor
  (2026-11-14, 45 s). Every new swing stays under about 425°/s (the swinging leg carries
  no load); the feet on the ground ask at most 150°/s (trot) and 35°/s
  (walk, crawl).
- *The trot* is limited by the servos alone: a longer stride or a shorter
  swing passes 425°/s, and a longer swing leaves the body on two feet
  longer, where it tips. Its top uses two ticks on four feet (was three).
- *The walk and the crawl* move the body over the three feet that stay
  down before every lift. Twice a cycle that move rolls the body from one
  side to the other (about 17°), and a side drops no faster than gravity
  lets it, so the move takes about 300 ms; four moves and four swings make
  a cycle of at least 2.2 s. The speed is then the stride over that time,
  and the stride is bounded by the legs' reach (the walk's body is 10/256
  lower than before so that its lean does not reach past the IK) and by the
  servos (the longer the stride, the faster the swing). The lift now starts
  60 ms before the move ends, in its slow tail (under 1° of roll left; the
  plan's margin counts it and stays at least 11.8 mm of the 14 asked); the
  lean and the lift go through the same servo lag, so the old wait of a
  servo lag after the move bought nothing the simulation could see.

**What the virtual body shows** (every level, forward, back and both
turns, servo lag 100 / 150 / 200 ms with the model at 150, the centre of
mass 10 mm ahead or behind and 5 mm left or right with 50 g more battery,
the IMU 1.5° and 2° off, a 60 ms stall of the stream): no fall, no
underrun, no late FEET. The walk and the crawl stay within 3.5° of the
plan with the exact model and within 7.2° with the model errors (their
lean is 12-14° on purpose), with no catch and no gate hold. The trot at
its default and top stays within 5.1°. At 75 to 140 mm/s (four-foot
phases of 4 to 8 ticks) the trot rocks with the body's balance loop, worst
walking backward (up to 10° off the plan, gate holds); with the balance off
the same steps stay within 3°. The old default trot (56 mm/s) sat at the
edge of that band too. The remote's level 2 (101 mm/s) is in it.

**What it cannot show.** It is a rigid body on point feet: no foot slip,
no servo torque sag with a low battery, no backlash, no servo that slows
under load, and on three or four feet the body follows the legs exactly
(a gravity-paced move is not checked against inertia). The servos' speed
under load and their lag were never measured; a slower servo would show as
error at the lift (the gate holds) and as catches.

## 12. Walking faster, from the first recordings (2026-11-16)

The owner, after run 4 (docs/fast-boot-first-boot-results.md): "The trot was
better. Could try faster. Cats can definitely walk faster than that." The
walk and the crawl of §11 lean the whole body over three feet before every
lift, about 17° from side to side, which caps them at 21 to 27 mm/s and
looks careful rather than like walking.

What changed, in short:

| gait | before (2026-11-15) | now |
|---|---|---|
| crawl | one leg at a time with the lean, 2..26.9 mm/s, default 22.2 | the same gait, the careful one; only the prediction changed |
| walk | the crawl's sequence with the lean, 2..21.5 mm/s, default 17.8 | a new gait: diagonal pairs, the hind leg 60 ms ahead of its fore leg, no lean, 26..97 mm/s, default 80 |
| trot | diagonal pairs, 6..152 mm/s, default 144, rocks at 75..140 | the same pairs at a fixed cadence (one tick on four feet), 42..184 mm/s, default 144, no rocking band |

The remote's levels (× the gait's default) are now 0.4, 0.7, 1, 1.15, 1.25
(were 0.4, 0.7, 1, 1.1, 1.2). In mm/s:

| level | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|
| crawl, before | 8.9 | 15.6 | 22.2 | 24.4 | 26.7 |
| crawl, now | 8.9 | 15.6 | 22.2 | 25.6 | 26.9 (its top) |
| walk, before | 7.1 | 12.4 | 17.8 | 19.6 | 21.3 |
| walk, now | 32 | 56 | 80 | 92 | 97 (its top) |
| trot, before | 57.6 | 100.8 | 144 | 152 (capped) | 152 (capped) |
| trot, now | 57.6 | 100.8 | 144 | 165.6 | 180 |

So the ladder goes up from slow: crawl 9 to 27, walk 32 to 97, trot 58 to
180 mm/s. A change between the walk and the trot now takes under a second
(before, every leg stepped home first: up to 5.5 s out of the walk; the
owner waited about 7 s for the trot in walk 1 of run 4).

### 12.1 How cats and small robots walk (the research)

**[E]** marks what is established (peer-reviewed or a textbook), **[L]**
hobby lore or a single source I could not check.

- **The cat's walk** [E]: about 0.72 m/s; a cycle of 0.71 s (1.4 Hz);
  swing 0.25 to 0.29 s; duty factor (the fraction of the cycle a foot is
  down) 0.60 to 0.64. Klishko et al. 2014, J Neurophysiol 112:1376,
  https://pmc.ncbi.nlm.nih.gov/articles/PMC4137248/ . Cats walk in the
  lateral sequence (a hind foot, then the fore foot on the same side), the
  footfalls often about evenly spaced, and change to a trot near 1 m/s,
  Froude number 0.5 (Fr = v² / g L, L the leg length): Bishop, Pai &
  Schmitt 2008, PLoS ONE 3:e3808. As speed rises the stance shortens and
  the swing stays about the same (Goslow, Reinking & Stuart 1973, J Morphol
  141:1, as cited by others; not read in the original).
- **Dynamic similarity** [E]: animals of different sizes at the same Froude
  number use similar duty factors and relative strides (Alexander & Jayes
  1983, J Zool 201:135; full text not read). For Kilo, L ≈ 0.08 m, the
  cat's walk (Fr ≈ 0.26) would be 0.45 m/s with a 0.44 s cycle and a 0.18 s
  swing. Kilo's servos cannot do that (below); the point is that a cat-like
  walk is quick and short-cycled, not a slow lean-first crawl.
- **Static stability** [E]: only the crawl in the lateral sequence with a
  duty factor of at least 0.75 keeps the centre of mass inside the support
  at every instant (McGhee & Frank 1968, Math Biosci; the stability margin
  is the distance from the centre of mass to the support polygon's edge).
  Any walk with a duty factor under 0.75, the cat's 0.6 included, balances
  dynamically. Hildebrand (1965, Science 150:701; 1980, Am Zool 20:255)
  puts all symmetrical gaits on one map, by how far the fore foot lags the
  hind foot of the same side: 0 % a pace, 25 % evenly spaced, 25 to 50 % a
  walk with diagonal couplets (a "trot-walk"), 50 % the trot, over 50 % a
  diagonal sequence (as primates walk).
- **Dynamic criteria** [E]: the zero-moment point (Vukobratović & Borovac
  2004, Int J Humanoid Robotics 1:157; cited from memory, not fetched) and
  the capture point, x + ẋ √(h/g) (Pratt et al. 2006, Humanoids). For h =
  80 mm, √(h/g) = 90 ms: a body moving sideways at 0.1 m/s needs its
  support 9 mm further out. Hirose's group keeps the zero-moment point on
  the diagonal line in a trot by swaying the body, and blends a crawl into
  a trot with four-foot phases as speed rises (Kurazume, Yoneda & Hirose,
  ICRA 2001, https://robotics.ait.kyushu-u.ac.jp/kurazume/en/papers/ICRA01.pdf).
  LittleDog's slow crawl plans the zero-moment point first and the centre
  of mass from it (Byl et al. 2009). Kilo can sway sideways only by rolling
  (its legs move only fore and aft), which is what the old lean did, slowly.
- **Petoi's own gaits** [my reading of the source, a rough method]:
  OpenCat's `src/InstinctBittle.h` (https://github.com/PetoiCamp/OpenCat):
  Bittle's `wkF` (walk) is 39 frames, duty about 0.62 to 0.67, lateral
  sequence, the fore foot lagging its hind foot by about 0.43 of the cycle:
  a walk with diagonal couplets, close to a trot. `trF` is a walking trot
  of duty about 0.6. Nybble's `wkF` swings the diagonal pairs almost
  together. Petoi's walk is not statically stable either.
- **Speeds** [L]: Petoi's FAQ gives "2 body lengths per second, or 40 mm/s"
  as a safe speed (the two figures disagree), and 3 to 4 body lengths per
  second in an unreleased mode. I found no independent measured speed for
  Nybble or Bittle. Stanford Pupper's trot settings: 0.15 s swings, 0.1 s
  four-foot overlaps, duty 0.7 (settings, not measurements).
- **Servos** [vendor sheets]: an MG90S is rated 0.1 s per 60° (600°/s) at
  4.8 V with no load; under load the speed falls roughly in proportion to
  the torque. Petoi quotes about 860°/s for its own servos [L]. Kilo's body
  slews a joint at most 500°/s (10° a tick); the floor runs asked up to
  414°/s.

### 12.2 The virtual body against the recordings

**How.** Each recording holds every FEET sent and every TELEMETRY. The
recorded FEET were fed tick by tick through the virtual body (its FEET
player and physics) two ways: with the body's own recorded corrections in
place of the virtual balance loop (this tests the physics alone), and with
the virtual balance loop running (physics and loop together). The
measured attitude was compared with the virtual one, the constant pitch
offset removed. The tool: `mind/tests/walkreplay.py`.

Roll rms, measured minus virtual (degrees):

| servo model in the virtual body | walk 3 (walk, crawl), recorded corrections | walk 3, virtual loop | walk 1 (crawl, walk, trot), virtual loop | walk 2 (trot turning), virtual loop |
|---|---|---|---|---|
| none: the plan's own prediction | 2.92 | 2.92 | 2.58 | 2.09 |
| before: a pure 150 ms delay | 1.73 | 2.39 | 2.18 | 1.67 |
| the best pure delay, 140 ms | 1.55 | 2.26 | 2.10 | 1.57 |
| **now: a 50 ms dead time, then a 70 ms first-order lag** | **1.22** | **1.34** | **1.52** | 1.98 |

A grid of dead times from 40 to 140 ms and time constants from 0 to 100 ms
put the best fit at 40 to 60 ms and 60 to 80 ms; the mean delay is then
about 120 ms (the cross-correlation's 140 ms is its peak, not where the
motion starts). Pitch rms is 0.4 to 0.9° in every case. The body model
(`servo_lag_ms` 50, `servo_tau_ms` 70), the virtual body's physics and the
prediction sent with FEET all use it now.

**The extra roll toward a lifted hind leg** (2.4 to 2.9° in the
recordings) turned out to be the body's own balance correction. In each
big lean move (from about -12° to +6° before RB, and the mirror before
LB) the measured roll started moving about 100 ms after the command,
ahead of the pure 150 ms prediction, by up to 3°. That passed the loop's
2° dead band, so the loop built a correction of 4 to 5° against the move
during the four-foot shift, then held it, as it does whenever a foot is
up, through the hind leg's swing: the body stood about 4° short of its
planned lean, toward the lifted leg. With the recorded corrections
replayed, the virtual body reproduces it (the mean residual with LB up
-0.8°, with RB up +0.8°, against +2.4° and -2.9° for the plan); with the
fitted servo model and its own loop, too (+0.6°, -0.7°). It is not servo
sag, the centre of mass or the foot contact. The fore legs show less of it
because the moves before them are smaller. The new prediction follows the
servos' fitted response, so the error during the moves should stay inside
the dead band and the correction should not build. In the virtual body
(the crawl, the fitted physics, old against new prediction): roll rms 1.35
→ 1.09° forward and 2.60 → 1.82° backward; largest error 3.8 → 3.3° and
6.1 → 4.4°. On the recorded data alone (the measured roll against the old
and the new prediction of the same plans) the gain is small, 2.56 → 2.46°
and 2.90 → 2.74°, because those recordings already contain the
corrections the old prediction caused; the floor has to show the rest.

**The pitch offset** (+1.7 to +2.0° measured, nose down) is about 2.6 to
2.9° once the body's own mean pitch correction (-1 to -1.3°) is counted.
On four feet the attitude is set by the legs alone, so a centre of mass
further forward cannot cause it. It is either the IMU mounted nose down or
the front legs standing about 3 mm shorter than commanded (calibration, or
sag under the 56 % of the weight they carry). These recordings cannot tell
the two apart (a check is in §12.6).

**The trot's rocking is not identified.** On the floor the trot rocks ±4
to 5° at the stride frequency (roll rms 1.4° forward, 2.1° turning),
tipping during each diagonal stance toward the fore leg in the air: the
direction the centre of mass predicts (7.5 mm ahead of the middle, 3.5 mm
from each diagonal). The virtual body rocks 0.6 to 1.1°: it underestimates
two-foot tipping about twice. A centre of mass 10 mm further forward
reproduces the turning recording's amplitude but makes the forward one
worse; changing its height does not help. Turning in place makes the feet
scrub sideways (the legs move only fore and aft), which the virtual body
does not model. So the evaluation below uses the wrong-centre-of-mass
cases (10 mm ahead or behind and 5 mm to a side, with 50 g more battery)
as the stand-in for a body that tips more than the model.

**Did the servos keep up in the trot at 144 to 152 mm/s?** These
recordings cannot say: no JOINTS were streamed (and the servos report no
position anyway), and the trot's planned roll is flat, so there is no lag
to measure in it. Nothing in them suggests the servos fell behind (no
catches, no holds, error rms about 2°), and nothing shows that they did
not. The planned joint speeds are known (§12.4).

The virtual body needed no new model for the two-foot phases: it already
tips as an inverted pendulum about the support line (§9, §10). What it
lacks is the real tip's size, as above.

### 12.3 The gaits tried

All in the virtual body with the fitted servos: forward, back and turning;
each model error (the servos faster, 20 + 50 ms, and slower, 100 + 100 ms;
the old pure 140 ms; the centre of mass 10 mm and 5 mm off with 50 g more
battery, four ways; the IMU 2° and 1.5° off); 10 s each; the body's
balance on at Kp 0.99. Errors are the largest |measured - predicted|.

| model | speed (mm/s) | error, nominal / worst | tilt | margin, one leg up | holds / catches | fastest joint (swing) | verdict |
|---|---|---|---|---|---|---|---|
| the old walk: lean first, one leg at a time | 17.8 to 21.5 | 2.1 / 7.2 (§11) | 12.5° on purpose | +12.7 mm | 0 / 0 | 474°/s | too slow; looks careful |
| (a) lean first with a small sway: margin 3 to 6 mm, roll 6 to 8°, 150 ms swings, 120 to 160 ms moves | 35 to 44 | 2.3 / 4.4 | 4 to 7° | +2 to +5 mm | 0 / 0 | 477 to 596°/s | rejected: past the body's 500°/s slew, still one leg at a time, under the 60 mm/s aimed for |
| (b) diagonal couplets, the fore leg leading (the cat's lateral sequence), 180 ms swings, 80 ms between the pair | 80 | 7.9 / 7.9 | 7.9° | -3.5 mm | up to 35 / 0 | 439°/s | rejected: with the fore leg up alone the mass is outside the three feet first, then on two |
| (b) the same, 140 to 160 ms swings, 40 to 60 ms between the pair | 80 | 4.1 to 4.6 / 6.6 | 4 to 6.6° | -3.5 mm | 0 to 3 / 0 | 417 to 426°/s | the fallback |
| **(b) diagonal couplets, the hind leg leading, 150 ms swings, 60 ms between the pair, 8 mm step** | 26 to 97 | 3.8 / 5.2 | 3.7° | -4.8 to -7.7 mm (dynamic) | 0 / 0 | 365°/s | **chosen** |
| the trot with the old speed mapping (the shift grows as it slows) | 100, backward | 9.4 | 9.4° | – | 15 / 1 | 498°/s | the rocking band |
| **the trot at a fixed cadence (one tick on four feet)** | 42 to 184 | 2.5 / 4.7 | 2.5° | – | 0 / 0 | 414°/s at 184 | **chosen** |

The chosen set at each gait's slowest, default and top speed ("worst" over
every direction and model error above; tilt and margins in degrees and mm;
"balance updates": whether the body's loop acts at all):

| gait | speed | stride | cycle (ms) | error, nominal | roll rms, nominal | tilt, nominal | error, worst | tilt, worst | margin, one leg up | holds / catches / falls, all cases | fastest joint, swing / on the ground (°/s) | balance updates |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| crawl | 8 | 32 | 4300 | 1.2 | 0.54 | 13.5 | 3.6 | 13.9 | +13.4 | 0 / 0 / 0 | 381 / 128 | yes |
| crawl | 22 | 50 | 2550 | 3.3 | 1.09 | 13.2 | 6.4 | 18.3 | +13.0 | 5 / 0 / 0 | 479 / 167 | yes |
| crawl | 27 | 50 | 2160 | 3.1 | 1.36 | 13.3 | 8.4 | 19.1 | +13.3 | 34 / 2 / 0 | 500 / 177 | yes |
| walk | 26 | 8 | 460 | 2.4 | 0.98 | 2.4 | 3.8 | 3.8 | -4.8 | 0 / 0 / 0 | 293 / 41 | none |
| walk | 80 | 24.8 | 460 | 3.7 | 1.67 | 3.7 | 5.2 | 4.4 | -7.2 | 0 / 0 / 0 | 362 / 92 | none |
| walk | 97 | 30 | 460 | 3.8 | 1.57 | 3.7 | 5.2 | 4.5 | -7.7 | 0 / 0 / 0 | 365 / 96 | none |
| trot | 42 | 8 | 340 | 2.5 | 0.79 | 2.5 | 4.0 | 3.9 | – | 0 / 0 / 0 | 293 / 41 | none |
| trot | 144 | 27.4 | 340 | 2.5 | 0.63 | 2.5 | 4.7 | 3.9 | – | 0 / 0 / 0 | 364 / 139 | none |
| trot | 184 | 35 | 340 | 2.5 | 0.61 | 2.5 | 4.6 | 4.0 | – | 0 / 0 / 0 | 414 / 196 | none |

The crawl's holds and catches are all in the slower-servo case walking
backward at its top. Its backward swing asks 516 to 595°/s at 22 to
27 mm/s, past the body's 500°/s slew, which then limits it; that was so
before too, and is not changed here.

### 12.4 Why these

- **The walk.** A walk faster than about 30 mm/s cannot stand on three
  feet all the time (§12.1), and the lean is what made it slow. A walk
  with diagonal couplets is the step from the trot that the evidence
  supports: the trot's two-foot phases are known to work on this body (§7,
  run 4), and the couplet shortens them (90 ms on two feet instead of 150)
  and adds 60 ms on three feet either side. Four footfalls a cycle, a duty
  factor of 0.67 (the cat's is 0.60 to 0.64), a 460 ms cycle. The hind leg
  leads because the centre of mass is 7.5 mm ahead of the middle: on three
  feet with a hind leg up it is about 3.5 mm inside their triangle, with a
  fore leg up about as far outside. Leading with the fore leg, the cat's
  way, tipped 6 to 8° with 180 ms swings and 4 to 6.6° with 140 to 160 ms;
  leading with the hind leg, 3.7° (5.2° in the worst case). The cost: it is
  a diagonal sequence, the primates' order, not the cat's. If the owner
  wants the cat's order, the fore-leading walk with 140 ms swings is the
  fallback (about 0.5° more tilt in the virtual body, 1.4° more in the
  worst case). The predicted attitude stays the plane of the support feet,
  which is flat for this gait: the real body will rock a few degrees that
  the plan does not predict, as the trot does, and the body's loop does not
  act on that (next point), so nothing is fed back.
- **A fixed cadence for the walk and the trot.** The trot's mid-speed
  rocking came from the body's balance loop: it corrects on four feet only
  and holds for two ticks after every touchdown, so with four to eight
  ticks on four feet it measured the tip of the diagonal just ended and
  applied the correction during the next one, which tips the other way.
  With one tick on four feet (plus the swing's last tick, whose foot is
  already down) it never runs. The speed then comes from the stride alone,
  8 to 35 mm: the trot at 100 mm/s backward stays within 2.8° (was 9.4°,
  with gate holds), the same as with the balance off. This is the mind's
  side of the fix; the body is unchanged. The price: while walking or
  trotting, the body no longer levels a slope (§12.5).
- **The crawl** stays the careful gait, statically stable, for slow work.
  Making it dynamic too would leave no gait that keeps the mass inside the
  support at every instant.
- **The trot, faster.** At its top, 184 mm/s, the swing asks 414°/s, the
  same as the floor runs of 2026-11-15 at 144 to 152 (which kept up),
  because the stride is the same 35 mm in the same 150 ms. New: the cycle
  (340 ms, was 380 to 393 ms), so the feet on the ground slide faster
  (196°/s at the joint). A longer stride or a shorter swing would pass
  425°/s; a longer swing tips further on two feet. A running trot (a flight
  phase) is out of reach: the feet would have to push the body up, and at
  414°/s the servos are already near their budget. The remote's top level
  is 180.

**Joint speeds asked** (the fastest joint in the FEET stream): trot 293°/s
at 42 mm/s, 321 at 100, 364 at 144, 414 at 184; walk 293 at 26, 317 at 60,
362 at 80, 365 at 97; the legs on the ground at most 196 (trot) and 96
(walk). All under the body's 500°/s slew.

**Gait changes.** Between the walk and the trot the new gait takes over at
the next slot start once the strides fit it (they shorten at most 0.5 mm a
tick), every foot is within half its stride (+4 mm) of its shoulder, and
the body is at its height; it starts with the pair whose feet are furthest
behind. In the virtual body: 0.6 s and 0.8 s at most. Into or out of the
crawl, every leg still steps home first: the crawl's cycle is 6 to 12
times longer, and switching while the feet were spread drove them out of
the legs' reach in a test. `stop_timeout` (two cycles + 3.5 s: 4.2 s for
the trot, 4.4 s for the walk, 8.6 s for the crawl) covers every stop at
each gait's top speed with at least a second to spare (a test checks it),
so it is unchanged.

### 12.5 What the body's firmware does that matters here (unchanged)

- It holds its corrections while any foot is up and for two ticks after.
  The walk and the trot now never have three ticks on four feet, so the
  balance loop does not act during them at all: on a slope they walk
  tilted (the error tracker learns the offset, and the gate and the catch
  allow for it). The crawl still levels. Slope levelling in a fast gait
  would need the firmware to update on two diagonal feet; not done, and
  not needed on the flat floor.
- The 10° per tick slew is not reached by the walk or the trot (at most
  8.3° a tick). The crawl's backward swing reaches it (above).
- Nothing else in the firmware blocked anything.

### 12.6 On the floor: risks, a first run, the next recordings

**Risks, most likely first.**

1. *The walk tips more than the virtual body says.* With a fore leg up on
   three feet the mass is 5 to 8 mm outside them (on purpose: it is a
   dynamic gait), and the real body tipped about twice as far as the
   virtual one in the trot. If so: gate holds (the walk pauses), then
   catches (a foot lands early). Watch the walk's error by phase.
2. *The trot above 152 mm/s* has never run. The swing's joint speed is the
   same as at 152, the feet on the ground are faster: foot slip is likely,
   and nothing measures it.
3. *No levelling in the walk and the trot* (§12.5).
4. *The direct walk-trot change* has only run in the virtual body.
5. *The new prediction* changes the crawl's reference; if the servos are
   slower with a low battery, the crawl's error grows (the slower-servo
   case is in the table).
6. *Looks.* The walk leads with the hind leg; the owner may prefer the
   cat's order (the fallback above).

**A first floor run**, Kilo on the floor only, battery charged, after
`kilo update --host`:

1. Crawl, level 3 (B4), about 10 s, then B6. In the recording, the roll
   error with a hind leg up should be well under run 4's 2.4 to 2.9°, and
   the largest correction under 3°.
2. Walk (C4) at level 1, about 10 s, B6. Then level 3, then level 5. Stop
   (B6) at any tip that looks like more than 15° or a stagger, and note
   which level.
3. Trot (A4) at level 3, then B7 once (165.6 mm/s), then B7 again (180),
   each for about 10 s; then the same backward.
4. Walking at level 3: A4, then C4, then A4: each change should come
   within a second.
5. A turn (A6, C6) in the walk and in the trot.

Fetch the recordings (`kilo runs --host IP --fetch all`) and analyse them
(`kilo log walk FILE.kwalk`; `mind/tests/walkreplay.py FILE.kwalk` for the
virtual body's replay).

**What the next recordings should check.**

- The servo model: the cross-correlation lag (expected near 120 ms now),
  and whether the crawl's corrections stay inside the dead band.
- The walk's error by phase: RF+LB and LF+RB (two feet), and RF and LF
  alone (a fore leg up, on three feet), against the virtual body's 3.7°.
- The trot's rocking at 144, 165.6 and 180 mm/s with the balance never
  acting, against run 4's ±4 to 5° (where the loop acted once a slot).
- The pitch offset: Kilo standing still on a level floor, a spirit level
  on his back; then the same with the front feet on 3 mm shims. That tells
  the IMU's mounting from the front legs' calibration.
- The real speed: a tape on the floor and the time taken for 1 m, per gait
  and level (nothing measures slip).
- A slow-motion phone video from the side during a trot and a walk: the
  footfall timing, and whether the swinging feet reach their height (the
  servos keeping up).

### 12.7 What is still unverified

Everything above comes from the virtual body, calibrated on three
recordings of the old gaits. Not verified: that the real servos follow
50 + 70 ms in other gaits and at other battery levels; that the real walk
tips no more than about twice the virtual one; any speed above 152 mm/s;
foot slip; the fore-leading fallback on the floor; the direct gait change
on the floor; the cause of the pitch offset.

## 13. Steering, out of a turn, and a faster trot (2026-11-17)

The owner, after run 5: "investigate if you can have kilo turn while moving
forward. Would be nice to steer without stopping"; "when turning in a
stopped position I would like an up or down button press stop the turn and
go forward or backward"; "I want the trot to go faster. Maybe lengthen out
the strides." (And the rear-up trick moved from B2 to A2: "matches the
picture better".)

**[measured]** marks what comes from the floor recordings, **[virtual
body]** what comes from `mind/tests/walksim.py` with the fitted servos and
the model errors of §12.3, **[estimate]** an inference from both.

### 13.1 What A6 / C6 did before

While walking forward, A6 asked the planner for the direction `left`: a
turn in place, each side 0.8 of the stride, the left side backward. Nothing
stopped: the strides moved 0.5 mm a tick toward their new values, so the
trot at 1x (27.4 mm on each side) went from straight to an arc that
tightened as the left stride shrank, crossed zero after about 1.1 s and
reached the turn in place after about 2 s (-21.9 / +21.9 mm). From then on
it turned on the spot. B7 and B5 during a turn changed only its speed; the
way back to straight was the gait key, another 2 s of arc. So there was no
steady curve, and no way from a turn straight into walking backward.

The turns in place on the floor **[measured]** (the gyro's yaw rate in the
recordings; Kilo's IMU reports it in every TELEMETRY):

| recording | turn | side's slide | yaw rate | yaw rate if the feet did not slip | share |
|---|---|---|---|---|---|
| 2026-11-15 walk 2 | trot left, 144 (35 mm, 243 ms on the ground) | 115 mm/s | +27.7°/s | 203°/s | 0.14 |
| 2026-11-16 walk 2 | trot right, 144 | 115 mm/s | -27.0°/s | 203°/s | 0.13 |
| 2026-11-16 walk 7 | trot left, 144 | 115 mm/s | +23.6°/s | 203°/s | 0.12 |
| 2026-11-16 walk 4 | walk left, 80 | 64 mm/s | +16.5°/s | 113°/s | 0.15 |
| 2026-11-17 walk 3 | trot right, 144 | 115 mm/s | -18.0°/s | 203°/s | 0.09 |
| 2026-11-17 walk 8 | trot left, 57.6 | 46 mm/s | +10.4°/s | 81°/s | 0.13 |

The legs move only fore and aft, so in a turn the feet scrub sideways and
Kilo turns at about an eighth of the rate the strides alone would give: the
two sides act as if they were about 500 mm apart (440 to 730 in these six),
not the 65 mm of the track. Walking straight, the gyro reads -3 to +6°/s
(a slight drift of the path or the gyro's offset): the smaller turns above
are within that of each other.

### 13.2 Steering

**How.** A forward or backward walk now takes a steer S from -1 to 1 (+
left; `Planner.set_steer`, `WalkGoal.steer`).
The left side's stride is the stride less S times it, the right side's the
stride plus it; when the longer of the two would pass the pattern's longest
stride (the trot 44 mm, the walk 30, the crawl 50), both are scaled down
together, which keeps the bend and slows the walk. S = 1: the inner side
steps in place. Backward the same S turns the nose the same way (A6 = nose
left, forward and backward alike, as the turn in place does). There is no
other way to steer this body: with two servos per leg in one plane the feet
cannot be placed sideways or turned. The strides change 1.5 mm a tick
(below), so a steer takes effect within a cycle or two, smoothly.

**The remote.** While walking forward or backward, each press of A6 bends
the path one level further left (C6 one further right; the other key steps
back through straight), levels S = 0.25, 0.5, 1, at most 3 each way, without
stopping. From standing, A6 / C6 still turn in place. I chose a level per
press over holding the key: the body relays a held key as repeats about
every 110 ms, but there is no release, so the mind would have to guess it
from a gap in the repeats, and a gap also comes from the remote pointing a
little away; a steer that flicks back to straight on its own while walking
at 230 mm/s is worse than one more press. B7 / B5 keep the steer; the gait
key in use straightens it; every stop forgets it.

**Radius per level.** The bend's ratio sets the radius, not the speed: R =
(half the effective track) / S. With the 500 mm of the recordings
**[estimate: the scrub measured in turns in place, assumed the same in a
curve]**, and with the 65 mm track as if no foot slipped (a lower bound):

| level | S | radius, estimated | radius, no slip | trot at level 3: speed, turn rate (est.) | trot at level 5 | walk at level 3 |
|---|---|---|---|---|---|---|
| 1 | 0.25 | about 1.0 m | 130 mm | 144 mm/s, 8°/s | 185 mm/s, 11°/s | 77 mm/s, 4°/s |
| 2 | 0.5 | about 0.5 m | 65 mm | 144 mm/s, 17°/s | 154 mm/s, 18°/s | 65 mm/s, 7°/s |
| 3 | 1 | about 0.25 m | 33 mm | 116 mm/s, 27°/s | 116 mm/s, 27°/s | 48 mm/s, 11°/s |

(The crawl: the same radii at 6 to 22 mm/s.) Level 3 turns about as fast as
the turn in place does (24 to 28°/s) while still walking forward. The real
radius may differ by half either way; the floor will tell (§13.5).

**In the virtual body [virtual body]** (it has no yaw: it checks the
attitude, the margins, the joints and the reach, not the curve). The trot
at levels 1, 3 and 5 and the walk at 1, 3 and 5, each steer level both ways,
forward and backward, each model error, 10 s: no fall, no catch, no gate
hold, every foot in reach and inside FEET's range. The largest error from
the plan: trot 2.5 to 3.6° nominal, 3.9 to 5.0 worst; walk 2.2 to 3.7°
nominal, 3.8 to 5.2 worst, the same as walking straight, but one case: the
trot at levels 1 and 2 backward, S = 1, with the old pure 140 ms servo
model, 5.4 to 6.6° (no catch, no hold; 2.5 with the fitted model). Fastest
joint: the outer side's stride never passes the pattern's longest, so the
joints ask no more than at the top speed (trot 476°/s, walk 408, the crawl
as before). The crawl steers too, nominal 1.1 to 3.7° forward and 1.4 to
6.3 backward (its backward swing at its top already asked past the slew,
§12.3), no hold, no catch, the margin above 7 mm. Changes while steering: steering left 3 to right 3, the
planned stop (inside stop_timeout), the walk-trot change, and into the crawl
(home first) all stay up; the trot steering into the crawl reaches 7.4° in
the worst model case, with one gate hold (the crawl's own worst is 8.4).

### 13.3 Out of a turn: B7 and B5

While turning in place, B7 now ends the turn and walks straight forward at
the turn's level (3 from standing), B5 straight backward, without a stop.
To make that quick, the walk and the trot move their strides 1.5 mm a tick
(0.5 before; the crawl keeps 0.5): from the turn at 1x to straight takes 33
ticks (0.66 s) in the trot, 30 in the walk, about two cycles; it was 2 s.
Any stride change stays inside the pattern's longest stride, so a swing
never carries a foot further than at the top speed, and the feet on the
ground change speed by at most 0.16 mm a tick each tick (about 0.04 g, a 3
mm shift of the zero-moment point at the body's height). The same faster
slew makes B7 / B5 and the steer act within a cycle. **[virtual body]** the
turn to forward and to backward, each model error: up, within 2.6 to 3.6°
nominal and 4.0 to 5.1 worst, no hold, no catch, the fastest joint 367°/s.

One thing the faster slew broke, and the fix: out of the crawl the first
walk or trot slot levels the body from the crawl's lean (about 20 ticks on
four feet), and the strides grew during it, sliding every foot 30 mm back
before its first lift; the next swings then asked up to 830°/s (found by
the test of two gait changes in a row). Now the strides stay at 0 until
that level-out ends.

### 13.4 A faster trot

**What limits it.** The trot's speed is its stride over the time a foot is
on the ground, which at its fixed cadence is the swing plus two ticks
(190 ms). So it goes faster with a longer stride, a shorter swing, or both;
each makes the swinging leg move faster. The legs' reach is not the limit:
at the 76 mm stance height a foot reaches 48 mm fore and aft (the IK's 95 %
of 95 mm), and at 44 mm the feet go to 23 mm. A lower body reaches further
but asks more of the knee for the same stride; a higher one the reverse,
with less reach to spare; neither changed the joint speed by more than the
FEET's 1 mm rounding (192/256 to 225/256 tried). The limit is the joint
speed: the body slews a joint at most 10° a tick (500°/s), the floor runs
asked up to 414 at 180 mm/s and kept up, and the servos' speed under load
was never measured (an MG90S is rated 600°/s with no load).

The fastest joint of the swing **[planned, the FEET stream]**:

| stride | swing | lift | top speed | fastest joint |
|---|---|---|---|---|
| 35 mm | 150 ms | 8 mm | 184 mm/s (before) | 414°/s |
| 40 | 150 | 8 | 210 | 465 |
| 44 | 150 | 8 | 232 | 517: past the slew |
| 44 | 150 | 6 | 232 | 473 |
| 40 | 140 | 6 | 222 | 458 |
| 44 | 140 | 6 | 244 | 506: past the slew |
| 50 | 150 | 6 | 263 | 539: past the slew |

The cheapest lever is the lift: the swing's fastest moment is the leg
lifting and moving forward at once, and 2 mm less lift saves about 45°/s.
A shorter swing (140 ms, 7 ticks) gives about the same speed for the same
joint speed, but through the fitted servo response (50 ms + 70 ms) a 7-tick
lift reaches 54 % of what is asked against 58 % for 8 ticks: less clearance
for nothing. **Chosen:** the stride to 44 mm (from 35), the swing unchanged
(150 ms), the lift 8 mm up to a 35 mm stride and tapering to 6 mm at 44
(so every speed up to the old top is exactly as on the floor). Top speed
232 mm/s (from 184); the fastest joint 476°/s at the top and while steering,
in every direction (414 at 184 and below, as before). The body height, the
cycle and the other gaits are unchanged.

The trade-offs: the swing joint asks 476°/s, 5 % under the body's slew and
15 % more than the floor ever asked; the feet on the ground slide at 249°/s
at the knee (196 before); the foot clears the floor by less at the top
(6 mm asked, about 3.5 mm reached through the servo response, against
about 4.6 before), so a rug or a seam will catch a toe sooner.

**A flight phase** (a running trot) is out of reach: the feet would have to
push the body up faster than gravity pulls it down, and the joints are
already at 95 % of the body's slew just carrying the swing.

**The remote's levels** for the trot are now 0.4, 0.7, 1, 1.3, 1.6 × 144
(were 1.15 and 1.25 at the top); the walk and the crawl keep 0.4, 0.7, 1,
1.15, 1.25 (their tops are their tops):

| level | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|
| trot, before | 57.6 | 100.8 | 144 | 165.6 | 180 |
| trot, now | 57.6 | 100.8 | 144 | 187.2 | 230.4 |

**[virtual body]** the trot at each level, forward and backward, nominal
and the worst of the model errors (servos faster and slower, the old pure
delay, the centre of mass 10 mm and 5 mm off with 50 g more battery three
ways, the IMU 2° and 1.5° off), 10 s each:

| speed | stride | lift | error, nominal fwd / back | roll rms, fwd / back | error, worst | tilt, worst | holds / catches / falls | fastest joint, swing / on the ground |
|---|---|---|---|---|---|---|---|---|
| 57.6 | 10.9 | 8 | 2.5 / 2.6 | 0.76 / 1.48 | 4.1 | 4.0 | 0 / 0 / 0 | 300 / 78°/s |
| 100.8 | 19.2 | 8 | 2.5 / 2.5 | 0.68 / 1.44 | 4.0 | 4.0 | 0 / 0 / 0 | 316 / 114 |
| 144 | 27.4 | 8 | 2.5 / 3.2 | 0.62 / 1.81 | 4.7 | 3.9 | 0 / 0 / 0 | 364 / 139 |
| 187.2 | 35.6 | 7.9 | 2.5 / 3.1 | 0.60 / 1.84 | 4.6 | 3.9 | 0 / 0 / 0 | 414 / 196 |
| 230.4 | 43.8 | 6.0 | 2.1 / 2.4 | 0.50 / 1.54 | 3.9 | 3.4 | 0 / 0 / 0 | 444 to 476 / 249 |

The faster trot is no less steady in the virtual body (the lower lift even
tips it a little less). On the floor the trot rocks about twice what the
virtual body shows (§12.2), and nothing here models a foot slipping, which
the faster slide on the ground makes more likely.

### 13.5 On the floor: risks, a first run, the next recordings

**Risks, most likely first.**

1. *The curve is not the estimated one.* The radii come from the scrub in
   turns in place; a curve may scrub differently, and foot slip is not
   modelled. Level 1 may barely bend, or level 3 may turn nearly on the
   spot. Harmless; the levels can be retuned from the gyro.
2. *The servos do not keep up above 414°/s* (the trot at level 5, and any
   steer at level 3 or above, whose outer side takes the longer stride): the
   swing lands late or short, the foot drags; seen as the error growing
   with the speed, catches, or a stumble.
3. *The lower lift catches a toe* at the top (6 mm asked, about 3.5
   reached): on a rug or a seam.
4. *Foot slip* at 230 mm/s (the feet on the ground slide faster): the real
   speed lower than planned; nothing measures it but a tape on the floor.
5. *The quicker changes* (1.5 mm a tick) feel abrupt: a jerk when B7 / B5
   ends a turn or when the speed steps. The virtual body cannot show
   inertia on four feet.
6. *Steering backward slowly with the tightest level* tipped 6.6° in the
   virtual body with the old servo model (2.5 with the fitted one).

**A first floor run**, Kilo on the floor with room around him, battery
charged, after `kilo update --host`:

1. A4 (trot), wait 3 s. A6 once, 5 s; A6 again, 5 s; A6 a third time, 5 s.
   Then C6 three times, a few seconds apart (back to straight), then C6
   three more (to the right). B6.
2. A6 from standing (the turn in place), 3 s, then B7: he should walk
   straight forward within a second, without stopping. B6. C6, 3 s, then
   B5: straight backward. B6.
3. A4, then B7 once (187 mm/s), 5 s; B7 again (230 mm/s), 5 s; B6. Stop at
   once (B6, or C7 to freeze) at a stumble or a toe catching. Then
   backward: A4, B5 five times (to level -3, 144 mm/s backward), 5 s; B5
   twice more (-5, 230 mm/s backward), 5 s; B6.
4. C4 (walk), A6 twice, 5 s, C6 twice, B6.
5. With the rear-up key switched on only: A2 (it was B2).

Fetch the recordings (`kilo runs --host IP --fetch all`) and analyse them
(`kilo log walk FILE.krec`; the walk lines carry `steer=` now).

**What the next recordings should check.**

- The yaw rate per steer level (the gyro's `rate_yaw` in the recording's
  TELEMETRY), against the estimates above (8, 17, 27°/s at level 3), and
  with the turns in place again for the effective track.
- The trot's error and rocking at 187 and 230 mm/s, forward and backward,
  against 144; holds and catches (expected 0).
- Whether the swing keeps up at 230: a slow-motion phone video from the
  side, the feet clearing the floor and landing where planned.
- The real speed at 230 (a tape, the time for 1 m): slip.
- The change out of a turn: how long until the yaw rate is near zero after
  B7.

## 14. The quick stop (2026-11-19)

The owner, after a floor run: "can you make him stop faster. Both when I
press stand and when I put my hand in front of his face, he slowly winds
down. He should basically stop right away."

### 14.1 What stopping did before

B6 (stand), A7, C5, a trick key while walking, and the distance guard all
used the **planned stop**: the strides (how far each foot travels per step)
shrink by 1.5 mm a tick in the walk and the trot and 0.5 mm in the crawl,
so Kilo keeps walking, slower and slower, for up to 0.6 s in the trot and
2 s in the crawl; then every leg still away from under its shoulder takes
one more step there, and the body levels. In the trot at its top speed that
is 0.56 s and about 5 cm of travel after the request; in the crawl about
4 s. C7 (freeze) cut the stream instead: the 100 ms of plan already queued
on the body played on at full speed, then the body froze every foot for
100 ms, with a diagonal pair in the air in the trot, then put the feet in
the air down at 2 mm a tick and slid all four under the shoulders.

### 14.2 What limits a stop

- **Feet in the air must come down first.** In the trot two diagonal feet
  carry Kilo for 150 ms at a time; stopping the feet on the ground while a
  pair is in the air leaves him balanced on a line, and he tips toward a
  raised foot. So the raised feet land at once, where they are.
- **Momentum.** At 230 mm/s, stopping the feet on the ground in one tick
  (20 ms) would decelerate the body at about 11 m/s², and his weight would
  shift forward by that over gravity times the height of his centre of mass
  (79 mm): 90 mm, past his front feet (32 to 76 mm ahead of it). The servos
  smooth any change over about 120 ms (50 ms dead time and 70 ms to
  follow), which alone keeps the peak near 3 m/s²; slowing the feet over
  120 ms (six ticks) keeps it under about 2 m/s², a weight shift of about
  15 mm, well inside four feet.
- **The plan already sent.** The mind plans five ticks (100 ms) ahead of
  the body, and the body queues them; a stop decided now starts 100 ms
  later unless those ticks are planned again.

### 14.3 The quick stop

`Planner::halt()` (kernel/mind/planner/plan.cpp, mind/kilo/planner/plan.py,
the two kept in step by the golden vectors):

1. **Land.** Every foot in the air comes down where it is, in three ticks
   (60 ms, the same move as a catch), and no other foot lifts.
2. **Brake.** The strides go to zero in six ticks (`HALT_TICKS`, 120 ms):
   the feet on the ground stop sliding back, so the body stops moving.
3. **Hold.** All four feet stay down until ten ticks (`HALT_HOLD_TICKS`,
   200 ms) after the request: the servos catch up and the body settles.
4. **Square up.** Each leg more than 5 mm (`HALT_SQUARE_MM`) from under its
   shoulder steps there, one group at a time, as the end of a planned stop
   does: in the trot and the walk two short steps in place (about 0.35 s),
   in the crawl up to four, one leg at a time with its lean, at the crawl's
   fastest cadence (about 3 s; the body does not travel, it shifts its
   weight over the three feet that stay down). A leg the body tips toward
   while it steps is caught and stays where it landed. Then the body levels
   and STANCE takes the legs (it sets the last few mm).

The C++ mind's walker also **plans again the ticks already sent**: it keeps
a copy of the planner from before each of the last ticks, and on a halt
goes back to the first tick the body has not played yet (its tick + 2: the
next FEET may be on the wire), halts from there, and sends those ticks
again; the body replaces a queued tick with a newer FEET for it. The stop
starts about 40 ms after the request instead of 100 ms. The Python walker
(`mind/kilo/planner/walker.py`, the reference) has the halt but not this.

**Where it is used.** Everything that asks for a stop on Kilo: B6, A7, C5,
a trick key while walking, C7 (freeze: now the quick stop, then frozen as
before), the distance guard's stops (docs/distance-guard.md), and its
back-offs' ends. **The planned stop stays** where a clean, gradual finish
matters and nobody is waiting for it: the legs' way home when the gait
changes into or out of the crawl, and the scripted walk of the host tests
(`kernel/host/walk_demo.cpp`).

### 14.4 The numbers

`PYTHONPATH=mind python mind/tests/stopsim.py --md` walks the virtual body
(the planner, the stream, the body's FEET player, the walking physics with
the servo lag, and its momentum switched on: braking shifts the weight
forward by a h / g) at each gait and speed, asks for the stop at every
second tick of a whole cycle (every sixth in the crawl), and measures from
the request: when the body is stopped (from then on it stays within 3 mm
of where it ends), how far it went until then, when the walk is over (the
legs squared up, STANCE), and how far the stop takes the pitch and roll
outside the walk's own sway. "planned" is the old stop, "quick" the halt as
the Python walker does it, "replan" as the C++ mind on Kilo does it (the
ticks already sent planned again).

| Gait, level | Speed | Stop | Stopped (median / worst) | Way (median / worst) | Done (median) | Pitch, roll beyond the walk's own |
|---|---|---|---|---|---|---|
| crawl +1 | 9 mm/s | planned | 5.19 / 5.84 s | 10 / 19 mm | 5.4 s | +0.2°, +11.6° |
| crawl +1 | 9 mm/s | quick | 2.87 / 3.12 s | 4 / 14 mm | 3.1 s | +0.5°, +13.6° |
| crawl +1 | 9 mm/s | replan | 2.80 / 3.04 s | 3 / 13 mm | 3.0 s | +0.6°, +12.6° |
| crawl +3 | 22 mm/s | planned | 4.42 / 4.76 s | 27 / 37 mm | 4.6 s | +0.4°, +1.2° |
| crawl +3 | 22 mm/s | quick | 2.88 / 3.16 s | 7 / 17 mm | 3.1 s | +0.6°, +2.5° |
| crawl +3 | 22 mm/s | replan | 2.80 / 3.08 s | 5 / 15 mm | 3.0 s | +0.6°, +2.6° |
| crawl +5 | 27 mm/s | planned | 4.21 / 4.54 s | 33 / 43 mm | 4.4 s | +0.4°, +1.3° |
| crawl +5 | 27 mm/s | quick | 2.88 / 3.16 s | 10 / 19 mm | 3.1 s | +0.4°, +2.8° |
| crawl +5 | 27 mm/s | replan | 2.80 / 3.08 s | 7 / 16 mm | 3.0 s | +0.5°, +2.8° |
| walk +1 | 32 mm/s | planned | 0.08 / 0.10 s | 2 / 3 mm | 1.0 s | +0.2°, +0.4° |
| walk +1 | 32 mm/s | quick | 0.10 / 0.12 s | 2 / 3 mm | 0.9 s | +0.4°, +1.5° |
| walk +1 | 32 mm/s | replan | 0.02 / 0.04 s | 1 / 1 mm | 0.8 s | +0.4°, +1.4° |
| walk +3 | 80 mm/s | planned | 0.30 / 0.32 s | 18 / 18 mm | 1.2 s | +0.6°, +0.8° |
| walk +3 | 80 mm/s | quick | 0.17 / 0.26 s | 11 / 15 mm | 1.1 s | +1.2°, +2.3° |
| walk +3 | 80 mm/s | replan | 0.09 / 0.18 s | 6 / 10 mm | 1.1 s | +1.2°, +2.3° |
| walk +5 | 97 mm/s | planned | 0.38 / 0.38 s | 25 / 26 mm | 1.2 s | +0.7°, +1.0° |
| walk +5 | 97 mm/s | quick | 0.19 / 0.38 s | 14 / 20 mm | 1.1 s | +1.2°, +2.4° |
| walk +5 | 97 mm/s | replan | 0.11 / 0.30 s | 8 / 14 mm | 1.1 s | +1.2°, +2.4° |
| trot +1 | 58 mm/s | planned | 0.14 / 0.14 s | 4 / 5 mm | 0.8 s | +0.9°, +1.9° |
| trot +1 | 58 mm/s | quick | 0.14 / 0.16 s | 5 / 6 mm | 0.8 s | +0.6°, +1.2° |
| trot +1 | 58 mm/s | replan | 0.06 / 0.08 s | 2 / 3 mm | 0.8 s | +0.6°, +1.2° |
| trot +3 | 144 mm/s | planned | 0.34 / 0.34 s | 26 / 27 mm | 1.0 s | +1.3°, +2.6° |
| trot +3 | 144 mm/s | quick | 0.20 / 0.22 s | 17 / 21 mm | 0.9 s | +1.2°, +2.6° |
| trot +3 | 144 mm/s | replan | 0.12 / 0.14 s | 10 / 15 mm | 0.8 s | +1.2°, +2.6° |
| trot +4 | 187 mm/s | planned | 0.44 / 0.48 s | 39 / 41 mm | 1.2 s | +1.9°, +1.6° |
| trot +4 | 187 mm/s | quick | 0.22 / 0.56 s | 24 / 29 mm | 0.9 s | +1.6°, +1.5° |
| trot +4 | 187 mm/s | replan | 0.14 / 0.48 s | 15 / 21 mm | 0.8 s | +1.7°, +1.8° |
| trot +5 | 230 mm/s | planned | 0.56 / 0.58 s | 51 / 53 mm | 1.3 s | +1.6°, +0.9° |
| trot +5 | 230 mm/s | quick | 0.22 / 0.56 s | 27 / 36 mm | 1.0 s | +1.6°, +1.2° |
| trot +5 | 230 mm/s | replan | 0.14 / 0.48 s | 18 / 26 mm | 0.9 s | +1.6°, +1.2° |

(The crawl +1 rows' roll is not the stop's: at 9 mm/s the crawl had not
finished its first 4-second cycle in the 3 s walked before the request, so
its own lean to the other side, up to 14 degrees, counts as "beyond".)

Read: on Kilo (the "replan" rows) a trot at its top speed is stopped about
0.15 s after the request (median; 0.5 s at the worst moment of the cycle,
where the last few mm come from STANCE setting the legs), after 2 cm
instead of 5; the walk after about 1 cm. Then two short steps in place
square him up, and he stands still within a second. The crawl goes on
for at most 1-2 cm (its weight shifting over the feet); its square-up takes
about 3 s of steps in place, against more than 4 s of walking on before. No case fell, and the
pitch and roll stay within about 2 degrees of the walk's own sway.

The support feet's travel in the real C++ planner (kernel/host/tests/
test_guard.cpp prints it for every gait and speed, the worst over the
cycle): walk 97 mm/s 5 mm, trot 144 mm/s 8 mm, trot 230 mm/s 13 mm, crawl
27 mm/s 10 mm (its lean while it squares up), against the planned stop's
19, 26, 66 and 23 mm.

### 14.5 What is computed and what is not

Computed (the model and the simulators): the plan itself (the C++ and the
Python planner agree to 0.01 degree on every tick of 20 halt scenarios:
every gait, every phase, during the planned stop, steering, turning, the
crawl mid-lean, with the body tipping while it squares up); the virtual
body's attitude and travel; the momentum as a weight shift.

Not verified on the floor: the virtual body is rigid, its feet never slip
and its frame does not flex; the momentum term is a first estimate. On the
real floor the braking may slide the feet a little (the stop then a few mm
longer), and the squaring up drags a foot by up to 5 mm when STANCE takes
over. The servos' real response to a quick change of direction of the feet
was never recorded. What to watch on the floor: section 14.6.

### 14.6 On the floor

1. A4 (trot) at +3, 3 s, B6: he should stop within about a step, then take
   one or two small steps in place and stand. Watch for a nose dip or a
   rock to one side at the stop.
2. A4, B7 twice (230 mm/s), 3 s, B6. The same; this is the hardest case.
3. C4 (walk), 3 s, B6. B4 (crawl), 5 s, B6: he stops at once and then
   shuffles his legs under him for about 3 s.
4. A4, 3 s, C7 (freeze): the same quick stop, then he stays frozen until
   B6 or A7.
5. The recordings (`kilo runs --host IP --fetch all`): the walk run's
   `stop tick=... quick=1 replanned=N` mark shows the quick stop and how
   many queued ticks it planned again.

## 15. A longer stride for the trot (2026-11-21)

The owner: "Re the trot. Can you find a way to really lengthen out the
stride? If he's moving at a pace I think you can figure out how to get the
feet where they need to be before he falls off balance. Maybe it's
something that you build up. Try and model it."

**[measured]** marks numbers from the floor recordings (`~/kilo-logs`, runs
of 2026-11-15 to 11-17), **[fitted]** a model fitted to them, **[virtual
body]** the simulators, **[assumed]** what nobody has measured.

### 15.1 The short answer

- **What limits the stride today is the servos, not the legs and not the
  balance.** The trot swings a leg in a fixed 160 ms (8 ticks). To carry a
  44 mm stride (today's top, level 5) in that time the fastest joint turns
  at 476°/s; the body's own safety limit is 500°/s. A longer stride in the
  same swing breaks that limit. The legs could reach a stride of about 90
  mm (48 mm each way at the trot's height).
- **So a longer stride needs a longer swing, and a longer swing leaves him
  longer on two diagonal feet**, falling sideways about the line between
  them. That is the owner's point, and the recordings show how he falls and
  where the feet must land to catch it (15.3). Landed right, the fall
  cancels itself: he starts each diagonal with his weight on one side of
  the line, walks across it, and ends upright.
- **The model supports a stride of about 52 mm** (nearly twice today's 27.4 mm
  at the default speed, 18 % more than today's longest), with a swing of 11
  ticks (220 ms) instead of 8, built up a tick per step as the speed rises
  from 150 to 207 mm/s. Each pair lands about 6 mm less far ahead than half
  a stride (19 mm ahead of the shoulders at the top, not 26).
- **It does not make him faster.** The speed is the stride over the time a
  foot is on the ground, and both grow together: the top is 207 mm/s,
  slower than today's level 5 (230). A longer stride gives fewer, longer
  steps (a 480 ms cycle instead of 360) at the same pace. Faster than about
  230 mm/s needs faster joints: a firmware slew above 500°/s and servos
  that can do it (never measured).
- **It is a prototype** (`kilo.planner.longtrot`), tested in the virtual
  body only, not in the C++ mind and not on the remote: the owner decides
  whether it goes further. The existing gaits are unchanged.

### 15.2 What limits the stride, in numbers

| limit | today's trot | what a longer stride needs | source |
|---|---|---|---|
| leg reach | the foot goes to ±26 mm at 230 mm/s | ±48.6 mm at the trot's height (76 mm): a stride of about 90 mm; lower bodies reach further (56 mm at 70 mm) | IK, measured links |
| joint speed, swinging leg | 476°/s at 44 mm (6 mm lift), 364°/s at 27.4 mm | the swing time grows with the stride: 8 ticks carry 40 mm, 9 ticks 46, 10 ticks 52, 11 ticks 59, 12 ticks 64 (8 mm lift, under 476°/s) | the plan's own profile; the 500°/s limit is the body's (`SLEW_MAX_A16`); the servos' own speed under load is **[assumed]** (rated 600°/s unloaded at 4.8 V); the floor ran 476°/s at 230 mm/s and kept up as far as anyone could see |
| joint speed, legs on the ground | 249°/s at 230 mm/s | under 300°/s at every speed of the long trot | the plan |
| clearance | 8 mm asked, about 4.7 mm reached through the servos (6 asked and 3.5 reached at 230 mm/s) | a longer swing reaches more of it: 5.2 mm at 10 ticks, 5.4 at 11 | the servo model (50 ms dead time, 70 ms response), **[fitted]** to the crawl's leans |
| balance | 140 ms on each diagonal pair | 20 ms more per tick of swing; what that costs is 15.3 | the recordings, 15.3 |

(`PYTHONPATH=mind python mind/tests/stridesim.py limits` prints the joint
table.)

### 15.3 How the trot balances: the pendulum on the diagonal

On a diagonal pair Kilo stands on a line, and nothing holds him sideways:
he is an inverted pendulum about that line, the tilt growing as e^(t/τ)
with τ = √(h/g) = 90 ms for his 79 mm high centre of mass (§2). Two things
push him off the line during each diagonal:

1. **Where his weight sits.** The centre of mass is ahead of the middle
   (7.5 mm, measured 2026-11-14), and the feet on the ground are wherever
   they landed: a pair that lands far ahead leaves the weight behind its
   line, and he tips toward the hind foot in the air.
2. **Walking across the line.** The diagonal runs at 28° to his length, so
   walking forward at v carries his weight across it at 0.47 v (67 mm/s at
   144 mm/s): that alone tips him toward the front foot in the air.

Land the pair at the right place and the two cancel: he ends the diagonal
upright. That place is x_c + k·v·τ ahead of the shoulders (x_c the
centre of mass's position, k between 0.5 and 0.9 depending on how long the
diagonal lasts): a small step ahead of where the body is going, the same
idea as a "capture point". Half a stride ahead, which is what the trot
does, is that place only at one speed.

**The recordings agree [measured].** Folding the measured roll of every
straight trot slot by slot (deduplicated: some runs are in two files) gives
the tip each diagonal ends with, + toward the front foot in the air:

| trot | stride | slots | tip measured | fitted pendulum | virtual body (Physics) | RockingPhysics |
|---|---|---|---|---|---|---|
| forward 101 mm/s | 19.2 mm | 6 | +1.85° | +1.53 | -0.99 | -0.41 |
| forward 144 mm/s | 27.4 mm | 21 | -1.15° | -0.91 | -0.86 | -0.73 |
| forward 166 mm/s | 31.5 mm | 7 | -1.73° | -1.78 | -0.85 | -0.83 |
| forward 180 mm/s | 34.2 mm | 24 | -2.17° | -2.05 | -0.76 | -1.04 |
| forward 187 mm/s | 35.6 mm | 4 | -2.13° | -2.13 | -0.73 | -1.38 |
| forward 230 mm/s | 43.8 mm (6 mm lift) | 8 | -1.32° | -2.28 | -0.68 | -1.88 |
| back 42 mm/s | 8.0 mm | 12 | +2.54° | +2.30 | +2.21 | +2.78 |
| back 58 mm/s | 10.9 mm | 6 | +2.38° | +2.30 | +2.35 | +2.96 |
| back 144 mm/s | 27.4 mm | 7 | +2.33° | +2.30 | +2.68 | +3.03 |

Read: forward, the tip changes sign between 100 and 144 mm/s (at 100 the
pair lands too little ahead, above 144 too far), and today's default sits
near the sweet spot, which may be why the owner liked the trot at 144.
Backward, the pairs always land too far back and he always tips the same
way. The rock is very regular: after the phase-locked part is taken out,
what is left is 0.1 to 0.3° rms at a steady speed.

**The fitted pendulum [fitted]** (`kilo.planner.longtrot.TrotFit`): the
pendulum above, from upright at each diagonal's start, with three numbers
fitted to the table (rms 0.15° over the eight rows with the 8 mm lift):

- the centre of mass 3.5 mm ahead of the middle (measured standing: 7.5);
- the body rocks as if each diagonal lasted 120 ms longer than the plan's
  140 ms on two feet: the rotation is not stopped at every touchdown, it
  runs on into the next diagonal (fits with 100 to 160 ms extra are almost
  as good, 0.13 to 0.31°);
- the tip stops at about 2.3°: the foot on the falling side, 4.7 mm off the
  ground, touches down early and catches him. Most of today's trots run
  against this stop (backward always, forward above 160 mm/s), and they
  walk fine: the early touchdown is the trot's real safety net. With the 6
  mm lift at 230 mm/s it catches sooner (-1.3° measured).

From the fit, a pair that lands 1 mm off its balanced point tips him 2.4°
by the diagonal's end with today's 8-tick swing; 3.1° with 9 ticks, 3.9
with 10, 5.0 with 11, 6.3 with 12. That growth is the price of a longer
swing.

**The virtual body was not faithful here.** Its physics (Physics, §10)
has the pendulum, but on a diagonal it often stands on one foot of the
pair (whichever is lower by 0.6 mm), stops every rotation as soon as a
third foot touches, and its rock hardly changes with the speed: it misses
the sign change forward and is half the size above 160 mm/s. A second
physics was built for this study (`kilo.vbody.rocking.RockingPhysics`:
both feet of a diagonal on the ground, the body free to turn only about
their line, a third foot that does not yet hold the weight leaves it
rocking, the rotation carried from one diagonal to the next); with its
best settings of those tried it gets the backward rock and the sign above
144 forward, still about half the size forward and the wrong sign at 101.
So the fitted pendulum is the model the design uses; the two physics check
the mechanics (reach, joints, clearance, falls, stops) and the direction of
each effect.

### 15.4 The long trot and its build-up

`kilo.planner.longtrot.LongTrotPlanner`, a Planner with:

- **A swing that grows with the speed, a tick per step.** 8 ticks up to 150
  mm/s (exactly today's trot: the same stride, cadence and feet within 2
  mm), then a tick more for every 18 mm/s, at most 11 ticks; a swing is
  never more than a tick longer than the one before, so a speed change
  lengthens the stride over a few steps (the stride itself grows 0.75 mm a
  tick, half the trot's 1.5). One tick on four feet, as today.
- **Each pair lands at its balanced point**, x_c + k v τ from the fitted
  pendulum, instead of half a stride ahead.
- **Each swing long enough for its step** (a square-up after a stop, a
  steer's outer side), the steer's outer stride capped for its swing.

**The schedule** (forward; [fitted] the tip and the gain, [virtual body]
the last four columns; the error is the largest |measured - predicted|
over 8 s; the model errors are the centre of mass 5 mm ahead, behind, or
5 mm left with 50 g more battery, and the servos slower (100 + 100 ms) or
faster (20 + 50 ms) than the plan assumes):

| speed | swing | stride | cycle | lands ahead (half a stride) | tip if it landed at half a stride | tip per mm off | error, RockingPhysics / Physics | worst of the model errors | fastest joint | feet |
|---|---|---|---|---|---|---|---|---|---|---|
| 144 | 8 (160 ms) | 27.4 | 360 ms | 13.3 (13.7) | -0.9° | 2.4° | 3.1 / 1.0° | 3.6° | 362°/s | -15..13 mm |
| 160 | 9 (180 ms) | 33.6 | 400 ms | 14.9 (16.8) | -6.0° (caught at -2.3) | 3.1° | 2.6 / 1.2° | 4.1° | 369°/s | -20..15 mm |
| 170 | 10 (200 ms) | 39.1 | 440 ms | 16.0 (19.6) | -14° (caught) | 3.9° | 1.9 / 1.4° | 4.6° | 369°/s | -25..16 mm |
| 180 | 10 (200 ms) | 41.4 | 440 ms | 16.7 (20.7) | -16° (caught) | 3.9° | 0.9 / 1.4° | 4.6° | 380°/s | -26..17 mm |
| 190 | 11 (220 ms) | 47.5 | 480 ms | 17.9 (23.8) | -29° (caught) | 5.0° | 2.4 / 1.6° | 5.0° | 430°/s | -31..18 mm |
| 200 | 11 (220 ms) | 50.0 | 480 ms | 18.7 (25.0) | -32° (caught) | 5.0° | 1.4 / 1.6° | 5.4° | 451°/s | -33..19 mm |
| 207 (top) | 11 (220 ms) | 51.8 | 480 ms | 19.2 (25.9) | -33° (caught) | 5.0° | 3.4 / 1.7° | 5.6° | 451°/s | -35..19 mm |

Backward it goes to 180 mm/s (41 mm, 10 ticks): its pairs land further
forward (-10 mm instead of -21), where the knees work harder, and above
that the joints pass the bound. Steering keeps every joint under 483°/s
(the trot allows 480 above 184 mm/s; the extra is the FEET's 1 mm
rounding) and every foot in reach.

No case fell, caught a foot or held the gait, in either physics, with any
of the model errors. In RockingPhysics, the same long strides landing at
half a stride rock three to five times as much (roll rms 1.9 to 2.6° at
180 to 207 mm/s against 0.4 to 0.6), so the placement matters in both
physics; where exactly the best landing point is, they disagree by about
3 mm either side of the fitted one (and so may the real body, 15.6).

**The margins, read honestly.** By the fitted pendulum, a landing more than
about 0.5 mm off (11 ticks; 1 mm with today's 8) already sends him to the
2.3° catch: like today's trot above 160 mm/s, the long trot will run
against its early touchdowns much of the time, and that is how it stays up.
The useful margin is against what the floor has shown he survives: today's
backward trot at 144 mm/s lands 7.4 mm off its balanced point, a fitted tip
of 18° that the early touchdown holds at 2.3°, and it walks. The same 18°
is a landing 7.5 mm off with 8 ticks, 5.8 with 9, 4.6 with 10 and 3.6 with
11. So with 11 ticks the placement must be right to about ±3.5 mm, and the
fit's own uncertainty (the 3 mm the physics disagree by) uses most of
that: 11 ticks is the edge, which is why the prototype stops there. The
long stride landed at half a stride (33° at the top) is outside anything
seen on the floor. A swing of 12 ticks (a 56 mm stride) planned fine but
rocked 3.6° in RockingPhysics and has a margin of about 2.9 mm.

### 15.5 Stopping from the long stride

Both stops are the Planner's; the long trot only makes each square-up
step's swing as long as its distance needs (the feet are up to 35 mm
behind the shoulders), and a stop asked mid-swing keeps that swing's
landing. The quick stop from the long stride, asked at every second tick of
a cycle, in RockingPhysics with the momentum on (the stopping study's
switch, §14.4):

| speed | stride | stopped (median / worst) | way (median / worst) | done | peak tilt | fell |
|---|---|---|---|---|---|---|
| 144 | 27.4 mm | 0.18 / 0.20 s | 13 / 15 mm | 0.9 s | 3.2° | no |
| 180 | 41.4 mm | 0.34 / 0.56 s | 9 / 12 mm | 1.0 s | 4.7° | no |
| 207 | 51.8 mm | 0.58 / 0.64 s | 11 / 16 mm | 1.0 s | 4.8° | no |

The stride does not have to shorten first: the feet in the air come down
where they are and the strides go to zero in 6 ticks, as in §14. It takes a
little longer to be still (the square-up steps from a long stride shift
the body a few mm, which this measure counts) and goes no further. The
planned stop takes about 2 s from the top (the strides shrink at 0.75 mm a
tick), inside the walker's 4.2 s limit.

### 15.6 What is measured, what is assumed, and the floor test that would settle most

- **Measured:** the links, the masses and the centre of mass standing
  (2026-11-14); the trot's rock at nine speeds (15.3); that the servos
  followed 476°/s at 230 mm/s well enough to walk (no measurement of the
  joints themselves: the servos report no position).
- **Fitted:** the servo response (50 + 70 ms, §12.2); the pendulum's three
  numbers (15.3), from 8-tick swings only. Its use for 9 to 11 ticks is an
  extrapolation: the 120 ms of extra rocking may not stay 120 ms with a
  longer swing, and the 2.3° catch grows with the clearance (5.4 mm reached
  at 11 ticks against 4.7) or not.
- **Assumed:** the servos' speed under load; that the feet do not slip
  (longer strides slide the feet on the ground no faster, under 300°/s);
  that the early touchdown keeps catching a longer swing as it catches
  today's.

**The one measurement that settles the most** is the long trot's own rock
at one speed with a 10-tick swing: run it at 180 mm/s forward for 10 s and
fold the recording as 15.3 does (`stridesim.py fit` reads it). It needs the
long trot on Kilo (the C++ port and a key), which is the owner's call.
**With today's tools**, the measurement worth taking first is a
slow-motion phone video (240 fps if the phone has it) from the side, at
floor level, of the trot at level 5 (A4, then B7 four times: 230 mm/s),
with a tape measure on the floor: whether the swinging foot reaches its
lift and lands where it should, and how early the falling side's foot
touches. If the servos keep up with room to spare, a stride longer than 44
mm in today's 8 ticks may be possible with a firmware slew above 500°/s and
no balance cost at all; if they already lag, the long trot's longer swing
is the only way to a longer stride.

### 15.7 What was built

- `mind/kilo/planner/longtrot.py`: `LongTrotPlanner`, `LONG_TROT` (its own
  pattern object, not in `patterns.PATTERNS`), `TrotFit` (the fitted
  pendulum, `TROT_FIT`), the schedule (`swing_for`, `timing`). The
  existing planner, patterns, golden vectors and C++ mind are unchanged.
- `mind/kilo/vbody/rocking.py`: `RockingPhysics`; `FeetPlayer.PHYSICS`
  chooses the physics new FEET players use (Physics by default).
- `mind/tests/stridesim.py`: the study (`limits`, `fit`, `schedule`,
  `stops`); `mind/tests/test_long_trot.py`: the prototype's tests (reach,
  joints, the build-up, the placement, the fit against the recorded
  numbers, walks in both physics, both stops); `WalkSim` takes a
  `planner_factory`.
