# Walking and balance: what we have, why the trot tips, what the model should be

Written 2026-10-13 after the first floor drives (trot at 1000 ms with
balance on: "wobbles so much when walking that he tipped over"; tricks fine).
Numbers use Kilo's measured geometry: legs 45 + 50 mm, body 123 mm long,
feet 65 mm apart, body height 0.80 × 95 = 76 mm, centre of mass about 80 mm
up. Code: `body/src/core/gait.cpp`, `balance.cpp`, `body.cpp` (the wiring);
design: `docs/architecture.md` §3d, §3f.

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
   `kilo walk --speed MM_S`, and the remote's C5 / C1 (slower / faster, 0.5x
   to 2x of the default speed). Each pattern keeps its default's swing time
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
and `physics.py`; how to use it: mind/README.md "Walking".

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

**The stream's timing.** A thread in the process that walks (the daemon or
`kilo walk`) wakes every 20 ms. The body's clock is estimated from the
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
