# Rear-up trick: sitting back on the hind feet and the tail (B2)

Kilo's first authored trick, `rear_up`: from standing, Kilo lowers its rear
until the tip of its tail touches the floor, steps its hind feet forward
under its belly, lifts its front paws and tilts its body up to 28 degrees.
It sits there on three points (the two hind feet and the tail tip) for 2
seconds, then goes back down the same way and ends **sitting**. One key
starts it: **B2** (column B, row 2 from the bottom). Pressing B2 again
while it plays brings Kilo down at once.

**B2 is switched off.** Until you have done the trials in section 4, B2
does nothing, like the other keys of rows 1 to 3. Section 3 says how to
switch it on for the trials.

This is a smaller trick than the first design (a 60 degree "meerkat"). That
design assumed the tail swings down to the floor. You measured the tail on
2026-11-16 and it does not: it swings left and right (section 2). With the
tail as it really is, 28 degrees is about the highest Kilo can sit with a
safe margin (section 1 says why).

Write what you see in `docs/rear-up-trick-results.md`. Nothing here is
timed: every step waits for you, and you can stop after any stage.

Words used below:

- **Pitch**: how far Kilo's body is tilted nose up or nose down. Level is 0
  degrees; 28 degrees nose up is a bit less than a third of the way to
  straight up.
- **Centre of mass**: the point where Kilo's weight balances. Kilo stays up
  as long as this point is above the area between the things touching the
  floor (the **support**); past its edge, Kilo tips over that edge.
- **Tripod**: three things on the floor: the two hind feet and the tail tip.
  In the held pose the centre of mass is between them.
- **Margin**: how far the centre of mass is from the edge of the support,
  in mm. The bigger, the harder it is to tip Kilo over.
- **Strut**: a stiff prop. The tail is used as one: it does not move, it
  just holds the rear up.
- **Rear end**: the bottom of the back end of Kilo's frame.
- **Servo**: the motor in each joint. **Stall torque** is the most it can
  push; holding near it for long makes it hot.
- **Keyframe**: one pose in the trick. The body moves smoothly from one
  keyframe to the next.
- **Crossfade**: when a trick is stopped, the body moves every joint in a
  straight line, all at the same pace, from where it is to a fixed posture.
- **Fall detector**: the body's own check. If Kilo tilts more than 75
  degrees for 0.12 s it decides Kilo has fallen, holds still, then lies
  down and goes limp.

## 1. What the trick does and why it is done this way

**Why a tripod.** Kilo cannot balance on its hind feet alone: they are round
(a 6 mm disc), so there is no area to balance over, and Kilo's servos answer
too slowly (about 0.12 s) to balance it actively. So the trick rests on
three points and is slow enough to be still at every moment: every pose on
the way is stable by itself. Petoi says the original Nybble can "stand up
with hind legs and tail" (the Nybble guide, chapter 7).

**How the tail is used.** The tail swings left and right around a tilted
axis (section 2). It is lowest when it points straight back (servo angle 0):
its tip is then 98 mm behind and 5 mm below the axles of the hind legs
(with Kilo level). It cannot be swung down any further. So the trick keeps
the tail at 0 the whole time and uses it as a strut. To put its tip on the
floor, Kilo tilts its body nose up and lowers its hips. The tip touches when
the hind axles are at this height above the floor:

> hip height = 98 x sin(pitch) + 5 x cos(pitch) mm

| Pitch | Hip height when the tail touches | Best margin to the hind feet (model) |
|---|---|---|
| 20 deg | 38 mm | 22 mm |
| 25 deg | 46 mm | 21 mm |
| 30 deg | 54 mm | 21 mm |
| 35 deg | 61 mm | 16 mm |
| 40 deg | 67 mm | 16 mm |
| 45 deg | 73 mm | 15 mm |
| 50 deg | 79 mm | 12 mm |
| 55 deg | 84 mm | 11 mm |
| 60 deg | 88 mm | not possible |

The last column is from the model: the best margin found between the
centre of mass and the hind feet, over every hind-foot position the legs can
reach and every way of holding the front paws. The higher the pitch, the
higher the hips must be, and the less far forward the hind feet can reach.
Above about 57 degrees the hind feet cannot get in front of the centre of
mass at all.

**Why 28 degrees and not more.** The centre of mass is known to about 10 mm
either way, and the tail to a few mm. If the centre of mass is 10 mm
further forward and 10 mm lower than the model says, it sits about 14 mm
further forward at these pitches, and a less accurate tail costs about 5 mm
more. At 25 to 30 degrees that still leaves a few mm; above 35 degrees it
leaves nothing. If the margin runs out, Kilo does not fall over backwards:
it tips forward onto its front paws (that is the safe way). The trick holds
the paws just above the floor (25 mm) so that this is a short drop.

**Verdict.** A static tripod with this tail works, at a low pitch. In the
model (not yet tried on Kilo), at the hold:

- pitch 28 degrees nose up, the hind axles 51 mm above the floor;
- the hind feet carry 322 g (90 %), the tail tip 36 g (10 %);
- margin: the centre of mass is 19 mm behind the hind feet (the edge it
  would tip forward over) and 146 mm in front of the tail tip; sideways it
  is 28 mm inside the triangle of feet and tail;
- the paws are 25 mm above the floor, the rear end 17 mm, the Pi 86 mm.

**The motion** (about 12 s; times at the end of each keyframe):

| # | Keyframe | What you see | ends at |
|---|---|---|---|
| 0 | tuck | the hind feet move forward a little, the rear goes down, the front feet stay put | 0.8 s |
| 1 | sink | the rear goes down onto the tail tip; the body is 24 degrees nose up, on the front feet and the tail (the hind feet still touch) | 1.4 s |
| 2 | step | the hind feet lift 10 mm and swing forward under the belly | 2.8 s |
| 3 | ready | the hind feet come down, 78 mm in front of their axles; the front feet now carry little | 3.4 s |
| 4 | paws | the front paws lift: Kilo sits on the tripod | 4.1 s |
| 5 | up | the hind legs push the hips up a little: 28 degrees, held 2 s | 6.7 s |
| 6-10 | down | the same keyframes in reverse, back to the tuck | 10.7 s |
| 11 | sit | the hind legs fold, the tail swings aside, Kilo sits (the SIT posture) | 12.2 s |

**How it comes down when it is cut short.** Whatever stops the trick (B2
again, B6, A7, C7, or the body losing the Pi for half a second) makes the
body crossfade from where it is to **SIT**. For this trick the mind asks for
that move to take 1.5 s (the body's own default is 0.2 s). It ends in SIT,
not STAND, for a reason found in the model: the straight line from the
tripod to STAND straightens the hind legs while the tail tip is still on the
floor. That pushes the hips up and tilts Kilo further back instead of
bringing the front feet down, and in over a quarter of the cases checked Kilo
ended up stuck at about 40 degrees, propped on the tail and its straight
hind legs, paws in the air. The line to SIT folds the hind legs and swings
the tail aside (its tip lifts off the floor), so the rear sinks and Kilo
always ends sitting on its four feet. Coming down from the top this way the
front paws drop 3 mm in the model, 22 mm at worst. If the body loses the Pi
(the Pi hangs, the cable comes out), the body does the same move in 0.2 s, a
faster drop onto the paws, then lies down after 1 s.

**What protects Kilo.**

- Backwards there is a lot of room: the centre of mass is 146 mm in front
  of the tail tip, and Kilo would have to be tilted back 60 degrees further
  to go over. Forwards it is 19 mm (13 degrees), and forwards means the paws.
- Nothing in the model ever touches the floor but the feet, the tail tip
  and (in a few cases) the rear end: never the back, the Pi or the nose.
- The pitch never goes past 43 degrees in any case checked, so the fall detector (75
  degrees) stays on: if Kilo really goes over backwards, the body still sees
  a fall.
- B2 refuses to start below 7400 mV on the battery (TELEMETRY's reading),
  so the body's low-battery reflex cannot fire half way up.
- The trick never uses the body's balance loop (its corrections are made
  for four feet).

## 2. The tail, and what is still to measure

**What you measured** (2026-11-16, your words): "24 mm above the rear leg
axis and 9 mm behind. And the tail protrudes out at an 18 degree angle. But
the motor is actually angled down at 40 degrees. The tail is 94 mm in
length. So to model where the tip of the tail is (goes left and right)
sweeps back and forth along an arc around a center point ~50mm above the
axis of the motor."

Then, describing the angles: "If the motor shaft was pointing straight up,
then tip it 40 degrees down (toward the back), that's where it sits. And
same for the tail. If it was sticking straight out level pointing straight
back, then you tip it down toward the ground to 18 degrees, that's where it
is."

**What the model takes from that.**

- The tail's pivot (where the servo's axis meets the middle of the body) is
  24 mm above and 9 mm behind the axles of the hind legs, with Kilo level.
- The servo's axis lies in the middle of the body (in the plane of the
  spine), tipped 40 degrees back from vertical: it points backward and up,
  50 degrees above straight back.
- With the tail straight back (servo at 0), the tail points backward and 18
  degrees below straight back.
- So the tail and the axis are 68 degrees apart. Turning the servo swings
  the tail round the axis like the side of a cone: the tip goes round a
  circle of 87 mm radius whose centre is 35 mm along the axis from the
  pivot. (Your "~50 mm" was a rough figure; the model uses the angles.)
- The tip is lowest with the tail straight back: 89 mm behind and 29 mm
  below the pivot (98 mm behind and 5 mm below the hind axles). Turned 90
  degrees to either side, it is 87 mm to the side, 23 mm behind and 27 mm
  above the pivot. The tail cannot be driven down; it can only move the tip
  sideways and up.
- With the tail straight back, the servo itself holds nothing while the tip
  is on the floor: the push of the floor goes through the servo's shaft and
  bearing, not round it. Turned aside under load, the floor's push tries to
  turn it further out.

Where this comes from in the code: `kilo.tricks.sagittal.Geometry` (the
numbers), `tail_point` and `tail_cone` (the sweep).

**What to check or measure.** Kilo standing (`kilo --host 192.168.88.28
posture stand`, or B6), servos on. "The hind axles" are the screws the
upper hind legs turn on.

**T1. A last look at the tail.** Your description settles the geometry;
this is only a check that the model moves the same way. Watch the tail go
from stand to sit (`kilo --host 192.168.88.28 posture sit`): in sit the
tail is at -60, so its tip should be well to one side and higher than in
stand (the model: 76 mm to the side, 28 mm higher). If the tip goes *down*
as it swings, tell me before switching B2 on.

**T2. Which side is "left".** In sit (tail at -60), is the tip to Kilo's
left or its right (as Kilo sees it, facing forward)? The trick does not
need this (it keeps the tail at 0); the model records it for any later use
of the tail sideways.

**T3. The rear end of the frame**: how far the bottom back corner of the
frame is behind the hind axles, and how far below them, mm. (Estimate: 27
mm behind, 24 mm below. In the hold it is 17 mm above the floor; this is
the backstop if the tail slips.)

**T4. The Pi**: how far the Pi's back edge (and its connectors) is behind
or in front of the hind axles, and how high its highest point (the header
pins, the USB and Ethernet sockets) is above them, mm. (Estimate: 6 mm in
front, 36 mm above. In the hold it is 86 mm above the floor.)

**T5. In sit, what touches the floor**: the hind feet, the rear end, the
front feet? (The model says the four feet, with the rear end 3 mm up and
the tail tip 26 mm up.)

**T6. The tail's weight**, only if it comes off easily: grams. (Estimate:
6 g. It changes almost nothing.)

How each changes the design: T1 confirms the tail is a strut.
T3 and T4 say how far Kilo is from landing on its rear end or its Pi. T5
checks the model against Kilo in a pose it already knows. New numbers go
into `kilo.tricks.sagittal.Geometry`, then `python -m kilo.tricks.rear_up`
re-solves the trick and the kernel tables are regenerated
(`kernel/mind/tricks/gen_library.py`, `body/tools/gen_trick_library.py`).

## 3. The B2 switch

B2 is off unless it is switched on, in both places the remote is handled:

- **The mind kernel on the Pi** (what normally runs): build it with the
  switch and update the Pi:

  ```
  make -C kernel KILO_REAR_UP=1
  ```

  At boot it then logs `Kilo mind, build ... (rear-up key B2 on)`. Built
  without it (`make -C kernel`), B2 is off again.
- **The laptop daemon** (`kilo run --remote`), if you use it instead: start
  it with the switch in its environment:

  ```
  KILO_REAR_UP=1 kilo run --remote
  ```

With the switch off, pressing B2 logs `remote B2 (0x38, none): ignored (no
action on this key)` and nothing moves. With it on, B2 logs `remote B2
(0x38, rear up): trick rear_up started (12 s; ...)`.

Switch it on only for the trials of section 4, and leave it on only once
stage 6 has gone well.

## 4. Before every trial

- **The Pi.** Tape a piece of foam or folded cardboard over the Pi on
  Kilo's back, covering its back edge and the pins. The model says the Pi
  stays far from the floor, but this is what lands if Kilo goes over
  backwards.
- **The tail.** Check it is firmly on its servo: it carries up to a fifth
  of Kilo's weight while the hind feet step (78 g in the model).
- **The floor.** A hard floor with a thin rug or a yoga mat. Not a thick
  towel or a cushion: the tail tip and the round feet sink in and slide,
  and the trick relies on them staying put.
- **The battery** charged: `kilo --host 192.168.88.28 status` shows at
  least 7.8 V. (B2 refuses below 7.4 V.)
- **Your hand.** Kneel beside Kilo, one hand open a few cm behind its back,
  not touching, the other a few cm in front of its chest. If it starts to
  go back or sideways, catch it; do not push it.
- **The way out.** Press **C7** (top right) to stop at once: Kilo comes
  down to sitting over 1.5 s and then ignores keys until B6 or A7. If
  something is badly wrong, lift Kilo by the body and switch it off.

## 5. The trials, in stages

Do each stage until it looks the same twice, then go on. After every press
of B2 note what you saw in the results file. Between stages, let Kilo rest
for a minute: the hind servos work hard in this trick.

**Stage 0: check and measure.** Section 2, T1 at least. Then switch B2 on
(section 3).

**Stage 1: onto the tail (keyframes 0-1).** Kilo standing. Press B2. When
the rear has gone down onto the tail tip and the body is tilted nose up
with all four feet still down (about 1.5 s), press B2 again: Kilo comes
down to sitting.
Watch: does the tail tip touch the floor? Does the tail stay pointing
straight back, or does it get pushed aside? Does anything slide?
Stop if: the tail swings aside under the load, the rear drops hard, Kilo
leans to one side.

**Stage 2: the step (keyframes 0-3).** Press B2; when the hind feet have
stepped forward under the belly and come down (about 3.5 s), press B2
again.
Watch: while the hind feet are in the air, Kilo stands on its front feet
and its tail; does it stay still? After they come down, the front feet
should carry little: if you slide a sheet of paper under a front foot, does
it pull out easily? Do the hind feet slide forward?
Stop if: Kilo rocks backwards, or the hind feet slide.

**Stage 3: paws up (keyframes 0-4).** Press B2; when the front paws have
lifted (about 4 s), press B2 again.
Watch: does Kilo stay up on the tripod, or do the paws stay on the floor,
or come back down? Staying on the paws is the safe failure: it means the
centre of mass is further forward than the model. Tell me; the fix is to
move the hind feet.
Stop if: it rocks backwards.

**Stage 4: all the way, your hands close.** Press B2 once and let the
trick play to the end (12 s). At the top Kilo should sit still at about 28
degrees for 2 s, then come back down and sit.
Watch: the pitch at the top (`kilo --host 192.168.88.28 monitor` shows it:
TELEMETRY pitch, nose up is negative, about -26 to -30 expected), any
wobble, any slipping, a sideways lean, servo noise or heat afterwards
(touch the hind shoulder servos: warm is fine, too hot to hold is not).
Stop if: the top pitch reads more nose up than -36, Kilo wobbles
backwards, or a hind servo buzzes and gives way.

**Stage 5: the toggle.** Start it, and at the top press B2 again: Kilo
should come down to sitting over 1.5 s. Then the same with B6 (it sits,
then stands up) and with C7.

**Stage 6: hands further away.** Repeat stage 4 a few times with your hands
farther away, still ready.

## 6. Recording

The mind records walks only (`kilo walks --host ... --fetch`). A trick is
not recorded. For now, `kilo --host 192.168.88.28 monitor` during a trial
shows the pitch the body measures, and the log line
`remote B2 (0x38, rear up): trick rear_up started` marks the start in
`kilo log --host`. What a trick recording would take, if the trials need
it: the recorder already keeps every link frame of a walk; starting a
recording at TRICK_PLAY and ending it after TRICK_DONE (with the trick's
name, frame times and the model's predicted pitch per keyframe in its
header) would let `kilo log` plot measured against predicted pitch, as it
does for walks. About a day's work; not built.

## 7. The mechanics (reference)

Everything in this section is from the model (simulated), not from Kilo.
The numbers come from `python -m kilo.tricks.rear_up --report`; the model is
`mind/kilo/tricks/sagittal.py`, the design `mind/kilo/tricks/rear_up.py`,
the test `mind/tests/test_rear_up.py`.

**The model.** A side view: Kilo as one rigid outline per pose (both legs
of a pair together), on a flat floor. The masses are the planner's (358 g,
centre of mass 54 mm behind the front shoulder axles, 3 mm below them, with
the battery, the Pi and the legs as separate lumps that move with the
legs), the legs 45 + 50 mm with 6 mm round feet. The tail is the cone of
section 2 (its tip's height and its sideways position from the servo
angle; 6 g, estimated). The frame's rear end and the Pi are corners of the
outline (estimates, T3 and T4). For each pose the model finds what the body
rests on; along a move it follows the body as it rocks from one support to
the next and notes each rock (a "roll": how many degrees, and how far the
point that lands falls). It is static: no inertia, no servo lag, rigid
servos and feet that do not slide. The sideways margin is measured in the
triangle of the two hind feet (65 mm apart) and the tail tip.

**The ranges checked** (the gate the test holds the design to): the centre
of mass 10 mm forward or back and 10 mm up or down; the tail 91-97 mm long,
13-23 degrees down at 0, its pivot within 3 mm of the measurement either
way, its axis 45-55 degrees above straight back; the rear end within 4 mm of the estimate.
Every end of every range, and every corner of the centre of mass with the
tail's length and droop; played from STAND and from SIT, and cut short half
way to and at each keyframe: 840 runs. Over these:

- nothing but the feet, the tail tip and the rear end ever touches the
  floor; every run ends down on its feet (sitting);
- the largest roll backwards is 4 degrees (a 10 mm drop, cut short while
  stepping); the largest drop forwards is 22 mm (the paws landing, cut
  short at the top);
- the held pitch stays between 25 and 31 degrees, never more than 38
  anywhere;
- at the hold the tripod margin is 7.6 mm at worst, the sideways margin
  25 mm at worst;
- in 2 of the 16 corners (the centre of mass 10 mm forward AND 10 mm low,
  AND the tail drooping 23 degrees) the paws cannot come up: Kilo stays on
  its four feet and the tail at -6 degrees, with no drop. That is the safe
  failure stage 3 watches for.

**Wider.** With every corner of the centre of mass and of the tail's
length, droop and pivot together (64 corners), the paws stay down in 7 of
them, always the same way (the centre of mass forward and low, the tail
drooping). If the measurements are off by more than that (the tail
85-103 mm, 8-28 degrees, its pivot 8 mm either way), the paws stay down in
more corners and, in two corners where the centre of mass is also 10 mm
back, cutting the trick short while the hind feet step lands the rear from
16-25 mm; nothing touches the floor with the back or the Pi.

**Servo torque.** The static torque each servo holds, worst over the ranges
and the whole motion, against an ESTIMATED stall torque of 3 kg cm (Petoi's
P1S as retailers list it; Kilo's actual servos are not recorded in the
repo):

| Joint | Worst, kg cm | When | of stall |
|---|---|---|---|
| hind shoulder | 1.33 | on the tripod at 24-28 degrees (keyframes 3-5) | 44 % |
| hind knee | 0.61 | the same | 20 % |
| tail | 0.67 | cut short early: the tail swings aside towards SIT while its tip is still loaded | 22 % |
| front shoulder | 0.48 | the paws pressed on the floor at the top, in a corner where they cannot come up | 16 % |
| front knee | 0.28 | | 9 % |

At the top the hind shoulders hold 1.18 kg cm (39 %), for about 3.5 s
(keyframes 3 to 6 and the hold). The tail servo holds nothing at the top
(see section 2), but its shaft carries the tail's load: 36 g at the top, 78 g
(a fifth of Kilo) while the hind feet step, up to 105 g over the ranges.

**Joints and collisions.** Every angle is inside the body's limits (the
back shoulders reach their -80 limit in the sink and in SIT). The front and
the hind leg of a side, which move in the same plane, stay at least 24 mm
apart (the step, where the hind foot is forward and the front leg reaches
back). The real body core played the trick in the tests: it reaches every
keyframe exactly, within its 10 degree per tick slew limit, and its
TRICK_STOP crossfade is the straight line the model assumed (SIT's back
shoulders at -90 are held at their -80 limit, which the model follows).

## 8. Feedback, and the tail as a balance (proposals, not built)

The trick is open loop: it plays its keyframes whatever the body measures.
The body's balance loop is not used: its corrections are made for four
feet, and on the tripod they would move the legs in ways nobody has
checked. The firmware has no message for direct joint goals from the mind.

**Balancing with the tail sideways.** You asked for Kilo "balanced by the
tail". In this trick the tail balances Kilo by being the third leg. Swinging
it sideways to correct a sideways lean does very little:

- The tail weighs about 6 g. Swung 20 degrees, it moves Kilo's centre of
  mass sideways by a quarter of a millimetre.
- On the floor, swinging it 20 degrees moves the tip 30 mm sideways. That
  narrows the triangle on one side (the sideways margin there drops from 28
  to 23 mm) and widens it on the other: it moves where Kilo can lean, it
  does not push Kilo back upright. And the sideways margin is already
  28 mm: the risk is forward, not sideways.
- It also changes the pitch: turned aside, the tip moves forward and up
  (5 mm at 20 degrees), and in the model Kilo settles 1.7 degrees further
  nose up.
- The tip would have to slide sideways on the floor under load, and the
  servo would then hold the floor's push (0.09 kg cm at 20 degrees).

So I do not propose it. What would help more, without new firmware: at the
top, the mind streams FEET (the foot positions the body turns into joint
angles, 50 times a second) for the hind feet, with the body's balance off,
and slowly moves the hind feet forward or back from the measured pitch (a
correction every half second): that holds the pitch, and so the margin,
against a centre of mass that is not where the model thinks. The delay
round that loop is about 0.25 s, fine for a pose that is stable on its own
and only drifts. It needs the measured offset between the model's pitch and
the IMU's (+1.7 to 2.0 degrees on the walks) and a test in the real body
core. Not built: the trials come first.

## When it goes wrong

- **B2 does nothing, the log says "B2 (0x38, none): ignored"**: the switch
  is off (section 3).
- **B2 says "refused: battery ... mV"**: charge the battery.
- **B2 does nothing, the log says "ignored (behaviour runner active)"**:
  press B6 first (it stops the runner), then B2.
- **The paws stay on the floor at stage 3**: the centre of mass is further
  forward than the model. Note it; the fix is the hind feet further forward
  or the hold lower.
- **The tail gets pushed aside**: stop the trials and tell me: the tail is
  not a strut on your Kilo, and the trick has to be redesigned.
- **Kilo rocks backwards at any stage**: catch it, press C7, and stop the
  trials. Note at which keyframe.
- **The body says FELL**: Kilo went past 75 degrees. It holds still, then
  lies down and goes limp after 2 s. Set it upright by hand and press B6.
