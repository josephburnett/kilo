# Rear-up trick: sitting up on the haunches (B2)

Kilo's first authored trick, `rear_up`: from standing, Kilo sits back,
stretches its hind feet forward under its belly, puts its tail on the
floor behind, lifts its front paws and turns its body up to about 60
degrees, like a meerkat. It holds that for 2 seconds and comes back down
the same way to standing. One key starts it: **B2** (column B, row 2 from
the bottom). Pressing B2 again while it plays brings Kilo down at once.

This page has the measurements to take first (section 2), then the trials,
in stages, each with your hand ready behind Kilo (section 4). Nothing here
is timed: every step waits for you, and you can stop after any stage.
Write what you see in `docs/rear-up-trick-results.md`.

Words used below:

- **Pitch**: how far Kilo's body is tilted nose up or nose down. Level is
  0 degrees; "60 degrees nose up" means the body points two thirds of the
  way to straight up.
- **Centre of mass**: the point where Kilo's weight balances. Kilo stays up
  as long as this point is above the area between the things touching the
  floor (the **support**); past its edge, Kilo tips over that edge.
- **Tripod**: three things on the floor: the two hind feet and the tail
  tip. In the held pose the centre of mass is between them.
- **Rear end** (or butt): the bottom of the back end of Kilo's frame. It is
  a few mm above the floor in the held pose and catches Kilo if the tail
  does not.
- **Servo**: the motor in each joint. **Stall torque** is the most it can
  push; holding near it for long makes it hot.
- **Keyframe**: one pose in the trick. The body moves smoothly from one
  keyframe to the next.
- **Fall detector**: the body's own check. If Kilo tilts more than 75
  degrees for 0.12 s it decides Kilo has fallen, holds still, then lies
  down and goes limp.

## 1. What the trick does and why it is done this way

**Why a tripod.** Kilo cannot balance on its hind feet alone: they are
round (a 6 mm disc), so there is no area to balance over, and keeping a
body with its centre of mass 12 cm up on a point would need corrections
within about a tenth of a second. Kilo's servos answer in about 0.14 s
(measured on the walks), so that loop cannot be fast enough. The tail
weighs a few grams: swung as a counterweight it moves the centre of mass by
2 mm at most, against an uncertainty of 10 mm. So the tail is used as a
**prop**, the third leg of a tripod, and the whole trick is slow enough to
be still at every moment (every pose on the way is stable by itself). Petoi
says the same of the original Nybble: "you can ... make it stand up with
hind legs and tail" (the Nybble guide, chapter 7), with its balance on.

**The motion** (about 19 s; times at the end of each keyframe):

| # | Keyframe | What you see | ends at |
|---|---|---|---|
| 0 | tuck | hind legs fold and swing forward, the tail swings down to the floor; the rear sinks onto the tail | 2.4 s |
| 1 | crouch | hind feet further forward under the belly; body 12 degrees nose up | 3.6 s |
| 2 | settle | hind feet stretched right forward | 4.2 s |
| 3 | ready | body 25 degrees nose up on the tail, the hind feet and the front feet | 5.2 s |
| 4 | paws | the front paws come up and forward: Kilo sits on the tripod | 6.7 s |
| 5 | rise | the body turns up about the hind feet, the tail following the floor | 7.9 s |
| 6 | up | 60 degrees nose up, paws tucked, head tilted down to look ahead: held 2 s | 10.6 s |
| 7-13 | down | the same keyframes in reverse, ending in STAND | 19.3 s |

**How it comes down when it is cut short.** Whatever stops the trick (B2
again, B6, A7, C7, or the body losing the Pi for half a second) makes the
body move every joint in a straight line from where it is to STAND. For
this trick the mind asks for that move to take 1.5 s (the body's own
default is 0.2 s). The model checked that path from every keyframe: the
front feet reach the floor before the weight comes forward onto them, and
nothing tips backwards. If the body loses the Pi (the Pi hangs, the cable
comes out), the body does that same move in 0.2 s, which is a fast drop
onto the front paws, then lies down after 1 s.

**What protects Kilo.**

- The held pose is 60 degrees nose up, and the model never goes past 64
  over the ranges checked, so the fall detector (75 degrees) stays on: if
  Kilo really goes over backwards, the body still sees a fall.
- B2 refuses to start below 7400 mV on the battery (TELEMETRY's reading),
  so the body's low-battery reflex cannot fire half way up. That reflex
  takes Kilo to the lying-down posture from wherever it is, which from the
  top is a 30 degree drop forward (or, in one corner of the ranges,
  backwards).
- The trick never uses the body's balance loop (its corrections are made
  for four feet).

**What is not known**, and what the measurements in section 2 settle: which
way the tail turns, how long it is, where it points at 0, where the rear
end of the frame and the Pi are. The trick is solved for estimates of
those; the checks pass for the estimates plus or minus a ruler's worth
(section 5 has the numbers).

## 2. Measure first

Kilo standing (`kilo --host 192.168.88.28 posture stand`, or B6), servos on.
Measure from the side, with a ruler. "The shoulder line" is the straight
line through the axles of the front and back shoulder servos (the screws
the upper legs turn on).

**M1. Which way the tail turns (this decides whether the trick can work).**
Watch the tail while Kilo goes from stand to sit and back:

```
kilo --host 192.168.88.28 posture sit
kilo --host 192.168.88.28 posture stand
```

In sit the tail servo is at -60, in stand at 0. Does the tail swing
**down toward the floor** (up and down, in the plane of the legs) or
**sideways**? Does its tip touch the floor in sit?

- Down, and it touches in sit: the trick is as designed.
- Down but the other way (up over the back in sit): the trick's tail
  angles must be mirrored before any trial. Tell me; do not press B2.
- Sideways: the tail cannot be a prop. The model says Kilo would then sit
  on its rear end instead (section 5), but that has to be re-checked with
  the real rear end first. Tell me; do not press B2.

**M2. The tail's length**: from the tail servo's axle (the screw the tail
turns on) to the point of the tail that would touch the floor, in a
straight line, mm. (Estimate: 60 mm.)

**M3. Where the tail points at 0**: in stand (tail at 0), the angle
between the tail (axle to tip, straight) and the shoulder line, and
whether it points up or down from it. A photo from the side, square on,
with Kilo's whole body in it, answers M2-M5 at once if a ruler lies in the
picture. (Estimate: 30 degrees up from straight back.)

**M4. Where the tail's axle is**: how far behind the back shoulder axle,
and how far above or below the shoulder line, mm. (Estimate: 25 mm behind,
6 mm above.)

**M5. The rear end and the Pi**: how far the frame's back end is behind
the back shoulder axle, and how far its bottom corner is below the
shoulder line; how far the Pi's back edge (and its connectors) is behind
the back shoulder axle, and how high the highest point of the Pi (the
header pins, the USB and Ethernet sockets) is above the shoulder line, mm.
(Estimates: frame end 26 mm behind, bottom 24 mm below; Pi back edge 6 mm
ahead of the back axle, top 36 mm above.)

**M6. The tail's weight**, if it comes off easily: grams. (Estimate: 6 g.
It changes nothing much; skip it if it means taking Kilo apart.)

**M7. In sit, what touches the floor**: the tail tip, the rear end, the
hind feet? (The model says the tail tip and the front feet, with the hind
feet 12 mm up.)

How each changes the design: the tail's length and angle (M2-M4) set how
high the rear is held, so they move the held pitch by up to 5 degrees and
every tail angle in the trick; the trick is re-solved from them
(`python -m kilo.tricks.rear_up` with the numbers in
`kilo.tricks.sagittal.Geometry`, then the kernel tables regenerated and the
kernel updated). The rear end (M5) is the backstop if the tail slips; the
Pi's position (M5) says how far Kilo is from landing on it. M7 checks the
model against the real Kilo in a pose it already knows.

## 3. Before every trial

- **The Pi.** Tape a piece of foam or folded cardboard (two or three
  layers, a few cm thick) over the Pi on Kilo's back, covering its back
  edge and the pins. If Kilo goes over backwards, this is what lands.
- **The floor.** A hard floor with a thin rug or a yoga mat. Not a thick
  towel or a cushion: the tail tip and the round feet sink in and slide,
  and the trick relies on them staying put.
- **The battery** charged: `kilo --host 192.168.88.28 status` shows at
  least 7.8 V. (B2 refuses below 7.4 V.)
- **Your hand.** Kneel beside Kilo, one hand open a few cm behind its back,
  not touching. If it starts to go back, catch it; do not push it forward.
- **The way out.** Press **C7** (top right) to stop at once: Kilo comes
  down to standing over 1.5 s and then ignores keys until B6 or A7. If
  something is badly wrong, lift Kilo by the body and switch it off.

## 4. The trials, in stages

Do each stage until it looks the same twice, then go on. After every press
of B2 note what you saw in the results file. Between stages, let Kilo
stand for a minute: the hind servos work hard in this trick.

**Stage 0: the measurements.** Section 2. Do not press B2 before M1 says
the tail swings down.

**Stage 1: sitting back (keyframes 0-1).** Kilo standing. Press B2. When
the rear has sunk onto the tail and the front legs are still straight
(about 3 s), press B2 again: Kilo comes back to standing.
Watch: does the tail reach the floor before the rear comes down? Does the
rear end touch the floor? Does anything slide?
Stop if: the tail swings up or sideways, the rear drops hard, Kilo leans
to one side.

**Stage 2: ready (keyframes 0-3).** Press B2; when the body is tilted
about 25 degrees with the hind feet far forward and the front feet still
down (about 5 s), press B2 again.
Watch: the front feet should carry almost nothing now. If you slide a
sheet of paper under a front foot, does it pull out easily? Do the hind
feet slide forward? Is the rear end on the floor or just above it?
Stop if: Kilo rocks backwards when the front feet are light, or the hind
feet slide.

**Stage 3: paws up (keyframes 0-4).** Press B2; when the front paws have
lifted and come forward and Kilo sits on its hind feet and tail at 25
degrees (about 7 s), press B2 again.
Watch: does Kilo stay up on the tripod, or tip forward onto its paws? (Tipping
forward is the safe way: it means the weight is further forward than the
model, and the trick should move the hind feet; tell me.)
Stop if: it rocks backwards.

**Stage 4: all the way, your hand close behind.** Press B2 once and let
the trick play to the end (19 s). At the top Kilo should sit still at
about 60 degrees for 2 s.
Watch: the pitch at the top (`kilo --host 192.168.88.28 monitor` shows it:
TELEMETRY pitch, nose up is negative, about -58 to -60 expected), any
wobble, any slipping, a sideways lean, servo noise or heat afterwards
(touch the hind shoulder servos: warm is fine, too hot to hold is not).
Stop if: the top pitch reads more nose up than -68, Kilo wobbles
backwards, or a hind servo buzzes and gives way.

**Stage 5: the toggle.** Start it, and at the top press B2 again: Kilo
should come down to standing over 1.5 s. Then the same with B6 and with
C7.

**Stage 6: hand further away.** Repeat stage 4 a few times with your hand
farther back, still ready.

## 5. Recording

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

## 6. The mechanics (reference)

The numbers come from `python -m kilo.tricks.rear_up --report`; the model
is `mind/kilo/tricks/sagittal.py`, the design `mind/kilo/tricks/rear_up.py`,
the test `mind/tests/test_rear_up.py`.

**The model.** A side view: Kilo as one rigid outline per pose (both legs
of a pair together), on a flat floor. The masses are the planner's (358 g,
centre of mass 54 mm behind the front shoulder axles, 3 mm below them, with
the battery, the Pi and the legs as separate lumps that move with the
legs), the legs 45 + 50 mm with 6 mm round feet. The tail is a straight
lever (estimate: 60 mm, 6 g, axle 25 mm behind the back axles), the frame's
rear end and the Pi are corners of the outline. For each pose the model
finds the floor contact the body rests on; along a move it follows the
body as it rocks from one support to the next and notes each rock (a
"roll": how many degrees, and how far the point that lands falls). It is
static: no inertia, no servo lag, rigid servos and feet that do not slide.

**The held pose (estimates):** 59.4 degrees nose up, on the hind feet
(271 g) and the tail tip (87 g). The centre of mass is 38 mm behind the
hind feet and 108 mm ahead of the tail tip; sideways it is 23 mm inside the
triangle of feet and tail. The rear end of the frame is 6 mm above the
floor, its top corner 25 mm, the Pi 65 mm.

**Over the ranges** (the centre of mass 10 mm forward or back, 10 mm up or
down; the tail 55-65 mm long, pointing 25-35 degrees up at 0, its axle and
the rear end within 4 mm of the estimates; every corner, played from
STAND and from SIT and cut short half way to and at each keyframe; 924
runs): nothing but feet, tail and rear end ever touches the floor; the
largest roll backwards is the tail tip landing from about 12 mm while
sitting down (frame 1, centre of mass 10 mm back); the held pitch stays
between 57 and 62 degrees, never more than 64 anywhere; the tripod margin
at the top is 19 mm at worst.

**The tightest moment** is the paws coming up (keyframe 4): the centre of
mass must already be behind the hind feet. Nominally it is 20 mm behind;
with the centre of mass 10 mm further forward it is 10 mm (6 mm if it is
also 10 mm lower). Beyond
that (in the wider ranges below) the paws cannot come up: Kilo tips
forward onto them, and, with a long tail as well, onto its nose (a 5 cm
drop). That is the failure to watch for in stage 3; it is never backwards.

**Before the measurements** (wider: the tail 45-80 mm, 15-45 degrees, its
axle 10 mm either way, the rear end 8 mm): of the corners, two fail
forward (a long low tail with the centre of mass forward: the paws do not
come up, Kilo tips onto its paws and nose) and two sit down hard (a short
high tail with the centre of mass back: the rear lands from up to 28 mm).
None touches the floor with its back or the Pi. With no tail at all, or a
tail turning the wrong way, Kilo sits on its rear end and hind feet
instead: 65 or 54 degrees, 19 or 28 mm of margin, a 15 mm bump when the
rear lands. That is why M1-M5 come first.

**Servo torque.** The static torque each servo holds, worst over the ranges
and the whole motion, against an ESTIMATED stall torque of 3 kg cm (Petoi's
P1S as retailers list it; the MG92B-class servos of older Nybbles are
similar; Kilo's actual servos are not recorded in the repo):

| Joint | Worst, kg cm | When | of stall |
|---|---|---|---|
| hind shoulder | 1.40 | the tripod at 25 degrees (keyframes 3-4), the hind feet 84 mm ahead of their axles | 47 % |
| hind knee | 0.69 | the same | 23 % |
| front shoulder | 0.95 | the front feet folded back under the chest (keyframe 3) | 32 % |
| tail | 0.81 | the tail tip carrying a third of the weight | 27 % |
| front knee | 0.53 | | 18 % |

At the top the hind shoulders hold 0.87 kg cm (29 %). The hind shoulders
are the joint to watch: they hold almost half their stall torque for the
3 s around the paws coming up. If they give way, the hind feet fold back
and the rear sinks onto the rear end and the tail, which is still a
support.

**Joints and collisions.** Every angle is inside the body's limits (the
back shoulders reach their -80 limit in keyframes 1-2). The front and the
hind leg of a side, which move in the same plane, stay at least 17 mm apart
(keyframes 2-3, where the hind foot is forward and the front leg folded
back). The real body core played the trick in the tests: it reaches every
keyframe exactly, within its 10 degree per tick slew limit, in the 964
ticks the trick takes, and its TRICK_STOP crossfade is the straight line
the model assumed.

## 7. Feedback: what the trick does not do yet

The trick is open loop: it plays its keyframes whatever the body measures.
The body's balance loop is not used: its corrections are the legacy
per-joint gains made for four feet, and on the tripod they would move the
legs in ways nobody has checked. The firmware has no message for direct
joint goals from the mind, so a mind-side loop cannot stream joint angles.

What could close the loop without new firmware: at the top, the mind
streams FEET (the foot positions the body turns into joint angles, 50 Hz)
for the hind feet and holds the tail with SET_HEAD, with the body's balance
off, and trims the hind feet's fore-aft position (and the tail) from the
measured pitch: a slow loop (a correction every 0.5 s or so) that holds the
pitch at, say, 60 degrees against a centre of mass that is not where the
model thinks. The delay round that loop is about 0.25 s (the FEET queue,
the 140 ms servo lag, the TELEMETRY), fine for a pose that is stable on its
own and only drifts. It needs the measured offset between the model's
pitch and the IMU's (+1.7 to 2.0 degrees on the walks) and a test in the
real body core. Not built: the static tripod has 19 mm of margin, so the
first trials do not need it.

## When it goes wrong

- **B2 says "refused: battery ... mV"**: charge the battery.
- **B2 does nothing, the log says "ignored (behaviour runner active)"**:
  press B6 first (it stops the runner), then B2.
- **Kilo tips forward onto its paws at stage 3**: the centre of mass is
  further forward than the model. Note it; the fix is the hind feet
  further forward or the tail pressing less.
- **Kilo rocks backwards at any stage**: catch it, press C7, and stop the
  trials. Note at which keyframe.
- **The body says FELL**: Kilo went past 75 degrees. It holds still, then
  lies down and goes limp after 2 s. Set it upright by hand and press B6.
