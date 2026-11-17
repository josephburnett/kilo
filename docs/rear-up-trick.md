# Rear-up trick: sitting up on the hind feet and the tail (B2)

Kilo's first authored trick: from standing, Kilo lowers its rear until the
tip of its tail touches the floor, steps its hind feet forward under its
belly, lifts its front paws and tilts its body up. It sits there on three
points (the two hind feet and the tail tip) for 2 seconds, then goes back
down the same way and ends **sitting**. One key starts it: **B2** (column B,
row 2 from the bottom). Pressing B2 again while it plays brings Kilo down at
once.

It comes in **three stages**, three heights, so that you can try the low
one first and step up:

| Stage | Trick | Held at | Takes |
|---|---|---|---|
| 1 | `rear_up` | 28 degrees nose up: a low sit-up | 12 s |
| 2 | `rear_up_2` | 40 degrees | 15 s |
| 3 | `rear_up_3` | 54 degrees: the pose of your photo | 18 s |

Each stage does exactly what the stage below does and then goes higher, so
a good stage 1 is also a test of the start of stages 2 and 3.

**B2 is switched off.** Until you have done the trials in section 5, B2
does nothing, like the other keys of rows 1 to 3. Section 3 says how to
switch it on, and how to pick the stage.

Write what you see in `docs/rear-up-trick-results.md`. Nothing here is
timed: every step waits for you, and you can stop after any stage.

Words used below:

- **Pitch**: how far Kilo's body is tilted nose up or nose down. Level is 0
  degrees; 54 degrees nose up is a bit more than half way to straight up.
- **Centre of mass**: the point where Kilo's weight balances. Kilo stays up
  as long as this point is above the area between the things touching the
  floor (the **support**); past its edge, Kilo tips over that edge.
- **Tripod**: three things on the floor: the two hind feet and the tail tip.
  In the held pose the centre of mass is between them.
- **Margin**: how far the centre of mass is from the edge of the support,
  in mm. The bigger, the harder it is to tip Kilo over. Here the edge that
  matters is the line between the two hind feet: the margin is how far the
  centre of mass is behind it.
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
- **The model**: the program that works out, for any pose, what Kilo rests
  on and how far it is from tipping (`kilo.tricks.sagittal`, section 7).
  Numbers marked "model" come from it, not from Kilo.

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

So the pitch is set by how high the hind legs hold the hips: 51 mm for 28
degrees, 67 mm for 40, 83 mm for 54.

### Your photo: Kilo standing by itself at 54 degrees

On 2026-11-16 you posed Kilo by hand, unpowered, and it stood free on its
hind feet and its tail tip, with a pen wedged between the front of the body
and the hind knee to keep the limp knee from folding. The first design of
this trick (stage 1 alone) had stopped at 28 degrees because the model said
nothing much higher would stand. The photo says otherwise, so I treated it
as a measurement.

**What I read off the photo** (the side view, nose to the left). I read
angles, not lengths: an angle does not depend on how far the camera was.

- **Pitch: 54 degrees**, from three things that agree: the line through
  the near front and hind shoulder axles (54.1), the Pi's circuit board
  (53.6), and the tail (72 degrees below level, less the 18 degrees it
  droops below the body: 54). You said "about 60"; I take that as by eye.
  How sure: within about 4 degrees (the camera may be tilted or turned a
  little).
- **The near hind leg**: the upper leg goes forward and down from the hip
  at 39 degrees below level, the lower leg forward and down at 73 degrees
  (each within about 3 degrees). With the measured leg lengths (45 and 50
  mm) that puts the hip 82 mm above the floor, the knee bent 33 degrees,
  and the foot 50 mm in front of the hip. The tail tip then is 54 mm
  behind the hip.
- **A check that came out right**: the model says the tail tip touches the
  floor at 54 degrees when the hips are 82.5 mm up. The leg angles say 82.
  So the photo, the leg lengths and your tail measurements agree.
- **The front legs**: the upper leg forward and a little down (7 degrees
  below level), the lower leg up and forward (68 degrees above level), the
  paws high in front of the head.
- **The pen**: wedged between the front of the body and the hind knee, it
  does not touch the desk, so it only adds weight. I guess 40 g, its middle
  about 5 mm in front of the hind feet: that moves the balance point by less
  than 1 mm, too little to matter.
- The scale, for the record: about 6 pixels per mm at the near legs (the
  123 mm between the shoulder axles is 734 pixels), within about 10 %. The
  lengths above come from the angles, not from this.

**What the model says about that pose.** Without any change, the model
stands the photographed pose on the tail tip and the hind feet at 53.6
degrees, with the centre of mass 5 mm behind the hind feet (the tail tip
carries 17 g of Kilo's 358 g). So the model agrees with the photo. Two
things had made the first design give up above 35 degrees, and neither
holds:

1. **The legs can reach further.** The model limited the legs to 95 % of
   their full length, the limit the body uses when the mind sends foot
   positions while walking. A trick sends joint angles instead, and for
   those the body only checks each joint's limits. In the photo the hind
   leg is at 96 % of its length. The design now lets the hind legs reach as
   far as a knee bent 25 degrees (98 %), short of straight.
2. **The worst case it planned for cannot be.** The centre of mass is known
   to about 10 mm either way. The first design had to work even if it were
   10 mm further forward AND 10 mm lower than the model says. At 54 degrees
   that puts it 14 mm further forward, and the photo's pose would then tip
   forward off its hind feet. It did not, so that case is ruled out.

**The fit, and how sure it is.** No correction to the model is needed, so
none was made: `kilo.planner.model` (your measurement of 2026-11-14: 54 mm
behind the front axles, 3 mm below them) is unchanged, and so is the trick's
model. What the photo adds is a limit: at this pitch, moving the centre of
mass 1 mm forward moves it 0.59 mm toward the hind feet, and moving it 1 mm
down moves it 0.81 mm toward them. That sum can be at most about 10 mm (the
5 mm the photo shows, plus 5 mm for my reading of the photo). It limits only
one direction: the photo cannot say how much further back or higher the
centre of mass might be. It also rules out a tail that is much longer or
shorter than you measured (the photographed pose would then stand at a pitch
the photo does not show). The check is `kilo.tricks.sagittal.photo_check`;
the design is tested only over the cases the photo allows (section 7).

**What the photo changes about the pose.** At the top of stage 3 the trick
holds your photo's pose with two changes, both to gain margin:

- the hind feet 2 mm further forward (52 mm in front of the hips, the knee
  bent 26 degrees instead of 33);
- the front paws tucked back near the shoulders instead of reaching forward.
  Reaching forward, as in the photo, the front legs' weight is further
  forward. Tucked, the centre of mass is 4 mm further back. I did not fold
  them right back, which would gain 2 mm more: then a forward tip would go
  on over the paws onto the nose. Tucked as they are, a forward tip lands
  on the paws (section 7, "catch").

Together these put the centre of mass 6.5 mm further behind the hind feet
than in the pose Kilo stood in by itself.

### The stages, held

All from the model, nominal (the model's best guess of every unknown), at
the moment of holding:

| | Stage 1 (`rear_up`) | Stage 2 (`rear_up_2`) | Stage 3 (`rear_up_3`) |
|---|---|---|---|
| Pitch | 28 degrees | 40 | 54 |
| Hind axles above the floor | 51 mm | 67 | 83 |
| Hind feet in front of the hind axles | 78 mm | 68 | 52 |
| Hind knee bent | 38 degrees | 32 | 26 |
| Weight on the hind feet / the tail tip | 322 g / 36 g | 320 / 38 | 319 / 39 |
| Margin: centre of mass behind the hind feet | 16.5 mm | 14.8 | 11.6 |
| ... at worst over the uncertainties (section 7) | 6.4 mm | 5.6 | 3.5 |
| How far Kilo can lean forward before tipping | 11 degrees | 8 | 5 |
| Centre of mass in front of the tail tip | 146 mm | 125 | 94 |
| How far Kilo can lean back before tipping | 61 degrees | 49 | 35 |
| Sideways margin | 29 mm | 28 | 28 |
| Hind shoulder servo (stall ~3 kg cm, an estimate) | 1.18 kg cm, 39 % | 1.02, 34 % | 0.78, 26 % |
| Hind knee servo | 0.53 kg cm, 18 % | 0.43, 14 % | 0.29, 10 % |
| Front paws above the floor | 25 mm | 66 | 109 |
| Rear end above the floor | 17 mm | 32 | 47 |
| Pi above the floor | 86 mm | 99 | 109 |

Higher is easier on the hind servos: the legs stand straighter under the
body. The hind knee, the one the pen had to brace while Kilo was limp,
holds least of all: powered, it holds a tenth of its estimated stall at the
top. What gets thinner as Kilo goes up is the margin forward: at stage 3,
Kilo leaning 5 degrees further forward than the model says would tip it
onto its paws.

**The motion** (stage 3; the times are when each keyframe ends, from the
start):

| # | Keyframe | What you see | ends at |
|---|---|---|---|
| 0 | tuck | the hind feet move forward a little, the rear goes down, the front feet stay put | 0.8 s |
| 1 | sink | the rear goes down onto the tail tip; the body is 24 degrees nose up, on the front feet and the tail (the hind feet still touch) | 1.4 s |
| 2 | step | the hind feet lift 10 mm and swing forward under the belly | 2.8 s |
| 3 | ready | the hind feet come down, 78 mm in front of their axles; the front feet now carry little | 3.4 s |
| 4 | paws | the front paws lift: Kilo sits on the tripod, 24 degrees | 4.1 s |
| 5 | up | the hind legs push the hips up a little: 28 degrees (stage 1 holds here 2 s, then comes down: 12.2 s in all) | 4.7 s |
| 6 | rise | slowly, the hind legs straighten and draw the feet in: 34 degrees | 5.3 s |
| 7 | mid | 40 degrees (stage 2 holds here 2 s, then comes down: 14.9 s in all) | 6.1 s |
| 8 | climb | 47 degrees | 6.8 s |
| 9 | top | 54 degrees, held 2 s | 9.6 s |
| 10-18 | down | the same keyframes in reverse, back to the tuck | 16.5 s |
| 19 | sit | the hind legs fold, the tail swings aside, Kilo sits (the SIT posture) | 18.0 s |

From 28 degrees up (keyframes 6 to 9, and the same on the way down) the
joints move at half the speed of the rest, about 20 degrees per second.

**The tail tip slides.** On the tripod, the higher Kilo sits, the closer
together the hind feet and the tail tip have to be: the centre of mass
comes back over the hips as the body tilts up, so the feet must come back
with it. From 24 to 54 degrees the distance between them shrinks from 166
mm to 106 mm. The hind feet carry 89 % of the weight and the tail tip 11 %,
so it is the tail tip that slides forward, about 6 cm at stage 3 (2.6 cm at
stage 2, under 1 cm at stage 1), and back again on the way down. The model
assumes it slides freely. On a grippy mat it may stick and jump instead:
section 4 puts a sheet of paper under it.

**How it comes down when it is cut short.** Whatever stops the trick (B2
again, B6, A7, C7, or the body losing the Pi for half a second) makes the
body crossfade from where it is to **SIT**. For this trick the mind asks
for that move to take 1.5 s at stage 1, 2 s at stage 2 and 2.5 s at stage
3 (the body's own default is 0.2 s); the higher stages have further to
come down. It ends in SIT, not STAND, for a reason found in the model: the
straight line from the tripod to STAND straightens the hind legs while the
tail tip is still on the floor. That pushes the hips up and tilts Kilo
further back instead of bringing the front feet down. The line to SIT folds
the hind legs and swings the tail aside (its tip lifts off the floor), so
the rear sinks and Kilo always ends sitting on its four feet. Coming down
from the top this way, the front paws drop onto the floor from up to 26 mm
in the model (at stage 3). If the body loses the Pi (the Pi hangs, the cable
comes out), the body does the same move in 0.2 s, a faster drop onto the
paws, then lies down after 1 s.

**What protects Kilo.**

- Backwards there is a lot of room: at stage 3 the centre of mass is 94 mm
  in front of the tail tip, and Kilo would have to be tilted back 35
  degrees further to go over. Forwards it is 11.6 mm (5 degrees), and
  forwards means the paws: tipped forward at any stage's top, Kilo lands on
  its front paws and stops there on four feet, nose up, never on its nose
  (from 25 mm at stage 1, 66 mm at stage 2, 11 cm at stage 3).
- Nothing in the model ever touches the floor but the feet, the tail tip
  and (in a few cases) the rear end: never the back, the Pi or the nose.
- The pitch never goes past 57 degrees in any case checked, so the fall
  detector (75 degrees) stays on: if Kilo really goes over backwards, the
  body still sees a fall.
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
  turn it further out. If it did give way at the top of stage 3, the tip
  would rise and Kilo would rock back: turned 60 degrees aside, the model
  has Kilo at 76 degrees, on its hind feet and the side of the tail, which
  the fall detector would call a fall.

Where this comes from in the code: `kilo.tricks.sagittal.Geometry` (the
numbers), `tail_point` and `tail_cone` (the sweep).

**What to check or measure.** Kilo standing (`kilo --host 192.168.88.28
posture stand`, or B6), servos on. "The hind axles" are the screws the
upper hind legs turn on.

**T1. A last look at the tail.** Your description and the photo settle the
geometry; this is only a check that the model moves the same way. Watch the
tail go from stand to sit (`kilo --host 192.168.88.28 posture sit`): in sit
the tail is at -60, so its tip should be well to one side and higher than
in stand (the model: 76 mm to the side, 28 mm higher). If the tip goes
*down* as it swings, tell me before switching B2 on.

**T2. Which side is "left".** In sit (tail at -60), is the tip to Kilo's
left or its right (as Kilo sees it, facing forward)? The trick does not
need this (it keeps the tail at 0); the model records it for any later use
of the tail sideways.

**T3. The rear end of the frame**: how far the bottom back corner of the
frame is behind the hind axles, and how far below them, mm. (Estimate: 27
mm behind, 24 mm below. At the holds it is 17, 32 and 47 mm above the
floor; this is the backstop if the tail slips.)

**T4. The Pi**: how far the Pi's back edge (and its connectors) is behind
or in front of the hind axles, and how high its highest point (the header
pins, the USB and Ethernet sockets) is above them, mm. (Estimate: 6 mm in
front, 36 mm above. At the holds it is 86 to 109 mm above the floor. Your
photo suggests the board's back edge is further forward than that, which
would be safer; I did not use it.)

**T5. In sit, what touches the floor**: the hind feet, the rear end, the
front feet? (The model says the four feet, with the rear end 3 mm up and
the tail tip 26 mm up.)

**T6. The tail's weight**, only if it comes off easily: grams. (Estimate:
6 g. It changes almost nothing.)

How each changes the design: T1 confirms the tail is a strut. T3 and T4 say
how far Kilo is from landing on its rear end or its Pi. T5 checks the model
against Kilo in a pose it already knows. New numbers go into
`kilo.tricks.sagittal.Geometry`, then `python -m kilo.tricks.rear_up`
re-solves the three stages and the kernel tables are regenerated
(`kernel/mind/tricks/gen_library.py`, `body/tools/gen_trick_library.py`).

## 3. The B2 switch, and the stage

B2 is off unless it is switched on, in both places the remote is handled.
The switch's value is the stage: 1, 2 or 3.

- **The mind kernel on the Pi** (what normally runs): build it with the
  switch and update the Pi. For stage 1:

  ```
  make -C kernel KILO_REAR_UP=1
  ```

  For stage 2, `KILO_REAR_UP=2`; for stage 3, `KILO_REAR_UP=3`. At boot it
  logs which: `Kilo mind, build ... (rear-up key B2 on: rear_up_2, stage
  2)`. Built without it (`make -C kernel`), B2 is off again. Any other value
  stops the build with an error.
- **The laptop daemon** (`kilo run --remote`), if you use it instead: start
  it with the switch in its environment:

  ```
  KILO_REAR_UP=1 kilo run --remote
  ```

  (or `=2`, `=3`).

With the switch off, pressing B2 logs `remote B2 (0x38, none): ignored (no
action on this key)` and nothing moves. With it on, B2 logs `remote B2
(0x38, rear up): trick rear_up started (12 s; ...)` at stage 1, and `rear up
2` / `rear_up_2`, `rear up 3` / `rear_up_3` at the others.

Switch it on only for the trials of section 5, and leave it on afterwards
only at a stage that has gone well.

## 4. Before every trial

- **The Pi.** Tape a piece of foam or folded cardboard over the Pi on
  Kilo's back, covering its back edge and the pins. The model says the Pi
  stays far from the floor, but this is what lands if Kilo goes over
  backwards.
- **The tail.** Check it is firmly on its servo and points straight back
  in stand: it carries up to a fifth of Kilo's weight while the hind feet
  step (77 g in the model) and about 38 g at every hold.
- **The floor.** A hard floor or desk with a thin mat (a yoga mat or a thin
  rug) under the feet. Not a thick towel or a cushion: the tail tip and the
  round feet sink in, and the trick relies on them staying where they are.
  For stages 2 and 3, tape a sheet of paper to the mat under where the tail
  tip lands and along 8 cm in front of it: the tail tip has to slide forward
  there as Kilo rises (section 1).
- **The battery** charged: `kilo --host 192.168.88.28 status` shows at
  least 7.8 V. (B2 refuses below 7.4 V.)
- **Your hands.** Kneel beside Kilo. One hand open a few cm behind its
  back, not touching: going over backwards is unlikely but is the worst
  fall (onto the Pi). The other hand open a few cm in front of its chest,
  lower as the stages go up: forward is the likely way to fail, onto the
  paws. If it starts to go, catch it; do not push it.
- **The way out.** Press **C7** (top right) to stop at once: Kilo comes
  down to sitting over 1.5 to 2.5 s and then ignores keys until B6 or A7.
  If something is badly wrong, lift Kilo by the body and switch it off.

## 5. The trials, in stages

Do each step until it looks the same twice, then go on. After every press
of B2 note what you saw in the results file. Between steps, let Kilo rest
for a minute: the hind servos work hard in this trick.

### Stage 1 (`KILO_REAR_UP=1`, 28 degrees)

**Step 0: check and measure.** Section 2, T1 at least. Then switch B2 on at
stage 1 (section 3).

**Step 1: onto the tail (keyframes 0-1).** Kilo standing. Press B2. When
the rear has gone down onto the tail tip and the body is tilted nose up
with all four feet still down (about 1.5 s), press B2 again: Kilo comes
down to sitting.
Watch: does the tail tip touch the floor? Does the tail stay pointing
straight back, or does it get pushed aside? Does anything slide?
Stop if: the tail swings aside under the load, the rear drops hard, Kilo
leans to one side.

**Step 2: the step (keyframes 0-3).** Press B2; when the hind feet have
stepped forward under the belly and come down (about 3.5 s), press B2
again.
Watch: while the hind feet are in the air, Kilo stands on its front feet
and its tail; does it stay still? After they come down, the front feet
should carry little: if you slide a sheet of paper under a front foot, does
it pull out easily? Do the hind feet slide forward?
Stop if: Kilo rocks backwards, or the hind feet slide.

**Step 3: paws up (keyframes 0-4).** Press B2; when the front paws have
lifted (about 4 s), press B2 again.
Watch: does Kilo stay up on the tripod, or do the paws stay on the floor,
or come back down? Staying on the paws is the safe failure: it means the
centre of mass is further forward than the model. Tell me; the fix is to
move the hind feet.
Stop if: it rocks backwards.

**Step 4: all the way, your hands close.** Press B2 once and let the trick
play to the end (12 s). At the top Kilo should sit still at about 28
degrees for 2 s, then come back down and sit.
Watch: the pitch at the top (`kilo --host 192.168.88.28 monitor` shows it:
TELEMETRY pitch, nose up is negative, about -25 to -31 expected), any
wobble, any slipping, a sideways lean, servo noise or heat afterwards
(touch the hind shoulder servos: warm is fine, too hot to hold is not).
Stop if: the top pitch reads more nose up than -36, Kilo wobbles
backwards, or a hind servo buzzes and gives way.
After every play, whatever happened: `kilo runs --host kilo.local --fetch
last` (section 6) keeps the whole trial and prints where it left the plan.

**Step 5: the toggle.** Start it, and at the top press B2 again: Kilo
should come down to sitting over 1.5 s. Then the same with B6 (it sits,
then stands up) and with C7.

**Step 6: hands further away.** Repeat step 4 a few times with your hands
farther away, still ready.

### Stage 2 (`KILO_REAR_UP=2`, 40 degrees)

Only after stage 1 has gone well. Build with `KILO_REAR_UP=2` (section 3).
The paper under the tail tip from now on (section 4).

**Step 7: the rise.** Press B2. The first 4.7 s are stage 1's way up; then,
slowly, the hind legs straighten and Kilo tilts up to 40 degrees. Press B2
again as soon as it gets there (about 6 s).
Watch: the tail tip should slide forward smoothly, about 2.5 cm. Does it
slide, or stick and jump? Does the tail stay pointing straight back? Do the
hind feet stay put?
Stop if: the tail tip sticks and the body jerks, the tail is pushed aside,
or Kilo tips forward onto its paws (tell me: the centre of mass is further
forward than the model says).

**Step 8: all the way.** Press B2 once and let it play (15 s): 2 s held at
40 degrees.
Watch: the pitch at the top (`monitor`: about -37 to -43), wobble, the
hind knees holding their bend, servo heat. Then the toggle at the top (B2
again: down over 2 s).
Stop if: the top pitch reads less nose up than -35 (Kilo is further forward
than the model and closer to tipping onto its paws) or more than -46 (the
tail is giving way), Kilo rocks, or a knee gives.

### Stage 3 (`KILO_REAR_UP=3`, 54 degrees: your photo's pose)

Only after stage 2 has gone well. This is the thin one: in the model
Kilo is 11.6 mm from tipping forward at the top, 3.5 mm in the worst case
the photo allows, and leaning 5 degrees further forward would tip it. Your
front hand stays close, under the chest, the whole time.

**Step 9: the rise to the top.** Press B2. After stage 2's way up it goes on
to 47 and then 54 degrees (about 7.6 s), and holds 2 s. Press B2 as soon
as it arrives.
Watch: the tail tip slides forward about 6 cm in all; Kilo should look
like your photo, front paws tucked under the chin instead of reaching out.
Stop if: anything from step 7 or 8; or the tail tip runs out of paper.

**Step 10: all the way, then the toggle and C7 at the top** (18 s; down
over 2.5 s).
Watch: the pitch at the top (`monitor`: about -51 to -57), a slow forward
creep (the paws sinking toward the floor), the hind knees (bent 26
degrees: they must not straighten further or buckle), servo heat.
Stop if: the top pitch reads less nose up than -49 (forward, toward the
paws) or more nose up than -60 (the tail giving way: Kilo rocks back
toward the fall detector's 75), any wobble, a knee giving.

**Step 11: hands further away.** Only if step 10 went well three times.

## 6. Recording: fetch a trial and read it

The mind records everything from the moment it starts (the *flight
recorder*, `kernel/README.md`): every message Kilo's board sends (each
TELEMETRY, 25-50 a second, with the measured pitch and roll, the mode, the
battery) and every command the mind sends, every remote key and what the
remote did with it, every log line. Each trick played is a *run* in it:
which trick and stage, the keyframes' durations, the moment the board
started playing, the keys pressed during it, and how it ended. A trick the
remote refuses (low battery) is a run too, so a press that did nothing
still shows why. Nothing has to be switched on before a trial.

The recording is in the Pi's memory (several hours of it). It survives
switching Kilo off only once saving to the SD card is on (step 3 below).

After a trial, with Kilo still on and the network cable in (`kilo.local`
finds him; an IP address works too):

1. List the runs:

   ```
   kilo runs --host kilo.local
   ```

   One line per walk or trick: its number, when it started, the trick and
   stage, how long it lasted, how it ended (`DONE`, `STOPPED`, `FELL`,
   `REFUSED`, ...).
2. Download the last one (or `--fetch 3` for run 3, `--fetch all`) and
   read its analysis:

   ```
   kilo runs --host kilo.local --fetch last
   ```

   It writes `~/kilo-logs/run-<date>-<time>-trick-<number>.krec` and
   prints the analysis. Read it again later, with a tick-by-tick table and
   a plot:

   ```
   kilo log trick ~/kilo-logs/run-....krec --csv trial.csv --plot trial.png
   ```

3. Once `kilo bench sd --host kilo.local` has shown the card is fast enough
   (`kernel/README.md` "Saving to the SD card"), `kilo record save on
   --host kilo.local` keeps the recordings across a power-off; the next
   boot lists them as `b<boot>:<run>` and fetches them the same way.

What the analysis says (pitch is given **nose up positive**, the design's
convention; TELEMETRY's own pitch has the opposite sign):

- **keyframes**: each keyframe's name (tuck, sink, step, ready, paws, up,
  ...; the way down as `down:...`), when it started after the play began,
  the pitch the design predicts at its end and the pitch measured, and the
  mean and largest difference (*error* = measured minus predicted) over it;
- the error over the whole trick (mean, rms, largest);
- the highest nose-up pitch and the largest roll, and the fastest pitch and
  roll rates, with their times;
- the **tip-over onset**: the first moment the error (or the roll) stayed
  beyond 8 degrees for 0.2 s, with the keyframe and the direction (back,
  forward, left, right): where Kilo left the plan;
- falls the board reported, the lowest battery voltage, how it ended and
  why, and which key stopped it.

The prediction for each keyframe comes from the design's model
(`python -m kilo.tricks.rear_up` writes it into the library's JSON as
`predicted_pitch_deg`), and between keyframes it follows the same cosine
easing the board plays. The CSV's columns are `t_s, uptime_s,
since_play_s, mode, flags, keyframe, keyframe_name, body_keyframe,
nose_up_deg, pred_nose_up_deg, err_deg, pitch_deg, roll_deg,
rate_pitch_dps, rate_roll_dps, batt_mv, dist_mm` (`mind/README.md` "The
flight recorder" explains each). For a stretch of time instead of a run
(say Kilo did something odd while standing): `kilo record fetch --host
kilo.local --last 10m`, then `kilo record decode FILE --csv out.csv`.

## 7. The mechanics (reference)

Everything in this section is from the model (simulated), not from Kilo,
except where it says "photo". The numbers come from `python -m
kilo.tricks.rear_up --report`; the model is `mind/kilo/tricks/sagittal.py`,
the design `mind/kilo/tricks/rear_up.py`, the test
`mind/tests/test_rear_up.py`.

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
servos, and the contacts slide freely (the tail tip's slide, section 1).
The sideways margin is measured in the triangle of the two hind feet (65 mm
apart) and the tail tip.

A round foot holds Kilo up at the point under its centre. Until the photo,
the model measured the margin to a corner of the many-sided shape it draws
for the foot, up to 3 mm too generous; it now measures to the point under
the centre. That is why stage 1's numbers here are a little smaller than
before (16.5 mm instead of 19 at the hold); stage 1's keyframes are
unchanged.

**The ranges checked** (the gate the test holds every stage to): the
centre of mass 10 mm forward or back and 10 mm up or down; the tail 91-97
mm long, 13-23 degrees down at 0, its pivot within 3 mm of the measurement
either way, its axis 45-55 degrees above straight back; the rear end within
4 mm of the estimate. Every end of every range, and every corner of the
centre of mass with the tail's length and droop: 43 cases, of which the
photo rules out 3 (the centre of mass 10 mm forward and 10 mm low, with
some of the tail's corners). Each stage is played from STAND and from SIT
and cut short half way to and at each keyframe: 768 runs for stage 1, 1024
for stage 2, 1280 for stage 3. Over these, at every stage:

- nothing but the feet, the tail tip and the rear end ever touches the
  floor; every run ends down on its feet (sitting);
- every play holds the tripod at the top (in the first design, without the
  photo, a few corners kept the paws down);
- the largest roll backwards is 4 degrees (a 10 mm drop of the rear end,
  cut short while the hind feet step); the largest drop forwards is 21, 22
  and 26 mm by stage (the paws landing, cut short at the top);
- the held pitch stays within 25-31, 37-43 and 51-57 degrees by stage,
  never more than 57 anywhere;
- the margin at the hold is at least 6.4, 5.6 and 3.5 mm by stage, the
  sideways margin at least 25, 24 and 22 mm.

**Where the thin cases are.** Every thin margin is the centre of mass
further forward and lower than the model, which the photo bounds but cannot
rule out entirely. At stage 3 the worst is the centre of mass 10 mm lower
(3.5 mm left); 10 mm forward leaves 5.7 mm. They all fail forward, onto the
paws. Backward the margin never drops below 90 mm.

**Wider.** With every corner of the centre of mass and of the tail's
length, droop and pivot together (64 corners, 18 ruled out by the photo),
stage 1's margin goes down to 3.4 mm in one corner (the gate wants 5). With
the tail measured even less well than assumed (PLAUSIBLE: 85-103 mm, 8-28
degrees, its pivot 8 mm either way; 13 of 27 cases ruled out by the photo),
stage 1's margin goes down to 2 mm in one corner and, in one corner where
the centre of mass is also 10 mm back, cutting the trick short while the
hind feet step lands the rear from 16 mm. Nothing touches the floor with
the back or the Pi in any of them. Stages 2 and 3 over the same grids:
every play still holds the tripod at the top; the thinnest margins are 2.9
mm (stage 2) and 2.5 mm (stage 3) in one corner each of the 64 (the centre
of mass forward and low with the tail's pivot high), and 2.2 and 3.5 mm over
PLAUSIBLE; the 16 mm drop of the rear is the same one as stage 1's (it
happens while the hind feet step, before the stages part). The pitch never
passes 58 degrees. If the low-battery reflex took Kilo to REST from the top
of stage 3 (B2 refuses to start on a low battery, so this needs the pack to
sag during the trick), the paws would land from up to 31 mm.

**A forward tip at the top ("catch").** If Kilo goes forward over its hind
feet at a stage's top (a push, a wobble, a centre of mass further forward
than the model), it turns nose down about its hind feet until the front
paws land, and stays there, on four feet, a little nose down: from 25 mm
at stage 1, 66 at stage 2, 109 at stage 3. With the paws folded right back
it would go on over them onto the nose; reaching forward as in the photo,
the margin would be 4 mm smaller.

**Servo torque.** The static torque each servo holds, worst over the ranges
and the whole motion, against an ESTIMATED stall torque of 3 kg cm (Petoi's
P1S as retailers list it; Kilo's actual servos are not recorded in the
repo). The worst is on the low tripod (keyframes 3 to 5), which every stage
goes through; the higher holds are easier (the table in section 1):

| Joint | Worst, kg cm | When | of stall |
|---|---|---|---|
| hind shoulder | 1.32 | on the tripod at 24-28 degrees (keyframes 3-5) | 44 % |
| hind knee | 0.61 | the same | 20 % |
| tail | 0.68 | cut short early: the tail swings aside towards SIT while its tip is still loaded | 23 % |
| front shoulder | 0.56 | cut short at stage 2 or 3's top: the paws land | 19 % |
| front knee | 0.28 | | 9 % |

The tail servo holds nothing at a hold (section 2), but its shaft carries
the tail's load: 36 to 39 g at the holds, 77 g (a fifth of Kilo) while the
hind feet step.

**Joints and collisions.** Every angle is inside the body's limits (the
back shoulders reach their -80 limit in the sink and in SIT). The hind legs
reach up to 97 % of their length (stage 3's top), more than the 95 % the
body allows foot positions while walking: trick frames are joint angles,
and the body applies only the joint limits and its 10 degree per tick slew
limit to them. The front and the hind leg of a side, which move in the same
plane, stay at least 23 mm apart. The real body core played every stage in
the tests: it reaches every keyframe exactly, within its slew limit, and
its TRICK_STOP crossfade is the straight line the model assumed (SIT's back
shoulders at -90 are held at their -80 limit, which the model follows).

**The head.** The model leaves the head's weight in the frame: turning it
moves the centre of mass by under 1 mm, so the trick only tilts it a little
for the look of it.

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
  narrows the triangle on one side and widens it on the other: it moves
  where Kilo can lean, it does not push Kilo back upright. And the sideways
  margin is already 28 mm: the risk is forward, not sideways.
- It also changes the pitch: turned aside, the tip rises, and Kilo settles
  further nose up (at stage 3, 3 degrees for 20 degrees of tail).
- The tip would have to slide sideways on the floor under load, and the
  servo would then hold the floor's push (0.17 kg cm at 20 degrees at stage
  3).

So I do not propose it. What would help more, without new firmware: at the
top, the mind streams FEET (the foot positions the body turns into joint
angles, 50 times a second) for the hind feet, with the body's balance off,
and slowly moves the hind feet forward or back from the measured pitch (a
correction every half second): that holds the pitch, and so the margin,
against a centre of mass that is not where the model thinks. The delay
round that loop is about 0.25 s, fine for a pose that is stable on its own
and only drifts. It needs the measured offset between the model's pitch and
the IMU's (+1.7 to 2.0 degrees on the walks), a FEET reach limit that lets
the hind legs as far as the trick takes them (stage 3 is past the 95 %), and
a test in the real body core. Not built: the trials come first.

## When it goes wrong

- **B2 does nothing, the log says "B2 (0x38, none): ignored"**: the switch
  is off (section 3).
- **B2 says "refused: battery ... mV"**: charge the battery.
- **B2 does nothing, the log says "ignored (behaviour runner active)"**:
  press B6 first (it stops the runner), then B2.
- **The paws stay on the floor at step 3**: the centre of mass is further
  forward than the model. Note it; the fix is the hind feet further forward
  or the hold lower.
- **Kilo tips forward onto its paws at stage 2 or 3**: the same, higher up.
  Fetch the run (section 6): its "tip-over onset" line says at which
  keyframe and time Kilo left the plan, and which way; stay at
  the stage below until I have looked.
- **The tail tip sticks and jumps during the rise**: more paper, or a
  smoother surface under the tail tip only.
- **The tail gets pushed aside**: stop the trials and tell me: the tail is
  not a strut on your Kilo, and the trick has to be redesigned.
- **Kilo rocks backwards at any stage**: catch it, press C7, and stop the
  trials. Note at which keyframe.
- **The body says FELL**: Kilo went past 75 degrees. It holds still, then
  lies down and goes limp after 2 s. Set it upright by hand and press B6.
