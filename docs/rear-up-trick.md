# Rear-up trick: sitting up on the hind feet and the tail (A1)

Kilo's first authored trick: Kilo sits back on its two hind feet and the tip
of its tail, front paws off the floor, holds, comes back down and ends
**sitting**. One key starts it: **A1** (column A, the bottom row; the owner's "A", the first experiment key; A2 until 2026-11-22).
It goes up **one stage at a time**: A1 takes him to stage 1 and he holds it;
A1 again while he holds takes him on to the next stage, as high as the build
allows (section 3). B6 or C7 bring him down from any stage.

**This is the third design (2026-11-21).** The second design took 12.5 s to
its first hold, stepping the hind feet forward one at a time, and on
2026-11-21 its own checks stopped it three times out of three
(`docs/rear-up-trick-results.md`). The third design gets onto the tripod in
under 3 s by crouching and pushing itself back with the front legs, your
idea. Section 1 says what he does and why it is fast now.

**It still needs the right hind shoulder re-seated** (section 2, "The right
hind shoulder"). The plays since 2026-11-19 look as if that was done (the
model reads it from how he sat and rolled), but nobody wrote it down: please
check `kilo cal show --host kilo.local` (joint 10 between +7 and +25) before
the first try. Played before the fix, the trick stops itself in the crouch
and Kilo sits down (section 8).

It comes in **three stages**, three heights:

| Stage | Trick | Held at (TELEMETRY nose up, model) | Paws up | Holding by | Takes, straight up and down |
|---|---|---|---|---|---|
| 1 | `rear_up` | 23 degrees: a low sit-up | 1.9 s | 3.0 s | 6.3 s |
| 2 | `rear_up_2` | 36 degrees | 1.9 s | 7.9 s | 12.1 s |
| 3 | `rear_up_3` | 50 degrees: the pose of your photo | 1.9 s | 13.0 s | 18.3 s |

Each stage does exactly what the stage below does on the way up and then
goes higher. In `rear_up_2` and `rear_up_3` each lower stage's top is held
for 4 seconds: that is the time to press A1 to go on.

**Try stage 1 first.** It is the one the checks hold to every case they know
(section 7). Stages 2 and 3 reach their tops in the model's fitted case and
in most of its other cases, and in the rest a check stops them and he sits
down.

**A1 is switched off** until you switch it on (section 3). Write what you
see in `docs/rear-up-trick-results.md`. Nothing here is timed: every step
waits for you.

Words used below:

- **Pitch**: how far Kilo's body is tilted nose up. Level is 0 degrees.
  The numbers here are what `kilo log trick` and TELEMETRY show (nose up),
  which reads 1.9 degrees nose down when Kilo stands level.
- **Roll**: how far Kilo leans to one side. Positive: the left side down.
  TELEMETRY reads about +2 degrees where the model has Kilo level (an offset
  in the IMU, or a left leg 2 mm short: the recordings cannot tell which).
- **Centre of mass**: the point where Kilo's weight balances.
- **Support**: the area on the floor between the things touching it, seen
  from above. Kilo stays up while its centre of mass is over that area; past
  its edge it tips over that edge.
- **Tripod**: the two hind feet and the tail tip on the floor.
- **Servo**: the motor in each joint. Its **pulse** is the signal that sets
  its angle; the body sends between 500 and 2500 microseconds, and the
  **calibration** says which pulse is which angle for each joint.
- **Give** (compliance): a servo holds its angle only so stiffly; under load
  it gives a little. Kilo's leg servos give about 4 degrees for every kg cm
  of load (measured from the recordings, section 7).
- **Keyframe**: one pose in the trick; the body moves smoothly between them.
- **Guard**: a check the Pi makes while the trick plays: at a keyframe
  where Kilo holds still, the measured pitch must be inside a window and the
  roll small; if not, the Pi stops the trick and Kilo sits down (section 8).
- **The model**: the program that plays the trick on a simulated Kilo, with
  his weights, his leg lengths, the give of his servos and their delay, and
  tells what he stands on, at what pitch and roll, at every moment
  (`kilo.tricks.dynamics`, section 7). Numbers marked "model" come from it,
  not from Kilo.
- **Case** (band): one way the real Kilo may differ from the model's best
  fit (the centre of mass a few mm off, the servos softer, the floor more
  slippery, ...). The checks play the trick in each case.

## 1. How the third design gets up

**Why a tripod.** Kilo cannot balance on his round hind feet alone, and his
servos answer too slowly to balance him actively. So he sits on three
points, the two hind feet and the tail tip, with his weight between them.
The tail swings left and right around a tilted axis (section 2) and is
lowest when it points straight back (servo angle 0); the trick keeps it
there as a stiff prop.

**What he does** (stage 1; the times are when each step ends; "model" pitch
in the fitted case; a guard is a check, section 8):

| # | Keyframe | What you see | Pitch (model) | Ends at |
|---|---|---|---|---|
| 0 | look | the head comes up; nothing else moves | -1.9 | 0.3 s |
| 1 | crouch | the hind knees fold, the body goes down and a little back; a short pause | -1.8 | 0.8 s |
| 2 | back | the front legs push the body back over the hind feet: he rocks back onto his rear (the back corners of the frame under the battery; the tail tip close above the floor or touching) and the front feet go light; **holds 0.4 s: guard** | +7 (it rocks to +11 and settles) | 1.6 s |
| 3 | paw_l | the left paw folds up in front of the chest | +7 | 1.8 s |
| 4 | paws | the right paw folds up too; **holds 0.4 s: guard** | +6 | 2.5 s |
| 5 | up | the hind legs lengthen: the hips rise until the tail tip carries the rear; on the tripod, held still 2 s; **guard** | +23 | 5.0 s |
| 6 | settle | the hind legs fold and he sits back on his rear, paws still up | +11 | 5.5 s |
| 7 | drop | the paws come down to the floor in front of him | +6 | 5.8 s |
| 8 | sit | SIT | +27 | 6.3 s |

From "paws" on, the front paws are off the floor (1.9 s in the model) and he
is on the tripod by 2.8 s; the hold starts at 3.0 s. The hind feet never
leave the floor, and they slide less than a centimetre.

Like a cat: the head comes up first, a short stillness before the push, one
paw folds before the other, he holds still on the tripod, and on the way
down the paws come down to the floor quickly in front of him.

**Why it is fast now.** The second design moved the hind feet forward one
at a time while standing (6.6 s), because a rigid model said he would tip
over any other way, and then lowered him slowly onto the tail. The third
design leaves the hind feet where they stand and moves the body instead:
the hind knees fold, and the front legs push the body back over the hind
feet until it rocks back onto its rear. That rock is the thing that went
"wrong" on 2026-11-21; it is now used on purpose, where the checks expect
it.

**What 2026-11-21 taught** (section 7 has the numbers):

1. **Kilo's legs give.** Under load each leg servo gives about 4 degrees
   per kg cm. When his weight moves onto the hind legs they give, he tips
   nose up, which moves more weight onto them, which makes them give more:
   he rocks back by himself. A model with rigid legs cannot see this; it had
   him 15 mm away from rocking at "gather", where he rocked every time. So
   every check now runs in a model with the give in it, fitted to the
   recordings.
2. **His centre of mass is about 5.5 mm further back** than the planner's
   model. That made "lift_r" (the right hind foot in the air) a coin flip:
   3 of 7 plays went over. (That fit also put it 5 mm to the right, and the
   tripod leaned him left against it, the left hind leg 2.5 mm shorter; the
   self-measurement of 2026-11-22 found it in the middle, and that lean
   tipped him left twice: docs/rear-up-trick-results.md. Both hind legs are
   alike now.) On the tripod a millimetre of leg or of centre of mass to one
   side is worth about a degree of roll, and the legs' give makes it grow,
   so the Pi watches the roll while the hind legs push (section 8).
3. Since 2026-11-22 the trick is judged on the one body model
   (`kilo.tricks.dynamics.NOMINAL`), the same numbers the walks are planned
   with (docs/model-fit-results.md). The walks' recordings suggest the left
   hind leg gives more than the right; the model does not carry that yet,
   because with it the check predicts this stage still tips him left. The
   next play of stage 1 is the test of it.

**How it comes down.** At the end of the hold, the way up backwards: the
hind legs fold and he sits back on his rear ("settle"), the paws come down
to the floor in front of him ("drop"), and he sits (SIT). A1 again, B6, A7,
C7, or the body losing the Pi make the body move every joint in a straight
line to SIT instead, over 1.5 s (stage 1), 2 s (stage 2) or 2.5 s (stage 3).
The model checks that line from half way into and the end of every
keyframe: he ends sitting on his feet every time (section 7).

**What protects Kilo.**

- The guards (section 8). With the right hind shoulder not re-seated, the
  model has him leaning left 9 degrees in the crouch: the Pi stops him, he
  rolls up to 17 degrees while he comes down, and ends sitting.
- The pitch never goes past 55 degrees; the fall detector (75 degrees)
  stays armed.
- A1 refuses to start below 7400 mV on the battery at rest (the highest
  reading of the last 20 seconds); his eyes then give three quick purple
  flashes (`docs/signals.md`).

### Your photo: Kilo standing by itself at 54 degrees (2026-11-16)

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

**What the photo gives the design.** Stage 3's top is close to the photo's
pose: 54 degrees by the model's own pitch, the hind feet 51 mm ahead of the
hips (the photo: 50), the paws folded up in front instead of reaching
forward. The photo also bounds the centre of mass: every case the checks
use is one the photo allows.

## 2. The tail, and the right hind shoulder

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

The frame's rear corner and the Pi's outline are still estimates (they
never touch the floor in any case checked).

### The right hind shoulder (do this before switching A1 on)

**What is wrong.** Each servo turns as far as its pulse says, and the body
keeps every pulse between 500 and 2500 microseconds (the legacy firmware
did the same). The calibration of the right hind shoulder (joint 10) is -16
degrees: its upper leg sits on the servo's splined shaft 16 degrees off,
toward the back. So at -57 degrees (upper leg forward) its pulse is already
2500 and the leg goes no further; the left one, calibrated +2, reaches -80.
The walk and the trot never ask more than -54 of it (the top speed), so
nobody noticed; SIT asks -80 (so in SIT the right hind leg is 23 degrees
short too), the crawl at its top speed -79, the rear-up trick -78.

**How I know joint 10 is the right hind upper leg.** The protocol's joint
table names joint 10 `rb_shoulder` (right back) and the body drives it on
servo channel 6 (`protocol/spec.toml` [[joints]], `body/src/core/joints.cpp`;
the legacy OpenCat order, which the EEPROM's offsets follow). And it fits
what happened: in the trials of 2026-11-17 the right hind leg was the one
that did not come forward (the video, `docs/rear-up-trick-results.md`), and
the same arithmetic on joint 12 explains your P1 finding that the left front
knee does not fold as far at REST. Step 2 below checks it on Kilo before
anything comes off.

**What offset to aim for** (arithmetic from the firmware and the EEPROM, not
measured on Kilo). With offset `o`, joint 10 reaches forward to -73.3 - o
degrees and back to +109.1 - o, and never past its limits, -80 and +100:

| Offset | Reaches | Pulse at 0 degrees | |
|---|---|---|---|
| -16 (now) | -57.3 .. +100 | 1872 us | SIT, the crawl and the rear-up fall short |
| 0 | -73.3 .. +100 | 1696 us | still 7 degrees short forward |
| +7 | -80 .. +100 | 1620 us | the least that reaches -80 |
| +9 | -80 .. +100 | 1598 us | the most that still reaches +100 |
| +16 (the target) | -80 .. +93 | 1521 us | |
| +18 | -80 .. +91 | 1500 us | the servo's middle |
| +25 (the body's limit) | -80 .. +84 | 1422 us | |

What is asked of it: forward -80 (SIT, the rear-up, the crawl); backward
nothing past -8 while walking, 0 in the calibration pose, and only the
getting-up-after-a-fall moves (+100) and the legacy push-up trick (+95) go
further. Only offsets +7 to +9 reach everything, a 2-degree window, smaller
than one tooth of the spline, so the leg cannot be put on to hit it. The
forward end is the one Kilo uses every time he sits; the back end only when
he pushes himself over after a fall on his side. So aim forward of the
window: **+16**. Wherever the teeth then put the leg, within half a tooth
(8 or 9 degrees) the offset that makes it straight lands between +7 and
+25, which always reaches -80; at worst the fall move stops at +84 instead
of +100.

Your idea of putting the shaft at the middle of its travel first is nearly
this: offset 0 is not the middle for this joint (1696 us, because of the
legacy firmware's built-in shift for this servo); the middle, 1500 us, is
offset +18. +16 is 21 us from it.

**The procedure** (your idea; your hands, your tools, any time; nothing here
is timed, and every command can be run again):

1. Switch Kilo on and lay him on his back, or hold him, so the legs hang
   free. In one window hold the calibration pose (every joint at 0, the
   upper legs straight down), and leave it running:

       kilo posture calib --host kilo.local

   It holds until you press Ctrl-C there. While it holds, the remote does
   nothing and Kilo cannot fall asleep.
2. In a second window, check which leg is joint 10:

       kilo cal nudge 10 5 --host kilo.local
       kilo cal nudge 10 -5 --host kilo.local

   The right hind upper leg should turn 5 degrees back, then return. If
   another leg moves, stop and tell me.
3. Put the shaft where a straight-down leg will be:

       kilo cal set 10 16 --host kilo.local

4. Take out the screw in the middle of the right hind shoulder and pull the
   upper leg straight off the shaft (the servo holds the shaft; do not turn
   it). Press it back on pointing straight down, as close as the teeth
   allow, and put the screw back in.
5. Nudge it until it points straight down like the left one; each command
   prints the new offset:

       kilo cal nudge 10 1 --host kilo.local     turns it back (toward the tail)
       kilo cal nudge 10 -1 --host kilo.local    turns it forward

   If it is straight with the offset between +7 and +25, go on. Below +7:
   the leg sits a tooth too far back; take it off again, put it on one
   tooth further forward, and go back to step 3. If a nudge is refused at
   +25 and the leg is still not straight: one tooth further back, step 3.
6. Save:

       kilo cal save --host kilo.local

   It answers `saved to the body's EEPROM`.
7. Press Ctrl-C in the first window (Kilo lies down), then check:

       kilo posture sit --host kilo.local

   Looking from above, both hind upper legs should point forward the same
   amount. Ctrl-C ends it.
8. `kilo cal show --host kilo.local`: tell me joint 10's offset. Its line
   should no longer say SHORT under postures for -80 (an offset above +9
   shows +100 of the fall move as not reached: expected).

If your servo's shaft has a different number of teeth, what matters is the
end result in step 5. `docs/calibration.md` has the commands in general and
the other joints' numbers.

## 3. The A1 switch, and the stage

A1 is off unless it is switched on, in both places the remote is handled.
The switch's value is the **highest stage allowed**: 1, 2 or 3.

**Stepping through the stages (the mind on the Pi).** With the switch at
N, A1 from standing plays the stage-N trick, which first goes to stage 1
and holds it 4 seconds:

- A1 while he goes up to or holds stage 1 (or 2): he goes on to the next
  stage when the hold ends (the trick's own checks of the pitch and roll at
  each stage's top still run, and stop him if he is not where the model
  says);
- no A1 in that time: about 0.3 s before the hold would end, the mind stops
  the trick and he comes down to SIT from where he is (the same straight
  move every stop takes, checked by the model from every keyframe);
- at the highest stage he holds 2 seconds and comes down by himself;
- A1 at any other moment (on the way up between two stages, or at the
  highest stage), B6 or C7: down at once.

So with the switch at 3 you can do stage 1, look, and only then go on; one
build does all three. The log says each step: `remote: rear_up: up to stage
1 (of 3)`, `rear_up: stage 1 (of 3): A1 to go on`, `rear_up: on to stage 2
(of 3)`, `rear_up: held stage 1 without A1: coming down`, `rear_up: coming
down (stage 2 allowed)`; each is also a mark in the trick's flight-recorder
run (section 6), next to the key that caused it. The run list (`kilo runs
--host`) and the fetched run (`kilo log run FILE`) name the highest stage
A1 allowed in that run, not the build's: `rear_up stage 1 of 3`, or
`rear_up stage 2 of 3 (reached 1)` when he came down before stage 2's top.
(Recordings from before 2026-11-21 show the build's stage: `rear_up_3
stage 3`.)

The switch is a build option of the mind kernel: build it with the switch
and update the Pi. For stage 1:

```
make -C kernel KILO_REAR_UP=1
```

For stage 2, `KILO_REAR_UP=2`; for stage 3, `KILO_REAR_UP=3`. At boot it
logs which: `Kilo mind, build ... (rear-up key A1 on: rear_up_2, stage 2)`.
Built without it (`make -C kernel`), A1 is off again. Any other value stops
the build with an error.

With the switch off, pressing A1 logs `remote A1 (0x42, none): ignored (no
action on this key)` and nothing moves. With it on, A1 logs `remote A1
(0x42, rear up): trick rear_up started (6 s; ...)` at 1, and at 2 and 3
`remote A1 (0x42, rear up 3): trick rear_up_3 started: up to stage 1 (A1
while he holds it: on, up to 3; ...)`. (At boot the mind still logs
`rear-up key A1 on: rear_up_3, stage 3`: read it as "up to stage 3".)

Switch it on only for the trials of section 5, and leave it on afterwards
only at a stage that has gone well.

## 4. Before every trial

- **The right hind shoulder** re-seated and calibrated (section 2):
  `kilo cal show --host kilo.local`, joint 10 between +7 and +25.
- **The Pi.** Foam or folded cardboard taped over it: it is what lands if
  Kilo goes over backwards.
- **The tail** firmly on its servo, straight back in stand. It carries
  about a tenth of Kilo's weight on the tripod.
- **The floor.** Hard, flat, not slippery for the feet: the hind feet stay
  put while the front legs push the body back over them; feet that slide
  change that. The back corners of the frame (under the battery) touch the
  floor when he rocks back, and the tail tip slides forward as the hips rise
  (its wheel helps).
- **The battery** at least 7.8 V (`kilo --host kilo.local status`).
- **Your hands.** Kneel beside Kilo, one open hand a few cm to his right
  side at hip height (the model's way to tip, if any, is to his right) and
  one behind him.
  If he starts to go, catch him; do not push.
- **The way out.** C7 stops at once (Kilo sits down over 1.5-2.5 s).

## 5. The trials

Do each step until it looks the same twice. After every press of A1:
`kilo runs --host kilo.local --fetch last` (section 6); its keyframe table
shows the measured pitch against the model's at each keyframe, and the log
shows any guard that stopped the trick.

**Step 1: onto the rear (stage 1, keyframes 0-4).** Build with
`KILO_REAR_UP=1`. Press A1, and press it again (or C7) once both paws are
folded (about 2 s). Watch: does the head come up first, the body crouch,
then rock back onto its rear without the hind feet moving? Do the paws fold
one after the other? The recorded roll should stay within 2 degrees of what
it reads standing.

**Step 2: all of stage 1.** Press A1 once and let it play (6.3 s). At about
2.5 s the hips rise and he sits on the hind feet and the tail tip; he should
hold still for 2 s at about 23 degrees nose up, then sit back, put the paws
down and sit.

**Steps 3-4: stages 2 and 3** (`KILO_REAR_UP=3` once, for both), only after
stage 1 has gone well twice: press A1; at stage 1's hold, A1 again: he goes
on to stage 2 and holds it; let him come down (no A1), or A1 again for stage
3. The tail tip slides forward as he rises. Without the second press he
comes down from stage 1 after its 4 s. Kneel close behind and to his right
for these: in the model it is the higher stages that can roll to his right.

What a recorded stage-1 play should show if the model is right (TELEMETRY
nose up, within about 2 degrees; roll about +2 throughout): look and crouch
-2; back +7 (a peak near +11 as he rocks); paws +6; up +23; settle +11;
drop +6; sit +27. The guards stop the trick when the average over a hold
is outside its window or the roll passes 6 degrees (section 8).

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

- **keyframes**: each keyframe's name (lean, lift_l, ..., onto, paws, up,
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
`predicted_pitch_deg`: TELEMETRY's nose-up pitch at the end of the keyframe,
IMU offset included, in the fitted compliant model), and between keyframes it
follows the same cosine easing the board plays. A recording of an older design
(its CRC is not the library's) is shown with numbered keyframes and no
prediction, and says so. The CSV's columns are `t_s, uptime_s,
since_play_s, mode, flags, keyframe, keyframe_name, body_keyframe,
nose_up_deg, pred_nose_up_deg, err_deg, pitch_deg, roll_deg,
rate_pitch_dps, rate_roll_dps, batt_mv, dist_mm` (`mind/README.md` "The
flight recorder" explains each). For a stretch of time instead of a run
(say Kilo did something odd while standing): `kilo record fetch --host
kilo.local --last 10m`, then `kilo record decode FILE --csv out.csv`.

## 7. The mechanics (reference)

Everything in this section is from the model (simulated), not from Kilo.
The numbers come from `python -m kilo.tricks.rear_up` (it prints every
case); the model is `mind/kilo/tricks/dynamics.py`, the design
`mind/kilo/tricks/rear_up.py`, the tests `mind/tests/test_rear_up.py`.

**The model.** One rigid body in 3-D with Kilo's masses and leg and tail
geometry (`kilo.tricks.stance`), on a flat floor, played tick by tick as the
body plays the keyframes: every contact (the four feet, the knees, the tail
tip, the frame's corners) is a stiff spring with friction that sticks or
slides; each servo follows its command 50 ms late and over 70 ms (the
planner's measurement of 2026-11-15), within its pulse range, and **gives**
under the load its leg carries (4 degrees per kg cm, fitted). The IMU reads
the body's pitch less 1.9 degrees and its roll plus 2. It does not model the
tail servo giving, a soft floor, a servo running out of strength, or
backlash.

**Held against the plays of 2026-11-19/21** (the fixture
`mind/tests/fixtures/rear_up_2026_11_21.json`; `python -m
kilo.tricks.dynamics` prints it): at "gather" the recorded plays passed
+5 degrees 0.55-0.59 s into the move and settled at +22.6 to +23.1, roll
-0.9 to -2.1; the model passes +5 at 0.56 s and settles at +22.1, roll
-2.7. At "lift_r" 3 of 7 plays went over the diagonal; with the fitted
centre of mass the model goes over, with it 0.5 mm further forward it does
not: the centre of mass sits on that line. With rigid legs the same model
keeps him on his four feet at "gather", which is how the second design's
checks passed it.

**Fitted** (what the recordings say about Kilo): the centre of mass 5.5 mm
further back than `kilo.planner.model` (the fit also said 5 mm to the
right; the self-measurement of 2026-11-22 found it in the middle, and the
trick is now judged with it there, `kilo.tricks.rear_up.MODEL`); the legs give
4 degrees per kg cm (3.5 to 4.5 fit); the tail droops 16 degrees, not 18.
Not separated by the data: further back and softer look alike at "gather";
right of centre and a left/right difference in the legs look alike; the
friction (nothing slid in the recordings). These stay the trick's own
numbers: the walking planner keeps its model, so the walks do not change.

**A bug found on the way** (2026-11-21): in the first version of this
model, the legs' give moved the contact points and that motion was fed into
the contacts' dampers; on three points of support that rang at the
simulation's step, the loads added up to 1.6 times Kilo's weight and the
roll depended on the step size. Fixed (and tested): the give moves the
points, only the commanded motion is damped. The fit above is the fixed
model's.

**The cases** (`kilo.tricks.dynamics.bands`), each one thing off the fit:
the centre of mass 4 mm forward or back, 3 mm left or right, 10 mm up or
down; the legs 1 degree per kg cm stiffer or 1.5 softer; the tail drooping
4 degrees more or less, 3 mm longer or shorter; the friction 0.3 and 1.0
(the fit takes 0.5; hardwood is not measured); the servos slower (70 + 100
ms); the frame's rear corner 10 mm higher; and three corners where two go
wrong together: stiffer and further back, softer and further forward,
softer and further right. 22 cases.

**What is checked** for each stage, playing from STAND in every case, from
SIT in the fitted one, with the Pi's guards running on the model's own
pitch and roll:

- stage 1 reaches the tripod in every case: each hind foot carries at
  least 20 g, the tail at least 15, the front paws nothing, every guard
  reads at least a degree inside its window, the roll stays under 7;
- stages 2 and 3 reach their top in the fitted case; in the other cases a
  guard may stop them, and then they must come down safely;
- never over 65 degrees nose up; nothing but the feet, the knees, the tail
  tip and the frame's rear corners touches the floor; the hind feet slide
  less than 25 mm; no leg servo carries more than 2.4 kg cm for a tenth of a
  second (the stall is about 3, an estimate);
- cut short half way into and at the end of every keyframe (the fitted case
  and the three worst corners): down on its feet, sitting, the roll within
  20 degrees;
- while the hind legs push in "up", the roll moves less than the push
  watch's 3.5 degrees, by at least half a degree, in every case;
- he ends sitting on his feet every time.

**What the checks found** (stage 1, 2026-11-22, both hind legs alike): in every one of the 22 cases, and from SIT, he is on the tripod by 2.8 s (the paws are off the floor by 1.9 s; by 2.6 s in the softest cases, where a front knee brushes the floor as the paws fold) and holds at +22.2 to +25.5 degrees nose up. The roll at the hold reads -1.1 to +4.6 (the IMU's +2 included; the worst are the centre of mass 3 mm to a side). While the hind legs push, the roll moves at most 3.0 degrees (soft-and-right), under the push watch's 3.5. The most a leg servo carries is about 1.6 kg cm. Cut short at any of the 64 points checked, he sits; the largest roll on the way down is 7.4 degrees (soft-and-right, stopped at "up").

**Stages 2 and 3** (2026-11-22, both hind legs alike): in the fitted case they hold at "mid" (+34.7 to +37.7 over the cases) and at "top" (+49.0 to +51.8); in the cases with the centre of mass 3 mm to a side, or soft and to the right, a guard or the roll limit may stop them and he comes down sitting (the roll reaches 8 to 12 degrees on the way down there). Cut short anywhere (96 and 128 points): he sits; the largest roll on the way down 12.3 degrees (soft-and-right).

**The roll.** On the tripod his weight is on the hind feet and the tail
tip, and the hind legs give under it. A centre of mass to one side loads
that hind leg more; it gives more; he rolls that way, which loads it more
again. At stage 1 a millimetre of centre of mass (or of one leg's length)
moves the roll by about a degree; the 3 mm cases roll him 2.6 degrees while
the legs push. The higher he sits, the softer this gets: at stages 2 and 3
about 2.5 degrees per millimetre, which is why those stages are the ones a
guard may stop.

**The tail** (unchanged, section 2): it stays at 0; turned aside under load
the floor's push would turn it further out.

## 8. The guards: the Pi checks the trick while it plays

The body plays the keyframes by itself; the Pi watches the pitch and roll
it reports 25 times a second. Each guard is at a keyframe where Kilo holds
still: over the second half of the hold the pitch must average inside the
guard's window, and the roll must stay within 6 degrees. And over the whole
play, a roll past 8 degrees twice in a row. And the push watch on "up":
from the moment the hind legs start pushing him onto the tripod until the
hold's second half, the roll may move at most 3.5 degrees from what it was
just before the push (whatever lean he brought into it); past that twice in
a row he is stopped at once (`guard up: roll +X deg, +Y while it moved
(limit 3.5)`). A tip there grows 20 to 40 degrees a second (2026-11-22): the
8 degree limit came 0.1 s too late and he rolled on to 30 degrees before he
sat. When a check fails, the Pi sends
the same stop as C7 (Kilo sits down over the trick's stop time) and logs
`trick rear_up: guard NAME: nose up X deg outside LO..HI: stopping` (or the
roll); the recording keeps it.

Each window holds every case the model knows, with at least a degree to
spare at stage 1:

| Guard | Window (nose up) | The cases read | What it catches |
|---|---|---|---|
| back | +3.5 to +12.5 | +6.8 to +9.6 | he did not rock back (centre of mass further forward, front feet slid), or went further back than modelled |
| paws | +2.5 to +12 | +5.0 to +9.5 | the paws did not come up, or he rocked on |
| up (stage 1's top) | +19.5 to +27 | +21.7 to +24.9 | not on the tripod: the tail not down, or a leg short; a roll at the hold |
| mid (stage 2's top) | +33 to +41 | +34.2 to +37.2 | the same, higher |
| top (stage 3's top) | +46 to +55 | +48.6 to +51.3 | the same, at the top |

The C++ mind on the Pi does this (`kernel/mind/tricks/player.cpp`), from the
trick's own entry in the library; `kilo.tricks.model.GuardWatch` is the
Python reference the C++ is checked against. The remote finds each stage's
top by its guard's name (`up`, `mid`, `top`).

## When it goes wrong

- **A1 does nothing, the log says "A1 (0x42, none): ignored"**: the switch
  is off (section 3).
- **Three purple flashes of his eyes, the log says "refused: battery ...
  mV at rest"**: charge the battery (the
  reading at rest, not the moment after a walk, is what counts).
- **He came down from stage 1 when you wanted stage 2**: press A1 while he
  holds (the log says `rear_up: stage 1 (of 3): A1 to go on`); a press
  earlier on the way up (before his last move up to the stage) brings him
  down instead.
- **"roll ... past 8" in the crouch, or "guard back: roll ..."**: a hind
  leg is not where it was asked. Is the right hind shoulder re-seated
  (`kilo cal show` joint 10 between +7 and +25)? Tell me the numbers.
- **"guard back: nose up ... outside ..."**: lower than the window: he did
  not rock back (his centre of mass is further forward than the model's, or
  the front feet slid back); higher: he went further back than the model
  says (the tail or the rear corner lower than modelled). Tell me which.
- **"guard paws: ..."**: the paws did not come up or he rocked further.
- **"guard up: roll ..."** (or mid, top), **or "... while it moved"**: he
  rolled on the tripod or while pushing onto it: which side (the sign: + is
  his left side down)? A slow-motion video from behind tells whether one hind
  leg lags the other (docs/rear-up-trick-results.md, 2026-11-22).
- **He rocks right over the tail** (catch him): C7, stop the trials, tell
  me the stage.
- **A foot slides during the push**: a less slippery floor.
- **The body says FELL**: Kilo went past 75 degrees. It holds still, then
  lies down and goes limp after 2 s. Set it upright by hand and press B6.
