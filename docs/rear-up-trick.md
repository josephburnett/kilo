# Rear-up trick: sitting up on the hind feet and the tail (A2)

Kilo's first authored trick: Kilo sits back on its two hind feet and the tip
of its tail, front paws off the floor, holds 2 seconds, comes back down and
ends **sitting**. One key starts it: **A2** (column A, row 2 from the
bottom). Pressing A2 again while it plays brings Kilo down at once.

**This is the second design.** The first one fell over twice on
2026-11-17; `docs/rear-up-trick-results.md` has the trials, the video and
the cause: the right hind shoulder servo stops at -57 degrees, where the
left one goes to -80, so the right hind leg never got where the trick
asked. **The second design needs that fixed first** (section 2, "The right
hind shoulder"). Played before the fix, the trick stops itself at its first
check and Kilo sits down (the model says so; section 8).

It comes in **three stages**, three heights:

| Stage | Trick | Held at (TELEMETRY nose up) | Takes |
|---|---|---|---|
| 1 | `rear_up` | 26 degrees: a low sit-up | 18 s |
| 2 | `rear_up_2` | 38 degrees | 22 s |
| 3 | `rear_up_3` | 52 degrees: the pose of your photo | 26 s |

Each stage does exactly what the stage below does on the way up and then
goes higher.

**A2 is switched off** until you switch it on (section 3). Write what you
see in `docs/rear-up-trick-results.md`. Nothing here is timed: every step
waits for you.

Words used below:

- **Pitch**: how far Kilo's body is tilted nose up. Level is 0 degrees.
  The numbers here are what `kilo log trick` and TELEMETRY show (nose up),
  which reads 1.9 degrees nose down when Kilo stands level.
- **Roll**: how far Kilo leans to one side. Positive: the left side down.
- **Centre of mass**: the point where Kilo's weight balances.
- **Support**: the area on the floor between the things touching it, seen
  from above. Kilo stays up while its centre of mass is over that area; past
  its edge it tips over that edge.
- **Margin**: how far the centre of mass is inside the support's edge, mm.
- **Tripod**: the two hind feet and the tail tip on the floor.
- **Servo**: the motor in each joint. Its **pulse** is the signal that sets
  its angle; the body sends between 500 and 2500 microseconds, and the
  **calibration** says which pulse is which angle for each joint.
- **Keyframe**: one pose in the trick; the body moves smoothly between them.
- **Guard**: a check the Pi makes while the trick plays: at a keyframe
  where Kilo holds still for 0.6 s, the measured pitch must be near what the
  model predicts and the roll small; if not, the Pi stops the trick and Kilo
  sits down (section 8).
- **The model**: the program that works out, for any pose, what Kilo stands
  on, at what pitch and roll, and how far it is from tipping
  (`kilo.tricks.stance`, section 7). Numbers marked "model" come from it,
  not from Kilo.

## 1. How the second design gets up

**Why a tripod.** Kilo cannot balance on its round hind feet alone, and its
servos answer too slowly to balance it actively. So the trick rests on
three points at every hold, and moves slowly enough that every moment on
the way is stable by itself.

**How the tail is used.** The tail swings left and right around a tilted
axis (section 2) and is lowest when it points straight back (servo angle
0). It cannot be pushed down further, so the trick keeps it at 0 as a stiff
prop and tilts the body nose up until its tip meets the floor.

**What the first design got wrong**, and what this one does about it:

1. It asked the hind shoulders for -80 degrees; the right one stops at -57.
   This design needs the right one re-seated (section 2) and is checked
   against what each servo really reaches.
2. It lowered the rear onto the tail and then swung both hind feet forward
   under the belly at once, trusting the tail to hold the rear. On Kilo the
   tail never reached the floor. And the model now shows that even with
   full reach a hind foot cannot swing under the hips while the rear sits on
   the tail: the hips are 45 mm up and the swing needs 47 to 50 (69 at -54
   degrees). So this design moves the hind feet forward while Kilo is still
   standing high, **one foot at a time with the other three down**, and
   never lifts a hind foot once the rear is low.
3. The side-view model took the two legs of a pair as one and could not see
   a lean to one side. The model is now 3-D, and every step is checked for
   sideways tipping too, including a tail a few degrees off centre and one
   side's servos 3 degrees off.
4. Nothing checked that the tail was down. Now the Pi checks the pitch and
   roll at five holds and brings Kilo down if they are off.

**The motion** (stage 1; times when each keyframe ends; "model" pitch):

| # | Keyframe | What you see | Pitch (model) | ends at |
|---|---|---|---|---|
| 0 | lean | Kilo lowers a little and leans forward over its feet | -1.9 | 0.7 s |
| 1 | lift_l | the left hind foot lifts 16 mm and moves forward | -1.9 | 1.8 s |
| 2 | place_l | it comes down under the belly, 68 mm ahead of where it was | -1.9 | 3.0 s |
| 3 | lean_r | Kilo leans further forward | -1.9 | 3.6 s |
| 4 | lift_r | the right hind foot lifts 28 mm and moves forward | -1.9 | 5.3 s |
| 5 | place_r | it comes down beside the left one | -1.9 | 6.6 s |
| 6 | gather | the feet stay put, the body moves back over them, 4 deg nose up; **holds 0.6 s: guard** | 4.3 | 7.8 s |
| 7 | sit_back | the rear goes down, nose up | 11.4 | 8.4 s |
| 8 | low | the rear lower still: the tail tip just touches the floor; **guard** | 22.5 | 9.8 s |
| 9 | onto | the body moves back over the hind feet onto the tail; the front feet carry nothing; **guard** | 21.9 | 11.1 s |
| 10 | paws | the front paws come up: on the tripod; **guard** | 21.9 | 12.5 s |
| 11 | up | the hind legs lift the hips: 26 degrees, held 2 s; **guard** | 26.0 | 15.1 s |
| 12-13 | down | paws down again, back to "onto" | 21.9 | 16.6 s |
| 14 | sit | straight to SIT from there | 26.5 | 18.4 s |

The four feet never move on the floor after step 5: from "gather" to
"onto" the body moves over them. The front feet stay where they started
the whole time. Stages 2 and 3 go on from "up" to "rise", "mid" (38
degrees, stage 2 holds) and "climb", "top" (52 degrees, stage 3 holds), at
half the speed, and come back the same way.

**The stages, held** (model, nominal):

| | Stage 1 | Stage 2 | Stage 3 |
|---|---|---|---|
| Pitch (TELEMETRY nose up) | 26.0 | 38.0 | 52.0 |
| Margin: centre of mass behind the hind feet, mm | 12.3 | 14.4 | 11.3 |
| ... worst over the geometries checked | 7.6 | 8.0 | 3.2 |
| Hind shoulder / knee torque, kg cm (stall about 3, an estimate) | 1.16 / 0.48, worst on the way (onto, up) | | |
| Front paws above the floor | 25 mm | 68 mm | 113 mm |

**How it comes down when cut short.** A2 again, B6, A7, C7, or the body
losing the Pi make the body move every joint in a straight line to SIT over
1.5 s (stage 1), 2 s (stage 2) or 2.5 s (stage 3). The model checks that
from half way to and at every keyframe it ends sitting on its feet, never
touching the floor with the back, the Pi or the nose. Cut short while a hind
foot is in the air, Kilo rocks onto that side by up to 16 degrees in the
model (it ends on its feet; the rigid model rocks further than Kilo does).
Cut short at the top of stages 2 and 3, the paws come down from up to 9 cm,
as they would if Kilo tipped forward there.

**What protects Kilo.**

- The guards (section 8). On Kilo as calibrated today the model has him
  stopped at "gather", leaning 5 degrees on three feet, before anything
  else happens; he then sits down.
- The pitch never goes past 55 degrees; the fall detector (75 degrees)
  stays armed.
- A2 refuses to start below 7400 mV on the battery.

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

The second design's top (stage 3) is the photo's pose with the front paws
tucked (55, 20) and the hind feet 52 mm ahead of the hips; the model holds
it at 52 degrees on TELEMETRY (54 by the model's own pitch).

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

### The right hind shoulder (do this before switching A2 on)

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

## 3. The A2 switch, and the stage

A2 is off unless it is switched on, in both places the remote is handled.
The switch's value is the stage: 1, 2 or 3.

- **The mind kernel on the Pi** (what normally runs): build it with the
  switch and update the Pi. For stage 1:

  ```
  make -C kernel KILO_REAR_UP=1
  ```

  For stage 2, `KILO_REAR_UP=2`; for stage 3, `KILO_REAR_UP=3`. At boot it
  logs which: `Kilo mind, build ... (rear-up key A2 on: rear_up_2, stage
  2)`. Built without it (`make -C kernel`), A2 is off again. Any other value
  stops the build with an error.
- **The laptop daemon** (`kilo run --remote`), if you use it instead: start
  it with the switch in its environment:

  ```
  KILO_REAR_UP=1 kilo run --remote
  ```

  (or `=2`, `=3`).

With the switch off, pressing A2 logs `remote A2 (0x10, none): ignored (no
action on this key)` and nothing moves. With it on, A2 logs `remote A2
(0x10, rear up): trick rear_up started (18 s; ...)` at stage 1, and `rear up
2` / `rear_up_2`, `rear up 3` / `rear_up_3` at the others.

Switch it on only for the trials of section 5, and leave it on afterwards
only at a stage that has gone well.

## 4. Before every trial

- **The right hind shoulder** re-seated and calibrated (section 2):
  `kilo cal show --host kilo.local`, joint 10 between +7 and +25.
- **The Pi.** Foam or folded cardboard taped over it: it is what lands if
  Kilo goes over backwards.
- **The tail** firmly on its servo, straight back in stand. It carries
  about a tenth of Kilo's weight on the tripod.
- **The floor.** Hard, flat, not slippery for the feet: the trick plants
  the four feet in step 5 and then moves the body over them; feet that slide
  change everything. The tail tip slides forward on the tripod at stages 2
  and 3 (its wheel helps).
- **The battery** at least 7.8 V (`kilo --host kilo.local status`).
- **Your hands.** Kneel beside Kilo, one open hand a few cm to his left and
  one to his right side at hip height: the first design fell sideways, and
  the steps of this one stand on three feet. Behind him for stages 2 and 3.
  If he starts to go, catch him; do not push.
- **The way out.** C7 stops at once (Kilo sits down over 1.5-2.5 s).

## 5. The trials

Do each step until it looks the same twice. After every press of A2:
`kilo runs --host kilo.local --fetch last` (section 6); its keyframe table
shows the measured pitch against the model's at each keyframe, and the log
shows any guard that stopped the trick.

**Step 1: the two steps (stage 1, keyframes 0-5).** Press A2, and press it
again when both hind feet have stepped forward (about 6.6 s).
Watch: does each hind foot lift clear and land under the belly? Does Kilo
lean to a side while a foot is up? The recorded roll should stay within 2
degrees.

**Step 2: down onto the tail (keyframes 6-9).** Press A2 and let it run to
"onto" (11 s), then A2. Watch: the feet stay where they landed; at "low"
the tail tip just touches; at "onto" the front feet carry almost nothing
(a sheet of paper under a front foot pulls out). If a guard stops it, note
which (the log line says "guard NAME: ...").

**Step 3: all the way.** Press A2 once and let it play (18 s). At the top
Kilo should sit still at about 26 degrees for 2 s, then come back down and
sit.

**Steps 4-6: stages 2 and 3** (`KILO_REAR_UP=2`, then `3`): as step 3. The
tail tip slides forward as Kilo rises; the top pitch should be about 38 and
52 degrees.

What a recorded trial should show if the model is right (stage 1,
TELEMETRY nose up, plus or minus about 2 degrees; roll within 2 degrees):
lean to place_r -1.9; gather 4.3; sit_back 11.4; low 22.5; onto 21.9; paws
21.9; up 26.0; back to onto 21.9; sit 26.5. The guards stop the trick when
the average over a hold is more than 2.5 degrees off (3 degrees of roll at
gather, 6 at the others).

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
`predicted_pitch_deg`, since the second design TELEMETRY's own nose-up
pitch, IMU offset included), and between keyframes it follows the same cosine
easing the board plays. The CSV's columns are `t_s, uptime_s,
since_play_s, mode, flags, keyframe, keyframe_name, body_keyframe,
nose_up_deg, pred_nose_up_deg, err_deg, pitch_deg, roll_deg,
rate_pitch_dps, rate_roll_dps, batt_mv, dist_mm` (`mind/README.md` "The
flight recorder" explains each). For a stretch of time instead of a run
(say Kilo did something odd while standing): `kilo record fetch --host
kilo.local --last 10m`, then `kilo record decode FILE --csv out.csv`.

## 7. The mechanics (reference)

Everything in this section is from the model (simulated), not from Kilo.
The numbers come from `python -m kilo.tricks.rear_up`; the 3-D model is
`mind/kilo/tricks/stance.py`, the side view (the tail, the photo, the
torques) `mind/kilo/tricks/sagittal.py`, the design
`mind/kilo/tricks/rear_up.py`, the tests `mind/tests/test_rear_up.py`.

**The 3-D model.** Each leg on its own, at its own side (the feet 65 mm
apart), with the angle the servo really reaches: the body's joint limits,
then the pulse range with Kilo's calibration (`stance.reach`; the design
uses the calibration after the re-seat, `rear_up.DESIGN`; Kilo as recorded
is `rear_up.AS_RECORDED`). The masses are the planner's (358 g); the tail
is the cone of section 2, its tip's height and sideways place from the
servo angle; the frame is a box of corners that must never touch the floor
(the Pi, the back, the nose, the shoulder servos' sides). For each pose it
finds how the rigid body rests: it sets it down on its lowest point and
tips it about the edge of the support the centre of mass is outside of,
until the centre of mass is over the support. It reports the pitch and roll
(as the IMU reads them), what touches, and the margin to the nearest edge
in any direction. It is rigid, static, with no sliding: on four feet with
one leg a little long it stands on three, where Kilo's legs give a little.

**Held against the trials of 2026-11-17** (results file): STAND within 0.2
degrees, the tuck within 0.5, the sink within 1.4, the left hind foot in the
air and the tail off the floor as in the video, the falling pitch during the
step, and the end of the step a few mm from tipping over the edge Kilo
fell over. Not explained: the step's pitch falls 12 degrees in the model
and 9 on Kilo, and SIT is a near tie between two diagonals.

**What is checked** for each stage, playing from STAND, and cut short half
way to and at every keyframe, over: the nominal geometry; the centre of
mass 5 mm back, 10 mm forward, 10 mm up or down, 3 mm to either side; the
tail 3 mm longer or shorter, drooping 5 degrees more or less, 5 degrees off
centre either way; one side's four leg servos 3 degrees off either way; and
the corners of the centre of mass the photo allows (20 geometries, 580 to
900 runs per stage):

- never both hind feet off the floor unless the tail is on it, and in the
  steps the lifted foot clear of the floor;
- the margin in every direction at least 3.5 mm at every moment (3 at stage
  3), except while the load moves between the feet and the tail ("low",
  "onto", "paws" and down), where no tip may drop anything more than 8 mm;
  with one side's servos off the rigid model stands on three feet across a
  diagonal, so there only the roll and the drops count;
- the roll within 7 degrees while playing;
- on the tripod at the hold with at least 5 mm of margin (3 at stage 3);
- or a guard stops it (with the centre of mass 10 mm forward the paws stay
  down and "paws" stops it; with one side's servos 3 degrees short of their
  angle "gather" stops it), and then it comes down on its feet;
- cut short anywhere: down on its feet, the Pi, back and nose never touch,
  the roll within 20 degrees, any drop onto something other than a foot
  within 15 mm;
- Kilo as recorded is stopped by a guard (at "gather").

**The limit the design cannot meet.** With the centre of mass 6 or more mm
further back than the model, the body tips back onto the tail before the
tail is down (up to 20 mm at 10 mm back). A search over every way of
sitting back with the four feet planted (`--search` shows the swing part)
finds none that keeps the centre of mass more than about 10 mm ahead of the
hind feet until the tail touches; so the design takes 5 mm. The photo does
not bound "further back". If the trials show Kilo rocking back at "low" or
"onto", that is this.

**Servo torque** (nominal, side view, against an ESTIMATED 3 kg cm stall):
hind shoulder 1.16 kg cm at "onto", hind knee 0.48 at "up", front shoulder
0.47, front knee 0.31, the tail servo nothing (at 0 the floor's push goes
through its bearing).

**The tail** (unchanged, section 2): it stays at 0; turned aside under load
the floor's push would turn it further out.

## 8. The guards: the Pi checks the trick while it plays

The body plays the keyframes by itself; the Pi watches the pitch and roll
it reports 25 times a second. The trick carries five checks (six at stage
2, seven at stage 3), each at a keyframe where Kilo holds still for 0.6 s
(the top's 2 s hold too): over the second half of the hold the pitch must
average within 2.5 degrees of the model's prediction, and the roll must
stay within 6 degrees (3 at "gather"). And over the whole play, a roll past
8 degrees twice in a row. When a check fails, the Pi sends the same stop as
C7 (Kilo sits down over the trick's stop time) and logs `trick rear_up:
guard NAME: nose up X deg outside LO..HI: stopping` (or the roll); the
recording keeps it.

| Guard | What it catches |
|---|---|
| gather (4.3 +- 2.5, roll <= 3) | a hind leg not where it was asked (today's right hind shoulder: the model leans 5 degrees here), a foot that landed short |
| low (22.5) | the rear not down: the tail not touching, or a leg short |
| onto (21.9) | the rear not on the tail, the body not back over the feet |
| paws (21.9) | the paws did not come up (the centre of mass too far forward) or Kilo rocked back |
| up, mid, top | the hold where the model says |

Both minds do this: the C++ mind on the Pi (`kernel/mind/tricks/player.cpp`)
and the laptop daemon (`kilo.body.client.play`, `kilo.tricks.player.GuardWatch`),
from the trick's own entry in the library.

## When it goes wrong

- **A2 does nothing, the log says "A2 (0x10, none): ignored"**: the switch
  is off (section 3).
- **A2 says "refused: battery ... mV"**: charge the battery.
- **The log says "guard gather: roll ..."**: a hind leg is not where it was
  asked. Is the right hind shoulder re-seated (`kilo cal show` joint 10
  between +7 and +25)? Did a foot land short or slide? Tell me the numbers.
- **"guard low: nose up ... outside ..."**: lower than the window: the tail
  is not down; higher: the rear went further down than the model says.
- **"guard paws: ..."**: the paws stayed down or came back down: the centre
  of mass is further forward than the model.
- **Kilo rocks back at "low" or "onto"**: the centre of mass is further
  back than the model (section 7, the limit). Catch him, C7, stop the trials.
- **A foot slides during the steps**: a less slippery floor.
- **The body says FELL**: Kilo went past 75 degrees. It holds still, then
  lies down and goes limp after 2 s. Set it upright by hand and press B6.
