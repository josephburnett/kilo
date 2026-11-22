# Rear-up trick: results

Runs of docs/rear-up-trick.md. Newest first. Copy the block below for each
run and fill in what you saw; "don't know" is a fine answer. After each
play, `kilo runs --host kilo.local --fetch last` keeps the recording and
prints the per-keyframe table (section 6 of the procedure).

## YYYY-MM-DD, run N

Build (mind kernel, with `KILO_REAR_UP=1`, `2` or `3`): ...   Battery at the start: ... mV
Right hind shoulder re-seated, `kilo cal show` joint 10: ...  (needs +7 to +25)
Floor: ...   Foam or cardboard over the Pi: yes / no   Tail straight back in stand: yes / no

| Keyframe | Pitch from `kilo log trick` (model) | What you saw |
|---|---|---|
| look, crouch | (-1.9, -1.8) | head up first? hind knees folded, a short pause? |
| back (guard) | (+7) | rocked back onto his rear? hind feet stayed put? |
| paw_l, paws (guard) | (+7, +6) | left paw first, then the right? |
| up (guard) | (+23) | hips up, sitting on the hind feet and the tail tip, still? leaning to a side? |
| settle, drop, sit | (+11, +6, +27) | sat back, paws down, sat? |
| stopped by a guard? which, and the log line | | |

Anything else:

## 2026-11-22, boot 17: stage 1 twice, tipped left both times

Build 7e92703c, `KILO_REAR_UP=1` (A1 plays stage 1). Battery 7659 mV at the
first start, 7476 mV lowest. Recordings (not in git):
`~/kilo-logs/run-b17-146s-trick-2.krec`, `run-b17-157s-trick-3.krec`, the
boot's log `kilo-card-b17.log`.

What you said: "Both times it looked good but the left leg didn't keep up
with the right and he tipped left."

### What the recordings show (measured)

The two plays are the same to within a degree. TELEMETRY's roll is +
when the left side is down; nose up is +. Times from the start of the play.

| | play 1 | play 2 | the model (as built) |
|---|---|---|---|
| standing before (pitch, roll) | -2.4, +0.1 | -2.1, -0.4 | -1.9, +2.0 |
| "back", end of its hold (1.5 s) | +10.4, **+4.2** | +10.2, **+3.3** | +7.1, +1.8 |
| "paws", end of its hold (2.4 s) | +11.4, +2.2 | +11.2, +1.5 | +6.4, +1.8 |
| "up" starts (2.46 s) | nothing moves until 2.70 s | the same | the pitch rises from 2.50 s |
| the roll starts growing | 2.70 s, before the pitch (2.78) | 2.72 s, pitch 2.80 | (no roll) |
| roll at 2.90 / 3.00 s | +6.1 / +8.6 | +5.3 / +8.1 | +1.4 / +1.6 |
| stopped by the roll limit | 3.06 s at **+11.3**, growing 36 deg/s | 3.04 s at **+9.9**, 35 deg/s | |
| nose up when stopped | +19.2 (the model: +23.3) | +18.9 | |
| after the stop | rolled on to +30 at 3.6 s (on his left side), then sat: roll +3.9 at 5 s | the same, +30.6, then +3.9 | |

The roll grew smoothly, 20 to 40 degrees a second, from the first moment
the hind legs pushed: a tip over the left side, not a slip (no jump in the
pitch or the roll), and it did not slow down by itself.

### Why: the trick told the left hind leg to stay shorter

**The commanded legs were not symmetric.** In "up" the left hind leg was
sent to shoulder -56 / knee -48, the right to -52 / -52: by design, the left
leg 2.5 mm shorter than the right (`RIGHT_DROP`, docs/rear-up-trick.md
section 7). The design leaned him left on purpose, against a centre of mass
that the fit of 2026-11-21 put 5 mm right of the middle. Seen from behind,
a left leg told to stay shorter is exactly "the left leg didn't keep up
with the right". Every other frame was symmetric, and the body applies each
servo's own calibration, so left and right reach the same angles. Before
"up" the commands were the same on both sides.

**The centre of mass is not 5 mm right.** The self-measurement of
2026-11-22 (docs/self-measurement-results.md) found it -0.7 +- 0.6 mm,
i.e. in the middle, and the sensor's own roll offset 0.29 degrees, not the
2 degrees the trick's model assumed. In the trick's model with those two
measured numbers (simulated):

| the model, at the moment of the stop (3.06 s) | roll |
|---|---|
| as built (the left leg 2.5 mm shorter) | +4.3, settling at +4.6 |
| both hind legs alike | +0.3 |
| as built, and something more on the left side, worth the centre of mass 5 to 6 mm left | +8.8 to +10.3, still growing |

So the lean the trick asked for explains about 40 % of the roll by itself
(alone it would have held him at +4.6 on the tripod, a degree and a half from the "up" guard's 6). On
the tripod a millimetre of leg or of centre of mass is worth about one
degree of roll, and the legs' give makes it grow: a lean of a few degrees
turns into a tip.

**And the robot has a left side that sits lower under load.** At "back",
where both hind legs were told the same, he already rolled +3.3 to +4.2
(the model: level), and it came back to +1.5..+2.2 when the paws came up.
The fit of 2026-11-21 saw the same "+2 in every trick pose" and took it for
the sensor; the sensor is now measured at 0.3, so it is the body. What it
is, the data cannot say. It looks the same as any of: the left hind servo
giving more under load (a weaker servo, or its horn slipping on the spline),
the left hind leg a few degrees off in its calibration only where it is
bent far, the right hind shoulder falling a few degrees short of -76 (a
partial version of the 2026-11-17 clamp: the model says a reach of about
-72 gives +4 at "back"; the full clamp of 2026-11-17 would have rolled him
29, so the re-seat is confirmed), or the centre of mass a few mm left only
when he sits back. Standing, none of it shows: the self-measurement found
no twist (0.00 +- 0.08 mm) and level poses within 0.3 degrees.

- (a) a weak or slow left hind servo: possible; nothing per servo has ever
  been measured (the lag of about 120 ms is the whole robot's). The
  self-measurement's tips do lean 1.2 to 2.1 degrees at once when the left
  front or the right back foot is lifted (the right-front-to-left-back
  diagonal holds him less firmly), which a soft left hind leg would do, but
  so would the centre of mass near that diagonal.
- (b) calibration: not the gross clamp (ruled out above); a few degrees on
  one hind joint remains possible.
- (c) the design: yes, certainly a cause: it commanded the lean, and the
  tripod is that sensitive.
- (d) the floor or a foot slipping: no sign of it (smooth growth, no jumps).
- The servos' lag: the pitch began to rise 0.25 s after the model's,
  on both sides alike; this does not separate left from right.

**One look settles (a) against (b)**: a slow-motion phone video from
directly behind, at the height of his hips, of one play of the new stage 1
(it is symmetric now, below). If the left hip rises later or less than the
right while both are told the same, it is the left hind leg (a); then put
him in the calibration pose (C5) and look at both hind legs from behind: if
they are not mirror images, it is calibration (b), else the servo itself.

### What changed

- Both hind legs alike on the tripod (`RIGHT_DROP` deleted); the trick is
  now judged with the centre of mass in the middle, the body model's and the
  self-measurement's (`kilo.tricks.rear_up.MODEL`). The "tricks" view in
  `protocol/spec.toml` keeps its 5 mm right for now: the intent layer's
  stand-to-rest moves and the righting experiment are checked on it, and on
  0 the REST moves would stop shipping (their margin in that view drops to
  2.6 mm); whoever owns those re-checks them, then the view's number goes.
- **The push watch** (`Guard.push_roll`, 3.5 degrees on "up"): while the
  hind legs push him onto the tripod, the Pi watches how far the roll moves
  from what he brought into the move; past 3.5 degrees on two readings in a
  row it stops the trick at once, instead of waiting for 8 degrees. On
  today's two plays it would have stopped him at 2.94 and 2.96 s, at +6.9,
  growing at about 20 degrees a second, instead of +11 at 36. The log says
  `trick rear_up: guard up: roll +X deg, +Y while it moved (limit 3.5):
  stopping`.
- The trick's check no longer plays the "right hind shoulder not re-seated"
  case: today's plays rule it out.
- The body model has a place for each servo's own lag, time constant and
  give (docs/movement-engine.md, "Each servo"); all are the robot's until
  measured.

All of this is simulated, not tried on Kilo.

### Also in the log: "B1 is refused" at boot was a false alarm

At 5.3 s the log said `righting: the body's fall reflex did not take
FALL_RC 0 (an older body?): B1 is refused`, and 53 ms later `will not play
rc (FALL_RC 0): B1 rights him`. The Pi had asked the body for FALL_RC 0
right after its HELLO, then its main loop waited 1.5 s for the SD card and
1.7 s for the USB scan; the request ran out of time unanswered, and the Pi
read "no answer" as "refused". It then asked again at once, and the body
said yes. Fixed: only the body's own refusal is reported as one (and then
the Pi stops asking until the body's next HELLO); no answer just asks again
quietly.

## 2026-11-21, tries 1-3: the second design, stopped by its own checks every time

Build: mind and body f3020d5e, `KILO_REAR_UP=3` (A2 played `rear_up_3`, which
goes to stage 1 first). Battery 7988 mV at the start of try 1. Wood floor.
Recordings: `~/kilo-logs/2026-11-21-f3020d5e/run-20261121-071301-trick-1.krec`,
`run-20261121-071325-trick-2.krec`, `run-20261121-071335-trick-3.krec`, the
session `rec-20261121-071923.krec`, and the earlier tries of 2026-11-19/20
in the same folder (`run-b8-*`, `run-b9-*`). Your video of try 1 or 3, 10
frames a second. The numbers the tests use are in
`mind/tests/fixtures/rear_up_2026_11_21.json`.

| Try | Started from | How it ended | Where |
|---|---|---|---|
| 1 | standing | stopped by the check at "gather": nose up 22.9, the window was 1.8 to 6.8 | 7.8 s in |
| 2 | standing | stopped: roll -12.3 past the 8 degree limit, while the right hind foot was in the air ("lift_r") | 5.4 s in |
| 3 | sitting (where try 2 left him) | stopped by the check at "gather": nose up 22.5 | 7.8 s in |

Nothing fell; each time he sat down. (`kilo log run` names the last key
pressed before the stop as what stopped it: that was an A2 repeat the
remote ignored; the stop was the check's.)

**What the video shows** (tries 1 or 3): the hind feet stepped forward one at
a time as designed (frames 1-70); then, at "gather", he rocked back onto his
tail with the tail tip planted on the floor (frames 70-80), one or two
keyframes before the design meant him to be on it; then the stop and the
sit (86-100).

**The timing check**: the recording's clock and the video agree to 0.1 s
over 7 s, so the model's timing is right. He was slow because the design
was slow (12.5 s to the first hold), not because the servos lagged.

**Why** (`kilo.tricks.dynamics`, fitted to these plays and to those of
2026-11-19/20): Kilo's leg servos give under load, about 4 degrees per kg
cm. The second design's checks used a model with rigid legs. At "gather" the
weight moves onto the hind legs; they give, which tips him nose up, which
moves the weight further back onto them, which makes them give more: he
rocks back onto the tail by himself, 0.55-0.59 s into the 0.6 s move, every
time (4 of 4 plays that got there). The rigid model had him 15 mm away from
that. The fit also puts his centre of mass about 5.5 mm further back and 5
mm further to the right than the planner's model: that is why try 2 (and 2
of the 4 tries before) went over the diagonal at "lift_r", where the other 4
did not; the centre of mass sits on that line, a coin flip.

**What changed**: the third design (docs/rear-up-trick.md): no stepping, a
deliberate rock back onto the rear, 2.5 s to the tripod instead of 12.5,
checked in the compliant model.

## 2026-11-17, runs 1 and 2: the first design fell twice the same way

Build: mind fdc41a79, body de6d2d7f, `KILO_REAR_UP=1` (stage 1, the first
design). Battery 7.92 V and 7.90 V at the start, lowest 7.83 V. Wood floor.
Recordings: `~/kilo-logs/run-20261117-214013-trick-1.krec`,
`run-20261117-214037-trick-2.krec`, the session
`rec-20261117-214454.krec` (STAND and SIT measured there too), and your
video of one run (IMG_1624, side view, nose to the left). A copy of the
numbers the tests use is `mind/tests/fixtures/rear_up_2026_11_17.json`.

What you said: "It tried the trick. I saw it. Just couldn't get up. Tipped
over." Earlier, unrecorded tries: "He leaned back. Then tipped to the left.
Sort of tipped slowly. Was tipped up with paws in the air."

### What the recordings show (measured)

Nose-up pitch from TELEMETRY at the end of each keyframe, both runs, against
what the first design predicted and what the corrected model says now.
STAND reads -1.9 on the IMU (as on the walks); the model's numbers below
include that offset.

| | Run 1 | Run 2 | First design predicted | Corrected model (simulated) |
|---|---|---|---|---|
| STAND (session) | -1.9 | | 0 | -1.9 |
| SIT (session, roll +2.1) | 26.9 | | 28.4 | 23.9 or 26.4 (a near tie: see below) |
| tuck (0.76 s) | 5.5 | 5.7 | 8.0 | 5.2 |
| sink (1.40 s) | 16.4 | 16.6 | 24.0 | 15.2 |
| lowest during step (2.40 s) | 7.7 | 7.9 | 24.0 | 3.3 |
| roll during sink and step | 0.4 rising to 3.0 | the same | 0 | 0 |

Then, at the end of the step (2.56-2.80 s), the pitch jumped from 8 to 27
degrees and the roll from 3 to 13 degrees within 0.25 s: the left rear
dropped. Kilo kept rolling to his left, through 180 degrees in 0.6 s, and
the body reported FELL 3.4 s after the start. Run 2 was the same within half
a degree.

### What the video shows (read off the frames)

Video time is about 1.0 s ahead of the recording's time since the play
started (matched on the pitch jump and the roll). Angles read relative to
the line through the near shoulder axles, which removes most of the
camera's tilt (that line reads 3-4 degrees more nose up than the IMU in
every frame, STAND included). Lengths at about 3.2 pixels per mm, within
about 15 %.

- **0.9 s, STAND**: the legs where the joint angles put them (front upper
  leg 30 degrees back, hind upper leg forward).
- **2.45 s, end of the sink**: the near (left) legs exactly as commanded:
  hind upper leg 80 degrees forward of straight down (asked: 80), hind lower
  leg 32 degrees back (model: 33), front upper leg 52 back (asked: 50),
  front lower leg 10 forward (model: 9). So the left side's angles and the
  model's leg geometry are right. But the **left hind foot is 10-12 mm off
  the floor**, and the **right hind foot (far side) is on the floor about
  20 mm behind the left one**. The **tail tip is about 11 mm above the
  floor** (plus or minus 6).
- **2.9 s and 3.4 s, the step**: the left hind foot swings forward in the
  air; the right hind foot stays planted, by 3.4 s about 45 mm behind the
  left one. The tail tip rises to about 18 and then 26 mm: the rear is going
  up (the falling pitch in the recording).
- **3.7-4.05 s**: the rear drops, the body pitches up to about 30 degrees,
  the tail tip lands, and Kilo rolls toward the camera (his left), onto his
  back by about 4.5 s.

### Root cause

**The right hind shoulder servo cannot go past -57 degrees.** The body turns
each joint angle into a servo pulse and clamps the pulse to 500..2500
microseconds (body/src/core/joints.cpp, as the legacy firmware did). With
Kilo's calibration (the EEPROM dump of 2026-10-10: offset -16 degrees on
joint 10) the right hind shoulder's pulse reaches 2500 at -57.3 degrees. The
left one reaches the -80 limit. The first design asked -80 in the sink and
-77 in the step, so the right hind leg stayed 20-23 degrees short of the
left: its foot 20 mm further back and, at that pitch, about 9 mm lower.

What followed, step by step:

1. Sink: Kilo stood on three feet, both front feet and the right hind, with
   the left hind foot in the air and the rear held higher than designed.
   The pitch stayed at 16 degrees instead of 24, so the tail never reached
   the floor.
2. Step: the design lifted both hind feet, assuming the tail carried the
   rear. The right hind foot was carrying it. As that leg unfolded toward
   the step pose, its foot pushed the rear up (the pitch fell to 8 degrees)
   while the centre of mass came to a few mm from the line between the left
   front foot and the right hind foot.
3. End of the step: Kilo went over that line, the left rear corner, where
   nothing stood: nose up and rolling left at the same time, landing on
   the tail with momentum, and on over.

Two weaknesses of the first design made it fatal: the side-view model took
the left and right legs as one and the joint angles as reached, so it could
not see this; and the design lifted both hind feet relying on a tail it
had never confirmed was on the floor.

The same clamp explains your P1 finding B4 (the left front knee does not
retract as far as the right at REST): REST asks -45 degrees, the left front
knee's pulse ends at -32.

**Evidence**: the clamp is arithmetic on the firmware and the EEPROM dump
(the test `test_the_pulse_clamp_stops_the_right_hind_shoulder_at_57_deg`).
The 3-D model with the clamp reproduces the tuck within 0.5 degrees, the
sink within 1.4, the three-foot stance with the left hind foot up and the
tail off the floor (the video), the falling pitch during the step, and puts
the end of the step a few mm from tipping over exactly the edge Kilo went
over. Without the clamp, both the old model and the new one predict 22-24
degrees at the sink, which the trials contradict
(`test_the_side_view_model_could_not_explain_the_falls`).

**What would prove it wrong**: in SIT or in a pose with the hind shoulders
at -80, the right hind upper leg pointing as far forward as the left one
(it should be 20-23 degrees less); or `kilo cal show` giving joint 10 an
offset other than -16 (then the dump is stale and the arithmetic changes).
One look settles it: stand Kilo in SIT and compare the two hind upper legs
from above.

### What the model still does not explain

- During the step the model's pitch falls by 12 degrees, the measured by 9,
  and the model's tail tip rises about twice as high as the video shows.
  The real right hind leg gives a little under the whole rear's weight, or
  its foot slides; the model is rigid and the feet do not slide.
- SIT on three feet is a near tie in the rigid model (23.9 degrees on one
  diagonal, 26.4 rolled 5 degrees on the other); Kilo sits at 26.9 rolled 2,
  sharing the load over four feet.
- Which way the near-tie stances fall depends on a centre of mass about 2 mm
  right of centre (fitted to the tuck and SIT rolls; never measured).

### What changed

- `kilo.tricks.stance`: the 3-D model, with the pulse clamp.
- The second design of the trick (docs/rear-up-trick.md): it needs the right
  hind upper leg re-seated on its servo first, keeps three feet (or two
  feet and the tail) down at every moment, and the mind checks the measured
  pitch and roll at five holds and stops the trick if they are off.
