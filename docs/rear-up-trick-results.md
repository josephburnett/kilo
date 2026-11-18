# Rear-up trick: results

Runs of docs/rear-up-trick.md. Newest first. Copy the block below for each
run and fill in what you saw; "don't know" is a fine answer. After each
play, `kilo runs --host kilo.local --fetch last` keeps the recording and
prints the per-keyframe table (section 6 of the procedure).

## YYYY-MM-DD, run N

Build (mind kernel, with `KILO_REAR_UP=1`, `2` or `3`): ...   Battery at the start: ... mV
Right hind shoulder re-seated, `kilo cal show` joint 10: ...  (needs +7 to +20)
Floor: ...   Foam or cardboard over the Pi: yes / no   Tail straight back in stand: yes / no

| Keyframe | Pitch from `kilo log trick` (model) | What you saw |
|---|---|---|
| lean, lift_l, place_l | (-1.9) | left hind foot stepped forward? Kilo leaned? |
| lean_r, lift_r, place_r | (-1.9) | right hind foot stepped forward? |
| gather (guard) | (4.3) | all four feet bunched under the body, level? |
| sit_back, low (guard) | (11.4, 22.5) | rear down, tail tip touching at the end? |
| onto (guard) | (21.9) | body back over the hind feet, tail carrying? |
| paws (guard), up | (21.9, 26.0) | paws up, sitting on hind feet and tail? |
| stopped by a guard? which, and the log line | | |

Anything else:

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
