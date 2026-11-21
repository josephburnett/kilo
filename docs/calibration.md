# Joint calibration

Each joint is a servo. The body turns a joint angle into a servo pulse
between 500 and 2500 microseconds; the **calibration offset** of a joint
says which pulse is that joint's 0 degrees. A leg sits on its servo's
splined shaft, so the offset makes up for the leg sitting a little off the
servo's middle. Kilo keeps the offsets in the body's EEPROM.

## The commands

All of them work over the network, through the mind on the Pi
(`--host kilo.local`; left out, the CLI looks for `kilo.local` itself).

    kilo cal show --host kilo.local              the offsets, and what each joint reaches (below)
    kilo cal set JOINT DEG --host kilo.local     one joint's offset, absolute (not saved)
    kilo cal nudge JOINT DEG --host kilo.local   one joint's offset changed by DEG, + or - (not saved)
    kilo cal save --host kilo.local              write the offsets to the body's EEPROM
    kilo cal revert --host kilo.local            back to the saved offsets
    kilo posture calib --host kilo.local         hold the calibration pose (every joint at 0) until Ctrl-C

- Offsets are degrees, from -25 to +25: the body refuses anything else.
  `set` and `nudge` check it first and say what still fits, e.g. `+25.50 deg
  is outside the body's offset range -25 .. +25 deg (now +13.50; a nudge of
  at most +11.50 fits)`. A leg that needs more sits a spline tooth off: take
  it off and put it back one tooth over.
- A **+ offset turns the joint toward its + angles**. For a shoulder that is
  the upper leg turning back, toward the tail; `nudge 10 -2` turns the right
  hind upper leg 2 degrees forward.
- A change takes effect at once: with the calibration pose held, the joint
  moves as you nudge it. It is lost at the next power-off unless you `save`.
- `set`, `nudge`, `save` and `revert` over `--host` need the calibration
  pose held (by `kilo posture calib --host` in another window) or control
  of Kilo, and nothing walking or playing a trick. `show` always works.
- `kilo posture calib --host` holds the pose with the servos powered for as
  long as it runs. While a laptop holds Kilo, the remote's keys are
  ignored and light sleep cannot start, so nothing else moves him. Ctrl-C
  ends it, and Kilo goes to REST (lying down).

## What `kilo cal show` says

    joint              offset          reaches       postures         gaits        tricks
    10 rb_shoulder     -16.00   -57.3 .. +100.0     -80 .. +100    -79 ..   -8    -80 ..  +95  SHORT
    12 lf_knee         -21.00   -32.3 .. +120.0     -45 ..  +81    -15 ..  +53    -60 ..  +85  SHORT
    ...
    short of what is asked (the servo stops at the end of its pulse range, 500 or 2500 us):
      joint 10 (right hind upper leg): -80 (SIT) stops at -57.3
      joint 10: an offset of +6.7 .. +9.1 reaches all the postures and gaits ask
      joint 12 (left front lower leg): -45 (REST) stops at -32.3

- **reaches**: the angles the joint really goes to with its offset. Two
  things stop it: the joint's limits (the body never asks past them), and
  the pulse range: past 500 or 2500 microseconds the servo goes no further.
  The offset slides the pulse range: +1 degree of offset takes 1 degree off
  the + end and adds 1 degree to the - end.
- **postures, gaits, tricks**: the lowest and highest angle that Kilo's
  postures (REST, SIT, STAND, the calibration pose, and the three moves of
  getting up after a fall, both ways round), its gaits (the walk, the trot
  and the crawl at every remote level, forward, backward, turning and
  steering) and its tricks ask of that joint. The list below the table
  names what asks each angle that is not reached, and for the postures and
  gaits the offsets that would reach all of it.

The numbers come from the body's own arithmetic (`body/src/core/joints.cpp`:
pulse = 10.963 microseconds per degree of servo angle + 600, Kilo's values
from the EEPROM), not from measuring the legs. The table of what is asked
is `ASKED` in `mind/kilo/calibration.py`; `mind/tests/test_calibration.py`
checks it is current.

### Kilo's joints as recorded (EEPROM dump of 2026-10-10)

From the arithmetic, not measured on the robot; `kilo cal show --host`
gives the offsets Kilo has now.

| Joint | Offset | Reaches | Not reached |
|---|---|---|---|
| 8 left front upper leg | +11 | -100 .. +62.3 | +73 (crawl, top level backward), +80 (trick wash_face) |
| 9 right front upper leg | -3 | -90.3 .. +80 | -99 (getting up after a fall) |
| 10 right hind upper leg | -16 | -57.3 .. +100 | -80 (SIT, the rear-up, the crawl's -79) |
| 11 left hind upper leg | +2 | -80 .. +91.3 | +100 (getting up after a fall), +95 (trick push_up) |
| 12 left front lower leg | -21 | -32.3 .. +120 | -45 (REST), -60 (trick butt_up) |
| 14 right hind lower leg | +7 | -120 .. +46.3 | +60 (trick hi) |

Joint 12 is your P1 finding B4 (the left front knee does not fold as far as
the right one at REST). Joint 10 is why the first rear-up fell
(`docs/rear-up-trick-results.md`); its fix is in `docs/rear-up-trick.md`
section 2. The others cost little: the crawl's top level backward, two
legacy tricks, and the far end of the get-up-after-a-fall moves.

## Which joint is which

| Joint | | Joint | |
|---|---|---|---|
| 0 | head pan | 8 | left front upper leg (shoulder) |
| 1 | head tilt | 9 | right front upper leg |
| 2 | tail | 10 | right hind upper leg |
| | | 11 | left hind upper leg |
| | | 12 | left front lower leg (knee) |
| | | 13 | right front lower leg |
| | | 14 | right hind lower leg |
| | | 15 | left hind lower leg |

(The numbers are the legacy OpenCat joint numbers, which the EEPROM and the
protocol use: `protocol/spec.toml` [[joints]], `body/src/core/joints.cpp`.)
Before changing a leg, check it: hold the calibration pose and nudge the
joint by 5 and back by -5; the leg you expect should move.
