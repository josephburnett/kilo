# The distance guard and the eyes

Kilo has an ultrasonic range finder in his head: the two silver cans are
the eyes, and the module has six small RGB lights (WS2812) around them.
The distance guard uses the range so that Kilo does not walk into a wall,
and the eyes show what it sees:

| Eyes | Zone | What Kilo does while walking forward |
|---|---|---|
| green | clear (or nothing seen) | walks on |
| blue | too close | stops (the normal, planned stop), sits, and starts the A5 behaviour in its sitting form |
| red | very close | backs off a short way (about 4-5 cm, slowly), then stops and sits as for blue |

Only walking **forward** sets the guard off. Walking backward, turning on the
spot, a trick (the rear-up), being picked up, and a laptop holding the body
do not. The eyes show the zone at all times, walking or not.

The code is `kernel/mind/guard/guard.h` (the thresholds are all at its top)
and `guard.cpp`; the mind wires it in (`kernel/mind/mind.cpp`). The Python
daemon (`kilo run --remote`) does not have the guard: it is in the C++ mind
that runs on Kilo.

## While the guard holds

After a stop the guard **holds** until the way is clear again (green for
1 s). While it holds:

- forward keys are refused: A4, B4, C4 (each starts or turns into a forward
  walk) and B7 when it would go forward. The log says
  `remote A4 (0x68, trot): refused (distance guard holding: blue, 287 mm
  ahead; turn, back up or B6)`, and the session metrics count it;
- turning (A6, C6), backing up (B5 down to the negative levels) and the
  posture keys (B6 stand, A7 sleep, C5) work as always. A turn or backing
  key also ends the sitting behaviour the guard started, so Kilo can be
  steered away;
- C7 (freeze) works as always.

A key that the guard lets through while it is stopping or backing takes
over from it: Kilo does what the key says, and the guard keeps holding.

## The thresholds

All in `kernel/mind/guard/guard.h`. Blue depends on the speed, because a
faster walk needs more room to stop:

    blue = 100 mm gap left after the stop
         + speed x 0.45 s   (the reading comes in, the stream ahead, the servos)
         + the planned stop's own way (speed^2 / 2a; a by gait: walk 75, trot 120, crawl 8 mm/s^2)
    never below 200 mm (also the threshold when Kilo is not walking, for the eyes)
    left again 80 mm above it

    red  = below 150 mm, left again above 200 mm

| Gait at the remote's top level (+5) | Speed | Blue below | Simulated stop: gap left to the wall |
|---|---|---|---|
| walk | 97 mm/s | 206 mm | 116 mm (123 mm with noisy readings) |
| trot | 180 mm/s | 316 mm | 140 mm (124 mm noisy) |
| trot | 200 mm/s (if the trot gets faster) | 357 mm | not simulated |
| crawl | 27 mm/s | 200 mm | 196 mm |

The planned stop's own way was measured in the planner (2026-11-17):
walk 80-97 mm/s stops within 39-57 mm, trot 144-184 mm/s within 79-127 mm,
crawl within 23-30 mm. The guard's figures are rounded up from those
(`kernel/host/tests/test_guard.cpp` checks they stay above the planner's).
The "gap left" column is the virtual walk of the same test: the body's
position follows the feet the mind streamed, 120 ms late (the servos), and
the range is read from there every 80 ms. It is a simulation: the real
floor (slipping feet, a wall at an angle, a soft curtain) can differ.

## How the readings are cleaned up

What the session recordings of 2026-11-17 show (`kilo record decode FILE
--csv`, the `dist_mm` column; `/home/joe/kilo-logs/rec-20261117-214454.krec`
and the `run-*-walk-*.krec` files):

- a new reading every 80 ms (12.5 a second; the TELEMETRY flag DIST_FRESH
  marks them; telemetry itself comes 25 or 50 times a second);
- still and with a flat target the readings agree within a few mm;
  approaching a wall at a walk the readings fall smoothly (926 mm to 540 mm
  in 5.4 s, 71 mm/s);
- while walking, 5-15 % of the readings jump by more than 100 mm for one
  reading: mostly too long (a far wall seen past the near one, up to
  2973 mm), sometimes too short (something at the side, a leg); and with two
  targets in view the readings can alternate between them;
- "no echo" (0) is rare: under 1 % of readings, at most a few in a row
  while walking;
- with the head down (lying, REST) the sensor reads the floor at 54-63 mm.

So the guard:

1. uses only new readings (DIST_FRESH);
2. drops "no echo" (0), anything beyond 2.5 m, and any reading taken while
   the beam points down at the floor: the head held tilted down plus the
   body's nose-down pitch over 12°;
3. takes the median of the readings of the last 0.3 s (up to 3) and needs
   at least two, so one short echo (a hand waved past, a stray reflection)
   never stops Kilo; a hand held in front for a quarter of a second does;
4. without a reading for 0.4 s it has no range: the zone is green.

## Switching it off and on

The guard is on whenever the mind starts. From the laptop:

    kilo guard off --host kilo.local
    kilo guard on --host kilo.local
    kilo guard --host kilo.local            how it stands now (also: --json)

Switching it does not need control of Kilo and moves nothing. Off, Kilo
walks on whatever he sees, but the eyes still show the zone. The setting
lasts until the mind restarts.

`kilo status --host kilo.local` shows it too:

    guard       on  zone green  watch  412 mm (raw 409)  blue < 200 mm  red < 150 mm
    eyes        green (on the eyes)
    guard count stops 1  pullbacks 0  forward keys refused 2  zone changes 6

"not shown: the body firmware has no eyes yet (flash it)" in the eyes line
means the body still runs a firmware from before the eyes (see below): the
mind keeps the colour and logs it, but the lights stay as they are.

## Reading a session afterwards

Every zone change and every guard action is a line in Kilo's log and a mark
in the flight recorder:

    guard zone=blue mm=287 raw=291 blue=316 red=150 v=180 gait=trot action=watch
    guard action=stopping zone=blue mm=287 blue=316 v=180 gait=trot: too close: the planned stop
    guard action=holding ...: stopped: sitting, the behaviour runner's sitting variant
    remote A4 (0x68, trot): refused (distance guard holding: blue, 287 mm ahead; ...)
    guard action=watch ...: clear: forward walking allowed again

To read them later:

    kilo log --host kilo.local | grep guard           the live log, and what it kept
    kilo record fetch --host kilo.local ...            a time range, then
    kilo record decode FILE --log | grep guard         the lines in it
    kilo record decode FILE --csv                       dist_mm per telemetry, to plot against them

The session metrics (`kilo metrics --host kilo.local`, also in `kilo status
--host`) have a `guard:` line: on, zone, action, eye colour, zone changes,
stops, pullbacks, refused keys, releases, readings, dropouts, floor
readings ignored, and the closest range seen while walking forward.

## The eyes

The module on the head is Petoi's RGB ultrasonic sensor: one signal wire
for the range finder on D6, and the six WS2812 lights' data on D7 (Grove
socket G4; the legacy firmware's `RgbUltrasonic(6, 7)` in
`src/ultrasonic.h`, which lit them white, yellow, blue and red by distance).
The new body firmware drives them from the mind's `SET_EYES`
(`body/src/hal/eyes.cpp`): green 0x30, blue 0x30 or red 0x30 on all six.

What it costs the body: 124 bytes of program memory, no RAM (the body has
3 bytes of RAM left, so it stores nothing: every 5 Hz refresh of the colour
is shown again), and no extra stack. Showing a colour turns interrupts off
for one light at a time, about 33 µs, six times; a byte on the link takes
40 µs and the serial port holds two, so nothing is lost; the servos are
driven by their own chip and do not notice. It never shows while an echo
is being timed, so the range is not disturbed.

**The lights need the new body firmware.** Until the body is flashed, the
body says (in its HELLO) that it has no eyes, and the mind does not send
the colour. Everything else (the guard, the zones, the log, `kilo status`)
works with the old firmware. After the flash, `kilo status --host` shows
`eyes ... (on the eyes)`.

## The sitting behaviour

The guard starts the A5 behaviour runner in its sitting form: everything
the behaviours would do standing, Kilo does sitting (the log says
`behaviour: posture stand: sit (the sitting variant)`), with the small body
moves of the alive layer (`kernel/mind/behaviour/alive.h`). B6, A7 and C7
stop it as they stop A5, and so do the turning and backing keys.

## Known limits

- He has no rear sensor: backing off is short (2 s at 40 mm/s, about
  4-5 cm in the simulation) and slow; something right behind him is hit.
- Very close (under about 3 cm) the sensor may give no echo or nonsense; a
  wall reached in the dark of a corner can read as clear.
- Soft things (a curtain, a cushion, fur) and walls at a steep angle may
  not echo: the guard sees nothing there.
- Only what is in front of his head counts: a table edge above the beam
  or a step down below it is not seen.
- The guard acts on the remote's walks; a walk a laptop drives is not
  stopped (the laptop holds the body then).
