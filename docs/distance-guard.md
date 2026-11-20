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
| dim orange | clear, but the battery is low ("Low battery: orange" below) | walks on; blue and red still show over it |

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
  walk), B7 when it would go forward (also out of a turn in place: that is
  straight forward), and A6 / C6 while walking forward (a steer bends the
  walk but keeps it going forward). The log says
  `remote A4 (0x68, trot): refused (distance guard holding: blue, 287 mm
  ahead; turn, back up or B6)`, and the session metrics count it;
- turning (A6, C6), backing up (B5 down to the negative levels) and the
  posture keys (B6 stand, A7 sleep, C5) work as always. A turn or backing
  key also ends the sitting behaviour the guard started, so Kilo can be
  steered away;
- C7 (freeze) works as always.

A key that the guard lets through while it is stopping or backing takes
over from it: Kilo does what the key says, and the guard keeps holding. If
that leaves a forward walk going (B5 from +5 only slows it to +4), the
guard stops it again.

## Lying down and light sleep

After A7 Kilo lies in REST with the head down, and in light sleep he goes
limp the same way; the sensor then reads the floor at 55-63 mm. The guard
treats every reading while he lies (the REST or LIMP posture asked and no
walk, or the body limp by itself) as the floor: not a reading, so the zone
stays green, the eyes stay green, nothing is logged, and nothing wakes him.
Without that the eyes showed red all night and flipped to green and back
at each missed echo.

## The thresholds

All in `kernel/mind/guard/guard.h`. Blue depends on the speed, because a
faster walk needs more room to stop:

    blue = 100 mm gap left after the stop
         + speed x 0.45 s   (the reading comes in, the stream ahead, the servos)
         + the planned stop's own way (speed^2 / 2a; a by gait: walk 75, trot 120, crawl 8 mm/s^2)
    never below 200 mm (also the threshold when Kilo is not walking, for the eyes)
    left again 80 mm above it

    red  = below 150 mm, left again above 200 mm

Simulated (the host test, below), at the remote's top levels; the trot's
levels are 57.6, 100.8, 144, 187.2 and 230.4 mm/s since the faster trot
(`docs/walking-and-balance.md` §13.4):

| Gait, level | Speed | Blue below | The planned stop's own way: planner / the guard allows | Gap left at the wall: clean / noisy |
|---|---|---|---|---|
| walk +5 | 96.8 mm/s | 206 mm | 19 / 62 mm | 155 / 155 mm |
| trot +4 | 187.2 mm/s | 330 mm | 43 / 146 mm | 220 / 220 mm |
| trot +5 | 230.4 mm/s | 425 mm | 66 / 221 mm | 280 / 280 mm |
| crawl +5 | 26.9 mm/s | 200 mm | 23 / 45 mm | 196 / 196 mm |

Steering while walking (A6 / C6, §13.2) is still walking forward for the
guard: the same thresholds at the same speed. A steered walk goes a little
slower (the strides are shortened to keep the bend), so it stops a little
further from the wall:

| Gait, level, steer | Blue below | The planned stop: planner | Gap left: clean / noisy |
|---|---|---|---|
| walk +5, left 3 | 206 mm | 9 mm | 182 / 182 mm |
| trot +4, left 1 | 330 mm | 45 mm | 234 / 234 mm |
| trot +5, left 1 | 425 mm | 45 mm | 320 / 305 mm |
| trot +5, right 2 | 425 mm | - | 333 / 333 mm |
| trot +5, left 3 | 425 mm | 33 mm | 354 / 345 mm |

"Noisy" is readings +-12 mm, 5 % outliers and 3 % dropouts. The planner
column is the real planner's stop (its support feet's travel from the stop
to standing), against the guard's speed^2 / 2a. The decelerations in the
guard date from before the faster stride slew (1.5 mm a tick since §13.3),
when the planner's stops were longer (walk 39-57 mm, trot 79-127 mm); now
they are about a third of what the guard allows, so the guard stops early:
about 150 to 350 mm short of the wall instead of the 100 mm it aims at. I
left the constants as they were: that is the safe side, and the floor
(slipping feet, a soft or slanted wall) may need the room. If the early stop
gets in the way, the trot's deceleration is the one to raise
(`kDecelTrotMmS2` 120 to about 300 would stop the trot at +4 and +5
116 to 152 mm short in the simulation).

`kernel/host/tests/test_guard.cpp` checks that the guard's allowance stays
above the planner's stop and that every approach above ends at least
100 mm from the wall. The approach is a simulation: the body's position
follows the feet the mind streamed, 120 ms late (the servos), and the range
is read from there every 80 ms. It does not turn the body (the virtual body
has no yaw), so a steered approach is a straight one at the steered speed.
The real floor can differ.

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
   body's nose-down pitch over 12°, or Kilo lying (REST, LIMP);
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
(`body/src/hal/eyes.cpp`): green 0x30, blue 0x30 or red 0x30 on all six,
or for orange red 0x30 with green 0x10.

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

## Low battery: orange

When the battery is low the eyes are a dim orange instead of green. Blue
and red always show over it: the way ahead matters more. Low means either:

- the battery **at rest** is below 20 % (7300 mV on the resting curve; the
  same mark at which the A5 behaviour lies down in REST to save the
  battery). It is no longer low once it reads 25 % (7400 mV) again;
- or the body itself says low battery (its LOW_BATT mode, below 6.6 V).

It does not need the A5 behaviour to be on. The log says when it starts
and ends, e.g. `battery: tired (7250 mV at rest, 16 %, below 20 %): the
eyes orange while the way is clear` and `battery: no longer tired (7400
mV at rest, 30 %)`; `kilo guard --host` and `kilo status --host` show
`eyes  orange: low battery, Kilo is tired ...`.

**At rest** matters: moving legs pull the battery's voltage down by 0.1 to
0.25 V for as long as they move (the "sag"), and it comes back within a
second of them stopping. On 2026-11-19 at a wall Kilo read 7408 mV lying
still, above the mark, but the guard's sitting behaviour lay him down in
REST instead of sitting: most likely the reading while his legs were still
moving was below it. So the mind now judges the battery by the highest
reading of the last 20 seconds (`kernel/mind/power/battery.h`
`RestedBattery`): a moment of rest, or the lightest moment of a walk, in
that time is enough to see the real level. A walk that never pauses for
20 seconds still reads a little low. `kilo status` and the metrics' `mv=`
show the reading as it is; the metrics' `battery:` line also has
`rested_mv=`.

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
