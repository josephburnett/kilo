# The distance guard and the eyes

Kilo has an ultrasonic range finder in his head: the two silver cans are
the eyes, and the module has six small RGB lights (WS2812) around them.
The distance guard uses the range so that Kilo does not walk into a wall
and reacts to a hand in front of his face, and the eyes show what it sees:
green clear (or nothing seen), blue too close, red very close, dim orange
clear but the battery is low ("Low battery: orange" below).

The code is `kernel/mind/guard/guard.h` (the thresholds are all at its top)
and `guard.cpp`; the mind wires it in (`kernel/mind/mind.cpp`).

## What he does

Since 2026-11-19 he reacts in every state but light sleep and a laptop
holding him (the owner: "he should always react, unless sleeping"):

| Kilo is | Blue (too close) | Red (very close) |
|---|---|---|
| walking forward, any gait, also steering | the quick stop (below), then he sits and starts the A5 behaviour in its sitting form | backs off a short way (4-8 cm, slowly), the quick stop, sits; hisses. In the crawl he only stops and sits (the crawl takes 4 s to turn round) |
| turning in place (A6, C6) | nothing: a wall seen as he turns is no reason to stop turning | the quick stop, then he sits; hisses |
| walking backward | nothing: he is going away from it | nothing (the eyes go red) |
| standing still | he looks at it (the head straight ahead) and sits | backs off a short way and sits; hisses |
| sitting | he looks at it | he leans back (shoulders and knees back, the head pulled up, the tail up, about 2 s) and hisses |
| lying (A7, REST) | nothing: lying, the sensor sees the floor under his nose | nothing |
| in light sleep, or a laptop holds him, or a trick (the rear-up) plays, or he is picked up | nothing | nothing |

"Something comes close" means the zone **changes** to blue or red: standing
next to a wall that was already there does nothing; a hand put in front of
him does. While walking forward the zone itself counts: he stops at blue
whenever he walks into it.

How fast: in the host test (`kernel/host/tests/test_guard.cpp`, a hand put
in front of him at 180 or 100 mm) the guard acts 0.10 to 0.14 s after the
hand appears (two readings, 80 ms apart, must agree), at every gait. The
quick stop then has the trot stopped about 0.15 s later; he sits about 1 s
after the hand appeared (the walk and the trot) or 3 s (the crawl, which
squares its legs up slowly); after a back-off about 3 s.

### The quick stop

`docs/walking-and-balance.md` §14: the feet in the air are put down where
they are, the strides go to zero in 120 ms so the body stops, all four feet
stay down for 200 ms, then the legs step under the shoulders. The same stop
is B6's, C7's and the other remote keys'.

## While the guard holds

After any reaction the guard **holds** until the way is clear again (green
for 1 s). While it holds:

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
- C7 (freeze) works as always;
- he still reacts: a hand that comes very close while he sits after a stop
  makes him lean back.

A key that the guard lets through while it is stopping or backing takes
over from it: Kilo does what the key says, and the guard keeps holding. If
that leaves a forward walk going (B5 from +5 only slows it to +4), the
guard stops it again.

**The release rule.** He backs off, leans back or stops a turn at most
**twice** for one thing in front of him; after that only the eyes show it,
until the zone has been green for a second. So a hand reaching for him, or
a hand going back and forth, never makes him back away over and over; the
log says `eyes only (backed off twice: clear for 1 s first)`.

## The floor, lying down and light sleep

The beam points a little down when the head is held tilted down or the
body leans nose down. Then the floor is in view: at a tilt of d degrees
below level it is about 80 mm / sin d away (the sensor is about 80 mm
above the floor when he stands; at 25 degrees that is 190 mm). The guard
counts a reading from three quarters of that distance on as the floor and
ignores it, and everything nearer as real: a hand in front of a lowered
head still counts. Up to 12 degrees down the floor is not in view.

After A7 Kilo lies in REST with the head down, and 3 seconds later (light
sleep) he goes limp the same way; the sensor then reads the floor at 55-63 mm. Every
reading while he lies (the REST or LIMP posture asked and no walk, or the
body limp by itself) is the floor: the zone stays green, the eyes stay
green, nothing is logged, and nothing wakes him. Without that the eyes
showed red all night and flipped to green and back at each missed echo.

While the mind walks Kilo (the remote's walks) and the A5 behaviour is
off, his head is held straight ahead: the behaviour's last look (held until
something else moves the head) could leave it turned or tilted, the beam
on the floor or to the side, and the guard blind to what is ahead.

## The thresholds

All in `kernel/mind/guard/guard.h`. Blue depends on the speed when walking
forward, because a faster walk needs more room to stop:

    blue = 100 mm gap left after the stop
         + speed x 0.40 s   (the readings come in, the stop reaches the body, the servos follow)
         + 5 mm + the quick stop's own way (speed^2 / 2a; a: walk 400, trot 700 mm/s^2; the crawl 20 mm)
    never below 200 mm (also the threshold when not walking forward)
    left again 80 mm above it

    red  = below 150 mm, left again above 200 mm

| Gait, level | Speed | Blue below (before 2026-11-19) | The quick stop's own way: planner / the guard allows | Stopped short of the wall (simulated, clean / noisy) |
|---|---|---|---|---|
| walk +5 | 96.8 mm/s | 200 mm (206) | 5 / 17 mm | 166 / 166 mm |
| trot +4 | 187.2 mm/s | 205 mm (330) | 10 / 30 mm | 142 / 142 mm |
| trot +5 | 230.4 mm/s | 235 mm (425) | 13 / 43 mm | 162 / 180 mm |
| crawl +5 | 26.9 mm/s | 200 mm (200) | 10 / 20 mm | 196 / 196 mm |

Steering (A6 / C6 while walking forward) is still walking forward: the
same thresholds at the same speed; the steered walk goes a little slower,
so it stops a little further from the wall (146 to 212 mm in the test).

"Noisy" is readings ±12 mm, 5 % outliers and 3 % dropouts. The planner
column is the real planner's quick stop (its support feet's travel from the
stop to standing, the worst over the moments of a cycle), against the
guard's allowance, which is at least twice it: the floor (slipping feet, a
soft or slanted wall) may need the room. With the old planned stop the
blue distance at the trot's top speed was 425 mm: he stopped 28 cm short
of a wall; now about 16-18 cm.

`kernel/host/tests/test_guard.cpp` checks that the guard's allowance stays
above the planner's stop and that every approach ends at least 100 mm from
the wall. The approach is a simulation: the body's position follows the
feet the mind streamed, 120 ms late (the servos), and the range is read
from there every 80 ms. It does not turn the body (the virtual body has no
yaw). The real floor can differ.

## How the readings are cleaned up

What the session recordings of 2026-11-17 show (`kilo record decode FILE
--csv`, the `dist_mm` column):

- a new reading every 80 ms (12.5 a second; the TELEMETRY flag DIST_FRESH
  marks them; telemetry itself comes 25 or 50 times a second);
- still and with a flat target the readings agree within a few mm;
  approaching a wall at a walk the readings fall smoothly;
- while walking, 5-15 % of the readings jump by more than 100 mm for one
  reading: mostly too long (a far wall seen past the near one), sometimes
  too short (something at the side, a leg);
- "no echo" (0) is rare: under 1 % of readings while walking; very close
  (under about 3 cm) the sensor gives no echo at all.

So the guard:

1. uses only new readings (DIST_FRESH);
2. drops "no echo" (0), anything beyond 2.5 m, and the floor (above);
3. takes the median of the readings of the last 0.3 s (up to 3) and needs
   at least two, so one short echo (a hand waved past, a stray reflection)
   never sets it off; a hand held in front for a sixth of a second does;
4. without a reading for 0.4 s it has no range: the zone is green; except
   right after something very close (under 150 mm): then no echo means
   closer than the sensor can measure, and the range holds for 1 s.

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

## Reading a session afterwards

Every zone change and every guard action is a line in Kilo's log and a mark
in the flight recorder, and so is every change of **what the guard makes of
it**: one line each time the state Kilo is in or the reason it acts or not
changes (not every reading):

    guard zone=blue mm=287 raw=291 blue=316 red=150 v=180 gait=trot action=watch
    guard decides: zone=blue mm=287 raw=291 seen=reading blue=316 floor_from=0 action=watch state=walk-forward: the quick stop
    guard action=stopping zone=blue mm=287 blue=316 v=180 gait=trot: too close: the quick stop
    guard action=holding ...: stopped: sitting, the behaviour runner's sitting variant
    guard decides: zone=red mm=89 raw=91 seen=reading ... state=sitting: react (holding)
    guard: leans back
    guard decides: zone=green mm=none raw=54 seen=floor ... state=lying: nothing seen (lying: every reading is the floor)
    guard decides: ... state=picked-up: eyes only
    remote A4 (0x68, trot): refused (distance guard holding: blue, 287 mm ahead; ...)
    guard action=watch ...: clear: forward walking allowed again

In a `guard decides` line: `seen` is what the last new reading was
(`reading`, `no-echo`, `too-far`, `floor`), `floor_from` where the floor
starts for the beam's tilt then (0: not in view), `state` what Kilo was
doing (`walk-forward`, `turn-in-place`, `walk-back`, `stopping`,
`standing`, `sitting`, `lying`, `moving` between postures; or why the
guard stays out of it: `laptop`, `asleep`, `trick`, `picked-up`, `paused`,
`off`), and after the colon what it did.

To read them later:

    kilo log --host kilo.local | grep guard           the live log, and what it kept
    kilo record fetch --host kilo.local ...            a time range, then
    kilo record decode FILE --log | grep guard         the lines in it
    kilo record decode FILE --csv                       dist_mm per telemetry, to plot against them

The session metrics (`kilo metrics --host kilo.local`, also in `kilo status
--host`) have a `guard:` line: on, zone, action, eye colour, zone changes,
stops, pullbacks, refused keys, releases, reactions (while standing,
sitting or turning), hisses, readings, dropouts, floor readings ignored,
and the closest range seen while walking forward.

### Why he did not react on 2026-11-19 (what was found)

- The guard acted only for a forward walk: standing, sitting and turning it
  only coloured the eyes.
- After the guard (or B6) changed his posture, the IMU percepts could read
  the sit's 26 degrees nose up, or standing up from it, as being **picked
  up**; picked up, the guard stays out of it until he has been calm at his
  old attitude for 1.5 s, and a walk started before that (B6, then A4 at
  once) went on unguarded to its end. In the host test a sit did exactly
  that. Every posture the mind asks for now keeps the IMU percepts quiet
  over the move (`Mind::posture_moves`), as the behaviour's own postures
  always did.
- The head held tilted down by the behaviour's last look stayed so while
  walking (nothing released it), and with the old floor rule every reading
  then counted as the floor. Now the head is straight while the mind walks,
  and only readings as far as the floor are the floor.
- A hand closer than about 3 cm gives no echo, which read as "nothing".
- The raw 54 and 63 mm readings with "no range" were taken lying in REST,
  where every reading is the floor (55-63 mm): that one is as intended.

## The eyes

The module on the head is Petoi's RGB ultrasonic sensor: one signal wire
for the range finder on D6, and the six WS2812 lights' data on D7 (Grove
socket G4; the legacy firmware's `RgbUltrasonic(6, 7)` in
`src/ultrasonic.h`, now in git history only, which lit them white, yellow, blue and red by distance).
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

- the battery **at rest** is below 5 % (7000 mV on the resting curve; the
  same mark at which the A5 behaviour lies down in REST to save the
  battery). It is no longer low once it reads 10 % (about 7150 mV) again;
- or the body itself says low battery (its LOW_BATT mode, below 6.2 V).

It does not need the A5 behaviour to be on. The log says when it starts
and ends, e.g. `battery: tired (6950 mV at rest, 4 %, below 5 %): the
eyes orange while the way is clear` and `battery: no longer tired (7200
mV at rest, 12 %)`; `kilo guard --host` and `kilo status --host` show
`eyes  orange: low battery, Kilo is tired ...`.

**At rest** matters: moving legs pull the battery's voltage down by 0.1 to
0.25 V for as long as they move (the "sag"), and it comes back within a
second of them stopping. So the mind judges the battery by the highest
reading of the last 20 seconds (`kernel/mind/power/battery.h`
`RestedBattery`): a moment of rest, or the lightest moment of a walk, in
that time is enough to see the real level. A walk that never pauses for
20 seconds still reads a little low. `kilo status` and the metrics' `mv=`
show the reading as it is; the metrics' `battery:` line also has
`rested_mv=`.

## The sitting behaviour

The guard starts the A5 behaviour runner in its sitting form: everything
the behaviours would do standing, Kilo does sitting (the log says
`behaviour: posture stand: sit (the sitting variant)`), with the moves of
the alive layer (`kernel/mind/behaviour/alive.h`: leans, shrugs, a paw
lifted, tail swishes, the head looking around). B6, A7 and C7 stop it as
they stop A5, and so do the turning and backing keys.

## Known limits

- He has no rear sensor: backing off is short (2 s at 40 mm/s, 4-8 cm in
  the simulation) and slow; something right behind him is hit.
- Very close (under about 3 cm) the sensor gives no echo; the guard holds
  the last very close range for a second, but something that arrives that
  close in one step is not seen.
- Soft things (a curtain, a cushion, fur) and walls at a steep angle may
  not echo: the guard sees nothing there.
- Only what is in front of his head counts: a table edge above the beam
  or a step down below it is not seen; with the head turned by the A5
  behaviour (standing or sitting) he sees what the head looks at.
- The guard acts on the mind's own walks and postures; a walk a laptop
  drives is not stopped (the laptop holds the body then).
