# Power: light sleep and the battery

Kilo's battery (a 2S Li-ion pack, 1000 mAh, 7.4 Wh) lasts about 15 minutes
of play. Left switched on, it runs flat. This page says where the power
goes, what light sleep switches off, how to measure the difference, and
what the recordings show about play time. Results go in
[power-results.md](power-results.md).

Kilo has no current sensor. The one measure there is: how fast the
battery's voltage falls (mV per minute, from the body's TELEMETRY). The
faster it falls, the more is drawn. Every number below marked **estimate**
is not measured.

## Light sleep

Press **A7** (sleep, column A, top row). Kilo stops, lies down (REST) and
then, if **nothing else happens for 30 seconds**, goes to light sleep:

| What | Light sleep |
|---|---|
| Servos | off (the body's LIMP: every servo output off). He is already lying down, so nothing falls: the body's own watchdog does the same, REST then LIMP. |
| The Pi's CPU | idles. Awake, the main loop runs 900 times a second and never stops; asleep, it runs every 10 ms and waits in `wfi` (the core's clock stops) in between. |
| Ethernet | kept on while a cable is in (the laptop can still fetch recordings). With no cable, the PHY (the chip's cable driver) powers down and wakes by itself when a cable brings a signal; every 10 s it is fully on for 3 s, in case the laptop's port is asleep in the same way. |
| The Pi's green LED (ACT) | off, but a short flash every 5 seconds, so that asleep does not look like dead. |
| The flight recorder | keeps recording (every TELEMETRY: the battery trace is the measurement). If saving to the card is on, it saves what it holds before he sleeps, and goes on saving while asleep. Nothing is lost by sleeping. |

The body keeps running: its 50 Hz loop, the IMU, the IR receiver, the
link and TELEMETRY at 25 a second. That is how the remote still wakes him.

"Nothing happens" means: no key, no laptop connected or asking anything,
nothing moving. Any of these starts the 30 seconds again. A key that
moves Kilo (B6, C5, A4, B4, C4, A6, C6, A5, A2) calls the sleep off.

The 30 seconds are `kSleepAfterUs` in `kernel/mind/power/sleep.h`.

### Waking him

- **Any key** on the remote wakes him. The Pi is back to full speed at the
  next pass of its loop (within 10 ms). Then the key does what it always
  does: B6 stands him up. Keys that do nothing (rows 1-3, a held key, C7)
  only wake the Pi; A7 still holds, so 30 seconds later he sleeps again.
  B2 (the meow) is one of those too: it wakes him, and the wake's mew is
  what you hear (docs/sound.md).
- A walk, a turn, a trick or the behaviour key (A4, B4, C4, A6, C6, A2,
  A5) first powers the servos back to REST (a trick cannot start while
  limp), then does its job.
- **The laptop**: any connection (`kilo status --host`, `kilo log --host`,
  `kilo power --host`, ...) wakes him and keeps him awake while it is
  connected. When it disconnects, A7 still holds: 30 seconds later he
  sleeps again.
- Being picked up does not wake him. Waking takes 10 ms, so there is
  nothing to gain by waking early, and a bump would wake him for nothing.

How long a key takes, asleep: the remote's signal (about 70 ms), the body
passing it on (up to 20 ms), the Pi's sleeping pass (up to 10 ms): under
0.1 s. Awake it is the same but for those 10 ms.

### What light sleep cannot switch off

| What | Why |
|---|---|
| The ATmega328P, the IMU, the PCA9685 servo driver, the IR receiver, the ultrasonic sensor | the body firmware has no sleep mode, and there are about 146 bytes of flash left in it. The body must keep running to hear the remote. |
| The servos' own electronics | they sit on the battery directly; LIMP stops the pulses, not their supply. |
| The NyBoard's power LED, the 5 V converter's own losses | hardware. |
| The LAN9514 (the Pi 2 B's USB hub and Ethernet chip) as a whole | putting it in USB suspend needs a USB stack that can resume a device; Circle's cannot, and this kernel scans USB once. Only its PHY powers down. |
| HDMI / the composite video output | the firmware's TV service has no call in the property mailbox the kernel uses. |
| A slower CPU clock | without `CCPUThrottle` Circle already runs the ARM at the firmware's minimum, 600 MHz (`kernel/circle/doc/realtime.txt`; the boot log line `power: ARM ... MHz` shows it). |
| The other three CPU cores | already parked by the firmware (this kernel is single core). |

### The estimate (not measured)

Two parts, very different in size.

1. **The servos.** In the recording of 2026-11-17 (below), Kilo lying in
   REST with the servos powered drained the battery at about **90 mV per
   minute**. The Pi and boards alone, with no servo power, drained it at
   about **7.6 mV per minute** (8044 to 7855 mV in 25 minutes,
   [fast-boot-first-boot-results.md](fast-boot-first-boot-results.md)).
   The two were measured on different days at different charge, so the
   ratio is rough, but the servos holding REST drew several times what
   the Pi does. LIMP takes almost all of that away.
2. **The Pi** (estimate, from the Pi 2 B's published draw, about 1.1 W
   idle under Linux and 2.1 W loaded, [raspberry-pi.md](raspberry-pi.md)):
   the busy loop on one core at 600 MHz becoming `wfi` saves roughly 0.1 to
   0.25 W; the PHY powered down with no cable roughly 0.05 to 0.15 W; the
   LED about 0.01 W. Together about **0.15 to 0.4 W** at 5 V, 0.2 to 0.45 W
   from the battery through the 5 V converter. That is about 10 to 25 %
   of what the Pi and boards draw.

The mind's own low-battery mark (the A5 behaviour lies down in REST to
save the battery below 20 %, 7300 mV, and the eyes turn orange:
docs/distance-guard.md, "Low battery: orange") is judged on the battery at
rest: the highest reading of the last 20 seconds, so the sag of a walk does
not trip it. The body's own trips below are on its raw reading.

So, **estimate**: lying awake in REST, about 90 mV per minute; in light
sleep, about 6 to 7 mV per minute. From 7.6 V to the low-battery trip at
6.6 V that is about 11 minutes awake against about 2 to 3 hours asleep.
The voltage does not fall at an even rate (a Li-ion pack falls faster
near empty), so take these as the size of the difference, not a forecast.

## How to measure it

You need: the charger, the remote, the laptop with its Ethernet cable,
and about two hours. Nothing has to be timed by you: the robot records
the battery itself, and the laptop reads it whenever you come back.

**Test A: awake, standing** (the "left him on" case)

1. Charge the battery full. Switch Kilo on, no cable in.
2. Press B6 (stand). Leave him for at least 15 minutes (longer is better;
   come back whenever suits you).
3. Plug the cable in, then on the laptop:

   ```sh
   kilo power --host kilo.local
   kilo record fetch --host kilo.local --last 30m
   kilo power ~/kilo-logs/rec-<the time>.krec
   ```

   The first shows the trend over the last 1, 5, 15 and 30 minutes; the
   last one shows the trend per stretch of the recording. Copy both into
   power-results.md.

**Test B: light sleep**

1. Charge the battery full again. Switch Kilo on, no cable in.
2. Press A7. After 30 seconds the green LED on the Pi goes out and
   flashes once every 5 seconds: he is asleep.
3. Leave him for at least 30 minutes.
4. Plug the cable in (he stays asleep until the laptop connects), then
   the same three commands as in Test A.

**Test C (optional): lying in REST, awake.** The same as Test B with a
kernel built with light sleep off (`make -C kernel KILO_LIGHT_SLEEP=0`,
then `kilo update --host kilo.local`), so A7 is plain REST. This separates
the servos' part from the Pi's: B against C is the whole of light sleep;
A against C is standing against lying.

What to look at: the `asleep` stretch's mV per minute in Test B against
the `idle` stretch in Test A (and C). The first minute of each stretch is
left out on purpose: the voltage jumps when the load changes.

Also note from `kilo power --host`: `throttled` (0x50005 or any value
with 0x10000 set means the Pi saw under-voltage), the SoC temperature,
and `net_errors` (the PHY refused its power-down: say so).

## Where the power goes in play (the recordings)

From `~/kilo-logs/rec-20261117-214454.krec` (5 minutes: tricks, walks,
then A7 and 2.5 minutes lying in REST), read with `kilo power FILE`:

- **Lying in REST, servos on: about -90 mV per minute**, steady. The whole
  session, walks and tricks included, averaged about -87 mV per minute
  (7916 mV at 12 s to 7497 mV at 299 s). Lying still with the servos on
  cost about as much as playing.
- **Walking**: the voltage sags while he walks and comes back 40 to 240 mV
  (about 110 mV typically) within a few seconds of stopping. With an
  assumed pack and wiring resistance of 0.15 to 0.3 ohm (**estimate**),
  walking draws roughly 0.4 to 0.8 A more than standing while it lasts.
- **Tricks** (rear-up): a 60 to 90 mV sag, the same kind.
- **Standing** stretches were too short (under 90 s) for a slope.
- At about 90 mV per minute, a full pack (8.1 V) reaches the low-battery
  trip (6.6 V) in about 16 minutes: the 15 minutes you see.

**Is the low-battery reflex ending play early?** Not in this recording
(it ended at 7.5 V, no LOW_BATTERY event). The trips (body parameters):
warning 6900 mV (held 5 s), low 6600 mV (REST, then limp 2 s later),
critical 6300 mV. Because walking sags the reading by about 0.1 to 0.25 V,
the low trip can come while trotting when the pack at rest would still
read 6.7 to 6.85 V: about 1 to 2.5 minutes early. Separately, the
remote's rear-up key (A2 now; B2 at the time) refuses below 7400 mV: at 90 mV per minute that is
about 7 minutes into a full pack, so for the second half of a session the
trick is not available.

**Cheap things that would extend play time** (none needs body flash):

1. Do not hold the servos when lying: light sleep does this 30 s after A7.
   The same LIMP could follow any REST after a few seconds.
2. Find out why REST draws so much. Likely a servo pushing against a hard
   stop or the floor in the REST pose, or the held head (TELEMETRY says
   the head was held, HEAD_HELD, the whole time he lay there). Feel for a
   warm servo or listen for buzzing after A7; compare the REST slope with
   the head released. A REST pose that does not press any joint past
   where the leg can go would cost nothing.
3. Standing holds every servo too, and the balance controller keeps
   making small corrections. Comparing a standing stretch with balance on
   and off (Test A twice) would show what balance costs.
