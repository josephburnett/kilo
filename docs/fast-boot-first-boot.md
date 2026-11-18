# Fast boot: first boot of the Circle kernel on the Pi

What you need to do to start the new mind kernel on the Pi 2 B for the
first time, and what you should see. It is the hardware check for phases
P1 to P4 of docs/fast-boot.md: the Pi says hello to Kilo's board and
keeps the link alive, with no Linux, the laptop talks to the Pi over the
network cable (`kilo status`, `kilo log`, `kilo posture`), Kilo answers
the IR remote (step 9), and, as an optional last step, the Pi rewrites
Kilo's board firmware itself (`kilo flash --host`, step 12); after a walk,
the laptop downloads what the mind recorded of it (step 14).

Nothing here is timed. Every step waits for you, and you can stop at any
step and carry on later. Write what you see in
`docs/fast-boot-first-boot-results.md`.

Words used below:

- **Kernel**: the one program the Pi runs, `kernel7.img`. Here it is our
  mind, built on the laptop.
- **Loader**: a small kernel on the SD card. It waits for the laptop to
  send it the real kernel over the network cable, then runs it. Nothing is
  written to the card.
- **ACT LED**: the small green light on the Pi, next to the red power light.
- **HELLO**: the first message Kilo's board sends when it starts, or when
  the Pi asks who is there.
- **Address**: the Pi's number on the network, like `192.168.88.23`. The
  router hands it out (that is DHCP) under a name you can look up.
- **Link-local**: addresses that start with `169.254.`. Two machines on one
  cable with no router between them can use them to reach each other.
- **Flashing**: writing new firmware (the program Kilo's board runs) into
  the board's chip. The board's chip has a tiny start-up program,
  **optiboot**, which listens for new firmware for about a second after
  the chip is reset.
- **Reset line**: the wire that restarts Kilo's board when it is pulled
  low. The Pi pulls it with its header pin 7 (GPIO4).
- **Key**: the mind talks over the network only to laptops it knows. Each
  laptop has a key pair: a private key, a file that stays on that laptop and
  is never shared, and a public key, a line of 44 letters and digits that
  may be shown to anyone. The public keys of the laptops Kilo knows are
  built into the kernel (kernel/README.md, "Who may talk to Kilo: keys").

## What you need

- The Pi 2 B, its own power supply, and an SD card you can wipe.
- An Ethernet cable from the Pi to the house router.
- On the laptop, the `kilo` program in `~/OpenCat/.venv` (it is there if
  `~/OpenCat/.venv/bin/kilo --help` prints its help).
- A clear patch of floor where Kilo can stand up and walk a few steps
  (steps 8 and 9).
- Kilo's IR remote. Keys are named by where they sit: columns A, B, C from
  left to right, rows 1 to 7 from the bottom to the top. So B7 is the middle
  key of the top row.
- Kilo, charged, and **one** of these two ways to connect it:
  - **B. The Pi's pin header** (the expected wiring; the kernel's default
    build), with Kilo's USB adapter unplugged from Kilo (the two must not be
    connected at the same time). Either:
    - **the Pi plugged onto the NyBoard's socket**: the socket joins the
      Pi's header pins 1 to 10 to the NyBoard, so the serial line (pins 8
      and 10) and the reset line (pin 7) are connected with nothing to add
      (docs/raspberry-pi.md); or
    - **three wires**, on the bench, with the Pi off Kilo:
      - Pi pin 8 (GPIO14, the Pi's transmit) to the NyBoard's receive (RX),
      - Pi pin 10 (GPIO15, the Pi's receive) to the NyBoard's transmit (TX),
      - Pi pin 6 (ground) to the NyBoard's ground.

      Pin 1 has a square solder pad underneath. It is at the end of the
      header away from the USB ports, in the inner row. Odd numbers run
      along the inner row (1, 3, 5, ...), even numbers along the outer row
      at the board's edge (2, 4, 6, ...). https://pinout.xyz shows the
      picture. Never connect pins 2 or 4 with a wire: they are 5 V. The
      NyBoard's serial on its Pi socket is 3.3 V (docs/raspberry-pi.md,
      which also says to check that with a meter first).
  - **A. The USB cable** (the old wiring, now the fallback): Kilo's USB
    adapter cable into one of the Pi's USB ports. It needs a kernel built
    for it: `make -C kernel KILO_BODY_SERIAL=auto` in step 1. The default
    kernel never looks for the cable.

## Before you start: power

**On the NyBoard's socket (way B)**, pins 2 and 4 bring the NyBoard's 5 V to
the Pi: the NyBoard makes it from Kilo's battery (docs/raspberry-pi.md,
"Power"; that is what Petoi's socket is for), so the Pi starts as soon as
Kilo is switched on. **Do not plug the Pi's own power supply in as well**:
that would join two 5 V supplies, and nothing here has checked whether that
is safe. The Pi then runs only while Kilo is on, from Kilo's battery. The
three bench wires carry no 5 V, so with them the Pi keeps its own supply.

**With the USB cable (way A)**: the cable joins Kilo's 5 V and the Pi's 5 V
(see docs/pi-outages-2026-10-13.md). With Kilo switched off, the Pi powers Kilo's
board through the cable and its supply dips; that has reset the Pi before.
So, with the USB cable (way A):

- **Switch Kilo on before the cable goes into the Pi**, and keep it on.
- If the Pi resets when you plug the cable in, that is this problem, not
  the kernel. Start again from step 3.

## Before you start: the remote

The kernel listens to the IR remote from the moment it starts, as the Pi
did under Linux. Keep the remote out of reach until step 9, so that a
stray key press does not make Kilo move on the desk.

## Steps

### 1. Build and write the card (on the laptop)

First this laptop's key, once (if `kilo key show` already prints a line,
and that line is in `kernel/keys/authorized_keys`, skip this):

```
~/OpenCat/.venv/bin/kilo key init
~/OpenCat/.venv/bin/kilo key show >> kernel/keys/authorized_keys
```

The first command makes the key pair: the private key goes into
`~/.config/kilo/host_key` (only you can read it; never copy it anywhere),
and it prints the public line, which ends with this laptop's name. The
second adds that line to the list the kernel is built with. Commit the
list (it holds public keys only). Without a key in it, the next command
stops and says this.

Then build:

```
git submodule update --init kernel/circle
make -C kernel
make -C kernel boot
```

The last line downloads the Pi's start-up files once; it needs the internet.
That builds the kernel for way B (the header). For way A (the USB cable),
build it with `make -C kernel KILO_BODY_SERIAL=auto` instead of the plain
`make -C kernel` (here and in the `make -C kernel card` below).
Put the SD card in the laptop. It must be FAT formatted and empty, or hold
only Pi boot files. Then (replace the path with where the card is mounted):

```
kernel/tools/make-sd.sh /media/$USER/BOOT --netboot
```

The script refuses a card that holds anything else, for example a Raspberry
Pi OS card. Eject the card and put it in the Pi.

**No card reader on the build machine?** Then stage the files and copy them
with any computer that has one:

```
make -C kernel card
```

makes the folder `kernel/build/card/netboot/` with the six files
(`bootcode.bin`, `start_cd.elf`, `fixup_cd.dat`, `LICENCE.broadcom`,
`config.txt`, `kernel7.img`). Copy that folder's files onto an empty,
FAT-formatted card with the other computer (a file manager is fine; the card
needs no partitioning when it came formatted). Nothing else may be on the
card. `kernel/build/card/kernel/` is the same for the production card.

### 2. Cable the Pi

- Ethernet cable from the Pi to the router.
- Way B: the Pi on the NyBoard's socket, or the three header wires, with
  Kilo's USB adapter unplugged from Kilo.
- Way A: do not plug the USB cable in yet.

### 3. Power the Pi

Plug in the Pi's power supply (on the NyBoard's socket, read "Before you
start: power" first). The red light comes on. The green ACT light
flickers while the Pi reads the card, then goes dark. The loader does not
use the ACT light, so dark is normal: the loader is now waiting.

### 4. Find the Pi's address

Open the router's page that lists the connected devices (often called
"DHCP leases" or "Clients"). Look for the name **kilo-netboot**. If no name is
shown, look for a device whose hardware address (MAC) starts with
`b8:27:eb`. Its address looks like `192.168.88.X`; write it down.

If it is not there after a minute: check the Ethernet cable's lights on the
Pi, then power the Pi off and on again.

### 5. Switch Kilo on, then connect it

- Switch Kilo on (way B and way A).
- Way A: now plug Kilo's USB cable into a Pi USB port.

### 6. Send the kernel (on the laptop)

```
kernel/tools/netboot.sh 192.168.88.X
```

with the address from step 4. It ends in a second or two. The Pi then runs
the mind kernel.

To send a kernel again later (a new build), you need the loader back.
Once the mind kernel runs and has an address (step 8), on the laptop:

```
~/OpenCat/.venv/bin/kilo reboot --host 192.168.88.Y
```

It prints `REBOOT: OK (rebooting)`, Kilo lies down (REST), and the Pi
restarts into the loader (the ACT light goes dark again, as in step 3).
Then run `kernel/tools/netboot.sh` again with the loader's address (step 4;
it is usually the same). Powering the Pi off and on does the same.

### 7. Watch the ACT LED

| What the green light does | What it means |
|---|---|
| Steady on, briefly (way A: a second or a few) | The kernel is starting (way A: it is looking for USB devices first). |
| Slow blink: 1 s on, 1 s off | The kernel runs but hears no Kilo yet. Normal for a few seconds: with the USB cable, the kernel opening the adapter may restart Kilo's board, which then takes a second or two to start. |
| The blink stops for about a second, once, soon after the start | Way B: the kernel is looking for USB devices (the network chip) now that Kilo answered. Normal. |
| Even blink, twice a second | **Success**: Kilo's board is talking to the Pi. |
| Fast blink, five times a second | Kilo's board runs firmware with another protocol version. The Pi keeps the link alive but will not drive Kilo. Flash the body firmware (step 12, or the way you do today, from Linux). |
| Two short flashes, then a pause, once a second | The Pi is flashing Kilo's board (step 12, or a `body.bin` on the card). Leave it alone until the pattern changes, about 10 to 20 s. |
| Steady on or steady off for more than 20 s | The kernel stopped (it could not start, or it crashed). |

Leave it running for a minute. The light should keep its even blink.

### 8. Talk to the mind from the laptop

The kernel now asks the router for an address of its own, under the name
**kilo** (not `kilo-netboot`: that was the loader). It may get the same
address as the loader, or another one.

**Find the address.** First try the name: the kernel answers to
**kilo.local** (that is mDNS: the laptop asks the network "who is
kilo.local?" and the Pi answers). On the laptop:

```
kilo find
```

It prints a line like `kilo.local  192.168.88.Y  port 5300  build xxxxxxxx`:
the address is in it, and you can use `--host kilo.local` instead of the
address everywhere below (the `kilo` command looks the name up itself; the
laptop needs no avahi). If it says "no answer for kilo.local", the Pi's
mDNS did not answer (or the laptop is on another network): note that, and
use the router instead.

Otherwise open the router's list of devices again (step 4) and
look for **kilo**, or for the same `b8:27:eb` hardware address. Write the
address down; below it is `192.168.88.Y`. If the list shows nothing new,
try the loader's address from step 4: routers often give the same machine
the same address.

(Without a router, with the production card and the Pi's cable straight
into the laptop: the kernel waits 3 s for a router, then takes
**169.254.75.73**. The laptop's wired port must then be set to link-local:
in NetworkManager, the wired connection's IPv4 method "Link-Local Only".
Use `169.254.75.73` as the address below. The loader cannot do this, so the
bench card needs the router.)

**Ask for its status**, on the laptop:

```
~/OpenCat/.venv/bin/kilo status --host 192.168.88.Y
```

(It uses this laptop's key from step 1. If it says `no host key`, step 1's
`kilo key init` was not run on this laptop; if it says `the mind does not
know this laptop's key`, the kernel was built without this laptop's line:
add it, rebuild, and send the kernel again, step 6.)

A good answer looks like this (your numbers will differ):

```
mind        build 3d441ec0  up 42.7 s  host channel v2  clients 1  controller NONE
link        connected=True alive=True contact=True
firmware    proto 2.0  build 1a2b3c4d  reset COLD  config OK
mode        REST  epoch 1  flags -  age 12 ms
attitude    pitch   +0.50°  roll   -1.25°
battery     7900 mV
distance    - mm
body        reboots 0  telemetry frames 1060
link stats  frames_rx=1100 frames_tx=60 heartbeats=40
```

What matters:

- `mind build`: the first 8 characters of the commit the kernel was built
  from (the same number the screen shows in step 10).
- `alive=True`: Kilo's board is talking to the Pi.
- `firmware ... build`: the body firmware's build number.
- `mode` and `battery` are filled in: the Pi gets Kilo's reports.

If it cannot connect, see "When it goes wrong" below.

**Read the mind's log** (it prints everything the mind has logged since it
started, then keeps printing new lines, and stops after 20 s; `--save`
also keeps a copy in `~/kilo-logs`):

```
~/OpenCat/.venv/bin/kilo log --host 192.168.88.Y --duration 20 --save
```

When the laptop first connects, the mind times its walking planner once,
for about 5 seconds, without moving Kilo. Copy into the results:

- the three lines starting `planner bench` (trot, walk, crawl): how long
  the walking planner takes on this Pi, and the line starting `noise
  bench`: how long the key handshake's three steps take on this Pi;
- the line starting `noise: client` (one per connection): this laptop's
  key, and how long its handshake took. The mind keeps its last 4096 log
  lines of what happened (remote keys, walks, tricks, warnings); the
  once-a-second counter lines are shown live but never pushed the others
  out. If the first line says the mind dropped older lines, it has been up
  a long time: `kilo record fetch` (step 14) still has them;
- one line starting `loop`: how regularly the mind's loop runs while the
  network is in use;
- if you see it, the line `listening on ...`.

**Make Kilo stand for 5 seconds.** Put Kilo on the floor, on its feet or
lying down, with room around it. Then:

```
~/OpenCat/.venv/bin/kilo posture stand --host 192.168.88.Y --for 5
```

Kilo should stand up, hold for 5 seconds, then lie down again (REST). The
laptop prints `holding stand for 5 s`, then `released: the mind latched
REST`. If Kilo does something else, switch it off and note what it did.

### 9. The remote

**Kilo must be on the floor for this step, never on the desk**: the desk
slopes and there is no stand to hold him.

First start the log on the laptop, so you see the mind's lines as you
press keys (it stops by itself after 2 minutes):

```
~/OpenCat/.venv/bin/kilo log --host 192.168.88.Y --lines 5 --duration 120
```

Then point the remote at Kilo and press one key at a time, waiting for
Kilo to finish moving before the next:

1. **B6** (column B, row 6). Kilo stands up. The log shows
   `remote B6 (0x02, stand): posture stand`.
2. **A7** (column A, top row). Kilo lies down. The log shows
   `remote A7 (0xA2, sleep): posture rest`.
3. Get ready to catch Kilo. Press **A4** (column A, row 4): Kilo trots
   forward, fast (about 30 cm in two seconds). The log shows
   `remote A4 (0x68, trot): walk forward (trot) 144.0 mm/s`.
4. After about two seconds, press **B6**. Kilo stops and stands. The log
   shows `remote B6 (0x02, stand): posture stand`.
5. Press **A7** to let him lie down again.

What the other keys do, for later: **B7** faster and **B5** slower while
walking (B5 past the slowest walks backward), **A6** and **C6** turn left and
right on the spot when Kilo stands, and steer left and right while he walks
(each press bends the path a little more, up to three presses; the other
key bends it back), **B7** or **B5** while he turns on the spot sets off
straight forward or backward, **B4** crawl and **C4** walk (the two slower
gaits), **C5** the calibration posture, **A5** Kilo's own behaviour (B6 or
A7 ends it), and **C7** freeze: Kilo stops where he is, feet on the ground,
and ignores every key except B6 and A7. The keys in rows 1 to 3 do nothing
(A2, the rear-up trick, only when it is switched on: docs/rear-up-trick.md).

If a key does nothing and no `remote` line appears, the key did not reach
the mind: note which key. If the line says `ignored (a laptop client holds
the body)`, a `kilo posture` command is still running: stop it (Ctrl-C)
and press the key again. Copy the `remote` lines into the results, and
anything Kilo did that you did not expect.

### 10. Optional: watch with a screen

With an HDMI monitor plugged in **before** step 3, the kernel prints its log
on the screen: the build number, which serial connection it chose
(`utty1 (USB serial)` for way A, `ttyS1 (PL011, GPIO14/15)` for way B),
`first HELLO ... s after the kernel started`, and once a second a line of
link counts (`telemetry 25/s`, decode errors). This may not work: the card
gives the graphics chip very little memory (`gpu_mem=16`). Without a
screen, the light is all there is to see in this phase.

### 11. Optional: time the boot with a scope

The kernel raises Pi header pin 40 (GPIO21) the moment it starts and drops
it when the first HELLO arrives. Pin 39 next to it is ground. With a scope
on pin 40 and the Pi's 5 V, from a cold power-on with the production card
(`make-sd.sh ... --kernel` instead of `--netboot`):

- power-on to the rising edge: the Pi's own start-up files plus Circle,
- rising to falling edge: our start-up plus Kilo's board starting (way A:
  the USB scan included; way B scans USB after the first HELLO).

### 12. Optional: flash Kilo's board from the laptop, through the Pi

This rewrites Kilo's board firmware with the build on the laptop, the way
`kilo flash` does today from Linux, but done by the Pi's new kernel. If it
fails, the board keeps working with the firmware it had or, at worst, waits
in optiboot until flashed again (optiboot itself is never overwritten).

**Wiring.** The Pi must be able to pull Kilo's reset line:

- With the Pi on the NyBoard's socket (way B, the expected wiring), pin 7
  already lands on the reset line (docs/raspberry-pi.md). Nothing to add.
- Way B with the three header wires: add a fourth wire, Pi pin 7 (GPIO4:
  the inner row, fourth pin counting pin 1 as the first) to the NyBoard's
  reset line: the pin marked DTR on the 6-pin header where the USB adapter
  plugs in. Check the marking on the board first; if there is none, stop
  here and note it.
- Way A (USB cable) does **not** work for this: on the cable, the reset
  line is the adapter's DTR, and the kernel cannot pulse it. The kernel
  will try for about 3 s and report `SYNC_TIMEOUT`; Kilo keeps its old
  firmware.

**Flash.** With Kilo on and the even blink (step 7), on the laptop:

```
~/OpenCat/.venv/bin/kilo flash --host 192.168.88.Y
```

It first builds the firmware on the laptop (a few minutes the first time),
then sends it. While the Pi flashes, the ACT light gives **two short
flashes, then a pause**, once a second; the laptop prints progress lines
starting `flash:`. The last line should be like:

```
verified: HELLO build 3d441ec0 reset FLASHED
```

and the light should go back to the even blink.

**Read the log** straight after:

```
~/OpenCat/.venv/bin/kilo log --host 192.168.88.Y --lines 64 --no-follow
```

Copy into the results the lines starting with:

- `flash: optiboot in sync ... ms after release`: how long optiboot took to
  answer after the reset;
- `flash: programmed`, `flash: verified` and `flash: done in ... ms`;
- `body flashed from the laptop` (or `body flash from the laptop FAILED`).

**Optional, keep it on the card.** Run the same command with `--persist`.
Then the log should also show `SD: --persist: wrote body.bin ...`. Power
the Pi off and on and, after the even blink, look in the log for `boot
flash check: the body runs build ..., the card's: no flash`: at every boot
the Pi compares the card's firmware with Kilo's and flashes only when they
differ.

If it fails: note the laptop's last line and the log lines above. Power
Kilo off and on; if the light then never gets to the even blink, Kilo's
board is waiting in optiboot: flash it the way you do today, from Linux.

### 13. Optional: start the mind from the card, with no network

So far the card holds the loader, and the mind runs only after the laptop
sends it (step 6). This step puts the mind itself on the card, over the
network cable: after it, the Pi starts the mind at every power-on with no
laptop and no network. Nothing needs the card reader unless it goes wrong.

**Kernel update**: the mind writes a new `kernel7.img` onto its own card.
It writes the new one as `kernel7.new`, reads it back to check it, and
only then renames the old `kernel7.img` to `kernel7.old` and the new one to
`kernel7.img`. Then it restarts into it.

With the mind running from step 6 and its address from step 8, on the
laptop:

```
~/OpenCat/.venv/bin/kilo update --host 192.168.88.Y
```

It sends `kernel/build/kernel7.img` (the build from step 1). It first
checks that this laptop's key is in that image (so that the new kernel will
still let this laptop in) and stops if it is not. While the Pi
writes the card, the ACT light gives two short flashes, then a pause (as
when flashing Kilo's board) and Kilo lies down. The laptop prints lines
starting `update:`, then

```
update: DONE: kernel7.img on the SD card is the new kernel (... bytes, crc32 ... read back); the previous one is kernel7.old
update: the mind is rebooting into it
update: the mind is back after ... s: mind build ...
```

The Pi restarts by itself, this time straight into the mind (no loader, no
`netboot.sh`). Copy those three lines into the results. Power the Pi off
and on once and check that the light comes to the even blink again (step
7) with no laptop involved; `kilo status --host 192.168.88.Y` should show
the same `mind build`. From now on a new build goes on the card the same
way: `make -C kernel`, then `kilo update --host 192.168.88.Y`.

**Back to the loader** (to try builds from RAM again, step 6):

```
~/OpenCat/.venv/bin/kilo update --host 192.168.88.Y --image kernel/build/netboot/kernel7.img --force
```

`--force` is needed because the loader is not the mind: without it the
mind refuses any file that is not a mind kernel (`NOT_A_KERNEL`), so a
wrong file cannot end up on the card by mistake.

**If it fails**, the laptop's last line says why. In every case but one the
card still has its old `kernel7.img` and boots as before: note the line and
the log (`kilo log --host 192.168.88.Y --lines 64 --no-follow`, the lines
starting `update:` and `SD:`). The exception is a line with `CARD_BROKEN`,
or a Pi that does not come back (the light steady for more than 20 s
after the restart): then put the card in a card reader and copy the files
of `kernel/build/card/netboot/` (the loader) or `kernel/build/card/kernel/`
(the mind), made by `make -C kernel card` (step 1), onto the card's FAT
partition, replacing what is there. `kernel7.old`, if the card has one, is
the kernel from before the update: renaming it to `kernel7.img` also works.

### 14. Optional: after a walk or a trick: plug the cable in, fetch, analyse

The mind records everything while it runs (the *flight recorder*), so you
can play with Kilo with the remote and no cable, then look at what
happened. Words used:

- **Run**: one walk (from a gait key to its stop) or one trick (from A2 to
  its end), a stretch of the recording. Each has a number.
- **Recording**: what the mind wrote down from the moment it started:
  every message between the Pi and Kilo's board (each 20 ms tick: where
  the plan put each foot, the tilt the plan expected, the tilt the board
  measured, the correction its own balance loop added, the battery), every
  remote key and what it did, every log line. Several hours of it in the
  Pi's memory; the oldest goes first when it is full.
- **Predicted** tilt: what the plan (or the trick's design) expected the
  body's pitch and roll to be at that tick. **Measured**: what Kilo's IMU
  (the tilt sensor) said. The **error** is measured minus predicted.

**Switching Kilo off loses what was not saved.** Saving to the SD card is
off until the card has been measured (point 6 below); until then plug the
cable in before you switch Kilo off.

1. Kilo on the floor, the cable unplugged. Walk him with the remote as in
   step 9 (A4 trot, C4 walk or B4 crawl, then B6 to stop), or play a trick.
2. With Kilo still on, plug the network cable in. `kilo find` prints his
   address (on a direct cable it is `169.254.75.73`); `kilo.local` works in
   every command below.
3. List the runs:

   ```
   ~/OpenCat/.venv/bin/kilo runs --host kilo.local
   ```

   One line per run: its number, when it started (the laptop's clock), a
   walk's gait, direction and speed or a trick's name and stage, how long
   it lasted and how it ended (`stopped` is a walk's normal end, `DONE` a
   trick's). `kilo walks` is the same command.
4. Download them:

   ```
   ~/OpenCat/.venv/bin/kilo runs --host kilo.local --fetch all
   ```

   For each run it writes `~/kilo-logs/run-<date>-<time>-<kind>-<number>.krec`
   (the recording from 2 s before to 2 s after; a walk also gets a
   `.kilolog` beside it) and prints its summary. A run still going is
   skipped: fetch it again after it ends. `--fetch last` takes the newest.
5. Look at one again later, and save its tick-by-tick table:

   ```
   ~/OpenCat/.venv/bin/kilo log walk ~/kilo-logs/run-....krec --csv walk.csv --plot walk.png
   ~/OpenCat/.venv/bin/kilo log trick ~/kilo-logs/run-....krec --csv trick.csv
   ```

   What a walk's summary says: the **error** per axis (`rms`: the typical
   size; `max`: the worst tick; `offset`: the average, a constant tilt
   such as a sensor that is not mounted level), the **lag** (how much later
   than the plan the body tilts), the error **by phase** (which legs were
   in the air), how long the board **held** its corrections, and the
   smallest **margin** (how far inside its feet the body's weight stayed,
   in mm). A trick's: see `docs/rear-up-trick.md` section 6.
6. Optional, once: measure the SD card and turn saving on. Kilo limp
   (`kilo posture rest --host kilo.local`, or the servos off), then:

   ```
   ~/OpenCat/.venv/bin/kilo bench sd --host kilo.local
   ```

   It prints, for writes of 512 B, 4 KB and 16 KB, the shortest, mean and
   longest time, and the time of the sync, close and delete. Copy the table
   into the results. If the longest 4 KB write is well under 100 ms:

   ```
   ~/OpenCat/.venv/bin/kilo record save on --host kilo.local
   ```

   From then on (it is remembered on the card) the mind writes the
   recording to the card after each run, while Kilo stands still, and a
   later boot lists the earlier boots' runs as `b<boot>:<run>`. `kilo record
   status --host kilo.local` says what a power cut would lose now.

Copy the `kilo runs` list and each run's summary into the results, and keep
the `.krec` files. If `kilo runs` says the mind does not record, the Pi runs
an older kernel: update it (step 13).

### 15. Finish

Power the Pi off. Way A: unplug the USB cable from the Pi before you switch
Kilo off.

## When it goes wrong

| What you see | What to do |
|---|---|
| `netboot.sh` says it timed out | Wrong address, or the loader is not running. Check step 4 again; power-cycle the Pi; try `kernel/tools/netboot.sh --http 192.168.88.X`. |
| `netboot.sh` worked but the light stays dark | The loader may not hand over on this board. Write the kernel straight to the card instead: `kernel/tools/make-sd.sh /media/$USER/BOOT --kernel` (on a card with only Pi boot files), and power on with it. Note it in the results. |
| Slow blink that never turns into the even blink | Is Kilo switched on? Way B: is the Pi seated on the socket the right way round (or, with wires: are TX and RX crossed, Pi transmit to NyBoard receive, and is ground connected)? Is the USB adapter unplugged from Kilo? Way A: was the kernel built with `KILO_BODY_SERIAL=auto` (step 1; the default kernel ignores the cable)? Is the USB cable in the Pi, and was it plugged in **before** step 6? (The kernel looks for USB devices only once, when it starts.) Then power-cycle the Pi and start from step 3. |
| Even blink, then slow blink | Kilo stopped talking (switched off, battery cut out, or the cable moved). |
| Fast blink | See step 7: flash the body firmware. |
| The Pi resets when Kilo's cable goes in | The 5 V problem above. Note it; Kilo must be on before the cable goes in. |
| Steady light for more than 20 s | Note whether it was on or off, and whether a screen showed anything. Power-cycle and try once more. |
| No **kilo** in the router's list | Wait a minute and look again, and try the loader's address anyway. If nothing answers, note it: the kernel's network did not come up. |
| `kilo status` says `no host key` | Step 1's `kilo key init` was not run on this laptop (or `--key`/`KILO_HOST_KEY` points elsewhere). |
| `kilo status` says `the mind does not know this laptop's key` | The kernel was built without this laptop's line in `kernel/keys/authorized_keys`: add it (`kilo key show >> kernel/keys/authorized_keys`), `make -C kernel`, send it again (step 6), or for the card in step 13 use a laptop the mind knows, or the card reader. |
| `kilo status` says `can be read by others` | The key file's permissions: `chmod 600 ~/.config/kilo/host_key`. |
| `kilo status` cannot connect, or waits and gives up | Check the address. Does the Pi answer at all? `ping 192.168.88.Y` (Ctrl-C stops it). Note what `ping` says. |
| `kilo status` works but shows `alive=False` | The network is fine; Kilo's board is not talking to the Pi (see the slow-blink row above). |
| `kilo posture` says `NO_BODY` or `NOT_CONTROLLER` | NO_BODY: the same as `alive=False`. NOT_CONTROLLER: another `kilo` command still holds Kilo; stop it (Ctrl-C) and try again. |

To try again with a fresh kernel: `kilo reboot --host 192.168.88.Y` (step 6)
or power the Pi off and on (either way the loader starts again), find it
(step 4; the address usually stays the same) and send the kernel (step 6).
