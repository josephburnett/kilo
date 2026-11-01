# Fast boot: first boot of the Circle kernel on the Pi

What you need to do to start the new mind kernel on the Pi 2 B for the
first time, and what you should see. It is the hardware check for phases
P1 to P4 of docs/fast-boot.md: the Pi says hello to Kilo's board and
keeps the link alive, with no Linux, the laptop talks to the Pi over the
network cable (`kilo status`, `kilo log`, `kilo posture`), Kilo answers
the IR remote (step 9), and, as an optional last step, the Pi rewrites
Kilo's board firmware itself (`kilo flash --host`, step 12).

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
  - **A. The USB cable** (today's wiring): Kilo's USB adapter cable into one
    of the Pi's USB ports.
  - **B. Three wires on the Pi's pin header**, with Kilo's USB adapter
    unplugged from Kilo (the two must not be connected at the same time):
    - Pi pin 8 (GPIO14, the Pi's transmit) to the NyBoard's receive (RX),
    - Pi pin 10 (GPIO15, the Pi's receive) to the NyBoard's transmit (TX),
    - Pi pin 6 (ground) to the NyBoard's ground.

    Pin 1 has a square solder pad underneath. It is at the end of the
    header away from the USB ports, in the inner row. Odd numbers run along
    the inner row (1, 3, 5, ...), even numbers along the outer row at the
    board's edge (2, 4, 6, ...). https://pinout.xyz shows the picture.
    Never connect pins 2 or 4: they are 5 V. The NyBoard's serial on its Pi
    socket is 3.3 V (docs/raspberry-pi.md, which also says to check that
    with a meter first).

## Before you start: power

The USB adapter cable joins Kilo's 5 V and the Pi's 5 V (see
docs/pi-outages-2026-10-13.md). With Kilo switched off, the Pi powers Kilo's
board through the cable and its supply dips; that has reset the Pi before.
So, with the USB cable (way A):

- **Switch Kilo on before the cable goes into the Pi**, and keep it on.
- If the Pi resets when you plug the cable in, that is this problem, not
  the kernel. Start again from step 3.

The three header wires (way B) carry no 5 V, so they do not have this
problem.

## Before you start: the remote

The kernel listens to the IR remote from the moment it starts, as the Pi
did under Linux. Keep the remote out of reach until step 9, so that a
stray key press does not make Kilo move on the desk.

## Steps

### 1. Build and write the card (on the laptop)

```
git submodule update --init kernel/circle
make -C kernel
make -C kernel boot
```

The last line downloads the Pi's start-up files once; it needs the internet.
Put the SD card in the laptop. It must be FAT formatted and empty, or hold
only Pi boot files. Then (replace the path with where the card is mounted):

```
kernel/tools/make-sd.sh /media/$USER/BOOT --netboot
```

The script refuses a card that holds anything else, for example a Raspberry
Pi OS card. Eject the card and put it in the Pi.

### 2. Cable the Pi

- Ethernet cable from the Pi to the router.
- Way B only: the three header wires, with Kilo's USB adapter unplugged.
- Way A: do not plug the USB cable in yet.

### 3. Power the Pi

Plug in the Pi's power supply. The red light comes on. The green ACT light
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

- Switch Kilo on (way A and way B).
- Way A: now plug Kilo's USB cable into a Pi USB port.

### 6. Send the kernel (on the laptop)

```
kernel/tools/netboot.sh 192.168.88.X
```

with the address from step 4. It ends in a second or two. The Pi then runs
the mind kernel.

### 7. Watch the ACT LED

| What the green light does | What it means |
|---|---|
| Steady on, for a second or a few | The kernel is starting; it is looking for USB devices. |
| Slow blink: 1 s on, 1 s off | The kernel runs but hears no Kilo yet. Normal for a few seconds: with the USB cable, the kernel opening the adapter may restart Kilo's board, which then takes a second or two to start. |
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
ping -c 3 kilo.local
```

If that prints replies, its address (`192.168.88.Y`) is in them, and you can
also use `kilo.local` instead of the address everywhere below. If it says
"Name or service not known", the laptop may not ask the network for
`.local` names (on Debian that needs the package `libnss-mdns`), or the
Pi's mDNS does not work: note which, and use the router instead.

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

A good answer looks like this (your numbers will differ):

```
mind        build 3d441ec0  up 42.7 s  host channel v1  clients 1  controller NONE
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

**Read the mind's log**, straight after the status command (it prints the
last lines, then keeps printing new ones, and stops after 20 s):

```
~/OpenCat/.venv/bin/kilo log --host 192.168.88.Y --lines 64 --duration 20
```

When the laptop first connects, the mind times its walking planner once,
for about 5 seconds, without moving Kilo. Copy into the results:

- the three lines starting `planner bench` (trot, walk, crawl): how long
  the walking planner takes on this Pi. The mind keeps only its last 64
  log lines, about 15 seconds' worth, so if these are missing, the status
  command was too long ago: power-cycle the Pi, send the kernel again
  (step 6) and run this log command first;
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
   `remote A7 (0xA2, rest): posture rest`.
3. Get ready to catch Kilo. Press **B7** (column B, top row): Kilo walks
   forward. The log shows `remote B7 (0x62, forward): walk forward (trot)`.
4. After about two seconds, press **B6**. Kilo stops and stands. The log
   shows `remote B6 (0x02, stand): posture stand`.
5. Press **A7** to let him lie down again.

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
- rising to falling edge: our start-up (USB scan included) plus Kilo's
  board starting.

### 12. Optional: flash Kilo's board from the laptop, through the Pi

This rewrites Kilo's board firmware with the build on the laptop, the way
`kilo flash` does today from Linux, but done by the Pi's new kernel. If it
fails, the board keeps working with the firmware it had or, at worst, waits
in optiboot until flashed again (optiboot itself is never overwritten).

**Wiring.** The Pi must be able to pull Kilo's reset line:

- With the Pi plugged into the NyBoard's socket, pin 7 already lands on the
  reset line (docs/raspberry-pi.md). Nothing to add.
- Way B (header wires): add a fourth wire, Pi pin 7 (GPIO4: the inner
  row, fourth pin counting pin 1 as the first) to the NyBoard's reset
  line: the pin marked DTR on the 6-pin header where the USB adapter plugs
  in. Check the marking on the board first; if there is none, stop here
  and note it.
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

**Read the log** straight after (the mind keeps only its last 64 lines):

```
~/OpenCat/.venv/bin/kilo log --host 192.168.88.Y --lines 64 --duration 5
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

### 13. Finish

Power the Pi off. Way A: unplug the USB cable from the Pi before you switch
Kilo off.

## When it goes wrong

| What you see | What to do |
|---|---|
| `netboot.sh` says it timed out | Wrong address, or the loader is not running. Check step 4 again; power-cycle the Pi; try `kernel/tools/netboot.sh --http 192.168.88.X`. |
| `netboot.sh` worked but the light stays dark | The loader may not hand over on this board. Write the kernel straight to the card instead: `kernel/tools/make-sd.sh /media/$USER/BOOT --kernel` (on a card with only Pi boot files), and power on with it. Note it in the results. |
| Slow blink that never turns into the even blink | Way A: is the USB cable in the Pi, and was it plugged in **before** step 6? (The kernel looks for USB devices only once, when it starts.) Is Kilo switched on? Way B: are TX and RX crossed (Pi transmit to NyBoard receive)? Is ground connected? Is the USB adapter unplugged from Kilo? Then power-cycle the Pi and start from step 3. |
| Even blink, then slow blink | Kilo stopped talking (switched off, battery cut out, or the cable moved). |
| Fast blink | See step 7: flash the body firmware. |
| The Pi resets when Kilo's cable goes in | The 5 V problem above. Note it; Kilo must be on before the cable goes in. |
| Steady light for more than 20 s | Note whether it was on or off, and whether a screen showed anything. Power-cycle and try once more. |
| No **kilo** in the router's list | Wait a minute and look again, and try the loader's address anyway. If nothing answers, note it: the kernel's network did not come up. |
| `kilo status` cannot connect, or waits and gives up | Check the address. Does the Pi answer at all? `ping 192.168.88.Y` (Ctrl-C stops it). Note what `ping` says. |
| `kilo status` works but shows `alive=False` | The network is fine; Kilo's board is not talking to the Pi (see the slow-blink row above). |
| `kilo posture` says `NO_BODY` or `NOT_CONTROLLER` | NO_BODY: the same as `alive=False`. NOT_CONTROLLER: another `kilo` command still holds Kilo; stop it (Ctrl-C) and try again. |

To try again with a fresh kernel: power the Pi off and on (the loader starts
again), find it (step 4; the address usually stays the same) and send the
kernel (step 6).
