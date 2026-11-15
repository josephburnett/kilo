# Fast boot: first boot results

Runs of docs/fast-boot-first-boot.md. Newest first. Copy the block below
for each run and fill in what you saw; "don't know" is a fine answer.

## 2026-11-15, run 3

Kilo assembled, on battery, LAN cable in. The card still held the netboot loader; the Pi came up at 192.168.88.28.

| Step | Result |
|---|---|
| Kernel with keys (host channel v2, build 899fe080) sent with `netboot.sh --http` | `kilo status --host` answered 11 s after the send: link alive, battery 8035 mV, distance 515 mm |
| Handshake, in Kilo's log | `handshake with key 1d104bb805e1b051 (joe@rtb) in 9.9 ms, slowest step 2622 us`; loop `work max 79 us` afterwards, PL011 errors not growing |
| A key Kilo does not know | Kilo logged `key ... is not authorized` and `refused: an unknown key`. The laptop printed "closed the connection without answering the hello": the refusal's 8 bytes did not arrive before the close (open, cosmetic) |
| `kilo update --host 192.168.88.28` (build c527a97d, 591140 bytes) | Written, read back with crc32 02363240, installed (`kernel7.img` is the mind kernel, `kernel7.old` the loader), rebooted: "the mind is back after 10.7 s: mind build c527a97d" |
| After the reboot | Status from build c527a97d; the loader's port 8080 no longer answers: Kilo runs from the card |
| Power cycle by the owner, nothing sent from the laptop | `kilo status --host 192.168.88.28` answered: build c527a97d, up 20.3 s when asked, link alive, battery 7984 mV, distance 508 mm. Same address as before the power cycle |

## 2026-11-14, run 2

Kilo reassembled by the owner: the Pi on the NyBoard's socket, all servos
and the ultrasonic eyes plugged in. Run from rtb over the network.

| Field | What was seen |
|---|---|
| Loader address | 192.168.88.2 this time (DHCP gave another address than run 1's .9; same MAC) |
| `kilo status --host` | `mind build a89f1dcf`; `firmware proto 2.0 build de6d2d7f reset COLD config OK`; `mode LIMP`; `attitude pitch +3.38° roll +4.94°`; `battery 7983 mV`; `distance 364 mm` (the eyes are on the D6/D7 socket, G4) |
| Servo plugs | the owner checked them against the port table (docs/architecture.md, joint table) |
| `kilo posture stand --for 5`, Kilo held in the owner's hand | laptop: `holding stand for 5 s`, `released: the mind latched REST`; the owner: "Yeah, that was good!" (asked: legs straightened evenly, head forward, back to rest). Battery 7840 mV after |
| Remote, walk, planner bench, boot probe | not run |

## 2026-11-14, run 1

First run on the Pi. Filled in from rtb over the network; the owner's own
observations (LEDs, screen) were not asked for and are marked so.

| Field | What was seen |
|---|---|
| Date and time | 2026-11-14, about 11:15 to 11:35 PST |
| Kernel build | 98a08f66, then the same commit plus the screen-log fix below |
| SD card | the only card (was Raspberry Pi OS): its FAT boot partition erased, the six `--netboot` files copied on rtb through a USB reader, checksums verified |
| Ethernet cable | to the house router |
| Kilo connection | B: the Pi on the NyBoard's socket (riser soldered by the owner); NyBoard out of Kilo, no servos connected; I2C switch on Arduino; the Pi powered from the battery through the socket |
| Loader address and name | 192.168.88.9 (MAC b8:27:eb:49:21:8e), found by a ping sweep; the page on :8080 says "Bare metal Bootloader", Circle 51.1.1 |
| `netboot.sh --http` output | `sent 512164 bytes to http://192.168.88.9:8080/, the loader is booting it` |
| ACT LED | not asked |
| HELLO seen? | yes: `link connected=True alive=True contact=True` |
| Kernel's address | 192.168.88.9 (DHCP, the loader's address again) |
| `kilo status --host` | `mind build 98a08f66 up 9.9 s`; `firmware proto 2.0 build 467b69b1 reset COLD config OK`; `mode LIMP`; `battery 8044 mV`; telemetry flowing |
| One `loop` line | `loop 889/s: period max 1127 mean 1125 us, work max 55 mean 7 us, mind.poll max 53 mean 5 us` (after the fix) |
| Link | PL011 at 250000 through the socket: telemetry 25/s; after the fix 1 CRC error in 3900 frames (at start-up), none after |
| `kilo reboot --host` | works: `REBOOT: OK (rebooting)`, the loader's page is back 13 s later, the mind's port 5300 answers about 14 s after the kernel is sent (DHCP included) |
| Flash (step 12): reset wiring | the socket (pin 7, GPIO4) |
| `kilo flash --host 192.168.88.9` | `flash: programming done, 214 pages`, `flash: verifying done, 214 pages`, `flash: done; HELLO build de6d2d7f reset FLASHED`, `flash ended in 9.0 s`, `verified: HELLO build de6d2d7f reset FLASHED` (the board ran 467b69b1 before) |
| After the flash | `firmware proto 2.0 build de6d2d7f reset FLASHED config OK`; telemetry 25/s; decode errors 2 in 6700 frames. Mode RECOVERING with SERVOS_POWERED: the bare board lies upside down (pitch 179°), so the body's fall recovery runs, with no servos connected |
| Battery | 8044 mV at the first status, 7855 mV 25 minutes later (Pi and board, no servos) |
| `planner bench`, posture, remote, boot probe, `--persist` | not run in this run |

**Found and fixed: one lost frame a second.** The first kernel showed
`PL011 line errors: ... overrun N`, N growing by one a second, with one CRC
error each, and `loop ... work max 21329 us`. The four stats lines went to
the HDMI screen once a second; a line on the screen holds interrupts off
for milliseconds, and the PL011's 16-byte FIFO lasts 0.64 ms at 250000
baud. Now only warnings and errors reach the screen once the main loop
runs (the ring, `kilo log --host`, keeps every line), and the PL011
warning is a warning only when its count grew. After: overruns stay at the
one from start-up, `work max 55 us`.

## YYYY-MM-DD, run N

| Field | What you saw |
|---|---|
| Date and time | |
| Kernel build (commit, or the number on the screen) | |
| SD card (which card; `--netboot` or `--kernel`) | |
| Ethernet cable (to the router, or other) | |
| Kilo connection (A: USB cable, or B: header wires 8/10/6) | |
| Kilo switched on before the cable went in? | |
| Did the Pi reset when the cable went in? | |
| Loader address (from the router) and name shown | |
| `netboot.sh` output (last line) | |
| ACT LED pattern, first 10 s | |
| ACT LED pattern after 1 minute | |
| HELLO seen? (even 2 Hz blink, or the screen line) | |
| Kernel's address (router name **kilo**, or 169.254.75.73 on a direct cable) | |
| `ping kilo.local`: did it answer? With which address? | |
| `kilo status --host` output (paste it; or the error) | |
| `mind build` in the status: matches the commit? | |
| `planner bench` lines from `kilo log --host` (three) | |
| One `loop` line from `kilo log --host` | |
| `kilo posture stand --for 5`: did Kilo stand, hold, then lie down? What did the laptop print? | |
| Remote (step 9), Kilo on the floor: B6 stood up? A7 lay down? | |
| Remote: B7 walked forward? B6 after two seconds stopped him and he stood? | |
| Remote: the `remote ...` lines from `kilo log --host` | |
| Screen used? What it showed (serial chosen, first HELLO time, telemetry/s, decode errors) | |
| Boot probe (scope on pin 40): power-on to rising edge, rising to falling edge | |
| Flash (step 12): reset wiring (socket, fourth wire to DTR, or skipped) | |
| `kilo flash --host`: the last line it printed | |
| ACT LED while flashing (two short flashes a second?) and after | |
| Log: `flash: optiboot in sync ... ms after release` line | |
| Log: `flash: programmed` / `verified` / `done in ... ms` lines | |
| Log: `body flashed from the laptop` (or `FAILED`) line | |
| `--persist`: the `SD: --persist: wrote ...` line; after a power cycle, the `boot flash check` line | |
| Anything else | |
