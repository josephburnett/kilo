# Fast boot: first boot results

Runs of docs/fast-boot-first-boot.md. Newest first. Copy the block below
for each run and fill in what you saw; "don't know" is a fine answer.

## 2026-11-22, boot 16: the first floor run of everything merged that day

Build 7e92703c (mind and body), kernel with `KILO_REAR_UP=1
KILO_RIGHTING=1`, from the SD card. Read from the mind's log
(`~/kilo-logs/kilo-card-b16.log`) and the whole-boot recording
(`rec-card-b16.krec`); what the owner saw has not been written down yet.
Times are seconds after the Pi started. The walk and the trot are in
walk-trial-results.md, the righting tries in righting-results.md, the
random mode's tricks in movement-engine-results.md §7.

**The body flash said FAILED, but the body was flashed correctly.** At
1.6 s the card flash found the body on 4eb35a20 and the card holding
7e92703c (27536 bytes). It programmed and verified all 216 pages (7.6 s;
every page read back equal), the body restarted with HELLO build 7e92703c,
and then the flash reported `FAILED: BAD_RESET_CAUSE` (red flashes and a
hiss), because the body said its restart was `COLD` (a power-on) instead
of `FLASHED`. The cause is in the body firmware, not the flash: the body
tells a flash from a power-on by a 10-byte record it keeps in RAM across
restarts, and that record sat right after the firmware's other variables,
so its address moved whenever they changed size. 7e92703c has 2 bytes
more of initialised variables than 4eb35a20 (`.data` 44 B against 42), so
it looked for the record at 0x6F9 where the old firmware had left it at
0x6F7, found no valid record and called it a power-on. Proof from the two
builds: the new firmware's epoch byte lands at 0x702, just past the old
firmware's variables, in the free RAM it paints with 0xC5 (197) at every
start; the epoch it reported was 198, that byte + 1. Earlier card flashes
(boots 13 to 15) reported FLASHED because their builds happened to have
the same size of variables. The body itself was healthy: right build,
verified image, calibration and settings read as OK ("config OK"), and
the stack never came closer than 157 bytes to the variables (STATS
`stack_free_min`; the static estimate says 144 B at worst). Fixed the
same day: the record now sits at a fixed address at the top of RAM
(docs/architecture.md §3a), checked by `body/tools/size_check.sh`. The
first flash onto the fixed firmware will report BAD_RESET_CAUSE once more
(the old firmware left its record at the old place); every flash after it
should report FLASHED.

**Watchdog 2.** Both were the body noticing the Pi silent for 0.5 s while
the Pi's main loop waited: at 1.6 s on the SD card (1.5 s) and at 12.2 s
on the USB scan for Ethernet (1.7 s). Both while he lay limp. Every boot
since boot 13 has had one or two of these; nothing moved.

**Falls 10, recovered 0.** One fall: at 110.5 s, standing still, he turned
onto his left side (probably laid there by hand: righting-results.md).
The other nine are the body's fall check finding him still on his back
each time the legs were given a pose again during the righting tries (and
after A7); with FALL_RC 0 it never tries its own get-up, so none can be
"recovered". Nothing fell during the walks or the trot.

**The eye signals** (the log's `signal` lines, for comparing with what
was seen):

| time | signal | what the eyes should have shown | why |
|---|---|---|---|
| 0.02 s | ready | 3 quick green flashes (and a mew) | the body answered the Pi |
| 1.61 s | flashing | steady white | the card flash started |
| 10.57 s | failed | 3 quick red flashes (and a hiss) | the flash's false BAD_RESET_CAUSE |
| 24.55 s | guard_red | steady red (and a hiss) | something 5.6 cm from his nose as he stood up; he backed off |
| 110.37 s | saved | 1 long white flash | a recording saved to the card |
| 145.82 s | gave_up | 3 quick red flashes (and a meow) | the first B1: three tries, he stayed on his back |
| 158.91 s | gave_up | 3 quick red flashes (and a meow) | the second B1, the same |

Without a `signal` line: blinking white (busy) during each B1 (133.9 to
145.8 s and 147.0 to 158.9 s), and steady red from the distance guard at
89.1 to 91.0 s during sniff (something 6 to 8 cm from his nose).

## 2026-11-16, run 5: the diagonal walk and the faster trot on the floor

Build af5f8920 (rear-up stage 1 on B2) installed with `kilo update --host 192.168.88.8` (back after 10.7 s). The owner ran Kilo by remote with the cable out, then plugged it in with Kilo still on (address now 192.168.88.9); `kilo walks --fetch all` brought eight recordings. Files: `~/kilo-logs/walks/walk-20261116-20*.kwalk` on rtb.

| | Walk 1 (11 s) | Walk 3 (22 s) | Walk 5 (8 s) | Walk 8 (7 s) |
|---|---|---|---|---|
| Gaits | trot 144 -> 165.6 -> 180 mm/s | crawl 22.2 -> 25.6, then walk 92 -> 96.8 | trot 144 -> 165.6 -> 180 | walk 80 |
| End | stopped; 0 holds, 0 catches, 0 waits | stopped; 0 / 0, 47 waits (the crawl) | stopped; 0 / 0 / 0 | stopped; 0 / 0 / 0 |
| Pitch, measured - predicted | offset +1.83, rms 0.62 less the offset, max +3.75 | offset +1.95, rms 0.43, max +3.31 | offset +1.73, rms 0.61, max +2.75 | offset +1.56, rms 0.49, max +2.81 |
| Roll, measured - predicted | rms 1.20 less the offset, max +3.75 | rms 1.49, max -4.69 | rms 1.19, max +3.50 | rms 1.39, max +2.75 |
| Body's correction, largest | pitch -1.56, roll -0.69 | pitch -1.56, roll -2.12 | pitch -1.12, roll -0.31 | pitch -0.56, roll -0.44 |

The change from the crawl to the walk in walk 3 took 9.1 s (asked at 9.12 s, walking at 18.22 s). Servo lag from the crawl's roll: 120 ms (r 0.976), the fitted model's mean. The other four recordings are short turns and a 100.8 mm/s trot, all stopped with 0 holds and 0 catches.

Not known: what the rear-up trick did. Trick runs are not recorded, and the log ring no longer held the session's lines when the cable went in.

## 2026-11-15, run 4: the recorder and the faster gaits on the floor

Build 457e180f installed with `kilo update --host` (back after 15.2 s). The owner walked Kilo by remote with the cable out, plugged it back in with Kilo still on; `kilo walks --host 192.168.88.28 --fetch all` brought three recordings (0.44 of 7.96 MB used), analysed with `kilo log walk`. Files: `~/kilo-logs/walks/walk-20261115-2135*.kwalk` on rtb.

| | Walk 1 (50 s) | Walk 2 (10 s) | Walk 3 (36 s) |
|---|---|---|---|
| Gaits | crawl 22.2 -> 26.7 mm/s, walk 21.3, trot for the last ~10 s | trot, turning left, 144 mm/s | walk 17.8 -> 21.3 mm/s, then crawl |
| End | stopped; 0 skipped, 0 underruns, 0 holds, 0 catches, 72 waits | stopped; 0 / 0 / 0 / 0, 0 waits | stopped; 0 / 0 / 0 / 0, 53 waits |
| Pitch, measured - predicted | offset +1.68 deg, rms 0.74 less the offset, max +4.31 | offset +1.70, rms 0.75, max +3.88 | offset +2.00, rms 0.39, max +4.25 |
| Roll, measured - predicted | offset +0.54, rms 2.51, max +6.06 | offset +0.25, rms 2.09, max +5.19 | offset +0.06, rms 2.90, max -8.56 |
| Servo lag from the roll (model 150 ms) | 140 ms (r 0.953) | - | 140 ms (r 0.961) |
| Body's correction, largest | pitch -2.06, roll -5.5 deg | pitch -2.5, roll 2.5 | pitch -1.44, roll -4.88 |
| Smallest margin with one leg up | 12.4 mm (as planned) | - | 12.4 mm (as planned) |

Roll error by the leg in the air (walks 1 and 3): LB +2.4 to +2.6 deg, RB -2.7 to -2.9, LF +2.1 to +2.7, RF -0.5 to -1.1: the body rolls toward a lifted rear leg by about 2.7 deg more than planned.

The owner: "Walking is still really slow."

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
| Remote: A4 trotted forward? B6 after two seconds stopped him and he stood? | |
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
