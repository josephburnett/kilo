# Fast boot: the mind as a Circle bare-metal kernel

Written 2026-11-01. A plan, not yet started. It replaces the mind's Linux +
Python stack on the Raspberry Pi 2 B with a single bare-metal C++ kernel built
on [Circle](https://github.com/rsta2/circle), cross-compiled on the laptop.
Numbers marked **(est.)** are estimates, not measurements.

## 1. Why

Measured on 2026-10-12 (docs/setup.md "The daemon as a system service, and
boot time"): kernel 7.8 s, `kilo.service` started at 22 s, first daemon log
line at 26.6 s after power, of which ~4.6 s is Python starting `kilo run`.
Whole boot 38.6 s.

The mind's job on the robot is narrow: one UART to the body, a planner that
streams `FEET` at 50 Hz, a behaviour tree at 10 Hz, and percepts from what the
body reports. Its only runtime dependency is `pyserial`. Nothing in it needs
an operating system.

Goals:

- Power to mind ready in about **1 s (est.)**, bounded by the Pi 2 B's closed
  GPU firmware (`bootcode.bin`, `start.elf`), which cannot be replaced.
- Less jitter on the 50 Hz stream: today body state reaches Python through
  the Linux tty layer, 1-10 ms **(est.)** (docs/architecture.md, latency table).
- Keep a usable development loop from the laptop over an Ethernet cable.

Non-goals: Wi-Fi (the 2 B has no radio, and a USB Wi-Fi dongle is not
practical bare-metal), on-robot builds, running the host tools on the Pi.

## 2. Decisions

| Decision | Choice | Why |
|---|---|---|
| Runtime | Circle, bare-metal, `RASPPI = 2`, AArch32 | Mature C++ framework with the drivers this needs: PL011 UART, GPIO, USB host + LAN9514 Ethernet, TCP/IP, FatFs, a cooperative scheduler. |
| Language | C++ only | Circle is C++. Rust would mean an FFI seam over Circle, two toolchains and allocator/linker glue; pure Rust has no USB host (DWC OTG) or LAN9514 driver for the 2 B. |
| Build | Laptop, `arm-none-eabi` cross toolchain | Nothing is compiled on the Pi, including the body firmware. |
| Laptop link | Ethernet cable | On the 2 B, Ethernet is the LAN9514, a USB device: it needs Circle's USB host, hub and SMSC951x drivers. Circle provides them. |
| Body link | PL011 on GPIO14/15 (`ttyAMA0` today) | Plain SoC peripheral, no USB. Same wiring as now. |
| Host tools | Stay in Python on the laptop | Simulator (`vbody`), log analysis, tests, trick importer, the `kilo` CLI. |

## 3. Architecture

```
laptop (Debian)                         Pi 2 B (Circle kernel7.img)                NyBoard (body)
---------------                         ---------------------------                --------------
kilo CLI --host  ── TCP / Ethernet ──>  net task (background)                     
  commands                                command server ─┐                      
  telemetry/log tail                      telemetry + log ┤                      
  flash --host                            flash receiver  ┤                      
  bridge                                  raw bridge      ┤                      
                                                          v                      
                                         mind core (foreground)                  
                                           link (COBS/CRC) ── PL011 UART ──────>  body firmware
                                           planner (FEET @ 50 Hz, 5 ahead)        (unchanged)
                                           behaviour tree @ 10 Hz                
                                           perception (IMU, ultrasonic, IR)      
                                           STK500v1 flasher ── GPIO4 reset ────>  optiboot
                                         SD (FAT): config, body.bin, logs
```

Two rules shape it:

1. **The control loop comes up before the network.** USB enumeration and
   Ethernet autonegotiation take on the order of a second or more **(est.)**.
   The mind starts the UART link and the planner first; the net task starts
   after and never blocks the core.
2. **The body stays the safety layer.** Its watchdog (stop at 0.5 s, REST at
   1.5 s, LIMP 10 s later, docs/architecture.md §3g) covers a hung, rebooting
   or flashing mind. Nothing in this plan changes the body's behaviour.

Circle's scheduler is cooperative: no task may block. The planner tick is
driven by a timer against a fixed deadline grid, the same pattern the body
uses (§3a).

## 4. Phases

Each phase ends with something that runs on Kilo.

### P0. Toolchain and bench loop

- Install the Arm GNU toolchain (`arm-none-eabi`) on the laptop. Clone Circle
  as a pinned submodule or vendored copy; `RASPPI = 2` in `Config.mk`.
- Build Circle sample `38-bootloader` (packaged as CircleNetboot by
  probonopd). It loads a kernel over Ethernet: HTTP upload on port 8080, or
  `tftp -m binary <ip> -c put kernel7.img`.
- **Check sample 38's README for Pi 2 B limitations before relying on it.**
- Circle's standard bootloader is serial only and would collide with the
  body on GPIO14/15, so it is not used.
- Done when: a hello-world kernel boots over `tftp put` and blinks the ACT LED.

### P1. Body link

- Use `protocol/c/` (`cobs`, `crc16`, `kilo_frame`) and `protocol/gen/kilo_protocol.h`
  as they are. Extend `protocol/vendor.py` to copy them into the mind tree.
- PL011 at the link baud from `spec.toml` (250000 today; optiboot flashing
  stays at 115200).
- Port `mind/kilo/link/` (transport, serial_link): framing, epochs, HELLO,
  reset-cause handling.
- Done when: the mind exchanges `HELLO` with the body and logs link stats.

### P2. Laptop interface

See §5. Command channel, telemetry, log stream, raw bridge. `kilo --host <ip>`
on the laptop.

Done when: `kilo status --host <ip>` and a live log tail work.

### P3. Mind core

Port in dependency order, each against the existing Python tests' cases where
they exist:

1. `body/` (client, state)
2. `planner/` (model, patterns, plan, walker): the 50 Hz `FEET` stream, 5 ticks
   ahead of the body's 8-deep queue
3. `tricks/` (model, player); the trick library ships as data on SD or
   compiled in
4. `perception/` (imu, ultrasonic, events, ring)
5. `behaviour/` (tree, selector, drives, personality, runner) at 10 Hz;
   `personality.yaml` converted on the laptop to a simple format the mind reads
6. IR remote key map (`ir.py`, `remote.py`); the body already decodes NEC and
   relays `EVENT IR_KEY`

Stays on the laptop: `vbody/`, `logs/`, `cli*.py`, `tricks/import_legacy.py`,
`tools/` (except the flasher, P4), `tests/`.

Done when: Kilo walks, does tricks and responds to the remote with no Linux.

### P4. Body flashing

See §6. Both network push and an SD card written on the laptop.

Done when: `kilo flash --host` and `kilo flash --sd` both end with a matching
`HELLO.fw_build`.

### P5. Production boot

- SD card holds the final `kernel7.img` directly (no netboot loader in the
  boot path).
- `config.txt`:
  ```
  boot_delay=0
  disable_splash=1
  gpu_mem=16
  start_file=start_cd.elf
  fixup_file=fixup_cd.dat
  kernel=kernel7.img
  ```
  Keep `device_tree=` only if Circle does not need it for the 2 B.
- Minimal FAT partition: the boot files, `kernel7.img`, `config.txt`,
  `body.bin` + manifest, mind config, a log directory.
- Bench mode (netboot loader) stays available on a second card, or as an
  option the production kernel can chain to (to decide in P5).

### P6. Measure

- Toggle a spare GPIO as the first instruction of `kernel7.img` and again
  when the first `FEET` frame goes out; scope both against the 5 V rail.
- Record in a `docs/p*-hardware-test-results.md`, against today's 26.6 s.

## 5. Laptop interface

One TCP listener on the Ethernet link. Messages defined in a new section of
`protocol/spec.toml` so the C++ mind and the Python CLI share one spec and
one generator. Framing: length-prefixed, CRC as on the body link.

| Channel | Direction | What |
|---|---|---|
| Commands | request/response | What the Unix socket serves today: postures, tricks, walk, status, params. The `kilo` CLI keeps its commands and gains `--host <ip>`. |
| Telemetry | mind → laptop stream | Body state, IMU, percepts, behaviour-tree state, link stats (CRC errors, overruns, tick lag). |
| Log | mind → laptop stream | The mind's in-RAM ring buffer, live tail; optional copy to SD. |
| Flash | laptop → mind | §6. |
| Raw bridge | both | The mind pauses its core and forwards UART bytes to and from TCP, so laptop tools can talk to the body directly. When the bridge closes the body's watchdog applies as usual. |

Rules:

- One controlling client at a time; any number of read-only observers.
- Losing the controlling client makes the mind send REST.
- Addressing: static IP or link-local on the cable; no DHCP server required
  on the laptop side (to decide in P2).

## 6. Body flashing

One image format for both paths: `body.bin` (raw binary from
`avr-objcopy -O binary`, so the mind never parses Intel HEX) and a manifest:
build hash (the `-DKILO_BUILD` value), size, CRC-32. The image is at most
32 KB less optiboot, so it fits in RAM.

**Build (laptop):** the existing `arduino-cli compile` step from
`mind/kilo/tools/flash.py`, then `body/tools/size_check.sh`, then `objcopy`
and the manifest. `arduino-cli` leaves the Pi.

**Path A, network push:** `kilo flash --host <ip> [--persist]`

1. Send manifest + image over the flash channel.
2. Mind checks CRC, size and build hash; with `--persist` writes both to SD.
3. Flash (below), report result.

**Path B, SD card written on the laptop:** `kilo flash --sd <mountpoint>`

1. Copy `body.bin` + manifest to the FAT partition.
2. At boot the mind compares the manifest's build hash with `HELLO.fw_build`
   and flashes only if they differ. Same image, no reflash, no boot cost.

**Flash sequence (mind):**

1. Stop streaming; send REST (or let the body watchdog do it).
2. Take the UART exclusively; switch to 115200.
3. Pulse GPIO4 as the emulated open-drain the Python side uses today
   (`mind/kilo/tools/gpio.py`: never driven high). Optiboot listens ~1 s.
4. STK500v1: `GET_SYNC` (retry until answered); per 128-byte page
   `LOAD_ADDRESS` + `PROG_PAGE`; verify with `READ_PAGE`; `LEAVE_PROGMODE`.
5. Back to the link baud; restart the link; expect `HELLO.fw_build` equal to
   the manifest's hash and the reset cause the mind expects after a flash.

Failure: optiboot survives an interrupted flash, so retry. Keep the last good
image on SD for a `kilo flash --rollback`.

## 7. Risks and open questions

- **Sample 38 on the 2 B.** Not verified. Fallback: write `kernel7.img` to SD
  on the laptop.
- **Ethernet bring-up time and stability** with Circle's USB stack: measure in
  P0. The core must not depend on it.
- **Floating point and timing** in the planner on a Cortex-A7 without Linux:
  enable the FPU in Circle; check the 20 ms budget early in P3.
- **Persistent logs** without journald: decide how much goes to SD and when,
  given power cuts (docs/setup.md "Power: safe mode").
- **Porting fidelity.** The Python tests and `vbody` are the oracle. Add a
  mode where the C++ mind runs on the laptop against `vbody` (Circle code
  behind a thin platform layer) so the same cases run in CI.
- **Power.** The USB-cable outages (docs/pi-outages-2026-10-13.md) are
  electrical; this plan does not fix them. The header UART does not share the
  USB 5 V path.

## 8. What is retired

On the Pi: Raspberry Pi OS, systemd units in `mind/systemd/`, the venv,
`arduino-cli`, ssh/Tailscale on the robot, `update.sh`. On the laptop nothing
is retired; the CLI gains `--host` and `--sd`.
