# Kilo kernel

The mind as a bare-metal kernel for the Raspberry Pi 2 B, built on
[Circle](https://github.com/rsta2/circle) (docs/fast-boot.md). Everything is
cross-compiled on the laptop; nothing is built on the Pi. Phase P0 gave the
toolchain, Circle, a network boot loader and a hello-world kernel; phase P1
runs the mind core's body link on Circle (below, "The mind kernel"). Not yet
run on a Pi: docs/fast-boot-first-boot.md is the procedure.

| Path | What |
|---|---|
| `circle/` | Circle, a git submodule pinned to `Step51.1.1` |
| `src/` | the mind kernel (`kernel7.img`): the core's seams on Circle (`seams.*`), the main loop (`kernel.*`), the boot probe (`bootprobe.*`) |
| `netboot/` | the network loader: Circle's `sample/38-bootloader` with our `kernel.cpp`, `kernel.h`, `main.cpp` (see below) |
| `boot/config.txt` | the Pi's `config.txt` |
| `tools/make-sd.sh` | writes a boot card |
| `tools/netboot.sh`, `tools/tftp_put.py` | send a kernel to the loader |
| `mind/` | the portable mind core, linked in through `mind/sources.mk` (`MIND_SRCS`, paths relative to `kernel/`, built with `-I mind`) |

## Toolchain

Arm GNU Toolchain 15.2.Rel1 (`arm-none-eabi`), Circle's recommended version,
unpacked outside the repository. No root needed:

```
mkdir -p ~/opt && cd ~/opt
curl -LO https://developer.arm.com/-/media/Files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi.tar.xz
curl -LO https://developer.arm.com/-/media/Files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi.tar.xz.sha256asc
sha256sum -c arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi.tar.xz.sha256asc
tar xJf arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi.tar.xz && rm arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi.tar.xz*
```

(sha256 `597893282ac8c6ab1a4073977f2362990184599643b4c5ee34870a8215783a16`.)
The Makefile finds it through `TOOLCHAIN` (default
`~/opt/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi`), not through `PATH`.

## Build

```
git submodule update --init kernel/circle
make -C kernel          # Circle's libraries, then both images
make -C kernel boot     # once: download the Pi firmware into circle/boot (wget)
```

`make -C kernel` writes `circle/Config.mk` (`RASPPI = 2`, `AARCH = 32`, the
toolchain prefix, `STDLIB_SUPPORT = 1`), builds the Circle libraries the
images link (`lib`, `usb`, `input`, `fs`, `fs/fat`, `sched`, `net`) inside the
submodule, then:

| Output | What | Size |
|---|---|---|
| `build/kernel7.img` | the mind kernel | 247,956 bytes |
| `build/netboot/kernel7.img` | the network loader | 305,420 bytes |

The `.elf`, `.map` and `.lst` beside each image are for debugging. A second
`make` with nothing changed does nothing. From clean (Circle included) the
build takes about 12 s on 4 cores. `make -C kernel clean` removes `build/`;
`make -C kernel circle-clean` removes Circle's objects.

The kernel logs `Kilo mind, build <hash>`: the first 8 hex digits of the
commit, as `-DKILO_BUILD=0x<hash>`, the same convention as the body firmware.

## Two SD cards

**Bench card** (network loader), on a FAT-formatted card, mounted:

```
kernel/tools/make-sd.sh /media/$USER/BOOT --netboot
```

**Production card** (the mind kernel boots directly):

```
kernel/tools/make-sd.sh /media/$USER/BOOT --kernel
```

Both write `bootcode.bin`, `start_cd.elf`, `fixup_cd.dat`, `LICENCE.broadcom`,
`config.txt` and `kernel7.img`, about 1.2 MB. The script never partitions or
formats. It refuses anything that is not the root of a mounted vfat
filesystem, and any partition holding files other than Pi boot files, so a
Raspberry Pi OS card (it has `cmdline.txt` and `overlays/`) is refused rather
than losing its Linux `kernel7.img`. `--force-dir` skips the mount checks; it
exists for testing on a plain directory and is not for real cards.

`boot/config.txt` is Circle's `boot/config32.txt` without its Pi 4 sections,
plus the fast-boot lines from docs/fast-boot.md P5: `boot_delay=0`,
`disable_splash=1`, `gpu_mem=16` with the cut-down firmware
(`start_cd.elf`, `fixup_cd.dat`), `kernel=kernel7.img`. It also sets an empty
`device_tree=`: Circle reads a device tree only on the Pi 4 and later
(`CMachineInfo::FetchDTB` is compiled for `RASPPI >= 4`), so the 2 B needs
none, and Circle's firmware download does not include a Pi 2 device tree
anyway.

## Netboot

With the bench card in, an Ethernet cable to the house router
(192.168.88.0/24, DHCP) and power on, the loader asks for an address under
the hostname `kilo-netboot` and waits. Find its address in the router's lease
table (or by the Pi's MAC, `b8:27:eb:…`), then:

```
kernel/tools/netboot.sh 192.168.88.X                 # TFTP, sends build/kernel7.img
kernel/tools/netboot.sh --http 192.168.88.X          # the same over HTTP, port 8080
kernel/tools/netboot.sh 192.168.88.X other.img       # any image
```

Or open `http://192.168.88.X:8080/` in a browser. The loader copies the image
over itself and jumps to it; nothing is written to the card. To load another
image, power-cycle the Pi so the loader starts again. The mind kernel shows
its state on the ACT LED (green, GPIO47; "ACT LED" below).

`netboot.sh` uses `tools/tftp_put.py` (Python standard library, RFC 1350,
512-byte blocks, resends after 1 s, 5 tries), so no TFTP client is needed.
`tftp -m binary <ip> -c put build/kernel7.img kernel7.img` from tftp-hpa
does the same.

### The loader on the Pi 2 B

From Circle's `sample/38-bootloader/README` and its source:

- No password or authorization: anyone on the network can boot any image on
  the Pi. Use it only on a network you trust.
- It accepts only file names `kernel*.img` (`netboot.sh` always sends
  `kernel7.img`) and at most 2 MB (`KERNEL_MAX_SIZE`): the mind kernel must
  stay under that to be netbooted.
- Chain boot does not work from a multi-core application; the loader is
  single-core. Whether a booted kernel can then start the other three cores
  is untested.
- After chain boot, accelerated graphics, HDMI sound "and possibly the ACT
  LED" cannot be restored if the loader used them, so it does not use them.
  The LED caveat fits boards whose LED is behind the firmware (Pi 3, a
  "virtual pin" in Circle); on the 2 B it is plain GPIO47, active high.
- Some drivers do not reset their device on the way out; the README says the
  loader's own devices (USB, Ethernet) are fine.
- Nothing in the README excludes the Pi 2 B. Not yet tried on Kilo's board.

Our copy changes three things in the sample, none in the HTTP and TFTP
servers (compiled from the submodule as they are):

- **No `CSerialDevice`.** The sample opens the PL011 on GPIO14/15 at 115200,
  which is where the body link goes. Without it the loader never touches
  those pins.
- **Logs go to the screen, or nowhere.** The sample logs to the HDMI screen
  (and to serial only if `cmdline.txt` asks). Kilo has no monitor, and with
  `gpu_mem=16` the firmware may not give a frame buffer, so a failed screen
  is not fatal: logs then go to a null device. The address it prints is
  therefore only visible in the router.
- **DHCP hostname `kilo-netboot`** instead of `raspberrypi`.

The mind kernel logs the same way: to the screen if there is one, else to a
null device.

## Circle without a C library

`STDLIB_SUPPORT = 1` is Circle's default: the kernel links `libgcc` and the
toolchain's `libm`, and nothing else (Circle's `doc/stdlib-support.txt`).
The mind core has to live within that:

- No C library: no `printf`, `FILE`, `strtod`, `time`. The toolchain's
  newlib headers are still on the include path, so `#include <stdio.h>`
  compiles and `printf` then fails at link time. Circle has its own
  `malloc`/`free`/`calloc` (`circle/alloc.h`), `new`/`delete`,
  `memcpy`/`strcmp`/`strtoul` and friends (`circle/util.h`),
  `CString::Format` (with `%f`), `__errno`, timers and file access (FatFs).
- `libm` is linked: `sinf`, `cosf`, `atan2f`, `sqrtf`, `powf`, `expf`,
  `hypotf` and their `double` versions link (checked; they add about 29 KB),
  on the Cortex-A7's VFPv4 (`-mfpu=neon-vfpv4 -mfloat-abi=hard`). Include
  `<math.h>`, not `<cmath>`.
- No C++ standard library: no `std::vector`, `std::string`, `<algorithm>`,
  `<array>`, `<functional>`, `<cmath>`, `<cstdint>`. Use `<stdint.h>` or
  Circle's `u8`…`u64`. Compiled with `-nostdinc++ -fno-exceptions -fno-rtti
  -ffreestanding`.
- No exceptions, no RTTI (`dynamic_cast`, `typeid`).

`STDLIB_SUPPORT = 2`, `3` or `4` (newlib, libstdc++) needs the separate
[circle-stdlib](https://github.com/smuehlst/circle-stdlib) project, which
brings its own copy of Circle. Moving to it later is possible but is a
change of build, not of a flag.

## The mind kernel

`src/kernel.cpp`: initialise (screen or null log, interrupts, timer, the
body link's serial device), then one loop, every 1 ms: `mark_alive()`,
`link.poll()`, link statistics once a second, the ACT LED. The core is
`mind/` (`mind/README.md`); `src/seams.*` implements its seams:

| Seam | On Circle |
|---|---|
| `Serial` | `Pl011Serial` or `UsbSerial` (below) |
| `Clock` | `CTimer::GetClockTicks64()`: 1 MHz, 64 bits, no wrap |
| `Log` | `CLogger::WriteNoAlloc` (no heap), Info and above; Debug -> `LogDebug`, Info -> `LogNotice`, Warn -> `LogWarning`, Error -> `LogError`. The RAM ring for P2's log tail plugs in at `CircleLog::write`. |
| `Gpio` | GPIO4 as an emulated open drain: released = input, no pull; asserted = latch low, then output. Never driven high. Released at boot; nothing pulses it until P4. |

The link's first TX seq comes from the SoC's hardware RNG
(`CBcmRandomNumberGenerator`, `SerialLink::seed_seq()`): the clock reads
nearly the same at every boot, so the core's clock-based seed would repeat.

The loop sleeps with `CScheduler::usSleep(1000)`, not `CTimer::usDelay`: a
delay spins with nothing else running; a sleep hands the CPU to Circle's
cooperative scheduler. Alone (P1) the scheduler spins until the wake tick,
the same cost; from P2 the net task runs in that gap. A task that does not
yield for more than a few ms delays the link (it tolerates about 10 ms).

### Body link: two serial back ends

**Selection rule** (`CKernel::ChooseSerial`): initialise Circle's USB host
(`CUSBHCIDevice`, one scan, no plug-and-play) and use the USB serial device
`utty1` if one enumerated; else the PL011. The log line `body link on ...`
says which. A USB adapter plugged in after boot is not seen.

`make -C kernel KILO_BODY_SERIAL=pl011` skips the USB scan and opens the
PL011 at once: the production wiring, where USB enumeration (about a second
or more, est.) must not come before the link (docs/fast-boot.md §3 rule 1).

- **USB** (today's wiring): Kilo's CH343 adapter enumerates as CDC-ACM
  (interface class 2-2-1), so Circle's `CUSBSerialCDCDevice` drives it and
  names it `utty1` (any USB serial adapter Circle knows gets that name).
  Baud is set with CDC `SET_LINE_CODING` (250000, 8N1), the same request
  Linux's `cdc_acm` sends. Opening the adapter raises DTR, which on the
  NyBoard may reset the body once at boot, as opening `/dev/ttyACM0` does
  on Linux; the link sees an unsolicited HELLO. Circle's driver writes
  synchronously (a bulk OUT transfer, no timeout: a frame is two
  full-speed packets) and reads one USB packet (64 B) per completed IN
  request: at a 1 ms loop that is up to 64 kB/s, against 25 kB/s line rate.
  `tx_free()` is a constant 512: there is no queue on this side to measure.
- **PL011** (GPIO14/15, header pins 8 and 10; `ttyS1`): Circle's
  `CSerialDevice`, interrupt driven. Built only when chosen, because its
  constructor takes over GPIO14/15. 250000 baud is exact: UARTCLK 48 MHz /
  (16 x 250000) = 12, remainder 0, so IBRD 12, FBRD 0. The NL -> CR NL
  translation that is on by default is switched off (it would break binary
  frames). Line errors (break, overrun, framing, parity) come back from
  `Read()` once, negative, with the data still queued: counted, logged with
  the stats, and the read repeated. `tx_free()` is the TX ring's free space.
  `set_baud()` only accepts the rate it was opened with: `CSerialDevice`
  cannot be initialised twice; switching to 115200 for flashing is P4's.

**Ring sizes** (PL011): Circle's `SERIAL_BUF_SIZE`, 2048 bytes each way,
kept. Needs: RX is TELEMETRY (<= 66 B wire) at 25-50 Hz plus events and
acks, about 4 kB/s; the worst case between two polls is the line rate for
the ~10 ms the link tolerates, 250 B; 4x margin = 1 KB. TX is FEET at 50 Hz
(<= 66 B), goal refreshes at 5 Hz and heartbeats, about 4.5 kB/s; the worst
burst in one poll is about 300 B (every goal plus FEET, HELLO_REQ and a
command); 4x = 1.2 KB. 2048 covers both (82 ms of line-rate input). Making
it smaller would mean defining `SERIAL_BUF_SIZE` for Circle's library build
too (it sets the class layout), for 2 KB of RAM.

**TX backpressure**: `Serial::tx_free()` (mind/platform.h). The link writes
a frame only when `tx_free()` covers all of it; otherwise it counts
`tx_deferred` and retries at the next poll, so a frame is never cut.

### ACT LED

| Pattern | Meaning |
|---|---|
| steady on | starting up (the USB scan) |
| 0.5 Hz (1 s on, 1 s off) | no body: no HELLO yet, or no TELEMETRY for 300 ms |
| 2 Hz | the body is alive: TELEMETRY flowing |
| 5 Hz | the body's HELLO has another protocol major: the kernel stays up and keeps the heartbeat, but sends no goals or FEET (kilo-mind-host exits 2 instead) |
| steady on or off for more than ~20 s | stopped: `Initialize` failed or a crash (Circle halts, the LED freezes) |

### Boot probe

GPIO21, header pin 40 (pin 39 is ground), outside the NyBoard socket's pins
1-10. High at the top of `main()`, low at the first HELLO, for a scope
against the 5 V rail (docs/fast-boot.md P6). On by default;
`make -C kernel KILO_BOOT_PROBE=0` leaves the pin alone.

### Floating point

`MIND_SRCS` are compiled with Circle's flags (`-mcpu=cortex-a7
-mfpu=neon-vfpv4 -mfloat-abi=hard`; `readelf -A` on the objects: VFPv4,
VFP-register arguments). The core has no floating point yet. A probe
compiled with the same flags turns float arithmetic into VFP instructions
(`vmul.f32`, `vfma.f32`, `vdiv.f32`) and calls `sqrtf`, which in the
toolchain's hard-float `libm` (`thumb/v7ve+simd/hard`) is a wrapper around
`vsqrt.f32` (the wrapper sets `errno` for negative inputs;
`-fno-math-errno` would inline `vsqrt.f32`).

### Not yet tested on hardware

All of it: nothing here has run on a Pi. In particular: the netboot loader
on the 2 B; Circle's USB host finding the CH343 behind the LAN9514 hub and
its CDC driver at 250000 baud; whether opening it resets the body; how long
the USB scan takes; USB write latency; the PL011 at 250000 against the
328P; the LED codes; the boot probe; the screen with `gpu_mem=16`; the
hardware RNG's warm-up time (Circle discards the first 0x40000 numbers).
`kernel/host/test.sh` covers the core and the link (including `tx_free()`
deferral) on Linux only.

## Open questions

- **Finding the loader without a monitor.** DHCP address from the router's
  lease table today. A cable straight to the laptop has no DHCP server: a
  static or link-local address is P2's decision. The loader could also blink
  its address on the LED or announce it over the network.
- **Loader on the 2 B.** Not run on hardware yet (docs/fast-boot.md §7). If it
  fails, write `kernel7.img` to the card with `make-sd.sh --kernel`.
- **Licence.** Circle is GPLv3, so any `kernel7.img` that links it is too,
  and the files in `netboot/` are copies of Circle's and keep its licence.
  The rest of the repository is MIT. This matters only if images or the
  kernel sources are given to others.
