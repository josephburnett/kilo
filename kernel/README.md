# Kilo kernel

The mind as a bare-metal kernel for the Raspberry Pi 2 B, built on
[Circle](https://github.com/rsta2/circle) (docs/fast-boot.md). Everything is
cross-compiled on the laptop; nothing is built on the Pi. Phase P0 gave the
toolchain, Circle, a network boot loader and a hello-world kernel; phase P1
runs the mind core's body link on Circle (below, "The mind kernel"); P2
serves `kilo --host` over Ethernet from it ("The laptop interface"), and the
P3 walker runs in its loop. Not yet run on a Pi:
docs/fast-boot-first-boot.md is the procedure.

| Path | What |
|---|---|
| `circle/` | Circle, a git submodule pinned to `Step51.1.1` |
| `src/` | the mind kernel (`kernel7.img`): the core's seams on Circle (`seams.*`), the main loop (`kernel.*`), the network and its seams (`net.*`), the planner bench (`plannerbench.*`), the boot probe (`bootprobe.*`) |
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
| `build/kernel7.img` | the mind kernel | 369,652 bytes |
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
null device; and into a RAM ring that `kilo log --host IP` reads.

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
body link's serial device, the link, the walker, the host server, then the
net task), then one loop, every 1 ms:

```
link.mark_alive()
host.poll()                       accept, answer, stream, flush (never blocks)
if the bridge is closed:
  link.poll()
  walker.poll()                   timed: the loop stats line
planner bench step                once, after the first laptop client (below)
Circle's log events -> the ring   (USB, Ethernet, DHCP lines for `kilo log --host`)
stats once a second, the ACT LED
CScheduler::usSleep(1000)         the net task runs here
```

This is the composition `mind/README.md` gives ("Laptop interface", "The
planner"), as `kernel/host/main.cpp` and `walk_demo.cpp` do on the laptop.
The core is `mind/` (`mind/README.md`); `src/seams.*` and `src/net.*`
implement its seams:

| Seam | On Circle |
|---|---|
| `Serial` | `Pl011Serial` or `UsbSerial` (below) |
| `Clock` | `CTimer::GetClockTicks64()`: 1 MHz, 64 bits, no wrap |
| `Log` | `KernelLog`: every line into the RAM ring (`LogRing`, 64 lines, `kilo log --host`, Debug included), then `CircleLog`: `CLogger::WriteNoAlloc` (no heap), Info and above; Debug -> `LogDebug`, Info -> `LogNotice`, Warn -> `LogWarning`, Error -> `LogError` |
| `Gpio` | GPIO4 as an emulated open drain: released = input, no pull; asserted = latch low, then output. Never driven high. Released at boot; nothing pulses it until P4. |
| `Listener`, `Conn` | `CircleListener`, `CircleConn`: four slots of RX and TX rings, served by the net task (below) |

Circle's own log lines (`usbdev0-1: ...`, `dhcp: IP address is ...`) are
copied into the ring too, Notice and above, four per pass: CLogger keeps
its last 50 as events, which `CLogger::ReadEvent` hands over. Our lines go
to CLogger with `WriteNoAlloc`, which makes no event, so none is copied
twice.

The build id (`KILO_BUILD`, below) is also `STATUS.mind_build`, as
`KILO_MIND_BUILD` is on the laptop: the same 8 hex digits of the commit.

The link's first TX seq comes from the SoC's hardware RNG
(`CBcmRandomNumberGenerator`, `SerialLink::seed_seq()`): the clock reads
nearly the same at every boot, so the core's clock-based seed would repeat.

The loop sleeps with `CScheduler::usSleep(1000)`, not `CTimer::usDelay`: a
delay spins with nothing else running; a sleep hands the CPU to Circle's
cooperative scheduler, which runs the other tasks round robin (Circle's
`net` task, which processes the TCP/IP stack and never sleeps, `netphy`,
`dhcp`, and our `kilonet`) until the wake tick. Each of them yields within
one short pass, so the loop wakes within a pass of its deadline. A task
that does not yield for more than a few ms delays the link (it tolerates
about 10 ms); the stats line below shows the longest gap.

### Loop timing

Once a second, after the link lines:

```
loop 998/s: period max 1043 mean 1001 us, work max 61 mean 9 us, walker.poll max 2 mean 0 us; 1 clients
```

(numbers made up: not run yet). *period* is pass to pass, the 1 ms sleep
plus whatever the other tasks took; *work* is one pass without the sleep;
*walker.poll* is the walker's share; `(bench)` marks seconds when the
planner bench ran. Timed with `CTimer::GetClockTicks()` (1 MHz), so with no
scope.

Estimate per pass (est., from the host: `make -C kernel/host bench` gives
`Planner::next()` p99 about 4 us, start about 30 us on the laptop; the
Cortex-A7 at 900 MHz is taken as 20 times slower): `link.poll()` a few us
(a 66-byte frame's COBS and CRC), `host.poll()` tens of us with four
clients streaming TELEM at 50 Hz, `walker.poll()` near 0 between wakes and
about 100 us on a 20 ms wake (one or two planned ticks, the attitude fit).
That is under 0.2 ms of a 1 ms pass, so the 1 ms sleep stays: shorter
gains nothing (the walker's grid is 20 ms, the link's deadlines 10 ms and
up), longer delays the walker's wake. The planner's real cost on the Pi 2 B
is still to be measured: the bench and this line do it at the first boot.

### Planner bench

Once, from 1 s after the first laptop client connects (so a `kilo log
--host` tail shows its lines live: the 64-line ring holds only about 20 s
of log), one call per loop pass, a `Planner` of its own (not
the walker's; nothing goes to the body) walks trot, walk and crawl as
`kernel/host/tests/bench_planner.cpp` does once each (a speed change, a
turn, a change of pattern, the planned stop), about 1400 ticks each, then
logs one line per pattern:

```
planner bench trot: start 612 us; next() x1450 mean 31.20 us max 140 us (a tick is 20000 us; Python on the Pi: max 4500 us)
```

(numbers made up.) It pauses during a walk or the raw bridge, takes about
5 s, then frees its planner. `make -C kernel KILO_PLANNER_BENCH=0` leaves
it out (about 400 bytes). Interrupts (USB, the PL011) land inside the timed calls,
so *max* includes them, as a real walk's would.

## The laptop interface

`kilo --host IP` on the laptop talks to the kernel over the Ethernet cable,
TCP port 5300 (docs/fast-boot.md §5, `mind/README.md` "Laptop interface").

### Network bring-up

Plan rule 1 (docs/fast-boot.md §3): the control loop comes first and never
waits for the network. `CKiloNet` (`src/net.*`), a Circle task, is created
last in `Initialize` and first runs at the main loop's first sleep:

1. **USB** (only with `KILO_BODY_SERIAL=pl011`). The LAN9514 Ethernet chip
   is a USB device, so Circle's USB host must scan first. With the default
   `auto`, `ChooseSerial` already did, before the link. With `pl011` the
   task waits for the body's first HELLO (or 1 s), then scans. Circle's
   enumeration spins (`CTimer::MsDelay`, no yield), so **the main loop stops
   for the length of the scan**, about a second (est.), once. The body's
   watchdog covers it; at boot nothing is walking. The log line `USB scan
   for Ethernet: done, N ms (the main loop waited)` measures it.
2. **TCP/IP**: `CNetSubSystem::Initialize(FALSE)`: do not wait for the PHY
   (Circle would spin up to 4 s). Circle's `net` task then finds the
   device, the `netphy` task watches the link, and a `dhcp` task asks for a
   lease under the hostname `kilo`.
3. **Address**: DHCP, or the fixed link-local fallback (`protocol/spec.toml`
   `[host]`): when no lease came `KILO_HOST_DHCP_TIMEOUT_MS` (3 s) after the
   Ethernet link came up, the task writes 169.254.75.73/16 into
   `CNetConfig`. The 3 s count from link-up, not from boot: the LAN9514's
   autonegotiation alone takes about 2 s (est.).
4. **Listening**: a TCP socket on that address, port 5300, backlog 4. The
   log says `listening on 192.168.88.X:5300 (DHCP lease): kilo status
   --host 192.168.88.X`, or `(link-local fallback)`.

Limits of the fallback, from Circle's API:

- Circle picks DHCP or a static address when `CNetSubSystem` is built and
  has no way to stop its DHCP client later. So after the fallback the
  client keeps trying (DISCOVER at 4, 8, 16, 32 s, then again), from
  169.254.75.73. If a lease comes later (the cable moved from the laptop
  to the router), it replaces the fallback, the task logs it and listens on
  the new address; connections on the old one die.
- A listening socket is tied to the address it was made with (Circle's TCP
  takes the own address for its checksum when the connection is created),
  so the task makes a new one at every change of address. Same port twice
  is allowed by Circle; the old socket is deleted first.
- No duplicate address check (ARP probe) for 169.254.75.73: two Kilos on
  one cable would clash. One robot, one cable.
- On a direct cable the laptop's port must be link-local too (NetworkManager
  "Link-Local Only"); no DHCP server needed on either side.

### The net task

Circle's sockets block by default; the core's network seams must not
(`mind/platform_net.h`). `CKiloNet` owns every socket and moves bytes
between them and four connection slots (`KILO_HOST_MAX_CLIENTS`), each a
pair of rings; `HostServer::poll()` in the main loop touches only the rings
(`CircleListener::accept`, `CircleConn::read`/`write`/`close`). Every 1 ms
(`usSleep(1000)`), the task:

- **accepts** only when `CSocket::GetStatus()` on the listening socket says
  a connection is already established, so `Accept()` returns at once (it
  would otherwise wait for one). With every slot busy it closes the new
  connection at once.
- **receives** with `MSG_DONTWAIT`, and only when the RX ring has room for a
  whole segment: Circle's `Receive()` hands over one TCP segment and drops
  what does not fit the buffer it is given.
- **sends** with `MSG_DONTWAIT`, at most 1 KB at a time (one segment), and
  only while `GetStatus().bTxReady`: Circle's TCP queue holds less than its
  64 KB limit. A non-blocking `Send()` past that would queue on the heap
  without bound when the laptop stops reading.
- **closes**: `CircleConn::close()` only marks the slot; the task deletes the
  socket (TCP FIN) and frees the slot on its next pass.

The two sides share the slots without locks: one core, a cooperative
scheduler, and neither yields in the middle of a ring update. `Send()`
yields once inside (after copying the bytes); meanwhile the main loop can
only append past the bytes being sent, or mark the slot closing.

**Ring sizes**, per slot: RX 4 KB, TX 4 KB, 32 KB for four. RX: a segment is
at most `FRAME_BUFFER_SIZE` (1600) bytes and the task receives only when one
fits, so 4 KB holds two and a margin; the laptop's requests are at most 246
bytes (`KILO_HOST_MAX_FRAME`) and `HostServer` reads up to 1 KB a pass. TX:
`HostServer` keeps its own 4 KB per client and flushes what this ring takes;
the task empties it every ~1 ms into Circle's queue (64 KB more). The
streams are at most about 10 kB/s (TELEM at 50 Hz, log lines), so 4 KB is
400 ms of output. A laptop that stops reading fills Circle's 64 KB, then
these 4 KB, then `HostServer`'s 4 KB; then `HostServer` drops it (TELEM is
skipped first).

### What `--host` does on the kernel today

What `HostServer` serves, the same as `kilo-mind-host` on the laptop:
`kilo status`, `kilo monitor` (TELEM), `kilo log` (the ring, Circle's lines
included), `kilo posture NAME [--for S]` (REST when the controller lets go
or goes silent), and the raw bridge (`kilo.link` over TCP; the link and the
walker pause). Not yet: walking, tricks, params (`PARAM_*` answer
UNSUPPORTED). `HostServer` has no walk command; the walker runs in the loop
and gets the link's TELEMETRY and EVENTs, but nothing on the kernel starts a
walk yet. P3's `Mind` composition will own that. A walk the raw bridge cut
off is aborted when the bridge closes.

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

The LED says nothing about the network (the log does). With
`KILO_BODY_SERIAL=pl011` the blink freezes once for about a second (est.)
early on: the USB scan for Ethernet ("Network bring-up").

### Boot probe

GPIO21, header pin 40 (pin 39 is ground), outside the NyBoard socket's pins
1-10. High at the top of `main()`, low at the first HELLO, for a scope
against the 5 V rail (docs/fast-boot.md P6). On by default;
`make -C kernel KILO_BOOT_PROBE=0` leaves the pin alone.

### Floating point

`MIND_SRCS` are compiled with Circle's flags (`-mcpu=cortex-a7
-mfpu=neon-vfpv4 -mfloat-abi=hard`; `readelf -A` on the objects: VFPv4,
VFP-register arguments). The planner is the core's floating point
(`mind/README.md`, "The planner"; built with `-ffp-contract=off
-fno-math-errno` as on the host). A probe
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
The network: the LAN9514 coming up through Circle's SMSC951x driver; how
long the link takes to come up; DHCP from the house router under the name
`kilo`; the link-local fallback on a direct cable (and a lease that comes
after it, from a DHCP client that has been sending from 169.254.75.73);
accept, receive and send through `GetStatus()` as read from Circle's source;
`kilo status`, `log`, `monitor`, `posture` and the bridge over the cable;
the loop's period with the net task running (the stats line); the USB scan
from the net task with `KILO_BODY_SERIAL=pl011` and how long it stalls the
loop; the planner's cost on the Cortex-A7 (the bench line).
`kernel/host/test.sh` covers the core, the link (including `tx_free()`
deferral), `HostServer` over POSIX sockets and the walker on Linux only;
`src/net.*` is compiled but has never run.

## Open questions

- **Finding the loader without a monitor.** DHCP address from the router's
  lease table today. The mind kernel has the link-local fallback (P2,
  above); the loader does not, so on a direct cable it gets no address.
  The loader could also blink its address on the LED or announce it over
  the network.
- **Finding the mind kernel by name.** Circle has an mDNS publisher
  (`CmDNSPublisher`); announcing `kilo.local` would save reading the
  router's lease table. Not done.
- **Loader on the 2 B.** Not run on hardware yet (docs/fast-boot.md §7). If it
  fails, write `kernel7.img` to the card with `make-sd.sh --kernel`.
- **Licence.** Circle is GPLv3, so any `kernel7.img` that links it is too,
  and the files in `netboot/` are copies of Circle's and keep its licence.
  The rest of the repository is MIT. This matters only if images or the
  kernel sources are given to others.
