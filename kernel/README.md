# Kilo kernel

The mind as a bare-metal kernel for the Raspberry Pi 2 B, built on
[Circle](https://github.com/rsta2/circle) (docs/fast-boot.md). Everything is
cross-compiled on the laptop; nothing is built on the Pi. Phase P0 gave the
toolchain, Circle, a network boot loader and a hello-world kernel; phase P1
runs the mind core's body link on Circle (below, "The mind kernel"); P2
serves `kilo --host` over Ethernet from it ("The laptop interface"), P3's
whole mind core runs in its loop as one `Mind` (the walker, tricks,
perception, the IR remote, the behaviour runner: "The Mind's settings"), and
P4 flashes the body from the laptop or the SD card ("Flashing the body"). P5's production card is
`make-sd.sh --kernel` ("SD cards"), and `kilo update --host` replaces the
kernel on it over the network ("Updating the kernel over the network").
docs/fast-boot-first-boot.md is the procedure on the Pi.

| Path | What |
|---|---|
| `circle/` | Circle, a git submodule pinned to `Step51.1.1` |
| `src/` | the mind kernel (`kernel7.img`): the core's seams on Circle (`seams.*`), the main loop (`kernel.*`), the network and its seams, mDNS (`net.*`), the SD card: the body image and kernel updates (`bootcard.*`), the planner bench (`plannerbench.*`), the boot probe (`bootprobe.*`) |
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
images link (`lib`, `usb`, `input`, `fs`, `sched`, `net`, and the addons
`fatfs` and `SDCard`) inside the submodule, then:

| Output | What | Size |
|---|---|---|
| `build/kernel7.img` | the mind kernel | 607,140 bytes with one key (`KILO_BODY_SERIAL=auto`: about 250 bytes less) |
| `build/netboot/kernel7.img` | the network loader | 305,420 bytes |

Of the mind kernel, the SD card (FatFs about 23 KB, the SD driver about
18 KB) and mDNS (about 9 KB) came with P4. The linker keeps every object of
`MIND_SRCS` whether or not it is called (no `--gc-sections`), so P3's
sources were in the image before the kernel used them; composing the
`Mind` changed its size by less than 1 KB. The keys (Monocypher, all of
it linked: about 55 KB; the Noise layer and the secure host server, about
13 KB) took the image from 522,532 to 591,140 bytes; each authorized key
adds 72 bytes. The walk recorder took it from 594,916 to 607,140 bytes;
the flight recorder, its card saver and the metrics ("The flight
recorder") and the mDNS responder to 649,188.

The build needs at least one laptop key in `keys/authorized_keys` ("Who may
talk to Kilo: keys"), or `KILO_AUTHORIZED_KEYS=FILE`; without one it stops
and says what to do.

The `.elf`, `.map` and `.lst` beside each image are for debugging. A second
`make` with nothing changed does nothing. From clean (Circle included) the
build takes about 12 s on 4 cores. `make -C kernel clean` removes `build/`;
`make -C kernel circle-clean` removes Circle's objects.

The kernel logs `Kilo mind, build <hash> (KILO-MIND-KERNEL-1)`: the first
8 hex digits of the commit, as `-DKILO_BUILD=0x<hash>`, the same convention
as the body firmware, and the image's identity for `kilo update` (below).

## SD cards

**One card is enough.** It boots the mind kernel straight from
`kernel7.img` at power-on, with no network, and `kilo update --host`
replaces that kernel over the network ("Updating the kernel over the
network"). P5 had settled on two cards (docs/fast-boot.md P5 left "a
second card, or an option the production kernel can chain to" open):
chaining from the mind kernel to the loader would put the loader's code
and a decision into every production boot. The update needs neither: the
kernel writes its successor to the card and reboots.

The **netboot loader** stays a bench option: a kernel sent over the
network runs from RAM and nothing is written, which suits trying builds
that may not work. Both kinds of card share one `config.txt`
(`boot/config.txt`; `make -C kernel card` stages byte-identical copies), so
`kilo update` can turn a loader card into a kernel card and back by
swapping `kernel7.img` alone.

**Kernel card** (the mind kernel boots directly), on a FAT-formatted card,
mounted:

```
kernel/tools/make-sd.sh /media/$USER/BOOT --kernel [--body ~/.cache/kilo-build]
```

**Loader card** (the bench):

```
kernel/tools/make-sd.sh /media/$USER/BOOT --netboot
```

A loader card becomes a kernel card without a card reader: netboot the
mind kernel (below), then `kilo update --host IP`. Back:
`kilo update --host IP --image kernel/build/netboot/kernel7.img --force`.

Both write `bootcode.bin`, `start_cd.elf`, `fixup_cd.dat`, `LICENCE.broadcom`,
`config.txt` and `kernel7.img`, about 1.3 MB. `--body DIR` also copies a
body image, `DIR/body.bin` and `DIR/body.json` (what `kilo flash` builds;
both or neither, and the manifest's size must be body.bin's), which the
mind kernel flashes at boot when the body runs another build ("Flashing
the body"). It works on either card: a kernel netbooted from the bench
card reads the bench card. Without `--body`, a body image already on the
card stays (the script says so). The script never partitions or
formats. It refuses anything that is not the root of a mounted vfat
filesystem, and any partition holding files other than Pi boot files, so a
Raspberry Pi OS card (it has `cmdline.txt` and `overlays/`) is refused rather
than losing its Linux `kernel7.img`; `kernel7.old` and `kernel7.new`, which
`kilo update` leaves, are accepted (and kept). `--force-dir` skips the mount
checks; it exists for testing on a plain directory and is not for real
cards. `make -C kernel card` stages both cards' files in
`build/card/netboot/` and `build/card/kernel/`, for copying by hand.

That is the whole production boot path: the GPU firmware reads
`config.txt`, loads `kernel7.img` and starts it; nothing else on the card
is read until the kernel reads `body.json` (after the body's first HELLO).
P5's "mind config" and "log directory" on the card do not exist: the
kernel has no settings file yet, and logs stay in RAM (the open question
of docs/fast-boot.md §7, "Persistent logs").

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
image, get the loader back: `kilo reboot --host IP` while the mind kernel
runs (it reboots the Pi, which boots the card again: the loader; "The
laptop interface" below), or power-cycle the Pi. Then `netboot.sh` again,
to the loader's address (`kilo-netboot` in the router; usually the same).
The mind kernel shows its state on the ACT LED (green, GPIO47; "ACT LED"
below).

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

`src/kernel.cpp` runs the whole core as one `kilo::Mind` (`mind/mind.h`,
`mind/README.md` "The Mind"), as `kernel/host/main.cpp` does on Linux.
`Initialize`: the screen or a null log, interrupts, timer, then the
`Mind` (first, so every later line is in its ring), the body link's
serial device (below), the flasher (attached through `mind.host()`),
`mind.start()` (balance on, then `link.connect()`: HELLO_REQ), the
planner bench, and last the net task. Then one loop, every 1 ms:

```
mind.poll()                       timed: the loop stats line
  link.mark_alive()               unless the behaviour runner stalled (mind/README.md "Liveness")
  host.poll()                     accept, answer, stream, flush; polls the flasher (never blocks)
  if host.link_paused(): done     the raw bridge is open or the flasher has the UART
  walker.abort() if a pause just ended and a walk was on
  link.poll()
  the command slot's answer       to the trick player; the body's STATS once a second
  walker.poll(), player.poll()
  remote.poll()                   IR keys (on)
  runner.poll()                   the behaviour runner at 10 Hz (KILO_BEHAVIOUR=1 only)
end of a flash: log it; --persist: write the image to SD
boot flash check                  once, after the first HELLO (or 3 s): SD image, flash if another build
link.connect() again              once a second, only if the Serial failed
planner bench step                once, after the first laptop client (below)
Circle's log events -> the ring   (USB, Ethernet, DHCP lines for `kilo log --host`)
stats once a second, the ACT LED
reboot                            after a laptop's REBOOT: 200 ms more passes (the OK leaves), then
                                  Run() returns ShutdownReboot and main() calls reboot()
CScheduler::usSleep(1000)         the net task runs here
```

Where the memory is: the flight recorder's store (64 MB, "The flight
recorder"), the log's history (4096 lines, 700 KB), the card saver (58 KB),
the `Mind` (133 KB: its recorder's run index 38 KB, the log's small ring
and status lines 16 KB, the host
server's five connections 36 KB (each 4 KB of frames to send, a 1 KB
record each way and its Noise state), perception's windows 17 KB, the
behaviour runner 6 KB), the network slots (`CircleListener`, 40 KB of
rings) and the two flash images (32 KB each) are on Circle's heap, built in
`Initialize`. `CKernel` itself (4 KB) lives on `main()`'s stack, so they
are not its members; as statics, their initialisers would go into the
image. With the net task's 64 KB stack that is about 0.25 MB of a heap of
about 1000 MB (1 GB less `gpu_mem=16` and Circle's low memory; est.).

The `Mind` is built before the serial device is chosen, so it takes a
`SerialSlot` (`src/seams.h`) that forwards to `Pl011Serial` or
`UsbSerial` once `ChooseSerial` has picked one; nothing calls it before
`mind.start()`.

This is the composition `mind/README.md` gives ("Laptop interface", "The
planner"), as `kernel/host/main.cpp` and `walk_demo.cpp` do on the laptop.
The core is `mind/` (`mind/README.md`); `src/seams.*` and `src/net.*`
implement its seams:

| Seam | On Circle |
|---|---|
| `Serial` | `Pl011Serial` or `UsbSerial` (below) |
| `Clock` | `CTimer::GetClockTicks64()`: 1 MHz, 64 bits, no wrap |
| `Log` | every part logs into the `Mind`'s RAM ring (`mind.log()`, a `LogRing` of 4096 lines of history, `kilo log --host`, Debug included; "The log"), which passes each line to the platform log, `KernelLog`, and that to `CircleLog`: `CLogger::WriteNoAlloc` (no heap), Info and above; Debug -> `LogDebug`, Info -> `LogNotice`, Warn -> `LogWarning`, Error -> `LogError` |
| `Gpio` | GPIO4 (header pin 7, the NyBoard's reset line) as an emulated open drain: released = input, no pull; asserted = latch low, then output. Never driven high. Released at boot; only the flasher pulses it. |
| `Listener`, `Conn` | `CircleListener`, `CircleConn`: four slots of RX and TX rings, served by the net task (below) |

Circle's own log lines (`usbdev0-1: ...`, `dhcp: IP address is ...`) are
copied into the ring too, Notice and above, four per pass: CLogger keeps
its last 50 as events, which `CLogger::ReadEvent` hands over. `KernelLog`
is muted while they are written, so the ring does not hand them back to
CLogger to print again. Our lines go to CLogger with `WriteNoAlloc`, which
makes no event, so none is copied twice.

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
loop 998/s: period max 1043 mean 1001 us, work max 61 mean 9 us, mind.poll max 120 mean 6 us; 1 clients
```

(numbers made up: not run yet). *period* is pass to pass, the 1 ms sleep
plus whatever the other tasks took; *work* is one pass without the sleep;
*mind.poll* is the mind core's share (the laptop, the link, the walker,
the trick player, the remote, the behaviour runner); `(bench)` marks
seconds when the
planner bench ran. Timed with `CTimer::GetClockTicks()` (1 MHz), so with no
scope.

Estimate per pass (est., from the host: `make -C kernel/host bench` gives
`Planner::next()` p99 about 4 us, start about 30 us on the laptop; the
Cortex-A7 at 900 MHz is taken as 20 times slower): `link.poll()` a few us
(a 66-byte frame's COBS and CRC), `host.poll()` tens of us with four
clients streaming TELEM at 50 Hz, `walker.poll()` near 0 between wakes and
about 100 us on a 20 ms wake (one or two planned ticks, the attitude fit),
perception a few us per TELEMETRY; the behaviour runner's tick every
100 ms (with `KILO_BEHAVIOUR=1`) is not estimated.
That is under 0.2 ms of a 1 ms pass, so the 1 ms sleep stays: shorter
gains nothing (the walker's grid is 20 ms, the link's deadlines 10 ms and
up), longer delays the walker's wake. The planner's real cost on the Pi 2 B
is still to be measured: the bench and this line do it at the first boot.

### Planner bench

Once, from 1 s after the first laptop client connects (so a `kilo log
--host` tail shows its lines live: the 64-line ring holds only about 15 s
of log), one call per loop pass, a `Planner` of its own (not
the walker's; nothing goes to the body) walks trot, walk and crawl as
`kernel/host/tests/bench_planner.cpp` does once each (a speed change, a
turn, a change of pattern, the planned stop), about 1400 ticks each, then
logs one line per pattern:

```
planner bench trot: start 612 us; next() x1450 mean 31.20 us max 140 us (a tick is 20000 us; Python on the Pi: max 4500 us)
```

(numbers made up.) Then the host channel's handshake ("Who may talk to
Kilo: keys"), between two fixed test keys, one X25519 per pass as the host
server runs it, and a full 1024-byte record sealed and opened 20 times:

```
noise bench: X25519 keygen 2900 us, ee 2950 us, se 2950 us (the mind's handshake, a pass each); the laptop's side 2900 + 5900 us; a 1024-byte record: seal mean 60 max 90 us, open mean 60 max 85 us
```

(numbers made up; on the laptop `make -C kernel/host bench` prints about
95 us per X25519 and 3.5 us per record.) It pauses during a walk, a trick,
a flash or the raw bridge, takes about 5 s, then frees its planner. `make -C kernel KILO_PLANNER_BENCH=0` leaves
it out (about 400 bytes). Interrupts (USB, the PL011) land inside the timed calls,
so *max* includes them, as a real walk's would.

## The laptop interface

`kilo --host IP` on the laptop talks to the kernel over the Ethernet cable,
TCP port 5300 (docs/fast-boot.md §5, `mind/README.md` "Laptop interface"):
`kilo status`, `monitor`, `log`, `posture`, `flash`, `reboot` and `update`,
each with `--host IP` ("What `--host` does on the kernel today" below).

### Who may talk to Kilo: keys

In plain words: the mind answers only laptops it knows. Each laptop has a
**key pair**, two linked numbers made once by `kilo key init`: a private
key, which stays in a file on that laptop (`~/.config/kilo/host_key`,
readable by its owner only) and is never copied anywhere, and a public key,
which may be shown to anyone. The public keys of the laptops that may talk
to Kilo are listed in `kernel/keys/authorized_keys`, in the repo, and the
build puts that list inside the kernel. When the laptop connects, it proves
it has the private key that belongs to a public key on the list, without
sending the private key; from then on everything between the two is
scrambled so that nobody else on the network can read it or change it
unnoticed. A laptop that is not on the list gets nothing at all: no
status, no log, no control, no flashing, no kernel update. Nothing on the
mind is secret: the mind has no key of its own.

To add a laptop (once per laptop):

```
kilo key init                                  # makes ~/.config/kilo/host_key; prints the public line
kilo key show >> kernel/keys/authorized_keys   # adds the line to the list
make -C kernel                                 # the list is built into kernel7.img
```

then install the new kernel: `kilo update --host IP` from a laptop the
running kernel already knows ("Updating the kernel over the network"), or
the SD card in a card reader (`make -C kernel card`, `tools/make-sd.sh`),
or for a bench run from RAM `tools/netboot.sh` (the loader takes any
kernel). `make -C kernel` stops, saying the above, while the list has no
key. A test build can use another list: `make -C kernel
KILO_AUTHORIZED_KEYS=/path/to/list`; `kilo-mind-host` reads its list at
start-up (`--authorized-keys FILE`, default the same committed file).
`kilo update` checks before sending that the image has this laptop's
public key in it, so an update cannot lock the laptop out by mistake
(`--force` skips that, as it does for the marker). `--key PATH` or
`$KILO_HOST_KEY` picks another key file; a key file that others can read
is refused (`chmod 600`), as ssh does. With no key file, every `--host`
command says to run `kilo key init`; with a key the mind does not know, it
says so and how to add it. The mind's log names each laptop that connects
by its key's fingerprint and name (`noise: client 0: handshake with key
1a2b3c4d5e6f7a8b (joe@rtb) in 9.1 ms, slowest step 2900 us`; numbers made
up) and each refused key by its fingerprint; `kilo key show` prints this
laptop's.

The technical description (the spec's `[host]` comments are the
reference):

- **Protocol**: the Noise Protocol Framework, revision 34,
  `Noise_KN_25519_ChaChaPoly_BLAKE2b`: X25519, ChaCha20-Poly1305 (IETF,
  the 64-bit counter nonce little endian after four zero bytes), BLAKE2b
  with HMAC-BLAKE2b HKDF. Pattern **KN**: the initiator (the laptop) has a
  static key the responder (the mind) knows before the handshake (`-> s`
  as a pre-message); the responder has none. Messages: `-> e`, then `<- e,
  ee, se`. The mind's side is `mind/net/noise.*` over Monocypher 4.0.3
  (`mind/third_party/monocypher/`, vendored byte for byte); the laptop's is
  the Python `noiseprotocol` package (`mind/kilo/hostsecure.py`), an
  independent implementation on purpose. Known-answer tests: the published
  cacophony and snow vectors for this exact protocol and noiseprotocol's
  output (`host/tests/test_noise.cpp`).
- **Hello** (clear, 40 bytes, then Noise message 1, 32 bytes):
  `"KILO" | version 2 | flags 0 | reserved u16 0 | the laptop's static
  public key`. The mind looks the key up in its list, comparing every entry
  whole in constant time, and answers in clear (8 bytes): `"KILO" | version
  | status | reserved`, status `OK` followed by message 2 (48 bytes), else
  `KEY_UNKNOWN`, `BAD_VERSION` or `REFUSED` (no randomness) and the
  connection is closed. A wrong magic (a version-1 client's plaintext
  frame, a port scanner) is closed at once, unanswered.
- **Prologue**: `kilo-host-2` followed by the 40 hello bytes, so changing
  any byte sent in clear makes the handshake fail.
- **Records**: after the handshake each direction is a stream of Noise
  transport messages, `len u16 LE | ciphertext | 16-byte tag`, each holding
  up to 1024 bytes of plaintext (`host.record_max`): the unchanged host
  frames (length, type, seq, payload, CRC) end to end, several to a record
  (a sender puts whole frames in a record; the receiver treats the
  plaintext as one stream). Every message of today fits a record many
  times over: the largest frame is 246 bytes (a 200-byte `UPDATE_DATA`
  chunk is 210).
- **Proof**: in KN the mind learns that the laptop holds the private key
  only from the laptop's first record, which the laptop sends at once
  (empty). Until a record from it authenticates, a connection is no
  client: it is served nothing, is not in `STATUS.clients`, cannot take
  control, and is closed 3 s after it connected (`host.handshake_timeout_ms`).
- **Failures close the connection, unanswered**: a record that does not
  authenticate (tampered, replayed, out of order, made with another key), a
  record length outside 16..1040, a nonce at 2^62 (never reached: that is
  2^62 records). There is no error message to probe.
- **Slots**: the mind serves four clients; the listener has a fifth slot,
  and a new connection always gets in: if that makes five, the oldest
  connection that is not a client yet is closed (the newest one, when four
  clients are connected). Idle sockets cannot keep a laptop out; a flood
  of new connections can (see below).
- **Scheduling**: each X25519 costs milliseconds on the Cortex-A7 (est.
  2-4 ms; about 0.1 ms on the laptop), so the mind's three (its ephemeral
  key, ee, se) run one per main-loop pass, for all connections together,
  round robin: a handshake takes three passes, and no pass does more than
  one. The boot bench measures them ("Planner bench"). A record costs a few
  microseconds per KB on the laptop; the server opens at most 1 KB of
  records and seals at most two records per client per pass.
- **Randomness**: the mind's ephemeral key comes from a new seam,
  `Random` (`mind/platform.h`): on the Pi `CircleRandom`, the SoC's
  hardware RNG (16 words per 32 bytes) hashed with BLAKE2b, refusing a
  generator stuck on one value; `getrandom()` in `kilo-mind-host`. If it
  fails, the hello is refused (`REFUSED`): fail closed. Secrets (the
  ephemeral private key, the chaining key, DH results) are wiped when used.

What this protects, and what it does not:

- It protects everything on the host channel, port 5300: who may
  connect, and the secrecy and integrity of every byte after the
  handshake, status and logs included.
- **The netboot loader is not protected.** It takes any kernel from anyone
  on the network (`tools/netboot.sh`, Circle's HTTP and TFTP servers), so
  while the bench card's loader runs, anyone on the LAN can run any kernel
  on the Pi. The production card (the mind on the card) has no loader.
- **Physical access wins**: whoever holds the SD card can write any kernel
  (and any key list) onto it; whoever has the serial wires talks to the
  body directly.
- **Kilo has no identity** (KN: the responder has no static key). A laptop
  cannot tell the real Kilo from an impostor on the network: anything that
  knows the laptop's public key (which is public) can finish the handshake
  as the responder, and then receives what the laptop sends: commands, a
  body firmware image, a kernel image, none of them secret. It gains
  nothing towards the real Kilo (every session has fresh keys, so nothing
  can be replayed there), and the laptop believes what it reports. The
  decision was that this is harmless.
- Not protected against: someone on the network who floods the port with
  new connections faster than a handshake completes (a denial of service:
  each new connection closes the oldest one still in its handshake), or
  fills the 64-line log with refusals. The public keys and the hello are
  in clear, so an observer learns which laptop connects.
- A key is revoked by removing its line, rebuilding and installing the
  kernel; there is no expiry.

### Network bring-up

Plan rule 1 (docs/fast-boot.md §3): the control loop comes first and never
waits for the network. `CKiloNet` (`src/net.*`), a Circle task, is created
last in `Initialize` and first runs at the main loop's first sleep:

1. **USB** (the default `KILO_BODY_SERIAL=pl011` build). The LAN9514
   Ethernet chip is a USB device, so Circle's USB host must scan first.
   The link is already up on the PL011, so the task waits for the main
   loop's go-ahead: the body's first HELLO (or 3 s), the boot flash check
   and any flash it starts ("Flashing the body"; at most 30 s), then scans.
   Circle's enumeration spins (`CTimer::MsDelay`, no yield), so **the main
   loop stops for the length of the scan**, about a second (est.), once.
   The body's watchdog covers it; at boot nothing is walking. The log line
   `USB scan for Ethernet: done, N ms (the main loop waited)` measures it.
   With `auto` (the USB cable), `ChooseSerial` already scanned, before the
   link, and this step is skipped.
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
between them and five connection slots (`KILO_HOST_MAX_CLIENTS` and a
spare, `kilo::kHostSlots`: "Who may talk to Kilo: keys"), each a
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

**Ring sizes**, per slot: RX 4 KB, TX 4 KB, 40 KB for five. RX: a segment is
at most `FRAME_BUFFER_SIZE` (1600) bytes and the task receives only when one
fits, so 4 KB holds two and a margin; the laptop's records are at most 1042
bytes (`host.record_max` and the length and tag) and `HostServer` reads up
to 1 KB a pass. TX:
`HostServer` keeps its own 4 KB per client and flushes what this ring takes;
the task empties it every ~1 ms into Circle's queue (64 KB more). The
streams are at most about 10 kB/s (TELEM at 50 Hz, log lines), so 4 KB is
400 ms of output. A laptop that stops reading fills Circle's 64 KB, then
these 4 KB, then `HostServer`'s 4 KB; then `HostServer` drops it (TELEM is
skipped first).

### What `--host` does on the kernel today

Everything `kilo-mind-host` serves on the laptop, from the same `Mind`:
`kilo status`, `kilo monitor` (TELEM), `kilo log` (the ring, Circle's lines
included), `kilo posture NAME [--for S]` (REST when the controller lets go
or goes silent), `kilo flash --host`, `kilo reboot --host`, `kilo update
--host` ("Updating the kernel over the network"), `kilo runs --host`,
`kilo record ... --host`, `kilo metrics --host` and `kilo bench sd --host`
with `--card DIR` ("The flight recorder") and the raw bridge (`kilo.link` over TCP; everything on
the link pauses).

`kilo reboot --host IP` takes control and sends `REBOOT`: the mind latches
REST, answers OK, and 200 ms later the kernel returns `ShutdownReboot`
(`main.cpp`: Circle's `reboot()`, the watchdog's full reset). The firmware
then boots the SD card again: on the bench card that is the netboot
loader, ready for the next `netboot.sh`, so a developer does not have to
pull the Pi's power; on the production card, the mind kernel again. The
CLI prints `REBOOT: OK (rebooting)`; a Pi that reboots sends no FIN, so
the CLI waits 2 s for a close and is done either way. Params (`PARAM_*`) answer
UNSUPPORTED. There is still no walk or trick command over `--host`: walks
come from the IR remote, tricks only from the behaviour runner (built in,
or started by the remote's A5). A laptop client holding control (`kilo posture`) silences
the remote and pauses the runner meanwhile. The distance guard (`mind/guard/`,
`docs/distance-guard.md`) is on from boot (`MindConfig::guard`); `kilo guard
on|off --host` (GUARD, any client) switches it, and GUARD_STATUS, `kilo
status --host` and the metrics' `guard:` line report it. A walk the raw bridge or a
flash cut off is aborted when the pause ends.

### Body link: two serial back ends

**The default** (`KILO_BODY_SERIAL=pl011`): the PL011, opened at once.
The Pi sits on the NyBoard's socket: the header UART (GPIO14/15) is the
body link and GPIO4 the body's reset line, and USB enumeration (about a
second or more, est.) does not come before the link (docs/fast-boot.md §3
rule 1); the net task scans USB for Ethernet later ("Network bring-up").
Kilo's USB adapter cable is retired.

**The fallback** (`make -C kernel KILO_BODY_SERIAL=auto`, for the USB
cable; `CKernel::ChooseSerial`): initialise Circle's USB host
(`CUSBHCIDevice`, one scan, no plug-and-play) and use the USB serial device
`utty1` if one enumerated; else the PL011. A USB adapter plugged in after
boot is not seen. Either way the log line `body link on ...` says which.

- **USB** (the `auto` fallback): Kilo's CH343 adapter enumerates as CDC-ACM
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
- **PL011** (the default; GPIO14/15, header pins 8 and 10; `ttyS1`): Circle's
  `CSerialDevice`, interrupt driven. Built only when chosen, because its
  constructor takes over GPIO14/15. 250000 baud is exact: UARTCLK 48 MHz /
  (16 x 250000) = 12, remainder 0, so IBRD 12, FBRD 0. The NL -> CR NL
  translation that is on by default is switched off (it would break binary
  frames). Line errors (break, overrun, framing, parity) come back from
  `Read()` once, negative, with the data still queued: counted, logged with
  the stats, and the read repeated. `tx_free()` is the TX ring's free space.
  `set_baud()` changes the rate in place ("Baud switching" below).

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

**Baud switching** (the flasher: 115200 for optiboot, then back to 250000):

- **PL011**: Circle's `CSerialDevice` has no call for a new rate, and its
  `Initialize()` must not run twice: each call adds `ttyS1` to the device
  name service again (a heap entry that is never freed) and counts the
  interrupt user again, so the destructor would never disconnect the IRQ.
  Destroying and re-creating the device is worse: the destructor
  disconnects the IRQ, turns GPIO14/15 into inputs (TXD floats, so the
  328P may see a break or garbage) and frees both rings. So
  `Pl011Serial::set_baud()` does what Circle's own `SetParity()` does for
  the line settings: wait until the TX ring and the shift register are
  empty (at most 100 ms; 2 KB at 250000 baud is 82 ms; a timeout is
  counted and shown with the line errors), then, with IRQs off for a few
  register writes, disable the UART, write IBRD/FBRD, write LCRH back (the
  PL011 latches the divisor on that write), enable it. The interrupt
  mask, the FIFOs, the pins and both rings stay as they are; `tx_free()`
  means the same before and after. Divisors from 48 MHz: 115200 is IBRD
  26, FBRD 3 (115176 baud, -0.02 %); 250000 is 12, 0 (exact). Optiboot's
  own 115200 on the 16 MHz 328P (U2X, UBRR 16) is 117647 baud, +2.1 %, as
  under avrdude on Linux. A byte arriving during the few microseconds the
  UART is off may be lost or garbled; the flasher drops what comes in
  around a switch anyway. Risk: the PL011 TRM wants the UART idle before
  its control registers change; the drain and the BUSY wait give that for
  TX, but a byte being received at that moment is lost.
- **USB serial**: `UsbSerial::set_baud()` sends CDC `SET_LINE_CODING`
  (Circle's `SetBaudRate()` and `SetLineProperties()`): a synchronous
  control transfer. Writes are synchronous too, so nothing is queued on
  the Pi's side; whether the CH343 sends its own FIFO out at the old rate
  first is the adapter's business (not checked). **But flashing does not
  work over the USB cable**: the body's RESET is wired to the adapter's
  DTR line there, and Circle's CDC driver raises DTR once when it
  configures the adapter and has no call to pulse it. The flasher pulses
  GPIO4 (header pin 7), so over USB it ends in `SYNC_TIMEOUT` (link restored)
  unless pin 7 is also wired to the NyBoard's reset line. Pulsing DTR
  would need a CDC `SET_CONTROL_LINE_STATE` request from a patched or
  subclassed Circle driver.

### ACT LED

| Pattern | Meaning |
|---|---|
| steady on | starting up (with `KILO_BODY_SERIAL=auto`, the USB scan: a second or more) |
| 0.5 Hz (1 s on, 1 s off) | no body: no HELLO yet, or no TELEMETRY for 300 ms |
| 2 Hz | the body is alive: TELEMETRY flowing |
| 5 Hz | the body's HELLO has another protocol major: the kernel stays up and keeps the heartbeat, but sends no goals or FEET (kilo-mind-host exits 2 instead) |
| double flash (two 100 ms flashes, once a second) | flashing the body, from the laptop or the SD card, until the new firmware's HELLO (or FAILED); or writing a kernel update to the card (the flashes stretch: the loop stalls on each 16 KB slice) |
| steady on or off for more than ~20 s | stopped: `Initialize` failed or a crash (Circle halts, the LED freezes) |

The LED says nothing about the network (the log does). The blink freezes
for a moment while the SD card is read (after the first HELLO) and while a
`--persist` writes to it; with the default `KILO_BODY_SERIAL=pl011` it
also freezes once for about a second (est.) early on: the USB scan for
Ethernet ("Network bring-up").

### The Mind's settings

`MindConfig` as the Pi's daemon ran (`kilo run --remote --balance 0.99`,
`KILO_RUN_ARGS` in docs/setup.md and `mind/systemd/env.example`). There is
no settings file on the card, so they are compiled in (`src/kernel.cpp`):

| Setting | Kernel | Why |
|---|---|---|
| `mind_build` | `KILO_BUILD` | `STATUS.mind_build`, the commit's 8 hex digits |
| `remote` | on | `--remote`: the IR remote drives Kilo; logged `IR remote on` |
| `balance_kp` | 0.99 | `--balance 0.99`: SET_BALANCE on at start, logged `balance on by default (Kp 253/256)` |
| `behaviour` | off; `make -C kernel KILO_BEHAVIOUR=1` turns it on | the daemon's default; the remote's A5 starts the runner at run time either way. While it runs, the remote's keys other than B6, A7 and C7 (which stop it) are ignored. Built in, the kernel logs `Kilo mind, build ... (behaviour runner on)` |
| `rear_up_key` | 0 (off); `make -C kernel KILO_REAR_UP=1` (or `2`, `3`) turns it on at that stage | the remote's B2 starts the rear-up trick only when switched on (docs/rear-up-trick.md "The B2 switch"): off until the owner's staged trials are done, B2 does nothing like the other keys of rows 1-3. The value is the stage: 1 `rear_up` (28 degrees), 2 `rear_up_2` (40), 3 `rear_up_3` (54). The daemon's switch is `KILO_REAR_UP=N` in its environment, `kilo-mind-host`'s is `--rear-up[=N]`. Built in, the kernel logs `Kilo mind, build ... (rear-up key B2 on: rear_up_2, stage 2)` |
| `seed` | 32 bits from the hardware RNG | the behaviour runner logs it (`seed N`, at boot or in the remote's A5 line), so a run can be replayed on the laptop |
| `stats_period_ms` | 1000 | a STATS_REQ to the body each second, logged `body STATS: ...`, as `kilo-mind-host` does |

The runner's wind-down ends in a poweroff, which the daemon ran through
systemd. The kernel cannot switch the Pi off: it logs `behaviour runner
asked for poweroff ...` once and keeps the link up (the runner has stopped).

### Boot probe

GPIO21, header pin 40 (pin 39 is ground), outside the NyBoard socket's pins
1-10. High at the top of `main()`, low at the first HELLO, for a scope
against the 5 V rail (docs/fast-boot.md P6). On by default;
`make -C kernel KILO_BOOT_PROBE=0` leaves the pin alone. The SD card is
read only after the first HELLO, so the probe never includes it, nor a
boot-time flash.

### Flashing the body

Phase P4 (docs/fast-boot.md §6; the core's side in `mind/README.md`
"Flashing"). The flasher (`mind/flash/flasher.*`) programs the 328P
through optiboot on the body link's UART, resetting it with GPIO4.

**From the laptop**, `kilo flash --host IP [--persist]`: `HostServer`
fills a 32 KB image from `FLASH_BEGIN`/`FLASH_DATA`, starts the flasher at
`FLASH_COMMIT` and polls it. While the flasher has the UART,
`host.link_paused()` is true and `Mind::poll()` polls nothing on the
link (the link, walker, trick player, remote, runner), as for the raw
bridge; a walk that was on (from the remote) is aborted when the pause
ends (HostServer starts the flash inside `poll()`, so there is no pass in
which to abort it first). The boot flash from the SD card aborts a walk
before it starts the flasher.
The kernel logs the end: `body flashed from the laptop: build ..., HELLO
build ... reset FLASHED`, or `body flash from the laptop FAILED: ...`.

**From the SD card at boot** (§6 path B, `kilo flash --sd MOUNT` or
`make-sd.sh --body DIR`): once the body's first HELLO has arrived, or 3 s
after `link.connect()` with none (no firmware, a broken one, or the body
is off), the main loop mounts the card's FAT partition, reads `body.json`
(`parse_manifest`) and `body.bin` (`load_from`: size and CRC), and asks
`needs_flash(hello or null)`. Another build: the flasher starts with that
image (no `--persist`), the host server polls it, and the kernel logs
`body flashed from the SD card (boot): ...`. The same build: nothing
(`boot flash check: the body runs build X, the card's: no flash`). A
missing or bad file is not an error: one Info line (`SD: no body.json:
nothing to flash at boot`, `... does not match ...`) and the boot goes on.
With no HELLO and a good image, the kernel flashes: with the body
switched off that fails in sync (two attempts, about 3 s) and the link is
restored; the check runs once per boot. The SD image has its own 32 KB
buffer, separate from the laptop's, so a laptop sending an image at that
moment cannot have it overwritten (its `FLASH_COMMIT` is refused
`FLASHING` while the boot flash runs).

**Order at boot**: link up -> HELLO (or 3 s) -> SD card -> flash if needed
-> the go-ahead for the net task's USB scan (the default
`KILO_BODY_SERIAL=pl011`). The scan stalls the main loop for about a
second (est.), and optiboot gives up after 1 s without a byte, so it must
not cut into a flash; it waits for the go-ahead (at most 30 s). With the
`auto` build (the USB cable) the scan came before the link, so nothing
waits.

**`--persist`**: after a DONE from a `FLASH_BEGIN` with `PERSIST`, the
kernel removes `body.json`, writes `body.bin`, then writes `body.json`
(`format_manifest`; manifest last, as `kilo flash --sd` does, so a power
cut never pairs a manifest with half an image; the CRC is checked at boot
anyway), then calls `persist_done()`. `persist_done()` is called whatever
happened: a card that is missing, write-protected or failing is logged
(`--persist: the image is NOT on the SD card; the body runs it anyway`)
and the next `FLASH_BEGIN` is accepted, rather than refused `FLASHING`
forever. The laptop does not learn the outcome (it prints "the mind keeps
the image on SD" when it asked): check `kilo log --host` for `SD:
--persist: wrote body.bin ...`. A power cut during the write can still
damage the FAT on the boot partition (FatFs updates the FAT and the
directory entry in place), which could stop the card from booting; the
write takes a fraction of a second right after a flash.

**SD card access**: Circle's FatFs addon (`addon/fatfs`, long file names,
read and write; Circle's own `lib/fs/fat` reads 8.3 names only, and
`body.json` is not one) over its SD driver (`addon/SDCard`; on the Pi 2 B
the SDHOST controller, PIO). Mounted read-write, at the first use; nothing
is written except by `--persist`. The SD driver busy-waits (Circle's
`NO_BUSY_WAIT` is off; turning it on changes the USB driver too), so every
card access stalls the main loop, logged: `SD: FAT partition mounted in
N ms (card M ms ...)`, `SD: body image ... (read in N ms)`, `SD: --persist:
wrote ... in N ms`.

**Boot time**: card initialisation (the GPU firmware already started the
card, but Circle resets it: CMD0, then ACMD41 until it is ready) about
50-300 ms (est.), the mount a few sectors, a 31 KB read at 25 MHz 4-bit
PIO a few ms: about 0.1-0.4 s (est.). Before `connect()` that would delay
the link and the boot probe; after the first HELLO it delays neither, only
the loop for that long once, with nothing walking. A HELLO that arrives
meanwhile waits in the PL011's 2 KB RX ring. So the read is after the
HELLO. A flash, when needed, takes about 10 s (est.) plus the new
firmware's boot.

### Updating the kernel over the network

`kilo update --host IP [--image PATH] [--force]` writes a new mind kernel
onto the SD card the Pi booted from, and the Pi boots it from the card from
then on, with no network. The image is `kernel/build/kernel7.img` by
default. The core's side is `mind/update/updater.*` (`mind/README.md`
"Updating the kernel"); the card's is `CBootCard` as the core's
`KernelStore` (`src/bootcard.*`).

1. **Receive.** `UPDATE_BEGIN` (size, CRC-32, flags), the image in
   `UPDATE_DATA` chunks into a 4 MB buffer on the heap (the largest image
   it takes; the kernel is about 512 KB), `UPDATE_COMMIT`. The link runs on
   meanwhile; the laptop channel reads 1 KB a pass, so 512 KB take about a
   second.
2. **Check** at `UPDATE_COMMIT`: all bytes in, the CRC-32 matches, and the
   string `KILO-MIND-KERNEL-1` is in the image (`KiloMindKernelMarker` in
   `src/kernel.cpp`; the start-up log line prints it). The netboot loader and
   any other file lack it and are refused `IMAGE_BAD` (`NOT_A_KERNEL`):
   nothing is written. `--force` (flag `FORCE`) skips this check only: it is
   how the loader goes back on the card. Check with `grep -ac
   KILO-MIND-KERNEL-1 kernel/build/kernel7.img` (a count above 0) and the
   same on `kernel/build/netboot/kernel7.img` (0).
3. **Write** `kernel7.new`, **read it back** and compare its length and
   CRC-32, then **install**: remove `kernel7.old` if there is one, rename
   `kernel7.img` to `kernel7.old`, `kernel7.new` to `kernel7.img`. FatFs
   writes the file's data, FAT and directory entry by `f_close`, and
   `f_unlink`/`f_rename` sync the volume before they return. Every step is
   logged (`update: ...`, with its time in ms), and `UPDATE_STATUS` reports
   each phase and slice to the laptop.
4. **Reboot**, as `kilo reboot` does: REST to the body, the DONE status out,
   200 ms, `ShutdownReboot`; the firmware boots the new `kernel7.img`. The
   CLI prints the result and `the mind is rebooting into it`, then polls
   port 5300 (`--wait S`, 30 s by default) and prints the new mind's
   `mind build`.

**Failures leave the card bootable.** Anything before the renames
(creating, writing or reading back `kernel7.new`; `WRITE_FAILED`,
`VERIFY_FAILED`) removes `kernel7.new` and leaves `kernel7.img` as it was;
so does a failed removal of `kernel7.old` or a failed first rename
(`INSTALL_FAILED`). If the last rename fails, `kernel7.old` is renamed back
to `kernel7.img` (`INSTALL_FAILED`). Only if that fails too is the card left
without `kernel7.img` (`CARD_BROKEN`, logged as an error): then, and after
a bad kernel that boots but does not work (the check proves the file is a
mind kernel, not that it runs), the card goes into a reader
(docs/fast-boot-first-boot.md step 13, "If it fails").

**The SD writes are sliced**: the updater writes or reads 16 KB per main
loop pass (`KernelUpdater::kSliceBytes`), so the net task runs between
slices and the laptop sees progress. Each slice still stalls the loop (the
SD driver busy-waits), so from `UPDATE_COMMIT` to DONE `link_paused()` is
true, as during a flash: the link, the walker and the rest pause, the body's
watchdog covers the silence (stop, then REST after 1.5 s), and a walk cut
off is aborted when the pause ends. For a 512 KB kernel that is 32 slices
written and 32 read back: about 0.5-2 s (est., depending on the card's
write speed; the log's `update: kernel7.new written: ... in N ms` and
`read back: ... in N ms` measure it), then the 200 ms before the reboot.
POSTURE, PARAM_SET, REBOOT, the bridge and a body flash are refused
`UPDATING` meanwhile, and an update is refused `FLASHING` while a body
flash runs or its `--persist` image waits for the card.

### The flight recorder

Like an aircraft's flight recorder, the mind records everything from the
moment it starts, whether or not Kilo is walking (`mind/record/`,
`mind/README.md` "The flight recorder"):

- every body link frame both ways: each TELEMETRY (attitude, mode, flags,
  battery, distance), EVENT (keys, falls, TRICK_DONE), JOINTS if the body
  streams them, and every command the mind sends (FEET, postures,
  TRICK_PLAY, TRICK_STOP); the HEARTBEATs are only counted;
- every log line, the routine once-a-second ones included;
- during a walk, each planned tick and the walk's parameters (as before);
  during a trick, the trick, its keyframes' timing, the moment the body
  started playing it, the remote keys pressed meanwhile and how it ended.

Walks and tricks are *runs*: stretches of that one stream, opened and
closed by a RUN record and listed by `kilo runs --host IP` (`kilo walks`
still works). The laptop can fetch a run (`kilo runs --host IP --fetch N`,
from 2 s before it to 2 s after) or any stretch of time (`kilo record
fetch --host IP --last 10m`), and the analysis tools read either.

The stream is kept in RAM in 16 KB *blocks*: a block has a header (its
number, the mind's clock at its start, its length, a CRC-32 checksum of
its contents) and records, each with its time in microseconds from the
block's start. When the RAM is full the oldest block goes. `Initialize`
allocates 64 MB once on Circle's heap (`record::kStoreBytes`): at about
2 KB/s while Kilo stands and 6 KB/s while it walks, that is between three
and nine hours. A record costs one copy of at most 255 bytes into RAM and
the block's running checksum (`make -C kernel/host bench`: about 1 us a
walk tick on the laptop, so some 10-20 us on the Pi). The log says
`flight recorder: 64 MB of RAM, 16 KB blocks; log history 4096 lines` at
start.

#### Saving to the SD card

The saver (`mind/record/saver.h`) writes the recorder to the card's FAT
partition, in the directory `kilorec/`, so a power-off loses only what was
not saved yet. The card's write times are the danger: Circle's SD driver
busy-waits, so every write stops the main loop, and the body's watchdog
(500 ms without a heartbeat) must never fire because of it. So the saver:

- writes only at a **safe moment**: no walk, trick or run's 2 s post-roll,
  the body not walking, playing a trick, fallen or recovering, no flash,
  update or bridge, the behaviour runner not running, and all of that for
  2 s;
- makes **one card call a pass**, a write of at most 4 KB, so the
  heartbeats keep going between them;
- **times every call** and keeps the slowest (`kilo record status`,
  `kilo metrics`); a call slower than 100 ms logs a warning and makes it
  wait 30 s (doubling, up to 10 minutes) before the next; five failures in
  a row stop it until `kilo record save on`;
- writes each file under a `.tmp` name and renames it when it is whole, so
  a power cut in the middle leaves a `.tmp`, which the next boot deletes;
- keeps at most 64 MB on the card: the oldest files go first.

What a save writes: the blocks sealed since the last save
(`b00007-s0000123.krc`: boot 7, blocks from 123 on), the boot's run index
(`b00007.run`) and the new log lines with the session metrics
(`b00007.log`). It saves after every run (the run's last block is sealed
for it), and while idle when 8 blocks wait or 5 minutes have passed. Boot
numbers count up from the highest found on the card at start.

**Saving is off** until the card has been measured: `make -C kernel
KILO_RECORD_SAVE=1` turns it on from the start; otherwise `kilo record
save on --host IP` turns it on and remembers that on the card
(`kilorec/SAVE_ON`), so later boots save too; `kilo record save off`
undoes it. First measure the card, with Kilo limp (`kilo posture rest`, or
the servos off): `kilo bench sd --host IP` writes 32 writes each of
512 B, 4 KB and 16 KB to `kilorec/bench.tmp`, then syncs, closes and
deletes it, and prints the shortest, mean and longest time of each, and of
the sync, close and delete. If the longest 4 KB write is well under
100 ms, saving is safe to turn on.

How much a power-off loses: with saving on, what was recorded since the
last save: at most the last run if Kilo is switched off within about 2 s
of it ending (before the safe moment), otherwise the last few minutes of
standing still (at most 8 blocks, 128 KB). `kilo record status` shows it
(`unsaved`).

On boot the saver reads `kilorec/` once, at the first safe moment: it
deletes torn files, takes the next boot number, and loads the earlier
boots' run indexes, so `kilo runs --host IP` lists their runs next to this
boot's (an earlier boot's run has the id `boot << 20 | id`) and fetches
them from the card. `kilo log --host IP --previous` shows an earlier
boot's saved log; `kilo runs --host IP --delete BOOT` (or `all`) deletes
an earlier boot's recordings. Reading the card back also waits for safe
moments, one call a pass.

`CBootCard`'s second object, `CRecordCard`, gives the saver its own FatFs
file and directory objects, so the kernel updater and the body image calls
never share a handle with it.

### The log

The mind's log (`mind/util/logring.h`) keeps 4096 lines of history: what
happened (remote keys, walks, tricks, refusals, warnings, errors, the
network coming up). The routine lines (the once-a-second `link`, `body`
and `loop` counters, the body's STATS, percepts, a held key's repeats)
are never history: a laptop reading the log gets them live, and the
flight recorder keeps them. `kilo log --host IP` shows the whole history
first and how many older lines were dropped, then follows; `--save`
keeps a copy in `~/kilo-logs`.

Why it was lost before: the ring held only the last 64 lines, and
the status lines went into the same 64 lines whenever a laptop read the
log (three a second). A 15-minute session with 8 walks makes about 100
history lines (`kernel/host/tests/test_record.cpp`
`test_play_session_log`: a walk makes four, a remote key one or two), and
`kilo log --host` asked for only the last 20 by default. With the cable
in, the network's lines (Circle's DHCP and Ethernet lines, the address,
mDNS), each laptop connection's three or four lines and the planner bench's
lines came on top, and once a client watched the log, 64 lines were
twenty seconds of counters: the session's keys and walks were gone.

### mDNS

Kilo's address comes from DHCP and changes from day to day, so the laptop
finds him by name: `kilo find`, or `--host kilo.local` on any `--host`
command (`kilo runs`, `record`, `bench`, `metrics`, `reboot` and `update` look for
`kilo.local` by themselves when `--host` is left out). That is mDNS
(multicast DNS): the laptop sends "who is kilo.local?" to everyone on the
wired network at once (the group address 224.0.0.251, UDP port 5353), and
the kernel answers with its address. The laptop side is `mind/kilo/mdns.py`
(standard library only: the server has no avahi or libnss-mdns, which is
what normally resolves `.local` names).

What the kernel answers is the core's responder, `mind/net/mdns.h`
(tested on the host: `kernel/host/tests/test_mdns.cpp`, and against
`kilo find` in `test.sh` through `build/mdns-host`):

| asked | type | answer |
|---|---|---|
| `kilo.local` | A | the address of the moment |
| `_kilo._tcp.local` | PTR | `kilo._kilo._tcp.local` (the laptop service), with its SRV, TXT and A |
| `kilo._kilo._tcp.local` | SRV, TXT | port 5300 on `kilo.local`; `build=<hash>` (the mind's build) |
| `_services._dns-sd._udp.local` | PTR | `_kilo._tcp.local` (for service browsers) |

- **Who gets the answer.** A query from a port other than 5353 (a simple
  resolver's "legacy unicast" query, as `kilo find` sends) is answered
  straight back to that address and port, with the query's ID and
  questions. A query from port 5353 asking for a unicast answer (the QU
  bit) is answered to the sender; any other on the multicast group.
- **The name.** Once it has an address the responder probes for 0.75 s
  (three queries for `kilo.local`); if another host answers for it, it
  takes `kilo-2`, then `kilo-3`. Then it announces its records three
  times, a second apart. A new address (the link-local fallback, then a
  lease) is announced again the same way. `kilo find` also finds a Kilo
  that had to take `kilo-2`, through the service.
- **Cost.** The `kilomdns` task owns one UDP socket on port 5353 (bound,
  joined to the group, not connected, so it hears queries from any port).
  It never blocks: `ReceiveFrom` with `MSG_DONTWAIT`, at most 8 datagrams
  a pass, then a 5 ms sleep; at most 20 replies a second whatever the
  network sends. It logs the name it claimed and the address, a name
  conflict, and the first query it answered (where from: evidence that a
  laptop's query reached it), never a line per query.
- **Security.** Finding gives only an address. Anyone on the network could
  answer for `kilo.local`; the host channel's handshake still checks that
  the mind at that address has a key this laptop trusts, and the mind
  still checks the laptop's.

Before this responder the kernel used Circle's (`CmDNSDaemon`,
`CmDNSPublisher`), which answered only `kilo.local` A queries sent from
port 5353 and only on the group, and announced the service without
answering for it. `kilo find` still finds such a kernel (it also asks from
port 5353 and listens on the group), without the build in the answer; it
then asks the mind for its build over the host channel.

`make -C kernel KILO_MDNS=0` leaves the task out.

### Floating point

`MIND_SRCS` are compiled with Circle's flags (`-mcpu=cortex-a7
-mfpu=neon-vfpv4 -mfloat-abi=hard`; `readelf -A` on the objects: VFPv4,
VFP-register arguments). The planner, perception and the behaviour layer
are the core's floating point (`mind/README.md`, "The planner",
"Perception", "Behaviour"); their objects are built with
`-ffp-contract=off -fno-math-errno` as on the host (which builds
everything so): no fused multiply-add, so the planner's prediction equals
what it computes, and perception and behaviour, in double, round as
Python does. A probe
compiled with the same flags turns float arithmetic into VFP instructions
(`vmul.f32`, `vfma.f32`, `vdiv.f32`) and calls `sqrtf`, which in the
toolchain's hard-float `libm` (`thumb/v7ve+simd/hard`) is a wrapper around
`vsqrt.f32` (the wrapper sets `errno` for negative inputs;
`-fno-math-errno` would inline `vsqrt.f32`).

### Not yet tested on hardware

The flight recorder: the 64 MB store on Circle's heap, its cost per pass
on the Cortex-A7 (the loop line's `mind.poll`), a download over the cable
during a walk and after it (`kilo runs --host`), and the recordings of a
real walk or trick (the IMU's real attitude against the plan: the virtual
body's IMU is scripted, so the lag and the error by phase have only been
checked on synthetic data, `mind/tests/test_walks.py`). The card saver:
every number about the card (`kilo bench sd` gives them), FatFs's
`f_mkdir`, `f_opendir`, `f_lseek`, `f_sync` and `FA_OPEN_APPEND` on
Circle, whether a write's stall shows in the loop line as expected, and a
real power cut in the middle of a save (the tests cut a fake card).

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
`kilo reboot` (the OK reaching the laptop in 200 ms; the loader coming back);
the loop's period with the net task running (the stats line); the USB scan
from the net task (the default `pl011` build) and how long it stalls the
loop; the planner's cost on the Cortex-A7 (the bench line). The keys:
the X25519 steps' time on the Cortex-A7 (the noise bench line, and the
handshake line of each connection), the hardware RNG through
`CircleRandom`, `kilo --host` over Noise to the Pi.
Flashing: the PL011 rate switch (`set_baud()` in place) at 115200
against optiboot and back; GPIO4 reaching the NyBoard's reset line (header
pin 7 in the socket; on the bench a fourth wire); the `optiboot in sync N
ms after release` timing on the real 328P; a whole flash's time; the SD
card: Circle's SDHOST driver on Kilo's card, how long initialisation,
mount and the 31 KB read take (the log lines), FatFs writing body.bin and
body.json and the laptop then reading them; the boot flash check with a
matching build, another build and no HELLO; `--persist`; the LED's double
flash; the USB scan waiting for the go-ahead (`pl011` build). Kernel
updates: FatFs writing, reading back and renaming `kernel7.*` on Kilo's
card, the slices' time, the 4 MB heap buffer, the reboot into the new
kernel and `kilo update` seeing it come back. The
`Mind` on the Pi: IR remote keys through the kernel (the body's
`EVENT IR_KEY` reaching the remote; walks and postures from the keys);
tricks from the compiled-in library (upload and play on the real body);
perception's percept lines from the real IMU and ultrasonic; balance on
at start (Kp 0.99); the body's STATS line each second; `mind.poll`'s
cost; the behaviour runner with `KILO_BEHAVIOUR=1` (never run on the
kernel: its 10 Hz tick, the liveness gate). mDNS:
Circle's responder and publisher on the 2 B, `kilo.local` from Debian, a
second announcement after an address change, and whether the responder's
multicast membership (joined from the first address) survives it.
`kernel/host/test.sh` covers the core, the link (including `tx_free()`
deferral), `HostServer` over POSIX sockets, the flasher against a fake
optiboot, the walker, tricks, perception, the remote, the behaviour
runner and the `Mind` (`kilo-mind-host`) on Linux only; `src/*` is
compiled but has never run.

## Open questions

- **Finding the loader without a monitor.** DHCP address from the router's
  lease table today. The mind kernel has the link-local fallback (P2,
  above); the loader does not, so on a direct cable it gets no address.
  The loader could also blink its address on the LED or announce it over
  the network.
- **Flashing over the USB cable** (the `auto` fallback). Needs a DTR
  pulse (CDC `SET_CONTROL_LINE_STATE`), which Circle's driver cannot send
  ("Baud switching"). On the cable it needs header pin 7 wired to the
  reset line; on the NyBoard's socket (the default) pin 7 is that line.
- **`--persist` outcome on the laptop.** `kilo flash --host --persist`
  does not hear whether the write worked; a FLASH_STATUS field or a log
  line the CLI waits for would tell it.
- **`kilo flash --rollback`** (keep the last good image on SD): not
  designed (mind/README.md).
- **A power cut during a kernel update.** Before the renames the card's
  `kernel7.img` is untouched, but FatFs updates the FAT in place, so a cut
  while `kernel7.new` is written could still damage the partition. Between
  the two renames (milliseconds) a cut leaves no `kernel7.img`. Either way:
  a card reader. A kernel that boots but does not work cannot fall back to
  `kernel7.old` by itself: the GPU firmware boots `kernel7.img` only.
- **Loader on the 2 B.** Not run on hardware yet (docs/fast-boot.md §7). If it
  fails, write `kernel7.img` to the card with `make-sd.sh --kernel`.
- **Licence.** Circle is GPLv3, so any `kernel7.img` that links it is too,
  and the files in `netboot/` are copies of Circle's and keep its licence.
  The rest of the repository is MIT. This matters only if images or the
  kernel sources are given to others.
