# Kilo kernel

The mind as a bare-metal kernel for the Raspberry Pi 2 B, built on
[Circle](https://github.com/rsta2/circle) (docs/fast-boot.md). Everything is
cross-compiled on the laptop; nothing is built on the Pi. Phase P0: the
toolchain, Circle, a network boot loader and a hello-world kernel that blinks
the ACT LED.

| Path | What |
|---|---|
| `circle/` | Circle, a git submodule pinned to `Step51.1.1` |
| `src/` | the mind kernel (`kernel7.img`); today hello world |
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
| `build/kernel7.img` | the mind kernel | 69,668 bytes |
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
image, power-cycle the Pi so the loader starts again. The hello-world kernel
blinks the ACT LED (green, GPIO47) at 2 Hz once it is running.

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

The hello-world kernel follows the same rules: no `CSerialDevice`, log to the
screen if there is one, else to a null device.

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

## Open questions

- **Body link wiring.** Today the body reaches the Pi through the USB CH343
  adapter (`/dev/ttyACM0`, CDC-ACM). Under Circle that is
  `CUSBSerialCDCDevice` (`lib/usb/usbserialcdc.cpp`), which depends on USB
  enumeration and so comes up after the network stack's USB. The plan's
  target is the PL011 on GPIO14/15 (`CSerialDevice`), up in microseconds and
  free of USB. P1 decides; the PL011 needs the Pi on the robot.
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
