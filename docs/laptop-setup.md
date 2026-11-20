# Setting up a laptop to update Kilo

This is for a laptop that has never built Kilo's software (written for the
GPD Pocket 4). After it, the laptop can build both of Kilo's programs and
put them on Kilo's SD card in its card reader, with no network cable:

- the **mind kernel**, the program the Raspberry Pi runs (`kernel7.img` on
  the card);
- the **body firmware**, the program on Kilo's own board, the NyBoard (its
  ATmega328P chip). The laptop puts it on the card as `body.bin` and
  `body.json`, and at the next power-on the Pi copies it into the board
  itself ("flashing"), then writes what happened to `body-result.txt` on
  the card.

The steps assume Linux (Debian or Ubuntu). The GPD Pocket 4 runs Debian
inside WSL (Windows' built-in Linux): every step is the same there except
where the card and the network cable are concerned, because those belong
to Windows. Read "Windows and macOS" near the end before the card route. Commands go in a terminal; `$`
lines are what you type, without the `$`.

## What the laptop needs

| What | Why | Version used on the server |
|---|---|---|
| git | to get the code | any |
| the Arm GNU Toolchain (`arm-none-eabi`) | the compiler for the Pi's kernel | 15.2.Rel1, in `~/opt` |
| arduino-cli and the Arduino AVR core | the compiler for Kilo's board | arduino-cli 1.5.1, core `arduino:avr` 1.8.8 (it brings avr-gcc 7.3.0-atmel3.6.1-arduino7) |
| make, a C++ compiler | to drive the kernel build | Debian 13's |
| Python 3.11 or newer with venv | the `kilo` command | 3.13.5 |
| Claude Code | to do the work with Claude | |

About 3 GB of disk space, most of it the two compilers.

## 1. Get the code

```
$ sudo apt install git make build-essential python3 python3-venv curl
$ git clone git@github.com:josephburnett/kilo.git ~/OpenCat
$ cd ~/OpenCat
$ git submodule update --init kernel/circle
```

The last line fetches Circle, the library the Pi's kernel is built on, into
`kernel/circle`. Keep the folder name `~/OpenCat`: the Pi's old setup files
expect it, and it decides the name of Claude's notes folder (section 6).

## 2. The compiler for the Pi's kernel

The Arm GNU Toolchain, unpacked in `~/opt` (no root needed). The kernel's
Makefile looks for it there.

```
$ mkdir -p ~/opt && cd ~/opt
$ curl -LO https://developer.arm.com/-/media/Files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi.tar.xz
$ curl -LO https://developer.arm.com/-/media/Files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi.tar.xz.sha256asc
$ sha256sum -c arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi.tar.xz.sha256asc
$ tar xJf arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi.tar.xz && rm arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi.tar.xz*
$ cd ~/OpenCat
```

The check line must print `OK`. If the toolchain is somewhere else, give
its folder to every build: `make -C kernel TOOLCHAIN=/path/to/it`, or
`kilo card PATH TOOLCHAIN=/path/to/it`.

## 3. The compiler for Kilo's board

arduino-cli, the Arduino command-line tool, and the Arduino AVR core (the
compiler for the board's chip). Kilo's firmware is built with arduino-cli,
not PlatformIO.

```
$ mkdir -p ~/.local/bin
$ curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh -s 1.5.1
$ ~/.local/bin/arduino-cli core update-index
$ ~/.local/bin/arduino-cli core install arduino:avr@1.8.8
```

## 4. Python and the `kilo` command

A Python environment inside the clone, `.venv` (the repository ignores
it), with the `kilo` command installed from `mind/`:

```
$ cd ~/OpenCat
$ python3 -m venv --system-site-packages .venv
$ .venv/bin/pip install -e mind pytest
```

Then put the tools on the `PATH` for this terminal (add the line to
`~/.bashrc` to keep it):

```
$ export PATH="$HOME/OpenCat/.venv/bin:$HOME/.local/bin:$PATH"
```

## 5. Check the build

```
$ make -C kernel
$ body/tools/size_check.sh
```

The first builds Circle and the kernel (about a minute the first time) and
ends with a line like `WC build/kernel7.img => 973148`. It stops if
`kernel/keys/authorized_keys` has no key in it; the clone has the
server's, so it should not. The second builds the body firmware and checks
that it fits the chip; it ends with `size_check: OK`.

Optional, the full checks (a few minutes): `kernel/host/test.sh`,
`protocol/test.sh`, `body/tools/host_tests.sh` and
`(cd mind && ../.venv/bin/python -m pytest -q)`.

## 6. Claude: its notes and the Kilo mod

**Claude's working notes** for this project (what the owner asked for,
measurements, decisions; Claude calls them memory) live outside the
repository, in Claude Code's own folder on the server:
`~/.claude/projects/-home-joe-OpenCat/memory/`. The folder's name is the
path of the clone with every character that is not a letter or a digit
turned into `-`: `/home/joe/OpenCat` became `-home-joe-OpenCat`. On the
laptop it is the laptop's clone path, turned the same way. For a clone at
`/home/USER/OpenCat` that is `-home-USER-OpenCat`. Copy the notes over
(`SERVER` is how the laptop reaches the server over ssh, `USER` the
laptop's user name):

```
$ mkdir -p ~/.claude/projects/-home-USER-OpenCat
$ scp -r SERVER:.claude/projects/-home-joe-OpenCat/memory ~/.claude/projects/-home-USER-OpenCat/
```

To check the name, start `claude` once in `~/OpenCat` and quit: Claude
Code makes the folder, and `ls ~/.claude/projects/` shows it. If the
names differ, move `memory/` into the one Claude made.

**The Kilo mod** (`tools/kilo-hud`) shows Kilo's state beside the chat
(whether he answers on the network, his battery, whether an update is
ready). Start Claude Code with it from the clone:

```
$ cd ~/OpenCat
$ claude --plugin-dir tools/kilo-hud
```

With no network to Kilo it shows him as not answering; that is expected on
the card route.

## The card route, step by step

1. **Switch Kilo off** and take the SD card out of the Pi.
2. **Put the card in the laptop's reader.** The desktop mounts it. Find the
   folder with `lsblk -o NAME,SIZE,FSTYPE,MOUNTPOINT`: it is the small
   `vfat` partition, usually `/media/USER/` and a name, holding
   `kernel7.img` and `config.txt`. Below, `CARD` stands for that folder.
3. **Write both programs:**

   ```
   $ cd ~/OpenCat
   $ kilo card CARD
   ```

   It builds the kernel and the body firmware (a few minutes the first
   time), checks the kernel (that it is the mind kernel, and that the
   laptop's key is built in if the laptop has one), copies both onto the
   card, reads each back to compare, and keeps the previous kernel as
   `kernel7.old`. It ends by saying what the next power-on will do.
   Kernel build options go after `CARD`, for example
   `kilo card CARD KILO_REAR_UP=1` (the options are listed at the top of
   `kernel/Makefile`). `--no-build` copies the last builds instead;
   `--no-kernel` or `--no-body` leaves that one as it is.
4. **Eject the card** (the file manager's eject button, or
   `udisksctl unmount -b /dev/sdX1` with the name `lsblk` showed), then
   put it back in the Pi.
5. **Switch Kilo on and watch.** The Pi's green light (ACT) is steady while
   it starts, then blinks slowly until Kilo's board answers. If the board
   runs another firmware than the card's, the Pi flashes it: the green
   light gives **two short flashes a second** for about 10 to 20 seconds
   (an estimate: never timed on Kilo) and Kilo's board restarts, as when he
   is switched on. Then Kilo **trills** if it worked, or **hisses** if the
   flash failed or was refused (battery too low, or the files on the card
   damaged). With the same firmware already on the board, nothing happens
   and there is no sound. Afterwards the light blinks twice a second: the
   board is talking to the Pi.
6. **Read what happened** whenever convenient: switch Kilo off, card into
   the laptop, then

   ```
   $ kilo card CARD --status
   $ kilo card CARD --fetch
   ```

   `--status` shows `body-result.txt` (flashed, failed, refused or same,
   why, the board's firmware before and after, the battery, the log lines
   of the flash), the last lines of the saved log, and the recordings on
   the card. `--fetch` copies the recordings to `~/kilo-logs/`, where
   `kilo log run FILE`, `kilo log walk FILE`, `kilo log trick FILE` and
   `kilo record decode FILE` read them. The log and the recordings are on
   the card only while saving is on (`kilo record save on --host`, or a
   kernel built with `KILO_RECORD_SAVE=1`).

What the Pi decides at power-on, in full, and how it flashes:
`kernel/README.md` "Flashing the body".

**If the flash failed**, the Pi does not try that same firmware again at
every power-on: it would fail the same way. It tries again when the files
change, and `kilo card` counts as a change (it removes `body-result.txt`
when it writes the firmware). So after fixing the cause (for example
charging the battery or reseating the board), run `kilo card CARD` again.
A refusal (battery low, files damaged) is checked afresh at every
power-on.

**If the Pi does not start** after a new kernel (the green light stays
steady for more than 20 seconds): card into the laptop, and in `CARD`
rename `kernel7.img` to `kernel7.bad` and `kernel7.old` to `kernel7.img`.
That is the kernel from before.

**Who holds the card holds Kilo.** Whoever has Kilo's SD card in hand can
put any kernel and any body firmware on it, and Kilo runs them at the next
power-on without asking for a key. That was already true of the kernel
before the card route; the network route still needs the laptop's key.

## Optional: the direct network cable

The network route (`kilo update`, `kilo flash --host`, `kilo status --host`,
the mod's live state) also works over one Ethernet cable between the laptop
and the Pi, with no router. When no router hands out an address, the Pi
gives itself the fixed address **169.254.75.73** three seconds after the
cable's link comes up (a "link-local" address: one that works only on that
cable). The laptop's wired port needs a link-local address too. With
NetworkManager (`IF` is the port's name in `ip link`, for example `enp1s0`;
a USB Ethernet adapter works the same):

```
$ nmcli connection add type ethernet ifname IF con-name kilo-direct ipv4.method link-local ipv6.method ignore
$ nmcli connection up kilo-direct
```

or, until the next reboot, `sudo ip addr add 169.254.1.1/16 dev IF`. Then:

```
$ kilo --host 169.254.75.73 status
```

The network route also needs a key the Pi knows, because the Pi talks over
the network only to laptops whose public key is built into its kernel.
Either copy the server's private key (only this way, never into the
repository):

```
$ mkdir -p ~/.config/kilo
$ scp SERVER:.config/kilo/host_key ~/.config/kilo/host_key
$ chmod 600 ~/.config/kilo/host_key
```

or give the laptop its own: `kilo key init`, then
`kilo key show >> kernel/keys/authorized_keys`, commit that line, and
install the rebuilt kernel by the card route (`kilo card CARD`). The second
way copies no secret; the server's key keeps working as well.

## Windows and macOS

**Windows.** Building the kernel and the firmware needs Linux tools, so
use WSL (Windows' built-in Linux, `wsl --install` then Debian or Ubuntu)
and follow this page inside it. In step 2 of the card route there is no
desktop inside WSL to mount the card, and `lsblk` does not show it. The card reader belongs to Windows; inside WSL mount
its drive letter (here `E:`) with
`sudo mkdir -p /mnt/e && sudo mount -t drvfs E: /mnt/e` and use `/mnt/e` as
`CARD` (not checked on WSL yet). Before ejecting, release it inside WSL with `sudo umount /mnt/e`, then
eject the card from Windows (the tray's "Safely Remove"); step 4's
`udisksctl` line does not apply.
For the direct cable, Windows gives the wired port a 169.254 address by
itself when no router answers (it can take a minute); WSL reaches it
through Windows, which may need WSL's "mirrored" networking mode (not
checked).

**macOS.** The Arm toolchain has a macOS download (pick the one for the
Mac's processor on the same Arm page, then
`kilo card CARD TOOLCHAIN=/path/to/it`); arduino-cli and Python work as on
Linux (Homebrew: `brew install arduino-cli python`). The card appears under
`/Volumes/`, for example `/Volumes/BOOT`; eject it from Finder. A wired
port with no router gets a 169.254 address by itself ("self-assigned").

## Background

How each install route works inside the Pi, and what is checked:
`kernel/README.md` "Three ways to install". The boot-time flash's rules:
`kernel/mind/flash/cardflash.h`.
