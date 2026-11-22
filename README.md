# Kilo

Kilo is a robot cat: a Petoi Nybble with a Raspberry Pi 2 B as its brain.
This repository is Kilo's software.

It began as a fork of Petoi's [OpenCat](https://github.com/PetoiCamp/OpenCat),
the stock firmware for the Nybble's NyBoard. The stock firmware does
everything on the board's small Arduino chip (an ATmega328P with 32 KB of
flash and 2 KB of RAM): postures, gaits, tricks, the remote, the sensors and
the personality. That left no room to grow, so we rewrote it as two minds
that share the work:

- `body/`: new firmware for the NyBoard, the lower mind. It does only what
  has to be fast and safe: moving the joints, playing the steps the Pi
  sends (one set of foot positions every 20 ms, a little ahead of time),
  correcting the small errors between what the Pi predicted and what the
  body feels, catching falls, watching the battery and the link to the Pi.
  If the Pi goes quiet, the body puts its feet down, stops, lies down and
  goes limp on its own.
- `kernel/`: the upper mind, a bare-metal C++ kernel the Pi boots from its
  SD card (`kernel/README.md`, `docs/fast-boot.md`). It decides what Kilo
  does: postures, tricks, walking (it plans every step from a model of the
  body and streams them to the body), the IR remote, perception and a
  personality. It talks to the body over the serial link on the NyBoard's
  Pi socket.
- `mind/`: the Python mind, where the kernel's design was first written and
  tested; today it is the laptop's `kilo` command line, which talks to the
  kernel over the network (`--host`) or updates it through the SD card, and
  the reference the kernel's tests are checked against.
- `protocol/`: the single definition of the messages between the two; the C
  and Python sides are generated from it.

Where to read more: `docs/architecture.md` is the design, `docs/setup.md`
sets up a dev host, `docs/laptop-setup.md` a laptop that updates
Kilo through his SD card, `docs/signals.md` what his eye colours and
flashes mean, `mind/README.md` lists the commands, and
`docs/p*-hardware-test*.md` record what has been tried on Kilo.

Of the original OpenCat firmware only five headers remain, in `legacy/`
(`legacy/README.md`): `legacy/InstinctNybble.h` holds the stock postures and
skills that the new firmware and the trick importer read, and the tests check
the others against the new code. The rest of the fork was removed (its
desktop UI, module tests and skill library on 2026-10-11, `OpenCat.ino` and
`src/` on 2026-11-20); git history has it. Licence: MIT, as OpenCat's.
