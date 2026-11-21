# Setting up a dev host

The Debian dev host where Kilo's tests and builds run. The checkout is still
called `~/OpenCat`; the tools and docs default to that name and to the venv
in `~/OpenCat/.venv`. A laptop that also builds Kilo's two programs and puts
them on his SD card is set up as `docs/laptop-setup.md` says; the mind on
Kilo is the kernel in `kernel/` (`kernel/README.md`).

## Dev host (Debian 13)

| Tool | Purpose |
|---|---|
| `build-essential`, `g++` (gcc 14, with libasan) | body host tests, built with `-fsanitize=address,undefined` |
| `python3`, `python3-venv` (3.11+) | the mind and its tests |
| `git`, `jq` | |
| `arduino-cli` 1.5.x with core `arduino:avr` 1.8.8 | build the body and run `size_check.sh` |

```sh
sudo apt install build-essential g++ git jq python3 python3-venv
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
~/.local/bin/arduino-cli core update-index && ~/.local/bin/arduino-cli core install arduino:avr
git clone git@github.com:josephburnett/kilo.git ~/OpenCat && cd ~/OpenCat
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -e mind pytest            # add numpy for the one skipped test
export PATH="$HOME/OpenCat/.venv/bin:$HOME/.local/bin:$PATH"
body/tools/host_tests.sh && body/tools/size_check.sh && protocol/test.sh && .venv/bin/python -m pytest mind/tests -q
```

The four suites are the check before every commit. `size_check.sh` needs
arduino-cli; it also runs `body/tools/stack_check.sh` (the static worst-case
stack estimate, a second build with `-fstack-usage`, a few seconds; it fails
below 128 B of margin; `STACK_CHECK=0` skips it); `protocol/test.sh` needs the venv on `PATH` (it runs pytest).
`.venv/` is gitignored.

## Day-to-day loop

1. Edit and run the four suites on the dev host. Commit on `main`, push.
2. Build and install the mind kernel and the body firmware from the laptop:
   `docs/laptop-setup.md`, and `kernel/README.md` "Flashing the body".
   Check with the owner before anything that moves Kilo.
