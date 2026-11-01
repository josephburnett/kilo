# kernel/mind: the portable mind core

The part of the mind that runs both in the Circle kernel on the Pi 2 B and
on the laptop (`kernel/host/`), against the virtual body. Phase P1 of
`docs/fast-boot.md`: the body link; P3: the walking planner; P4: flashing
the body. Platform-free
C++17: no exceptions, RTTI, heap, `std::` or `printf`; the only libc
functions are `memcpy` and `memset`, through the compiler's builtins
(`util/mem.h`), and the only libm functions the planner's `sinf`, `cosf`,
`sincosf`, `tanf`, `atanf`, `atan2f` (`planner/fmath.h`; Circle links libm).
`make -C kernel/host freestanding` proves it by linking the core with
`-nostdlib`.

`docs/fast-boot.md`: the body link; P2: the laptop interface; P3: the
walking planner, tricks, perception, the behaviour layer and the IR remote,
composed in one `Mind`. Platform-free C++17: no exceptions, RTTI, heap,
`std::` or `printf`; the only libc functions are `memcpy` and `memset`,
through the compiler's builtins (`util/mem.h`), and the only libm functions
the planner's `sinf`, `cosf`, `sincosf`, `tanf`, `atanf`, `atan2f`
(`planner/fmath.h`) and the drives' `exp`, `log`, `sin`, `cos`, `sincos`
(`util/dmath.h`; Circle links libm). `make -C kernel/host freestanding`
proves it by linking the core with `-nostdlib`.

```
platform.h        the seams: Serial (non-blocking bytes, tx_free), Clock (us), Log (lines), Gpio (body RESET)
platform_net.h    the network seams: Listener (accept into a fixed slot) and Conn (non-blocking TCP stream)
util/fmt, log     a small printf (%d %u %x %s %c %%, width, '0') and log lines through Log
util/logring      the RAM log: a Log sink keeping the last 64 lines, for `kilo log --host`
net/host_frame    the laptop channel's framing: len u16 | type | seq | payload | crc16
net/host_server   the laptop interface (P2): commands, telemetry and log streams, the raw bridge
link/serial_link  the link state machine: a port of mind/kilo/link/serial_link.py
link/body_state   what the link knows of the body (kilo/body/state.py's link part)
link/goals        latched goals, one per type
link/consts       message classes and timings (kilo/link/consts.py, LinkConfig)
link/names        names of message types and enums, for logs
link/stats.h      link counters (kilo/body/state.py LinkStats)
planner/          the walking planner: a port of mind/kilo/planner (model, patterns, plan, errors, walker)
flash/image       the body image in RAM (32 KB) and its manifest (body.json parse and write)
flash/crc32       the image's CRC-32, as zlib.crc32
flash/flasher     the STK500v1 flasher: RESET, optiboot, pages, verify, the HELLO after

tricks/           the trick model, the compiled-in library (library.gen.cpp) and the player (upload, play)
perception/       ultrasonic and IMU detectors, the Percepts aggregator (kilo/perception)
behaviour/        tree engine, drives, utility selector, library, 10 Hz runner, personality.gen.cpp (kilo/behaviour)
remote/           the IR key map and RemoteControl (kilo/ir.py); client.cpp: BodyClient's posture/balance/look
mind.h, mind.cpp  Mind: all of the above, the link and the laptop server in one object
util/dmath, str   double helpers with Python's sums (dmath), string compare (str)
proto/            vendored by protocol/vendor.py: never edit; run python3 protocol/vendor.py
sources.mk        MIND_SRCS, for the Circle Makefile and kernel/host/Makefile
```

Headers include each other relative to this directory (`#include
"link/serial_link.h"`), so a build adds `-I mind` from `kernel/`.

## Using the link

```cpp
SerialLink link(serial, clock, log);   // your Serial at KILO_BAUD (250000 8N1)
link.connect();                        // the port is open: HELLO_REQ goes out
for (;;) {                             // the mind's loop, every few ms
  link.mark_alive();                   // no stamp for 1 s: the link stops feeding the body's watchdog
  link.poll();                         // never blocks
  // link.state(): HELLO, TELEMETRY, epoch; link.set_goal(), send_stream(), request()
}
```

`poll()` must run at least every 10 ms or so: the heartbeat is due after
100 ms of silence and a command is retried after 50 ms.

## What differs from the Python link

- One cooperative state machine instead of reader and writer threads.
  `request()` does not block: it starts the one command in flight and
  `poll()` completes it (`req().state`).
- The owner opens the device and calls `connect()`; the link calls
  `disconnect()` when the Serial fails. No reconnect backoff (a PL011 does
  not go away), no silent/passive mode, no JSONL or `.kilolog` logging.
- A body with another protocol major is refused in the link (`set_goal`,
  `send_stream` return false), where Python refuses in `BodyClient`.
- The initial TX seq comes from the clock's low bits unless the platform
  seeds it (`seed_seq()`; the Circle kernel uses the hardware RNG).
- Enum names and the query -> reply map are listed by hand
  (`link/names.cpp`, `link/consts.cpp`) because the generated C header lacks
  them; `kernel/host/tests/test_names.cpp` checks them against
  `protocol/gen/kilo_protocol.py`.

## Laptop interface

Phase P2 of `docs/fast-boot.md` (§5). `net/host_server` serves one TCP
port (5300, `KILO_HOST_PORT`) to up to four clients; the laptop's side is
`kilo --host IP[:PORT]` (`mind/kilo/hostlink.py`, `mind/kilo/cli_host.py`).
The messages are in `protocol/spec.toml` (`[host]`, `[[host_messages]]`,
with the framing and rules in its comments), generated into
`proto/kilo_protocol.h` (`kilo_host_*_t`, `KILO_HOST_*`) and
`kilo_protocol.py` (`Host*`) like the body link's.

```cpp
LogRing log(clock, &platform_log);            // every line: the ring, then the platform's sink
SerialLink link(serial, clock, log);
HostServer host(listener, link, serial, clock, log, log, build_id);
for (;;) {
  link.mark_alive();
  host.poll();                                // accept, answer, stream, flush; never blocks
  if (!host.link_paused()) link.poll();       // the raw bridge and the flasher pause the link
}
```

| Channel | Messages | Who |
|---|---|---|
| Commands | `STATUS_REQ` -> `STATUS`; `CONTROL`, `POSTURE`, `TELEM_SUB`, `LOG_SUB`, `BRIDGE_OPEN`/`_CLOSE`, `REBOOT` -> `RESULT`; `PARAM_GET`/`_SET` answer `UNSUPPORTED` until P3 | any client; `POSTURE`, `PARAM_SET`, `REBOOT` and the bridge need control |
| Telemetry | `TELEM` at 1-50 Hz: the latest TELEMETRY, last EVENT, link counters | any client |
| Log | `LOG_LINE`: the last N lines of the ring, then live; seq gaps are lost lines | any client |
| Raw bridge | `BRIDGE_TX`/`BRIDGE_RX`: UART bytes, the link paused | the controller |
| Flash (P4) | `FLASH_BEGIN`, `FLASH_DATA`, `FLASH_COMMIT` -> `RESULT`; `FLASH_STATUS` until DONE or FAILED (ids 0x40-0x4F, 0xC0-0xCF) | the controller; see "Flashing" |

Control: the first client to send `CONTROL take=1` is the controller, the
rest observe. When the controller gives control up, closes, goes out of
step, or asked for keepalive and sent nothing for 2 s, the bridge closes and
the mind latches `SET_POSTURE REST` (the daemon's end of a hold in Python).

Reboot: `REBOOT` (`kilo reboot --host IP`) is the controller's (an observer
gets `NOT_CONTROLLER`, as for `POSTURE`; a running flash gets `FLASHING`).
The server latches REST if the link is up, logs the request, writes
`RESULT OK` to the socket at once, then sets `reboot_requested()` (also
`Mind::reboot_requested()`), which the platform polls: the Circle kernel
runs 200 ms more so the answer leaves, then reboots (on the bench card,
back to the netboot loader); `kilo-mind-host` exits with status 3. Nothing
blocks; the flag stays set.

Addressing: Circle asks DHCP for 3 s (`KILO_HOST_DHCP_TIMEOUT_MS`) and
otherwise takes 169.254.75.73/16 (`KILO_HOST_FALLBACK_IP_INIT`), so a
direct cable works with the laptop's port set to link-local and no DHCP
server. The server itself only sees connections: the address is the
platform's (`kernel/src/`), and on the laptop the `--listen` address.

On the laptop: `kilo-mind-host --port PTY --listen 127.0.0.1:0` logs
`listening on port N`; `kernel/host/test.sh` runs the real CLI against it
and the virtual body (status, monitor, posture, log tail, REST when the
controller is killed, `kilo.link` through the bridge,
`tests/bridge_client.py`, and `kilo reboot`: exit status 3).

## The planner

`planner/` is a port of `mind/kilo/planner` and the stream part of
`kilo/planner/walker.py`: the same algorithms, constants (margin 14 mm,
hold 25 ticks, catch 7 deg, ...) and outputs, one file per Python module.
`Walker` drives a walk from the mind's loop:

```cpp
Walker walker(link, clock, log);      // owns a Planner; fixed size, no heap
// in your LinkListener: walker.on_telemetry(t); walker.on_event(e);
WalkGoal g;                           // pattern (find_pattern), direction, speed, balance
const char* why = walker.start(g);    // null: accepted; while walking it changes the walk
for (;;) { link.mark_alive(); link.poll(); walker.poll(); }   // every few ms
walker.stop();                        // the planned stop; abort() ends at once
```

`poll()` does what Python's walk thread does: SET_STREAM telem_div 1, the
STANCE posture for 0.6 s, then every 20 ms (on a deadline grid) the
TELEMETRY and EVENTs since the last wake, and FEET up to the body's tick + 5
through `SerialLink::send_stream`; at the end STANCE, then the rates and
balance put back.

What differs from Python:

- `float`, except fit_attitude's normal equations and the lean's quadratic
  program, which stay in double (on two feet the regularisation is 1e-7 of
  the matrix; the program tests feasibility to 1e-6). The plan has exact
  .5 mm ties (a trot's foot lands at stride / 2 = 12.5 mm), where float and
  Python's double round either way: 14 of 18800 foot values in the vectors
  differ by 1 mm there, and nothing else does. `-DKILO_PLANNER_DOUBLE` builds
  everything in double, which matches Python exactly.
- Built with `-ffp-contract=off` (no fused multiply-add): the prediction of
  a slot's end must equal what `next()` then computes, or the lean is solved
  again and the slot waits.
- `Walker::start()` returns at once; a start during a planned stop is
  queued (Python blocks). No per-tick frame log. `errors.from_history` is
  not ported (the link keeps no history).

The golden test (`kernel/host/tests/test_planner_vectors.cpp`, float and
double) replays `kernel/host/tests/planner_vectors.txt`, made from the
Python planner. After changing either planner, regenerate it:

```
~/OpenCat/.venv/bin/python kernel/host/tests/gen_planner_vectors.py > kernel/host/tests/planner_vectors.txt
```

`make -C kernel/host bench` prints the cost per tick; `build/walk-demo`
trots the virtual body (`kernel/host/test.sh` runs both).

## Flashing

Phase P4 of `docs/fast-boot.md` (§6): the mind programs the body's
ATmega328P itself, through optiboot on the body link's UART, as `kilo flash`
does on Linux with avrdude (mind/kilo/tools/flash.py). The laptop's side is
`kilo flash --host IP [--persist]` and `kilo flash --sd MOUNT`
(mind/README.md "Flashing").

```cpp
static FlashImage image;                              // 32 KB: the image and its manifest
static Flasher flasher(serial, clock, gpio, link, log);
host.attach_flasher(flasher, image);                  // FLASH_*: the server fills image, starts and polls flasher
for (;;) {
  link.mark_alive();
  host.poll();
  if (!host.link_paused()) { link.poll(); walker.poll(); }   // nothing on the link while the flasher has the UART
}
```

### The image

`body.bin` is the firmware's flash from address 0 (what `avr-objcopy -O
binary` makes of the build; the laptop converts arduino-cli's Intel HEX), at
most `KILO_HOST_FLASH_MAX_IMAGE` = 32256 bytes (32 KB less optiboot's 512),
programmed in 128-byte pages, the last one padded with 0xFF. Its manifest
`body.json` is one flat JSON object:

```json
{"format": 1, "build": "3d441ec0", "size": 31234, "crc32": "cbf43926", "commit": "..."}
```

`build` is the `-DKILO_BUILD` hash (8 hex digits: the body's `HELLO.fw_build`
once it runs the image), `size` the bytes of body.bin, `crc32` its CRC-32 as
`zlib.crc32` computes it (`flash/crc32.h`). Other keys are skipped.
`parse_manifest()` reads it, `format_manifest()` writes it; JSON because
Python writes it from its standard library and one flat object needs a
small parser only.

### The flash channel

`FLASH_BEGIN` (the manifest, flags `PERSIST`) -> `FLASH_DATA` (offset +
up to 200 bytes, in order, not answered) -> `FLASH_COMMIT`. The mind checks
size and CRC at COMMIT and answers `RESULT OK` (flashing has started) or
`IMAGE_BAD` (a `FLASH_STATUS FAILED` with the error goes first). Then
`FLASH_STATUS` goes to that client at every phase change and page:
RESETTING, SYNCING, PROGRAMMING n/N, VERIFYING n/N, BOOTING, and last DONE
(with the new HELLO's fw_build and reset cause) or FAILED (error, STK500
command, attempt). While a flash runs, `POSTURE`, `PARAM_SET`, the bridge
and another `FLASH_BEGIN` are refused `FLASHING`, and STATUS has the
`FLASHING` state bit. Losing control while receiving drops the image; a
flash that has started runs to its end.

### The sequence (`flash/flasher.cpp`)

| Step | What | Time | From |
|---|---|---|---|
| take the UART | `owns_uart()`: the main loop stops polling the link; no REST first (the body is reset anyway; its watchdog covers the gap) | | docs/fast-boot.md §6 |
| baud | `Serial::set_baud(115200)` (`KILO_HOST_FLASH_BAUD`) | | optiboot, avrdude `-b 115200` |
| RESET | `Gpio::reset_assert()`, then `reset_release()` | 100 ms low | spec `reset_pulse_ms`, `mind/kilo/tools/gpio.py` |
| first GET_SYNC | after release; it waits in the 328P's 2-byte RX FIFO while optiboot blinks | +100 ms | Uno fuses' start-up delay (65 ms) |
| its answer | `STK_INSYNC STK_OK` | up to 600 ms | 3 LED flashes of 125 ms in the Uno build |
| later GET_SYNCs | one at a time (a third byte in the FIFO desyncs optiboot, which then starts the firmware) | 200 ms each, none after 1.3 s | optiboot's 1 s watchdog after the blink |
| drain | drop late answers to earlier GET_SYNCs | 20 ms | |
| ENTER_PROGMODE, READ_SIGN | the signature must be `1E 95 0F` (ATmega328P), else BAD_SIGNATURE, not retried | 200 ms per answer | avrdude's sequence |
| per page | `LOAD_ADDRESS` (word address) + `PROG_PAGE` (128 B, `'F'`) | 200 ms per answer | 133 B at 115200 = 11.5 ms, erase + write <= 9 ms |
| verify | per page `LOAD_ADDRESS` + `READ_PAGE`, compared byte for byte | 200 ms per answer | optiboot implements READ_PAGE for flash |
| LEAVE_PROGMODE | optiboot resets into the firmware 16 ms later | | |
| back | `set_baud(250000)` after 50 ms, a 0x00 frame delimiter (the firmware may hold STK500 bytes), the link restarted (as after the bridge) | | |
| HELLO | `fw_build` == build, `reset_cause` FLASHED (WARM_UNEXPECTED is taken when the body already ran this build: an identical image leaves its reset record as it was, docs/architecture.md §3a) | up to 8 s | `kilo flash --verify-timeout` |

A failure in sync, a missing or bad answer, or a verify mismatch ends the
attempt; a second attempt starts from a fresh reset (as `kilo flash` retries
avrdude once; optiboot survives an interrupted flash). Any failure ends
with RESET released, the UART at the link baud and the link restarted. On
the bench (kernel/host/test.sh, against `tests/fake_optiboot.py`) a 31 KB
image takes about 3 s; on the 328P expect about 10 s **(est.)**, as avrdude.

On the laptop `kilo-mind-host --reset-fifo PATH` makes the RESET line a
named pipe ('L' low, 'H' released; `posix.h` FifoResetGpio), which the fake
optiboot body reads. Without it the edges are only logged.

### On the Circle kernel (`kernel/src/`)

Wired as above, with the GPIO4 `Gpio`, the PL011's `set_baud()` switching
in place, the boot-time flash from SD (path B: after the first HELLO or
3 s, `parse_manifest()`, `load_from()`, `needs_flash()`) and `--persist`
(`body.bin`, then `body.json`; `persist_done()` even when the write
fails): kernel/README.md "Flashing the body". `kilo flash --rollback`
(keep the last good image on SD) is not designed yet.

## The Mind

`mind.h` puts the whole core in one object, so a platform's main loop is
a few lines (`kernel/host/main.cpp`):

```cpp
MindConfig cfg;                       // remote, behaviour, balance_kp, seed, stats_period_ms ...
static Mind mind(serial, clock, platform_log, &listener, cfg);  // ~70 KB: static; listener null = no laptop
mind.start();                         // the port is open: start goals, HELLO_REQ
for (;;) mind.poll();                 // a few ms apart; never blocks
```

One pass: stamp the link's liveness, serve the laptop, then, unless the
raw bridge is open, poll the link, hand the command slot's answer to its
owner (the trick player, the STATS poll) before anything may send
another request, and run the walker, the trick player, the remote and the
behaviour runner. Every TELEMETRY feeds perception; every EVENT goes to
the walker, the player, the snapshot's last 8 and, for IR_KEY, the remote.

**Liveness.** `Mind::poll()` stamps `link.mark_alive()` once per pass.
Every part runs in this one cooperative loop, so a pass that returns
proves nothing is stuck, and a part that hangs stops the stamp, which is
what the body's watchdog needs. Python stamps from the behaviour runner's
tick and lets the daemon's pump stamp only while the runner ticked within
1 s (`daemon/behaviour.py`); here the same check gates the stamp when the
runner is on (10 s of grace before its first tick), because its 10 Hz grid
could stall while the loop runs.

**Who drives.** A laptop client holding control (`CONTROL take=1`) pauses
the behaviour runner and silences the remote: Python refuses client goals
while the runner is active unless `--override` pauses it, and a controller
is that override here. With the runner active the remote is ignored, as
in `remote_ir.py`. `MindConfig` defaults are the daemon's: remote and
behaviour off (the Pi's `KILO_RUN_ARGS` turn on `--remote --balance 0.99`).

`kilo-mind-host` takes `--remote`, `--behaviour`, `--seed N` and
`--balance KP` besides its P1/P2 flags.

## Tricks

`tricks/` ports `kilo/tricks/model.py` and `player.py` and the trick part
of `kilo/body/client.py`. The library is compiled in: the mind only ever
uploads it, so a `Trick` is already in wire form (the TRICK_BEGIN fields and
the 14 B keyframes exactly as Python's `Trick.begin()` and
`Keyframe.to_wire()` make them). `tricks/gen_library.py` writes
`library.gen.{h,cpp}` from `mind/kilo/tricks/library/*.json` (13 tricks,
43 frames, about 940 B of tables), with Python's CRC, trick_id and duration
for the tests:

```
~/OpenCat/.venv/bin/python kernel/mind/tricks/gen_library.py          # after changing the JSON
~/OpenCat/.venv/bin/python kernel/mind/tricks/gen_library.py --check  # test.sh
```

`TrickPlayer` is `upload_trick`/`start_trick`/`play`/`stop_trick` as a
state machine over the link's one command slot: TRICK_COMMIT alone asks
the body whether it holds the trick (cached by id, CRC and reboots), else
TRICK_BEGIN, the TRICK_FRAMES chunks and TRICK_COMMIT; the posture goal is
released; TRICK_PLAY (NO_TRICK: upload again once); done on TRICK_DONE for
its id, or failed after the expected duration + 1 s (then TRICK_STOP).
`KeyframeTimer` and `total_ticks` are the body's player timing
(`player.py`).

What differs: no float angles, JSON, `mirror()` or `speed()` in the mind
(the body mirrors and changes speed from TRICK_PLAY); `start()` returns at
once and `result()` reports the play; a second play while one runs is
refused until it is stopped.

## Perception

`perception/` ports `kilo/perception` (ultrasonic: median, range rate,
zones with hysteresis, approaching; IMU: tilted/level, picked_up/put_down,
tap, pet, startle; `Percepts`) with the same thresholds, windows and
hysteresis, in double. `link/body_state` gains `state.py`'s derived values
(distance median and approach speed, battery percent, fallen, upright,
moving). The Mind feeds `Percepts` on every TELEMETRY with its arrival
time, as the daemon does, and logs each percept event.

The golden test replays `mind/tests/test_perception.py`'s traces through
both: `kernel/host/tests/gen_perception_vectors.py` writes them (quantised
to TELEMETRY's 1/16 degree and whole mm) with Python's events, and every
event must come out on the same sample. Sums are Python's (`util/dmath.h`:
Neumaier `sum()`, exact `fsum` for `fmean`), so they round alike.

What differs: fixed rings instead of deques (the IMU history holds 256
samples, 4.5 s at up to 56 Hz); the 10 s `TelemetryRing` is not ported (no
detector reads it); no `SharedPercepts` lock (one loop).

## Behaviour

`behaviour/` ports `kilo/behaviour`: the tree engine (`tree`), the drives,
the utility selector with the safety branches, the library of behaviours
node for node with Python's names (the active path reads the same), and
the 10 Hz runner. `personality.yaml` is converted on the laptop with
Python's loader (merge over the defaults, validation) into a 1000 B table:

```
~/OpenCat/.venv/bin/python kernel/mind/behaviour/gen_personality.py [FILE.yaml]   # writes personality.gen.cpp
~/OpenCat/.venv/bin/python kernel/mind/behaviour/gen_personality.py --check
```

The RNG is Python's Mersenne Twister seeded as `random.Random(int)` seeds
it (`random`, `uniform`, `gauss`), so a seeded C++ runner makes the same
choices. `kernel/host/tests/gen_behaviour_vectors.py` runs
`test_behaviour.py`'s scenarios and three more through the Python runner
and `test_behaviour.cpp` replays them: 9432 ticks, every choice, reason,
path, drive, score and call on the body equal.

In the Mind the behaviours' body is the link (`remote/client.cpp`:
`BodyClient`'s posture, set_balance and look) and the trick player;
perception's flags are the percepts (`runner.py` `PerceptFeed`, face always
false). Each posture, look and trick first quiets the IMU detectors
(`adapter.py`'s 0.5, 0.3, 2 s).

What differs: no JSONL behaviour log (a log line when the choice changes,
body events and the behaviours' postures as log lines); `poll()` keeps the
10 Hz grid instead of an asyncio loop; the wind-down's poweroff only stops
the runner and sets `Mind::poweroff_requested()`; `beep` is not in the body
(as in Python's adapter). Python's `play:` action fails after every real
trick (`_as_status` rejects the `TrickResult` that `BodyClient.play`
returns); here a trick that ends DONE succeeds.

## Remote

`remote/` ports `kilo/ir.py`: the key table and map (keys named by grid,
`remote/keys.h`) and `RemoteControl`, driving the walker, the postures and
the trick player as `kilo run --remote` does. Direction keys walk with the
selected pattern and speed; pattern and speed keys change a walk in
progress; a posture or trick key makes the planned stop and waits for it,
stops a playing trick, and for a trick from LIMP takes STAND and waits 1 s;
C7 toggles the balance. The daemon's blocking waits are phases of `poll()`
and keys that come meanwhile queue (4), as its lock queues them. Repeats and
other remotes' keys are ignored; there is no shutdown key (Python has none).

What differs: log lines name a key by grid and legacy meaning, `B3 (0x18,
sit)`, without the printed label.

Integration (`kernel/host/test.sh`): `python -m kilo.vbody.core --control
FIFO` takes `ir CODE`, `att P R`, `batt MV` and `imu ...` lines
(`CoreBody.serve_control`; the harness has no IR decoder, so IR_KEY frames
are made in Python), and the script plays hi to TRICK_DONE, trots, turns,
stands after the planned stop and rests by IR keys, idles the behaviour
runner 5 s at 10 Hz, and tilts and drops the virtual body for perception
and the fallen branch.
