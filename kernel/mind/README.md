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
| Commands | `STATUS_REQ` -> `STATUS`; `CONTROL`, `POSTURE`, `TELEM_SUB`, `LOG_SUB`, `BRIDGE_OPEN`/`_CLOSE` -> `RESULT`; `PARAM_GET`/`_SET` answer `UNSUPPORTED` until P3 | any client; `POSTURE`, `PARAM_SET` and the bridge need control |
| Telemetry | `TELEM` at 1-50 Hz: the latest TELEMETRY, last EVENT, link counters | any client |
| Log | `LOG_LINE`: the last N lines of the ring, then live; seq gaps are lost lines | any client |
| Raw bridge | `BRIDGE_TX`/`BRIDGE_RX`: UART bytes, the link paused | the controller |
| Flash (P4) | `FLASH_BEGIN`, `FLASH_DATA`, `FLASH_COMMIT` -> `RESULT`; `FLASH_STATUS` until DONE or FAILED (ids 0x40-0x4F, 0xC0-0xCF) | the controller; see "Flashing" |

Control: the first client to send `CONTROL take=1` is the controller, the
rest observe. When the controller gives control up, closes, goes out of
step, or asked for keepalive and sent nothing for 2 s, the bridge closes and
the mind latches `SET_POSTURE REST` (the daemon's end of a hold in Python).

Addressing: Circle asks DHCP for 3 s (`KILO_HOST_DHCP_TIMEOUT_MS`) and
otherwise takes 169.254.75.73/16 (`KILO_HOST_FALLBACK_IP_INIT`), so a
direct cable works with the laptop's port set to link-local and no DHCP
server. The server itself only sees connections: the address is the
platform's (`kernel/src/`), and on the laptop the `--listen` address.

On the laptop: `kilo-mind-host --port PTY --listen 127.0.0.1:0` logs
`listening on port N`; `kernel/host/test.sh` runs the real CLI against it
and the virtual body (status, monitor, posture, log tail, REST when the
controller is killed, and `kilo.link` through the bridge,
`tests/bridge_client.py`).

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

### Left to the Circle kernel (`kernel/src/`)

- The `Gpio` on GPIO4 (`CircleResetGpio` exists: low = output driven low,
  released = input, no pull) and `Serial::set_baud(115200)` and back on the
  PL011 (today it only accepts the rate it has).
- Wiring: `host.attach_flasher(flasher, image)`, and `link_paused()` instead
  of `bridge_open()` in the main loop, with the walker stopped too.
- Boot-time flash from SD (Path B): read `body.json` and `body.bin` from the
  FAT partition's root (next to `kernel7.img`; `kManifestFile`,
  `kImageFile` in `flash/image.h`), `parse_manifest()`, then
  `image.load_from(manifest, data, size)` (checks size and CRC). Once the
  link has a HELLO (or none came within a few seconds: an empty or broken
  firmware), `image.needs_flash(hello)` (or `needs_flash(nullptr)`) says
  whether to `flasher.start(image, false)` and poll it until `!busy()`. A
  matching build costs nothing at boot.
- `--persist` (FLASH_BEGIN flag PERSIST): after DONE, `flasher.persist_pending()`
  is true; write `image()`'s bytes as `body.bin` and `format_manifest()` as
  `body.json` (manifest last), then `persist_done()`.
- `kilo flash --rollback` (keep the last good image on SD) is not designed
  yet.

