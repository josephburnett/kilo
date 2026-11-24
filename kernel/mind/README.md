# kernel/mind: the portable mind core

The part of the mind that runs both in the Circle kernel on the Pi 2 B and
on the laptop (`kernel/host/`), against the virtual body. By the phases of
`docs/fast-boot.md`: P1 the body link; P2 the laptop interface; P3 the
walking planner, tricks, perception, the behaviour layer and the IR remote,
composed in one `Mind`; P4 flashing the body and updating the kernel on the
SD card. Platform-free C++17: no exceptions, RTTI, heap, `std::` or
`printf`; the only libc functions are `memcpy` and `memset`, through the
compiler's builtins (`util/mem.h`), and the only libm functions the
planner's `sinf`, `cosf`, `sincosf`, `tanf`, `atanf`, `atan2f`
(`planner/fmath.h`) and the drives' `exp`, `log`, `sin`, `cos`, `sincos`
(`util/dmath.h`; Circle links libm). `make -C kernel/host freestanding`
proves it by linking the core with `-nostdlib`.

```
platform.h        the seams: Serial (non-blocking bytes, tx_free), Clock (us), Log (lines), Gpio (body RESET),
                  KernelStore (the SD card's kernel7.* files: blocking, used only while the link is paused),
                  RecordCard (the card's kilorec/ for the flight recorder: blocking, one call a pass at safe moments)
platform_net.h    the network seams: Listener (accept into a fixed slot) and Conn (non-blocking TCP stream)
util/fmt, log     a small printf (%d %u %x %s %c %%, width, '0') and log lines through Log
util/logring      the RAM log: 4096 lines of history for `kilo log --host`; status lines live only, and every line to the recorder
record/flight     the flight recorder's RAM: a ring of 16 KB blocks (header, CRC-32, records), the oldest dropped
record/recorder   everything into it from boot: link frames, log lines, walks and tricks as runs, the run index
record/saver      the recorder onto the SD card at safe moments, a 4 KB call a pass; earlier boots' runs; the bench
record/metrics.h  the session's counters (METRICS)
net/host_frame    the laptop channel's framing: len u16 | type | seq | payload | crc16
net/host_server   the laptop interface (P2): commands, telemetry and log streams, the raw bridge
net/mdns          the mDNS responder: kilo.local and the _kilo._tcp service, so `kilo find` finds Kilo (kernel/README.md "mDNS")
link/serial_link  the link state machine: a port of mind/kilo/link/serial_link.py
link/body_state   what the link knows of the body (kilo/body/state.py's link part)
link/goals        latched goals, one per type
link/consts       message classes and timings (kilo/link/consts.py, LinkConfig)
link/names        names of message types and enums, for logs
link/stats.h      link counters (kilo/body/state.py LinkStats)
planner/          the body model (model.h, as mind/kilo/planner/model.py) and the model's error from the telemetry (errors)
engine/           the movement engine (docs/movement-engine.md): gait (the gaits as table rows), clock (the one step clock), solve (the tick solver: feet and weight every tick), walker (its ticks streamed to the body as POSE), paw (the paw wash: a front paw lifted on three feet, made as a trick from the model)
flash/image       the body image in RAM (32 KB) and its manifest (body.json parse and write)
flash/crc32       the image's CRC-32, as zlib.crc32
flash/flasher     the STK500v1 flasher: RESET, optiboot, pages, verify, the HELLO after
update/updater    the mind's own kernel onto the SD card: receive, check (CRC, marker), write, read back, rename

tricks/           the trick model, the compiled-in library (library.gen.cpp) and the player (its keyframes as POSE keys)
perception/       ultrasonic and IMU detectors, the Percepts aggregator (kilo/perception)
behaviour/        tree engine, drives, utility selector, library, 10 Hz runner, personality.gen.cpp (kilo/behaviour)
behaviour/alive   small body moves while the runner idles or explores (no Python counterpart)
guard/            the distance guard: the range filtered, zones, stop / back off / sit, forward keys refused, the eyes' colour
sound/            Kilo's sounds: the player (requests, families, the queue, the gap, volume), the renderer the audio
                  interrupt runs, the output seam (no Python counterpart; docs/sound.md)
signal/           what Kilo tells the owner: named signals (ready, done, refused, failed, ...) on the eyes (the eye
                  language, its priority under the guard's blue and red, SET_EYES) with their sounds, from one call
                  (Mind::signal; no Python counterpart; docs/signals.md)
remote/           the IR key map and RemoteControl; client.cpp: BodyClient's posture/balance/look
measure/          the self-measurement (C5 twice): the script (a port of mind/kilo/measure/script.py) and its run;
                  docs/self-measurement.md
intent/           the state estimate (always on, read only: a port of mind/kilo/intent/estimate.py) and the
                  intents "be standing / sitting / resting" (MindConfig.intent, KILO_INTENT; intent.h), their
                  tables in intent.gen.cpp from intent/gen_intent.py (the model's predictions, the moves checked
                  against it, the first step from every state to every goal); docs/intent.md
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
host.set_keys(keys, n_keys, random);          // who may connect; without it nobody may
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
| Log | `LOG_LINE`: the last N lines of history (255: all), then live, status lines too (seq bit 31); seq gaps are lost lines | any client |
| Raw bridge | `BRIDGE_TX`/`BRIDGE_RX`: UART bytes, the link paused | the controller |
| Flash (P4) | `FLASH_BEGIN`, `FLASH_DATA`, `FLASH_COMMIT` -> `RESULT`; `FLASH_STATUS` until DONE or FAILED (ids 0x40-0x42, 0xC0) | the controller; see "Flashing" |
| Kernel update | `UPDATE_BEGIN`, `UPDATE_DATA`, `UPDATE_COMMIT` -> `RESULT`; `UPDATE_STATUS` until DONE (then the reboot) or FAILED (ids 0x48-0x4A, 0xC1) | the controller; see "Updating the kernel" |
| Flight recorder | `WALK_LIST` -> `WALK_INFO` (one run a request); `WALK_FETCH` (a run), `REC_FETCH` (a range, or an earlier boot's log) -> `RESULT`, then `WALK_DATA` until END (or GONE); `REC_STATUS_REQ` -> `REC_STATUS`; `METRICS_REQ` -> `METRICS` | any client; see "The flight recorder" |
| The card | `REC_SAVE`, `REC_DELETE`, `REC_BENCH` -> `RESULT` (`BENCH_RESULT` after the bench) | the controller |

Who may connect (`kernel/README.md` "Who may talk to Kilo: keys" for the
whole story; the spec's `[host]` comments for the bytes): every
connection opens with a clear hello naming the laptop's X25519 public
key, which must be in the list `set_keys()` gives (the Circle kernel's is
built in from `kernel/keys/authorized_keys`; `kilo-mind-host` reads
`--authorized-keys FILE`), then the Noise handshake
`Noise_KN_25519_ChaChaPoly_BLAKE2b` (`net/noise.*`, over Monocypher in
`third_party/monocypher/`), then records (`len u16 | ciphertext | tag`,
up to 1024 bytes) that carry the frames above unchanged
(`net/host_secure.cpp`). The responder's three X25519 operations run one
per `poll()`, for all connections together. A connection is a client only
once a record from it authenticates; until then it gets nothing, counts
for nothing (`clients()`, `STATUS.clients`, control) and is closed after
`KILO_HOST_HANDSHAKE_TIMEOUT_MS` (3 s). A bad record closes the
connection, unanswered. The listener has `kHostSlots` (5) slots, one more
than the clients, and a new connection closes the oldest one still in its
handshake when it must, so idle sockets cannot keep a laptop out. The
ephemeral keys come from the `Random` seam (`platform.h`); if it fails,
the hello is refused. There is no plaintext mode, for tests either: the
unit tests' fake connections run the handshake (`host/tests/fake_net.h`)
and `kernel/host/test.sh` gives `kilo-mind-host` a throwaway key.

Control: the first client to send `CONTROL take=1` is the controller, the
rest observe. When the controller gives control up, closes, goes out of
step, or asked for keepalive and sent nothing for 2 s, the bridge closes and
the mind latches `SET_POSTURE REST`.

Reboot: `REBOOT` (`kilo reboot --host IP`) is the controller's (an observer
gets `NOT_CONTROLLER`, as for `POSTURE`; a running flash gets `FLASHING`, a
kernel update that is writing `UPDATING`).
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

On the laptop: `kilo-mind-host --port PTY --listen 127.0.0.1:0
--authorized-keys FILE` logs `listening on port N`; `kernel/host/test.sh`
makes a throwaway key (`kilo key init --key ...`) and runs the real CLI against it
and the virtual body (status, monitor, posture, log tail, REST when the
controller is killed, `kilo.link` through the bridge,
`tests/bridge_client.py`, `kilo reboot`: exit status 3, and `kilo update`
into `--card DIR`: exit status 3, then the card's files).

## The engine's walks

`engine/` is the movement engine's walking (docs/movement-engine.md, step
2): one step clock (`clock.h`), the gaits as rows of a table (`gait.h`:
when each leg lifts, its swing, the weight target), and the tick solver
(`solve.h`) that every tick works out the feet and where the weight goes.
It is written only here, in C++ (no Python copy); the laptop runs it through
`kernel/host`'s `engine-judge` (`kilo.engine`), and
`mind/tests/enginejudge.py` judges it on the virtual body.
`Walker` drives a walk from the mind's loop:

```cpp
Walker walker(link, clock, log);      // owns a Solver; fixed size, no heap
// in your LinkListener: walker.on_telemetry(t); walker.on_event(e);
WalkGoal g;                           // gait (find_gait), direction, speed, steer, cadence, balance
const char* why = walker.start(g);    // null: accepted; while walking it changes the walk
for (;;) { link.mark_alive(); link.poll(); walker.poll(); }   // every few ms
walker.stop();                        // the planned stop; abort() ends at once
walker.halt();                        // the quick stop (feet down where they are, then squared up)
```

`poll()` drives the stream (Python's `kilo.planner.walker.Stream`, which the
tests' walking simulations drive): SET_STREAM telem_div 1, the
STANCE posture for 0.6 s, then every 20 ms (on a deadline grid) the
TELEMETRY and EVENTs since the last wake, and POSE (four feet) up to the body's tick + 3
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
  queued. No per-tick frame log. `errors.from_history` is
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
ATmega328P itself, through optiboot on the body link's UART, as avrdude
`-c arduino` does on Linux. The laptop's side (mind/kilo/tools/flash.py) is
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
| RESET | `Gpio::reset_assert()`, then `reset_release()` | 100 ms low | spec `reset_pulse_ms` |
| first GET_SYNC | after release; it waits in the 328P's 2-byte RX FIFO while optiboot blinks | +100 ms | Uno fuses' start-up delay (65 ms) |
| its answer | `STK_INSYNC STK_OK` | up to 600 ms | 3 LED flashes of 125 ms in the Uno build |
| later GET_SYNCs | one at a time (a third byte in the FIFO desyncs optiboot, which then starts the firmware) | 200 ms each, none after 1.3 s | optiboot's 1 s watchdog after the blink |
| drain | drop late answers to earlier GET_SYNCs | 20 ms | |
| ENTER_PROGMODE, READ_SIGN | the signature must be `1E 95 0F` (ATmega328P), else BAD_SIGNATURE, not retried | 200 ms per answer | avrdude's sequence |
| per page | `LOAD_ADDRESS` (word address) + `PROG_PAGE` (128 B, `'F'`) | 200 ms per answer | 133 B at 115200 = 11.5 ms, erase + write <= 9 ms |
| verify | per page `LOAD_ADDRESS` + `READ_PAGE`, compared byte for byte | 200 ms per answer | optiboot implements READ_PAGE for flash |
| LEAVE_PROGMODE | optiboot resets into the firmware 16 ms later | | |
| back | `set_baud(250000)` after 50 ms, a 0x00 frame delimiter (the firmware may hold STK500 bytes), the link restarted (as after the bridge) | | |
| HELLO | `fw_build` == build, `reset_cause` FLASHED (WARM_UNEXPECTED is taken when the body already ran this build: an identical image leaves its reset record as it was, docs/architecture.md §3a) | up to 8 s | the body's boot (optiboot's wait, DMP init, the first HELLO: about 2.2 s) with margin |

A failure in sync, a missing or bad answer, or a verify mismatch ends the
attempt; a second attempt starts from a fresh reset (optiboot survives an
interrupted flash). Any failure ends
with RESET released, the UART at the link baud and the link restarted. On
the bench (kernel/host/test.sh, against `tests/fake_optiboot.py`) a 31 KB
image takes about 3 s; on the 328P expect about 10 s **(est.)**, as with avrdude.

On the laptop `kilo-mind-host --reset-fifo PATH` makes the RESET line a
named pipe ('L' low, 'H' released; `posix.h` FifoResetGpio), which the fake
optiboot body reads. Without it the edges are only logged.

### On the Circle kernel (`kernel/src/`)

Wired as above, with the GPIO4 `Gpio`, the PL011's `set_baud()` switching
in place, the boot-time flash from SD (`flash/cardflash.h`, `CardFlash`:
after the first HELLO and TELEMETRY or 3 s, the rules there, the outcome in
`body-result.txt`, a failed image not retried) and `--persist` (`body.bin`,
then `body.json`; `persist_done()` even when the write fails):
kernel/README.md "Flashing the body" and "Three ways to install". `kilo flash --rollback`
(keep the last good image on SD) is not designed yet.

## Updating the kernel

The mind writes its own successor onto the SD card it booted from
(`update/updater.*`; the laptop's side is `kilo update --host IP [--image
PATH] [--force]`, the Circle side kernel/README.md "Updating the kernel over
the network"):

```cpp
static KernelUpdater updater(buffer, capacity, store, clock, log);  // buffer: the platform's (4 MB on Circle)
host.attach_updater(updater);   // UPDATE_*: the server fills and checks it, polls it, reboots after DONE
```

`UPDATE_BEGIN` (image_size, crc32, flags `FORCE`) -> `UPDATE_DATA` (offset +
up to 200 bytes, in order, not answered) -> `UPDATE_COMMIT`. At COMMIT the
updater checks that every byte arrived, the CRC-32 (`flash/crc32`) and,
unless `FORCE`, that `KILO_HOST_UPDATE_MARKER` (`"KILO-MIND-KERNEL-1"`,
spec `host.update_marker`) is in the image: the Circle kernel embeds it
(`kernel/src/kernel.cpp`), the netboot loader does not, so a wrong file is
refused `IMAGE_BAD` (an `UPDATE_STATUS FAILED NOT_A_KERNEL` first) and
nothing is written. Then, through the platform's `KernelStore`
(`platform.h`), a 16 KB slice per `poll()`:

| Phase | What |
|---|---|
| CHECKING | checked; reported right behind `RESULT OK` |
| WRITING | `kernel7.new` created, written slice by slice, closed |
| VERIFYING | `kernel7.new` read back to its end: length and CRC-32 must be the image's |
| INSTALLING | `kernel7.old` removed (if any), `kernel7.img` -> `kernel7.old`, `kernel7.new` -> `kernel7.img`, sync |
| DONE | the server latches REST, sends the status, sets `reboot_requested()` (as REBOOT) |
| FAILED | `WRITE_FAILED`, `VERIFY_FAILED`: `kernel7.new` removed, `kernel7.img` untouched; `INSTALL_FAILED`: a remove or rename failed, `kernel7.img` is the old kernel (renamed back if it had moved); `CARD_BROKEN`: renaming it back failed too, the card has no `kernel7.img` |

`KernelStore`'s calls block (the SD card), so from COMMIT to the end
`host.updating()` and `host.link_paused()` are true: the main loop polls
nothing on the link, as during a flash, and `Mind` aborts a walk the pause
cut off. Meanwhile `POSTURE`, `PARAM_SET`, `REBOOT`, the bridge and
`FLASH_*` are refused `UPDATING`; `UPDATE_BEGIN` is refused `FLASHING`
while a body flash runs (or its `--persist` image waits for the card). A
`BEGIN` of either drops the other's image still being received, and losing
control drops a receiving image. A server without an updater answers
`UNSUPPORTED` (`kilo-mind-host` without `--card DIR`). The host channel
version stays 1.

On the laptop `kilo-mind-host --card DIR` is the store on a directory
(`posix.h` DirStore): `kernel/host/test.sh` sends a fake kernel with the
marker, checks `kernel7.img`, `kernel7.old` and that `kernel7.new` is gone,
and the exit status 3; `tests/test_update.cpp` runs every store failure
against an in-memory card (`tests/fake_store.h`).

## The flight recorder

`record/` records everything the mind sees and does, from boot, into one
RAM stream, so that a walk or a trick can be studied afterwards (the owner
plays with Kilo with the cable unplugged, then plugs it in): `kilo runs
--host IP` lists the runs, `kilo runs --host IP --fetch N` downloads one,
`kilo record fetch --host IP --last 10m` any stretch of time, and `kilo
log walk|trick|run FILE` analyses them (`mind/README.md`).

What the stream holds (`record/recorder.h`):

- every body link frame both ways, as it crossed the UART, with the mind's
  clock in microseconds (`SerialLink::set_frame_tap()`): TELEMETRY
  (measured pitch and roll and their rates, mode, flags, battery,
  distance, the body's reference `ref_*` and correction `corr_*`,
  support, `pose_queued`, `limited`, `tick_load`), EVENTs (IR keys,
  falls, STREAM_STOPPED, the watchdog), JOINTS if they are streamed; POSE
  (a walk's feet, a trick's keys), goals (postures among them) and
  commands; HEARTBEAT only counted;
- every log line (`LOG`: the LogRing's tap), the routine status lines
  included;
- during a walk, each planned tick (`walk_plan`, through the Walker's
  `WalkObserver`), the planner's hold, catch and wait counters when they
  move (`walk_planner`), and `MARK` lines, `word key=value ...`: `walk`
  (id, format, the setup's time on the mind's clock, builds, battery),
  `setup`, `stream` (the timing the planner chose), `config`, `model` and
  `masses`, `change` / `timing` / `stop` / `queued`, and at the end
  `result`, `error` and `end` (ok, reason);
- during a trick (any: the remote's, the behaviour runner's), `MARK`
  lines: `trick` (id, the trick, its rear-up stage, its id and CRC (which
  version of it), frames, speed, mirror, builds, battery, mode, whether a
  remote key started it), `frames` (each keyframe's `dur` and `hold` in
  ticks at 1x, and the loop), `play` (the trick's start on the body: the
  keyframes' times count from it), `key` (a remote key acted on while the
  run is open: which key stopped it), and `end` (`reason=DONE`,
  `STOPPED`, `PREEMPTED`, `FELL`, `FAILED` or `REFUSED`, the elapsed and
  expected time, battery, mode, `why=` the refusal or failure). A trick
  the remote refuses (the battery under 7400 mV, USB power, the player
  busy) is a run too: `trick`, `frames`, `end reason=REFUSED`;
- `RUN` records (`run_mark`: id, kind, BEGIN or END) around each run.

**The blocks** (`record/flight.h`): the stream is a ring of 16 KB blocks
in one allocation (`record::kStoreBytes`, 64 MB on the Pi,
`Mind::attach_recorder()`). A block is a `kilo_record_block_header_t`
(its number, the boot's number on the card, its length, the mind's clock
at its start, a CRC-32 of its records, the HEARTBEATs counted) and
records, each a `kilo_walk_record_header_t` (kind, len, t_us since the
block's start) and its payload; a record never spans two blocks. The
oldest block goes when the ring is full. A block is sealed (its CRC
written; the CRC is kept up as records come, so sealing costs nothing)
when the next record does not fit, when 35 minutes have passed since its
start, or when the saver wants it. The run index (`Recorder::run_after()`,
256 runs) gives each run's kind, flags, start, end and WALK_INFO text; a
run is RECORDING until 2 s after its end, and its fetch covers 2 s before
its start to 2 s after its end.

**The card** (`record/saver.h`, the `RecordCard` seam in `platform.h`):
see `kernel/README.md` "Saving to the SD card". In short: only at safe
moments (the Mind's `card_safe()`), one call of at most 4 KB a pass, each
timed, a back-off after a slow one, `.tmp` files renamed when whole,
64 MB kept, off until switched on. kilo-mind-host takes `--card DIR`
(`DirRecordCard`, DIR/kilorec); `tests/test_record.cpp` runs it on a fake
card (`tests/fake_card.h`) with slow writes, refusals and a power cut in
the middle of a save, then the next boot's scan.

**The download** (`HostServer`, `net/host_records.cpp`): `WALK_DATA`
chunks of 200 bytes go only into room in that client's 4 KB send ring, so
a laptop that reads slowly slows its download and is never dropped for
it, and the link is never touched: 8 chunks a pass, 1 while a run is being
recorded. Blocks the RAM no longer has come from the card (through the
saver, a call a pass at safe moments). On this laptop (`make -C
kernel/host bench`, `bench_walks`) a pass that downloads costs about
23 us at 1.6 KB a pass, 3.3 us at the one-chunk rate, against 0.1 us idle;
a walk tick's recording about 1 us.

**The metrics** (`record/metrics.h`, `Mind::metric_line()`): counters
since boot, served as `METRICS` lines (`kilo metrics --host`) and saved
with the log: `session` (uptime, builds, boot), `keys` (pressed, acted,
ignored, no action, held by a laptop, repeats, failed, and per key),
`walks` (started, ok, holds, catches, by end reason), `tricks` (started,
by end reason, by trick), `refusals` (by why), `body` (falls, recovered,
watchdog, low battery, events, reboots, battery now and lowest), `link`
(frames, heartbeats, decode errors, TX deferred and short), `record` (RAM
used, blocks dropped, records, runs, the loop's slowest pass and period)
and `card` (state, saved, unsaved, bytes, files, writes, the slowest
write, slow writes, errors).

**The RAM log** (`util/logring.h`) keeps 4096 lines of history once the
platform attaches the storage (`Mind::attach_log()`). Lines that come many
times a second go through `LogRing::status()` and are never history: a
held IR key's EVENT lines (`event IR_KEY 0x01xx`, every ~110 ms) and its
`repeat: ignored` lines, the percept lines, the platform's once-a-second
counters; a laptop reading the log gets them live (their `seq` has bit 31
set), the flight recorder keeps them. A first press, another remote's
key, and what the remote did with a key are history. At the end of every
walk that streamed, the walker logs its summary in two lines at Info.

## The Mind

`mind.h` puts the whole core in one object, so a platform's main loop is
a few lines (`kernel/host/main.cpp`):

```cpp
MindConfig cfg;                       // remote, behaviour, balance_kp, seed, stats_period_ms ...
static Mind mind(serial, clock, platform_log, &listener, cfg);  // ~135 KB: static; listener null = no laptop
mind.attach_log(new LogEntry[LogRing::kLines], LogRing::kLines);  // the log's history (700 KB)
mind.attach_recorder(new uint8_t[record::kStoreBytes], record::kStoreBytes);  // the flight recorder (64 MB)
mind.attach_saver(*new record::Saver(clock, log, mind.recorder(), mind.log(), card, false));  // with an SD card
mind.start();                         // the port is open: start goals, HELLO_REQ
for (;;) mind.poll();                 // a few ms apart; never blocks
```

One pass: stamp the link's liveness, serve the laptop, then, unless the
raw bridge is open, the flasher has the UART or a kernel update is
writing the SD card, poll the link, hand the command slot's answer to its
owner (the STATS poll) before anything may send another request, and run
the walker, the trick player, the remote and the behaviour runner. Every
TELEMETRY feeds perception and the trick player's clock of the body; every
EVENT goes to the walker, the player, the snapshot's last 8 and, for
IR_KEY, the remote.

**Liveness.** `Mind::poll()` stamps `link.mark_alive()` once per pass.
Every part runs in this one cooperative loop, so a pass that returns
proves nothing is stuck, and a part that hangs stops the stamp, which is
what the body's watchdog needs. One check more gates the stamp when the
runner is on: a tick within 1 s (10 s of grace before its first tick), because its 10 Hz grid
could stall while the loop runs.

**Who drives.** A laptop client holding control (`CONTROL take=1`) pauses
the behaviour runner and silences the remote. The remote's A5 starts the runner at run time;
while it runs, B6, A7 and C7 stop it (`Runner::stop()` halts the running
behaviour, which cancels its trick), A5 again plays one of the random mode's tricks
("The random mode's tricks" below) and its other keys are ignored.
`MindConfig` defaults: remote and behaviour off (the kernel turns on the
remote and the balance at Kp 0.99, kernel/README.md "The Mind's settings"),
so without `--behaviour` the runner is off until A5.

`kilo-mind-host` takes `--remote`, `--behaviour`, `--seed N` and
`--balance KP` besides its P1/P2 flags.

## Tricks

The body has no trick store or player any more (protocol 3, step 1 of
docs/movement-engine.md): the mind plays every trick through the body's
one stream of poses, POSE. The library is compiled in: a `Trick` is its
header fields and the 14 B keyframes exactly as Python's design tools make
them (`kilo.tricks.model`: `Trick.begin()`, `Keyframe.to_wire()`).
`tricks/gen_library.py` writes
`library.gen.{h,cpp}` from `mind/kilo/tricks/library/*.json` (26 tricks,
202 frames, about 3.6 KB of tables), with Python's CRC, trick_id and duration
for the tests:

```
~/OpenCat/.venv/bin/python kernel/mind/tricks/gen_library.py          # after changing the JSON
~/OpenCat/.venv/bin/python kernel/mind/tricks/gen_library.py --check  # test.sh
```

`TrickPlayer` (`tricks/player.h`) is a state machine with no command at
all. `KeySeq` turns a trick into keys in play order (per keyframe the end
of its move, eased as the frame says, a step's jump one tick in, the end
of its hold; the loop's repeats; mirror and speed applied here, `phase_ticks`
as before). The player waits for the body to be heard and for its clock
(TELEMETRY's tick, `planner::BodyClock`), clears the latched SET_POSTURE
goal and holds that goal type back while the trick plays (`SerialLink::
hold_goal`: a posture ends a stream on the body, so one asked for meanwhile
goes out when the trick ends; LIMP never waits), then sends each key as a
POSE with every leg as joint angles, the head joints the trick sets, the
frame's balance flag with the trick's expected attitude as the reference,
FALL_OK for a get-up. The first key is reached from wherever the joints
are; each next one goes out once the one before it is within the look-ahead
(3 ticks) of the body's tick, so the body's 5-deep queue always holds the
next. When the body's TELEMETRY tick reaches the last key the trick's end
posture goes out as the SET_POSTURE goal (over XFADE_TICKS; a stop over its
`stop_ticks`), which ends the stream; once the body has left it the goal is
cleared and the play is Done (DONE, STOPPED, or PREEMPTED / FELL when the
body ended it). The guards (pitch and roll at guarded holds) read every
TELEMETRY as before. `start()` returns at once and `result()` reports the
play; a second play while one runs is refused until it is stopped.

## Perception

`perception/` ports `kilo/perception` (ultrasonic: median, range rate,
zones with hysteresis, approaching; IMU: tilted/level, picked_up/put_down,
tap, pet, startle; `Percepts`) with the same thresholds, windows and
hysteresis, in double. `link/body_state` gains `state.py`'s derived values
(distance median and approach speed, battery percent, fallen, upright,
moving). The Mind feeds `Percepts` on every TELEMETRY with its arrival
time and logs each percept event.

The golden test replays `mind/tests/test_perception.py`'s traces through
both: `kernel/host/tests/gen_perception_vectors.py` writes them (quantised
to TELEMETRY's 1/16 degree and whole mm) with Python's events, and every
event must come out on the same sample. Sums are Python's (`util/dmath.h`:
Neumaier `sum()`, exact `fsum` for `fmean`), so they round alike.

What differs: fixed rings instead of deques (the IMU history holds 256
samples, 4.5 s at up to 56 Hz); the 10 s `TelemetryRing` is not ported (no
detector reads it).

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
perception's flags are the percepts (face always false). Each posture,
look and trick first quiets the IMU detectors (0.5, 0.3, 2 s).

What differs: no JSONL behaviour log (a log line when the choice changes,
body events and the behaviours' postures as log lines); `poll()` keeps the
10 Hz grid instead of an asyncio loop; the wind-down's poweroff only stops
the runner and sets `Mind::poweroff_requested()`; `beep` is not in the body.
Python's `play:` action fails after every real
trick (`_as_status` rejects the `TrickResult` that `BodyClient.play`
returns); here a trick that ends DONE succeeds.

### The random mode's tricks

The random mode is the behaviour runner started by A5 (or by the distance
guard). While it idles he sits, and while it explores he stands; the alive
layer (`behaviour/alive.h`, no Python counterpart) then makes small moves:
shifts, leans, a tail swish, looks around. Now and then, instead of a small
move, he does one of the original Nybble's tricks:

| Trick | What he does | From sitting | From standing |
|---|---|---|---|
| stretch | front low, rear high, head up (the original's `str`), held 2 s; at the end of the hold his rear shakes a little (3 quick swings, the hind feet 4 mm forward and back, the tail with them: not the original's). Made by the movement engine from the body model (`engine/stretch.h`) | goes straight into it and straight back to sitting | as the original |
| wash_face | sits and lifts his left front paw beside his muzzle three times, then puts it down (`wsf`; the original's head turned into the paw, which bumped his head, so here it faces straight ahead, 14 mm from the paw) | as the original | the movement engine's paw wash (`engine/paw.h`): standing on three feet, a front paw up beside his face, two to four strokes |
| pee | lifts a hind leg (`pee`) | stands up first | as the original |
| sniff | head down, sniffing the floor (`snf`) | stands up first | as the original |
| check_around | looks round to each side (`ck`) | stands up first | as the original |

- Each trick ends exactly in the pose it started from (SIT or STAND; standing
  he goes back to STANCE afterwards), so he never jumps at the end.
- But for the stretch and the standing face wash (the movement engine's, made
  each time from the body model: docs/movement-engine.md step 4), they are the
  library's `alive_<name>_sit` and `alive_<name>_stand`, written by
  `mind/kilo/tricks/alive_set.py` from the original skill tables with the
  original's timing (`mind/README.md` "The original firmware's trick timing"), each checked against the
  body model (`mind/tests/test_alive_tricks.py`): standing on his feet, and the
  head at least 10 mm from the front legs whichever way the head's pan turns
  (that way is not known). Left out: `butt_up` (it asks
  for more than the front shoulders reach and the model puts him on three feet),
  `scratch` (the model has him tip 7.7 mm onto a foot when the hind leg comes up),
  `hi` and `push_up` (not a cat's idle moves).
- How often: `personality.yaml` `alive:` section. `trick_every_s: 40` is the
  mean wait between two tricks (each wait drawn between 20 and 60 s); the small
  moves come every few seconds, so a trick is about one move in ten. Never the
  same trick twice in a row.
- How fast: `trick_speed: 0.75`, three quarters of the original's pace (a first,
  careful setting; 1.0 is the original's).
- A5 while the random mode runs (however it started): one trick at random at
  once. If a small move or another trick is playing, the new one waits for it to
  end (it is not cut short); one waits at most, so a press while one is already
  waiting is refused (the log says so). If he is napping or busy with another
  behaviour, it waits up to 10 s for him to sit or stand, then is dropped.
- B6, A7 and C7 stop a trick as they stop the random mode. For the rest of the
  mind a trick is a trick: he is quiet during it, and the distance guard waits
  for its end (6 s at the longest, wash_face) before it acts.

To try it on Kilo (nothing here has been tried on the robot; the model checks
are a rigid, slow-motion model without inertia):

1. Put Kilo on the floor, away from edges, and press **A5**. He sits and looks
   around, shifting now and then.
2. Wait. Within about a minute he does one of the tricks above, then goes on
   with the small moves.
3. Press **A5** again: he does a trick at once (or right after the move he is
   in). Press it again for another.
4. **B6** stops it all and he stands.

## Remote

`remote/` holds the key table and map (keys named by grid:
columns A-C left to right, rows 1-7 from the bottom; `remote/keys.h`) and
`RemoteControl`, driving the walker, the postures, the trick player and the
behaviour runner (`remote/remote.h` says what each key does):

| Key | Does |
|---|---|
| B6 | stand and stop: the quick stop of a walk (the feet in the air put down where they are, the body stopped in about 0.2 s, the legs squared up under the shoulders; docs/walking-and-balance.md §14), then STAND |
| A7 | sleep: the quick stop, then REST; 30 s later, if nothing else happened, light sleep (servos off, the Pi idles; any key or the laptop wakes him: `power/sleep.h`, docs/power.md) |
| C5 | the quick stop, then the calibration posture (CALIB) |
| C5 again (no other key between) | the self-measurement (`measure/`, docs/self-measurement.md): he stands (STAND), and once he is still the routine starts: small poses streamed as POSE (four feet) with the balance off, recorded as a run of its own; any key stops it into STAND; refused (logged, he stands) if he is not still within 6 s, below 7400 mV at rest or on USB power |
| A4, B4, C4 | trot, crawl, walk: walk forward in that gait from standing; change the gait while walking; the gait in use: straight forward again |
| B7, B5 | faster, slower by one speed level (the trot 0.4, 0.7, 1, 1.25, 1.4 × its speed: 58 to 202 mm/s, its swing 8 ticks up to +3, 10 and 11 at +4 and +5 (`engine/gait.h`, docs/walking-and-balance.md §15); the walk and the crawl 0.4, 0.7, 1, 1.15, 1.25); past the slowest B5 goes backward (the trot to -3 only, 144 mm/s); only while walking. Turning in place: stop turning and walk straight forward (B7) or backward (B5) at once |
| A6, C6 | from standing: turn left, right in place; walking forward or backward: steer one level left or right (0.25, 0.5, 1 of the stride off the inner side, at most 3 each way; the trot at +4 and +5 one, a sharper steer eased to it), without stopping (docs/walking-and-balance.md §13) |
| C7 | freeze: the quick stop of the walk (as B6, ending in STANCE, powered), a trick and the behaviour runner stopped; then only B6 and A7 act |
| A5 | start the behaviour runner (the random mode); while it runs: one of its tricks at random, at once or after the move playing ("The random mode's tricks") |
| A1 | an experiment key (row 1: one experiment a key): off unless built with `KILO_REAR_UP=1`, `2` or `3` (the highest stage allowed): rear up (docs/rear-up-trick.md): up to stage 1 (28 degrees nose up) and hold; A1 again while he holds a stage (4 s) takes him on to the next (40, then 54 degrees), up to the build's; without it he comes down to SIT from there; the top stage holds 2 s and comes down. A1 at another moment, B6 or C7: down at once (1.5 to 2.5 s); refused below 7400 mV at rest (the highest reading of the last 20 s). A2 until 2026-11-22, B2 until 2026-11-17 |
| B2 | a meow: one of the meow family, picked at random (`sound/`, docs/sound.md); it moves nothing, so it plays at once, also while a walk or the behaviour runs |
| B1 | the righting experiment (docs/righting.md), only in a kernel built with `KILO_RIGHTING=1`: lying on a side or his back, he rights himself by the righting moves (`intent/`, played as POSE streams that take the joints from the body's fall reflex), then stands; otherwise refused with a log line; any other key stops it. The body has no get-up of its own: without this he lies where he fell |
| C1 | an experiment key, free; nothing (logged as ignored) |
| rows 1-3 but A1, B1 and B2 | nothing (logged as ignored) |

With intents on (`MindConfig.intent`, `KILO_INTENT=1`; docs/intent.md) the posture of B6 and of A7 is not the
SET_POSTURE goal but `be(STANDING)` and `be(RESTING)` (`intent/intent.h`, through the remote's `PostureHook`):
the log line is the same (`remote B6 (0x02, stand): posture stand`), followed by the intent's own lines.

The remote's waits are phases of `poll()` and keys that come
meanwhile queue (4); C7 does not queue: it cancels
the action under way and drops the keys behind it (each logged). Repeats and
other remotes' keys are ignored; there is no shutdown key.
Every key leaves one line, `remote B6 (0x02, stand): posture stand`, naming
the key by grid and by what it does.

Integration (`kernel/host/test.sh`): `python -m kilo.vbody.core --control
FIFO` takes `ir CODE`, `att P R`, `batt MV` and `imu ...` lines
(`CoreBody.serve_control`; the harness has no IR decoder, so IR_KEY frames
are made in Python), and the script by IR keys stands, trots, speeds up
and slows down, steers, stands after the quick stop, freezes a trot (the
quick stop, the virtual body in STANCE), rests, takes the
calibration posture and starts and stops the behaviour runner, idles the behaviour
runner 5 s at 10 Hz, and tilts and drops the virtual body for perception
and the fallen branch.
