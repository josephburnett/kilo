# kernel/mind: the portable mind core

The part of the mind that runs both in the Circle kernel on the Pi 2 B and
on the laptop (`kernel/host/`), against the virtual body. Phase P1 of
`docs/fast-boot.md`: the body link. Platform-free C++17: no exceptions, RTTI,
heap, `std::`, `printf` or libm; the only libc functions are
`memcpy` and `memset`, through the compiler's builtins (`util/mem.h`).
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
  if (!host.bridge_open()) link.poll();       // the raw bridge pauses the link
}
```

| Channel | Messages | Who |
|---|---|---|
| Commands | `STATUS_REQ` -> `STATUS`; `CONTROL`, `POSTURE`, `TELEM_SUB`, `LOG_SUB`, `BRIDGE_OPEN`/`_CLOSE` -> `RESULT`; `PARAM_GET`/`_SET` answer `UNSUPPORTED` until P3 | any client; `POSTURE`, `PARAM_SET` and the bridge need control |
| Telemetry | `TELEM` at 1-50 Hz: the latest TELEMETRY, last EVENT, link counters | any client |
| Log | `LOG_LINE`: the last N lines of the ring, then live; seq gaps are lost lines | any client |
| Raw bridge | `BRIDGE_TX`/`BRIDGE_RX`: UART bytes, the link paused | the controller |
| Flash (P4) | ids 0x40-0x4F and 0xC0-0xCF reserved, nothing defined | |

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
