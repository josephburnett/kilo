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
util/fmt, log     a small printf (%d %u %x %s %c %%, width, '0') and log lines through Log
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
