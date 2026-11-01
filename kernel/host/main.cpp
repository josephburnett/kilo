// kilo-mind-host: the portable mind core (mind/mind.h) on Linux, for the
// virtual body (python -m kilo.vbody.core prints its pty) or a NyBoard on a
// USB adapter. It does what the Circle kernel does in phase P1
// (docs/fast-boot.md): exchange HELLO, keep the link fed (HEARTBEAT), and
// log link statistics once a second, with a STATS_REQ to the body each
// second to exercise the command path. With --listen it also serves the
// laptop channel of phase P2 (mind/net/host_server.h) on a TCP port, for
// `kilo --host 127.0.0.1:PORT`; it logs "listening on port N" (port 0 picks
// a free one). The laptop channel includes flashing the body (P4,
// mind/flash/flasher.h): the body's RESET line is the named pipe given by
// --reset-fifo (posix.h FifoResetGpio), which the fake optiboot body of
// kernel/host/tests/fake_optiboot.py reads.
//
//   kilo-mind-host --port /dev/pts/N [--listen [ADDR:]PORT] [--reset-fifo PATH] [--duration S] [--verbose]

// USB adapter. It runs what the Circle kernel runs: the link (HELLO,
// HEARTBEAT, goals), the walker, the trick player and perception, and with
// --remote the IR remote, with --behaviour the behaviour runner; it logs
// link statistics once a second, with a STATS_REQ to the body each second
// to exercise the command path. With --listen it also serves the laptop
// channel of phase P2 (mind/net/host_server.h) on a TCP port, for
// `kilo --host 127.0.0.1:PORT`; it logs "listening on port N" (port 0
// picks a free one).
//
//   kilo-mind-host --port /dev/pts/N [--listen [ADDR:]PORT] [--duration S] [--verbose]
//                  [--remote] [--behaviour] [--seed N] [--balance KP]
//
// Exit status: 0 after --duration (or SIGINT/SIGTERM) with a HELLO seen;
// 1 no HELLO, or the port failed; 2 the body speaks another protocol major
// (refusing to drive); 64 bad usage.
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "link/names.h"
#include "flash/flasher.h"
#include "flash/image.h"
#include "link/serial_link.h"
#include "net/host_server.h"

#include "mind.h"
#include "net_posix.h"
#include "posix.h"
#include "util/log.h"

#ifndef KILO_MIND_BUILD
#define KILO_MIND_BUILD 0
#endif

using namespace kilo;

namespace {

volatile sig_atomic_t g_stop = 0;

void on_signal(int) { g_stop = 1; }

// Telemetry seen in the current one-second window.
struct Window : LinkListener {
  explicit Window(Clock& clock) : clock_(clock) {}

  void on_telemetry(const kilo_telemetry_t& t) override {
    uint64_t now = clock_.now_us();
    if (have_last) {
      uint64_t gap = now - last_us;
      if (gap > max_gap_us) max_gap_us = gap;
    }
    have_last = true;
    last_us = now;
    frames++;
    if (t.flags & KILO_TELEMETRY_FLAGS_OVERRUN) overruns++;
    if (t.tick_load > max_load) max_load = t.tick_load;
  }

  void reset() {
    frames = 0;
    overruns = 0;
    max_load = 0;
    max_gap_us = 0;
  }

  Clock& clock_;
  bool have_last = false;
  uint64_t last_us = 0;
  uint32_t frames = 0;
  uint32_t overruns = 0;  // TELEMETRY with the OVERRUN flag (a body tick overran)
  uint8_t max_load = 0;   // TELEMETRY.tick_load, 100 us units
  uint64_t max_gap_us = 0;
};

void log_stats(Log& log, const SerialLink& link, const Window& w) {
  const LinkStats& s = link.stats();
  const BodyState& b = link.state();
  logf(log, LogLevel::Info,
       "link: telemetry %u/s (max gap %u ms, total %u) rx %u frames, tx %u (%u heartbeats) "
       "decode errors %u (crc %u cobs %u len %u+%u type %u overflow %u)",
       w.frames, (unsigned)(w.max_gap_us / 1000u), b.telemetry_count, s.frames_rx, s.frames_tx, s.heartbeats,
       s.decode_error_total(), s.decode_errors[KILO_FRAME_BAD_CRC], s.decode_errors[KILO_FRAME_BAD_COBS],
       s.decode_errors[KILO_FRAME_BAD_LENGTH], s.decode_errors[KILO_FRAME_BAD_PAYLOAD_LEN],
       s.decode_errors[KILO_FRAME_UNKNOWN_TYPE], s.decode_errors[KILO_FRAME_OVERFLOW]);
  if (b.has_telemetry)
    logf(log, LogLevel::Info, "body: mode %s epoch %u tick %u rx_err %u tick_load max %u.%u ms overruns %u/%u",
         mode_name(b.telemetry.mode), b.telemetry.epoch, b.tick, b.telemetry.rx_err, w.max_load / 10u,
         w.max_load % 10u, w.overruns, w.frames);
  logf(log, LogLevel::Info, "commands %u retries %u timeouts %u nacks %u dup acks %u unmatched acks %u tx short %u",
       s.commands, s.retries, s.timeouts, s.nacks, s.dup_acks, s.unmatched_acks, s.tx_short);
}

int usage() {
  fprintf(stderr,
          "usage: kilo-mind-host --port PATH [--listen [ADDR:]PORT] [--reset-fifo PATH] [--duration SECONDS] "
          "[--verbose]\n");

          "usage: kilo-mind-host --port PATH [--listen [ADDR:]PORT] [--duration SECONDS] [--verbose]\n"
          "                      [--remote] [--behaviour] [--seed N] [--balance KP]\n");
  return 64;
}

}  // namespace

int main(int argc, char** argv) {
  const char* port = nullptr;
  const char* listen = nullptr;
  const char* reset_fifo = nullptr;
  double duration_s = -1;
  bool verbose = false;
  MindConfig cfg;
  cfg.mind_build = (uint32_t)KILO_MIND_BUILD;
  cfg.stats_period_ms = 1000;
  bool seeded = false;
  for (int i = 1; i < argc; i++) {
    char* end;
    if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      port = argv[++i];
    } else if (strcmp(argv[i], "--listen") == 0 && i + 1 < argc) {
      listen = argv[++i];
    } else if (strcmp(argv[i], "--reset-fifo") == 0 && i + 1 < argc) {
      reset_fifo = argv[++i];
    } else if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
      duration_s = strtod(argv[++i], &end);
      if (*end != '\0' || duration_s < 0) return usage();
    } else if (strcmp(argv[i], "--verbose") == 0) {
      verbose = true;
    } else if (strcmp(argv[i], "--remote") == 0) {
      cfg.remote = true;
    } else if (strcmp(argv[i], "--behaviour") == 0) {
      cfg.behaviour = true;
    } else if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
      cfg.seed = strtoull(argv[++i], &end, 10);
      if (*end != '\0') return usage();
      seeded = true;
    } else if (strcmp(argv[i], "--balance") == 0 && i + 1 < argc) {
      cfg.balance_kp = strtod(argv[++i], &end);
      if (*end != '\0' || cfg.balance_kp < 0 || cfg.balance_kp > 0.996) return usage();
    } else {
      return usage();
    }
  }
  if (port == nullptr) return usage();

  struct sigaction sa;
  memset(&sa, 0, sizeof sa);
  sa.sa_handler = on_signal;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
  signal(SIGPIPE, SIG_IGN);  // a --reset-fifo reader that went away: write() fails with EPIPE instead

  MonotonicClock clock;
  StderrLog stderr_log(clock, verbose ? LogLevel::Debug : LogLevel::Info);
  if (!seeded) cfg.seed = clock.now_us() ^ ((uint64_t)getpid() << 20);  // Python: a random seed, logged
  PosixSerial serial;
  PosixListener listener;
  // static: the Mind holds ~60 KB of buffers (the laptop channel's, perception's windows)
  static Mind mind(serial, clock, stderr_log, listen != nullptr ? &listener : nullptr, cfg);
  Log& log = mind.log();  // every line: the RAM ring (`kilo log --host`), then stderr
  if (!serial.open(port)) {
    logf(log, LogLevel::Error, "%s", serial.error());
    return 1;
  }
  if (!serial.set_baud(KILO_BAUD)) logf(log, LogLevel::Warn, "cannot set %u baud on %s", (unsigned)KILO_BAUD, port);
  logf(log, LogLevel::Info, "port %s at %u baud", port, (unsigned)KILO_BAUD);

  Window window(clock);
  link.set_listener(&window);
  link.connect();

  PosixListener listener;
  static HostServer host(listener, link, serial, clock, log, log, (uint32_t)KILO_MIND_BUILD);  // static: 18 KB of buffers
  HostServer* server = nullptr;  // only with --listen
  FifoResetGpio gpio(reset_fifo, log);
  static FlashImage image;  // 32 KB
  static Flasher flasher(serial, clock, gpio, link, log);
  host.attach_flasher(flasher, image);

  mind.set_tap(&window);
  if (listen != nullptr) {
    if (!listener.open(listen)) {
      logf(log, LogLevel::Error, "%s", listener.error());
      return 1;
    }
    logf(log, LogLevel::Info, "listening on port %u (--listen %s; mind build %08x)", (unsigned)listener.port(),
         listen, (unsigned)KILO_MIND_BUILD);
  }
  mind.start();

  const uint64_t start = clock.now_us();
  const uint64_t end = duration_s >= 0 ? start + (uint64_t)(duration_s * 1e6) : UINT64_MAX;
  uint64_t next_report = start + 1000000u;
  int status = 0;
  SerialLink& link = mind.link();
  while (!g_stop && clock.now_us() < end) {
    link.mark_alive();  // this loop is the whole mind: it is alive while it runs
    if (server != nullptr) server->poll();
    bool bridged = server != nullptr && server->link_paused();
    if (!bridged) link.poll();  // paused while the bridge or the flasher has the UART

    mind.poll();
    if (!link.connected()) {
      logf(log, LogLevel::Error, "port %s failed", port);
      status = 1;
      break;
    }
    if (link.state().proto_ok == 0) {
      status = 2;
      break;
    }
    uint64_t now = clock.now_us();
    if (now >= next_report) {
      next_report += 1000000u;
      log_stats(log, link, window);
      window.reset();
    }
    struct pollfd p[2 + PosixListener::kSlots];
    nfds_t np = 0;
    p[np++] = {serial.fd(), POLLIN, 0};
    if (listen != nullptr) {
      p[np++] = {listener.fd(), POLLIN, 0};
      for (uint32_t s = 0; s < PosixListener::kSlots; s++) {
        PosixConn* c = static_cast<PosixConn*>(listener.conn(s));
        if (c->is_open()) p[np++] = {c->fd(), POLLIN, 0};
      }
    }
    poll(p, np, 2);  // wake on input, else every 2 ms for the timers
  }
  if (status == 0 && !link.state().has_hello) {
    logf(log, LogLevel::Error, "no HELLO from the body");
    status = 1;
  }
  log_stats(log, link, window);
  if (cfg.behaviour)
    logf(log, LogLevel::Info, "behaviour: %u ticks, %u overruns%s", (unsigned)mind.runner().ticks(),
         (unsigned)mind.runner().overruns(), mind.poweroff_requested() ? ", poweroff requested" : "");
  listener.close();
  serial.close();
  return status;
}
