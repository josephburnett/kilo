// kilo-mind-host: the portable mind core's body link on Linux, for the
// virtual body (python -m kilo.vbody.core prints its pty) or a NyBoard on a
// USB adapter. It does what the Circle kernel does in phase P1
// (docs/fast-boot.md): exchange HELLO, keep the link fed (HEARTBEAT), and
// log link statistics once a second, with a STATS_REQ to the body each
// second to exercise the command path. With --listen it also serves the
// laptop channel of phase P2 (mind/net/host_server.h) on a TCP port, for
// `kilo --host 127.0.0.1:PORT`; it logs "listening on port N" (port 0 picks
// a free one).
//
//   kilo-mind-host --port /dev/pts/N [--listen [ADDR:]PORT] [--duration S] [--verbose]
//
// Exit status: 0 after --duration (or SIGINT/SIGTERM) with a HELLO seen;
// 1 no HELLO, or the port failed; 2 the body speaks another protocol major
// (refusing to drive); 64 bad usage.
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "link/names.h"
#include "link/serial_link.h"
#include "net/host_server.h"
#include "net_posix.h"
#include "posix.h"
#include "util/log.h"
#include "util/logring.h"
#include "util/mem.h"

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

void log_body_stats(Log& log, const Request& r) {
  if (r.reply_len != sizeof(kilo_stats_t)) return;
  kilo_stats_t st;
  mem_copy(&st, r.reply, sizeof st);
  logf(log, LogLevel::Info,
       "body STATS: uptime %u s overruns %u max tick %u us rx_crc_err %u rx_overflow %u feet_late %u "
       "feet_underruns %u",
       (unsigned)st.uptime_s, st.overruns, st.max_tick_us, st.rx_crc_err, st.rx_overflow, st.feet_late,
       st.feet_underruns);
}

int usage() {
  fprintf(stderr, "usage: kilo-mind-host --port PATH [--listen [ADDR:]PORT] [--duration SECONDS] [--verbose]\n");
  return 64;
}

}  // namespace

int main(int argc, char** argv) {
  const char* port = nullptr;
  const char* listen = nullptr;
  double duration_s = -1;
  bool verbose = false;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      port = argv[++i];
    } else if (strcmp(argv[i], "--listen") == 0 && i + 1 < argc) {
      listen = argv[++i];
    } else if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
      char* end;
      duration_s = strtod(argv[++i], &end);
      if (*end != '\0' || duration_s < 0) return usage();
    } else if (strcmp(argv[i], "--verbose") == 0) {
      verbose = true;
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

  MonotonicClock clock;
  StderrLog stderr_log(clock, verbose ? LogLevel::Debug : LogLevel::Info);
  LogRing log(clock, &stderr_log);  // every line: the RAM ring (`kilo log --host`), then stderr
  PosixSerial serial;
  if (!serial.open(port)) {
    logf(log, LogLevel::Error, "%s", serial.error());
    return 1;
  }
  if (!serial.set_baud(KILO_BAUD)) logf(log, LogLevel::Warn, "cannot set %u baud on %s", (unsigned)KILO_BAUD, port);
  logf(log, LogLevel::Info, "port %s at %u baud", port, (unsigned)KILO_BAUD);

  SerialLink link(serial, clock, log);
  Window window(clock);
  link.set_listener(&window);
  link.connect();

  PosixListener listener;
  static HostServer host(listener, link, serial, clock, log, log, (uint32_t)KILO_MIND_BUILD);  // static: 18 KB of buffers
  HostServer* server = nullptr;  // only with --listen
  if (listen != nullptr) {
    if (!listener.open(listen)) {
      logf(log, LogLevel::Error, "%s", listener.error());
      return 1;
    }
    server = &host;
    logf(log, LogLevel::Info, "listening on port %u (--listen %s; mind build %08x)", (unsigned)listener.port(),
         listen, (unsigned)KILO_MIND_BUILD);
  }

  const uint64_t start = clock.now_us();
  const uint64_t end = duration_s >= 0 ? start + (uint64_t)(duration_s * 1e6) : UINT64_MAX;
  uint64_t next_report = start + 1000000u;
  int status = 0;
  bool stats_asked = false;
  while (!g_stop && clock.now_us() < end) {
    link.mark_alive();  // this loop is the whole mind: it is alive while it runs
    if (server != nullptr) server->poll();
    bool bridged = server != nullptr && server->bridge_open();
    if (!bridged) link.poll();  // paused while the laptop speaks to the body through the bridge
    if (!link.connected()) {
      logf(log, LogLevel::Error, "port %s failed", port);
      status = 1;
      break;
    }
    if (link.state().proto_ok == 0) {
      status = 2;
      break;
    }
    const Request& r = link.req();
    if (stats_asked && r.state != RequestState::Waiting) {
      stats_asked = false;
      if (r.state == RequestState::Done) log_body_stats(log, r);
    }
    uint64_t now = clock.now_us();
    if (now >= next_report) {
      next_report += 1000000u;
      log_stats(log, link, window);
      window.reset();
      if (link.state().has_hello && !stats_asked && !bridged)
        stats_asked = link.request(KILO_MSG_STATS_REQ, nullptr, 0);
    }
    struct pollfd p[2 + PosixListener::kSlots];
    nfds_t np = 0;
    p[np++] = {serial.fd(), POLLIN, 0};
    if (server != nullptr) {
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
  listener.close();
  serial.close();
  return status;
}
