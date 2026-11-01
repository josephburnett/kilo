// walk-demo: the portable mind core walks the virtual body (python -m
// kilo.vbody.core prints its pty) or a NyBoard: HELLO, then a walk for
// --seconds, then the planned stop, with the planner (kernel/mind/planner)
// streaming FEET five ticks ahead as the Circle kernel will.
//
//   walk-demo --port /dev/pts/N [--seconds S] [--pattern trot|walk|crawl]
//             [--direction forward|back|left|right|march] [--speed MM_S] [--verbose]
//
// It checks from TELEMETRY and EVENTs that the body walked: the tick
// advanced, GAIT was entered, the support flags cycled through the
// pattern's lift groups, no GOAL_REFUSED or FELL, and the walk ended with
// the planned stop. Exit status 0 when all held, 1 when one failed or no
// body answered, 64 bad usage.
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "link/names.h"
#include "link/serial_link.h"
#include "planner/walker.h"
#include "posix.h"
#include "util/log.h"

using namespace kilo;
using namespace kilo::planner;

namespace {

volatile sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

// What the body reported, forwarded to the walker.
struct Watch : LinkListener {
  explicit Watch(Walker& w) : walker(w) {}
  void on_telemetry(const kilo_telemetry_t& t) override {
    walker.on_telemetry(t);
    if (t.mode == KILO_MODE_GAIT) {
      gait_frames++;
      supports |= 1u << (t.support & 0xF);  // which support sets were seen
      if (t.feet_queued > max_queued) max_queued = t.feet_queued;
    }
  }
  void on_event(const kilo_event_t& e) override {
    walker.on_event(e);
    if (e.code == KILO_EVENT_GOAL_REFUSED) refused++;
    if (e.code == KILO_EVENT_FELL) fell++;
    if (e.code == KILO_EVENT_FEET_UNDERRUN) underruns++;
  }
  Walker& walker;
  uint32_t gait_frames = 0;
  uint32_t supports = 0;
  uint32_t refused = 0, fell = 0, underruns = 0;
  uint8_t max_queued = 0;
};

int usage() {
  fprintf(stderr,
          "usage: walk-demo --port PATH [--seconds S] [--pattern NAME] [--direction NAME] [--speed MM_S] "
          "[--verbose]\n");
  return 64;
}

int popcount(uint32_t v) { return __builtin_popcount(v); }

}  // namespace

int main(int argc, char** argv) {
  const char* port = nullptr;
  double seconds = 3;
  bool verbose = false;
  WalkGoal goal;
  for (int i = 1; i < argc; i++) {
    const char* a = argv[i];
    const char* v = i + 1 < argc ? argv[i + 1] : nullptr;
    if (strcmp(a, "--verbose") == 0) {
      verbose = true;
    } else if (v == nullptr) {
      return usage();
    } else if (strcmp(a, "--port") == 0) {
      port = argv[++i];
    } else if (strcmp(a, "--seconds") == 0) {
      seconds = atof(argv[++i]);
    } else if (strcmp(a, "--pattern") == 0) {
      goal.pattern = find_pattern(argv[++i]);
      if (goal.pattern == nullptr) return usage();
    } else if (strcmp(a, "--direction") == 0) {
      if (!find_direction(argv[++i], &goal.direction)) return usage();
    } else if (strcmp(a, "--speed") == 0) {
      goal.speed_mm_s = (real)atof(argv[++i]);
    } else {
      return usage();
    }
  }
  if (port == nullptr || seconds <= 0) return usage();

  struct sigaction sa;
  memset(&sa, 0, sizeof sa);
  sa.sa_handler = on_signal;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  MonotonicClock clock;
  StderrLog log(clock, verbose ? LogLevel::Debug : LogLevel::Info);
  PosixSerial serial;
  if (!serial.open(port)) {
    logf(log, LogLevel::Error, "%s", serial.error());
    return 1;
  }
  serial.set_baud(KILO_BAUD);
  SerialLink link(serial, clock, log);
  static Walker walker(link, clock, log);  // static: the walker holds the planner, a few KB
  Watch watch(walker);
  link.set_listener(&watch);
  link.connect();

  // The mind's loop: link, walker, a 2 ms sleep (or input).
  auto step = [&]() {
    link.mark_alive();
    link.poll();
    walker.poll();
    struct pollfd p = {serial.fd(), POLLIN, 0};
    poll(&p, 1, 1);
    return link.connected() && !g_stop;
  };

  uint64_t t0 = clock.now_us();
  while (step() && !(link.contact() && link.state().has_telemetry && link.state().epoch != kNoEpoch))
    if (clock.now_us() - t0 > 5000000u) break;
  if (!link.contact() || !link.state().has_telemetry) {
    logf(log, LogLevel::Error, "no HELLO and TELEMETRY from the body");
    return 1;
  }
  uint32_t tick0 = link.state().tick;
  const char* err = walker.start(goal);
  if (err != nullptr) {
    logf(log, LogLevel::Error, "walk refused: %s", err);
    return 1;
  }
  uint64_t walk_at = clock.now_us();
  uint64_t stop_at = walk_at + (uint64_t)(seconds * 1e6) + kStanceWaitUs;
  uint64_t report_at = walk_at + 1000000u;
  bool stopped = false;
  while (step()) {
    uint64_t now = clock.now_us();
    if (!stopped && now >= stop_at) {
      walker.stop();
      stopped = true;
      logf(log, LogLevel::Info, "stop");
    }
    if (now >= report_at) {
      report_at += 1000000u;
      walker.log_status(LogLevel::Info);
    }
    if (!walker.active()) break;
    if (now - walk_at > (uint64_t)(seconds * 1e6) + 15000000u) {
      logf(log, LogLevel::Error, "the walk did not end");
      break;
    }
  }
  walker.log_status(LogLevel::Info);
  uint32_t ticks = link.state().tick - tick0;
  const StreamStats& s = walker.stream_stats();
  logf(log, LogLevel::Info, "walked: body ticks %u, GAIT frames %u, support sets %d (mask %04x), ended: %s", ticks,
       watch.gait_frames, popcount(watch.supports), watch.supports, walker.ended() ? walker.ended() : "(running)");
  logf(log, LogLevel::Info, "FEET sent %u skipped %u max behind %u, body queue max %u, underruns %u, refused %u, fell %u",
       s.sent, s.skipped, s.max_behind, watch.max_queued, watch.underruns, watch.refused, watch.fell);
  serial.close();

  int fails = 0;
  auto expect = [&](bool ok, const char* what) {
    if (!ok) {
      logf(log, LogLevel::Error, "FAILED: %s", what);
      fails++;
    }
  };
  expect(ticks >= (uint32_t)(seconds * 50 * 0.8), "the body's tick advanced");
  expect(watch.gait_frames >= (uint32_t)(seconds * 50 * 0.5), "TELEMETRY in GAIT");
  // trot: all four (0xF) and the two diagonal pairs; walk, crawl: four and three feet
  expect(popcount(watch.supports) >= 3, "the support flags cycled");
  expect(watch.refused == 0, "no GOAL_REFUSED");
  expect(watch.fell == 0, "no FELL");
  expect(walker.ended() != nullptr && walker.ended_ok(), "the planned stop");
  expect(!g_stop, "not interrupted");
  return fails ? 1 : 0;
}
