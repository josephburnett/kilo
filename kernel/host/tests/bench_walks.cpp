// The flight recorder's cost on this machine (`make bench`), optimised as
// the kernel is, no sanitizers: what a walk tick adds to the mind's pass
// (the link's frame tap for TELEMETRY in and POSE out, a PLAN record, a
// block sealed now and then), and a host server pass while a client
// downloads 5 MB of blocks (REC_FETCH, at the idle rate and at the
// one-chunk rate used while a run is recorded), against a pass with the
// same client connected and idle. The Pi's Cortex-A7 is some 10-20 times
// slower per instruction than a laptop core.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>
#include "net/host_server.h"
#include "record/recorder.h"
#include "tests/check.h"
#include "tests/fake_net.h"
#include "tests/fakes.h"
#include "tests/host_rig.h"
#include "util/logring.h"

using namespace kilo;

namespace {

double now_us() {
  using namespace std::chrono;
  return (double)duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count() / 1000.0;
}

struct Stat {
  double sum = 0, max = 0;
  int n = 0;
  void add(double v) {
    sum += v;
    max = v > max ? v : max;
    n++;
  }
  double mean() const { return n ? sum / n : 0; }
};

struct Rig {
  FakeSerial serial;
  FakeClock clock;
  CaptureLog capture;
  LogRing ring{clock, &capture};
  SerialLink link{serial, clock, ring};
  FakeListener listener;
  FakeRandom random;
  HostServer server{listener, link, serial, clock, ring, ring, 0};
  Rig() {
    link.connect();
    use_test_keys(server, random);
  }
  void pass() {
    link.mark_alive();
    server.poll();
    link.poll();
  }
};

}  // namespace

int main() {
  std::vector<uint8_t> mem(record::kStoreBytes);
  Rig r;
  record::Recorder rec(r.clock, r.link);
  rec.attach(mem.data(), (uint32_t)mem.size(), 0);
  record::FlightStore& f = rec.flight();

  // a walk tick's recording: TELEMETRY in, POSE out (the tap), a PLAN record
  uint8_t tel[KILO_TELEMETRY_LEN] = {}, feet[KILO_POSE_LEN] = {};
  kilo_walk_plan_t plan = {};
  Stat tick;
  const int ticks = 60000;  // 20 minutes
  for (int t = 0; t < ticks; t++) {
    uint64_t at = 1000000 + (uint64_t)t * 20000;
    double a = now_us();
    rec.on_frame(false, KILO_MSG_TELEMETRY, (uint8_t)t, tel, sizeof tel, at);
    rec.on_frame(true, KILO_MSG_POSE, (uint8_t)t, feet, sizeof feet, at);
    f.append(KILO_WALK_RECORD_PLAN, at, &plan, sizeof plan);
    tick.add(now_us() - a);
  }
  std::printf("recorder: a walk tick (2 frames, a PLAN; a 16 KB block sealed every %u ticks) %.2f us mean, %.1f us max; "
              "20 minutes of walking: %.1f MB\n",
              (unsigned)(ticks / (f.next_seq() ? f.next_seq() : 1)), tick.mean(), tick.max,
              (double)f.used_bytes() / 1048576.0);
  r.clock.t = 1000000 + (uint64_t)ticks * 20000;
  r.server.attach_recorder(rec);
  FakeConn& c = connect_client(r.listener, 0, [&r] { r.pass(); });
  c.take();

  Stat idle;
  for (int k = 0; k < 2000; k++) {
    double a = now_us();
    r.pass();
    idle.add(now_us() - a);
    c.take();
  }
  for (int rate = 0; rate < 2; rate++) {
    if (rate == 1) rec.trick_refused("rear_up", "bench");  // a run in its post-roll: one chunk a pass
    kilo_host_rec_fetch_t q = {0, 0, 0xFFFFFFFFu, KILO_REC_FETCH_WHAT_BLOCKS};
    c.send(KILO_HOST_MSG_REC_FETCH, (uint8_t)(1 + rate), &q, sizeof q);
    Stat fetch;
    size_t bytes = 0;
    int passes = 0;
    bool end = false;
    while (!end && passes < 200000) {
      double a = now_us();
      r.pass();
      fetch.add(now_us() - a);
      passes++;
      for (const HostFrame& fr : c.take()) {
        if (fr.type != KILO_HOST_MSG_WALK_DATA) continue;
        bytes += fr.len - KILO_HOST_WALK_DATA_MIN_LEN;
        if (fr.payload[12] == KILO_WALK_DATA_STATUS_END) end = true;
      }
      if (rate == 1 && passes == 5000) break;  // enough for the per-pass cost
    }
    std::printf("host pass, %s: %.1f us mean, %.1f us max (idle client: %.1f / %.1f us); %.0f B a pass, %s\n",
                rate == 0 ? "the blocks downloading" : "the blocks downloading while a run is recorded", fetch.mean(),
                fetch.max, idle.mean(), idle.max, (double)bytes / passes,
                end ? "all of it" : "stopped after 5000 passes");
    if (rate == 1) {
      kilo_host_walk_fetch_t stop = {0, 0};
      c.send(KILO_HOST_MSG_WALK_FETCH, 9, &stop, sizeof stop);
      r.pass();
      c.take();
    }
  }
  return report("bench walks");  // the fakes' CHECKs: nonzero if one failed
}
