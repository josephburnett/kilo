// The walk recorder's cost on this machine (`make bench`), optimised as the
// kernel is, no sanitizers: what a walk tick adds to the mind's pass
// (the link's frame tap for TELEMETRY in and FEET out, a PLAN record), and a
// host server pass while a client downloads a 5 MB walk (WALK_FETCH, at the
// idle rate and at the one-chunk rate used while a walk is recorded),
// against a pass with the same client connected and idle. The Pi's
// Cortex-A7 is some 10-20 times slower per instruction than a laptop core.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>
#include "net/host_server.h"
#include "record/recorder.h"
#include "tests/check.h"
#include "tests/fake_net.h"
#include "tests/fakes.h"
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
    uint32_t n;
    const AuthorizedKey* keys = test_keys(&n);
    server.set_keys(keys, n, random);
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
  record::WalkRecorder rec(r.clock, r.link);
  rec.attach(mem.data(), (uint32_t)mem.size(), 0);
  record::WalkStore& st = rec.store();

  // a walk tick's recording: TELEMETRY in, FEET out (the tap), a PLAN record
  st.open(0);
  uint8_t tel[KILO_TELEMETRY_LEN] = {}, feet[KILO_FEET_LEN] = {};
  kilo_walk_plan_t plan = {};
  Stat tick;
  const int ticks = 60000;  // 20 minutes
  for (int t = 0; t < ticks; t++) {
    double a = now_us();
    rec.on_frame(false, KILO_MSG_TELEMETRY, (uint8_t)t, tel, sizeof tel, (uint64_t)t * 20000);
    rec.on_frame(true, KILO_MSG_FEET, (uint8_t)t, feet, sizeof feet, (uint64_t)t * 20000);
    // what the recorder appends for them while a walk is recorded, and the PLAN
    uint8_t ts[2] = {KILO_MSG_TELEMETRY, (uint8_t)t};
    st.append(KILO_WALK_RECORD_RX, (uint32_t)t * 20000u, ts, 2, tel, sizeof tel);
    ts[0] = KILO_MSG_FEET;
    st.append(KILO_WALK_RECORD_TX, (uint32_t)t * 20000u, ts, 2, feet, sizeof feet);
    st.append(KILO_WALK_RECORD_PLAN, (uint32_t)t * 20000u, &plan, sizeof plan);
    tick.add(now_us() - a);
  }
  st.close();
  std::printf("recorder: a walk tick (2 frames into the pre-roll and the walk, a PLAN) %.2f us mean, %.1f us max\n",
              tick.mean(), tick.max);
  // a 5 MB walk to download (about 18 minutes of walking)
  st.open(0);
  uint8_t p[60] = {};
  while (st.current()->size < 5u << 20) st.append(KILO_WALK_RECORD_RX, 0, p, sizeof p);
  uint32_t id = st.current()->id;
  st.close();
  r.server.attach_walks(st);
  FakeConn& c = r.listener.connect(0);
  for (int k = 0; k < 10 && !c.established; k++) {
    r.pass();
    c.pump();
  }
  r.pass();
  c.take();

  Stat idle;
  for (int k = 0; k < 2000; k++) {
    double a = now_us();
    r.pass();
    idle.add(now_us() - a);
    c.take();
  }
  for (int rate = 0; rate < 2; rate++) {
    if (rate == 1) st.open(0);  // a walk being recorded: one chunk a pass
    kilo_host_walk_fetch_t q = {id, 0};
    c.send(KILO_HOST_MSG_WALK_FETCH, (uint8_t)(1 + rate), &q, sizeof q);
    Stat fetch;
    size_t bytes = 0;
    int passes = 0;
    bool end = false;
    while (!end && passes < 200000) {
      double a = now_us();
      r.pass();
      fetch.add(now_us() - a);
      passes++;
      for (const HostFrame& f : c.take()) {
        if (f.type != KILO_HOST_MSG_WALK_DATA) continue;
        bytes += f.len - KILO_HOST_WALK_DATA_MIN_LEN;
        if (f.payload[12] == KILO_WALK_DATA_STATUS_END) end = true;
      }
      if (rate == 1 && passes == 5000) break;  // enough for the per-pass cost
    }
    std::printf("host pass, %s: %.1f us mean, %.1f us max (idle client: %.1f / %.1f us); %.0f B a pass, %s\n",
                rate == 0 ? "a walk downloading" : "a walk downloading while one is recorded", fetch.mean(),
                fetch.max, idle.mean(), idle.max, (double)bytes / passes,
                end ? "the whole 5 MB" : "stopped after 5000 passes");
    if (rate == 1) {
      kilo_host_walk_fetch_t stop = {0, 0};
      c.send(KILO_HOST_MSG_WALK_FETCH, 9, &stop, sizeof stop);
      r.pass();
      c.take();
    }
  }
  return report("bench walks");  // the fakes' CHECKs: nonzero if one failed
}
