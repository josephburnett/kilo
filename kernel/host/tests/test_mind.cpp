// The Mind's composition on a scripted body: the remote acting on IR_KEY,
// ignored while the behaviour runner is active; the behaviour layer
// latching its posture through the link; percepts fed by every TELEMETRY;
// liveness stamped per pass, and held back when the runner stops ticking.
#include <cstdio>
#include <cstring>
#include "mind.h"
#include "tests/body_rig.h"
#include "tests/check.h"

using namespace kilo;

namespace {

struct Rig {
  FakeBody body;
  CaptureLog log;
  Mind mind;
  explicit Rig(const MindConfig& cfg) : mind(body.serial, body.clock, log, nullptr, cfg) {
    mind.start();
    run_ms(100);
  }
  void pass() {
    body.step();
    mind.poll();
    body.react();
  }
  void run_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i += 4) pass();
  }
  int posture_goal() {
    const Goal* g = mind.link().goal(KILO_MSG_SET_POSTURE);
    return g == nullptr ? -1 : g->payload[1];
  }
};

void test_remote_through_the_mind() {
  MindConfig cfg;
  cfg.remote = true;
  Rig r(cfg);
  r.body.event(KILO_EVENT_IR_KEY, 0x18);  // B3: sit
  r.run_ms(40);
  CHECK(r.posture_goal() == KILO_POSTURE_SIT && r.log.has("remote B3 (0x18, sit): posture sit"));
  r.body.event(KILO_EVENT_IR_KEY, 0x118);  // a held-key repeat: nothing
  r.body.event(KILO_EVENT_IR_KEY, 0x45);   // another remote's key: nothing
  r.body.event(KILO_EVENT_IR_KEY, 0xA2);   // A7: rest
  r.run_ms(40);
  CHECK(r.posture_goal() == KILO_POSTURE_REST);
  CHECK(r.log.has("remote B3 (0x18, sit) repeat: ignored") && r.log.has("remote ? (0x45): ignored"));
  // without --remote the keys do nothing
  Rig off{MindConfig()};
  off.body.event(KILO_EVENT_IR_KEY, 0x18);
  off.run_ms(40);
  CHECK(off.posture_goal() == -1);
}

void test_behaviour_through_the_mind() {
  MindConfig cfg;
  cfg.remote = true;
  cfg.behaviour = true;
  cfg.seed = 3;
  Rig r(cfg);
  r.run_ms(400);
  CHECK(r.posture_goal() == KILO_POSTURE_SIT && r.log.has("behaviour: idle (free)"));
  CHECK(r.log.has("behaviour: posture sit"));
  uint32_t ticks = r.mind.runner().ticks();
  CHECKF(ticks >= 4 && ticks <= 6, "%u ticks in 0.5 s", ticks);  // 10 Hz
  r.body.event(KILO_EVENT_IR_KEY, 0x10);
  r.run_ms(40);
  CHECK(r.log.has("remote A2 (0x10, hi): ignored (behaviour runner active)") && r.body.count(KILO_MSG_TRICK_PLAY) == 0);
  CHECK(r.mind.stamping());
  // the loop stalls 2 s: the pass after it does not stamp (the runner has not ticked for 1 s), then it does again
  r.body.clock.advance_ms(2000);
  r.mind.poll();
  CHECK(!r.mind.stamping() && r.log.has("no longer stamping"));
  r.pass();
  CHECK(r.mind.stamping() && r.log.has("ticking again"));
  // a fall: the safety branch
  r.body.mode = KILO_MODE_FALLEN;
  r.body.epoch++;
  r.body.event(KILO_EVENT_FELL, 2);
  r.run_ms(300);
  CHECK(r.log.has("behaviour: fallen (safety)"));
}

void test_percepts_and_snapshot() {
  Rig r{MindConfig()};
  r.body.pitch = 40 * 16;  // nose down 40 deg for 1 s: tilted
  r.run_ms(1000);
  CHECK(r.mind.percepts().tilted() && r.log.has("percept tilted nose_down"));
  r.body.pitch = 0;
  r.body.dist_mm = 300;
  r.run_ms(1000);
  CHECK(!r.mind.percepts().tilted() && r.log.has("percept level"));
  double d;
  CHECK(r.mind.link().state().distance_mm(&d) && d == 300.0);
}

}  // namespace

int main() {
  test_remote_through_the_mind();
  test_behaviour_through_the_mind();
  test_percepts_and_snapshot();
  std::printf("sizeof(Mind) %zu B\n", sizeof(Mind));
  return report("mind");
}
