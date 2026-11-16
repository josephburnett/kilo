// The Mind's composition on a scripted body: the remote acting on IR_KEY
// (rows 1-3 ignored), A5 starting the behaviour runner and B6 / A7 / C7
// stopping it, the other keys ignored while it runs; the behaviour layer
// latching its posture through the link; percepts fed by every TELEMETRY;
// liveness stamped per pass, and held back when the runner stops ticking.
#include <cstdio>
#include <cstring>
#include "mind.h"
#include "tests/body_rig.h"
#include "tests/check.h"
#include "tricks/library.h"

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
  r.body.event(KILO_EVENT_IR_KEY, 0x02);  // B6: stand
  r.run_ms(40);
  CHECK(r.posture_goal() == KILO_POSTURE_STAND && r.log.has("remote B6 (0x02, stand): posture stand"));
  r.body.event(KILO_EVENT_IR_KEY, 0x1A2);  // a held-key repeat: nothing
  r.body.event(KILO_EVENT_IR_KEY, 0x45);   // another remote's key: nothing
  r.body.event(KILO_EVENT_IR_KEY, 0x18);   // B3, row 3: nothing
  r.body.event(KILO_EVENT_IR_KEY, 0x10);   // A2, row 2: nothing
  r.run_ms(40);
  CHECK(r.posture_goal() == KILO_POSTURE_STAND && r.body.count(KILO_MSG_TRICK_PLAY) == 0);
  CHECK(r.log.has("remote A7 (0xA2, sleep) repeat: ignored") && r.log.has("remote ? (0x45): ignored"));
  CHECK(r.log.has("remote B3 (0x18, none): ignored (no action on this key)") &&
        r.log.has("remote A2 (0x10, none): ignored (no action on this key)"));
  r.body.event(KILO_EVENT_IR_KEY, 0xA2);  // A7: rest
  r.run_ms(40);
  CHECK(r.posture_goal() == KILO_POSTURE_REST);
  // without --remote the keys do nothing
  Rig off{MindConfig()};
  off.body.event(KILO_EVENT_IR_KEY, 0x02);
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
  r.body.event(KILO_EVENT_IR_KEY, 0x68);  // A4: trot, ignored while the runner runs
  r.run_ms(40);
  CHECK(r.log.has("remote A4 (0x68, trot): ignored (behaviour runner active)") && !r.mind.walker().active());
  r.body.event(KILO_EVENT_IR_KEY, 0xE0);  // A5 again: nothing
  r.run_ms(40);
  CHECK(r.log.has("remote A5 (0xE0, behaviour): behaviour runner already running"));
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

// A5 starts the runner at run time (off by the config), B6 / A7 / C7 stop it, A5 starts it again.
void test_behaviour_from_the_remote() {
  MindConfig cfg;
  cfg.remote = true;
  cfg.seed = 3;
  Rig r(cfg);
  r.run_ms(400);
  CHECK(!r.mind.behaviour_active() && r.mind.runner().ticks() == 0);
  r.body.event(KILO_EVENT_IR_KEY, 0xE0);  // A5
  r.run_ms(500);
  CHECK(r.mind.behaviour_active() && r.log.has("remote A5 (0xE0, behaviour): behaviour runner started"));
  CHECK(r.log.has("behaviour: idle (free)") && r.posture_goal() == KILO_POSTURE_SIT);
  uint32_t ticks = r.mind.runner().ticks();
  CHECKF(ticks >= 4 && ticks <= 6, "%u ticks in 0.5 s", ticks);
  CHECK(r.mind.stamping());
  r.body.event(KILO_EVENT_IR_KEY, 0x02);  // B6: stops it, then stands
  r.run_ms(100);
  CHECK(!r.mind.behaviour_active() && r.posture_goal() == KILO_POSTURE_STAND);
  CHECK(r.log.has("remote B6 (0x02, stand): behaviour runner stopped, posture stand"));
  ticks = r.mind.runner().ticks();
  r.run_ms(2000);  // stopped: no ticks, and the liveness is stamped without it
  CHECK(r.mind.runner().ticks() == ticks && r.mind.stamping() && !r.log.has("no longer stamping"));
  r.body.event(KILO_EVENT_IR_KEY, 0xE0);  // A5: a fresh run
  r.run_ms(300);
  CHECK(r.mind.behaviour_active() && r.mind.runner().ticks() > ticks && r.mind.stamping());
  CHECK(r.posture_goal() == KILO_POSTURE_SIT);  // its goals re-asserted
  // a trick playing is stopped when C7 stops the runner
  r.mind.runner().stop();
  r.mind.runner().restart();
  r.run_ms(100);
  uint32_t play = r.mind.player().start(*tricks::find_trick("hi"));
  CHECK(play != 0);
  r.run_ms(200);
  r.body.event(KILO_EVENT_IR_KEY, 0xE2);  // C7
  r.run_ms(100);
  CHECK(!r.mind.behaviour_active() && r.mind.remote().frozen());
  CHECK(r.body.count(KILO_MSG_TRICK_STOP) >= 1);  // (the scripted body sends no TRICK_DONE)
  CHECK(r.log.has("remote C7 (0xE2, freeze): frozen (behaviour runner stopped"));
  r.body.event(KILO_EVENT_IR_KEY, 0xE0);  // A5 while frozen: ignored
  r.run_ms(100);
  CHECK(!r.mind.behaviour_active() && r.log.has("remote A5 (0xE0, behaviour): ignored (frozen: B6 or A7 to go on)"));
  r.body.event(KILO_EVENT_IR_KEY, 0xA2);  // A7
  r.run_ms(100);
  CHECK(!r.mind.remote().frozen() && r.posture_goal() == KILO_POSTURE_REST);
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
  test_behaviour_from_the_remote();
  test_percepts_and_snapshot();
  std::printf("sizeof(Mind) %zu B\n", sizeof(Mind));
  return report("mind");
}
