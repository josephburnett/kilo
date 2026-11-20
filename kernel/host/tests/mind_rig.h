// The whole Mind against the scripted body (body_rig.h), 4 ms a pass: what
// the Mind tests (mind, record, power, guard) share. A test's rig derives
// from it and adds its own fakes and probes; pass() is virtual, so a rig
// that shapes the body's readings each pass overrides it.
#ifndef KILO_HOST_TESTS_MIND_RIG_H
#define KILO_HOST_TESTS_MIND_RIG_H
#include <cstdint>
#include <vector>
#include "mind.h"
#include "tests/body_rig.h"
#include "tests/check.h"
#include "tests/fake_net.h"
#include "util/logring.h"

namespace kilo {

struct MindRig {
  FakeBody body;
  CaptureLog log;
  FakeListener listener;  // the host channel's, when the rig is made with_listener
  Mind mind;
  std::vector<uint8_t> mem;       // the flight recorder's RAM (attach_recorder)
  std::vector<LogEntry> history;  // the RAM log's history (attach_history)

  explicit MindRig(const MindConfig& cfg, bool with_listener = false)
      : mind(body.serial, body.clock, log, with_listener ? &listener : nullptr, cfg) {}
  virtual ~MindRig() = default;

  // The remote on (kilo-mind-host --remote): the tests' usual start.
  static MindConfig remote_cfg() {
    MindConfig c;
    c.remote = true;
    return c;
  }
  void attach_history() {
    history.resize(LogRing::kLines);
    mind.attach_log(history.data(), (uint32_t)history.size());
  }
  void attach_recorder(uint32_t bytes = 4u << 20) {
    mem.resize(bytes);
    mind.attach_recorder(mem.data(), (uint32_t)mem.size());
  }
  void start(uint32_t settle_ms = 100) {
    mind.start();
    run_ms(settle_ms);
  }
  virtual void pass() {
    body.step();
    mind.poll();
    body.react();
  }
  void run_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i += 4) pass();
  }
  // A remote key (EVENT IR_KEY), then ms of passes.
  void key(uint16_t cmd, uint32_t ms = 40) {
    body.event(KILO_EVENT_IR_KEY, (int16_t)cmd);
    run_ms(ms);
  }
  int posture_goal() {
    const Goal* g = mind.link().goal(KILO_MSG_SET_POSTURE);
    return g == nullptr ? -1 : g->payload[1];
  }
};

}  // namespace kilo

#endif
