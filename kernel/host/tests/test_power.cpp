// Light sleep (power/sleep.h) on a scripted body: A7 limps the servos as
// soon as he lies still (with or without a laptop), then 30 s of nothing
// puts the Pi to sleep (the loop's sleep period); keys and laptop activity
// restart or cancel the count but never power the servos; any key or a
// laptop wakes him; a walk, a trick or the runner from limp gets REST
// first; lying idle in REST for 2 min sleeps as after A7; turned over and
// righted before the limp calls it off; the network policy with and
// without an Ethernet link; the card saver's save before sleeping; the
// battery trend (power/battery.h) and the metrics lines.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "mind.h"
#include "power/battery.h"
#include "power/sleep.h"
#include "tests/check.h"
#include "tests/fake_card.h"
#include "tests/mind_rig.h"

using namespace kilo;
using power::State;

namespace {

// The hardware seams: an Ethernet device whose link the test sets, and
// every net_save() call recorded.
struct FakeHw final : power::Hardware {
  bool device = true;
  bool link = false;
  bool fail = false;
  std::vector<bool> saves;  // each net_save(on)
  bool phy_save = false;    // the PHY's mode now
  bool net_device() override { return device; }
  bool net_link() override { return link; }
  bool net_save(bool on) override {
    saves.push_back(on);
    if (fail) return false;
    phy_save = on;
    return true;
  }
  bool soc(power::Soc* s) override {
    s->arm_hz = 600000000;
    s->temp_mc = 41500;
    s->throttled = 0;
    return true;
  }
};

constexpr uint8_t kA7 = 0xA2, kB6 = 0x02, kA4 = 0x68, kC1 = 0x52, kC7 = 0xE2, kB2 = 0x38;

MindConfig sleep_cfg() {
  MindConfig c = MindRig::remote_cfg();
  c.light_sleep = true;
  return c;
}

struct Rig : MindRig {
  FakeHw hw;
  explicit Rig(const MindConfig& cfg, bool with_listener = false, bool recorder = false) : MindRig(cfg, with_listener) {
    mind.sleep().set_hardware(&hw);
    if (recorder) {
      attach_history();
      attach_recorder();
    }
  }
  // the postures sent from frame index `from` on, in order (each latch once: refreshes are the same)
  std::vector<int> postures_since(size_t from) {
    std::vector<int> out;
    for (size_t i = from; i < body.sent.size(); i++)
      if (body.sent[i].type == KILO_MSG_SET_POSTURE && body.sent[i].payload.size() >= 2 &&
          (out.empty() || out.back() != body.sent[i].payload[1]))
        out.push_back(body.sent[i].payload[1]);
    return out;
  }
  State state() { return mind.sleep().state(); }
};

// A7, then 30 s of nothing: asleep, LIMP, the loop's sleep period; the 30 s are the constant
void test_enter_after_30s() {
  Rig r(sleep_cfg());
  r.start();
  CHECK(r.state() == State::Awake && r.mind.sleep().loop_period_us() == 0);
  r.key(kA7);
  CHECK(r.posture_goal() == KILO_POSTURE_REST && r.state() == State::Drowsy);
  CHECK(r.log.has("power: A7: light sleep in 30 s unless something happens"));
  CHECK(r.log.has("the servos off once he lies still (3 s)"));
  r.run_ms((uint32_t)(power::kLimpSettleUs / 1000) - 200);  // still settling: REST
  CHECK(r.state() == State::Drowsy && r.posture_goal() == KILO_POSTURE_REST && !r.mind.sleep().limp());
  r.run_ms(300);  // the settle is over: LIMP, the Pi still awake
  CHECK(r.state() == State::Drowsy && r.posture_goal() == KILO_POSTURE_LIMP && r.mind.sleep().limp());
  CHECK(r.log.has("power: A7, lying still: servos off (LIMP): inert until a key moves him"));
  CHECK(r.mind.sleep().limps() == 1);
  r.run_ms((uint32_t)(power::kSleepAfterUs / 1000) - 4000);  // 29 s after the key
  CHECK(r.state() == State::Drowsy && r.posture_goal() == KILO_POSTURE_LIMP);
  r.run_ms(1100);
  CHECKF(r.state() == State::Asleep, "state %s", power::state_name(r.state()));
  CHECK(r.posture_goal() == KILO_POSTURE_LIMP && r.mind.sleep().asleep());
  CHECK(r.mind.sleep().loop_period_us() == power::kSleepLoopUs && power::kSleepLoopUs == 10000);
  CHECK(r.log.has("power: light sleep (30 s after A7 with nothing else): servos off (LIMP already)"));
  CHECK(r.mind.sleep().sleeps() == 1 && r.mind.sleep().wakes() == 0 && r.mind.sleep().limps() == 1);
  // the body's watchdog stays fed: heartbeats or goal refreshes keep going while asleep
  size_t before = r.body.sent.size();
  r.run_ms(2000);
  CHECK(r.body.sent.size() > before + 5 && r.mind.stamping());
  // without light sleep (MindConfig default) A7 is just rest
  Rig off{[] {
    MindConfig c;
    c.remote = true;
    return c;
  }()};
  off.start();
  off.key(kA7);
  off.run_ms(40000);
  CHECK(off.posture_goal() == KILO_POSTURE_REST && off.state() == State::Awake && !off.log.has("power: light"));
  // ... and lying in REST for long never limps him either (KILO_LIGHT_SLEEP=0: the awake baseline)
  off.run_ms(200000);
  CHECK(off.posture_goal() == KILO_POSTURE_REST && off.state() == State::Awake && off.mind.sleep().limps() == 0);
}

// any key or activity in the 30 s restarts the count; a key that moves Kilo cancels it
void test_cancel_and_restart() {
  {  // B6 after 15 s: cancelled, he stands and stays awake
    Rig r(sleep_cfg());
    r.start();
    r.key(kA7);
    r.run_ms(15000);
    r.key(kB6);
    CHECK(r.state() == State::Awake && r.posture_goal() == KILO_POSTURE_STAND);
    CHECK(r.log.has("power: light sleep called off (key:B6 moves Kilo)"));
    r.run_ms(40000);
    CHECK(r.state() == State::Awake && r.posture_goal() == KILO_POSTURE_STAND && r.mind.sleep().sleeps() == 0);
  }
  {  // C1 (a key that does nothing) after 20 s: the count starts again
    Rig r(sleep_cfg());
    r.start();
    r.key(kA7);
    r.run_ms(20000);
    r.key(kC1);
    r.run_ms(15000);  // 35 s after A7, 15 s after C1
    CHECK(r.state() == State::Drowsy);
    r.run_ms(16000);
    CHECK(r.state() == State::Asleep);
  }
  {  // a held key's repeat and C7 count as activity too, but do not cancel
    Rig r(sleep_cfg());
    r.start();
    r.key(kA7);
    r.run_ms(25000);
    r.key((uint8_t)0);  // ? (0x00): another remote's key
    r.body.event(KILO_EVENT_IR_KEY, 0x1A2);  // A7 held: its repeat
    r.run_ms(40);
    r.run_ms(25000);
    CHECK(r.state() == State::Drowsy);
    r.key(kC7);  // freeze while resting: holds the posture
    r.run_ms(29000);
    CHECK(r.state() == State::Drowsy);
    r.run_ms(2000);
    CHECK(r.state() == State::Asleep);
  }
  {  // a walk counts as busy: A7 stops it the planned way, the 30 s run from when nothing moves
    Rig r(sleep_cfg());
    r.start();
    r.key(kA4);  // trot
    r.run_ms(2000);
    CHECK(r.mind.walker().active());
    r.key(kA7);
    r.run_ms(29000);
    CHECK(r.state() == State::Drowsy || !r.mind.walker().active());
    r.run_ms(10000);
    CHECK(r.state() == State::Asleep && !r.mind.walker().active() && r.posture_goal() == KILO_POSTURE_LIMP);
  }
  {  // a laptop holding the body: keys are not acted on, so A7 does not arm
    Rig r(sleep_cfg());
    r.start();
    r.mind.sleep().key(kA7, false);
    CHECK(r.state() == State::Awake);
  }
}

// asleep: any key wakes him; B6 stands; a walk or a trick key gets REST first
void test_wake_by_key() {
  {
    Rig r(sleep_cfg());
    r.start();
    r.key(kA7);
    r.run_ms(31000);
    CHECK(r.state() == State::Asleep);
    r.key(kB6);
    CHECK(r.state() == State::Awake && r.mind.sleep().loop_period_us() == 0);
    CHECK(r.posture_goal() == KILO_POSTURE_STAND && r.log.has("power: awake (key:B6) after "));
    CHECK(r.mind.sleep().wakes() == 1 && std::strcmp(r.mind.sleep().last_wake(), "key:B6") == 0);
    r.run_ms(40000);
    CHECK(r.state() == State::Awake);  // B6 cancelled A7: he stays awake
  }
  {  // a key that does nothing wakes the Pi, A7 still holds: asleep again 30 s later, still LIMP meanwhile
    Rig r(sleep_cfg());
    r.start();
    r.key(kA7);
    r.run_ms(31000);
    size_t from = r.body.sent.size();
    r.key(kC1);
    CHECK(r.state() == State::Drowsy && r.posture_goal() == KILO_POSTURE_LIMP);
    CHECK(r.log.has("power: awake (key:C1) after ") && r.log.has("A7 still holds"));
    r.run_ms(31000);
    CHECK(r.state() == State::Asleep && r.mind.sleep().sleeps() == 2);
    for (int p : r.postures_since(from)) CHECKF(p == KILO_POSTURE_LIMP, "posture %d", p);
  }
  {  // A4 from sleep: REST goes out before the walk's FEET
    Rig r(sleep_cfg());
    r.start();
    r.key(kA7);
    r.run_ms(31000);
    size_t from = r.body.sent.size();
    r.key(kA4);
    r.run_ms(3000);
    CHECK(r.state() == State::Awake && r.mind.walker().active());
    size_t rest_at = 0, feet_at = 0;
    for (size_t i = from; i < r.body.sent.size(); i++) {
      const Sent& s = r.body.sent[i];
      if (rest_at == 0 && s.type == KILO_MSG_SET_POSTURE && s.payload.size() >= 2 && s.payload[1] == KILO_POSTURE_REST)
        rest_at = i;
      if (feet_at == 0 && s.type == KILO_MSG_FEET) feet_at = i;
    }
    CHECKF(rest_at != 0 && feet_at != 0 && rest_at < feet_at, "rest at %zu feet at %zu", rest_at, feet_at);
  }
  {  // A7 asleep: wakes, rests (the remote's own posture), asleep again after 30 s
    Rig r(sleep_cfg());
    r.start();
    r.key(kA7);
    r.run_ms(31000);
    r.key(kA7);
    CHECK(r.state() == State::Drowsy && r.posture_goal() == KILO_POSTURE_REST);
    r.run_ms(31000);
    CHECK(r.state() == State::Asleep && r.posture_goal() == KILO_POSTURE_LIMP);
  }
  {  // B2 (the meow: it moves nothing, like a row-1 key): wakes the Pi only
    Rig r(sleep_cfg());
    r.start();
    r.key(kA7);
    r.run_ms(31000);
    r.key(kB2);
    CHECK(r.state() == State::Drowsy && r.posture_goal() == KILO_POSTURE_LIMP);
  }
}


// A7 with a laptop connected (the cable in, a client open): the servos go off all the same; the Pi
// stays awake while the client is there; a do-nothing key and the laptop's requests keep him limp
void test_limp_with_laptop() {
  Rig r(sleep_cfg());
  r.start();
  power::Sleep& s = r.mind.sleep();
  r.key(kA7);
  power::Activity a;
  a.host_clients = 1;
  a.lying_rest = true;
  for (int i = 0; i < 40; i++) {  // 4 s of passes with a client connected and asking
    a.host_events++;
    r.body.clock.advance_ms(100);
    s.poll(a);
  }
  CHECK(s.state() == State::Drowsy && s.limp() && r.posture_goal() == KILO_POSTURE_LIMP);
  r.key(kC1);  // does nothing: limp still
  CHECK(s.limp() && r.posture_goal() == KILO_POSTURE_LIMP);
  for (int i = 0; i < 100; i++) {  // 100 s more, the client still there: no Asleep
    r.body.clock.advance_ms(1000);
    s.poll(a);
  }
  CHECK(s.state() == State::Drowsy && s.limp() && s.sleeps() == 0 && s.wakes() == 0);
  CHECK(r.posture_goal() == KILO_POSTURE_LIMP && !r.log.has("power: awake"));
  // the whole Mind with a real connection: A7 limps after the settle, and nothing re-powers him
  Rig m(sleep_cfg(), true);
  m.start();
  m.listener.pending.push_back(0);
  m.run_ms(40);
  m.key(kA7);
  m.run_ms(4000);
  CHECKF(m.posture_goal() == KILO_POSTURE_LIMP && m.mind.sleep().limp(), "posture %d", m.posture_goal());
}

// A7 during a walk: no LIMP while the planned stop and REST are still under way; then after the settle
void test_no_limp_while_stopping() {
  Rig r(sleep_cfg());
  r.start();
  r.key(kA4);  // trot
  r.run_ms(2000);
  CHECK(r.mind.walker().active());
  r.key(kA7);
  bool limp_while_busy = false, saw_busy = false;
  uint64_t free_at = 0, limp_at = 0;
  for (int i = 0; i < 5000 && limp_at == 0; i++) {
    r.pass();
    bool busy = r.mind.walker().active() || r.mind.remote().busy() || r.body.mode == KILO_MODE_GAIT;
    if (busy) {
      saw_busy = true;
      free_at = 0;
    } else if (free_at == 0) {
      free_at = r.body.clock.t;
    }
    if (r.mind.sleep().limp()) {
      limp_at = r.body.clock.t;
      limp_while_busy = busy;
    }
  }
  CHECK(saw_busy && !limp_while_busy && limp_at != 0);
  CHECKF(limp_at - free_at + 10000 >= power::kLimpSettleUs && limp_at - free_at < power::kLimpSettleUs + 100000,
         "limp %llu us after the stop", (unsigned long long)(limp_at - free_at));
  r.run_ms(100);  // the LIMP goal goes out
  std::vector<int> p = r.postures_since(0);
  std::string seq;
  for (int x : p) seq += std::to_string(x) + " ";
  CHECKF(p.size() >= 2 && p[p.size() - 2] == KILO_POSTURE_REST && p.back() == KILO_POSTURE_LIMP, "postures %s",
         seq.c_str());
}

// limp, not yet asleep: a second A7 rests him again and the limp follows after the settle again; a
// posture key goes straight there; a walk or a trick key gets REST first
void test_keys_from_limp() {
  {
    Rig r(sleep_cfg());
    r.start();
    r.key(kA7);
    r.run_ms(4000);
    CHECK(r.mind.sleep().limp());
    r.key(kA7);
    CHECK(r.posture_goal() == KILO_POSTURE_REST && !r.mind.sleep().limp() && r.state() == State::Drowsy);
    CHECK(r.log.has("power: A7 again: REST, the servos off again once he lies still"));
    r.run_ms(2000);
    CHECK(r.posture_goal() == KILO_POSTURE_REST);
    r.run_ms(1500);
    CHECK(r.posture_goal() == KILO_POSTURE_LIMP && r.mind.sleep().limps() == 2);
  }
  {  // B6 from limp: STAND directly
    Rig r(sleep_cfg());
    r.start();
    r.key(kA7);
    r.run_ms(4000);
    size_t from = r.body.sent.size();
    r.key(kB6);
    CHECK(r.state() == State::Awake && r.posture_goal() == KILO_POSTURE_STAND && !r.mind.sleep().limp());
    std::vector<int> p = r.postures_since(from);
    CHECK(p.size() == 1 && p[0] == KILO_POSTURE_STAND);
  }
  {  // A4 from limp (the Pi awake): REST goes out before the walk's FEET
    Rig r(sleep_cfg());
    r.start();
    r.key(kA7);
    r.run_ms(4000);
    CHECK(r.state() == State::Drowsy && r.mind.sleep().limp());
    size_t from = r.body.sent.size();
    r.key(kA4);
    r.run_ms(3000);
    CHECK(r.state() == State::Awake && r.mind.walker().active());
    size_t rest_at = 0, feet_at = 0;
    for (size_t i = from; i < r.body.sent.size(); i++) {
      const Sent& s = r.body.sent[i];
      if (rest_at == 0 && s.type == KILO_MSG_SET_POSTURE && s.payload.size() >= 2 && s.payload[1] == KILO_POSTURE_REST)
        rest_at = i;
      if (feet_at == 0 && s.type == KILO_MSG_FEET) feet_at = i;
    }
    CHECKF(rest_at != 0 && feet_at != 0 && rest_at < feet_at, "rest at %zu feet at %zu", rest_at, feet_at);
  }
  {  // a laptop connecting while he is asleep and limp: the Pi wakes, the servos stay off
    Rig r(sleep_cfg(), true);
    r.start();
    r.key(kA7);
    r.run_ms(31000);
    CHECK(r.state() == State::Asleep);
    size_t from = r.body.sent.size();
    r.listener.pending.push_back(0);
    r.run_ms(2000);
    CHECK(r.state() == State::Drowsy && r.log.has("power: awake (laptop) after "));
    for (int p : r.postures_since(from)) CHECKF(p == KILO_POSTURE_LIMP, "posture %d", p);
    CHECK(r.mind.sleep().limp());
  }
}

// turned over after A7 before the limp (the recording of 2026-11-21): the body's fall reflex rights
// him and leaves him STANDING; the sleep is called off, never a LIMP from standing
void test_righted_before_limp() {
  for (int late = 0; late < 2; late++) {
    Rig r(sleep_cfg());
    r.start();
    r.key(kA7);
    r.run_ms(late ? 2900 : 1000);
    r.body.mode = KILO_MODE_FALLEN;
    r.body.event(KILO_EVENT_FELL, -1);
    r.run_ms(1000);
    r.body.mode = KILO_MODE_RECOVERING;
    r.run_ms(3000);
    r.body.mode = KILO_MODE_IDLE;  // RECOVERED: IDLE STAND
    r.body.support = 0xF;
    r.body.event(KILO_EVENT_RECOVERED, 0);
    r.run_ms(60000);
    CHECKF(r.state() == State::Awake && !r.mind.sleep().limp() && r.mind.sleep().limps() == 0, "state %s limps %u",
           power::state_name(r.state()), (unsigned)r.mind.sleep().limps());
    CHECK(r.posture_goal() != KILO_POSTURE_LIMP && r.log.has("power: light sleep called off (no longer lying in REST)"));
    for (int p : r.postures_since(0)) CHECKF(p != KILO_POSTURE_LIMP, "posture %d", p);
  }
}

// awake, lying in REST with nothing happening for 2 min: asleep as after A7 (LIMP at once, Asleep 30 s later)
void test_idle_sleep() {
  CHECK(power::kIdleSleepUs == 120000000);
  {  // the mind restarted while the body kept REST: no goal on record, TELEMETRY support 0
    Rig r(sleep_cfg(), true);
    r.body.dist_mm = 59;  // the floor under his nose
    r.start();
    CHECK(r.posture_goal() == -1);
    for (int i = 0; i < 119; i++) {  // a laptop polling now and then (the HUD): not activity for this
      if (i % 10 == 0) r.listener.pending.push_back(0);
      r.run_ms(1000);
    }
    CHECK(r.state() == State::Awake && !r.mind.sleep().limp());
    r.run_ms(1200);
    CHECKF(r.state() == State::Drowsy && r.mind.sleep().limp() && r.posture_goal() == KILO_POSTURE_LIMP, "state %s",
           power::state_name(r.state()));
    CHECK(r.log.has("power: lying idle in REST for 2 min: asleep as after A7"));
    CHECK(r.log.has("power: lying idle: servos off (LIMP): inert until a key moves him"));
    CHECK(r.mind.sleep().idle_sleeps() == 1);
    r.run_ms(31000);
    CHECK(r.state() == State::Asleep);
    r.key(kB6);  // B6 stands him up: awake, and standing he never sleeps on his own
    r.body.support = 0xF;
    r.run_ms(300000);
    CHECK(r.state() == State::Awake && r.posture_goal() == KILO_POSTURE_STAND && r.mind.sleep().idle_sleeps() == 1);
  }
  {  // REST asked by the remote (A7) with light sleep on is the A7 path; a REST from a laptop then
     // released: the idle path. Standing (the real body's support 0xF): never.
    Rig r(sleep_cfg());
    r.start();
    r.body.support = 0xF;
    r.key(kB6);
    r.run_ms(300000);
    CHECK(r.state() == State::Awake && r.mind.sleep().limps() == 0);
  }
  {  // no goal on record and no floor in view (a warm reboot's held pose, perhaps standing): never
    Rig r(sleep_cfg());
    r.body.dist_mm = 400;
    r.start();
    r.run_ms(300000);
    CHECK(r.state() == State::Awake && r.mind.sleep().limps() == 0);
  }
  {  // the floor with a missed echo now and then (0 mm, every 0.5 s) still counts
    Rig r(sleep_cfg());
    r.start();
    for (int i = 0; i < 130 * 2; i++) {
      r.body.dist_mm = 0;
      r.run_ms(40);
      r.body.dist_mm = 60;
      r.run_ms(460);
    }
    CHECK(r.state() == State::Drowsy && r.mind.sleep().idle_sleeps() == 1);
  }
  {  // a key within the 2 min starts them again, any key (a do-nothing one, a repeat)
    Rig r(sleep_cfg());
    r.body.dist_mm = 59;
    r.start();
    r.run_ms(100000);
    r.key(kC1);
    r.run_ms(100000);
    r.body.event(KILO_EVENT_IR_KEY, 0x152);  // C1 held: its repeat
    r.run_ms(100000);
    CHECK(r.state() == State::Awake && !r.mind.sleep().limp());
    r.run_ms(21000);
    CHECK(r.state() == State::Drowsy && r.mind.sleep().limp());
  }
  {  // the Sleep alone: busy, a moving mode, a laptop holding the body, not lying: no count
    Rig r(sleep_cfg());
    r.start();
    power::Sleep& s = r.mind.sleep();
    power::Activity a;
    a.lying_rest = true;
    auto run = [&](int secs) {
      for (int i = 0; i < secs; i++) {
        r.body.clock.advance_ms(1000);
        s.poll(a);
      }
    };
    a.busy = true;  // the behaviour runner (resting when tired), a trick, the walker, the remote
    run(300);
    CHECK(s.state() == State::Awake);
    a.busy = false;
    a.host_holds = true;
    run(300);
    CHECK(s.state() == State::Awake);
    a.host_holds = false;
    a.lying_rest = false;
    run(300);
    CHECK(s.state() == State::Awake);
    a.lying_rest = true;
    a.host_clients = 1;  // merely connected and polling: counts on
    run(119);
    CHECK(s.state() == State::Awake);
    a.host_events += 5;
    run(2);
    CHECK(s.state() == State::Drowsy && s.limp() && s.idle_sleeps() == 1);
  }
  {  // the behaviour runner on, the body reporting REST: never asleep on his own
    MindConfig c = sleep_cfg();
    Rig r(c);
    r.start();
    r.key(0xE0);  // A5: the runner
    r.run_ms(1000);
    CHECK(r.mind.behaviour_active());
    r.run_ms(200000);
    CHECK(r.mind.behaviour_active() && r.mind.sleep().idle_sleeps() == 0 && r.mind.sleep().limps() == 0);
  }
}

// a laptop connecting or asking anything wakes him; no sleep while a client is connected
void test_wake_by_host() {
  {
    Rig r(sleep_cfg(), true);
    r.start();
    r.key(kA7);
    r.run_ms(31000);
    CHECK(r.state() == State::Asleep);
    r.listener.pending.push_back(0);  // a connection (this Mind has no keys: refused after its hello)
    r.run_ms(40);
    CHECKF(r.state() == State::Drowsy && r.log.has("power: awake (laptop) after "), "state %s accepted %u",
           power::state_name(r.state()), (unsigned)r.mind.host().stats().accepted);
    CHECK(std::strcmp(r.mind.sleep().last_wake(), "laptop") == 0);
    r.run_ms(31000);
    CHECK(r.state() == State::Asleep && r.mind.sleep().sleeps() == 2);
  }
  {  // the Sleep alone: host events and clients
    Rig r(sleep_cfg());
    r.start();
    power::Sleep& s = r.mind.sleep();
    r.key(kA7);
    power::Activity a;
    a.host_events = 7;
    a.lying_rest = true;  // the body in REST, as Mind::lying_rest() reads it
    s.poll(a);
    a.host_clients = 1;  // a client connected: the count holds at 0
    for (int i = 0; i < 100; i++) {
      r.body.clock.advance_ms(1000);
      s.poll(a);
    }
    CHECK(s.state() == State::Drowsy);
    a.host_clients = 0;
    r.body.clock.advance_ms(29000);
    s.poll(a);
    CHECK(s.state() == State::Drowsy);
    a.host_events = 8;  // a request
    s.poll(a);
    r.body.clock.advance_ms(29000);
    s.poll(a);
    CHECK(s.state() == State::Drowsy);
    r.body.clock.advance_ms(1100);
    s.poll(a);
    CHECK(s.state() == State::Asleep);
    a.host_events = 9;
    s.poll(a);
    CHECK(s.state() == State::Drowsy && std::strcmp(s.last_wake(), "laptop") == 0);
  }
}

// the network: kept up with a link; without one, the PHY's power-down and a look every 10 s; normal on waking
void test_network_policy() {
  {  // a link: nothing changes
    Rig r(sleep_cfg());
    r.hw.link = true;
    r.start();
    r.key(kA7);
    r.run_ms(31000);
    CHECK(r.state() == State::Asleep && r.mind.sleep().net() == power::Net::On && r.hw.saves.empty());
    CHECK(r.log.has("Ethernet kept on (a cable is in)"));
  }
  {  // no link
    Rig r(sleep_cfg());
    r.start();
    r.run_ms(1000);
    CHECK(r.hw.saves.empty());  // awake: never touched
    r.key(kA7);
    r.run_ms(31000);
    CHECK(r.state() == State::Asleep && r.mind.sleep().net() == power::Net::Save && r.hw.phy_save);
    CHECK(r.hw.saves.size() == 1 && r.log.has("power: no Ethernet link: PHY power-down"));
    r.run_ms(10100);  // the look: PHY normal for 3 s
    CHECK(r.mind.sleep().net() == power::Net::Look && !r.hw.phy_save);
    r.run_ms(3100);
    CHECK(r.mind.sleep().net() == power::Net::Save && r.hw.phy_save && r.hw.saves.size() == 3);
    // 60 s: six looks a minute, the PHY in power-down 7 s of every 10
    r.run_ms(60000);
    CHECKF(r.hw.saves.size() == 15, "%zu net_save calls", r.hw.saves.size());
    // a cable plugged in: the PHY wakes on the signal, the link comes up, the policy keeps it on
    r.hw.link = true;
    r.run_ms(100);
    CHECK(r.mind.sleep().net() == power::Net::On && !r.hw.phy_save && r.log.has("Ethernet link while asleep: PHY on"));
    CHECK(r.state() == State::Asleep);  // a link alone does not wake him; a connection does
    // the link goes again: back to the power-down
    r.hw.link = false;
    r.run_ms(100);
    CHECK(r.mind.sleep().net() == power::Net::Save && r.hw.phy_save);
    // waking: the PHY back to normal
    r.key(kB6);
    CHECK(r.state() == State::Awake && r.mind.sleep().net() == power::Net::On && !r.hw.phy_save);
  }
  {  // no network device (kilo-mind-host, or no USB): nothing is touched
    Rig r(sleep_cfg());
    r.hw.device = false;
    r.start();
    r.key(kA7);
    r.run_ms(45000);
    CHECK(r.state() == State::Asleep && r.hw.saves.empty() && r.log.has("Ethernet none"));
  }
  {  // a PHY that refuses: logged and counted, sleep goes on
    Rig r(sleep_cfg());
    r.hw.fail = true;
    r.start();
    r.key(kA7);
    r.run_ms(31000);
    CHECK(r.state() == State::Asleep && r.log.has("power: the Ethernet PHY did not take its power-down"));
  }
}

// the card saver: asked to save now, its save finishes before sleep
void test_recorder_flushed_first() {
  FakeCard card;
  Rig r(sleep_cfg(), false, true);
  card.set_clock(r.body.clock);
  card.write_us = 30000;  // slow writes: the save spans many passes
  record::Saver saver(r.body.clock, r.mind.log(), r.mind.recorder(), r.mind.log(), card, true);
  r.mind.attach_saver(saver);
  r.start();
  r.run_ms(5000);  // the saver scans the card
  CHECK(saver.scanned());
  r.key(kA7);
  r.run_ms(29000);
  uint32_t writes_before = card.writes;
  CHECK(r.state() == State::Drowsy);
  // run until asleep, watching that the saver is never mid-save when sleep begins
  bool saw_saving = false, saving_at_sleep = false;
  for (int i = 0; i < 5000 && r.state() != State::Asleep; i++) {
    r.pass();
    if (saver.state() == KILO_RECORD_CARD_STATE_SAVING) saw_saving = true;
    if (r.state() == State::Asleep) saving_at_sleep = saver.state() == KILO_RECORD_CARD_STATE_SAVING;
  }
  CHECK(r.state() == State::Asleep && !saving_at_sleep);
  CHECKF(saw_saving && card.writes > writes_before, "saving %d writes %u -> %u", (int)saw_saving,
         (unsigned)writes_before, (unsigned)card.writes);
  // the sleep is in the flight recorder: its log line is in the RAM blocks (a LOG record holds the text)
  const record::FlightStore& f = r.mind.recorder().flight();
  std::string all;
  for (uint32_t s = f.first_seq(); !f.empty() && s <= f.last_seq(); s++)
    all.append((const char*)f.block(s), f.used(s));
  CHECK(all.find("power: A7: light sleep in 30 s") != std::string::npos &&
        all.find("power: light sleep (") != std::string::npos);
}

void test_battery_trend() {
  power::BatteryTrend b;
  // 8000 mV falling 10 mV/min, 25 readings a second, 10 min awake
  uint64_t t = 0;
  for (int i = 0; i < 25 * 600; i++, t += 40000) b.add(t, (uint16_t)(8000 - (int)(t / 6000000)), 0);
  CHECKF(b.samples() == 59 || b.samples() == 60, "%u samples", (unsigned)b.samples());
  int32_t s5 = b.trend(t, 300);
  CHECKF(s5 >= -105 && s5 <= -95, "5 min trend %d", (int)s5);
  CHECK(b.trend(t, 20) == power::BatteryTrend::kNoTrend);  // too few samples
  CHECK(b.last_mv() <= 7901 && b.min_mv() == b.last_mv());
  // asleep 10 min at 2 mV/min after a 60 mV jump up (the load is off): the segment's slope leaves the jump out
  b.state_changed(t, 2);
  uint16_t base = (uint16_t)(b.last_mv() + 60);
  uint64_t t0 = t;
  for (int i = 0; i < 25 * 600; i++, t += 40000) b.add(t, (uint16_t)(base - (int)((t - t0) / 30000000)), 2);
  int32_t seg = b.segment_trend();
  CHECKF(seg >= -25 && seg <= -15, "asleep segment %d", (int)seg);
  b.state_changed(t, 0);
  CHECK(b.segments() == 2 && b.segment(0).state == 2 && b.segment(1).state == 0);
  CHECKF(b.segment(0).slope_x10 >= -25 && b.segment(0).slope_x10 <= -15, "%d", (int)b.segment(0).slope_x10);
  CHECK(b.segment(0).end_s - b.segment(0).start_s == 600 && b.segment(0).first_mv > b.segment(0).last_mv);
  // the ring keeps 30 min
  for (int i = 0; i < 25 * 3600; i++, t += 40000) b.add(t, 7000, 0);
  CHECK(b.samples() == power::BatteryTrend::kSamples && b.trend(t, 1800) == 0);
  CHECK(b.segments() == 2);
  for (int k = 0; k < 6; k++) b.state_changed(t + (uint64_t)k * 1000000u, (uint8_t)(k % 2 ? 0 : 2));
  CHECK(b.segments() == power::BatteryTrend::kSegments);
  b.add(t, 0, 0);  // no reading: skipped
  CHECK(b.last_mv() == 7000);
}

// the metrics lines (`kilo power --host`): key=value, under the METRICS 200-byte line
void test_metric_lines() {
  Rig r(sleep_cfg());
  r.body.batt_mv = 7600;
  r.start();
  r.key(kA7);
  r.run_ms(31000);
  r.run_ms(120000);
  r.key(kB6);
  uint32_t n = r.mind.metric_lines();
  CHECK(n == 11 + power::Sleep::kMetricLines + 1);  // then the sound line (test_sound)
  std::vector<std::string> got;
  for (uint32_t i = 0; i < n; i++) {
    char line[256];
    r.mind.metric_line(i, line, 201);
    CHECKF(std::strlen(line) <= 200 && std::strlen(line) > 0, "line %u: %zu bytes", (unsigned)i, std::strlen(line));
    got.push_back(line);
  }
  CHECKF(got[15].find("sound: on=1 ") == 0, "%s", got[15].c_str());
  const std::string& p = got[11];
  CHECKF(p.find("power: state=awake ") == 0 && p.find(" sleeps=1 wakes=1 last_wake=key:B6 ") != std::string::npos &&
             p.find(" asleep_s=12") != std::string::npos && p.find("light_sleep=on") != std::string::npos &&
             p.find(" servos=on limps=1 idle_sleeps=0") != std::string::npos,
         "%s", p.c_str());
  CHECKF(got[12].find("power_hw: net=on net_device=yes net_saves=1 ") == 0 &&
             got[12].find("arm_mhz=600 temp_c=41 throttled=0x0") != std::string::npos,
         "%s", got[12].c_str());
  CHECKF(got[13].find("battery: mv=7600 min_mv=7600 mv_per_min_1m=0.0 5m=0.0 ") == 0, "%s", got[13].c_str());
  CHECKF(got[14].find("power_segments: asleep:12") == 0 && got[14].find("7600-7600mV:0.0") != std::string::npos,
         "%s", got[14].c_str());
}

}  // namespace

// The calibration hold over the host channel (docs/rear-up-trick.md section 2):
// a laptop holds the calibration posture (kilo posture calib --host: control
// with keepalive, POSTURE CALIB) for as long as the owner needs, with light
// sleep, the remote and the behaviour layer on: nothing drops the pose, A7 is
// ignored while the laptop holds the body, and a second client's CAL SET
// (kilo cal nudge) reaches the body while the pose is held.
void test_calibration_hold_over_the_host() {
  MindConfig c = sleep_cfg();
  c.behaviour = true;
  uint32_t nkeys;
  c.authorized_keys = test_keys(&nkeys);
  c.authorized_key_count = nkeys;
  FakeRandom rnd;
  c.random = &rnd;
  Rig r(c, true);
  r.start();
  FakeConn& a = r.listener.connect(0);
  FakeConn& b = r.listener.connect(1);
  for (int k = 0; k < 40 && !(a.established && b.established); k++) {
    r.pass();
    a.pump();
    b.pump();
  }
  CHECK(a.established && b.established);
  uint8_t take[2] = {1, 1};
  kilo_host_posture_t calib = {KILO_POSTURE_CALIB, 0, 0};
  a.send(KILO_HOST_MSG_CONTROL, 1, take, 2);
  a.send(KILO_HOST_MSG_POSTURE, 2, &calib, sizeof calib);
  r.run_ms(100);
  CHECK(r.mind.host().controller() == 0 && r.posture_goal() == KILO_POSTURE_CALIB);
  size_t from = r.body.sent.size();
  r.key(kA7);  // the remote while the laptop holds the body: ignored
  bool held = true;
  for (int s = 0; s < 600 * 2; s++) {  // 10 minutes, the CLI's KEEPALIVE every 500 ms
    r.run_ms(500);
    a.send(KILO_HOST_MSG_KEEPALIVE, 0);
    a.pump();
    b.pump();
    held = held && r.posture_goal() == KILO_POSTURE_CALIB && r.mind.host().controller() == 0;
    if (s == 600) {  // half way: the nudge from the other window
      kilo_host_cal_t set = {KILO_CAL_OP_SET, 10, 136};
      b.send(KILO_HOST_MSG_CAL, 1, &set, sizeof set);
    }
  }
  CHECKF(held && r.state() == State::Awake, "held %d state %s behaviour %d controller %d",
         held, power::state_name(r.state()), r.mind.behaviour_active(), (int)r.mind.host().controller());
  CHECK(r.log.has("remote A7 (0xA2, sleep): ignored (a laptop client holds the body)"));
  CHECK(r.body.cal[10] == 136 && r.body.count(KILO_MSG_CAL_SET) == 1);
  bool status = false;
  for (const HostFrame& f : b.take()) status = status || (f.type == KILO_HOST_MSG_CAL_STATUS && f.seq == 1);
  CHECK(status);
  std::vector<int> since = r.postures_since(from);  // only CALIB went to the body (its 5 Hz refresh)
  CHECKF(since.size() == 1 && since[0] == KILO_POSTURE_CALIB, "%zu postures since", since.size());
}

int main() {
  test_enter_after_30s();
  test_cancel_and_restart();
  test_wake_by_key();
  test_wake_by_host();
  test_limp_with_laptop();
  test_no_limp_while_stopping();
  test_keys_from_limp();
  test_righted_before_limp();
  test_idle_sleep();
  test_network_policy();
  test_recorder_flushed_first();
  test_battery_trend();
  test_metric_lines();
  test_calibration_hold_over_the_host();
  return report("test_power");
}
