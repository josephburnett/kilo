// The IR remote: the key table and map (mind/tests/test_ir.py), and
// RemoteControl on the real link, walker and trick player over a scripted
// body: postures, tricks (from IDLE, TRICK and LIMP), walking with the
// pattern and speed keys, the balance toggle.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include "link/serial_link.h"
#include "planner/walker.h"
#include "remote/client.h"
#include "remote/remote.h"
#include "tests/body_rig.h"
#include "tests/check.h"
#include "tricks/library.h"
#include "tricks/player.h"

using namespace kilo;
using namespace kilo::remote;

namespace {

uint8_t cmd_of(const char* grid) {
  for (const Key& k : kKeyTable)
    if (std::strcmp(k.grid, grid) == 0) return k.command;
  return 0;
}

void test_key_table() {
  std::set<int> cmds;
  std::set<std::string> grids;
  for (int i = 0; i < kKeys; i++) {
    const Key& k = kKeyTable[i];
    cmds.insert(k.command);
    grids.insert(k.grid);
    // row by row from the top: A7 B7 C7, A6 ...
    char want[3] = {(char)('A' + i % 3), (char)('0' + 7 - i / 3), 0};
    CHECKF(std::strcmp(k.grid, want) == 0, "%s at %d", k.grid, i);
    CHECK(action_for(k.command) == &k && action_for((uint16_t)(k.command | kRepeat)) == nullptr);
    if (k.kind == Kind::Trick) CHECKF(tricks::find_trick(k.name) != nullptr, "trick %s", k.name);
    if (k.kind == Kind::Posture) CHECKF(posture_code(k.name) >= 0, "posture %s", k.name);
  }
  CHECK(cmds.size() == 21 && grids.size() == 21);
  CHECK(action_for(0x45) == nullptr);
  // ir.py ACTIONS by grid
  struct {
    const char* grid;
    Kind kind;
    const char* name;
  } want[] = {{"A7", Kind::Posture, "rest"},      {"B6", Kind::Posture, "stand"},    {"A5", Kind::Posture, "limp"},
              {"B3", Kind::Posture, "sit"},       {"C3", Kind::Trick, "stretch"},    {"A2", Kind::Trick, "hi"},
              {"B2", Kind::Trick, "push_up"},     {"C2", Kind::Trick, "pee"},        {"A1", Kind::Trick, "wash_face"},
              {"B1", Kind::Trick, "butt_up"},     {"C7", Kind::BalanceToggle, ""},   {"B7", Kind::Walk, "forward"},
              {"A6", Kind::Walk, "left"},         {"C6", Kind::Walk, "right"},       {"B5", Kind::Walk, "backward"},
              {"A4", Kind::Walk, "march"},        {"B4", Kind::GaitPattern, "crawl"}, {"C4", Kind::GaitPattern, "walk"},
              {"A3", Kind::GaitPattern, "trot"},  {"C5", Kind::Speed, "slower"},     {"C1", Kind::Speed, "faster"}};
  for (const auto& w : want) {
    const Key* k = action_for(cmd_of(w.grid));
    CHECKF(k != nullptr && k->kind == w.kind && std::strcmp(k->name, w.name) == 0, "%s", w.grid);
  }
  char d[48];
  describe(0x18, d, sizeof d);
  CHECK(std::strcmp(d, "B3 (0x18, sit)") == 0);
  describe(0x118, d, sizeof d);
  CHECK(std::strcmp(d, "B3 (0x18, sit) repeat") == 0);
  describe(0xA2, d, sizeof d);
  CHECK(std::strcmp(d, "A7 (0xA2, rest)") == 0);
  describe(0x45, d, sizeof d);
  CHECK(std::strcmp(d, "? (0x45)") == 0);
}

struct Rig : LinkListener {
  FakeBody body;
  CaptureLog log;
  SerialLink link{body.serial, body.clock, log};
  planner::Walker walker{link, body.clock, log};
  tricks::TrickPlayer player{link, body.clock, log};
  RemoteControl remote;

  explicit Rig(uint8_t mode = KILO_MODE_IDLE, const RemoteConfig& cfg = RemoteConfig())
      : remote(link, walker, player, body.clock, log, cfg) {
    body.mode = mode;
    link.set_listener(this);
    link.connect();
    run_ms(80);
    body.sent.clear();
  }
  void on_telemetry(const kilo_telemetry_t& t) override { walker.on_telemetry(t); }
  void on_event(const kilo_event_t& e) override {
    walker.on_event(e);
    player.on_event(e);
  }
  void pass() {
    body.step();
    link.mark_alive();
    link.poll();
    player.collect();
    walker.poll();
    player.poll();
    remote.poll();
    body.react();
  }
  void run_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i += 4) pass();
  }
  const char* key(const char* grid, uint32_t settle_ms = 40) {
    const Key* k = action_for(cmd_of(grid));
    remote.perform(*k, k->command);
    run_ms(settle_ms);
    return remote.last();
  }
  int posture_goal() const {
    const Goal* g = const_cast<SerialLink&>(link).goal(KILO_MSG_SET_POSTURE);
    return g == nullptr ? -1 : g->payload[1];
  }
};

void test_posture_keys() {
  {
    Rig r;
    CHECK(std::strcmp(r.key("B3"), "posture sit") == 0 && r.posture_goal() == KILO_POSTURE_SIT);
    CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 0 && r.log.has("remote B3 (0x18, sit): posture sit"));
  }
  {  // a playing trick is stopped first
    Rig r(KILO_MODE_TRICK);
    CHECK(std::strcmp(r.key("B3", 100), "posture sit") == 0 && r.body.count(KILO_MSG_TRICK_STOP) == 1);
    CHECK(r.posture_goal() == KILO_POSTURE_SIT);
  }
}

void test_trick_keys() {
  {
    Rig r;
    CHECK(std::strcmp(r.key("A2", 200), "trick hi") == 0 && r.body.count(KILO_MSG_TRICK_PLAY) == 1);
    CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 0);
  }
  {  // TRICK: stopped first
    Rig r(KILO_MODE_TRICK);
    r.key("A2", 300);
    CHECK(std::strcmp(r.remote.last(), "trick hi") == 0 && r.body.count(KILO_MSG_TRICK_STOP) == 1 &&
          r.body.count(KILO_MSG_TRICK_PLAY) == 1);
  }
  {  // LIMP: stand, settle 1 s, then the trick
    Rig r(KILO_MODE_LIMP);
    r.key("A2", 40);
    CHECK(r.posture_goal() == KILO_POSTURE_STAND && r.body.count(KILO_MSG_TRICK_PLAY) == 0 && r.remote.busy());
    r.body.mode = KILO_MODE_IDLE;
    r.run_ms(900);
    CHECK(r.body.count(KILO_MSG_TRICK_PLAY) == 0);
    r.run_ms(300);
    CHECK(r.body.count(KILO_MSG_TRICK_PLAY) == 1 && std::strcmp(r.remote.last(), "trick hi") == 0);
  }
}

void test_walking_keys() {
  Rig r;
  CHECK(std::strcmp(r.key("C4"), "pattern walk") == 0 && !r.walker.active());  // not walking: only stored
  CHECK(std::strcmp(r.key("B7", 800), "walk forward (walk)") == 0 && r.walker.active());
  CHECKF(std::fabs(r.remote.speed_mm_s() - 11.1) < 0.05, "%.2f", r.remote.speed_mm_s());  // the pattern's default
  CHECK(std::strcmp(r.walker.planner().pattern().name, "walk") == 0);
  CHECK(std::strcmp(r.key("A3"), "walk forward (trot)") == 0);  // walking: changed to the new pattern
  CHECKF(std::fabs(r.remote.speed_mm_s() - 55.6) < 0.05, "%.2f", r.remote.speed_mm_s());
  CHECK(std::strcmp(r.key("B5"), "walk backward (trot)") == 0 && r.walker.planner().direction() == planner::Direction::Back);
  CHECK(std::strcmp(r.key("A6"), "walk left (trot)") == 0 && r.walker.planner().direction() == planner::Direction::Left);
  CHECK(std::strcmp(r.key("C6"), "walk right (trot)") == 0);
  CHECK(std::strcmp(r.key("A4"), "walk march (trot)") == 0);
  CHECK(std::strcmp(r.key("B7"), "walk forward (trot)") == 0);
  // speed keys while walking change it at once (ir.py test_remote_speed_keys)
  CHECK(std::strcmp(r.key("C1"), "walk forward speed 1.4x (trot 78 mm/s)") == 0);
  CHECK(std::strcmp(r.key("C1"), "walk forward speed 2x (trot 93 mm/s)") == 0);  // capped
  CHECK(std::strcmp(r.key("C1"), "walk forward speed 2x (trot 93 mm/s)") == 0);  // the top step
  for (int i = 0; i < 5; i++) r.key("C5");
  CHECK(r.remote.speed_step() == 0 && std::fabs(r.remote.speed_mm_s() - 27.8) < 0.05);
  // a posture key stops the walk the planned way, waits, then takes the posture
  r.key("B3", 40);
  CHECK(r.remote.busy() && r.walker.active());
  for (int i = 0; i < 2000 && r.remote.busy(); i++) r.pass();
  CHECK(!r.walker.active() && r.walker.ended_ok() && std::strcmp(r.remote.last(), "posture sit") == 0);
  CHECK(r.posture_goal() == KILO_POSTURE_SIT);
  CHECK(std::strcmp(r.key("A3"), "pattern trot") == 0 && !r.walker.active());  // stopped: only stored
  CHECK(std::strcmp(r.key("C5"), "speed 0.5x (trot 28 mm/s)") == 0);
  // a trick key stops a walk too
  r.key("B7", 800);
  CHECK(r.walker.active());
  r.key("A2", 40);
  for (int i = 0; i < 2000 && r.remote.busy(); i++) r.pass();
  r.run_ms(100);
  CHECK(!r.walker.active() && std::strcmp(r.remote.last(), "trick hi") == 0 && r.body.count(KILO_MSG_TRICK_PLAY) == 1);
}

void test_balance_toggle() {
  {
    Rig r;
    CHECK(std::strcmp(r.key("C7"), "balance on") == 0 && client_balance_enabled(r.link));
    CHECK(r.link.goal(KILO_MSG_SET_BALANCE)->payload[1] == 128);  // the firmware's Kp 0.5
    CHECK(std::strcmp(r.key("C7"), "balance off") == 0 && !client_balance_enabled(r.link));
  }
  {
    RemoteConfig cfg;
    cfg.balance_kp = 0.99;  // kilo run --balance 0.99
    Rig r(KILO_MODE_IDLE, cfg);
    r.key("C7");
    CHECK(r.link.goal(KILO_MSG_SET_BALANCE)->payload[1] == 253);
  }
}

void test_keys_queue() {
  Rig r(KILO_MODE_LIMP);  // the trick waits 1 s: the keys behind it queue
  r.key("A2", 8);
  r.key("B3", 8);
  r.key("C7", 8);
  CHECK(r.remote.busy() && r.body.count(KILO_MSG_TRICK_PLAY) == 0);
  r.body.mode = KILO_MODE_IDLE;
  for (int i = 0; i < 1000 && r.remote.busy(); i++) r.pass();
  CHECK(r.log.has("remote A2 (0x10, hi): trick hi") && r.log.has("remote B3 (0x18, sit): posture sit") &&
        std::strcmp(r.remote.last(), "balance on") == 0);
}

}  // namespace

int main() {
  test_key_table();
  test_posture_keys();
  test_trick_keys();
  test_walking_keys();
  test_balance_toggle();
  test_keys_queue();
  return report("remote");
}
