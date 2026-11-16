// The IR remote: the key table and map (mind/tests/test_ir.py), and
// RemoteControl on the real link, walker and trick player over a scripted
// body: the posture keys (B6, A7, C5), the gait keys (A4, B4, C4), the turn
// keys (A6, C6), the speed ladder (B7, B5) through zero into backward, the
// freeze (C7), the behaviour key (A5) on a fake runner and the rear-up key
// (B2: start, the toggle, the slow stop, the battery guard).
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
    if (k.kind == Kind::Posture) CHECKF(posture_code(k.name) >= 0, "posture %s", k.name);
    if (k.kind == Kind::Gait) CHECKF(planner::find_pattern(k.name) != nullptr, "gait %s", k.name);
    if (k.kind == Kind::Trick) CHECKF(tricks::find_trick(k.name) != nullptr, "trick %s", k.name);
    // rows 1-3 do nothing, but B2
    CHECKF((k.grid[1] <= '3' && std::strcmp(k.grid, "B2") != 0) == (k.kind == Kind::Ignore), "%s", k.grid);
  }
  CHECK(cmds.size() == 21 && grids.size() == 21);
  CHECK(action_for(0x45) == nullptr);
  // ir.py ACTIONS by grid
  struct {
    const char* grid;
    Kind kind;
    const char* name;
  } want[] = {{"A7", Kind::Posture, "rest"},  {"B7", Kind::Speed, "faster"},  {"C7", Kind::Freeze, ""},
              {"A6", Kind::Turn, "left"},     {"B6", Kind::Posture, "stand"}, {"C6", Kind::Turn, "right"},
              {"A5", Kind::Behaviour, ""},    {"B5", Kind::Speed, "slower"},  {"C5", Kind::Posture, "calib"},
              {"A4", Kind::Gait, "trot"},     {"B4", Kind::Gait, "crawl"},    {"C4", Kind::Gait, "walk"},
              {"B2", Kind::Trick, "rear_up"}};
  for (const auto& w : want) {
    const Key* k = action_for(cmd_of(w.grid));
    CHECKF(k != nullptr && k->kind == w.kind && std::strcmp(k->name, w.name) == 0, "%s", w.grid);
  }
  char d[48];
  describe(0x02, d, sizeof d);
  CHECK(std::strcmp(d, "B6 (0x02, stand)") == 0);
  describe(0x102, d, sizeof d);
  CHECK(std::strcmp(d, "B6 (0x02, stand) repeat") == 0);
  describe(0xA2, d, sizeof d);
  CHECK(std::strcmp(d, "A7 (0xA2, sleep)") == 0);
  describe(0x90, d, sizeof d);
  CHECK(std::strcmp(d, "C5 (0x90, calibration posture)") == 0);
  describe(0x18, d, sizeof d);
  CHECK(std::strcmp(d, "B3 (0x18, none)") == 0);
  describe(0x45, d, sizeof d);
  CHECK(std::strcmp(d, "? (0x45)") == 0);
}

// The Mind's runner as the remote sees it.
struct FakeRunner : BehaviourHook {
  bool on = false;
  int starts = 0, stops = 0;
  bool active() override { return on; }
  bool start(const char** what) override {
    *what = nullptr;
    on = true;
    starts++;
    return true;
  }
  void stop() override {
    on = false;
    stops++;
  }
};

struct Rig : LinkListener {
  FakeBody body;
  CaptureLog log;
  SerialLink link{body.serial, body.clock, log};
  planner::Walker walker{link, body.clock, log};
  tricks::TrickPlayer player{link, body.clock, log};
  RemoteControl remote{link, walker, player, body.clock, log};
  FakeRunner runner;

  explicit Rig(uint8_t mode = KILO_MODE_IDLE) {
    body.mode = mode;
    link.set_listener(this);
    remote.set_behaviour(&runner);
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
  // the key, then until the remote is done (a planned stop takes seconds)
  const char* key_done(const char* grid) {
    key(grid, 8);
    for (int i = 0; i < 3000 && remote.busy(); i++) pass();
    return remote.last();
  }
  int posture_goal() const {
    const Goal* g = const_cast<SerialLink&>(link).goal(KILO_MSG_SET_POSTURE);
    return g == nullptr ? -1 : g->payload[1];
  }
  planner::Direction dir() { return walker.planner().direction(); }
  double speed() { return (double)walker.planner().timing().speed_mm_s; }
};

#define LAST_IS(r, grid, want)                                                            \
  do {                                                                                    \
    const char* got_ = (r).key(grid);                                                     \
    CHECKF(std::strcmp(got_, want) == 0, "%s: got \"%s\", want \"%s\"", grid, got_, want); \
  } while (0)

void test_posture_keys() {
  {
    Rig r;
    CHECK(std::strcmp(r.key("B6"), "posture stand") == 0 && r.posture_goal() == KILO_POSTURE_STAND);
    CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 0 && r.log.has("remote B6 (0x02, stand): posture stand"));
    CHECK(std::strcmp(r.key("A7"), "posture rest") == 0 && r.posture_goal() == KILO_POSTURE_REST);
    CHECK(r.log.has("remote A7 (0xA2, sleep): posture rest"));
    // C5: the body's calibration posture
    CHECK(std::strcmp(r.key("C5"), "posture calib") == 0 && r.posture_goal() == KILO_POSTURE_CALIB);
    CHECK(r.log.has("remote C5 (0x90, calibration posture): posture calib"));
  }
  {  // a playing trick is stopped first
    Rig r(KILO_MODE_TRICK);
    CHECK(std::strcmp(r.key("B6", 100), "posture stand") == 0 && r.body.count(KILO_MSG_TRICK_STOP) == 1);
    CHECK(r.posture_goal() == KILO_POSTURE_STAND);
  }
  {  // a walk is stopped the planned way first, also by C5
    Rig r;
    r.key("A4", 800);
    CHECK(r.walker.active());
    r.key("C5", 40);
    CHECK(r.remote.busy() && r.walker.active());
    for (int i = 0; i < 2000 && r.remote.busy(); i++) r.pass();
    CHECK(!r.walker.active() && r.walker.ended_ok() && std::strcmp(r.remote.last(), "posture calib") == 0);
    CHECK(r.posture_goal() == KILO_POSTURE_CALIB);
  }
}

void test_gait_keys() {
  Rig r;
  // not walking: a gait key starts walking forward in it at 1x
  LAST_IS(r, "C4", "walk forward (walk) 80.0 mm/s");
  r.run_ms(800);
  CHECK(r.walker.active() && r.dir() == planner::Direction::Forward && r.remote.level() == 3);
  CHECK(std::strcmp(r.walker.planner().pattern().name, "walk") == 0);
  CHECK(r.log.has("remote C4 (0xB0, walk): walk forward (walk) 80.0 mm/s"));
  // the same gait again: nothing (no restart, the speed kept)
  size_t feet = r.body.count(KILO_MSG_FEET);
  LAST_IS(r, "C4", "walk forward (walk) 80.0 mm/s: unchanged");
  CHECK(r.walker.active() && r.body.count(KILO_MSG_FEET) > feet);
  // another gait while walking: changed, the level kept
  r.key("B7");  // +4
  LAST_IS(r, "A4", "walk forward (trot) 165.6 mm/s");  // 1.15x
  CHECK(r.remote.level() == 4 && std::strcmp(r.remote.pattern(), "trot") == 0);
  LAST_IS(r, "B4", "walk forward (crawl) 25.6 mm/s");
  // backward, then the gait in use: forward again at the level's magnitude
  for (int i = 0; i < 5; i++) r.key("B5");  // +4 +3 +2 +1 -1 -2
  CHECK(r.remote.level() == -2);
  LAST_IS(r, "B4", "walk forward (crawl) 15.6 mm/s");
  CHECK(r.remote.level() == 2 && r.dir() == planner::Direction::Forward);
  // every stop puts the level back: B6, then a gait key starts at 1x
  CHECK(std::strcmp(r.key_done("B6"), "posture stand") == 0 && !r.walker.active());
  CHECK(r.remote.level() == 3 && r.remote.heading() == Heading::Straight);
  LAST_IS(r, "A4", "walk forward (trot) 144.0 mm/s");
  r.run_ms(800);
  CHECK(r.walker.active() && std::fabs(r.speed() - 144.0) < 0.05);
  // a walk that ended by itself (the body left GAIT): the next gait key starts again at 1x
  r.key("B7");
  r.walker.abort();
  LAST_IS(r, "A4", "walk forward (trot) 144.0 mm/s");
  CHECK(r.remote.level() == 3);
}

void test_speed_ladder() {
  Rig r;
  // not walking: ignored, never a walk
  LAST_IS(r, "B7", "ignored (not walking)");
  LAST_IS(r, "B5", "ignored (not walking)");
  CHECK(!r.walker.active() && r.body.count(KILO_MSG_FEET) == 0);
  CHECK(r.log.has("remote B7 (0x62, faster): ignored (not walking)"));
  r.key("A4", 800);
  CHECK(r.walker.active());
  // down from 1x through zero into backward (since 2026-11-16 the levels are 0.4, 0.7, 1, 1.15,
  // 1.25 x the default: the trot 58 to 180 mm/s, inside its 42..184)
  LAST_IS(r, "B5", "speed 0.7x (trot 100.8 mm/s forward)");
  LAST_IS(r, "B5", "speed 0.4x (trot 57.6 mm/s forward)");
  LAST_IS(r, "B5", "speed -0.4x (trot 57.6 mm/s back)");
  CHECK(r.remote.level() == -1 && r.dir() == planner::Direction::Back);
  LAST_IS(r, "B5", "speed -0.7x (trot 100.8 mm/s back)");
  LAST_IS(r, "B5", "speed -1x (trot 144.0 mm/s back)");
  LAST_IS(r, "B5", "speed -1.15x (trot 165.6 mm/s back)");
  LAST_IS(r, "B5", "speed -1.25x (trot 180.0 mm/s back)");
  LAST_IS(r, "B5", "speed -1.25x (trot 180.0 mm/s back): already the fastest back");
  CHECK(r.remote.level() == -5 && std::fabs(r.speed() - 180.0) < 0.05);
  // and up again, through zero into forward, to the top
  for (int i = 0; i < 4; i++) r.key("B7");
  CHECK(r.remote.level() == -1);
  LAST_IS(r, "B7", "speed 0.4x (trot 57.6 mm/s forward)");
  CHECK(r.dir() == planner::Direction::Forward);
  for (int i = 0; i < 3; i++) r.key("B7");
  LAST_IS(r, "B7", "speed 1.25x (trot 180.0 mm/s forward)");
  LAST_IS(r, "B7", "speed 1.25x (trot 180.0 mm/s forward): already the fastest");
  CHECK(r.remote.level() == 5);
  LAST_IS(r, "B5", "speed 1.15x (trot 165.6 mm/s forward)");
  LAST_IS(r, "B5", "speed 1x (trot 144.0 mm/s forward)");
  CHECK(r.log.has("remote B5 (0xA8, slower): speed 1.15x (trot 165.6 mm/s forward)"));
  LAST_IS(r, "C4", "walk forward (walk) 80.0 mm/s");
}

void test_turn_keys() {
  Rig r;
  // from standing: the turn in the selected gait at 1x
  LAST_IS(r, "A6", "walk left (trot) 144.0 mm/s");
  r.run_ms(800);
  CHECK(r.walker.active() && r.dir() == planner::Direction::Left);
  LAST_IS(r, "A6", "walk left (trot) 144.0 mm/s: unchanged");
  LAST_IS(r, "C6", "walk right (trot) 144.0 mm/s");
  CHECK(r.dir() == planner::Direction::Right);
  // the magnitude only, 1..5, no sign flip during a turn
  LAST_IS(r, "B5", "speed 0.7x (trot 100.8 mm/s right)");
  LAST_IS(r, "B5", "speed 0.4x (trot 57.6 mm/s right)");
  LAST_IS(r, "B5", "speed 0.4x (trot 57.6 mm/s right): already the slowest");
  CHECK(r.remote.level() == 1 && r.dir() == planner::Direction::Right);
  LAST_IS(r, "B7", "speed 0.7x (trot 100.8 mm/s right)");
  // the gait in use: back to straight forward at the magnitude
  LAST_IS(r, "A4", "walk forward (trot) 100.8 mm/s");
  CHECK(r.remote.heading() == Heading::Straight && r.dir() == planner::Direction::Forward);
  // another gait during a turn: the turn goes on in it
  r.key("C6");
  LAST_IS(r, "C4", "walk right (walk) 56.0 mm/s");
  // backward, then a turn: the turn at the magnitude, then the gait key forward
  r.key("C4");
  r.key("B5");
  r.key("B5");  // +2: +1, -1
  CHECK(r.remote.level() == -1);
  LAST_IS(r, "A6", "walk left (walk) 32.0 mm/s");
  CHECK(r.remote.level() == 1);
  LAST_IS(r, "C4", "walk forward (walk) 32.0 mm/s");
}

void test_freeze() {
  {  // walking: the walk is aborted at once (no planned stop), then only B6 / A7 act
    Rig r;
    r.key("A4", 800);
    CHECK(r.walker.active() && r.body.mode == KILO_MODE_GAIT);
    size_t postures = r.body.count(KILO_MSG_SET_POSTURE);
    LAST_IS(r, "C7", "frozen (walk aborted): only B6 or A7 act now");
    CHECK(!r.walker.active() && r.walker.ended_ok() && std::strcmp(r.walker.ended(), "aborted (C7 freeze)") == 0);
    CHECK(r.remote.frozen() && r.posture_goal() == -1);  // no posture over the body's own settle, no LIMP
    CHECK(r.body.count(KILO_MSG_SET_POSTURE) == postures);
    // the body: the queue runs dry, it holds, settles and stays in STANCE (spec FEET, feet.h)
    size_t lines = r.log.lines.size();
    r.body.event(KILO_EVENT_FEET_UNDERRUN, 3);
    r.run_ms(120);
    r.body.mode = KILO_MODE_IDLE;
    r.body.event(KILO_EVENT_GAIT_STOPPED, 1);
    r.run_ms(100);
    // only the link's two event lines: no walk ended over again, no remote line, nothing that reacts
    CHECKF(r.log.lines.size() == lines + 2, "%zu new lines after the settle", r.log.lines.size() - lines);
    for (size_t i = lines; i < r.log.lines.size(); i++)
      CHECKF(r.log.lines[i].rfind("event ", 0) == 0, "%s", r.log.lines[i].c_str());
    CHECK(!r.walker.active() && r.remote.frozen() && r.body.count(KILO_MSG_FEET) > 0);
    // every key but B6 and A7: ignored
    const char* others[] = {"B7", "B5", "A4", "B4", "C4", "A6", "C6", "C5", "A5", "C7"};
    for (const char* g : others) {
      LAST_IS(r, g, "ignored (frozen: B6 or A7 to go on)");
      CHECK(!r.walker.active() && r.posture_goal() == -1 && r.runner.starts == 0);
    }
    CHECK(r.log.has("remote C5 (0x90, calibration posture): ignored (frozen: B6 or A7 to go on)"));
    CHECK(std::strcmp(r.key_done("B6"), "unfrozen, posture stand") == 0 && !r.remote.frozen());
    CHECK(r.posture_goal() == KILO_POSTURE_STAND);
    LAST_IS(r, "A4", "walk forward (trot) 144.0 mm/s");  // and the keys act again, from 1x
  }
  {  // standing: holds the posture; A7 goes on too
    Rig r;
    r.key("B6");
    LAST_IS(r, "C7", "frozen (holding the posture): only B6 or A7 act now");
    CHECK(r.posture_goal() == KILO_POSTURE_STAND);
    LAST_IS(r, "C5", "ignored (frozen: B6 or A7 to go on)");
    CHECK(r.posture_goal() == KILO_POSTURE_STAND);
    LAST_IS(r, "A7", "unfrozen, posture rest");
  }
  {  // a trick playing (the behaviour runner's): stopped
    Rig r(KILO_MODE_TRICK);
    LAST_IS(r, "C7", "frozen (trick stopped): only B6 or A7 act now");
    CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 1);
  }
  {  // ahead of a planned stop under way: that key is cancelled, the ones behind it dropped
    Rig r;
    r.key("A4", 800);
    r.key("B6", 40);
    r.key("A7", 8);
    CHECK(r.remote.busy() && r.walker.active());
    LAST_IS(r, "C7", "frozen (walk aborted): only B6 or A7 act now");
    CHECK(!r.remote.busy() && !r.walker.active() && r.posture_goal() == -1);
    CHECK(r.log.has("remote B6 (0x02, stand): cancelled (C7 froze Kilo)") &&
          r.log.has("remote A7 (0xA2, sleep): dropped (C7 froze Kilo)"));
    r.run_ms(500);
    CHECK(r.posture_goal() == -1 && r.remote.frozen());
  }
}

void test_behaviour_key() {
  Rig r;
  r.key("A4", 800);
  CHECK(r.walker.active());
  // A5: the walk stopped the planned way first, then the runner
  CHECK(std::strcmp(r.key_done("A5"), "behaviour runner started") == 0 && r.runner.on && r.runner.starts == 1);
  CHECK(!r.walker.active() && r.walker.ended_ok());
  LAST_IS(r, "A5", "behaviour runner already running");  // idempotent
  CHECK(r.runner.starts == 1);
  const char* others[] = {"B7", "B5", "A4", "C4", "A6", "C5"};
  for (const char* g : others) LAST_IS(r, g, "ignored (behaviour runner active)");
  CHECK(!r.walker.active() && r.runner.on);
  // B6: stops it, then stands
  CHECK(std::strcmp(r.key_done("B6"), "behaviour runner stopped, posture stand") == 0 && !r.runner.on);
  CHECK(r.posture_goal() == KILO_POSTURE_STAND);
  // A7 likewise
  r.key_done("A5");
  CHECK(std::strcmp(r.key_done("A7"), "behaviour runner stopped, posture rest") == 0 && !r.runner.on);
  // C7: stops it and freezes
  r.key_done("A5");
  CHECK(r.runner.starts == 3);
  LAST_IS(r, "C7", "frozen (behaviour runner stopped): only B6 or A7 act now");
  CHECK(!r.runner.on && r.runner.stops == 3 && r.remote.frozen());
  CHECK(r.log.has("remote A5 (0xE0, behaviour): behaviour runner started"));
}

uint8_t stop_ticks_sent(const Rig& r) {
  std::vector<Sent> s = r.body.of(KILO_MSG_TRICK_STOP);
  return s.empty() ? 0xFF : s.back().payload[0];
}

void test_rear_up_key() {
  const tricks::Trick* t = tricks::find_trick("rear_up");
  CHECK(t != nullptr && t->stop_ticks == 75 && t->legacy[0] == '\0' && tricks::find_trick("") == nullptr);
  {
    Rig r;
    LAST_IS(r, "B2", "trick rear_up started (19 s; B2 again, B6 or C7 bring it down)");
    CHECK(r.body.count(KILO_MSG_TRICK_PLAY) == 1 && r.player.busy());
    CHECK(r.log.has("remote B2 (0x38, rear up): trick rear_up started"));
    r.body.mode = KILO_MODE_TRICK;
    r.run_ms(40);
    // pressed again while it plays: down, over the trick's stop_ticks
    LAST_IS(r, "B2", "trick stopped: coming down");
    CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 1 && stop_ticks_sent(r) == 75);
  }
  {  // B6 while it plays: TRICK_STOP over 75 ticks, the crossfade waited for, then STAND
    Rig r;
    r.key("B2");
    r.body.mode = KILO_MODE_TRICK;
    r.run_ms(40);
    r.key("B6", 40);
    CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 1 && stop_ticks_sent(r) == 75 && r.remote.busy());
    r.body.mode = KILO_MODE_IDLE;
    r.body.event(KILO_EVENT_TRICK_DONE, (int16_t)((KILO_TRICK_DONE_REASON_STOPPED << 8) | r.player.result().trick_id));
    r.run_ms(1400);  // the 1.5 s crossfade (+0.1 s) is not over yet
    CHECK(r.remote.busy() && r.posture_goal() != KILO_POSTURE_STAND);
    r.run_ms(400);
    CHECK(!r.remote.busy() && r.posture_goal() == KILO_POSTURE_STAND && std::strcmp(r.remote.last(), "posture stand") == 0);
  }
  {  // C7 while it plays: stopped the same way (it comes down), frozen; B2 does not unfreeze
    Rig r;
    r.key("B2");
    r.body.mode = KILO_MODE_TRICK;
    r.run_ms(40);
    LAST_IS(r, "C7", "frozen (trick stopped): only B6 or A7 act now");
    CHECK(stop_ticks_sent(r) == 75);
    LAST_IS(r, "B2", "ignored (frozen: B6 or A7 to go on)");
  }
  {  // walking: the planned stop first, then the trick
    Rig r;
    r.key("A4", 800);
    CHECK(r.walker.active());
    CHECK(std::strncmp(r.key_done("B2"), "trick rear_up started", 21) == 0 && !r.walker.active());
    r.run_ms(100);  // the player's TRICK_COMMIT, then TRICK_PLAY
    CHECKF(r.body.count(KILO_MSG_TRICK_PLAY) == 1 && r.remote.level() == 3, "plays %zu level %d",
           r.body.count(KILO_MSG_TRICK_PLAY), r.remote.level());
  }
  {  // a flat pack, USB power: refused, nothing moves
    Rig r;
    r.body.batt_mv = 7300;
    r.run_ms(40);
    LAST_IS(r, "B2", "rear_up refused: battery 7300 mV (needs 7400)");
    r.body.batt_mv = 0;
    r.body.flags |= KILO_TELEMETRY_FLAGS_NO_BATTERY;
    r.run_ms(40);
    LAST_IS(r, "B2", "rear_up refused: on USB power (no battery)");
    CHECK(r.body.count(KILO_MSG_TRICK_PLAY) == 0 && r.body.count(KILO_MSG_TRICK_COMMIT) == 0);
  }
}

void test_keys_queue() {
  Rig r;
  r.key("A4", 800);
  r.key("B6", 8);  // the planned stop takes a while: the keys behind it queue
  r.key("C4", 8);
  r.key("B7", 8);
  CHECK(r.remote.busy());
  for (int i = 0; i < 3000 && r.remote.busy(); i++) r.pass();
  CHECK(r.log.has("remote B6 (0x02, stand): posture stand") &&
        r.log.has("remote C4 (0xB0, walk): walk forward (walk) 80.0 mm/s") &&
        std::strcmp(r.remote.last(), "speed 1.15x (walk 92.0 mm/s forward)") == 0);
}

}  // namespace

int main() {
  test_key_table();
  test_posture_keys();
  test_gait_keys();
  test_speed_ladder();
  test_turn_keys();
  test_freeze();
  test_behaviour_key();
  test_rear_up_key();
  test_keys_queue();
  return report("remote");
}
