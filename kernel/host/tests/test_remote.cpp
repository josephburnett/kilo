// The IR remote: the key table and map, and
// RemoteControl on the real link, walker and trick player over a scripted
// body: the posture keys (B6, A7, C5), the gait keys (A4, B4, C4), the turn
// keys (A6, C6: in place from standing, steering while walking), out of a
// turn straight on (B7, B5), the speed ladder (B7, B5) through zero into
// backward, the freeze (C7), the behaviour key (A5) on a fake runner, C5
// twice (the calibration posture, then the self-measurement on a fake
// routine) and the rear-up key (A1 since 2026-11-22, A2 and B2 before: off
// by default, like rows 1-3; switched on: start, the toggle, the slow stop,
// the battery guard).
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

void test_key_table(uint8_t stage) {
  // the rear-up key's stage: 0 off, 1-3 the trick A1 starts (rear_up, rear_up_2, rear_up_3)
  static const char* const kStageTrick[] = {"", "rear_up", "rear_up_2", "rear_up_3"};
  static const char* const kStageLabel[] = {"none", "rear up", "rear up 2", "rear up 3"};
  set_rear_up_key(stage);
  CHECK(rear_up_key() == stage);
  const bool rear_up = stage != 0;
  std::set<int> cmds;
  std::set<std::string> grids;
  for (int i = 0; i < kKeys; i++) {
    const Key& k = kKeyTable[i];
    cmds.insert(k.command);
    grids.insert(k.grid);
    // row by row from the top: A7 B7 C7, A6 ...
    char want[3] = {(char)('A' + i % 3), (char)('0' + 7 - i / 3), 0};
    CHECKF(std::strcmp(k.grid, want) == 0, "%s at %d", k.grid, i);
    // A1 with the rear-up key off: an Ignore entry; at stage 2 or 3 another Trick entry
    const bool off_a1 = k.kind == Kind::Trick && stage != 1;
    CHECK((action_for(k.command) == &k) != off_a1 && action_for((uint16_t)(k.command | kRepeat)) == nullptr);
    if (k.kind == Kind::Trick) {
      const Key* a1 = action_for(k.command);
      CHECK(a1->kind == (rear_up ? Kind::Trick : Kind::Ignore) && std::strcmp(a1->label, kStageLabel[stage]) == 0 &&
            std::strcmp(a1->name, kStageTrick[stage]) == 0 && std::strcmp(a1->grid, "A1") == 0);
      if (rear_up) CHECKF(tricks::find_trick(a1->name) != nullptr, "trick %s", a1->name);
    }
    if (k.kind == Kind::Posture) CHECKF(posture_code(k.name) >= 0, "posture %s", k.name);
    if (k.kind == Kind::Gait) CHECKF(planner::find_pattern(k.name) != nullptr, "gait %s", k.name);
    if (k.kind == Kind::Trick) CHECKF(tricks::find_trick(k.name) != nullptr, "trick %s", k.name);
    // rows 1-3 do nothing, but A1 (the rear-up) and B2 (the meow)
    CHECKF((k.grid[1] <= '3' && std::strcmp(k.grid, "A1") != 0 && std::strcmp(k.grid, "B2") != 0) ==
               (k.kind == Kind::Ignore),
           "%s", k.grid);
  }
  CHECK(cmds.size() == 21 && grids.size() == 21);
  CHECK(action_for(0x45) == nullptr);
  // the owner's map by grid (keys.h)
  struct {
    const char* grid;
    Kind kind;
    const char* name;
  } want[] = {{"A7", Kind::Posture, "rest"},  {"B7", Kind::Speed, "faster"},  {"C7", Kind::Freeze, ""},
              {"A6", Kind::Turn, "left"},     {"B6", Kind::Posture, "stand"}, {"C6", Kind::Turn, "right"},
              {"A5", Kind::Behaviour, ""},    {"B5", Kind::Speed, "slower"},  {"C5", Kind::Posture, "calib"},
              {"A4", Kind::Gait, "trot"},     {"B4", Kind::Gait, "crawl"},    {"C4", Kind::Gait, "walk"},
              {"A1", rear_up ? Kind::Trick : Kind::Ignore, kStageTrick[stage]},  // A2 until 2026-11-22
              {"B2", Kind::Sound, "meow"},  // the rear-up key until 2026-11-17; a meow since 2026-11-19 (docs/sound.md)
              {"A2", Kind::Ignore, ""},     // the rear-up until 2026-11-22
              {"B1", Kind::Ignore, ""}};    // the self-measurement until 2026-11-22 (now C5 twice)
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
  describe(0x42, d, sizeof d);
  char want_a1[32];
  std::snprintf(want_a1, sizeof want_a1, "A1 (0x42, %s)", kStageLabel[stage]);
  CHECKF(std::strcmp(d, want_a1) == 0, "%s", d);
  describe(0x10, d, sizeof d);
  CHECKF(std::strcmp(d, "A2 (0x10, none)") == 0, "%s", d);
  describe(0x38, d, sizeof d);
  CHECKF(std::strcmp(d, "B2 (0x38, meow)") == 0, "%s", d);
  set_rear_up_key(0);
}

// The Mind's self-measurement as the remote sees it: refuses until the body has been in STAND for
// `ready_after` passes of start() (as the routine refuses while he settles), then runs until stopped.
struct FakeMeasure : MeasureHook {
  bool on = false;
  int asks = 0, ready_after = 3, stops = 0;
  const char* refusal = "not level and still";
  bool active() override { return on; }
  const char* start() override {
    if (++asks < ready_after) return refusal;
    on = true;
    return nullptr;
  }
  void stop(const char* /*why*/) override {
    on = false;
    stops++;
  }
};

// The Mind's runner as the remote sees it.
struct FakeRunner : BehaviourHook {
  bool on = false, refuse_trick = false;
  int starts = 0, stops = 0, tricks = 0;
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
  bool trick(const char** what) override {
    if (refuse_trick) {
      *what = "random mode: pee is already waiting";
      return false;
    }
    tricks++;
    *what = "random mode: pee now";
    return true;
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
  FakeMeasure measure;

  explicit Rig(uint8_t mode = KILO_MODE_IDLE) {
    body.mode = mode;
    link.set_listener(this);
    remote.set_behaviour(&runner);
    remote.set_measure(&measure);
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
    CHECK(std::strcmp(r.key("C5"), "posture calib (C5 again: measure himself)") == 0 && r.posture_goal() == KILO_POSTURE_CALIB);
    CHECK(r.log.has("remote C5 (0x90, calibration posture): posture calib (C5 again: measure himself)"));
  }
  {  // C5 twice, no key between: the calibration posture, then STAND and the self-measurement
    Rig r;
    CHECK(std::strcmp(r.key("C5"), "posture calib (C5 again: measure himself)") == 0);
    CHECK(r.measure.asks == 0);
    r.key("C5", 8);
    CHECK(r.remote.busy() && r.posture_goal() == KILO_POSTURE_STAND && !r.measure.on);
    for (int i = 0; i < 3000 && r.remote.busy(); i++) r.pass();
    CHECKF(std::strcmp(r.remote.last(), "standing, measuring (about 80 s; any key stops it)") == 0, "%s",
           r.remote.last());
    CHECK(r.measure.on && r.measure.asks == 3 && !r.remote.last_failed());
    // a key while it runs: stops it; C5 only that, and no calibration posture
    LAST_IS(r, "C5", "measurement stopped: standing");
    CHECK(!r.measure.on && r.measure.stops == 1 && r.posture_goal() == KILO_POSTURE_STAND);
    // C5 again now: the calibration posture (the last key was not its posture)
    LAST_IS(r, "C5", "posture calib (C5 again: measure himself)");
  }
  {  // another key between: the second C5 is a first one again
    Rig r;
    r.key("C5");
    r.key("B3");  // row 3: nothing here (the Mind logs it), but it is a key between
    LAST_IS(r, "C5", "posture calib (C5 again: measure himself)");
    r.key("B6");
    LAST_IS(r, "C5", "posture calib (C5 again: measure himself)");
    CHECK(r.measure.asks == 0);
  }
  {  // never ready: refused after the wait, he stands
    Rig r;
    r.measure.ready_after = 1000000;
    r.key("C5");
    r.key("C5", 8);
    for (int i = 0; i < 4000 && r.remote.busy(); i++) r.pass();
    CHECKF(std::strcmp(r.remote.last(), "standing, measure refused: not level and still") == 0, "%s",
           r.remote.last());
    CHECK(r.remote.last_failed() && !r.measure.on && r.posture_goal() == KILO_POSTURE_STAND);
  }
  {  // a key during the wait cancels it, then acts
    Rig r;
    r.measure.ready_after = 1000000;
    r.key("C5");
    r.key("C5", 200);
    CHECK(r.remote.busy());
    CHECK(std::strcmp(r.key("A7"), "posture rest") == 0 && !r.remote.busy() && r.posture_goal() == KILO_POSTURE_REST);
    CHECK(r.log.has("remote C5 (0x90, calibration posture): measure cancelled (a key was pressed): standing"));
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
    CHECK(!r.walker.active() && r.walker.ended_ok() && std::strcmp(r.remote.last(), "posture calib (C5 again: measure himself)") == 0);
    CHECK(r.posture_goal() == KILO_POSTURE_CALIB);
  }
}

void test_gait_keys() {
  Rig r;
  // not walking: a gait key starts walking forward in it at 1x
  LAST_IS(r, "C4", "walk forward (walk) 58.0 mm/s");
  r.run_ms(800);
  CHECK(r.walker.active() && r.dir() == planner::Direction::Forward && r.remote.level() == 3);
  CHECK(std::strcmp(r.walker.planner().pattern().name, "walk") == 0);
  CHECK(r.log.has("remote C4 (0xB0, walk): walk forward (walk) 58.0 mm/s"));
  // the same gait again: nothing (no restart, the speed kept)
  size_t feet = r.body.count(KILO_MSG_FEET);
  LAST_IS(r, "C4", "walk forward (walk) 58.0 mm/s: unchanged");
  CHECK(r.walker.active() && r.body.count(KILO_MSG_FEET) > feet);
  // another gait while walking: changed, the level kept
  r.key("B7");  // +4
  LAST_IS(r, "A4", "walk forward (trot) 180.0 mm/s");  // the trot's 1.25x
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
  // down from 1x through zero into backward (since 2026-11-22 the trot's levels are 0.4, 0.7, 1,
  // 1.25, 1.4 x its default: 58 to 202 mm/s, inside its 42..207, backward to -3 only; the walk's and
  // the crawl's still 0.4, 0.7, 1, 1.15, 1.25)
  LAST_IS(r, "B5", "speed 0.7x (trot 100.8 mm/s forward)");
  LAST_IS(r, "B5", "speed 0.4x (trot 57.6 mm/s forward)");
  LAST_IS(r, "B5", "speed -0.4x (trot 57.6 mm/s back)");
  CHECK(r.remote.level() == -1 && r.dir() == planner::Direction::Back);
  LAST_IS(r, "B5", "speed -0.7x (trot 100.8 mm/s back)");
  LAST_IS(r, "B5", "speed -1x (trot 144.0 mm/s back)");
  LAST_IS(r, "B5", "speed -1x (trot 144.0 mm/s back): already the fastest back");
  CHECK(r.remote.level() == -3 && std::fabs(r.speed() - 144.0) < 0.05);
  // and up again, through zero into forward, to the top
  for (int i = 0; i < 2; i++) r.key("B7");
  CHECK(r.remote.level() == -1);
  LAST_IS(r, "B7", "speed 0.4x (trot 57.6 mm/s forward)");
  CHECK(r.dir() == planner::Direction::Forward);
  for (int i = 0; i < 3; i++) r.key("B7");
  LAST_IS(r, "B7", "speed 1.4x (trot 201.6 mm/s forward)");
  LAST_IS(r, "B7", "speed 1.4x (trot 201.6 mm/s forward): already the fastest");
  CHECK(r.remote.level() == 5);
  r.run_ms(2500);  // the long swing built up a tick a step
  CHECK(r.walker.planner().swing_ticks() == 11);
  LAST_IS(r, "B5", "speed 1.25x (trot 180.0 mm/s forward)");
  LAST_IS(r, "B5", "speed 1x (trot 144.0 mm/s forward)");
  CHECK(r.log.has("remote B5 (0xA8, slower): speed 1.25x (trot 180.0 mm/s forward)"));
  LAST_IS(r, "C4", "walk forward (walk) 58.0 mm/s");
  r.key("B7");  // the walk's levels: its top two are about its top
  LAST_IS(r, "B7", "speed 1.25x (walk 72.5 mm/s forward)");
}

void test_turn_keys() {
  Rig r;
  // from standing: the turn in place in the selected gait at 1x
  LAST_IS(r, "A6", "walk left (trot) 144.0 mm/s");
  r.run_ms(800);
  CHECK(r.walker.active() && r.dir() == planner::Direction::Left);
  LAST_IS(r, "A6", "walk left (trot) 144.0 mm/s: unchanged");
  LAST_IS(r, "C6", "walk right (trot) 144.0 mm/s");  // turning in place: the other key turns the other way
  CHECK(r.dir() == planner::Direction::Right);
  // B7 while turning: straight forward at once, the walk going on (no planned stop)
  LAST_IS(r, "B7", "turn ended: walk forward (trot) 144.0 mm/s");
  CHECK(r.walker.active() && !r.walker.planner().stopping() && r.dir() == planner::Direction::Forward);
  CHECK(r.remote.level() == 3 && r.remote.heading() == Heading::Straight);
  CHECK(r.log.has("remote B7 (0x62, faster): turn ended: walk forward (trot) 144.0 mm/s"));
  r.run_ms(1400);  // the trot's strides 0.75 mm a tick: straight within 1.4 s
  CHECK(r.walker.planner().s_cur()[0] == r.walker.planner().s_cur()[1]);
  LAST_IS(r, "B7", "speed 1.25x (trot 180.0 mm/s forward)");  // then the ladder as ever
  // B5 while turning: straight backward
  CHECK(std::strcmp(r.key_done("B6"), "posture stand") == 0);
  r.key("C6", 800);
  LAST_IS(r, "B5", "turn ended: walk back (trot) 144.0 mm/s");
  CHECK(r.remote.level() == -3 && r.dir() == planner::Direction::Back && r.walker.active());
  // the gait in use during a turn: straight forward; another gait: the turn goes on in it
  CHECK(std::strcmp(r.key_done("B6"), "posture stand") == 0);
  r.key("C6", 800);
  LAST_IS(r, "C4", "walk right (walk) 58.0 mm/s");
  LAST_IS(r, "C4", "walk forward (walk) 58.0 mm/s");
  CHECK(r.remote.heading() == Heading::Straight && r.dir() == planner::Direction::Forward);
}

void test_steering() {
  Rig r;
  r.key("A4", 800);
  CHECK(r.walker.active());
  // A6 while walking forward: steer a level more to the left, without stopping (kSteerLevels)
  LAST_IS(r, "A6", "walk forward (trot) 144.0 mm/s, steering left 1");
  CHECK(r.dir() == planner::Direction::Forward && std::fabs(r.walker.planner().steer() - 0.25) < 1e-6);
  CHECK(r.walker.goal().steer == (planner::real)0.25 && !r.walker.planner().stopping());
  LAST_IS(r, "A6", "walk forward (trot) 144.0 mm/s, steering left 2");
  LAST_IS(r, "A6", "walk forward (trot) 144.0 mm/s, steering left 3");
  LAST_IS(r, "A6", "walk forward (trot) 144.0 mm/s, steering left 3: already the tightest");
  CHECK(r.remote.steer() == 3 && std::fabs(r.walker.planner().steer() - 1.0) < 1e-6);
  r.run_ms(2000);  // the inner (left) side steps in place, the outer at the 8-tick swing's longest: 37.5 mm
  const planner::real* s = r.walker.planner().s_cur();
  CHECKF(std::fabs(s[0]) < 1e-4 && std::fabs(s[1] - 37.5) < 1e-3, "%g %g", (double)s[0], (double)s[1]);
  CHECK(r.log.has("remote A6 (0x22, turn left): walk forward (trot) 144.0 mm/s, steering left 3"));
  // C6 back through straight to the right
  LAST_IS(r, "C6", "walk forward (trot) 144.0 mm/s, steering left 2");
  LAST_IS(r, "C6", "walk forward (trot) 144.0 mm/s, steering left 1");
  LAST_IS(r, "C6", "walk forward (trot) 144.0 mm/s");
  CHECK(r.walker.planner().steer() == 0);
  LAST_IS(r, "C6", "walk forward (trot) 144.0 mm/s, steering right 1");
  // the speed keys keep the bend, into backward too
  LAST_IS(r, "B7", "speed 1.25x (trot 180.0 mm/s forward, steering right 1)");
  for (int i = 0; i < 4; i++) r.key("B5");
  CHECK(r.remote.level() == -1 && r.dir() == planner::Direction::Back);
  CHECK(std::fabs(r.walker.planner().steer() + 0.25) < 1e-6);
  LAST_IS(r, "C6", "walk back (trot) 57.6 mm/s, steering right 2");
  // another gait keeps it; the gait in use straightens it, forward
  LAST_IS(r, "C4", "walk back (walk) 23.2 mm/s, steering right 2");
  LAST_IS(r, "C4", "walk forward (walk) 23.2 mm/s");
  CHECK(r.remote.steer() == 0 && r.walker.planner().steer() == 0);
  // a stop forgets it
  r.key("A6");
  CHECK(std::strcmp(r.key_done("B6"), "posture stand") == 0 && r.remote.steer() == 0);
  LAST_IS(r, "A4", "walk forward (trot) 144.0 mm/s");
}

// The trot's long swing (remote.h): +4 and +5 steer one level at most (a sharper steer eased to it
// as B7 goes up, logged); B5 back to +3 steers as before; backward only to -3.
void test_trot_long_swing() {
  Rig r;
  r.key("A4", 800);
  r.key("A6");
  r.key("A6");
  CHECK(r.remote.steer() == 2);
  LAST_IS(r, "B7", "speed 1.25x (trot 180.0 mm/s forward, steering left 1), the steer eased to 1 (the trot's long swing)");
  CHECK(r.remote.steer() == 1 && std::fabs(r.walker.planner().steer() - 0.25) < 1e-6);
  LAST_IS(r, "A6", "walk forward (trot) 180.0 mm/s, steering left 1: already the tightest at this speed (B5 for more)");
  LAST_IS(r, "B7", "speed 1.4x (trot 201.6 mm/s forward, steering left 1)");
  r.run_ms(3000);
  CHECK(r.walker.planner().swing_ticks() == 11);
  LAST_IS(r, "C6", "walk forward (trot) 201.6 mm/s");
  for (int i = 0; i < 2; i++) r.key("B5");
  CHECK(r.remote.level() == 3);
  LAST_IS(r, "A6", "walk forward (trot) 144.0 mm/s, steering left 1");
  LAST_IS(r, "A6", "walk forward (trot) 144.0 mm/s, steering left 2");
  // from the walk at +5 steering hard, into the trot: the steer eased with the change
  CHECK(std::strcmp(r.key_done("B6"), "posture stand") == 0);
  r.key("C4", 800);
  r.key("B7");
  r.key("B7");
  for (int i = 0; i < 3; i++) r.key("C6");
  CHECK(r.remote.level() == 5 && r.remote.steer() == -3);
  LAST_IS(r, "A4", "walk forward (trot) 201.6 mm/s, steering right 1");
  CHECK(r.remote.steer() == -1);
  // B6: the quick stop, then stand
  CHECK(std::strcmp(r.key_done("B6"), "posture stand") == 0 && !r.walker.active() && r.walker.ended_ok());
  CHECK(r.remote.level() == 3);
}

void test_freeze() {
  {  // walking: the quick stop (the feet down where they are, squared up, STANCE), then only B6 / A7 act
    Rig r;
    r.key("A4", 800);
    CHECK(r.walker.active() && r.body.mode == KILO_MODE_GAIT);
    LAST_IS(r, "C7", "frozen (walk: the quick stop): only B6 or A7 act now");
    CHECK(r.walker.active() && r.walker.planner().halting() && r.remote.frozen());
    r.run_ms(2000);
    CHECKF(!r.walker.active() && r.walker.ended_ok() && std::strcmp(r.walker.ended(), "stopped") == 0, "ended %s",
           r.walker.ended() ? r.walker.ended() : "-");
    CHECK(r.remote.frozen() && r.posture_goal() == -1 && r.body.count(KILO_MSG_FEET) > 0);
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
  {  // ahead of a stop under way: that key is cancelled, the ones behind it dropped
    Rig r;
    r.key("A4", 800);
    r.key("B6", 40);
    r.key("A7", 8);
    CHECK(r.remote.busy() && r.walker.active());
    LAST_IS(r, "C7", "frozen (walk: the quick stop): only B6 or A7 act now");
    CHECK(!r.remote.busy() && r.walker.planner().halting());
    r.run_ms(2000);
    CHECK(!r.walker.active() && r.posture_goal() == -1);
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
  // A5 again: one of the random mode's tricks, each press another; the runner is not started again
  LAST_IS(r, "A5", "random mode: pee now");
  LAST_IS(r, "A5", "random mode: pee now");
  CHECK(r.runner.starts == 1 && r.runner.tricks == 2 && r.runner.on && r.runner.stops == 0);
  r.runner.refuse_trick = true;  // refused (one waits already): a failed key, the runner left alone
  LAST_IS(r, "A5", "random mode: pee is already waiting");
  CHECK(r.remote.last_failed() && r.runner.on && r.runner.tricks == 2);
  r.runner.refuse_trick = false;
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

void test_rear_up_key_off() {
  // the default until the owner's staged trials: A1 is a row 1-3 key
  CHECK(!rear_up_key());
  Rig r;
  const Key* k = action_for(cmd_of("A1"));
  CHECK(k != nullptr && k->kind == Kind::Ignore);
  r.remote.perform(*k, k->command);
  r.run_ms(40);
  CHECK(r.body.count(KILO_MSG_TRICK_BEGIN) == 0 && r.body.count(KILO_MSG_TRICK_PLAY) == 0 && !r.player.busy());
}

void test_rear_up_key() {
  set_rear_up_key(true);
  const tricks::Trick* t = tricks::find_trick("rear_up");
  CHECK(t != nullptr && t->stop_ticks == 75 && t->legacy[0] == '\0' && tricks::find_trick("") == nullptr);
  {
    Rig r;
    LAST_IS(r, "A1", "trick rear_up started (6 s; A1 again, B6 or C7 bring it down)");
    CHECK(r.body.count(KILO_MSG_TRICK_PLAY) == 1 && r.player.busy());
    CHECK(r.log.has("remote A1 (0x42, rear up): trick rear_up started"));
    r.body.mode = KILO_MODE_TRICK;
    r.run_ms(40);
    // pressed again while it plays: down, over the trick's stop_ticks
    LAST_IS(r, "A1", "trick stopped: coming down");
    CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 1 && stop_ticks_sent(r) == 75);
  }
  {  // B6 while it plays: TRICK_STOP over 75 ticks, the crossfade (to SIT) waited for, then STAND
    Rig r;
    r.key("A1");
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
  {  // C7 while it plays: stopped the same way (it comes down), frozen; A1 does not unfreeze
    Rig r;
    r.key("A1");
    r.body.mode = KILO_MODE_TRICK;
    r.run_ms(40);
    LAST_IS(r, "C7", "frozen (trick stopped): only B6 or A7 act now");
    CHECK(stop_ticks_sent(r) == 75);
    LAST_IS(r, "A1", "ignored (frozen: B6 or A7 to go on)");
  }
  {  // walking: the quick stop first, then the trick
    Rig r;
    r.key("A4", 800);
    CHECK(r.walker.active());
    CHECK(std::strncmp(r.key_done("A1"), "trick rear_up started", 21) == 0 && !r.walker.active());
    r.run_ms(100);  // the player's TRICK_COMMIT, then TRICK_PLAY
    CHECKF(r.body.count(KILO_MSG_TRICK_PLAY) == 1 && r.remote.level() == 3, "plays %zu level %d",
           r.body.count(KILO_MSG_TRICK_PLAY), r.remote.level());
  }
  {  // a flat pack, USB power: refused, nothing moves
    Rig r;
    r.body.batt_mv = 7300;
    r.run_ms(40);
    LAST_IS(r, "A1", "rear_up refused: battery 7300 mV (needs 7400)");
    r.body.batt_mv = 0;
    r.body.flags |= KILO_TELEMETRY_FLAGS_NO_BATTERY;
    r.run_ms(40);
    LAST_IS(r, "A1", "rear_up refused: on USB power (no battery)");
    CHECK(r.body.count(KILO_MSG_TRICK_PLAY) == 0 && r.body.count(KILO_MSG_TRICK_COMMIT) == 0);
  }
  // the higher stages (KILO_REAR_UP=2, 3): the same key, the stage's trick, its own slower stop
  for (uint8_t stage = 2; stage <= kRearUpStages; stage++) {
    set_rear_up_key(stage);
    const char* name = stage == 2 ? "rear_up_2" : "rear_up_3";
    const uint8_t stop = stage == 2 ? 100 : 125;
    const tricks::Trick* ts = tricks::find_trick(name);
    CHECKF(ts != nullptr && ts->stop_ticks == stop, "%s", name);
    Rig r;
    char want[40];
    std::snprintf(want, sizeof want, "trick %s started", name);
    CHECKF(std::strncmp(r.key_done("A1"), want, std::strlen(want)) == 0, "%s", r.remote.last());
    std::snprintf(want, sizeof want, "remote A1 (0x42, rear up %u): %s", (unsigned)stage, "trick");
    CHECKF(r.log.has(want), "%s", want);
    r.body.mode = KILO_MODE_TRICK;
    r.run_ms(40);
    LAST_IS(r, "A1", "trick stopped: coming down");
    CHECK(stop_ticks_sent(r) == stop);
  }
  set_rear_up_key(0);
}

void test_keys_queue() {
  Rig r;
  r.key("A4", 800);
  r.key("B6", 8);  // the quick stop takes a moment: the keys behind it queue
  r.key("C4", 8);
  r.key("B7", 8);
  CHECK(r.remote.busy());
  for (int i = 0; i < 3000 && r.remote.busy(); i++) r.pass();
  CHECK(r.log.has("remote B6 (0x02, stand): posture stand") &&
        r.log.has("remote C4 (0xB0, walk): walk forward (walk) 58.0 mm/s") &&
        std::strcmp(r.remote.last(), "speed 1.15x (walk 66.7 mm/s forward)") == 0);  // the walk's levels
}

}  // namespace

int main() {
  for (uint8_t stage = 0; stage <= kRearUpStages; stage++) test_key_table(stage);
  set_rear_up_key(4);  // past the last stage: off
  CHECK(rear_up_key() == 0);
  test_posture_keys();
  test_gait_keys();
  test_speed_ladder();
  test_turn_keys();
  test_steering();
  test_trot_long_swing();
  test_freeze();
  test_behaviour_key();
  test_rear_up_key_off();
  test_rear_up_key();
  test_keys_queue();
  return report("remote");
}
