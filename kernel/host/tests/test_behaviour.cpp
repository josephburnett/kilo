// The behaviour layer: the engine, drives and selector cases of
// mind/tests/test_behaviour.py by hand, then behaviour_vectors.txt
// (gen_behaviour_vectors.py): the Python runner's ticks over scripted
// scenarios, which the C++ runner must repeat choice for choice, path for
// path, with the same drives, scores and calls on the body, from the same
// seed (the RNG is Python's Mersenne Twister, checked first).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <vector>
#include "behaviour/runner.h"
#include "proto/kilo_protocol.h"
#include "tests/check.h"
#include "tests/fakes.h"

using namespace kilo;
using namespace kilo::behaviour;

namespace {

const Status RUNNING = Status::Running, SUCCESS = Status::Success, FAILURE = Status::Failure;

struct Clock {
  double t = 100.0;
  static double read(void* arg) { return static_cast<Clock*>(arg)->t; }
};

class Probe : public Node {
 public:
  explicit Probe(const char* name, Status st = Status::Running) : Node(name), result(st) {}
  Status result;
  int ticks = 0, halts = 0;
  void halt() override {
    halts++;
    Node::halt();
  }

 protected:
  Status do_tick(Context& ctx) override {
    ticks++;
    ctx.leaf(result);
    return result;
  }
};

class Flag : public Condition {
 public:
  Flag(const char* name, bool* on) : Condition(name), on_(on) {}

 protected:
  bool test(Context&) override { return *on_; }

 private:
  bool* on_;
};

class Counter : public Action {
 public:
  explicit Counter(const char* name) : Action(name) {}
  int calls = 0;

 protected:
  Status run(Context&) override {
    calls++;
    return Status::Success;
  }
};

struct Ctx {
  Clock clock;
  PyRandom rng{1};
  Context ctx;
  Ctx() {
    ctx.clock = Clock::read;
    ctx.clock_arg = &clock;
    ctx.rng = &rng;
  }
};

bool path_is(const Context& c, std::vector<std::string> want) {
  if ((int)want.size() != c.path_n) return false;
  for (int i = 0; i < c.path_n; i++)
    if (want[i] != c.path[i]) return false;
  return true;
}

// ---- RNG, engine ---------------------------------------------------------------------------

void test_selector_rechecks_and_preempts() {
  Ctx c;
  bool flag = false;
  Probe high("high"), low("low");
  Flag guard("flag", &flag);
  Node* gc[2] = {&guard, &high};
  Sequence guarded("guarded", gc, 2, false);
  Node* sc[2] = {&guarded, &low};
  Selector root("sel", sc, 2);
  CHECK(tick_root(root, c.ctx) == RUNNING && path_is(c.ctx, {"sel", "low"}));
  CHECK(tick_root(root, c.ctx) == RUNNING && low.ticks == 2 && high.ticks == 0);
  flag = true;
  CHECK(tick_root(root, c.ctx) == RUNNING);
  CHECK(path_is(c.ctx, {"sel", "guarded", "high"}) && low.halts == 1 && high.ticks == 1);
  flag = false;
  CHECK(tick_root(root, c.ctx) == RUNNING && path_is(c.ctx, {"sel", "low"}));
  CHECK(high.halts == 1 && guarded.index() == 0);
  flag = true;
  Probe act("act");
  Flag g2("flag", &flag);
  Node* mc[2] = {&g2, &act};
  Sequence mem("mem", mc, 2);
  CHECK(tick_root(mem, c.ctx) == RUNNING);
  flag = false;
  CHECK(tick_root(mem, c.ctx) == RUNNING && path_is(c.ctx, {"mem", "act"}));
  // all fail; the first success wins
  Probe a("a", FAILURE), b("b", FAILURE), b2("b", SUCCESS), cc("c");
  Node* f[2] = {&a, &b};
  Selector s1("s", f, 2);
  CHECK(tick_root(s1, c.ctx) == FAILURE);
  Node* f2[3] = {&a, &b2, &cc};
  Selector s2("s", f2, 3);
  CHECK(tick_root(s2, c.ctx) == SUCCESS && cc.ticks == 0 && path_is(c.ctx, {"s", "b"}));
}

void test_sequence_memory_parallel_decorators() {
  Ctx c;
  Counter once("once");
  Wait w(1.0, "Wait(1s)");
  Probe end("end", SUCCESS);
  Node* ch[3] = {&once, &w, &end};
  Sequence seq("seq", ch, 3);
  CHECK(tick_root(seq, c.ctx) == RUNNING);
  c.clock.t += 0.5;
  CHECK(tick_root(seq, c.ctx) == RUNNING && once.calls == 1 && path_is(c.ctx, {"seq", "Wait(1s)"}));
  c.clock.t += 0.5;
  CHECK(tick_root(seq, c.ctx) == SUCCESS && once.calls == 1);
  CHECK(tick_root(seq, c.ctx) == RUNNING && once.calls == 2);
  // parallel
  Probe pa("a", SUCCESS), pb("b");
  Node* pc[2] = {&pa, &pb};
  Parallel all("all", pc, 2);
  CHECK(tick_root(all, c.ctx) == RUNNING);
  CHECK(tick_root(all, c.ctx) == RUNNING && pa.ticks == 1 && pb.ticks == 2);
  pb.result = SUCCESS;
  CHECK(tick_root(all, c.ctx) == SUCCESS);
  Probe e("e"), f("f", SUCCESS);
  Node* ac[2] = {&e, &f};
  Parallel any("any", ac, 2, Parallel::Policy::Any);
  CHECK(tick_root(any, c.ctx) == SUCCESS && e.halts == 1);
  // active path: running wins over a later finished leaf
  Probe sa("a", SUCCESS), sb("b"), pc2("c", SUCCESS);
  Node* s1c[2] = {&sa, &sb};
  Sequence s1("s1", s1c, 2);
  Node* rc[2] = {&s1, &pc2};
  Parallel root("par", rc, 2);
  tick_root(root, c.ctx);
  CHECK(path_is(c.ctx, {"par", "s1", "b"}));
  // cooldown
  Probe p("p", SUCCESS);
  Cooldown cd(5.0, &p, "Cooldown(5s)");
  CHECK(tick_root(cd, c.ctx) == SUCCESS);
  c.clock.t += 4.9;
  CHECK(tick_root(cd, c.ctx) == FAILURE && p.ticks == 1);
  c.clock.t += 0.1;
  CHECK(tick_root(cd, c.ctx) == SUCCESS && p.ticks == 2);
  p.result = RUNNING;
  c.clock.t += 10;
  CHECK(tick_root(cd, c.ctx) == RUNNING && tick_root(cd, c.ctx) == RUNNING);
  p.result = FAILURE;
  CHECK(tick_root(cd, c.ctx) == FAILURE);
  p.result = SUCCESS;
  c.clock.t += 1;
  CHECK(tick_root(cd, c.ctx) == FAILURE && p.ticks == 5);
  c.clock.t += 10;
  p.result = RUNNING;
  CHECK(tick_root(cd, c.ctx) == RUNNING);
  cd.halt();
  CHECK(p.halts >= 1 && tick_root(cd, c.ctx) == FAILURE);
  // timeout
  Probe tp("p");
  Timeout to(2.0, &tp, "Timeout(2s)");
  CHECK(tick_root(to, c.ctx) == RUNNING);
  c.clock.t += 1.9;
  CHECK(tick_root(to, c.ctx) == RUNNING);
  c.clock.t += 0.1;
  CHECK(tick_root(to, c.ctx) == FAILURE && tp.halts == 1);
  CHECK(tick_root(to, c.ctx) == RUNNING);
  tp.result = SUCCESS;
  c.clock.t += 1.0;
  CHECK(tick_root(to, c.ctx) == SUCCESS);
  // inverter, force success, repeat
  Probe ok("a", SUCCESS), bad("a", FAILURE), run("a");
  Inverter i1(&ok), i2(&bad), i3(&run);
  CHECK(tick_root(i1, c.ctx) == FAILURE && tick_root(i2, c.ctx) == SUCCESS && tick_root(i3, c.ctx) == RUNNING);
  ForceSuccess fs(&bad);
  CHECK(tick_root(fs, c.ctx) == SUCCESS);
  Probe rp("p", SUCCESS);
  Repeat r(&rp, 3, "Repeat(3)");
  Status got[4];
  for (int k = 0; k < 4; k++) got[k] = tick_root(r, c.ctx);
  CHECK(got[0] == RUNNING && got[1] == RUNNING && got[2] == SUCCESS && got[3] == RUNNING);
  Repeat rf(&bad, 0, "Forever");
  CHECK(tick_root(rf, c.ctx) == FAILURE);
  // chance: seeded, about p, sticky while its child runs
  auto chance_run = [](uint64_t seed) {
    Ctx k;
    k.rng.seed_with(seed);
    Probe cp("p", SUCCESS);
    Chance ch(0.3, &cp, "Chance(0.3)");
    std::vector<bool> out;
    for (int n = 0; n < 2000; n++) out.push_back(tick_root(ch, k.ctx) == SUCCESS);
    return out;
  };
  std::vector<bool> a = chance_run(7);
  int hits = 0;
  for (bool x : a) hits += x;
  CHECK(a == chance_run(7) && a != chance_run(8) && hits > 500 && hits < 700);
  char g[16];
  format_g(g, sizeof g, 6.0);
  CHECK(std::strcmp(g, "6") == 0);
  format_g(g, sizeof g, 1.5);
  CHECK(std::strcmp(g, "1.5") == 0);
  format_g(g, sizeof g, 30.0);
  CHECK(std::strcmp(g, "30") == 0);
  format_g(g, sizeof g, 0.05);
  CHECK(std::strcmp(g, "0.05") == 0);
  format_g(g, sizeof g, 12.0);
  CHECK(std::strcmp(g, "12") == 0);
}

// ---- drives -----------------------------------------------------------------------------

void run_drives(Drives& d, double seconds, const DriveInputs& in, const Personality& cfg = kPersonality) {
  for (int i = 0; i < (int)(seconds / 0.1 + 0.5); i++) d.update(0.1, in, cfg, nullptr);
}

void test_drives() {
  Drives d;
  d.energy = d.curiosity = d.sociability = d.arousal = 0.5;
  DriveInputs in;
  in.activity = Activity::Explore;
  run_drives(d, 10, in);
  CHECK(d.energy < 0.5 && d.curiosity < 0.5 && d.sociability > 0.5 && d.arousal < 0.5);
  Drives d2;
  d2.energy = d2.curiosity = d2.sociability = 0.5;
  d2.arousal = 0;
  in = DriveInputs();
  in.activity = Activity::Rest;
  in.social = true;
  run_drives(d2, 10, in);
  CHECK(d2.energy > 0.5 && d2.curiosity > 0.5 && d2.sociability < 0.5);
  Drives d4;
  d4.energy = 0.5;
  in = DriveInputs();
  in.activity = Activity::Rest;
  in.moving = true;
  run_drives(d4, 10, in);
  CHECK(d4.energy < 0.5);
  // energy tied to the battery
  Drives b;
  b.energy = 1.0;
  in = DriveInputs();
  in.activity = Activity::Rest;
  in.has_battery = true;
  in.battery_pct = 35.0;
  b.update(0.1, in, kPersonality, nullptr);
  CHECK(std::fabs(b.energy - 0.5) < 1e-9);
  in.battery_pct = 5.0;
  b.update(0.1, in, kPersonality, nullptr);
  CHECK(b.energy == 0.0);
  // arousal: spike, half-life
  Drives a;
  a.arousal = 0;
  in = DriveInputs();
  in.startle = 1.0;
  a.update(0.1, in, kPersonality, nullptr);
  CHECK(std::fabs(a.arousal - 0.9) < 1e-12);
  run_drives(a, 3.0, DriveInputs());
  CHECK(std::fabs(a.arousal - 0.45) < 0.0045);
  in = DriveInputs();
  in.approaching = true;
  a.update(0.1, in, kPersonality, nullptr);
  CHECK(std::fabs(a.arousal - 0.6) < 1e-12);
  // bounds with noise
  Personality noisy = kPersonality;
  noisy.drive_noise = 0.2;
  PyRandom rng(3);
  Drives n;
  bool inside = true;
  const Activity acts[4] = {Activity::Rest, Activity::Explore, Activity::Social, Activity::Idle};
  for (int i = 0; i < 20000; i++) {
    in = DriveInputs();
    in.activity = acts[i / 3000 % 4];
    in.social = i % 7 == 0;
    in.startle = i % 500 == 0 ? 1.0 : 0.0;
    n.update(0.1, in, noisy, &rng);
    for (double v : {n.energy, n.curiosity, n.sociability, n.arousal}) inside = inside && v >= 0.0 && v <= 1.0;
  }
  CHECK(inside);
}

// ---- the selector ----------------------------------------------------------------------------

struct SelRig {
  Personality cfg = kPersonality;
  Wait waits[kBehaviours] = {{1.0, "idle_hold"}, {1.0, "explore_hold"}, {1.0, "nap_hold"}, {1.0, "greet_hold"},
                             {1.0, "startle_hold"}};
  Repeat trees[kBehaviours] = {{&waits[0], 0, "idle_tree"}, {&waits[1], 0, "explore_tree"},
                               {&waits[2], 0, "nap_tree"}, {&waits[3], 0, "greet_tree"},
                               {&waits[4], 0, "startle_tree"}};
  BehaviourEntry entries[kBehaviours];
  Ctx c;
  // sel_cfg(): idle base 0.4, explore curiosity, nap fatigue, greet face, startle arousal
  SelRig(double noise, double period, double margin, double dwell) {
    cfg.n_weights[kIdle] = 1;
    cfg.weights[kIdle][0] = {1, {kBase, 0, 0}, 0.4};
    cfg.n_weights[kExplore] = 1;
    cfg.weights[kExplore][0] = {1, {kCuriosity, 0, 0}, 1.0};
    cfg.n_weights[kNap] = 1;
    cfg.weights[kNap][0] = {1, {kFatigue, 0, 0}, 1.0};
    cfg.n_weights[kGreet] = 1;
    cfg.weights[kGreet][0] = {1, {kFace, 0, 0}, 1.0};
    cfg.n_weights[kStartle] = 1;
    cfg.weights[kStartle][0] = {1, {kArousal, 0, 0}, 1.0};
    cfg.sel_noise = noise;
    cfg.sel_noise_period_s = period;
    cfg.sel_margin = margin;
    cfg.sel_min_dwell_s = dwell;
    const Activity act[kBehaviours] = {Activity::Idle, Activity::Explore, Activity::Rest, Activity::Social,
                                       Activity::Active};
    for (int i = 0; i < kBehaviours; i++) entries[i] = {behaviour_name((uint8_t)i), &trees[i], act[i], 0.0};
  }
};

void test_selector() {
  {  // test_selector_no_flicker_under_noise
    SelRig r(0.04, 0.5, 0.1, 1.0);
    UtilitySelector sel(r.entries, nullptr, 0, r.cfg);
    r.c.ctx.drives = Drives();
    r.c.ctx.drives.energy = 1.0;
    r.c.ctx.drives.curiosity = 0.4;
    r.c.ctx.drives.sociability = 0.0;
    r.c.ctx.drives.arousal = 0.0;
    bool only = true;
    for (int i = 0; i < 6000; i++) {
      tick_root(sel, r.c.ctx);
      only = only && (sel.last().chosen == kIdle || sel.last().chosen == kExplore);
      r.c.clock.t += 0.1;
    }
    CHECK(sel.switches() == 1 && only);
    SelRig r0(0.04, 0.5, 0.0, 0.0);
    UtilitySelector sel0(r0.entries, nullptr, 0, r0.cfg);
    r0.c.ctx.drives = Drives();
    r0.c.ctx.drives.energy = 1.0;
    r0.c.ctx.drives.curiosity = 0.4;
    for (int i = 0; i < 6000; i++) {
      tick_root(sel0, r0.c.ctx);
      r0.c.clock.t += 0.1;
    }
    CHECKF(sel0.switches() > 50, "%u switches", sel0.switches());
  }
  {  // test_selector_switches_past_margin_after_dwell
    SelRig r(0.0, 8.0, 0.1, 5.0);
    r.cfg.urgent = 1u << kStartle;
    UtilitySelector sel(r.entries, nullptr, 0, r.cfg);
    Drives& d = r.c.ctx.drives;
    d = Drives();
    d.energy = 1.0;
    d.curiosity = 0.0;
    tick_root(sel, r.c.ctx);
    CHECK(sel.last().chosen == kIdle && r.c.ctx.path_n >= 3 && std::strcmp(r.c.ctx.path[2], "idle_tree") == 0);
    r.c.clock.t += 10;
    d.curiosity = 0.49;
    tick_root(sel, r.c.ctx);
    CHECK(sel.last().chosen == kIdle && std::strcmp(sel.last().reason, "margin") == 0);
    d.curiosity = 0.51;
    tick_root(sel, r.c.ctx);
    CHECK(sel.last().chosen == kExplore && std::strcmp(sel.last().reason, "switch") == 0);
    CHECK(std::fabs(sel.last().scores[kExplore] - 0.51) < 1e-12);
    r.c.clock.t += 1;
    d.energy = 0.3;
    tick_root(sel, r.c.ctx);
    CHECK(sel.last().chosen == kExplore && std::strcmp(sel.last().reason, "dwell") == 0);
    r.c.clock.t += 4;
    tick_root(sel, r.c.ctx);
    CHECK(sel.last().chosen == kNap);
    r.c.clock.t += 0.1;
    d.arousal = 0.9;
    tick_root(sel, r.c.ctx);
    CHECK(sel.last().chosen == kStartle && std::strcmp(sel.last().reason, "urgent") == 0);
  }
  {  // test_selector_product_terms
    SelRig r(0.0, 8.0, 0.1, 6.0);
    r.cfg.weights[kGreet][0] = {2, {kSociability, kFace, 0}, 1.0};
    UtilitySelector sel(r.entries, nullptr, 0, r.cfg);
    r.c.ctx.drives = Drives();
    r.c.ctx.drives.sociability = 0.8;
    tick_root(sel, r.c.ctx);
    CHECK(sel.last().scores[kGreet] == 0.0);
    r.c.ctx.percepts.face = true;
    tick_root(sel, r.c.ctx);
    CHECK(std::fabs(sel.last().scores[kGreet] - 0.8) < 1e-12);
  }
  {  // test_selector_falls_back_when_choice_fails
    SelRig r(0.0, 8.0, 0.1, 0.0);
    Probe fails("greet_fails", FAILURE);
    r.entries[kGreet].tree = &fails;
    UtilitySelector sel(r.entries, nullptr, 0, r.cfg);
    r.c.ctx.drives = Drives();
    r.c.ctx.drives.energy = 1.0;
    r.c.ctx.drives.curiosity = 0.0;
    r.c.ctx.percepts.face = true;
    tick_root(sel, r.c.ctx);
    CHECK(sel.last().chosen == kIdle && std::strncmp(sel.last().reason, "fallback", 8) == 0);
  }
  {  // test_safety_preempts_and_releases
    SelRig r(0.0, 8.0, 0.1, 60.0);
    static bool fallen = false;
    Probe probe("wait_recover");
    SafetyEntry safety[1] = {{"fallen", [](Context&, void*) { return fallen; }, nullptr, &probe, Activity::Rest}};
    UtilitySelector sel(r.entries, safety, 1, r.cfg);
    r.c.ctx.drives = Drives();
    r.c.ctx.drives.energy = 1.0;
    r.c.ctx.drives.curiosity = 0.0;
    tick_root(sel, r.c.ctx);
    CHECK(sel.last().chosen == kIdle);
    fallen = true;
    tick_root(sel, r.c.ctx);
    CHECK(std::strcmp(sel.last().name, "fallen") == 0 && std::strcmp(sel.last().reason, "safety") == 0);
    CHECK(path_is(r.c.ctx, {"root", "fallen", "wait_recover"}) && !r.waits[kIdle].waiting());
    fallen = false;
    r.c.clock.t += 0.1;
    tick_root(sel, r.c.ctx);
    CHECK(sel.last().chosen == kIdle && std::strcmp(sel.last().reason, "free") == 0);
  }
  {  // test_build_root_names
    Library lib(kPersonality);
    CHECK(lib.root().switches() == 0);
  }
}

// ---- the golden runs -------------------------------------------------------------------------

std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out;
  size_t i = 0;
  while (i < s.size()) {
    while (i < s.size() && s[i] == ' ') i++;
    size_t j = i;
    while (j < s.size() && s[j] != ' ') j++;
    if (j > i) out.push_back(s.substr(i, j - i));
    i = j;
  }
  return out;
}

double num(const std::string& s) { return std::strtod(s.c_str(), nullptr); }

struct GoldBody : BodyIface {
  Snapshot snap;
  uint32_t ticks = 0;
  std::vector<std::vector<std::string>> calls;
  uint32_t handle = 0;
  uint32_t play_start = 0;
  char buf[2][40];

  void snapshot(Snapshot& out) override { out = snap; }
  void on_tick() override { ticks++; }
  bool posture(const char* name, int8_t balance) override {
    calls.push_back({"posture", name, balance < 0 ? "-" : std::to_string(balance)});
    return true;
  }
  bool set_balance(bool enable) override {
    calls.push_back({"set_balance", enable ? "1" : "0"});
    return true;
  }
  bool look(double pan, double tilt) override {
    std::snprintf(buf[0], sizeof buf[0], "%.17g", pan);
    std::snprintf(buf[1], sizeof buf[1], "%.17g", tilt);
    calls.push_back({"look", buf[0], buf[1]});
    return true;
  }
  uint32_t play(const char* trick) override {
    calls.push_back({"play", trick});
    play_start = ticks;
    return ++handle;
  }
  // the generator's coroutine sees two more body ticks, so the action succeeds on the third
  Status play_status(uint32_t) override { return ticks >= play_start + 3 ? Status::Success : Status::Running; }
  void play_cancel(uint32_t) override {}
  void beep(const uint16_t (*)[2], int n) override { calls.push_back({"beep", std::to_string(n)}); }
};

struct GoldSource : PerceptSource {
  bool face = false, pet = false, approaching = false;
  bool has_face = false, has_pet = false, has_approaching = false, has_face_pan = false, has_startle = false;
  double face_pan = 0, startle = 0;
  void provide(BehaviourPercepts& p, double* s) override {
    if (has_face) p.face = face;
    if (has_pet) p.pet = pet;
    if (has_approaching) p.approaching = approaching;
    if (has_face_pan) {
      p.has_face_pan = true;
      p.face_pan = face_pan;
    }
    *s = has_startle ? startle : 0.0;
  }
};

bool paused_fn(void* arg) { return *static_cast<bool*>(arg); }

struct Golden {
  std::deque<std::string> strings;  // the personality's strings live here (a deque: they never move)
  Personality cfg = {};
  std::string name;
  double t0 = 0, t = 0;
  uint64_t seed = 0;
  std::unique_ptr<GoldBody> body;
  std::unique_ptr<GoldSource> source;
  std::unique_ptr<Runner> runner;
  CaptureLog log;
  bool paused = false;
  bool source_on = false;
  // this tick
  bool in_tick = false;
  int k = -1;
  size_t call_i = 0;
  std::vector<std::string> want_key;  // the expected choice/path (k line), carried over ticks
  int ticks = 0, k_lines = 0, d_lines = 0, calls = 0, scenarios = 0;
  bool stopped = false;
  double max_drive_diff = 0;

  const char* keep(const std::string& s) {
    strings.push_back(s);
    return strings.back().c_str();
  }

  std::vector<std::string> actual_key() {
    const TickRecord& rec = runner->record();
    if (rec.paused) return {"paused"};
    const Choice& c = *rec.choice;
    std::vector<std::string> out = {c.name ? c.name : "-", c.reason, c.has_status ? status_name(c.status) : "-"};
    for (int i = 0; i < rec.path_n; i++) out.push_back(rec.path[i]);
    return out;
  }

  void end_tick() {
    if (!in_tick) return;
    in_tick = false;
    CHECKF(call_i == body->calls.size(), "%s tick %d: %zu calls not expected (first %s)", name.c_str(), k,
           body->calls.size() - call_i, call_i < body->calls.size() ? body->calls[call_i][0].c_str() : "");
    std::vector<std::string> got = actual_key();
    if (got != want_key) {
      std::string a, b;
      for (auto& s : got) a += s + " ";
      for (auto& s : want_key) b += s + " ";
      CHECKF(false, "%s tick %d: got [%s] want [%s]", name.c_str(), k, a.c_str(), b.c_str());
    }
  }

  void start(const std::vector<std::string>& w) {
    end_tick();
    runner.reset();
    body.reset(new GoldBody());
    source.reset(new GoldSource());
    cfg = Personality();
    name = w[1];
    seed = std::strtoull(w[2].c_str(), nullptr, 10);
    t0 = num(w[3]);
    paused = false;
    source_on = false;
    want_key.clear();
    stopped = false;
    scenarios++;
    // the FakeBody's initial snapshot
    Snapshot& s = body->snap;
    s = Snapshot();
    s.has_epoch = true;
    s.epoch = 1;
    s.mode = KILO_MODE_IDLE;
    s.has_battery = true;
    s.battery_pct = 90.0;
  }

  void set(const std::string& key, const std::string& v) {
    static const struct {
      const char* key;
      double Personality::*field;
    } kDoubles[] = {
        {"drives.initial.energy", &Personality::initial_energy},
        {"drives.initial.curiosity", &Personality::initial_curiosity},
        {"drives.initial.sociability", &Personality::initial_sociability},
        {"drives.initial.arousal", &Personality::initial_arousal},
        {"drives.energy.drain_active", &Personality::drain_active},
        {"drives.energy.drain_idle", &Personality::drain_idle},
        {"drives.energy.recover_rest", &Personality::recover_rest},
        {"drives.energy.battery_full_pct", &Personality::battery_full_pct},
        {"drives.energy.battery_empty_pct", &Personality::battery_empty_pct},
        {"drives.curiosity.growth", &Personality::curiosity_growth},
        {"drives.curiosity.satisfy", &Personality::curiosity_satisfy},
        {"drives.sociability.growth", &Personality::sociability_growth},
        {"drives.sociability.satisfy", &Personality::sociability_satisfy},
        {"drives.arousal.startle", &Personality::arousal_startle},
        {"drives.arousal.approach", &Personality::arousal_approach},
        {"drives.arousal.half_life_s", &Personality::arousal_half_life_s},
        {"drives.noise", &Personality::drive_noise},
        {"selector.noise", &Personality::sel_noise},
        {"selector.noise_period_s", &Personality::sel_noise_period_s},
        {"selector.margin", &Personality::sel_margin},
        {"selector.min_dwell_s", &Personality::sel_min_dwell_s},
        {"safety.low_battery_pct", &Personality::low_battery_pct},
        {"safety.critical_battery_pct", &Personality::critical_battery_pct},
        {"safety.wind_down_s", &Personality::wind_down_s},
        {"safety.fallen_settle_s", &Personality::fallen_settle_s},
        {"percepts.near_mm", &Personality::near_mm},
        {"percepts.approach_mm_s", &Personality::approach_mm_s},
        {"behaviours.idle.look_every_s", &Personality::idle_look_every_s},
        {"behaviours.idle.look_chance", &Personality::idle_look_chance},
        {"behaviours.idle.look_pan_deg", &Personality::idle_look_pan_deg},
        {"behaviours.idle.look_tilt_deg", &Personality::idle_look_tilt_deg},
        {"behaviours.explore.look_every_s", &Personality::explore_look_every_s},
        {"behaviours.explore.look_pan_deg", &Personality::explore_look_pan_deg},
        {"behaviours.explore.look_tilt_deg", &Personality::explore_look_tilt_deg},
        {"behaviours.nap.min_s", &Personality::nap_min_s},
        {"behaviours.greet.cooldown_s", &Personality::greet_cooldown_s},
        {"behaviours.startle.hold_s", &Personality::startle_hold_s},
    };
    for (const auto& d : kDoubles)
      if (key == d.key) {
        cfg.*(d.field) = num(v);
        return;
      }
    if (key == "name") cfg.name = keep(v);
    else if (key == "behaviours.idle.posture") cfg.idle_posture = keep(v);
    else if (key == "behaviours.explore.posture") cfg.explore_posture = keep(v);
    else if (key == "behaviours.nap.posture") cfg.nap_posture = keep(v);
    else if (key == "behaviours.greet.trick") cfg.greet_trick = keep(v);
    else if (key == "behaviours.explore.looks") cfg.explore_looks = (uint32_t)num(v);
    else if (key == "selector.urgent") {
      cfg.urgent = 0;
      for (int b = 0; b < kBehaviours; b++)
        if (("," + v + ",").find("," + std::string(behaviour_name((uint8_t)b)) + ",") != std::string::npos)
          cfg.urgent |= (uint8_t)(1u << b);
    } else CHECKF(false, "unknown personality key %s", key.c_str());
  }

  void weight(const std::vector<std::string>& w) {
    int b = 0;
    while (b < kBehaviours && w[1] != behaviour_name((uint8_t)b)) b++;
    Weight x = {(uint8_t)std::atoi(w[2].c_str()),
                {(uint8_t)std::atoi(w[3].c_str()), (uint8_t)std::atoi(w[4].c_str()), (uint8_t)std::atoi(w[5].c_str())},
                num(w[6])};
    cfg.weights[b][cfg.n_weights[b]++] = x;
  }

  void tick(int kk) {
    end_tick();
    if (!runner) {
      runner.reset(new Runner(*body, cfg, log, seed));
      runner->set_clock(Clock::read, &clock_);
      runner->set_paused(paused_fn, &paused);
    }
    if (source_on) runner->set_percept_source(source.get());
    k = kk;
    clock_.t = t0 + kk * 0.1;
    body->calls.clear();
    call_i = 0;
    runner->step();
    ticks++;
    in_tick = true;
  }

  void call(const std::vector<std::string>& w) {
    calls++;
    if (call_i >= body->calls.size()) {
      CHECKF(false, "%s tick %d: missing call %s", name.c_str(), k, w[1].c_str());
      return;
    }
    const std::vector<std::string>& got = body->calls[call_i++];
    bool same = got.size() == w.size() - 1;
    for (size_t i = 0; same && i < got.size(); i++) {
      const std::string& want = w[i + 1];
      char* end;
      double v = std::strtod(want.c_str(), &end);
      if (*end == '\0' && !want.empty() && want != "-") same = std::strtod(got[i].c_str(), nullptr) == v;
      else same = got[i] == want;
    }
    std::string a;
    for (auto& s : got) a += s + " ";
    CHECKF(same, "%s tick %d: call %s, got %s", name.c_str(), k, w[1].c_str(), a.c_str());
  }

  void drives(const std::vector<std::string>& w) {
    d_lines++;
    const Drives& d = runner->record().drives;
    double want[4] = {num(w[1]), num(w[2]), num(w[3]), num(w[4])};
    double got[4] = {d.energy, d.curiosity, d.sociability, d.arousal};
    for (int i = 0; i < 4; i++) {
      double diff = std::fabs(got[i] - want[i]);
      if (diff > max_drive_diff) max_drive_diff = diff;
      CHECKF(diff <= 1e-12, "%s tick %d: drive %d %.17g want %.17g", name.c_str(), k, i, got[i], want[i]);
    }
  }

  void scores(const std::vector<std::string>& w) {
    const Choice& c = *runner->record().choice;
    for (int b = 0; b < kBehaviours; b++) {
      if (w[b + 1] == "-") {
        CHECKF(!c.has_scores, "%s tick %d: scores where Python has none", name.c_str(), k);
        continue;
      }
      CHECKF(c.has_scores && std::fabs(c.scores[b] - num(w[b + 1])) <= 1e-12, "%s tick %d: score %d %.17g want %s",
             name.c_str(), k, b, c.scores[b], w[b + 1].c_str());
    }
  }

  void snap(const std::vector<std::string>& w) {
    Snapshot& s = body->snap;
    const std::string& key = w[1];
    bool none = w[2] == "-";
    if (key == "epoch") s.epoch = std::atoi(w[2].c_str());
    else if (key == "mode") s.mode = std::atoi(w[2].c_str());
    else if (key == "fallen") s.fallen = w[2] == "1";
    else if (key == "moving") s.moving = w[2] == "1";
    else if (key == "battery") {
      s.has_battery = !none;
      s.battery_pct = none ? 0 : num(w[2]);
    } else if (key == "dist") {
      s.has_dist = !none;
      s.dist_mm = none ? 0 : num(w[2]);
    } else if (key == "approach") {
      s.has_approach = !none;
      s.approach_mm_s = none ? 0 : num(w[2]);
    } else if (key == "event") {
      if (s.n_events == Snapshot::kEvents) {
        for (int i = 1; i < s.n_events; i++) s.events[i - 1] = s.events[i];
        s.n_events--;
      }
      s.events[s.n_events++] = {(uint8_t)std::atoi(w[2].c_str()), (int16_t)std::atoi(w[3].c_str()), 0,
                                (uint8_t)s.epoch};
      s.event_count++;
    } else CHECKF(false, "unknown snapshot key %s", key.c_str());
  }

  void src(const std::vector<std::string>& w) {
    source_on = true;
    GoldSource& s = *source;
    const std::string& key = w[1];
    if (key == "face") {
      s.has_face = true;
      s.face = w[2] == "1";
    } else if (key == "pet") {
      s.has_pet = true;
      s.pet = w[2] == "1";
    } else if (key == "approaching") {
      s.has_approaching = true;
      s.approaching = w[2] == "1";
    } else if (key == "face_pan") {
      s.has_face_pan = true;
      s.face_pan = num(w[2]);
    } else if (key == "startle") {
      s.has_startle = true;
      s.startle = num(w[2]);
    } else CHECKF(false, "unknown percept %s", key.c_str());
  }

  Clock clock_;
};

void test_rng_line(const std::vector<std::string>& w) {
  uint64_t seed = std::strtoull(w[1].c_str(), nullptr, 10);
  PyRandom r(seed);
  if (w[0] == "R") {
    for (size_t i = 2; i < w.size(); i++) {
      double v = r.random();
      CHECKF(v == num(w[i]), "random.Random(%llu) draw %zu: %.17g want %s", (unsigned long long)seed, i - 2, v,
             w[i].c_str());
    }
  } else {
    double mu = num(w[2]), sigma = num(w[3]);
    for (size_t i = 4; i < w.size(); i++) {
      double v = r.gauss(mu, sigma);
      CHECKF(std::fabs(v - num(w[i])) <= 1e-15, "gauss %zu: %.17g want %s", i - 4, v, w[i].c_str());
    }
  }
}

void replay(const char* path) {
  FILE* f = std::fopen(path, "r");
  if (f == nullptr) {
    CHECKF(false, "cannot open %s", path);
    return;
  }
  Golden g;
  char buf[1024];
  int poweroffs = 0;
  while (std::fgets(buf, sizeof buf, f)) {
    std::string s(buf);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    std::vector<std::string> w = split(s);
    if (w.empty() || w[0] == "#") continue;
    const std::string& c = w[0];
    if (c == "R" || c == "G") test_rng_line(w);
    else if (c == "S") g.start(w);
    else if (c == "c") g.set(w[1], w.size() > 2 ? w[2] : "");
    else if (c == "w") g.weight(w);
    else if (c == "s") g.snap(w);
    else if (c == "f") g.src(w);
    else if (c == "q") g.paused = w[1] == "1";
    else if (c == "h") {
      g.end_tick();
      if (g.runner) g.runner->request_shutdown();
      else CHECK(false);
    } else if (c == "T") g.tick(std::atoi(w[1].c_str()));
    else if (c == "b") g.call(w);
    else if (c == "k") {
      g.want_key.assign(w.begin() + 1, w.end());
      g.k_lines++;
    } else if (c == "d") g.drives(w);
    else if (c == "o") g.scores(w);
    else if (c == "P") {
      poweroffs++;
      CHECKF(g.runner->poweroff_requested() && g.runner->stopped(), "%s tick %d: no poweroff", g.name.c_str(), g.k);
    } else CHECKF(false, "unknown line %s", c.c_str());
  }
  g.end_tick();
  std::fclose(f);
  std::printf("behaviour vectors: %d scenarios, %d ticks, %d choice changes, %d drive checks, %d body calls, "
              "%d poweroffs, max drive diff %.3g\n",
              g.scenarios, g.ticks, g.k_lines, g.d_lines, g.calls, poweroffs, g.max_drive_diff);
  CHECK(g.scenarios == 5 && g.ticks > 9000 && poweroffs == 2);
}

}  // namespace

int main(int argc, char** argv) {
  test_selector_rechecks_and_preempts();
  test_sequence_memory_parallel_decorators();
  test_drives();
  test_selector();
  const char* vectors = "tests/behaviour_vectors.txt";
  for (int i = 1; i < argc; i++)
    if (std::strstr(argv[i], "behaviour_vectors") != nullptr) vectors = argv[i];
  replay(vectors);
  std::printf("sizeof Personality %zu, Library %zu, Runner %zu B\n", sizeof(Personality), sizeof(Library),
              sizeof(Runner));
  return report("behaviour");
}
