// The self-measurement routine (measure/; docs/self-measurement.md): the
// script against the Python's vectors (tests/measure_vectors.txt), its
// poses' safety (every pose in the FEET ranges and the legs' reach, the
// centre of mass inside the feet with margin under the uncertainty bands,
// the worst tilt a tip can give), and C5 twice in the whole Mind on a scripted
// body: a run to its end (the stream, the balance off and back, the run in
// the flight recorder, STAND), every key stopping it, a fall, the
// refusals (battery, USB, not standing, not level, not still), a tip
// cutting a sweep short, a tripod's lean that is not a tip, and the stop on
// a tilt.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>
#include "measure/measure.h"
#include "measure/script.h"
#include "mind.h"
#include "tests/check.h"
#include "tests/mind_rig.h"

using namespace kilo;
using namespace kilo::measure;

namespace {

// ---- the script against the Python --------------------------------------------------------------
void test_vectors(const char* path) {
  std::ifstream in(path);
  CHECKF(in.good(), "no %s", path);
  std::string line;
  Script s;
  planner::BodyModel m;
  struct Tip {
    int exp;
    uint32_t seen;
  };
  std::vector<Tip> tips;
  uint8_t prev = 0xF;
  long ticks = 0, runs = 0;
  double worst = 0;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::string w;
    ss >> w;
    if (w == "run") {
      s.reset();
      tips.clear();
      prev = 0xF;
      runs++;
    } else if (w == "tip") {
      Tip t;
      ss >> t.exp >> t.seen;
      tips.push_back(t);
    } else if (w == "t") {
      // script.poses: a tip once the planner is TIP_SEEN_LAG_TICKS (7) past it, in its experiment
      for (size_t i = 0; i < tips.size(); i++) {
        if (!s.done() && s.step().exp == tips[i].exp && tips[i].seen + 7 <= s.tick()) {
          s.tip(tips[i].seen);
          tips.erase(tips.begin() + (long)i);
          break;
        }
      }
      uint32_t tick;
      int step, first, x[4], z[4], sup;
      double p, r;
      ss >> tick >> step >> first >> x[0] >> x[1] >> x[2] >> x[3] >> z[0] >> z[1] >> z[2] >> z[3] >> p >> r >> sup;
      Pose pose;
      bool f = false;
      bool ok = s.next(&pose, &f);
      CHECKF(ok, "tick %u: the script ended early", tick);
      if (!ok) return;
      bool same = (int)f == first && s.step_index() == step;
      for (int k = 0; k < 4; k++) same = same && pose.x[k] == x[k] && pose.z[k] == z[k];
      CHECKF(same, "tick %u: step %d/%d first %d/%d x %d %d %d %d (want %d %d %d %d) z %d %d %d %d", tick,
             s.step_index(), step, (int)f, first, pose.x[0], pose.x[1], pose.x[2], pose.x[3], x[0], x[1], x[2], x[3],
             pose.z[0], pose.z[1], pose.z[2], pose.z[3]);
      real xs[4], zs[4];
      for (int k = 0; k < 4; k++) {
        xs[k] = pose.x[k];
        zs[k] = pose.z[k];
      }
      uint8_t got = 0;
      planner::Attitude a = rigid_rest(m, xs, zs, prev, &got);
      prev = got;
      double d = std::fmax(std::fabs(a.pitch - p), std::fabs(a.roll - r));
      worst = std::fmax(worst, d);
      CHECKF(d < 0.02 && got == sup, "tick %u: rest %.4f %.4f sup %d, want %.4f %.4f sup %d", tick, (double)a.pitch,
             (double)a.roll, got, p, r, sup);
      ticks++;
      if (!same) return;
    } else if (w == "end") {
      long n;
      ss >> n;
      Pose pose;
      bool f;
      CHECKF(!s.next(&pose, &f) && s.done(), "the script goes on past %ld ticks", n);
    }
  }
  std::printf("measure vectors (%s): %ld runs, %ld ticks, rest within %.4f deg\n", path, runs, ticks, worst);
  CHECK(runs == 2 && ticks > 7500);
}

// ---- the poses' safety ------------------------------------------------------------------------------
planner::BodyModel moved(real dx, real dy, real dz) {
  planner::BodyModel m;
  real k = m.total_g() / m.body.grams;
  m.body.x += dx * k;
  m.body.y += dy * k;
  m.body.z += dz * k;
  return m;
}

void test_safety() {
  Script s;
  planner::BodyModel m;
  Pose p;
  bool f;
  uint32_t n = 0;
  int xmin = 99, xmax = -99, zmin = 999, zmax = -1;
  real reach_margin = 1e9, min_four = 1e9, max_tilt = 0;
  // the centre of mass's uncertainty bands: +-10 mm fore-aft, +-7 sideways, +-15 up and down
  const real band[][3] = {{0, 0, 0},     {10, 0, 0},   {-10, 0, 0},    {0, 7, 0},    {0, -7, 0},
                          {10, 7, 15},   {-10, -7, 15}, {10, -7, -15}, {-10, 7, -15}, {-10, -7, 0},
                          {10, 7, 0},    {0, 7, 15},   {0, -7, 15},    {10, -7, 15},  {-10, 7, 15}};
  const int nb = (int)(sizeof band / sizeof band[0]);
  std::vector<planner::BodyModel> ms;
  for (int b = 0; b < nb; b++) ms.push_back(moved(band[b][0], band[b][1], band[b][2]));
  std::vector<uint8_t> prev(nb, 0xF);
  while (s.next(&p, &f)) {
    n++;
    real xs[4], zs[4];
    for (int k = 0; k < 4; k++) {
      xs[k] = p.x[k];
      zs[k] = p.z[k];
      xmin = p.x[k] < xmin ? p.x[k] : xmin;
      xmax = p.x[k] > xmax ? p.x[k] : xmax;
      zmin = p.z[k] < zmin ? p.z[k] : zmin;
      zmax = p.z[k] > zmax ? p.z[k] : zmax;
      real d = std::sqrt(xs[k] * xs[k] + zs[k] * zs[k]);
      reach_margin = std::fmin(reach_margin, std::fmin(m.reach_mm() - d, d - m.min_reach_mm()));
    }
    for (int b = 0; b < nb; b++) {
      uint8_t sup = 0;
      planner::Attitude a = rigid_rest(ms[b], xs, zs, prev[b], &sup);
      prev[b] = sup;
      max_tilt = std::fmax(max_tilt, std::fmax(std::fabs(a.pitch), std::fabs(a.roll)));
      // the four feet's polygon: where the mass must stay whatever tripod he is on
      real mg = planner::stability(ms[b], xs, zs, 0xF, a.pitch, a.roll);
      min_four = std::fmin(min_four, mg);
    }
  }
  std::printf("measure poses: %u ticks (%.1f s), x %d..%d z %d..%d mm, reach margin %.1f mm, the mass %.1f mm inside "
              "the feet at least, tilt at most %.1f deg (the mass anywhere in its bands)\n",
              n, n * 0.02, xmin, xmax, zmin, zmax, (double)reach_margin, (double)min_four, (double)max_tilt);
  CHECK(n == max_ticks() && n * 20 <= 95000);
  CHECK(xmin >= -60 && xmax <= 60 && zmin >= 30 && zmax <= 95);  // FEET's ranges
  CHECK(reach_margin >= 0.5);
  CHECK(min_four >= 10);       // inside the four feet with 10 mm to spare, the mass anywhere in its bands
  CHECK(max_tilt <= 12);       // a tip is caught 5 mm down; far from the body's fall reflex (75 deg)
}

// ---- C5 twice in the whole Mind ---------------------------------------------------------------------------
struct Rig : MindRig {
  std::function<void(Rig&)> each;
  // The scripted body's IMU reads what each FEET predicted for its tick (a body that does what the
  // model says), level outside GAIT; off: it reads body.pitch / body.roll as the test sets them.
  bool follow = true;
  std::vector<int16_t> fp = std::vector<int16_t>(65536), fr = std::vector<int16_t>(65536);
  explicit Rig(const MindConfig& cfg) : MindRig(cfg) {
    attach_history();
    attach_recorder();
  }
  void pass() override {
    if (follow) {
      body.pitch = body.mode == KILO_MODE_GAIT ? fp[body.tick] : (int16_t)0;
      body.roll = body.mode == KILO_MODE_GAIT ? fr[body.tick] : (int16_t)0;
    }
    body.step();
    mind.poll();
    size_t before = body.sent.size();
    body.react();
    for (size_t i = before; i < body.sent.size(); i++) {
      const Sent& s = body.sent[i];
      if (s.type != KILO_MSG_FEET) continue;
      kilo_feet_t f;
      std::memcpy(&f, s.payload.data(), sizeof f);
      fp[f.tick] = f.pitch;
      fr[f.tick] = f.roll;
    }
    // the real body: any posture ends GAIT (the rig's ends it only on STANCE)
    for (size_t i = before; i < body.sent.size(); i++) {
      const Sent& s = body.sent[i];
      if (s.type == KILO_MSG_SET_POSTURE && s.payload.size() == 4 && body.mode == KILO_MODE_GAIT) {
        body.mode = KILO_MODE_IDLE;
        body.event(KILO_EVENT_GAIT_STOPPED, 0);
      }
    }
    if (each) each(*this);
  }
  bool ring_has(const char* t) {
    LogRing& r = mind.log();
    for (uint32_t s = r.first_seq(); s < r.next_seq(); s++) {
      const LogEntry* e = r.get(s);
      if (e != nullptr && std::strstr(e->text, t) != nullptr) return true;
    }
    return false;
  }
  // every MARK the flight recorder holds, in order
  std::vector<std::string> marks() {
    std::vector<std::string> out;
    const record::FlightStore& f = mind.recorder().flight();
    if (f.empty()) return out;
    for (uint32_t s = f.first_seq(); s <= f.last_seq(); s++) {
      const uint8_t* b = f.block(s);
      uint32_t used = f.used(s), p = KILO_RECORD_BLOCK_HEADER_SIZE;
      while (p + KILO_WALK_RECORD_HEADER_SIZE <= used) {
        kilo_walk_record_header_t rh;
        std::memcpy(&rh, b + p, sizeof rh);
        p += sizeof rh;
        if (rh.kind == KILO_WALK_RECORD_MARK) out.push_back(std::string((const char*)b + p, rh.len));
        p += rh.len;
      }
    }
    return out;
  }
  bool marked(const char* prefix) {
    for (const std::string& m : marks())
      if (m.compare(0, std::strlen(prefix), prefix) == 0) return true;
    return false;
  }
  void stand() {  // B6, then a second of standing still
    key(0x02, 1500);
  }
  // C5 twice (remote.h): the calibration posture, then STAND and the routine once he is still
  void measure_keys() {
    key(0x90, 1500);
    key(0x90, 100);
    for (int i = 0; i < 80 && !measuring(); i++) run_ms(100);
  }
  bool measuring() { return mind.measure().active(); }
  const Goal* goal(uint8_t t) { return mind.link().goal(t); }
};

MindConfig cfg() {
  MindConfig c = MindRig::remote_cfg();
  c.balance_kp = 0.5;  // a balance goal held from the start: the routine turns it off and back
  return c;
}

void test_full_run() {
  Rig r(cfg());
  r.start();
  r.measure_keys();
  CHECKF(r.measuring(), "not measuring");
  CHECK(r.ring_has("remote C5 (0x90, calibration posture): posture calib (C5 again: measure himself)"));
  CHECK(r.ring_has("remote C5 (0x90, calibration posture): standing, measuring (about 80 s; any key stops it)"));
  r.run_ms(1000);
  const Goal* b = r.goal(KILO_MSG_SET_BALANCE);
  CHECK(b != nullptr && b->payload[0] == 0);  // the balance off while it measures
  CHECK(r.goal(KILO_MSG_SET_POSTURE) == nullptr && r.body.mode == KILO_MODE_GAIT);
  for (int i = 0; i < 1100 && r.measuring(); i++) r.run_ms(100);
  CHECKF(!r.measuring(), "still measuring after 110 s");
  // the body did what the model said: every sweep tipped where the model has its crossing, and ended early
  uint32_t ticks = r.mind.measure().script().tick();
  CHECKF(r.mind.measure().ended_ok() && r.mind.measure().tips() == 8, "%s, %u tips", r.mind.measure().ended(),
         r.mind.measure().tips());
  CHECKF(ticks < max_ticks() - 500 && ticks * 20 <= 80000, "%u ticks of %u", ticks, max_ticks());
  std::printf("measure: a run on a body that does what the model says: %u ticks (%.1f s), 8 tips\n", ticks,
              ticks * 0.02);
  // the stream: every script tick sent once, in order, 50 Hz TELEMETRY asked for and put back
  std::vector<Sent> feet = r.body.of(KILO_MSG_FEET);
  CHECKF(feet.size() == ticks && r.mind.measure().ticks_sent() == ticks, "%zu FEET", feet.size());
  bool seq = true, ranges = true;
  for (size_t i = 1; i < feet.size(); i++) {
    kilo_feet_t a, c;
    std::memcpy(&a, feet[i - 1].payload.data(), sizeof a);
    std::memcpy(&c, feet[i].payload.data(), sizeof c);
    seq = seq && (uint16_t)(a.tick + 1) == c.tick;
    for (int k = 0; k < 4; k++) ranges = ranges && c.x_mm[k] >= -60 && c.x_mm[k] <= 60 && c.z_mm[k] >= 30 && c.z_mm[k] <= 95;
  }
  CHECK(seq && ranges);
  std::vector<Sent> ss = r.body.of(KILO_MSG_SET_STREAM);
  CHECK(ss.size() >= 2 && ss.front().payload[0] == 1 && ss.back().payload[0] == KILO_TELEMETRY_DEFAULT_DIV);
  // the end: STAND, the balance goal as it was
  const Goal* p = r.goal(KILO_MSG_SET_POSTURE);
  CHECK(p != nullptr && p->payload[1] == KILO_POSTURE_STAND);
  b = r.goal(KILO_MSG_SET_BALANCE);
  CHECK(b != nullptr && b->payload[0] == 1);
  // the run in the flight recorder
  r.run_ms((uint32_t)(record::Recorder::kPostRollUs / 1000) + 200);
  const record::Run* run = r.mind.recorder().find(1);
  CHECK(run != nullptr && run->kind == KILO_RUN_KIND_TRICK && (run->flags & KILO_WALK_FLAGS_ENDED_OK) &&
        !(run->flags & KILO_WALK_FLAGS_RECORDING));
  if (run != nullptr) CHECKF(std::string(run->text, run->text_len) == "measure=self end=DONE", "%.*s", run->text_len, run->text);
  CHECK(r.marked("measure id=1 format=1") && r.marked("model com_x=") && r.marked("step i=0 exp=level phase=level t=0") &&
        r.marked("step i=") && r.marked("end reason=DONE"));
  // the header says which script and its tip rule; each sweep left its two baselines and its tip
  bool script2 = false;
  int bases = 0, tip_lines = 0;
  for (const std::string& m : r.marks()) {
    if (m.compare(0, 8, "measure ") == 0)
      script2 = m.find(" script=2 ") != std::string::npos && m.find(" tip_rise_c=150 ") != std::string::npos &&
                m.find(" tip_jump_c=100 tip_jump_t=8 tip_base_t=10 tip_up_pct=60 ") != std::string::npos &&
                m.find(" battery_mv=") != std::string::npos;
    bases += m.compare(0, 9, "base exp=") == 0;
    tip_lines += m.compare(0, 8, "tip exp=") == 0 && m.find(" how=tip ") != std::string::npos;
  }
  CHECKF(script2 && bases == 8 && tip_lines == 8, "script2 %d, %d base lines, %d tip lines", (int)script2, bases,
         tip_lines);
  int steps = 0;
  for (const std::string& m : r.marks()) steps += m.compare(0, 5, "step ") == 0;
  const Step* st;
  CHECK(steps == measure::steps(&st));
  CHECK(r.ring_has("measure: done"));
}

void test_stops() {
  struct Case {
    uint16_t key;
    const char* line;
    int posture;  // the posture goal after
  } cases[] = {
      {0x02, "remote B6 (0x02, stand): measurement stopped, posture stand", KILO_POSTURE_STAND},
      {0x90, "remote C5 (0x90, calibration posture): measurement stopped: standing", KILO_POSTURE_STAND},
      {0xA2, "remote A7 (0xA2, sleep): measurement stopped, posture rest", KILO_POSTURE_REST},
      {0xE2, "remote C7 (0xE2, freeze): frozen (measurement stopped: standing)", KILO_POSTURE_STAND},
      {0x68, "remote A4 (0x68, trot): measurement stopped: standing (only B6, A7 and C7 act after it)",
       KILO_POSTURE_STAND},
      {0x18, "remote B3 (0x18, none): measurement stopped: standing (only B6, A7 and C7 act after it)",
       KILO_POSTURE_STAND},  // a key that does nothing else stops it too
      {0x38, "remote B2 (0x38, meow): measurement stopped: standing (only B6, A7 and C7 act after it)",
       KILO_POSTURE_STAND},
  };
  for (const Case& c : cases) {
    Rig r(cfg());
    r.start();
    r.measure_keys();
    r.run_ms(4000);
    CHECK(r.measuring());
    size_t feet = r.body.count(KILO_MSG_FEET);
    r.key(c.key, 60);
    CHECKF(r.ring_has(c.line), "key 0x%02x: no '%s'", c.key, c.line);
    r.run_ms(1600);
    CHECKF(!r.measuring(), "key 0x%02x: still measuring", c.key);
    CHECK(r.body.count(KILO_MSG_FEET) <= feet + 8);  // at once: what was in flight at most
    CHECK(!r.mind.walker().active());
    const Goal* p = r.goal(KILO_MSG_SET_POSTURE);
    CHECKF(p != nullptr && p->payload[1] == c.posture, "key 0x%02x: posture %d", c.key, p ? p->payload[1] : -1);
    CHECK(std::strncmp(r.mind.measure().ended(), "stopped by ", 11) == 0 && !r.mind.measure().ended_ok());
    CHECK(r.marked("end reason=STOPPED"));
  }
}

void test_fall_and_tilt() {
  {
    Rig r(cfg());
    r.start();
    r.measure_keys();
    r.run_ms(3000);
    r.body.event(KILO_EVENT_FELL, 2);
    r.run_ms(100);
    CHECK(!r.measuring() || r.mind.measure().phase() == Phase::Ending);
    r.run_ms(1600);
    CHECK(!r.measuring() && r.marked("end reason=FELL"));
  }
  {
    Rig r(cfg());
    r.start();
    r.measure_keys();
    r.run_ms(3000);
    r.follow = false;
    r.body.roll = 30 * 16;  // picked up, or over
    r.run_ms(200);
    r.run_ms(1600);
    CHECK(!r.measuring() && r.marked("end reason=ABORTED") && r.ring_has("tilted past 25 deg"));
    const Goal* p = r.goal(KILO_MSG_SET_POSTURE);
    CHECK(p != nullptr && p->payload[1] == KILO_POSTURE_STAND);
  }
  {
    Rig r(cfg());
    r.start();
    r.measure_keys();
    r.run_ms(3000);
    r.follow = false;
    r.body.pitch = 12 * 16;  // 12 deg off the model's level
    r.run_ms(1800);
    CHECK(!r.measuring() && r.ring_has("deg off the model's prediction"));
  }
}

void test_refusals() {
  struct Case {
    const char* name;
    std::function<void(Rig&)> before;  // before the start (the battery at rest is the last 20 s' highest)
    std::function<void(Rig&)> set;     // after standing
    const char* why;
  } cases[] = {
      {"battery", [](Rig& r) { r.body.batt_mv = 7300; }, [](Rig&) {}, "measure refused: battery 7300 mV at rest (needs 7400)"},
      {"usb", [](Rig& r) { r.body.flags |= KILO_TELEMETRY_FLAGS_NO_BATTERY; }, [](Rig&) {},
       "measure refused: on USB power (no battery)"},
      {"tilted", [](Rig&) {}, [](Rig& r) { r.body.pitch = 12 * 16; r.run_ms(1100); },
       "measure refused: not level and still (pitch 12.00 roll 0.00"},
      {"moving", [](Rig&) {},
       [](Rig& r) {
         r.each = [](Rig& q) { q.body.roll = (int16_t)((q.body.tick / 5) % 2 ? 4 * 16 : 0); };
         r.run_ms(1100);
       },
       "measure refused: not level and still"},
  };
  for (const Case& c : cases) {
    Rig r(cfg());
    r.follow = false;
    c.before(r);
    r.start();
    r.stand();
    c.set(r);
    r.key(0x90, 1500);  // C5 twice: the calibration posture, then STAND and the routine asked until it gives up
    r.key(0x90, 100);
    r.run_ms(7000);
    CHECKF(!r.measuring() && r.ring_has(c.why), "%s: no '%s'", c.name, c.why);
    if (!r.ring_has(c.why))
      for (const std::string& l : r.log.lines)
        if (l.find("remote B") != std::string::npos) std::printf("    %s\n", l.c_str());
    CHECK(r.body.count(KILO_MSG_FEET) == 0 && r.mind.recorder().runs_total() == 0);
  }
  {  // C5 once: the calibration posture, nothing measured
    Rig r(cfg());
    r.start();
    r.run_ms(1000);
    r.key(0x90, 3000);
    CHECK(!r.measuring() && r.body.count(KILO_MSG_FEET) == 0);
    const Goal* p = r.goal(KILO_MSG_SET_POSTURE);
    CHECK(p != nullptr && p->payload[1] == KILO_POSTURE_CALIB);
  }
}

// The first sweep's experiment (edge_lf) in kExperimentTable.
int edge_lf() {
  for (int i = 0; i < kExperiments; i++)
    if (std::strcmp(kExperimentTable[i].name, "edge_lf") == 0) return i;
  return -1;
}

// A tip in the first sweep: the scripted body tips toward the lifted foot 30 ticks into it.
void test_tip() {
  Rig r(cfg());
  r.follow = false;  // level, but for the tip it is told
  r.start();
  r.stand();
  uint32_t sweep_from = 0;
  const int e = edge_lf();
  r.each = [&](Rig& q) {
    const Measure& m = q.mind.measure();
    if (!m.active() || m.script().done()) return;
    const Step& s = m.script().step();
    if (s.exp == e && std::strcmp(s.phase, "sweep") == 0 && sweep_from == 0) sweep_from = m.script().tick();
    if (sweep_from != 0 && m.script().tick() > sweep_from + 30 && s.exp == e) {
      q.body.pitch = 37;  // the model's tip, a16
      q.body.roll = 70;
    }
    if (s.exp > e) q.body.pitch = q.body.roll = 0;
  };
  r.measure_keys();
  for (int i = 0; i < 1100 && r.measuring(); i++) r.run_ms(100);
  CHECK(!r.measuring() && r.mind.measure().ended_ok() && r.mind.measure().tips() == 1);
  CHECK(r.marked("tip exp=edge_lf") && r.marked("base exp=edge_lf settle_p=0.00 settle_r=0.00 lifted_p=0.00"));
  CHECKF(r.mind.measure().script().tick() < max_ticks() - 100, "%u of %u ticks", r.mind.measure().script().tick(),
         max_ticks());
}

// A tripod's lean is not a tip: as the foot comes up the scripted body leans 1.8 deg toward it (legs
// that give, a twist: Kilo's first run, 2026-11-21) and creeps 0.5 deg more along the sweep. No tip is
// seen and the sweep runs to its end. The same lean and then a jump of 2 deg is a tip.
void test_lean_is_not_a_tip() {
  for (int jump = 0; jump < 2; jump++) {
    Rig r(cfg());
    r.follow = false;
    r.start();
    r.stand();
    const int e = edge_lf();
    uint32_t sweep_from = 0, sweep_ticks = 0;
    r.each = [&](Rig& q) {
      const Measure& m = q.mind.measure();
      if (!m.active() || m.script().done()) return;
      const Step& s = m.script().step();
      bool in = s.exp == e && (std::strcmp(s.phase, "lifted") == 0 || std::strcmp(s.phase, "sweep") == 0);
      if (!in) {
        q.body.pitch = q.body.roll = 0;
        return;
      }
      double deg = 1.8;
      if (std::strcmp(s.phase, "sweep") == 0) {
        if (sweep_from == 0) sweep_from = m.script().tick();
        sweep_ticks = m.script().tick() - sweep_from;
        deg += 0.5 * sweep_ticks / 240.0;
        if (jump && sweep_ticks > 60) deg += 2.0;
      }
      q.body.pitch = (int16_t)std::lround(deg * 37 / 79.18 * 16);  // along the model's tip (37, 70) a16
      q.body.roll = (int16_t)std::lround(deg * 70 / 79.18 * 16);
    };
    r.measure_keys();
    for (int i = 0; i < 1100 && r.measuring(); i++) r.run_ms(100);
    CHECK(!r.measuring() && r.mind.measure().ended_ok());
    CHECK(r.marked("base exp=edge_lf settle_p=0.00 settle_r=0.00 lifted_p=0.8"));
    if (jump) {
      CHECKF(r.mind.measure().tips() == 1 && r.marked("tip exp=edge_lf") && sweep_ticks < 100, "%u tips, %u ticks",
             r.mind.measure().tips(), sweep_ticks);
    } else {
      CHECKF(r.mind.measure().tips() == 0 && !r.marked("tip exp=") && sweep_ticks >= 235, "%u tips, %u ticks",
             r.mind.measure().tips(), sweep_ticks);
      CHECK(r.mind.measure().script().tick() == max_ticks());
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  const char* path = "tests/measure_vectors.txt";
  for (int i = 1; i < argc; i++)
    if (std::strstr(argv[i], "measure_vectors")) path = argv[i];
  test_vectors(path);
  test_safety();
  test_full_run();
  test_stops();
  test_fall_and_tilt();
  test_refusals();
  test_tip();
  test_lean_is_not_a_tip();
  return report("measure");
}
