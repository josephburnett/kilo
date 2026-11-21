// The distance guard (guard/guard.h) and the alive layer (behaviour/alive.h).
//
// The guard's filter and zones on their own; the planned stop's way, the
// guard's stop_way_mm() against the real planner's; then the whole Mind on
// a scripted body that walks toward a wall: the body's position is the sum
// of the support feet's travel in the FEET the mind streamed, delayed by the
// servos (kLagTicks), and the ultrasonic reads the wall's distance from
// there every 80 ms (DIST_FRESH every 4th TELEMETRY), with noise, outliers
// and dropouts from a fixed seed. Approaching at each gait's top speed
// (the remote's level +5; the trot also +4), straight and steered (A6 / C6),
// Kilo must stop at least 100 mm short of the wall,
// sit and start the behaviour runner's sitting variant; a very close wall
// backs him off first; forward keys are refused while the guard holds,
// turning is not; a hand waved past and floor echoes with the head tilted
// down do not stop him; a steer or B7 out of a turn is a forward key while
// the guard holds; lying in light sleep (the floor at 55-63 mm) changes
// nothing; the eyes follow the zone when the body has them. Not walking
// forward the zones are closer (BLUE 120, RED 80): standing at 190 mm
// nothing happens, B6 next to something stays standing, and a forward key
// that would stop at once is refused.
// The alive layer: its standing poses inside the margin, its sitting
// tricks inside the joint limits, its choice seeded and never the same
// move twice in a row, and everything stopped by B6.
#include <cstdio>
#include <cstring>
#include <map>
#include "behaviour/alive.h"
#include "guard/guard.h"
#include "mind.h"
#include "planner/plan.h"
#include "power/battery.h"
#include "remote/client.h"
#include "tests/check.h"
#include "tests/mind_rig.h"
#include "tricks/model.h"
#include "util/str.h"

using namespace kilo;
using namespace kilo::guard;

namespace {

// keys (remote/keys.h)
constexpr uint16_t A1 = 0x42, A4 = 0x68, B4 = 0x98, C4 = 0xB0, B7 = 0x62, B5 = 0xA8, A6 = 0x22, C6 = 0xC2, B6 = 0x02, A5 = 0xE0, C7 = 0xE2;
constexpr int kLagTicks = 6;  // the servos: 50 ms dead time + 70 ms (model.h), about 6 ticks

struct Lcg {
  uint32_t s;
  explicit Lcg(uint32_t seed) : s(seed) {}
  double u() {  // [0, 1)
    s = s * 1664525u + 1013904223u;
    return (double)(s >> 8) / 16777216.0;
  }
};

// ---- the filter and the zones ------------------------------------------------------------

void test_filter() {
  DistanceFilter f;
  double t = 0;
  auto feed = [&](uint16_t mm, bool floor = false) {
    t += 0.08;
    f.feed(t, mm, true, floor);
  };
  feed(500);
  CHECK(!f.has());  // one reading is not enough
  feed(510);
  CHECK(f.has() && f.mm() == 510);  // two: the farther
  feed(120);  // one short echo (a hand waved past): the median ignores it
  CHECK(f.has() && f.mm() == 500);
  feed(505);
  feed(0);  // no echo: not a reading, the range holds
  CHECK(f.has() && f.dropouts() == 1);
  for (int i = 0; i < 6; i++) feed(0);  // 0.5 s of nothing: no range
  CHECK(!f.has());
  feed(230, true);  // the floor (the beam tilted down): not a reading
  feed(230, true);
  CHECK(!f.has() && f.floor_ignored() == 2);
  feed(300);
  feed(3000);  // beyond the range: as no echo
  CHECK(!f.has());
  // non-fresh telemetry between readings changes nothing
  feed(400);
  feed(410);
  f.feed(t + 0.02, 50, false, false);
  CHECK(f.has() && f.mm() == 400);  // 300, 400, 410 in the window
}

void test_zones() {
  const double blue = 316, red = kRedMm;
  Zone z = Zone::Green;
  z = next_zone(z, true, 400, blue, red);
  CHECK(z == Zone::Green);
  z = next_zone(z, true, 300, blue, red);
  CHECK(z == Zone::Blue);
  z = next_zone(z, true, 380, blue, red);  // inside the hysteresis
  CHECK(z == Zone::Blue);
  z = next_zone(z, true, 400, blue, red);
  CHECK(z == Zone::Green);
  z = next_zone(z, true, 140, blue, red);
  CHECK(z == Zone::Red);
  z = next_zone(z, true, 190, blue, red);
  CHECK(z == Zone::Red);
  z = next_zone(z, true, 210, blue, red);
  CHECK(z == Zone::Blue);
  CHECK(next_zone(Zone::Red, false, 0, blue, red) == Zone::Green);  // nothing seen
  // not walking forward: BLUE below 120 (left above 160), RED below 80 (left above 120)
  auto still = [](Zone now, double mm) {
    return next_zone(now, true, mm, kBlueStillMm, kRedStillMm, kBlueStillExitMm, kRedStillExitMm);
  };
  CHECK(still(Zone::Green, 190) == Zone::Green && still(Zone::Green, 150) == Zone::Green);
  CHECK(still(Zone::Green, 121) == Zone::Green && still(Zone::Green, 119) == Zone::Blue);
  CHECK(still(Zone::Blue, 160) == Zone::Blue && still(Zone::Blue, 161) == Zone::Green);
  CHECK(still(Zone::Blue, 81) == Zone::Blue && still(Zone::Blue, 79) == Zone::Red);
  CHECK(still(Zone::Red, 120) == Zone::Red && still(Zone::Red, 121) == Zone::Blue);
  CHECK(still(Zone::Green, 60) == Zone::Red);
  // the thresholds: never below kBlueMinMm; faster is further
  CHECK(blue_mm("trot", 0) == kBlueMinMm && blue_mm("trot", 230.4) > blue_mm("trot", 187.2));
  CHECK(blue_mm("walk", 97) <= blue_mm("trot", 230.4) && blue_mm("crawl", 27) == kBlueMinMm);
  std::printf("guard: BLUE at walk 97 mm/s %.0f, trot 144 %.0f, trot 180 %.0f, trot 200 %.0f, crawl 27 %.0f mm\n",
              blue_mm("walk", 97), blue_mm("trot", 144), blue_mm("trot", 180), blue_mm("trot", 200),
              blue_mm("crawl", 27));
}

// The quick stop's way in the real planner (its support feet's travel from halt() to done()),
// against the guard's v^2 / 2a; at several moments of the cycle (the worst counts), and the
// planned stop's for comparison (docs/walking-and-balance.md §14).
void test_stop_way_against_the_planner() {
  for (const char* g : {"walk", "trot", "crawl"}) {
    const planner::Pattern* pat = planner::find_pattern(g);
    for (double f : {0.4, 1.0, 1.25, 1.3, 1.6}) {  // the trot's remote levels +1, +3, +4 and +5: 1.3, 1.6
      if (f > 1.25 && !str_eq(g, "trot")) continue;  // the walk and the crawl top at 1.25 (their +5)
      for (double steer : {0.0, 0.25, 1.0}) {  // straight, and steered (A6 level 1 and 3) at the top speeds
        if (steer != 0 && f < 1.25) continue;
        double v = 0, worst = 0, planned = 0, ticks = 0;
        int period = (int)(planner::timing(*pat, (planner::real)(planner::default_speed(*pat) * f)).period_ms / 20);
        for (int quick = 0; quick < 2; quick++)
          for (int at = 0; at < period; at += (quick ? 2 : period)) {
            planner::Planner p;
            p.start(*pat, planner::Direction::Forward, (planner::real)(planner::default_speed(*pat) * f), 0, -1,
                    (planner::real)steer);
            planner::TickPlan prev = p.next();
            for (int t = 0; t < 250 + at; t++) prev = p.next();
            v = (double)p.timing().speed_mm_s;
            double moved = 0;
            if (quick) p.halt();
            else p.stop();
            int t = 0;
            for (; t < 1000 && !p.done(); t++) {
              planner::TickPlan cur = p.next();
              double s = 0;
              int n = 0;
              for (int i = 0; i < planner::kLegs; i++)
                if ((prev.support & cur.support) & (1 << i)) {
                  s += prev.x[i] - cur.x[i];
                  n++;
                }
              if (n) moved += s / n;
              prev = cur;
            }
            if (!quick) planned = moved;
            else if (moved > worst) worst = moved, ticks = t;
          }
        CHECKF(stop_way_mm(g, v) >= worst, "%s %.0f mm/s steer %.2f: planner %.1f mm, guard %.1f mm", g, v, steer,
               worst, stop_way_mm(g, v));
        std::printf("guard: %-5s %5.1f mm/s steer %.2f: the quick stop %4.1f mm, done in %3.0f ticks (guard allows "
                    "%4.1f); the planned stop %5.1f mm\n", g, v, steer, worst, ticks, stop_way_mm(g, v), planned);
      }
    }
  }
}

// The long trot's quick stop (A1's three stages, straight and at its one steer level) against the
// guard's way for it (the trot's): from every second tick of a cycle.
void test_long_trot_stop_way_against_the_planner() {
  for (double v : {160.0, 180.0, 200.0})
    for (double steer : {0.0, 0.25}) {
      double worst = 0, ticks = 0, sv = 0;
      int period = (int)(planner::pattern_timing(planner::kLongTrot, (planner::real)v).period_ms / 20);
      for (int at = 0; at < period; at += 2) {
        planner::Planner p;
        p.start(planner::kLongTrot, planner::Direction::Forward, (planner::real)v, 0, -1, (planner::real)steer);
        planner::TickPlan prev = p.next();
        for (int t = 0; t < 250 + at; t++) prev = p.next();
        sv = (double)p.timing().speed_mm_s;
        p.halt();
        double moved = 0;
        int t = 0;
        for (; t < 1000 && !p.done(); t++) {
          planner::TickPlan cur = p.next();
          double sum = 0;
          int n = 0;
          for (int i = 0; i < planner::kLegs; i++)
            if ((prev.support & cur.support) & (1 << i)) {
              sum += prev.x[i] - cur.x[i];
              n++;
            }
          if (n) moved += sum / n;
          prev = cur;
        }
        if (moved > worst) worst = moved, ticks = t;
      }
      CHECKF(stop_way_mm("longtrot", sv) >= worst, "longtrot %.0f steer %.2f: planner %.1f mm, guard %.1f mm", sv,
             steer, worst, stop_way_mm("longtrot", sv));
      std::printf("guard: longtrot %5.1f mm/s steer %.2f: the quick stop %4.1f mm, done in %3.0f ticks (guard allows "
                  "%4.1f), BLUE below %3.0f mm\n", sv, steer, worst, ticks, stop_way_mm("longtrot", sv),
                  blue_mm("longtrot", sv));
    }
}

// ---- the Mind walking at a wall ----------------------------------------------------------

struct WallRig : MindRig {
  double wall_mm = 2000;  // from the sensor at the start
  bool wall = true;
  double noise_mm = 0, outliers = 0, dropouts = 0;
  Lcg rng{12345};
  // the body's position: cumulative travel by FEET tick
  std::map<uint32_t, double> pos;
  size_t seen = 0;
  bool have_prev = false;
  kilo_feet_t prev = {};
  double travel = 0, max_travel = 0, back_travel = 0;
  double override_mm = -1;  // a reading forced (a hand), for one fresh reading
  double fixed_mm = -1;     // every reading this (the floor, which moves with him), when >= 0
  std::vector<std::pair<uint64_t, uint8_t>> done_at;  // TRICK_DONE to send: (time, trick_id)

  explicit WallRig(const MindConfig& cfg, bool eyes = true) : MindRig(cfg) {
    body.eyes = eyes;
    body.posture_pitch = true;
    start(200);
  }
  static MindConfig remote_cfg() {
    MindConfig c = MindRig::remote_cfg();
    c.seed = 7;
    return c;
  }
  double body_pos() const {  // the travel kLagTicks before the body's tick
    if (pos.empty()) return 0;
    auto it = pos.upper_bound((uint32_t)body.tick > (uint32_t)kLagTicks ? body.tick - kLagTicks : 0);
    if (it == pos.begin()) return 0;
    return std::prev(it)->second;
  }
  double gap() const { return wall_mm - max_travel; }
  void key(uint16_t k) { MindRig::key(k, 60); }
  void pass() override {
    // the next TELEMETRY's range
    bool fresh = body.tick % 4 == 0;
    body.flags = (uint16_t)((body.flags & ~KILO_TELEMETRY_FLAGS_DIST_FRESH) | (fresh ? KILO_TELEMETRY_FLAGS_DIST_FRESH : 0));
    if (fresh && body.clock.t + 4000 >= body.next_tel) {
      double d = fixed_mm >= 0 ? fixed_mm : wall ? wall_mm - body_pos() : 0;
      if (wall && noise_mm > 0) d += (rng.u() * 2 - 1) * noise_mm;
      if (wall && outliers > 0 && rng.u() < outliers) d = rng.u() < 0.5 ? d + 600 : d * 0.4;
      if (dropouts > 0 && rng.u() < dropouts) d = 0;
      if (override_mm >= 0) {
        d = override_mm;
        override_mm = -1;
      }
      body.dist_mm = (uint16_t)(d < 0 ? 1 : d > 4000 ? 0 : d);
    }
    MindRig::pass();
    for (; seen < body.sent.size(); seen++) {
      const Sent& s = body.sent[seen];
      if (s.type == KILO_MSG_TRICK_PLAY && s.payload.size() >= 2) done_at.push_back({body.clock.t + 1000000, s.payload[1]});
      if (s.type != KILO_MSG_FEET || s.payload.size() != sizeof(kilo_feet_t)) continue;
      kilo_feet_t f;
      std::memcpy(&f, s.payload.data(), sizeof f);
      if (have_prev && f.tick != prev.tick) {
        double sum = 0;
        int n = 0;
        for (int i = 0; i < 4; i++)
          if ((prev.support & f.support) & (1 << i)) {
            sum += prev.x_mm[i] - f.x_mm[i];
            n++;
          }
        double d = n ? sum / n : 0;
        travel += d;
        if (d < 0) back_travel -= d;
        if (travel > max_travel) max_travel = travel;
        pos[f.tick] = travel;
      }
      prev = f;
      have_prev = true;
    }
    for (size_t i = 0; i < done_at.size();)
      if (body.clock.t >= done_at[i].first) {
        body.event(KILO_EVENT_TRICK_DONE, (int16_t)done_at[i].second);
        done_at.erase(done_at.begin() + (long)i);
      } else {
        i++;
      }
  }
  // Run until the guard holds (or ms pass); true if it held.
  bool run_until_hold(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i += 4) {
      pass();
      if (mind.guard().action() == Action::Holding) return true;
    }
    return false;
  }
};

struct Approach {
  double gap, v, blue;
  bool held, sitting;
};

struct Run {
  int level = 5;  // the remote's speed level (the gait key starts at +3, each B7 one more)
  int steer = 0;  // A6 (+, left) or C6 (-) presses once at speed: a steered forward walk
  double noise = 0, outliers = 0, dropouts = 0;
};

Approach approach(uint16_t gait_key, double wall_mm, const Run& run = Run()) {
  WallRig r(WallRig::remote_cfg());
  r.wall_mm = wall_mm;
  r.noise_mm = run.noise;
  r.outliers = run.outliers;
  r.dropouts = run.dropouts;
  r.key(gait_key);
  for (int l = 3; l < run.level; l++) r.key(B7);  // the long trot: from its stage 1 (+1), each a stage
  for (int i = 0; i < (run.steer < 0 ? -run.steer : run.steer); i++) r.key(run.steer > 0 ? A6 : C6);
  Approach a = {};
  for (int i = 0; i < 90000 / 4 && !a.held; i++) {
    r.pass();
    if (r.mind.guard().action() == Action::Stopping && a.v == 0) {
      a.v = r.mind.guard().speed_now();
      a.blue = r.mind.guard().blue_now();
    }
    a.held = r.mind.guard().action() == Action::Holding;
  }
  r.run_ms(1500);
  a.gap = r.gap();
  // SIT, or the alive layer's sitting move (its trick clears the posture goal while it plays)
  a.sitting = (r.posture_goal() == KILO_POSTURE_SIT || r.mind.alive().sitting()) && r.mind.behaviour_active() &&
              r.mind.sitting_variant();
  return a;
}

void test_approach_each_gait_at_top_speed() {
  // level +4 and +5 (the trot 187.2 and 230.4 mm/s), straight and steered (A6 / C6 while walking:
  // still a forward walk for the guard)
  struct G {
    const char* name;
    uint16_t key;
    double wall;
    int level, steer;
  } gaits[] = {{"walk", C4, 700, 5, 0},  {"trot", A4, 900, 4, 0},  {"trot", A4, 1000, 5, 0},
               {"crawl", B4, 420, 5, 0}, {"walk", C4, 700, 5, 3},  {"trot", A4, 900, 4, 1},
               {"trot", A4, 1000, 5, 1}, {"trot", A4, 1000, 5, -2}, {"trot", A4, 1000, 5, 3},
               // the long trot (A1 starts at its stage 1, each B7 a stage more: level 4 is stage 2, 5 stage 3)
               {"longtrot", A1, 900, 4, 0}, {"longtrot", A1, 1000, 5, 0}, {"longtrot", A1, 1000, 5, 1}};
  for (const G& g : gaits) {
    for (int noisy = 0; noisy < 2; noisy++) {
      Run run;
      run.level = g.level;
      run.steer = g.steer;
      if (noisy) {
        run.noise = 12;
        run.outliers = 0.05;
        run.dropouts = 0.03;
      }
      Approach a = approach(g.key, g.wall, run);
      char what[48];
      if (g.steer == 0)
        std::snprintf(what, sizeof what, "%s +%d", g.name, g.level);
      else
        std::snprintf(what, sizeof what, "%s +%d steer %s %d", g.name, g.level, g.steer > 0 ? "left" : "right",
                      g.steer > 0 ? g.steer : -g.steer);
      CHECKF(a.held && a.sitting, "%s%s: held %d sitting %d", what, noisy ? " (noisy)" : "", a.held, a.sitting);
      CHECKF(a.gap >= 100, "%s%s: stopped %.0f mm from the wall", what, noisy ? " (noisy)" : "", a.gap);
      std::printf("guard: %-20s at %5.1f mm/s%s: BLUE below %3.0f mm, stopped %3.0f mm short of the wall\n", what,
                  a.v, noisy ? " (noise +-12 mm, 5 % outliers, 3 % dropouts)" : "", a.blue, a.gap);
    }
  }
}

void test_red_backs_off_then_sits() {
  WallRig r(WallRig::remote_cfg());
  r.wall_mm = 5000;
  r.key(A4);
  r.run_ms(3000);
  CHECK(r.mind.walker().active());
  r.wall_mm = r.body_pos() + 130;  // a box put down right in front of him
  bool backed = false;
  for (int i = 0; i < 8000 / 4; i++) {
    r.pass();
    if (r.mind.guard().action() == Action::Backing) backed = true;
    if (r.mind.guard().action() == Action::Holding) break;
  }
  r.run_ms(1500);
  CHECK(backed && r.mind.guard().pullbacks() == 1 && r.mind.guard().action() == Action::Holding);
  CHECK((r.posture_goal() == KILO_POSTURE_SIT || r.mind.alive().sitting()) && r.mind.sitting_variant());
  CHECKF(r.back_travel > 5 && r.back_travel < 120, "backed %.0f mm", r.back_travel);
  CHECK(r.log.has("guard action=backing") && r.log.has("guard zone=red"));
  std::printf("guard: RED at trot: backed off %.0f mm, then sat (final gap %.0f mm)\n", r.back_travel,
              r.wall_mm - r.travel);
}

void test_hold_refuses_forward_allows_turning_and_releases() {
  WallRig r(WallRig::remote_cfg());
  r.wall_mm = 700;
  r.key(A4);
  CHECK(r.run_until_hold(20000));
  r.run_ms(1500);
  CHECK(r.mind.behaviour_active());
  r.key(A4);  // forward: refused
  CHECK(r.log.has("remote A4 (0x68, trot): refused (distance guard") && !r.mind.walker().active());
  CHECK(r.mind.guard().refused() == 1);
  r.key(A6);  // a turn: the guard's runner gives way, Kilo turns
  r.run_ms(400);
  CHECK(r.log.has("remote A6 (0x22, turn left): behaviour runner stopped, walk left"));
  CHECK(r.mind.walker().active() && !r.mind.behaviour_active());
  r.key(B5);  // slower from a turn: still a turn (magnitude)
  r.key(B6);  // stand
  r.run_ms(3000);
  CHECK(!r.mind.walker().active());
  r.wall = false;  // turned away: nothing ahead
  r.run_ms(1500);
  CHECK(r.mind.guard().action() == Action::Watch && r.log.has("clear: forward walking allowed again"));
  r.key(A4);
  r.run_ms(800);
  CHECK(r.mind.walker().active() && r.mind.walker().goal().direction == planner::Direction::Forward);
  // the metrics line and the recorder's marks (no recorder attached here: the line is the check)
  char line[256];
  r.mind.metric_line(9, line, sizeof line);
  CHECK(std::strstr(line, "guard: on=1") != nullptr && std::strstr(line, "stops=1") != nullptr &&
        std::strstr(line, "refused=1") != nullptr);
}

// Steering and the guard: a steer key (A6 / C6 while walking forward) keeps the walk forward, so
// while the guard stops or holds it is a forward key; so is B7 out of a turn in place (straight
// forward). Without that, A6 pressed during the guard's stop restarted the forward walk at the wall.
void test_steering_while_the_guard_acts() {
  {  // A6 during the guard's planned stop: refused, the stop goes on
    WallRig r(WallRig::remote_cfg());
    r.wall_mm = 1000;
    r.key(A4);
    r.key(B7);
    r.key(B7);
    bool pressed = false;
    for (int i = 0; i < 30000 / 4 && r.mind.guard().action() != Action::Holding; i++) {
      r.pass();
      if (!pressed && r.mind.guard().action() == Action::Stopping) {
        pressed = true;
        r.key(A6);
      }
    }
    r.run_ms(3000);
    CHECKF(pressed && r.mind.guard().action() == Action::Holding, "action %s decision %s", action_name(r.mind.guard().action()), r.mind.guard().decision());
    CHECKF(r.log.has("remote A6 (0x22, turn left): refused (distance guard"), "A6 not refused");
    CHECKF(!r.mind.walker().active() && r.gap() >= 100, "walking %d, gap %.0f mm", r.mind.walker().active(), r.gap());
    std::printf("guard: A6 (steer) during the stop at trot +5: refused, stopped %.0f mm short of the wall\n", r.gap());
  }
  {  // B5 during the guard's stop (+5 to +4: still forward) restarts a forward walk: the guard stops it again
    WallRig r(WallRig::remote_cfg());
    r.wall_mm = 1000;
    r.key(A4);
    r.key(B7);
    r.key(B7);
    bool pressed = false;
    for (int i = 0; i < 30000 / 4 && !(pressed && r.mind.guard().action() == Action::Holding); i++) {
      r.pass();
      if (!pressed && r.mind.guard().action() == Action::Stopping) {
        pressed = true;
        r.key(B5);
      }
    }
    r.run_ms(3000);
    CHECKF(!r.mind.walker().active() && r.gap() >= 100 && r.log.has("a forward walk while holding"),
           "walking %d, gap %.0f mm", r.mind.walker().active(), r.gap());
    std::printf("guard: B5 (+5 to +4) during the stop: stopped again, %.0f mm short of the wall\n", r.gap());
  }
  {  // holding: A6 from standing turns in place (allowed); B7 then would walk straight forward: refused
    WallRig r(WallRig::remote_cfg());
    r.wall_mm = 700;
    r.key(A4);
    CHECK(r.run_until_hold(20000));
    r.run_ms(1500);
    r.key(A6);
    r.run_ms(1000);
    CHECK(r.mind.walker().active() && r.mind.walker().goal().direction == planner::Direction::Left);
    uint32_t refused = r.mind.guard().refused();
    r.key(B7);
    r.run_ms(1000);
    CHECK(r.mind.guard().refused() == refused + 1 && r.mind.walker().goal().direction == planner::Direction::Left);
    r.key(B5);  // out of the turn backward: allowed
    r.run_ms(1000);
    CHECK(r.mind.walker().active() && r.mind.walker().goal().direction == planner::Direction::Back);
  }
}

// Light sleep (power/sleep.h): after A7 Kilo lies in REST, then LIMP, the head down, and the sensor
// reads the floor at 55-63 mm (the recordings of 2026-11-17), with the odd dropout and outlier. The
// guard must not act, wake him or count refusals, and the eyes must not flap between colours.
void test_light_sleep_floor() {
  MindConfig c = WallRig::remote_cfg();
  c.light_sleep = true;
  WallRig r(c, true);
  r.wall_mm = 1500;  // standing: something far ahead
  r.run_ms(2000);
  r.key(0xA2);  // A7: rest, light sleep in 30 s
  r.wall_mm = 59;  // lying: the floor under his nose, +-4 mm, 5 % outliers, 3 % dropouts
  r.noise_mm = 4;
  r.outliers = 0.05;
  r.dropouts = 0.03;
  size_t eyes_before = r.body.count(KILO_MSG_SET_EYES);
  uint32_t changes_before = 0;
  {
    kilo_host_guard_status_t s;
    r.mind.guard().fill(s);
    changes_before = s.zone_changes;
  }
  r.run_ms(60000);
  kilo_host_guard_status_t s;
  r.mind.guard().fill(s);
  CHECKF(r.mind.sleep().asleep() && r.mind.sleep().wakes() == 0, "asleep %d wakes %u", r.mind.sleep().asleep(),
         (unsigned)r.mind.sleep().wakes());
  CHECK(r.mind.guard().action() == Action::Watch && s.stops == 0 && s.pullbacks == 0 && s.refused == 0);
  CHECKF(s.zone_changes == changes_before && r.mind.guard().zone() == Zone::Green, "zone changes %u while lying",
         (unsigned)(s.zone_changes - changes_before));
  const Goal* g = r.mind.link().goal(KILO_MSG_SET_EYES);  // green all along
  CHECK(g != nullptr && g->payload[2] == kEyeLevel && g->payload[1] == 0 && g->payload[3] == 0);
  CHECK(r.posture_goal() == KILO_POSTURE_LIMP);
  std::vector<Sent> sent = r.body.of(KILO_MSG_SET_EYES);
  size_t eyes = sent.size() - eyes_before, other = 0;  // the goal's refreshes: every one green
  for (size_t i = eyes_before; i < sent.size(); i++)
    if (sent[i].payload.size() != 4 || sent[i].payload[2] != kEyeLevel || sent[i].payload[1] != 0) other++;
  CHECK(other == 0);
  std::printf("guard: light sleep, the floor at 55-63 mm for 60 s: zone %s, %u zone changes, %zu SET_EYES (the goal's refresh, "
              "all green), %u floor readings ignored\n",
              zone_name(r.mind.guard().zone()), (unsigned)(s.zone_changes - changes_before), eyes,
              (unsigned)r.mind.guard().filter().floor_ignored());
}

void test_hand_and_floor_do_not_stop() {
  WallRig r(WallRig::remote_cfg());
  r.wall_mm = 100000;  // nothing ahead
  r.key(A4);
  r.run_ms(2500);
  for (int i = 0; i < 5; i++) {  // a hand waved past: one short reading at a time
    r.override_mm = 120;
    r.run_ms(500);
  }
  CHECK(r.mind.guard().action() == Action::Watch && r.mind.walker().active() && r.mind.guard().stops() == 0);
  // the head held tilted down 25 deg: the floor at ~230 mm, ignored
  CHECK(client_look(r.mind.link(), 0, -25));
  r.fixed_mm = 230;  // the floor stays 230 mm ahead as he walks
  uint32_t floor_before = r.mind.guard().filter().floor_ignored();
  r.run_ms(1000);
  r.fixed_mm = -1;
  CHECKF(r.mind.guard().action() == Action::Watch && r.mind.guard().filter().floor_ignored() > floor_before, "action %s decision %s floor %u/%u", action_name(r.mind.guard().action()), r.mind.guard().decision(), (unsigned)r.mind.guard().filter().floor_ignored(), (unsigned)floor_before);
  r.wall_mm = 100000;
  // a hand held there: stops
  CHECK(client_look(r.mind.link(), 0, 0));
  r.wall_mm = r.body_pos() + 280;
  bool held = r.run_until_hold(15000);  // 280 mm at 144 mm/s: inside BLUE's way, so RED and a back-off first
  CHECKF(held, "a hand held: zone %s action %s mm %.0f walking %d", zone_name(r.mind.guard().zone()),
         action_name(r.mind.guard().action()), r.mind.guard().filter().mm(), r.mind.walker().active());
}

void test_backward_trick_pickup_and_off_do_not_trigger() {
  {  // walking backward toward something "ahead" (the sensor looks forward): nothing
    WallRig r(WallRig::remote_cfg());
    r.key(A4);
    r.key(B5);
    r.key(B5);
    r.key(B5);  // +3 -> +2 -> +1 -> -1: backward
    r.run_ms(1500);
    CHECK(r.mind.walker().active() && r.mind.walker().goal().direction == planner::Direction::Back);
    r.wall_mm = 160 - r.travel;
    r.run_ms(1500);
    CHECK(r.mind.guard().stops() == 0 && r.mind.guard().pullbacks() == 0 && r.mind.walker().active());
  }
  {  // the guard off (kilo guard off --host): nothing; on again: it acts
    MindConfig c = WallRig::remote_cfg();
    c.guard = false;
    WallRig r(c);
    r.wall_mm = 600;
    r.key(A4);
    r.run_ms(6000);
    CHECK(r.mind.guard().stops() == 0 && r.mind.walker().active() && r.mind.guard().zone() != Zone::Green);
    r.mind.guard().set_enabled(true);
    CHECK(r.run_until_hold(3000));
  }
}

void test_eyes() {
  {  // a body with eyes: green, then blue at the stop, the colour latched as SET_EYES
    WallRig r(WallRig::remote_cfg(), true);
    const Goal* g = r.mind.link().goal(KILO_MSG_SET_EYES);
    CHECK(g != nullptr && g->payload[0] == 0x3F && g->payload[2] == kEyeLevel && g->payload[1] == 0 &&
          g->payload[3] == 0);
    CHECK(r.mind.guard().eyes_driven());
    r.wall_mm = 700;
    r.key(A4);
    CHECK(r.run_until_hold(20000));
    g = r.mind.link().goal(KILO_MSG_SET_EYES);
    CHECK(g != nullptr && g->payload[3] == kEyeLevel && r.mind.guard().eyes() == Eyes::Blue);
    r.wall_mm = r.travel + 60;  // closer: red (the walk has ended: the still thresholds)
    r.run_ms(600);
    g = r.mind.link().goal(KILO_MSG_SET_EYES);
    CHECK(g != nullptr && g->payload[1] == kEyeLevel && r.mind.guard().eyes() == Eyes::Red);
  }
  {  // the old firmware (no EYES in HELLO caps): no SET_EYES, the colour kept in the mind only
    WallRig r(WallRig::remote_cfg(), false);
    r.run_ms(500);
    CHECK(r.body.count(KILO_MSG_SET_EYES) == 0 && !r.mind.guard().eyes_driven());
    kilo_host_guard_status_t s;
    r.mind.guard().fill(s);
    CHECK(s.enabled == 1 && s.eyes_driven == 0 && s.eyes == KILO_GUARD_EYES_GREEN);
  }
}

// ---- a hand in his face, in every state (2026-11-19: "he should always react") -------------

// Run until pred (or ms pass); the time it took, or -1.
template <typename P>
double run_until(WallRig& r, uint32_t ms, P pred) {
  for (uint32_t i = 0; i < ms; i += 4) {
    if (pred()) return i / 1000.0;
    r.pass();
  }
  return pred() ? ms / 1000.0 : -1;
}

void test_hand_in_face_walking_every_gait() {
  struct G {
    const char* name;
    uint16_t key;
    int steer;
  } gaits[] = {{"trot", A4, 0}, {"walk", C4, 0}, {"crawl", B4, 0}, {"trot", A4, 2}, {"walk", C4, -1},
               {"longtrot", A1, 0}, {"longtrot", A1, 1}};
  for (const G& g : gaits) {
    for (int red = 0; red < 2; red++) {
      WallRig r(WallRig::remote_cfg());
      r.wall = false;  // nothing ahead: no echo
      r.key(g.key);
      for (int i = 0; i < (g.steer < 0 ? -g.steer : g.steer); i++) r.key(g.steer > 0 ? A6 : C6);
      r.run_ms(3000);
      CHECK(r.mind.walker().active() && r.mind.walker().goal().direction == planner::Direction::Forward);
      r.fixed_mm = red ? 100 : 180;  // a hand put in front of his face, held there as he comes
      uint32_t hisses = r.mind.guard().hisses();
      double t_act = run_until(r, 3000, [&] { return r.mind.guard().action() != Action::Watch; });
      // the crawl and the long trot, very close: they stop (reversing takes 4 s, 1.5 s)
      bool slow = std::strcmp(g.name, "crawl") == 0 || std::strcmp(g.name, "longtrot") == 0;
      bool backs = red && !slow;
      double t_halt = run_until(r, 3000, [&] { return r.mind.walker().planner().halting() || backs; });
      CHECKF(t_act >= 0 && t_act < 0.45, "%s steer %d %s: reacted after %.2f s", g.name, g.steer, red ? "RED" : "BLUE",
             t_act);
      CHECK(backs ? r.mind.guard().action() == Action::Backing : t_halt >= 0);
      double t_held = run_until(r, 12000, [&] { return r.mind.guard().action() == Action::Holding; });
      bool held = t_held >= 0;
      r.run_ms(1500);
      CHECKF(held && r.mind.sitting_variant() && r.mind.behaviour_active(), "%s steer %d: held %d (%s, %s)", g.name,
             g.steer, held, action_name(r.mind.guard().action()), r.mind.guard().decision());
      CHECK(!red || (r.mind.guard().hisses() == hisses + 1 && r.mind.guard().pullbacks() == (backs ? 1u : 0u)));
      std::printf("guard: a hand in his face (%s, %3.0f mm) walking %-5s%s: acts after %.2f s, sits %.1f s later\n",
                  red ? "RED " : "BLUE", r.fixed_mm, g.name, g.steer ? " steering" : "", t_act, t_held);
    }
  }
}

void test_hand_in_face_turning_backward_standing_sitting_lying() {
  {  // turning in place: BLUE keeps turning (a wall seen as he turns), RED stops him, he sits and hisses
    WallRig r(WallRig::remote_cfg());
    r.wall = false;
    r.key(A6);
    r.run_ms(2000);
    CHECK(r.mind.walker().active() && r.mind.walker().goal().direction == planner::Direction::Left);
    r.fixed_mm = 180;  // closer than a forward walk stops for: not while turning (green)
    r.run_ms(1500);
    CHECK(r.mind.walker().active() && !r.mind.walker().planner().stopping() && r.mind.guard().zone() == Zone::Green);
    r.fixed_mm = 100;
    r.run_ms(1500);
    CHECK(r.mind.walker().active() && !r.mind.walker().planner().stopping() && r.mind.guard().zone() == Zone::Blue);
    r.fixed_mm = 60;
    double t = run_until(r, 3000, [&] { return r.mind.walker().planner().halting(); });
    CHECKF(t >= 0 && t < 0.45, "turning: halted after %.2f s", t);
    CHECK(r.run_until_hold(6000) && r.mind.guard().hisses() == 1 && r.mind.guard().pullbacks() == 0);
    std::printf("guard: a hand in his face (RED) turning in place: the quick stop after %.2f s, sits\n", t);
  }
  {  // walking backward: away from it already, nothing but the eyes
    WallRig r(WallRig::remote_cfg());
    r.wall = false;
    r.key(A4);
    r.key(B5);
    r.key(B5);
    r.key(B5);
    r.run_ms(1500);
    CHECK(r.mind.walker().goal().direction == planner::Direction::Back);
    r.fixed_mm = 60;
    r.run_ms(1500);
    CHECK(r.mind.walker().active() && !r.mind.walker().planner().stopping() && r.mind.guard().zone() == Zone::Red &&
          r.mind.guard().action() == Action::Watch && r.mind.guard().eyes() == Eyes::Red);
    CHECK(r.log.has("state=walk-back: eyes only (walking away from it)"));
  }
  {  // standing (B6): BLUE (110 mm): he looks at it and sits; then RED while sitting: he leans back and hisses
    WallRig r(WallRig::remote_cfg());
    r.wall = false;
    r.key(B6);
    r.run_ms(1500);
    r.fixed_mm = 110;
    double t = run_until(r, 2000, [&] { return r.mind.guard().action() == Action::Holding; });
    CHECKF(t >= 0 && t < 0.4, "standing BLUE: %.2f s", t);
    r.run_ms(200);
    const Goal* h = r.mind.link().goal(KILO_MSG_SET_HEAD);
    CHECK(h != nullptr && r.mind.sitting_variant() && r.log.has("guard: looks at it"));
    CHECK(r.posture_goal() == KILO_POSTURE_SIT || r.mind.alive().sitting());
    size_t tricks_before = r.body.count(KILO_MSG_TRICK_PLAY);
    r.fixed_mm = 60;
    t = run_until(r, 2000, [&] { return r.mind.guard().hisses() == 1; });
    r.run_ms(1500);
    CHECKF(t >= 0 && t < 0.4 && r.log.has("guard: leans back"), "sitting RED: %.2f s", t);
    CHECK(r.body.count(KILO_MSG_TRICK_PLAY) > tricks_before && r.mind.alive().last_move() == behaviour::AliveMove::Flinch);
    std::printf("guard: standing, a hand (BLUE): looks and sits after %.2f s; sitting, closer (RED): leans back\n", t);
  }
  {  // standing, RED at once: a short back-off, then he sits
    WallRig r(WallRig::remote_cfg());
    r.wall = false;
    r.key(B6);
    r.run_ms(1500);
    r.fixed_mm = 60;
    double t = run_until(r, 2000, [&] { return r.mind.guard().action() == Action::Backing; });
    CHECKF(t >= 0 && t < 0.4, "standing RED: %.2f s", t);
    CHECK(run_until(r, 3000, [&] { return r.mind.walker().active() && r.mind.walker().goal().direction == planner::Direction::Back; }) >= 0);
    CHECK(r.run_until_hold(10000) && r.mind.guard().pullbacks() == 1 && r.mind.guard().hisses() == 1);
    r.run_ms(1500);
    CHECK(r.mind.sitting_variant());
    std::printf("guard: standing, a hand very close (RED): backs off after %.2f s (%.0f mm), sits\n", t, r.back_travel);
  }
  {  // the release rule: a hand that comes and goes while he sits backs him off at most kMaxBacks times
    WallRig r(WallRig::remote_cfg());
    r.wall = false;
    CHECK(client_posture(r.mind.link(), nullptr, "sit"));
    r.run_ms(1500);
    for (int i = 0; i < 5; i++) {
      r.fixed_mm = 60;
      r.run_ms(700);
      r.fixed_mm = 140;  // the hand pulled back a little: blue, not green
      r.run_ms(2500);
    }
    CHECKF(r.mind.guard().hisses() == (uint32_t)kMaxBacks, "%u hisses (%s)", (unsigned)r.mind.guard().hisses(),
           r.mind.guard().decision());
    CHECK(r.log.has("eyes only (backed off twice: clear for 1 s first)"));
    r.fixed_mm = -1;  // gone: green for a second releases it
    r.run_ms(2500);
    CHECK(r.mind.guard().action() == Action::Watch && r.mind.guard().backs() == 0);
    r.fixed_mm = 60;
    r.run_ms(700);
    CHECK(r.mind.guard().hisses() == (uint32_t)kMaxBacks + 1);
  }
  {  // lying (A7, REST): the floor under his nose and a hand alike are the floor: the eyes stay green
    WallRig r(WallRig::remote_cfg());
    r.wall = false;
    r.key(0xA2);
    r.run_ms(1500);
    r.fixed_mm = 60;
    r.run_ms(2000);
    CHECKF(r.mind.guard().action() == Action::Watch && r.mind.guard().zone() == Zone::Green &&
               r.log.has("state=lying: nothing seen (lying: every reading is the floor)"),
           "%s %s %s", action_name(r.mind.guard().action()), zone_name(r.mind.guard().zone()), r.mind.guard().decision());
  }
  {  // the head held tilted down while standing: the floor (230 mm) is ignored, a hand nearer is not
    WallRig r(WallRig::remote_cfg());
    r.wall = false;
    r.key(B6);
    r.run_ms(1000);
    CHECK(client_look(r.mind.link(), 0, -25));  // the floor from 0.75 x 80 / sin 25 = 142 mm on
    r.fixed_mm = 230;
    r.run_ms(2000);
    CHECK(r.mind.guard().zone() == Zone::Green && r.mind.guard().action() == Action::Watch &&
          r.mind.guard().filter().floor_ignored() > 10);
    r.fixed_mm = 110;
    CHECK(run_until(r, 1500, [&] { return r.mind.guard().action() != Action::Watch; }) >= 0);
  }
  {  // no echo right after very close (a hand closer than the sensor measures): still red for a second
    WallRig r(WallRig::remote_cfg());
    r.wall = false;
    CHECK(client_posture(r.mind.link(), nullptr, "sit"));
    r.run_ms(1500);
    r.fixed_mm = 70;
    r.run_ms(500);
    CHECK(r.mind.guard().zone() == Zone::Red);
    r.fixed_mm = 0;  // no echo
    r.run_ms(700);
    CHECK(r.mind.guard().zone() == Zone::Red && r.mind.guard().filter().close_hold());
    r.run_ms(800);
    CHECK(r.mind.guard().zone() == Zone::Green);
  }
}

// ---- not walking forward: the closer thresholds (2026-11-21: "maybe the threshold can be lowered
// when he's not moving forward"), and the moves between them and the walking ones -------------

void test_still_thresholds() {
  {  // standing (B6) with something at 190 mm (the owner's floor): nothing; at 110: looks at it and sits
    WallRig r(WallRig::remote_cfg());
    r.wall = false;
    r.key(B6);
    r.run_ms(1500);
    r.fixed_mm = 190;
    r.run_ms(3000);
    CHECKF(r.mind.guard().action() == Action::Watch && r.mind.guard().reactions() == 0 &&
               r.mind.guard().zone() == Zone::Green && r.mind.guard().eyes() == Eyes::Green &&
               r.posture_goal() == KILO_POSTURE_STAND,
           "standing at 190: %s %s posture %d", action_name(r.mind.guard().action()), zone_name(r.mind.guard().zone()),
           r.posture_goal());
    CHECK(r.mind.guard().walk_zone() == Zone::Blue && r.mind.guard().blue_now() == kBlueStillMm &&
          r.mind.guard().red_now() == kRedStillMm);
    r.fixed_mm = 110;
    double t = run_until(r, 2000, [&] { return r.mind.guard().action() == Action::Holding; });
    r.run_ms(1500);
    CHECKF(t >= 0 && t < 0.4 && r.log.has("guard: looks at it") && r.mind.sitting_variant(), "standing at 110: %.2f s", t);
    std::printf("guard: standing at 190 mm: nothing; at 110 mm: looks and sits after %.2f s\n", t);
  }
  {  // sitting with something at 150 mm: B6 stands him up, and he stays standing; a forward key is refused
    WallRig r(WallRig::remote_cfg());
    r.wall = false;
    CHECK(client_posture(r.mind.link(), nullptr, "sit"));
    r.run_ms(1500);
    r.fixed_mm = 150;
    r.run_ms(1000);
    r.key(B6);
    r.run_ms(5000);
    CHECKF(r.posture_goal() == KILO_POSTURE_STAND && r.mind.guard().action() == Action::Watch &&
               r.mind.guard().reactions() == 0 && !r.mind.behaviour_active(),
           "B6 at 150: posture %d %s reactions %u", r.posture_goal(), action_name(r.mind.guard().action()),
           (unsigned)r.mind.guard().reactions());
    r.key(A4);  // (a): a walk would stop at once (BLUE below 200): it does not start
    r.run_ms(1000);
    CHECKF(r.log.has("remote A4 (0x68, trot): refused (distance guard 150 mm ahead, a forward walk needs 200 (clear "
                     "above 280); turn, back up or B6)"),
           "A4 at 150 not refused");
    CHECK(!r.mind.walker().active() && r.mind.guard().refused() == 1 && r.mind.guard().stops() == 0);
    r.key(A6);  // turning is allowed
    r.run_ms(1000);
    CHECK(r.mind.walker().active() && r.mind.walker().goal().direction == planner::Direction::Left);
    r.key(B6);
    r.run_ms(2000);
    r.fixed_mm = 300;  // moved away: forward walking again
    r.run_ms(500);
    r.key(A4);
    r.run_ms(800);
    CHECK(r.mind.walker().active() && r.mind.walker().goal().direction == planner::Direction::Forward);
    std::printf("guard: B6 with something at 150 mm: stands and stays standing; A4 refused\n");
  }
  {  // (b): the quick stop at the walking distance, then B6: he stands and stays standing, forward keys stay
     // refused until the way is clear for walking, and then the hold releases
    WallRig r(WallRig::remote_cfg());
    r.wall_mm = 700;
    r.key(A4);
    CHECK(r.run_until_hold(20000));
    double gap = r.gap();
    r.run_ms(1500);
    CHECK(r.mind.sitting_variant());
    CHECKF(gap > kBlueStillMm && gap < kBlueMinMm, "stopped %.0f mm short", gap);
    CHECK(r.mind.guard().zone() == Zone::Green && r.mind.guard().walk_zone() != Zone::Green);
    r.key(B6);
    r.run_ms(5000);
    CHECKF(r.posture_goal() == KILO_POSTURE_STAND && r.mind.guard().action() == Action::Holding &&
               r.mind.guard().reactions() == 0 && !r.mind.behaviour_active(),
           "after the stop, B6: posture %d %s reactions %u (%s)", r.posture_goal(), action_name(r.mind.guard().action()),
           (unsigned)r.mind.guard().reactions(), r.mind.guard().decision());
    CHECK(r.mind.guard().eyes() == Eyes::Green);
    r.key(A4);
    r.run_ms(500);
    CHECKF(r.log.has("remote A4 (0x68, trot): refused (distance guard holding: green, ") &&
               r.log.has("mm ahead; a forward walk needs 200 (clear above 280); turn, back up or B6)"),
           "A4 after the stop not refused as expected");
    CHECK(!r.mind.walker().active());
    r.wall = false;  // taken away: the hold releases, forward walking again
    r.run_ms(1500);
    CHECK(r.mind.guard().action() == Action::Watch && r.log.has("clear: forward walking allowed again"));
    r.key(A4);
    r.run_ms(800);
    CHECK(r.mind.walker().active() && r.mind.walker().goal().direction == planner::Direction::Forward);
    std::printf("guard: stopped %.0f mm short at trot +3, then B6: stands, stays standing, A4 refused until clear\n",
                gap);
  }
  {  // through the guard: the still hysteresis, BLUE from 119 to 160, RED from 79 to 120 (sitting: the eyes only
     // after the first look and lean)
    WallRig r(WallRig::remote_cfg());
    r.wall = false;
    CHECK(client_posture(r.mind.link(), nullptr, "sit"));
    r.run_ms(1500);
    struct S {
      double mm;
      Zone z;
    } steps[] = {{125, Zone::Green}, {115, Zone::Blue}, {155, Zone::Blue}, {165, Zone::Green}, {115, Zone::Blue},
                 {85, Zone::Blue},   {75, Zone::Red},   {115, Zone::Red},  {125, Zone::Blue},  {170, Zone::Green}};
    for (const S& st : steps) {
      r.fixed_mm = st.mm;
      r.run_ms(600);
      CHECKF(r.mind.guard().zone() == st.z, "sitting at %.0f mm: zone %s", st.mm, zone_name(r.mind.guard().zone()));
    }
  }
}

// ---- the battery at rest, and the eyes' orange --------------------------------------------

// The owner's wall of 2026-11-19: the legs' load sags the reading below the 5 % mark (7000 mV) while
// he walks and for a moment after; at rest the pack holds 7450 mV. The guard stops him and starts the
// sitting runner: it must sit, not take the low-battery REST, and the eyes stay the zone's.
void test_sag_does_not_send_the_runner_to_rest() {
  WallRig r(WallRig::remote_cfg(), true);
  r.body.batt_mv = 7450;
  r.run_ms(21000);  // the rig's first 8000 mV out of the window
  r.wall_mm = r.travel + 1000;
  r.key(A4);
  uint64_t walked_until = 0;
  bool held = false;
  for (int i = 0; i < 20000 / 4 && !held; i++) {
    bool loaded = r.mind.walker().active() || r.body.mode == KILO_MODE_GAIT;
    if (loaded) walked_until = r.body.clock.t;
    r.body.batt_mv = loaded || r.body.clock.t < walked_until + 1500000 ? 6900 : 7450;  // the sag, 1.5 s after too
    r.pass();
    held = r.mind.guard().action() == Action::Holding;
  }
  CHECK(held);
  for (int i = 0; i < 4000 / 4; i++) {
    r.body.batt_mv = r.body.clock.t < walked_until + 1500000 ? 6900 : 7450;
    r.pass();
  }
  CHECK(r.mind.behaviour_active() && r.mind.sitting_variant());
  CHECKF(r.posture_goal() != KILO_POSTURE_REST, "posture %d", r.posture_goal());  // SIT (or an alive move's trick)
  CHECK(!r.mind.runner().context().data.low_battery && !r.mind.tired());
  double pct = 0;
  CHECK(r.mind.rested_pct(&pct) && pct > 30);
  CHECK(!r.log.has("battery: tired"));
  // the raw reading is still what the metrics and kilo status show
  char line[256];
  r.mind.metric_line(13, line, 201);
  CHECKF(std::strstr(line, "battery: mv=7450 min_mv=6900 ") == line && std::strstr(line, " rested_mv=7450") != nullptr,
         "%s", line);
}

// Genuinely low at rest (6950 mV, about 4 %): the runner rests, the eyes are orange while the way is
// clear (with the runner on or off), blue and red win; +5 % clears it; the body's LOW_BATT too.
void test_low_at_rest_rests_and_the_eyes_are_orange() {
  auto rgb = [](WallRig& r) {
    const Goal* g = r.mind.link().goal(KILO_MSG_SET_EYES);
    return g == nullptr ? -1 : (g->payload[1] << 16) | (g->payload[2] << 8) | g->payload[3];
  };
  const int green = kEyeLevel << 8, orange = (kEyeLevel << 16) | (kOrangeGreen << 8), blue = kEyeLevel,
            red = kEyeLevel << 16;
  {
    WallRig r(WallRig::remote_cfg(), true);
    r.wall = false;
    CHECK(rgb(r) == green);
    r.body.batt_mv = 6950;
    r.run_ms(500);  // the higher readings before still in the window: not yet
    CHECK(!r.mind.tired());
    r.run_ms(21000);
    CHECK(r.mind.tired() && r.mind.guard().eyes() == Eyes::Orange && rgb(r) == orange);
    CHECK(r.log.has("battery: tired (6950 mV at rest, 4 %, below 5 %): the eyes orange while the way is clear"));
    kilo_host_guard_status_t st;
    r.mind.guard().fill(st);
    CHECK(st.eyes == KILO_GUARD_EYES_ORANGE && st.zone == KILO_GUARD_ZONE_GREEN);
    char line[256];
    r.mind.metric_line(9, line, 201);
    CHECKF(std::strstr(line, " zone=green ") != nullptr && std::strstr(line, " eyes=orange ") != nullptr, "%s", line);
    // the runner: the low-battery branch, REST
    r.key(A5);
    r.run_ms(3000);
    CHECK(r.mind.runner().context().data.low_battery);
    CHECKF(r.posture_goal() == KILO_POSTURE_REST, "posture %d", r.posture_goal());
    r.key(B6);
    // 8 %: still tired (it clears at 10 %)
    r.body.batt_mv = 7100;
    r.run_ms(1000);
    CHECK(r.mind.tired());
    r.body.batt_mv = 7200;  // 12 %
    r.run_ms(1000);
    CHECK(!r.mind.tired() && rgb(r) == green && r.log.has("battery: no longer tired (7200 mV at rest, 12 %)"));
  }
  {  // tired, then a wall: blue and red win over orange, and orange comes back when clear
    WallRig r(WallRig::remote_cfg(), true);
    r.body.batt_mv = 6950;
    r.run_ms(21000);
    CHECK(r.mind.tired() && rgb(r) == orange);
    // (a tired Kilo that stops at a wall then lies down in the runner's REST, and lying the range is the
    // floor's: so blue and red are checked while he walks)
    r.wall_mm = r.travel + 900;
    r.key(A4);
    bool blue_seen = false, red_seen = false, ok = true;
    for (int i = 0; i < 20000 / 4 && !red_seen; i++) {
      r.pass();
      Zone z = r.mind.guard().zone();
      if (z == Zone::Blue && !blue_seen) {
        blue_seen = true;
        ok = ok && rgb(r) == blue && r.mind.guard().eyes() == Eyes::Blue;
        r.wall_mm = r.travel + 100;  // then very close
      }
      if (z == Zone::Red) {
        red_seen = true;
        ok = ok && rgb(r) == red && r.mind.guard().eyes() == Eyes::Red;
      }
    }
    CHECK(blue_seen && red_seen && ok);
    r.wall = false;  // gone: no echo (held as very close for kCloseHoldS), then green
    r.run_ms(2000);
    CHECKF(r.mind.guard().zone() == Zone::Green && rgb(r) == orange, "zone %s", zone_name(r.mind.guard().zone()));
  }
  {  // the body's LOW_BATT mode: tired whatever the reading
    WallRig r(WallRig::remote_cfg(), true);
    r.wall = false;
    r.body.mode = KILO_MODE_LOW_BATT;
    r.run_ms(200);
    CHECK(r.mind.tired() && rgb(r) == orange && r.log.has("battery: tired (the body's LOW_BATT mode)"));
    r.body.mode = KILO_MODE_IDLE;
    r.run_ms(200);
    CHECK(!r.mind.tired() && rgb(r) == green);
  }
  {  // USB power: no reading, never tired
    WallRig r(WallRig::remote_cfg(), true);
    r.wall = false;
    r.body.flags |= KILO_TELEMETRY_FLAGS_NO_BATTERY;
    r.body.batt_mv = 5000;
    r.run_ms(2000);
    double pct;
    CHECK(!r.mind.tired() && !r.mind.rested_pct(&pct));
  }
}

void test_rested_battery() {
  power::RestedBattery b;
  uint16_t mv = 0;
  CHECK(!b.mv(0, &mv));
  uint64_t t = 1000000;
  for (int i = 0; i < 100; i++, t += 40000) b.add(t, i % 10 == 0 ? 7450 : 7150, false);  // a reading high every 0.4 s
  CHECK(b.mv(t, &mv) && mv == 7450);
  for (int i = 0; i < 600; i++, t += 40000) b.add(t, 7150, false);  // 24 s of sag only: the window forgets the high
  CHECK(b.mv(t, &mv) && mv == 7150);
  CHECK(!b.mv(t + 21000000, &mv));  // nothing for 21 s: none
  b.add(t, 7300, false);
  b.add(t + 40000, 0, true);  // USB power: forgotten
  CHECK(!b.mv(t + 40000, &mv));
}

// ---- the alive layer ---------------------------------------------------------------------

void test_alive_moves() {
  FakeBody body;
  CaptureLog log;
  SerialLink link(body.serial, body.clock, log);
  LogRing ring(body.clock, &log);
  tricks::TrickPlayer player(link, body.clock, ring);
  behaviour::Alive a(link, player, ring, 3);
  planner::BodyModel model;
  // every standing pose the planner can draw stays inside the margin; every sitting trick inside the limits
  double worst = 1e9;
  for (int i = 0; i < 400; i++) {
    behaviour::AliveMove m = (behaviour::AliveMove)(i % (int)behaviour::AliveMove::kCount);
    behaviour::BodyPose ps[behaviour::Alive::kMaxSteps];
    int n = a.plan_stand_move(m, (i % 7 - 3) * 15.0, ps, behaviour::Alive::kMaxSteps);
    for (int k = 0; k < n; k++) {
      double mg = behaviour::stance_margin_mm(model, ps[k]);
      if (mg < worst) worst = mg;
      CHECKF(mg >= behaviour::kMinMarginMm, "%s: margin %.1f", behaviour::alive_move_name(m), mg);
      CHECK(ps[k].pitch <= 6 && ps[k].pitch >= -3 && ps[k].roll <= 3 && ps[k].roll >= -3 && ps[k].dh >= -7 &&
            ps[k].dh <= 3 && ps[k].xs >= -6 && ps[k].xs <= 6);
    }
    const tricks::Trick& t = a.plan_sit_move(m);
    const char* why = tricks::trick_problem(t);
    CHECKF(why == nullptr, "%s: %s", behaviour::alive_move_name(m), why ? why : "");
    CHECK(t.end_posture == KILO_POSTURE_SIT && (t.joint_mask & 0x3u) == 0);  // never the head
    uint32_t ticks = 0;
    CHECK(tricks::total_ticks(t, tricks::kSpeedUnity, &ticks) && ticks >= 50 && ticks <= 400);
  }
  std::printf("alive: the smallest standing margin over 400 drawn moves %.1f mm (minimum %.0f)\n", worst,
              behaviour::kMinMarginMm);
}

// Through the Mind: A5, the tree sits (idle), the alive layer plays sitting moves, seeded and
// never twice the same in a row; with the tree standing (explore) the body pose eases; B6 ends it all.
void test_alive_through_the_mind() {
  MindConfig cfg = WallRig::remote_cfg();
  cfg.seed = 11;
  WallRig r(cfg);
  r.wall = false;
  r.key(A5);
  behaviour::AliveMove last = behaviour::AliveMove::kCount;
  uint32_t moves = 0;
  bool repeat = false;
  for (int i = 0; i < 60000 / 4; i++) {
    r.pass();
    if (r.mind.alive().moves() != moves) {
      moves = r.mind.alive().moves();
      if (r.mind.alive().last_move() == last) repeat = true;
      last = r.mind.alive().last_move();
    }
  }
  CHECKF(moves >= 6 && !repeat, "%u moves in 60 s, repeat %d", moves, repeat);
  CHECK(r.body.count(KILO_MSG_TRICK_PLAY) >= 3 && r.log.has("alive: sitting"));
  // standing: put the tree's posture to STAND (as explore does) and let the layer take it
  CHECK(client_posture(r.mind.link(), nullptr, "stand"));
  r.run_ms(15000);
  size_t poses = r.body.count(KILO_MSG_SET_BODY_POSE);
  CHECKF(r.mind.alive().standing() && r.posture_goal() == KILO_POSTURE_STANCE && poses > 10, "poses %zu", poses);
  r.key(B6);  // stops it: the pose back to level, then stand
  r.run_ms(200);
  const Goal* g = r.mind.link().goal(KILO_MSG_SET_BODY_POSE);
  CHECK(!r.mind.behaviour_active() && r.posture_goal() == KILO_POSTURE_STAND);
  CHECK(g != nullptr && g->payload[1] == 0 && g->payload[2] == 0 && g->payload[3] == 0 && g->payload[4] == 0 &&
        g->payload[5] == 0 && g->payload[6] == 0);
  size_t after = r.body.count(KILO_MSG_SET_BODY_POSE) + r.body.count(KILO_MSG_TRICK_PLAY);
  r.run_ms(5000);
  CHECK(r.body.count(KILO_MSG_SET_BODY_POSE) + r.body.count(KILO_MSG_TRICK_PLAY) <= after + 25);  // refreshes only
  CHECK(r.mind.alive().moves() == moves + (r.mind.alive().moves() - moves));
  // C7 during a sitting move: frozen, the move's trick stopped
  r.key(A5);
  r.run_ms(12000);
  r.key(C7);
  CHECK(!r.mind.behaviour_active() && r.mind.remote().frozen());
  // the same seed: the same moves
  WallRig r2(cfg);
  r2.wall = false;
  r2.key(A5);
  r2.run_ms(30000);
  WallRig r3(cfg);
  r3.wall = false;
  r3.key(A5);
  r3.run_ms(30000);
  CHECK(r2.mind.alive().moves() == r3.mind.alive().moves() && r2.mind.alive().last_move() == r3.mind.alive().last_move());
}

}  // namespace

int main() {
  test_filter();
  test_zones();
  test_stop_way_against_the_planner();
  test_long_trot_stop_way_against_the_planner();
  test_approach_each_gait_at_top_speed();
  test_red_backs_off_then_sits();
  test_hold_refuses_forward_allows_turning_and_releases();
  test_steering_while_the_guard_acts();
  test_light_sleep_floor();
  test_hand_and_floor_do_not_stop();
  test_backward_trick_pickup_and_off_do_not_trigger();
  test_hand_in_face_walking_every_gait();
  test_hand_in_face_turning_backward_standing_sitting_lying();
  test_still_thresholds();
  test_eyes();
  test_rested_battery();
  test_sag_does_not_send_the_runner_to_rest();
  test_low_at_rest_rests_and_the_eyes_are_orange();
  test_alive_moves();
  test_alive_through_the_mind();
  return report("guard");
}
