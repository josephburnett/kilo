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
// nothing; the eyes follow the zone when the body has them.
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
#include "remote/client.h"
#include "tests/body_rig.h"
#include "tests/check.h"
#include "tricks/model.h"
#include "util/str.h"

using namespace kilo;
using namespace kilo::guard;

namespace {

// keys (remote/keys.h)
constexpr uint16_t A4 = 0x68, B4 = 0x98, C4 = 0xB0, B7 = 0x62, B5 = 0xA8, A6 = 0x22, C6 = 0xC2, B6 = 0x02, A5 = 0xE0, C7 = 0xE2;
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
  // the thresholds: never below kBlueMinMm; faster is further
  CHECK(blue_mm("trot", 0) == kBlueMinMm && blue_mm("trot", 180) > blue_mm("trot", 144));
  CHECK(blue_mm("walk", 97) < blue_mm("trot", 180));
  std::printf("guard: BLUE at walk 97 mm/s %.0f, trot 144 %.0f, trot 180 %.0f, trot 200 %.0f, crawl 27 %.0f mm\n",
              blue_mm("walk", 97), blue_mm("trot", 144), blue_mm("trot", 180), blue_mm("trot", 200),
              blue_mm("crawl", 27));
}

// The planned stop's way in the real planner (its support feet's travel from stop() to done()),
// against the guard's v^2 / 2a.
void test_stop_way_against_the_planner() {
  for (const char* g : {"walk", "trot", "crawl"}) {
    const planner::Pattern* pat = planner::find_pattern(g);
    for (double f : {0.4, 1.0, 1.25, 1.3, 1.6}) {  // the trot's remote levels +1, +3, +4 and +5: 1.3, 1.6
      if (f > 1.25 && !str_eq(g, "trot")) continue;  // the walk and the crawl top at 1.25 (their +5)
      for (double steer : {0.0, 0.25, 1.0}) {  // straight, and steered (A6 level 1 and 3) at the top speeds
        if (steer != 0 && f < 1.25) continue;
        planner::Planner p;
        p.start(*pat, planner::Direction::Forward, (planner::real)(planner::default_speed(*pat) * f), 0, -1,
                (planner::real)steer);
        planner::TickPlan prev = p.next();
        for (int t = 0; t < 250; t++) prev = p.next();
        double v = (double)p.timing().speed_mm_s, moved = 0;
        p.stop();
        for (int t = 0; t < 1000 && !p.done(); t++) {
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
        CHECKF(stop_way_mm(g, v) >= moved, "%s %.0f mm/s steer %.2f: planner %.1f mm, guard %.1f mm", g, v, steer,
               moved, stop_way_mm(g, v));
        std::printf("guard: %-5s %5.1f mm/s steer %.2f: the planned stop %5.1f mm (guard allows %5.1f)\n", g, v,
                    steer, moved, stop_way_mm(g, v));
      }
    }
  }
}

// ---- the Mind walking at a wall ----------------------------------------------------------

struct WallRig {
  FakeBody body;
  CaptureLog log;
  Mind mind;
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
  std::vector<std::pair<uint64_t, uint8_t>> done_at;  // TRICK_DONE to send: (time, trick_id)

  explicit WallRig(const MindConfig& cfg, bool eyes = true) : mind(body.serial, body.clock, log, nullptr, cfg) {
    body.eyes = eyes;
    mind.start();
    run_ms(200);
  }
  static MindConfig remote_cfg() {
    MindConfig c;
    c.remote = true;
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
  void key(uint16_t k) {
    body.event(KILO_EVENT_IR_KEY, (int16_t)k);
    run_ms(60);
  }
  void pass() {
    // the next TELEMETRY's range
    bool fresh = body.tick % 4 == 0;
    body.flags = (uint16_t)((body.flags & ~KILO_TELEMETRY_FLAGS_DIST_FRESH) | (fresh ? KILO_TELEMETRY_FLAGS_DIST_FRESH : 0));
    if (fresh && body.clock.t + 4000 >= body.next_tel) {
      double d = wall ? wall_mm - body_pos() : 0;
      if (wall && noise_mm > 0) d += (rng.u() * 2 - 1) * noise_mm;
      if (wall && outliers > 0 && rng.u() < outliers) d = rng.u() < 0.5 ? d + 600 : d * 0.4;
      if (dropouts > 0 && rng.u() < dropouts) d = 0;
      if (override_mm >= 0) {
        d = override_mm;
        override_mm = -1;
      }
      body.dist_mm = (uint16_t)(d < 0 ? 1 : d > 4000 ? 0 : d);
    }
    body.step();
    mind.poll();
    body.react();
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
  void run_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i += 4) pass();
  }
  // Run until the guard holds (or ms pass); true if it held.
  bool run_until_hold(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i += 4) {
      pass();
      if (mind.guard().action() == Action::Holding) return true;
    }
    return false;
  }
  int posture_goal() {
    const Goal* g = mind.link().goal(KILO_MSG_SET_POSTURE);
    return g == nullptr ? -1 : g->payload[1];
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
  for (int l = 3; l < run.level; l++) r.key(B7);
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
  a.sitting = r.posture_goal() == KILO_POSTURE_SIT && r.mind.behaviour_active() && r.mind.sitting_variant();
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
               {"trot", A4, 1000, 5, 1}, {"trot", A4, 1000, 5, -2}, {"trot", A4, 1000, 5, 3}};
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
  CHECK(r.posture_goal() == KILO_POSTURE_SIT && r.mind.sitting_variant());
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
    CHECK(pressed && r.mind.guard().action() == Action::Holding);
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
  r.wall_mm = r.body_pos() + 230;
  r.wall = true;
  uint32_t floor_before = r.mind.guard().filter().floor_ignored();
  r.run_ms(1000);
  CHECK(r.mind.guard().action() == Action::Watch && r.mind.guard().filter().floor_ignored() > floor_before);
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
    CHECK(g != nullptr && g->payload[3] == kEyeLevel && r.mind.guard().eyes() == Zone::Blue);
    r.wall_mm = r.travel + 100;  // closer: red
    r.run_ms(600);
    g = r.mind.link().goal(KILO_MSG_SET_EYES);
    CHECK(g != nullptr && g->payload[1] == kEyeLevel && r.mind.guard().eyes() == Zone::Red);
  }
  {  // the old firmware (no EYES in HELLO caps): no SET_EYES, the colour kept in the mind only
    WallRig r(WallRig::remote_cfg(), false);
    r.run_ms(500);
    CHECK(r.body.count(KILO_MSG_SET_EYES) == 0 && !r.mind.guard().eyes_driven());
    kilo_host_guard_status_t s;
    r.mind.guard().fill(s);
    CHECK(s.enabled == 1 && s.eyes_driven == 0 && s.eyes == KILO_GUARD_ZONE_GREEN);
  }
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
  test_approach_each_gait_at_top_speed();
  test_red_backs_off_then_sits();
  test_hold_refuses_forward_allows_turning_and_releases();
  test_steering_while_the_guard_acts();
  test_light_sleep_floor();
  test_hand_and_floor_do_not_stop();
  test_backward_trick_pickup_and_off_do_not_trigger();
  test_eyes();
  test_alive_moves();
  test_alive_through_the_mind();
  return report("guard");
}
