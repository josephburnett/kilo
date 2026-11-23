// The planner's unit tests, the cases of mind/tests/test_planner.py: the
// body model's geometry and sign conventions, the patterns' speed mapping,
// the planned steps (ranges, slew, lead, prediction, speed / direction /
// pattern changes, the stop, the gate and the catch) and the error
// estimates. Tolerances are Python's, widened where float needs it.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include "planner/errors.h"
#include "planner/plan.h"
#include "planner/walker.h"
#include "tests/check.h"

using namespace kilo::planner;

namespace {

const BodyModel M;
const real H = M.stance_height(205);  // 76 mm: STANCE's body height

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

const Pattern& P(const char* name) { return *find_pattern(name); }

Planner& fresh(const char* pattern, uint32_t tick = 1000, real speed = -1, const BodyModel& m = M,
               const PlannerConfig& cfg = PlannerConfig()) {
  static Planner* p = nullptr;
  delete p;
  p = new Planner(m, cfg);
  p->start(P(pattern), Direction::Forward, speed, tick);
  return *p;
}

void plan(std::vector<TickPlan>* out, Planner& p, int n) {
  for (int i = 0; i < n; i++) out->push_back(p.next());
}

double joint_step_deg(const TickPlan& a, const TickPlan& b) {
  double worst = 0;
  for (int leg = 0; leg < 4; leg++) {
    real t0, t1, u0, u1;
    M.ik(leg, a.x[leg], a.z[leg], &t0, &t1);
    M.ik(leg, b.x[leg], b.z[leg], &u0, &u1);
    worst = std::fmax(worst, std::fmax(std::fabs(t0 - u0), std::fabs(t1 - u1)) * 180.0 / M_PI);
  }
  return worst;
}

double max_joint_step(const std::vector<TickPlan>& t) {
  double w = 0;
  for (size_t i = 1; i < t.size(); i++) w = std::fmax(w, joint_step_deg(t[i - 1], t[i]));
  return w;
}

int bits(int v) { return __builtin_popcount(v); }

// check_ticks of test_planner.py: ranges, reach, every joint at most 9.5 deg from the tick before.
void check_ticks(const std::vector<TickPlan>& ticks, const char* what) {
  for (const TickPlan& t : ticks)
    for (int i = 0; i < 4; i++) {
      CHECKF(t.x[i] >= -60 && t.x[i] <= 60 && t.z[i] >= 30 && t.z[i] <= 95, "%s: tick %u range", what, t.tick);
      CHECKF(M.reachable(t.x[i], t.z[i]), "%s: tick %u leg %d unreachable (%d, %d)", what, t.tick, i, t.x[i],
             t.z[i]);
      CHECKF(std::fabs(t.pitch) <= 30 && std::fabs(t.roll) <= 30, "%s: attitude", what);
    }
  double worst = max_joint_step(ticks);  // under the body's slew (test_planner.py check_ticks: why not 9.5)
  CHECKF(worst < M.slew_deg_per_tick, "%s: joint step %.2f", what, worst);
}

// ---- the body model ----------------------------------------------------------------------

void test_model_masses_and_reach() {
  CHECK(near(M.total_g(), 358.0, 0.01));  // weighed 2026-11-14, the Pi mounted
  real xs[4] = {0, 0, 0, 0}, zs[4] = {H, H, H, H};
  V3 c = M.com(xs, zs);
  // as measured by hand (7.5 ahead), moved by the correction the recordings predict best with
  CHECK(std::fabs(c.y) < 1e-5 && near(c.x, 7.5 + M.com_dx_mm, 0.2) && near(c.z, -3.0, 0.3));
  CHECK(near(H, 76.0, 0.1));
  CHECK(near(M.tau_s(80), 0.090, 0.001));  // the 90 ms tip time constant of the doc
  const real pts[4][2] = {{0, H}, {20, 70}, {-25, 60}, {10, 85}};
  for (int leg = 0; leg < 4; leg++)
    for (const auto& p : pts) {
      real t1, t2, x, z;
      M.ik(leg, p[0], p[1], &t1, &t2);
      M.fk(t1, t2, &x, &z);
      CHECKF(near(x, p[0], 1e-3) && near(z, p[1], 1e-3), "fk(ik) leg %d (%g %g) -> (%g %g)", leg, p[0], p[1], x, z);
      CHECK((t2 > 0) == (leg == LF || leg == RF));  // front knees point back, back knees forward
    }
  CHECK(!M.reachable(0, 95) && M.reachable(0, 90));
}

void test_leg_lengths_and_attitude_conventions() {
  // pitch + = nose down: the back legs are the longer; roll + = right side up: the right legs
  real xs[4] = {0, 0, 0, 0}, zs[4];
  leg_lengths(M, xs, H, 5, 0, zs);
  CHECK(zs[RB] > zs[RF] && zs[LB] > zs[LF] && near((zs[0] + zs[1] + zs[2] + zs[3]) / 4, H, 1e-4));
  leg_lengths(M, xs, H, 0, 5, zs);
  CHECK(zs[RF] > zs[LF] && zs[RB] > zs[LB]);
  const real atts[3][2] = {{3, -7}, {-4, 9}, {0, 12}};
  real xo[4] = {5, -3, 2, -8};
  for (const auto& a : atts) {
    leg_lengths(M, xo, H, a[0], a[1], zs);
    Attitude f = fit_attitude(M, xo, zs, 0xF);
    CHECKF(near(f.pitch, a[0], 1e-3) && near(f.roll, a[1], 1e-3), "fit four %g %g", f.pitch, f.roll);
    Attitude t = fit_attitude(M, xo, zs, 0xF & ~(1 << RB));
    CHECKF(near(t.pitch, a[0], 1e-3) && near(t.roll, a[1], 1e-3), "fit three %g %g", t.pitch, t.roll);
  }
  // on two feet the plane is free about their line: the previous attitude fills in
  real hz[4] = {H, H, H, H};
  Attitude d = fit_attitude(M, xs, hz, (1 << LF) | (1 << RB), {0, 0});
  CHECK(near(d.pitch, 0, 1e-4) && near(d.roll, 0, 1e-4));
  // the world: a nose-down pitch lowers the front, a positive roll the left
  real R[3][3];
  rotation(10, 0, R);
  CHECK(R[2][0] < 0);
  rotation(0, 10, R);
  CHECK(R[2][1] < 0);
  V3 com, feet[4];
  leg_lengths(M, xs, H, 0, 10, zs);
  world_points(M, xs, zs, 0, 10, &com, feet);
  CHECK(near(feet[RF].z, feet[LF].z, 1e-4));  // all four on the ground
  real feet_y = (feet[0].y + feet[1].y + feet[2].y + feet[3].y) / 4;
  V3 c0 = M.com(xs, hz);
  CHECK(near(com.y - feet_y, (H + M.foot_r_mm + c0.z) * std::sin(10 * M_PI / 180), 0.5));
}

void test_stability_margin() {
  real xs[4] = {0, 0, 0, 0}, zs[4] = {H, H, H, H};
  CHECK(near(stability(M, xs, zs, 0xF, 0, 0), M.track_mm / 2, 0.5));  // the half track
  for (int lifted = 0; lifted < 4; lifted++)  // on three feet the mass sits near the diagonal's edge
    CHECK(std::fabs(stability(M, xs, zs, (uint8_t)(0xF & ~(1 << lifted)), 0, 0)) < 4.0);
  CHECK(stability(M, xs, zs, (1 << LF) | (1 << RB), 0, 0) <= 0);  // a diagonal pair has no inside
}

// ---- patterns and speed ------------------------------------------------------------------------

void test_pattern_tables_and_default_timing() {
  Timing t = default_timing(P("trot"));  // the tables of 2026-11-16 and the trot of 2026-11-22 (test_planner.py)
  CHECK(near(t.stride_mm, 27.36, 1e-4) && std::lround(t.period_ms) == 350 && near(t.shift_ms, 20, 1e-4));
  CHECK(near(t.speed_mm_s, 144.0, 0.1));
  const Pattern& tr = P("trot");
  CHECK(tr.n_groups == 2 && tr.groups[0][0] == LF && tr.groups[0][1] == RB && tr.groups[1][0] == RF &&
        tr.groups[1][1] == LB && tr.couple_ms == 0);
  const Pattern& wk = P("walk");  // the lateral sequence one leg at a time (2026-11-22)
  CHECK(wk.n_groups == 4 && wk.group_size == 1 && wk.groups[0][0] == LB && wk.groups[1][0] == LF &&
        wk.groups[2][0] == RB && wk.groups[3][0] == RF && wk.couple_ms == 0 && !wk.lean && wk.centre_com);
  CHECK(near(step_height(wk, 40), 8, 1e-5) && near(step_height(wk, 47), 7, 1e-5) && near(step_height(wk, 54), 6, 1e-5));
  const Pattern& cr = P("crawl");
  CHECK(cr.n_groups == 4 && cr.groups[0][0] == LB && cr.groups[1][0] == LF && cr.groups[2][0] == RB &&
        cr.groups[3][0] == RF && cr.lean);
  CHECK(P("trot").swing_ms == 160 && P("walk").swing_ms == 200 && P("crawl").swing_ms == 300);
  // the walk's second version (2026-11-22): 3 ticks on four feet before a hind swing, 1 before a fore
  // swing; above 63 mm/s a 220 ms swing with 2 and 0; a 1 deg roll and 3 mm fore-aft sway
  CHECK(wk.n_handover == 4 && wk.handover[0] == 3 && wk.handover[1] == 1 && wk.handover_long[0] == 2 &&
        wk.handover_long[1] == 0 && wk.sway_roll_deg == -1 && wk.sway_dx_mm == 3);
  CHECK(swing_at(wk, 63) == 200 && swing_at(wk, (real)63.1) == 220 && near(timing(wk, 72.5).shift_ms, 20, 1e-4));
  Timing w = default_timing(P("walk"));
  CHECK(near(w.stride_mm, 44.08, 1e-4) && std::lround(w.period_ms) == 960 && near(w.shift_ms, 40, 1e-4));
  CHECK(near(w.speed_mm_s, 58.0, 0.1));
  Timing c = default_timing(P("crawl"));
  CHECK(c.stride_mm == 50 && std::lround(c.period_ms) == 2550 && near(c.speed_mm_s, 22.2, 0.1));
  CHECK(P("crawl").body_h_256 == 179 && P("walk").body_h_256 == 205);
  // walk, trot, crawl; the trot's top 207 (51.8 mm on an 11-tick swing)
  const double los[3] = {10.5, 42.1, -1}, tops[3] = {73.0, 207.0, 26.9};
  for (int i = 0; i < kPatterns; i++) {
    real lo, hi;
    speed_range(kPatternTable[i], &lo, &hi);
    CHECKF(near(hi, tops[i], 0.1) && (los[i] < 0 || near(lo, los[i], 0.1)), "%s %.2f..%.2f", kPatternTable[i].name,
           lo, hi);
  }
  CHECK(&default_pattern() == &P("trot"));
  // the trot's lift 8 mm at every stride (patterns.py step_height); the strides' slew
  CHECK(step_height(P("trot"), 50) == 8 && step_height(P("crawl"), 50) == 15);
  CHECK(P("walk").stride_slew_mm == (real)1.0 && P("trot").stride_slew_mm == (real)0.75 &&
        P("crawl").stride_slew_mm == (real)0.5);
}

void test_timing_clamps() {
  for (const Pattern& p : kPatternTable) {
    real lo, hi;
    speed_range(p, &lo, &hi);
    for (real v : {lo, hi, (lo + hi) / 2}) {
      Timing t = timing(p, v);
      CHECKF(near(t.speed_mm_s, v, 1e-4 * v), "%s %g", p.name, v);
      real long_ms = swing_at(p, v) - p.swing_ms;  // the walk's long swing takes its tick from the shift
      CHECK(t.shift_ms >= shift_min_ms(p) - long_ms - 1e-4 && t.stance_ms <= p.stance_max_ms + 1e-4);
    }
    CHECK(near(timing(p, hi * 3).speed_mm_s, hi, 1e-4 * hi));  // capped
    Timing slow = timing(p, lo / 3);
    CHECK(slow.stride_mm == kStrideMinMm && near(slow.speed_mm_s, lo, 1e-4 * lo));
    Timing a = timing(p, -20), b = timing(p, 20);  // a magnitude
    CHECK(a.stride_mm == b.stride_mm && a.period_ms == b.period_ms && a.speed_mm_s == b.speed_mm_s);
    if (is_trot(p)) {  // the trot: one tick on four feet, the swing growing with the speed
      CHECK(near(timing(p, lo).shift_ms, 20, 1e-4) && near(timing(p, hi).shift_ms, 20, 1e-4));
      CHECK(timing(p, lo).period_ms < timing(p, hi).period_ms);
    } else if (fixed_cadence(p)) {  // the walk: one cycle, the stride alone changes
      CHECK(timing(p, lo).period_ms == timing(p, hi).period_ms && near(timing(p, lo).shift_ms, 40, 1e-4) &&
            near(timing(p, hi).shift_ms, 20, 1e-4));
      CHECK(timing(p, lo).stride_mm < timing(p, hi).stride_mm);
    } else {
      CHECK(timing(p, lo).period_ms - timing(p, hi).period_ms > 0);  // the shift changes, not the swing
    }
  }
}

void test_the_crawl_shift_covers_the_lean_move() {
  CHECK(shift_min_ms(P("crawl")) == 240);  // the move (300 ms), the lift 60 ms before its end
  CHECK(near(timing(P("crawl"), 1e6).shift_ms, 240, 1e-3));
  CHECK(shift_min_ms(P("trot")) == 20 && shift_min_ms(P("walk")) == 40);  // one tick on four feet; the walk 2
}

void test_side_strides_and_names() {
  const Pattern& p = P("trot");
  real s[2];
  side_strides(Direction::Forward, 25, s);
  CHECK(s[0] == 25 && s[1] == 25);
  Direction d;
  CHECK(find_direction("backward", &d) && d == Direction::Back);
  side_strides(d, 25, s);
  CHECK(s[0] == -25 && s[1] == -25);
  side_strides(Direction::Left, 25, s);
  CHECK(s[0] == -20 && s[1] == 20);  // turning left: the right side pushes forward
  side_strides(Direction::Right, 12.5, s);
  CHECK(s[0] == 10 && s[1] == -10);
  side_strides(Direction::March, 25, s);
  CHECK(s[0] == 0 && s[1] == 0);
  // a steer bends forward and backward walks (+ left: the nose turns left), capped at the longest stride
  side_strides(Direction::Forward, 20, s, 0.25);
  CHECK(s[0] == 15 && s[1] == 25);
  side_strides(Direction::Back, 20, s, 0.5);
  CHECK(s[0] == -30 && s[1] == -10);
  side_strides(Direction::Forward, 20, s, -1);
  CHECK(s[0] == 40 && s[1] == 0);  // the inner side steps in place
  side_strides(Direction::Forward, 40, s, 0.5, 44);
  CHECK(near(s[0], 44.0 / 3, 1e-4) && near(s[1], 44, 1e-4));
  side_strides(Direction::Forward, 20, s, 3);
  CHECK(s[0] == 0 && s[1] == 40);
  side_strides(Direction::Left, 25, s, 0.5);
  CHECK(s[0] == -20 && s[1] == 20);
  CHECK(!find_direction("sideways", &d) && find_pattern("gallop") == nullptr);
  CHECK(find_direction("LEFT", &d) && d == Direction::Left && find_pattern("Trot") == &p);
}

// ---- the plan --------------------------------------------------------------------------------------

void test_plan_fits_feet_and_the_slew() {
  for (const Pattern& pat : kPatternTable) {
    Planner& p = fresh(pat.name);
    std::vector<TickPlan> t;
    plan(&t, p, 600);
    bool ticks_ok = true;
    for (size_t i = 0; i < t.size(); i++) ticks_ok = ticks_ok && t[i].tick == 1000 + i;
    CHECK(ticks_ok);
    for (const TickPlan& k : t) {
      for (int i = 0; i < 4; i++)
        CHECKF(k.x[i] >= -60 && k.x[i] <= 60 && k.z[i] >= 30 && k.z[i] <= 95 && M.reachable(k.x[i], k.z[i]),
               "%s tick %u leg %d", pat.name, k.tick, i);
      CHECK(bits(k.support) >= (pat.lean ? 3 : 2));
      if (bits(k.support) == 2) CHECK(k.support == 0x5 || k.support == 0xA);  // two feet: a diagonal pair
    }
    CHECKF(max_joint_step(t) < M.slew_deg_per_tick, "%s: %.2f", pat.name, max_joint_step(t));
  }
}

// test_planner.py fast_bound(): 8.3 deg a tick (the floor ran 414 deg/s at 180 mm/s); the trot above
// its 8-tick swing (150 mm/s) and steering 9.7 (485 deg/s, under the body's 10; test_trot.py BOUND).
double fast_bound(const char* name, real v, real steer = 0) {
  if (std::strcmp(name, "walk") == 0) return 9.5;  // up to 471 deg/s backward, steering (test_planner.py)
  return std::strcmp(name, "trot") == 0 && (v > (real)150 || steer != 0) ? 9.7 : 8.3;
}

void test_every_speed_of_the_fast_gaits_stays_inside_the_slew() {
  // slowest, default and top, forward, back and turning: inside fast_bound() (test_planner.py)
  for (const char* name : {"walk", "trot"}) {
    real lo, hi;
    speed_range(P(name), &lo, &hi);
    for (Direction d : {Direction::Forward, Direction::Back, Direction::Left})
      for (real v : {lo, default_speed(P(name)), hi}) {
        Planner p;
        p.start(P(name), d, v, 1000);
        std::vector<TickPlan> t;
        plan(&t, p, 300);
        CHECKF(max_joint_step(t) < fast_bound(name, v), "%s %g: %.2f", name, v, max_joint_step(t));
      }
  }
}

void test_steering_bends_the_path_inside_the_reach_and_the_slew() {
  // test_planner.py: the steer never takes a side past the longest stride, so every joint stays as at
  // the top speed (the crawl backward at its top asked 11.9 deg a tick before steering existed)
  for (const Pattern& pat : kPatternTable)
    for (Direction d : {Direction::Forward, Direction::Back}) {
      real lo, hi;
      speed_range(pat, &lo, &hi);
      for (real v : {default_speed(pat), hi})
        for (real steer : {(real)0.25, (real)0.5, (real)1.0, (real)-1.0}) {
          Planner p;
          p.start(pat, d, v, 1000, -1, steer);
          std::vector<TickPlan> t;
          plan(&t, p, 400);
          real sl = p.s_cur()[0], sr = p.s_cur()[1];
          CHECK(std::fmax(std::fabs(sl), std::fabs(sr)) <= pat.stride_mm + 1e-4);
          CHECKF((sr - sl) * steer > 0 && near(std::fabs((sr - sl) / (sr + sl)), std::fabs(steer), 1e-4),
                 "%s %g %g: %g %g", pat.name, (double)v, (double)steer, (double)sl, (double)sr);
          for (const TickPlan& k : t)
            for (int i = 0; i < 4; i++) CHECK(k.x[i] >= -60 && k.x[i] <= 60 && M.reachable(k.x[i], k.z[i]));
          double bound = fixed_cadence(pat) ? fast_bound(pat.name, v, steer)
                                            : (d == Direction::Back ? 12.0 : (double)M.slew_deg_per_tick);
          CHECKF(max_joint_step(t) < bound, "%s %g %g: %.2f", pat.name, (double)v, (double)steer, max_joint_step(t));
        }
    }
}

void test_a_turn_in_place_goes_straight_within_a_second() {
  // the remote's B7 / B5 while turning: the trot's strides 0.75 mm a tick, straight in under 1.4 s;
  // the walk's 1.0 mm (strides to 44 mm), under 1.7 s
  for (const char* name : {"walk", "trot"})
    for (Direction to : {Direction::Forward, Direction::Back}) {
      Planner p;
      p.start(P(name), Direction::Left, -1, 0);
      std::vector<TickPlan> t;
      plan(&t, p, 150);
      p.set_direction(to);
      int n = 0;
      real want[2];
      side_strides(to, p.timing().stride_mm, want);
      while ((p.s_cur()[0] != want[0] || p.s_cur()[1] != want[1]) && n < 400) {
        t.push_back(p.next());
        n++;
      }
      CHECKF(n * 20 < (std::strcmp(name, "walk") == 0 ? 1700 : 1400), "%s: %d ticks", name, n);
      plan(&t, p, 100);
      check_ticks(t, name);
      CHECKF(max_joint_step(t) < fast_bound(name, 0), "%s: %.2f", name, max_joint_step(t));
    }
}

void test_out_of_the_crawl_the_feet_wait_for_the_level_out() {
  // test_planner.py: the strides stay at 0 while the first walk slot levels the crawl's lean
  Planner& p = fresh("crawl", 0);
  for (int i = 0; i < 200; i++) p.next();
  p.set_pattern(P("walk"));
  int n = 0;
  while (&p.pattern() != &P("walk") && n++ < 1000) p.next();
  int held = 0;
  for (int i = 0; i < 15; i++) {
    p.next();
    if (p.s_cur()[0] == 0 && p.s_cur()[1] == 0) held++;
  }
  CHECKF(held == 15, "%d of 15 ticks with the strides at 0", held);
}

void test_the_walk_is_the_lateral_sequence_evenly_spaced() {
  // test_planner.py: one leg at a time, LB LF RB RF (backward too), a footfall 11 ticks after a hind
  // foot's and 13 after a fore foot's, at least three feet down; forward the centre of mass at least
  // 0.2 mm inside the three feet (the plan's margin) at every speed
  for (Direction d : {Direction::Forward, Direction::Back}) {
    const int order[4] = {LB, LF, RB, RF};
    real lo, hi;
    speed_range(P("walk"), &lo, &hi);
    for (real v : {lo * 2, default_speed(P("walk")), hi}) {
      Planner p;
      p.start(P("walk"), d, v, 1000);
      std::vector<TickPlan> t;
      plan(&t, p, 500);
      t.erase(t.begin(), t.begin() + 150);
      std::vector<int> seen;
      double worst = 1e9;
      for (const TickPlan& k : t) {
        CHECKF((k.lifting & (k.lifting - 1)) == 0, "tick %u lifting %u", (unsigned)k.tick, (unsigned)k.lifting);
        if (k.lifting) {
          int leg = 0;
          while (!(k.lifting >> leg & 1)) leg++;
          if (seen.empty() || seen.back() != leg) seen.push_back(leg);
          worst = std::fmin(worst, (double)k.margin);
        }
      }
      size_t i = 0;
      while (i < seen.size() && seen[i] != order[0]) i++;
      CHECK(i + 4 <= seen.size());
      for (size_t j = 0; j < 4 && i + j < seen.size(); j++) CHECK(seen[i + j] == order[j]);
      std::vector<uint32_t> downs;
      std::vector<int> landed;
      for (size_t n = 1; n < t.size(); n++)
        if (__builtin_popcount(t[n].support) > __builtin_popcount(t[n - 1].support)) {
          downs.push_back(t[n].tick);
          int leg = 0;
          while (!(t[n - 1].lifting >> leg & 1)) leg++;
          landed.push_back(leg);
        }
      for (size_t n = 1; n < downs.size(); n++) {
        // forward 11 ticks after a hind foot, 13 after a fore foot; backward evenly, 12
        uint32_t want = d == Direction::Back ? 12 : (landed[n - 1] == LB || landed[n - 1] == RB) ? 11 : 13;
        CHECKF(downs[n] - downs[n - 1] == want, "%u", (unsigned)(downs[n] - downs[n - 1]));
      }
      CHECKF(worst > (d == Direction::Forward ? 0.2 : -7.5), "%g: margin %g", (double)v, worst);
    }
  }
}

void test_the_lean_keeps_the_mass_inside_three_feet() {
  for (const char* name : {"crawl"}) {
    Planner& p = fresh(name);
    std::vector<TickPlan> t;
    plan(&t, p, 600);
    real worst = 1e9;
    bool any = false;
    for (const TickPlan& k : t)
      if (k.n_lifting == 1) {
        any = true;
        worst = std::fmin(worst, k.margin);
      }
    // margin_mm 14, linearised, the move's last three ticks after the lift counted
    CHECKF(any && worst > 11.0, "%s: margin %.2f", name, worst);
    PlannerConfig flat;
    flat.lean = false;
    Planner& q = fresh(name, 1000, -1, M, flat);
    std::vector<TickPlan> f;
    plan(&f, q, 600);
    real fw = 1e9;
    for (const TickPlan& k : f)
      if (k.n_lifting == 1) fw = std::fmin(fw, k.margin);
    CHECKF(fw < 0, "%s without the lean: %.2f", name, fw);  // over the edge
    Planner& r = fresh(name);
    t.clear();
    plan(&t, r, 600);
    int lead = r.lead();
    CHECK(lead == -3);  // the lift 60 ms before the move's end, in its cosine tail
    for (size_t i = 1; i + (size_t)-lead < t.size(); i++) {
      if (!(t[i].lifting && !t[i - 1].lifting)) continue;
      const TickPlan& held = t[i - lead];
      double w = 0;  // about a degree of roll left at the lift at most
      for (size_t j = i; j < i - lead; j++) w = std::fmax(w, std::fabs(t[j].plan_roll - held.plan_roll));
      CHECKF(w < 1.25, "%s tick %zu: roll moves %.2f after the lift", name, i, w);
      // then held to the touchdown: plan_roll is the plane through the whole-mm feet, a foot's z rounding
      // the other way moves it up to 0.8 deg (1 mm over the 70 mm track); test_planner.py the same
      for (size_t j = i - lead + 1; j < t.size() && t[j].lifting; j++)
        CHECKF(std::fabs(t[j].plan_roll - held.plan_roll) < 0.85, "%s tick %zu: the lean moves %.2f", name, j,
               std::fabs(t[j].plan_roll - held.plan_roll));
    }
    for (const TickPlan& k : t)  // the mass goes away from the lifting leg (roll + = mass left)
      if (k.n_lifting == 1) {
        bool left = (k.lifting & ((1 << LF) | (1 << LB))) != 0;
        CHECKF((k.plan_roll < 0) == left, "%s tick %u roll %.2f", name, k.tick, k.plan_roll);
      }
  }
}

void test_trot_stays_level_on_diagonal_pairs() {
  Planner& p = fresh("trot");
  std::vector<TickPlan> t;
  plan(&t, p, 600);
  bool a = false, b = false;
  for (const TickPlan& k : t) {
    CHECK(k.lifting == 0 || k.lifting == ((1 << LF) | (1 << RB)) || k.lifting == ((1 << RF) | (1 << LB)));
    CHECK(std::fabs(k.pitch) < 0.5 && std::fabs(k.roll) < 0.5);
    a = a || k.lifting == ((1 << LF) | (1 << RB));
    b = b || k.lifting == ((1 << RF) | (1 << LB));
  }
  CHECK(a && b);
}

void test_prediction_is_the_plan_through_the_servos_response() {
  // the plan's attitude a dead time earlier (2.5 ticks), through a 70 ms first-order response
  Planner& p = fresh("crawl");
  std::vector<TickPlan> t;
  plan(&t, p, 300);
  double lag = M.lag_ticks();
  CHECK(near(lag, 2.5, 1e-9) && M.servo_tau_ms == 70);
  double a = 1 - std::exp(-20.0 / 70.0), y = 0;
  for (int i = 0; i < 300; i++) {
    double u0 = i >= 2 ? t[i - 2].plan_roll : 0, u1 = i >= 3 ? t[i - 3].plan_roll : 0;
    y += (u0 + (u1 - u0) * (lag - 2) - y) * a;
    CHECKF(near(t[i].roll, y, 1e-3), "tick %d: %.4f vs %.4f", i, t[i].roll, y);
  }
}

void test_speed_changes_mid_walk_are_smooth() {
  // Down from the trot's default (144 mm/s) to 90 and 10, up to its top (test_planner.py).
  Planner& p = fresh("trot", 0);
  std::vector<TickPlan> t;
  plan(&t, p, 200);
  p.set_speed(90);
  plan(&t, p, 200);
  p.set_speed(10);
  plan(&t, p, 300);
  real lo0, hi0;
  speed_range(P("trot"), &lo0, &hi0);
  CHECK(near(p.timing().speed_mm_s, lo0, 1e-4));  // 42: the trot's slowest
  p.set_speed(250);
  plan(&t, p, 300);
  CHECK(max_joint_step(t) < M.slew_deg_per_tick);
  int jump = 0;  // feet on the ground never jump: at most the top speed's slide a tick, plus the rounding
  for (size_t i = 1; i < t.size(); i++)
    for (int leg = 0; leg < 4; leg++)
      if ((t[i - 1].support >> leg & 1) && (t[i].support >> leg & 1))
        jump = std::max(jump, std::abs(t[i].x[leg] - t[i - 1].x[leg]));
  real lo, hi;
  speed_range(P("trot"), &lo, &hi);
  CHECKF(jump <= hi * kTickMs / 1000 + 1, "jump %d", jump);
}

void test_direction_and_pattern_changes() {
  Planner& p = fresh("trot", 0);
  std::vector<TickPlan> t;
  plan(&t, p, 100);
  p.set_direction(Direction::Back);
  plan(&t, p, 150);  // 0.5 mm a tick: 110 ticks from +27.4 to -27.4
  CHECK(near(p.s_cur()[0], -27.36, 1e-3) && near(p.s_cur()[1], -27.36, 1e-3));
  p.set_direction(Direction::Left);
  plan(&t, p, 130);  // the right side from -27.4 to +21.9
  CHECK(near(p.s_cur()[0], -21.888, 1e-3) && near(p.s_cur()[1], 21.888, 1e-3));
  p.set_pattern(P("walk"));
  CHECK(&p.pattern() == &P("trot") && p.changing());
  int first = -1;
  bool walk_pair = false;
  for (int i = 0; i < 300; i++) {
    const TickPlan& k = p.next();
    if (&p.pattern() == &P("walk")) {
      if (first < 0) first = i;
      walk_pair = walk_pair || (k.lifting & (k.lifting - 1)) != 0;
    }
  }
  CHECK(!walk_pair);  // one leg at a time
  CHECKF(first >= 0 && first * 20 < 800, "%d", first);  // through the way home (2026-11-22): 0.7 s from this turn
  CHECK(near(p.s_cur()[0], -0.8 * 54, 1e-3) && near(p.s_cur()[1], 0.8 * 54, 1e-3));  // turning left, its top stride
  p.set_pattern(P("crawl"));  // into the crawl: the legs go home first, as before
  int cf = -1;
  bool home_ok = false, one = true;
  for (int i = 0; i < 400; i++) {
    const TickPlan& k = p.next();
    if (&p.pattern() == &P("crawl")) {
      if (cf < 0) {
        cf = i;
        home_ok = true;
        for (int leg = 0; leg < 4; leg++) home_ok = home_ok && std::abs(k.x[leg]) <= 2;
      }
      one = one && k.n_lifting <= 1;
    }
  }
  CHECK(cf >= 0 && home_ok && one);
}

void test_pattern_changes_one_after_another_stay_in_reach() {
  const char* seqs[4][3] = {{"walk", "trot", "crawl"}, {"trot", "walk", "trot"}, {"crawl", "walk", "trot"},
                            {"walk", "crawl", "walk"}};
  for (const auto& seq : seqs)
    for (int t0 : {100, 157, 211})
      for (int gap : {10, 60, 140}) {
        Planner& p = fresh(seq[0], 0);
        std::vector<TickPlan> t;
        plan(&t, p, t0);
        p.set_pattern(P(seq[1]));
        plan(&t, p, gap);
        p.set_pattern(P(seq[2]));
        plan(&t, p, 500);
        char what[64];
        std::snprintf(what, sizeof what, "%s>%s>%s at %d gap %d", seq[0], seq[1], seq[2], t0, gap);
        check_ticks(t, what);
        CHECKF(&p.pattern() == &P(seq[2]), "%s: ends in %s", what, p.pattern().name);
      }
}

void test_the_gate_pauses_the_feet() {
  Planner& p = fresh("trot", 0, 2 * default_speed(P("trot")));
  std::vector<TickPlan> t;
  plan(&t, p, 40);
  p.observe(6, 0);
  while (p.stats().holds == 0) t.push_back(p.next());
  std::vector<TickPlan> held = {t.back()};
  plan(&held, p, 24);
  CHECK(p.stats().holds == 25);
  for (const TickPlan& k : held) {
    bool same = true;
    for (int i = 0; i < 4; i++) same = same && k.x[i] == held[0].x[i] && k.z[i] == held[0].z[i];
    CHECK(same && !k.lifting);
  }
  t.insert(t.end(), held.begin() + 1, held.end());
  t.push_back(p.next());
  CHECK(p.last().lifting);  // hold_max: then it lifts anyway
  p.observe(0, 0);
  plan(&t, p, 100);
  check_ticks(t, "gate");
}

void test_the_stance_time_is_slewed() {
  Planner& p = fresh("crawl", 0, 10);
  for (int i = 0; i < 100; i++) p.next();
  real before = p.stance_ticks();
  p.set_speed(default_speed(P("crawl")));  // faster: a shorter stance
  p.next();
  CHECK(near(p.stance_ticks(), before / 1.02, 1e-4 * before));
  for (int i = 0; i < 200; i++) p.next();
  CHECK(near(p.stance_ticks(), p.stance_target(), 1e-4));
}

void test_the_sent_attitude_stays_inside_pose_range() {
  // The planner clamps to +-30 deg (plan.py); pack_pose to the field's +-480.
  TickPlan t = {};
  t.pitch = 45;
  t.roll = -45;
  t.x[1] = -12;
  t.z[0] = t.z[1] = t.z[2] = t.z[3] = 76;
  kilo_pose_t f;
  pack_pose(1, t, &f);
  CHECK(f.pitch == 480 && f.roll == -480 && f.epoch == 1);
  // four feet in whole mm as q2, the head left to the body
  CHECK(f.flags == 0 && f.leg[1] == 76 * 4 && f.leg[2] == -12 * 4 && f.head[0] == KILO_HEAD_NOT_HELD &&
        f.head[2] == KILO_HEAD_NOT_HELD);
  t.tick = 0x12345;
  t.pitch = 1.53125;  // 24.5 / 16: Python's round() to even gives 24
  pack_pose(1, t, &f);
  CHECK(f.tick == 0x2345 && f.pitch == 24);
}

void test_the_lean_is_solved_during_the_swing_before() {
  for (const char* name : {"crawl"}) {
    Planner& p = fresh(name, 0);
    for (int i = 0; i < 1500; i++) p.next();
    CHECKF(p.stats().waits == 0, "%s: %u waits", name, p.stats().waits);
  }
  Planner& p = fresh("crawl", 0);
  for (int i = 0; i < 1500; i++) {
    if (i % 97 == 0) p.set_speed((real)(5 + (i % 7)));
    p.next();
  }
  CHECK(p.stats().waits < 40);
}

void test_the_stop_comes_home_level() {
  for (const char* name : {"walk", "trot", "crawl"}) {
  Planner& p = fresh(name, 0);
  for (int i = 0; i < 300; i++) p.next();
  p.stop();
  int n = 0;
  TickPlan last = {};
  while (!p.done() && n < 1000) {
    last = p.next();
    n++;
  }
  CHECK(p.done() && n < 600);
  for (int i = 0; i < 4; i++) CHECK(std::abs(last.x[i]) <= 1 && std::fabs(last.z[i] - H) <= 1);
  CHECK(std::fabs(last.plan_roll) < 0.5 && std::fabs(last.plan_pitch) < 0.5 && last.support == 0xF);
  }
}

void test_gate_holds_the_lift_and_catch_lands_the_foot() {
  Planner& p = fresh("crawl", 0);
  p.observe(0, 8);  // the body is 8 deg off the plan: no lift
  bool lifted = false;
  for (int i = 0; i < 40; i++) lifted = lifted || p.next().lifting;
  CHECK(!lifted && p.stats().holds > 0);
  p.observe(0, 0);
  while (!p.last().lifting) p.next();
  CHECK(p.last().lifting == (1 << LB));  // tipping toward LB: + roll (left down), - pitch (tail down)
  p.next();
  p.observe(-3, 6);
  TickPlan down = {};
  for (int i = 0; i < 4; i++) down = p.next();
  CHECK(p.stats().catches == 1);
  CHECK(!down.lifting && down.support == 0xF);  // down in three ticks, where it was
}

void test_the_walk_catches_its_swinging_leg() {
  Planner& p = fresh("walk", 0);
  for (int i = 0; i < 100; i++) p.next();
  while (!p.has_last() || p.last().lifting != (1 << LB)) p.next();
  p.next();
  p.observe(0, 8);  // tipping left: toward LB
  TickPlan down = {};
  for (int i = 0; i < 4; i++) down = p.next();
  p.observe(0, 0);
  CHECK(p.stats().catches == 1);
  CHECK(!(down.lifting >> LB & 1) && down.support == 0xF);  // LB down in three ticks, where it was
  std::vector<TickPlan> rest;
  plan(&rest, p, 30);
  bool next_leg = false;
  for (const TickPlan& k : rest) next_leg = next_leg || k.lifting == (1 << LF);
  CHECK(next_leg);
  check_ticks(rest, "walk catch");
}

void test_a_heavier_side_shifts_the_lean() {
  BodyModel right = M;
  right.battery = {150, 0, -12, -15};
  Planner& a = fresh("crawl");
  real ma = 1e9;
  for (int i = 0; i < 200; i++) ma = std::fmin(ma, a.next().plan_roll);
  Planner& b = fresh("crawl", 1000, -1, right);
  real mb = 1e9;
  for (int i = 0; i < 200; i++) mb = std::fmin(mb, b.next().plan_roll);
  CHECK(mb > ma);
}

// ---- the error estimates ----------------------------------------------------------------------------

int16_t a16(double deg) { return (int16_t)std::nearbyint(deg * 16); }

// The trot's long swing (test_trot.py; the golden vectors compare it tick for tick): the remote's +4
// and +5 and 160 mm/s from standing, the swing a tick a step, every tick in reach and under the trot's
// bound (9.7: test_trot.py BOUND), the quick stop from every phase at 200 mm/s done in 1.3 s, and
// into the walk once its swing is 8 ticks again.
void test_the_trots_long_swing() {
  CHECK(is_trot(P("trot")) && !is_trot(P("walk")) && find_pattern("longtrot") == nullptr);
  const real speeds[3] = {160, 180, 201.6};
  const int swings[3] = {9, 10, 11};
  Planner& p = fresh("trot", 1000, speeds[0]);
  std::vector<TickPlan> t;
  int last_w = p.swing_ticks();
  CHECK(last_w == 9);
  for (int k = 0; k < 3; k++) {
    p.set_speed(speeds[k]);
    for (int i = 0; i < 150; i++) {
      t.push_back(p.next());
      CHECKF(p.swing_ticks() - last_w <= 1, "%g mm/s: swing %d after %d", (double)speeds[k], p.swing_ticks(), last_w);
      last_w = p.swing_ticks();
    }
    CHECKF(p.swing_ticks() == swings[k], "%g mm/s: swing %d", (double)speeds[k], p.swing_ticks());
    Timing tm = p.timing();
    CHECKF(near(tm.speed_mm_s, speeds[k], 0.01) && near(tm.stride_mm, p.s_cur()[0], 0.01), "%g mm/s: %g mm/s",
           (double)speeds[k], (double)tm.speed_mm_s);
  }
  check_ticks(t, "the trot's long swing");
  CHECKF(max_joint_step(t) < 9.7, "the trot's long swing: %.2f", max_joint_step(t));
  for (int at = 0; at < 24; at++) {
    Planner& q = fresh("trot", 1000, 200);
    std::vector<TickPlan> u;
    plan(&u, q, 200 + at);
    q.halt();
    int n = 0;
    for (; !q.done() && n < 400; n++) u.push_back(q.next());
    CHECKF(q.done() && n * 0.02 < 1.3, "halt at %d: %d ticks", at, n);
    std::vector<TickPlan> tail(u.begin() + 150, u.end());
    check_ticks(tail, "trot halt");
    CHECKF(max_joint_step(tail) < 9.7, "trot halt at %d: %.2f", at, max_joint_step(tail));
  }
  // out of the long swing into the walk: once the swing is 8 ticks again
  Planner& r = fresh("trot", 1000, 200);
  std::vector<TickPlan> u;
  plan(&u, r, 150);
  r.set_pattern(P("walk"));
  r.set_speed(80);
  int n = 0;
  for (; r.trot() && n < 200; n++) {
    u.push_back(r.next());
    if (r.trot()) last_w = r.swing_ticks();
  }
  CHECKF(!r.trot() && last_w == 8 && n < 120, "into the walk after %d ticks, swing %d", n, last_w);
  plan(&u, r, 100);
  check_ticks(u, "trot to walk");
}

void test_error_offset_lag_and_spike() {
  ErrorTracker tr;
  for (int t = 0; t < 300; t++) {
    double ref = 8 * std::sin(2 * M_PI * t / 75);
    double meas = 8 * std::sin(2 * M_PI * (t - 2) / 75) + 1.5;  // 2 ticks (40 ms) late, 1.5 deg off
    tr.add(t, a16(0.4), a16(meas), 0, a16(ref), 0, a16(-0.5), nullptr);
  }
  ErrorEstimate e;
  CHECK(tr.estimate(&e));
  CHECK(e.has_lag && e.lag_ms == 40 && e.lag_quality > 0.95);
  CHECK(near(e.offset_roll_deg, 1.5, 0.2) && near(e.offset_pitch_deg, 0.4, 0.05));
  CHECK(near(e.corr_roll_deg, -0.5, 0.05));
  CHECK(near(tr.offset().pitch, 0.4, 0.05));
  ErrorTracker none;
  CHECK(!none.estimate(&e));
  ErrorTracker flat;  // a plan that does not move (the trot) gives no lag
  for (int t = 0; t < 100; t++) flat.add(t, 0, a16(0.3), 0, 0, 0, 0, nullptr);
  CHECK(flat.estimate(&e) && !e.has_lag);
  ErrorTracker gap;  // ticks with holes (a TELEMETRY lost): pairs only where both exist
  for (int t = 0; t < 300; t++) {
    if (t % 11 == 5) continue;
    double ref = 8 * std::sin(2 * M_PI * t / 75), meas = 8 * std::sin(2 * M_PI * (t - 3) / 75);
    gap.add(t, 0, a16(meas), 0, a16(ref), 0, 0, nullptr);
  }
  CHECK(gap.estimate(&e) && e.has_lag && e.lag_ms == 60);
}

}  // namespace

int main() {
  test_model_masses_and_reach();
  test_leg_lengths_and_attitude_conventions();
  test_stability_margin();
  test_pattern_tables_and_default_timing();
  test_timing_clamps();
  test_the_crawl_shift_covers_the_lean_move();
  test_side_strides_and_names();
  test_plan_fits_feet_and_the_slew();
  test_every_speed_of_the_fast_gaits_stays_inside_the_slew();
  test_steering_bends_the_path_inside_the_reach_and_the_slew();
  test_a_turn_in_place_goes_straight_within_a_second();
  test_out_of_the_crawl_the_feet_wait_for_the_level_out();
  test_the_walk_is_the_lateral_sequence_evenly_spaced();
  test_the_lean_keeps_the_mass_inside_three_feet();
  test_trot_stays_level_on_diagonal_pairs();
  test_prediction_is_the_plan_through_the_servos_response();
  test_speed_changes_mid_walk_are_smooth();
  test_direction_and_pattern_changes();
  test_pattern_changes_one_after_another_stay_in_reach();
  test_the_gate_pauses_the_feet();
  test_the_stance_time_is_slewed();
  test_the_sent_attitude_stays_inside_pose_range();
  test_the_lean_is_solved_during_the_swing_before();
  test_the_stop_comes_home_level();
  test_gate_holds_the_lift_and_catch_lands_the_foot();
  test_the_walk_catches_its_swinging_leg();
  test_a_heavier_side_shifts_the_lean();
  test_the_trots_long_swing();
  test_error_offset_lag_and_spike();
  return report("planner");
}
