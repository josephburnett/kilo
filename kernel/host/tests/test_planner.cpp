// The planner's unit tests, the cases of mind/tests/test_planner.py: the
// body model's geometry and sign conventions, the patterns' speed mapping,
// the planned steps (ranges, slew, lead, prediction, speed / direction /
// pattern changes, the stop, the gate and the catch) and the error
// estimates. Tolerances are Python's, widened where float needs it.
#include <algorithm>
#include <cmath>
#include <cstdio>
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
  double worst = max_joint_step(ticks);
  CHECKF(worst <= 9.5, "%s: joint step %.2f", what, worst);
}

// ---- the body model ----------------------------------------------------------------------

void test_model_masses_and_reach() {
  CHECK(M.total_g() >= 900 && M.total_g() <= 1100);
  real xs[4] = {0, 0, 0, 0}, zs[4] = {H, H, H, H};
  V3 c = M.com(xs, zs);
  CHECK(std::fabs(c.y) < 1e-5 && c.x > 0 && c.x < 6 && c.z > -10 && c.z < 5);
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
  CHECK(near(stability(M, xs, zs, 0xF, 0, 0), 32.5, 0.5));  // the half track
  for (int lifted = 0; lifted < 4; lifted++)  // on three feet the mass sits on the diagonal's edge
    CHECK(std::fabs(stability(M, xs, zs, (uint8_t)(0xF & ~(1 << lifted)), 0, 0)) < 3.0);
  CHECK(stability(M, xs, zs, (1 << LF) | (1 << RB), 0, 0) <= 0);  // a diagonal pair has no inside
}

// ---- patterns and speed ------------------------------------------------------------------------

void test_pattern_tables_and_default_timing() {
  Timing t = default_timing(P("trot"));
  CHECK(t.stride_mm == 25 && std::lround(t.period_ms) == 600 && std::lround(t.shift_ms) == 150);
  CHECK(near(t.speed_mm_s, 55.6, 0.1));
  const Pattern& tr = P("trot");
  CHECK(tr.n_groups == 2 && tr.groups[0][0] == LF && tr.groups[0][1] == RB && tr.groups[1][0] == RF &&
        tr.groups[1][1] == LB);
  for (const char* n : {"walk", "crawl"}) {
    const Pattern& p = P(n);
    CHECK(p.n_groups == 4 && p.groups[0][0] == LB && p.groups[1][0] == LF && p.groups[2][0] == RB &&
          p.groups[3][0] == RF);
  }
  CHECK(P("trot").swing_ms == 150 && P("walk").swing_ms == 500 && P("crawl").swing_ms == 297);
  Timing w = default_timing(P("walk"));
  CHECK(w.stride_mm == 30 && std::lround(w.period_ms) == 3200 && std::lround(w.shift_ms) == 300);
  CHECK(P("crawl").body_h_256 == 179 && P("walk").body_h_256 == 205);
  CHECK(&default_pattern() == &P("trot"));
}

void test_timing_clamps() {
  for (const Pattern& p : kPatternTable) {
    real lo, hi;
    speed_range(p, 150, &lo, &hi);
    for (real v : {lo, hi, (lo + hi) / 2}) {
      Timing t = timing(p, v);
      CHECKF(near(t.speed_mm_s, v, 1e-4 * v), "%s %g", p.name, v);
      CHECK(t.shift_ms >= shift_min_ms(p, 150) - 1e-4 && t.stance_ms <= p.stance_max_ms + 1e-4);
    }
    CHECK(near(timing(p, hi * 3).speed_mm_s, hi, 1e-4 * hi));  // capped
    Timing slow = timing(p, lo / 3);
    CHECK(slow.stride_mm == kStrideMinMm && near(slow.speed_mm_s, lo, 1e-4 * lo));
    Timing a = timing(p, -20), b = timing(p, 20);  // a magnitude
    CHECK(a.stride_mm == b.stride_mm && a.period_ms == b.period_ms && a.speed_mm_s == b.speed_mm_s);
    CHECK(timing(p, lo).period_ms - timing(p, hi).period_ms > 0);  // the shift changes, not the swing
  }
}

void test_walk_and_crawl_shift_covers_the_lag() {
  for (const char* n : {"walk", "crawl"}) {
    CHECK(shift_min_ms(P(n), 150) == 250);
    CHECK(near(timing(P(n), 1e6).shift_ms, 250, 1e-3));
  }
  CHECK(shift_min_ms(P("trot"), 150) == 60);  // three ticks on four feet
}

void test_side_strides_and_names() {
  const Pattern& p = P("trot");
  real s[2];
  side_strides(Direction::Forward, 25, p, s);
  CHECK(s[0] == 25 && s[1] == 25);
  Direction d;
  CHECK(find_direction("backward", &d) && d == Direction::Back);
  side_strides(d, 25, p, s);
  CHECK(s[0] == -25 && s[1] == -25);
  side_strides(Direction::Left, 25, p, s);
  CHECK(s[0] == -20 && s[1] == 20);  // turning left: the right side pushes forward
  side_strides(Direction::Right, 12.5, p, s);
  CHECK(s[0] == 10 && s[1] == -10);
  side_strides(Direction::March, 25, p, s);
  CHECK(s[0] == 0 && s[1] == 0);
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
    }
    CHECKF(max_joint_step(t) < M.slew_deg_per_tick, "%s: %.2f", pat.name, max_joint_step(t));
  }
}

void test_the_lean_keeps_the_mass_inside_three_feet() {
  for (const char* name : {"walk", "crawl"}) {
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
    CHECKF(any && worst > 7.0, "%s: margin %.2f", name, worst);
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
    CHECK(lead == 8);  // 150 ms
    for (size_t i = (size_t)lead; i < t.size(); i++) {
      if (!(t[i].lifting && !t[i - 1].lifting)) continue;
      double w = 0;  // the lean is in place a servo lag before every lift
      for (size_t j = i - lead; j < i; j++) w = std::fmax(w, std::fabs(t[j].plan_roll - t[i].plan_roll));
      CHECKF(w < 0.6, "%s tick %zu: roll moves %.2f in the lead", name, i, w);
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

void test_prediction_is_the_plan_a_servo_lag_earlier() {
  Planner& p = fresh("walk");
  std::vector<TickPlan> t;
  plan(&t, p, 300);
  double lag = M.lag_ticks();  // 7.5
  for (int i = 20; i < 300; i++) {
    const TickPlan &a = t[i - 7], &b = t[i - 8];
    CHECKF(near(t[i].roll, a.plan_roll + (b.plan_roll - a.plan_roll) * (lag - 7), 1e-4), "tick %d", i);
  }
}

void test_speed_changes_mid_walk_are_smooth() {
  Planner& p = fresh("trot", 0);
  std::vector<TickPlan> t;
  plan(&t, p, 200);
  p.set_speed(90);
  plan(&t, p, 200);
  p.set_speed(10);
  plan(&t, p, 300);
  CHECK(max_joint_step(t) < M.slew_deg_per_tick);
  int jump = 0;  // feet on the ground never jump
  for (size_t i = 1; i < t.size(); i++)
    for (int leg = 0; leg < 4; leg++)
      if ((t[i - 1].support >> leg & 1) && (t[i].support >> leg & 1))
        jump = std::max(jump, std::abs(t[i].x[leg] - t[i - 1].x[leg]));
  CHECK(jump <= 2);
  CHECK(near(p.timing().speed_mm_s, 10, 1e-4));
}

void test_direction_and_pattern_changes() {
  Planner& p = fresh("trot", 0);
  std::vector<TickPlan> t;
  plan(&t, p, 100);
  p.set_direction(Direction::Back);
  plan(&t, p, 100);
  CHECK(p.s_cur()[0] == -25 && p.s_cur()[1] == -25);
  p.set_direction(Direction::Left);
  plan(&t, p, 100);
  CHECK(p.s_cur()[0] == -20 && p.s_cur()[1] == 20);
  p.set_pattern(P("walk"));
  CHECK(&p.pattern() == &P("trot") && p.changing());  // the legs go home first
  int first = -1;
  bool walk_lb = false, walk_pair = false;
  for (int i = 0; i < 300; i++) {
    const TickPlan& k = p.next();
    if (&p.pattern() == &P("walk")) {
      if (first < 0) first = i;
      walk_lb = walk_lb || k.lifting == (1 << LB);
      walk_pair = walk_pair || k.n_lifting == 2;
    }
  }
  CHECK(walk_lb && !walk_pair);
  CHECK(first >= 0 && first * 20 < 1500);  // the strides ramp down and each pair steps home
  CHECK(p.s_cur()[0] == -20 && p.s_cur()[1] == 20);  // the direction carried over
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
  Planner& p = fresh("trot", 0);
  for (int i = 0; i < 100; i++) p.next();
  real before = p.stance_ticks();
  p.set_speed(90);
  p.next();
  CHECK(near(p.stance_ticks(), before / 1.02, 1e-4));
  for (int i = 0; i < 100; i++) p.next();
  CHECK(near(p.stance_ticks(), p.stance_target(), 1e-4));
}

void test_the_sent_attitude_stays_inside_feet_range() {
  // The planner clamps to +-30 deg (plan.py); pack_feet to the field's +-480.
  TickPlan t = {};
  t.pitch = 45;
  t.roll = -45;
  t.z[0] = t.z[1] = t.z[2] = t.z[3] = 76;
  kilo_feet_t f;
  pack_feet(1, t, &f);
  CHECK(f.pitch == 480 && f.roll == -480 && f.epoch == 1);
  t.tick = 0x12345;
  t.pitch = 1.53125;  // 24.5 / 16: Python's round() to even gives 24
  pack_feet(1, t, &f);
  CHECK(f.tick == 0x2345 && f.pitch == 24);
}

void test_the_lean_is_solved_during_the_swing_before() {
  for (const char* name : {"walk", "crawl"}) {
    Planner& p = fresh(name, 0);
    for (int i = 0; i < 1500; i++) p.next();
    CHECKF(p.stats().waits == 0, "%s: %u waits", name, p.stats().waits);
  }
  Planner& p = fresh("walk", 0);
  for (int i = 0; i < 1500; i++) {
    if (i % 97 == 0) p.set_speed((real)(5 + (i % 7)));
    p.next();
  }
  CHECK(p.stats().waits < 40);
}

void test_the_stop_comes_home_level() {
  Planner& p = fresh("walk", 0);
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

void test_gate_holds_the_lift_and_catch_lands_the_foot() {
  Planner& p = fresh("walk", 0);
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

void test_a_heavier_side_shifts_the_lean() {
  BodyModel right = M;
  right.battery = {150, 0, -12, -15};
  Planner& a = fresh("walk");
  real ma = 1e9;
  for (int i = 0; i < 200; i++) ma = std::fmin(ma, a.next().plan_roll);
  Planner& b = fresh("walk", 1000, -1, right);
  real mb = 1e9;
  for (int i = 0; i < 200; i++) mb = std::fmin(mb, b.next().plan_roll);
  CHECK(mb > ma);
}

// ---- the error estimates ----------------------------------------------------------------------------

int16_t a16(double deg) { return (int16_t)std::nearbyint(deg * 16); }

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
  test_walk_and_crawl_shift_covers_the_lag();
  test_side_strides_and_names();
  test_plan_fits_feet_and_the_slew();
  test_the_lean_keeps_the_mass_inside_three_feet();
  test_trot_stays_level_on_diagonal_pairs();
  test_prediction_is_the_plan_a_servo_lag_earlier();
  test_speed_changes_mid_walk_are_smooth();
  test_direction_and_pattern_changes();
  test_pattern_changes_one_after_another_stay_in_reach();
  test_the_gate_pauses_the_feet();
  test_the_stance_time_is_slewed();
  test_the_sent_attitude_stays_inside_feet_range();
  test_the_lean_is_solved_during_the_swing_before();
  test_the_stop_comes_home_level();
  test_gate_holds_the_lift_and_catch_lands_the_foot();
  test_a_heavier_side_shifts_the_lean();
  test_error_offset_lag_and_spike();
  return report("planner");
}
