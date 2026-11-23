// The engine's step clock and solver (engine/clock.h, engine/solve.h) on
// their own, tick by tick, no body: every gait walks with its lifts in
// order and its weight target kept, stays inside POSE's ranges and the
// body's slew; speed, steer, direction and cadence change without a stop;
// the planned stop and the quick stop end on four feet at home; a gait
// change goes through home; the clock waits for the gate and for the weight;
// a catch lands the foot; a copy of a solver solves the same ticks (the
// walker's quick stop relies on it). How well the walks rock and predict
// is the judge's (mind/tests/enginejudge.py), not this file's.
#include <cmath>
#include <cstdio>
#include <cstring>
#include "engine/solve.h"
#include "tests/check.h"

using namespace kilo;
using namespace kilo::engine;

namespace {

Solver g_s;  // static storage: a solver is a few KB

int bits(unsigned v) {
  int n = 0;
  for (; v; v >>= 1) n += v & 1;
  return n;
}

// The fastest joint (deg) between two ticks' feet.
double joint_step(const TickPlan& a, const TickPlan& b, const planner::BodyModel& m) {
  double worst = 0;
  for (int leg = 0; leg < kLegs; leg++) {
    real a1, a2, b1, b2;
    m.ik(leg, (real)a.xq[leg] / 4, (real)a.zq[leg] / 4, &a1, &a2);
    m.ik(leg, (real)b.xq[leg] / 4, (real)b.zq[leg] / 4, &b1, &b2);
    worst = std::fmax(worst, std::fmax(std::fabs(a1 - b1), std::fabs(a2 - b2)) * 180.0 / M_PI);
  }
  return worst;
}

struct Walked {
  int ticks = 0;
  int lifts[kLegs] = {};  // lift-offs per leg
  int order[16] = {};     // the first lift-offs, in order
  int n_order = 0;
  int most_up = 0;
  double min_margin = 1e9;  // on ticks with one foot up, after the warm-up
  double fastest = 0;
  bool in_range = true;
  double dist = 0;          // how far the feet on the ground carried the body (mm)
};

Walked walk(Solver& s, int ticks, int warm = 100) {
  Walked w;
  TickPlan prev = {};
  bool has_prev = false;
  for (int t = 0; t < ticks && !s.done(); t++) {
    TickPlan p = s.next();
    w.ticks++;
    for (int leg = 0; leg < kLegs; leg++) {
      if (p.xq[leg] < -480 || p.xq[leg] > 480 || p.zq[leg] < 60 || p.zq[leg] > 440) w.in_range = false;
      if (has_prev && (p.lifting & (1u << leg)) && !(prev.lifting & (1u << leg))) {
        w.lifts[leg]++;
        if (w.n_order < 16) w.order[w.n_order++] = leg;
      }
    }
    w.most_up = bits(p.lifting) > w.most_up ? bits(p.lifting) : w.most_up;
    if (t >= warm && bits(p.lifting) == 1) w.min_margin = std::fmin(w.min_margin, (double)p.margin);
    if (has_prev) {
      w.fastest = std::fmax(w.fastest, joint_step(prev, p, s.model()));
      double d = 0;
      int n = 0;
      for (int leg = 0; leg < kLegs; leg++)
        if (prev.support & p.support & (1u << leg)) {
          d += (prev.xq[leg] - p.xq[leg]) / 4.0;
          n++;
        }
      if (n) w.dist += d / n;
    }
    prev = p;
    has_prev = true;
  }
  return w;
}

void test_clock_lifts_each_leg_once_a_cycle() {
  StepClock c;
  const real off[kLegs] = {(real)0.25, (real)0.75, (real)0.5, 0};
  c.start(off, 48, (real)-0.01);
  int lifts[kLegs] = {};
  int first_lb = -1;
  for (int t = 0; t < 480; t++) {
    uint8_t m = c.advance();
    for (int leg = 0; leg < kLegs; leg++)
      if (m & (1u << leg)) lifts[leg]++;
    if ((m & 8) && first_lb < 0) first_lb = t;
    CHECKF(bits(m) <= 1, "tick %d lifts %x", t, m);
  }
  for (int leg = 0; leg < kLegs; leg++) CHECKF(lifts[leg] == 10, "leg %d lifted %d times in 10 cycles", leg, lifts[leg]);
  CHECKF(first_lb == 0, "LB first at %d", first_lb);
  CHECK(c.advance(true) == 0);  // waiting: nothing lifts, the phase stands
}

void test_every_gait_walks_inside_its_limits() {
  for (const Gait& g : kGaitTable) {
    Solver& s = g_s;
    s.start(g, Direction::Forward, -1, 1000);
    Walked w = walk(s, 900);
    Timing tm = s.timing();
    for (int leg = 0; leg < kLegs; leg++)
      CHECKF(w.lifts[leg] >= 900 / (int)(tm.cycle_ticks + 8) - 1, "%s: leg %d lifted %d times", g.name, leg,
             w.lifts[leg]);
    CHECKF(w.in_range, "%s: a foot outside POSE's range", g.name);
    CHECKF(w.fastest <= 10.0, "%s: a joint turned %.1f deg in a tick (the body's slew: 10)", g.name, w.fastest);
    CHECKF(w.most_up == g.group_size, "%s: %d feet up at once", g.name, w.most_up);
    if (g.margin_mm >= 0)
      CHECKF(w.min_margin >= g.margin_mm - 1.5, "%s: the weight %.2f mm inside three feet (target %.1f)", g.name,
             w.min_margin, (double)g.margin_mm);
    double v = w.dist / ((w.ticks - 1) * 0.02);
    CHECKF(v > 0.6 * tm.speed_mm_s && v < 1.2 * tm.speed_mm_s, "%s: %.1f mm/s for %.1f", g.name, v,
           (double)tm.speed_mm_s);
  }
  // the walk's lateral sequence: LB, LF, RB, RF
  Solver& s = g_s;
  s.start(*find_gait("walk"), Direction::Forward, -1, 0);
  Walked w = walk(s, 200);
  const int want[4] = {LB, LF, RB, RF};
  for (int i = 0; i < 8 && i < w.n_order; i++) CHECKF(w.order[i] == want[i % 4], "lift %d: leg %d", i, w.order[i]);
}

void test_speed_cadence_and_steer_change_without_a_stop() {
  Solver& s = g_s;
  s.start(*find_gait("walk"), Direction::Forward, 40, 0);
  walk(s, 200);
  real stride0 = s.strides()[0];
  s.set_speed(66);
  Walked w = walk(s, 300, 0);
  CHECK(!s.stopping() && !s.done());
  CHECKF(s.strides()[0] > stride0 + 10, "the stride %.1f -> %.1f", (double)stride0, (double)s.strides()[0]);
  CHECKF(w.min_margin >= 2.0, "margin %.2f while speeding up", w.min_margin);
  real cycle0 = s.timing().cycle_ticks;
  s.set_cadence((real)1.3);
  CHECKF(s.timing().cycle_ticks < cycle0 - 5, "cadence 1.3: cycle %.1f from %.1f", (double)s.timing().cycle_ticks,
         (double)cycle0);
  w = walk(s, 300, 100);
  CHECKF(w.min_margin >= 2.0 && w.fastest <= 10.0, "cadence 1.3: margin %.2f, fastest %.1f", w.min_margin, w.fastest);
  s.set_cadence(0);
  s.set_steer((real)0.5);
  walk(s, 200);
  CHECKF(s.strides()[0] < s.strides()[1] - 5, "steer left: strides %.1f %.1f", (double)s.strides()[0],
         (double)s.strides()[1]);
  s.set_direction(Direction::Back);
  walk(s, 200);
  CHECK(s.strides()[0] < 0 && s.strides()[1] < 0);
  s.set_direction(Direction::Left);
  walk(s, 200);
  CHECK(s.strides()[0] < 0 && s.strides()[1] > 0);
}

// `kept`: the legs the quick stop caught in the air (they stay where they landed).
void check_home(const Solver& s, const char* what, real tol, unsigned kept = 0) {
  const TickPlan& p = s.last();
  CHECKF(s.done(), "%s: not done", what);
  CHECKF(p.support == 0xF, "%s: support %x at the end", what, p.support);
  for (int leg = 0; leg < kLegs; leg++)
    if (!(kept & (1u << leg))) CHECKF(std::fabs(p.xq[leg] / 4.0) <= tol, "%s: leg %d ends at %.2f mm", what, leg, p.xq[leg] / 4.0);
  CHECKF(std::fabs(p.lean_dx) < 0.01 && std::fabs(p.lean_roll) < 0.01, "%s: lean %.2f %.2f", what, (double)p.lean_dx,
         (double)p.lean_roll);
}

void test_the_planned_stop_ends_at_home() {
  for (const Gait& g : kGaitTable) {
    Solver& s = g_s;
    s.start(g, Direction::Forward, -1, 0);
    walk(s, 400);
    s.stop();
    walk(s, 1500);
    check_home(s, g.name, kRecentredMm + (real)0.5);
  }
}

void test_the_quick_stop() {
  for (const Gait& g : kGaitTable) {
    Solver& s = g_s;
    s.start(g, Direction::Forward, -1, 0);
    walk(s, 300);
    while (s.last().lifting == 0) s.next();  // a foot in the air
    unsigned kept = s.last().lifting;
    s.halt();
    CHECK(s.halting() && s.stopping());
    int t = 0;
    for (; t < kHaltHoldTicks && !s.done(); t++) {
      const TickPlan& p = s.next();
      if (t >= 3) CHECKF(p.lifting == 0, "%s: feet up %x %d ticks after the halt", g.name, p.lifting, t);
    }
    walk(s, 1500);
    check_home(s, g.name, kHaltSquareMm + (real)0.5, kept);
  }
}

void test_a_gait_change_goes_through_home() {
  Solver& s = g_s;
  s.start(*find_gait("walk"), Direction::Forward, -1, 0);
  walk(s, 300);
  s.set_gait(*find_gait("trot"));
  CHECK(s.changing());
  int t = 0;
  for (; t < 1500 && s.changing(); t++) s.next();
  CHECKF(!s.changing() && s.trot(), "still changing after %d ticks", t);
  Walked w = walk(s, 300);
  CHECK(w.most_up == 2 && !s.done());
  s.set_gait(*find_gait("crawl"));
  s.set_gait(*find_gait("trot"));  // the walking gait cancels the change
  CHECK(!s.changing());
}

void test_the_clock_waits_for_the_gate_and_lands_a_caught_foot() {
  Solver& s = g_s;
  s.start(*find_gait("walk"), Direction::Forward, -1, 0);
  walk(s, 200);
  while (s.last().lifting != 0) s.next();  // four feet down
  s.observe(8, 0);                          // off the plan: no lift
  uint32_t holds = s.stats().holds;
  for (int t = 0; t < 30; t++) {
    const TickPlan& p = s.next();
    if (s.stats().holds - holds < (uint32_t)s.config().hold_max)
      CHECKF(p.lifting == 0, "tick %d: lifted %x while off the plan", t, p.lifting);
  }
  CHECKF(s.stats().holds - holds == (uint32_t)s.config().hold_max, "holds %u", (unsigned)(s.stats().holds - holds));
  s.observe(0, 0);
  while (s.last().lifting == 0) s.next();
  int leg = 0;
  while (!(s.last().lifting & (1u << leg))) leg++;
  // tip toward the swinging leg: + roll for a left leg, + pitch for a front one
  s.observe(planner::kFront[leg] ? 10 : -10, 0);
  uint32_t catches = s.stats().catches;
  int t = 0;
  for (; t < 5 && (s.last().lifting & (1u << leg)); t++) s.next();
  CHECKF(s.stats().catches == catches + 1 && !(s.last().lifting & (1u << leg)), "caught %u, down after %d ticks",
         (unsigned)(s.stats().catches - catches), t);
  s.observe(0, 0);
}

void test_a_copy_solves_the_same_ticks() {
  Solver& s = g_s;
  s.start(*find_gait("crawl"), Direction::Forward, -1, 0);
  walk(s, 150);
  static Solver copy;
  copy = s;
  for (int t = 0; t < 200; t++) {
    TickPlan a = s.next();
    TickPlan b = copy.next();
    CHECKF(std::memcmp(a.xq, b.xq, sizeof a.xq) == 0 && std::memcmp(a.zq, b.zq, sizeof a.zq) == 0 &&
               a.tick == b.tick,
           "tick %d differs", t);
  }
}

}  // namespace

int main() {
  test_clock_lifts_each_leg_once_a_cycle();
  test_every_gait_walks_inside_its_limits();
  test_speed_cadence_and_steer_change_without_a_stop();
  test_the_planned_stop_ends_at_home();
  test_the_quick_stop();
  test_a_gait_change_goes_through_home();
  test_the_clock_waits_for_the_gate_and_lands_a_caught_foot();
  test_a_copy_solves_the_same_ticks();
  return report("test_engine");
}
