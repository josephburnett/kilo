#include "engine/solve.h"

namespace kilo {
namespace engine {

using planner::ceil_i;
using planner::clamp;
using planner::fit_attitude;
using planner::floor_i;
using planner::kPi;
using planner::leg_lengths;
using planner::m_cos;
using planner::m_fabs;
using planner::m_inf;
using planner::m_max;
using planner::m_min;
using planner::m_sqrt;
using planner::radians;
using planner::round_i;
using planner::stability;
using planner::V2;
using planner::V3;
using planner::world_points;
using planner::hull;
using planner::kLeft;
using planner::kFront;

static real smooth01(real s) {  // 0..1 -> 0..1 with zero slope at both ends
  s = m_min((real)1.0, m_max((real)0.0, s));
  return ((real)1.0 - m_cos(kPi * s)) / (real)2.0;
}

// Each leg's servos by the body's joint id (LF RF RB LB): its shoulder and its knee.
constexpr int kShoulder[kLegs] = {8, 9, 10, 11};
constexpr int kKnee[kLegs] = {12, 13, 14, 15};

static real lift_at(real h, int j, int w) { return h * ((real)1.0 - m_cos(2 * kPi * (real)j / (real)w)) / 2; }

// ---- the legs' kinematics: one tick of the clock, the swings and the slide ---------------------

void Solver::Kin::step(const Ctx& c, bool wait) {
  for (int leg = 0; leg < kLegs; leg++) lift[leg] = 0;
  uint8_t lifts = 0;
  if (!wait) {
    if (!c.trot) clock.set_cycle(stance + (real)c.w_new);
    if (!c.hold_clock) {
      clock.slew_offsets(swinging());
      lifts = clock.advance(false) & c.allow;
    }
    if (c.next_cycle > 0) clock.set_cycle(c.next_cycle);
    // the commands move toward what was asked
    for (int i = 0; i < 2; i++) s[i] += clamp(c.s_tgt[i] - s[i], -c.slew, c.slew);
    stance = m_min(stance * ((real)1.0 + kStanceSlew), m_max(stance / ((real)1.0 + kStanceSlew), c.stance_tgt));
    h += clamp(c.h_tgt - h, -kHeightSlewMm, kHeightSlewMm);
    centre += clamp(c.centre_tgt - centre, -kCentreSlewMm, kCentreSlewMm);
  }
  for (int leg = 0; leg < kLegs; leg++) {
    int side = kLeft[leg] ? 0 : 1;
    if (j[leg] == 0 && (lifts & (1u << leg))) {  // lift-off
      j[leg] = 1;
      w[leg] = c.w_new;
      x0[leg] = x[leg];
      hl[leg] = c.step_h;
      caught[leg] = false;
    } else if (j[leg] > 0 && !wait) {
      j[leg]++;
    }
    if (j[leg] == 0) {
      if (!wait) x[leg] -= s[side] / stance;
      continue;
    }
    if (caught[leg]) {  // a caught foot comes down where it is, in three ticks
      if (!wait) caught_t[leg]++;
      lift[leg] = caught_lift[leg] * ((real)1.0 - smooth01((real)caught_t[leg] / (real)3.0));
      if (caught_t[leg] >= 3) {
        j[leg] = 0;
        caught[leg] = false;
        lift[leg] = 0;
      }
      continue;
    }
    real sj = (real)j[leg] / (real)w[leg];
    real to = c.trot ? trot_land(leg) : s[side] / (real)2.0 + centre;
    x[leg] = x0[leg] + (to - x0[leg]) * smooth01(sj);
    lift[leg] = lift_at(hl[leg], j[leg], w[leg]);
    if (j[leg] >= w[leg]) {  // touchdown
      j[leg] = 0;
      lift[leg] = 0;
    }
  }
}

// The trot's landing: this step's pendulum point, its shift from half a stride scaled with the
// stride as it slews (plan.cpp's target_x, as the trot the owner saw).
real Solver::Kin::trot_land(int leg) const {
  int side = kLeft[leg] ? 0 : 1;
  real st = slot_s[side];
  if (m_fabs(st) < (real)1e-6) return s[side] / (real)2.0;
  return s[side] / (real)2.0 - (st / (real)2.0 - land[side]) * m_min((real)1.0, m_max((real)0.0, s[side] / st));
}

uint8_t Solver::Kin::swinging() const {
  uint8_t m = 0;
  for (int leg = 0; leg < kLegs; leg++)
    if (j[leg] > 0) m |= (uint8_t)(1u << leg);
  return m;
}

// ---- the solver -----------------------------------------------------------------------------

Solver::Solver(const BodyModel& model, const SolverConfig& cfg) : model_(model), cfg_(cfg), gait_(&default_gait()) {
  stats_.min_margin = m_inf();
}

void Solver::start(const Gait& g, Direction d, real speed_mm_s, uint32_t tick, real start_height, real steer,
                   real cadence) {
  tick_ = tick;
  lag_ = model_.lag_ticks(kTickMs);
  settle_ = model_.settle_ticks(kTickMs);
  alpha_ = model_.response_alpha(kTickMs);
  for (int leg = 0; leg < kLegs; leg++) {  // the leg's servos' dead time beyond the robot's, in ticks
    real d = (model_.joint_lag_ms(kShoulder[leg]) + model_.joint_lag_ms(kKnee[leg])) / 2 - model_.servo_lag_ms;
    lead_[leg] = d / kTickMs;
  }
  has_prev_feet_ = false;
  resp_ = {0, 0};
  gait_ = &g;
  pending_ = nullptr;
  direction_ = d;
  steer_ = clamp(steer, -1, 1);
  speed_mm_s_ = speed_mm_s < 0 ? default_speed(g) : m_fabs(speed_mm_s);
  cadence_ = cadence;
  k_ = Kin();
  k_.h = start_height < 0 ? model_.stance_height(KILO_GAIT_STANCE_H_256) : start_height;
  for (int leg = 0; leg < kLegs; leg++) halt_kept_[leg] = false;
  stopping_ = halting_ = finishing_ = done_ = false;
  halt_t_ = 0;
  halt_slew_ = 0;
  finish_t_ = finish_n_ = 0;
  held_ = 0;
  lw_ = kTrotWMin;
  err_[0] = err_[1] = 0;
  prev_att_ = {0, 0};
  hist_n_ = (int)lag_ + 3;
  if (hist_n_ > kHistMax) hist_n_ = kHistMax;
  for (int i = 0; i < hist_n_; i++) att_hist_[i] = {0, 0};
  for (int k = 0; k < kHorizon; k++) path_dx_[k] = path_roll_[k] = 0;
  dx_ = roll_ = dx_prev_ = roll_prev_ = 0;
  has_last_ = false;
  stats_ = {0, 0, 0, m_inf()};
  apply_timing();
  k_.stance = stance_tgt_;
  begin_gait();
}

// The clock for the gait now: its first lift kStartTicks from now.
void Solver::begin_gait() {
  Timing tm = timing();
  real cycle = gait_->pendulum ? (real)(2 * (kTrotWMin + 1)) : tm.cycle_ticks;
  real off[kLegs];
  offsets(*gait_, gait_->pendulum ? kTrotWMin : tm.swing_ticks, cycle, off);
  real first = 1;
  for (int leg = 0; leg < kLegs; leg++) first = m_min(first, off[leg]);
  k_.clock.start(off, cycle, first - (real)kStartTicks / cycle);
  lw_ = kTrotWMin;
  for (int i = 0; i < 2; i++) k_.land[i] = k_.slot_s[i] = 0;
}

void Solver::apply_timing() {
  Timing tm = timing();
  w_tgt_ = tm.swing_ticks;
  stance_tgt_ = tm.stance_ticks;
  if (!gait_->pendulum) {  // the lifts' places in the cycle for this swing and cadence
    real off[kLegs];
    offsets(*gait_, w_tgt_, tm.cycle_ticks, off);
    k_.clock.retarget(off);
  }
  if (homing()) {
    s_tgt_[0] = s_tgt_[1] = 0;
  } else {
    real cap = gait_->pendulum ? (real)4.75 * (real)trot_swing_for(tm.speed_mm_s) - (real)0.5 : gait_->stride_mm;
    side_strides(direction_, tm.stride_mm, s_tgt_, steer_, cap);
  }
}

void Solver::set_speed(real v) {
  speed_mm_s_ = m_fabs(v);
  apply_timing();
}
void Solver::set_direction(Direction d) {
  direction_ = d;
  apply_timing();
}
void Solver::set_steer(real s) {
  steer_ = clamp(s, -1, 1);
  apply_timing();
}
void Solver::set_cadence(real c) {
  cadence_ = c;
  apply_timing();
}

void Solver::set_gait(const Gait& g) {
  if (stopping_) return;
  if (&g == gait_) {
    if (pending_ != nullptr) {
      pending_ = nullptr;
      apply_timing();
    }
    return;
  }
  pending_ = &g;
  apply_timing();
}

void Solver::stop() {
  stopping_ = true;
  pending_ = nullptr;
  apply_timing();
}

void Solver::halt() {
  if (done_ || finishing_ || halting_) return;
  stopping_ = halting_ = true;
  halt_t_ = 0;
  pending_ = nullptr;
  for (int leg = 0; leg < kLegs; leg++) halt_kept_[leg] = false;
  halt_slew_ = m_max(gait_->stride_slew_mm, m_max(m_fabs(k_.s[0]), m_fabs(k_.s[1])) / (real)kHaltTicks);
  apply_timing();
  for (int leg = 0; leg < kLegs; leg++) {  // every foot in the air comes down where it is
    // a foot in its last tick of swing lands anyway
    if (k_.j[leg] == 0 || k_.caught[leg] || k_.j[leg] >= k_.w[leg] - 1) continue;
    k_.caught[leg] = true;
    k_.caught_t[leg] = 0;
    k_.caught_lift[leg] = lift_at(k_.hl[leg], k_.j[leg], k_.w[leg]);
    halt_kept_[leg] = true;
  }
}

real Solver::height_target() const {
  if (finishing_) return model_.stance_height(KILO_GAIT_STANCE_H_256);
  return model_.stance_height(gait_->body_h_256);
}

real Solver::centre_target() const {
  if (!gait_->centre_com || homing()) return 0;
  real xs[kLegs] = {0, 0, 0, 0};
  real z = model_.stance_height(gait_->body_h_256);
  real zs[kLegs] = {z, z, z, z};
  return model_.com(xs, zs).x;
}

bool Solver::leg_home(int leg) const {
  if (!(m_fabs(k_.s[0]) < (real)0.05 && m_fabs(k_.s[1]) < (real)0.05)) return false;
  real tol = halting_ ? kHaltSquareMm : kRecentredMm;
  return m_fabs(k_.x[leg]) < tol || halt_kept_[leg];
}

bool Solver::home() const {
  if (k_.swinging() != 0) return false;
  for (int leg = 0; leg < kLegs; leg++)
    if (!leg_home(leg)) return false;
  return true;
}

// Where this side's pair lands at the target stride: the fitted pendulum's balanced point, no
// further out than half the stride + 6 mm, forward never behind the shoulder, backward never ahead.
real Solver::land_for(int side) const {
  real s = s_tgt_[side];
  if (homing() || m_fabs(s) < (real)1e-6) return s / (real)2.0;
  int w = trot_swing_for(timing().speed_mm_s);
  real v = s / (((real)w + kTrotStancePlus) * kTickS);
  real x = fit_land_x(w, v);
  return s > 0 ? m_min(s / (real)2.0 + (real)6.0, m_max((real)0.0, x)) : m_max(s / (real)2.0 - (real)6.0, m_min((real)0.0, x));
}

// A w-tick swing from where the foot is to `to` turns no joint more than kTrotJointDegPerTick a tick.
bool Solver::swing_ok(int leg, real to, int w) const {
  const real lim = radians(kTrotJointDegPerTick);
  const real x0 = k_.x[leg], h = k_.h, lift = gait_->step_h_mm;
  real a1, a2;
  model_.ik(leg, x0, h, &a1, &a2);
  for (int j = 1; j <= w; j++) {
    real b1, b2;
    real sj = (real)j / (real)w;
    model_.ik(leg, x0 + (to - x0) * smooth01(sj), h - lift * ((real)1.0 - m_cos(2 * kPi * sj)) / 2, &b1, &b2);
    if (m_fabs(b1 - a1) > lim || m_fabs(b2 - a2) > lim) return false;
    a1 = b1;
    a2 = b2;
  }
  return true;
}

// The trot's swing for the pair lifting now (plan.cpp's start_slot, as the trot the owner saw): the
// speed's, at most a tick longer than the last pair's (the build-up) and at most a tick shorter,
// long enough for the step it carries, and away from the steady trot long enough for no joint to
// pass kTrotJointDegPerTick a tick. Sets this step's landing.
int Solver::trot_swing(uint8_t legs) {
  const bool going_home = homing();
  int wt = trot_swing_for(timing().speed_mm_s);
  int w = going_home ? kTrotWMin : (wt < lw_ + 1 ? wt : lw_ + 1);
  real to[kLegs] = {};
  for (int side = 0; side < 2; side++) {
    k_.land[side] = land_for(side);
    k_.slot_s[side] = s_tgt_[side];
  }
  for (int leg = 0; leg < kLegs; leg++) {
    if (!(legs & (1u << leg))) continue;
    int side = kLeft[leg] ? 0 : 1;
    real sc = k_.s[side];
    if (going_home) {
      to[leg] = sc / (real)2.0;
    } else {
      real lt = k_.land[side], st = s_tgt_[side];
      real lc = m_fabs(st) > (real)1e-6 ? sc / (real)2.0 - (st / (real)2.0 - lt) * m_min((real)1.0, m_max((real)0.0, sc / st))
                                         : sc / (real)2.0;
      to[leg] = m_fabs(lc - k_.x[leg]) > m_fabs(lt - k_.x[leg]) ? lc : lt;
    }
    real d = m_fabs(to[leg] - k_.x[leg]) + m_fabs(sc) / m_max((real)1.0, k_.stance);
    int need = ceil_i(d / kTrotStepMmPerTick - (real)1e-9);
    if (need > w) w = need;
  }
  if (lw_ - 1 > w) w = lw_ - 1;
  if (w > kTrotWMax + 2) w = kTrotWMax + 2;
  if (w < kTrotWMin) w = kTrotWMin;
  if (going_home || pending_ != nullptr) {
    for (; w < kTrotWMax + 2; w++) {
      bool ok = true;
      for (int leg = 0; leg < kLegs && ok; leg++)
        if (legs & (1u << leg)) ok = swing_ok(leg, to[leg], w);
      if (ok) break;
    }
  }
  lw_ = w;
  return w;
}

Solver::Ctx Solver::context() const {
  Ctx c;
  c.s_tgt[0] = s_tgt_[0];
  c.s_tgt[1] = s_tgt_[1];
  c.slew = halting_ ? halt_slew_ : gait_->stride_slew_mm;
  c.stance_tgt = stance_tgt_;
  c.h_tgt = height_target();
  c.centre_tgt = centre_target();
  c.w_new = w_tgt_;
  c.step_h = gait_->step_h_mm;
  c.trot = gait_->pendulum;
  c.allow = 0xF;
  c.hold_clock = false;
  c.next_cycle = 0;
  return c;
}

// Signed distance (mm, + inside) of the centre of mass from each edge of the support polygon, the
// gait-frame feet shifted by dx and the body rolled.
int Solver::edge_margins(const real xg[kLegs], const real lift[kLegs], uint8_t support, real h, real dx,
                         real roll, real out[kLegs]) const {
  real xs[kLegs], zs[kLegs];
  for (int leg = 0; leg < kLegs; leg++) xs[leg] = xg[leg] - dx;
  leg_lengths(model_, xs, h, 0, roll, zs);
  for (int leg = 0; leg < kLegs; leg++) zs[leg] -= lift[leg];
  V3 com, feet[kLegs];
  world_points(model_, xs, zs, 0, roll, &com, feet);
  V2 pts[kLegs], poly[kLegs];
  int n = 0;
  for (int leg = 0; leg < kLegs; leg++)
    if (support & (1u << leg)) pts[n++] = {feet[leg].x, feet[leg].y};
  n = hull(pts, n, poly);
  for (int i = 0; i < n; i++) {
    V2 a = poly[i], b = poly[(i + 1) % n];
    real ex = b.x - a.x, ey = b.y - a.y;
    real L = m_sqrt(ex * ex + ey * ey);
    if (L == 0) L = 1;
    out[i] = (ex * (com.y - a.y) - ey * (com.x - a.x)) / L;
  }
  return n;
}

// The weight's path over the next kHorizon ticks (solve.h, 3.): the margins linearised about the
// last path, then sweeps of the smoothest path inside them; this tick's pose is its first point.
void Solver::solve_lean(const Ctx& now, bool advanced) {
  const SolverConfig& cfg = cfg_;
  const bool want = cfg.lean && gait_->margin_mm >= 0 && !finishing_;
  if (advanced) {
    for (int k = 0; k + 1 < kHorizon; k++) {
      path_dx_[k] = path_dx_[k + 1];
      path_roll_[k] = path_roll_[k + 1];
    }
  }
  struct Row {
    real g[2];
    real b;
  };
  static_assert(kHorizon <= 64, "the path's rows");
  Row rows[kHorizon][3];
  uint8_t nrows[kHorizon] = {};
  if (want) {
    // the feet over the horizon: tick 0 is this tick's (already stepped), then as the clock goes
    Kin k = k_;
    Ctx c = now;
    c.next_cycle = 0;
    real lift0[kLegs];
    for (int leg = 0; leg < kLegs; leg++) lift0[leg] = k_.lift[leg];
    for (int t = 0; t < kHorizon; t++) {
      const real* lift = lift0;
      if (t > 0) {
        c.hold_clock = halting_ && halt_t_ + t < kHaltHoldTicks;  // the quick stop's hold, as it runs out
        k.step(c, false);
        lift = k.lift;
      }
      uint8_t support = 0;
      int n_up = 0;
      for (int leg = 0; leg < kLegs; leg++) {
        if (lift[leg] <= (real)1e-6) support |= (uint8_t)(1u << leg);
        else n_up++;
      }
      if (n_up != 1) continue;
      real m0[kLegs], m1[kLegs], m2[kLegs];
      real pdx = path_dx_[t], pr = path_roll_[t];
      int n = edge_margins(k.x, lift, support, k.h, pdx, pr, m0);
      edge_margins(k.x, lift, support, k.h, pdx + 1, pr, m1);
      edge_margins(k.x, lift, support, k.h, pdx, pr + 1, m2);
      int e2 = 0;
      for (int e = 0; e < n && e2 < 3; e++) {
        real g0 = m1[e] - m0[e], g1 = m2[e] - m0[e];
        if (m0[e] > gait_->margin_mm + 12) continue;  // far inside: not a constraint here
        rows[t][e2] = {{g0, g1}, gait_->margin_mm - m0[e] + g0 * pdx + g1 * pr};
        e2++;
      }
      nrows[t] = (uint8_t)e2;
    }
  }
  // sweeps: each point the smoothest given its neighbours, then inside its margins and bounds
  const real a = 1, b = cfg.smooth, c2 = cfg.smooth2;
  const real den = a + 2 * b + 6 * c2;
  const real wr = gait_->roll_cost;
  for (int it = 0; it < kSweeps; it++) {
    for (int t = 0; t < kHorizon; t++) {
      real u[2];
      for (int q = 0; q < 2; q++) {
        const real* p = q == 0 ? path_dx_ : path_roll_;
        real m1 = t >= 1 ? p[t - 1] : (q == 0 ? dx_ : roll_);
        real m2 = t >= 2 ? p[t - 2] : t == 1 ? (q == 0 ? dx_ : roll_) : (q == 0 ? dx_prev_ : roll_prev_);
        real p1 = t + 1 < kHorizon ? p[t + 1] : p[kHorizon - 1];
        real p2 = t + 2 < kHorizon ? p[t + 2] : p[kHorizon - 1];
        u[q] = (b * (m1 + p1) + c2 * (4 * m1 + 4 * p1 - m2 - p2)) / den;
      }
      for (int pass = 0; pass < 3; pass++)
        for (int e = 0; e < nrows[t]; e++) {
          const Row& r = rows[t][e];
          real v = r.g[0] * u[0] + r.g[1] * u[1];
          if (v >= r.b) continue;
          real nn = r.g[0] * r.g[0] + r.g[1] * r.g[1] / wr;
          if (nn < (real)1e-9) continue;
          real l = (r.b - v) / nn;
          u[0] += r.g[0] * l;
          u[1] += r.g[1] * l / wr;
        }
      path_dx_[t] = clamp(u[0], -cfg.dx_max_mm, cfg.dx_max_mm);
      path_roll_[t] = clamp(u[1], -gait_->roll_max_deg, gait_->roll_max_deg);
    }
  }
  if (!want) {
    for (int t = 0; t < kHorizon; t++) path_dx_[t] = path_roll_[t] = 0;
  }
  dx_prev_ = dx_;
  roll_prev_ = roll_;
  dx_ += clamp(path_dx_[0] - dx_, -cfg.dx_step_mm, cfg.dx_step_mm);
  roll_ += clamp(path_roll_[0] - roll_, -cfg.roll_step_deg, cfg.roll_step_deg);
}

// The margin the lifts due now would leave: the feet after this tick, the body where the path
// puts it this tick (the lean gate).
real Solver::lift_margin(const Ctx& c) const {
  Kin k = k_;
  k.step(c, false);
  uint8_t support = 0;
  for (int leg = 0; leg < kLegs; leg++)
    if (k.lift[leg] <= (real)1e-6) support |= (uint8_t)(1u << leg);
  real dx = dx_ + clamp(path_dx_[1] - dx_, -cfg_.dx_step_mm, cfg_.dx_step_mm);
  real roll = roll_ + clamp(path_roll_[1] - roll_, -cfg_.roll_step_deg, cfg_.roll_step_deg);
  real m[kLegs];
  int n = edge_margins(k.x, k.lift, support, k.h, dx, roll, m);
  real lo = m_inf();
  for (int e = 0; e < n; e++) lo = m_min(lo, m[e]);
  return lo;
}

const TickPlan& Solver::next() {
  const SolverConfig& cfg = cfg_;
  // a gait change: once every leg is home, the new gait from its first lift
  if (pending_ != nullptr && home()) {
    gait_ = pending_;
    pending_ = nullptr;
    apply_timing();
    k_.stance = stance_tgt_;
    begin_gait();
  }
  if (stopping_ && !finishing_ && home()) {
    finishing_ = true;
    finish_t_ = 0;
    finish_dx_ = dx_;
    finish_roll_ = roll_;
    real need = kPi / 2 * m_fabs(dx_) / (real)100.0 / kTickS;  // the shift back at most 100 mm/s
    finish_n_ = m_max((real)(settle_ + 5), need) > (real)(settle_ + 5) ? ceil_i(need) : settle_ + 5;
  }
  if (finishing_ && finish_t_ >= finish_n_ + settle_) done_ = true;

  Ctx c = context();
  // which legs may lift: none in the quick stop's hold, none at home while stopping
  uint8_t allow = 0xF;
  if (finishing_) allow = 0;
  c.hold_clock = halting_ && halt_t_ < kHaltHoldTicks;
  for (int leg = 0; leg < kLegs; leg++)
    if (homing() && leg_home(leg)) allow &= (uint8_t)~(1u << leg);
  c.allow = allow;
  StepClock peek = k_.clock;
  if (!c.trot) peek.set_cycle(k_.stance + (real)c.w_new);
  uint8_t due = c.hold_clock ? 0 : peek.advance(false) & allow & (uint8_t)~k_.swinging();

  // the clock waits: a lift due while the body is off the plan, or the weight not yet there
  bool wait = false;
  if (due != 0 && !halting_ && held_ < cfg.hold_max) {
    if (m_max(m_fabs(err_[0]), m_fabs(err_[1])) > cfg.gate_deg) {
      wait = true;
      stats_.holds++;
    } else if (cfg.lean && gait_->margin_mm >= 0 && lift_margin(c) < gait_->margin_mm - cfg.lean_tol_mm) {
      wait = true;
      stats_.waits++;
    }
  }
  if (wait) held_++;
  else if (due != 0) held_ = 0;
  if (due != 0 && c.trot && !wait) {
    c.w_new = trot_swing(due);
    c.next_cycle = (real)(2 * (c.w_new + 1));
  }
  // a swinging leg lands at once when the body tips toward it
  for (int leg = 0; leg < kLegs; leg++) {
    int j = k_.j[leg];
    if (j < 1 || j >= k_.w[leg] || k_.caught[leg]) continue;
    if (tip_toward(leg) > cfg.catch_deg) {
      k_.caught[leg] = true;
      k_.caught_t[leg] = 0;
      k_.caught_lift[leg] = lift_at(k_.hl[leg], j, k_.w[leg]);
      stats_.catches++;
      if (halting_) halt_kept_[leg] = true;
    }
  }
  k_.step(c, wait);
  solve_lean(c, !wait);
  if (finishing_) {  // the level-out: the shift and the roll back to 0
    real f = smooth01((real)(finish_t_ + 1) / (real)finish_n_);
    dx_ = finish_dx_ * ((real)1.0 - f);
    roll_ = finish_roll_ * ((real)1.0 - f);
    finish_t_++;
  }
  if (halting_) halt_t_++;

  // the feet in the body frame, the legs' lengths, the lift
  TickPlan& out = last_;
  real xs[kLegs], zs[kLegs], xi[kLegs], zi[kLegs];
  for (int leg = 0; leg < kLegs; leg++) xs[leg] = k_.x[leg] - dx_;
  leg_lengths(model_, xs, k_.h, 0, roll_, zs);
  uint8_t support = 0, lifting = 0;
  int n_lifting = 0;
  for (int leg = 0; leg < kLegs; leg++) {
    real x = clamp(xs[leg], kXMinMm, kXMaxMm);
    real z = clamp(zs[leg] - k_.lift[leg], kZMinMm, kZMaxMm);
    xi[leg] = (real)round_i(x * 4) / 4;  // the plan's feet, as POSE can carry them
    zi[leg] = (real)round_i(z * 4) / 4;
    // each leg's own servo lead ([model.correction]): a leg whose servos start later than the
    // robot's gets its foot that much earlier, along its path (0 for every leg until measured)
    real lead = lead_[leg];
    real xl = x, zl = z;
    if (lead != 0 && has_prev_feet_) {
      xl = clamp(x + (x - prev_x_[leg]) * lead, kXMinMm, kXMaxMm);
      zl = clamp(z + (z - prev_z_[leg]) * lead, kZMinMm, kZMaxMm);
    }
    prev_x_[leg] = x;
    prev_z_[leg] = z;
    out.xq[leg] = (int16_t)round_i(xl * 4);
    out.zq[leg] = (int16_t)round_i(zl * 4);
    if (k_.lift[leg] <= (real)1e-6) support |= (uint8_t)(1u << leg);
    else {
      lifting |= (uint8_t)(1u << leg);
      n_lifting++;
    }
  }
  Attitude att = fit_attitude(model_, xi, zi, support, prev_att_);
  prev_att_ = att;
  for (int i = 0; i + 1 < hist_n_; i++) att_hist_[i] = att_hist_[i + 1];
  att_hist_[hist_n_ - 1] = att;
  real lp, lr;
  lagged(&lp, &lr);
  resp_.pitch += (lp - resp_.pitch) * alpha_;
  resp_.roll += (lr - resp_.roll) * alpha_;
  real m = stability(model_, xi, zi, support, att.pitch, att.roll);
  if (n_lifting == 1) stats_.min_margin = m_min(stats_.min_margin, m);
  out.tick = tick_;
  out.support = support;
  out.lifting = lifting;
  out.n_lifting = (uint8_t)n_lifting;
  out.pitch = clamp(resp_.pitch, -kAttMaxDeg, kAttMaxDeg);
  out.roll = clamp(resp_.roll, -kAttMaxDeg, kAttMaxDeg);
  out.plan_pitch = att.pitch;
  out.plan_roll = att.roll;
  out.margin = m;
  out.lean_dx = dx_;
  out.lean_roll = roll_;
  has_last_ = true;
  has_prev_feet_ = true;
  tick_++;
  return out;
}

void Solver::lagged(real* pitch, real* roll) const {
  int n = hist_n_ - 1;
  int i = floor_i(lag_);
  real f = lag_ - (real)i;
  int ia = n - i > 0 ? n - i : 0;
  int ib = n - i - 1 > 0 ? n - i - 1 : 0;
  Attitude a = att_hist_[ia], b = att_hist_[ib];
  *pitch = a.pitch + (b.pitch - a.pitch) * f;
  *roll = a.roll + (b.roll - a.roll) * f;
}

real Solver::tip_toward(int leg) const {
  real side = kLeft[leg] ? (real)1.0 : (real)-1.0;
  real end = kFront[leg] ? (real)1.0 : (real)-1.0;
  return side * err_[1] + end * err_[0];
}

}  // namespace engine
}  // namespace kilo
