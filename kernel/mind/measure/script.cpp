#include "measure/script.h"
#include "util/str.h"

namespace kilo {
namespace measure {

using planner::LB;
using planner::LF;
using planner::RB;
using planner::RF;

const Experiment kExperimentTable[kExperiments] = {
    {"level", -1, 0, 0},
    {"steps", -1, 0, 0},
    {"edge_lf", LF, 37, 70},
    {"edge_rf", RF, 37, -70},
    {"edge_lb", LB, -37, 70},
    {"edge_rb", RB, -37, -70},
    {"edge_rf_roll", RF, 37, -70},
    {"edge_rb_roll", RB, -37, -69},
    {"level_end", -1, 0, 0},
};

namespace {

// The model's crossings (the feet's x, mm; script.py, test_measure checks them against the model).
constexpr int kXFront = 8, kXBack = 8, kXRoll = -12, kXRollB = -7;

Step g_steps[kMaxSteps];
int g_n = 0;

void add(uint8_t exp, const char* phase, StepKind kind, int ticks, bool keep_x, int x, const int dz[4]) {
  Step& s = g_steps[g_n++];
  s.exp = exp;
  s.phase = phase;
  s.kind = kind;
  s.ticks = (uint16_t)ticks;
  s.keep_x = keep_x;
  for (int k = 0; k < 4; k++) {
    s.dx[k] = (int8_t)x;
    s.dz[k] = (int8_t)dz[k];
  }
}

void roll_dz(int mm, int out[4]) {  // + left side down (TELEMETRY roll +)
  out[LF] = -mm;
  out[RF] = mm;
  out[RB] = mm;
  out[LB] = -mm;
}

int iabs(int v) { return v < 0 ? -v : v; }

void edge(uint8_t exp, int leg, int x0, int x1, int roll_mm) {
  int dz[4], up[4], zero[4] = {0, 0, 0, 0};
  roll_dz(roll_mm, dz);
  for (int k = 0; k < 4; k++) up[k] = k == leg ? dz[k] - kLiftMm : dz[k];
  add(exp, "to_start", StepKind::Move, 25, false, x0, dz);
  add(exp, "settle", StepKind::Hold, 10, false, x0, dz);
  add(exp, "lift", StepKind::Move, 10, false, x0, up);
  add(exp, "lifted", StepKind::Hold, 15, false, x0, up);
  add(exp, "sweep", StepKind::Sweep, iabs(x1 - x0) * kTicksPerMm, false, x1, up);
  add(exp, "lower", StepKind::Move, 10, true, 0, dz);
  add(exp, "lowered", StepKind::Hold, 10, true, 0, dz);
  add(exp, "home", StepKind::Move, 25, false, 0, zero);
  add(exp, "home_hold", StepKind::Hold, 10, false, 0, zero);
}

void build() {
  if (g_n != 0) return;
  int zero[4] = {0, 0, 0, 0};
  add(0, "level", StepKind::Hold, 75, false, 0, zero);
  const int p = kStepPitchMm, r = kStepRollMm;
  int down[4] = {-p, -p, p, p}, up[4] = {p, p, -p, -p}, left[4], right[4];
  roll_dz(r, left);
  roll_dz(-r, right);
  const struct {
    const char* name;
    const char* hold;
    const int* dz;
  } kSteps[] = {{"pitch_down", "pitch_down_hold", down},        {"pitch_level", "pitch_level_hold", zero},
                {"pitch_up", "pitch_up_hold", up},              {"pitch_level", "pitch_level_hold", zero},
                {"roll_left_down", "roll_left_down_hold", left}, {"roll_level", "roll_level_hold", zero},
                {"roll_right_down", "roll_right_down_hold", right}, {"roll_level", "roll_level_hold", zero}};
  for (const auto& s : kSteps) {
    add(1, s.name, StepKind::Move, 2, false, 0, s.dz);
    add(1, s.hold, StepKind::Hold, 38, false, 0, s.dz);
  }
  int S = kSweepSpanMm;
  edge(2, LF, kXFront + S, kXFront - S, 0);  // front feet lifted: the tripod stands with the feet forward
  edge(3, RF, kXFront + S, kXFront - S, 0);
  edge(4, LB, kXBack - S, kXBack + S, 0);  // back feet lifted: the other way
  edge(5, RB, kXBack - S - kRbBackMm, kRbMaxX, 0);  // the RB shoulder's clamp: no further forward
  S = kRollSpanMm;
  edge(6, RF, kXRoll + S, kXRoll - S, kRollSweepMm);     // RF up, the left side down: the mass leans away from it
  edge(7, RB, kXRollB - S, kXRollB + S, -kRollSweepBMm);  // RB up, the right side down: toward it
  add(8, "level_end", StepKind::Hold, 50, false, 0, zero);
}

// a + (b - a) j / n, rounded half away from zero, in integers (script.py _lerp)
int lerp(int a, int b, uint32_t j, uint32_t n) {
  int num = (b - a) * (int)j;
  int an = iabs(num);
  int q = an / (int)n, rem = an % (int)n;
  if (2 * rem >= (int)n) q++;
  return a + (num >= 0 ? q : -q);
}

}  // namespace

int steps(const Step** out) {
  build();
  *out = g_steps;
  return g_n;
}

uint32_t max_ticks() {
  const Step* s;
  int n = steps(&s);
  uint32_t t = 0;
  for (int i = 0; i < n; i++) t += s[i].ticks;
  return t;
}

Script::Script() {
  n_ = steps(&steps_);
  reset();
}

void Script::reset() {
  i_ = 0;
  j_ = t_ = 0;
  for (int k = 0; k < 4; k++) {
    x_[k] = sx_[k] = 0;
    z_[k] = sz_[k] = kHomeZ;
  }
  sweep_end_ = 0;
  tipped_ = tip_pending_ = false;
  tipped_at_ = 0;
  done_ = false;
  for (Hist& h : hist_) h = {0xFFFFFFFFu, 0};
}

void Script::begin() {
  for (int k = 0; k < 4; k++) {
    sx_[k] = x_[k];
    sz_[k] = z_[k];
  }
  const Step& st = steps_[i_];
  if (st.kind == StepKind::Sweep) {
    sweep_end_ = st.dx[0];
    if (tip_pending_) {  // tipped as the foot came up: the sweep is only kTipExtraMm
      int d = st.dx[0] > sx_[0] ? 1 : -1;
      int end = sx_[0] + d * kTipExtraMm;
      sweep_end_ = d > 0 ? (end < st.dx[0] ? end : st.dx[0]) : (end > st.dx[0] ? end : st.dx[0]);
      tip_pending_ = false;
    }
  } else if (!str_eq(st.phase, "lifted")) {
    tipped_ = false;
    tip_pending_ = false;
  }
}

int Script::x_at(uint32_t tick) const {
  const Hist& h = hist_[tick % kHistory];
  return h.tick == tick ? h.x : x_[0];
}

bool Script::tip(uint32_t tick_seen) {
  if (done_ || tipped_) return false;
  const Step& st = steps_[i_];
  if (str_eq(st.phase, "lifted") && i_ + 1 < n_ && steps_[i_ + 1].kind == StepKind::Sweep) {
    tipped_ = true;
    tip_pending_ = true;
    tipped_at_ = tick_seen;
    return true;
  }
  if (st.kind != StepKind::Sweep) return false;
  int x_then = x_at(tick_seen);
  int d = st.dx[0] > sx_[0] ? 1 : -1;
  int end = x_then + d * kTipExtraMm;
  int lim = st.dx[0];
  sweep_end_ = d > 0 ? (end < lim ? end : lim) : (end > lim ? end : lim);
  tipped_ = true;
  tipped_at_ = tick_seen;
  return true;
}

bool Script::next(Pose* out, bool* first) {
  if (done_) return false;
  const Step* st;
  for (;;) {
    st = &steps_[i_];
    if (j_ == 0) begin();
    uint32_t need = st->ticks;
    if (st->kind == StepKind::Sweep) need = (uint32_t)(iabs(sweep_end_ - sx_[0]) * kTicksPerMm);
    if (j_ < need) break;
    i_++;
    j_ = 0;
    if (i_ == n_) {
      done_ = true;
      return false;
    }
  }
  *first = j_ == 0;
  j_++;
  int tx[4], tz[4];
  for (int k = 0; k < 4; k++) {
    tx[k] = st->keep_x ? sx_[k] : st->dx[k];
    tz[k] = kHomeZ + st->dz[k];
  }
  if (st->kind == StepKind::Move) {
    for (int k = 0; k < 4; k++) {
      x_[k] = lerp(sx_[k], tx[k], j_, st->ticks);
      z_[k] = lerp(sz_[k], tz[k], j_, st->ticks);
    }
  } else if (st->kind == StepKind::Sweep) {
    int d = st->dx[0] > sx_[0] ? 1 : -1;
    int mm = (int)((j_ - 1) / kTicksPerMm) + 1;  // 1 mm at the step's first tick, then every kTicksPerMm
    for (int k = 0; k < 4; k++) {
      x_[k] = sx_[k] + d * mm;
      z_[k] = tz[k];
    }
    hist_[t_ % kHistory] = {t_, x_[0]};
  }
  for (int k = 0; k < 4; k++) {
    out->x[k] = (int8_t)x_[k];
    out->z[k] = (uint8_t)z_[k];
  }
  t_++;
  return true;
}

// ---- the rigid model -----------------------------------------------------------------------------

namespace {
real plane_h(const planner::BodyModel& m, const real xs[4], const real zs[4], int leg, planner::Attitude a) {
  real pa, pb;
  planner::plane_coeffs(a.pitch, a.roll, &pa, &pb);
  planner::V2 h = m.hip(leg);
  return zs[leg] + m.foot_r_mm + pa * (h.x + xs[leg]) + pb * h.y;
}
}  // namespace

planner::Attitude rigid_rest(const planner::BodyModel& m, const real xs[4], const real zs[4], uint8_t prev,
                             uint8_t* support) {
  planner::Attitude a4 = planner::fit_attitude(m, xs, zs, 0xF);
  real H = 0;
  for (int leg = 0; leg < 4; leg++) H += plane_h(m, xs, zs, leg, a4);
  H /= 4;
  real worst = 0;
  for (int leg = 0; leg < 4; leg++) worst = planner::m_max(worst, planner::m_fabs(plane_h(m, xs, zs, leg, a4) - H));
  if (worst <= kTouchMm) {
    *support = 0xF;
    return a4;
  }
  bool have = false;
  real best_m = 0;
  planner::Attitude best = a4;
  uint8_t best_sup = 0xF;
  for (int off = 0; off < 4; off++) {
    uint8_t sup = (uint8_t)(0xF & ~(1u << off));
    planner::Attitude a = planner::fit_attitude(m, xs, zs, sup);
    int first = off == 0 ? 1 : 0;
    real h = plane_h(m, xs, zs, first, a);
    if (plane_h(m, xs, zs, off, a) - h > kTouchMm) continue;  // the fourth leg reaches below the floor
    real mg = planner::stability(m, xs, zs, sup, a.pitch, a.roll);
    if (sup == prev && mg >= 0) {
      *support = sup;
      return a;
    }
    if (!have || mg > best_m) {
      have = true;
      best_m = mg;
      best = a;
      best_sup = sup;
    }
  }
  *support = have ? best_sup : 0xF;
  return have ? best : a4;
}

}  // namespace measure
}  // namespace kilo
