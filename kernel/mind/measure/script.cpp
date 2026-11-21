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
    {"stairs_pitch", -1, 0, 0},
    {"stairs_roll", -1, 0, 0},
    {"twist", -1, 0, 0},
    {"splay", -1, 0, 0},
    {"edge_lf", LF, 37, 70},
    {"edge_rf", RF, 37, -70},
    {"edge_lb", LB, -37, 70},
    {"edge_rb", RB, -37, -70},
    {"edge_lf_away", LF, 37, 70},
    {"edge_lf_toward", LF, 37, 69},
    {"edge_rf_away", RF, 37, -70},
    {"edge_lb_toward", LB, -37, 69},
    {"check", -1, 0, 0},
    {"level_end", -1, 0, 0},
};

namespace {

// The experiments' indices (kExperimentTable's order).
enum : uint8_t {
  kLevel,
  kStairsPitch,
  kStairsRoll,
  kTwist,
  kSplay,
  kEdgeLf,
  kEdgeRf,
  kEdgeLb,
  kEdgeRb,
  kEdgeLfAway,
  kEdgeLfToward,
  kEdgeRfAway,
  kEdgeLbToward,
  kCheck,
  kLevelEnd
};

// The model's crossings (the feet's x, mm; script.py, test_measure checks them against the model):
// front and back feet lifted, flat; rolled 4 mm away (front); 3 mm toward, LF and LB.
constexpr int kXFront = 8, kXBack = 8, kXAway = -12, kXLfToward = 23, kXLbToward = -7;

Step g_steps[kMaxSteps];
int g_n = 0;

void add(uint8_t exp, const char* phase, StepKind kind, int ticks, bool keep_x, const int dx[4], const int dz[4]) {
  Step& s = g_steps[g_n++];
  s.exp = exp;
  s.phase = phase;
  s.kind = kind;
  s.ticks = (uint16_t)ticks;
  s.keep_x = keep_x;
  for (int k = 0; k < 4; k++) {
    s.dx[k] = (int8_t)dx[k];
    s.dz[k] = (int8_t)dz[k];
  }
}

void all(int v, int out[4]) {
  for (int k = 0; k < 4; k++) out[k] = v;
}

void roll_dz(int mm, int out[4]) {  // + left side down (TELEMETRY roll +)
  out[LF] = -mm;
  out[RF] = mm;
  out[RB] = mm;
  out[LB] = -mm;
}

void pitch_dz(int mm, int out[4]) {  // + nose down (TELEMETRY pitch +)
  out[LF] = -mm;
  out[RF] = -mm;
  out[RB] = mm;
  out[LB] = mm;
}

void twist_dz(int mm, int out[4]) {  // + LF and RB longer, RF and LB shorter
  out[LF] = mm;
  out[RF] = -mm;
  out[RB] = mm;
  out[LB] = -mm;
}

int iabs(int v) { return v < 0 ? -v : v; }

void edge(uint8_t exp, int leg, int x0, int x1, int roll_mm) {
  int dz[4], up[4], zero[4] = {0, 0, 0, 0}, a[4], b[4];
  roll_dz(roll_mm, dz);
  for (int k = 0; k < 4; k++) up[k] = k == leg ? dz[k] - kLiftMm : dz[k];
  all(x0, a);
  all(x1, b);
  add(exp, "to_start", StepKind::Move, 20, false, a, dz);
  add(exp, "settle", StepKind::Hold, 10, false, a, dz);
  add(exp, "lift", StepKind::Move, 10, false, a, up);
  add(exp, "lifted", StepKind::Hold, kLiftedTicks, false, a, up);
  add(exp, "sweep", StepKind::Sweep, iabs(x1 - x0) * kTicksPerMm, false, b, up);
  add(exp, "lower", StepKind::Move, 10, true, zero, dz);
  add(exp, "lowered", StepKind::Hold, 10, true, zero, dz);
  add(exp, "home", StepKind::Move, 20, false, zero, zero);
  add(exp, "home_hold", StepKind::Hold, kHomeHold, false, zero, zero);
}

void build() {
  if (g_n != 0) return;
  int zero[4] = {0, 0, 0, 0}, dz[4], dx[4];
  add(kLevel, "level", StepKind::Hold, 75, false, zero, zero);
  // the stairs: each level's half difference, in the order played from home (script.py STAIRS_*)
  const int kPitch[] = {2, 4, 6, 0, -2, -4, -6, 0, 6, 4, 2, 0, -6, -4, -2, 0};
  const int kRoll[] = {2, 4, 0, -2, -4, 0, 4, 2, 0, -4, -2, 0};
  for (int lv : kPitch) {
    pitch_dz(lv, dz);
    add(kStairsPitch, "step", StepKind::Move, kStairMove, false, zero, dz);
    add(kStairsPitch, "hold", StepKind::Hold, kStairHold, false, zero, dz);
  }
  for (int lv : kRoll) {
    roll_dz(lv, dz);
    add(kStairsRoll, "step", StepKind::Move, kStairMove, false, zero, dz);
    add(kStairsRoll, "hold", StepKind::Hold, kStairHold, false, zero, dz);
  }
  // the twist, the body kTwistX forward
  all(kTwistX, dx);
  add(kTwist, "to_start", StepKind::Move, 20, false, dx, zero);
  add(kTwist, "hold", StepKind::Hold, kStairHold, false, dx, zero);
  const int kTwists[] = {1, 2, 1, 0, -1, -2, -1, 0};
  for (int k : kTwists) {
    twist_dz(k, dz);
    add(kTwist, "step", StepKind::Move, kStairMove, false, dx, dz);
    add(kTwist, "hold", StepKind::Hold, kStairHold, false, dx, dz);
  }
  add(kTwist, "home", StepKind::Move, 20, false, zero, zero);
  add(kTwist, "home_hold", StepKind::Hold, kHomeHold, false, zero, zero);
  // the splay: nose down, the feet's rectangle longer, shorter, twice
  pitch_dz(kSplayPitchMm, dz);
  const int out[4] = {kSplayOutMm, kSplayOutMm, -kSplayOutMm, -kSplayOutMm};
  const int inn[4] = {-kSplayInMm, -kSplayInMm, kSplayInMm, kSplayInMm};
  add(kSplay, "step", StepKind::Move, 4, false, zero, dz);
  add(kSplay, "hold", StepKind::Hold, kStairHold, false, zero, dz);
  const int* kSplays[] = {out, inn, out, inn, zero};
  for (const int* x : kSplays) {
    add(kSplay, "step", StepKind::Move, 12, false, x, dz);
    add(kSplay, "hold", StepKind::Hold, kStairHold, false, x, dz);
  }
  add(kSplay, "home", StepKind::Move, 4, false, zero, zero);
  add(kSplay, "home_hold", StepKind::Hold, kHomeHold, false, zero, zero);
  int S = kSweepSpanMm;
  edge(kEdgeLf, LF, kXFront + S, kXFront - S, 0);  // front feet lifted: the tripod stands with the feet forward
  edge(kEdgeRf, RF, kXFront + S, kXFront - S, 0);
  edge(kEdgeLb, LB, kXBack - S, kXBack + S, 0);  // back feet lifted: the other way
  edge(kEdgeRb, RB, kXBack - S - kRbBackMm, kRbMaxX, 0);  // the RB shoulder's clamp: no further forward
  // rolled, against the flat sweep of the same foot: LF both ways, RF away, LB toward (what the RB
  // shoulder's reach allows, all four feet sweeping together)
  S = kRollSpanMm;
  edge(kEdgeLfAway, LF, kXAway + S, kXAway - S, -kRollSweepMm);
  edge(kEdgeLfToward, LF, kXLfToward + S, kXLfToward - S, kRollSweepBMm);
  edge(kEdgeRfAway, RF, kXAway + S, kXAway - S, kRollSweepMm);
  edge(kEdgeLbToward, LB, kXLbToward - S, kXLbToward + S, kRollSweepBMm);
  // the check poses (script.py CHECK_POSES): x for all four feet, dz, the foot lifted or -1
  const struct {
    int x;
    int dz[4];
    int leg;
  } kChecks[] = {
      {0, {-5, -1, 5, 1}, -1},    // pitch 3 and roll 2
      {0, {7, 1, -7, -1}, -1},    // pitch -4 and roll -3
      {-10, {-3, 3, 3, -3}, -1},  // forward, rolled 3
      {10, {-2, -2, 2, 2}, -1},   // back, pitched 2
      {-16, {0, 0, 0, 0}, LB},    // LB up
      {22, {-2, 2, 2, -2}, RF},   // RF up, rolled 2
  };
  for (const auto& c : kChecks) {
    all(c.x, dx);
    add(kCheck, "step", StepKind::Move, 15, false, dx, c.dz);
    if (c.leg < 0) {
      add(kCheck, "hold", StepKind::Hold, kStairHold, false, dx, c.dz);
    } else {
      int up[4];
      for (int k = 0; k < 4; k++) up[k] = k == c.leg ? c.dz[k] - kLiftMm : c.dz[k];
      add(kCheck, "hold", StepKind::Hold, 10, false, dx, c.dz);
      add(kCheck, "up", StepKind::Move, 10, false, dx, up);
      add(kCheck, "up_hold", StepKind::Hold, kStairHold, false, dx, up);
      add(kCheck, "down", StepKind::Move, 10, false, dx, c.dz);
    }
  }
  add(kCheck, "home", StepKind::Move, 20, false, zero, zero);
  add(kCheck, "home_hold", StepKind::Hold, kHomeHold, false, zero, zero);
  add(kLevelEnd, "level_end", StepKind::Hold, 50, false, zero, zero);
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
