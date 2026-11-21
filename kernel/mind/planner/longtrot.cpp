#include "planner/longtrot.h"
#include "planner/model.h"

namespace kilo {
namespace planner {

int long_swing_for(real speed_mm_s) {
  real v = m_fabs(speed_mm_s);
  if (v <= kLtVFrom) return kLtWMin;
  int w = kLtWMin + ceil_i((v - kLtVFrom) / kLtVStep - (real)1e-9);
  return w < kLtWMax ? w : kLtWMax;
}

static real fit_x(int w) {  // the effective time on one diagonal over the pendulum's time constant
  real tau = m_sqrt(kFitComHMm / kGmmS2);
  return ((real)(w - 1) * kTickMs + kFitExtraMs) / (real)1000.0 / tau;
}

real fit_k(int w) {
  real x = fit_x(w);
  real e = m_exp(x), ei = (real)1.0 / e;
  real sh = (e - ei) / 2, ch = (e + ei) / 2;
  return (sh - x) / (ch - (real)1.0);
}

real fit_land_x(int w, real v) { return kFitComXMm + fit_k(w) * v * m_sqrt(kFitComHMm / kGmmS2); }

LongTiming long_timing(real speed_mm_s, bool back) {
  real v = speed_mm_s < 0 ? kLongTrot.default_mm_s : speed_mm_s;
  real lo = kStrideMinMm / (((real)kLtWMin + kLtStancePlus) * kTickMs / (real)1000.0);
  v = m_min(back ? kLtTopBackMmS : kLtTopMmS, m_max(lo, v));
  int w = long_swing_for(v);
  real stance = ((real)w + kLtStancePlus) * kTickMs / (real)1000.0;
  real s = m_max(kStrideMinMm, v * stance);
  return {w, s, s / stance};
}

Timing pattern_timing(const Pattern& p, real speed_mm_s, Direction d) {
  if (!is_long_trot(p)) return timing(p, speed_mm_s);
  LongTiming t = long_timing(m_fabs(speed_mm_s), d == Direction::Back);
  real st = ((real)t.swing + kLtStancePlus) * kTickMs;
  return {t.stride_mm, kTickMs, st, st + (real)t.swing * kTickMs, t.speed_mm_s};
}

}  // namespace planner
}  // namespace kilo
