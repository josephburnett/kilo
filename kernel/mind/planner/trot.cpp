#include "planner/trot.h"
#include "planner/model.h"
#include "planner/patterns.h"

namespace kilo {
namespace planner {

int trot_swing_for(real speed_mm_s) {
  real v = m_fabs(speed_mm_s);
  if (v <= kTrotVFrom) return kTrotWMin;
  int w = kTrotWMin + ceil_i((v - kTrotVFrom) / kTrotVStep - (real)1e-9);
  return w < kTrotWMax ? w : kTrotWMax;
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

}  // namespace planner
}  // namespace kilo
