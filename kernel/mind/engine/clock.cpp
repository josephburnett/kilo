#include "engine/clock.h"

namespace kilo {
namespace engine {

using planner::m_fabs;

static real frac(real x) {
  int32_t i = planner::floor_i(x);
  return x - (real)i;
}

void StepClock::start(const real offsets[kLegs], real cycle_ticks, real phase) {
  set_cycle(cycle_ticks);
  phase_ = frac(phase);
  for (int i = 0; i < kLegs; i++) lift_at_[i] = target_[i] = offsets[i];
}

real StepClock::to_lift(int leg) const {
  real d = frac(lift_at_[leg] - phase_);
  return d;
}

uint8_t StepClock::advance(bool wait) {
  if (wait) return 0;
  real d = (real)1.0 / cycle_;
  uint8_t out = 0;
  for (int i = 0; i < kLegs; i++) {
    real u = frac(lift_at_[i] - phase_);  // how far ahead the lift is
    // a lift exactly at the phase now was passed on the tick that reached it
    if (u > (real)1e-4 && u <= d + (real)1e-4) out |= (uint8_t)(1u << i);
    else if (u > (real)1.0 - (real)1e-4 && d >= (real)1.0) out |= (uint8_t)(1u << i);
  }
  phase_ = frac(phase_ + d);
  return out;
}

void StepClock::preview(int n, uint8_t* out) const {
  StepClock c = *this;
  for (int k = 0; k < n; k++) out[k] = c.advance(false);
}

void StepClock::slew_offsets(uint8_t swinging) {
  real d = (real)1.0 / cycle_;
  for (int i = 0; i < kLegs; i++) {
    real diff = target_[i] - lift_at_[i];
    if (diff > (real)0.5) diff -= 1;
    if (diff < (real)-0.5) diff += 1;
    if (m_fabs(diff) < (real)1e-6) continue;
    if (swinging & (1u << i)) continue;
    // not about to lift: the lift point stays more than two ticks ahead after the move
    real step = diff > kOffsetSlew ? kOffsetSlew : diff < -kOffsetSlew ? -kOffsetSlew : diff;
    real ahead = frac(lift_at_[i] + step - phase_);
    if (ahead < 3 * d || frac(lift_at_[i] - phase_) < 3 * d) continue;
    lift_at_[i] = frac(lift_at_[i] + step);
  }
}

bool StepClock::offsets_settled() const {
  for (int i = 0; i < kLegs; i++) {
    real diff = m_fabs(target_[i] - lift_at_[i]);
    if (diff > (real)1e-6 && diff < (real)1.0 - (real)1e-6) return false;
  }
  return true;
}

}  // namespace engine
}  // namespace kilo
