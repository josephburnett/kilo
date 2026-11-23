// clock: the one step clock every gait runs on (docs/movement-engine.md, step 2).
//
// One phase, 0..1 over a cycle, advancing every tick by 1 / cycle (the
// cadence: a dial, changed at any tick). Each leg lifts when the phase passes
// its own point in the cycle (its offset, from the gait: the walk's four a
// quarter apart, the trot's two pairs half a cycle apart). The swing that
// follows is the solver's: the clock only says when.
//
// The clock can wait: a tick it does not advance (the solver holds a lift
// while the body is off the plan, or while the weight is not yet where the
// lift needs it). Its offsets can move under way (a longer swing, another
// cadence: the four-foot time shared out again), at most kOffsetSlew of a
// cycle a tick and only while the leg is on the ground and not about to
// lift, so no leg's swing is cut or doubled.
#ifndef KILO_MIND_ENGINE_CLOCK_H
#define KILO_MIND_ENGINE_CLOCK_H
#include <stdint.h>
#include "engine/gait.h"

namespace kilo {
namespace engine {

constexpr real kOffsetSlew = (real)0.01;  // an offset moves at most this much of a cycle a tick

class StepClock {
 public:
  void start(const real offsets[kLegs], real cycle_ticks, real phase = 0);
  // One tick: the phase advances by 1 / cycle unless waiting; returns the
  // legs whose lift point it passed (bit i: leg i).
  uint8_t advance(bool wait = false);
  void set_cycle(real cycle_ticks) { cycle_ = cycle_ticks < 2 ? 2 : cycle_ticks; }
  // Move the offsets toward these (a new swing or cadence, a gait change under way), kOffsetSlew a tick.
  void retarget(const real offsets[kLegs]) {
    for (int i = 0; i < kLegs; i++) target_[i] = offsets[i];
  }
  void slew_offsets(uint8_t swinging);
  real phase() const { return phase_; }
  real cycle() const { return cycle_; }
  real offset(int leg) const { return lift_at_[leg]; }

 private:
  real phase_ = 0;
  real cycle_ = 48;
  real lift_at_[kLegs] = {};
  real target_[kLegs] = {};
};

}  // namespace engine
}  // namespace kilo

#endif
