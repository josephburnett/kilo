// drives: energy, curiosity, sociability, arousal, each in [0, 1]. A port
// of kilo/behaviour/drives.py:
// - energy drains while active (slowly while awake and still), recovers
//   while resting, and is capped by the battery (no cap at battery_full_pct
//   or more, 0 at battery_empty_pct or less, linear between);
// - curiosity grows unless exploring (half as fast while resting) and is
//   satisfied by exploring;
// - sociability grows while alone and drops while a face is seen or Kilo
//   is petted;
// - arousal spikes on a startle or a fast approach and halves every
//   half_life_s.
// Deterministic given dt, the inputs and the RNG (drawn from only when the
// personality's drive noise is > 0).
#ifndef KILO_MIND_BEHAVIOUR_DRIVES_H
#define KILO_MIND_BEHAVIOUR_DRIVES_H
#include <stdint.h>
#include "behaviour/personality.h"
#include "behaviour/random.h"

namespace kilo {
namespace behaviour {

// drives.ACTIVITIES: how the running behaviour spends energy.
enum class Activity : uint8_t { Rest, Idle, Active, Explore, Social };
const char* activity_name(Activity a);

struct DriveInputs {
  Activity activity = Activity::Idle;
  bool moving = false;        // the body reports gait/trick/recovering
  bool has_battery = false;   // battery_pct None: no cap
  double battery_pct = 0;
  bool social = false;        // a face is seen or Kilo is petted
  double startle = 0;         // 0..1 startle strength this tick
  bool approaching = false;   // fast approach on the ultrasonic
};

struct Drives {
  double energy = 0.8, curiosity = 0.2, sociability = 0.3, arousal = 0.0;

  static Drives from_personality(const Personality& p);
  double battery_cap(const Personality& p, bool has_battery, double battery_pct) const;
  void update(double dt, const DriveInputs& in, const Personality& p, PyRandom* rng);
};

}  // namespace behaviour
}  // namespace kilo

#endif
