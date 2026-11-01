// personality: drive rates, behaviour weights and selector tuning. Python
// counterpart: kilo/behaviour/personality.py, which loads and validates
// personality.yaml over its DEFAULTS. The mind has no YAML parser: the
// laptop converts the file (personality.load(), so the merge, the
// validation and the defaults are Python's) into personality.gen.cpp with
// behaviour/gen_personality.py, and the mind reads the table.
#ifndef KILO_MIND_BEHAVIOUR_PERSONALITY_H
#define KILO_MIND_BEHAVIOUR_PERSONALITY_H
#include <stdint.h>

namespace kilo {
namespace behaviour {

// personality.BEHAVIOURS, in order (the selector's order).
enum BehaviourId : uint8_t { kIdle, kExplore, kNap, kGreet, kStartle, kBehaviours };
// personality.TERMS, in order.
enum Term : uint8_t {
  kBase, kEnergy, kFatigue, kCuriosity, kSociability, kArousal, kCalm, kFace, kPet, kNear, kApproaching, kAlone,
  kTerms
};

const char* behaviour_name(uint8_t b);
const char* term_name(uint8_t t);

// One `term: weight` of a behaviour; a product "a*b" has up to kMaxFactors factors.
struct Weight {
  static constexpr int kMaxFactors = 3;
  uint8_t n;  // factors
  uint8_t factors[kMaxFactors];
  double w;
};

struct Personality {
  static constexpr int kMaxWeights = 8;  // per behaviour
  const char* name;
  // drives
  double initial_energy, initial_curiosity, initial_sociability, initial_arousal;
  double drain_active, drain_idle, recover_rest, battery_full_pct, battery_empty_pct;
  double curiosity_growth, curiosity_satisfy;
  double sociability_growth, sociability_satisfy;
  double arousal_startle, arousal_approach, arousal_half_life_s;
  double drive_noise;
  // weights, in the file's order (the order Python sums them in)
  uint8_t n_weights[kBehaviours];
  Weight weights[kBehaviours][kMaxWeights];
  // selector
  double sel_noise, sel_noise_period_s, sel_margin, sel_min_dwell_s;
  uint8_t urgent;  // bit per BehaviourId
  // safety
  double low_battery_pct, critical_battery_pct, wind_down_s, fallen_settle_s;
  // percepts (the snapshot's ultrasonic fields)
  double near_mm, approach_mm_s;
  // behaviours
  const char* idle_posture;
  double idle_look_every_s, idle_look_chance, idle_look_pan_deg, idle_look_tilt_deg;
  const char* explore_posture;
  double explore_look_every_s;
  uint32_t explore_looks;
  double explore_look_pan_deg, explore_look_tilt_deg;
  const char* nap_posture;
  double nap_min_s;
  const char* greet_trick;
  double greet_cooldown_s;
  double startle_hold_s;
};

// mind/kilo/behaviour/personality.yaml, as personality.load() reads it.
extern const Personality kPersonality;

}  // namespace behaviour
}  // namespace kilo

#endif
