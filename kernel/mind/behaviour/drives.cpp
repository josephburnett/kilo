#include "behaviour/drives.h"
#include "util/dmath.h"

namespace kilo {
namespace behaviour {

namespace {

constexpr double kLn2 = 0.6931471805599453;  // math.log(2), correctly rounded

}  // namespace

Drives Drives::from_personality(const Personality& p) {
  Drives d;
  d.energy = p.initial_energy;
  d.curiosity = p.initial_curiosity;
  d.sociability = p.initial_sociability;
  d.arousal = p.initial_arousal;
  return d;
}

double Drives::battery_cap(const Personality& p, bool has_battery, double pct) const {
  if (!has_battery) return 1.0;
  double lo = p.battery_empty_pct, hi = p.battery_full_pct;
  return d_clamp((pct - lo) / (hi - lo), 0.0, 1.0);
}

void Drives::update(double dt, const DriveInputs& in, const Personality& p, PyRandom* rng) {
  if (dt <= 0) return;
  Activity act = in.activity;
  // energy
  if (act == Activity::Rest && !in.moving) energy += p.recover_rest * dt;
  else if (in.moving || act == Activity::Active || act == Activity::Explore || act == Activity::Social)
    energy -= p.drain_active * dt;
  else energy -= p.drain_idle * dt;
  // curiosity
  if (act == Activity::Explore) curiosity -= p.curiosity_satisfy * dt;
  else if (act != Activity::Rest) curiosity += p.curiosity_growth * dt;
  else curiosity += 0.5 * p.curiosity_growth * dt;  // dreams of elsewhere, slowly
  // sociability
  if (in.social) sociability -= p.sociability_satisfy * dt;
  else sociability += p.sociability_growth * dt;
  // arousal: decay, then spikes
  arousal *= d_exp(-kLn2 * dt / p.arousal_half_life_s);
  if (in.startle > 0) arousal = d_max(arousal, d_min(1.0, in.startle) * p.arousal_startle);
  if (in.approaching) arousal = d_max(arousal, p.arousal_approach);
  // noise
  if (p.drive_noise > 0 && rng != nullptr) {
    double k = p.drive_noise * d_sqrt(dt);
    energy += rng->gauss(0, k);
    curiosity += rng->gauss(0, k);
    sociability += rng->gauss(0, k);
  }
  energy = d_clamp(d_min(energy, battery_cap(p, in.has_battery, in.battery_pct)), 0.0, 1.0);
  curiosity = d_clamp(curiosity, 0.0, 1.0);
  sociability = d_clamp(sociability, 0.0, 1.0);
  arousal = d_clamp(arousal, 0.0, 1.0);
}

}  // namespace behaviour
}  // namespace kilo
