// errors: the model's error, from what the body reports every tick while it
// walks. A port of mind/kilo/planner/errors.py (docs/walking-and-balance.md
// §9 item 4, §10): measured minus predicted attitude gives
// - a steady offset (the mean): a mass or the IMU placed differently from the model;
// - a lag (the shift that best lines the predicted roll up with the
//   measured one, by cross-correlation): + = the servos are slower than the
//   model's servo lag;
// - a spike (the largest error once the offset is out): a bump, a slip.
// Reported only; no model parameter is adjusted from them yet.
//
// Not ported: errors.from_history (kilo status's estimate from the
// telemetry history; the C++ link keeps no history).
#ifndef KILO_MIND_PLANNER_ERRORS_H
#define KILO_MIND_PLANNER_ERRORS_H
#include <stdint.h>
#include "planner/fmath.h"
#include "planner/model.h"

namespace kilo {
namespace planner {

constexpr int kErrWindowTicks = 200;  // 4 s of 50 Hz telemetry
constexpr int kErrMinSamples = 50;
constexpr int kErrMaxLagTicks = 10;   // +-200 ms around the model's lag
constexpr real kErrMinSwingDeg = 1;   // the predicted roll must move this much (sd) for a lag estimate

struct ErrorSample {
  int64_t tick;  // unwrapped
  real mp, mr;   // measured pitch, roll
  real rp, rr;   // predicted (reference)
  real cp, cr;   // the body's correction
};

struct ErrorEstimate {
  uint32_t samples;
  real offset_pitch_deg, offset_roll_deg;  // rounded to 0.01 as Python's round(x, 2)
  bool has_lag;                            // false: the prediction hardly moves (Python None)
  real lag_ms, lag_quality;
  real spike_deg;
  real corr_pitch_deg, corr_roll_deg;
};

class ErrorTracker {
 public:
  explicit ErrorTracker(real offset_s = 2) : alpha_(kTickDefault / (real)1000.0 / offset_s) {}
  void reset();

  // One TELEMETRY in GAIT (tick unwrapped; angles in TELEMETRY's 1/16 deg)
  // and the prediction for its tick (null: the reference the body reports,
  // 0 while its balance is off). Returns its error less the offset.
  Attitude add(int64_t tick, int16_t pitch, int16_t roll, int16_t ref_pitch, int16_t ref_roll, int16_t corr_pitch,
               int16_t corr_roll, const Attitude* ref);
  // False with fewer than kErrMinSamples samples.
  bool estimate(ErrorEstimate* out) const;

  Attitude offset() const { return offset_; }
  uint32_t count() const { return count_; }

 private:
  static constexpr real kTickDefault = 20;
  const ErrorSample& at(int i) const { return ring_[(head_ + i) % kErrWindowTicks]; }
  void lag(bool* has, real* lag_ms, real* quality) const;

  ErrorSample ring_[kErrWindowTicks] = {};
  int head_ = 0;  // oldest
  int count_ = 0;
  uint32_t n_ = 0;
  real alpha_;
  Attitude offset_ = {0, 0};
};

}  // namespace planner
}  // namespace kilo

#endif
