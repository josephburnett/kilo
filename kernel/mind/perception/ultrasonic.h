// ultrasonic: near / approaching / clear from the body's range readings.
// A port of kilo/perception/ultrasonic.py, the same pipeline and defaults:
//
// 1. median of the last 3 valid echoes no older than stale_s;
// 2. range rate (mm/s, negative = closer) by least squares over the
//    filtered values of the last speed_window_s (at least min_speed_points);
//    a filtered jump over jump_mm restarts the window;
// 3. zones with hysteresis, each change confirmed by confirm_samples:
//    near < 150 mm (left > 200), clear > 400 mm (left < 350) or no echo for
//    no_echo_clear_s, mid between;
// 4. approaching: rate < -100 mm/s for 3 samples within 2 m, released after
//    3 samples above -40 mm/s or on a sustained no-echo.
//
// Events: near and clear on entering the zone (the first zone after start
// is silent unless it is near), approaching on the rising edge at most once
// per approach_cooldown_s. Fed once per fresh range (12.5 Hz on the body).
//
// What differs: Python's deques are fixed rings here; the speed window holds
// at most kHist values (64: 0.8 s at 80 Hz), the oldest dropped beyond.
#ifndef KILO_MIND_PERCEPTION_ULTRASONIC_H
#define KILO_MIND_PERCEPTION_ULTRASONIC_H
#include <stdint.h>
#include "perception/events.h"

namespace kilo {
namespace perception {

struct UltrasonicConfig {
  double near_mm = 150.0;
  double near_exit_mm = 200.0;
  double clear_mm = 400.0;
  double clear_exit_mm = 350.0;
  int confirm_samples = 2;
  double max_range_mm = 4000.0;
  double stale_s = 0.5;
  double no_echo_clear_s = 1.0;
  double speed_window_s = 0.8;
  int min_speed_points = 4;
  double jump_mm = 200.0;
  double approach_mm_s = -100.0;
  double approach_release_mm_s = -40.0;
  int approach_samples = 3;
  double approach_max_mm = 2000.0;
  double approach_cooldown_s = 2.0;
};

enum class Zone : uint8_t { Unknown, Near, Mid, Clear };

class UltrasonicDetector {
 public:
  explicit UltrasonicDetector(const UltrasonicConfig& cfg = UltrasonicConfig()) : cfg_(cfg) { reset(); }
  void reset();
  // One range: has_distance false (or 0, or beyond max_range_mm) is a dropout.
  void feed(double t, bool has_distance, double distance_mm, PerceptEvents& out);

  Zone zone() const { return zone_; }
  bool near() const { return zone_ == Zone::Near; }
  bool clear() const { return zone_ == Zone::Clear; }
  bool approaching() const { return approaching_; }
  bool has_distance() const { return has_distance_; }
  double distance_mm() const { return distance_mm_; }  // filtered
  bool has_rate() const { return has_rate_; }
  double range_rate_mm_s() const { return rate_; }      // negative = closer
  double approach_mm_s() const { return -rate_; }       // positive = closer (as BodyState)

 private:
  struct Pt {
    double t, v;
  };
  static constexpr int kHist = 64;
  void zone_step(double t, double f, PerceptEvents& out);
  void approach_step(double t, double f, PerceptEvents& out);
  void hist_pop_front();
  const Pt& hist(int i) const { return hist_[(hist_head_ + i) % kHist]; }

  UltrasonicConfig cfg_;
  Pt raw_[3];
  int raw_n_ = 0;
  Pt hist_[kHist];
  int hist_head_ = 0, hist_n_ = 0;
  bool has_distance_ = false;
  double distance_mm_ = 0;
  bool has_rate_ = false;
  double rate_ = 0;
  Zone zone_ = Zone::Unknown;
  bool approaching_ = false;
  bool has_last_valid_ = false;
  double last_valid_t_ = 0;
  bool has_first_ = false;
  double first_t_ = 0;
  Zone pending_ = Zone::Unknown;
  int pending_n_ = 0;
  int neg_n_ = 0;
  int rel_n_ = 0;
  bool has_last_approach_ = false;
  double last_approach_ev_ = 0;
};

}  // namespace perception
}  // namespace kilo

#endif
