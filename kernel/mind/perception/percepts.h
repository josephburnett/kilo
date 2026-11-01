// percepts: the perception aggregator the behaviour layer reads. A port of
// kilo/perception/percepts.py (Percepts, Sample, extract) and the flag view
// of shared.py: each TELEMETRY feeds the IMU detector, and the ultrasonic
// one once per fresh range (DIST_FRESH, once the body has been seen to set
// it; every sample before). tap and startle read true for event_hold_s
// (1 s) after their event.
//
// The mind feeds it from its LinkListener on every TELEMETRY (as the
// daemon does), with the link's arrival time; nothing else locks or shares
// it (one cooperative loop: Python's SharedPercepts lock is not needed).
//
// What differs: the 10 s TelemetryRing is not ported (no detector reads it:
// the IMU keeps its own window); recent events are a ring of kRecent
// instead of a deque of 256.
#ifndef KILO_MIND_PERCEPTION_PERCEPTS_H
#define KILO_MIND_PERCEPTION_PERCEPTS_H
#include <stdint.h>
#include "perception/imu.h"
#include "perception/ultrasonic.h"
#include "proto/kilo_protocol.h"

namespace kilo {
namespace perception {

// percepts.py Sample: one telemetry sample, normalised.
struct Sample {
  double t;
  double pitch, roll;
  bool has_rates;
  double rates[3];
  bool has_dist;     // false: no echo (0) or unknown
  double dist_mm;
  int8_t dist_fresh; // -1 unknown, 0, 1
  int mode;          // -1 unknown
  bool has_flags;
  uint16_t flags;
  bool same(const Sample& o) const;
};

// percepts.py _from_raw: a TELEMETRY as a Sample at time t (seconds).
Sample sample_from_telemetry(const kilo_telemetry_t& m, double t);

class Percepts {
 public:
  static constexpr int kRecent = 16;

  explicit Percepts(const UltrasonicConfig& us = UltrasonicConfig(), const ImuConfig& imu = ImuConfig(),
                    double event_hold_s = 1.0)
      : us_(us), imu_(imu), hold_s_(event_hold_s) {}
  void reset();
  // The mind starts a posture move or similar: the IMU detectors stand down.
  void notify_self_motion(double t, double seconds) { imu_.notify_self_motion(t, seconds); }
  // One sample; returns the events it produced (none for the same sample twice).
  const PerceptEvents& update(const Sample& s);
  const PerceptEvents& update(const kilo_telemetry_t& m, double t) { return update(sample_from_telemetry(m, t)); }

  bool has_t() const { return has_t_; }
  double t() const { return t_; }
  bool near() const { return us_.near(); }
  bool approaching() const { return us_.approaching(); }
  bool clear() const { return us_.clear(); }
  bool tilted() const { return imu_.tilted(); }
  bool picked_up() const { return imu_.picked_up(); }
  bool pet() const { return imu_.pet(); }
  bool tap() const { return recent(Percept::Tap); }
  bool startle() const { return recent(Percept::Startle); }
  // When the last event of a kind came (false: never).
  bool last_event_time(Percept p, double* t) const;
  const UltrasonicDetector& ultrasonic() const { return us_; }
  const ImuDetector& imu() const { return imu_; }
  // The last kRecent events, oldest first: i in 0..recent_count().
  int recent_count() const { return recent_n_; }
  const PerceptEvent& recent_event(int i) const { return recent_[(recent_head_ + i) % kRecent]; }
  uint32_t event_total() const { return event_total_; }

 private:
  bool recent(Percept p) const;

  UltrasonicDetector us_;
  ImuDetector imu_;
  double hold_s_;
  bool has_t_ = false;
  double t_ = 0;
  bool has_sample_ = false;
  Sample sample_ = {};
  bool seen_fresh_ = false;
  PerceptEvents new_;
  bool has_last_[(int)Percept::Count] = {};
  double last_[(int)Percept::Count] = {};
  PerceptEvent recent_[kRecent];
  int recent_head_ = 0, recent_n_ = 0;
  uint32_t event_total_ = 0;
};

}  // namespace perception
}  // namespace kilo

#endif
