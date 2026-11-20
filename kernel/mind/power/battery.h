// battery: the pack's voltage over the last half hour, from TELEMETRY, so
// that the owner can compare how fast it falls awake and asleep (Kilo has
// no current sensor: the voltage's slope is the only measure of the draw
// there is; docs/power.md).
//
// Every TELEMETRY's batt_mv (the body's filtered reading) goes into a
// 10 s bucket; each full bucket is one sample: its mean and the power
// state it was taken in. The last kSamples samples (30 min) stay.
//
// trend(): the least-squares slope in mV per minute over the samples of
// the last window, in tenths (-84 is -8.4 mV/min), or kNoTrend with fewer
// than kMinSamples. Segments: a stretch of one power state; when it ends
// its slope is kept (the last kSegments), measured from kSettleUs after
// the state began, because the voltage jumps when the load changes (it
// sags under load and climbs back after) and that jump is not the drain.
#ifndef KILO_MIND_POWER_BATTERY_H
#define KILO_MIND_POWER_BATTERY_H
#include <stdint.h>

namespace kilo {
namespace power {

// The battery as it would read at rest (2026-11-19: at a wall the guard
// backed Kilo off, then the behaviour runner's low-battery branch laid him
// down in REST: lying still he read 7408 mV, above the 20 % mark of
// 7300 mV, so the reading that tripped it was most likely the sag while
// the legs moved). Moving legs only pull the pack's voltage down, and it
// comes back within about a second once they stop or between steps, so
// the highest reading of the last kWindowS is what the pack holds at rest,
// give or take its drain over those seconds (about 30 mV at docs/power.md's
// 90 mV a minute). Kept as the highest reading of each second, for the last
// kWindowS seconds. A body on USB power (NO_BATTERY) or no reading for
// kWindowS: none. During a walk that never pauses for kWindowS the
// estimate is the walk's lightest moment, still a little low.
//
// What judges the battery uses it: the behaviour runner's low and critical
// battery (Mind::Body::snapshot), the eyes' orange (Mind, the guard). What
// shows the reading (kilo status, the metrics' mv=, the trend) and the
// rear-up's 7400 mV refusal keep the raw reading.
class RestedBattery {
 public:
  static constexpr uint32_t kWindowS = 20;

  // A TELEMETRY at now_us: its reading (0: none), or USB power (none at all: everything forgotten).
  void add(uint64_t now_us, uint16_t mv, bool no_battery);
  // The highest reading of the last kWindowS seconds; false if none.
  bool mv(uint64_t now_us, uint16_t* out) const;

 private:
  uint16_t max_[kWindowS] = {};   // by second (now_s % kWindowS), 0: none
  uint32_t sec_[kWindowS] = {};   // ... which second it holds
};

class BatteryTrend {
 public:
  static constexpr uint64_t kBucketUs = 10000000;   // one sample: the mean of 10 s of TELEMETRY
  static constexpr uint32_t kSamples = 180;         // 30 min of them
  static constexpr uint32_t kMinSamples = 3;        // a slope needs 30 s at least
  static constexpr uint64_t kSettleUs = 60000000;   // a segment's slope leaves out its first minute
  static constexpr uint32_t kSegments = 4;
  static constexpr int32_t kNoTrend = 0x7FFFFFFF;

  struct Sample {
    uint32_t t_s;   // the clock's seconds at the bucket's end
    uint16_t mv;    // the bucket's mean
    uint8_t state;  // the power state (sleep.h State) it was taken in
  };
  struct Segment {
    uint8_t state;
    uint32_t start_s, end_s;
    uint16_t first_mv, last_mv;  // the first and last sample in it
    int32_t slope_x10;           // mV/min x 10 after kSettleUs; kNoTrend if too short
  };

  // A reading at now_us in power state `state`. 0 mV (no reading) is skipped.
  void add(uint64_t now_us, uint16_t mv, uint8_t state);
  // The power state changed at now_us: the running segment ends.
  void state_changed(uint64_t now_us, uint8_t state);

  // mV/min x 10 over the samples of the last window_s seconds (any state).
  int32_t trend(uint64_t now_us, uint32_t window_s) const;
  // ... over the running segment, after its first kSettleUs.
  int32_t segment_trend() const;
  uint32_t samples() const { return n_; }
  const Sample& sample(uint32_t i) const;  // 0 = the oldest kept
  uint32_t segments() const { return n_seg_; }
  const Segment& segment(uint32_t i) const { return seg_[i]; }  // 0 = the most recent finished one
  uint16_t last_mv() const { return last_mv_; }
  uint16_t min_mv() const { return min_mv_; }

 private:
  int32_t slope(uint32_t from_s, uint32_t to_s, bool any_state, uint8_t state) const;

  Sample ring_[kSamples] = {};
  uint32_t head_ = 0, n_ = 0;     // the oldest sample's slot, samples kept
  uint64_t bucket_end_us_ = 0;    // 0: no bucket open
  uint32_t bucket_sum_ = 0, bucket_n_ = 0;
  uint8_t bucket_state_ = 0;
  uint16_t last_mv_ = 0, min_mv_ = 0;
  // the running segment
  uint8_t cur_state_ = 0;
  uint32_t cur_start_s_ = 0;
  Segment seg_[kSegments] = {};
  uint32_t n_seg_ = 0;
};

}  // namespace power
}  // namespace kilo

#endif
