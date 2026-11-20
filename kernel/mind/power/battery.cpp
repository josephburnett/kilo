#include "power/battery.h"

namespace kilo {
namespace power {

const BatteryTrend::Sample& BatteryTrend::sample(uint32_t i) const { return ring_[(head_ + i) % kSamples]; }

void BatteryTrend::add(uint64_t now_us, uint16_t mv, uint8_t state) {
  if (mv == 0) return;
  last_mv_ = mv;
  if (min_mv_ == 0 || mv < min_mv_) min_mv_ = mv;
  if (bucket_end_us_ != 0 && (now_us >= bucket_end_us_ || state != bucket_state_)) {
    if (bucket_n_ != 0 && now_us >= bucket_end_us_) {  // a full bucket: one sample (a cut-short one is dropped)
      Sample s = {(uint32_t)(bucket_end_us_ / 1000000u), (uint16_t)(bucket_sum_ / bucket_n_), bucket_state_};
      if (n_ < kSamples) ring_[(head_ + n_++) % kSamples] = s;
      else {
        ring_[head_] = s;
        head_ = (head_ + 1) % kSamples;
      }
    }
    bucket_end_us_ = 0;
  }
  if (bucket_end_us_ == 0) {
    bucket_end_us_ = now_us + kBucketUs;
    bucket_sum_ = 0;
    bucket_n_ = 0;
    bucket_state_ = state;
  }
  bucket_sum_ += mv;
  bucket_n_++;
}

int32_t BatteryTrend::slope(uint32_t from_s, uint32_t to_s, bool any_state, uint8_t state) const {
  // least squares on (t, mv), t relative to from_s (in doubles: no libm needed)
  double n = 0, st = 0, sm = 0, stt = 0, stm = 0;
  for (uint32_t i = 0; i < n_; i++) {
    const Sample& s = sample(i);
    if (s.t_s < from_s || s.t_s > to_s || (!any_state && s.state != state)) continue;
    double t = (double)(s.t_s - from_s), m = (double)s.mv;
    n += 1;
    st += t;
    sm += m;
    stt += t * t;
    stm += t * m;
  }
  if (n < kMinSamples) return kNoTrend;
  double d = n * stt - st * st;
  if (d <= 0) return kNoTrend;
  double per_s = (n * stm - st * sm) / d;
  double x10 = per_s * 600.0;  // mV/s -> mV/min x 10
  return (int32_t)(x10 < 0 ? x10 - 0.5 : x10 + 0.5);
}

int32_t BatteryTrend::trend(uint64_t now_us, uint32_t window_s) const {
  uint32_t now_s = (uint32_t)(now_us / 1000000u);
  return slope(now_s > window_s ? now_s - window_s : 0, now_s, true, 0);
}

int32_t BatteryTrend::segment_trend() const {
  return slope(cur_start_s_ + (uint32_t)(kSettleUs / 1000000u), 0xFFFFFFFFu, false, cur_state_);
}

void BatteryTrend::state_changed(uint64_t now_us, uint8_t state) {
  uint32_t now_s = (uint32_t)(now_us / 1000000u);
  Segment g = {cur_state_, cur_start_s_, now_s, 0, 0, segment_trend()};
  for (uint32_t i = 0; i < n_; i++) {  // the segment's first and last sample
    const Sample& s = sample(i);
    if (s.t_s < cur_start_s_ || s.state != cur_state_) continue;
    if (g.first_mv == 0) g.first_mv = s.mv;
    g.last_mv = s.mv;
  }
  for (uint32_t i = (n_seg_ < kSegments ? n_seg_ : kSegments - 1); i > 0; i--) seg_[i] = seg_[i - 1];
  seg_[0] = g;
  if (n_seg_ < kSegments) n_seg_++;
  cur_state_ = state;
  cur_start_s_ = now_s;
}

void RestedBattery::add(uint64_t now_us, uint16_t mv, bool no_battery) {
  if (no_battery) {
    for (uint32_t i = 0; i < kWindowS; i++) max_[i] = 0;
    return;
  }
  if (mv == 0) return;
  uint32_t s = (uint32_t)(now_us / 1000000u);
  uint32_t k = s % kWindowS;
  if (sec_[k] != s) {  // a new second in this slot: the old one (kWindowS ago) leaves
    sec_[k] = s;
    max_[k] = 0;
  }
  if (mv > max_[k]) max_[k] = mv;
}

bool RestedBattery::mv(uint64_t now_us, uint16_t* out) const {
  uint32_t s = (uint32_t)(now_us / 1000000u);
  uint16_t best = 0;
  for (uint32_t i = 0; i < kWindowS; i++)
    if (max_[i] != 0 && s - sec_[i] < kWindowS && max_[i] > best) best = max_[i];
  if (best == 0) return false;
  *out = best;
  return true;
}

}  // namespace power
}  // namespace kilo
