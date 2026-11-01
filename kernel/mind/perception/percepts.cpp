#include "perception/percepts.h"

namespace kilo {
namespace perception {

namespace {
constexpr double kA16 = 16.0;  // 1/16 degree
}

bool Sample::same(const Sample& o) const {
  if (t != o.t || pitch != o.pitch || roll != o.roll || has_rates != o.has_rates || has_dist != o.has_dist ||
      dist_fresh != o.dist_fresh || mode != o.mode || has_flags != o.has_flags)
    return false;
  if (has_rates && (rates[0] != o.rates[0] || rates[1] != o.rates[1] || rates[2] != o.rates[2])) return false;
  if (has_dist && dist_mm != o.dist_mm) return false;
  return !has_flags || flags == o.flags;
}

Sample sample_from_telemetry(const kilo_telemetry_t& m, double t) {
  Sample s = {};
  s.t = t;
  s.pitch = m.pitch / kA16;
  s.roll = m.roll / kA16;
  s.has_rates = true;
  s.rates[0] = m.rate_pitch / kA16;
  s.rates[1] = m.rate_roll / kA16;
  s.rates[2] = m.rate_yaw / kA16;
  s.has_dist = m.dist_mm != 0;
  s.dist_mm = m.dist_mm;
  s.has_flags = true;
  s.flags = m.flags;
  s.dist_fresh = (m.flags & KILO_TELEMETRY_FLAGS_DIST_FRESH) ? 1 : 0;
  s.mode = m.mode;
  return s;
}

void Percepts::reset() {
  us_.reset();
  imu_.reset();
  has_t_ = false;
  has_sample_ = false;
  seen_fresh_ = false;
  for (int i = 0; i < (int)Percept::Count; i++) has_last_[i] = false;
  recent_head_ = recent_n_ = 0;
}

const PerceptEvents& Percepts::update(const Sample& s) {
  new_.n = 0;
  double now = s.t;
  if (has_t_ && now <= t_ && has_sample_ && s.same(sample_)) return new_;  // the same telemetry polled twice
  has_t_ = true;
  t_ = now;
  has_sample_ = true;
  sample_ = s;
  if (s.dist_fresh == 1) seen_fresh_ = true;
  if (s.dist_fresh == 1 || !seen_fresh_) us_.feed(now, s.has_dist, s.dist_mm, new_);
  ImuSample im;
  im.t = now;
  im.pitch = s.pitch;
  im.roll = s.roll;
  im.has_rates = s.has_rates;
  for (int i = 0; i < 3; i++) im.rates[i] = s.rates[i];
  im.has_flags = s.has_flags;
  im.flags = s.flags;
  im.mode = s.mode;
  imu_.feed(im, new_);
  for (int i = 0; i < new_.n; i++) {
    const PerceptEvent& e = new_.e[i];
    has_last_[(int)e.kind] = true;
    last_[(int)e.kind] = e.t;
    if (recent_n_ == kRecent) {
      recent_head_ = (recent_head_ + 1) % kRecent;
      recent_n_--;
    }
    recent_[(recent_head_ + recent_n_++) % kRecent] = e;
    event_total_++;
  }
  return new_;
}

bool Percepts::recent(Percept p) const {
  return has_last_[(int)p] && has_t_ && t_ - last_[(int)p] <= hold_s_;
}

bool Percepts::last_event_time(Percept p, double* t) const {
  if (!has_last_[(int)p]) return false;
  *t = last_[(int)p];
  return true;
}

}  // namespace perception
}  // namespace kilo
