#include "perception/ultrasonic.h"
#include "util/dmath.h"

namespace kilo {
namespace perception {

const char* percept_name(Percept p) {
  switch (p) {
    case Percept::Near: return "near";
    case Percept::Approaching: return "approaching";
    case Percept::Clear: return "clear";
    case Percept::Tilted: return "tilted";
    case Percept::Level: return "level";
    case Percept::PickedUp: return "picked_up";
    case Percept::PutDown: return "put_down";
    case Percept::Tap: return "tap";
    case Percept::Pet: return "pet";
    case Percept::Startle: return "startle";
    default: return "?";
  }
}

const char* tilt_direction_name(TiltDirection d) {
  switch (d) {
    case TiltDirection::NoseDown: return "nose_down";
    case TiltDirection::NoseUp: return "nose_up";
    case TiltDirection::RightUp: return "right_up";
    case TiltDirection::LeftUp: return "left_up";
    default: return "";
  }
}

void UltrasonicDetector::reset() {
  raw_n_ = 0;
  hist_head_ = hist_n_ = 0;
  has_distance_ = false;
  has_rate_ = false;
  zone_ = Zone::Unknown;
  approaching_ = false;
  has_last_valid_ = false;
  has_first_ = false;
  pending_ = Zone::Unknown;
  pending_n_ = 0;
  neg_n_ = rel_n_ = 0;
  has_last_approach_ = false;
}

void UltrasonicDetector::hist_pop_front() {
  hist_head_ = (hist_head_ + 1) % kHist;
  hist_n_--;
}

void UltrasonicDetector::feed(double t, bool has_distance, double distance_mm, PerceptEvents& out) {
  const UltrasonicConfig& c = cfg_;
  if (!has_first_) {
    has_first_ = true;
    first_t_ = t;
  }
  bool valid = has_distance && 0 < distance_mm && distance_mm <= c.max_range_mm;
  if (!valid) {
    double since = has_last_valid_ ? last_valid_t_ : first_t_;
    if (t - since >= c.no_echo_clear_s) {
      raw_n_ = 0;
      hist_head_ = hist_n_ = 0;
      has_distance_ = false;
      has_rate_ = false;
      approaching_ = false;
      neg_n_ = rel_n_ = 0;
      pending_ = Zone::Unknown;
      pending_n_ = 0;
      if (zone_ != Zone::Clear) {
        if (zone_ != Zone::Unknown) out.add(t, Percept::Clear).no_echo = true;
        zone_ = Zone::Clear;
      }
    }
    return;
  }

  has_last_valid_ = true;
  last_valid_t_ = t;
  // the last 3 echoes (deque maxlen 3), none older than stale_s
  if (raw_n_ == 3) {
    raw_[0] = raw_[1];
    raw_[1] = raw_[2];
    raw_n_ = 2;
  }
  raw_[raw_n_++] = {t, distance_mm};
  while (raw_n_ > 0 && t - raw_[0].t > c.stale_s) {
    for (int i = 1; i < raw_n_; i++) raw_[i - 1] = raw_[i];
    raw_n_--;
  }
  double f;  // statistics.median
  if (raw_n_ == 1) {
    f = raw_[0].v;
  } else if (raw_n_ == 2) {
    f = (raw_[0].v + raw_[1].v) / 2;
  } else {
    double a = raw_[0].v, b = raw_[1].v, x = raw_[2].v;
    f = a > b ? (b > x ? b : (a > x ? x : a)) : (a > x ? a : (b > x ? x : b));
  }
  if (hist_n_ > 0 && d_fabs(f - hist(hist_n_ - 1).v) > c.jump_mm) hist_head_ = hist_n_ = 0;
  if (hist_n_ == kHist) hist_pop_front();
  hist_[(hist_head_ + hist_n_++) % kHist] = {t, f};
  while (hist_n_ > 0 && t - hist(0).t > c.speed_window_s) hist_pop_front();
  has_distance_ = true;
  distance_mm_ = f;
  has_rate_ = hist_n_ >= c.min_speed_points;
  if (has_rate_) {  // _slope(): least squares, Python's sums
    PySum st, sv;
    for (int i = 0; i < hist_n_; i++) {
      st.add(hist(i).t);
      sv.add(hist(i).v);
    }
    double tm = st.value() / hist_n_, vm = sv.value() / hist_n_;
    PySum sxx, sxy;
    for (int i = 0; i < hist_n_; i++) {
      double dt = hist(i).t - tm;
      sxx.add(dt * dt);
    }
    double xx = sxx.value();
    if (xx <= 0) {
      rate_ = 0.0;
    } else {
      for (int i = 0; i < hist_n_; i++) sxy.add((hist(i).t - tm) * (hist(i).v - vm));
      rate_ = sxy.value() / xx;
    }
  }
  zone_step(t, f, out);
  approach_step(t, f, out);
}

void UltrasonicDetector::zone_step(double t, double f, PerceptEvents& out) {
  const UltrasonicConfig& c = cfg_;
  Zone z = zone_;
  Zone cand;
  if (f < c.near_mm || (z == Zone::Near && f <= c.near_exit_mm)) cand = Zone::Near;
  else if (f > c.clear_mm || (z == Zone::Clear && f >= c.clear_exit_mm)) cand = Zone::Clear;
  else cand = Zone::Mid;
  if (cand == z) {
    pending_ = Zone::Unknown;
    pending_n_ = 0;
    return;
  }
  if (cand == pending_) {
    pending_n_++;
  } else {
    pending_ = cand;
    pending_n_ = 1;
  }
  if (pending_n_ < c.confirm_samples) return;
  zone_ = cand;
  pending_ = Zone::Unknown;
  pending_n_ = 0;
  if (cand == Zone::Mid || (z == Zone::Unknown && cand != Zone::Near)) return;
  PerceptEvent& e = out.add(t, cand == Zone::Near ? Percept::Near : Percept::Clear);
  e.has_a = true;
  e.a = f;
}

void UltrasonicDetector::approach_step(double t, double f, PerceptEvents& out) {
  const UltrasonicConfig& c = cfg_;
  if (has_rate_ && rate_ < c.approach_mm_s && f <= c.approach_max_mm) neg_n_++;
  else neg_n_ = 0;
  if (!approaching_) {
    if (neg_n_ >= c.approach_samples) {
      approaching_ = true;
      rel_n_ = 0;
      if (!has_last_approach_ || t - last_approach_ev_ >= c.approach_cooldown_s) {
        has_last_approach_ = true;
        last_approach_ev_ = t;
        PerceptEvent& e = out.add(t, Percept::Approaching);
        e.has_a = e.has_b = true;
        e.a = f;
        e.b = -rate_;
      }
    }
    return;
  }
  if (!has_rate_ || rate_ > c.approach_release_mm_s) {
    if (++rel_n_ >= c.approach_samples) approaching_ = false;
  } else {
    rel_n_ = 0;
  }
}

}  // namespace perception
}  // namespace kilo
