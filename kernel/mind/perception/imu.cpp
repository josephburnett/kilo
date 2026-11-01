#include "perception/imu.h"
#include "proto/kilo_protocol.h"
#include "util/dmath.h"

namespace kilo {
namespace perception {

namespace {

constexpr uint16_t kSelfMotionFlags = KILO_TELEMETRY_FLAGS_GAIT | KILO_TELEMETRY_FLAGS_TRICK;

bool self_motion_mode(int mode) {  // imu.py SELF_MOTION_MODES
  return mode == KILO_MODE_GAIT || mode == KILO_MODE_TRICK || mode == KILO_MODE_RECOVERING ||
         mode == KILO_MODE_CALIB || mode == KILO_MODE_BOOT;
}

TiltDirection direction(double dp, double dr) {
  if (d_fabs(dp) >= d_fabs(dr)) return dp > 0 ? TiltDirection::NoseDown : TiltDirection::NoseUp;
  return dr > 0 ? TiltDirection::RightUp : TiltDirection::LeftUp;
}

// statistics.median of v[0..n) (sorted in place)
double median(double* v, int n) {
  for (int i = 1; i < n; i++) {
    double x = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > x) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = x;
  }
  return n % 2 ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2;
}

// statistics.fmean: fsum / n
double fmean(const double* v, int n) {
  FSum s;
  for (int i = 0; i < n; i++) s.add(v[i]);
  return s.value() / n;
}

}  // namespace

void ImuDetector::reset() {
  const ImuConfig& c = cfg_;
  keep_s_ = d_max(d_max(d_max(c.pet_window_s, c.pre_window_s), c.put_down_calm_s), c.pickup_window_s) + 0.5;
  head_ = n_ = 0;
  d_head_ = d_n_ = 0;
  ref_p_ = ref_r_ = 0;
  ref_fixed_ = false;
  has_relearn_ = false;
  mode_ = -1;
  quiet_from_ = kNegInf;
  self_moving_ = false;
  has_att_ = false;
  tilted_ = false;
  tilt_dir_ = TiltDirection::None;
  has_tilt_since_ = false;
  picked_up_ = false;
  has_pick_since_ = false;
  tap_state_ = TapState::Idle;
  tap_t0_ = tap_peak_ = 0;
  pre_quiet_ = false;
  has_pet_seen_ = false;
  for (int i = 0; i < (int)Percept::Count; i++) has_last_[i] = false;
}

void ImuDetector::set_reference(double pitch, double roll) {
  ref_p_ = pitch;
  ref_r_ = roll;
  ref_fixed_ = true;
}

void ImuDetector::learn_reference() {
  ref_fixed_ = false;
  has_relearn_ = n_ > 0;
  if (n_ > 0) relearn_from_ = buf(n_ - 1).t;
}

void ImuDetector::notify_self_motion(double t, double seconds) {
  quiet_from_ = d_max(quiet_from_, t + seconds + cfg_.settle_s);
  if (!ref_fixed_) {
    has_relearn_ = true;
    relearn_from_ = t + seconds;
  }
}

bool ImuDetector::active() const { return !self_moving_ && n_ > 0 && buf(n_ - 1).t >= quiet_from_; }

bool ImuDetector::pet() const {
  return has_pet_seen_ && n_ > 0 && buf(n_ - 1).t - pet_seen_ <= cfg_.pet_hold_s;
}

int ImuDetector::window_start(double t, double seconds) const {
  int i = n_;
  while (i > 0 && buf(i - 1).t >= t - seconds - 1e-9) i--;
  return i;
}

double ImuDetector::excursion(int from) const {
  if (n_ - from < 2) return 0.0;
  double pmin = buf(from).p, pmax = pmin, rmin = buf(from).r, rmax = rmin;
  for (int i = from + 1; i < n_; i++) {
    const S& s = buf(i);
    pmin = d_min(pmin, s.p);
    pmax = d_max(pmax, s.p);
    rmin = d_min(rmin, s.r);
    rmax = d_max(rmax, s.r);
  }
  return d_max(pmax - pmin, rmax - rmin);
}

bool ImuDetector::cooldown_ok(Percept p, double t, double cooldown) const {
  return !has_last_[(int)p] || t - last_[(int)p] >= cooldown;
}

void ImuDetector::feed(const ImuSample& s, PerceptEvents& out) {
  const ImuConfig& c = cfg_;
  const double t = s.t;
  int first_event = out.n;
  if (n_ == 0) {
    has_relearn_ = !ref_fixed_;
    relearn_from_ = t;
  }
  // self motion
  bool moving = self_motion_mode(s.mode) || (s.has_flags && (s.flags & kSelfMotionFlags) != 0);
  if (s.mode >= 0 && mode_ >= 0 && s.mode != mode_ && s.mode != KILO_MODE_FALLEN) {
    quiet_from_ = d_max(quiet_from_, t + c.settle_s);
    if (!ref_fixed_) {
      has_relearn_ = true;
      relearn_from_ = t;
    }
  }
  if (self_moving_ && !moving) {
    quiet_from_ = d_max(quiet_from_, t + c.settle_s);
    if (!ref_fixed_) {
      has_relearn_ = true;
      relearn_from_ = t;
    }
  }
  if (s.mode >= 0) mode_ = s.mode;
  self_moving_ = moving;

  has_att_ = true;
  pitch_ = s.pitch;
  roll_ = s.roll;
  if (n_ == kBuf) {  // more than kBuf samples in keep_s: drop the oldest
    head_ = (head_ + 1) % kBuf;
    n_--;
  }
  buf_[(head_ + n_++) % kBuf] = {t, pitch_, roll_};
  while (n_ > 0 && t - buf(0).t > keep_s_) {
    head_ = (head_ + 1) % kBuf;
    n_--;
  }

  learn(t);
  tilt(t, out);
  bool has_rate = s.has_rates;
  double rate_mag = 0;
  if (has_rate) {
    PySum sq;
    for (int i = 0; i < 3; i++) sq.add(s.rates[i] * s.rates[i]);
    rate_mag = d_sqrt(sq.value());
  }
  bool powered = !s.has_flags || (s.flags & KILO_TELEMETRY_FLAGS_SERVOS_POWERED) != 0;
  pickup(t, powered, out);
  double d = 0;
  bool has_d = impulse(t, &d);
  startle(t, has_d, d, has_rate, rate_mag, out);
  tap(t, has_d, d, out);
  pet_step(t, out);
  for (int i = first_event; i < out.n; i++) {
    has_last_[(int)out.e[i].kind] = true;
    last_[(int)out.e[i].kind] = out.e[i].t;
  }
}

// ---- detectors ------------------------------------------------------------------------

void ImuDetector::learn(double t) {
  const ImuConfig& c = cfg_;
  if (ref_fixed_ || !has_relearn_ || picked_up_) return;
  if (t < d_max(quiet_from_, relearn_from_ + c.settle_s) || self_moving_) return;
  int w = window_start(t, c.settle_s);
  if (w >= n_ || buf(n_ - 1).t - buf(w).t < c.settle_s * 0.8 - 1e-9 || excursion(w) >= c.calm_deg) return;
  int k = 0;
  for (int i = w; i < n_; i++, k++) {
    x_[k] = buf(i).p;
    tmp_[k] = buf(i).r;
  }
  double p = fmean(x_, k), r = fmean(tmp_, k);
  if (d_max(d_fabs(p), d_fabs(r)) <= c.max_ref_deg) {
    ref_p_ = p;
    ref_r_ = r;
  }
  has_relearn_ = false;
}

void ImuDetector::tilt(double t, PerceptEvents& out) {
  const ImuConfig& c = cfg_;
  double dp = pitch_ - ref_p_, dr = roll_ - ref_r_;
  double over = d_max(d_fabs(dp), d_fabs(dr));
  bool want = tilted_ ? over < c.tilt_exit_deg : over > c.tilt_deg;
  if (!want) {
    has_tilt_since_ = false;
    if (tilted_) tilt_dir_ = direction(dp, dr);
    return;
  }
  if (!has_tilt_since_) {
    has_tilt_since_ = true;
    tilt_since_ = t;
    return;
  }
  if (t - tilt_since_ < c.tilt_confirm_s) return;
  has_tilt_since_ = false;
  tilted_ = !tilted_;
  PerceptEvent& e = out.add(t, tilted_ ? Percept::Tilted : Percept::Level);
  e.has_a = e.has_b = true;
  e.a = pitch_;
  e.b = roll_;
  if (tilted_) {
    tilt_dir_ = direction(dp, dr);
    e.direction = tilt_dir_;
  } else {
    tilt_dir_ = TiltDirection::None;
  }
}

void ImuDetector::pickup(double t, bool powered, PerceptEvents& out) {
  const ImuConfig& c = cfg_;
  if (!picked_up_) {
    double thr = powered ? c.pickup_deg : c.pickup_unpowered_deg;
    int w = window_start(t, c.pickup_window_s);
    if (!active() || n_ - w < c.pickup_min_samples || excursion(w) < thr) {
      has_pick_since_ = false;
      return;
    }
    if (!has_pick_since_) {
      has_pick_since_ = true;
      pick_since_ = t;
    }
    if (t - pick_since_ < c.pickup_confirm_s) return;
    picked_up_ = true;
    has_pick_since_ = false;
    tap_state_ = TapState::Idle;
    PerceptEvent& e = out.add(t, Percept::PickedUp);
    e.has_a = e.has_b = true;
    e.a = pitch_;
    e.b = roll_;
    e.servos_powered = powered;
    return;
  }
  int w = window_start(t, c.put_down_calm_s);
  bool span_ok = n_ - w >= 2 && buf(n_ - 1).t - buf(w).t >= c.put_down_calm_s * 0.9;
  bool near_ref = d_max(d_fabs(pitch_ - ref_p_), d_fabs(roll_ - ref_r_)) < c.tilt_deg;
  if (span_ok && near_ref && excursion(w) < c.calm_deg) {
    picked_up_ = false;
    if (!ref_fixed_) {
      has_relearn_ = true;
      relearn_from_ = t;
    }
    PerceptEvent& e = out.add(t, Percept::PutDown);
    e.has_a = e.has_b = true;
    e.a = pitch_;
    e.b = roll_;
  }
}

bool ImuDetector::impulse(double t, double* d_out) {
  const ImuConfig& c = cfg_;
  // the samples in [t - pre_gap - pre_window, t - pre_gap]
  double lo = t - c.pre_gap_s - c.pre_window_s, hi = t - c.pre_gap_s;
  int k = 0;
  double pmin = 0, pmax = 0, rmin = 0, rmax = 0;
  for (int i = 0; i < n_; i++) {
    const S& s = buf(i);
    if (s.t < lo || s.t > hi) continue;
    if (k == 0) {
      pmin = pmax = s.p;
      rmin = rmax = s.r;
    } else {
      pmin = d_min(pmin, s.p);
      pmax = d_max(pmax, s.p);
      rmin = d_min(rmin, s.r);
      rmax = d_max(rmax, s.r);
    }
    x_[k] = s.p;
    tmp_[k] = s.r;
    k++;
  }
  if (k < 5) {
    d_head_ = d_n_ = 0;
    return false;
  }
  double pp = median(x_, k), pr = median(tmp_, k);
  double d = d_max(d_fabs(pitch_ - pp), d_fabs(roll_ - pr));
  pre_quiet_ = d_max(pmax - pmin, rmax - rmin) < c.tap_quiet_deg;
  if (d_n_ == kD) {
    d_head_ = (d_head_ + 1) % kD;
    d_n_--;
  }
  d_[(d_head_ + d_n_++) % kD] = {t, d};
  double keep = d_max(c.tap_rise_s, c.startle_rise_s) + 0.05;
  while (d_n_ > 0 && t - d_[d_head_].t > keep) {
    d_head_ = (d_head_ + 1) % kD;
    d_n_--;
  }
  *d_out = d;
  return true;
}

bool ImuDetector::rose_from(double t, double below, double within) const {
  for (int i = 0; i < d_n_; i++) {
    const D& x = d_[(d_head_ + i) % kD];
    if (t - within - 1e-9 <= x.t && x.t < t && x.d < below) return true;
  }
  return false;
}

void ImuDetector::startle(double t, bool has_d, double d, bool has_rate, double rate, PerceptEvents& out) {
  const ImuConfig& c = cfg_;
  if (!active() || !cooldown_ok(Percept::Startle, t, c.startle_cooldown_s)) return;
  bool by_angle = has_d && d >= c.startle_deg && rose_from(t, c.calm_deg, c.startle_rise_s);
  bool by_rate = has_rate && rate >= c.startle_rate_dps;
  if (!(by_angle || by_rate)) return;
  PerceptEvent& e = out.add(t, Percept::Startle);
  e.has_a = has_d;
  e.a = d;
  e.has_b = has_rate;
  e.b = rate;
}

void ImuDetector::tap(double t, bool has_d, double d, PerceptEvents& out) {
  const ImuConfig& c = cfg_;
  if (!has_d || !active() || picked_up_) {
    tap_state_ = TapState::Idle;
    return;
  }
  if (tap_state_ == TapState::Idle) {
    if (d >= c.tap_min_deg && pre_quiet_ && rose_from(t, c.tap_min_deg / 2, c.tap_rise_s)) {
      tap_state_ = TapState::Cand;
      tap_t0_ = t;
      tap_peak_ = d;
    } else if (d >= c.tap_min_deg) {
      tap_state_ = TapState::Busy;
    }
    return;
  }
  if (tap_state_ == TapState::Busy) {
    if (d <= c.tap_return_deg) tap_state_ = TapState::Idle;
    return;
  }
  tap_peak_ = d_max(tap_peak_, d);
  if (d <= c.tap_return_deg) {
    tap_state_ = TapState::Idle;
    if (tap_peak_ < c.startle_deg && cooldown_ok(Percept::Tap, t, c.tap_cooldown_s)) {
      PerceptEvent& e = out.add(t, Percept::Tap);
      e.has_a = e.has_b = true;
      e.a = tap_t0_;
      e.b = tap_peak_;
      e.c = t - tap_t0_;
    }
    return;
  }
  if (t - tap_t0_ > c.tap_max_s) tap_state_ = TapState::Busy;
}

void ImuDetector::pet_step(double t, PerceptEvents& out) {
  const ImuConfig& c = cfg_;
  if (!active() || picked_up_) return;
  int w = window_start(t, c.pet_window_s);
  int n = n_ - w;
  if (n < 20 || buf(n_ - 1).t - buf(w).t < c.pet_window_s * 0.9) return;
  double best_pp = 0;
  bool has_best = false;
  for (int k = 1; k <= 2; k++) {  // pitch, then roll: the one with the larger swing
    for (int i = 0; i < n; i++) {
      const S& s = buf(w + i);
      ts_[i] = s.t;
      tmp_[i] = k == 1 ? s.p : s.r;
    }
    // _smooth3
    x_[0] = tmp_[0];
    x_[n - 1] = tmp_[n - 1];
    for (int i = 1; i < n - 1; i++) x_[i] = (tmp_[i - 1] + tmp_[i] + tmp_[i + 1]) / 3;
    double mn = x_[0], mx = x_[0];
    for (int i = 1; i < n; i++) {
      mn = d_min(mn, x_[i]);
      mx = d_max(mx, x_[i]);
    }
    double pp = mx - mn;
    if (!has_best || pp > best_pp) {
      has_best = true;
      best_pp = pp;
      for (int i = 0; i < n; i++) best_[i] = x_[i];
    }
  }
  double pp = best_pp;
  const double* x = best_;
  if (!(c.pet_min_pp <= pp && pp <= c.pet_max_pp)) return;
  double rate = 0;
  for (int i = 0; i < n - 1; i++) {
    double r = d_fabs(x[i + 1] - x[i]) / d_max(ts_[i + 1] - ts_[i], 1e-3);
    rate = i == 0 ? r : d_max(rate, r);
  }
  if (rate > c.pet_max_rate_dps) return;
  double m = fmean(x, n);
  double band = d_max(c.pet_min_band, c.pet_band_frac * pp);
  int side = 0, ncross = 0;
  for (int i = 0; i < n; i++) {
    int s = x[i] > m + band ? 1 : (x[i] < m - band ? -1 : 0);
    if (s != 0 && s != side) {
      if (side != 0) tmp_[ncross++] = ts_[i];  // the crossing times
      side = s;
    }
  }
  if (ncross < c.pet_min_half_cycles) return;
  if (ncross >= 2) {
    for (int i = 0; i + 1 < ncross; i++) x_[i] = tmp_[i + 1] - tmp_[i];  // half periods
    double h = median(x_, ncross - 1);
    if (!(c.pet_half_period_lo_s <= h && h <= c.pet_half_period_hi_s)) return;
  }
  has_pet_seen_ = true;
  pet_seen_ = t;
  if (!cooldown_ok(Percept::Pet, t, c.pet_cooldown_s)) return;
  PerceptEvent& e = out.add(t, Percept::Pet);
  e.has_a = e.has_b = true;
  e.a = pp;
  e.b = ncross / 2;
}

}  // namespace perception
}  // namespace kilo
