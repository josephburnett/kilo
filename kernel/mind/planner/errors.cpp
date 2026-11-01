#include "planner/errors.h"

namespace kilo {
namespace planner {

static constexpr real kA16 = 16;

static real round2(double v) {  // Python's round(v, 2), half to even
  double s = v * 100.0;
  int64_t i = (int64_t)s;
  if ((double)i > s) i--;
  double d = s - (double)i;
  if (d > 0.5 || (d == 0.5 && (i & 1))) i++;
  return (real)((double)i / 100.0);
}

static double d_sqrt(double v) { return __builtin_sqrt(v); }

void ErrorTracker::reset() {
  head_ = 0;
  count_ = 0;
  n_ = 0;
  offset_ = {0, 0};
}

Attitude ErrorTracker::add(int64_t tick, int16_t pitch, int16_t roll, int16_t ref_pitch, int16_t ref_roll,
                           int16_t corr_pitch, int16_t corr_roll, const Attitude* ref) {
  ErrorSample s;
  s.tick = tick;
  s.mp = (real)pitch / kA16;
  s.mr = (real)roll / kA16;
  if (ref == nullptr) {
    s.rp = (real)ref_pitch / kA16;
    s.rr = (real)ref_roll / kA16;
  } else {
    s.rp = ref->pitch;
    s.rr = ref->roll;
  }
  s.cp = (real)corr_pitch / kA16;
  s.cr = (real)corr_roll / kA16;
  if (count_ < kErrWindowTicks) {
    ring_[(head_ + count_) % kErrWindowTicks] = s;
    count_++;
  } else {  // the deque's maxlen: the oldest goes
    ring_[head_] = s;
    head_ = (head_ + 1) % kErrWindowTicks;
  }
  real ep = s.mp - s.rp, er = s.mr - s.rr;
  n_++;
  real a = m_max(alpha_, (real)1.0 / (real)n_);
  offset_ = {offset_.pitch + (ep - offset_.pitch) * a, offset_.roll + (er - offset_.roll) * a};
  return {ep - offset_.pitch, er - offset_.roll};
}

bool ErrorTracker::estimate(ErrorEstimate* out) const {
  int n = count_;
  if (n < kErrMinSamples) return false;
  double op = 0, orr = 0, cp = 0, cr = 0;
  for (int i = 0; i < n; i++) {
    const ErrorSample& s = at(i);
    op += (double)s.mp - s.rp;
    orr += (double)s.mr - s.rr;
    cp += s.cp;
    cr += s.cr;
  }
  op /= n;
  orr /= n;
  double spike = 0;
  for (int i = 0; i < n; i++) {
    const ErrorSample& s = at(i);
    double a = d_fabs((double)s.mp - s.rp - op), b = d_fabs((double)s.mr - s.rr - orr);
    double m = a > b ? a : b;
    if (i == 0 || m > spike) spike = m;
  }
  out->samples = (uint32_t)n;
  out->offset_pitch_deg = round2(op);
  out->offset_roll_deg = round2(orr);
  out->spike_deg = round2(spike);
  out->corr_pitch_deg = round2(cp / n);
  out->corr_roll_deg = round2(cr / n);
  lag(&out->has_lag, &out->lag_ms, &out->lag_quality);
  return true;
}

// The shift k (ticks) maximising the Pearson correlation, over the
// overlapping samples, of the predicted roll at t - k with the measured roll
// at t: + = the body is later than the prediction. None (has false) while
// the prediction hardly moves. errors.py builds a dict tick -> measured
// roll (the last sample of a tick wins); here a search from the end does it.
void ErrorTracker::lag(bool* has, real* lag_ms, real* quality) const {
  int n = count_;
  *has = false;
  *lag_ms = 0;
  *quality = 0;
  double mean = 0;
  for (int i = 0; i < n; i++) mean += at(i).rr;
  mean /= n;
  double var = 0;
  for (int i = 0; i < n; i++) {
    double d = at(i).rr - mean;
    var += d * d;
  }
  if (d_sqrt(var / n) < kErrMinSwingDeg) return;
  // Ticks rising (the normal case): a merge walk finds tick t + k; else a
  // search from the end.
  bool rising = true;
  for (int i = 1; i < n; i++) rising = rising && at(i).tick > at(i - 1).tick;
  int best_k = 0;
  bool found = false;
  double best_c = -2.0;
  for (int k = -kErrMaxLagTicks; k <= kErrMaxLagTicks; k++) {
    double sa = 0, sb = 0;
    int cnt = 0;
    int16_t idx[kErrWindowTicks];  // the sample at t + k for sample i, -1 none
    int walk = 0;
    for (int i = 0; i < n; i++) {
      int64_t want = at(i).tick + k;
      int found_j = -1;
      if (rising) {
        while (walk < n && at(walk).tick < want) walk++;
        if (walk < n && at(walk).tick == want) found_j = walk;
      } else {
        for (int j = n - 1; j >= 0 && found_j < 0; j--)
          if (at(j).tick == want) found_j = j;
      }
      idx[i] = (int16_t)found_j;
      if (found_j >= 0) {
        sa += at(i).rr;
        sb += at(found_j).mr;
        cnt++;
      }
    }
    if (cnt < kErrMinSamples / 2) continue;
    double ma = sa / cnt, mb = sb / cnt;
    double cov = 0, va = 0, vb = 0;
    for (int i = 0; i < n; i++) {
      if (idx[i] < 0) continue;
      double a = at(i).rr - ma, b = at(idx[i]).mr - mb;
      cov += a * b;
      va += a * a;
      vb += b * b;
    }
    if (va <= 0 || vb <= 0) continue;
    double c = cov / d_sqrt(va * vb);
    if (c > best_c + 1e-9) {
      best_k = k;
      best_c = c;
      found = true;
    }
  }
  if (!found) return;
  *has = true;
  *lag_ms = (real)best_k * kTickDefault;
  *quality = round2(best_c);
}

}  // namespace planner
}  // namespace kilo
