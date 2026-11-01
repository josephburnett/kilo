// dmath: double-precision helpers for perception and behaviour, which
// compute in double as their Python counterparts do (they run at 10-50 Hz;
// the Cortex-A7's VFPv4 does double in hardware).
//
// sqrt and fabs are instructions (-fno-math-errno). exp, log, sin and cos
// are libm calls (Circle links libm; kernel/host's freestanding check
// allows exactly these): the drives' arousal decay and their optional
// gaussian noise. py_sum and fsum reproduce Python's sum() of floats
// (Neumaier-compensated since 3.12) and math.fsum (exact), so a port's
// sums round as Python's do and the golden tests can compare exactly.
#ifndef KILO_MIND_UTIL_DMATH_H
#define KILO_MIND_UTIL_DMATH_H
#include <stdint.h>

namespace kilo {

inline double d_sqrt(double x) { return __builtin_sqrt(x); }
inline double d_fabs(double x) { return __builtin_fabs(x); }
inline double d_exp(double x) { return __builtin_exp(x); }
inline double d_log(double x) { return __builtin_log(x); }
inline double d_sin(double x) { return __builtin_sin(x); }
inline double d_cos(double x) { return __builtin_cos(x); }
inline double d_min(double a, double b) { return b < a ? b : a; }  // Python min(a, b)
inline double d_max(double a, double b) { return b > a ? b : a; }  // Python max(a, b)
inline bool d_isfinite(double x) { return x - x == 0.0; }
constexpr double kNegInf = -__builtin_inf();

// Python round(x) for |x| < 2^52, half away from zero (Python rounds exact
// halves to even; the callers' values are never exact halves).
inline double d_round(double x) {
  return x >= 0 ? (double)(int64_t)(x + 0.5) : -(double)(int64_t)(-x + 0.5);
}

// Python's sum() of floats (CPython >= 3.12: Neumaier's compensated sum).
class PySum {
 public:
  void add(double x) {
    double t = s_ + x;
    if (d_fabs(s_) >= d_fabs(x)) c_ += (s_ - t) + x;
    else c_ += (x - t) + s_;
    s_ = t;
  }
  double value() const { return (c_ != 0.0 && d_isfinite(c_)) ? s_ + c_ : s_; }

 private:
  double s_ = 0.0, c_ = 0.0;
};

// math.fsum: Shewchuk's exact partials (Python's msum recipe), for
// statistics.fmean. Up to 32 partials; a few values never need more.
class FSum {
 public:
  void add(double x) {
    int i = 0;
    for (int j = 0; j < n_; j++) {
      double y = p_[j];
      if (d_fabs(x) < d_fabs(y)) {
        double t = x;
        x = y;
        y = t;
      }
      double hi = x + y;
      double lo = y - (hi - x);
      if (lo != 0.0) p_[i++] = lo;
      x = hi;
    }
    n_ = i;
    if (x == 0.0) return;
    if (n_ < kMax) p_[n_++] = x;
    else p_[kMax - 1] += x;
  }
  double value() const {
    // CPython's fsum: sum the partials from the top, with its half-way correction
    if (n_ == 0) return 0.0;
    int k = n_ - 1;
    double hi = p_[k], lo = 0.0;
    while (k > 0) {
      double x = hi;
      double y = p_[--k];
      hi = x + y;
      double yr = hi - x;
      lo = y - yr;
      if (lo != 0.0) break;
    }
    if (k > 0 && ((lo < 0.0 && p_[k - 1] < 0.0) || (lo > 0.0 && p_[k - 1] > 0.0))) {
      double y = lo * 2.0;
      double x = hi + y;
      double yr = x - hi;
      if (y == yr) hi = x;
    }
    return hi;
  }

 private:
  static constexpr int kMax = 32;
  double p_[kMax];
  int n_ = 0;
};

}  // namespace kilo

#endif
