// fmath: the planner's scalar type and the few libm functions it uses.
//
// The planner computes in float (the Cortex-A7's VFPv4 does float and
// double in hardware; float is what the plan needs: millimetres and tenths
// of a degree). Where a result hangs on a near-singular solve or a 1e-6
// feasibility test (fit_attitude's normal equations, the lean's quadratic
// program) it computes in double, as Python does everywhere: in float those
// lose the answer, not just digits. Building with -DKILO_PLANNER_DOUBLE
// makes everything double, which the golden test uses to separate a wrong
// port from float rounding.
//
// The functions go through GCC's builtins, so the core includes no libc
// header: they become instructions (sqrt with -fno-math-errno, fabs) or
// calls to libm's sinf, cosf, tanf, atanf, atan2f, which Circle links
// (STDLIB_SUPPORT 1). kernel/host's freestanding check lists exactly these.
// Rounding and floor are done here on integers (the planner's values are
// small), so floorf/ceilf/roundf are not needed.
#ifndef KILO_MIND_PLANNER_FMATH_H
#define KILO_MIND_PLANNER_FMATH_H
#include <stdint.h>

namespace kilo {
namespace planner {

#ifdef KILO_PLANNER_DOUBLE
typedef double real;
inline real m_sin(real x) { return __builtin_sin(x); }
inline real m_cos(real x) { return __builtin_cos(x); }
inline real m_tan(real x) { return __builtin_tan(x); }
inline real m_atan(real x) { return __builtin_atan(x); }
inline real m_atan2(real y, real x) { return __builtin_atan2(y, x); }
inline real m_sqrt(real x) { return __builtin_sqrt(x); }
inline real m_fabs(real x) { return __builtin_fabs(x); }
inline real m_inf() { return __builtin_inf(); }
#else
typedef float real;
inline real m_sin(real x) { return __builtin_sinf(x); }
inline real m_cos(real x) { return __builtin_cosf(x); }
inline real m_tan(real x) { return __builtin_tanf(x); }
inline real m_atan(real x) { return __builtin_atanf(x); }
inline real m_atan2(real y, real x) { return __builtin_atan2f(y, x); }
inline real m_sqrt(real x) { return __builtin_sqrtf(x); }
inline real m_fabs(real x) { return __builtin_fabsf(x); }
inline real m_inf() { return __builtin_inff(); }
#endif

constexpr real kPi = (real)3.14159265358979323846;
inline real radians(real deg) { return deg * (kPi / (real)180.0); }
inline real degrees(real rad) { return rad * ((real)180.0 / kPi); }

inline real m_min(real a, real b) { return b < a ? b : a; }  // Python's min(a, b): a on a tie
inline real m_max(real a, real b) { return b > a ? b : a; }
inline real clamp(real v, real lo, real hi) { return m_min(hi, m_max(lo, v)); }  // min(hi, max(lo, v))
inline double d_fabs(double x) { return __builtin_fabs(x); }

// floor of a value well inside int32 (no libm floorf).
inline int32_t floor_i(real v) {
  int32_t i = (int32_t)v;  // toward zero
  return (real)i > v ? i - 1 : i;
}
inline int32_t ceil_i(real v) {
  int32_t i = (int32_t)v;
  return (real)i < v ? i + 1 : i;
}
// Python's round(): half to even (12.5 -> 12, 13.5 -> 14).
inline int32_t round_i(real v) {
  int32_t f = floor_i(v);
  real d = v - (real)f;  // exact: f <= v < f + 1
  if (d > (real)0.5) return f + 1;
  if (d < (real)0.5) return f;
  return (f & 1) ? f + 1 : f;
}

}  // namespace planner
}  // namespace kilo

#endif
