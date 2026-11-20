#include "planner/patterns.h"
#include "planner/model.h"
#include "util/str.h"

namespace kilo {
namespace planner {

// The walk lifts LB then RF 60 ms later, then RB and LF (diagonal couplets);
// the trot the diagonal pairs LF+RB, RF+LB; the crawl LB, LF, RB, RF one at
// a time with the lean. Values (and why) as in patterns.py's PATTERNS (the
// trot's 44 mm stride and its lift tapering from 8 mm at 35 mm to 6 at 44,
// the strides' 1.5 mm a tick in the walk and the trot: since 2026-11-17).
const Pattern kPatternTable[kPatterns] = {
    {"walk", 2, 2, {{LB, RF}, {RB, LF}, {0, 0}, {0, 0}}, 150, 30, 8, 205, 460, 20, 310, false, 60, 80, -1, 0, 1.5},
    {"trot", 2, 2, {{LF, RB}, {RF, LB}, {0, 0}, {0, 0}}, 150, 44, 8, 205, 340, 20, 190, false, 0, 144, 6, 35, 1.5},
    {"crawl", 4, 1, {{LB, 0}, {LF, 0}, {RB, 0}, {RF, 0}}, 300, 50, 15, 179, 2550, 0, 4000, true, 0, -1, -1, 0, 0.5},
};

const Pattern& default_pattern() { return kPatternTable[1]; }

const Pattern* find_pattern(const char* name) {
  for (const Pattern& p : kPatternTable)
    if (str_ieq(name, p.name)) return &p;
  return nullptr;
}

static const char* const kDirectionNames[] = {"forward", "back", "left", "right", "march"};

bool find_direction(const char* name, Direction* out) {
  if (str_ieq(name, "backward") || str_ieq(name, "backwards")) {
    *out = Direction::Back;
    return true;
  }
  for (int i = 0; i < 5; i++)
    if (str_ieq(name, kDirectionNames[i])) {
      *out = (Direction)i;
      return true;
    }
  return false;
}

const char* direction_name(Direction d) { return kDirectionNames[(int)d]; }

real shift_min_ms(const Pattern& p) {
  if (p.lean) return m_max(p.shift_min_ms, kLeanMoveMs + kLeanLeadMs);
  return p.shift_min_ms;
}

static void stance_limits(const Pattern& p, real* lo, real* hi) {
  real n = p.n_groups;
  *lo = n * (shift_min_ms(p) + p.span_ms()) - p.swing_ms;
  *hi = m_max(*lo, p.stance_max_ms);
}

real default_speed(const Pattern& p) {
  if (p.default_mm_s >= 0) return p.default_mm_s;
  return p.stride_mm / (p.period_ms - p.swing_ms) * (real)1000.0;
}

void speed_range(const Pattern& p, real* lo, real* hi) {
  real st_lo, st_hi;
  stance_limits(p, &st_lo, &st_hi);
  if (fixed_cadence(p)) st_hi = st_lo;
  *lo = kStrideMinMm / st_hi * (real)1000.0;
  *hi = p.stride_mm / st_lo * (real)1000.0;
}

Timing timing(const Pattern& p, real speed_mm_s) {
  real n = p.n_groups;
  real st_lo, st_hi;
  stance_limits(p, &st_lo, &st_hi);
  real v = m_fabs(speed_mm_s);
  real stride, stance;
  if (fixed_cadence(p)) {
    stance = st_lo;
    stride = m_min(p.stride_mm, m_max(kStrideMinMm, v * stance / (real)1000.0));
  } else {
    stride = p.stride_mm;
    stance = v > 0 ? stride / v * (real)1000.0 : m_inf();
    if (stance > st_hi) {
      stance = st_hi;
      stride = m_max(kStrideMinMm, v * st_hi / (real)1000.0);
    }
    stance = m_max(stance, st_lo);
  }
  real period = stance + p.swing_ms;
  return {stride, period / n - p.span_ms(), stance, period, stride / stance * (real)1000.0};
}

Timing default_timing(const Pattern& p) { return timing(p, default_speed(p)); }

real step_height(const Pattern& p, real stride) {
  if (p.step_h_long_mm < 0 || stride <= p.taper_from_mm) return p.step_h_mm;
  real f = m_min((real)1.0, (stride - p.taper_from_mm) / (p.stride_mm - p.taper_from_mm));
  return p.step_h_mm + (p.step_h_long_mm - p.step_h_mm) * f;
}

void side_strides(Direction d, real stride, real out[2], real steer, real cap) {
  switch (d) {
    case Direction::Forward:
    case Direction::Back: {
      real s = d == Direction::Forward ? stride : -stride;
      real w = m_max((real)-1.0, m_min((real)1.0, steer)) * m_fabs(s);
      real left = s - w, right = s + w;
      real m = m_max(m_fabs(left), m_fabs(right));
      if (cap > 0 && m > cap) {
        left = left * cap / m;
        right = right * cap / m;
      }
      out[0] = left;
      out[1] = right;
      return;
    }
    case Direction::March:
      out[0] = out[1] = 0;
      return;
    case Direction::Left:
    case Direction::Right: {
      real t = kTurnFrac * stride;
      out[0] = d == Direction::Left ? -t : t;
      out[1] = d == Direction::Left ? t : -t;
      return;
    }
  }
}

}  // namespace planner
}  // namespace kilo
