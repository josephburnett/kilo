#include "planner/patterns.h"
#include "planner/model.h"

namespace kilo {
namespace planner {

// The walk lifts LB then RF 60 ms later, then RB and LF (diagonal couplets);
// the trot the diagonal pairs LF+RB, RF+LB; the crawl LB, LF, RB, RF one at
// a time with the lean. Values (and why) as in patterns.py's PATTERNS.
const Pattern kPatternTable[kPatterns] = {
    {"walk", 2, 2, {{LB, RF}, {RB, LF}, {0, 0}, {0, 0}}, 150, 30, 8, 205, 460, 20, 310, false, 60, 80},
    {"trot", 2, 2, {{LF, RB}, {RF, LB}, {0, 0}, {0, 0}}, 150, 35, 8, 205, 340, 20, 190, false, 0, 144},
    {"crawl", 4, 1, {{LB, 0}, {LF, 0}, {RB, 0}, {RF, 0}}, 300, 50, 15, 179, 2550, 0, 4000, true, 0, -1},
};

const Pattern& default_pattern() { return kPatternTable[1]; }

static char lower(char c) { return c >= 'A' && c <= 'Z' ? (char)(c - 'A' + 'a') : c; }

static bool same(const char* a, const char* b) {
  for (; *a && *b; a++, b++)
    if (lower(*a) != lower(*b)) return false;
  return *a == *b;
}

const Pattern* find_pattern(const char* name) {
  for (const Pattern& p : kPatternTable)
    if (same(name, p.name)) return &p;
  return nullptr;
}

static const char* const kDirectionNames[] = {"forward", "back", "left", "right", "march"};

bool find_direction(const char* name, Direction* out) {
  if (same(name, "backward") || same(name, "backwards")) {
    *out = Direction::Back;
    return true;
  }
  for (int i = 0; i < 5; i++)
    if (same(name, kDirectionNames[i])) {
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

void side_strides(Direction d, real stride, real out[2]) {
  switch (d) {
    case Direction::Forward:
      out[0] = out[1] = stride;
      return;
    case Direction::Back:
      out[0] = out[1] = -stride;
      return;
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
