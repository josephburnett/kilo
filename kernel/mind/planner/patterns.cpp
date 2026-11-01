#include "planner/patterns.h"
#include "planner/model.h"

namespace kilo {
namespace planner {

// Walk and crawl lift LB, LF, RB, RF (the lateral sequence); the trot the
// diagonal pairs LF+RB, RF+LB. Values as in patterns.py's PATTERNS.
const Pattern kPatternTable[kPatterns] = {
    {"walk", 4, 1, {{LB, 0}, {LF, 0}, {RB, 0}, {RF, 0}}, 500, 30, 15, 205, 3200, 0, 4000, true},
    {"trot", 2, 2, {{LF, RB}, {RF, LB}, {0, 0}, {0, 0}}, 150, 25, 8, 205, 600, 60, 1327, false},
    {"crawl", 4, 1, {{LB, 0}, {LF, 0}, {RB, 0}, {RF, 0}}, 297, 25, 18, 179, 2800, 0, 4000, true},
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

real shift_min_ms(const Pattern& p, real lag_ms) {
  if (p.lean) return m_max(p.shift_min_ms, lag_ms + kLeanMoveMs);
  return p.shift_min_ms;
}

static void stance_limits(const Pattern& p, real lag_ms, real* lo, real* hi) {
  real n = p.n_groups;
  *lo = n * (shift_min_ms(p, lag_ms) + p.swing_ms) - p.swing_ms;
  *hi = m_max(*lo, p.stance_max_ms);
}

real default_speed(const Pattern& p) { return p.stride_mm / (p.period_ms - p.swing_ms) * (real)1000.0; }

void speed_range(const Pattern& p, real lag_ms, real* lo, real* hi) {
  real st_lo, st_hi;
  stance_limits(p, lag_ms, &st_lo, &st_hi);
  *lo = kStrideMinMm / st_hi * (real)1000.0;
  *hi = p.stride_mm / st_lo * (real)1000.0;
}

Timing timing(const Pattern& p, real speed_mm_s, real lag_ms) {
  real n = p.n_groups;
  real st_lo, st_hi;
  stance_limits(p, lag_ms, &st_lo, &st_hi);
  real v = m_fabs(speed_mm_s);
  real stride = p.stride_mm;
  real stance = v > 0 ? stride / v * (real)1000.0 : m_inf();
  if (stance > st_hi) {
    stance = st_hi;
    stride = m_max(kStrideMinMm, v * st_hi / (real)1000.0);
  }
  stance = m_max(stance, st_lo);
  real period = stance + p.swing_ms;
  return {stride, period / n - p.swing_ms, stance, period, stride / stance * (real)1000.0};
}

Timing default_timing(const Pattern& p, real lag_ms) { return timing(p, default_speed(p), lag_ms); }

void side_strides(Direction d, real stride, const Pattern& p, real out[2]) {
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
      real t = kDefaultTurnMm * stride / p.stride_mm;
      out[0] = d == Direction::Left ? -t : t;
      out[1] = d == Direction::Left ? t : -t;
      return;
    }
  }
}

}  // namespace planner
}  // namespace kilo
