// patterns: the gait patterns and the speed -> timing mapping. A port of
// mind/kilo/planner/patterns.py (docs/walking-and-balance.md §3, §10).
//
// A pattern is a sequence of swing groups (the legs that lift together);
// each group's slot is a four-foot shift, then the swing, so
//   period = groups x (shift + swing),  stance = period - swing.
// The swing time is fixed per pattern; speed comes from the stride and the
// stance time (stance = stride / speed). Slower than the longest stance the
// stride shrinks (to STRIDE_MIN_MM); faster than the default stride at the
// shortest shift, the speed is capped.
#ifndef KILO_MIND_PLANNER_PATTERNS_H
#define KILO_MIND_PLANNER_PATTERNS_H
#include <stdint.h>
#include "planner/fmath.h"

namespace kilo {
namespace planner {

constexpr real kTickMs = 20;
constexpr real kStrideMinMm = 8;
constexpr real kDefaultTurnMm = 20;  // per-side stride of a turn in place at the default stride
constexpr real kLeanMoveMs = 100;    // the shortest body move between two swings (walk, crawl)

struct Pattern {
  const char* name;
  uint8_t n_groups;
  uint8_t group_size;
  uint8_t groups[4][2];  // who swings, in order
  real swing_ms;         // fixed swing time
  real stride_mm;        // default stride
  real step_h_mm;        // foot lift at mid-swing
  int body_h_256;        // body height, /256 of L1 + L2
  real period_ms;        // the cycle at the default speed
  real shift_min_ms;     // the shortest four-foot phase before a swing
  real stance_max_ms;    // the longest stance: slower, the stride shrinks
  bool lean;             // one leg at a time: the body moves over the three feet that stay down

  bool in_group(int k, int leg) const {
    for (int i = 0; i < group_size; i++)
      if (groups[k][i] == leg) return true;
    return false;
  }
};

// PATTERNS in patterns.py's order: walk, trot, crawl.
constexpr int kPatterns = 3;
extern const Pattern kPatternTable[kPatterns];
const Pattern& default_pattern();  // trot
// By name, ignoring case; null if unknown.
const Pattern* find_pattern(const char* name);

enum class Direction : uint8_t { Forward, Back, Left, Right, March };
// By name, ignoring case ("backward", "backwards" = back); false if unknown.
bool find_direction(const char* name, Direction* out);
const char* direction_name(Direction d);

struct Timing {
  real stride_mm;  // magnitude
  real shift_ms;
  real stance_ms;
  real period_ms;
  real speed_mm_s;  // what this timing walks at with the stride
};

real shift_min_ms(const Pattern& p, real lag_ms);
// The pattern table's speed: its stride over its default stance.
real default_speed(const Pattern& p);
// (slowest, fastest) speed that timing() gives.
void speed_range(const Pattern& p, real lag_ms, real* lo, real* hi);
// Stride, shift and period for walking at |speed_mm_s|, clamped to speed_range().
Timing timing(const Pattern& p, real speed_mm_s, real lag_ms = 150);
Timing default_timing(const Pattern& p, real lag_ms = 150);
// (left, right) stride, + = the feet push the body forward: forward and back
// both sides, left and right turn in place (kDefaultTurnMm scaled with the
// stride), march in place.
void side_strides(Direction d, real stride, const Pattern& p, real out[2]);

}  // namespace planner
}  // namespace kilo

#endif
