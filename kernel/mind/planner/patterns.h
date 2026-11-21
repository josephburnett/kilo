// patterns: the gait patterns and the speed -> timing mapping. A port of
// mind/kilo/planner/patterns.py (docs/walking-and-balance.md §3, §10).
//
// A pattern is a sequence of swing groups (the legs that lift together, the
// second of a pair couple_ms after the first); each group's slot is a
// four-foot shift, then the swing, so
//   period = groups x (shift + span),  span = swing + couple,  stance = period - swing.
// The swing time is fixed per pattern. The walk and the trot (no lean) have
// a fixed cadence: the shift stays at its shortest (one tick) and speed
// comes from the stride alone. The crawl: speed from the stride and the
// stance time (stance = stride / speed); slower than the longest stance the
// stride shrinks (to STRIDE_MIN_MM); faster than the default stride at the
// shortest shift, the speed is capped. The crawl's shortest shift:
// kLeanMoveMs (the body's move over the next three feet) plus kLeanLeadMs.
#ifndef KILO_MIND_PLANNER_PATTERNS_H
#define KILO_MIND_PLANNER_PATTERNS_H
#include <stdint.h>
#include "link/consts.h"
#include "planner/fmath.h"

namespace kilo {
namespace planner {

constexpr real kTickMs = (real)KILO_TICK_MS;
constexpr real kStrideMinMm = 8;
constexpr real kTurnFrac = (real)0.8;  // a turn in place: each side's stride, this fraction of the stride
constexpr real kLeanMoveMs = 300;      // the body's typical move between two swings (walk, crawl)
constexpr real kLeanLeadMs = -60;      // the lift starts this long after the move's end (negative: in its tail)

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
  real couple_ms;        // a group's second leg lifts (and lands) this long after its first
  real default_mm_s;     // the default speed (< 0: the stride over the default stance)
  real step_h_long_mm;   // the lift at the longest stride (< 0: step_h_mm at every stride; step_height())
  real taper_from_mm;    // ...tapering from step_h_mm at this stride
  real stride_slew_mm;   // the strides move at most this much a tick (a speed, direction or steer change)

  bool in_group(int k, int leg) const {
    for (int i = 0; i < group_size; i++)
      if (groups[k][i] == leg) return true;
    return false;
  }
  // the leg's place in group k (0 first, 1 second), -1 if not in it
  int index_in_group(int k, int leg) const {
    for (int i = 0; i < group_size; i++)
      if (groups[k][i] == leg) return i;
    return -1;
  }
  // a slot's swing part: the first leg's lift to the last leg's touchdown
  real span_ms() const { return swing_ms + couple_ms * (real)(group_size - 1); }
};

// PATTERNS in patterns.py's order: walk, trot, crawl.
constexpr int kPatterns = 3;
extern const Pattern kPatternTable[kPatterns];
// The long trot ("longtrot", remote key A1; planner/longtrot.h): not in the table, as LONG_TROT is
// not in patterns.py's PATTERNS. Its timing is its own (pattern_timing(), not timing()).
extern const Pattern kLongTrot;
inline bool is_long_trot(const Pattern& p) { return &p == &kLongTrot; }
const Pattern& default_pattern();  // trot
// By name, ignoring case (the table's three and "longtrot"); null if unknown.
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

real shift_min_ms(const Pattern& p);
// Speed from the stride alone, the cycle fixed (the patterns without a lean).
inline bool fixed_cadence(const Pattern& p) { return !p.lean; }
// The pattern's default speed: its table's, or its stride over its default stance.
real default_speed(const Pattern& p);
// patterns.py step_height(): the lift at mid-swing for a swing of |stride|:
// step_h_mm, tapering to step_h_long_mm at the longest stride.
real step_height(const Pattern& p, real stride);
// (slowest, fastest) speed that timing() gives.
void speed_range(const Pattern& p, real* lo, real* hi);
// Stride, shift and period for walking at |speed_mm_s|, clamped to speed_range().
Timing timing(const Pattern& p, real speed_mm_s);
Timing default_timing(const Pattern& p);
// (left, right) stride, + = the feet push the body forward: forward and back
// both sides, bent by steer (-1..1, + left: the left side's stride less steer
// x its size, the right's plus it, both scaled down together when the longer
// passes cap > 0, the pattern's longest stride); left and right turn in place
// (kTurnFrac of the stride), march in place (steer ignored for these).
void side_strides(Direction d, real stride, real out[2], real steer = 0, real cap = 0);

}  // namespace planner
}  // namespace kilo

#endif
