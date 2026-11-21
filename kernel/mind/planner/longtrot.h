// longtrot: the long trot's schedule and the fitted pendulum. A port of
// mind/kilo/planner/longtrot.py (docs/walking-and-balance.md §15); the
// planner's slot rules for it are in plan.cpp (Planner, where the pattern is
// kLongTrot), as LongTrotPlanner's are in longtrot.py.
//
// The trot's stride is limited by its joints: its 8-tick swing cannot carry
// more than about 44 mm under the body's 500 deg/s. The long trot's swing
// grows with the speed instead (long_swing_for: 8 ticks up to 150 mm/s, a
// tick more for every 18 mm/s, at most 11, 220 ms), a tick a step at most (the
// build-up), with one tick on four feet as in the trot: the stride is the
// speed x (swing + 1.5 ticks), 51.8 mm at the top (207 mm/s). Each diagonal
// pair lands where the body, falling about the line between its two feet as
// an inverted pendulum, ends the diagonal upright: x_c + k v tau ahead of its
// shoulder (fit_land_x; the pendulum's numbers fitted to the floor
// recordings of the trot, TrotFit in longtrot.py).
#ifndef KILO_MIND_PLANNER_LONGTROT_H
#define KILO_MIND_PLANNER_LONGTROT_H
#include "planner/fmath.h"
#include "planner/patterns.h"

namespace kilo {
namespace planner {

// LongTrotConfig's defaults (longtrot.py)
constexpr real kLtVFrom = 150;             // up to this speed the swing is the trot's 8 ticks ...
constexpr real kLtVStep = 18;              // ... then a tick more every this many mm/s ...
constexpr int kLtWMin = 8;
constexpr int kLtWMax = 11;                // ... up to 11 ticks (220 ms)
constexpr real kLtTopMmS = 207;            // the fastest forward (51.8 mm at 11 ticks)
constexpr real kLtTopBackMmS = 180;        // ... and backward
constexpr real kLtStepMmPerTick = (real)5.4;   // a swing carrying a foot d mm takes at least d / this ticks
constexpr real kLtStridePerTick = (real)4.75;  // a side's stride at most this x the swing's ticks less 0.5 mm
constexpr real kLtJointDegPerTick = 8;     // a swing that is not the steady long trot's: no joint faster
constexpr real kLtStancePlus = (real)1.5;  // a foot's ground time: the swing + this many ticks

// TrotFit (TROT_FIT): the trot's rock fitted to the floor recordings
constexpr real kFitComXMm = (real)3.5;  // the centre of mass's effective position ahead of the shoulders' middle
constexpr real kFitExtraMs = 120;       // the diagonal rocks as if it lasted this much longer
constexpr real kFitComHMm = 79;         // the centre of mass's height

// The swing's ticks at |speed| (the build-up's target).
int long_swing_for(real speed_mm_s);
// The pendulum's gain k for a w-tick swing, and where a pair walking at v (+ forward) lands for no
// tip (mm ahead of its shoulder).
real fit_k(int swing_ticks);
real fit_land_x(int swing_ticks, real speed_mm_s);
// longtrot.py timing(): (swing, stride, speed) at |speed| (< 0: 144), clamped to the trot's slowest
// and to kLtTopMmS (kLtTopBackMmS backward).
struct LongTiming {
  int swing;
  real stride_mm;
  real speed_mm_s;
};
LongTiming long_timing(real speed_mm_s, bool back);
// longtrot.py pattern_timing(): timing() for the walk, the trot and the crawl; the long trot's own
// for kLongTrot (shift one tick, stance (swing + 1.5) ticks, period stance + swing).
Timing pattern_timing(const Pattern& p, real speed_mm_s, Direction d = Direction::Forward);

}  // namespace planner
}  // namespace kilo

#endif
