// trot: the trot's schedule and its fitted pendulum (docs/walking-and-balance.md §15). A port of
// mind/kilo/planner/patterns.py's TrotConfig, swing_for and trot_timing and plan.py's TrotFit; the
// planner's slot rules for the trot are in plan.cpp (Planner, where the pattern is_trot()), as in
// plan.py's Planner.
//
// The trot's stride is limited by its joints: a fixed 8-tick swing cannot carry more than about
// 44 mm under the body's 500 deg/s. So its swing grows with the speed (trot_swing_for: 8 ticks up
// to 150 mm/s, a tick more for every 18 mm/s, at most 11, 220 ms), a tick a step at most (the
// build-up), with one tick on four feet: the stride is the speed x (swing + 1.5 ticks), 51.8 mm at
// the top (207 mm/s). Each diagonal pair lands where the body, falling about the line between its
// two feet as an inverted pendulum, ends the diagonal upright: x_c + k v tau ahead of its shoulder
// (fit_land_x; the pendulum's numbers fitted to the floor recordings of the trot).
#ifndef KILO_MIND_PLANNER_TROT_H
#define KILO_MIND_PLANNER_TROT_H
#include "planner/fmath.h"
#include "proto/kilo_protocol.h"

namespace kilo {
namespace planner {

// TrotConfig's defaults (patterns.py)
constexpr real kTrotVFrom = 150;             // up to this speed the swing is 8 ticks ...
constexpr real kTrotVStep = 18;              // ... then a tick more every this many mm/s ...
constexpr int kTrotWMin = 8;
constexpr int kTrotWMax = 11;                // ... up to 11 ticks (220 ms)
constexpr real kTrotTopMmS = 207;            // the fastest forward (51.8 mm at 11 ticks)
constexpr real kTrotTopBackMmS = 180;        // ... and backward
constexpr real kTrotStepMmPerTick = (real)5.4;   // a swing carrying a foot d mm takes at least d / this ticks
constexpr real kTrotStridePerTick = (real)4.75;  // a side's stride at most this x the swing's ticks less 0.5 mm
constexpr real kTrotJointDegPerTick = 8;     // a swing that is not the steady trot's: no joint faster
constexpr real kTrotStancePlus = (real)1.5;  // a foot's ground time: the swing + this many ticks

// TrotFit (TROT_FIT): the trot's rock fitted to the floor recordings. Its numbers are the "trot"
// view of the body model (protocol/spec.toml [model.views.trot]): they disagree with the model's
// own centre of mass and stay the trot's until a self-measurement settles it (kilo model show).
constexpr real kFitComXMm = (real)KILO_MODEL_VIEW_TROT_COM_X_MM;   // the centre of mass's effective position
                                                                   //   ahead of the shoulders' middle
constexpr real kFitExtraMs = (real)KILO_MODEL_VIEW_TROT_EXTRA_MS;  // the diagonal rocks as if it lasted this
                                                                   //   much longer
constexpr real kFitComHMm = (real)KILO_MODEL_VIEW_TROT_COM_H_MM;   // the centre of mass's height

// The trot's swing ticks at |speed| (the build-up's target).
int trot_swing_for(real speed_mm_s);
// The pendulum's gain k for a w-tick swing, and where a pair walking at v (+ forward) lands for no
// tip (mm ahead of its shoulder).
real fit_k(int swing_ticks);
real fit_land_x(int swing_ticks, real speed_mm_s);

}  // namespace planner
}  // namespace kilo

#endif
