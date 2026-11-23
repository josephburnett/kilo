// gait: what a gait is to the engine (docs/movement-engine.md, step 2): where in
// one cycle of the step clock each leg lifts, how long a swing lasts, the feet's
// lift, the body's height and the weight target the solver keeps. A gait is a
// table row, not a program: the step clock (engine/clock.h) turns it into lifts
// and the solver (engine/solve.h) into feet.
//
// The three gaits:
// - walk: one leg at a time in the lateral sequence LB LF RB RF, a 10-tick
//   swing (11 past a 50 mm stride), a 48-tick (960 ms) cycle at its own
//   cadence, three shares of the four-foot time before a hind leg's lift and
//   one before a fore leg's (the weight crosses a diagonal there); the weight
//   kept 3.5 mm inside the three feet that stay down (docs/movement-engine-results.md,
//   §2 and "Step 2").
// - trot: the diagonal pairs LF+RB, RF+LB, one tick on four feet between
//   them; the swing grows with the speed (8 ticks to 150 mm/s, a tick more
//   every 18 mm/s, at most 11), each pair lands where the body, falling about
//   the line between its two feet as an inverted pendulum, ends the diagonal
//   upright (fit_land_x(): the pendulum fitted to the floor recordings of the trot).
//   No weight target: two feet carry him, the landing balances him.
// - crawl: one leg at a time, a 15-tick swing, the stance from the speed (the
//   stride 50 mm, the cycle 108 to 200 ticks), the weight 8 mm inside.
//
// Speed is stride x cadence. Each gait has its own cadence; a goal may ask for
// another (Timing's cadence, 0.6 to 1.6 of the gait's): the stride follows
// from the speed and the cadence, and a stride past the gait's longest raises
// the cadence instead, as far as the swing allows.
#ifndef KILO_MIND_ENGINE_GAIT_H
#define KILO_MIND_ENGINE_GAIT_H
#include <stdint.h>
#include "link/consts.h"
#include "planner/fmath.h"
#include "planner/model.h"

namespace kilo {
namespace engine {

using planner::real;
using planner::kLegs;
using planner::LF;
using planner::RF;
using planner::RB;
using planner::LB;

constexpr real kTickMs = (real)KILO_TICK_MS;
constexpr real kTickS = kTickMs / (real)1000.0;
constexpr real kStrideMinMm = 8;
constexpr real kTurnFrac = (real)0.8;  // a turn in place: each side's stride, this fraction of the stride
constexpr real kCadenceMin = (real)0.6;  // a goal's cadence, as a fraction of the gait's own
constexpr real kCadenceMax = (real)1.6;

// The trot's schedule and pendulum (docs/walking-and-balance.md §15).
constexpr real kTrotVFrom = 150;   // up to this speed the swing is 8 ticks ...
constexpr real kTrotVStep = 18;    // ... then a tick more every this many mm/s ...
constexpr int kTrotWMin = 8;
constexpr int kTrotWMax = 11;      // ... up to 11 ticks (220 ms)
constexpr real kTrotTopMmS = 207;  // the fastest forward ...
constexpr real kTrotTopBackMmS = 180;  // ... and backward
constexpr real kTrotStancePlus = (real)1.5;  // the stride: the speed x (swing + this many ticks)
constexpr real kTrotStepMmPerTick = (real)5.4;  // a swing carrying a foot d mm takes at least d / this ticks
constexpr real kTrotJointDegPerTick = 8;  // a swing that is not the steady trot's: no joint faster
constexpr real kSwingMmPerTick = (real)5.0;  // the walk's and the crawl's swing: at most this much of the stride a tick
// The pendulum: the "trot" view of the body model (protocol/spec.toml [model.views.trot]).
constexpr real kFitComXMm = (real)KILO_MODEL_VIEW_TROT_COM_X_MM;
constexpr real kFitExtraMs = (real)KILO_MODEL_VIEW_TROT_EXTRA_MS;
constexpr real kFitComHMm = (real)KILO_MODEL_VIEW_TROT_COM_H_MM;

struct Gait {
  const char* name;
  uint8_t n_groups;     // the legs that lift together, in the order they lift
  uint8_t group_size;
  uint8_t groups[4][2];
  real gap_share[4];    // each group's share of the cycle's four-foot time, before its lift
  int swing_ticks;      // a swing (the trot's at its slowest: it grows with the speed)
  real cycle_ticks;     // the cycle at the gait's own cadence (the trot's: 2 x (swing + 1))
  real stride_mm;       // the longest stride at the gait's own cadence
  real step_h_mm;       // the foot's lift at mid-swing
  int body_h_256;       // the body's height, /256 of L1 + L2
  real margin_mm;       // the weight this far inside the feet that stay down; < 0: none (the trot)
  real default_mm_s;
  real stride_slew_mm;  // a side's stride changes at most this much a tick
  bool centre_com;      // the strides centred under the model's centre of mass
  bool pendulum;        // the trot: its swing by the speed, its landing by the pendulum
  bool stance_from_speed;  // the crawl: the speed sets the cycle (slower: a longer stance), not the stride
  real cycle_max_ticks;    // ...as far as this
  real roll_max_deg;       // the body's roll the solver may use for the weight
  real roll_cost;          // a degree of it costs as this many mm of fore-aft shift (squared)
};

constexpr int kGaits = 3;
extern const Gait kGaitTable[kGaits];  // walk, trot, crawl
inline bool is_trot(const Gait& g) { return g.pendulum; }
const Gait& default_gait();             // the trot
const Gait* find_gait(const char* name);  // by name, ignoring case; null if unknown

enum class Direction : uint8_t { Forward, Back, Left, Right, March };
// By name, ignoring case ("backward", "backwards" = back); false if unknown.
bool find_direction(const char* name, Direction* out);
const char* direction_name(Direction d);

// What a gait does at a speed and cadence.
struct Timing {
  real stride_mm;    // magnitude
  int swing_ticks;
  real cycle_ticks;  // one cycle of the clock
  real stance_ticks;  // a foot's time on the ground: cycle - swing (the trot's: swing + 1.5)
  real period_ms;    // the cycle, ms
  real speed_mm_s;   // what it walks at
  real cadence;      // the cycle against the gait's own (1 = its own; > 1 faster)
};

// Where in the cycle (0..1) each leg lifts, for a w-tick swing in a cycle of `cycle` ticks: each
// group after its share of the four-foot time, then its swing.
void offsets(const Gait& g, int w, real cycle, real out[kLegs]);

// The trot's swing at |speed| (the build-up's target), the pendulum's gain for a w-tick swing and
// where a pair walking at v (+ forward) lands for no tip (mm ahead of its shoulder).
int trot_swing_for(real speed_mm_s);
real fit_k(int swing_ticks);
real fit_land_x(int swing_ticks, real speed_mm_s);

real default_speed(const Gait& g);
// The slowest and fastest speed timing() gives at the gait's own cadence.
void speed_range(const Gait& g, real* lo, real* hi);
// The timing for |speed|, clamped to what the gait can do; cadence 0: the gait's own.
Timing timing(const Gait& g, real speed_mm_s, Direction d = Direction::Forward, real cadence = 0);
// The lift at mid-swing for a stride (the gait's step_h_mm).
inline real step_height(const Gait& g, real) { return g.step_h_mm; }
// (left, right) stride, + = the feet push the body forward: forward and back
// both sides, bent by steer (-1..1, + left: the left side's stride less steer
// x its size, the right's plus it, both scaled down together when the longer
// passes cap > 0); left and right turn in place (kTurnFrac of the stride),
// march in place (steer ignored for these).
void side_strides(Direction d, real stride, real out[2], real steer = 0, real cap = 0);

}  // namespace engine
}  // namespace kilo

#endif
