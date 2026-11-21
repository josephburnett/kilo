// script: the self-measurement routine's poses, tick by tick (docs/self-measurement.md).
// A port of mind/kilo/measure/script.py (the experiments, the steps and
// Runner) and of kilo.measure.statics.rigid_rest (the model's prediction);
// kernel/host/tests/measure_vectors.txt, made by gen_measure_vectors.py
// from the Python, checks the two tick for tick.
//
// A pose is the four feet as FEET gives them: x mm fore-aft from under the
// shoulder (+ forward), z mm below it, whole mm. Every pose is the home
// pose (x 0, z kHomeZ: the body's STANCE) plus a step's offsets; a step
// moves to its offsets (Move, linearly, in integers), holds (Hold) or
// sweeps the feet's x 1 mm every kTicksPerMm ticks toward its offsets
// (Sweep). This is script 2 (script.py SCRIPTS[2]; script 1, of 2026-11-21,
// lives on in the Python only, for its recordings). The experiments, in order:
//
//   level           stand still 1.5 s: the IMU's offset (with the floor's slope)
//   stairs_pitch    front legs shorter, back longer by 2, 4, 6 mm and the other way, each level held and
//                   reached from below and from above: tilt per mm (the feet's rectangle), hysteresis,
//                   and in the 6 mm jumps the servos' dead time and time constant
//   stairs_roll     the same left and right by 2, 4 mm
//   twist           the body 20 mm forward; LF and RB longer, RF and LB shorter by 1, 2 mm and the other
//                   way: the legs' built-in twist (the point of the pitch's V), their sag
//   splay           nose down; front feet forward and back feet back, then inward, twice: the x scale
//   edge_lf ..      a foot 5 mm up, held (the tripod's lean is data), the feet swept until the centre of
//   edge_rb         mass crosses the diagonal and Kilo tips onto the lifted foot (caught by the floor 5 mm
//                   down): the centre of mass fore-aft and sideways
//   edge_lf_away, edge_lf_toward, edge_rf_away, edge_lb_toward
//                   the same rolled, the mass leaning away from the lifted foot (7 deg) or toward it
//                   (5.3 deg): against the flat sweep of the same foot, the centre of mass's height
//   check           six held poses the fit does not use (it is tested on them)
//   level_end       stand still 1 s: the IMU's drift
//
// A sweep ends early on a tip (Measure watches TELEMETRY; script.py TipWatch is the rule's reference):
// lifting a foot leans the tripod by itself, so the baseline is the mean of the lifted hold's last
// kTipBaseTicks, and a tip is the tilt toward the lifted foot kTipRiseDeg above it having risen
// kTipJumpDeg within kTipJumpTicks. tip() then makes the sweep go on kTipExtraMm past the x of the tick
// the tip was seen at; kTipUpFraction of the model's tip from the settle hold's tilt during the lifted
// hold (he tipped as the foot came up) makes the sweep only kTipExtraMm long.
#ifndef KILO_MIND_MEASURE_SCRIPT_H
#define KILO_MIND_MEASURE_SCRIPT_H
#include <stdint.h>
#include "planner/model.h"

namespace kilo {
namespace measure {

using planner::real;

constexpr int kScriptVersion = 2;
constexpr int kHomeZ = 76;           // STANCE's height (gait.stance_h_256 205 of L1 + L2 = 76.07 mm)
constexpr int kLiftMm = 5;           // a lifted foot's height: how far Kilo can tip before the floor catches him
constexpr int kTicksPerMm = 6;       // a sweep's pace (8.3 mm/s)
constexpr int kTipExtraMm = 3;       // a sweep goes on this far past the tip
constexpr int kTipBaseTicks = 10;    // the tip's baseline: the mean of the lifted hold's last ticks
constexpr real kTipRiseDeg = (real)1.5;   // a tip: the tilt toward the lifted foot this far above the baseline
constexpr real kTipJumpDeg = (real)1.0;   // ...and risen this much
constexpr int kTipJumpTicks = 8;          // ...within this many ticks
constexpr real kTipUpFraction = (real)0.6;  // tipped as the foot came up: this much of the model's tip from the
                                            // settle hold's tilt, during the lifted hold
constexpr int kLiftedTicks = 25;     // the lifted hold (its lean is data)
constexpr int kHomeHold = 16;        // an experiment's last hold, back at home
constexpr int kStepPitchMm = 6;      // the largest pitch stair: front legs this much shorter, back longer (5.6 deg)
constexpr int kStepRollMm = 4;       // the largest roll stair: left legs shorter, right longer (7.0 deg)
constexpr int kStairHold = 26;       // ticks a stair is held, after its kStairMove ticks of move
constexpr int kStairMove = 2;
constexpr int kTwistX = -20;         // the twist steps' feet x: the body 20 mm forward
constexpr int kSplayPitchMm = 6;     // the splay's pitch (5.6 deg)
constexpr int kSplayOutMm = 16;      // front feet this far forward and back feet as far back
constexpr int kSplayInMm = 14;       // ...and inward (the RB shoulder's reach)
constexpr int kRollSweepMm = 4;      // a rolled sweep away from the lifted foot (7.0 deg)
constexpr int kRollSweepBMm = 3;     // ...and toward it (5.3 deg: deeper leaves the mass too near the side)
constexpr int kSweepSpanMm = 20;     // a sweep starts this far before the model's crossing and ends at most as far past
constexpr int kRollSpanMm = 16;      // ...rolled (the feet's reach)
constexpr int kRbBackMm = 6;         // edge_rb starts this much further back and ends at kRbMaxX:
constexpr int kRbMaxX = 14;          // the RB shoulder's pulse clamp (Kilo's calibration) with RB up

enum class StepKind : uint8_t { Move, Hold, Sweep };

struct Experiment {
  const char* name;
  int8_t lifted;                     // the foot a sweep lifts (planner leg), -1 none
  int16_t tip_pitch_a16, tip_roll_a16;  // the model's tip onto the lifted foot, 1/16 deg
};

constexpr int kExperiments = 15;
extern const Experiment kExperimentTable[kExperiments];

struct Step {
  uint8_t exp;           // kExperimentTable index
  const char* phase;     // the mark's name for this part
  StepKind kind;
  uint16_t ticks;        // Move, Hold: its length; Sweep: its longest
  bool keep_x;           // the x where the step before ended (a sweep's lower and lowered), not dx
  int8_t dx[4];          // the feet's x offsets at the step's end, LF RF RB LB
  int8_t dz[4];          // the legs' z offsets (- = shorter: that corner lower, or that foot lifted)
};

constexpr int kMaxSteps = 224;

struct Pose {
  int8_t x[4];
  uint8_t z[4];
};

// The steps (built once, as script.py _steps()).
int steps(const Step** out);
// Ticks of the whole script with no sweep ended early.
uint32_t max_ticks();

class Script {
 public:
  Script();
  void reset();
  // The next tick's pose (and whether it begins its step: a mark); false once over.
  bool next(Pose* out, bool* first);
  // A tip seen in the TELEMETRY of script tick tick_seen (above). False if no lifted hold before a
  // sweep or sweep is under way, or this one has tipped already.
  bool tip(uint32_t tick_seen);
  bool done() const { return done_; }
  // The step the next tick belongs to (valid until done()), the ticks planned so far.
  int step_index() const { return i_; }
  const Step& step() const { return steps_[i_]; }
  uint32_t tick() const { return t_; }
  // Ticks into its step of the tick next() gave last (0 = the step's first).
  uint32_t into() const { return j_ > 0 ? j_ - 1 : 0; }
  bool tipped() const { return tipped_; }
  uint32_t tipped_at() const { return tipped_at_; }
  // The sweep's x at a script tick it planned (the last kHistory), or the current x.
  int x_at(uint32_t tick) const;

 private:
  static constexpr int kHistory = 64;
  void begin();
  const Step* steps_;
  int n_ = 0;
  int i_ = 0;
  uint32_t j_ = 0, t_ = 0;
  int x_[4] = {}, z_[4] = {};
  int sx_[4] = {}, sz_[4] = {};
  int sweep_end_ = 0;
  bool tipped_ = false, tip_pending_ = false;
  uint32_t tipped_at_ = 0;
  bool done_ = false;
  struct Hist {
    uint32_t tick;
    int x;
  };
  Hist hist_[kHistory] = {};
};

// The rigid model's rest for feet (xs, zs) (statics.py rigid_rest): all four feet on one plane
// (within kTouchMm), else the tripod of the tick before (prev) while possible, else the possible
// tripod with the most margin; *support the feet down.
constexpr real kTouchMm = (real)0.1;
planner::Attitude rigid_rest(const planner::BodyModel& m, const real xs[4], const real zs[4], uint8_t prev,
                             uint8_t* support);

}  // namespace measure
}  // namespace kilo

#endif
