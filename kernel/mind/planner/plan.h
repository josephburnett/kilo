// plan: the step planner, one tick of the whole body at a time. A port of
// mind/kilo/planner/plan.py; its docstring explains the method in full
// (docs/walking-and-balance.md §10, §12). In short:
//
// - Slots: the pattern's swing groups take turns; each slot is a four-foot
//   shift of S ticks, then a swing of W ticks per leg (the walk's second leg
//   couple_ticks() after its first). Feet on the ground slide back at their
//   side's stride / stance time; a swinging foot lifts on a cosine profile
//   to half a stride ahead of its shoulder. Stride targets move at most the
//   pattern's stride_slew_mm a tick (1.5 mm in the walk and the trot, 0.5 in
//   the crawl), the stance time 2 % a tick. A steer bends a forward or
//   backward walk (side_strides). A change between the walk
//   and the trot goes straight over at a slot start; into or out of the
//   crawl through the legs' way home.
// - The lean (the crawl): before each lift the body's centre of mass is
//   moved at least margin_mm inside the three feet that stay down (a small
//   quadratic program on the linearised model), the lift starting
//   kLeanLeadMs after the move's end (three ticks before it, in its cosine
//   tail; the servo lag delays the lean and the lift alike). The solve is spread over the swing before the slot, a few
//   units of work a tick (LeanJob), so no tick carries a whole solve.
// - Prediction: the attitude for tick t is the plane through the support
//   feet planned, delayed by the servos' dead time and passed through their
//   first-order response; it goes in FEET as the balance's reference.
// - From the measured state (observe): no lift while the body is off the
//   plan by more than gate_deg (the plan waits, up to hold_max ticks); a
//   swinging leg lands at once when the body tips toward it by catch_deg.
// - Stop: the strides go to 0, each leg steps home, the body levels at the
//   stance height, then done.
// - The quick stop (halt(); the remote's B6 and C7, the distance guard):
//   every foot in the air comes down where it is (three ticks, as a catch)
//   and no other lifts; the strides go to 0 in kHaltTicks (the feet on the
//   ground stop sliding: the body stops); four feet down until
//   kHaltHoldTicks after the halt; then each leg further than kHaltSquareMm
//   from home steps there (the crawl at its fastest cadence), the body
//   levels, done. A leg caught on its way stays where it landed.
//
// - The long trot (kLongTrot, longtrot.h; plan.py's LongTrotPlanner in
//   mind/kilo/planner/longtrot.py, which this matches tick for tick): the
//   swing's length varies (swing_ticks(): the speed's, at most a tick longer
//   than the last slot's, long enough for its step, and away from the steady
//   long trot long enough for no joint to pass kLtJointDegPerTick a tick), one
//   tick on four feet, each pair landing at fit_land_x() instead of half a
//   stride ahead. Changes to and from the walk and the trot go straight over
//   (out of it once its swing is 8 ticks again); to and from the crawl
//   through the legs' way home. In the walk, the trot and the crawl nothing of
//   this applies.
//
// Fixed-size state, no heap. Planner::next() is what the Walker calls once
// per tick, about five ticks ahead of the body.
#ifndef KILO_MIND_PLANNER_PLAN_H
#define KILO_MIND_PLANNER_PLAN_H
#include <stdint.h>
#include "planner/fmath.h"
#include "planner/longtrot.h"
#include "planner/model.h"
#include "planner/patterns.h"

namespace kilo {
namespace planner {

constexpr int kXMin = -60, kXMax = 60;  // FEET x_mm (spec range)
constexpr int kZMin = 30, kZMax = 95;   // FEET z_mm
constexpr real kAttMaxDeg = 30;         // FEET pitch / roll: the body refuses a FEET beyond it
constexpr real kHeightSlewMm = (real)0.5;
constexpr real kRecentredMm = (real)1.5;  // a stopping leg this close to its target does not step again
constexpr real kCentreSlewMm = (real)0.5;  // the strides' centre (Pattern::centre_com) moves at most this a tick
constexpr real kSwitchSlackMm = (real)4.0;  // a pattern change: every foot within the new half stride plus this
constexpr real kStanceSlew = (real)0.02;
constexpr int kLeanUnitsPerTick = 4;      // units of a lean solve per tick
constexpr int kHaltTicks = 6;             // the quick stop: the strides to 0 in this many ticks (120 ms) ...
constexpr int kHaltHoldTicks = 10;        // ... four feet down this long from the halt (200 ms) ...
constexpr real kHaltSquareMm = (real)5.0;  // ... then a leg further than this from home steps there
constexpr int kHaltNotLifted = -1000;     // caught_j_ of a leg the quick stop keeps down (it slides)

struct TickPlan {
  uint32_t tick;     // unwrapped body tick
  int8_t x[kLegs];   // mm, as sent
  uint8_t z[kLegs];
  uint8_t support;   // bit i: leg i on the ground
  real pitch;        // predicted at this tick (the servo lag included), degrees
  real roll;
  real plan_pitch;   // the attitude these feet give (no lag)
  real plan_roll;
  real margin;       // mm of the centre of mass inside the support polygon
  uint8_t lifting;   // bit i: leg i in the air this tick
  uint8_t n_lifting;
  // x and z before rounding to whole mm. The plan has exact .5 ties (a
  // trot's foot lands at stride / 2 = 12.5 mm): there float and Python's
  // double round either way, and the golden test allows that 1 mm.
  real x_raw[kLegs];
  real z_raw[kLegs];
};

struct Lean {
  real dx = 0;     // body forward of its feet, mm
  real pitch = 0;  // degrees, nose down +
  real roll = 0;   // degrees, right side up +

  Lean lerp(const Lean& o, real s) const {
    return {dx + (o.dx - dx) * s, pitch + (o.pitch - pitch) * s, roll + (o.roll - roll) * s};
  }
  bool operator==(const Lean& o) const { return dx == o.dx && pitch == o.pitch && roll == o.roll; }
  bool operator!=(const Lean& o) const { return !(*this == o); }
};

struct PlannerConfig {
  real margin_mm = 14;       // the centre of mass this far inside the three feet, every swing
  real dx_max_mm = 18;       // the body's fore-aft shift over its feet
  real dx_speed_mm_s = 100;  // ...and its fastest move
  real roll_max_deg = 14;
  real pitch_max_deg = 6;
  real dx_scale_mm = 10;     // the lean's cost per unit: a mm of shift, a degree of pitch, of roll
  real pitch_scale_deg = 2;
  real roll_scale_deg = 8;
  real gate_deg = 5;         // no lift while the body is off the plan by more than this
  int hold_max = 25;         // ...for at most this many ticks (then it lifts anyway)
  real catch_deg = 7;        // a swinging leg lands at once when the body tips toward it this far
  bool lean = true;          // false: the same steps without the body's move
  bool fixed_shift = false;  // true: shift_ms below instead of from the speed
  real shift_ms = 0;
};

struct PlannerStats {
  uint32_t holds;
  uint32_t catches;
  uint32_t waits;
  real min_margin;  // inf until a tick on three feet
};

// One quadratic program of the lean (plan.py _qp_steps): min sum((d_j /
// scale_j)^2) s.t. m_e + g_e . d >= want, by enumerating active sets of up
// to three rows. Double precision (its feasibility test is to 1e-6).
// resume() is one unit of work (one next() of the Python generator).
struct QpRow {
  double g[3];
  double m;
};
constexpr int kMaxRows = 2 * kLegs;

class Qp {
 public:
  void init(const QpRow* rows, int n, double want, const double scale[3]);
  bool resume();  // true when done: found, step
  bool found = false;
  double step[3] = {};

 private:
  bool feasible(const double w[3]) const;
  void try_set(const int* idx, int k);
  double W_[kMaxRows][4] = {};  // a . w >= b
  double s_[3] = {};
  int n_ = 0;
  int pc_ = 0;
  int size_ = 0;
  bool has_best_ = false;
  double best_[3] = {};
  double best_n_ = 0;
};

// One solve of the lean (plan.py _solve_steps): the smallest body move that
// keeps the centre of mass margin_mm inside the feet that stay down at
// lift-off and at touchdown. resume() is one unit (one margin evaluation,
// or one part of the quadratic program).
class LeanSolve {
 public:
  void init(const BodyModel* m, const PlannerConfig* cfg, const Pattern& p, int k, const real xg[kLegs],
            const real speed[kLegs], const real tgt[kLegs], real h, int shift, int swing);
  bool resume();  // true when done: result
  Lean result;

 private:
  int edge_margins(const real xs_gait[kLegs], const real u[3], real out[kLegs]) const;
  enum Phase : uint8_t { Eval, QpStart, QpRun };
  const BodyModel* m_ = nullptr;
  const PlannerConfig* cfg_ = nullptr;
  real configs_[2][kLegs] = {};
  int support_[kLegs] = {};
  int nsup_ = 0;
  real h_ = 0;
  real u_[3] = {};
  int it_ = 0, ci_ = 0, j_ = -1;
  real base_[kLegs] = {};
  int nb_ = 0;
  real grads_[3][kLegs] = {};
  QpRow rows_[kMaxRows] = {};
  int nrows_ = 0;
  Phase phase_ = Eval;
  double want_ = 0;
  Qp qp_;
};

// One slot's lean, solved a few units at a time (plan.py _LeanJob and
// _lean_steps): the inputs it was started from, and once done the slot's
// shift ticks, the lean and its move's ticks.
struct LeanJob {
  const Pattern* pattern;
  int k;
  real xg[kLegs];
  real s[2];
  real stance;
  int shift;
  Lean lean_from;
  bool has_result;
  int res_S;
  Lean res_lean_to;
  int res_tr;
  // _lean_steps' state
  int pc;
  int S;
  int swing;
  real h, h_now;
  real speed[kLegs], tgt[kLegs];
  LeanSolve solve;

  bool matches(const Pattern* p, int k, const real xg[kLegs], const real s[2], real stance, int shift,
               const Lean& lean_from) const;
};

class Planner {
 public:
  explicit Planner(const BodyModel& model = BodyModel(), const PlannerConfig& cfg = PlannerConfig());

  // A new walk (Python's Planner(...)): the first slot's lean is solved
  // here. speed_mm_s < 0: the pattern's default speed (Python's None; a
  // negative speed is a magnitude in Python). start_height < 0: STANCE's.
  void start(const Pattern& p, Direction d, real speed_mm_s, uint32_t tick, real start_height = -1, real steer = 0);

  void set_speed(real speed_mm_s);
  void set_direction(Direction d);
  // Bend the path of a forward or backward walk: -1..1, + left (side_strides;
  // the remote's A6 / C6 while walking). Ignored by the turns in place.
  void set_steer(real steer);
  // Change the pattern. Between the walk and the trot (both fixed cadence)
  // directly, at the first slot start where the strides, the feet and the
  // height allow (plan.py _switch_ok); into or out of the crawl every leg
  // steps home first, then the new one starts from its first group. The
  // walking pattern cancels a change under way; ignored once stopping.
  void set_pattern(const Pattern& p);
  void stop();
  // The quick stop (above). Also during the planned stop (it becomes this);
  // nothing once finishing.
  void halt();
  // The body's latest error: measured - predicted - the steady offset (degrees).
  void observe(real err_pitch, real err_roll) {
    err_[0] = err_pitch;
    err_[1] = err_roll;
  }

  // Plan the next tick (tick()) and advance.
  const TickPlan& next();

  // The pattern's timing at the speed set (the long trot's own: pattern_timing()).
  Timing timing() const { return pattern_timing(*pattern_, speed_mm_s_, direction_); }
  int swing_ticks() const;
  bool changing() const { return pending_ != nullptr; }
  bool long_trot() const { return is_long_trot(*pattern_); }
  bool stopping() const { return stopping_; }
  bool halting() const { return halting_; }
  int halt_ticks() const { return halt_t_; }  // ticks planned since the halt
  bool done() const { return done_; }
  uint32_t tick() const { return tick_; }
  int lead() const { return lead_; }      // the lift after the lean's move, ticks (negative: before its end)
  int settle() const { return settle_; }  // a level-out done this long (the servos' response) before what follows
  int couple_ticks() const;                // a group's second leg lifts this many ticks after its first
  const Pattern& pattern() const { return *pattern_; }
  const Pattern* pending_pattern() const { return pending_; }
  Direction direction() const { return direction_; }
  real steer() const { return steer_; }
  real speed_mm_s() const { return speed_mm_s_; }
  const PlannerStats& stats() const { return stats_; }
  const TickPlan& last() const { return last_; }
  bool has_last() const { return has_last_; }
  const BodyModel& model() const { return model_; }
  const PlannerConfig& config() const { return cfg_; }
  // Internals the tests look at (plan.py's attributes of the same names).
  const real* s_cur() const { return s_cur_; }
  real stance_ticks() const { return stance_ticks_; }
  real stance_target() const { return stance_tgt_; }

  // plan.py _move_ticks, public for LeanJob.
  int move_ticks(const Lean& a, const Lean& b, real height) const;

 private:
  bool leaning() const { return pattern_->lean && cfg_.lean; }
  int shift_for(real stance) const;
  int span() const;            // a slot's ticks from the first lift to the last touchdown
  int offset(int leg) const;   // the tick after the shift at which this leg of the slot's group lifts
  bool direct_change() const;  // the change under way goes straight over (both patterns fixed cadence)
  bool switch_ok(const Pattern& p) const;
  int first_group() const;
  bool home() const;
  real recentred() const { return halting_ ? kHaltSquareMm : kRecentredMm; }
  void apply_timing();
  void start_slot();
  void start_slot_base();  // plan.py Planner._start_slot (start_slot adds the long trot's swing)
  bool homing() const;      // every leg goes home: a stop, or a change into or out of the crawl
  bool swing_ok(int leg, real to, int w) const;
  real land_for(int side) const;
  void slewed(real s[2], real* stance, real* h) const;
  real height_target() const;
  void new_job(LeanJob* job, int k, const real xg[kLegs], const real s[2], real stance, const Lean& lean_from,
               real h_now) const;
  bool run_job(LeanJob* job, int units) const;  // units < 0: all of it
  void apply_job(const LeanJob& job);
  void predict_slot_end(real xg[kLegs], real s[2], real* st, real* h) const;
  void lagged(real* pitch, real* roll) const;
  real tip_toward(int leg) const;
  real side_speed(int leg) const { return s_cur_[kLeft[leg] ? 0 : 1] / stance_ticks_; }
  real target_x(int leg) const;
  real centre_target() const;  // plan.py _centre_target

  static constexpr int kHistMax = 32;

  BodyModel model_;
  PlannerConfig cfg_;
  uint32_t tick_ = 0;
  int lead_ = 0;
  int settle_ = 0;
  const Pattern* pattern_;
  Direction direction_ = Direction::Forward;
  real steer_ = 0;
  real speed_mm_s_ = 0;
  const Pattern* pending_ = nullptr;
  real xg_[kLegs] = {};  // foot x in the gait frame (no lean shift)
  real h_ = 0;
  real s_cur_[2] = {};   // stride now, left and right
  real s_tgt_[2] = {};
  real centre_ = 0;      // mm ahead of its shoulder a swing's landing is centred (centre_target())
  real stance_ticks_ = 1;  // a foot's time on the ground now (slewed toward stance_tgt_)
  real stance_tgt_ = 1;
  Lean lean_, lean_from_, lean_to_;
  int tr_ticks_ = 1;  // the lean's move takes this many ticks from the slot's start
  int k_ = -1;        // slot index into the groups
  int t_ = 0;         // tick within the slot (stands still while the plan waits)
  int S_ = 0;         // this slot's shift ticks
  int held_ = 0;      // ticks the gate held this slot
  real swing_[kLegs] = {};  // per leg: x at lift-off
  real swing_h_[kLegs] = {};  // ...and its lift at mid-swing (step_height() for its side's stride then)
  bool skip_ = false;
  bool levelling_ = false;  // this slot's shift levels the body out of the crawl's lean: strides held at 0
  bool caught_[kLegs] = {};
  int caught_j_[kLegs] = {};  // the slot tick (after the shift) of the catch
  real caught_lift_[kLegs] = {};
  bool stopping_ = false;
  bool halting_ = false;
  int halt_t_ = 0;
  real halt_slew_ = 0;
  bool halt_fast_ = false;
  bool halt_kept_[kLegs] = {};  // caught in the quick stop: it stays where it landed
  bool finishing_ = false;
  bool done_ = false;
  real err_[2] = {};
  Attitude prev_att_ = {0, 0};
  real lag_ = 0;    // the servos' dead time, ticks
  real alpha_ = 1;  // their first-order response's step per tick
  Attitude resp_ = {0, 0};  // the predicted attitude: the plan's, delayed, through the response
  int hist_n_ = 0;
  Attitude att_hist_[kHistMax] = {};
  bool has_job_ = false;   // this slot's lean while it is still being solved (the slot waits)
  bool has_next_ = false;  // the next slot's, solved during this slot's swing
  LeanJob job_ = {};
  LeanJob next_job_ = {};
  TickPlan last_ = {};
  bool has_last_ = false;
  PlannerStats stats_ = {};
  // the long trot (longtrot.py LongTrotPlanner): this slot's swing ticks (kLtWMin while not in it), the
  // speed's, and this slot's landing per side (left, right) for the target stride at its start
  int lw_ = kLtWMin;
  int lw_tgt_ = kLtWMin;
  real land_[2] = {};
  real slot_s_[2] = {};
};

}  // namespace planner
}  // namespace kilo

#endif
