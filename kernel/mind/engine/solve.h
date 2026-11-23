// solve: the engine's tick solver for walking (docs/movement-engine.md, step 2).
// It replaces the hand-tuned gait planner: one step clock (engine/clock.h) and
// a few wishes, solved again every tick from where the feet are now.
//
// Every tick (next()):
// 1. The commands move toward what was asked: each side's stride at most the
//    gait's stride_slew_mm a tick, the stance time 2 % a tick, the height
//    0.5 mm a tick. Speed, steer, direction and cadence change at any tick,
//    without a stop.
// 2. The step clock advances (unless the solver waits, below) and says which
//    legs lift now. A lifted foot swings swing_ticks on a cosine path to its
//    landing: half a stride ahead of its shoulder (the walk's and the crawl's
//    centred under the model's centre of mass), or the trot's pendulum point
//    (gait.h). The feet on the ground slide back at their side's speed.
// 3. The weight (the solver proper). Over the next kHorizon ticks the clock
//    and the feet are known; at every one of those ticks with a foot in the
//    air and three on the ground, the centre of mass must be the gait's
//    margin_mm inside those three (the walk 3.5 mm, the crawl 8). The solver
//    finds the smoothest body path, a fore-aft shift and a roll per tick, that
//    keeps it there: the least shift and roll, the least change from tick to
//    tick and of that change, subject to every margin (each margin linearised
//    about the last solution), by a few sweeps a tick that start from the
//    last tick's path. This tick's body pose is the path's first point. The
//    model says where the centre of mass is (planner/model.h: masses, legs,
//    and [model.correction]).
// 4. The feet in the body frame (the gait frame less the shift), the legs'
//    lengths for the roll and height, the lift; quarter millimetres, as POSE
//    carries them. A leg whose servos start later than the robot's
//    ([model.correction] servo_lag_d_ms, the mean of its shoulder's and its
//    knee's) gets its foot that much earlier along its path, so the legs
//    arrive together (every leg 0 until a measurement says otherwise). The
//    attitude the body takes on its support feet, delayed by the servos'
//    dead time and passed through their response, is the prediction POSE
//    sends as the balance's reference.
//
// The clock waits (nothing advances, the feet hold) when a lift is due and
// the body is off the plan by more than gate_deg (the board's report through
// observe()), or the weight is not yet where the lift needs it, for at most
// hold_max ticks. A swinging foot comes down where it is (three ticks) when
// the body tips toward it by catch_deg. These are the clock's, so every gait
// has them.
//
// Stop: the strides go to 0, each leg not home steps home on its turn, the
// body levels at the stance height, then done. The quick stop (halt()):
// every foot in the air comes down where it is, no other lifts for
// kHaltHoldTicks, the strides to 0 in kHaltTicks, then each leg further than
// kHaltSquareMm from home steps there, the body levels, done. A gait change
// goes through home (the legs step home, then the new gait starts).
//
// Fixed-size state, no heap. Copyable (the walker keeps copies for the quick stop).
#ifndef KILO_MIND_ENGINE_SOLVE_H
#define KILO_MIND_ENGINE_SOLVE_H
#include <stdint.h>
#include "engine/clock.h"
#include "engine/gait.h"
#include "planner/model.h"

namespace kilo {
namespace engine {

using planner::Attitude;
using planner::BodyModel;

constexpr int kHorizon = 40;   // ticks of look-ahead for the weight (0.8 s)
constexpr int kSweeps = 6;     // sweeps of the body path a tick
constexpr real kXMinMm = -110, kXMaxMm = 110;  // POSE's feet: x within +-120 mm, z 15..110 mm
constexpr real kZMinMm = 20, kZMaxMm = 105;
constexpr real kAttMaxDeg = 30;  // POSE pitch / roll
constexpr real kHeightSlewMm = (real)0.5;
constexpr real kRecentredMm = (real)1.5;   // a stopping leg this close to home does not step again
constexpr real kCentreSlewMm = (real)0.5;
constexpr real kStanceSlew = (real)0.02;
constexpr int kHaltTicks = 6;
constexpr int kHaltHoldTicks = 10;
constexpr real kHaltSquareMm = (real)5.0;

struct TickPlan {
  uint32_t tick;      // unwrapped body tick
  int16_t xq[kLegs];  // foot x, quarter mm, as POSE carries it
  int16_t zq[kLegs];
  uint8_t support;    // bit i: leg i on the ground
  uint8_t lifting;    // bit i: leg i in the air
  uint8_t n_lifting;
  real pitch;         // predicted at this tick (the servo lag included), degrees
  real roll;
  real plan_pitch;    // the attitude these feet give (no lag)
  real plan_roll;
  real margin;        // mm of the centre of mass inside the support polygon
  real lean_dx;       // the body's shift over its feet this tick, mm forward
  real lean_roll;     // ...and its roll, degrees
};

struct SolverConfig {
  real dx_max_mm = 18;      // the body's fore-aft shift over its feet
  real dx_step_mm = 2;      // ...and its fastest change a tick
  real roll_step_deg = (real)0.3;
  real smooth = 40;         // the cost of a change from tick to tick, against the cost of the shift itself
  real smooth2 = 200;       // ...and of a change of that change
  real lean_tol_mm = (real)1.0;  // a lift waits while the margin it leaves is this much short
  real gate_deg = 5;        // no lift while the body is off the plan by more than this
  int hold_max = 25;        // ...for at most this many ticks
  real catch_deg = 7;       // a swinging leg lands at once when the body tips toward it this far
  bool lean = true;         // false: no weight target (the feet's path only)
};

struct SolverStats {
  uint32_t holds;    // ticks the gate held a lift
  uint32_t catches;
  uint32_t waits;    // ticks a lift waited for the weight
  real min_margin;   // inf until a tick on three feet
};

class Solver {
 public:
  explicit Solver(const BodyModel& model = BodyModel(), const SolverConfig& cfg = SolverConfig());

  // A new walk from four feet at home. speed < 0: the gait's default; start_height < 0: STANCE's.
  void start(const Gait& g, Direction d, real speed_mm_s, uint32_t tick, real start_height = -1, real steer = 0,
             real cadence = 0);
  void set_speed(real speed_mm_s);
  void set_direction(Direction d);
  void set_steer(real steer);
  void set_cadence(real cadence);  // 0: the gait's own; else kCadenceMin..kCadenceMax of it
  void set_gait(const Gait& g);    // through home; the walking gait cancels a change under way
  void stop();
  void halt();
  // The body's latest error: measured - predicted - the steady offset (degrees).
  void observe(real err_pitch, real err_roll) {
    err_[0] = err_pitch;
    err_[1] = err_roll;
  }
  const TickPlan& next();

  Timing timing() const { return engine::timing(*gait_, speed_mm_s_, direction_, cadence_); }
  const Gait& gait() const { return *gait_; }
  const Gait* pending_gait() const { return pending_; }
  bool changing() const { return pending_ != nullptr; }
  bool trot() const { return is_trot(*gait_); }
  bool stopping() const { return stopping_; }
  bool halting() const { return halting_; }
  int halt_ticks() const { return halt_t_; }
  bool done() const { return done_; }
  uint32_t tick() const { return tick_; }
  Direction direction() const { return direction_; }
  real steer() const { return steer_; }
  real speed_mm_s() const { return speed_mm_s_; }
  real cadence() const { return cadence_; }
  const SolverStats& stats() const { return stats_; }
  const TickPlan& last() const { return last_; }
  bool has_last() const { return has_last_; }
  const BodyModel& model() const { return model_; }
  const SolverConfig& config() const { return cfg_; }
  const StepClock& clock() const { return k_.clock; }
  int swing_ticks() const { return trot() ? lw_ : w_tgt_; }  // the trot: its last step's; else the speed's
  int leg_swing_ticks(int leg) const { return k_.w[leg]; }
  bool swinging(int leg) const { return k_.j[leg] > 0; }
  const real* strides() const { return k_.s; }
  real height() const { return k_.h; }
  real foot_x(int leg) const { return k_.x[leg]; }

 private:
  // What one tick of the commands, the clock and the legs needs (the look-ahead runs it too).
  struct Ctx {
    real s_tgt[2];
    real slew;
    real stance_tgt, h_tgt, centre_tgt;
    int w_new;        // a swing starting now lasts this many ticks
    real step_h;
    bool trot;
    uint8_t allow;    // the legs that may lift
    bool hold_clock;  // the clock stands, the rest goes on (the quick stop's hold)
    real next_cycle;  // > 0: the clock's cycle after this tick's advance (the trot's next step)
  };
  // The commands, the clock and the legs: everything a tick of kinematics changes.
  struct Kin {
    StepClock clock;
    real s[2] = {};      // the strides now, left and right
    real stance = 1;     // a foot's time on the ground now, ticks
    real h = 0;
    real centre = 0;
    real x[kLegs] = {};  // foot x in the gait frame, mm
    int j[kLegs] = {};   // 1..w while it swings, 0 on the ground
    int w[kLegs] = {};
    real x0[kLegs] = {};
    real hl[kLegs] = {};
    bool caught[kLegs] = {};
    int caught_t[kLegs] = {};
    real caught_lift[kLegs] = {};
    real lift[kLegs] = {};  // this tick's
    real land[2] = {}, slot_s[2] = {};  // the trot: this step's landing per side, the stride it was for
    void step(const Ctx& c, bool wait);
    real trot_land(int leg) const;
    uint8_t swinging() const;
  };
  static constexpr int kStartTicks = 3;  // a walk's first lift this many ticks after its start
  static constexpr int kHistMax = 32;

  void apply_timing();
  void begin_gait();
  Ctx context() const;
  real height_target() const;
  real centre_target() const;
  real land_for(int side) const;
  bool home() const;
  bool homing() const { return stopping_ || pending_ != nullptr; }
  bool leg_home(int leg) const;
  int trot_swing(uint8_t legs);
  bool swing_ok(int leg, real to, int w) const;
  void solve_lean(const Ctx& now, bool advanced);
  real lift_margin(const Ctx& c) const;
  int edge_margins(const real xg[kLegs], const real lift[kLegs], uint8_t support, real h, real dx, real roll,
                   real out[kLegs]) const;
  real tip_toward(int leg) const;
  void lagged(real* pitch, real* roll) const;

  BodyModel model_;
  SolverConfig cfg_;
  const Gait* gait_;
  const Gait* pending_ = nullptr;
  Kin k_;
  Direction direction_ = Direction::Forward;
  real steer_ = 0;
  real speed_mm_s_ = 0;
  real cadence_ = 0;
  uint32_t tick_ = 0;
  real s_tgt_[2] = {};
  real stance_tgt_ = 1;
  int w_tgt_ = 10;      // the gait's swing at the speed
  int lw_ = kTrotWMin;  // the trot: the last pair's swing (the build-up)
  bool halt_kept_[kLegs] = {};
  // the weight's path: (shift, roll) for this tick and the next kHorizon - 1
  real path_dx_[kHorizon] = {};
  real path_roll_[kHorizon] = {};
  real dx_ = 0, roll_ = 0;            // the body pose sent this tick
  real dx_prev_ = 0, roll_prev_ = 0;  // ...and the tick before
  int held_ = 0;
  bool stopping_ = false, halting_ = false, finishing_ = false, done_ = false;
  int halt_t_ = 0;
  real halt_slew_ = 0;
  int finish_t_ = 0, finish_n_ = 0;
  real finish_dx_ = 0, finish_roll_ = 0;
  real err_[2] = {};
  Attitude prev_att_ = {0, 0};
  real lag_ = 0, alpha_ = 1;
  int settle_ = 0;
  Attitude resp_ = {0, 0};
  int hist_n_ = 0;
  Attitude att_hist_[kHistMax] = {};
  TickPlan last_ = {};
  bool has_last_ = false;
  SolverStats stats_ = {};
  real lead_[kLegs] = {};  // each leg's servo lead, ticks ([model.correction] servo_lag_d_ms)
  real prev_x_[kLegs] = {}, prev_z_[kLegs] = {};  // the last tick's planned feet
  bool has_prev_feet_ = false;
};

}  // namespace engine
}  // namespace kilo

#endif
