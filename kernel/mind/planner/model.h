// model: Kilo's geometry and masses, and the attitude and support-polygon
// arithmetic on them. A port of mind/kilo/planner/model.py; its docstring
// has the frames and conventions, kept here in short:
//
// - Body frame: origin midway between the four shoulder axles, x forward,
//   y LEFT, z UP, mm. Legs in protocol order LF RF RB LB.
// - A foot as a POSE gives it (in mm): x fore-aft from under its shoulder (+ forward),
//   z its depth below the shoulder, to the centre of the foot disc; the
//   ground is foot_r_mm lower.
// - Attitude as TELEMETRY reports it: pitch nose DOWN +, roll RIGHT side up
//   +. Body to world: R = Ry(pitch) Rx(-roll).
// - With every support foot on flat ground,
//     z_i + r = H - a X_i - b Y_i,  a = tan(pitch) / cos(roll),  b = tan(roll).
//
// The numbers are protocol/spec.toml's ([model] and the LEG_* params), from
// the generated header, as model.py's: change them there and regenerate.
#ifndef KILO_MIND_PLANNER_MODEL_H
#define KILO_MIND_PLANNER_MODEL_H
#include <stdint.h>
#include "planner/fmath.h"
#include "proto/kilo_protocol.h"

namespace kilo {
namespace planner {

constexpr int LF = 0, RF = 1, RB = 2, LB = 3;
constexpr int kLegs = 4;
constexpr bool kFront[kLegs] = {true, true, false, false};
constexpr bool kLeft[kLegs] = {true, false, false, true};
constexpr real kGmmS2 = (real)9810.0;

struct Mass {
  real grams, x, y, z;  // a lumped mass at a point of the body frame
};

struct V2 {
  real x, y;
};
struct V3 {
  real x, y, z;
};
struct Attitude {
  real pitch, roll;  // degrees
};

struct BodyModel {
  // The defaults are protocol/spec.toml's (the LEG_* and SLEW_MAX_A16 params,
  // [model]), as model.py's: geometry, measured
  real l1_mm = KILO_PARAM_LEG_L1_MM_DEFAULT;
  real l2_mm = KILO_PARAM_LEG_L2_MM_DEFAULT;
  real foot_r_mm = KILO_MODEL_FOOT_R_MM;
  real length_mm = KILO_PARAM_LEG_LB_MM_DEFAULT;  // front to back shoulder axle
  real track_mm = KILO_PARAM_LEG_W_MM_DEFAULT;    // left to right foot centre
  // masses: a fit to the centre of gravity measured 2026-11-14 at STAND
  real stand_com_x_mm = (real)KILO_MODEL_STAND_COM_X_MM;  // ahead of the origin
  Mass body = KILO_MODEL_BODY_MASS_INIT;
  Mass battery = KILO_MODEL_BATTERY_MASS_INIT;
  Mass pi = KILO_MODEL_PI_MASS_INIT;
  real upper_leg_g = KILO_MODEL_UPPER_LEG_G;
  real upper_leg_at = (real)KILO_MODEL_UPPER_LEG_AT;
  real lower_leg_g = KILO_MODEL_LOWER_LEG_G;
  real lower_leg_at = (real)KILO_MODEL_LOWER_LEG_AT;
  // servos: fitted to the floor recordings of 2026-11-15: a dead time, then a
  // first-order response
  real servo_lag_ms = KILO_MODEL_SERVO_LAG_MS;
  real servo_tau_ms = KILO_MODEL_SERVO_TAU_MS;
  real slew_deg_per_tick = (real)KILO_PARAM_SLEW_MAX_A16_DEFAULT / 16;
  real reach_frac = (real)KILO_MODEL_REACH_FRAC;
  real knee_max_deg = KILO_MODEL_KNEE_MAX_DEG;
  // corrections for this robot (spec [model.correction]): what a self-measurement finds, all
  // zero until one is applied (kilo model apply); com() adds them. model.py has the others
  // (the legs' give, the legs' lengths), which nothing here uses.
  real com_dx_mm = (real)KILO_MODEL_CORRECTION_COM_DX_MM;  // the whole robot's centre of mass, forward
  real com_dy_mm = (real)KILO_MODEL_CORRECTION_COM_DY_MM;  // ...to the left
  real com_dz_mm = (real)KILO_MODEL_CORRECTION_COM_DZ_MM;  // ...up
  // what TELEMETRY reads when he stands level: pitch nose down +, roll right side up + (the rest
  // finder's attitudes as TELEMETRY shows them, engine/rest.h)
  real imu_pitch_deg = (real)KILO_MODEL_CORRECTION_IMU_PITCH_DEG;
  real imu_roll_deg = (real)KILO_MODEL_CORRECTION_IMU_ROLL_DEG;
  // the tail (spec [model.tail]) and the frame's outline ([model.outline], ESTIMATES): what may touch
  // the floor sitting and lying, for the rest finder (engine/rest.h)
  struct Tail {
    real pivot_x = (real)KILO_MODEL_TAIL_PIVOT_X_MM, pivot_z = (real)KILO_MODEL_TAIL_PIVOT_Z_MM;
    real len = (real)KILO_MODEL_TAIL_LEN_MM, droop_deg = (real)KILO_MODEL_TAIL_DROOP_DEG;
    real axis_deg = (real)KILO_MODEL_TAIL_AXIS_DEG, sign = (real)KILO_MODEL_TAIL_SIGN;
    real g = (real)KILO_MODEL_TAIL_G, com_frac = (real)KILO_MODEL_TAIL_COM_FRAC;
  } tail;
  struct Outline {
    real rear_x = (real)KILO_MODEL_OUTLINE_REAR_X_MM, belly_z = (real)KILO_MODEL_OUTLINE_BELLY_Z_MM;
    real back_z = (real)KILO_MODEL_OUTLINE_BACK_Z_MM, pi_rear_x = (real)KILO_MODEL_OUTLINE_PI_REAR_X_MM;
    real pi_top_z = (real)KILO_MODEL_OUTLINE_PI_TOP_Z_MM, nose_x = (real)KILO_MODEL_OUTLINE_NOSE_X_MM;
    real nose_z = (real)KILO_MODEL_OUTLINE_NOSE_Z_MM, half_width = (real)KILO_MODEL_OUTLINE_HALF_WIDTH_MM;
    real hip_r = (real)KILO_MODEL_OUTLINE_HIP_R_MM;
  } outline;
  // what the body does with a joint angle (spec [model.servo], body/src/core/joints.cpp): servo =
  // czp + dir * angle, pulse = a2p / 1000 * servo + b us, clamped to pulse_lo..pulse_hi
  real czp[16] = KILO_MODEL_SERVO_CZP_INIT;
  real servo_dir[16] = KILO_MODEL_SERVO_DIR_INIT;
  real a2p_x1000 = KILO_MODEL_SERVO_A2P_X1000;
  real b_us = KILO_MODEL_SERVO_B_US;
  real pulse_us[2] = KILO_MODEL_SERVO_PULSE_US_INIT;
  // The angles joint `id` (legacy id 0..15) really reaches with servo zero `zero` (czp[id]: Kilo's):
  // its limits and its pulse window (kilo.tricks.stance reach()). A joint the body does not drive: its
  // limits.
  void reach(int id, real zero, real lo_limit, real hi_limit, real* lo, real* hi) const;
  // each servo's own performance, by legacy joint id 0..15 (model.py; none measured yet: all as the robot's)
  real servo_lag_d_ms[16] = KILO_MODEL_CORRECTION_SERVO_LAG_D_MS_INIT;  // later than servo_lag_ms
  real servo_tau_d_ms[16] = KILO_MODEL_CORRECTION_SERVO_TAU_D_MS_INIT;  // slower than servo_tau_ms

  real total_g() const;
  V2 hip(int leg) const;  // shoulder axle (x, y) in the body frame
  real reach_mm() const { return reach_frac * (l1_mm + l2_mm); }
  real min_reach_mm() const;
  real stance_height(int h_256) const { return (real)h_256 / (real)256.0 * (l1_mm + l2_mm); }
  // rad/s^2 a side (or end) of the body can drop at under gravity, pivoting
  // on the far feet lever_mm away from the mass.
  real drop_accel(real lever_mm, real com_height_mm) const {
    return kGmmS2 * lever_mm / (com_height_mm * com_height_mm + lever_mm * lever_mm);
  }
  real lag_ticks(real tick_ms = 20) const { return servo_lag_ms / tick_ms; }  // the dead time
  // One servo's (legacy joint id 0..15): the robot's plus its own difference (model.py joint_lag_ms).
  real joint_lag_ms(int joint) const { return servo_lag_ms + servo_lag_d_ms[joint & 15]; }
  real joint_tau_ms(int joint) const { return servo_tau_ms + servo_tau_d_ms[joint & 15]; }
  // Ticks until a commanded move has nearly arrived (dead time + 1.5 time constants).
  int settle_ticks(real tick_ms = 20) const {
    real t = (servo_lag_ms + (real)1.5 * servo_tau_ms) / tick_ms - (real)1e-9;
    int i = (int)t;
    return (real)i < t ? i + 1 : i;
  }
  // The first-order response's step per tick: y += (u - y) * alpha.
  real response_alpha(real tick_ms = 20) const {
    return servo_tau_ms <= 0 ? (real)1.0 : (real)1.0 - m_exp(-tick_ms / servo_tau_ms);
  }
  real tau_s(real com_height_mm) const { return m_sqrt(m_max(com_height_mm, 1) / kGmmS2); }

  bool reachable(real x, real z) const;
  // Foot (x, z) -> (theta1, theta2) radians, the distance clamped to the workspace.
  void ik(int leg, real x, real z, real* t1, real* t2) const;
  void fk(real t1, real t2, real* x, real* z) const;
  // Centre of mass in the body frame for feet (xs, zs).
  V3 com(const real xs[kLegs], const real zs[kLegs]) const;
};

// (a, b) of the support plane for an attitude.
void plane_coeffs(real pitch_deg, real roll_deg, real* a, real* b);
Attitude attitude_of(real a, real b);
// z for every leg so that all four feet stand on flat ground with the body
// at pitch/roll and the legs' mean length height.
void leg_lengths(const BodyModel& m, const real xs[kLegs], real height, real pitch_deg, real roll_deg,
                 real zs[kLegs]);
// The attitude the body takes standing on the feet in support (bit i = leg
// i): least squares of the plane through them, pulled toward prev with
// weight reg (mm^2), which matters only on fewer than three feet. Solved in
// double (on two feet the regularisation is 1e-7 of the matrix: float loses it).
Attitude fit_attitude(const BodyModel& m, const real xs[kLegs], const real zs[kLegs], uint8_t support,
                      Attitude prev = {0, 0}, double reg = 1e-3);
// The symmetric-or-not 3x3 system m x = v; false if |det| < 1e-12.
bool solve3(const double m[3][3], const double v[3], double x[3]);
void rotation(real pitch_deg, real roll_deg, real R[3][3]);
// World (x, y, height) of the centre of mass and of the four feet's ground points.
void world_points(const BodyModel& m, const real xs[kLegs], const real zs[kLegs], real pitch_deg, real roll_deg,
                  V3* com, V3 feet[kLegs]);
// Convex hull (counter-clockwise) of n <= 4 points; returns its size.
int hull(const V2* pts, int n, V2 out[kLegs]);
// Signed distance (mm) of p from the convex polygon's boundary, + inside;
// a segment or a point has no inside (minus the distance to it).
real margin(V2 p, const V2* poly, int n);
// Margin of the centre of mass's projection inside the support feet's polygon.
real stability(const BodyModel& m, const real xs[kLegs], const real zs[kLegs], uint8_t support, real pitch_deg,
               real roll_deg);

}  // namespace planner
}  // namespace kilo

#endif
