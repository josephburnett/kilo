// model: Kilo's geometry and masses, and the attitude and support-polygon
// arithmetic on them. A port of mind/kilo/planner/model.py; its docstring
// has the frames and conventions, kept here in short:
//
// - Body frame: origin midway between the four shoulder axles, x forward,
//   y LEFT, z UP, mm. Legs in protocol order LF RF RB LB.
// - A foot as FEET gives it: x fore-aft from under its shoulder (+ forward),
//   z its depth below the shoulder, to the centre of the foot disc; the
//   ground is foot_r_mm lower.
// - Attitude as TELEMETRY reports it: pitch nose DOWN +, roll RIGHT side up
//   +. Body to world: R = Ry(pitch) Rx(-roll).
// - With every support foot on flat ground,
//     z_i + r = H - a X_i - b Y_i,  a = tan(pitch) / cos(roll),  b = tan(roll).
//
// The masses and the servo lag are estimates until Kilo is weighed
// (docs/walking-and-balance.md §9, §10): change them in model.py and here.
#ifndef KILO_MIND_PLANNER_MODEL_H
#define KILO_MIND_PLANNER_MODEL_H
#include <stdint.h>
#include "planner/fmath.h"

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
  // geometry: measured (params LEG_L1_MM, LEG_L2_MM, LEG_LB_MM, LEG_W_MM)
  real l1_mm = 45;
  real l2_mm = 50;
  real foot_r_mm = 6;
  real length_mm = 123;  // front to back shoulder axle
  real track_mm = 65;    // left to right foot centre
  // masses: ESTIMATES
  Mass body = {700, 5, 0, 5};
  Mass battery = {150, 0, 0, -15};
  Mass pi = {0, -10, 0, 25};  // not carried yet
  real upper_leg_g = 20;
  real upper_leg_at = (real)0.8;
  real lower_leg_g = 8;
  real lower_leg_at = (real)0.5;
  // servos: ESTIMATES
  real servo_lag_ms = 150;
  real slew_deg_per_tick = 10;
  real reach_frac = (real)0.95;
  real knee_max_deg = 150;

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
  real lag_ticks(real tick_ms = 20) const { return servo_lag_ms / tick_ms; }
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
