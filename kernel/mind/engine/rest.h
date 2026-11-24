// rest: how Kilo's rigid body comes to rest on flat ground for a set of joint
// angles: on which points (the feet, the knees, the haunches, the belly, the
// tail ...), at what pitch and roll, and how far the centre of mass is inside
// what carries him (docs/movement-engine.md, step 3: what the engine needs to
// reason about sitting and lying, where the haunches and the belly may carry
// him, not only the feet).
//
// The C++ port of kilo.tricks.stance (rest(), follow()), which the tricks'
// checks and the state estimate's table use; mind/tests/test_engine_postures.py
// holds the two to each other. Quasi-static and rigid: no inertia, no give of
// the legs, contacts that do not slide. Each leg's joints as the servos really
// reach them (BodyModel::reach: the joint limits and the pulse window of
// Kilo's calibration, which stops some joints short of what is asked). The body
// is set down on its lowest point and toppled about the support edge (or point)
// the centre of mass is outside of, by the angle that brings the next point
// down, until the centre of mass is over the support polygon.
//
// The points, body frame (x forward, y left, z up, mm, origin midway between
// the shoulder axles): the four feet (discs foot_r_mm), knees and shoulder
// housings (hip_r_mm); the tail's tip; and the frame's outline ([model.outline],
// ESTIMATES): the haunches (the underside's rear corners), the belly (its front
// corners), the back's and the Pi's rear corners, the nose, the top.
// Attitude here: pitch NOSE UP +, roll LEFT side down + (stance's); TELEMETRY
// reads telemetry_pitch()/telemetry_roll() (nose down +, with the IMU's
// offsets).
#ifndef KILO_MIND_ENGINE_REST_H
#define KILO_MIND_ENGINE_REST_H
#include <stdint.h>
#include "planner/model.h"
#include "proto/kilo_protocol.h"

namespace kilo {
namespace engine {

using planner::BodyModel;
using planner::real;
using planner::V3;

// Joint angles in degrees, in the tricks' canonical order (tricks::Keyframe ang[]): head pan, head tilt,
// tail, the shoulders LF RF RB LB, the knees LF RF RB LB.
constexpr int kJoints = KILO_JOINT_COUNT;
struct Joints {
  real a[kJoints];
};
constexpr int kPan = 0, kTilt = 1, kTailJ = 2;
inline int shoulder(int leg) { return 3 + leg; }
inline int knee(int leg) { return 7 + leg; }

// The points that may touch the floor (bit i of Rest::contacts).
enum Point : uint8_t {
  kFootLF = 0, kKneeLF = 4, kHipLF = 8,  // + leg (LF RF RB LB)
  kTailTip = 12,
  kHaunchL = 13, kHaunchR = 14,  // the underside's rear corners (rear_x, +-half_width, belly_z)
  kBellyL = 15, kBellyR = 16,    // ...its front corners, under the front axles
  kBackL = 17, kBackR = 18,      // the top of the frame at the rear
  kPiL = 19, kPiR = 20,          // the Pi's rear edge, at its highest
  kNoseL = 21, kNoseR = 22,
  kTopL = 23, kTopR = 24,        // the Pi's top, over the front axles
  kPoints = 25,
};
constexpr uint32_t kFeet = 0xFu;
constexpr uint32_t kKnees = 0xF0u;
constexpr uint32_t kHaunches = (1u << kHaunchL) | (1u << kHaunchR);
constexpr uint32_t kBelly = (1u << kBellyL) | (1u << kBellyR);
constexpr uint32_t kTail = 1u << kTailTip;
// What may carry him in a posture and between postures: the feet, the knees, the haunches, the belly and
// the tail (not his back, the Pi, the nose or a shoulder).
constexpr uint32_t kMayCarry = kFeet | kKnees | kHaunches | kBelly | kTail;
const char* point_name(int p);  // "lf_foot", "haunch_l" (stance's labels; "butt_l" there for a haunch)

// Which calibration the servos' pulse windows come from.
enum class Servos : uint8_t {
  Asked,     // none: every joint reaches what is asked within its limits
  Eeprom,    // BodyModel::czp: Kilo's EEPROM of 2026-10-10 (the right hind shoulder stops at -57 deg)
  Reseated,  // ...with the right hind shoulder re-seated (reseated_rb_shoulder_czp): what the tricks are designed for
};

struct Rest {
  real R[3][3];       // body -> world
  real pitch, roll;   // degrees: nose up +, left side down +
  real margin;        // mm: the centre of mass inside the support polygon (< 0: never at rest)
  uint32_t contacts;  // the points on the floor
  int topples;        // on the way here from the orientation started from
  real drop;          // mm: the furthest a point fell in one of those topples
  real com_h;         // mm: the centre of mass above the floor
};

// The angle joint k (canonical) really goes to when asked for `a` (degrees).
real applied(const BodyModel& m, Servos s, int k, real a);
// The centre of mass, body frame, at joints j (as the servos reach them).
V3 com(const BodyModel& m, Servos s, const Joints& j);
// The rest of pose j, starting from the orientation of `from` (null: level).
Rest rest(const BodyModel& m, Servos s, const Joints& j, const Rest* from = nullptr);
// TELEMETRY's pitch (nose DOWN +) and roll (right side up +) at rest r, degrees.
real telemetry_pitch(const BodyModel& m, const Rest& r);
real telemetry_roll(const BodyModel& m, const Rest& r);

// What a straight move in joint space does (the body's ease along it changes the timing, not the path).
struct Along {
  real margin = 1e9f;    // the least margin
  real drop = 0;         // the largest single topple's drop
  uint32_t contacts = 0; // everything that touched
  real roll_max = 0;     // the largest |roll|
  real pitch_lo = 1e9f, pitch_hi = -1e9f;
  void add(const Rest& r);
};
// From joints a to b, sampled every deg_per_sample degrees of the largest joint change (at least two
// samples), each rest from the one before (*cur in and out: the orientation carried). Adds to *along.
void follow(const BodyModel& m, Servos s, const Joints& a, const Joints& b, Rest* cur, Along* along,
            real deg_per_sample = 2);

}  // namespace engine
}  // namespace kilo

#endif
