#include "engine/paw.h"

namespace kilo {
namespace engine {

using planner::BodyModel;
using planner::clamp;
using planner::degrees;
using planner::leg_lengths;
using planner::m_fabs;
using planner::m_max;
using planner::m_min;
using planner::radians;
using planner::round_i;
using planner::stability;

namespace {

// STAND (the posture's joints, as body/src/core/posture.cpp holds them: shoulders 30, -30, knees 30, -30)
constexpr int kStandShoulder[kLegs] = {30, 30, -30, -30};
constexpr int kStandKnee[kLegs] = {30, 30, -30, -30};
constexpr int kLeanDxMax = 18;     // mm
constexpr real kLeanRollMax = 8;   // degrees
constexpr real kRollCost = 4;      // a degree of roll costs as this many mm of shift (squared)
constexpr int kPostureTicks = 25;  // back to STAND no quicker than the body's own posture change
constexpr uint8_t kSupport = (1u << RF) | (1u << RB) | (1u << LB);  // the left paw up

real sigma(int leg) { return planner::kFront[leg] ? (real)1.0 : (real)-1.0; }

// A foot (x, z) as the leg's two joints (degrees, OpenCat convention: core/ik.h before the per-joint dir).
void angles(const BodyModel& m, int leg, real x, real z, int* sh, int* kn) {
  real t1, t2;
  m.ik(leg, x, z, &t1, &t2);
  *sh = round_i(-degrees(t1));
  *kn = round_i(-(degrees(t2) - sigma(leg) * 90));
}

struct Pose {
  real x[kLegs], z[kLegs];
};

// The four feet with the body shifted dx over them and rolled; the left paw at (px, pz) or down.
Pose pose(const BodyModel& m, const Pose& base, real h, real dx, real roll, bool paw, real px, real pz) {
  Pose p;
  for (int leg = 0; leg < kLegs; leg++) p.x[leg] = base.x[leg] - dx;
  leg_lengths(m, p.x, h, 0, roll, p.z);
  if (paw) {
    p.x[LF] = px;
    p.z[LF] = pz;
  }
  return p;
}

real margin_of(const BodyModel& m, const Pose& p, real roll) { return stability(m, p.x, p.z, kSupport, 0, roll); }

void frame(tricks::Keyframe* f, const BodyModel& m, const Pose& p, int pan, int tilt, int tail, int dur, int hold,
           bool* ok) {
  f->dur = (uint8_t)(dur < 1 ? 1 : dur > 255 ? 255 : dur);
  f->hold = (uint8_t)(hold < 0 ? 0 : hold > 255 ? 255 : hold);
  f->flags = tricks::kCosine;
  f->ang[0] = (int8_t)pan;
  f->ang[1] = (int8_t)tilt;
  f->ang[2] = (int8_t)tail;
  for (int leg = 0; leg < kLegs; leg++) {
    int sh, kn;
    if (!m.reachable(p.x[leg], p.z[leg])) *ok = false;
    angles(m, leg, p.x[leg], p.z[leg], &sh, &kn);
    if (sh < -127 || sh > 127 || kn < -127 || kn > 127) *ok = false;
    f->ang[3 + leg] = (int8_t)clamp((real)sh, -127, 127);
    f->ang[7 + leg] = (int8_t)clamp((real)kn, -127, 127);
  }
}

}  // namespace

bool PawWash::build(const BodyModel& m, const PawGoal& g) {
  why_ = nullptr;
  real tempo = clamp(g.tempo, kPawTempoMin, kPawTempoMax);
  int strokes = g.strokes < 1 ? 1 : g.strokes > kPawStrokesMax ? kPawStrokesMax : g.strokes;
  auto T = [&](int n) { return round_i((real)n / tempo) > 1 ? round_i((real)n / tempo) : 1; };

  // STAND's feet
  Pose base;
  real h = 0;
  for (int leg = 0; leg < kLegs; leg++) {
    real t1 = radians((real)-kStandShoulder[leg]);
    real t2 = radians(sigma(leg) * 90 - (real)kStandKnee[leg]);
    m.fk(t1, t2, &base.x[leg], &base.z[leg]);
    h += base.z[leg] / kLegs;
  }
  // the weight onto the three feet: the least shift and roll (roll costing kRollCost) that keeps
  // the centre of mass kPawMarginMm inside them with the paw up and at the bottom of a stroke
  real dx = 0, roll = 0;
  for (int it = 0; it < 40; it++) {
    real worst = 1e9, g0 = 0, g1 = 0;
    for (int s = 0; s < 2; s++) {
      real pz = kPawZ + (s ? kStrokeMm : 0);
      real m0 = margin_of(m, pose(m, base, h, dx, roll, true, kPawX, pz), roll);
      if (m0 < worst) {
        worst = m0;
        g0 = margin_of(m, pose(m, base, h, dx + (real)0.5, roll, true, kPawX, pz), roll) - m0;
        g1 = margin_of(m, pose(m, base, h, dx, roll + (real)0.5, true, kPawX, pz), roll + (real)0.5) - m0;
        g0 /= (real)0.5;
        g1 /= (real)0.5;
      }
    }
    margin_ = worst;
    if (worst >= kPawMarginMm - (real)0.05) break;
    real nn = g0 * g0 + g1 * g1 / kRollCost;
    if (nn < (real)1e-9) break;
    real l = (kPawMarginMm + (real)0.1 - worst) / nn;
    dx = clamp(dx + g0 * l, -kLeanDxMax, kLeanDxMax);
    roll = clamp(roll + g1 * l / kRollCost, -kLeanRollMax, kLeanRollMax);
  }
  dx_ = dx;
  roll_ = roll;
  if (margin_ < kPawMarginMm - (real)0.5) {
    why_ = "no lean within the limits keeps the weight inside the three feet";
    return false;
  }
  bool ok = true;
  Pose lean = pose(m, base, h, dx, roll, false, 0, 0);
  Pose up = pose(m, base, h, dx, roll, true, kPawX, kPawZ);
  Pose top = pose(m, base, h, dx, roll, true, kPawX, kPawZ + kStrokeMm);
  Pose off = lean;  // the paw just off the floor where it stood: the weight leaves it slowly
  off.z[LF] -= kPawOffMm;
  // the frames: the lean (held), the paw off the floor, up (held: the guard), a stroke down and up (the
  // loop), the paw down, STAND
  // the lean held until the servos have it (their dead time and 1.5 time constants, and two ticks)
  // before the paw leaves the floor, whatever the tempo
  int settle = m.settle_ticks() + 2;
  frame(&frames_[0], m, lean, 0, 0, 0, T(20), T(4) > settle ? T(4) : settle, &ok);
  frame(&frames_[1], m, off, 0, 0, 0, T(16), 0, &ok);
  frame(&frames_[2], m, up, 0, kHeadTilt, 0, T(15), 20, &ok);
  frame(&frames_[3], m, top, 0, kHeadTilt + kNodDeg, 0, T(8), 0, &ok);
  frame(&frames_[4], m, up, 0, kHeadTilt, 0, T(8), 0, &ok);
  frame(&frames_[5], m, lean, 0, 0, 0, T(15), 0, &ok);
  frame(&frames_[6], m, base, 0, 0, 0, T(20) > kPostureTicks ? T(20) : kPostureTicks, 0, &ok);
  if (!ok) {
    why_ = "a pose outside the leg's reach";
    return false;
  }
  // the guard: on three feet, still, the roll and the pitch where the lean puts him (TELEMETRY nose up = -pitch)
  real r = m_fabs(roll);
  guard_ = {2, (int16_t)(-6 * 16), (int16_t)(6 * 16), (int16_t)round_i((r + 6) * 16), "three feet", 0};
  trick_ = {};
  trick_.name = "paw_wash";
  trick_.legacy = "";
  trick_.n_frames = 7;
  trick_.loop_start = 3;
  trick_.loop_end = 4;
  trick_.repeats = (uint8_t)strokes;
  trick_.joint_mask = 0;
  for (int k = 0; k < (int)KILO_JOINT_COUNT; k++) trick_.joint_mask = (uint16_t)(trick_.joint_mask | (1u << tricks::kCanonical[k]));
  trick_.flags = 0;  // no balance: the board's joint-space correction stays off on three feet
  trick_.end_posture = KILO_POSTURE_STAND;
  trick_.exp_roll = (int8_t)round_i(roll);
  trick_.exp_pitch = 0;
  trick_.frames = frames_;
  trick_.stop_ticks = kPostureTicks;
  trick_.guards = &guard_;
  trick_.n_guards = 1;
  trick_.roll_max_a16 = (int16_t)round_i((r + 10) * 16);
  const char* p = tricks::trick_problem(trick_);
  if (p != nullptr) {
    why_ = p;
    return false;
  }
  return true;
}

}  // namespace engine
}  // namespace kilo
