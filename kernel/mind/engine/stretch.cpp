#include "engine/stretch.h"

namespace kilo {
namespace engine {

using planner::clamp;
using planner::degrees;
using planner::kFront;
using planner::kLegs;
using planner::m_max;
using planner::m_min;
using planner::radians;
using planner::round_i;

namespace {

constexpr int kRB = planner::RB, kLB = planner::LB;
constexpr real kDownRate = (real)105 / kStretchDownTicks;  // the original's pace down (the front shoulders' 105 deg)
constexpr real kBackRate = (real)105 / kStretchBackTicks;  // ...and back up
constexpr real kStretchRollMaxDeg = 15;  // the player stops it past this roll (he stands on four feet)

real sigma(int leg) { return kFront[leg] ? (real)1.0 : (real)-1.0; }

// A leg's foot (x forward of its shoulder, z below it, mm) from its two joint angles (OpenCat degrees).
void foot_of(const BodyModel& m, int leg, real sh, real kn, real* x, real* z) {
  m.fk(radians(-sh), radians(sigma(leg) * 90 - kn), x, z);
}

// ...and back (paw.cpp's angles()); false outside the leg's reach.
bool angles_of(const BodyModel& m, int leg, real x, real z, int* sh, int* kn) {
  real t1, t2;
  bool ok = m.reachable(x, z);
  m.ik(leg, x, z, &t1, &t2);
  *sh = round_i(-degrees(t1));
  *kn = round_i(-(degrees(t2) - sigma(leg) * 90));
  return ok;
}

Joints joints_of(const tricks::Keyframe& f) {
  Joints j;
  for (int k = 0; k < kJoints; k++) j.a[k] = (real)f.ang[k];
  return j;
}

}  // namespace

int shake_frames(const BodyModel& m, const tricks::Keyframe& pose, const Shake& s, real tempo,
                 tricks::Keyframe* out) {
  int half = round_i((real)s.half_ticks / m_max(tempo, (real)0.1));
  if (half < 4) half = 4;  // no quicker than the servos can follow at all
  int edge = (half + 1) / 2;
  tricks::Keyframe side[2];
  for (int i = 0; i < 2; i++) {
    real dir = i == 0 ? (real)1 : (real)-1;  // the hind feet forward (the rear back over them), the tail one way
    side[i] = pose;
    for (int h = 0; h < 2; h++) {
      int leg = h == 0 ? kRB : kLB;
      real x, z;
      foot_of(m, leg, pose.ang[shoulder(leg)], pose.ang[knee(leg)], &x, &z);
      int sh, kn;
      if (!angles_of(m, leg, x + dir * s.amp_mm, z, &sh, &kn)) return 0;
      side[i].ang[shoulder(leg)] = (int8_t)sh;
      side[i].ang[knee(leg)] = (int8_t)kn;
    }
    side[i].ang[kTailJ] = (int8_t)clamp((real)(pose.ang[kTailJ] + round_i(dir * s.tail_deg)), -120, 120);
    side[i].hold = 0;
    side[i].flags = tricks::kCosine;
  }
  int n = 0;
  out[n] = side[0];
  out[n++].dur = (uint8_t)edge;
  for (int i = 1; i < 2 * s.cycles; i++) {
    out[n] = side[i % 2];
    out[n++].dur = (uint8_t)half;
  }
  out[n] = pose;
  out[n].hold = 0;
  out[n].flags = tricks::kCosine;
  out[n++].dur = (uint8_t)edge;
  return n;
}

bool Stretch::build(const BodyModel& m, const StretchGoal& g) {
  why_ = nullptr;
  real tempo = clamp(g.tempo, kStretchTempoMin, kStretchTempoMax);
  real depth = clamp(g.depth, kStretchDepthMin, 1);
  Shake sh = g.shake;
  sh.cycles = sh.cycles < 1 ? 1 : sh.cycles > 4 ? 4 : sh.cycles;
  sh.amp_mm = clamp(sh.amp_mm, 0, 6);
  sh.tail_deg = clamp(sh.tail_deg, 0, 30);
  Joints base, stand;
  if ((g.from != KILO_POSTURE_STAND && g.from != KILO_POSTURE_SIT) || !posture_joints(g.from, &base) ||
      !posture_joints(KILO_POSTURE_STAND, &stand)) {
    why_ = "the stretch starts from STAND or SIT";
    return false;
  }
  // the stretch: from STAND's feet, the front ones `depth` of the way to straight out in front, the hind
  // ones drawn forward at standing depth; the head up, the tail low
  Joints pose = stand;
  for (int leg = 0; leg < kLegs; leg++) {
    real x, z;
    foot_of(m, leg, stand.a[shoulder(leg)], stand.a[knee(leg)], &x, &z);
    real tx = kFront[leg] ? kFrontReachMm : kHindDxMm, tz = kFront[leg] ? kFrontDropMm : z;
    int a, k;
    if (!angles_of(m, leg, x + (tx - x) * depth, z + (tz - z) * depth, &a, &k)) {
      why_ = "the stretch is outside a leg's reach";
      return false;
    }
    pose.a[shoulder(leg)] = (real)a;
    pose.a[knee(leg)] = (real)k;
  }
  pose.a[kPan] = kStretchPan;
  pose.a[kTilt] = kStretchTilt;
  pose.a[kTailJ] = kStretchTail;
  // down there, held, the shake, back: the ways from the posture planner (from SIT: up first), at the
  // original's pace
  int n = 0;
  PostureGoal pg;
  pg.posture = g.from;
  pg.tempo = clamp(tempo, kPostureTempoMin, kPostureTempoMax);
  pg.rate = kDownRate * tempo / pg.tempo;  // (the posture planner's tempo range is narrower: the rest in the pace)
  if (!posture_.build_to(m, base, pose, pg)) {
    why_ = posture_.why();
    return false;
  }
  for (int i = 0; i < posture_.trick().n_frames; i++) frames_[n++] = posture_.trick().frames[i];
  tricks::Keyframe held = frames_[n - 1];
  int half = round_i((real)sh.half_ticks / tempo);
  if (half < 4) half = 4;  // (as shake_frames times it)
  int shake_ticks = (2 * sh.cycles - 1) * half + 2 * ((half + 1) / 2);
  int still = round_i((real)kStretchHoldTicks / tempo) - shake_ticks;
  frames_[n - 1].hold = (uint8_t)clamp((real)still, 10, 255);
  shake_from_ = n;
  int ns = shake_frames(m, held, sh, tempo, frames_ + n);
  if (ns == 0) {
    why_ = "the shake is outside a hind leg's reach";
    return false;
  }
  n += ns;
  pg.rate = kBackRate * tempo / pg.tempo;
  if (!posture_.build_to(m, pose, base, pg)) {
    why_ = posture_.why();
    return false;
  }
  if (n + posture_.trick().n_frames > tricks::kMaxFrames) {
    why_ = "too many frames";
    return false;
  }
  for (int i = 0; i < posture_.trick().n_frames; i++) {
    frames_[n] = posture_.trick().frames[i];
    // back no quicker than the body's own posture change
    if (frames_[n].dur < kStretchBackTicks) frames_[n].dur = (uint8_t)kStretchBackTicks;
    n++;
  }
  // the whole of it in the rest finder: on his feet (and knees, haunches, tail) with room
  Rest cur = rest(m, Servos::Reseated, base);
  cur.drop = 0;
  Along al;
  al.add(cur);
  Joints prev = base;
  real pitch = 0;
  for (int i = 0; i < n; i++) {
    Joints j = joints_of(frames_[i]);
    follow(m, Servos::Reseated, prev, j, &cur, &al);
    if (i == shake_from_ - 1) pitch = cur.pitch;
    prev = j;
  }
  margin_ = al.margin;
  pitch_ = -pitch;
  const uint32_t allowed = kFeet | kKnees | kHaunches | kTail;
  if ((al.contacts & ~allowed) != 0 || al.drop > kDropMaxMm || margin_ < kStretchMarginMm) {
    why_ = "the stretch comes too close to the edge of his feet, or puts something else on the floor";
    return false;
  }
  trick_ = {};
  trick_.name = g.from == KILO_POSTURE_SIT ? "stretch_sit" : "stretch_stand";
  trick_.legacy = "";
  trick_.n_frames = (uint8_t)n;
  trick_.loop_start = 0;
  trick_.loop_end = 0;
  trick_.repeats = 0;
  trick_.joint_mask = 0;
  for (int k = 0; k < kJoints; k++) trick_.joint_mask = (uint16_t)(trick_.joint_mask | (1u << tricks::kCanonical[k]));
  trick_.flags = 0;
  trick_.end_posture = g.from;
  trick_.exp_roll = 0;
  trick_.exp_pitch = 0;
  trick_.frames = frames_;
  trick_.stop_ticks = (uint8_t)kStretchBackTicks;
  trick_.guards = nullptr;
  trick_.n_guards = 0;
  trick_.roll_max_a16 = (int16_t)round_i(kStretchRollMaxDeg * 16);
  const char* p = tricks::trick_problem(trick_);
  if (p != nullptr) {
    why_ = p;
    return false;
  }
  return true;
}

}  // namespace engine
}  // namespace kilo
