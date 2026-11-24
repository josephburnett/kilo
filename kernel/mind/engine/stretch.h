// stretch: the random mode's stretch as a goal (docs/movement-engine.md, step 4).
//
// The original Nybble's "str" posture, as wishes on the body model, made each
// time it is asked for at the tempo and depth asked: from STAND (or from SIT,
// through the engine's own posture moves, engine/posture.h, up and back down)
// 1. down into the stretch: the front feet forward along the floor until the
//    legs are straight out in front (kFrontReachMm ahead of the shoulders,
//    kFrontDropMm below them: the chest low), the hind legs at their standing
//    depth with the feet kHindDx forward under him (the rear stays high),
//    the head up (kStretchTilt), the tail low; `depth` < 1 goes only that
//    share of the way (the front feet stop on the straight line there);
// 2. held, the last part of the hold the shake: an overlay on the held pose
//    (Shake): both hind feet amp_mm forward and back together, the tail with
//    them, `cycles` swings (the owner, 2026-11-22: "make him shake a little
//    before coming back up");
// 3. back to STAND (or SIT) no quicker than the body's own posture change.
// The joints come from the model's leg (IK, BodyModel::ik); the rest finder
// (engine/rest.h) follows the whole trick and refuses it when the centre of
// mass comes within kStretchMarginMm of the edge of what carries him or
// something but the feet and knees (the haunches from SIT) touches the floor.
// The head's clearance from the front legs (at least 10 mm) and the
// dynamics are checked on the laptop (mind/tests/test_engine_tricks.py).
#ifndef KILO_MIND_ENGINE_STRETCH_H
#define KILO_MIND_ENGINE_STRETCH_H
#include <stdint.h>
#include "engine/posture.h"
#include "tricks/model.h"

namespace kilo {
namespace engine {

// A small oscillation put on a held pose: both hind feet amp_mm forward and back (the rear back and
// forth over them), the tail tail_deg to each side in step; cycles full swings of 2 x half_ticks
// ticks, eased in and out over half a swing (kilo.tricks.alive_set's shake before step 4).
struct Shake {
  real amp_mm = 4;
  int half_ticks = 5;   // 100 ms: the servos (120 ms late, 70 ms to close 63 %) move the feet about half of it
  int cycles = 3;
  real tail_deg = 20;
};
// The shake's keyframes on `pose` (its hind feet moved by IK), ending on it: 2 x cycles + 1 frames into
// out (room for them), the count returned; 0 when a foot falls outside the leg's reach.
int shake_frames(const BodyModel& m, const tricks::Keyframe& pose, const Shake& s, real tempo,
                 tricks::Keyframe* out);

constexpr real kFrontReachMm = 90;    // the front feet this far ahead of their shoulders (straight out: "str")
constexpr real kFrontDropMm = 2;      // ...and this far below them
constexpr real kHindDxMm = 32;        // the hind feet this far forward of their shoulders, at standing depth
constexpr int kStretchPan = 10, kStretchTilt = 45, kStretchTail = -30;  // the head up (the original's)
constexpr real kStretchMarginMm = 15; // the centre of mass at least this far inside the feet all along
constexpr int kStretchDownTicks = 20; // the original's posture rate from STAND
constexpr int kStretchHoldTicks = 100; // the original holds a posture skill this long (the shake its end)
constexpr int kStretchBackTicks = 25; // back, the body's own posture change
constexpr real kStretchTempoMin = (real)0.5, kStretchTempoMax = 2;
constexpr real kStretchDepthMin = (real)0.5;

struct StretchGoal {
  uint8_t from = KILO_POSTURE_STAND;  // STAND or SIT: where it starts and ends
  real tempo = 1;                     // kStretchTempoMin..Max
  real depth = 1;                     // kStretchDepthMin..1: how far down the front goes
  Shake shake;
};

class Stretch {
 public:
  bool build(const BodyModel& m, const StretchGoal& g);
  const tricks::Trick& trick() const { return trick_; }
  const char* why() const { return why_; }
  real margin() const { return margin_; }       // the least margin along it, mm (the rest finder)
  real pitch() const { return pitch_; }         // the stretch's nose-down pitch, degrees
  int shake_from() const { return shake_from_; }  // the frame the shake starts at

 private:
  tricks::Keyframe frames_[tricks::kMaxFrames] = {};
  tricks::Trick trick_ = {};
  PostureMove posture_;
  const char* why_ = nullptr;
  real margin_ = 0, pitch_ = 0;
  int shake_from_ = 0;
};

}  // namespace engine
}  // namespace kilo

#endif
