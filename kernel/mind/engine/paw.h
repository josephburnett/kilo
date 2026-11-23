// paw: the first trick as a goal (docs/movement-engine.md, step 4; the owner's
// decision 4): lift a front paw on three feet and wash the face with it.
//
// Not a recording: the trick is worked out from the body model each time it
// is asked for, at the tempo and the number of strokes asked, from STAND:
// 1. the weight onto the three feet that stay down: the solver's weight
//    target (the centre of mass kMarginMm inside the triangle of the other
//    front foot and the two hind feet, with the paw already where it will be
//    lifted), by the smallest body shift fore and aft and roll that does it;
// 2. the paw off the floor where it stood (kPawOffMm), then up beside the
//    muzzle (kPawX, kPawZ from its shoulder) and
//    the head tilted down toward it, then a hold the player's guard watches:
//    if he does not stand still on three feet (the roll or the pitch off
//    what the lean expects), the trick stops and he goes back to STAND;
// 3. the strokes: the paw down the face and back up, the head nodding with it,
//    `strokes` times at the tempo (a loop of the trick);
// 4. the paw down where it was, the weight back, STAND.
// The left paw; the player mirrors it for the right (TrickPlayer::start).
// The head's clearance from the paw (at least 10 mm, either way the pan
// turns) and the statics along the whole trick are checked on the laptop
// against the body model (mind/tests/test_engine_tricks.py, the checks of
// kilo.tricks.alive_set); no balance correction on three feet (the board's
// joint-space balance is off for these frames).
#ifndef KILO_MIND_ENGINE_PAW_H
#define KILO_MIND_ENGINE_PAW_H
#include <stdint.h>
#include "engine/gait.h"
#include "planner/model.h"
#include "tricks/model.h"

namespace kilo {
namespace engine {

constexpr real kPawMarginMm = 7;    // the weight inside the three feet while the paw is up (held, not walked: twice the walk's 3.5)
// The paw beside the muzzle (it cannot come closer than its leg's plane, 32 mm to the side of the
// head's middle: about 16 mm from the muzzle with the head straight, by the model's head; a
// search over the paw's point and the head's tilt in kilo.tricks.alive_set's head model).
constexpr real kPawX = 43;         // the paw beside the muzzle: mm ahead of its shoulder ...
constexpr real kPawZ = 13;         // ... and below it (the foot's centre): about as high as the cheek
constexpr real kStrokeMm = 8;      // a stroke takes it this much down the face, and back up
constexpr real kPawOffMm = 10;     // first the paw just off the floor where it stood (the weight leaves it slowly)
constexpr int kHeadTilt = -35;     // the head tilted down to meet it (degrees; + lifts the nose)
constexpr int kNodDeg = -6;        // the head follows the paw down each stroke
constexpr real kPawTempoMin = (real)0.5;
constexpr real kPawTempoMax = 2;
constexpr int kPawStrokesMax = 8;

struct PawGoal {
  real tempo = 1;   // 0.5..2: the trick's moves this many times as fast
  int strokes = 3;  // 1..kPawStrokesMax
};

class PawWash {
 public:
  // The trick for the left paw; false (why()) when no lean within the limits keeps the weight
  // kPawMarginMm inside the three feet, or a pose falls outside a joint's limits.
  bool build(const planner::BodyModel& m, const PawGoal& g);
  const tricks::Trick& trick() const { return trick_; }
  const char* why() const { return why_; }
  real lean_dx() const { return dx_; }      // the body's shift over its feet, mm forward
  real lean_roll() const { return roll_; }  // degrees, right side up +
  real margin() const { return margin_; }   // the least margin with the paw up (and stroking), mm

 private:
  tricks::Keyframe frames_[8] = {};
  tricks::TrickGuard guard_ = {};
  tricks::Trick trick_ = {};
  const char* why_ = nullptr;
  real dx_ = 0, roll_ = 0, margin_ = 0;
};

}  // namespace engine
}  // namespace kilo

#endif
