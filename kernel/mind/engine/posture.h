// posture: stand, sit and rest as goals (docs/movement-engine.md, step 3).
//
// Asked for a posture, the engine works out the way there from the joints he
// is in, from the body model, each time: no stored move. The eight leg joints
// are four groups (the front shoulders, the front knees, the hind shoulders,
// the hind knees). A candidate moves them all at once (the body's own way of
// changing posture) or in two or three stages (every order of the groups);
// the head and the tail go with the first stage. Each candidate is followed in
// the rest finder (engine/rest.h: the feet, knees, haunches, belly and tail may
// carry him) with Kilo's servos as they are since his right hind shoulder was
// re-seated, and the one whose centre of mass stays furthest inside what
// carries him at its worst moment is the move (each stage it adds must keep
// kTieMm more, so all at once wins a tie). Nothing else may touch the floor
// (his back, the Pi, the nose, a shoulder), and no topple may drop a point
// more than kDropMaxMm. When no direct way keeps kPostureMarginMm, the way
// through each other posture is tried (sitting from lying: up to standing
// first, then down), and the better is taken.
//
// Tempo: at 1 a stage takes as long as the body's own posture change for its
// largest joint (POSTURE_TICKS for the 75 degrees of STAND to SIT or REST: a
// joint turns at most kRateDegPerTick a tick on average), never under
// kStageMinTicks; tempo t divides that (kPostureTempoMin..Max). Each stage
// moves along the body's cosine ease, and stages kStageHoldTicks apart.
//
// The move is a trick for the player (tricks/player.h): joint angles, no
// balance flag (the board's joint-space balance takes over once the end
// posture is latched, if the goal asks for it: the board is not driven
// through its balance), the end posture the goal's, so the board holds the
// posture from where the move left the joints (the same angles: no move).
// Stopped (a key, a guard), it ends in that posture over kStopTicks. The
// player stops it past kRollMaxDeg of roll.
#ifndef KILO_MIND_ENGINE_POSTURE_H
#define KILO_MIND_ENGINE_POSTURE_H
#include <stdint.h>
#include "engine/rest.h"
#include "tricks/model.h"

namespace kilo {
namespace engine {

// The centre of mass inside what carries him, all along. The floor holds him on three feet with the weight
// 0-3 mm inside them (docs/movement-engine-results.md §2); the best way up from REST keeps 8.0 at its worst
// moment (sampled every 2 degrees; the Python planner's 12 samples a stage had seen 8.7 and set 8.5).
constexpr real kPostureMarginMm = (real)7.5;
constexpr real kTieMm = (real)0.5;            // all at once wins over two stages no more than this better
constexpr real kDropMaxMm = 6;                // the largest single topple (a rock onto a foot, not a fall)
constexpr real kRateDegPerTick = 3;           // tempo 1: the body's own pace (75 deg in POSTURE_TICKS)
constexpr int kStageMinTicks = 10;            // ...a stage never quicker than this at tempo 1
constexpr int kStageHoldTicks = 5;            // between two stages
constexpr int kEndHoldTicks = 5;              // at the end, before the posture is latched
constexpr int kStopTicks = 40;                // stopped: to the end posture over this
constexpr real kRollMaxDeg = 25;              // the player stops the move past this roll
constexpr real kPostureTempoMin = (real)0.5;
constexpr real kPostureTempoMax = (real)1.5;
constexpr int kMaxStages = 6;                 // two moves (through another posture) of three stages

// The joints of a body posture (KILO_POSTURE_STAND, _SIT, _REST): the body's tables inside the joint
// limits (the library's stand, sit and rest, body/src/core/posture.cpp). False for another posture.
bool posture_joints(uint8_t posture, Joints* j);
const char* posture_word(uint8_t posture);  // "stand", "sit", "rest"

struct PostureGoal {
  uint8_t posture = KILO_POSTURE_STAND;  // STAND, SIT or REST
  real tempo = 1;                        // kPostureTempoMin..Max
};

// The way a move goes (whatever its tempo): the joints after each stage, and what the model says of it.
struct PostureWay {
  int n = 0;                       // stages (0: no way)
  Joints at[kMaxStages];
  real margin = 0;                 // the least margin along it, mm
  real direct = 0;                 // ...of all at once, straight there (the body's own way)
  uint8_t via = 0;                 // the posture it goes through (0: straight there)
  char text[176] = {};             // the stages, for the log ("hind shoulders+hind knees, then ...")
};

class PostureMove {
 public:
  // The move from joints `from` to the goal. False (why()) when the goal is not a posture or no way keeps
  // kPostureMarginMm.
  bool build(const BodyModel& m, const Joints& from, const PostureGoal& g);
  // The same from the body posture `from` (STAND, SIT or REST): the way is worked out once per pair of
  // postures (it does not change with the tempo) and kept.
  bool build_from(const BodyModel& m, uint8_t from, const PostureGoal& g);
  const tricks::Trick& trick() const { return trick_; }
  const char* why() const { return why_; }
  const PostureWay& way() const { return way_; }
  real margin() const { return way_.margin; }
  real direct_margin() const { return way_.direct; }
  uint8_t via() const { return way_.via; }
  int stages() const { return trick_.n_frames; }
  const char* plan_text() const { return way_.text; }

 private:
  bool timed(const BodyModel& m, const Joints& from, const PostureGoal& g);
  tricks::Keyframe frames_[kMaxStages] = {};
  tricks::Trick trick_ = {};
  const char* why_ = nullptr;
  PostureWay way_;
  PostureWay cache_[3][3];  // by posture (REST 1, SIT 2, STAND 3, less 1): from, to
  bool cached_[3][3] = {};
};

}  // namespace engine
}  // namespace kilo

#endif
