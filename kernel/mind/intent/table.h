// table: the types of the state estimate's and the intents' generated
// tables (intent.gen.h, made by intent/gen_intent.py from mind/kilo/intent).
// Python counterparts: kilo/intent/estimate.py (State, Cmd, Row) and
// kilo/intent/graph.py (Kind, Edge). docs/intent.md is the owner's description.
#ifndef KILO_MIND_INTENT_TABLE_H
#define KILO_MIND_INTENT_TABLE_H
#include <stdint.h>
#include "tricks/model.h"

namespace kilo {
namespace intent {

// Where Kilo is, as far as the mind can tell (estimate.py State, the same numbers).
enum class State : uint8_t {
  Unknown = 0,      // settled, but in no pose the model knows (leaning on something, held at an angle)
  Moving = 1,       // not settled, in a hand, or a move, a walk or a trick under way
  Standing = 2,
  Sitting = 3,
  Resting = 4,      // lying on his belly, legs folded (REST), or limp there
  OnLeftSide = 5,
  OnRightSide = 6,
  OnBack = 7,
  OnEnd = 8,        // the nose or the tail down, near vertical
};
constexpr int kStates = 9;
const char* state_name(State s);  // "STANDING", "ON_LEFT_SIDE"
inline bool fallen(State s) { return s >= State::OnLeftSide; }

// What the mind last asked the body for, as far as the attitude goes (estimate.py Cmd).
enum class Cmd : uint8_t {
  None = 0,   // nothing with a known resting attitude
  Stand = 1,  // STAND or STANCE
  Sit = 2,
  Rest = 3,
  Limp = 4,
};
const char* cmd_name(Cmd c);  // "stand", "none"
// The prior a SET_POSTURE code gives (estimate.py cmd_of_posture).
Cmd cmd_of_posture(uint8_t posture);

// One upright state's attitude in the model, as TELEMETRY reports it (a16), and its tolerance.
struct Row {
  State state;
  int16_t pitch, roll;
  int16_t tol_pitch, tol_roll;
};

// The goals an intent can name, in kNext's column order.
constexpr int kGoals = 3;
constexpr State kGoalStates[kGoals] = {State::Standing, State::Sitting, State::Resting};
inline int goal_index(State s) { return s == State::Standing ? 0 : s == State::Sitting ? 1 : s == State::Resting ? 2 : -1; }

// One step toward a goal (graph.py Kind, Edge).
enum class EdgeKind : uint8_t {
  None = 0,
  Engine = 1,      // stand, sit, rest: the move the engine makes from the model when asked (engine/posture.h)
  Posture = 2,     // the body's own posture change (SET_POSTURE)
  WaitReflex = 3,  // fallen: wait for the body's fall reflex (without the righting experiment)
  Right = 4,       // a side or his back: a righting move, a FALL_OK trick (the experiment, docs/righting.md)
};
struct Edge {
  State from, to;
  EdgeKind kind;
  const tricks::Trick* move;  // Right
  uint8_t posture;            // Posture: KILO_POSTURE_*
  bool mirror;                // Right: played mirrored
};

}  // namespace intent
}  // namespace kilo

#endif
