// intent: "be standing", "be sitting", "be resting" (docs/intent.md). The
// mind is asked for a state, not for a move: it looks where Kilo is (the
// state estimate, intent/estimate.h), takes the first step toward the goal
// from the generated table (intent.gen.h kNext, kEdges: mind/kilo/intent/
// graph.py), plays it, waits until he is settled, looks again, and goes on
// from where he really is.
//
// A step (an edge) is one of:
//   Move        a move planned and checked from the body model
//               (mind/kilo/intent/moves.py), played through the trick player
//               (tricks/player.h) with its roll limit;
//   Posture     the body's own posture change (SET_POSTURE): where no checked
//               move leads, and from UNKNOWN;
//   WaitReflex  fallen: wait for the body's fall reflex to lay him down
//               (EVENT LYING: given up, he needs a hand; the body has no
//               get-up of its own) or for a hand to right him, up to
//               kReflexWaitUs;
//   Right       instead, with the righting experiment (set_righting:
//               KILO_RIGHTING, docs/righting.md): from a side or his back, a
//               righting move played as a FALL_OK trick, whose POSE stream
//               takes the joints from the body's fall reflex (mirrored from
//               his left side). At most kMaxRightings of
//               them per intent; then, or when it gives up while he is down,
//               he is sent LIMP and the host says so (a meow).
//
// be(goal):
//   Settle   wait (up to kSettleWaitUs) for the estimate to leave MOVING;
//            still moving then: the body's own posture change to the goal.
//            The body's fall reflex at work (mode FALLEN: it has the joints
//            and refuses every goal): the Reflex wait at once.
//   decide   there already: the goal's posture is latched (SET_POSTURE, as
//            the remote's key does today) and it is done. The body LIMP
//            (servos off): REST first (a POSE stream is refused in LIMP), then
//            again. Else the first step toward the goal starts.
//   Move / Posture / Reflex   the step runs.
//   Check    wait (kCheckMinUs to kCheckWaitUs) for a settled estimate. Where the
//            step meant to end: go on. Somewhere else: a failed step; go on
//            from there all the same.
//   It gives up after kMaxFails failed steps or kMaxSteps steps in all: a
//   log line says why, and if he is upright he is left in REST (the body's
//   own change: the lowest posture), if fallen as he lies.
//
// cancel(): someone else takes the body (a walk, a trick, the laptop): a
// move under way is stopped (its end posture) and the intent forgotten.
//
// Nothing here runs unless the mind is built or started with intents on
// (KILO_INTENT=1, kilo-mind-host --intent): MindConfig.intent.
#ifndef KILO_MIND_INTENT_INTENT_H
#define KILO_MIND_INTENT_INTENT_H
#include <stdint.h>
#include "intent/estimate.h"
#include "link/serial_link.h"
#include "engine/walker.h"
#include "platform.h"
#include "tricks/player.h"

namespace kilo {
namespace intent {

// What the owner (the Mind) does for an intent.
class IntentHost {
 public:
  // A step begins: Kilo is about to move himself (the perception's self-motion window).
  virtual void intent_moves() {}
  // A line for the flight recorder ("intent goal=STANDING ...").
  virtual void intent_mark(const char* /*line*/) {}
  // The righting experiment gave up and left him limp: a sound for the owner.
  virtual void intent_gave_up_down() {}

 protected:
  ~IntentHost() = default;
};

enum class Phase : uint8_t { Idle, Settle, Wake, Move, Posture, Reflex, Check };

class Intent {
 public:
  static constexpr uint64_t kSettleWaitUs = 3000000;   // Settle: this long for the estimate to leave MOVING
  static constexpr uint64_t kWakeUs = 1500000;         // from LIMP: REST held this long before a move
  static constexpr uint64_t kPostureMinUs = 1000000;   // the body's own posture change: at least this before Check
  static constexpr uint64_t kCheckMinUs = 800000;      // Check: at least this (the estimate's still + confirm times)
  static constexpr uint64_t kCheckWaitUs = 4000000;    // ...and at most this for a settled estimate
  static constexpr uint64_t kReflexWaitUs = 15000000;  // WaitReflex: this long for EVENT LYING or a hand
  static constexpr uint64_t kPlayerWaitUs = 2000000;   // a move waits this long for the trick player to be free
  static constexpr int kMaxFails = 3;                  // failed steps before it gives up
  static constexpr int kMaxSteps = 6;                  // steps in all
  static constexpr int kMaxRightings = 3;              // righting moves per intent (the experiment)

  Intent(SerialLink& link, Clock& clock, Log& log, tricks::TrickPlayer& player, engine::Walker& walker,
         const Estimator& estimator)
      : link_(link), clock_(clock), log_(log), player_(player), walker_(walker), est_(estimator) {}
  void set_host(IntentHost* h) { host_ = h; }
  // The righting experiment (KILO_RIGHTING): Right edges instead of waiting for the body's reflex.
  void set_righting(bool on) { righting_ = on; }
  bool righting() const { return righting_; }
  // A righting move is playing or about to.
  bool righting_now() const { return active() && righting_ && fallen(from_) && phase_ == Phase::Move; }

  // Be in goal (Standing, Sitting or Resting). balance: the posture's balance flag once there (1 on, else
  // off). who: for the log ("remote B6", "behaviour"). False (and *why) when the goal is none of the three or
  // the body's protocol is not ours; nothing is sent then.
  bool be(State goal, int8_t balance = -1, const char* who = "", const char** why = nullptr);
  // Someone else takes the body: stop a move under way, forget the intent.
  void cancel(const char* why);
  // From the owner's LinkListener.
  void on_event(const kilo_event_t& e);
  // The intent's work. Never blocks. After the trick player's collect() and poll().
  void poll();

  bool active() const { return phase_ != Phase::Idle; }
  Phase phase() const { return phase_; }
  State goal() const { return goal_; }
  // The play (tricks::TrickPlayer::start's number) of the move under way, 0 for none.
  uint32_t play() const { return phase_ == Phase::Move ? play_ : 0; }
  // The trick player's last play this intent knows of: the one before it was asked (a trick still being
  // stopped then is waited for, not a reason to drop the intent), then its own moves. Another play
  // since: someone else started a trick.
  uint32_t known_play() const { return known_play_; }
  // How the last intent ended: null while one runs or before any; reached() says whether at the goal.
  const char* ended() const { return ended_; }
  bool reached() const { return reached_; }

  struct Counters {
    uint32_t asked, reached, gave_up, cancelled;
    uint32_t moves, postures, reflex_waits, failed_steps;
    uint32_t rightings, left_limp;
  };
  const Counters& counters() const { return n_; }

 private:
  void enter(Phase p, uint64_t now);
  void decide(State st, uint64_t now);
  void start_edge(const Edge& e, State st, uint64_t now);
  void wait_reflex(State st, uint64_t now);
  bool reflex_busy() const;  // the body's fall reflex has the joints
  bool latch(uint8_t posture, bool balance);
  void body_posture(State to, uint64_t now, const char* why);
  void finish(bool reached, const char* why);
  void give_up(State st, const char* why);
  void mark(const char* line);

  SerialLink& link_;
  Clock& clock_;
  Log& log_;
  tricks::TrickPlayer& player_;
  engine::Walker& walker_;
  const Estimator& est_;
  IntentHost* host_ = nullptr;

  Phase phase_ = Phase::Idle;
  uint64_t phase_at_ = 0;
  uint64_t asked_at_ = 0;
  State goal_ = State::Unknown;
  bool balance_ = false;
  State from_ = State::Unknown;     // the state the step under way began in
  State to_ = State::Unknown;       // ...and where it means to end
  const tricks::Trick* move_ = nullptr;
  bool mirror_ = false;             // the move plays mirrored (a righting move from his left side)
  bool righting_ = false;
  int rightings_ = 0;               // righting moves this intent
  bool move_started_ = false;       // the trick player took the move
  uint32_t play_ = 0;
  uint32_t known_play_ = 0;
  int steps_ = 0, fails_ = 0;
  bool lying_ = false;  // EVENT LYING seen in Reflex
  const char* ended_ = nullptr;
  bool reached_ = false;
  char ended_buf_[96] = {};
  char who_[24] = {};
  Counters n_ = {};
};

uint8_t posture_of(State s);  // KILO_POSTURE_STAND, _SIT or _REST for the three goals

}  // namespace intent
}  // namespace kilo

#endif
