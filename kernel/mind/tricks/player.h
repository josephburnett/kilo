// player: plays a trick from the compiled-in library through the body's
// POSE stream, as a cooperative state machine (docs/movement-engine.md,
// step 1: the body no longer stores or plays tricks; the mind sends each
// keyframe as a pose for the body to reach by its tick, and the body moves
// there by itself, in a straight line or along a cosine as the frame says,
// correcting its balance where the frame asks for it).
//
// A play (start()):
// 1. Waits for the body to be heard on this connection, its epoch and its
//    clock (TELEMETRY: engine::BodyClock), at most kStartWaitUs.
// 2. Clears the latched SET_POSTURE goal (a refresh would end the stream),
//    then sends the keys (KeySeq: per frame its move's end, a step's jump,
//    its hold's end) as POSE with every leg as joint angles, the head joints
//    the trick sets (the others left to the body's posture), the frame's
//    BALANCE flag with the trick's expected attitude as the reference, EASE
//    for a cosine frame, FALL_OK for a get-up. The trick starts on the body's
//    next tick: frame 0's move from wherever the joints are, its key at
//    start + its dur (never closer than KILO_POSE_LOOKAHEAD_TICKS: a key
//    that close, and the ones after it, wait the difference). Each
//    key goes out once the one before it is within KILO_POSE_LOOKAHEAD_TICKS
//    of the body's tick, so the body always holds the next one.
// 3. When the body's TELEMETRY tick reaches the last key it is done: the
//    trick's end posture as a SET_POSTURE goal over XFADE_TICKS (that ends
//    the stream: the body takes the posture from where the trick left the
//    joints), cleared again once the body has left the stream; then Done
//    DONE. A trick that loops until stopped never gets there.
// 4. stop() (or a guard: model.h TrickGuard, Trick.roll_max; GuardWatch
//    reads every TELEMETRY): the end posture over the transition asked for
//    (the trick's stop_ticks by default), then Done STOPPED (result().guard
//    says which guard). EVENT FELL: Done FELL (the fall reflex has the
//    joints). The stream ended by someone else (EVENT STREAM_STOPPED, an
//    epoch change, the body leaving mode STREAM): Done PREEMPTED. POSE
//    refused, the link lost, keys not reached within kDoneMarginUs of the
//    last one's tick: Failed.
// cancel() stops a play not yet done.
#ifndef KILO_MIND_TRICKS_PLAYER_H
#define KILO_MIND_TRICKS_PLAYER_H
#include <stdint.h>
#include "link/serial_link.h"
#include "engine/walker.h"
#include "platform.h"
#include "tricks/model.h"

namespace kilo {
namespace tricks {

constexpr uint64_t kDoneMarginUs = 1000000;  // the last key not reached this long after its tick: Failed
constexpr uint64_t kStartWaitUs = 2000000;   // no contact, epoch or body clock this long after start(): Failed
constexpr uint64_t kEndWaitUs = 1000000;     // the body still in the stream this long after the end posture: done anyway

// How a play ended (Done).
enum DoneReason : uint8_t {
  kDone = 0,       // played to its last key
  kStopped = 1,    // stop(), cancel() or a guard
  kPreempted = 2,  // a reflex, an epoch bump or another goal ended the stream
  kFell = 3,       // the fall reflex took the joints
};
const char* done_reason_name(uint8_t reason);

enum class PlayState : uint8_t {
  Idle,     // never played
  Running,  // starting, playing or ending
  Done,     // ended: reason
  Failed,   // refused, timed out or no link: error
};

// How the last play went.
struct PlayResult {
  PlayState state;
  uint32_t play;         // which start() this is (start() returns it)
  uint8_t trick_id;      // trick_id_for(trick)
  uint8_t reason;        // Done: DoneReason
  uint32_t elapsed_ms;   // the trick's start on the body to its end (the last key, a stop, a fall)
  bool has_expected;
  uint32_t expected_ticks;
  const char* error;     // Failed: why
  const char* guard;     // the mind stopped it: which guard and why (null: it did not)
};

class TrickPlayer {
 public:
  TrickPlayer(SerialLink& link, Clock& clock, Log& log) : link_(link), clock_(clock), log_(log) {}

  // From the owner's LinkListener.
  void on_telemetry(const kilo_telemetry_t& t);
  void on_event(const kilo_event_t& e);

  // Play trick. Returns the play's number (> 0), or 0 with *why set: an
  // invalid trick or speed, or a play still running (stop or cancel it first).
  uint32_t start(const Trick& trick, bool mirror = false, uint8_t speed_q6 = kSpeedUnity,
                 const char** why = nullptr);
  // The end posture over transition_ticks (kTrickStopTicks: the trick's own
  // stop_ticks; 0: XFADE_TICKS). A play still starting is abandoned
  // (Failed "stopped").
  static constexpr int kTrickStopTicks = -1;
  void stop(int transition_ticks = kTrickStopTicks);
  // The transition a stop() with kTrickStopTicks asks for now.
  uint8_t stop_ticks() const { return trick_ != nullptr ? trick_->stop_ticks : 0; }
  // A play not yet done is stopped; nothing when none runs.
  void cancel() {
    if (busy()) stop();
  }

  // Send the keys due, watch the guards and the end. Never blocks.
  void poll();

  // A play is running (starting, playing or ending).
  bool busy() const { return step_ != Step::Idle; }
  const PlayResult& result() const { return result_; }
  // The trick last started (null before any).
  const Trick* trick() const { return trick_; }
  // The body is playing it: its keys go out (from played_at_us()).
  bool playing() const { return step_ == Step::Playing; }
  // The local time of the trick's start tick on the body (0 before it).
  uint64_t played_at_us() const { return played_at_; }
  uint8_t speed() const { return speed_; }
  bool mirrored() const { return mirror_; }
  // POSE keys sent in this play.
  uint32_t keys_sent() const { return sent_; }

 private:
  enum class Step : uint8_t { Idle, Starting, Playing, Ending };

  bool begin(uint64_t now);
  bool send_key(const TrickKey& k);
  void end_posture(uint8_t ticks);
  void fail(const char* why);
  void ended(uint8_t reason, uint64_t now);  // the stream is over: Ending (or Done at once for FELL, PREEMPTED)
  void finish();
  int64_t body_tick_now(uint64_t now) const;

  SerialLink& link_;
  Clock& clock_;
  Log& log_;
  engine::BodyClock body_;

  Step step_ = Step::Idle;
  const Trick* trick_ = nullptr;
  bool mirror_ = false;
  uint8_t speed_ = kSpeedUnity;
  uint64_t step_at_ = 0;
  uint64_t played_at_ = 0;
  int16_t epoch_ = kNoEpoch;
  uint8_t posture_flags_ = 0;  // the SET_POSTURE goal's flags when the play began (its balance flag)
  bool own_goal_ = false;      // the latched SET_POSTURE is our end posture (cleared once the body has it)

  KeySeq seq_;
  bool have_key_ = false;  // key_ is the next to send
  TrickKey key_ = {};
  int64_t base_ = 0;       // the trick's start tick (unwrapped): a key's tick is base_ + at
  int64_t last_sent_ = 0;  // the tick of the last key sent
  bool all_sent_ = false;
  uint32_t sent_ = 0;
  uint8_t reason_ = kDone;

  GuardWatch watch_;
  uint32_t seen_telemetry_ = 0;
  bool stream_seen_ = false;  // TELEMETRY mode STREAM since the first key

  uint32_t plays_ = 0;
  PlayResult result_ = {};
  char error_[64] = {};
};

}  // namespace tricks
}  // namespace kilo

#endif
