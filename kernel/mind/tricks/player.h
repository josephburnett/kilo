// player: uploads a trick into the body's store and plays it, over the
// link's one command slot, as a cooperative state machine. Python
// counterpart: the trick part of kilo/body/client.py (upload_trick,
// start_trick, play, stop_trick), whose blocking calls become steps of
// poll().
//
// A play (start()):
// 1. Upload, cached: if this player committed the same trick (trick_id and
//    CRC) and the body has not rebooted since, nothing is sent; otherwise
//    TRICK_COMMIT alone asks the body (OK = it holds exactly this trick),
//    and only on a refusal do TRICK_BEGIN, the TRICK_FRAMES chunks and
//    TRICK_COMMIT go out.
// 2. The latched SET_POSTURE goal is cleared (the trick's end posture
//    replaces it on the body), then TRICK_PLAY with the body's epoch, the
//    speed and the mirror flag. NO_TRICK (someone replaced the body's
//    trick since): uploaded again, once, and played again.
// 3. It ends on EVENT TRICK_DONE for its trick_id (reason DONE, STOPPED,
//    PREEMPTED, FELL), or after the expected duration + 1 s
//    (TRICK_DONE_MARGIN_S; none for a trick that loops until stopped): then
//    TRICK_STOP goes out and the play fails.
// cancel() is what cancelling Python's play() coroutine does: a play not yet
// done is stopped on the body (TRICK_STOP) and ends STOPPED.
//
// The link has one command slot. The owner calls collect() right after
// link.poll() and before anything else may send a request in that pass, so
// the player always sees its own answer before the slot is reused; poll()
// then sends the next step if the slot is free (waiting up to 2 s for it).
#ifndef KILO_MIND_TRICKS_PLAYER_H
#define KILO_MIND_TRICKS_PLAYER_H
#include <stdint.h>
#include "link/serial_link.h"
#include "platform.h"
#include "tricks/model.h"

namespace kilo {
namespace tricks {

constexpr uint64_t kDoneMarginUs = 1000000;  // client.py TRICK_DONE_MARGIN_S
constexpr uint64_t kSlotWaitUs = 2000000;    // the command slot busy this long: the play fails

enum class PlayState : uint8_t {
  Idle,     // never played
  Running,  // uploading, starting or playing
  Done,     // TRICK_DONE arrived: reason
  Failed,   // refused, timed out or no link: error
};

// How the last play went (client.py TrickResult, or the TrickError it raised).
struct PlayResult {
  PlayState state;
  uint32_t play;         // which start() this is (start() returns it)
  uint8_t trick_id;
  uint8_t reason;        // Done: KILO_TRICK_DONE_REASON_*
  bool uploaded;         // this play uploaded the trick
  uint32_t elapsed_ms;   // TRICK_PLAY's ACK to the TRICK_DONE
  bool has_expected;
  uint32_t expected_ticks;
  const char* error;     // Failed: why
};

const char* done_reason_name(uint8_t reason);

class TrickPlayer {
 public:
  TrickPlayer(SerialLink& link, Clock& clock, Log& log) : link_(link), clock_(clock), log_(log) {}

  // From the owner's LinkListener.
  void on_event(const kilo_event_t& e);

  // Upload (cached) and play trick. Returns the play's number (> 0), or 0
  // with *why set: an invalid trick or speed, or a play still running
  // (stop or cancel it first).
  uint32_t start(const Trick& trick, bool mirror = false, uint8_t speed_q6 = kSpeedUnity,
                 const char** why = nullptr);
  // TRICK_STOP (client.stop_trick): the body crossfades to the trick's end
  // posture over transition_ticks (0 = the body's default). A play still
  // uploading or starting is abandoned (Failed "stopped").
  void stop(uint8_t transition_ticks = 0);
  // A play not yet done is stopped on the body; nothing when none runs.
  void cancel() {
    if (busy()) stop();
  }

  // The answer to our command, if it came in this pass (see above).
  void collect();
  // Send the next step; timeouts. Never blocks.
  void poll();

  // A play is running (uploading, starting or waiting for TRICK_DONE) or a TRICK_STOP is to go out.
  bool busy() const { return step_ != Step::Idle || stop_pending_; }
  const PlayResult& result() const { return result_; }
  // The trick last started (null before any).
  const Trick* trick() const { return trick_; }

 private:
  enum class Step : uint8_t { Idle, Check, Begin, Frames, Commit, Play, Playing };

  bool send(uint8_t type, const void* payload, uint8_t len, uint64_t now);
  void answered(uint8_t type, bool ok, uint8_t status, uint64_t now);
  void fail(const char* why);
  void finish(uint8_t reason, uint64_t now);

  SerialLink& link_;
  Clock& clock_;
  Log& log_;

  Step step_ = Step::Idle;
  const Trick* trick_ = nullptr;
  bool mirror_ = false;
  uint8_t speed_ = kSpeedUnity;
  uint8_t tid_ = 0;
  uint16_t crc_ = 0;
  uint8_t chunk_ = 0;
  bool forced_ = false;     // uploading after the cache check failed or NO_TRICK
  bool replayed_ = false;   // NO_TRICK seen once already
  uint64_t step_at_ = 0;    // the step became due (the slot wait)
  uint64_t played_at_ = 0;  // TRICK_PLAY's ACK
  uint64_t timeout_us_ = 0; // 0: until stopped

  // our command in the link's slot
  bool waiting_ = false;
  uint8_t req_type_ = 0;
  uint8_t req_seq_ = 0;

  bool stop_pending_ = false;
  uint8_t stop_ticks_ = 0;

  // TRICK_DONE seen since our TRICK_PLAY went out
  bool done_seen_ = false;
  uint8_t done_reason_ = 0;

  // the trick the body holds as far as we know (client.py _trick_cached)
  bool cached_ = false;
  uint8_t cached_tid_ = 0;
  uint16_t cached_crc_ = 0;
  uint32_t cached_reboots_ = 0;

  uint32_t plays_ = 0;
  PlayResult result_ = {};
  char error_[64] = {};
};

}  // namespace tricks
}  // namespace kilo

#endif
