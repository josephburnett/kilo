// remote: what the IR remote's keys do. A port of RemoteControl in
// mind/kilo/ir.py (kilo run --remote) as a cooperative state machine: the
// daemon runs perform() in a pool thread, one action at a time, and blocks
// on the planned stop, a trick's TRICK_STOP and the posture settle; here
// those waits are phases of poll(), and keys that come meanwhile queue (as
// the daemon's lock queues them).
//
// - A direction key (B7 forward, B5 back, A6 / C6 turn left / right, A4
//   march) walks with the selected pattern (trot at start) at the selected
//   speed until another key; a trick playing is stopped first.
// - A pattern key (A3 trot, C4 walk, B4 crawl) selects the pattern and
//   changes a walk in progress (after every leg has stepped home).
// - C5 / C1 step the speed down / up through 0.5, 0.7, 1, 1.4, 2 x the
//   pattern's default speed (kept across patterns and walks, stopping at the
//   ends); a walk in progress changes at once.
// - A posture key first stops a walk the planned way and waits for it (up
//   to stop_timeout), then stops a playing trick, then takes the posture.
// - A trick key does the same, takes STAND first from LIMP (and waits
//   settle_s), then starts the trick (uploaded if the body does not hold it),
//   not waiting for it to end.
// - C7 toggles the balance (SET_BALANCE with balance_kp).
// Which keys are ignored (repeats, other remotes' keys, the behaviour runner
// active, a laptop client holding the body) is the owner's call
// (Mind::on_event, remote_ir.py _on_ir_key).
#ifndef KILO_MIND_REMOTE_REMOTE_H
#define KILO_MIND_REMOTE_REMOTE_H
#include <stdint.h>
#include "link/serial_link.h"
#include "planner/walker.h"
#include "remote/keys.h"
#include "tricks/player.h"

namespace kilo {
namespace remote {

constexpr int kSpeedSteps = 5;
extern const double kSpeedFactors[kSpeedSteps];  // ir.py SPEED_FACTORS
constexpr int kDefaultSpeedStep = 2;              // 1.0: the pattern's spec defaults

struct RemoteConfig {
  double settle_s = 1.0;                      // the posture move from LIMP before a trick
  double balance_kp = KILO_BALANCE_KP_Q8 / 256.0;  // C7 on: the daemon's --balance Kp
};

// walker.py stop_timeout(): how long a planned stop may take.
double stop_timeout_s(double period_ms);

class RemoteControl {
 public:
  static constexpr int kQueue = 4;

  RemoteControl(SerialLink& link, planner::Walker& walker, tricks::TrickPlayer& player, Clock& clock, Log& log,
                const RemoteConfig& cfg = RemoteConfig());

  // Carry out a key's action (now, or after the ones queued). False when
  // the queue is full (the key is dropped, and logged).
  bool perform(const Key& key, uint16_t arg);
  void poll();
  bool busy() const { return phase_ != Phase::Idle || n_ > 0; }

  const char* pattern() const { return pattern_->name; }
  int speed_step() const { return speed_step_; }
  // The walk the remote started; false when not walking.
  bool direction(planner::Direction* d) const;
  // The last action's outcome ("walk forward (trot)", "posture sit") and whether it failed.
  const char* last() const { return last_; }
  bool last_failed() const { return last_failed_; }
  double speed_mm_s() const;

 private:
  enum class Phase : uint8_t { Idle, StopWalk, StopTrick, Settle };
  int mode() const;
  bool walking() const;
  void begin(const Key& key, uint16_t arg);
  void step();
  void act();
  void done(const char* what, bool failed);
  bool walk(planner::Direction d, const char** why);
  bool rewalk(const char** why);

  SerialLink& link_;
  planner::Walker& walker_;
  tricks::TrickPlayer& player_;
  Clock& clock_;
  Log& log_;
  RemoteConfig cfg_;

  const planner::Pattern* pattern_;
  int speed_step_ = kDefaultSpeedStep;
  bool walking_dir_ = false;
  planner::Direction dir_ = planner::Direction::Forward;

  struct Pending {
    const Key* key;
    uint16_t arg;
  };
  Pending q_[kQueue];
  int head_ = 0, n_ = 0;

  Phase phase_ = Phase::Idle;
  const Key* key_ = nullptr;
  uint16_t arg_ = 0;
  const tricks::Trick* trick_ = nullptr;
  uint64_t deadline_ = 0;
  const char* dir_name_ = "";  // the direction key's name ("backward"), for the log
  char text_[40] = {};
  char last_[80] = {};
  bool last_failed_ = false;
};

}  // namespace remote
}  // namespace kilo

#endif
