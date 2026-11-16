// remote: what the IR remote's keys do. A port of RemoteControl in
// mind/kilo/ir.py (kilo run --remote) as a cooperative state machine: the
// daemon runs perform() in a pool thread, one action at a time, and blocks
// on the planned stop and a trick's TRICK_STOP; here those waits are phases
// of poll(), and keys that come meanwhile queue (as the daemon's lock
// queues them). Keys by grid (keys.h): columns A-C left to right, rows 1-7
// from the bottom.
//
// The remote's state: the gait (trot at start), a signed speed level and
// the heading (straight, left, right). Levels +1..+5 walk forward at 0.4,
// 0.7, 1, 1.15, 1.25 x the gait's default speed (kSpeedFactors), -1..-5
// backward at the same factors; there is no level 0. Every stop (B6, A7, C5, C7, a walk that
// ended by itself) puts the level back to +3 (1x) and the heading straight.
//
// - A4 trot, B4 crawl, C4 walk: not walking, start walking forward in that
//   gait at +3. Walking in another gait: change the gait (after every leg
//   has stepped home), level and heading kept. Walking in that gait:
//   straight forward at the current level's magnitude (from a turn or from
//   backward); already straight forward, nothing changes.
// - A6 / C6: turn left / right in place at the current level's magnitude;
//   not walking, start the turn in the selected gait at 1x.
// - B7 faster, B5 slower: one level up / down (... -2 -1 +1 +2 ...),
//   stopping at +5 and -5; while turning, the magnitude only (1..5). Not
//   walking: ignored (they never start a walk). A level the planner clamps
//   to the same speed still changes the level, and the log shows the real
//   mm/s (planner::timing()).
// - B6 stand, A7 rest (sleep), C5 the calibration posture (CALIB): stop a
//   walk the planned way and wait for it (up to stop_timeout), stop a
//   playing trick, then take the posture.
// - C7 freeze: at once, ahead of any key waiting (the action under way is
//   cancelled, keys queued behind it dropped, each logged): abort the walk
//   (no planned stop: the body holds the last pose for link.feet_hold_ticks,
//   settles the swing feet to the ground and stays in STANCE, powered,
//   GAIT_STOPPED arg 1), stop a playing trick, stop the behaviour runner;
//   not walking, Kilo keeps the posture he is in. Then every key but B6 and
//   A7 is ignored ("frozen") until one of them is pressed.
// - A5: start the behaviour runner (after stopping a walk and a trick as a
//   posture key does). While it runs, B6, A7 and C7 stop it first and then
//   do their own action, A5 again does nothing, every other key is ignored.
// - B2 rear up (Kind::Trick, the trick rear_up, docs/rear-up-trick.md):
//   not playing a trick, stop a walk the planned way, then start the trick
//   (the key is done once TRICK_PLAY is out; the trick plays on). While a
//   trick plays, B2 stops it: the toggle, a second press brings rear_up
//   down. Refused below kTrickMinMv or on USB power (NO_BATTERY), so that
//   the body's low-battery reflex does not take Kilo to REST half way up.
// - Stopping a trick (B2, the posture keys, a walk from standing, C7) sends
//   TRICK_STOP over the trick's stop_ticks (rear_up: 1.5 s to STAND, not
//   the body's 0.2 s); a key that then takes a posture or walks waits for
//   that crossfade first (Phase::Settle).
// - Rows 1-3 but B2 do nothing (Mind::ir_key logs them as ignored).
// Every key leaves one log line, "remote <grid> (0x.., <label>): <outcome>".
// Which keys never reach perform() (repeats, other remotes' keys, rows 1-3,
// a laptop client holding the body) is the owner's call (Mind::ir_key,
// remote_ir.py _on_ir_key).
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
constexpr int kDefaultLevel = 3;                  // +3: 1x, the gait's spec defaults
constexpr uint16_t kTrickMinMv = 7400;            // ir.py TRICK_MIN_MV: B2 refuses below this pack voltage

enum class Heading : uint8_t { Straight, Left, Right };

// The behaviour runner, as the remote starts and stops it (the Mind's).
// Not pure: a pure virtual would need __cxa_pure_virtual (behaviour/tree.h).
class BehaviourHook {
 public:
  virtual bool active() { return false; }
  // Start it: true (and *what: "seed 3, personality kilo", or null), or
  // false and *what says why not.
  virtual bool start(const char** what) {
    *what = "no behaviour runner";
    return false;
  }
  // Stop it now, leaving nothing it started running (its trick cancelled).
  virtual void stop() {}

 protected:
  ~BehaviourHook() = default;
};

// walker.py stop_timeout(): how long a planned stop may take.
double stop_timeout_s(double period_ms);

class RemoteControl {
 public:
  static constexpr int kQueue = 4;

  RemoteControl(SerialLink& link, planner::Walker& walker, tricks::TrickPlayer& player, Clock& clock, Log& log);
  void set_behaviour(BehaviourHook* hook) { behaviour_ = hook; }

  // Carry out a key's action (now, or after the ones queued; C7 at once).
  // False when the queue is full (the key is dropped, and logged).
  bool perform(const Key& key, uint16_t arg);
  void poll();
  bool busy() const { return phase_ != Phase::Idle || n_ > 0; }

  const char* pattern() const { return pattern_->name; }
  int level() const { return level_; }
  Heading heading() const { return heading_; }
  bool frozen() const { return frozen_; }
  // The walk the remote started; false when not walking.
  bool direction(planner::Direction* d) const;
  // The last action's outcome ("walk forward (trot) 55.6 mm/s", "posture stand") and whether it failed.
  const char* last() const { return last_; }
  bool last_failed() const { return last_failed_; }
  // The speed asked for: the level's factor of the gait's default speed (a magnitude).
  double speed_mm_s() const;

 private:
  enum class Phase : uint8_t { Idle, StopWalk, StopTrick, Settle };
  void stop_trick();  // TRICK_STOP over the trick's stop_ticks; settle_us_ for Phase::Settle
  void start_trick();
  int mode() const;
  bool body_walking() const;
  bool walking();
  void reset_motion();
  planner::Direction dir() const;
  void begin(const Key& key, uint16_t arg);
  void freeze(const Key& key, uint16_t arg);
  void step();
  void act();
  void done(const char* what, bool failed);
  void walk_done(const char* suffix);
  void stop_then_act(uint64_t now);

  SerialLink& link_;
  planner::Walker& walker_;
  tricks::TrickPlayer& player_;
  Clock& clock_;
  Log& log_;
  BehaviourHook* behaviour_ = nullptr;

  const planner::Pattern* pattern_;
  int level_ = kDefaultLevel;
  Heading heading_ = Heading::Straight;
  bool walking_ = false;  // the remote started the walker's walk
  bool frozen_ = false;

  struct Pending {
    const Key* key;
    uint16_t arg;
  };
  Pending q_[kQueue];
  int head_ = 0, n_ = 0;

  Phase phase_ = Phase::Idle;
  const Key* key_ = nullptr;
  uint16_t arg_ = 0;
  uint64_t deadline_ = 0;
  uint64_t settle_us_ = 0;  // the stopped trick's crossfade, waited for before act()
  const char* note_ = "";  // "behaviour runner stopped, ", "unfrozen, ": put before the outcome
  char text_[48] = {};
  char last_[112] = {};
  bool last_failed_ = false;
};

}  // namespace remote
}  // namespace kilo

#endif
