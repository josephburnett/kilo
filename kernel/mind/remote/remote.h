// remote: what the IR remote's keys do, as a cooperative state machine:
// one action at a time; the waits for the planned stop and a trick's
// TRICK_STOP are phases of poll(), and keys that come meanwhile queue.
// Keys by grid (keys.h): columns A-C left to right, rows 1-7
// from the bottom.
//
// The remote's state: the gait (trot at start), a signed speed level, the
// heading (straight, or left / right: turning in place) and the steer
// (-3..+3, + left, while walking straight). Levels +1..+5 walk forward at
// speed_factors() x the gait's default speed (the trot 0.4, 0.7, 1, 1.3, 1.6
// since 2026-11-17; the walk and the crawl 0.4, 0.7, 1, 1.15, 1.25),
// -1..-5 backward at the same factors; there is no level 0. Every stop (B6,
// A7, C5, C7, a walk that ended by itself) puts the level back to +3 (1x),
// the heading straight and the steer to 0.
//
// - A4 trot, B4 crawl, C4 walk: not walking, start walking forward in that
//   gait at +3. Walking in another gait: change the gait (the walk and the
//   trot directly, the crawl after every leg has stepped home), level,
//   heading and steer kept. Walking in that gait: straight forward at the
//   current level's magnitude, no steer (from a turn, a bend or backward);
//   already straight forward, nothing changes.
// - A6 / C6: not walking, turn left / right in place in the selected gait
//   at 1x. Walking forward or backward: steer one level to the left / right
//   (kSteerLevels: the inner side's stride 0.25, 0.5 or 1 of it shorter,
//   the outer's longer; at most 3 each way), without stopping; the nose
//   turns that way forward and backward alike. Turning in place: the other
//   key turns the other way.
// - B7 faster, B5 slower: one level up / down (... -2 -1 +1 +2 ...),
//   stopping at +5 and -5, the steer kept. Turning in place: stop turning
//   and walk straight forward (B7) or backward (B5) at the turn's level,
//   without a stop first. Not walking: ignored (they never start a walk). A
//   level the planner clamps to the same speed still changes the level, and
//   the log shows the real mm/s (planner::timing()).
// - B6 stand, A7 rest (sleep), C5 the calibration posture (CALIB): stop a
//   walk with the quick stop (planner/plan.h halt(): the feet down where
//   they are, the body stopped in about 0.2 s, the legs squared up) and wait
//   for it (up to stop_timeout), stop a playing trick, then take the posture.
// - C7 freeze: at once, ahead of any key waiting (the action under way is
//   cancelled, keys queued behind it dropped, each logged): the quick stop
//   of the walk (as B6; it ends in STANCE, powered), stop a playing trick,
//   stop the behaviour runner; not walking, Kilo keeps the posture he is in.
//   Then every key but B6 and A7 is ignored ("frozen") until one of them is
//   pressed.
// - A5: start the behaviour runner (after stopping a walk and a trick as a
//   posture key does). While it runs, B6, A7 and C7 stop it first and then
//   do their own action, A5 again does nothing, every other key is ignored
//   (but those the hook yields() to: the distance guard's sitting runner
//   gives way to the walking keys; docs/distance-guard.md).
// - A2 rear up (Kind::Trick, docs/rear-up-trick.md; B2 until 2026-11-17),
//   only with the rear-up key switched on (keys.h set_rear_up_key,
//   KILO_REAR_UP=1|2|3: the highest stage allowed; off, A2 is a row 1-3 key
//   and never reaches perform()): not playing a trick, stop a walk, then
//   start the stage's trick (rear_up, rear_up_2 or rear_up_3: the highest
//   allowed; the key is done once TRICK_PLAY is out). Kilo goes up to stage
//   1 and holds it: in rear_up_2 and rear_up_3 the lower stages' tops hold
//   4 s (rear_up.py STEP_HOLD). A2 while he goes up to or holds such a top
//   takes him on to the next stage (up to the build's); without it, the
//   remote stops the trick kStepLeadUs before that hold ends and he comes
//   down to SIT (step_gate()). The trick's own guards (pitch and roll at
//   every guarded hold, each stage's top among them) still stop it when
//   the body is not where the model says. A2 at any other moment, B6 and
//   C7 bring him down (TRICK_STOP over the trick's stop_ticks). Each stage
//   reached, gone on from or left is a log line and a mark in the trick's
//   flight-recorder run. Refused below kTrickMinMv at rest (the highest
//   reading of the last 20 s, TrickNotes::rested_mv: a walk just stopped
//   sags the reading for a moment) or on USB power (NO_BATTERY), so that
//   the body's low-battery reflex does not take Kilo to REST half way up.
// - Stopping a trick (A2, the posture keys, a walk from standing, C7) sends
//   TRICK_STOP over the trick's stop_ticks (rear_up: 1.5 s to SIT, the
//   higher stages 2 and 2.5 s, not the body's 0.2 s); a key that then
//   takes a posture or walks waits for
//   that crossfade first (Phase::Settle).
// - B2 meows (Kind::Sound): the Mind asks its sound player (sound/player.h)
//   and never calls perform() for it.
// - Rows 1-3 but B2 (and A2 unless switched on) do nothing (Mind::ir_key logs them as ignored).
// Every key leaves one log line, "remote <grid> (0x.., <label>): <outcome>".
// Which keys never reach perform() (repeats, other remotes' keys, rows 1-3,
// a laptop client holding the body) is the caller's call (Mind::ir_key).
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
extern const double kSpeedFactors[kSpeedSteps];      // the walk's and the crawl's levels
extern const double kTrotSpeedFactors[kSpeedSteps];  // the trot's levels
// The five levels of a gait, x its default speed.
const double* speed_factors(const planner::Pattern& p);
constexpr int kSteerSteps = 3;
extern const double kSteerLevels[kSteerSteps];   // the steer's magnitudes, level 1..3
constexpr int kDefaultLevel = 3;                  // +3: 1x, the gait's spec defaults
constexpr uint16_t kTrickMinMv = KILO_BATTERY_TRICK_MIN_MV;  // spec [battery]: A2 refuses below this pack voltage

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
  // A key that stops it and then acts, like B6 / A7 (the distance guard's
  // runner gives way to the walking keys: docs/distance-guard.md).
  virtual bool yields(const Key& /*key*/) { return false; }

 protected:
  ~BehaviourHook() = default;
};

// Who hears of a trick the remote refused to start (the flight recorder:
// a REFUSED trick run). Not pure, as BehaviourHook.
class TrickNotes {
 public:
  virtual void refused(const char* /*trick*/, const char* /*why*/) {}
  // A line into the trick's run (the rear-up's stages).
  virtual void trick_mark(const char* /*line*/) {}
  // The rear-up's stages in the player's play `play`: the highest stage allowed so far (1 at the
  // start, A2 raises it), the highest whose top was reached (0: none yet), of `of` (the run's label).
  virtual void trick_stage(uint32_t /*play*/, int /*allowed*/, int /*reached*/, int /*of*/) {}
  // The battery at rest (power/battery.h RestedBattery), false if none: the trick's refusal.
  virtual bool rested_mv(uint16_t* /*mv*/) { return false; }

 protected:
  ~TrickNotes() = default;
};

// walker.py stop_timeout(): how long a planned stop may take.
double stop_timeout_s(double period_ms);

class RemoteControl {
 public:
  static constexpr int kQueue = 4;

  RemoteControl(SerialLink& link, planner::Walker& walker, tricks::TrickPlayer& player, Clock& clock, Log& log);
  void set_behaviour(BehaviourHook* hook) { behaviour_ = hook; }
  void set_trick_notes(TrickNotes* notes) { notes_ = notes; }

  // Carry out a key's action (now, or after the ones queued; C7 at once).
  // False when the queue is full (the key is dropped, and logged).
  bool perform(const Key& key, uint16_t arg);
  void poll();
  bool busy() const { return phase_ != Phase::Idle || n_ > 0; }

  const char* pattern() const { return pattern_->name; }
  int level() const { return level_; }
  Heading heading() const { return heading_; }
  int steer() const { return steer_; }  // -3..+3, + left
  // The planner's steer for the steer level (-1..1, + left; 0 turning in place).
  double steer_value() const;
  bool frozen() const { return frozen_; }
  // The walk the remote started; false when not walking.
  bool direction(planner::Direction* d) const;
  // The last action's outcome ("walk forward (trot) 55.6 mm/s", "posture stand") and whether it failed.
  const char* last() const { return last_; }
  bool last_failed() const { return last_failed_; }
  // Actions done, and failed or refused, since start (the session metrics).
  uint32_t actions() const { return actions_; }
  uint32_t failures() const { return failures_; }
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
  void walk_done(const char* suffix, const char* prefix = "");
  // ", steering left 2", or "" when straight.
  void steer_text(char* buf, uint32_t cap) const;
  planner::WalkGoal walk_goal() const;
  void stop_then_act(uint64_t now);
  // The rear-up's stages (above): the stage top the play is going up to or holding (1..), 0 none.
  int step_at(uint64_t* hold_end_us) const;
  void step_gate();
  void step_note(const char* what);

  SerialLink& link_;
  planner::Walker& walker_;
  tricks::TrickPlayer& player_;
  Clock& clock_;
  Log& log_;
  BehaviourHook* behaviour_ = nullptr;

  const planner::Pattern* pattern_;
  int level_ = kDefaultLevel;
  Heading heading_ = Heading::Straight;
  int steer_ = 0;
  bool walking_ = false;  // the remote started the walker's walk
  bool frozen_ = false;
  // the rear-up's stages: the play, its highest stage, the stage A2 allows now
  const tricks::Trick* step_trick_ = nullptr;
  uint32_t step_play_ = 0;
  int step_max_ = 0, step_allowed_ = 0, step_held_ = 0, step_reached_ = 0;

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
  uint32_t actions_ = 0, failures_ = 0;
  TrickNotes* notes_ = nullptr;
};

}  // namespace remote
}  // namespace kilo

#endif
