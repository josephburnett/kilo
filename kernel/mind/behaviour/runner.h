// runner: the behaviour layer's 10 Hz tick. A port of
// kilo/behaviour/runner.py (Runner.step, run, behaviour_percepts) and
// body_events.py. Each tick:
//
// 1. body.on_tick() and body.snapshot();
// 2. on an epoch change, a body reboot or a connection change the cached
//    posture/balance goals are forgotten (the body or the link dropped
//    them), so the behaviours re-assert them. While paused() (Python's
//    --override; here: a laptop client holds control) only this runs: the
//    running behaviour is halted (a trick stopped), nothing is ticked;
// 3. body events: the epoch, new EVENTs (FELL, WATCHDOG, LOW_BATTERY,
//    BATTERY_OK, BOOT, LINK_RESTORED, GOAL_REFUSED) and connection changes
//    are logged;
// 4. percepts: near / approaching from the snapshot's ultrasonic values
//    (the personality's thresholds), a startle on a new fall, then the
//    percept source's (perception's Percepts in the mind) over them, a
//    startle kept if either says so;
// 5. the drives advance by the elapsed time with the activity of the
//    behaviour that ran last tick;
// 6. the root is ticked.
// It stops when stop() is called or the wind-down asked for poweroff
// (poweroff_requested(); the runner never powers anything off).
//
// What differs from Python: no JSONL behaviour log; a log line when the
// choice changes (record() has the last tick for the laptop and tests),
// and body events as log lines. poll() keeps the 10 Hz grid (resynced when
// more than 1 s behind, as run() does) instead of an asyncio loop.
#ifndef KILO_MIND_BEHAVIOUR_RUNNER_H
#define KILO_MIND_BEHAVIOUR_RUNNER_H
#include <stdint.h>
#include "behaviour/body.h"
#include "behaviour/library.h"
#include "platform.h"

namespace kilo {
namespace behaviour {

// The runner's percept source (runner.py PerceptFeed / percepts=): writes
// the percepts it knows over p (dict.update) and its startle level.
class PerceptSource {
 public:
  virtual void provide(BehaviourPercepts& /*p*/, double* startle) { *startle = 0; }

 protected:
  ~PerceptSource() = default;
};

// One tick's record (runner.py step()'s rec).
struct TickRecord {
  double t;
  uint32_t tick;
  bool paused;
  const Choice* choice;
  const char* const* path;
  int path_n;
  Drives drives;
  BehaviourPercepts percepts;
  int mode;
  bool has_battery;
  double battery_pct;
  uint32_t errors;  // actions that failed with an error this tick
};

class Runner {
 public:
  // seed: the RNG's (Python picks a random one and logs it; so does the Mind).
  Runner(BodyIface& body, const Personality& cfg, Log& log, uint64_t seed, double hz = 10.0);
  void set_percept_source(PerceptSource* src) { source_ = src; }
  void set_paused(bool (*paused)(void*), void* arg) {
    paused_ = paused;
    paused_arg_ = arg;
  }
  // The clock the tree reads (seconds); the runner's own time base.
  void set_clock(double (*clock)(void*), void* arg) {
    ctx_.clock = clock;
    ctx_.clock_arg = arg;
  }

  // One tick (runner.step()).
  const TickRecord& step();
  // Tick when due on the hz grid; true when it ticked. now in seconds.
  bool poll(double now);
  void stop() { stopped_ = true; }
  // Ask for a clean wind-down (rest, then poweroff).
  void request_shutdown() { ctx_.data.shutdown_requested = true; }
  bool stopped() const { return stopped_; }
  bool poweroff_requested() const { return poweroff_done_; }

  const TickRecord& record() const { return rec_; }
  uint32_t ticks() const { return ticks_; }
  uint32_t overruns() const { return overruns_; }
  uint64_t seed() const { return seed_; }
  const Context& context() const { return ctx_; }
  Context& context() { return ctx_; }
  UtilitySelector& root() { return lib_.root(); }
  // When the last tick ran (seconds; false before the first).
  bool last_tick(double* t) const;

 private:
  void percepts(const Snapshot& snap);
  void body_events(const Snapshot& snap);
  void log_choice();

  BodyIface& body_;
  const Personality& cfg_;
  Log& log_;
  uint64_t seed_;
  double period_;
  PyRandom rng_;
  Library lib_;
  Context ctx_;
  PerceptSource* source_ = nullptr;
  bool (*paused_)(void*) = nullptr;
  void* paused_arg_ = nullptr;

  TickRecord rec_ = {};
  uint32_t ticks_ = 0;
  uint32_t overruns_ = 0;
  bool stopped_ = false;
  bool poweroff_done_ = false;
  bool has_last_now_ = false;
  double last_now_ = 0;
  bool has_key_ = false;
  int key_epoch_ = 0;
  bool key_has_epoch_ = false;
  uint32_t key_reboots_ = 0;
  bool key_connected_ = false;
  bool was_paused_ = false;
  bool was_fallen_ = false;
  // poll()'s grid
  bool started_ = false;
  double start_ = 0;
  uint64_t k_ = 0;
  double last_tick_at_ = 0;
  // body_events.py
  bool be_primed_ = false;
  bool be_has_epoch_ = false;
  int be_epoch_ = 0;
  bool be_has_link_ = false;
  bool be_connected_ = false, be_alive_ = false;
  uint32_t be_events_seen_ = 0;
  // the choice last logged
  int logged_choice_ = -2;
};

}  // namespace behaviour
}  // namespace kilo

#endif
