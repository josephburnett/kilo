#include "behaviour/runner.h"
#include "link/names.h"
#include "proto/kilo_protocol.h"
#include "util/dmath.h"
#include "util/log.h"

namespace kilo {
namespace behaviour {

namespace {

// body_events.py WATCHED
bool watched(uint8_t code) {
  return code == KILO_EVENT_FELL || code == KILO_EVENT_WATCHDOG || code == KILO_EVENT_LOW_BATTERY ||
         code == KILO_EVENT_BATTERY_OK || code == KILO_EVENT_BOOT || code == KILO_EVENT_LINK_RESTORED ||
         code == KILO_EVENT_GOAL_REFUSED;
}

int pct(double v) { return (int)(v * 100.0 + 0.5); }

}  // namespace

Runner::Runner(BodyIface& body, const Personality& cfg, Log& log, uint64_t seed, double hz)
    : body_(body), cfg_(cfg), log_(log), seed_(seed), period_(1.0 / hz), rng_(seed), lib_(cfg) {
  ctx_.body = &body;
  ctx_.cfg = &cfg;
  ctx_.rng = &rng_;
  ctx_.drives = Drives::from_personality(cfg);
}

bool Runner::last_tick(double* t) const {
  if (ticks_ == 0) return false;
  *t = last_tick_at_;
  return true;
}

void Runner::percepts(const Snapshot& snap) {
  BehaviourPercepts p;
  p.near = snap.has_dist && 0 < snap.dist_mm && snap.dist_mm < cfg_.near_mm;
  p.approaching = p.near && snap.has_approach && snap.approach_mm_s > cfg_.approach_mm_s;
  double startle = 0.0;
  if (snap.fallen && !was_fallen_) startle = 1.0;
  was_fallen_ = snap.fallen;
  if (source_ != nullptr) {
    double s = 0.0;
    source_->provide(p, &s);
    startle = d_max(startle, s);
  }
  p.startle = startle;
  ctx_.percepts = p;
}

void Runner::body_events(const Snapshot& snap) {
  if (be_has_link_ && (snap.connected != be_connected_ || snap.alive != be_alive_))
    logf(log_, LogLevel::Info, "behaviour: body connection connected=%d alive=%d", snap.connected ? 1 : 0,
         snap.alive ? 1 : 0);
  be_has_link_ = true;
  be_connected_ = snap.connected;
  be_alive_ = snap.alive;
  if (snap.has_epoch) {
    if (be_has_epoch_ && snap.epoch != be_epoch_)
      logf(log_, LogLevel::Info, "behaviour: body epoch %d -> %d", be_epoch_, snap.epoch);
    be_has_epoch_ = true;
    be_epoch_ = snap.epoch;
  }
  uint32_t fresh = snap.event_count - be_events_seen_;
  if (fresh > (uint32_t)snap.n_events) fresh = (uint32_t)snap.n_events;
  if (be_primed_) {  // events already there at the start happened before the watcher
    for (int i = snap.n_events - (int)fresh; i < snap.n_events; i++) {
      const Snapshot::Event& e = snap.events[i];
      if (watched(e.code))
        logf(log_, LogLevel::Info, "behaviour: body event %s arg %d epoch %u", event_name(e.code), e.arg, e.epoch);
    }
  }
  be_events_seen_ = snap.event_count;
  be_primed_ = true;
}

void Runner::log_choice() {
  const Choice& c = lib_.root().last();
  if (c.chosen == logged_choice_) return;
  logged_choice_ = c.chosen;
  const Drives& d = ctx_.drives;
  logf(log_, LogLevel::Info, "behaviour: %s (%s) energy %d%% curiosity %d%% sociability %d%% arousal %d%%",
       c.name != nullptr ? c.name : "none", c.reason, pct(d.energy), pct(d.curiosity), pct(d.sociability),
       pct(d.arousal));
}

const TickRecord& Runner::step() {
  Snapshot snap;
  body_.on_tick();
  body_.snapshot(snap);
  if (!has_key_ || snap.has_epoch != key_has_epoch_ || snap.epoch != key_epoch_ || snap.reboots != key_reboots_ ||
      snap.connected != key_connected_) {
    ctx_.data.forget_goals();
    has_key_ = true;
    key_has_epoch_ = snap.has_epoch;
    key_epoch_ = snap.epoch;
    key_reboots_ = snap.reboots;
    key_connected_ = snap.connected;
  }
  bool paused = paused_ != nullptr && paused_(paused_arg_);
  if (paused != was_paused_) {
    was_paused_ = paused;
    lib_.root().halt();  // pause: stop what runs; resume: start afresh
    ctx_.data.forget_goals();
    logf(log_, LogLevel::Info, "behaviour: %s", paused ? "paused (a laptop client holds the body)" : "resumed");
    logged_choice_ = -2;
  }
  ctx_.state = snap;
  if (paused) {
    ticks_++;
    rec_ = TickRecord();
    rec_.t = ctx_.clock != nullptr ? ctx_.clock(ctx_.clock_arg) : ctx_.now;
    rec_.tick = ticks_;
    rec_.paused = true;
    last_tick_at_ = rec_.t;
    return rec_;
  }
  percepts(snap);
  ctx_.begin_tick();
  double now = ctx_.now;
  body_events(snap);
  double dt = has_last_now_ ? now - last_now_ : 0.0;
  has_last_now_ = true;
  last_now_ = now;
  DriveInputs in;
  in.activity = lib_.root().last().activity;
  in.moving = snap.moving;
  in.has_battery = snap.has_battery;
  in.battery_pct = snap.battery_pct;
  in.social = ctx_.percepts.face || ctx_.percepts.pet;
  in.startle = ctx_.percepts.startle;
  in.approaching = ctx_.percepts.approaching;
  ctx_.drives.update(dt, in, cfg_, &rng_);
  uint32_t errors_before = ctx_.errors;
  lib_.root().tick(ctx_);
  ctx_.end_tick();
  ticks_++;
  last_tick_at_ = now;
  rec_ = TickRecord();
  rec_.t = now;
  rec_.tick = ticks_;
  rec_.choice = &lib_.root().last();
  rec_.path = ctx_.path;
  rec_.path_n = ctx_.path_n;
  rec_.drives = ctx_.drives;
  rec_.percepts = ctx_.percepts;
  rec_.mode = snap.mode;
  rec_.has_battery = snap.has_battery;
  rec_.battery_pct = snap.battery_pct;
  rec_.errors = ctx_.errors - errors_before;
  if (rec_.errors != 0 && ctx_.error_node != nullptr)
    logf(log_, LogLevel::Warn, "behaviour: %s: %s", ctx_.error_node, ctx_.error_text);
  log_choice();
  if (ctx_.data.poweroff && !poweroff_done_) {
    poweroff_done_ = true;
    stopped_ = true;
    logf(log_, LogLevel::Warn, "behaviour: wind-down done: poweroff requested (seed %u)", (unsigned)seed_);
  }
  return rec_;
}

bool Runner::poll(double now) {
  if (stopped_) return false;
  if (!started_) {
    started_ = true;
    start_ = now;
    k_ = 0;
  }
  if (now < start_ + (double)k_ * period_) return false;
  step();
  k_++;
  if (start_ + (double)k_ * period_ - now < -1.0) {  // fell far behind: resync rather than burst
    overruns_++;
    start_ = now;
    k_ = 0;
  }
  return true;
}

}  // namespace behaviour
}  // namespace kilo
