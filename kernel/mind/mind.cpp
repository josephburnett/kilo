#include "mind.h"
#include "link/names.h"
#include "remote/client.h"
#include "remote/keys.h"
#include "tricks/library.h"
#include "util/log.h"
#include "util/mem.h"

namespace kilo {

namespace {

constexpr uint32_t kBodyLostUs = KILO_BODY_LOST_MS * 1000u;
// adapter.py: how long each move the behaviours start keeps the IMU detectors quiet
constexpr double kPostureMoveS = 0.5;
constexpr double kLookMoveS = 0.3;
constexpr double kTrickMoveS = 2.0;

const behaviour::Personality& personality(const MindConfig& c) {
  return c.personality != nullptr ? *c.personality : behaviour::kPersonality;
}

}  // namespace

Mind::Mind(Serial& serial, Clock& clock, Log& platform_log, Listener* listener, const MindConfig& cfg)
    : clock_(clock),
      cfg_(cfg),
      log_(clock, &platform_log),
      link_(serial, clock, log_),
      listener_(listener),
      host_(listener != nullptr ? *listener : no_listener_, link_, serial, clock, log_, log_, cfg.mind_build),
      walker_(link_, clock, log_),
      player_(link_, clock, log_),
      body_(*this),
      runner_(body_, personality(cfg), log_, cfg.seed),
      remote_(link_, walker_, player_, clock, log_, cfg.remote_cfg) {
  link_.set_listener(this);
  runner_.set_clock(seconds, this);
  runner_.set_percept_source(&body_);
  runner_.set_paused(paused, this);
}

void Mind::start() {
  started_us_ = clock_.now_us();
  next_stats_us_ = started_us_ + cfg_.stats_period_ms * 1000u;
  if (cfg_.balance_kp >= 0) {  // daemon --balance: balance on by default, held by nobody
    bool ok = client_set_balance(link_, true, cfg_.balance_kp);
    logf(log_, LogLevel::Info, "balance on by default (Kp %d/256)%s", (int)(cfg_.balance_kp * 256 + 0.5),
         ok ? "" : ": refused");
  }
  if (cfg_.behaviour)
    logf(log_, LogLevel::Info, "behaviour runner: seed %u, personality %s", (unsigned)cfg_.seed,
         personality(cfg_).name);
  if (cfg_.remote) logf(log_, LogLevel::Info, "IR remote on");
  link_.connect();
}

bool Mind::liveness(uint64_t now) {
  if (!behaviour_active()) return true;
  double last;
  bool ok;
  if (!runner_.last_tick(&last)) ok = now - started_us_ < kRunnerStartGraceUs;
  else ok = (double)now / 1e6 - last < (double)kRunnerStallUs / 1e6;
  if (ok != stamping_)
    logf(log_, LogLevel::Warn, ok ? "behaviour runner ticking again: stamping liveness"
                                  : "behaviour runner has not ticked for 1 s: no longer stamping the link's "
                                    "liveness (the body's watchdog takes over)");
  return ok;
}

void Mind::poll() {
  uint64_t now = clock_.now_us();
  stamping_ = liveness(now);
  if (stamping_) link_.mark_alive();
  if (listener_ != nullptr) host_.poll();
  if (host_.link_paused()) {  // the bridge or the flasher has the UART: the link and everything on it pause
    paused_ = true;
    return;
  }
  if (paused_) {  // a walk cut off by the pause cannot resume: end it now that the UART is ours again
    paused_ = false;
    if (walker_.active()) walker_.abort();
  }
  link_.poll();
  // the command slot's answer, to its owner, before anyone sends another
  player_.collect();
  stats(now);
  walker_.poll();
  player_.poll();
  if (cfg_.remote) remote_.poll();
  if (cfg_.behaviour) runner_.poll((double)clock_.now_us() / 1e6);
}

void Mind::stats(uint64_t now) {
  const Request& r = link_.req();
  if (stats_asked_ && !(r.type == KILO_MSG_STATS_REQ && r.seq == stats_seq_ && r.state == RequestState::Waiting)) {
    stats_asked_ = false;
    if (r.type == KILO_MSG_STATS_REQ && r.seq == stats_seq_ && r.state == RequestState::Done &&
        r.reply_len == sizeof(kilo_stats_t)) {
      kilo_stats_t st;
      mem_copy(&st, r.reply, sizeof st);
      logf(log_, LogLevel::Info,
           "body STATS: uptime %u s overruns %u max tick %u us rx_crc_err %u rx_overflow %u feet_late %u "
           "feet_underruns %u",
           (unsigned)st.uptime_s, st.overruns, st.max_tick_us, st.rx_crc_err, st.rx_overflow, st.feet_late,
           st.feet_underruns);
    }
  }
  if (cfg_.stats_period_ms == 0 || now < next_stats_us_) return;
  next_stats_us_ += cfg_.stats_period_ms * 1000u;
  if (next_stats_us_ <= now) next_stats_us_ = now + cfg_.stats_period_ms * 1000u;
  if (link_.state().has_hello && !stats_asked_ && link_.request(KILO_MSG_STATS_REQ, nullptr, 0)) {
    stats_asked_ = true;
    stats_seq_ = link_.req().seq;
  }
}

// ---- the link's notices ----------------------------------------------------------------------

void Mind::on_connection(bool connected, bool alive) {
  if (tap_ != nullptr) tap_->on_connection(connected, alive);
}

void Mind::on_hello(const kilo_hello_t& hello, bool solicited) {
  if (tap_ != nullptr) tap_->on_hello(hello, solicited);
}

void Mind::on_telemetry(const kilo_telemetry_t& t) {
  walker_.on_telemetry(t);
  // every TELEMETRY, at its arrival time (the daemon's _feed_percepts)
  const perception::PerceptEvents& ev = percepts_.update(t, (double)link_.state().telemetry_at_us / 1e6);
  for (int i = 0; i < ev.n; i++) {
    const perception::PerceptEvent& e = ev.e[i];
    logf(log_, LogLevel::Info, "percept %s%s%s", perception::percept_name(e.kind),
         e.direction != perception::TiltDirection::None ? " " : "", perception::tilt_direction_name(e.direction));
  }
  if (tap_ != nullptr) tap_->on_telemetry(t);
}

void Mind::on_event(const kilo_event_t& e) {
  walker_.on_event(e);
  player_.on_event(e);
  if (n_events_ == behaviour::Snapshot::kEvents) {
    for (int i = 1; i < n_events_; i++) events_[i - 1] = events_[i];
    n_events_--;
  }
  events_[n_events_++] = {e.code, e.arg, e.tick, e.epoch};
  event_count_++;
  if (e.code == KILO_EVENT_IR_KEY) ir_key(e);
  if (tap_ != nullptr) tap_->on_event(e);
}

void Mind::on_reboot(const kilo_hello_t* hello, const char* why) {
  if (tap_ != nullptr) tap_->on_reboot(hello, why);
}

void Mind::on_goals_cleared(const char* reason, uint32_t count) {
  if (tap_ != nullptr) tap_->on_goals_cleared(reason, count);
}

void Mind::on_refused(uint8_t goal_type, uint8_t reason) {
  if (tap_ != nullptr) tap_->on_refused(goal_type, reason);
}

// remote_ir.py _on_ir_key
void Mind::ir_key(const kilo_event_t& e) {
  if (!cfg_.remote) return;
  uint16_t arg = (uint16_t)e.arg;
  char text[48];
  remote::describe(arg, text, sizeof text);
  const remote::Key* key = remote::action_for(arg);
  if (key == nullptr) {
    logf(log_, LogLevel::Debug, "remote %s: ignored", text);
    return;
  }
  if (behaviour_active()) {
    logf(log_, LogLevel::Info, "remote %s: ignored (behaviour runner active)", text);
    return;
  }
  if (host_controls()) {
    logf(log_, LogLevel::Info, "remote %s: ignored (a laptop client holds the body)", text);
    return;
  }
  remote_.perform(*key, arg);
}

// ---- the behaviour layer's body -------------------------------------------------------------

void Mind::Body::snapshot(behaviour::Snapshot& out) {
  const BodyState& s = m_.link_.state();
  out = behaviour::Snapshot();
  out.has_epoch = s.epoch != kNoEpoch;
  out.epoch = s.epoch;
  out.reboots = s.reboots;
  out.connected = s.connected;
  out.alive = s.alive(m_.clock_.now_us(), kBodyLostUs);
  out.mode = s.has_telemetry ? s.telemetry.mode : -1;
  out.fallen = s.fallen();
  out.moving = s.moving();
  out.has_battery = s.battery_pct(&out.battery_pct);
  out.has_dist = s.distance_mm(&out.dist_mm);
  out.has_approach = s.approach_mm_s(&out.approach_mm_s);
  for (int i = 0; i < m_.n_events_; i++) out.events[i] = m_.events_[i];
  out.n_events = m_.n_events_;
  out.event_count = m_.event_count_;
}

bool Mind::Body::posture(const char* name, int8_t balance) {
  m_.percepts_.notify_self_motion(seconds(&m_), kPostureMoveS);
  bool ok = client_posture(m_.link_, &m_.walker_, name, balance);
  logf(m_.log_, ok ? LogLevel::Info : LogLevel::Warn, "behaviour: posture %s%s%s", name, balance == 1 ? " +balance" : "",
       ok ? "" : ": refused");
  return ok;
}

bool Mind::Body::set_balance(bool enable) {
  bool ok = client_set_balance(m_.link_, enable);
  logf(m_.log_, ok ? LogLevel::Info : LogLevel::Warn, "behaviour: balance %s%s", enable ? "on" : "off",
       ok ? "" : ": refused");
  return ok;
}

bool Mind::Body::look(double pan, double tilt) {
  m_.percepts_.notify_self_motion(seconds(&m_), kLookMoveS);
  return client_look(m_.link_, pan, tilt);
}

uint32_t Mind::Body::play(const char* trick) {
  const tricks::Trick* t = tricks::find_trick(trick);
  if (t == nullptr) return 0;
  m_.percepts_.notify_self_motion(seconds(&m_), kTrickMoveS);
  return m_.player_.start(*t);
}

behaviour::Status Mind::Body::play_status(uint32_t handle) {
  const tricks::PlayResult& r = m_.player_.result();
  if (r.play != handle) return behaviour::Status::Failure;  // another play took the player
  if (r.state == tricks::PlayState::Running) return behaviour::Status::Running;
  if (r.state == tricks::PlayState::Done && r.reason == KILO_TRICK_DONE_REASON_DONE) return behaviour::Status::Success;
  return behaviour::Status::Failure;
}

void Mind::Body::play_cancel(uint32_t handle) {
  if (m_.player_.result().play == handle) m_.player_.cancel();
}

void Mind::Body::beep(const uint16_t (*)[2], int) { unsupported_beeps++; }  // BodyClient has no beep (adapter.unsupported)

void Mind::Body::provide(behaviour::BehaviourPercepts& p, double* startle) {
  // runner.py behaviour_percepts(): what Percepts knows, face false (no camera)
  const perception::Percepts& pc = m_.percepts_;
  p.face = false;
  p.pet = pc.pet();
  p.near = pc.near();
  p.approaching = pc.approaching();
  p.clear = pc.clear();
  p.tilted = pc.tilted();
  p.tap = pc.tap();
  p.picked_up = pc.picked_up();
  *startle = pc.startle() ? 1.0 : 0.0;
}

}  // namespace kilo
