#include "mind.h"
#include "link/names.h"
#include "remote/client.h"
#include "remote/keys.h"
#include "tricks/library.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/mem.h"
#include "util/str.h"

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
      runner_hook_(*this),
      remote_(link_, walker_, player_, clock, log_),
      sleep_(link_, walker_, clock, log_),
      guard_(link_, walker_, clock, log_),
      alive_(link_, player_, log_, cfg.seed),
      recorder_(clock, link_) {
  sleep_.set_enabled(cfg.light_sleep);
  remote::set_rear_up_key(cfg.rear_up_key);
  link_.set_listener(this);
  link_.set_quiet_log(&log_.status());  // a held IR key's repeats: not into the ring unless a laptop reads it
  remote_.set_behaviour(&runner_hook_);
  remote_.set_trick_notes(this);
  recorder_.set_metrics(&metrics_);
  host_.attach_metrics(*this);
  host_.attach_guard(guard_);
  guard_.set_host(this);
  guard_.set_enabled(cfg.guard);
  if (cfg.random != nullptr) host_.set_keys(cfg.authorized_keys, cfg.authorized_key_count, *cfg.random);
  runner_.set_clock(seconds, this);
  runner_.set_percept_source(&body_);
  runner_.set_paused(paused, this);
}

void Mind::attach_recorder(uint8_t* storage, uint32_t bytes) {
  recorder_.attach(storage, bytes, cfg_.mind_build);
  if (!recorder_.attached()) {
    logf(log_, LogLevel::Warn, "flight recorder: %u bytes is too small: nothing is recorded", (unsigned)bytes);
    return;
  }
  link_.set_frame_tap(&recorder_);
  walker_.set_observer(&recorder_);
  log_.set_tap(&recorder_);
  host_.attach_recorder(recorder_);
  logf(log_, LogLevel::Info, "flight recorder: %u MB of RAM, %u KB blocks; log history %u lines",
       (unsigned)(recorder_.flight().capacity() >> 20), (unsigned)(record::kBlockBytes >> 10), (unsigned)log_.capacity());
}

void Mind::attach_saver(record::Saver& saver) {
  saver_ = &saver;
  sleep_.set_saver(&saver);
  saver.set_metrics(this);
  host_.attach_saver(saver);
}

bool Mind::body_limp() const {
  const BodyState& b = link_.state();
  if (!b.has_telemetry || !link_.connected()) return true;
  return b.telemetry.mode == KILO_MODE_LIMP || !(b.telemetry.flags & KILO_TELEMETRY_FLAGS_SERVOS_POWERED);
}

// The card stalls the loop: only with nothing moving, no run open or in its
// post-roll, nothing that needs the loop (a flash, an update, the bridge),
// and so for kCardQuietUs.
bool Mind::safe_for_card(uint64_t now) {
  constexpr uint64_t kCardQuietUs = 2000000;
  const BodyState& b = link_.state();
  bool moving = b.has_telemetry && (b.telemetry.mode == KILO_MODE_GAIT || b.telemetry.mode == KILO_MODE_TRICK ||
                                    b.telemetry.mode == KILO_MODE_FALLEN || b.telemetry.mode == KILO_MODE_RECOVERING ||
                                    b.telemetry.mode == KILO_MODE_CALIB || b.telemetry.mode == KILO_MODE_BOOT);
  bool busy = moving || walker_.active() || player_.busy() || remote_.busy() || recorder_.recording() ||
              host_.link_paused() || host_.flashing() || host_.updating() || behaviour_active();
  if (busy) unsafe_at_us_ = now;
  return !busy && now - unsafe_at_us_ >= kCardQuietUs;
}

void Mind::start() {
  started_us_ = clock_.now_us();
  next_stats_us_ = started_us_ + cfg_.stats_period_ms * 1000u;
  if (cfg_.balance_kp >= 0) {  // daemon --balance: balance on by default, held by nobody
    bool ok = client_set_balance(link_, true, cfg_.balance_kp);
    logf(log_, LogLevel::Info, "balance on by default (Kp %d/256)%s", (int)(cfg_.balance_kp * 256 + 0.5),
         ok ? "" : ": refused");
  }
  behaviour_on_ = cfg_.behaviour;
  runner_started_us_ = started_us_;
  if (cfg_.behaviour)
    logf(log_, LogLevel::Info, "behaviour runner: seed %u, personality %s", (unsigned)cfg_.seed,
         personality(cfg_).name);
  if (cfg_.remote) logf(log_, LogLevel::Info, "IR remote on");
  logf(log_, LogLevel::Info, "distance guard %s (BLUE from %d mm standing, RED below %d mm; docs/distance-guard.md)",
       guard_.enabled() ? "on" : "off", (int)guard::kBlueMinMm, (int)guard::kRedMm);
  if (saver_ != nullptr && cfg_.record_save) saver_->set_mode(1);
  link_.connect();
}

bool Mind::liveness(uint64_t now) {
  if (!behaviour_active()) return true;
  double last;
  bool ok;
  if (!runner_.last_tick(&last)) ok = now - runner_started_us_ < kRunnerStartGraceUs;
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
  recorder_.poll(player_);  // runs end their post-rolls, also while the link is paused
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
  if (behaviour_on_) runner_.poll((double)clock_.now_us() / 1e6);
  {
    const behaviour::Choice* c = runner_.record().choice;
    bool calm = c != nullptr && (c->chosen == behaviour::kIdle || c->chosen == behaviour::kExplore);
    alive_.poll((double)clock_.now_us() / 1e6, behaviour_active() && !host_controls(), calm, head_held(2));
  }
  guard_.poll();
  recorder_.poll(player_);
  card_safe_ = safe_for_card(clock_.now_us());
  if (saver_ != nullptr) saver_->poll(card_safe_, body_limp());
  power::Activity act;  // after the saver: a save it was asked for has started by now
  act.busy = walker_.active() || player_.busy() || remote_.busy() || behaviour_active();
  act.host_events = host_.stats().accepted + host_.stats().requests;
  act.host_clients = listener_ != nullptr ? host_.clients() : 0;
  sleep_.poll(act);
}

void Mind::stats(uint64_t now) {
  const Request& r = link_.req();
  if (stats_asked_ && !(r.type == KILO_MSG_STATS_REQ && r.seq == stats_seq_ && r.state == RequestState::Waiting)) {
    stats_asked_ = false;
    if (r.type == KILO_MSG_STATS_REQ && r.seq == stats_seq_ && r.state == RequestState::Done &&
        r.reply_len == sizeof(kilo_stats_t)) {
      kilo_stats_t st;
      mem_copy(&st, r.reply, sizeof st);
      logf(log_.status(), LogLevel::Info,
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
  sleep_.on_telemetry(t);
  guard_.on_telemetry(t, head_held(4));
  if (t.batt_mv != 0 && !(t.flags & KILO_TELEMETRY_FLAGS_NO_BATTERY)) {
    metrics_.battery_mv = t.batt_mv;
    if (metrics_.battery_min_mv == 0 || t.batt_mv < metrics_.battery_min_mv) metrics_.battery_min_mv = t.batt_mv;
  }
  // every TELEMETRY, at its arrival time (the daemon's _feed_percepts)
  const perception::PerceptEvents& ev = percepts_.update(t, (double)link_.state().telemetry_at_us / 1e6);
  for (int i = 0; i < ev.n; i++) {
    const perception::PerceptEvent& e = ev.e[i];
    // Status: a hand near the sensor makes these several times a second (in the ring while a laptop reads it)
    logf(log_.status(), LogLevel::Info, "percept %s%s%s", perception::percept_name(e.kind),
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
  metrics_.events++;
  if (e.code == KILO_EVENT_FELL) metrics_.falls++;
  if (e.code == KILO_EVENT_RECOVERED) metrics_.recovered++;
  if (e.code == KILO_EVENT_WATCHDOG) metrics_.watchdogs++;
  if (e.code == KILO_EVENT_LOW_BATTERY) metrics_.low_battery++;
  if (e.code == KILO_EVENT_IR_KEY) sleep_.key((uint16_t)e.arg, cfg_.remote && !host_controls());  // wakes first
  if (e.code == KILO_EVENT_IR_KEY) ir_key(e);
  if (tap_ != nullptr) tap_->on_event(e);
}

void Mind::on_reboot(const kilo_hello_t* hello, const char* why) {
  metrics_.body_reboots++;
  if (tap_ != nullptr) tap_->on_reboot(hello, why);
}

void Mind::on_goals_cleared(const char* reason, uint32_t count) {
  if (tap_ != nullptr) tap_->on_goals_cleared(reason, count);
}

void Mind::on_refused(uint8_t goal_type, uint8_t reason) {
  char what[40];
  format(what, sizeof what, "goal %s %s", msg_name(goal_type), ack_status_name(reason));
  metrics_.refusals.add(what);
  if (tap_ != nullptr) tap_->on_refused(goal_type, reason);
}

// remote_ir.py _on_ir_key
void Mind::ir_key(const kilo_event_t& e) {
  if (!cfg_.remote) return;
  uint16_t arg = (uint16_t)e.arg;
  char text[48];
  remote::describe(arg, text, sizeof text);
  const remote::Key* key = remote::action_for(arg);
  if (remote::key_repeat(arg)) metrics_.key_repeats++;
  else metrics_.keys++;
  if (key == nullptr) {  // a held-key repeat (status: every ~110 ms while held) or another remote's key
    logf(remote::key_repeat(arg) ? log_.status() : log_, LogLevel::Debug, "remote %s: ignored", text);
    if (!remote::key_repeat(arg)) metrics_.keys_ignored++;
    return;
  }
  for (int k = 0; k < remote::kKeys && k < 32; k++)
    if (&remote::kKeyTable[k] == key) metrics_.key_counts[k]++;
  if (key->kind == remote::Kind::Ignore) {  // rows 1-3
    logf(log_, LogLevel::Info, "remote %s: ignored (no action on this key)", text);
    metrics_.keys_no_action++;
    return;
  }
  if (host_controls()) {
    logf(log_, LogLevel::Info, "remote %s: ignored (a laptop client holds the body)", text);
    metrics_.keys_laptop++;
    return;
  }
  if (guard_.holds() && forward_key(*key)) {  // the distance guard: only the safe directions
    char why[96];
    format(why, sizeof why, "remote %s: refused (distance guard %s: %s, %d mm ahead; turn, back up or B6)", text,
           guard::action_name(guard_.action()), guard::zone_name(guard_.zone()),
           guard_.filter().has() ? (int)guard_.filter().mm() : -1);
    logf(log_, LogLevel::Info, "%s", why);
    recorder_.note(why);
    guard_.count_refused();
    metrics_.refusals.add("guard forward key");
    return;
  }
  metrics_.keys_acted++;
  guard_.key_took_over();  // a key the guard lets through overrides what it was doing (it keeps holding)
  recorder_.key(text);  // into a trick or walk run: which key stopped it
  remote_.perform(*key, arg);
}

// ---- the session metrics (METRICS, kilo metrics --host; saved with the log) --------------

namespace {

void named(char* out, uint32_t n, size_t* len, const record::NamedCounts& c) {
  for (uint32_t i = 0; i < c.n(); i++) *len += format(out + *len, n - *len, " %s=%u", c.name(i), (unsigned)c.count(i));
  if (c.other() != 0) *len += format(out + *len, n - *len, " other=%u", (unsigned)c.other());
}

}  // namespace

uint32_t Mind::metric_lines() { return 11 + power::Sleep::kMetricLines; }

void Mind::metric_line(uint32_t i, char* out, uint32_t n) {
  const record::Metrics& m = metrics_;
  uint64_t now = clock_.now_us();
  size_t len = 0;
  out[0] = '\0';
  switch (i) {
    case 0:
      format(out, n, "session: uptime_s=%u mind_build=%08x boot=%u", (unsigned)((now - started_us_) / 1000000u),
             (unsigned)cfg_.mind_build, saver_ != nullptr ? (unsigned)saver_->boot() : 0u);
      return;
    case 1:
      len = format(out, n, "keys: pressed=%u acted=%u ignored=%u no_action=%u laptop_held=%u repeats=%u failed=%u",
                   (unsigned)m.keys, (unsigned)m.keys_acted, (unsigned)m.keys_ignored, (unsigned)m.keys_no_action,
                   (unsigned)m.keys_laptop, (unsigned)m.key_repeats, (unsigned)remote_.failures());
      for (int k = 0; k < remote::kKeys && k < 32; k++)
        if (m.key_counts[k] != 0)
          len += format(out + len, n - len, " %s=%u", remote::kKeyTable[k].grid, (unsigned)m.key_counts[k]);
      return;
    case 2:
      len = format(out, n, "walks: started=%u ok=%u holds=%u catches=%u", (unsigned)m.walks, (unsigned)m.walks_ok,
                   (unsigned)m.walk_holds, (unsigned)m.walk_catches);
      named(out, n, &len, m.walk_ends);
      return;
    case 3:
      len = format(out, n, "tricks: started=%u", (unsigned)m.tricks);
      named(out, n, &len, m.trick_ends);
      len += format(out + len, n - len, " |");
      named(out, n, &len, m.trick_names);
      return;
    case 4:
      len = format(out, n, "refusals: total=%u", (unsigned)m.refusals.total());
      named(out, n, &len, m.refusals);
      return;
    case 5:
      format(out, n, "body: falls=%u recovered=%u watchdog=%u low_battery=%u events=%u reboots=%u battery_mv=%u "
                     "battery_min_mv=%u",
             (unsigned)m.falls, (unsigned)m.recovered, (unsigned)m.watchdogs, (unsigned)m.low_battery,
             (unsigned)m.events, (unsigned)m.body_reboots, (unsigned)m.battery_mv, (unsigned)m.battery_min_mv);
      return;
    case 6: {
      const LinkStats& s = link_.stats();
      format(out, n, "link: rx=%u tx=%u heartbeats=%u decode_errors=%u crc=%u cobs=%u overflow=%u tx_deferred=%u "
                     "tx_short=%u",
             (unsigned)s.frames_rx, (unsigned)s.frames_tx, (unsigned)s.heartbeats, (unsigned)s.decode_error_total(),
             (unsigned)s.decode_errors[KILO_FRAME_BAD_CRC], (unsigned)s.decode_errors[KILO_FRAME_BAD_COBS],
             (unsigned)s.decode_errors[KILO_FRAME_OVERFLOW], (unsigned)s.tx_deferred, (unsigned)s.tx_short);
      return;
    }
    case 7: {
      const record::FlightStore& f = recorder_.flight();
      format(out, n, "record: ram_used=%u ram_capacity=%u blocks=%u dropped_blocks=%u records=%u heartbeats=%u "
                     "runs=%u loop_max_us=%u period_max_us=%u",
             (unsigned)f.used_bytes(), (unsigned)f.capacity(), (unsigned)f.blocks(), (unsigned)f.dropped_blocks(),
             (unsigned)f.records(), (unsigned)f.skipped(), (unsigned)recorder_.runs_total(), (unsigned)m.loop_max_us,
             (unsigned)m.loop_period_max_us);
      return;
    }
    case 8: {
      if (saver_ == nullptr) {
        format(out, n, "card: none");
        return;
      }
      kilo_host_rec_status_t st;
      mem_zero(&st, sizeof st);
      saver_->fill_status(st);
      static const char* const kState[] = {"none", "off", "idle", "saving", "backoff", "failed"};
      format(out, n, "card: state=%s save_on=%u boot=%u saved_seq=%u unsaved=%u bytes=%u files=%u writes=%u "
                     "write_max_us=%u slow=%u errors=%u",
             st.card_state < 6 ? kState[st.card_state] : "?", (unsigned)st.save_on, (unsigned)saver_->boot(),
             (unsigned)st.saved_seq, (unsigned)st.unsaved_bytes, (unsigned)st.card_bytes, (unsigned)st.card_files,
             (unsigned)st.writes, (unsigned)st.write_max_us, (unsigned)st.slow_writes, (unsigned)st.card_errors);
      return;
    }
    case 9:
      guard_.metrics_line(out, n);
      return;
    case 10:
      alive_.metrics_line(out, n);
      return;
    default:
      sleep_.metric_line(i - 11, out, n);  // power, power_hw, battery, power_segments
      return;
  }
}

// ---- the distance guard ---------------------------------------------------------------------

// The SET_HEAD goal's pan (field 2) or tilt (field 4), degrees; 0 when not held.
double Mind::head_held(int field) const {
  const Goal* g = const_cast<SerialLink&>(link_).goal(KILO_MSG_SET_HEAD);
  if (g == nullptr || g->payload[1] != KILO_HEAD_MODE_HOLD) return 0;
  int16_t v = (int16_t)(g->payload[field] | (g->payload[field + 1] << 8));
  return v == KILO_HEAD_NOT_HELD ? 0 : (double)v / 16.0;
}

// A key that would walk forward now (remote.h): the gait keys (start forward, or straight forward
// from a turn or from backward), and B7 while walking straight forward or from -1 to +1.
bool Mind::forward_key(const remote::Key& key) const {
  if (key.kind == remote::Kind::Gait) return true;
  if (key.kind != remote::Kind::Speed || !str_eq(key.name, "faster")) return false;
  planner::Direction d;
  if (!remote_.direction(&d) || remote_.heading() != remote::Heading::Straight) return false;
  return remote_.level() > 0 || remote_.level() == -1;
}

bool Mind::may_act() {
  const BodyState& b = link_.state();
  bool trick = player_.busy() || (b.has_telemetry && b.telemetry.mode == KILO_MODE_TRICK);
  return !host_controls() && !paused_ && !trick && !percepts_.picked_up();
}

// BLUE (after the stop) and RED (after backing off): sit, and the behaviour runner's sitting variant.
void Mind::sit_and_behave() {
  sitting_variant_ = true;
  if (behaviour_active()) {
    runner_.context().data.forget_goals();  // the tree re-asserts its posture, now as a sit
    bool ok = client_posture(link_, &walker_, "sit");
    logf(log_, LogLevel::Info, "guard: the behaviour runner goes on sitting%s", ok ? "" : " (SIT refused)");
    return;
  }
  const char* what = nullptr;
  bool ok = runner_hook_.start(&what);
  runner_by_guard_ = ok;
  if (ok) client_posture(link_, &walker_, "sit");
  logf(log_, ok ? LogLevel::Info : LogLevel::Warn, "guard: behaviour runner %s (sitting variant%s%s)",
       ok ? "started" : "not started", what != nullptr ? ": " : "", what != nullptr ? what : "");
}

// ---- the behaviour runner, as the remote starts and stops it --------------------------------

bool Mind::RunnerHook::start(const char** what) {
  Mind& m = m_;
  *what = nullptr;
  if (m.behaviour_active()) return true;
  if (m.runner_.poweroff_requested()) {
    *what = "the wind-down asked for poweroff";
    return false;
  }
  if (m.behaviour_on_) m.runner_.restart();  // stopped before (by the remote): a fresh run
  m.behaviour_on_ = true;
  m.runner_started_us_ = m.clock_.now_us();
  format(what_, sizeof what_, "seed %u, personality %s", (unsigned)m.cfg_.seed, personality(m.cfg_).name);
  *what = what_;  // in the remote's line (one line per key)
  return true;
}

void Mind::RunnerHook::stop() {
  m_.runner_.stop();
  m_.alive_.stop();
  m_.sitting_variant_ = false;
  m_.runner_by_guard_ = false;
}

// The runner the guard started gives way to the walking keys (turn, back up; a forward key the guard
// refuses before it gets here), so Kilo can be steered away from what stopped him.
bool Mind::RunnerHook::yields(const remote::Key& key) {
  return m_.runner_by_guard_ && (key.kind == remote::Kind::Gait || key.kind == remote::Kind::Turn ||
                                 key.kind == remote::Kind::Speed);
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
  if (m_.sitting_variant_ && (str_eq(name, "stand") || str_eq(name, "balance") || str_eq(name, "stance"))) {
    logf(m_.log_, LogLevel::Info, "behaviour: posture %s: sit (the sitting variant)", name);
    name = "sit";
    balance = -1;
  }
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
