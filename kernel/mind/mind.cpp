#include "mind.h"
#include "behaviour/library.h"
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
// How long each move the behaviours start keeps the IMU detectors quiet
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
      measure_(link_, clock, log_),
      measure_hook_(*this),
      intent_(link_, clock, log_, player_, walker_, estimator_),
      posture_hook_(*this),
      righting_hook_(*this),
      sleep_(link_, walker_, clock, log_),
      guard_(link_, walker_, clock, log_),
      alive_(link_, player_, log_, cfg.seed, personality(cfg)),
      sound_(clock, log_, cfg.seed),
      chatter_(cfg.seed),
      recorder_(clock, link_) {
  sleep_.set_enabled(cfg.light_sleep);
  remote::set_rear_up_key(cfg.rear_up_key);
  link_.set_listener(this);
  link_.set_quiet_log(&log_.status());  // a held IR key's repeats: not into the ring unless a laptop reads it
  remote_.set_behaviour(&runner_hook_);
  remote_.set_trick_notes(this);
  remote_.set_measure(&measure_hook_);
  measure_.set_observer(this);
  intent_.set_host(this);
  if (cfg.intent) remote_.set_posture_hook(&posture_hook_);  // B6, A7: be(STANDING), be(RESTING)
  intent_.set_righting(cfg.righting);
  remote_.set_righting(&righting_hook_);
  recorder_.set_metrics(&metrics_);
  host_.attach_metrics(*this);
  host_.attach_guard(guard_);
  host_.attach_cal_gate(*this);
  guard_.set_host(this);
  guard_.set_enabled(cfg.guard);
  sound_.set_sounds(cfg.sounds, cfg.sound_count);
  sound_.set_output(cfg.sound_output);
  sound_.set_host(this);
  sound_.set_volume(cfg.sound_volume);
  sound_.set_enabled(cfg.sound);
  host_.attach_sound(sound_);
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
  constexpr uint64_t kCardHelloWaitUs = 3000000;  // as the boot flash check (flash/cardflash.h kHelloWaitUs)
  const BodyState& b = link_.state();
  // Not before the body's first HELLO (or 3 s without one): the card's first call mounts it,
  // which stalls the loop (1.5 s on the Pi), and the HELLO's answer was lost in that stall.
  bool booting = !b.has_hello && now - started_us_ < kCardHelloWaitUs;
  bool moving = b.has_telemetry && (b.telemetry.mode == KILO_MODE_GAIT || b.telemetry.mode == KILO_MODE_TRICK ||
                                    b.telemetry.mode == KILO_MODE_FALLEN || b.telemetry.mode == KILO_MODE_RECOVERING ||
                                    b.telemetry.mode == KILO_MODE_CALIB || b.telemetry.mode == KILO_MODE_BOOT);
  bool busy = booting || moving || walker_.active() || player_.busy() || remote_.busy() || measure_.active() ||
              recorder_.recording() ||
              host_.link_paused() || host_.flashing() || host_.updating() || behaviour_active();
  if (busy) unsafe_at_us_ = now;
  return !busy && now - unsafe_at_us_ >= kCardQuietUs;
}

void Mind::start() {
  started_us_ = clock_.now_us();
  next_stats_us_ = started_us_ + cfg_.stats_period_ms * 1000u;
  if (cfg_.balance_kp >= 0) {  // balance on by default, held by nobody
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
  logf(log_, LogLevel::Info,
       "state estimate on; intents %s (docs/intent.md)",
       cfg_.intent ? "ON: stand, sit and rest are moves planned from the model (B6, A7, the behaviour, the guard)"
                   : "off: postures change as before (KILO_INTENT=1 turns them on)");
  logf(log_, LogLevel::Info, "righting experiment %s (docs/righting.md)",
       cfg_.righting ? "ON: B1 rights him from a side or his back; the body's rc is off (FALL_RC 0)"
                     : "off: the body's fall reflex rights him (KILO_RIGHTING=1 turns it on)");
  logf(log_, LogLevel::Info,
       "distance guard %s (walking forward BLUE from %d mm, RED below %d mm; otherwise BLUE below %d, RED below %d; "
       "docs/distance-guard.md)",
       guard_.enabled() ? "on" : "off", (int)guard::kBlueMinMm, (int)guard::kRedMm, (int)guard::kBlueStillMm,
       (int)guard::kRedStillMm);
  {
    uint32_t ms = sound_.total_ms();
    if (sound_.count() == 0)
      logf(log_, LogLevel::Info, "sound: no sounds in this image%s", cfg_.sound_output == nullptr ? " (no output)" : "");
    else
      logf(log_, LogLevel::Info, "sound: %u sounds (%u.%u s) in the image, %s, volume %u; docs/sound.md",
           (unsigned)sound_.count(), (unsigned)(ms / 1000u), (unsigned)(ms / 100u % 10u), sound_.enabled() ? "on" : "off",
           (unsigned)sound_.volume());
  }
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
  host_.collect();  // a CAL's answer, before anyone else uses the command slot
  // the command slot's answer, to its owner, before anyone sends another
  player_.collect();
  stats(now);
  fall_rc_step();
  walker_.poll();
  if (measure_.active() && host_controls()) measure_.stop("a laptop client taking the body");
  measure_.poll();
  player_.poll();
  if (intent_.active()) {
    // someone else took the body: the intent is dropped (a move under way is stopped)
    if (host_controls()) intent_.cancel("a laptop client took the body");
    else if (measure_.active()) intent_.cancel("the self-measurement started");
    else if (walker_.active()) intent_.cancel("a walk started");
    else if (player_.busy() && player_.result().play != intent_.known_play()) intent_.cancel("a trick started");
  }
  intent_.poll();
  if (flinch_pending_ && !player_.busy()) {
    flinch_pending_ = false;
    if (!alive_.flinch()) logf(log_, LogLevel::Warn, "guard: the lean back was not played");
  }
  if (cfg_.remote) remote_.poll();
  if (behaviour_on_) runner_.poll((double)clock_.now_us() / 1e6);
  {
    const behaviour::Choice* c = runner_.record().choice;
    bool calm = c != nullptr && (c->chosen == behaviour::kIdle || c->chosen == behaviour::kExplore);
    alive_.poll((double)clock_.now_us() / 1e6, behaviour_active() && !host_controls() && !intent_.active(), calm,
                head_held(2));
  }
  battery_step();
  head_for_walk();
  posture_moves();
  guard_.poll();
  recorder_.poll(player_);
  card_safe_ = safe_for_card(clock_.now_us());
  if (saver_ != nullptr) saver_->poll(card_safe_, body_limp());
  power::Activity act;  // after the saver: a save it was asked for has started by now
  act.busy = walker_.active() || player_.busy() || remote_.busy() || measure_.active() || behaviour_active() ||
             intent_.active();
  act.host_events = host_.stats().accepted + host_.stats().requests;
  act.host_clients = listener_ != nullptr ? host_.clients() : 0;
  act.host_holds = host_controls();
  act.lying_rest = lying_rest();
  sleep_.poll(act);
  sound_events();
  sound_.poll();
}

// ---- low battery ----------------------------------------------------------------------------

bool Mind::rested_pct(double* pct) const {
  const BodyState& b = link_.state();
  uint16_t mv;
  if (!b.has_telemetry || (b.telemetry.flags & KILO_TELEMETRY_FLAGS_NO_BATTERY) || !sleep_.rested_mv(&mv)) return false;
  *pct = battery_percent(mv);
  return true;
}

// The remote's trick refusal (remote.h A1): the battery at rest, not the sag right after a walk.
bool Mind::rested_mv(uint16_t* mv) {
  const BodyState& b = link_.state();
  if (!b.has_telemetry || (b.telemetry.flags & KILO_TELEMETRY_FLAGS_NO_BATTERY)) return false;
  return sleep_.rested_mv(mv);
}

// Tired: the battery at rest below the personality's low mark (the behaviour tree's rule and its +5 %
// to clear), or the body in LOW_BATT; whether or not the behaviour runner is on. The eyes turn orange.
void Mind::battery_step() {
  const BodyState& b = link_.state();
  bool low_mode = b.has_telemetry && b.telemetry.mode == KILO_MODE_LOW_BATT;
  double pct = 0;
  bool has = rested_pct(&pct);
  double low = personality(cfg_).low_battery_pct;
  uint16_t mv = 0;
  sleep_.rested_mv(&mv);
  if (!tired_ && (low_mode || (has && pct < low))) {
    tired_ = true;
    if (low_mode && !(has && pct < low))
      logf(log_, LogLevel::Info, "battery: tired (the body's LOW_BATT mode): the eyes orange while the way is clear");
    else
      logf(log_, LogLevel::Info, "battery: tired (%u mV at rest, %d %%, below %d %%): the eyes orange while the way is clear",
           (unsigned)mv, (int)pct, (int)low);
  } else if (tired_ && !low_mode && (!has || pct >= low + behaviour::kLowBatteryHysteresisPct)) {
    tired_ = false;
    if (has)
      logf(log_, LogLevel::Info, "battery: no longer tired (%u mV at rest, %d %%)", (unsigned)mv, (int)pct);
    else
      logf(log_, LogLevel::Info, "battery: no longer tired (no battery reading: USB power?)");
  }
  guard_.set_tired(tired_);
}

// ---- Kilo's sounds (docs/sound.md) ----------------------------------------------------------

bool Mind::trick_playing() const {
  const BodyState& b = link_.state();
  if (player_.busy()) return player_.trick() != &alive_.sit_trick();
  return b.has_telemetry && b.telemetry.mode == KILO_MODE_TRICK && !alive_.sitting();
}

uint8_t Mind::say(const char* name, const char* why, bool always) {
  if (!always && (trick_playing() || host_controls())) return KILO_SOUND_RESULT_OK;  // quiet now: not asked
  return sound_.request(name, why);
}

// The events' sounds, once a pass: light sleep's start and end, the guard backing off, the behaviour's chatter.
void Mind::sound_events() {
  bool asleep = sleep_.asleep();
  if (asleep != was_asleep_) {
    was_asleep_ = asleep;
    sound_.suspend(asleep);
    if (!asleep) say("mew", "wake", true);
  }
  if (guard_.hisses() != hisses_said_) {  // RED: backing off, leaning back, a turn stopped: once per event
    hisses_said_ = guard_.hisses();
    say("hiss", "guard red", true);
  }
  sound::Chatter::Mode mode = sound::Chatter::Mode::None;
  if (behaviour_active() && !host_controls()) {
    if (alive_.sitting() || sitting_variant_) mode = sound::Chatter::Mode::Sitting;
    else if (alive_.standing()) mode = sound::Chatter::Mode::Standing;
  }
  const char* what = chatter_.poll((double)clock_.now_us() / 1e6, mode, trick_playing() || sound_.busy());
  if (what != nullptr) say(what, "behaviour");
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
  fall_rc_off_ = false;  // a body that (re)booted has FALL_RC 1 again: set it once more
  if (!hello_said_) {  // the first HELLO after boot: ready
    hello_said_ = true;
    say("mew", "ready", true);
  }
  if (tap_ != nullptr) tap_->on_hello(hello, solicited);
}

// The state estimate's prior (mind.h).
intent::Cmd Mind::intent_cmd() const {
  const Goal* g = const_cast<SerialLink&>(link_).goal(KILO_MSG_SET_POSTURE);
  if (g != nullptr && g->len >= 2) return intent::cmd_of_posture(g->payload[1]);
  if (walker_.active() || measure_.active() || player_.busy()) return intent::Cmd::None;
  return prior_;
}

// The state estimate, every TELEMETRY: it only reads. A change is a log line, a MARK and a counter.
void Mind::estimate_step(const kilo_telemetry_t& t) {
  if (const_cast<SerialLink&>(link_).goal(KILO_MSG_SET_POSTURE) != nullptr || walker_.active() || measure_.active())
    prior_ = intent::Cmd::None;  // (a later trick's end, or the fall reflex's, sets it again)
  intent::Cmd cmd = intent_cmd();
  intent::Sample s = {link_.state().telemetry_at_us, t.pitch, t.roll, t.rate_pitch, t.rate_roll, t.rate_yaw, t.mode,
                      cmd};
  intent::State before = estimator_.state();
  if (!estimator_.feed(s)) return;
  const intent::Estimate& e = estimator_.estimate();
  if ((int)e.state < intent::kStates) state_entered_[(int)e.state]++;
  char p[12], r[12], l[120];
  format_fixed(p, sizeof p, (int32_t)t.pitch * 10 / 16, 1, true);
  format_fixed(r, sizeof r, (int32_t)t.roll * 10 / 16, 1, true);
  logf(log_, LogLevel::Info, "state: %s -> %s (pitch %s roll %s, asked %s, confidence %u)", intent::state_name(before),
       intent::state_name(e.state), p, r, intent::cmd_name(cmd), (unsigned)e.confidence);
  format(l, sizeof l, "state to=%s from=%s pitch_a16=%d roll_a16=%d asked=%s confidence=%u", intent::state_name(e.state),
         intent::state_name(before), (int)t.pitch, (int)t.roll, intent::cmd_name(cmd), (unsigned)e.confidence);
  recorder_.note(l);
}

void Mind::intent_moves() { percepts_.notify_self_motion(seconds(this), kPostureMoveS); }

void Mind::intent_gave_up_down() { say("meow", "righting gave up", true); }

// ---- the righting experiment (docs/righting.md) --------------------------------------------

// With it on, the body's param FALL_RC goes to 0 (live, not persisted) after every HELLO: its fall reflex
// then never plays rc, and takes the mind's FALL_OK righting move instead.
void Mind::fall_rc_step() {
  if (!cfg_.righting) return;
  const Request& r = link_.req();
  if (fall_rc_asked_ && !(r.type == KILO_MSG_PARAM_SET && r.seq == fall_rc_seq_ && r.state == RequestState::Waiting)) {
    fall_rc_asked_ = false;
    bool ok = r.type == KILO_MSG_PARAM_SET && r.seq == fall_rc_seq_ && r.state == RequestState::Done &&
              r.status == KILO_ACK_STATUS_OK;
    fall_rc_off_ = ok;
    logf(log_, ok ? LogLevel::Info : LogLevel::Warn, "righting: the body's fall reflex %s",
         ok ? "will not play rc (FALL_RC 0): B1 rights him" : "did not take FALL_RC 0 (an older body?): B1 is refused");
  }
  const BodyState& b = link_.state();
  if (fall_rc_off_ || fall_rc_asked_ || !b.has_hello || b.proto_ok != 1 || b.hello.proto_minor < 1) return;
  kilo_param_set_t p = {(uint8_t)KILO_PARAM_FALL_RC, 0, 0};
  if (link_.request(KILO_MSG_PARAM_SET, (const uint8_t*)&p, sizeof p)) {
    fall_rc_asked_ = true;
    fall_rc_seq_ = link_.req().seq;
  }
}

const char* Mind::RightingHookImpl::start() {
  Mind& m = m_;
  if (!m.cfg_.righting) return "the righting experiment is not built in (KILO_RIGHTING=1, docs/righting.md)";
  const BodyState& b = m.link_.state();
  if (b.has_hello && b.hello.proto_minor < 1) return "the body firmware is older than protocol 2.1 (no FALL_RC): flash it";
  if (!m.fall_rc_off_) return "the body has not taken FALL_RC 0 yet";
  intent::State st = m.estimator_.state();
  if (st != intent::State::OnLeftSide && st != intent::State::OnRightSide && st != intent::State::OnBack) {
    format(why_, sizeof why_, "he is %s: B1 rights him only from a side or his back", intent::state_name(st));
    return why_;
  }
  const char* why = nullptr;
  if (!m.intent_.be(intent::State::Standing, -1, "remote B1", &why)) return why != nullptr ? why : "refused";
  return nullptr;
}

// A posture by name (mind.h): an intent when they are on and the name is one of the three states.
bool Mind::posture(const char* name, int8_t balance, const char* who) {
  if (cfg_.intent) {
    intent::State goal = str_eq(name, "stand") || str_eq(name, "balance") ? intent::State::Standing
                         : str_eq(name, "sit") ? intent::State::Sitting
                         : str_eq(name, "rest") ? intent::State::Resting
                                                : intent::State::Unknown;
    if (goal != intent::State::Unknown) {
      if (str_eq(name, "balance") && balance < 0) balance = 1;
      return intent_.be(goal, balance, who);
    }
    intent_.cancel("another posture was asked for");
  }
  return client_posture(link_, &walker_, name, balance);
}

void Mind::on_telemetry(const kilo_telemetry_t& t) {
  estimate_step(t);
  walker_.on_telemetry(t);
  measure_.on_telemetry(t);
  sleep_.on_telemetry(t);
  guard_.on_telemetry(t, head_held(4), lying(t));
  if (t.batt_mv != 0 && !(t.flags & KILO_TELEMETRY_FLAGS_NO_BATTERY)) {
    metrics_.battery_mv = t.batt_mv;
    if (metrics_.battery_min_mv == 0 || t.batt_mv < metrics_.battery_min_mv) metrics_.battery_min_mv = t.batt_mv;
  }
  // every TELEMETRY, at its arrival time
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
  measure_.on_event(e);
  host_.on_body_event(e);
  player_.on_event(e);
  intent_.on_event(e);
  // the state estimate's prior when no posture goal is latched (intent_cmd())
  if (e.code == KILO_EVENT_TRICK_DONE) {
    uint8_t reason = (uint8_t)((uint16_t)e.arg >> 8);
    const tricks::Trick* t = player_.trick();
    bool ended = reason == KILO_TRICK_DONE_REASON_DONE || reason == KILO_TRICK_DONE_REASON_STOPPED;
    prior_ = ended && t != nullptr ? intent::cmd_of_posture(t->end_posture) : intent::Cmd::None;
  } else if (e.code == KILO_EVENT_FELL) {
    prior_ = intent::Cmd::None;
  } else if (e.code == KILO_EVENT_RECOVERED) {
    prior_ = intent::Cmd::Stand;  // the body's fall reflex ends in STAND
  } else if (e.code == KILO_EVENT_RECOVERY_FAILED) {
    prior_ = intent::Cmd::Rest;   // ...or in REST, then LIMP
  }
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

// EVENT IR_KEY: which keys reach the remote (remote/remote.h)
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
  remote_.key_seen(*key);  // C5's second press must follow C5 with no other key between (remote.h)
  if (measure_.active() && !host_controls()) {  // every key stops the self-measurement first (remote.h, C5 twice)
    metrics_.keys_acted++;
    recorder_.key(text);
    remote_.perform(*key, arg);
    return;
  }
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
  if (key->kind == remote::Kind::Sound) {  // B2: a sound moves nothing, so it goes at once, past the remote's queue
    static const char* const kResult[] = {"ok", "queued", "unknown", "off", "no sounds in this image", "asleep"};
    char why[16];
    format(why, sizeof why, "key:%s", key->grid);
    uint8_t r = sound_.request(key->name, why);
    logf(log_, LogLevel::Info, "remote %s: %s %s", text, key->name, r < 6 ? kResult[r] : "?");
    metrics_.keys_acted++;
    recorder_.key(text);
    return;
  }
  char guard_why[112];
  if (forward_key(*key) && guard_.refuses_forward(guard_why, sizeof guard_why)) {  // the distance guard: only the safe directions
    char why[192];
    format(why, sizeof why, "remote %s: refused (distance guard %s; turn, back up or B6)", text, guard_why);
    logf(log_, LogLevel::Info, "%s", why);
    recorder_.note(why);
    guard_.count_refused();
    metrics_.refusals.add("guard forward key");
    return;
  }
  metrics_.keys_acted++;
  guard_.key_took_over();  // a key the guard lets through overrides what it was doing (it keeps holding)
  recorder_.key(text);  // into a trick or walk run: which key stopped it
  if (intent_.active() && intent_.righting() && intent::fallen(estimator_.state()) && key->kind != remote::Kind::Righting)
    intent_.cancel("a key was pressed while righting");  // any other key stops the experiment
  remote_.perform(*key, arg);
}

// ---- the session metrics (METRICS, kilo metrics --host; saved with the log) --------------

namespace {

void named(char* out, uint32_t n, size_t* len, const record::NamedCounts& c) {
  for (uint32_t i = 0; i < c.n(); i++) *len += format(out + *len, n - *len, " %s=%u", c.name(i), (unsigned)c.count(i));
  if (c.other() != 0) *len += format(out + *len, n - *len, " other=%u", (unsigned)c.other());
}

}  // namespace

// 0-10 the Mind's, then the power lines (11-14), then sound (15): test_power and test_guard index them.
uint32_t Mind::metric_lines() { return 11 + power::Sleep::kMetricLines + 3; }

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
      if (i < 11 + power::Sleep::kMetricLines) sleep_.metric_line(i - 11, out, n);  // power, power_hw, battery, power_segments
      else if (i == 11 + power::Sleep::kMetricLines) sound_.metrics_line(out, n);
      else if (i == 12 + power::Sleep::kMetricLines) {
        // the state estimate (how often each state was entered), then the intents (docs/intent.md)
        const intent::Estimate& e = estimator_.estimate();
        len = format(out, n, "state: now=%s confidence=%u changes=%u", intent::state_name(e.state),
                     (unsigned)e.confidence, (unsigned)estimator_.changes());
        for (int k = 0; k < intent::kStates; k++)
          if (state_entered_[k] != 0 && len < n)
            len += format(out + len, n - len, " %s=%u", intent::state_name((intent::State)k),
                          (unsigned)state_entered_[k]);
      } else if (i == 13 + power::Sleep::kMetricLines) {
        const intent::Intent::Counters& c = intent_.counters();
        format(out, n,
               "intents: on=%u asked=%u reached=%u gave_up=%u cancelled=%u moves=%u body_postures=%u reflex_waits=%u "
               "failed_steps=%u",
               cfg_.intent ? 1u : 0u, (unsigned)c.asked, (unsigned)c.reached, (unsigned)c.gave_up,
               (unsigned)c.cancelled, (unsigned)c.moves, (unsigned)c.postures, (unsigned)c.reflex_waits,
               (unsigned)c.failed_steps);
      }
      return;
  }
}

// ---- the distance guard ---------------------------------------------------------------------

// The SET_HEAD goal's pan (field 2) or tilt (field 4), degrees; 0 when not held.
double Mind::head_held(int field) const {
  const Goal* g = const_cast<SerialLink&>(link_).goal(KILO_MSG_SET_HEAD);
  if (g == nullptr || g->payload[1] != KILO_HEAD_MODE_HOLD) return 0;
  int16_t v = (int16_t)(g->payload[field] | (g->payload[field + 1] << 8));
  return v == KILO_HEAD_NOT_HELD ? 0 : from_a16((double)v);
}

// Lying with the head down: the REST or LIMP posture asked (A7, light sleep) and no walk, or the
// body LIMP by itself. The ultrasonic reads the floor then (54-63 mm, docs/distance-guard.md).
bool Mind::lying(const kilo_telemetry_t& t) const {
  if (t.mode == KILO_MODE_LIMP) return true;
  const Goal* g = const_cast<SerialLink&>(link_).goal(KILO_MSG_SET_POSTURE);
  if (g == nullptr || g->len < 2 || walker_.active()) return false;
  return g->payload[1] == KILO_POSTURE_REST || g->payload[1] == KILO_POSTURE_LIMP;
}

// Lying in REST with the servos powered, for light sleep's idle count (power/sleep.h): the body IDLE
// with its servos on, no walk, trick or FEET, no feet it balances on (TELEMETRY support 0: the body
// reports all four feet in SIT, STAND and STANCE, none in REST, LIMP, CALIB and ZERO), and no other
// posture asked. After a mind restart there is no goal on record (the body kept the REST its watchdog
// took, or the last one asked): then TELEMETRY alone must say it. support 0 is also the body's pose
// held after a warm reboot (standing, perhaps) and the calibration postures CALIB and ZERO, so with
// no goal the ultrasonic must also have read the floor under his nose (55-63 mm lying, docs/distance-
// guard.md) within the last 2 s (a missed echo does not count against it).
bool Mind::lying_rest() {
  constexpr uint16_t kFloorMinMm = 40, kFloorMaxMm = 90;
  constexpr uint64_t kFloorFreshUs = 2000000;
  const BodyState& b = link_.state();
  uint64_t now = clock_.now_us();
  if (!b.has_telemetry || !link_.connected()) return false;
  const kilo_telemetry_t& t = b.telemetry;
  if (t.dist_mm >= kFloorMinMm && t.dist_mm <= kFloorMaxMm) {
    floor_seen_ = true;
    floor_seen_us_ = now;
  }
  if (walker_.active() || player_.busy()) return false;
  if (t.mode != KILO_MODE_IDLE || !(t.flags & KILO_TELEMETRY_FLAGS_SERVOS_POWERED) ||
      (t.flags & (KILO_TELEMETRY_FLAGS_GAIT | KILO_TELEMETRY_FLAGS_TRICK)) || t.support != 0)
    return false;
  const Goal* g = link_.goal(KILO_MSG_SET_POSTURE);
  if (g != nullptr && g->len >= 2) return g->payload[1] == KILO_POSTURE_REST;
  return floor_seen_ && now - floor_seen_us_ < kFloorFreshUs;
}

// A key that would walk forward now (remote.h): the gait keys (start forward, or straight forward
// from a turn or from backward); B7 while walking straight forward, from -1 to +1, or out of a turn
// in place (straight forward, docs/walking-and-balance.md §13.3); and A6 / C6 while walking forward
// (a steer: the walk bends but stays forward, §13.2). A6 / C6 from standing or backward are not.
bool Mind::forward_key(const remote::Key& key) const {
  if (key.kind == remote::Kind::Gait) return true;
  planner::Direction d;
  if (!remote_.direction(&d)) return false;
  bool straight = remote_.heading() == remote::Heading::Straight;
  if (key.kind == remote::Kind::Turn) return straight && remote_.level() > 0;
  if (key.kind != remote::Kind::Speed || !str_eq(key.name, "faster")) return false;
  return !straight || remote_.level() > 0 || remote_.level() == -1;
}

const char* Mind::cal_refusal() {
  if (walker_.active()) return "a walk is running: stop it first";
  const BodyState& b = link_.state();
  if (player_.busy() || (b.has_telemetry && b.telemetry.mode == KILO_MODE_TRICK)) return "a trick is playing";
  if (behaviour_active() && !host_controls()) return "the behaviour runner is on (B6 stops it)";
  return nullptr;
}

// Why the guard must not act now (guard.h GuardHost::not_now), or null. The alive layer's small
// sitting moves are not a trick here: the guard leans him back over them.
const char* Mind::not_now() {
  if (host_controls()) return "laptop";
  if (paused_) return "paused";
  if (sleep_.asleep()) return "asleep";
  if (measure_.active()) return "measuring";
  if (trick_playing()) return "trick";
  if (percepts_.picked_up()) return "picked-up";
  return nullptr;
}

// How Kilo is when the walker is idle: the posture asked (the alive layer's sitting moves are sitting).
guard::Still Mind::still() {
  if (walker_.active()) return guard::Still::None;
  const BodyState& b = link_.state();
  if (!b.has_telemetry) return guard::Still::None;
  if (player_.busy()) return player_.trick() == &alive_.sit_trick() ? guard::Still::Sitting : guard::Still::None;
  if (b.telemetry.mode != KILO_MODE_IDLE) return guard::Still::None;
  const Goal* g = link_.goal(KILO_MSG_SET_POSTURE);
  uint8_t posture = 0xFF;
  if (g != nullptr && g->len >= 2) posture = g->payload[1];
  else if (player_.trick() != nullptr && player_.result().state == tricks::PlayState::Done)
    posture = player_.trick()->end_posture;  // a trick cleared the goal: the body took its end posture
  else if (alive_.sitting() || sitting_variant_) posture = KILO_POSTURE_SIT;
  switch (posture) {
    case KILO_POSTURE_SIT: return guard::Still::Sitting;
    case KILO_POSTURE_STAND:
    case KILO_POSTURE_STANCE: return guard::Still::Standing;
    case KILO_POSTURE_REST:
    case KILO_POSTURE_LIMP: return guard::Still::Lying;
    default: return guard::Still::None;
  }
}

// BLUE while still: the head straight at it (the behaviour tree may look elsewhere again later).
void Mind::notice() {
  bool ok = client_look(link_, 0, 0);
  percepts_.notify_self_motion(seconds(this), kLookMoveS);
  logf(log_, LogLevel::Info, "guard: looks at it (the head ahead)%s", ok ? "" : ": refused");
}

// RED while sitting: the alive layer's lean back (a short trick on the legs, the head and the tail).
bool Mind::flinch() {
  percepts_.notify_self_motion(seconds(this), kTrickMoveS);
  if (player_.busy() && player_.trick() == &alive_.sit_trick()) {  // a sitting move under way: stopped, then the lean
    player_.stop();
    flinch_pending_ = true;
    logf(log_, LogLevel::Info, "guard: leans back (after the sitting move it stopped)");
    return true;
  }
  bool ok = alive_.flinch();
  logf(log_, ok ? LogLevel::Info : LogLevel::Warn, "guard: leans back%s", ok ? "" : ": not played");
  return ok;
}

// Every posture the mind asks for (the remote's B6, A7, the guard's sit, a walk's STANCE) moves the
// body: the IMU percepts stay quiet over it, as for the behaviour runner's (Body::posture). Without it
// a sit (26 degrees nose up in half a second) read as Kilo picked up, and picked up the guard stays
// out of it until he is put down calm at his old attitude: blind through the walk that followed.
void Mind::posture_moves() {
  const Goal* g = link_.goal(KILO_MSG_SET_POSTURE);
  uint16_t now = g == nullptr || g->len < 4 ? 0xFFFFu : (uint16_t)(g->payload[1] | (g->payload[3] << 8));
  if (now == posture_seen_) return;
  posture_seen_ = now;
  if (now != 0xFFFFu) percepts_.notify_self_motion(seconds(this), kPostureMoveS);
}

// While the mind walks and the behaviour runner is off, the head looks straight ahead: the sensor is
// on it, and a head left tilted or turned by the runner's last look (SET_HEAD is held until changed)
// sends the beam at the floor or to the side, and the guard would not see what is ahead.
void Mind::head_for_walk() {
  if (!walker_.active() || behaviour_active() || host_controls()) {
    head_centred_ = false;
    return;
  }
  if (head_centred_) return;
  head_centred_ = true;
  double pan = head_held(2), tilt = head_held(4);
  if (pan == 0 && tilt == 0) return;
  bool ok = client_look(link_, 0, 0);
  logf(log_, LogLevel::Info, "head: straight ahead for the walk (it was held at pan %d tilt %d)%s", (int)pan,
       (int)tilt, ok ? "" : ": refused");
}

// BLUE (after the stop) and RED (after backing off): sit, and the behaviour runner's sitting variant.
void Mind::sit_and_behave() {
  sitting_variant_ = true;
  if (behaviour_active()) {
    runner_.context().data.forget_goals();  // the tree re-asserts its posture, now as a sit
    bool ok = posture("sit", -1, "guard");
    logf(log_, LogLevel::Info, "guard: the behaviour runner goes on sitting%s", ok ? "" : " (SIT refused)");
    return;
  }
  const char* what = nullptr;
  bool ok = runner_hook_.start(&what);
  runner_by_guard_ = ok;
  if (ok) posture("sit", -1, "guard");
  logf(log_, ok ? LogLevel::Info : LogLevel::Warn, "guard: behaviour runner %s (sitting variant%s%s)",
       ok ? "started" : "not started", what != nullptr ? ": " : "", what != nullptr ? what : "");
}

// ---- the self-measurement, as the remote starts it (C5 twice) --------------------------------

const char* Mind::MeasureHookImpl::start() {
  uint16_t mv = 0;
  bool rested = m_.rested_mv(&mv);  // the battery at rest, as the tricks' refusal
  return m_.measure_.start(rested, mv);
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

// A5 while the runner runs: one of the alive layer's tricks (behaviour/alive.h).
bool Mind::RunnerHook::trick(const char** what) {
  return m_.alive_.request_trick((double)m_.clock_.now_us() / 1e6, what);
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
  out.has_battery = m_.rested_pct(&out.battery_pct);  // at rest: a walk's sag does not send him to REST
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
  bool ok = m_.posture(name, balance, "behaviour");
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

void Mind::Body::beep(const uint16_t (*)[2], int) { unsupported_beeps++; }  // the body has no beep from the mind: counted

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
