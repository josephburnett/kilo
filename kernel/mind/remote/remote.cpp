#include "remote/remote.h"
#include "behaviour/tree.h"
#include "remote/client.h"
#include "tricks/library.h"
#include "util/dmath.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/str.h"

namespace kilo {
namespace remote {

const double kSpeedFactors[kSpeedSteps] = {0.4, 0.7, 1.0, 1.15, 1.25};
const double kTrotSpeedFactors[kSpeedSteps] = {0.4, 0.7, 1.0, 1.25, 1.4};  // 57.6 .. 201.6 mm/s: 8, 8, 8, 10, 11-tick swings
const double kSteerLevels[kSteerSteps] = {0.25, 0.5, 1.0};

const double* speed_factors(const engine::Gait& p) {
  return str_eq(p.name, "trot") ? kTrotSpeedFactors : kSpeedFactors;
}

namespace {
constexpr uint64_t kTrickStopWaitUs = 2000000;  // a stopped trick's end (the body leaving the stream)
constexpr uint64_t kStepLeadUs = 300000;  // a lower stage's hold: down this long before it ends without A1
const char* const kStageTop[] = {"up", "mid", "top"};  // rear_up.py: each stage's top frame (its last guard)

// The frame of stage k's top in trick t (its guard named so), -1 if none.
int stage_frame(const tricks::Trick& t, int k) {
  for (uint8_t i = 0; i < t.n_guards; i++)
    if (str_eq(t.guards[i].name, kStageTop[k - 1])) return t.guards[i].frame;
  return -1;
}

int iabs(int v) { return v < 0 ? -v : v; }

// "55.6": one decimal, as the walker logs its speed
void format_speed(char* buf, uint32_t cap, double mm_s) { format_fixed(buf, cap, d_iround(mm_s * 10.0), 1); }

bool stop_key(const Key& k) { return k.kind == Kind::Posture && (str_eq(k.name, "stand") || str_eq(k.name, "rest")); }
bool calib_key(const Key& k) { return k.kind == Kind::Posture && str_eq(k.name, "calib"); }
}  // namespace

double stop_timeout_s(double period_ms) {
  if (period_ms <= 0) {  // the slowest gait's cycle at its default speed
    for (const engine::Gait& p : engine::kGaitTable) {
      double t = (double)engine::timing(p, engine::default_speed(p)).period_ms;
      if (t > period_ms) period_ms = t;
    }
  }
  return 2 * period_ms / 1000.0 + 3.5;  // walker.py stop_timeout(): the stride's ramp-down is up to 2 s
}

RemoteControl::RemoteControl(SerialLink& link, engine::Walker& walker, tricks::TrickPlayer& player, Clock& clock,
                             Log& log)
    : link_(link), walker_(walker), player_(player), clock_(clock), log_(log), pattern_(engine::find_gait("trot")) {}

bool RemoteControl::direction(engine::Direction* d) const {
  if (!walking_ || !walker_.active()) return false;
  *d = dir();
  return true;
}

double RemoteControl::speed_mm_s() const {
  return speed_factors(*pattern_)[iabs(level_) - 1] * (double)engine::default_speed(*pattern_);
}

// The steer levels allowed now: the trot's long swing (+4, +5) at most kTrotFastSteerSteps.
int RemoteControl::steer_most() const {
  return engine::is_trot(*pattern_) && level_ > kTrotLongFrom - 1 ? kTrotFastSteerSteps : kSteerSteps;
}

// The steer eased to what steer_most() allows (true if it was); before every walk the remote starts.
bool RemoteControl::limit_steer() {
  int most = steer_most();
  if (iabs(steer_) <= most) return false;
  steer_ = steer_ > 0 ? most : -most;
  return true;
}

void RemoteControl::key_seen(const Key& key) {
  if (!calib_key(key)) calib_held_ = false;
}

bool RemoteControl::calib_held() const {
  const Goal* g = const_cast<SerialLink&>(link_).goal(KILO_MSG_SET_POSTURE);
  return calib_held_ && g != nullptr && g->len >= 2 && g->payload[1] == KILO_POSTURE_CALIB;
}

double RemoteControl::steer_value() const {
  if (steer_ == 0 || heading_ != Heading::Straight) return 0;
  double v = kSteerLevels[iabs(steer_) - 1];
  return steer_ > 0 ? v : -v;
}

void RemoteControl::steer_text(char* buf, uint32_t cap) const {
  if (steer_ == 0 || heading_ != Heading::Straight) buf[0] = '\0';
  else format(buf, cap, ", steering %s %d", steer_ > 0 ? "left" : "right", iabs(steer_));
}

engine::WalkGoal RemoteControl::walk_goal() const {
  engine::WalkGoal g;
  g.gait = pattern_;
  g.direction = dir();
  g.speed_mm_s = (planner::real)speed_mm_s();
  g.balance = true;
  g.steer = (planner::real)steer_value();
  return g;
}

engine::Direction RemoteControl::dir() const {
  if (heading_ == Heading::Left) return engine::Direction::Left;
  if (heading_ == Heading::Right) return engine::Direction::Right;
  return level_ > 0 ? engine::Direction::Forward : engine::Direction::Back;
}

int RemoteControl::mode() const {
  const BodyState& s = link_.state();
  return s.has_telemetry ? s.telemetry.mode : -1;
}

// Anything on the legs to stop first: the walker, or the body still in mode STREAM.
bool RemoteControl::body_walking() const { return walker_.active() || mode() == KILO_MODE_STREAM; }

// The remote's own walk is going on; when it has ended (by a key or by
// itself), the level and heading go back to the start.
bool RemoteControl::walking() {
  if (walking_ && walker_.active()) return true;
  if (walking_) reset_motion();
  return false;
}

void RemoteControl::reset_motion() {
  walking_ = false;
  level_ = kDefaultLevel;
  heading_ = Heading::Straight;
  steer_ = 0;
}

bool RemoteControl::perform(const Key& key, uint16_t arg) {
  if (key.kind == Kind::Freeze) {  // at once, ahead of everything
    freeze(key, arg);
    return true;
  }
  if (phase_ == Phase::MeasureStand) done("measure cancelled (a key was pressed): standing", false);
  if (phase_ == Phase::Idle && n_ == 0) {
    begin(key, arg);
    return true;
  }
  if (n_ == kQueue) {
    char d[48];
    describe(arg, d, sizeof d);
    logf(log_, LogLevel::Warn, "remote %s: dropped (%d keys waiting)", d, n_);
    return false;
  }
  q_[(head_ + n_++) % kQueue] = {&key, arg};
  return true;
}

void RemoteControl::poll() {
  step();
  step_gate();
  while (phase_ == Phase::Idle && n_ > 0) {
    Pending p = q_[head_];
    head_ = (head_ + 1) % kQueue;
    n_--;
    begin(*p.key, p.arg);
  }
}

void RemoteControl::done(const char* what, bool failed) {
  phase_ = Phase::Idle;
  last_failed_ = failed;
  actions_++;
  if (failed) failures_++;
  format(last_, sizeof last_, "%s%s", note_, what);
  logf(log_, failed ? LogLevel::Warn : LogLevel::Info, "remote %s: %s", text_, last_);
}

// The walk as it now is ("walk forward (trot) 55.6 mm/s, steering left 1").
void RemoteControl::walk_done(const char* suffix, const char* prefix) {
  char what[112], v[16], st[24];
  engine::Timing t = engine::timing(*pattern_, (planner::real)speed_mm_s(), dir());
  format_speed(v, sizeof v, (double)t.speed_mm_s);
  steer_text(st, sizeof st);
  format(what, sizeof what, "%swalk %s (%s) %s mm/s%s%s", prefix, engine::direction_name(dir()), pattern_->name, v,
         st, suffix);
  done(what, false);
}

void RemoteControl::freeze(const Key& key, uint16_t arg) {
  char d[48];
  if (phase_ != Phase::Idle) {  // the action under way: cancelled (its posture or walk never goes out)
    logf(log_, LogLevel::Info, "remote %s: cancelled (C7 froze Kilo)", text_);
    phase_ = Phase::Idle;
  }
  for (; n_ > 0; head_ = (head_ + 1) % kQueue, n_--) {
    describe(q_[head_].arg, d, sizeof d);
    logf(log_, LogLevel::Info, "remote %s: dropped (C7 froze Kilo)", d);
  }
  key_ = &key;
  arg_ = arg;
  note_ = "";
  describe(arg, text_, sizeof text_);
  if (frozen_) {
    done("ignored (frozen: B6 or A7 to go on)", false);
    return;
  }
  bool trick = player_.busy();
  char what[96] = "frozen (";
  uint32_t n = 8;
  auto part = [&](const char* s) {
    n += format(what + n, sizeof what - n, "%s%s", n > 8 ? ", " : "", s);
  };
  if (walker_.active()) {  // the quick stop: the feet down where they are, the legs squared up, STANCE
    walker_.halt();
    part("walk: the quick stop");
  }
  if (measure_ != nullptr && measure_->active()) {  // into STAND (docs/self-measurement.md)
    measure_->stop(text_);
    part("measurement stopped: standing");
  }
  if (behaviour_ != nullptr && behaviour_->active()) {  // its own trick is cancelled as it stops
    behaviour_->stop();
    part("behaviour runner stopped");
  }
  if (trick) {
    stop_trick();  // over its stop_ticks: rear_up comes down to SIT
    part("trick stopped");
  }
  if (n == 8) part("holding the posture");
  format(what + n, sizeof what - n, "): only B6 or A7 act now");
  reset_motion();
  frozen_ = true;
  calib_held_ = false;
  done(what, false);
}

void RemoteControl::begin(const Key& key, uint16_t arg) {
  key_ = &key;
  arg_ = arg;
  note_ = "";
  describe(arg, text_, sizeof text_);
  uint64_t now = clock_.now_us();
  if (frozen_) {
    if (!stop_key(key)) {
      done("ignored (frozen: B6 or A7 to go on)", false);
      return;
    }
    frozen_ = false;
    note_ = "unfrozen, ";
  }
  const bool second_c5 = calib_key(key) && calib_held();  // C5 again, no key between: the self-measurement
  calib_held_ = false;
  if (measure_ != nullptr && measure_->active()) {  // every key stops it first (into STAND)
    measure_->stop(text_);
    if (calib_key(key)) {
      done("measurement stopped: standing", false);
      return;
    }
    if (!stop_key(key)) {
      done("measurement stopped: standing (only B6, A7 and C7 act after it)", false);
      return;
    }
    note_ = "measurement stopped, ";
  }
  if (behaviour_ != nullptr && behaviour_->active()) {
    if (key.kind == Kind::Behaviour) {  // A5 again: one of the random mode's tricks (behaviour/alive.h)
      const char* what = nullptr;
      bool ok = behaviour_->trick(&what);
      done(what != nullptr ? what : ok ? "random mode: a trick" : "random mode: no trick", !ok);
      return;
    }
    if (!stop_key(key) && !behaviour_->yields(key)) {
      done("ignored (behaviour runner active)", false);
      return;
    }
    behaviour_->stop();
    note_ = "behaviour runner stopped, ";
  }
  switch (key.kind) {
    case Kind::Posture:
      measure_next_ = second_c5;
      stop_then_act(now);
      return;
    case Kind::Behaviour:
    case Kind::Righting:
      stop_then_act(now);
      return;
    case Kind::Trick:
      if (player_.busy()) {
        uint64_t end = 0;
        int k = step_at(&end);
        if (k != 0 && k == step_allowed_ && k < step_max_) {  // the rear-up: on to the next stage
          step_allowed_ = k + 1;
          if (notes_ != nullptr) notes_->trick_stage(step_play_, step_allowed_, step_reached_, step_max_);
          char what[96];
          format(what, sizeof what, "rear_up: on to stage %d (of %d)", k + 1, step_max_);
          step_note(what);
          done(what, false);
          return;
        }
        stop_trick();  // the toggle: a second press brings it down
        done("trick stopped: coming down", false);
        return;
      }
      stop_then_act(now);
      return;
    case Kind::Gait: {
      const engine::Gait* p = engine::find_gait(key.name);
      if (p == nullptr) {
        done("walk failed: no such gait", true);
        return;
      }
      if (walking()) {
        bool same = p == pattern_;
        int was_level = level_, was_steer = steer_;
        Heading was_heading = heading_;
        pattern_ = p;
        if (same) {  // that gait already: straight forward at the level's magnitude, no steer
          heading_ = Heading::Straight;
          level_ = iabs(level_);
          steer_ = 0;
          if (level_ == was_level && heading_ == was_heading && steer_ == was_steer) {
            walk_done(": unchanged");
            return;
          }
        }
        act();
        return;
      }
      reset_motion();
      pattern_ = p;
      break;  // a trick first, then the walk (act)
    }
    case Kind::Turn: {
      Heading h = str_eq(key.name, "left") ? Heading::Left : Heading::Right;
      if (walking()) {
        if (heading_ != Heading::Straight) {  // turning in place: the other key turns the other way
          if (heading_ == h) {
            walk_done(": unchanged");
            return;
          }
          heading_ = h;
          act();
          return;
        }
        // walking forward or backward: steer a level that way, without stopping (the trot at +4, +5: one)
        int most = steer_most();
        int sv = h == Heading::Left ? (steer_ < most ? steer_ + 1 : steer_) : (steer_ > -most ? steer_ - 1 : steer_);
        if (sv == steer_) {
          walk_done(most < kSteerSteps ? ": already the tightest at this speed (B5 for more)" : ": already the tightest");
          return;
        }
        steer_ = sv;
        act();
        return;
      }
      reset_motion();
      heading_ = h;
      break;
    }
    case Kind::Speed: {
      if (!walking()) {
        done("ignored (not walking)", false);
        return;
      }
      bool up = str_eq(key.name, "faster");
      if (heading_ != Heading::Straight) {  // turning in place: straight forward (B7) or backward (B5) at once
        heading_ = Heading::Straight;
        level_ = up ? iabs(level_) : -iabs(level_);
        steer_ = 0;
        const char* why = walker_.start(walk_goal());
        if (why != nullptr) {  // refused: the walk's state starts over
          reset_motion();
          char what[96];
          format(what, sizeof what, "turn ended: walk failed: %s", why);
          done(what, true);
          return;
        }
        walk_done("", "turn ended: ");
        return;
      }
      int l = level_;
      int back_most = engine::is_trot(*pattern_) ? kTrotBackSteps : kSpeedSteps;  // the trot backward: its 8-tick swing
      if (up) {
        l = l == -1 ? 1 : l < kSpeedSteps ? l + 1 : l;
      } else {
        l = l == 1 ? -1 : l > -back_most ? l - 1 : l;
      }
      const char* end = l == level_ ? (up ? ": already the fastest" : ": already the fastest back") : "";
      level_ = l;
      const char* eased = limit_steer() ? ", the steer eased to 1 (the trot's long swing)" : "";
      const char* why = nullptr;
      if (end[0] == '\0') why = walker_.start(walk_goal());
      char factor[16], v[16], st[24], what[160];
      behaviour::format_g(factor, sizeof factor, speed_factors(*pattern_)[iabs(level_) - 1]);
      engine::Timing t = engine::timing(*pattern_, (planner::real)speed_mm_s(), dir());
      format_speed(v, sizeof v, (double)t.speed_mm_s);
      steer_text(st, sizeof st);
      format(what, sizeof what, "speed %s%sx (%s %s mm/s %s%s)%s%s%s%s", level_ < 0 ? "-" : "", factor, pattern_->name,
             v, engine::direction_name(dir()), st, end, eased, why != nullptr ? ": walk failed: " : "",
             why != nullptr ? why : "");
      done(what, why != nullptr);
      return;
    }
    default:
      done("ignored (no action on this key)", false);
      return;
  }
  // A walk from standing: a playing trick is stopped first
  if (player_.busy()) {
    stop_trick();
    phase_ = Phase::StopTrick;
    deadline_ = now + kTrickStopWaitUs;
    return;
  }
  act();
}

// The quick stop of a walk (planner/plan.h halt()), waited for, then a trick
// stopped, then act().
void RemoteControl::stop_then_act(uint64_t now) {
  reset_motion();
  if (body_walking()) {
    walker_.halt();
    double period = walker_.active() ? (double)walker_.solver().timing().period_ms : 0.0;
    phase_ = Phase::StopWalk;
    deadline_ = now + (uint64_t)(stop_timeout_s(period) * 1e6);
    return;
  }
  phase_ = Phase::StopWalk;
  deadline_ = now;
  step();
}

void RemoteControl::step() {
  uint64_t now = clock_.now_us();
  switch (phase_) {
    case Phase::Idle:
      return;
    case Phase::StopWalk: {
      if (walker_.active() && now < deadline_) return;
      if (walker_.active()) walker_.abort();  // client.posture() takes the legs anyway
      if (player_.busy()) {
        stop_trick();
        phase_ = Phase::StopTrick;
        deadline_ = now + kTrickStopWaitUs;
      } else {
        act();
      }
      return;
    }
    case Phase::StopTrick:
      if (player_.busy() && now < deadline_) return;
      if (settle_us_ != 0) {  // the play ends as the transition starts: wait for its end
        phase_ = Phase::Settle;
        deadline_ = now + settle_us_;
        settle_us_ = 0;
        return;
      }
      act();
      return;
    case Phase::Settle:
      if (now < deadline_) return;
      act();
      return;
    case Phase::MeasureStand: {  // the second C5: standing, then the self-measurement once he is still
      if (now < measure_from_ + kMeasureSettleUs) return;
      const char* why = measure_ != nullptr ? measure_->start() : "no self-measurement";
      char what[112];
      if (why == nullptr) {
        done("standing, measuring (about 80 s; any key stops it)", false);
      } else if (now >= deadline_) {
        format(what, sizeof what, "standing, measure refused: %s", why);
        done(what, true);
      }
      return;
    }
  }
}

void RemoteControl::stop_trick() {
  uint8_t ticks = player_.stop_ticks();
  if (step_trick_ != nullptr && player_.busy() && player_.trick() == step_trick_) {
    char what[64];
    format(what, sizeof what, "rear_up: coming down (stage %d allowed)", step_allowed_);
    step_note(what);
  }
  step_trick_ = nullptr;
  player_.stop();
  settle_us_ = ticks != 0 ? ticks_to_us(ticks) + 100000u : 0;  // the crossfade + 0.1 s
}

void RemoteControl::start_trick() {
  const BodyState& s = link_.state();
  char what[96];
  if (s.has_telemetry && (s.telemetry.flags & KILO_TELEMETRY_FLAGS_NO_BATTERY) != 0) {
    format(what, sizeof what, "%s refused: on USB power (no battery)", key_->name);
    if (notes_ != nullptr) notes_->refused(key_->name, "usb_power (no battery)");
    done(what, true);
    return;
  }
  // the battery at rest (the highest reading of the last 20 s): right after a walk the reading sags
  uint16_t mv = s.has_telemetry ? s.telemetry.batt_mv : 0;
  uint16_t rested = 0;
  bool at_rest = notes_ != nullptr && notes_->rested_mv(&rested);
  if (at_rest) mv = rested;
  if (!s.has_telemetry || mv < kTrickMinMv) {
    if (s.has_telemetry) format(what, sizeof what, "%s refused: battery %u mV%s (needs %u)", key_->name, (unsigned)mv,
                                at_rest ? " at rest" : "", (unsigned)kTrickMinMv);
    else format(what, sizeof what, "%s refused: battery None mV (needs %u)", key_->name, (unsigned)kTrickMinMv);
    if (notes_ != nullptr) {
      char why[48];
      format(why, sizeof why, "battery %u mV%s (needs %u)", (unsigned)mv, at_rest ? " at rest" : "",
             (unsigned)kTrickMinMv);
      notes_->refused(key_->name, why);
    }
    done(what, true);
    return;
  }
  const tricks::Trick* t = tricks::find_trick(key_->name);
  const char* why = "no such trick";
  if (t == nullptr || player_.start(*t, false, tricks::kSpeedUnity, &why) == 0) {
    format(what, sizeof what, "%s failed: %s", key_->name, why);
    if (notes_ != nullptr) notes_->refused(key_->name, why);
    done(what, true);
    return;
  }
  uint32_t ticks = 0;
  tricks::total_ticks(*t, tricks::kSpeedUnity, &ticks);
  int stages = (int)tricks::rear_up_stage(t->name);
  step_trick_ = stages > 0 ? t : nullptr;
  step_play_ = player_.result().play;
  step_max_ = stages;
  step_allowed_ = 1;
  step_held_ = 0;
  step_reached_ = 0;
  if (stages > 0 && notes_ != nullptr) notes_->trick_stage(step_play_, step_allowed_, step_reached_, step_max_);
  if (stages > 1) {
    format(what, sizeof what, "trick %s started: up to stage 1 (A1 while he holds it: on, up to %d; B6 or C7 bring "
           "him down)", key_->name, stages);
    char line[64];
    format(line, sizeof line, "rear_up: up to stage 1 (of %d)", stages);
    step_note(line);
  } else {
    format(what, sizeof what, "trick %s started (%u s; A1 again, B6 or C7 bring it down)", key_->name,
           (unsigned)((ticks * KILO_TICK_MS + 500u) / 1000u));
  }
  done(what, false);
}

void RemoteControl::step_note(const char* what) {
  logf(log_, LogLevel::Info, "remote: %s", what);
  if (notes_ != nullptr) notes_->trick_mark(what);
}

// The stage top (1..) the rear-up play is going up to or holding now, with when its hold ends (us on
// the clock); 0 if none (another moment of the play, or not this play).
int RemoteControl::step_at(uint64_t* hold_end_us) const {
  if (step_trick_ == nullptr || !player_.playing() || player_.trick() != step_trick_ ||
      player_.result().play != step_play_)
    return 0;
  const tricks::Trick& t = *step_trick_;
  uint64_t now = clock_.now_us(), t0 = player_.played_at_us();
  for (int k = 1; k <= step_max_; k++) {
    int f = stage_frame(t, k);
    if (f < 0) continue;
    uint32_t start = 0;
    for (int i = 0; i < f; i++) start += t.frames[i].dur + t.frames[i].hold;
    uint64_t from = t0 + ticks_to_us(start);
    uint64_t end = from + ticks_to_us(t.frames[f].dur + t.frames[f].hold);
    if (now >= from && now < end) {
      *hold_end_us = end;
      return k;
    }
  }
  return 0;
}

// Every pass: a lower stage's hold about to end without A1: down (stopped); a stage's top reached
// or the play over: a line and a mark.
void RemoteControl::step_gate() {
  if (step_trick_ == nullptr) return;
  if (!player_.busy() || player_.trick() != step_trick_ || player_.result().play != step_play_) {
    const tricks::PlayResult& r = player_.result();
    if (r.play == step_play_ && r.state == tricks::PlayState::Done) {
      char what[80];
      format(what, sizeof what, "rear_up: down (the trick ended %s, stage %d allowed)", tricks::done_reason_name(r.reason),
             step_allowed_);
      step_note(what);
    }
    step_trick_ = nullptr;
    return;
  }
  uint64_t end = 0;
  int k = step_at(&end);
  if (k == 0) return;
  if (k != step_held_) {
    step_held_ = k;
    if (k > step_reached_) {
      step_reached_ = k;
      if (notes_ != nullptr) notes_->trick_stage(step_play_, step_allowed_, step_reached_, step_max_);
    }
    char what[64];
    format(what, sizeof what, "rear_up: stage %d (of %d)%s", k, step_max_,
           k < step_max_ && step_allowed_ == k ? ": A1 to go on" : "");
    step_note(what);
  }
  if (k < step_max_ && step_allowed_ == k && clock_.now_us() + kStepLeadUs >= end) {
    char what[72];
    format(what, sizeof what, "rear_up: held stage %d without A1: coming down", k);
    step_note(what);
    step_trick_ = nullptr;  // (stop_trick's own note would repeat it)
    player_.stop();  // over the trick's stop_ticks, to SIT
  }
}

bool RemoteControl::take_posture(const char* name) {
  if (posture_hook_ != nullptr) {
    char who[24];
    format(who, sizeof who, "remote %s", key_->grid);
    return posture_hook_->posture(name, who);
  }
  return client_posture(link_, &walker_, name);
}

void RemoteControl::act() {
  phase_ = Phase::Idle;
  char what[96];
  const char* why = nullptr;
  switch (key_->kind) {
    case Kind::Posture: {
      if (measure_next_) {  // the second C5: stand (the routine starts from STAND), then measure (step())
        measure_next_ = false;
        if (!take_posture("stand")) {
          done("measure failed: the link refused SET_POSTURE stand", true);
          return;
        }
        phase_ = Phase::MeasureStand;
        measure_from_ = clock_.now_us();
        deadline_ = measure_from_ + kMeasureWaitUs;
        logf(log_, LogLevel::Info, "remote %s: standing, then the self-measurement", text_);
        return;
      }
      bool ok = take_posture(key_->name);
      calib_held_ = ok && calib_key(*key_);
      format(what, sizeof what, "posture %s%s%s", key_->name, ok ? "" : " failed: the link refused SET_POSTURE",
             calib_held_ ? " (C5 again: measure himself)" : "");
      done(what, !ok);
      return;
    }
    case Kind::Trick:
      start_trick();
      return;
    case Kind::Behaviour: {
      why = "no behaviour runner";
      bool ok = behaviour_ != nullptr && behaviour_->start(&why);
      if (ok && why != nullptr) format(what, sizeof what, "behaviour runner started (%s)", why);
      else if (ok) format(what, sizeof what, "behaviour runner started");
      else format(what, sizeof what, "behaviour runner failed: %s", why);
      done(what, !ok);
      return;
    }
    case Kind::Righting: {
      why = righting_ != nullptr ? righting_->start() : "no righting";
      if (why == nullptr) format(what, sizeof what, "righting (the experiment; any key but B1 stops it)");
      else format(what, sizeof what, "righting refused: %s", why);
      done(what, why != nullptr);
      return;
    }
    case Kind::Gait:
    case Kind::Turn: {
      limit_steer();
      why = walker_.start(walk_goal());
      if (why != nullptr) {
        reset_motion();
        format(what, sizeof what, "walk failed: %s", why);
        done(what, true);
        return;
      }
      walking_ = true;
      walk_done("");
      return;
    }
    default:
      done("ignored", false);
      return;
  }
}

}  // namespace remote
}  // namespace kilo
