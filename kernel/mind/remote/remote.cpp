#include "remote/remote.h"
#include "behaviour/tree.h"
#include "remote/client.h"
#include "tricks/library.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/str.h"

namespace kilo {
namespace remote {

const double kSpeedFactors[kSpeedSteps] = {0.4, 0.7, 1.0, 1.15, 1.25};
const double kTrotSpeedFactors[kSpeedSteps] = {0.4, 0.7, 1.0, 1.3, 1.6};  // 57.6 .. 230.4 mm/s
const double kSteerLevels[kSteerSteps] = {0.25, 0.5, 1.0};

const double* speed_factors(const planner::Pattern& p) {
  return str_eq(p.name, "trot") ? kTrotSpeedFactors : kSpeedFactors;
}

namespace {
constexpr uint64_t kTrickStopWaitUs = 2000000;  // TRICK_STOP's ACK and the TRICK_DONE it brings

int iabs(int v) { return v < 0 ? -v : v; }

// "55.6": one decimal, as the walker logs its speed
void format_speed(char* buf, uint32_t cap, double mm_s) {
  int v10 = (int)(mm_s * 10.0 + 0.5);
  format(buf, cap, "%d.%d", v10 / 10, v10 % 10);
}

bool stop_key(const Key& k) { return k.kind == Kind::Posture && (str_eq(k.name, "stand") || str_eq(k.name, "rest")); }
}  // namespace

double stop_timeout_s(double period_ms) {
  if (period_ms <= 0) {  // the slowest pattern's cycle
    for (const planner::Pattern& p : planner::kPatternTable)
      if (p.period_ms > period_ms) period_ms = p.period_ms;
  }
  return 2 * period_ms / 1000.0 + 3.5;  // walker.py stop_timeout(): the stride's ramp-down is up to 2 s
}

RemoteControl::RemoteControl(SerialLink& link, planner::Walker& walker, tricks::TrickPlayer& player, Clock& clock,
                             Log& log)
    : link_(link), walker_(walker), player_(player), clock_(clock), log_(log), pattern_(planner::find_pattern("trot")) {}

bool RemoteControl::direction(planner::Direction* d) const {
  if (!walking_ || !walker_.active()) return false;
  *d = dir();
  return true;
}

double RemoteControl::speed_mm_s() const {
  return speed_factors(*pattern_)[iabs(level_) - 1] * (double)planner::default_speed(*pattern_);
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

planner::WalkGoal RemoteControl::walk_goal() const {
  planner::WalkGoal g;
  g.pattern = pattern_;
  g.direction = dir();
  g.speed_mm_s = (planner::real)speed_mm_s();
  g.balance = true;
  g.steer = (planner::real)steer_value();
  return g;
}

planner::Direction RemoteControl::dir() const {
  if (heading_ == Heading::Left) return planner::Direction::Left;
  if (heading_ == Heading::Right) return planner::Direction::Right;
  return level_ > 0 ? planner::Direction::Forward : planner::Direction::Back;
}

int RemoteControl::mode() const {
  const BodyState& s = link_.state();
  return s.has_telemetry ? s.telemetry.mode : -1;
}

// Anything on the legs to stop first: the walker, or the body still in GAIT.
bool RemoteControl::body_walking() const { return walker_.active() || mode() == KILO_MODE_GAIT; }

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
  planner::Timing t = planner::timing(*pattern_, (planner::real)speed_mm_s());
  format_speed(v, sizeof v, (double)t.speed_mm_s);
  steer_text(st, sizeof st);
  format(what, sizeof what, "%swalk %s (%s) %s mm/s%s%s", prefix, planner::direction_name(dir()), pattern_->name, v,
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
  bool trick = mode() == KILO_MODE_TRICK || player_.busy();
  char what[96] = "frozen (";
  uint32_t n = 8;
  auto part = [&](const char* s) {
    n += format(what + n, sizeof what - n, "%s%s", n > 8 ? ", " : "", s);
  };
  if (walker_.active()) {  // no planned stop: the body holds, settles the swing feet down, stays in STANCE
    walker_.abort("aborted (C7 freeze)", true);
    part("walk aborted");
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
  if (behaviour_ != nullptr && behaviour_->active()) {
    if (key.kind == Kind::Behaviour) {
      done("behaviour runner already running", false);
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
    case Kind::Behaviour:
      stop_then_act(now);
      return;
    case Kind::Trick:
      if (mode() == KILO_MODE_TRICK || player_.busy()) {  // the toggle: a second press brings it down
        stop_trick();
        done("trick stopped: coming down", false);
        return;
      }
      stop_then_act(now);
      return;
    case Kind::Gait: {
      const planner::Pattern* p = planner::find_pattern(key.name);
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
        // walking forward or backward: steer a level that way, without stopping
        int sv = h == Heading::Left ? (steer_ < kSteerSteps ? steer_ + 1 : steer_)
                                    : (steer_ > -kSteerSteps ? steer_ - 1 : steer_);
        if (sv == steer_) {
          walk_done(": already the tightest");
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
        if (why != nullptr) {  // ir.py _go: the walk's state starts over
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
      if (up) {
        l = l == -1 ? 1 : l < kSpeedSteps ? l + 1 : l;
      } else {
        l = l == 1 ? -1 : l > -kSpeedSteps ? l - 1 : l;
      }
      const char* end = l == level_ ? (up ? ": already the fastest" : ": already the fastest back") : "";
      level_ = l;
      const char* why = nullptr;
      if (end[0] == '\0') why = walker_.start(walk_goal());
      char factor[16], v[16], st[24], what[128];
      behaviour::format_g(factor, sizeof factor, speed_factors(*pattern_)[iabs(level_) - 1]);
      planner::Timing t = planner::timing(*pattern_, (planner::real)speed_mm_s());
      format_speed(v, sizeof v, (double)t.speed_mm_s);
      steer_text(st, sizeof st);
      format(what, sizeof what, "speed %s%sx (%s %s mm/s %s%s)%s%s%s", level_ < 0 ? "-" : "", factor, pattern_->name,
             v, planner::direction_name(dir()), st, end, why != nullptr ? ": walk failed: " : "",
             why != nullptr ? why : "");
      done(what, why != nullptr);
      return;
    }
    default:
      done("ignored (no action on this key)", false);
      return;
  }
  // A walk from standing: a playing trick is stopped first (ir.py _walk)
  if (mode() == KILO_MODE_TRICK) {
    stop_trick();
    phase_ = Phase::StopTrick;
    deadline_ = now + kTrickStopWaitUs;
    return;
  }
  act();
}

// The planned stop of a walk, waited for (ir.py _stop_walking), then a trick stopped, then act().
void RemoteControl::stop_then_act(uint64_t now) {
  reset_motion();
  if (body_walking()) {
    walker_.stop();
    double period = walker_.active() ? (double)walker_.planner().timing().period_ms : 0.0;
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
      if (mode() == KILO_MODE_TRICK) {
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
      if (settle_us_ != 0) {  // TRICK_DONE comes as the crossfade starts: wait for its end
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
  }
}

void RemoteControl::stop_trick() {
  uint8_t ticks = player_.stop_ticks();
  player_.stop();
  settle_us_ = ticks != 0 ? (uint64_t)ticks * KILO_TICK_MS * 1000u + 100000u : 0;  // ir.py _stop_trick
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
  if (!s.has_telemetry || s.telemetry.batt_mv < kTrickMinMv) {
    if (s.has_telemetry) format(what, sizeof what, "%s refused: battery %u mV (needs %u)", key_->name,
                                (unsigned)s.telemetry.batt_mv, (unsigned)kTrickMinMv);
    else format(what, sizeof what, "%s refused: battery None mV (needs %u)", key_->name, (unsigned)kTrickMinMv);
    if (notes_ != nullptr) {
      char why[48];
      format(why, sizeof why, "battery %u mV (needs %u)", s.has_telemetry ? (unsigned)s.telemetry.batt_mv : 0u,
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
  format(what, sizeof what, "trick %s started (%u s; A2 again, B6 or C7 bring it down)", key_->name,
         (unsigned)((ticks * KILO_TICK_MS + 500u) / 1000u));
  done(what, false);
}

void RemoteControl::act() {
  phase_ = Phase::Idle;
  char what[96];
  const char* why = nullptr;
  switch (key_->kind) {
    case Kind::Posture: {
      bool ok = client_posture(link_, &walker_, key_->name);
      format(what, sizeof what, "posture %s%s", key_->name, ok ? "" : " failed: the link refused SET_POSTURE");
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
    case Kind::Gait:
    case Kind::Turn: {
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
