#include "remote/remote.h"
#include "behaviour/tree.h"
#include "remote/client.h"
#include "tricks/library.h"
#include "util/fmt.h"
#include "util/log.h"

namespace kilo {
namespace remote {

const double kSpeedFactors[kSpeedSteps] = {0.5, 0.7, 1.0, 1.4, 2.0};

namespace {
constexpr uint64_t kTrickStopWaitUs = 2000000;  // TRICK_STOP's ACK and the TRICK_DONE it brings
}

double stop_timeout_s(double period_ms) {
  if (period_ms <= 0) {  // the slowest pattern's cycle
    for (const planner::Pattern& p : planner::kPatternTable)
      if (p.period_ms > period_ms) period_ms = p.period_ms;
  }
  return 2 * period_ms / 1000.0 + 1.5;
}

RemoteControl::RemoteControl(SerialLink& link, planner::Walker& walker, tricks::TrickPlayer& player, Clock& clock,
                             Log& log, const RemoteConfig& cfg)
    : link_(link),
      walker_(walker),
      player_(player),
      clock_(clock),
      log_(log),
      cfg_(cfg),
      pattern_(planner::find_pattern("trot")) {}

bool RemoteControl::direction(planner::Direction* d) const {
  if (!walking_dir_) return false;
  *d = dir_;
  return true;
}

double RemoteControl::speed_mm_s() const {
  return kSpeedFactors[speed_step_] * (double)planner::default_speed(*pattern_);
}

int RemoteControl::mode() const {
  const BodyState& s = link_.state();
  return s.has_telemetry ? s.telemetry.mode : -1;
}

bool RemoteControl::walking() const { return walker_.active() || mode() == KILO_MODE_GAIT; }

bool RemoteControl::perform(const Key& key, uint16_t arg) {
  if (phase_ == Phase::Idle && n_ == 0) {
    begin(key, arg);
    return true;
  }
  if (n_ == kQueue) {
    char d[40];
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
  format(last_, sizeof last_, "%s", what);
  logf(log_, failed ? LogLevel::Warn : LogLevel::Info, "remote %s: %s", text_, last_);
}

bool RemoteControl::walk(planner::Direction d, const char** why) {
  planner::WalkGoal g;
  g.pattern = pattern_;
  g.direction = d;
  g.speed_mm_s = (planner::real)speed_mm_s();
  g.balance = true;
  *why = walker_.start(g);
  if (*why != nullptr) return false;
  walking_dir_ = true;
  dir_ = d;
  return true;
}

bool RemoteControl::rewalk(const char** why) {
  *why = nullptr;
  if (!walking_dir_ || !walker_.active()) return false;
  walk(dir_, why);
  return true;
}

void RemoteControl::begin(const Key& key, uint16_t arg) {
  key_ = &key;
  arg_ = arg;
  describe(arg, text_, sizeof text_);
  uint64_t now = clock_.now_us();
  char what[80];
  const char* why = nullptr;
  switch (key.kind) {
    case Kind::Walk: {
      if (mode() == KILO_MODE_TRICK) {  // ir.py _walk: a playing trick is stopped first
        player_.stop();
        phase_ = Phase::StopTrick;
        deadline_ = now + kTrickStopWaitUs;
        return;
      }
      act();
      return;
    }
    case Kind::GaitPattern: {
      const planner::Pattern* p = planner::find_pattern(key.name);
      if (p != nullptr) pattern_ = p;
      if (rewalk(&why)) {
        if (why != nullptr) format(what, sizeof what, "walk failed: %s", why);
        else format(what, sizeof what, "walk %s (%s)", dir_name_, pattern_->name);
      } else {
        format(what, sizeof what, "pattern %s", pattern_->name);
      }
      done(what, why != nullptr);
      return;
    }
    case Kind::Speed: {
      int s = speed_step_ + (key.name[0] == 'f' ? 1 : -1);  // faster / slower
      speed_step_ = s < 0 ? 0 : s >= kSpeedSteps ? kSpeedSteps - 1 : s;
      char factor[16];
      behaviour::format_g(factor, sizeof factor, kSpeedFactors[speed_step_]);
      planner::Timing t = planner::timing(*pattern_, (planner::real)speed_mm_s());
      bool re = rewalk(&why);
      char prefix[32] = "";
      if (re) format(prefix, sizeof prefix, "walk %s ", dir_name_);
      format(what, sizeof what, "%sspeed %sx (%s %d mm/s)%s%s", prefix, factor, pattern_->name,
             (int)(t.speed_mm_s + 0.5f), why != nullptr ? ": walk failed: " : "", why != nullptr ? why : "");
      done(what, why != nullptr);
      return;
    }
    case Kind::BalanceToggle: {
      bool on = !client_balance_enabled(link_);
      bool ok = client_set_balance(link_, on, cfg_.balance_kp);
      format(what, sizeof what, "balance %s%s", on ? "on" : "off", ok ? "" : " failed: the link refused SET_BALANCE");
      done(what, !ok);
      return;
    }
    case Kind::Posture:
    case Kind::Trick: {
      trick_ = nullptr;
      if (key.kind == Kind::Trick) {
        trick_ = tricks::find_trick(key.name);
        if (trick_ == nullptr) {
          format(what, sizeof what, "trick failed: no trick %s in the library", key.name);
          done(what, true);
          return;
        }
      }
      if (walking()) {  // the planned stop, waited for (ir.py _stop_walking)
        walker_.stop();
        double period = walker_.active() ? (double)walker_.planner().timing().period_ms : 0.0;
        phase_ = Phase::StopWalk;
        deadline_ = now + (uint64_t)(stop_timeout_s(period) * 1e6);
        return;
      }
      walking_dir_ = false;
      phase_ = Phase::StopWalk;
      deadline_ = now;
      step();
      return;
    }
    default:
      done("ignored", false);
      return;
  }
}

void RemoteControl::step() {
  uint64_t now = clock_.now_us();
  switch (phase_) {
    case Phase::Idle:
      return;
    case Phase::StopWalk: {
      if (walker_.active() && now < deadline_) return;
      if (walker_.active()) walker_.abort();  // client.posture() takes the legs anyway
      walking_dir_ = false;
      int m = mode();
      if (m == KILO_MODE_TRICK) {
        player_.stop();
        phase_ = Phase::StopTrick;
        deadline_ = now + kTrickStopWaitUs;
      } else if (key_->kind == Kind::Trick && m == KILO_MODE_LIMP) {  // a trick cannot start from LIMP
        client_posture(link_, &walker_, "stand");
        phase_ = Phase::Settle;
        deadline_ = now + (uint64_t)(cfg_.settle_s * 1e6);
      } else {
        act();
      }
      return;
    }
    case Phase::StopTrick:
      if (player_.busy() && now < deadline_) return;
      act();
      return;
    case Phase::Settle:
      if (now < deadline_) return;
      act();
      return;
  }
}

void RemoteControl::act() {
  phase_ = Phase::Idle;
  char what[80];
  const char* why = nullptr;
  switch (key_->kind) {
    case Kind::Posture: {
      bool ok = client_posture(link_, &walker_, key_->name);
      format(what, sizeof what, "posture %s%s", key_->name, ok ? "" : " failed: the link refused SET_POSTURE");
      done(what, !ok);
      return;
    }
    case Kind::Trick: {
      uint32_t play = player_.start(*trick_, false, tricks::kSpeedUnity, &why);
      if (play == 0) format(what, sizeof what, "trick failed: %s", why);
      else format(what, sizeof what, "trick %s", trick_->name);
      done(what, play == 0);
      return;
    }
    case Kind::Walk: {
      planner::Direction d;
      if (!planner::find_direction(key_->name, &d)) d = planner::Direction::Forward;
      if (walk(d, &why)) {
        dir_name_ = key_->name;
        format(what, sizeof what, "walk %s (%s)", key_->name, pattern_->name);
      } else {
        format(what, sizeof what, "walk failed: %s", why);
      }
      done(what, why != nullptr);
      return;
    }
    default:
      done("ignored", false);
      return;
  }
}

}  // namespace remote
}  // namespace kilo
