#include "power/sleep.h"
#include "remote/client.h"
#include "remote/keys.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/str.h"

namespace kilo {
namespace power {

namespace {

// The battery trend's state: asleep or not (Drowsy is 30 s of awake).
uint8_t trend_state(State s) { return s == State::Asleep ? (uint8_t)State::Asleep : (uint8_t)State::Awake; }

// "12 min 3 s", "45 s", "2 h 5 min"
void duration(char* out, uint32_t n, uint64_t us) {
  unsigned s = (unsigned)(us / 1000000u);
  if (s >= 3600) format(out, n, "%u h %u min", s / 3600, s / 60 % 60);
  else if (s >= 60) format(out, n, "%u min %u s", s / 60, s % 60);
  else format(out, n, "%u s", s);
}

// mV/min x 10 as "-8.4", "?" for none
void tenths(char* out, uint32_t n, int32_t x10) {
  if (x10 == BatteryTrend::kNoTrend) {
    format(out, n, "?");
    return;
  }
  int32_t a = x10 < 0 ? -x10 : x10;
  format(out, n, "%s%d.%d", x10 < 0 ? "-" : "", (int)(a / 10), (int)(a % 10));
}

}  // namespace

const char* state_name(State s) {
  switch (s) {
    case State::Awake: return "awake";
    case State::Drowsy: return "drowsy";
    case State::Asleep: return "asleep";
  }
  return "?";
}

const char* net_name(Net n) {
  switch (n) {
    case Net::On: return "on";
    case Net::Save: return "save";
    case Net::Look: return "look";
  }
  return "?";
}

Sleep::Sleep(SerialLink& link, planner::Walker& walker, Clock& clock, Log& log)
    : link_(link), walker_(walker), clock_(clock), log_(log) {}

uint64_t Sleep::time_in_us(State s) const {
  uint64_t t = time_us_[(int)s];
  if (s == state_) t += clock_.now_us() - since_us_;
  return t;
}

void Sleep::set_state(State s, uint64_t now) {
  if (s == state_) return;
  bool was_asleep = asleep();
  time_us_[(int)state_] += now - since_us_;
  since_us_ = now;
  state_ = s;
  if (was_asleep != asleep()) battery_.state_changed(now, trend_state(s));
}

bool Sleep::moving_mode() const {
  const BodyState& b = link_.state();
  if (!b.has_telemetry) return false;
  uint8_t m = b.telemetry.mode;
  return m == KILO_MODE_GAIT || m == KILO_MODE_TRICK || m == KILO_MODE_FALLEN || m == KILO_MODE_RECOVERING ||
         m == KILO_MODE_CALIB || m == KILO_MODE_BOOT;
}

void Sleep::on_telemetry(const kilo_telemetry_t& t) {
  if (t.flags & KILO_TELEMETRY_FLAGS_NO_BATTERY) return;
  battery_.add(clock_.now_us(), t.batt_mv, trend_state(state_));
}

void Sleep::key(uint16_t arg, bool acts) {
  if (!enabled_) return;
  uint64_t now = clock_.now_us();
  // null for a held key's repeat, another remote's key, or a key the mind does not act on now
  const remote::Key* k = acts ? remote::action_for(arg) : nullptr;
  bool a7 = k != nullptr && k->kind == remote::Kind::Posture && str_eq(k->name, "rest");
  bool moves = k != nullptr && !a7 &&
               (k->kind == remote::Kind::Posture || k->kind == remote::Kind::Gait || k->kind == remote::Kind::Turn ||
                k->kind == remote::Kind::Behaviour || k->kind == remote::Kind::Trick);
  char why[24];
  format(why, sizeof why, "key:%s%s", k != nullptr ? k->grid : "?", remote::key_repeat(arg) ? "_repeat" : "");
  if (k == nullptr && !remote::key_repeat(arg)) {
    const remote::Key* any = remote::key_for(remote::key_command(arg));
    if (any != nullptr) format(why, sizeof why, "key:%s", any->grid);
  }
  if (a7) {
    if (state_ == State::Asleep) wake(State::Drowsy, why, now);
    else if (state_ == State::Awake) {
      set_state(State::Drowsy, now);
      logf(log_, LogLevel::Info, "power: A7: light sleep in %u s unless something happens",
           (unsigned)(kSleepAfterUs / 1000000u));
    }
    quiet_since_us_ = now;
    flush_asked_ = false;
    return;
  }
  if (moves) {
    if (state_ == State::Awake) return;
    if (state_ == State::Asleep && k->kind != remote::Kind::Posture) {
      // a walk, a trick or the runner from LIMP: the servos take REST first (TRICK_PLAY is BUSY in LIMP)
      client_posture(link_, &walker_, "rest");
    }
    if (state_ == State::Asleep) wake(State::Awake, why, now);
    else {
      set_state(State::Awake, now);
      logf(log_, LogLevel::Info, "power: light sleep called off (%s moves Kilo)", why);
    }
    return;
  }
  // a key that does nothing here: activity, the count starts again
  if (state_ == State::Awake) return;
  if (state_ == State::Asleep) wake(State::Drowsy, why, now);
  quiet_since_us_ = now;
  flush_asked_ = false;
}

void Sleep::poll(const Activity& a) {
  uint64_t now = clock_.now_us();
  bool host = host_seen_ && a.host_events != host_events_;
  host_events_ = a.host_events;
  host_seen_ = true;
  if (hw_ != nullptr && now >= soc_next_us_) {
    soc_next_us_ = now + kSocPeriodUs;
    soc_ok_ = hw_->soc(&soc_);
  }
  if (!enabled_) return;
  switch (state_) {
    case State::Awake:
      break;
    case State::Drowsy: {
      if (a.busy || host || a.host_clients > 0 || moving_mode()) {
        quiet_since_us_ = now;
        flush_asked_ = false;
        break;
      }
      if (now - quiet_since_us_ < kSleepAfterUs) break;
      // The card saver first: what it holds onto the card, the save waited for.
      if (saver_ != nullptr && saver_->save_on()) {
        if (!flush_asked_) {
          saver_->set_mode(2);  // REC_SAVE now: it starts at its next poll (the next pass)
          flush_asked_ = true;
          flush_at_us_ = now;
          flushes_++;
          break;
        }
        if (saver_->state() == KILO_RECORD_CARD_STATE_SAVING && now - flush_at_us_ < kFlushWaitUs) break;
      }
      enter(now);
      break;
    }
    case State::Asleep:
      if (host || a.host_clients > 0) wake(State::Drowsy, "laptop", now);
      else if (a.busy || moving_mode()) wake(State::Drowsy, "busy", now);
      break;
  }
  network(now);
}

void Sleep::enter(uint64_t now) {
  uint64_t quiet = now - quiet_since_us_;
  set_state(State::Asleep, now);
  sleeps_++;
  flush_asked_ = false;
  bool ok = client_posture(link_, &walker_, "limp");
  const char* net = "none";
  if (hw_ != nullptr && hw_->net_device()) net = hw_->net_link() ? "kept on (a cable is in)" : "no link: PHY power-down";
  char q[24];
  duration(q, sizeof q, quiet);
  logf(log_, LogLevel::Info,
       "power: light sleep (%s after A7 with nothing else): servos off (LIMP%s), the loop idles %u ms a pass, "
       "Ethernet %s; battery %u mV",
       q, ok ? "" : ": REFUSED by the link", (unsigned)(kSleepLoopUs / 1000u), net, (unsigned)battery_.last_mv());
}

void Sleep::wake(State to, const char* why, uint64_t now) {
  uint64_t slept = now - since_us_;
  set_state(to, now);
  quiet_since_us_ = now;  // Drowsy: the 30 s count from the wake
  flush_asked_ = false;
  wakes_++;
  format(last_wake_, sizeof last_wake_, "%s", why);
  if (net_ != Net::On) net_set(Net::On, now);
  char d[24], s[16];
  duration(d, sizeof d, slept);
  tenths(s, sizeof s, battery_.segments() != 0 ? battery_.segment(0).slope_x10 : BatteryTrend::kNoTrend);
  char again[64] = "";
  if (to == State::Drowsy)
    format(again, sizeof again, "; A7 still holds: asleep again after %u s of nothing",
           (unsigned)(kSleepAfterUs / 1000000u));
  logf(log_, LogLevel::Info, "power: awake (%s) after %s asleep; battery %u mV, %s mV/min while asleep%s", why, d,
       (unsigned)battery_.last_mv(), s, again);
}

void Sleep::net_set(Net n, uint64_t now) {
  (void)now;
  bool save = n == Net::Save;
  if (save != (net_ == Net::Save) && hw_ != nullptr && !hw_->net_save(save)) {
    net_errors_++;
    logf(log_, LogLevel::Warn, "power: the Ethernet PHY did not take %s", save ? "its power-down" : "normal mode");
  }
  if (n == Net::Save && net_ == Net::On) {
    net_saves_++;
    logf(log_, LogLevel::Info, "power: no Ethernet link: PHY power-down (on by itself when a cable brings a signal; "
                               "a %u s look every %u s)",
         (unsigned)(kNetLookUs / 1000000u), (unsigned)(kNetRecheckUs / 1000000u));
  }
  if (n == Net::Look) net_looks_++;
  if (n == Net::On && net_ != Net::On && asleep())
    logf(log_, LogLevel::Info, "power: Ethernet link while asleep: PHY on");
  net_ = n;
}

void Sleep::network(uint64_t now) {
  net_device_ = hw_ != nullptr && hw_->net_device();
  if (!net_device_) return;
  if (state_ != State::Asleep) {
    if (net_ != Net::On) net_set(Net::On, now);
    return;
  }
  bool link = hw_->net_link();
  switch (net_) {
    case Net::On:
      if (!link) {
        net_set(Net::Save, now);
        net_next_us_ = now + kNetRecheckUs;
      }
      return;
    case Net::Save:
      if (link) net_set(Net::On, now);
      else if (now >= net_next_us_) {  // a look starts every kNetRecheckUs
        net_set(Net::Look, now);
        net_next_us_ += kNetRecheckUs;
        if (net_next_us_ < now + kNetLookUs) net_next_us_ = now + kNetRecheckUs;  // a long stall
      }
      return;
    case Net::Look:
      if (link) net_set(Net::On, now);
      else if (now + kNetRecheckUs >= net_next_us_ + kNetLookUs) net_set(Net::Save, now);  // its kNetLookUs are up
      return;
  }
}

void Sleep::metric_line(uint32_t i, char* out, uint32_t n) const {
  uint64_t now = clock_.now_us();
  out[0] = '\0';
  switch (i) {
    case 0:
      format(out, n, "power: state=%s for_s=%u light_sleep=%s sleeps=%u wakes=%u last_wake=%s awake_s=%u "
                     "drowsy_s=%u asleep_s=%u",
             state_name(state_), (unsigned)((now - since_us_) / 1000000u), enabled_ ? "on" : "off", (unsigned)sleeps_,
             (unsigned)wakes_, last_wake_, (unsigned)(time_in_us(State::Awake) / 1000000u),
             (unsigned)(time_in_us(State::Drowsy) / 1000000u), (unsigned)(time_in_us(State::Asleep) / 1000000u));
      return;
    case 1: {
      int32_t t = soc_.temp_mc;
      format(out, n, "power_hw: net=%s net_device=%s net_saves=%u net_looks=%u net_errors=%u card_flushes=%u "
                     "arm_mhz=%u temp_c=%d throttled=0x%x",
             net_name(net_), net_device_ ? "yes" : "no",
             (unsigned)net_saves_, (unsigned)net_looks_, (unsigned)net_errors_, (unsigned)flushes_,
             soc_ok_ ? (unsigned)(soc_.arm_hz / 1000000u) : 0u, soc_ok_ ? (int)(t / 1000) : 0,
             soc_ok_ ? (unsigned)soc_.throttled : 0u);
      return;
    }
    case 2: {
      char t1[16], t5[16], t15[16], t30[16], ts[16];
      tenths(t1, sizeof t1, battery_.trend(now, 60));
      tenths(t5, sizeof t5, battery_.trend(now, 300));
      tenths(t15, sizeof t15, battery_.trend(now, 900));
      tenths(t30, sizeof t30, battery_.trend(now, 1800));
      tenths(ts, sizeof ts, battery_.segment_trend());
      format(out, n, "battery: mv=%u min_mv=%u mv_per_min_1m=%s 5m=%s 15m=%s 30m=%s this_state=%s samples=%u",
             (unsigned)battery_.last_mv(), (unsigned)battery_.min_mv(), t1, t5, t15, t30, ts,
             (unsigned)battery_.samples());
      return;
    }
    case 3: {
      size_t len = format(out, n, "power_segments:");
      if (battery_.segments() == 0) len += format(out + len, n - len, " none");
      for (uint32_t k = 0; k < battery_.segments(); k++) {
        const BatteryTrend::Segment& g = battery_.segment(k);
        char s[16];
        tenths(s, sizeof s, g.slope_x10);
        len += format(out + len, n - len, " %s:%us:%u-%umV:%s", state_name((State)g.state),
                      (unsigned)(g.end_s - g.start_s), (unsigned)g.first_mv, (unsigned)g.last_mv, s);
      }
      return;
    }
    default:
      return;
  }
}

}  // namespace power
}  // namespace kilo
