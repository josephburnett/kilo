#include "guard/guard.h"
#include "remote/remote.h"
#include "util/dmath.h"
#include "util/fmt.h"
#include "util/dmath.h"
#include "util/log.h"
#include "util/mem.h"
#include "util/str.h"

namespace kilo {
namespace guard {

namespace {
constexpr double kPi = 3.14159265358979;
}  // namespace

const char* zone_name(Zone z) {
  switch (z) {
    case Zone::Blue: return "blue";
    case Zone::Red: return "red";
    default: return "green";
  }
}

const char* eyes_name(Eyes e) {
  switch (e) {
    case Eyes::Blue: return "blue";
    case Eyes::Red: return "red";
    case Eyes::Orange: return "orange";
    default: return "green";
  }
}

const char* action_name(Action a) {
  switch (a) {
    case Action::Stopping: return "stopping";
    case Action::Backing: return "backing";
    case Action::Holding: return "holding";
    default: return "watch";
  }
}

const char* seen_name(Seen s) {
  switch (s) {
    case Seen::Reading: return "reading";
    case Seen::NoEcho: return "no-echo";
    case Seen::TooFar: return "too-far";
    case Seen::Floor: return "floor";
    default: return "none";
  }
}

const char* still_name(Still s) {
  switch (s) {
    case Still::Standing: return "standing";
    case Still::Sitting: return "sitting";
    case Still::Lying: return "lying";
    default: return "moving";
  }
}

double stop_way_mm(const char* gait, double v) {
  if (v <= 0) return 0;
  if (str_eq(gait, "crawl")) return kCrawlStopMm;  // slow: the lean as the legs square up, not the speed
  double a = str_eq(gait, "walk") ? kDecelWalkMmS2 : kDecelTrotMmS2;
  return kStopSlackMm + v * v / (2 * a);
}

double blue_mm(const char* gait, double v) {
  double b = kStopGapMm + (v > 0 ? v * kReactS + stop_way_mm(gait, v) : 0);
  return b < kBlueMinMm ? kBlueMinMm : b;
}

double floor_from_mm(double down) {
  if (down <= kFloorTiltDeg) return 0;
  return kFloorShare * kSensorHeightMm / d_sin(down * kPi / 180.0);
}

// ---- the filter -----------------------------------------------------------------------------

void DistanceFilter::feed(double t, uint16_t raw_mm, bool fresh, double floor_mm) {
  // the window: readings older than kWindowS leave it
  int k = 0;
  for (int i = 0; i < n_; i++)
    if (t - pts_[i].t <= kWindowS) pts_[k++] = pts_[i];
  n_ = k;
  if (fresh) {
    raw_ = raw_mm;
    if (raw_mm == 0 || raw_mm > kMaxRangeMm) {
      dropouts_++;
      last_ = raw_mm == 0 ? Seen::NoEcho : Seen::TooFar;
      // right after something very close, no echo is closer than the sensor measures: the range holds
      if (has_ && mm_ < kRedMm && !hold_) {
        hold_ = true;
        hold_until_ = t + kCloseHoldS;
      }
    } else if (floor_mm < 0 || (floor_mm > 0 && raw_mm >= floor_mm)) {
      floor_++;
      last_ = Seen::Floor;
    } else {
      readings_++;
      last_ = Seen::Reading;
      hold_ = false;
      if (n_ == 3) {
        pts_[0] = pts_[1];
        pts_[1] = pts_[2];
        n_ = 2;
      }
      pts_[n_++] = {t, (double)raw_mm};
    }
  }
  if (hold_ && t >= hold_until_) hold_ = false;
  if (n_ >= 2) {
    double a = pts_[0].mm, b = pts_[1].mm;
    if (n_ == 2) {
      mm_ = a > b ? a : b;  // both must be close: the farther counts
    } else {
      double c = pts_[2].mm;
      mm_ = a > b ? (b > c ? b : (a > c ? c : a)) : (a > c ? a : (b > c ? c : b));
    }
    has_ = true;
  } else if (hold_) {
    // no echo after something very close: the last range holds (kCloseHoldS)
  } else if (n_ == 0 || (has_ && t - pts_[n_ - 1].t > kStaleS)) {
    has_ = false;
  }
  // one reading left in the window: the last range holds until kStaleS
}

Zone next_zone(Zone now, bool has, double mm, double blue, double red) {
  if (!has) return Zone::Green;
  if (now == Zone::Red && mm <= kRedExitMm) return Zone::Red;
  if (mm < red) return Zone::Red;
  if (now != Zone::Green && mm <= blue + kBlueExitMm) return Zone::Blue;
  if (mm < blue) return Zone::Blue;
  return Zone::Green;
}

// ---- the guard ------------------------------------------------------------------------------

DistanceGuard::DistanceGuard(SerialLink& link, planner::Walker& walker, Clock& clock, Log& log)
    : link_(link), walker_(walker), clock_(clock), log_(log) {}

void DistanceGuard::set_enabled(bool on) {
  if (on == on_) return;
  on_ = on;
  if (!on && action_ != Action::Watch) set_action(Action::Watch, "the guard was switched off");
  char line[64];
  format(line, sizeof line, "guard %s", on ? "on" : "off");
  note(line);
}

// What the walker does now, as the guard sees it: the forward walk's speed and gait in *v, *gait.
DistanceGuard::Walk DistanceGuard::walk_kind(double* v, const char** gait) const {
  if (!walker_.active()) return Walk::None;
  if (walker_.planner().stopping() && walker_.phase() == planner::WalkPhase::Streaming) return Walk::Stopping;
  const planner::WalkGoal& g = walker_.goal();
  switch (g.direction) {
    case planner::Direction::Forward: {
      double s = walker_.phase() == planner::WalkPhase::Streaming ? (double)walker_.planner().timing().speed_mm_s
                                                                    : (double)g.speed_mm_s;
      if (s < 0) s = (double)planner::default_speed(*g.pattern);
      *v = s;
      *gait = g.pattern->name;
      return Walk::Forward;
    }
    case planner::Direction::Left:
    case planner::Direction::Right: return Walk::Turn;
    case planner::Direction::Back: return Walk::Back;
    default: return Walk::Other;
  }
}

bool DistanceGuard::forward_walk(double* v, const char** gait) const { return walk_kind(v, gait) == Walk::Forward; }

void DistanceGuard::note(const char* what) {
  logf(log_, LogLevel::Info, "%s", what);
  if (host_ != nullptr) host_->mark(what);
}

// One compact line each time what the guard makes of it changes: the state and the reason (not the
// readings themselves, which change every sample).
void DistanceGuard::decide(const char* state, const char* reason) {
  char key[sizeof decided_];
  format(key, sizeof key, "%s: %s", state, reason);
  if (str_eq(key, decided_)) return;
  format(decided_, sizeof decided_, "%s", key);
  char line[192];
  if (filter_.has())
    format(line, sizeof line, "guard decides: zone=%s mm=%d raw=%u seen=%s%s blue=%d floor_from=%d action=%s state=%s",
           zone_name(zone_), d_iround(filter_.mm()), (unsigned)filter_.raw(), seen_name(filter_.last()),
           filter_.close_hold() ? " (holding: no echo after very close)" : "", d_iround(blue_), d_iround(floor_mm_),
           action_name(action_), key);
  else
    format(line, sizeof line, "guard decides: zone=%s mm=none raw=%u seen=%s blue=%d floor_from=%d action=%s state=%s",
           zone_name(zone_), (unsigned)filter_.raw(), seen_name(filter_.last()), d_iround(blue_), d_iround(floor_mm_),
           action_name(action_), key);
  note(line);
}

void DistanceGuard::on_telemetry(const kilo_telemetry_t& t, double head_tilt_deg, bool lying) {
  double now = (double)clock_.now_us() / 1e6;
  // the beam's tilt below level: the head held down, plus the body nose down (TELEMETRY pitch, nose down +)
  double down = -head_tilt_deg + from_a16((double)t.pitch);
  bool fresh = (t.flags & KILO_TELEMETRY_FLAGS_DIST_FRESH) != 0;
  lying_ = lying;
  floor_mm_ = lying ? -1 : floor_from_mm(down);
  filter_.feed(now, t.dist_mm, fresh, floor_mm_);

  double v = 0;
  const char* gait = "";
  bool fwd = forward_walk(&v, &gait);
  if (action_ != Action::Watch) {  // the speed when the guard acted holds the thresholds
    v = v_;
    gait = gait_;
  } else {
    v_ = fwd ? v : 0;
    gait_ = fwd ? gait : "";
    v = v_;
  }
  blue_ = blue_mm(gait, v);
  Zone z = next_zone(zone_, filter_.has(), filter_.mm(), blue_, kRedMm);
  if (fwd && filter_.has() && (!has_min_ || filter_.mm() < min_mm_)) {
    min_mm_ = filter_.mm();
    has_min_ = true;
  }
  if (z != zone_) {
    zone_changes_++;
    if (z == Zone::Red) red_new_ = true;
    if (z == Zone::Blue && zone_ == Zone::Green) blue_new_ = true;
    if (z == Zone::Green) blue_new_ = red_new_ = false;
    if (z == Zone::Blue) red_new_ = false;
    zone_ = z;
    char line[128];
    if (filter_.has())
      format(line, sizeof line, "guard zone=%s mm=%d raw=%u blue=%d red=%d v=%d gait=%s action=%s", zone_name(z),
             d_iround(filter_.mm()), (unsigned)filter_.raw(), d_iround(blue_), d_iround(kRedMm), d_iround(v), gait[0] ? gait : "-",
             action_name(action_));
    else
      format(line, sizeof line, "guard zone=%s mm=none raw=%u blue=%d red=%d v=%d action=%s", zone_name(z),
             (unsigned)filter_.raw(), d_iround(blue_), d_iround(kRedMm), d_iround(v), action_name(action_));
    note(line);
  }
  uint64_t us = clock_.now_us();
  bool green = zone_ == Zone::Green;
  if (green && !green_) green_since_ = us;
  green_ = green;
}

void DistanceGuard::set_action(Action a, const char* why) {
  action_ = a;
  action_at_ = clock_.now_us();
  char line[160];
  format(line, sizeof line, "guard action=%s zone=%s mm=%d blue=%d v=%d gait=%s: %s", action_name(a), zone_name(zone_),
         filter_.has() ? d_iround(filter_.mm()) : 0, d_iround(blue_), d_iround(v_), gait_[0] ? gait_ : "-", why);
  note(line);
}

// The quick stop of whatever walk runs, then (Stopping) sit and the sitting variant.
void DistanceGuard::quick_stop(uint64_t now, const char* why) {
  walker_.halt();
  deadline_ = now + (uint64_t)(remote::stop_timeout_s((double)walker_.planner().timing().period_ms) * 1e6);
  set_action(Action::Stopping, why);
}

// Back off: the walk running turns backward, or (standing) a backward walk starts; kBackS once it walks back.
void DistanceGuard::back_off(uint64_t now, const char* why) {
  pullbacks_++;
  hisses_++;
  backs_++;
  planner::WalkGoal g = walker_.goal();
  if (!walker_.active()) {  // standing: a slow walk (the walk pattern: three feet down most of the time)
    g = planner::WalkGoal();
    g.pattern = planner::find_pattern("walk");
  }
  g.direction = planner::Direction::Back;
  g.speed_mm_s = (planner::real)kBackSpeedMmS;
  g.steer = 0;
  bool streaming = walker_.active() && walker_.phase() == planner::WalkPhase::Streaming && !walker_.planner().stopping();
  const char* err = walker_.start(g);
  if (err == nullptr) {
    back_started_ = streaming;  // a walk already streaming turns backward at once (the strides slew through zero)
    deadline_ = now + (uint64_t)((streaming ? kBackS : remote::stop_timeout_s(0) + 1) * 1e6);
    set_action(Action::Backing, why);
    return;
  }
  quick_stop(now, "very close, backing refused: the quick stop");
}

// Standing or sitting still (or lying), something new in BLUE or RED: true if he did something.
bool DistanceGuard::react_still(uint64_t now, Still s) {
  if (s == Still::None || s == Still::Lying) return false;
  if (red_new_) {
    red_new_ = blue_new_ = false;
    if (backs_ >= kMaxBacks) {
      decide(still_name(s), "eyes only (backed off twice: clear for 1 s first)");
      return false;
    }
    reactions_++;
    if (s == Still::Standing) {
      back_off(now, "very close while standing: backing off");
      return true;
    }
    backs_++;
    hisses_++;
    bool ok = host_ != nullptr && host_->flinch();
    set_action(Action::Holding, ok ? "very close while sitting: leaning back" : "very close while sitting: the lean refused");
    return true;
  }
  if (blue_new_) {
    blue_new_ = false;
    reactions_++;
    if (host_ != nullptr) host_->notice();
    if (s == Still::Standing) {
      if (host_ != nullptr) host_->sit_and_behave();
      set_action(Action::Holding, "something close while standing: looks at it, sits");
    } else {
      set_action(Action::Holding, "something close while sitting: looks at it");
    }
    return true;
  }
  return false;
}

void DistanceGuard::poll() {
  uint64_t now = clock_.now_us();
  eyes_step();
  if (!on_) {
    decide("off", "eyes only (the guard is off)");
    return;
  }
  double v = 0;
  const char* gait = "";
  Walk wk = walk_kind(&v, &gait);
  const char* blocked = host_ != nullptr ? host_->not_now() : nullptr;
  bool may = blocked == nullptr;
  Still still = wk == Walk::None && host_ != nullptr ? host_->still() : Still::None;
  const char* state = wk == Walk::Forward ? "walk-forward"
                      : wk == Walk::Turn  ? "turn-in-place"
                      : wk == Walk::Back  ? "walk-back"
                      : wk == Walk::Other ? "walk-in-place"
                      : wk == Walk::Stopping ? "stopping"
                                             : still_name(still);
  if (!may) {
    decide(blocked, "eyes only");
    return;
  }
  switch (action_) {
    case Action::Watch: {
      if (zone_ == Zone::Green) {
        decide(state, filter_.last() != Seen::Floor ? "clear"
                      : lying_ ? "nothing seen (lying: every reading is the floor)"
                               : "nothing seen (the beam on the floor)");
        return;
      }
      if (wk == Walk::Forward) {
        v_ = v;
        gait_ = gait;
        // the crawl turns round slowly (its strides 0.5 mm a tick: 4 s to walk back): it only stops
        if (zone_ == Zone::Red && backs_ < kMaxBacks && !str_eq(gait, "crawl")) {
          red_new_ = blue_new_ = false;
          decide(state, "back off");
          back_off(now, "very close: backing off");
          return;
        }
        if (zone_ == Zone::Red && backs_ < kMaxBacks) {  // the crawl, very close: stop (the hiss, counted)
          hisses_++;
          backs_++;
        }
        stops_++;
        red_new_ = blue_new_ = false;
        decide(state, "the quick stop");
        quick_stop(now, zone_ == Zone::Red ? "very close: the quick stop" : "too close: the quick stop");
        return;
      }
      if (wk == Walk::Turn) {
        if (zone_ == Zone::Red && red_new_ && backs_ < kMaxBacks) {
          red_new_ = blue_new_ = false;
          stops_++;
          hisses_++;
          backs_++;
          reactions_++;
          decide(state, "the quick stop (very close while turning)");
          quick_stop(now, "very close while turning in place: the quick stop");
          return;
        }
        decide(state, zone_ == Zone::Red ? "eyes only (red was there before the turn)" : "eyes only (blue while turning)");
        return;
      }
      if (wk == Walk::None) {
        if (react_still(now, still)) {
          decide(state, action_ == Action::Backing ? "back off" : "react");
          return;
        }
        decide(state, still == Still::Lying ? "eyes only (lying: the floor)"
                      : still == Still::None ? "eyes only (between postures)"
                                             : "eyes only (no change of zone)");
        return;
      }
      decide(state, wk == Walk::Back ? "eyes only (walking away from it)" : "eyes only");
      return;
    }
    case Action::Stopping:
      if (zone_ == Zone::Red && backs_ < kMaxBacks && walker_.active() && !str_eq(gait_, "crawl") &&
          walker_.goal().direction == planner::Direction::Forward) {
        // RED during BLUE's stop: back off once it has stopped (the back walk waits for the stop)
        red_new_ = false;
        pullbacks_++;
        hisses_++;
        backs_++;
        planner::WalkGoal g = walker_.goal();
        g.direction = planner::Direction::Back;
        g.speed_mm_s = (planner::real)kBackSpeedMmS;
        g.steer = 0;
        if (walker_.start(g) == nullptr) {  // queued after the stop: kBackS from when it walks back
          back_started_ = false;
          deadline_ = now + (uint64_t)((remote::stop_timeout_s((double)walker_.planner().timing().period_ms) + 1) * 1e6);
          set_action(Action::Backing, "very close while stopping: backing off");
          return;
        }
      }
      decide(state, "stopping");
      if (walker_.active() && now < deadline_) return;
      if (walker_.active()) walker_.abort("the guard's stop took too long");
      if (host_ != nullptr) host_->sit_and_behave();
      set_action(Action::Holding, "stopped: sitting, the behaviour runner's sitting variant");
      return;
    case Action::Backing:
      decide(state, "backing off");
      if (!back_started_) {  // a back walk queued behind the stop or still in its setup: kBackS once it walks back
        bool back = walker_.active() && walker_.phase() == planner::WalkPhase::Streaming &&
                    walker_.goal().direction == planner::Direction::Back && !walker_.planner().stopping();
        if (back) {
          back_started_ = true;
          deadline_ = now + (uint64_t)(kBackS * 1e6);
          return;
        }
        if (now < deadline_ && walker_.active()) return;
        if (walker_.active()) walker_.halt();
        if (host_ != nullptr) host_->sit_and_behave();
        set_action(Action::Holding, "backing did not start: sitting");
        return;
      }
      if (!walker_.active()) {  // ended by itself (a key, a fall): sit
        if (host_ != nullptr) host_->sit_and_behave();
        set_action(Action::Holding, "backing ended: sitting");
        return;
      }
      if (now < deadline_) return;
      walker_.halt();
      deadline_ = now + (uint64_t)(remote::stop_timeout_s(0) * 1e6);
      set_action(Action::Stopping, "backed off: the quick stop");
      return;
    case Action::Holding:
      // A forward walk while holding (a key that took over the stop and kept the walk forward,
      // B5 from +5 to +4): the quick stop again. Forward keys are refused; this catches the rest.
      if (wk == Walk::Forward) {
        stops_++;
        decide(state, "the quick stop (a forward walk while holding)");
        quick_stop(now, "a forward walk while holding: the quick stop");
        return;
      }
      if (wk == Walk::Turn && zone_ == Zone::Red && red_new_ && backs_ < kMaxBacks) {
        red_new_ = blue_new_ = false;
        stops_++;
        hisses_++;
        backs_++;
        reactions_++;
        decide(state, "the quick stop (very close while turning)");
        quick_stop(now, "very close while turning in place: the quick stop");
        return;
      }
      if (wk == Walk::None && react_still(now, still)) {
        decide(state, "react (holding)");
        return;
      }
      if (green_ && now - green_since_ >= (uint64_t)(kClearS * 1e6)) {
        releases_++;
        backs_ = 0;
        v_ = 0;
        gait_ = "";
        set_action(Action::Watch, "clear: forward walking allowed again");
        decide(state, "clear");
        return;
      }
      decide(state, zone_ == Zone::Green ? "holding (clear soon)" : "holding: eyes only");
      return;
  }
}

void DistanceGuard::key_took_over() {
  if (action_ == Action::Stopping || action_ == Action::Backing) set_action(Action::Holding, "a key took over");
}

bool DistanceGuard::eyes_driven() const {
  const BodyState& b = link_.state();
  return b.has_hello && b.proto_ok == 1 && (b.hello.caps & KILO_HELLO_CAPS_EYES) != 0;
}

void DistanceGuard::eyes_step() {
  eyes_ = zone_ == Zone::Red ? Eyes::Red : zone_ == Zone::Blue ? Eyes::Blue : tired_ ? Eyes::Orange : Eyes::Green;
  if (!eyes_driven()) {
    eyes_sent_ = false;
    return;
  }
  const Goal* g = link_.goal(KILO_MSG_SET_EYES);
  uint8_t want[KILO_SET_EYES_LEN] = {0x3F, 0, 0, 0};  // the mask, then r, g, b
  if (eyes_ == Eyes::Orange) {
    want[1] = kEyeLevel;
    want[2] = kOrangeGreen;
  } else {
    want[1 + (eyes_ == Eyes::Red ? 0 : eyes_ == Eyes::Green ? 1 : 2)] = kEyeLevel;
  }
  bool same = g != nullptr && g->len == sizeof want;
  for (uint32_t i = 0; same && i < sizeof want; i++) same = g->payload[i] == want[i];
  if (same) return;
  eyes_sent_ = link_.set_goal(KILO_MSG_SET_EYES, want, sizeof want);
}

void DistanceGuard::fill(kilo_host_guard_status_t& s) const {
  mem_zero(&s, sizeof s);
  s.enabled = on_ ? 1 : 0;
  s.zone = (uint8_t)zone_;
  s.action = (uint8_t)action_;
  s.eyes = (uint8_t)eyes_;
  s.eyes_driven = eyes_driven() ? 1 : 0;
  s.dist_mm = filter_.has() ? (uint16_t)d_iround(filter_.mm()) : 0;
  s.raw_mm = filter_.raw();
  s.blue_mm = (uint16_t)d_iround(blue_);
  s.red_mm = (uint16_t)d_iround(kRedMm);
  s.speed_mm_s = (uint16_t)d_iround(v_);
  s.zone_changes = zone_changes_;
  s.stops = stops_;
  s.pullbacks = pullbacks_;
  s.refused = refused_;
}

void DistanceGuard::metrics_line(char* out, uint32_t n) const {
  format(out, n,
         "guard: on=%u zone=%s action=%s eyes=%s eyes_driven=%u zone_changes=%u stops=%u pullbacks=%u refused=%u "
         "releases=%u reactions=%u hisses=%u readings=%u dropouts=%u floor_ignored=%u min_mm=%d",
         on_ ? 1u : 0u, zone_name(zone_), action_name(action_), eyes_name(eyes_), eyes_driven() ? 1u : 0u,
         (unsigned)zone_changes_, (unsigned)stops_, (unsigned)pullbacks_, (unsigned)refused_, (unsigned)releases_,
         (unsigned)reactions_, (unsigned)hisses_, (unsigned)filter_.readings(), (unsigned)filter_.dropouts(),
         (unsigned)filter_.floor_ignored(), has_min_ ? d_iround(min_mm_) : -1);
}

}  // namespace guard
}  // namespace kilo
