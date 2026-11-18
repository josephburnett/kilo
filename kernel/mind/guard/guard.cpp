#include "guard/guard.h"
#include "remote/remote.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/mem.h"
#include "util/str.h"

namespace kilo {
namespace guard {

namespace {
int rnd(double v) { return (int)(v < 0 ? v - 0.5 : v + 0.5); }
}  // namespace

const char* zone_name(Zone z) {
  switch (z) {
    case Zone::Blue: return "blue";
    case Zone::Red: return "red";
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

double stop_way_mm(const char* gait, double v) {
  if (v <= 0) return 0;
  double a = kDecelTrotMmS2;
  if (str_eq(gait, "walk")) a = kDecelWalkMmS2;
  else if (str_eq(gait, "crawl")) a = kDecelCrawlMmS2;
  return v * v / (2 * a);
}

double blue_mm(const char* gait, double v) {
  double b = kStopGapMm + (v > 0 ? v * kReactS + stop_way_mm(gait, v) : 0);
  return b < kBlueMinMm ? kBlueMinMm : b;
}

// ---- the filter -----------------------------------------------------------------------------

void DistanceFilter::feed(double t, uint16_t raw_mm, bool fresh, bool floor) {
  // the window: readings older than kWindowS leave it
  int k = 0;
  for (int i = 0; i < n_; i++)
    if (t - pts_[i].t <= kWindowS) pts_[k++] = pts_[i];
  n_ = k;
  if (fresh) {
    raw_ = raw_mm;
    if (raw_mm == 0 || raw_mm > kMaxRangeMm) {
      dropouts_++;
    } else if (floor) {
      floor_++;
    } else {
      readings_++;
      if (n_ == 3) {
        pts_[0] = pts_[1];
        pts_[1] = pts_[2];
        n_ = 2;
      }
      pts_[n_++] = {t, (double)raw_mm};
    }
  }
  if (n_ >= 2) {
    double a = pts_[0].mm, b = pts_[1].mm;
    if (n_ == 2) {
      mm_ = a > b ? a : b;  // both must be close: the farther counts
    } else {
      double c = pts_[2].mm;
      mm_ = a > b ? (b > c ? b : (a > c ? c : a)) : (a > c ? a : (b > c ? c : b));
    }
    has_ = true;
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

bool DistanceGuard::forward_walk(double* v, const char** gait) const {
  if (!walker_.active() || walker_.planner().stopping()) return false;
  const planner::WalkGoal& g = walker_.goal();
  if (g.direction != planner::Direction::Forward) return false;
  double s = walker_.phase() == planner::WalkPhase::Streaming ? (double)walker_.planner().timing().speed_mm_s
                                                                  : (double)g.speed_mm_s;
  if (s < 0) s = (double)planner::default_speed(*g.pattern);
  *v = s;
  *gait = g.pattern->name;
  return true;
}

void DistanceGuard::note(const char* what) {
  logf(log_, LogLevel::Info, "%s", what);
  if (host_ != nullptr) host_->mark(what);
}

void DistanceGuard::on_telemetry(const kilo_telemetry_t& t, double head_tilt_deg) {
  double now = (double)clock_.now_us() / 1e6;
  // the beam's tilt below level: the head held down, plus the body nose down (TELEMETRY pitch, nose down +)
  double down = -head_tilt_deg + (double)t.pitch / 16.0;
  bool fresh = (t.flags & KILO_TELEMETRY_FLAGS_DIST_FRESH) != 0;
  filter_.feed(now, t.dist_mm, fresh, down > kFloorTiltDeg);

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
    zone_ = z;
    char line[128];
    if (filter_.has())
      format(line, sizeof line, "guard zone=%s mm=%d raw=%u blue=%d red=%d v=%d gait=%s action=%s", zone_name(z),
             rnd(filter_.mm()), (unsigned)filter_.raw(), rnd(blue_), rnd(kRedMm), rnd(v), gait[0] ? gait : "-",
             action_name(action_));
    else
      format(line, sizeof line, "guard zone=%s mm=none raw=%u blue=%d red=%d v=%d action=%s", zone_name(z),
             (unsigned)filter_.raw(), rnd(blue_), rnd(kRedMm), rnd(v), action_name(action_));
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
         filter_.has() ? rnd(filter_.mm()) : 0, rnd(blue_), rnd(v_), gait_[0] ? gait_ : "-", why);
  note(line);
}

void DistanceGuard::poll() {
  uint64_t now = clock_.now_us();
  eyes_step();
  if (!on_) return;
  double v;
  const char* gait;
  bool may = host_ == nullptr || host_->may_act();
  switch (action_) {
    case Action::Watch: {
      if (!forward_walk(&v, &gait) || !may || zone_ == Zone::Green) return;
      v_ = v;
      gait_ = gait;
      if (zone_ == Zone::Red) {
        pullbacks_++;
        planner::WalkGoal g = walker_.goal();
        g.direction = planner::Direction::Back;
        g.speed_mm_s = (planner::real)kBackSpeedMmS;
        const char* why = walker_.start(g);
        if (why == nullptr) {
          back_started_ = true;  // the walk turns backward at once (the strides slew through zero)
          deadline_ = now + (uint64_t)(kBackS * 1e6);
          set_action(Action::Backing, "very close: backing off");
          return;
        }
        walker_.stop();
        set_action(Action::Stopping, "very close, backing refused: stopping");
        deadline_ = now + (uint64_t)(remote::stop_timeout_s(0) * 1e6);
        return;
      }
      stops_++;
      walker_.stop();
      deadline_ = now + (uint64_t)(remote::stop_timeout_s((double)walker_.planner().timing().period_ms) * 1e6);
      set_action(Action::Stopping, "too close: the planned stop");
      return;
    }
    case Action::Stopping:
      if (zone_ == Zone::Red && walker_.active() && walker_.goal().direction == planner::Direction::Forward && may) {
        pullbacks_++;
        planner::WalkGoal g = walker_.goal();
        g.direction = planner::Direction::Back;
        g.speed_mm_s = (planner::real)kBackSpeedMmS;
        if (walker_.start(g) == nullptr) {  // queued after the stop: kBackS from when it walks back
          back_started_ = false;
          deadline_ = now + (uint64_t)((remote::stop_timeout_s((double)walker_.planner().timing().period_ms) + 1) * 1e6);
          set_action(Action::Backing, "very close while stopping: backing off");
          return;
        }
      }
      if (walker_.active() && now < deadline_) return;
      if (walker_.active()) walker_.abort("the guard's stop took too long");
      if (host_ != nullptr) host_->sit_and_behave();
      set_action(Action::Holding, "stopped: sitting, the behaviour runner's sitting variant");
      return;
    case Action::Backing:
      if (!back_started_) {  // a back walk queued behind the planned stop: kBackS counts once it walks back
        bool back = walker_.active() && walker_.phase() == planner::WalkPhase::Streaming &&
                    walker_.goal().direction == planner::Direction::Back && !walker_.planner().stopping();
        if (back) {
          back_started_ = true;
          deadline_ = now + (uint64_t)(kBackS * 1e6);
          return;
        }
        if (now < deadline_ && walker_.active()) return;
        if (walker_.active()) walker_.stop();
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
      walker_.stop();
      deadline_ = now + (uint64_t)(remote::stop_timeout_s(0) * 1e6);
      set_action(Action::Stopping, "backed off: the planned stop");
      return;
    case Action::Holding:
      if (green_ && now - green_since_ >= (uint64_t)(kClearS * 1e6)) {
        releases_++;
        v_ = 0;
        gait_ = "";
        set_action(Action::Watch, "clear: forward walking allowed again");
      }
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
  eyes_ = zone_;
  if (!eyes_driven()) {
    eyes_sent_ = false;
    return;
  }
  const Goal* g = link_.goal(KILO_MSG_SET_EYES);
  uint8_t want[KILO_SET_EYES_LEN] = {0x3F, 0, 0, 0};
  want[1 + (eyes_ == Zone::Red ? 0 : eyes_ == Zone::Green ? 1 : 2)] = kEyeLevel;
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
  s.dist_mm = filter_.has() ? (uint16_t)rnd(filter_.mm()) : 0;
  s.raw_mm = filter_.raw();
  s.blue_mm = (uint16_t)rnd(blue_);
  s.red_mm = (uint16_t)rnd(kRedMm);
  s.speed_mm_s = (uint16_t)rnd(v_);
  s.zone_changes = zone_changes_;
  s.stops = stops_;
  s.pullbacks = pullbacks_;
  s.refused = refused_;
}

void DistanceGuard::metrics_line(char* out, uint32_t n) const {
  format(out, n,
         "guard: on=%u zone=%s action=%s eyes=%s eyes_driven=%u zone_changes=%u stops=%u pullbacks=%u refused=%u "
         "releases=%u readings=%u dropouts=%u floor_ignored=%u min_mm=%d",
         on_ ? 1u : 0u, zone_name(zone_), action_name(action_), zone_name(eyes_), eyes_driven() ? 1u : 0u,
         (unsigned)zone_changes_, (unsigned)stops_, (unsigned)pullbacks_, (unsigned)refused_, (unsigned)releases_,
         (unsigned)filter_.readings(), (unsigned)filter_.dropouts(), (unsigned)filter_.floor_ignored(),
         has_min_ ? rnd(min_mm_) : -1);
}

}  // namespace guard
}  // namespace kilo
