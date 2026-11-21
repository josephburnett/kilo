#include "measure/measure.h"
#include "link/names.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/mem.h"
#include "util/str.h"

namespace kilo {
namespace measure {

namespace {

real deg(int16_t a16) { return (real)a16 / (real)16; }
real rabs(real v) { return v < 0 ? -v : v; }

// "1.23" (two decimals)
void fixed2(char* buf, uint32_t cap, real v) { format_fixed(buf, cap, planner::fixed_i(v, 2), 2); }

int16_t a16(real d) {
  int v = planner::round_i(to_a16(d));
  return (int16_t)(v < -480 ? -480 : v > 480 ? 480 : v);  // FEET's +-30 deg
}

}  // namespace

Measure::Measure(SerialLink& link, Clock& clock, Log& log, const planner::BodyModel& model)
    : link_(link), clock_(clock), log_(log), model_(model) {
  for (Planned& p : ring_) p.tick = -1;
}

void Measure::on_telemetry(const kilo_telemetry_t& t) {
  const BodyState& st = link_.state();
  uint64_t rx = st.telemetry_at_us;
  seen_[(seen_head_ + seen_n_) % kSeen] = {rx, t.pitch, t.roll};
  if (seen_n_ < kSeen) seen_n_++;
  else seen_head_ = (seen_head_ + 1) % kSeen;
  int64_t tick = clock_est_.add(t.tick, rx);
  if (phase_ != Phase::Streaming && phase_ != Phase::Finish) return;
  if (t.mode == KILO_MODE_GAIT) {
    gait_seen_ = true;
  } else if (gait_seen_ && abort_why_ == nullptr) {
    format(abort_buf_, sizeof abort_buf_, "the body left GAIT (mode %s)", mode_name(t.mode));
    abort_why_ = abort_buf_;
    abort_reason_ = t.mode == KILO_MODE_FALLEN || t.mode == KILO_MODE_RECOVERING ? "FELL" : "ABORTED";
    return;
  }
  if (t.mode == KILO_MODE_GAIT) watch(t, tick);
}

void Measure::on_event(const kilo_event_t& e) {
  if (phase_ != Phase::Streaming && phase_ != Phase::Finish) return;
  if (abort_why_ != nullptr) return;
  if (e.code == KILO_EVENT_FELL) {
    abort_why_ = "Kilo fell (the body's fall reflex)";
    abort_reason_ = "FELL";
  } else if (e.code == KILO_EVENT_GAIT_STOPPED && gait_seen_) {
    abort_why_ = e.arg == 1 ? "the body settled after a FEET underrun (the stream was late)"
                            : "the body stopped the gait (a posture or an epoch bump took the legs)";
    abort_reason_ = "ABORTED";
  } else if (e.code == KILO_EVENT_GOAL_REFUSED && ((uint16_t)e.arg >> 8) == KILO_MSG_FEET) {
    format(abort_buf_, sizeof abort_buf_, "the body refused FEET: %s", ack_status_name((uint8_t)(e.arg & 0xFF)));
    abort_why_ = abort_buf_;
    abort_reason_ = "ABORTED";
  }
}

// One TELEMETRY in GAIT at body tick `tick`: the safety limits, the settle hold's average, the tip.
void Measure::watch(const kilo_telemetry_t& t, int64_t tick) {
  int64_t ts = tick - tick0_;
  if (ts < 0 || abort_why_ != nullptr) return;
  real p = deg(t.pitch), r = deg(t.roll);
  if (rabs(p) > kAbortTiltDeg || rabs(r) > kAbortTiltDeg) {
    char a[16], b[16];
    fixed2(a, sizeof a, p);
    fixed2(b, sizeof b, r);
    format(abort_buf_, sizeof abort_buf_, "tilted past %d deg (pitch %s roll %s)", (int)kAbortTiltDeg, a, b);
    abort_why_ = abort_buf_;
    abort_reason_ = "ABORTED";
    return;
  }
  const Planned& pl = ring_[ts % kRing];
  if (pl.tick != ts) return;
  real ep = p - deg(pl.pitch_a16), er = r - deg(pl.roll_a16);
  if (!have_offset_ && ts >= 25) {  // half a second into the level hold: the IMU's offset, roughly
    have_offset_ = true;
    off_p_ = ep;
    off_r_ = er;
  }
  if (have_offset_ && (rabs(ep - off_p_) > kAbortOffDeg || rabs(er - off_r_) > kAbortOffDeg)) {
    char a[16], b[16];
    fixed2(a, sizeof a, ep - off_p_);
    fixed2(b, sizeof b, er - off_r_);
    format(abort_buf_, sizeof abort_buf_, "%d deg off the model's prediction (pitch %s roll %s)", (int)kAbortOffDeg,
           a, b);
    abort_why_ = abort_buf_;
    abort_reason_ = "ABORTED";
    return;
  }
  const Step* steps;
  int n = measure::steps(&steps);
  if (pl.step < 0 || pl.step >= n) return;
  const Step& st = steps[pl.step];
  const Experiment& ex = kExperimentTable[st.exp];
  if (str_eq(st.phase, "settle")) {  // the baseline: all four feet down, before the lift
    if (base_exp_ != st.exp) {
      base_exp_ = st.exp;
      base_n_ = 0;
      base_p_ = base_r_ = 0;
    }
    base_p_ = (base_p_ * (real)base_n_ + p) / (real)(base_n_ + 1);
    base_r_ = (base_r_ * (real)base_n_ + r) / (real)(base_n_ + 1);
    base_n_++;
    return;
  }
  if (ex.lifted < 0 || base_exp_ != st.exp || base_n_ == 0) return;
  if (!str_eq(st.phase, "lifted") && !str_eq(st.phase, "sweep")) return;
  if (script_.done() || script_.step().exp != st.exp) return;
  real up = deg(ex.tip_pitch_a16), ur = deg(ex.tip_roll_a16);
  real T = planner::m_sqrt(up * up + ur * ur);
  real proj = ((p - base_p_) * up + (r - base_r_) * ur) / T;
  if (proj < kTipFraction * T) return;
  int x = script_.x_at((uint32_t)ts);
  if (!script_.tip((uint32_t)ts)) return;
  tips_++;
  char l[96];
  format(l, sizeof l, "tip exp=%s seen=%d x=%d phase=%s", ex.name, (int)ts, x, st.phase);
  line(l);
}

const char* Measure::start(bool has_rested, uint16_t rested_mv) {
  if (active()) return "already measuring";
  const BodyState& st = link_.state();
  uint64_t now = clock_.now_us();
  if (st.proto_ok == 0) return "protocol mismatch: refusing to drive";
  if (!st.has_telemetry || st.epoch == kNoEpoch || !st.alive(now, KILO_BODY_LOST_MS * 1000u))
    return "no telemetry from the body";
  const kilo_telemetry_t& t = st.telemetry;
  if (t.flags & KILO_TELEMETRY_FLAGS_NO_BATTERY) return "on USB power (no battery)";
  uint16_t mv = has_rested ? rested_mv : t.batt_mv;
  if (mv < kMinMv) {
    format(why_, sizeof why_, "battery %u mV%s (needs %u)", (unsigned)mv, has_rested ? " at rest" : "",
           (unsigned)kMinMv);
    return why_;
  }
  if (t.mode != KILO_MODE_IDLE) {
    format(why_, sizeof why_, "not standing still (mode %s): B6 first", mode_name(t.mode));
    return why_;
  }
  const Goal* g = link_.goal(KILO_MSG_SET_POSTURE);
  uint8_t posture = g != nullptr && g->len >= 2 ? g->payload[1] : 0xFF;
  if (posture != KILO_POSTURE_STAND && posture != KILO_POSTURE_STANCE) return "not standing: B6 first";
  // level and still: the last TELEMETRY, and how much the last second's moved
  real p = deg(t.pitch), r = deg(t.roll);
  real lo_p = p, hi_p = p, lo_r = r, hi_r = r;
  int n = 0;
  for (int i = 0; i < seen_n_; i++) {
    const Seen& s = seen_[(seen_head_ + i) % kSeen];
    if (now - s.rx_us > 1000000u) continue;
    n++;
    lo_p = planner::m_min(lo_p, deg(s.pitch));
    hi_p = planner::m_max(hi_p, deg(s.pitch));
    lo_r = planner::m_min(lo_r, deg(s.roll));
    hi_r = planner::m_max(hi_r, deg(s.roll));
  }
  real moved = planner::m_max(hi_p - lo_p, hi_r - lo_r);
  if (rabs(p) > kLevelMaxDeg || rabs(r) > kLevelMaxDeg || moved > kStillPtpDeg || n < 5) {
    char a[16], b[16], c[16];
    fixed2(a, sizeof a, p);
    fixed2(b, sizeof b, r);
    fixed2(c, sizeof c, moved);
    format(why_, sizeof why_, "not level and still (pitch %s roll %s, moved %s deg in 1 s; wants %d and %d)", a, b, c,
           (int)kLevelMaxDeg, (int)kStillPtpDeg);
    return why_;
  }
  // go
  script_.reset();
  for (Planned& pl : ring_) pl.tick = -1;
  ended_ = nullptr;
  ok_ = false;
  abort_why_ = abort_reason_ = nullptr;
  sent_ = late_ = tips_ = 0;
  have_offset_ = false;
  base_exp_ = -1;
  base_n_ = 0;
  last_mark_step_ = -1;
  gait_seen_ = stalled_ = false;
  prev_support_ = 0xF;
  balance_changed_ = restore_balance_ = false;
  req_sent_ = false;
  restore_divs_ = false;
  epoch_ = st.epoch;
  reboots_ = st.reboots;
  runs_++;
  phase_ = Phase::SetStream;
  phase_at_ = now;
  logf(log_, LogLevel::Info, "measure: starting (%u ticks at most, %u s; battery %u mV)", (unsigned)max_ticks(),
       (unsigned)((max_ticks() * KILO_TICK_MS + 999u) / 1000u), (unsigned)mv);
  return nullptr;
}

void Measure::send_stance() {
  kilo_set_posture_t p = {};
  p.posture = KILO_POSTURE_STANCE;
  p.flags = 0;
  link_.set_goal(KILO_MSG_SET_POSTURE, (const uint8_t*)&p, sizeof p);
}

void Measure::stop(const char* why) {
  if (!active() || phase_ == Phase::Ending) return;
  format(why_, sizeof why_, "stopped by %s", why);
  abort(why_, "STOPPED");
}

void Measure::abort(const char* why, const char* reason) {
  bool ok = str_eq(reason, "DONE");
  finish(reason, why, ok);
}

void Measure::line(const char* text) {
  if (obs_ != nullptr) obs_->measure_line(text);
}

void Measure::begin_stream(uint64_t now) {
  double body;
  if (!clock_est_.now(now, &body)) {
    abort("no telemetry from the body", "ABORTED");
    return;
  }
  int64_t cur = (int64_t)body;
  if ((double)cur > body) cur--;
  tick0_ = cur + planner::kLookahead;
  planned_ = tick0_;
  last_tick_ = tick0_ - 1;
  phase_ = Phase::Streaming;
  phase_at_ = now;
  stream_at_ = now;
  next_wake_ = now;
  const BodyState& st = link_.state();
  const Step* sp;
  int n_steps = measure::steps(&sp);
  char fields[200], cx[16], cy[16], cz[16], mass[16];
  format(fields, sizeof fields,
         "script=1 tick0=%u steps=%d max_ticks=%u home_z=%d lift_mm=%d ticks_per_mm=%d tip_fraction=40 "
         "tip_extra_mm=%d battery_mv=%u",
         (unsigned)(tick0_ & 0xFFFF), n_steps, (unsigned)max_ticks(), kHomeZ, kLiftMm, kTicksPerMm, kTipExtraMm,
         st.has_telemetry ? (unsigned)st.telemetry.batt_mv : 0u);
  if (obs_ != nullptr) obs_->measure_begin(fields);
  real xs[4] = {0, 0, 0, 0}, zs[4] = {(real)kHomeZ, (real)kHomeZ, (real)kHomeZ, (real)kHomeZ};
  planner::V3 c = model_.com(xs, zs);
  fixed2(cx, sizeof cx, c.x);
  fixed2(cy, sizeof cy, c.y);
  fixed2(cz, sizeof cz, c.z);
  fixed2(mass, sizeof mass, model_.total_g());
  char l[200];
  format(l, sizeof l, "model com_x=%s com_y=%s com_z=%s mass_g=%s lag_ms=%d tau_ms=%d", cx, cy, cz, mass,
         (int)planner::round_i(model_.servo_lag_ms), (int)planner::round_i(model_.servo_tau_ms));
  line(l);
  logf(log_, LogLevel::Info, "measure: streaming from body tick %u", (unsigned)(tick0_ & 0xFFFF));
}

void Measure::wake(uint64_t now) {
  const BodyState& st = link_.state();
  if (abort_why_ != nullptr) {
    abort(abort_why_, abort_reason_);
    return;
  }
  if (st.reboots != reboots_) {
    abort("the body rebooted", "ABORTED");
    return;
  }
  if (st.epoch != epoch_) {
    format(abort_buf_, sizeof abort_buf_, "the body's epoch changed (%d -> %d)", epoch_, st.epoch);
    abort(abort_buf_, "ABORTED");
    return;
  }
  if (!link_.mind_alive()) {  // nothing goes out while the mind looks dead: the body holds, then settles
    if (!stalled_) {
      stalled_ = true;
      stalled_since_ = now;
    } else if (now - stalled_since_ > kMindStallUs) {
      abort("the mind's main loop stalled", "ABORTED");
    }
    return;
  }
  stalled_ = false;
  if (!gait_seen_ && now - stream_at_ > kEnterTimeoutUs) {
    abort("the body did not enter GAIT", "ABORTED");
    return;
  }
  double body;
  if (!clock_est_.now(now, &body)) return;
  int64_t cur = (int64_t)body;
  if ((double)cur > body) cur--;
  if (phase_ == Phase::Finish) {
    if (cur >= last_tick_ + 2) finish("DONE", "done", true);
    return;
  }
  int64_t target = cur + planner::kLookahead;
  while (planned_ <= target) {
    Pose pose;
    bool first = false;
    int step = script_.step_index();
    if (!script_.next(&pose, &first)) {
      phase_ = Phase::Finish;
      return;
    }
    step = script_.step_index();
    uint32_t ts = (uint32_t)(planned_ - tick0_);
    real xs[4], zs[4];
    for (int k = 0; k < 4; k++) {
      xs[k] = pose.x[k];
      zs[k] = pose.z[k];
    }
    uint8_t sup = 0xF;
    planner::Attitude a = rigid_rest(model_, xs, zs, prev_support_, &sup);
    prev_support_ = sup;
    Planned& pl = ring_[ts % kRing];
    pl = {(int64_t)ts, (int16_t)step, a16(a.pitch), a16(a.roll)};
    if (first && step != last_mark_step_) {
      last_mark_step_ = step;
      const Step& s = script_.step();
      char l[120];
      format(l, sizeof l, "step i=%d exp=%s phase=%s t=%u tick=%u", step, kExperimentTable[s.exp].name, s.phase,
             (unsigned)ts, (unsigned)(planned_ & 0xFFFF));
      line(l);
    }
    if (planned_ <= cur) {  // already past: not sent
      late_++;
    } else {
      kilo_feet_t f = {};
      f.epoch = (uint8_t)epoch_;
      f.tick = (uint16_t)(planned_ & 0xFFFF);
      for (int k = 0; k < 4; k++) {
        f.x_mm[k] = pose.x[k];
        f.z_mm[k] = pose.z[k];
      }
      f.pitch = pl.pitch_a16;
      f.roll = pl.roll_a16;
      f.support = sup;
      if (link_.send_stream(KILO_MSG_FEET, (const uint8_t*)&f, sizeof f)) sent_++;
    }
    last_tick_ = planned_;
    planned_++;
  }
}

// The run's end, whatever ended it: STAND, the goals put back, then Ending until the body has left GAIT.
void Measure::finish(const char* reason, const char* why, bool ok) {
  if (phase_ == Phase::Idle || phase_ == Phase::Ending) return;
  bool streamed = phase_ == Phase::Streaming || phase_ == Phase::Finish;
  kilo_set_posture_t p = {};
  p.posture = KILO_POSTURE_STAND;
  link_.set_goal(KILO_MSG_SET_POSTURE, (const uint8_t*)&p, sizeof p);
  if (balance_changed_) {
    balance_changed_ = false;
    if (restore_balance_) link_.set_goal(KILO_MSG_SET_BALANCE, balance_before_, sizeof balance_before_);
    else link_.clear_goal(KILO_MSG_SET_BALANCE);
  }
  restore_divs_ = req_sent_ || phase_ != Phase::SetStream;
  restore_at_ = clock_.now_us();
  format(ended_buf_, sizeof ended_buf_, "%s", why);
  ended_ = ended_buf_;
  ok_ = ok;
  const BodyState& st = link_.state();
  char l[200];
  uint32_t played = streamed ? script_.tick() : 0;
  format(l, sizeof l, "end reason=%s ticks=%u sent=%u late=%u tips=%u battery_mv=%u why=%s", reason, (unsigned)played,
         (unsigned)sent_, (unsigned)late_, (unsigned)tips_, st.has_telemetry ? (unsigned)st.telemetry.batt_mv : 0u,
         why);
  logf(log_, ok ? LogLevel::Info : LogLevel::Warn, "measure: %s (%u ticks, %u s, %u tips)", why, (unsigned)played,
       (unsigned)((played * KILO_TICK_MS + 500u) / 1000u), (unsigned)tips_);
  if (obs_ != nullptr) {
    if (streamed) obs_->measure_end(l, reason, ok);
    else obs_->measure_line(l);
  }
  phase_ = Phase::Ending;
  phase_at_ = clock_.now_us();
}

void Measure::ending(uint64_t now) {
  if (restore_divs_ && link_.connected()) {
    kilo_set_stream_t s = {KILO_TELEMETRY_DEFAULT_DIV, 0, 0};
    if (link_.request(KILO_MSG_SET_STREAM, (const uint8_t*)&s, sizeof s) || now - restore_at_ > 1000000u)
      restore_divs_ = false;
  }
  const BodyState& st = link_.state();
  bool left = !st.has_telemetry || st.telemetry.mode != KILO_MODE_GAIT;
  if ((left && !restore_divs_) || now - phase_at_ > kEndWaitUs) phase_ = Phase::Idle;
}

void Measure::poll() {
  uint64_t now = clock_.now_us();
  switch (phase_) {
    case Phase::Idle:
      return;
    case Phase::SetStream: {
      if (!req_sent_) {
        kilo_set_stream_t s = {1, 0, 0};
        if (link_.request(KILO_MSG_SET_STREAM, (const uint8_t*)&s, sizeof s)) {
          req_sent_ = true;
          req_seq_ = link_.req().seq;
        } else if (now - phase_at_ > kSetStreamWaitUs) {
          finish("ABORTED", "SET_STREAM could not be sent", false);
        }
        return;
      }
      const Request& r = link_.req();
      if (r.type != KILO_MSG_SET_STREAM || r.seq != req_seq_ || r.state == RequestState::Waiting) return;
      if (r.state != RequestState::Done || r.status != KILO_ACK_STATUS_OK) {
        format(abort_buf_, sizeof abort_buf_, "SET_STREAM %s",
               r.state == RequestState::Done ? ack_status_name(r.status) : "timed out");
        finish("ABORTED", abort_buf_, false);
        return;
      }
      const Goal* g = link_.goal(KILO_MSG_SET_BALANCE);
      restore_balance_ = g != nullptr;
      if (g != nullptr) mem_copy(balance_before_, g->payload, sizeof balance_before_);
      kilo_set_balance_t b = {0, KILO_BALANCE_KP_Q8, KILO_BALANCE_KD_MS, KILO_BALANCE_DEADBAND_DEG,
                              KILO_BALANCE_MAX_CORR_DEG};
      link_.set_goal(KILO_MSG_SET_BALANCE, (const uint8_t*)&b, sizeof b);
      balance_changed_ = true;
      link_.clear_goal(KILO_MSG_SET_HEAD);  // the head as the posture has it (a held head moves the mass)
      send_stance();
      phase_ = Phase::Stance;
      phase_at_ = now;
      return;
    }
    case Phase::Stance:
      if (now - phase_at_ < kStanceWaitUs) return;
      link_.clear_goal(KILO_MSG_SET_POSTURE);  // a refresh would end GAIT
      begin_stream(now);
      return;
    case Phase::Streaming:
    case Phase::Finish:
      if (now < next_wake_) return;
      wake(now);
      next_wake_ += kTickUs;
      if (next_wake_ + 2 * kTickUs < clock_.now_us()) next_wake_ = clock_.now_us();  // far behind: no burst
      return;
    case Phase::Ending:
      ending(now);
      return;
  }
}

}  // namespace measure
}  // namespace kilo
