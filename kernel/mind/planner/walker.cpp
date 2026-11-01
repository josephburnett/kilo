#include "planner/walker.h"
#include "link/names.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/mem.h"

namespace kilo {
namespace planner {

// ---- the body clock -------------------------------------------------------------------

void BodyClock::reset() {
  head_ = n_ = 0;
  has_raw_ = false;
  max_ = 0;
  below_ = 0;
  tick_ = 0;
  resets_ = 0;
}

int64_t BodyClock::add(uint16_t raw_tick, uint64_t rx_us) {
  if (!has_raw_) {
    tick_ = raw_tick;
  } else {
    uint16_t d = (uint16_t)(raw_tick - raw_);
    if (d >= 0x8000) {  // backwards: a restart (the stream ends on the reboot anyway)
      head_ = n_ = 0;
      below_ = 0;
      tick_ = raw_tick;
    } else {
      tick_ += d;
    }
  }
  has_raw_ = true;
  raw_ = raw_tick;
  int64_t off = tick_ * (int64_t)kTickUs - (int64_t)rx_us;
  bool full = n_ == kClockWindow;
  int64_t dropped = full ? offs_[head_] : 0;
  if (full) {
    offs_[head_] = off;
    head_ = (head_ + 1) % kClockWindow;
  } else {
    offs_[(head_ + n_) % kClockWindow] = off;
    n_++;
  }
  if (full && dropped >= max_) {
    max_ = offs_[0];
    for (int i = 1; i < n_; i++) max_ = offs_[i] > max_ ? offs_[i] : max_;
  } else if (off > max_ || n_ == 1) {
    max_ = off;
  }
  if (off < max_ - (int64_t)kTickUs) {
    if (++below_ >= kClockResetFrames) {  // the body's tick fell back: start again from these
      int64_t recent[kClockResetFrames];
      for (int i = 0; i < kClockResetFrames; i++)
        recent[i] = offs_[(head_ + n_ - kClockResetFrames + i) % kClockWindow];
      head_ = 0;
      n_ = kClockResetFrames;
      max_ = recent[0];
      for (int i = 0; i < kClockResetFrames; i++) {
        offs_[i] = recent[i];
        max_ = recent[i] > max_ ? recent[i] : max_;
      }
      below_ = 0;
      resets_++;
    }
  } else {
    below_ = 0;
  }
  return tick_;
}

bool BodyClock::now(uint64_t t_us, double* tick) const {
  if (n_ == 0) return false;
  *tick = ((double)t_us + (double)max_) / (double)kTickUs;
  return true;
}

void pack_feet(uint8_t epoch, const TickPlan& tp, kilo_feet_t* out) {
  out->epoch = epoch;
  out->tick = (uint16_t)(tp.tick & 0xFFFF);
  for (int i = 0; i < kLegs; i++) {
    out->x_mm[i] = tp.x[i];
    out->z_mm[i] = tp.z[i];
  }
  int p = round_i(tp.pitch * 16), r = round_i(tp.roll * 16);
  out->pitch = (int16_t)(p < -480 ? -480 : p > 480 ? 480 : p);
  out->roll = (int16_t)(r < -480 ? -480 : r > 480 ? 480 : r);
  out->support = tp.support;
}

// ---- the walker -----------------------------------------------------------------------

Walker::Walker(SerialLink& link, Clock& clock, Log& log, const BodyModel& model, const PlannerConfig& config)
    : link_(link), clock_(clock), log_(log), planner_(model, config) {}

void Walker::set_stream_divs(uint8_t telem, uint8_t joints, uint8_t imu_raw) {
  divs_[0] = telem;
  divs_[1] = joints;
  divs_[2] = imu_raw;
}

void Walker::on_telemetry(const kilo_telemetry_t& t) {
  uint64_t rx = link_.state().telemetry_at_us;
  seen_[(seen_head_ + seen_n_) % kClockWindow] = {t.tick, rx};
  if (seen_n_ < kClockWindow) seen_n_++;
  else seen_head_ = (seen_head_ + 1) % kClockWindow;
  if (phase_ != WalkPhase::Streaming) return;
  if (tel_n_ == kQueue) {  // a wake very late: the oldest goes (BodyState.history keeps the newest too)
    tel_head_ = (tel_head_ + 1) % kQueue;
    tel_n_--;
    stats_.dropped_rx++;
  }
  tel_q_[(tel_head_ + tel_n_++) % kQueue] = {t, rx};
}

void Walker::on_event(const kilo_event_t& e) {
  if (phase_ != WalkPhase::Streaming) return;
  if (ev_n_ == kQueue) {
    ev_head_ = (ev_head_ + 1) % kQueue;
    ev_n_--;
    stats_.dropped_rx++;
  }
  ev_q_[(ev_head_ + ev_n_++) % kQueue] = e;
}

const char* Walker::check_can_walk() const {
  const BodyState& st = link_.state();
  if (st.proto_ok == 0) return "protocol mismatch: refusing to drive";
  if (st.epoch == kNoEpoch || !st.alive(clock_.now_us(), KILO_BODY_LOST_MS * 1000u))
    return "no telemetry from the body";
  uint8_t mode = st.telemetry.mode;
  if (mode == KILO_MODE_FALLEN || mode == KILO_MODE_RECOVERING) return "Kilo is down: the fall reflex holds the joints";
  return nullptr;
}

const char* Walker::start(const WalkGoal& goal) {
  uint64_t now = clock_.now_us();
  if (phase_ == WalkPhase::Streaming && !planner_.stopping()) {
    update(goal);
    return nullptr;
  }
  if (phase_ == WalkPhase::SetStream || phase_ == WalkPhase::Stance) {  // not streaming yet: walk this instead
    goal_ = goal;
    return nullptr;
  }
  if (phase_ == WalkPhase::Streaming) {  // a planned stop under way: after it (or after kStartWaitUs)
    queued_ = true;
    queued_goal_ = goal;
    queued_at_ = now;
    return nullptr;
  }
  const char* err = check_can_walk();
  if (err != nullptr) return err;
  begin_setup(goal, now);
  return nullptr;
}

void Walker::update(const WalkGoal& goal) {
  planner_.set_pattern(*goal.pattern);
  planner_.set_direction(goal.direction);
  if (goal.speed_mm_s >= 0) planner_.set_speed(goal.speed_mm_s);
  goal_ = goal;
}

void Walker::stop() {
  queued_ = false;
  if (phase_ == WalkPhase::Streaming) {
    planner_.stop();
  } else if (phase_ != WalkPhase::Idle) {  // not streaming yet: no walk at all
    end("stopped before the stream started", true);
    teardown(false);
  }
}

void Walker::abort() {
  queued_ = false;
  if (phase_ == WalkPhase::Idle) return;
  bool streamed = phase_ == WalkPhase::Streaming;
  end("aborted", false);
  teardown(streamed);
}

void Walker::begin_setup(const WalkGoal& goal, uint64_t now) {
  goal_ = goal;
  ended_ = nullptr;
  ok_ = false;
  if (!restore_divs_) telem_div_before_ = divs_[0];  // else the previous walk's rate is still to be put back
  restore_divs_ = false;
  balance_changed_ = false;
  stream_req_sent_ = false;
  phase_ = WalkPhase::SetStream;
  phase_at_ = now;
  setup_step(now);
}

void Walker::posture_stance(bool balance) {
  kilo_set_posture_t p = {};
  p.posture = KILO_POSTURE_STANCE;
  p.flags = balance ? KILO_POSTURE_FLAGS_BALANCE : 0;
  link_.set_goal(KILO_MSG_SET_POSTURE, (const uint8_t*)&p, sizeof p);
}

void Walker::setup_step(uint64_t now) {
  if (phase_ == WalkPhase::SetStream) {
    const Request& r = link_.req();
    if (!stream_req_sent_) {
      kilo_set_stream_t s = {1, divs_[1], divs_[2]};
      if (link_.request(KILO_MSG_SET_STREAM, (const uint8_t*)&s, sizeof s)) {
        stream_req_sent_ = true;
        stream_req_seq_ = link_.req().seq;
      } else if (now - phase_at_ > 1000000u) {  // the request slot stayed busy, or no link
        end("SET_STREAM could not be sent", false);
        teardown(false);
      }
      return;
    }
    if (r.type != KILO_MSG_SET_STREAM || r.seq != stream_req_seq_ || r.state == RequestState::Waiting) return;
    if (r.state != RequestState::Done || r.status != KILO_ACK_STATUS_OK) {
      format(why_, sizeof why_, "SET_STREAM %s",
             r.state == RequestState::Done ? ack_status_name(r.status) : "timed out");
      end(why_, false);
      teardown(false);
      return;
    }
    divs_[0] = 1;
    const Goal* g = link_.goal(KILO_MSG_SET_POSTURE);
    if (g != nullptr)  // the posture's balance flag, carried through the walk and its end
      posture_balance_ = (g->payload[3] & KILO_POSTURE_FLAGS_BALANCE) != 0;
    if (!goal_.balance) {
      g = link_.goal(KILO_MSG_SET_BALANCE);
      restore_balance_ = g != nullptr;
      if (g != nullptr) mem_copy(balance_before_, g->payload, sizeof balance_before_);
      kilo_set_balance_t b = {0, KILO_BALANCE_KP_Q8, KILO_BALANCE_KD_MS, KILO_BALANCE_DEADBAND_DEG,
                              KILO_BALANCE_MAX_CORR_DEG};
      link_.set_goal(KILO_MSG_SET_BALANCE, (const uint8_t*)&b, sizeof b);
      balance_changed_ = true;
    }
    // STANCE, also when the body still holds GAIT from a stream that ended
    // without the planned stop: the posture ends that.
    posture_stance(goal_.balance && posture_balance_);
    phase_ = WalkPhase::Stance;
    phase_at_ = now;
    return;
  }
  if (phase_ == WalkPhase::Stance && now - phase_at_ >= kStanceWaitUs) {
    link_.clear_goal(KILO_MSG_SET_POSTURE);  // a refresh would end GAIT
    if (!begin_stream(now)) {
      end("no telemetry from the body", false);
      teardown(false);
    }
  }
}

bool Walker::begin_stream(uint64_t now) {
  clock_est_.reset();
  for (int i = 0; i < seen_n_; i++) {
    const Seen& s = seen_[(seen_head_ + i) % kClockWindow];
    clock_est_.add(s.tick, s.rx_us);
  }
  double now_tick;
  if (!clock_est_.now(now, &now_tick)) return false;
  int64_t first = (int64_t)now_tick;
  if ((double)first > now_tick) first--;
  tracker_.reset();
  planner_.start(*goal_.pattern, goal_.direction, goal_.speed_mm_s, (uint32_t)(first + kLookahead));
  const BodyState& st = link_.state();
  epoch_ = st.epoch;
  reboots_ = st.reboots;
  tel_head_ = tel_n_ = ev_head_ = ev_n_ = 0;
  stats_ = {};
  gait_seen_ = has_last_tel_ = handover_ = started_ = stalled_ = false;
  for (Pred& p : pred_) p.used = false;
  phase_ = WalkPhase::Streaming;
  next_wake_ = now;
  int v10 = round_i(planner_.timing().speed_mm_s * 10);
  logf(log_, LogLevel::Info, "walk %s %s at %d.%d mm/s", goal_.pattern->name, direction_name(goal_.direction),
       v10 / 10, v10 % 10);
  return true;
}

void Walker::end(const char* why, bool ok) {
  if (ended_ != nullptr) return;
  format(ended_buf_, sizeof ended_buf_, "%s", why);
  ended_ = ended_buf_;
  ok_ = ok;
  logf(log_, ok ? LogLevel::Info : LogLevel::Warn, "walk ended: %s (sent %u skipped %u underruns %u)", ended_,
       (unsigned)stats_.sent, (unsigned)stats_.skipped, (unsigned)stats_.underruns);
}

void Walker::teardown(bool streamed) {
  if (streamed && handover_) {
    const Goal* g = link_.goal(KILO_MSG_SET_POSTURE);
    if (g != nullptr && g->payload[1] == KILO_POSTURE_STANCE) link_.clear_goal(KILO_MSG_SET_POSTURE);
  }
  restore_divs_ = true;  // a monitor's JOINTS / IMU_RAW divisors go on
  send_stream_divs_step();
  if (balance_changed_) {
    balance_changed_ = false;
    if (restore_balance_) link_.set_goal(KILO_MSG_SET_BALANCE, balance_before_, sizeof balance_before_);
    else link_.clear_goal(KILO_MSG_SET_BALANCE);
    restore_balance_ = false;
  }
  phase_ = WalkPhase::Idle;
}

void Walker::send_stream_divs_step() {
  if (!restore_divs_ || !link_.connected()) return;
  kilo_set_stream_t s = {telem_div_before_, divs_[1], divs_[2]};
  if (link_.request(KILO_MSG_SET_STREAM, (const uint8_t*)&s, sizeof s)) {
    restore_divs_ = false;
    divs_[0] = telem_div_before_;
  }
}

// ---- the stream -----------------------------------------------------------------------

void Walker::ingest() {
  for (; ev_n_ > 0; ev_head_ = (ev_head_ + 1) % kQueue, ev_n_--) {
    const kilo_event_t& e = ev_q_[ev_head_];
    if (e.code == KILO_EVENT_GAIT_STOPPED) {
      if (handover_) end("stopped", true);
      else if (e.arg == 1) end("the body settled after a FEET underrun (the stream was late)", false);
      else end("the body stopped the gait (a posture or an epoch bump took the legs)", false);
    } else if (e.code == KILO_EVENT_FEET_UNDERRUN) {
      stats_.underruns++;
    } else if (e.code == KILO_EVENT_GOAL_REFUSED) {
      uint16_t a = (uint16_t)e.arg;
      if ((a >> 8) == KILO_MSG_FEET) {
        format(why_, sizeof why_, "the body refused FEET: %s", ack_status_name((uint8_t)(a & 0xFF)));
        end(why_, false);
      }
    } else if (e.code == KILO_EVENT_FELL) {
      end("Kilo fell", false);
    }
  }
  for (; tel_n_ > 0; tel_head_ = (tel_head_ + 1) % kQueue, tel_n_--) {
    const Tel& q = tel_q_[tel_head_];
    const kilo_telemetry_t& m = q.t;
    int64_t tick = clock_est_.add(m.tick, q.rx_us);
    has_last_tel_ = true;
    last_mode_ = m.mode;
    if (m.mode == KILO_MODE_GAIT) {
      gait_seen_ = true;
      const Pred& p = pred_[m.tick % kPred];
      const Attitude* ref = p.used && p.tick == m.tick ? &p.att : nullptr;
      Attitude e = tracker_.add(tick, m.pitch, m.roll, m.ref_pitch, m.ref_roll, m.corr_pitch, m.corr_roll, ref);
      planner_.observe(e.pitch, e.roll);
    } else if (gait_seen_ && !handover_ && ended_ == nullptr) {
      format(why_, sizeof why_, "the body left GAIT (mode %s)", mode_name(m.mode));
      end(why_, false);
    }
  }
}

// One 20 ms step of the stream (Stream.wake). False once it is over.
bool Walker::wake(uint64_t now) {
  if (ended_ != nullptr) return false;
  ingest();
  if (ended_ != nullptr) return false;
  const BodyState& st = link_.state();
  if (st.reboots != reboots_) {
    end("the body rebooted", false);
    return false;
  }
  if (st.epoch != epoch_) {
    format(why_, sizeof why_, "the body's epoch changed (%d -> %d)", epoch_, st.epoch);
    end(why_, false);
    return false;
  }
  if (handover_) {
    if (gait_seen_ && has_last_tel_ && last_mode_ != KILO_MODE_GAIT) {
      end("stopped", true);
      return false;
    }
    if (now - handover_at_ > kHandoverTimeoutUs) {
      end("stopped (no GAIT_STOPPED seen after the STANCE posture)", true);
      return false;
    }
    return true;
  }
  if (!link_.mind_alive()) {
    if (!stalled_) {
      stalled_ = true;
      stalled_since_ = now;
    } else if (now - stalled_since_ > kMindStallUs) {
      end("the mind's main loop stalled", false);
      return false;
    }
    return true;  // nothing goes out while the mind looks dead: the body holds, then settles
  }
  stalled_ = false;
  double body;
  if (!clock_est_.now(now, &body)) return true;
  int64_t cur = (int64_t)body;
  if ((double)cur > body) cur--;
  if (!started_) {
    started_ = true;
    started_at_ = now;
  } else if (!gait_seen_ && now - started_at_ > kEnterTimeoutUs) {
    end("the body did not enter GAIT", false);
    return false;
  }
  int64_t target = cur + kLookahead;
  uint32_t behind = 0;
  while ((int64_t)planner_.tick() <= target && !planner_.done()) {
    const TickPlan& tp = planner_.next();
    if ((int64_t)tp.tick <= cur) {
      stats_.skipped++;
      continue;
    }
    behind++;
    kilo_feet_t f;
    pack_feet((uint8_t)epoch_, tp, &f);
    if (link_.send_stream(KILO_MSG_FEET, (const uint8_t*)&f, sizeof f)) {
      stats_.sent++;
      Pred& p = pred_[f.tick % kPred];
      p = {true, f.tick, {tp.pitch, tp.roll}};
    }
  }
  if (behind > stats_.max_behind) stats_.max_behind = behind;
  if (planner_.done() && !handover_) {
    // every planned tick is out; the STANCE posture ends GAIT after them
    handover_ = true;
    handover_at_ = now;
    posture_stance(posture_balance_);
  }
  return true;
}

void Walker::poll() {
  uint64_t now = clock_.now_us();
  send_stream_divs_step();
  switch (phase_) {
    case WalkPhase::Idle:
      break;
    case WalkPhase::SetStream:
    case WalkPhase::Stance:
      setup_step(now);
      break;
    case WalkPhase::Streaming:
      if (queued_ && now - queued_at_ > kStartWaitUs) end("aborted (a new walk waited for its stop)", false);
      if (now >= next_wake_) {
        bool on = wake(now);
        next_wake_ += kTickUs;
        uint64_t after = clock_.now_us();
        if (next_wake_ + 2 * kTickUs < after) next_wake_ = after;  // far behind: no burst of wakes
        if (!on) teardown(true);
      }
      break;
  }
  if (phase_ == WalkPhase::Idle && queued_) {
    queued_ = false;
    const char* err = check_can_walk();
    if (err != nullptr) logf(log_, LogLevel::Warn, "walk: the queued walk cannot start: %s", err);
    else begin_setup(queued_goal_, now);
  }
}

static void fixed(char* buf, size_t cap, real v, int decimals) {
  int scale = decimals == 2 ? 100 : 10;
  int n = round_i(v * (real)scale);
  const char* sign = n < 0 ? "-" : "";
  if (n < 0) n = -n;
  format(buf, cap, decimals == 2 ? "%s%d.%02d" : "%s%d.%d", sign, n / scale, n % scale);
}

void Walker::log_status(LogLevel level) const {
  const Planner& p = planner_;
  Timing tm = p.timing();
  char speed[16], stride[16], margin[16], off_p[16], off_r[16], spike[16];
  fixed(speed, sizeof speed, tm.speed_mm_s, 1);
  fixed(stride, sizeof stride, tm.stride_mm, 1);
  if (p.stats().min_margin < m_inf()) fixed(margin, sizeof margin, p.stats().min_margin, 1);
  else format(margin, sizeof margin, "-");
  logf(log_, level,
       "walk %s %s %s mm/s stride %s mm period %d ms%s%s: sent %u skipped %u underruns %u max behind %u "
       "holds %u catches %u waits %u min margin %s mm",
       p.pattern().name, direction_name(p.direction()), speed, stride, (int)round_i(tm.period_ms),
       p.stopping() ? " stopping" : "", p.changing() ? " changing" : "", (unsigned)stats_.sent, (unsigned)stats_.skipped,
       (unsigned)stats_.underruns, (unsigned)stats_.max_behind, (unsigned)p.stats().holds,
       (unsigned)p.stats().catches, (unsigned)p.stats().waits, margin);
  ErrorEstimate e;
  if (tracker_.estimate(&e)) {
    fixed(off_p, sizeof off_p, e.offset_pitch_deg, 2);
    fixed(off_r, sizeof off_r, e.offset_roll_deg, 2);
    fixed(spike, sizeof spike, e.spike_deg, 2);
    char lag[16];
    if (e.has_lag) format(lag, sizeof lag, "%d ms", (int)e.lag_ms);
    else format(lag, sizeof lag, "-");
    logf(log_, level, "walk error: %u samples offset pitch %s roll %s deg, lag %s, spike %s deg", (unsigned)e.samples,
         off_p, off_r, lag, spike);
  }
}

}  // namespace planner
}  // namespace kilo
