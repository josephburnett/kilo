#include "record/recorder.h"
#include "link/names.h"
#include "util/fmt.h"
#include "util/mem.h"

namespace kilo {
namespace record {

namespace {

using planner::real;

// One MARK line, "word key=value key=value ...", built a field at a time.
// Out of line: a walk's start writes some sixty fields, which inlined cost
// the image tens of KB.
class Line {
 public:
  explicit Line(const char* word) __attribute__((noinline));
  void num(const char* key, int v) __attribute__((noinline));
  void unum(const char* key, unsigned v) __attribute__((noinline));
  void hex(const char* key, unsigned v) __attribute__((noinline));
  void str(const char* key, const char* v) __attribute__((noinline));
  // A real with this many decimals (1..3); "nan", "inf", "-inf" past +-2e6.
  void fixed(const char* key, real v, int decimals) __attribute__((noinline));
  const char* text() const { return buf_; }

 private:
  char buf_[250];
  size_t n_;
};

Line::Line(const char* word) { n_ = format(buf_, sizeof buf_, "%s", word); }
void Line::num(const char* key, int v) { n_ += format(buf_ + n_, sizeof buf_ - n_, " %s=%d", key, v); }
void Line::unum(const char* key, unsigned v) { n_ += format(buf_ + n_, sizeof buf_ - n_, " %s=%u", key, v); }
void Line::hex(const char* key, unsigned v) { n_ += format(buf_ + n_, sizeof buf_ - n_, " %s=%08x", key, v); }
void Line::str(const char* key, const char* v) { n_ += format(buf_ + n_, sizeof buf_ - n_, " %s=%s", key, v); }

void Line::fixed(const char* key, real v, int decimals) {
  if (v != v) return str(key, "nan");
  if (v > (real)2e6 || v < (real)-2e6) return str(key, v > 0 ? "inf" : "-inf");
  uint32_t scale = decimals == 1 ? 10u : decimals == 2 ? 100u : 1000u;
  int32_t q = planner::round_i(v * (real)scale);  // |q| < 2e9
  const char* sign = q < 0 ? "-" : "";
  uint32_t a = q < 0 ? (uint32_t)-q : (uint32_t)q;
  const char* f = decimals == 1 ? " %s=%s%u.%01u" : decimals == 2 ? " %s=%s%u.%02u" : " %s=%s%u.%03u";
  n_ += format(buf_ + n_, sizeof buf_ - n_, f, key, sign, (unsigned)(a / scale), (unsigned)(a % scale));
}

int16_t a16(real deg) {
  long v = (long)planner::round_i(deg * 16);
  return (int16_t)(v < -32767 ? -32767 : v > 32767 ? 32767 : v);
}

int16_t mm10(real mm) {
  if (mm != mm || mm > (real)3276 || mm < (real)-3276) return -32768;  // none, or off the scale
  return (int16_t)planner::round_i(mm * 10);
}

}  // namespace

WalkRecorder::WalkRecorder(Clock& clock, const SerialLink& link) : clock_(clock), link_(link) {}

void WalkRecorder::attach(uint8_t* storage, uint32_t bytes, uint32_t mind_build) {
  mind_build_ = mind_build;
  pre_ = nullptr;
  pre_head_ = pre_n_ = 0;
  recording_ = ending_ = false;
  uint32_t pre = kPreRoll * (uint32_t)sizeof(PreRec) + 8;
  if (storage == nullptr || bytes <= pre) {
    store_.attach(nullptr, 0);
    return;
  }
  uintptr_t a = ((uintptr_t)storage + 7u) & ~(uintptr_t)7u;
  pre_ = reinterpret_cast<PreRec*>(a);
  store_.attach(storage + pre, bytes - pre);
  if (!store_.ready()) pre_ = nullptr;
}

uint32_t WalkRecorder::rel(uint64_t now) const {
  if (now <= t0_) return 0;
  uint64_t d = now - t0_;
  return d > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)d;
}

bool WalkRecorder::frame(uint8_t kind, uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len, uint64_t now) {
  uint8_t ts[2] = {type, seq};
  return store_.append(kind, rel(now), ts, 2, payload, len);
}

void WalkRecorder::mark_line(const char* line, uint64_t now, bool essential) {
  uint32_t n = 0;
  while (line[n] != '\0' && n < 255) n++;
  store_.append(KILO_WALK_RECORD_MARK, rel(now), line, n, nullptr, 0, essential);
}

void WalkRecorder::on_frame(bool tx, uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len, uint64_t now) {
  if (pre_ == nullptr) return;
  if (len > KILO_MAX_PAYLOAD) return;  // cannot be: the link checks lengths
  uint8_t kind = tx ? KILO_WALK_RECORD_TX : KILO_WALK_RECORD_RX;
  PreRec& p = pre_[(pre_head_ + pre_n_) % kPreRoll];
  if (pre_n_ < kPreRoll) pre_n_++;
  else pre_head_ = (pre_head_ + 1) % kPreRoll;
  p.t_us = now;
  p.kind = kind;
  p.type = type;
  p.seq = seq;
  p.len = len;
  mem_copy(p.payload, payload, len);
  if (!recording_) return;
  frame(kind, type, seq, payload, len, now);
  WalkEntry* e = store_.current();
  if (tx && type == KILO_MSG_FEET && e != nullptr) e->feet++;
}

void WalkRecorder::poll() {
  if (recording_ && ending_ && clock_.now_us() >= end_at_) finish();
}

void WalkRecorder::finish() {
  store_.close();
  recording_ = ending_ = false;
}

void WalkRecorder::set_text(const planner::Walker& w, const char* end) {
  WalkEntry* e = store_.current();
  if (e == nullptr) return;
  const planner::WalkGoal& g = w.goal();
  real speed = (e->flags & KILO_WALK_FLAGS_STREAMED) ? w.planner().timing().speed_mm_s : g.speed_mm_s;
  // "gait=trot dir=forward speed=55.6[ end=REASON]" (Line puts a space before each key)
  Line s("");
  s.str("gait", g.pattern->name);
  s.str("dir", planner::direction_name(g.direction));
  if (speed >= 0) s.fixed("speed", speed, 1);
  else s.str("speed", "default");
  if (end != nullptr) s.str("end", end);
  char buf[KILO_HOST_WALK_TEXT_MAX + 2];
  size_t n = format(buf, sizeof buf, "%s", s.text() + 1);  // without the leading space
  if (n > KILO_HOST_WALK_TEXT_MAX) n = KILO_HOST_WALK_TEXT_MAX;
  mem_copy(e->text, buf, n);
  e->text_len = (uint8_t)n;
}

void WalkRecorder::goal_line(const char* word, const planner::WalkGoal& g, uint64_t now) {
  Line l(word);
  l.str("pattern", g.pattern->name);
  l.str("dir", planner::direction_name(g.direction));
  if (g.speed_mm_s >= 0) l.fixed("speed_asked", g.speed_mm_s, 1);
  else l.str("speed_asked", "default");
  l.unum("balance", g.balance ? 1u : 0u);
  mark_line(l.text(), now);
}

void WalkRecorder::timing_line(const char* word, const planner::Walker& w, uint64_t now) {
  const planner::Planner& p = w.planner();
  planner::Timing tm = p.timing();
  Line l(word);
  l.unum("tick", (unsigned)p.tick());
  l.str("pattern", p.pattern().name);
  l.str("dir", planner::direction_name(p.direction()));
  l.fixed("speed", tm.speed_mm_s, 1);
  l.fixed("speed_set", p.speed_mm_s(), 1);
  l.fixed("stride_mm", tm.stride_mm, 1);
  l.fixed("period_ms", tm.period_ms, 1);
  l.fixed("shift_ms", tm.shift_ms, 1);
  l.fixed("stance_ms", tm.stance_ms, 1);
  l.num("swing_ticks", p.swing_ticks());
  l.num("lead", p.lead());
  if (p.changing()) l.str("pending", p.pending_pattern()->name);
  if (p.stopping()) l.unum("stopping", 1);
  mark_line(l.text(), now);
}

void WalkRecorder::begin(const planner::Walker& w, uint64_t now) {
  if (recording_) finish();  // the previous walk's post-roll ends here
  uint32_t id = store_.open(now);
  if (id == 0) return;
  recording_ = true;
  ending_ = false;
  last_ = {};
  // t0: the oldest pre-roll frame within kPreRollUs
  t0_ = now;
  uint32_t first = pre_n_;
  for (uint32_t i = 0; i < pre_n_; i++) {
    const PreRec& p = pre_[(pre_head_ + i) % kPreRoll];
    if (now - p.t_us <= kPreRollUs) {
      first = i;
      t0_ = p.t_us;
      break;
    }
  }
  const BodyState& b = link_.state();
  Line l("walk");
  l.unum("id", (unsigned)id);
  l.unum("format", KILO_HOST_WALK_FORMAT);
  // t0 on the mind's clock, in two 32-bit halves of microseconds (fmt has no 64-bit conversion)
  l.unum("t0_us_hi", (unsigned)(t0_ >> 32));
  l.unum("t0_us_lo", (unsigned)(t0_ & 0xFFFFFFFFu));
  l.unum("setup_us", rel(now));
  l.hex("mind_build", (unsigned)mind_build_);
  if (b.has_hello) {
    l.hex("body_build", (unsigned)b.hello.fw_build);
    l.unum("proto", (unsigned)b.hello.proto_major * 100u + b.hello.proto_minor);
  }
  l.num("epoch", b.epoch);
  l.unum("tick_ms", KILO_TICK_MS);
  mark_line(l.text(), t0_, true);
  for (uint32_t i = first; i < pre_n_; i++) {
    const PreRec& p = pre_[(pre_head_ + i) % kPreRoll];
    frame(p.kind, p.type, p.seq, p.payload, p.len, p.t_us);
  }
  goal_line("setup", w.goal(), now);
  set_text(w, nullptr);
}

void WalkRecorder::on_walk_mark(const planner::Walker& w, planner::WalkMark mark) {
  if (!store_.ready()) return;
  uint64_t now = clock_.now_us();
  using planner::WalkMark;
  if (mark == WalkMark::Setup) {
    begin(w, now);
    return;
  }
  if (!recording_ || ending_) return;
  WalkEntry* e = store_.current();
  switch (mark) {
    case WalkMark::Setup:
      break;
    case WalkMark::Stream: {
      if (e != nullptr) e->flags |= KILO_WALK_FLAGS_STREAMED;
      timing_line("stream", w, now);
      const planner::PlannerConfig& c = w.planner().config();
      Line l("config");
      l.fixed("margin_mm", c.margin_mm, 1);
      l.fixed("dx_max_mm", c.dx_max_mm, 1);
      l.fixed("dx_speed_mm_s", c.dx_speed_mm_s, 1);
      l.fixed("roll_max_deg", c.roll_max_deg, 1);
      l.fixed("pitch_max_deg", c.pitch_max_deg, 1);
      l.fixed("dx_scale_mm", c.dx_scale_mm, 1);
      l.fixed("pitch_scale_deg", c.pitch_scale_deg, 1);
      l.fixed("roll_scale_deg", c.roll_scale_deg, 1);
      l.fixed("gate_deg", c.gate_deg, 1);
      l.num("hold_max", c.hold_max);
      l.fixed("catch_deg", c.catch_deg, 1);
      l.unum("lean", c.lean ? 1u : 0u);
      l.unum("fixed_shift", c.fixed_shift ? 1u : 0u);
      l.fixed("shift_ms", c.shift_ms, 1);
      mark_line(l.text(), now);
      const planner::BodyModel& m = w.planner().model();
      Line g("model");
      g.fixed("l1_mm", m.l1_mm, 1);
      g.fixed("l2_mm", m.l2_mm, 1);
      g.fixed("foot_r_mm", m.foot_r_mm, 1);
      g.fixed("length_mm", m.length_mm, 1);
      g.fixed("track_mm", m.track_mm, 1);
      g.fixed("servo_lag_ms", m.servo_lag_ms, 1);
      g.fixed("slew_deg_per_tick", m.slew_deg_per_tick, 1);
      g.fixed("reach_frac", m.reach_frac, 3);
      g.fixed("knee_max_deg", m.knee_max_deg, 1);
      g.fixed("total_g", m.total_g(), 1);
      mark_line(g.text(), now);
      // the lumped masses: grams at x, y, z (mm, body frame)
      Line ms("masses");
      const planner::Mass* mass[3] = {&m.body, &m.battery, &m.pi};
      const char* names[3][4] = {{"body_g", "body_x", "body_y", "body_z"},
                                 {"battery_g", "battery_x", "battery_y", "battery_z"},
                                 {"pi_g", "pi_x", "pi_y", "pi_z"}};
      for (int i = 0; i < 3; i++) {
        ms.fixed(names[i][0], mass[i]->grams, 1);
        ms.fixed(names[i][1], mass[i]->x, 1);
        ms.fixed(names[i][2], mass[i]->y, 1);
        ms.fixed(names[i][3], mass[i]->z, 1);
      }
      ms.fixed("upper_leg_g", m.upper_leg_g, 1);
      ms.fixed("upper_leg_at", m.upper_leg_at, 2);
      ms.fixed("lower_leg_g", m.lower_leg_g, 1);
      ms.fixed("lower_leg_at", m.lower_leg_at, 2);
      mark_line(ms.text(), now);
      set_text(w, nullptr);
      break;
    }
    case WalkMark::Change:
      goal_line("change", w.goal(), now);
      if (w.phase() == planner::WalkPhase::Streaming) timing_line("timing", w, now);
      set_text(w, nullptr);
      break;
    case WalkMark::Stop: {
      Line l("stop");
      l.unum("tick", (unsigned)w.planner().tick());
      mark_line(l.text(), now);
      break;
    }
    case WalkMark::Queued:
      goal_line("queued", w.queued_goal(), now);
      break;
    case WalkMark::End: {
      bool streamed = e != nullptr && (e->flags & KILO_WALK_FLAGS_STREAMED);
      const char* why = w.ended() != nullptr ? w.ended() : "?";
      const planner::StreamStats& s = w.stream_stats();
      const planner::PlannerStats& ps = w.planner().stats();
      Line r("result");
      r.unum("sent", s.sent);
      r.unum("skipped", s.skipped);
      r.unum("underruns", s.underruns);
      r.unum("max_behind", s.max_behind);
      r.unum("dropped_rx", s.dropped_rx);
      if (streamed) {
        r.unum("holds", ps.holds);
        r.unum("catches", ps.catches);
        r.unum("waits", ps.waits);
        r.fixed("min_margin_mm", ps.min_margin, 1);
        r.unum("tick", (unsigned)w.planner().tick());
      }
      r.unum("clock_resets", w.body_clock().resets());
      mark_line(r.text(), now, true);
      planner::ErrorEstimate est;
      if (streamed && w.tracker().estimate(&est)) {
        Line x("error");
        x.unum("samples", est.samples);
        x.fixed("offset_pitch_deg", est.offset_pitch_deg, 2);
        x.fixed("offset_roll_deg", est.offset_roll_deg, 2);
        if (est.has_lag) {
          x.fixed("lag_ms", est.lag_ms, 1);
          x.fixed("lag_quality", est.lag_quality, 2);
        }
        x.fixed("spike_deg", est.spike_deg, 2);
        x.fixed("corr_pitch_deg", est.corr_pitch_deg, 2);
        x.fixed("corr_roll_deg", est.corr_roll_deg, 2);
        mark_line(x.text(), now, true);
      }
      Line l("end");
      l.unum("ok", w.ended_ok() ? 1u : 0u);
      l.unum("streamed", streamed ? 1u : 0u);
      l.str("reason", why);  // last: it may hold spaces
      mark_line(l.text(), now, true);
      if (e != nullptr) {
        e->end_us = now;
        if (w.ended_ok()) e->flags |= KILO_WALK_FLAGS_ENDED_OK;
      }
      set_text(w, why);
      ending_ = true;
      end_at_ = now + kPostRollUs;
      break;
    }
  }
}

void WalkRecorder::on_walk_tick(const planner::Walker& w, const planner::TickPlan& tp, bool sent) {
  if (!recording_ || ending_) return;
  uint64_t now = clock_.now_us();
  kilo_walk_plan_t p;
  p.tick = tp.tick;
  p.sent = sent ? 1 : 0;
  p.lifting = tp.lifting;
  p.support = tp.support;
  p.margin = mm10(tp.margin);
  p.plan_pitch = a16(tp.plan_pitch);
  p.plan_roll = a16(tp.plan_roll);
  p.pitch = a16(tp.pitch);
  p.roll = a16(tp.roll);
  store_.append(KILO_WALK_RECORD_PLAN, rel(now), &p, sizeof p);
  const planner::PlannerStats& s = w.planner().stats();
  if (s.holds != last_.holds || s.catches != last_.catches || s.waits != last_.waits) {
    kilo_walk_planner_t q;
    q.tick = tp.tick;
    q.holds = s.holds;
    q.catches = s.catches;
    q.waits = s.waits;
    q.min_margin = mm10(s.min_margin);
    store_.append(KILO_WALK_RECORD_PLANNER, rel(now), &q, sizeof q);
    last_ = s;
  }
}

}  // namespace record
}  // namespace kilo
