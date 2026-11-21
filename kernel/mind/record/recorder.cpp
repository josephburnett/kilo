#include "record/recorder.h"
#include "link/names.h"
#include "tricks/library.h"
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
  n_ += format(buf_ + n_, sizeof buf_ - n_, " %s=", key);
  n_ += format_fixed(buf_ + n_, sizeof buf_ - n_, planner::fixed_i(v, decimals), decimals);  // |n| < 2e9
}

int16_t a16(real deg) {
  long v = (long)planner::round_i(to_a16(deg));
  return (int16_t)(v < -32767 ? -32767 : v > 32767 ? 32767 : v);
}

int16_t mm10(real mm) {
  if (mm != mm || mm > (real)3276 || mm < (real)-3276) return -32768;  // none, or off the scale
  return (int16_t)planner::round_i(mm * 10);
}

constexpr uint64_t kKeyRecentUs = 5000000;  // a trick started this soon after a remote key: by=remote

}  // namespace

void NamedCounts::add(const char* name) {
  total_++;
  if (name == nullptr) name = "?";
  // the key: the text up to a ':', '(' or digit, spaces as '_', at most kNameMax - 1 characters
  char key[kNameMax];
  uint32_t k = 0;
  for (; k < kNameMax - 1 && name[k] != '\0' && name[k] != ':' && name[k] != '(' && !(name[k] >= '0' && name[k] <= '9');
       k++)
    key[k] = name[k] == ' ' ? '_' : name[k];
  while (k > 0 && key[k - 1] == '_') k--;
  if (k == 0) key[k++] = '?';
  key[k] = '\0';
  for (uint32_t i = 0; i < n_; i++) {
    uint32_t j = 0;
    while (key[j] != '\0' && key[j] == names_[i][j]) j++;
    if (key[j] == names_[i][j]) {
      counts_[i]++;
      return;
    }
  }
  if (n_ == kSlots) {
    other_++;
    return;
  }
  for (uint32_t j = 0; j <= k; j++) names_[n_][j] = key[j];
  counts_[n_++] = 1;
}

Recorder::Recorder(Clock& clock, const SerialLink& link) : clock_(clock), link_(link) {}

void Recorder::attach(uint8_t* storage, uint32_t bytes, uint32_t mind_build) {
  mind_build_ = mind_build;
  flight_.attach(storage, bytes);
}

// ---- runs ---------------------------------------------------------------------------------

Run* Recorder::run(uint32_t id) {
  for (uint32_t i = 0; i < n_runs_; i++) {
    Run& r = runs_[(head_ + i) % kMaxRuns];
    if (r.id == id) return &r;
  }
  return nullptr;
}

const Run* Recorder::find(uint32_t id) const { return const_cast<Recorder*>(this)->run(id); }

const Run* Recorder::run_after(uint32_t after) const {
  for (uint32_t i = 0; i < n_runs_; i++) {
    const Run& r = runs_[(head_ + i) % kMaxRuns];
    if (r.id > after) return &r;  // ids grow with the index order
  }
  return nullptr;
}

Run* Recorder::begin_run(uint8_t kind, uint64_t now) {
  if (n_runs_ == kMaxRuns) {  // the oldest entry goes (its blocks may still be in the stream)
    head_ = (head_ + 1) % kMaxRuns;
    n_runs_--;
  }
  Run& r = runs_[(head_ + n_runs_++) % kMaxRuns];
  mem_zero(&r, sizeof r);
  r.id = next_id_++;
  r.kind = kind;
  r.flags = KILO_WALK_FLAGS_RECORDING | (kind == KILO_RUN_KIND_TRICK ? KILO_WALK_FLAGS_TRICK : 0);
  r.start_us = now;
  runs_changed_++;
  run_mark(r.id, kind, KILO_RUN_EVENT_BEGIN, now);
  return &r;
}

void Recorder::end_run(uint32_t id, uint64_t now) {
  Run* r = run(id);
  if (r == nullptr) return;
  r->end_us = now;
  run_mark(id, r->kind, KILO_RUN_EVENT_END, now);
  last_end_us_ = now;
  if (now + kPostRollUs > tail_end_us_) tail_end_us_ = now + kPostRollUs;
  runs_changed_++;
}

bool Recorder::span(const Run& r, uint32_t* first_seq, uint32_t* last_seq) const {
  if ((r.flags & KILO_WALK_FLAGS_RECORDING) || r.end_us == 0 || flight_.empty()) return false;
  uint64_t a = r.start_us > kPreRollUs ? r.start_us - kPreRollUs : 0;
  uint32_t f = flight_.seq_at(a), l = flight_.seq_at(r.end_us + kPostRollUs);
  if (flight_.t0(f) > r.start_us && flight_.first_seq() > 0) return false;  // its start is overwritten
  *first_seq = f;
  *last_seq = l;
  return true;
}

void Recorder::poll(const tricks::TrickPlayer& p) {
  uint64_t now = clock_.now_us();
  const tricks::PlayResult& r = p.result();
  if (r.play != 0 && r.play != trick_play_) {  // a new play
    if (trick_ != 0) end_trick("FAILED", "another play began", 0, 0, now);
    const tricks::Trick* t = p.trick();
    if (t != nullptr && t->name[0] == 'a' && t->name[1] == 'l' && t->name[2] == 'i' && t->name[3] == 'v' &&
        t->name[4] == 'e') {  // the alive layer's small sitting moves (behaviour/alive.h): a MARK, not a run
      trick_play_ = r.play;
      mark_line("alive trick=alive_sit", now);
    } else {
      begin_trick(p, now);
    }
  }
  if (trick_ != 0) {
    if (r.uploaded && !upload_marked_) {
      upload_marked_ = true;
      mark_line("uploaded", now);
    }
    bool acked = p.playing() || (r.state == tricks::PlayState::Done);
    if (acked && !play_marked_) {
      play_marked_ = true;
      Line l("play");
      l.unum("speed", p.speed());
      mark_line(l.text(), p.played_at_us() != 0 ? p.played_at_us() : now);
      Run* run_ = run(trick_);
      if (run_ != nullptr) run_->flags |= KILO_WALK_FLAGS_STREAMED;
    }
    if (r.state == tricks::PlayState::Done)
      end_trick(tricks::done_reason_name(r.reason), "", r.elapsed_ms, r.expected_ticks, now);
    else if (r.state == tricks::PlayState::Failed)
      end_trick("FAILED", r.error != nullptr ? r.error : "?", 0, r.expected_ticks, now);
  }
  if (tail_end_us_ != 0 && now >= tail_end_us_) tail_end_us_ = 0;
  for (uint32_t i = 0; i < n_runs_; i++) {  // post-rolls over: fetchable
    Run& x = runs_[(head_ + i) % kMaxRuns];
    if ((x.flags & KILO_WALK_FLAGS_RECORDING) && x.end_us != 0 && now >= x.end_us + kPostRollUs) {
      x.flags &= (uint8_t)~KILO_WALK_FLAGS_RECORDING;
      runs_changed_++;
    }
  }
}

void Recorder::trick_header(const tricks::Trick& t, uint32_t id, uint8_t speed, bool mirror, uint64_t now) {
  const BodyState& b = link_.state();
  Line l("trick");
  l.unum("id", (unsigned)id);
  l.unum("format", KILO_HOST_WALK_FORMAT);
  l.str("trick", t.name);
  if (stage_of_ != 0) {
    l.unum("stage", (unsigned)stage_allowed_);
    l.unum("of", (unsigned)stage_of_);
  }
  l.unum("tid", tricks::trick_id_for(t));
  char crc[8];
  format(crc, sizeof crc, "%04x", (unsigned)tricks::trick_crc16(t));
  l.str("crc", crc);
  l.unum("frames", t.n_frames);
  l.unum("speed", speed);
  l.unum("mirror", mirror ? 1u : 0u);
  l.hex("mind_build", (unsigned)mind_build_);
  if (b.has_hello) l.hex("body_build", (unsigned)b.hello.fw_build);
  if (b.has_telemetry) {
    l.unum("battery_mv", b.telemetry.batt_mv);
    l.str("mode", mode_name(b.telemetry.mode));
  }
  l.str("by", have_key_ && now - last_key_us_ < kKeyRecentUs ? "remote" : "other");
  mark_line(l.text(), now);
  // each keyframe's ticks at 1x, and the loop
  char buf[250];
  size_t n = format(buf, sizeof buf, "frames dur=");
  for (uint8_t i = 0; i < t.n_frames; i++) n += format(buf + n, sizeof buf - n, i ? ",%u" : "%u", (unsigned)t.frames[i].dur);
  n += format(buf + n, sizeof buf - n, " hold=");
  for (uint8_t i = 0; i < t.n_frames; i++) n += format(buf + n, sizeof buf - n, i ? ",%u" : "%u", (unsigned)t.frames[i].hold);
  format(buf + n, sizeof buf - n, " loop=%u,%u,%u", (unsigned)t.loop_start, (unsigned)t.loop_end, (unsigned)t.repeats);
  mark_line(buf, now);
}

void Recorder::begin_trick(const tricks::TrickPlayer& p, uint64_t now) {
  trick_play_ = p.result().play;
  play_marked_ = upload_marked_ = false;
  const tricks::Trick* t = p.trick();
  Run* r = begin_run(KILO_RUN_KIND_TRICK, now);
  trick_ = r->id;
  stage_of_ = 0;
  stage_allowed_ = 0;
  stage_reached_ = -1;
  if (t == nullptr) return;
  trick_name_ = t->name;
  int of = (int)tricks::rear_up_stage(t->name);
  if (of != 0 && pending_play_ == trick_play_ && pending_of_ == of) {  // the remote's: up a stage at a time
    stage_of_ = of;
    stage_allowed_ = pending_allowed_;
    stage_reached_ = pending_reached_;
  } else if (of != 0) {  // another client's play: the whole trick
    stage_of_ = stage_allowed_ = of;
  }
  pending_play_ = 0;
  trick_header(*t, r->id, p.speed(), p.mirrored(), now);
  trick_text(r);
  if (metrics_ != nullptr) {
    metrics_->tricks++;
    metrics_->trick_names.add(t->name);
  }
}

void Recorder::end_trick(const char* reason, const char* why, uint32_t elapsed_ms, uint32_t expected, uint64_t now) {
  const BodyState& b = link_.state();
  Line l("end");
  l.str("reason", reason);
  l.unum("elapsed_ms", elapsed_ms);
  l.unum("expected_ticks", expected);
  if (b.has_telemetry) {
    l.unum("battery_mv", b.telemetry.batt_mv);
    l.str("mode", mode_name(b.telemetry.mode));
  }
  Run* r = run(trick_);
  if (stage_of_ != 0) {
    if (reason[0] == 'D') stage_allowed_ = stage_reached_ = stage_of_;  // DONE: the whole trick played
    trick_text(r);
    l.unum("stage", (unsigned)stage_allowed_);
    l.unum("of", (unsigned)stage_of_);
    if (stage_reached_ >= 0) l.unum("reached", (unsigned)stage_reached_);
  }
  l.str("why", why);  // last: it may hold spaces
  mark_line(l.text(), now);
  if (r != nullptr) {
    if (reason[0] == 'D') r->flags |= KILO_WALK_FLAGS_ENDED_OK;  // DONE
    char text[KILO_HOST_WALK_TEXT_MAX + 1];
    format(text, sizeof text, "%s end=%s", r->text, reason);
    set_text(r, text);
  }
  if (metrics_ != nullptr) metrics_->trick_ends.add(reason);
  end_run(trick_, now);
  trick_ = 0;
  stage_of_ = 0;
}

// The trick run's label: "trick=NAME [stage=S of=N [reached=R]]" (reached only when short of S's top).
void Recorder::trick_text(Run* r) {
  char text[KILO_HOST_WALK_TEXT_MAX + 1];
  if (stage_of_ == 0)
    format(text, sizeof text, "trick=%s", trick_name_);
  else if (stage_reached_ >= 0 && stage_reached_ < stage_allowed_)
    format(text, sizeof text, "trick=%s stage=%d of=%d reached=%d", trick_name_, stage_allowed_, stage_of_,
           stage_reached_);
  else
    format(text, sizeof text, "trick=%s stage=%d of=%d", trick_name_, stage_allowed_, stage_of_);
  set_text(r, text);
}

void Recorder::trick_stage(uint32_t play, int allowed, int reached, int of) {
  if (trick_ == 0 || play != trick_play_ || stage_of_ != of) {  // its run opens at the player's next poll
    pending_play_ = play;
    pending_allowed_ = allowed;
    pending_reached_ = reached;
    pending_of_ = of;
    return;
  }
  bool up = allowed > stage_allowed_ || reached > stage_reached_;
  if (!up) return;
  if (allowed > stage_allowed_) stage_allowed_ = allowed;
  if (reached > stage_reached_) stage_reached_ = reached;
  Line l("stage");
  l.unum("allowed", (unsigned)stage_allowed_);
  l.unum("reached", (unsigned)(stage_reached_ < 0 ? 0 : stage_reached_));
  l.unum("of", (unsigned)stage_of_);
  mark_line(l.text(), clock_.now_us());
  trick_text(run(trick_));
}

void Recorder::trick_refused(const char* trick, const char* why) {
  if (!attached()) return;
  uint64_t now = clock_.now_us();
  if (trick_ != 0) end_trick("FAILED", "another play began", 0, 0, now);
  Run* r = begin_run(KILO_RUN_KIND_TRICK, now);
  trick_ = r->id;
  stage_of_ = 0;  // nothing played: no stage
  stage_allowed_ = 0;
  stage_reached_ = -1;
  trick_name_ = trick;
  const tricks::Trick* t = tricks::find_trick(trick);
  if (t != nullptr) trick_header(*t, r->id, tricks::kSpeedUnity, false, now);
  trick_text(r);
  if (metrics_ != nullptr) {
    metrics_->tricks++;
    metrics_->trick_names.add(trick);
    metrics_->refusals.add(why);
  }
  end_trick("REFUSED", why, 0, 0, now);
}

void Recorder::measure_begin(const char* fields) {
  if (!attached()) return;
  uint64_t now = clock_.now_us();
  if (measure_ != 0) measure_end("end reason=ABORTED why=another measurement began", "ABORTED", false);
  Run* r = begin_run(KILO_RUN_KIND_TRICK, now);
  measure_ = r->id;
  const BodyState& b = link_.state();
  char line[250];
  size_t n = format(line, sizeof line, "measure id=%u format=1 mind_build=%08x", (unsigned)r->id, (unsigned)mind_build_);
  if (b.has_hello) n += format(line + n, sizeof line - n, " body_build=%08x", (unsigned)b.hello.fw_build);
  format(line + n, sizeof line - n, " %s", fields);
  mark_line(line, now);
  set_text(r, "measure=self");
  r->flags |= KILO_WALK_FLAGS_STREAMED;
}

void Recorder::measure_end(const char* line, const char* reason, bool ok) {
  if (measure_ == 0) return;
  uint64_t now = clock_.now_us();
  mark_line(line, now);
  Run* r = run(measure_);
  if (r != nullptr) {
    if (ok) r->flags |= KILO_WALK_FLAGS_ENDED_OK;
    char text[KILO_HOST_WALK_TEXT_MAX + 1];
    format(text, sizeof text, "measure=self end=%s", reason);
    set_text(r, text);
  }
  end_run(measure_, now);
  measure_ = 0;
}

void Recorder::key(const char* text) {
  uint64_t now = clock_.now_us();
  last_key_us_ = now;
  have_key_ = true;
  if (!in_run()) return;
  char buf[200];
  format(buf, sizeof buf, "key %s", text);
  mark_line(buf, now);
}

// ---- records --------------------------------------------------------------------------------

void Recorder::run_mark(uint32_t id, uint8_t kind, uint8_t event, uint64_t now) {
  kilo_run_mark_t m;
  m.id = id;
  m.kind = kind;
  m.event = event;
  flight_.append(KILO_WALK_RECORD_RUN, now, &m, sizeof m);
}

void Recorder::mark_line(const char* line, uint64_t now) {
  uint32_t n = 0;
  while (line[n] != '\0' && n < 255) n++;
  flight_.append(KILO_WALK_RECORD_MARK, now, line, n);
}

void Recorder::set_text(Run* r, const char* text) {
  if (r == nullptr) return;
  uint32_t n = 0;
  while (text[n] != '\0' && n < KILO_HOST_WALK_TEXT_MAX) n++;
  mem_copy(r->text, text, n);
  if (n < KILO_HOST_WALK_TEXT_MAX) r->text[n] = '\0';
  r->text_len = (uint8_t)n;
  runs_changed_++;
}

void Recorder::write(LogLevel level, const char* line) {
  uint8_t lv = (uint8_t)level;
  uint32_t n = 0;
  while (line[n] != '\0' && n < 254) n++;
  flight_.append(KILO_WALK_RECORD_LOG, clock_.now_us(), &lv, 1, line, n);
}

void Recorder::on_frame(bool tx, uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len, uint64_t now) {
  if (tx && type == KILO_MSG_HEARTBEAT) {
    flight_.count_skipped(now);
    return;
  }
  uint8_t ts[2] = {type, seq};
  flight_.append(tx ? KILO_WALK_RECORD_TX : KILO_WALK_RECORD_RX, now, ts, 2, payload, len);
  if (tx && type == KILO_MSG_FEET && walk_ != 0) {
    Run* r = run(walk_);
    if (r != nullptr) r->feet++;
  }
}

// ---- walks ----------------------------------------------------------------------------------

void Recorder::walk_text(const planner::Walker& w, const char* end) {
  Run* r = run(walk_);
  if (r == nullptr) return;
  const planner::WalkGoal& g = w.goal();
  real speed = (r->flags & KILO_WALK_FLAGS_STREAMED) ? w.planner().timing().speed_mm_s : g.speed_mm_s;
  // "gait=trot dir=forward speed=55.6[ end=REASON]" (Line puts a space before each key)
  Line s("");
  s.str("gait", g.pattern->name);
  s.str("dir", planner::direction_name(g.direction));
  if (speed >= 0) s.fixed("speed", speed, 1);
  else s.str("speed", "default");
  if (end != nullptr) s.str("end", end);
  set_text(r, s.text() + 1);  // without the leading space
}

void Recorder::goal_line(const char* word, const planner::WalkGoal& g, uint64_t now) {
  Line l(word);
  l.str("pattern", g.pattern->name);
  l.str("dir", planner::direction_name(g.direction));
  if (g.steer != 0) l.fixed("steer", g.steer, 2);  // the remote's A6 / C6 while walking (+ left)
  if (g.speed_mm_s >= 0) l.fixed("speed_asked", g.speed_mm_s, 1);
  else l.str("speed_asked", "default");
  l.unum("balance", g.balance ? 1u : 0u);
  mark_line(l.text(), now);
}

void Recorder::timing_line(const char* word, const planner::Walker& w, uint64_t now) {
  const planner::Planner& p = w.planner();
  planner::Timing tm = p.timing();
  Line l(word);
  l.unum("tick", (unsigned)p.tick());
  l.str("pattern", p.pattern().name);
  l.str("dir", planner::direction_name(p.direction()));
  if (p.steer() != 0) l.fixed("steer", p.steer(), 2);
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

void Recorder::on_walk_mark(const planner::Walker& w, planner::WalkMark mark) {
  if (!attached()) return;
  uint64_t now = clock_.now_us();
  using planner::WalkMark;
  if (mark == WalkMark::Setup) {
    if (walk_ != 0) end_run(walk_, now);  // cannot be: a walk ends before the next one's setup
    Run* r = begin_run(KILO_RUN_KIND_WALK, now);
    walk_ = r->id;
    last_ = {};
    const BodyState& b = link_.state();
    Line l("walk");
    l.unum("id", (unsigned)r->id);
    l.unum("format", KILO_HOST_WALK_FORMAT);
    // t0 on the mind's clock, in two 32-bit halves of microseconds (fmt has no 64-bit conversion)
    l.unum("t0_us_hi", (unsigned)(now >> 32));
    l.unum("t0_us_lo", (unsigned)(now & 0xFFFFFFFFu));
    l.unum("setup_us", 0);
    l.hex("mind_build", (unsigned)mind_build_);
    if (b.has_hello) {
      l.hex("body_build", (unsigned)b.hello.fw_build);
      l.unum("proto", (unsigned)b.hello.proto_major * 100u + b.hello.proto_minor);
    }
    l.num("epoch", b.epoch);
    l.unum("tick_ms", KILO_TICK_MS);
    if (b.has_telemetry) l.unum("battery_mv", b.telemetry.batt_mv);
    mark_line(l.text(), now);
    goal_line("setup", w.goal(), now);
    walk_text(w, nullptr);
    if (metrics_ != nullptr) metrics_->walks++;
    return;
  }
  if (walk_ == 0) return;
  Run* r = run(walk_);
  switch (mark) {
    case WalkMark::Setup:
      break;
    case WalkMark::Stream: {
      if (r != nullptr) r->flags |= KILO_WALK_FLAGS_STREAMED;
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
      g.fixed("servo_tau_ms", m.servo_tau_ms, 1);
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
      walk_text(w, nullptr);
      break;
    }
    case WalkMark::Change:
      goal_line("change", w.goal(), now);
      if (w.phase() == planner::WalkPhase::Streaming) timing_line("timing", w, now);
      walk_text(w, nullptr);
      break;
    case WalkMark::Stop: {
      Line l("stop");
      l.unum("tick", (unsigned)w.planner().tick());
      if (w.planner().halting()) {  // the quick stop: from this tick on, the ticks before it planned again
        l.unum("quick", 1);
        l.unum("replanned", w.replanned());
      }
      mark_line(l.text(), now);
      break;
    }
    case WalkMark::Queued:
      goal_line("queued", w.queued_goal(), now);
      break;
    case WalkMark::End: {
      bool streamed = r != nullptr && (r->flags & KILO_WALK_FLAGS_STREAMED);
      const char* why = w.ended() != nullptr ? w.ended() : "?";
      const planner::StreamStats& s = w.stream_stats();
      const planner::PlannerStats& ps = w.planner().stats();
      Line res("result");
      res.unum("sent", s.sent);
      res.unum("skipped", s.skipped);
      res.unum("underruns", s.underruns);
      res.unum("max_behind", s.max_behind);
      res.unum("dropped_rx", s.dropped_rx);
      if (streamed) {
        res.unum("holds", ps.holds);
        res.unum("catches", ps.catches);
        res.unum("waits", ps.waits);
        res.fixed("min_margin_mm", ps.min_margin, 1);
        res.unum("tick", (unsigned)w.planner().tick());
      }
      res.unum("clock_resets", w.body_clock().resets());
      mark_line(res.text(), now);
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
        mark_line(x.text(), now);
      }
      Line l("end");
      l.unum("ok", w.ended_ok() ? 1u : 0u);
      l.unum("streamed", streamed ? 1u : 0u);
      l.str("reason", why);  // last: it may hold spaces
      mark_line(l.text(), now);
      if (r != nullptr && w.ended_ok()) r->flags |= KILO_WALK_FLAGS_ENDED_OK;
      walk_text(w, why);
      if (metrics_ != nullptr) {
        if (w.ended_ok()) metrics_->walks_ok++;
        metrics_->walk_ends.add(why);
        if (streamed) {
          metrics_->walk_holds += ps.holds;
          metrics_->walk_catches += ps.catches;
        }
      }
      end_run(walk_, now);
      walk_ = 0;
      break;
    }
  }
}

void Recorder::on_walk_tick(const planner::Walker& w, const planner::TickPlan& tp, bool sent) {
  if (walk_ == 0) return;
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
  flight_.append(KILO_WALK_RECORD_PLAN, now, &p, sizeof p);
  const planner::PlannerStats& s = w.planner().stats();
  if (s.holds != last_.holds || s.catches != last_.catches || s.waits != last_.waits) {
    kilo_walk_planner_t q;
    q.tick = tp.tick;
    q.holds = s.holds;
    q.catches = s.catches;
    q.waits = s.waits;
    q.min_margin = mm10(s.min_margin);
    flight_.append(KILO_WALK_RECORD_PLANNER, now, &q, sizeof q);
    last_ = s;
  }
}

}  // namespace record
}  // namespace kilo
