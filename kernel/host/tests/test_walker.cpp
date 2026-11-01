// The walker (planner/walker) on the real link over a fake Serial and a
// stepped clock: the body clock, the setup (SET_STREAM, STANCE), the stream
// five ticks ahead and its catch-up, every way a stream ends, the planned
// stop's handover and the teardown. Cases follow mind/tests/test_walker.py.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "link/serial_link.h"
#include "planner/walker.h"
#include "tests/check.h"
#include "tests/fakes.h"

using namespace kilo;
using namespace kilo::planner;

namespace {

// ---- the body clock -------------------------------------------------------------------

void test_body_clock_takes_the_least_delayed_frame() {
  BodyClock bc;
  double t;
  CHECK(!bc.now(5000000, &t));
  const uint64_t delays[4] = {4000, 1000, 9000, 2000};
  for (int i = 0; i < 4; i++) bc.add((uint16_t)(100 + i), 10000000u + i * 20000u + delays[i]);
  // the frame of tick 101 arrived 1 ms after its tick: body time = local + (101 x 20 ms - 10.021 s)
  CHECK(bc.now(10021000, &t) && std::fabs(t - 101.0) < 1e-9);
  CHECK(bc.now(10031000, &t) && std::fabs(t - 101.5) < 1e-9);
  bc.add(0xFFFF, 0);  // backwards: a restart
  CHECK(bc.tick() == 0xFFFF && bc.add(2, 60000) == 0x10002);  // unwrapped across the u16 wrap
}

void test_body_clock_follows_a_body_stall() {
  BodyClock bc;
  const uint64_t t0 = 10000000;
  for (int i = 0; i < 100; i++) bc.add((uint16_t)i, t0 + i * 20000u + 1000u + (i % 3) * 2000u);
  double t;
  CHECK(bc.now(t0 + 99 * 20000u + 1000u, &t) && std::fabs(t - 99.0) < 1e-9);
  const uint64_t stall = 80000;  // the body's tick falls back 80 ms
  for (int i = 100; i < 100 + kClockResetFrames; i++) bc.add((uint16_t)i, t0 + i * 20000u + stall + 1000u);
  uint64_t rx = t0 + (100 + kClockResetFrames - 1) * 20000u + stall + 1000u;
  CHECK(bc.now(rx, &t) && std::fabs(t - (100 + kClockResetFrames - 1)) < 0.15 && bc.resets() == 1);
  // a late frame or two (the reader paused) does not restart it
  bc.add(105, rx + 20000 + 50000);
  bc.add(106, rx + 40000 + 1000);
  CHECK(bc.resets() == 1 && bc.now(rx + 41000, &t) && std::fabs(t - 106) < 0.15);
}

// ---- the walk on a link ------------------------------------------------------------------

struct Rig : LinkListener {
  FakeSerial serial;
  FakeClock clock;
  CaptureLog log;
  SerialLink link{serial, clock, log};
  Walker walker{link, clock, log};
  uint8_t body_seq = 0;
  uint16_t tick = 50;
  uint8_t epoch = 3;
  uint8_t mode = KILO_MODE_IDLE;
  bool enter_gait = true;  // the body enters GAIT on the first FEET
  bool alive = true;       // the mind's loop stamps its liveness
  uint64_t body_t0 = 0;    // the clock at body tick 0
  std::vector<kilo_feet_t> feet;
  std::vector<Sent> other;

  Rig() { link.set_listener(this); }
  void on_telemetry(const kilo_telemetry_t& t) override { walker.on_telemetry(t); }
  void on_event(const kilo_event_t& e) override { walker.on_event(e); }

  void collect() {
    for (const Sent& s : serial.take_sent()) {
      if (s.type == KILO_MSG_FEET && s.payload.size() == sizeof(kilo_feet_t)) {
        kilo_feet_t f;
        std::memcpy(&f, s.payload.data(), sizeof f);
        feet.push_back(f);
        if (enter_gait && mode == KILO_MODE_IDLE) mode = KILO_MODE_GAIT;
      } else {
        other.push_back(s);
      }
    }
  }
  void poll(bool walker_too = true) {
    if (alive) link.mark_alive();
    link.poll();
    if (walker_too) walker.poll();
    collect();
  }
  void telemetry() {
    kilo_telemetry_t t = {};
    t.tick = tick++;
    t.mode = mode;
    t.epoch = epoch;
    serial.feed_frame(KILO_MSG_TELEMETRY, body_seq++, &t, sizeof t);
  }
  void event(uint8_t code, int16_t arg) {
    kilo_event_t e = {};
    e.code = code;
    e.arg = arg;
    e.epoch = epoch;
    serial.feed_frame(KILO_MSG_EVENT, body_seq++, &e, sizeof e);
  }
  // Time passes in 4 ms steps; the body sends TELEMETRY 1 ms after each of its ticks.
  void run_ms(uint32_t ms, bool walker_too = true) {
    for (uint32_t i = 0; i < ms; i += 4) {
      clock.advance_ms(4);
      if (clock.t >= body_t0 + (uint64_t)tick * 20000u + 1000u) telemetry();
      poll(walker_too);
    }
  }
  // Connected, HELLO (epoch 3) answered, TELEMETRY flowing.
  void up() {
    body_t0 = clock.t - (uint64_t)tick * 20000u;
    link.connect();
    poll(false);
    kilo_hello_t h = {};
    h.proto_major = 2;
    h.max_payload = KILO_MAX_PAYLOAD;
    h.joint_count = KILO_JOINT_COUNT;
    h.epoch = epoch;
    serial.feed_frame(KILO_MSG_HELLO, body_seq++, &h, sizeof h);
    run_ms(40, false);
    other.clear();
  }
  const Sent* last_of(uint8_t type) const {
    for (size_t i = other.size(); i-- > 0;)
      if (other[i].type == type) return &other[i];
    return nullptr;
  }
  // start(), ACK the SET_STREAM, STANCE for 0.6 s: then the stream runs.
  void walk(const WalkGoal& g = WalkGoal()) {
    const char* err = walker.start(g);
    CHECKF(err == nullptr, "start: %s", err ? err : "");
    poll();
    const Sent* s = last_of(KILO_MSG_SET_STREAM);
    CHECK(s != nullptr && s->payload.size() == 3 && s->payload[0] == 1);
    if (s == nullptr) return;
    kilo_ack_t a = {s->seq, KILO_MSG_SET_STREAM, KILO_ACK_STATUS_OK};
    serial.feed_frame(KILO_MSG_ACK, body_seq++, &a, sizeof a);
    run_ms(8);
    CHECK(walker.phase() == WalkPhase::Stance);
    const Goal* p = link.goal(KILO_MSG_SET_POSTURE);
    CHECK(p != nullptr && p->payload[1] == KILO_POSTURE_STANCE);
    run_ms(600);
    CHECK(walker.phase() == WalkPhase::Streaming && link.goal(KILO_MSG_SET_POSTURE) == nullptr);
  }
  double body_now() const { return ((double)clock.t - (double)body_t0 - 1000.0) / 20000.0; }
};

void test_stream_sends_five_ticks_ahead_and_catches_up() {
  Rig r;
  r.up();
  r.walk();
  CHECK(!r.feet.empty() && r.feet[0].epoch == 3);
  CHECK(r.feet[0].z_mm[0] == 76 && r.feet[0].z_mm[1] == 76 && r.feet[0].z_mm[2] == 76 && r.feet[0].z_mm[3] == 76);
  r.run_ms(400);
  CHECK(r.mode == KILO_MODE_GAIT && r.walker.active());
  // consecutive ticks, each sent before its time and at most five ahead of the body
  bool consecutive = true;
  for (size_t i = 1; i < r.feet.size(); i++) consecutive = consecutive && r.feet[i].tick == r.feet[i - 1].tick + 1;
  CHECK(consecutive);
  int ahead = (int)r.feet.back().tick - (int)r.body_now();
  CHECKF(ahead >= 4 && ahead <= 5, "last FEET %u, body at %.2f", r.feet.back().tick, r.body_now());
  // four wakes missed (the mind's loop busy, the link polled): the next one catches up
  size_t n = r.feet.size();
  r.run_ms(80, false);
  CHECK(r.feet.size() == n);
  r.run_ms(4);
  CHECKF(r.feet.size() >= n + 4 && r.walker.stream_stats().max_behind >= 4, "%zu sent, max behind %u",
         r.feet.size() - n, r.walker.stream_stats().max_behind);
  consecutive = true;
  for (size_t i = 1; i < r.feet.size(); i++) consecutive = consecutive && r.feet[i].tick == r.feet[i - 1].tick + 1;
  CHECK(consecutive);
  // far behind: ticks already past are planned (the plan stays continuous) but not sent
  r.run_ms(400, false);
  r.run_ms(4);
  CHECK(r.walker.stream_stats().skipped > 0);
  ahead = (int)r.feet.back().tick - (int)r.body_now();
  CHECKF(ahead >= 4 && ahead <= 5, "after the catch-up %d ahead", ahead);
  CHECK(r.walker.active());
}

void test_stream_ends(int what, const char* why) {
  Rig r;
  r.up();
  r.walk();
  r.run_ms(100);
  r.other.clear();
  switch (what) {
    case 0: r.event(KILO_EVENT_GAIT_STOPPED, 1); break;
    case 1: r.event(KILO_EVENT_GAIT_STOPPED, 0); break;
    case 2: r.event(KILO_EVENT_GOAL_REFUSED, (int16_t)(KILO_MSG_FEET << 8 | KILO_GOAL_REFUSED_REASON_FALLEN)); break;
    case 3: r.event(KILO_EVENT_FELL, 1); break;
    case 4: r.epoch = 9; break;
  }
  r.run_ms(60);
  CHECKF(!r.walker.active() && r.walker.ended() != nullptr && std::strstr(r.walker.ended(), why) != nullptr &&
             !r.walker.ended_ok(),
         "%d: ended %s", what, r.walker.ended() ? r.walker.ended() : "(null)");
  // teardown: the telemetry rate back to the default
  const Sent* s = r.last_of(KILO_MSG_SET_STREAM);
  CHECK(s != nullptr && s->payload[0] == KILO_TELEMETRY_DEFAULT_DIV);
  size_t n = r.feet.size();
  r.run_ms(100);
  CHECK(r.feet.size() == n);
}

void test_stream_ends_when_gait_never_comes_or_goes_away() {
  Rig r;
  r.enter_gait = false;
  r.up();
  r.walk();
  r.run_ms(800);
  CHECKF(!r.walker.active() && r.walker.ended() && std::strstr(r.walker.ended(), "did not enter GAIT"), "%s",
         r.walker.ended() ? r.walker.ended() : "(null)");
  Rig q;
  q.up();
  q.walk();
  q.run_ms(100);
  CHECK(q.mode == KILO_MODE_GAIT);
  q.mode = KILO_MODE_IDLE;
  q.run_ms(60);
  CHECKF(!q.walker.active() && q.walker.ended() && std::strstr(q.walker.ended(), "left GAIT (mode IDLE)"), "%s",
         q.walker.ended() ? q.walker.ended() : "(null)");
}

void test_stream_stops_sending_while_the_mind_stalls() {
  Rig r;
  r.up();
  r.walk();
  r.run_ms(100);
  r.alive = false;  // the link's liveness lasts 1 s, then the stream holds, then ends
  r.run_ms(1000);
  size_t n = r.feet.size();
  r.run_ms(300);
  CHECK(r.feet.size() == n && r.walker.active());
  r.run_ms(300);
  CHECKF(!r.walker.active() && r.walker.ended() && std::strstr(r.walker.ended(), "stalled"), "%s",
         r.walker.ended() ? r.walker.ended() : "(null)");
}

void test_stream_handover_on_the_planned_stop() {
  Rig r;
  r.up();
  r.walk();
  r.run_ms(300);
  r.walker.stop();
  for (int i = 0; i < 400 && !r.walker.planner().done(); i++) r.run_ms(20);
  CHECK(r.walker.planner().done());
  r.run_ms(8);
  const Goal* g = r.link.goal(KILO_MSG_SET_POSTURE);
  CHECK(g != nullptr && g->payload[1] == KILO_POSTURE_STANCE);  // the posture ends GAIT
  size_t n = r.feet.size();
  r.mode = KILO_MODE_IDLE;  // the body left GAIT for the posture
  r.event(KILO_EVENT_GAIT_STOPPED, 0);
  r.run_ms(60);
  CHECKF(!r.walker.active() && r.walker.ended() && std::strcmp(r.walker.ended(), "stopped") == 0 &&
             r.walker.ended_ok(),
         "%s", r.walker.ended() ? r.walker.ended() : "(null)");
  CHECK(r.feet.size() == n);                                // nothing after the plan's end
  CHECK(r.link.goal(KILO_MSG_SET_POSTURE) == nullptr);       // the STANCE goal cleared again
  const Sent* s = r.last_of(KILO_MSG_SET_STREAM);
  CHECK(s != nullptr && s->payload[0] == KILO_TELEMETRY_DEFAULT_DIV);
  const TickPlan& last = r.walker.planner().last();
  CHECK(last.support == 0xF);
}

void test_balance_off_for_the_walk_and_restored() {
  Rig r;
  r.up();
  kilo_set_balance_t before = {1, 100, 20, 3, 12};
  r.link.set_goal(KILO_MSG_SET_BALANCE, (const uint8_t*)&before, sizeof before);
  WalkGoal g;
  g.balance = false;
  r.walk(g);
  const Goal* b = r.link.goal(KILO_MSG_SET_BALANCE);
  CHECK(b != nullptr && b->payload[0] == 0 && b->payload[1] == KILO_BALANCE_KP_Q8);
  r.walker.abort();
  b = r.link.goal(KILO_MSG_SET_BALANCE);
  CHECK(b != nullptr && std::memcmp(b->payload, &before, sizeof before) == 0);
  CHECK(r.walker.ended() && std::strcmp(r.walker.ended(), "aborted") == 0 && !r.walker.active());
}

void test_changes_while_walking_and_a_start_during_the_stop() {
  Rig r;
  r.up();
  r.walk();
  r.run_ms(100);
  WalkGoal g;
  g.pattern = find_pattern("walk");
  g.direction = Direction::Left;
  g.speed_mm_s = 8;
  CHECK(r.walker.start(g) == nullptr);
  CHECK(r.walker.planner().changing() && r.walker.planner().direction() == Direction::Left);
  r.walker.stop();
  CHECK(r.walker.planner().stopping() && !r.walker.planner().changing());
  WalkGoal t;  // a trot asked for during the stop: after it
  CHECK(r.walker.start(t) == nullptr);
  for (int i = 0; i < 600 && !r.walker.planner().done(); i++) r.run_ms(20);
  r.mode = KILO_MODE_IDLE;
  r.run_ms(60);
  CHECKF(r.walker.ended_ok() || r.walker.phase() != WalkPhase::Streaming, "%s",
         r.walker.ended() ? r.walker.ended() : "(null)");
  CHECK(r.walker.phase() == WalkPhase::SetStream);  // the queued walk's setup
}

void test_refusals() {
  Rig r;
  CHECK(r.walker.start(WalkGoal()) != nullptr);  // no link, no telemetry
  r.up();
  r.run_ms(400, false);
  r.serial.in.clear();
  r.serial.in_pos = 0;
  r.clock.advance_ms(400);  // TELEMETRY stopped: the body is lost
  r.poll(false);
  const char* err = r.walker.start(WalkGoal());
  CHECKF(err && std::strstr(err, "no telemetry"), "%s", err ? err : "(null)");
  Rig f;
  f.up();
  f.mode = KILO_MODE_FALLEN;
  f.run_ms(40, false);
  err = f.walker.start(WalkGoal());
  CHECKF(err && std::strstr(err, "Kilo is down"), "%s", err ? err : "(null)");
  // a SET_STREAM refused ends the setup
  Rig s;
  s.up();
  CHECK(s.walker.start(WalkGoal()) == nullptr);
  s.poll();
  const Sent* q = s.last_of(KILO_MSG_SET_STREAM);
  CHECK(q != nullptr);
  if (q) {
    kilo_ack_t a = {q->seq, KILO_MSG_SET_STREAM, KILO_ACK_STATUS_BUSY};
    s.serial.feed_frame(KILO_MSG_ACK, s.body_seq++, &a, sizeof a);
  }
  s.run_ms(20);
  CHECKF(!s.walker.active() && s.walker.ended() && std::strstr(s.walker.ended(), "SET_STREAM"), "%s",
         s.walker.ended() ? s.walker.ended() : "(null)");
}

}  // namespace

int main() {
  test_body_clock_takes_the_least_delayed_frame();
  test_body_clock_follows_a_body_stall();
  test_stream_sends_five_ticks_ahead_and_catches_up();
  test_stream_ends(0, "settled after a FEET underrun");
  test_stream_ends(1, "a posture or an epoch bump");
  test_stream_ends(2, "refused FEET: REFUSED_FALLEN");
  test_stream_ends(3, "Kilo fell");
  test_stream_ends(4, "epoch changed");
  test_stream_ends_when_gait_never_comes_or_goes_away();
  test_stream_stops_sending_while_the_mind_stalls();
  test_stream_handover_on_the_planned_stop();
  test_balance_off_for_the_walk_and_restored();
  test_changes_while_walking_and_a_start_during_the_stop();
  test_refusals();
  return report("walker");
}
