// The link state machine (link/serial_link) fed canned frames on a fake
// Serial and a stepped clock: HELLO and the protocol check, reboot
// detection, epochs and the goals they clear, contact, decode error
// counting, heartbeat and liveness timing, commands and queries with their
// retries. Cases follow kilo/link/serial_link.py's behaviour.
#include <string>
#include "link/serial_link.h"
#include "tests/check.h"
#include "tests/fakes.h"
#include "util/mem.h"

using namespace kilo;

namespace {

constexpr uint32_t kCoreBuild = 0x45524F43;  // "CORE", the host body core's fw_build

struct Rig : LinkListener {
  FakeSerial serial;
  FakeClock clock;
  CaptureLog log;
  SerialLink link{serial, clock, log};
  uint8_t body_seq = 0;

  int hellos = 0, telemetries = 0, events = 0, reboots = 0, cleared = 0, refused = 0, connections = 0;
  bool solicited = false;
  bool reboot_had_hello = false;
  std::string reboot_why, cleared_reason;
  uint8_t refused_type = 0, refused_reason = 0;

  Rig() { link.set_listener(this); }

  void on_connection(bool, bool) override { connections++; }
  void on_hello(const kilo_hello_t&, bool s) override {
    hellos++;
    solicited = s;
  }
  void on_telemetry(const kilo_telemetry_t&) override { telemetries++; }
  void on_event(const kilo_event_t&) override { events++; }
  void on_reboot(const kilo_hello_t* h, const char* why) override {
    reboots++;
    reboot_had_hello = h != nullptr;
    reboot_why = why;
  }
  void on_goals_cleared(const char* reason, uint32_t n) override {
    cleared += (int)n;
    cleared_reason = reason;
  }
  void on_refused(uint8_t type, uint8_t reason) override {
    refused++;
    refused_type = type;
    refused_reason = reason;
  }

  // The mind's loop: stamp liveness, poll.
  void step(uint32_t ms = 10) {
    clock.advance_ms(ms);
    link.mark_alive();
    link.poll();
  }
  // Time passes with the mind alive, the body streaming TELEMETRY at 25 Hz.
  void run(uint32_t ms, uint16_t* tick, uint8_t epoch) {
    for (uint32_t t = 0; t < ms; t += 10) {
      if (t % 40 == 0) telemetry((*tick)++, epoch);
      step(10);
    }
  }

  void hello(uint8_t major = 2, uint8_t cause = KILO_RESET_CAUSE_COLD, uint16_t last_alive = 0, uint8_t epoch = 0,
             uint8_t config = KILO_CONFIG_STATE_OK) {
    kilo_hello_t h = {};
    h.proto_major = major;
    h.fw_build = kCoreBuild;
    h.reset_cause = cause;
    h.last_alive_tick = last_alive;
    h.max_payload = KILO_MAX_PAYLOAD;
    h.joint_count = KILO_JOINT_COUNT;
    h.config_state = config;
    h.epoch = epoch;
    serial.feed_frame(KILO_MSG_HELLO, body_seq++, &h, sizeof h);
  }
  void telemetry(uint16_t tick, uint8_t epoch) {
    kilo_telemetry_t t = {};
    t.tick = tick;
    t.mode = KILO_MODE_IDLE;
    t.epoch = epoch;
    serial.feed_frame(KILO_MSG_TELEMETRY, body_seq++, &t, sizeof t);
  }
  void event(uint8_t code, int16_t arg, uint8_t epoch) {
    kilo_event_t e = {};
    e.code = code;
    e.arg = arg;
    e.epoch = epoch;
    serial.feed_frame(KILO_MSG_EVENT, body_seq++, &e, sizeof e);
  }
  void ack(uint8_t seq, uint8_t type, uint8_t status) {
    kilo_ack_t a = {seq, type, status};
    serial.feed_frame(KILO_MSG_ACK, body_seq++, &a, sizeof a);
  }
  // Connected, HELLO (epoch) answered, TELEMETRY flowing: the body is alive and heard.
  void up(uint8_t epoch, uint16_t* tick) {
    link.connect();
    step();
    hello(2, KILO_RESET_CAUSE_COLD, 0, epoch);
    telemetry((*tick)++, epoch);
    step();
    serial.take_sent();
  }
};

kilo_set_posture_t posture(uint8_t p) {
  kilo_set_posture_t s = {};
  s.posture = p;
  return s;
}

bool set_posture(Rig& r, uint8_t p) {
  kilo_set_posture_t s = posture(p);
  return r.link.set_goal(KILO_MSG_SET_POSTURE, (const uint8_t*)&s, sizeof s);
}

const Sent* find_sent(const std::vector<Sent>& s, uint8_t type) {
  for (const Sent& f : s)
    if (f.type == type) return &f;
  return nullptr;
}

void test_hello_good() {
  Rig r;
  r.link.connect();
  r.step();
  std::vector<Sent> s = r.serial.take_sent();
  // HELLO_REQ is traffic: no HEARTBEAT due yet
  CHECKF(s.size() == 1 && count_type(s, KILO_MSG_HELLO_REQ) == 1, "%zu frames", s.size());
  CHECK(r.link.stats().hello_polls == 1);
  r.hello(2, KILO_RESET_CAUSE_COLD, 0, 3, KILO_CONFIG_STATE_DEFAULTED);
  r.step();
  const BodyState& b = r.link.state();
  CHECK(b.has_hello && b.proto_ok == 1 && b.epoch == 3 && b.hello.fw_build == kCoreBuild);
  CHECK(r.hellos == 1 && r.solicited && r.reboots == 0);
  CHECK(r.log.has("HELLO proto 2.0 build 45524f43 reset COLD config DEFAULTED caps 0x0 epoch 3"));
  CHECK(r.link.contact());  // first HELLO on the first connection: heard
  CHECK(!b.alive(r.clock.t, KILO_BODY_LOST_MS * 1000u));
  r.telemetry(100, 3);
  r.step();
  CHECK(b.alive(r.clock.t, KILO_BODY_LOST_MS * 1000u) && b.tick == 100 && r.telemetries == 1);
  CHECK(r.log.has("body alive"));
  CHECK(r.link.stats().frames_rx == 2 && r.link.stats().decode_error_total() == 0);
}

void test_hello_wrong_major() {
  Rig r;
  r.link.connect();
  r.step();
  r.hello(1);
  r.step();
  CHECK(r.link.state().proto_ok == 0);
  CHECK(r.log.has("protocol major 1 != expected 2: refusing to drive; flash the body"));
  CHECK(!set_posture(r, KILO_POSTURE_STAND));
  kilo_feet_t feet = {};
  CHECK(!r.link.send_stream(KILO_MSG_FEET, (const uint8_t*)&feet, sizeof feet));
  // still fed: the body's watchdog must not see a dead mind
  r.serial.take_sent();
  r.step(150);
  CHECK(count_type(r.serial.take_sent(), KILO_MSG_HEARTBEAT) == 1);
}

void test_decode_errors() {
  Rig r;
  r.link.connect();
  uint8_t wire[KILO_MAX_WIRE];
  kilo_telemetry_t t = {};
  t.tick = 7;
  uint8_t n = kilo_frame_encode(KILO_MSG_TELEMETRY, 1, (const uint8_t*)&t, sizeof t, wire);
  std::vector<uint8_t> bad(wire, wire + n);
  bad[3] ^= 0x40;  // tick, a data byte: the CRC no longer matches
  r.serial.feed(bad);
  r.step();
  CHECK(r.link.stats().decode_errors[KILO_FRAME_BAD_CRC] == 1 && r.link.stats().frames_rx == 0);
  CHECK(!r.link.state().has_telemetry);
  r.serial.feed(std::vector<uint8_t>(80, 0x11));  // > 65 B before a delimiter
  r.serial.feed({0, 0, 0});                       // repeated delimiters are ignored
  r.serial.feed(std::vector<uint8_t>(wire, wire + n));
  r.step();
  CHECK(r.link.stats().decode_errors[KILO_FRAME_OVERFLOW] == 1);
  CHECK(r.link.stats().frames_rx == 1 && r.link.state().has_telemetry && r.link.state().tick == 7);
  CHECK(r.link.stats().decode_error_total() == 2);
}

void test_epoch_change_clears_motion_goals() {
  Rig r;
  uint16_t tick = 0;
  r.up(3, &tick);
  CHECK(set_posture(r, KILO_POSTURE_STAND));
  kilo_set_eyes_t eyes = {};
  CHECK(r.link.set_goal(KILO_MSG_SET_EYES, (const uint8_t*)&eyes, sizeof eyes));
  r.step();
  std::vector<Sent> s = r.serial.take_sent();
  const Sent* p = find_sent(s, KILO_MSG_SET_POSTURE);
  CHECK(p != nullptr && p->payload[0] == 3 && p->payload[1] == KILO_POSTURE_STAND);  // stamped with its epoch
  CHECK(find_sent(s, KILO_MSG_SET_EYES) != nullptr);
  r.telemetry(tick++, 4);
  r.step();
  CHECK(r.link.goal(KILO_MSG_SET_POSTURE) == nullptr && r.link.goal(KILO_MSG_SET_EYES) != nullptr);
  CHECK(r.cleared == 1 && r.cleared_reason == "epoch 3->4" && r.link.stats().goals_cleared == 1);
  CHECK(r.log.has("cleared goals SET_POSTURE (epoch 3->4)"));
  CHECK(r.link.state().epoch == 4);
  // EVENT carries the post-bump epoch too
  CHECK(set_posture(r, KILO_POSTURE_SIT));
  r.event(KILO_EVENT_WATCHDOG, 0, 5);
  r.step();
  CHECK(r.link.goal(KILO_MSG_SET_POSTURE) == nullptr && r.cleared == 2 && r.cleared_reason == "WATCHDOG");
  CHECK(r.events == 1 && r.link.state().epoch == 5);
}

void test_goal_waits_for_first_epoch() {
  Rig r;
  r.link.connect();
  CHECK(set_posture(r, KILO_POSTURE_REST));  // not a motion goal: no contact needed
  r.step();
  CHECK(find_sent(r.serial.take_sent(), KILO_MSG_SET_POSTURE) == nullptr);  // no epoch yet
  r.hello(2, KILO_RESET_CAUSE_COLD, 0, 7);
  r.step();
  const Sent* p = nullptr;
  std::vector<Sent> s = r.serial.take_sent();
  p = find_sent(s, KILO_MSG_SET_POSTURE);
  CHECK(p != nullptr && p->payload[0] == 7);  // adopted the first epoch
  CHECK(r.cleared == 0);
}

void test_refresh_heartbeat_and_liveness() {
  Rig r;
  uint16_t tick = 0;
  r.up(1, &tick);
  CHECK(set_posture(r, KILO_POSTURE_STAND));
  r.run(1000, &tick, 1);
  std::vector<Sent> s = r.serial.take_sent();
  size_t goals = count_type(s, KILO_MSG_SET_POSTURE), beats = count_type(s, KILO_MSG_HEARTBEAT);
  CHECKF(goals >= 5 && goals <= 7, "SET_POSTURE x%zu in 1 s", goals);  // set, then refreshed at 5 Hz
  CHECKF(beats >= 4 && beats <= 6, "HEARTBEAT x%zu in 1 s", beats);    // only in the gaps > 100 ms
  // Without goals: a HEARTBEAT every 100 ms of silence.
  r.link.clear_goal(KILO_MSG_SET_POSTURE);
  r.run(1000, &tick, 1);
  beats = count_type(r.serial.take_sent(), KILO_MSG_HEARTBEAT);
  CHECKF(beats >= 9 && beats <= 11, "HEARTBEAT x%zu in 1 s", beats);
  // A stalled mind: nothing autonomous after the liveness window, so the
  // body's watchdog trips.
  CHECK(set_posture(r, KILO_POSTURE_STAND));
  r.step();
  r.serial.take_sent();
  for (int i = 0; i < 150; i++) {
    if (i % 4 == 0) r.telemetry(tick++, 1);
    r.clock.advance_ms(10);
    r.link.poll();  // no mark_alive()
    if (i == 99) r.serial.take_sent();
  }
  CHECKF(r.serial.take_sent().empty(), "frames sent by a stalled mind");
  CHECK(!r.link.mind_alive());
}

void test_reboot_by_unsolicited_hello() {
  Rig r;
  uint16_t tick = 0;
  r.up(2, &tick);
  CHECK(set_posture(r, KILO_POSTURE_STAND));
  r.run(2000, &tick, 2);  // past the solicit window; alive, so no HELLO_REQ polls
  CHECK(r.link.goal(KILO_MSG_SET_POSTURE) != nullptr);
  r.hello(2, KILO_RESET_CAUSE_WARM_UNEXPECTED, 1234, 0);
  r.step();
  CHECK(r.reboots == 1 && r.reboot_had_hello && r.link.state().reboots == 1);
  CHECKF(r.reboot_why == "unsolicited HELLO (reset cause WARM_UNEXPECTED)", "%s", r.reboot_why.c_str());
  CHECK(r.log.has("body rebooted: unsolicited HELLO (reset cause WARM_UNEXPECTED)"));
  CHECK(r.link.goal(KILO_MSG_SET_POSTURE) == nullptr && r.cleared_reason == "reboot");
  CHECK(r.link.state().epoch == 0 && !r.link.state().has_tick);
  CHECK(r.link.contact());  // a HELLO that proves a reboot is contact
}

void test_reboot_by_tick_and_debounce() {
  Rig r;
  uint16_t tick = 5000;
  r.up(1, &tick);
  r.telemetry(10, 1);  // more than half a wrap backwards
  r.step();
  CHECK(r.reboots == 1 && !r.reboot_had_hello && r.reboot_why == "tick went backwards");
  CHECK(!r.link.state().has_tick);  // forgotten: the next TELEMETRY starts it again
  // the same reboot's HELLO (new identity, solicited or not) within 3 s: not counted again
  r.hello(2, KILO_RESET_CAUSE_COLD, 4000, 1);
  r.step();
  CHECK(r.link.state().reboots == 1 && r.reboots == 1);
}

void test_contact_after_reconnect_needs_telemetry() {
  Rig r;
  uint16_t tick = 0;
  r.up(1, &tick);
  r.link.connect();  // the port reopened: the body may have been reset by DTR
  CHECK(!r.link.contact() && r.link.stats().reconnects == 1);
  CHECK(set_posture(r, KILO_POSTURE_STAND));
  kilo_set_balance_t off = {};
  CHECK(r.link.set_goal(KILO_MSG_SET_BALANCE, (const uint8_t*)&off, sizeof off));
  r.step();
  std::vector<Sent> s = r.serial.take_sent();
  CHECK(count_type(s, KILO_MSG_HELLO_REQ) == 1);
  CHECK(find_sent(s, KILO_MSG_SET_POSTURE) == nullptr);  // motion: held
  CHECK(find_sent(s, KILO_MSG_SET_BALANCE) != nullptr);  // balance off is not motion
  r.hello(2, KILO_RESET_CAUSE_COLD, 0, 1);  // the same identity: no proof of a reboot
  r.step();
  CHECK(!r.link.contact() && r.reboots == 0);
  CHECK(find_sent(r.serial.take_sent(), KILO_MSG_SET_POSTURE) == nullptr);
  r.telemetry(tick++, 1);  // its tick carries on: no reboot, contact
  r.step();
  CHECK(r.link.contact() && r.reboots == 0);
  CHECK(find_sent(r.serial.take_sent(), KILO_MSG_SET_POSTURE) != nullptr);
  kilo_feet_t feet = {};
  CHECK(r.link.send_stream(KILO_MSG_FEET, (const uint8_t*)&feet, sizeof feet));
}

void test_body_lost_and_polled() {
  Rig r;
  uint16_t tick = 0;
  r.up(1, &tick);
  r.run(100, &tick, 1);
  r.serial.take_sent();
  for (int i = 0; i < 110; i++) r.step(10);  // 1.1 s without TELEMETRY
  CHECK(r.log.has("body lost (no telemetry for 300 ms)") && !r.link.contact());
  size_t polls = count_type(r.serial.take_sent(), KILO_MSG_HELLO_REQ);
  CHECKF(polls == 2, "HELLO_REQ x%zu while lost 0.8 s", polls);  // 2 Hz
  kilo_feet_t feet = {};
  CHECK(!r.link.send_stream(KILO_MSG_FEET, (const uint8_t*)&feet, sizeof feet));
}

void test_goal_refused() {
  Rig r;
  uint16_t tick = 0;
  r.up(1, &tick);
  kilo_set_eyes_t eyes = {};
  CHECK(r.link.set_goal(KILO_MSG_SET_EYES, (const uint8_t*)&eyes, sizeof eyes));
  CHECK(set_posture(r, KILO_POSTURE_STAND));
  r.step();
  r.event(KILO_EVENT_GOAL_REFUSED, (int16_t)(KILO_MSG_SET_EYES << 8 | KILO_GOAL_REFUSED_REASON_STALE), 1);
  r.event(KILO_EVENT_GOAL_REFUSED, (int16_t)(KILO_MSG_SET_POSTURE << 8 | KILO_GOAL_REFUSED_REASON_STALE), 1);
  r.event(KILO_EVENT_GOAL_REFUSED, (int16_t)(KILO_MSG_SET_HEAD << 8 | KILO_GOAL_REFUSED_REASON_FALLEN), 1);
  r.step();
  CHECK(r.link.goal(KILO_MSG_SET_EYES) == nullptr);       // no epoch of its own: dropped
  CHECK(r.link.goal(KILO_MSG_SET_POSTURE) != nullptr);    // set under the current epoch: kept
  CHECK(r.log.has("goal SET_EYES refused as STALE (epoch now 1); dropped"));
  CHECK(r.log.has("goal SET_HEAD refused: REFUSED_FALLEN"));
  CHECK(r.refused == 3 && r.link.stats().goals_refused == 3 && r.refused_type == KILO_MSG_SET_HEAD);
}

void test_query_and_commands() {
  Rig r;
  uint16_t tick = 0;
  r.up(1, &tick);
  // a query answered by its data reply
  CHECK(r.link.request(KILO_MSG_STATS_REQ, nullptr, 0));
  CHECK(!r.link.request(KILO_MSG_PING, (const uint8_t*)"\1\2", 2));  // one at a time
  std::vector<Sent> s = r.serial.take_sent();
  const Sent* q = find_sent(s, KILO_MSG_STATS_REQ);
  CHECK(q != nullptr);
  kilo_stats_t st = {};
  st.uptime_s = 42;
  r.serial.feed_frame(KILO_MSG_STATS, 9, &st, sizeof st);
  r.step();
  const Request& req = r.link.req();
  CHECK(req.state == RequestState::Done && req.status == KILO_ACK_STATUS_OK && req.reply_len == sizeof st);
  kilo_stats_t got;
  mem_copy(&got, req.reply, sizeof got);
  CHECK(got.uptime_s == 42);

  // a command nobody answers: 1 + 3 sends with the same seq, then a timeout
  kilo_set_stream_t ss = {2, 0, 0};
  CHECK(r.link.request(KILO_MSG_SET_STREAM, (const uint8_t*)&ss, sizeof ss));
  for (int i = 0; i < 30; i++) r.step(10);
  s = r.serial.take_sent();
  CHECK(count_type(s, KILO_MSG_SET_STREAM) == 4);
  uint8_t seq0 = find_sent(s, KILO_MSG_SET_STREAM)->seq;
  for (const Sent& f : s)
    if (f.type == KILO_MSG_SET_STREAM) CHECK(f.seq == seq0);
  CHECK(r.link.req().state == RequestState::TimedOut && r.link.stats().timeouts == 1 && r.link.stats().retries == 3);

  // a query nobody answers retries with a fresh seq each time
  CHECK(r.link.request(KILO_MSG_PING, (const uint8_t*)"\1\2", 2));
  for (int i = 0; i < 30; i++) r.step(10);
  s = r.serial.take_sent();
  CHECK(count_type(s, KILO_MSG_PING) == 4);
  uint8_t last = 0;
  bool first = true, fresh = true;
  for (const Sent& f : s)
    if (f.type == KILO_MSG_PING) {
      if (!first && f.seq == last) fresh = false;
      first = false;
      last = f.seq;
    }
  CHECK(fresh);

  // an ACK refusing a command; its repeat is a dup, a stranger is unmatched
  CHECK(r.link.request(KILO_MSG_SET_STREAM, (const uint8_t*)&ss, sizeof ss));
  uint8_t seq = r.link.req().seq;
  r.ack(seq, KILO_MSG_SET_STREAM, KILO_ACK_STATUS_BAD_ARG);
  r.ack(seq, KILO_MSG_SET_STREAM, KILO_ACK_STATUS_BAD_ARG);
  r.ack((uint8_t)(seq + 100), KILO_MSG_CAL_SAVE, KILO_ACK_STATUS_OK);
  r.step();
  CHECK(r.link.req().state == RequestState::Done && r.link.req().status == KILO_ACK_STATUS_BAD_ARG);
  CHECK(r.link.stats().nacks == 1 && r.link.stats().dup_acks == 1 && r.link.stats().unmatched_acks == 1);

  // an OK ACK of a query: still waiting for the reply
  CHECK(r.link.request(KILO_MSG_PING, (const uint8_t*)"\1\2", 2));
  r.ack(r.link.req().seq, KILO_MSG_PING, KILO_ACK_STATUS_OK);
  r.step();
  CHECK(r.link.req().state == RequestState::Waiting && r.link.req().acked);
  kilo_pong_t pong = {0x0201, 5};
  r.serial.feed_frame(KILO_MSG_PONG, 3, &pong, sizeof pong);
  r.step();
  CHECK(r.link.req().state == RequestState::Done && r.link.req().status == KILO_ACK_STATUS_OK);

  // refused before sending
  CHECK(!r.link.request(KILO_MSG_SET_POSTURE, nullptr, 0));       // a goal, not a command
  CHECK(!r.link.request(KILO_MSG_SET_STREAM, (const uint8_t*)&ss, 2));  // wrong length
}

void test_trick_play_needs_contact() {
  Rig r;
  r.link.connect();
  kilo_trick_play_t tp = {};
  CHECK(!r.link.request(KILO_MSG_TRICK_PLAY, (const uint8_t*)&tp, sizeof tp));
}

void test_serial_trouble() {
  Rig r;
  uint16_t tick = 0;
  r.up(1, &tick);
  r.serial.write_room = 3;  // TX buffer nearly full
  r.step(150);
  CHECK(r.link.stats().tx_short >= 1);
  r.serial.write_room = -1;
  r.serial.failed = true;
  r.step();
  CHECK(!r.link.connected() && r.link.stats().disconnects == 1 && r.log.has("read failed"));
  CHECK(!r.link.request(KILO_MSG_STATS_REQ, nullptr, 0));
}

}  // namespace

int main() {
  test_hello_good();
  test_hello_wrong_major();
  test_decode_errors();
  test_epoch_change_clears_motion_goals();
  test_goal_waits_for_first_epoch();
  test_refresh_heartbeat_and_liveness();
  test_reboot_by_unsolicited_hello();
  test_reboot_by_tick_and_debounce();
  test_contact_after_reconnect_needs_telemetry();
  test_body_lost_and_polled();
  test_goal_refused();
  test_query_and_commands();
  test_trick_play_needs_contact();
  test_serial_trouble();
  return report("link");
}
