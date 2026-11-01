// A scripted body on a FakeSerial for the unit tests of the parts above the
// link (tricks, remote, Mind): HELLO on connect, TELEMETRY every 20 ms of
// the fake clock, an ACK for every command (status from `answer`, OK by
// default), EVENTs on demand. Time moves in 4 ms passes.
#ifndef KILO_HOST_TESTS_BODY_RIG_H
#define KILO_HOST_TESTS_BODY_RIG_H
#include <cstring>
#include <functional>
#include <vector>
#include "link/serial_link.h"
#include "tests/fakes.h"

namespace kilo {

struct FakeBody {
  FakeSerial serial;
  FakeClock clock;
  uint8_t seq = 0;
  uint16_t tick = 100;
  uint8_t epoch = 4;
  uint8_t mode = KILO_MODE_IDLE;
  uint16_t flags = KILO_TELEMETRY_FLAGS_SERVOS_POWERED;
  int16_t pitch = 0, roll = 0;
  uint16_t dist_mm = 0;
  uint16_t batt_mv = 8000;
  bool telemetry_on = true;
  uint64_t next_tel = 0;
  std::vector<Sent> sent;  // every frame the mind wrote
  // the ACK status for a command (return 0xFF: no answer)
  std::function<uint8_t(const Sent&)> answer = [](const Sent&) -> uint8_t { return KILO_ACK_STATUS_OK; };

  void hello() {
    kilo_hello_t h = {};
    h.proto_major = KILO_PROTO_MAJOR;
    h.max_payload = KILO_MAX_PAYLOAD;
    h.joint_count = KILO_JOINT_COUNT;
    h.trick_capacity = KILO_TRICK_CAPACITY;
    h.epoch = epoch;
    serial.feed_frame(KILO_MSG_HELLO, seq++, &h, sizeof h);
  }
  void telemetry() {
    kilo_telemetry_t t = {};
    t.tick = tick++;
    t.mode = mode;
    t.epoch = epoch;
    t.flags = flags;
    t.pitch = pitch;
    t.roll = roll;
    t.dist_mm = dist_mm;
    t.batt_mv = batt_mv;
    serial.feed_frame(KILO_MSG_TELEMETRY, seq++, &t, sizeof t);
  }
  void event(uint8_t code, int16_t arg) {
    kilo_event_t e = {};
    e.code = code;
    e.arg = arg;
    e.tick = tick;
    e.epoch = epoch;
    serial.feed_frame(KILO_MSG_EVENT, seq++, &e, sizeof e);
  }
  // Read what the mind wrote; ACK its commands. FEET enter GAIT; the
  // STANCE posture ends it (EVENT GAIT_STOPPED), as the body does.
  void react() {
    for (const Sent& s : serial.take_sent()) {
      sent.push_back(s);
      if (s.type == KILO_MSG_FEET && mode == KILO_MODE_IDLE) mode = KILO_MODE_GAIT;
      if (s.type == KILO_MSG_SET_POSTURE && s.payload.size() == 4 && s.payload[1] == KILO_POSTURE_STANCE &&
          mode == KILO_MODE_GAIT) {
        mode = KILO_MODE_IDLE;
        event(KILO_EVENT_GAIT_STOPPED, 0);
      }
      if (s.type == 0xFF || kilo_msg_kind(s.type) != KILO_KIND_COMMAND) continue;
      if (s.type == KILO_MSG_HELLO_REQ) {
        hello();
        continue;
      }
      uint8_t st = answer(s);
      if (st == 0xFF) continue;
      if (query_reply(s.type) != 0 && st == KILO_ACK_STATUS_OK) {
        if (s.type == KILO_MSG_STATS_REQ) {
          kilo_stats_t r = {};
          serial.feed_frame(KILO_MSG_STATS, seq++, &r, sizeof r);
        }
        continue;
      }
      kilo_ack_t a = {s.seq, s.type, st};
      serial.feed_frame(KILO_MSG_ACK, seq++, &a, sizeof a);
    }
  }
  // Called once per 4 ms pass, before the mind polls.
  void step() {
    clock.advance_ms(4);
    if (telemetry_on && clock.t >= next_tel) {
      next_tel = clock.t + 20000;
      telemetry();
    }
  }
  std::vector<Sent> of(uint8_t type) const {
    std::vector<Sent> out;
    for (const Sent& s : sent)
      if (s.type == type) out.push_back(s);
    return out;
  }
  size_t count(uint8_t type) const { return of(type).size(); }
};

}  // namespace kilo

#endif
