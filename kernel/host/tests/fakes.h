// Fakes of the platform seams for the unit tests: a Serial with scripted
// input that records what the link writes, a clock that moves only when
// told, and a log that keeps its lines.
#ifndef KILO_HOST_TESTS_FAKES_H
#define KILO_HOST_TESTS_FAKES_H
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "platform.h"
#include "proto/kilo_frame.h"
#include "proto/kilo_protocol.h"

namespace kilo {

// The vectors' hex strings ("-": none) as bytes.
inline std::vector<uint8_t> unhex(const std::string& s) {
  std::vector<uint8_t> out;
  if (s == "-") return out;
  for (size_t i = 0; i + 1 < s.size(); i += 2) out.push_back((uint8_t)std::strtoul(s.substr(i, 2).c_str(), nullptr, 16));
  return out;
}

// A body's HELLO as the core fills it: protocol KILO_PROTO_MAJOR, the
// core's limits, the epoch. A test sets the rest.
inline kilo_hello_t body_hello(uint8_t epoch = 0) {
  kilo_hello_t h = {};
  h.proto_major = KILO_PROTO_MAJOR;
  h.max_payload = KILO_MAX_PAYLOAD;
  h.joint_count = KILO_JOINT_COUNT;
  h.epoch = epoch;
  return h;
}

struct Sent {
  uint8_t type;
  uint8_t seq;
  std::vector<uint8_t> payload;
};

class FakeSerial final : public Serial {
 public:
  std::vector<uint8_t> in;   // bytes the body "sent", read from in_pos on
  size_t in_pos = 0;
  std::vector<uint8_t> out;  // bytes the link wrote
  bool failed = false;
  int32_t write_room = -1;   // >= 0: take at most this many bytes per write
  int32_t free_room = -1;    // >= 0: what tx_free() reports (else write_room, else plenty)

  int32_t read(uint8_t* buf, uint32_t n) override {
    if (failed) return -1;
    size_t k = std::min<size_t>(n, in.size() - in_pos);
    if (k) std::memcpy(buf, in.data() + in_pos, k);
    in_pos += k;
    return (int32_t)k;
  }
  int32_t write(const uint8_t* buf, uint32_t n) override {
    if (failed) return -1;
    uint32_t k = write_room >= 0 && (uint32_t)write_room < n ? (uint32_t)write_room : n;
    out.insert(out.end(), buf, buf + k);
    return (int32_t)k;
  }
  uint32_t tx_free() override {
    if (free_room >= 0) return (uint32_t)free_room;
    return write_room >= 0 ? (uint32_t)write_room : 4096u;
  }
  bool set_baud(uint32_t) override { return true; }

  void feed(const std::vector<uint8_t>& bytes) { in.insert(in.end(), bytes.begin(), bytes.end()); }
  void feed_frame(uint8_t type, uint8_t seq, const void* payload, uint8_t len) {
    uint8_t wire[KILO_MAX_WIRE];
    uint8_t n = kilo_frame_encode(type, seq, (const uint8_t*)payload, len, wire);
    in.insert(in.end(), wire, wire + n);
  }
  // The body's frames (body_hello(), a bare TELEMETRY, an EVENT).
  void feed_hello(uint8_t seq, const kilo_hello_t& h) { feed_frame(KILO_MSG_HELLO, seq, &h, sizeof h); }
  void feed_telemetry(uint8_t seq, uint16_t tick, uint8_t epoch, uint8_t mode = KILO_MODE_IDLE) {
    kilo_telemetry_t t = {};
    t.tick = tick;
    t.mode = mode;
    t.epoch = epoch;
    feed_frame(KILO_MSG_TELEMETRY, seq, &t, sizeof t);
  }
  void feed_event(uint8_t seq, uint8_t code, int16_t arg, uint8_t epoch, uint16_t tick = 0) {
    kilo_event_t e = {};
    e.code = code;
    e.arg = arg;
    e.tick = tick;
    e.epoch = epoch;
    feed_frame(KILO_MSG_EVENT, seq, &e, sizeof e);
  }
  // The frames written since the last call (all must decode).
  std::vector<Sent> take_sent() {
    std::vector<Sent> frames;
    kilo_rx_t rx;
    kilo_rx_init(&rx);
    for (uint8_t b : out) {
      kilo_frame_t f;
      uint8_t st = kilo_rx_push(&rx, b, &f);
      if (st == KILO_FRAME_OK) frames.push_back({f.type, f.seq, std::vector<uint8_t>(f.payload, f.payload + f.len)});
      else if (st != KILO_FRAME_NONE) frames.push_back({0xFF, st, {}});  // a broken frame: type 0xFF
    }
    out.clear();
    return frames;
  }
};

class FakeClock final : public Clock {
 public:
  uint64_t t = 5000000;  // not 0: nothing may treat "0" as "never"
  uint64_t now_us() override { return t; }
  void advance_ms(uint64_t ms) { t += ms * 1000u; }
};

class CaptureLog final : public Log {
 public:
  std::vector<std::string> lines;
  bool echo = false;
  void write(LogLevel level, const char* line) override {
    lines.push_back(line);
    if (echo) std::printf("  log %d: %s\n", (int)level, line);
  }
  bool has(const char* text) const {
    for (const std::string& l : lines)
      if (l.find(text) != std::string::npos) return true;
    return false;
  }
};

inline size_t count_type(const std::vector<Sent>& s, uint8_t type) {
  size_t n = 0;
  for (const Sent& f : s) n += f.type == type;
  return n;
}

}  // namespace kilo

#endif
