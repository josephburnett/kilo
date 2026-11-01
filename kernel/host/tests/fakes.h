// Fakes of the platform seams for the unit tests: a Serial with scripted
// input that records what the link writes, a clock that moves only when
// told, and a log that keeps its lines.
#ifndef KILO_HOST_TESTS_FAKES_H
#define KILO_HOST_TESTS_FAKES_H
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "platform.h"
#include "proto/kilo_frame.h"

namespace kilo {

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
  bool set_baud(uint32_t) override { return true; }

  void feed(const std::vector<uint8_t>& bytes) { in.insert(in.end(), bytes.begin(), bytes.end()); }
  void feed_frame(uint8_t type, uint8_t seq, const void* payload, uint8_t len) {
    uint8_t wire[KILO_MAX_WIRE];
    uint8_t n = kilo_frame_encode(type, seq, (const uint8_t*)payload, len, wire);
    in.insert(in.end(), wire, wire + n);
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
