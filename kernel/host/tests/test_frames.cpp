// Framing against protocol/vectors (flattened by flatten_vectors.py), the
// way protocol/c/test_protocol.cpp and protocol/py/test_codec.py use them,
// but through the kernel's vendored copies and the link: every frame
// vector round-trips (encode == wire, decode == type/seq/payload), every
// body->mind vector fed to a SerialLink is received without a decode error,
// every error vector lands in the right decode_errors counter, and the
// stream vectors give the expected sequence of frames and errors.
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "link/names.h"
#include "link/serial_link.h"
#include "proto/crc16.h"
#include "tests/check.h"
#include "tests/fakes.h"

using namespace kilo;

namespace {

std::vector<uint8_t> unhex(const std::string& s) {
  std::vector<uint8_t> out;
  if (s == "-") return out;
  for (size_t i = 0; i + 1 < s.size(); i += 2) out.push_back((uint8_t)std::strtoul(s.substr(i, 2).c_str(), nullptr, 16));
  return out;
}

// The vectors name both length errors "bad_length"; the C codec tells a
// trusted-header payload length apart.
std::string vector_kind(uint8_t status) {
  return status == KILO_FRAME_BAD_PAYLOAD_LEN ? "bad_length" : frame_status_name(status);
}

int g_frames = 0, g_errors = 0, g_streams = 0;

void check_frame(const std::string& name, uint8_t type, uint8_t seq, const std::vector<uint8_t>& payload,
                 const std::vector<uint8_t>& wire) {
  uint8_t w[KILO_MAX_WIRE];
  uint8_t n = kilo_frame_encode(type, seq, payload.data(), (uint8_t)payload.size(), w);
  CHECKF(std::vector<uint8_t>(w, w + n) == wire, "%s: encode", name.c_str());
  // decode in place, without the delimiter
  std::vector<uint8_t> buf(wire.begin(), wire.end() - 1);
  kilo_frame_t f;
  uint8_t st = kilo_frame_decode(buf.data(), (uint8_t)buf.size(), &f);
  CHECKF(st == KILO_FRAME_OK && f.type == type && f.seq == seq &&
             std::vector<uint8_t>(f.payload, f.payload + f.len) == payload,
         "%s: decode status %s", name.c_str(), frame_status_name(st));
  g_frames++;
  if (type < 0x80) return;
  // body -> mind: the link takes it whole, and parses what it handles
  FakeSerial serial;
  FakeClock clock;
  CaptureLog log;
  SerialLink link(serial, clock, log);
  link.connect();
  serial.feed(wire);
  link.poll();
  CHECKF(link.stats().frames_rx == 1 && link.stats().decode_error_total() == 0, "%s: link rx", name.c_str());
  if (type == KILO_MSG_HELLO) CHECKF(link.state().has_hello, "%s", name.c_str());
  if (type == KILO_MSG_TELEMETRY)
    CHECKF(link.state().has_telemetry && std::memcmp(&link.state().telemetry, payload.data(), payload.size()) == 0,
           "%s", name.c_str());
  if (type == KILO_MSG_EVENT) CHECKF(link.state().event_count == 1, "%s", name.c_str());
}

void check_error(const std::string& name, const std::vector<uint8_t>& wire, const std::string& kind, int type,
                 int seq) {
  // One frame decoded on its own (the delimiter stripped), as the vectors mean it.
  std::vector<uint8_t> buf(wire.begin(), wire.end() - 1);
  kilo_frame_t f;
  uint8_t st = kilo_frame_decode(buf.data(), (uint8_t)buf.size(), &f);
  CHECKF(vector_kind(st) == kind, "%s: %s != %s", name.c_str(), frame_status_name(st), kind.c_str());
  if (type >= 0)  // the header is trusted: the body ACKs such a command with BAD_LEN / UNKNOWN_TYPE
    CHECKF(f.type == type && f.seq == seq, "%s: header", name.c_str());
  // The link receives a stream, where the same bytes can read differently
  // (an empty segment is no frame, a zero inside splits one, 66 B overflow):
  // it must count what the stream receiver reports, and never accept a frame.
  uint32_t want[8] = {};
  kilo_rx_t rx;
  kilo_rx_init(&rx);
  for (uint8_t b : wire) {
    uint8_t s = kilo_rx_push(&rx, b, &f);
    if (s != KILO_FRAME_NONE) want[s]++;
  }
  FakeSerial serial;
  FakeClock clock;
  CaptureLog log;
  SerialLink link(serial, clock, log);
  link.connect();
  serial.feed(wire);
  link.poll();
  bool same = link.stats().frames_rx == 0 && want[KILO_FRAME_OK] == 0;
  for (uint8_t s = 1; s <= KILO_FRAME_OVERFLOW; s++) same = same && link.stats().decode_errors[s] == want[s];
  CHECKF(same, "%s: link counters", name.c_str());
  g_errors++;
}

void check_stream(const std::string& name, const std::vector<uint8_t>& bytes, const std::vector<std::string>& expect) {
  std::vector<std::string> got;
  kilo_rx_t rx;
  kilo_rx_init(&rx);
  for (uint8_t b : bytes) {
    kilo_frame_t f;
    uint8_t st = kilo_rx_push(&rx, b, &f);
    if (st == KILO_FRAME_OK)
      got.push_back("ok:" + std::to_string(f.type) + ":" + std::to_string(f.seq));
    else if (st != KILO_FRAME_NONE)
      got.push_back("err:" + vector_kind(st));
  }
  CHECKF(got == expect, "stream %s", name.c_str());
  // the link counts the same, fed in odd-sized chunks
  FakeSerial serial;
  FakeClock clock;
  CaptureLog log;
  SerialLink link(serial, clock, log);
  link.connect();
  size_t ok = 0, errors = 0;
  for (const std::string& e : expect) (e.rfind("ok:", 0) == 0 ? ok : errors)++;
  for (size_t i = 0; i < bytes.size(); i += 7) {
    serial.feed(std::vector<uint8_t>(bytes.begin() + i, bytes.begin() + std::min(bytes.size(), i + 7)));
    link.poll();
  }
  CHECKF(link.stats().frames_rx == ok && link.stats().decode_error_total() == errors, "stream %s: link",
         name.c_str());
  g_streams++;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: test_frames vectors.txt\n");
    return 2;
  }
  std::ifstream in(argv[1]);
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::string what, name;
    ss >> what;
    if (what == "crc") {
      std::string data;
      unsigned crc;
      ss >> data >> crc;
      std::vector<uint8_t> d = unhex(data);
      CHECK(kilo_crc16(d.data(), (uint16_t)d.size()) == crc);
    } else if (what == "frame") {
      std::string msg, payload, wire;
      int type, seq;
      ss >> name >> msg >> type >> seq >> payload >> wire;
      check_frame(name, (uint8_t)type, (uint8_t)seq, unhex(payload), unhex(wire));
    } else if (what == "error") {
      std::string wire, kind;
      int type, seq;
      ss >> name >> wire >> kind;
      if (!(ss >> type >> seq)) type = seq = -1;  // no trusted header
      check_error(name, unhex(wire), kind, type, seq);
    } else if (what == "stream") {
      std::string bytes, e;
      std::vector<std::string> expect;
      ss >> name >> bytes;
      while (ss >> e) expect.push_back(e);
      check_stream(name, unhex(bytes), expect);
    }
  }
  std::printf("  vectors: %d frames, %d errors, %d streams\n", g_frames, g_errors, g_streams);
  CHECK(g_frames > 40 && g_errors > 10 && g_streams > 2);
  return report("frames");
}
