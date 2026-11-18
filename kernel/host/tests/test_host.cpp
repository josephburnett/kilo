// The laptop interface (docs/fast-boot.md P2): host framing against
// protocol/vectors/host_frames.json (flattened by flatten_vectors.py), the
// RAM log ring, and HostServer driven through fake connections, a fake UART
// and a fake clock: replies and their seqs, the control rules (one
// controller, observers refused, REST on release, close and keepalive
// timeout), TELEM rate, the log tail, the raw bridge both ways, out-of-step
// streams and back-pressure.
#include <algorithm>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "link/serial_link.h"
#include "net/host_frame.h"
#include "net/host_server.h"
#include "tests/check.h"
#include "tests/fake_net.h"
#include "tests/fakes.h"
#include "util/logring.h"

using namespace kilo;

namespace {

std::vector<uint8_t> unhex(const std::string& s) {
  std::vector<uint8_t> out;
  if (s == "-") return out;
  for (size_t i = 0; i + 1 < s.size(); i += 2) out.push_back((uint8_t)std::strtoul(s.substr(i, 2).c_str(), nullptr, 16));
  return out;
}

struct Rig {
  FakeSerial serial;
  FakeClock clock;
  CaptureLog capture;
  LogRing ring{clock, &capture};
  SerialLink link{serial, clock, ring};
  FakeListener listener;
  FakeRandom random;
  HostServer server{listener, link, serial, clock, ring, ring, 0x1E1405F0u};

  Rig() {
    link.connect();
    uint32_t n;
    const AuthorizedKey* keys = test_keys(&n);
    server.set_keys(keys, n, random);
  }
  // A connection that becomes a client: the hello, the handshake (one
  // X25519 per pass: three passes), the client's first record.
  FakeConn& connect(int32_t slot) {
    FakeConn& c = listener.connect(slot);
    for (int k = 0; k < 10 && !c.established; k++) {
      pass();
      c.pump();
    }
    pass();
    CHECKF(c.established && c.open, "slot %d: no handshake", (int)slot);
    return c;
  }
  // One pass of the mind's loop, as kilo-mind-host does it.
  void pass() {
    link.mark_alive();
    server.poll();
    if (!server.bridge_open()) link.poll();
    server.collect();
  }
  void run_ms(uint32_t ms, uint32_t step = 5) {
    for (uint32_t t = 0; t < ms; t += step) {
      clock.advance_ms(step);
      pass();
    }
  }
};

const HostFrame* find(const std::vector<HostFrame>& fs, uint8_t type) {
  for (const HostFrame& f : fs)
    if (f.type == type) return &f;
  return nullptr;
}

// The one RESULT among fs answering seq: its code, or -1.
int result_code(const std::vector<HostFrame>& fs, uint8_t seq) {
  for (const HostFrame& f : fs)
    if (f.type == KILO_HOST_MSG_RESULT && f.seq == seq) return f.payload[1];
  return -1;
}

size_t count(const std::vector<HostFrame>& fs, uint8_t type) {
  size_t n = 0;
  for (const HostFrame& f : fs) n += f.type == type;
  return n;
}

const Goal* posture_goal(SerialLink& link) { return link.goal(KILO_MSG_SET_POSTURE); }

// ---- tests ---------------------------------------------------------------------

int g_vectors = 0;

void test_vectors(const char* path) {
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::string what, name;
    ss >> what >> name;
    if (what == "hframe") {
      std::string msg, payload, wire;
      int type, seq;
      ss >> msg >> type >> seq >> payload >> wire;
      std::vector<uint8_t> p = unhex(payload), w = unhex(wire);
      uint8_t buf[KILO_HOST_MAX_FRAME];
      uint32_t n = host_frame_encode((uint8_t)type, (uint8_t)seq, p.data(), (uint32_t)p.size(), buf);
      CHECKF(std::vector<uint8_t>(buf, buf + n) == w, "%s: encode", name.c_str());
      HostRx rx;
      HostRxStatus st = HostRxStatus::None;
      for (uint8_t b : w) st = rx.push(b);
      CHECKF(st == HostRxStatus::Frame && rx.frame().type == type && rx.frame().seq == seq &&
                 std::vector<uint8_t>(rx.frame().payload, rx.frame().payload + rx.frame().len) == p,
             "%s: decode", name.c_str());
      CHECKF(kilo_host_payload_len_check((uint8_t)type, (uint16_t)p.size()) == KILO_LEN_OK, "%s: length rule",
             name.c_str());
      g_vectors++;
    } else if (what == "herror") {
      std::string wire, kind;
      ss >> wire >> kind;
      HostRx rx;
      HostRxStatus st = HostRxStatus::None;
      for (uint8_t b : unhex(wire)) st = rx.push(b);
      CHECKF(st == (kind == "bad_crc" ? HostRxStatus::BadCrc : HostRxStatus::BadLength) && rx.failed(), "%s",
             name.c_str());
      g_vectors++;
    } else if (what == "hcheck") {
      std::string wire, status;
      ss >> wire >> status;
      HostRx rx;
      HostRxStatus st = HostRxStatus::None;
      for (uint8_t b : unhex(wire)) st = rx.push(b);
      CHECK(st == HostRxStatus::Frame);
      uint8_t want = status == "bad_len" ? KILO_LEN_BAD : KILO_LEN_UNKNOWN_TYPE;
      CHECKF(kilo_host_payload_len_check(rx.frame().type, rx.frame().len) == want, "%s", name.c_str());
      g_vectors++;
    }
  }
  CHECKF(g_vectors >= 20, "%d host vectors", g_vectors);
}

void test_rx_split_and_back_to_back() {
  uint8_t a[KILO_HOST_MAX_FRAME], b[KILO_HOST_MAX_FRAME];
  uint8_t pa[KILO_HOST_MAX_PAYLOAD];
  for (uint32_t i = 0; i < sizeof pa; i++) pa[i] = (uint8_t)i;
  uint32_t na = host_frame_encode(0x83, 7, pa, KILO_HOST_MAX_PAYLOAD, a);
  uint32_t nb = host_frame_encode(0x01, 8, nullptr, 0, b);
  CHECK(na == KILO_HOST_MAX_FRAME && nb == 6);
  CHECK(host_frame_encode(0x83, 0, pa, KILO_HOST_MAX_PAYLOAD + 1, a + 300 - 300) == 0);
  na = host_frame_encode(0x83, 7, pa, KILO_HOST_MAX_PAYLOAD, a);
  HostRx rx;
  int frames = 0;
  for (uint32_t i = 0; i < na; i++) frames += rx.push(a[i]) == HostRxStatus::Frame;
  CHECK(frames == 1 && rx.frame().len == KILO_HOST_MAX_PAYLOAD && rx.frame().payload[239] == 239);
  for (uint32_t i = 0; i < nb; i++) frames += rx.push(b[i]) == HostRxStatus::Frame;
  CHECK(frames == 2 && rx.frame().type == 0x01 && rx.frame().seq == 8 && rx.frame().len == 0);
  b[3] ^= 1;  // seq: the CRC no longer matches
  HostRxStatus st = HostRxStatus::None;
  for (uint32_t i = 0; i < nb; i++) st = rx.push(b[i]);
  CHECK(st == HostRxStatus::BadCrc && rx.failed());
  CHECK(rx.push(0) == HostRxStatus::BadLength);  // stays failed
  rx.reset();
  CHECK(!rx.failed());
}

void test_logring() {
  FakeClock clock;
  CaptureLog next;
  LogRing ring(clock, &next);
  CHECK(ring.next_seq() == 0 && ring.first_seq() == 0 && ring.get(0) == nullptr);
  logf(ring, LogLevel::Info, "line %d", 0);
  clock.advance_ms(1500);
  logf(ring, LogLevel::Warn, "line %d", 1);
  CHECK(next.lines.size() == 2 && next.lines[1] == "line 1");
  const LogEntry* e = ring.get(1);
  CHECK(e != nullptr && e->seq == 1 && e->t_ms == 1500 && e->level == LogLevel::Warn && e->len == 6 &&
        std::string(e->text) == "line 1");
  const uint32_t cap = ring.capacity();  // kSmallLines until the platform attaches its storage
  CHECK(cap == LogRing::kSmallLines);
  for (int i = 2; i < (int)cap + 10; i++) logf(ring, LogLevel::Debug, "line %d", i);
  CHECK(ring.next_seq() == cap + 10 && ring.first_seq() == 10);
  CHECK(ring.get(9) == nullptr && ring.get(10) != nullptr && std::string(ring.get(10)->text) == "line 10");
  CHECK(ring.get(cap + 10) == nullptr);
  std::string longline(300, 'x');
  ring.write(LogLevel::Info, longline.c_str());
  CHECK(ring.get(ring.next_seq() - 1)->len == kLogLineMax - 1);
  // Status lines: always on to the platform's sink; kept only while a
  // laptop reads the log, or when they are warnings.
  uint32_t kept = ring.next_seq();
  size_t passed = next.lines.size();
  logf(ring.status(), LogLevel::Info, "status %d", 1);
  CHECK(ring.next_seq() == kept && next.lines.size() == passed + 1 && next.lines.back() == "status 1");
  logf(ring.status(), LogLevel::Warn, "status %d", 2);
  CHECK(ring.next_seq() == kept + 1 && next.lines.size() == passed + 2);
  ring.set_watched(true);  // a laptop reads the log: status lines go to it, live, but are not history
  logf(ring.status(), LogLevel::Info, "status %d", 3);
  CHECK(ring.next_seq() == kept + 1 && ring.status_next() == 1 && std::string(ring.status_get(0)->text) == "status 3" &&
        ring.status_get(0)->seq == (LogRing::kStatusBit | 0u) && next.lines.size() == passed + 3);
  for (int i = 0; i < (int)LogRing::kStatusLines + 5; i++) logf(ring.status(), LogLevel::Info, "status x%d", i);
  CHECK(ring.status_get(0) == nullptr && ring.status_first() == 6 && ring.next_seq() == kept + 1);
  // the tap (the flight recorder) gets every line, history and status
  CaptureLog tap;
  ring.set_tap(&tap);
  logf(ring, LogLevel::Info, "history");
  ring.set_watched(false);
  logf(ring.status(), LogLevel::Info, "status unwatched");
  CHECK(tap.lines.size() == 2 && tap.lines[1] == "status unwatched");
  // the platform's history storage: the lines so far are copied in
  std::vector<LogEntry> big(LogRing::kLines);
  uint32_t first = ring.first_seq(), nxt = ring.next_seq();
  std::string last = ring.get(nxt - 1)->text;
  ring.attach(big.data(), (uint32_t)big.size());
  CHECK(ring.capacity() == LogRing::kLines && ring.first_seq() == first && ring.next_seq() == nxt &&
        std::string(ring.get(nxt - 1)->text) == last);
  for (int i = 0; i < 1000; i++) logf(ring, LogLevel::Info, "more %d", i);
  CHECK(ring.first_seq() == first && ring.get(first) != nullptr);  // 4096 lines: nothing dropped yet
}

void test_status_and_errors() {
  Rig r;
  FakeConn& c = r.connect(0);
  r.pass();
  CHECK(r.server.clients() == 1);
  c.send(KILO_HOST_MSG_STATUS_REQ, 42);
  r.pass();
  std::vector<HostFrame> got = c.take();
  const HostFrame* s = find(got, KILO_HOST_MSG_STATUS);
  CHECK(s != nullptr && s->seq == 42 && s->len == sizeof(kilo_host_status_t));
  if (s != nullptr) {
    kilo_host_status_t st;
    std::memcpy(&st, s->payload, sizeof st);
    CHECK(st.host_version == KILO_HOST_VERSION && st.mind_build == 0x1E1405F0u && st.clients == 1);
    CHECK(st.controller == KILO_HOST_CONTROLLER_NONE && st.proto_ok == -1 && st.telemetry_age_ms == 0xFFFF);
    CHECK((st.state & KILO_HOST_STATE_LINK_CONNECTED) && !(st.state & KILO_HOST_STATE_HAS_HELLO));
    CHECK(st.link.frames_tx == r.link.stats().frames_tx);
  }
  // A body HELLO and TELEMETRY show up in the next STATUS.
  kilo_hello_t h = {};
  h.proto_major = KILO_PROTO_MAJOR;
  h.fw_build = 0x45524F43u;
  r.serial.feed_frame(KILO_MSG_HELLO, 0, &h, sizeof h);
  kilo_telemetry_t t = {};
  t.tick = 100;
  t.batt_mv = 7400;
  t.mode = KILO_MODE_IDLE;
  r.serial.feed_frame(KILO_MSG_TELEMETRY, 1, &t, sizeof t);
  r.run_ms(10);
  c.send(KILO_HOST_MSG_STATUS_REQ, 43);
  r.pass();
  got = c.take();
  s = find(got, KILO_HOST_MSG_STATUS);
  CHECK(s != nullptr && s->seq == 43);
  if (s != nullptr) {
    kilo_host_status_t st;
    std::memcpy(&st, s->payload, sizeof st);
    CHECK(st.proto_ok == 1 && st.hello.fw_build == 0x45524F43u && st.telemetry.batt_mv == 7400);
    CHECK((st.state & KILO_HOST_STATE_HAS_HELLO) && (st.state & KILO_HOST_STATE_BODY_ALIVE) &&
          st.telemetry_age_ms <= 10 && st.telemetry_count == 1);
  }
  // Unknown type, a mind->client type, a wrong length, the stubs: answered, the connection stays.
  c.send(0x7D, 1);
  c.send(KILO_HOST_MSG_STATUS, 2);
  uint8_t one = 0;
  c.send(KILO_HOST_MSG_STATUS_REQ, 3, &one, 1);
  uint8_t pid = 5;
  c.send(KILO_HOST_MSG_PARAM_GET, 4, &pid, 1);
  r.pass();
  got = c.take();
  CHECK(result_code(got, 1) == KILO_HOST_RESULT_CODE_UNKNOWN_TYPE);
  CHECK(result_code(got, 2) == KILO_HOST_RESULT_CODE_UNKNOWN_TYPE);
  CHECK(result_code(got, 3) == KILO_HOST_RESULT_CODE_BAD_LEN);
  CHECK(result_code(got, 4) == KILO_HOST_RESULT_CODE_UNSUPPORTED);
  CHECK(c.open && r.server.clients() == 1);
  // A bad CRC (in a record that authenticates): out of step, closed.
  uint8_t f[KILO_HOST_MAX_FRAME];
  uint32_t fn = host_frame_encode(KILO_HOST_MSG_STATUS_REQ, 5, nullptr, 0, f);
  f[3] ^= 0x40;
  c.send_record(f, fn);
  r.pass();
  CHECK(!c.open && r.server.clients() == 0 && r.server.stats().out_of_step == 1);
  // A slot past what the server serves is closed at accept.
  FakeConn& extra = r.listener.connect(FakeListener::kConns - 1);
  r.pass();
  CHECK(!extra.open && r.server.stats().refused == 1);
}

void test_control_rules() {
  Rig r;
  kilo_telemetry_t t = {};
  t.epoch = 2;  // epoch goals wait for the body's first epoch
  r.serial.feed_frame(KILO_MSG_TELEMETRY, 0, &t, sizeof t);
  FakeConn& a = r.connect(0);
  FakeConn& b = r.connect(1);
  r.pass();
  uint8_t take[2] = {1, 0};
  kilo_host_posture_t stand = {KILO_POSTURE_STAND, 0, 0};
  b.send(KILO_HOST_MSG_POSTURE, 1, &stand, sizeof stand);  // nobody has control yet
  a.send(KILO_HOST_MSG_CONTROL, 1, take, 2);
  r.pass();
  b.send(KILO_HOST_MSG_CONTROL, 2, take, 2);
  r.pass();
  std::vector<HostFrame> g1 = b.take();
  CHECK(result_code(g1, 1) == KILO_HOST_RESULT_CODE_NOT_CONTROLLER);
  CHECK(result_code(g1, 2) == KILO_HOST_RESULT_CODE_BUSY);
  CHECK(result_code(a.take(), 1) == KILO_HOST_RESULT_CODE_OK && r.server.controller() == 0);
  b.send(KILO_HOST_MSG_POSTURE, 3, &stand, sizeof stand);
  int16_t pset[2] = {0, 0};
  b.send(KILO_HOST_MSG_PARAM_SET, 4, pset, 4);
  a.send(KILO_HOST_MSG_POSTURE, 2, &stand, sizeof stand);
  uint8_t pset_a[4] = {5, 0, 0, 0};
  a.send(KILO_HOST_MSG_PARAM_SET, 3, pset_a, 4);
  kilo_host_posture_t bad = {99, 0, 0};
  a.send(KILO_HOST_MSG_POSTURE, 4, &bad, sizeof bad);
  r.pass();
  std::vector<HostFrame> gb = b.take(), ga = a.take();
  CHECK(result_code(gb, 3) == KILO_HOST_RESULT_CODE_NOT_CONTROLLER);
  CHECK(result_code(gb, 4) == KILO_HOST_RESULT_CODE_NOT_CONTROLLER);
  CHECK(result_code(ga, 2) == KILO_HOST_RESULT_CODE_OK);
  CHECK(result_code(ga, 3) == KILO_HOST_RESULT_CODE_UNSUPPORTED);
  CHECK(result_code(ga, 4) == KILO_HOST_RESULT_CODE_BAD_ARG);
  const Goal* g = posture_goal(r.link);
  CHECK(g != nullptr && g->payload[1] == KILO_POSTURE_STAND);
  // An observer's STATUS says OTHER, the controller's YOU.
  b.send(KILO_HOST_MSG_STATUS_REQ, 5);
  a.send(KILO_HOST_MSG_STATUS_REQ, 5);
  r.pass();
  std::vector<HostFrame> rb = b.take(), ra = a.take();
  const HostFrame* sb = find(rb, KILO_HOST_MSG_STATUS);
  const HostFrame* sa = find(ra, KILO_HOST_MSG_STATUS);
  CHECK(sb != nullptr && sb->payload[offsetof(kilo_host_status_t, controller)] == KILO_HOST_CONTROLLER_OTHER);
  CHECK(sa != nullptr && sa->payload[offsetof(kilo_host_status_t, controller)] == KILO_HOST_CONTROLLER_YOU);
  // Release: REST, and control is free for b.
  uint8_t give[2] = {0, 0};
  b.send(KILO_HOST_MSG_CONTROL, 6, give, 2);  // b has nothing to give up
  a.send(KILO_HOST_MSG_CONTROL, 6, give, 2);
  r.pass();
  CHECK(result_code(b.take(), 6) == KILO_HOST_RESULT_CODE_NOT_CONTROLLER);
  CHECK(result_code(a.take(), 6) == KILO_HOST_RESULT_CODE_OK && r.server.controller() == -1);
  g = posture_goal(r.link);
  CHECK(g != nullptr && g->payload[1] == KILO_POSTURE_REST);
  // b takes control, stands, then its socket closes: REST.
  b.send(KILO_HOST_MSG_CONTROL, 7, take, 2);
  b.send(KILO_HOST_MSG_POSTURE, 8, &stand, sizeof stand);
  r.pass();
  CHECK(r.server.controller() == 1 && posture_goal(r.link)->payload[1] == KILO_POSTURE_STAND);
  b.peer_closed = true;
  r.pass();
  CHECK(r.server.controller() == -1 && !b.open && posture_goal(r.link)->payload[1] == KILO_POSTURE_REST);
  CHECK(r.server.stats().control_lost == 2);
  // The REST goes out on the link (REST is not a motion goal: no contact needed).
  r.serial.take_sent();
  r.run_ms(250);
  bool rest_sent = false;
  for (const Sent& s : r.serial.take_sent())
    if (s.type == KILO_MSG_SET_POSTURE && s.payload.size() == 4 && s.payload[1] == KILO_POSTURE_REST) rest_sent = true;
  CHECK(rest_sent);
}

void test_keepalive() {
  Rig r;
  FakeConn& a = r.connect(0);
  r.pass();
  uint8_t take_ka[2] = {1, 1};
  kilo_host_posture_t stand = {KILO_POSTURE_STAND, 0, 0};
  a.send(KILO_HOST_MSG_CONTROL, 1, take_ka, 2);
  a.send(KILO_HOST_MSG_POSTURE, 2, &stand, sizeof stand);
  r.pass();
  // KEEPALIVE every 500 ms holds control for 5 s.
  for (int k = 0; k < 10; k++) {
    r.run_ms(500, 10);
    a.send(KILO_HOST_MSG_KEEPALIVE, 0);
  }
  r.pass();
  CHECK(r.server.controller() == 0 && posture_goal(r.link)->payload[1] == KILO_POSTURE_STAND);
  CHECK(find(a.take(), KILO_HOST_MSG_RESULT) != nullptr);  // the two replies; KEEPALIVE has none
  // Silent for 2 s: control lost, REST; the connection stays open.
  r.run_ms(1900, 10);
  CHECK(r.server.controller() == 0);
  r.run_ms(200, 10);
  CHECK(r.server.controller() == -1 && posture_goal(r.link)->payload[1] == KILO_POSTURE_REST && a.open);
  // Without keepalive, silence keeps control.
  uint8_t take[2] = {1, 0};
  a.send(KILO_HOST_MSG_CONTROL, 3, take, 2);
  r.pass();
  r.run_ms(5000, 50);
  CHECK(r.server.controller() == 0);
}

void test_telem_rate() {
  Rig r;
  FakeConn& a = r.connect(0);
  r.pass();
  uint8_t hz = 10;
  a.send(KILO_HOST_MSG_TELEM_SUB, 1, &hz, 1);
  r.pass();
  a.take();
  r.run_ms(1000, 5);
  std::vector<HostFrame> got = a.take();
  size_t n = count(got, KILO_HOST_MSG_TELEM);
  CHECKF(n >= 9 && n <= 11, "%zu TELEM in 1 s at 10 Hz", n);
  if (!got.empty()) CHECK(got.back().len == sizeof(kilo_host_telem_t));
  uint8_t bad = 51, off = 0;
  a.send(KILO_HOST_MSG_TELEM_SUB, 2, &bad, 1);
  a.send(KILO_HOST_MSG_TELEM_SUB, 3, &off, 1);
  r.pass();
  got = a.take();
  CHECK(result_code(got, 2) == KILO_HOST_RESULT_CODE_BAD_ARG && result_code(got, 3) == KILO_HOST_RESULT_CODE_OK);
  r.run_ms(500);
  CHECK(count(a.take(), KILO_HOST_MSG_TELEM) == 0);
  // A client that stops reading: TELEM is skipped, not queued without bound.
  a.write_room = 0;
  a.send(KILO_HOST_MSG_TELEM_SUB, 4, &hz, 1);
  r.run_ms(5000, 10);
  CHECK(r.server.stats().telem_skipped > 0);
}

void test_log_tail() {
  Rig r;
  for (int i = 0; i < 5; i++) logf(r.ring, LogLevel::Info, "early %d", i);
  logf(r.ring, LogLevel::Debug, "debug line");
  FakeConn& a = r.connect(0);
  // "client 0 connected", the handshake and "is a client" are logged too
  uint8_t sub[3] = {1, 6, KILO_LOG_LEVEL_INFO};
  a.send(KILO_HOST_MSG_LOG_SUB, 1, sub, 3);
  r.pass();
  std::vector<std::string> lines;
  std::vector<uint32_t> seqs;
  for (const HostFrame& f : a.take()) {
    if (f.type != KILO_HOST_MSG_LOG_LINE) continue;
    lines.push_back(std::string((const char*)f.payload + KILO_HOST_LOG_LINE_MIN_LEN, f.len - KILO_HOST_LOG_LINE_MIN_LEN));
    uint32_t s;
    std::memcpy(&s, f.payload, 4);
    seqs.push_back(s);
  }
  // the last 6 lines: early 3, early 4, (debug: filtered), client connected, the handshake, a client
  CHECKF(lines.size() == 5, "%zu lines", lines.size());
  if (lines.size() == 5) {
    CHECK(lines[0] == "early 3" && lines[1] == "early 4" && lines[2].find("client 0 connected") != std::string::npos);
    CHECK(lines[3].find("noise: client 0: handshake with key ") != std::string::npos &&
          lines[3].find("(test@laptop)") != std::string::npos && lines[4].find("client 0 is a client") != std::string::npos);
    CHECK(seqs[1] == seqs[0] + 1 && seqs[2] == seqs[1] + 2);  // the debug line between is filtered
  }
  logf(r.ring, LogLevel::Warn, "live line");
  r.pass();
  std::vector<HostFrame> live = a.take();
  const HostFrame* f = find(live, KILO_HOST_MSG_LOG_LINE);
  CHECK(f != nullptr && f->payload[8] == KILO_LOG_LEVEL_WARN &&
        std::string((const char*)f->payload + 9, f->len - 9) == "live line");
  // A status line goes to a watching client live, with bit 31 set.
  logf(r.ring.status(), LogLevel::Info, "link: telemetry 50/s");
  r.pass();
  live = a.take();
  f = find(live, KILO_HOST_MSG_LOG_LINE);
  uint32_t sseq = 0;
  if (f != nullptr) std::memcpy(&sseq, f->payload, 4);
  CHECK(f != nullptr && (sseq & LogRing::kStatusBit) && std::string((const char*)f->payload + 9, f->len - 9) == "link: telemetry 50/s");
  // tail 255: all the history kept, the status line not in it
  FakeConn& b = r.connect(1);
  uint8_t all[3] = {1, 255, KILO_LOG_LEVEL_DEBUG};
  b.send(KILO_HOST_MSG_LOG_SUB, 1, all, 3);
  r.run_ms(50, 1);
  size_t n_all = 0;
  bool first_zero = false;
  for (const HostFrame& g : b.take())
    if (g.type == KILO_HOST_MSG_LOG_LINE) {
      uint32_t s;
      std::memcpy(&s, g.payload, 4);
      if (n_all++ == 0) first_zero = s == 0;
      CHECK(!(s & LogRing::kStatusBit));
    }
  CHECKF(first_zero && n_all >= r.ring.next_seq() - 1, "%zu lines of %u", n_all, (unsigned)r.ring.next_seq());
  // Off: nothing more.
  uint8_t stop[3] = {0, 0, 0};
  a.send(KILO_HOST_MSG_LOG_SUB, 2, stop, 3);
  r.pass();
  a.take();
  logf(r.ring, LogLevel::Error, "after");
  r.pass();
  CHECK(count(a.take(), KILO_HOST_MSG_LOG_LINE) == 0);
}

void test_bridge() {
  Rig r;
  FakeConn& a = r.connect(0);
  FakeConn& b = r.connect(1);
  r.pass();
  b.send(KILO_HOST_MSG_BRIDGE_OPEN, 1);  // an observer may not
  uint8_t take[2] = {1, 0};
  a.send(KILO_HOST_MSG_CONTROL, 1, take, 2);
  a.send(KILO_HOST_MSG_BRIDGE_CLOSE, 2);  // nothing open
  a.send(KILO_HOST_MSG_BRIDGE_OPEN, 3);
  r.pass();
  CHECK(result_code(b.take(), 1) == KILO_HOST_RESULT_CODE_NOT_CONTROLLER);
  std::vector<HostFrame> got = a.take();
  CHECK(result_code(got, 2) == KILO_HOST_RESULT_CODE_NOT_BRIDGED && result_code(got, 3) == KILO_HOST_RESULT_CODE_OK);
  CHECK(r.server.bridge_open());
  // Paused: nothing from the link for a second (no heartbeat), only what the laptop sends.
  r.serial.take_sent();
  r.serial.out.clear();
  r.run_ms(1000);
  CHECK(r.serial.out.empty());
  uint8_t req[6] = {0x02, 0x01, 0x03, 0x3e, 0x2e, 0x00};  // HELLO_REQ seq 0, on the wire
  a.send(KILO_HOST_MSG_BRIDGE_TX, 0, req, sizeof req);
  b.send(KILO_HOST_MSG_BRIDGE_TX, 0, req, sizeof req);  // not the controller: dropped
  kilo_host_posture_t stand = {KILO_POSTURE_STAND, 0, 0};
  a.send(KILO_HOST_MSG_POSTURE, 4, &stand, sizeof stand);
  r.pass();
  CHECK(r.serial.out == std::vector<uint8_t>(req, req + sizeof req));
  CHECK(result_code(a.take(), 4) == KILO_HOST_RESULT_CODE_BRIDGED);
  // The body's bytes go to the controller only, in chunks of at most 200.
  std::vector<uint8_t> body(450);
  for (size_t i = 0; i < body.size(); i++) body[i] = (uint8_t)(i * 7);
  r.serial.feed(body);
  r.pass();
  std::vector<uint8_t> rx;
  for (const HostFrame& f : a.take()) {
    CHECK(f.type == KILO_HOST_MSG_BRIDGE_RX && f.len <= KILO_HOST_BRIDGE_CHUNK_MAX);
    rx.insert(rx.end(), f.payload, f.payload + f.len);
  }
  CHECK(rx == body && b.take().empty());
  CHECK(r.link.stats().frames_rx == 0);  // the link did not read them
  // Closing resumes the link: it reconnects and asks HELLO_REQ.
  uint32_t reconnects = r.link.stats().reconnects;
  a.send(KILO_HOST_MSG_BRIDGE_CLOSE, 5);
  r.serial.out.clear();
  r.run_ms(50);
  CHECK(result_code(a.take(), 5) == KILO_HOST_RESULT_CODE_OK && !r.server.bridge_open());
  CHECK(r.link.stats().reconnects == reconnects + 1 && count_type(r.serial.take_sent(), KILO_MSG_HELLO_REQ) >= 1);
  // Losing control closes the bridge too.
  a.send(KILO_HOST_MSG_BRIDGE_OPEN, 6);
  r.pass();
  CHECK(r.server.bridge_open());
  a.peer_closed = true;
  r.pass();
  CHECK(!r.server.bridge_open() && r.server.controller() == -1 && posture_goal(r.link)->payload[1] == KILO_POSTURE_REST);
}

void test_reboot() {
  Rig r;
  kilo_telemetry_t t = {};
  t.epoch = 2;  // epoch goals wait for the body's first epoch
  r.serial.feed_frame(KILO_MSG_TELEMETRY, 0, &t, sizeof t);
  FakeConn& a = r.connect(0);
  FakeConn& b = r.connect(1);
  r.pass();
  b.send(KILO_HOST_MSG_REBOOT, 1);  // an observer may not
  r.pass();
  CHECK(result_code(b.take(), 1) == KILO_HOST_RESULT_CODE_NOT_CONTROLLER && !r.server.reboot_requested());
  // The controller: STAND latched, then REBOOT -> REST latched, OK written, the flag set.
  uint8_t take[2] = {1, 0};
  a.send(KILO_HOST_MSG_CONTROL, 1, take, 2);
  r.pass();
  kilo_host_posture_t stand = {KILO_POSTURE_STAND, 0, 0};
  a.send(KILO_HOST_MSG_POSTURE, 2, &stand, sizeof stand);
  r.pass();
  CHECK(posture_goal(r.link) != nullptr && posture_goal(r.link)->payload[1] == KILO_POSTURE_STAND);
  a.take();
  a.send(KILO_HOST_MSG_REBOOT, 3);
  r.link.mark_alive();
  r.server.poll();
  CHECK(r.server.reboot_requested());
  CHECK(result_code(a.take(), 3) == KILO_HOST_RESULT_CODE_OK);  // written to the socket by that poll
  CHECK(posture_goal(r.link)->payload[1] == KILO_POSTURE_REST);
  CHECK(r.capture.has("asked for a reboot (REST latched)"));
}

void test_slow_client_dropped() {
  Rig r;
  FakeConn& a = r.connect(0);
  r.pass();
  a.write_room = 0;  // never drains
  for (int i = 0; i < 40; i++) a.send(KILO_HOST_MSG_STATUS_REQ, (uint8_t)i);  // 40 x 170 B > the 4 KB ring
  r.pass();
  CHECK(!a.open && r.server.clients() == 0);
}

// ---- the flight recorder over the channel (WALK_LIST, WALK_FETCH, REC_*) ----------------

kilo_host_walk_info_t walk_list(Rig& r, FakeConn& c, uint32_t after, uint8_t seq) {
  c.send(KILO_HOST_MSG_WALK_LIST, seq, &after, sizeof after);
  r.pass();
  kilo_host_walk_info_t w;
  std::memset(&w, 0, sizeof w);
  for (const HostFrame& f : c.take())
    if (f.type == KILO_HOST_MSG_WALK_INFO && f.seq == seq) std::memcpy(&w, f.payload, f.len);
  return w;
}

// The WALK_DATA bytes of a fetch (all of them, END or GONE at the end).
std::vector<uint8_t> fetch_all(Rig& r, FakeConn& c, uint8_t type, const void* q, uint32_t n, uint8_t seq, int* status,
                               size_t* max_per_pass = nullptr, int code_want = KILO_HOST_RESULT_CODE_OK) {
  c.send(type, seq, q, n);
  std::vector<uint8_t> got;
  *status = -1;
  for (int k = 0; k < 5000 && *status < 0; k++) {
    r.pass();
    std::vector<HostFrame> fs = c.take();
    int rc = k == 0 ? result_code(fs, seq) : KILO_HOST_RESULT_CODE_OK;
    if (rc != KILO_HOST_RESULT_CODE_OK || code_want != KILO_HOST_RESULT_CODE_OK) {
      *status = rc == code_want ? -2 : -3;  // -2: refused as expected
      return got;
    }
    if (max_per_pass != nullptr) *max_per_pass = std::max(*max_per_pass, count(fs, KILO_HOST_MSG_WALK_DATA));
    for (const HostFrame& f : fs) {
      if (f.type != KILO_HOST_MSG_WALK_DATA) continue;
      kilo_host_walk_data_t d;
      std::memcpy(&d, f.payload, f.len);
      CHECK(d.offset == got.size());
      if (d.status != KILO_WALK_DATA_STATUS_DATA) *status = d.status;
      else got.insert(got.end(), d.data, d.data + (f.len - KILO_HOST_WALK_DATA_MIN_LEN));
    }
  }
  return got;
}

// Records of a block stream: (kind, absolute t_us, payload); false if a block is malformed.
struct BRec {
  uint8_t kind;
  uint64_t t_us;
  std::string payload;
};
bool parse_blocks(const std::vector<uint8_t>& b, std::vector<BRec>* out, uint32_t* blocks = nullptr) {
  size_t at = 0;
  uint32_t n = 0;
  while (at + KILO_RECORD_BLOCK_HEADER_SIZE <= b.size()) {
    kilo_record_block_header_t h;
    std::memcpy(&h, b.data() + at, sizeof h);
    if (h.magic != KILO_HOST_RECORD_BLOCK_MAGIC || h.used < sizeof h || at + h.used > b.size()) return false;
    uint64_t t0 = ((uint64_t)h.t0_us_hi << 32) | h.t0_us_lo;
    size_t p = at + sizeof h;
    while (p + KILO_WALK_RECORD_HEADER_SIZE <= at + h.used) {
      kilo_walk_record_header_t rh;
      std::memcpy(&rh, b.data() + p, sizeof rh);
      p += sizeof rh;
      if (p + rh.len > at + h.used) return false;
      out->push_back({rh.kind, t0 + rh.t_us, std::string((const char*)b.data() + p, rh.len)});
      p += rh.len;
    }
    at += h.used;
    n++;
  }
  if (blocks != nullptr) *blocks = n;
  return at == b.size();
}

void test_runs() {
  Rig r;
  FakeConn& a = r.connect(0);
  r.pass();
  a.take();
  uint32_t zero = 0;
  a.send(KILO_HOST_MSG_WALK_LIST, 1, &zero, 4);
  a.send(KILO_HOST_MSG_REC_STATUS_REQ, 2);
  r.pass();
  std::vector<HostFrame> fs = a.take();
  CHECK(result_code(fs, 1) == KILO_HOST_RESULT_CODE_UNSUPPORTED && result_code(fs, 2) == KILO_HOST_RESULT_CODE_UNSUPPORTED);

  // a recorder of 8 blocks, link frames every 20 ms, a refused trick, more frames
  std::vector<uint8_t> mem(8 * record::kBlockBytes);
  record::Recorder rec(r.clock, r.link);
  rec.attach(mem.data(), (uint32_t)mem.size(), 0x12345678u);
  tricks::TrickPlayer idle(r.link, r.clock, r.ring);
  r.server.attach_recorder(rec);
  uint8_t pl[30] = {};
  auto frames = [&](uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 20) {
      r.clock.advance_ms(20);
      pl[0]++;
      rec.on_frame(false, KILO_MSG_TELEMETRY, pl[0], pl, sizeof pl, r.clock.now_us());
      rec.on_frame(true, KILO_MSG_HEARTBEAT, pl[0], nullptr, 0, r.clock.now_us());
      rec.poll(idle);
      r.pass();
    }
  };
  frames(1000);
  rec.trick_refused("rear_up", "battery 7000 mV (needs 7400)");
  kilo_host_walk_info_t w = walk_list(r, a, 0, 3);
  CHECK(w.found == 1 && w.id == 1 && (w.flags & KILO_WALK_FLAGS_TRICK) && (w.flags & KILO_WALK_FLAGS_RECORDING) &&
        w.recording == 1);
  kilo_host_walk_fetch_t q = {1, 0};
  int status;
  fetch_all(r, a, KILO_HOST_MSG_WALK_FETCH, &q, sizeof q, 4, &status, nullptr, KILO_HOST_RESULT_CODE_BAD_ARG);
  CHECK(status == -2);  // still recording (the post-roll)
  frames(2100);
  w = walk_list(r, a, 0, 5);
  std::string text(w.text, w.text + std::strlen((const char*)w.text));
  CHECKF(w.found == 1 && !(w.flags & KILO_WALK_FLAGS_RECORDING) && text == "trick=rear_up stage=1 end=REFUSED" &&
             w.bytes > 0 && w.capacity == rec.flight().capacity(),
         "flags %x text %s bytes %u", w.flags, text.c_str(), (unsigned)w.bytes);
  CHECK(walk_list(r, a, 1, 6).found == 0);

  // the run's span: blocks from 2 s before to 2 s after
  size_t per = 0;
  std::vector<uint8_t> got = fetch_all(r, a, KILO_HOST_MSG_WALK_FETCH, &q, sizeof q, 7, &status, &per);
  std::vector<BRec> rs;
  uint32_t nb = 0;
  CHECKF(status == KILO_WALK_DATA_STATUS_END && got.size() == w.bytes && parse_blocks(got, &rs, &nb), "status %d, %zu B of %u",
         status, got.size(), (unsigned)w.bytes);
  size_t tel = 0, runs = 0, refused = 0, logs = 0;
  uint64_t first = UINT64_MAX, last = 0;
  for (const BRec& x : rs) {
    if (x.kind == KILO_WALK_RECORD_RX) tel++, first = std::min(first, x.t_us), last = std::max(last, x.t_us);
    if (x.kind == KILO_WALK_RECORD_RUN) runs++;
    if (x.kind == KILO_WALK_RECORD_MARK && x.payload.find("end reason=REFUSED") == 0 &&
        x.payload.find("why=battery 7000 mV (needs 7400)") != std::string::npos)
      refused++;
    if (x.kind == KILO_WALK_RECORD_TX) CHECK(false);  // heartbeats are counted, not stored
    if (x.kind == KILO_WALK_RECORD_LOG) logs++;
  }
  // from the recording's start (the run was 1 s in) to 2 s after the run
  CHECKF(tel >= 150 && runs == 2 && refused == 1 && last - first >= 3000000, "%zu telemetry, %zu runs, %zu refused",
         tel, runs, refused);
  CHECK(per >= 1 && per <= HostServer::kFetchChunks);

  // any range: the last 500 ms
  kilo_host_rec_fetch_t rq = {0, (uint32_t)(r.clock.now_us() / 1000u) - 500u, 0xFFFFFFFFu, KILO_REC_FETCH_WHAT_BLOCKS};
  got = fetch_all(r, a, KILO_HOST_MSG_REC_FETCH, &rq, sizeof rq, 8, &status);
  rs.clear();
  CHECK(status == KILO_WALK_DATA_STATUS_END && parse_blocks(got, &rs) && !rs.empty() &&
        rs.back().t_us >= r.clock.now_us() - 40000);
  rq = {0, 0, 1, KILO_REC_FETCH_WHAT_LOG};  // no card: no saved log
  fetch_all(r, a, KILO_HOST_MSG_REC_FETCH, &rq, sizeof rq, 9, &status, nullptr, KILO_HOST_RESULT_CODE_BAD_ARG);
  CHECK(status == -2);

  // REC_STATUS
  a.send(KILO_HOST_MSG_REC_STATUS_REQ, 10);
  r.pass();
  fs = a.take();
  const HostFrame* sf = find(fs, KILO_HOST_MSG_REC_STATUS);
  CHECK(sf != nullptr && sf->len == sizeof(kilo_host_rec_status_t));
  if (sf != nullptr) {
    kilo_host_rec_status_t st;
    std::memcpy(&st, sf->payload, sizeof st);
    CHECK(st.capacity == mem.size() && st.block_bytes == record::kBlockBytes && st.runs == 1 &&
          st.skipped == rec.flight().skipped() && st.skipped > 150 && st.card_state == KILO_RECORD_CARD_STATE_NONE &&
          st.unsaved_bytes == st.used && st.clock_ms == (uint32_t)(r.clock.now_us() / 1000u));
  }
  // METRICS without a source: no lines; REC_SAVE without a card: UNSUPPORTED
  uint8_t idx = 0;
  a.send(KILO_HOST_MSG_METRICS_REQ, 11, &idx, 1);
  uint8_t mode = 1;
  a.send(KILO_HOST_MSG_REC_SAVE, 12, &mode, 1);
  r.pass();
  fs = a.take();
  const HostFrame* mf = find(fs, KILO_HOST_MSG_METRICS);
  CHECK(mf != nullptr && mf->payload[1] == 0 && result_code(fs, 12) == KILO_HOST_RESULT_CODE_UNSUPPORTED);

  // a client that stops reading only stalls its download
  a.write_room = 0;
  a.send(KILO_HOST_MSG_WALK_FETCH, 13, &q, sizeof q);
  for (int k = 0; k < 50; k++) r.pass();
  CHECK(a.open && r.server.clients() == 1);
  a.write_room = -1;
  for (int k = 0; k < 300; k++) r.pass();
  a.take();
  // the ring overwrites the blocks during a download: GONE
  a.send(KILO_HOST_MSG_WALK_FETCH, 14, &q, sizeof q);
  r.pass();
  a.take();
  uint8_t big[200] = {};
  for (int i = 0; i < 1000; i++) rec.flight().append(KILO_WALK_RECORD_MARK, r.clock.now_us(), big, sizeof big);
  bool gone = false;
  for (int k = 0; k < 200 && !gone; k++) {
    r.pass();
    for (const HostFrame& f : a.take())
      if (f.type == KILO_HOST_MSG_WALK_DATA && f.payload[12] == KILO_WALK_DATA_STATUS_GONE) gone = true;
  }
  CHECK(gone);
  w = walk_list(r, a, 0, 15);
  CHECK(w.found == 1 && (w.flags & KILO_WALK_FLAGS_TRUNCATED) && w.dropped_walks > 0);
  // id 0 stops a fetch
  kilo_host_walk_fetch_t stop = {0, 0};
  a.send(KILO_HOST_MSG_WALK_FETCH, 16, &stop, sizeof stop);
  r.pass();
  CHECK(result_code(a.take(), 16) == KILO_HOST_RESULT_CODE_OK);
}

// ---- the secure layer (net/host_secure.cpp) -------------------------------------------

// Passes until c has the mind's answer (and message 2 when OK), at most max.
int passes_to_answer(Rig& r, FakeConn& c, int max = 20) {
  for (int k = 1; k <= max; k++) {
    r.pass();
    c.pump();
    if (c.answered && (c.status != KILO_HOST_HELLO_STATUS_OK || c.established || c.hs_failed)) return k;
  }
  return -1;
}

void test_handshake_and_records() {
  Rig r;
  FakeConn& c = r.connect(0);
  CHECK(r.server.clients() == 1 && r.server.stats().handshakes == 1 && c.status == KILO_HOST_HELLO_STATUS_OK);
  CHECK(r.capture.has("noise: client 0: handshake with key ") && r.capture.has("(test@laptop) in "));
  // Several frames in one record, and a frame cut across two records: the
  // plaintext is one stream.
  uint8_t f[3 * KILO_HOST_MAX_FRAME];
  uint32_t n = host_frame_encode(KILO_HOST_MSG_STATUS_REQ, 1, nullptr, 0, f);
  n += host_frame_encode(KILO_HOST_MSG_STATUS_REQ, 2, nullptr, 0, f + n);
  n += host_frame_encode(KILO_HOST_MSG_STATUS_REQ, 3, nullptr, 0, f + n);
  c.send_record(f, 9);
  c.send_record(f + 9, n - 9);
  c.send_record(nullptr, 0);  // an empty record is fine
  r.pass();
  std::vector<HostFrame> got = c.take();
  CHECK(count(got, KILO_HOST_MSG_STATUS) == 3 && c.open && !c.bad_record);
  // The log tail comes back in records of at most KILO_HOST_RECORD_MAX (FakeConn checks each).
  uint8_t sub[3] = {1, 60, KILO_LOG_LEVEL_DEBUG};
  for (int i = 0; i < 70; i++) logf(r.ring, LogLevel::Info, "a line long enough to fill records quickly %d", i);
  c.send(KILO_HOST_MSG_LOG_SUB, 4, sub, 3);
  r.run_ms(50, 1);
  CHECK(count(c.take(), KILO_HOST_MSG_LOG_LINE) == 60 && !c.bad_record);
}

void test_one_x25519_per_pass() {
  Rig r;
  FakeConn& a = r.listener.connect(0);
  FakeConn& b = r.listener.connect(1);
  FakeConn& d = r.listener.connect(2);
  auto answered = [&]() { return (int)a.established + (int)b.established + (int)d.established; };
  // Three connections need nine X25519 steps; one runs per pass, round robin.
  int seen[10] = {};
  for (int k = 1; k <= 9; k++) {
    r.pass();
    a.pump();
    b.pump();
    d.pump();
    seen[k] = answered();
  }
  CHECKF(seen[6] == 0 && seen[7] == 1 && seen[8] == 2 && seen[9] == 3, "answered after 6..9 passes: %d %d %d %d",
         seen[6], seen[7], seen[8], seen[9]);
  r.pass();
  CHECK(r.server.clients() == 3);
}

void test_no_hello_timeout() {
  Rig r;
  FakeConn& idle = r.listener.connect_raw(0);   // sends nothing
  FakeConn& half = r.listener.connect_raw(1);   // half a hello
  half.send_raw({'K', 'I', 'L', 'O', KILO_HOST_VERSION, 0});
  FakeConn& mute = r.listener.connect(2);       // the handshake, but never a record
  mute.first_record = false;
  r.run_ms(2900, 10);
  mute.pump();
  CHECK(idle.open && half.open && mute.open && mute.established);
  CHECK(r.server.clients() == 0 && r.server.connections() == 3);
  // An observer's STATUS counts only clients; a pending connection cannot take control.
  FakeConn& c = r.connect(3);
  c.send(KILO_HOST_MSG_STATUS_REQ, 1);
  r.pass();
  std::vector<HostFrame> got = c.take();
  const HostFrame* s = find(got, KILO_HOST_MSG_STATUS);
  CHECK(s != nullptr && s->payload[offsetof(kilo_host_status_t, clients)] == 1);
  r.run_ms(200, 10);
  CHECK(!idle.open && !half.open && !mute.open && c.open);
  CHECK(r.server.stats().handshake_timeouts == 3 && r.server.clients() == 1);
  CHECK(idle.out.empty() && half.out.empty());  // nothing was ever answered
}

void test_refusals() {
  Rig r;
  // An unknown key: KEY_UNKNOWN in clear, then closed, nothing more.
  FakeConn& u = r.listener.connect_raw(0);
  u.start(stranger_key().priv, stranger_key().pub);
  CHECK(passes_to_answer(r, u) == 1 && u.status == KILO_HOST_HELLO_STATUS_KEY_UNKNOWN && !u.open);
  CHECK(u.raw.empty() && r.capture.has("is not authorized (3 keys known)"));
  // Another version.
  FakeConn& v = r.listener.connect_raw(0);
  v.start(laptop_key().priv, laptop_key().pub, KILO_HOST_VERSION + 1);
  CHECK(passes_to_answer(r, v) == 1 && v.status == KILO_HOST_HELLO_STATUS_BAD_VERSION && !v.open);
  // A plaintext frame (a version-1 client): closed at once, unanswered.
  FakeConn& p = r.listener.connect_raw(0);
  uint8_t f[KILO_HOST_MAX_FRAME];
  uint32_t n = host_frame_encode(KILO_HOST_MSG_STATUS_REQ, 1, nullptr, 0, f);
  p.send_raw(std::vector<uint8_t>(f, f + n));
  r.pass();
  CHECK(!p.open && p.out.empty());
  // No randomness: REFUSED, fail closed.
  r.random.fail = true;
  FakeConn& q = r.listener.connect(0);
  CHECK(passes_to_answer(r, q) == 1 && q.status == KILO_HOST_HELLO_STATUS_REFUSED && !q.open);
  r.random.fail = false;
  CHECK(r.server.stats().hello_refused == 4 && r.server.stats().handshakes == 0 && r.server.clients() == 0);
}

void test_wrong_private_key() {
  Rig r;
  // The laptop's public key with another private key: the mind answers (it
  // cannot tell yet), message 2 does not authenticate for the impostor, and
  // a record it makes up does not authenticate for the mind.
  FakeConn& m = r.listener.connect_raw(0);
  m.start(stranger_key().priv, laptop_key().pub);
  passes_to_answer(r, m);
  CHECK(m.status == KILO_HOST_HELLO_STATUS_OK && m.hs_failed && !m.established && m.open);
  CHECK(r.server.clients() == 0);
  uint8_t key[32] = {9};
  m.send_cs.init(key);
  uint8_t f[KILO_HOST_MAX_FRAME];
  uint32_t n = host_frame_encode(KILO_HOST_MSG_STATUS_REQ, 1, nullptr, 0, f);
  m.send_record(f, n);
  r.pass();
  CHECK(!m.open && r.server.stats().auth_failed == 1 && r.server.stats().handshakes == 0);
}

void test_bad_records() {
  Rig r;
  uint8_t f[KILO_HOST_MAX_FRAME];
  uint32_t n = host_frame_encode(KILO_HOST_MSG_STATUS_REQ, 1, nullptr, 0, f);
  // Tampered: a flipped bit in the ciphertext, in the tag, in the length.
  for (size_t where : {size_t(2), size_t(2 + n + 3), size_t(0)}) {
    FakeConn& c = r.connect(0);
    size_t at = c.in.size();
    c.send(KILO_HOST_MSG_STATUS_REQ, 1);
    c.in[at + where] ^= (where == 0 ? 0x01 : 0x20);
    c.send(KILO_HOST_MSG_STATUS_REQ, 2);  // (a longer length takes a byte of this one)
    r.pass();
    CHECKF(!c.open && c.take().empty(), "tampered at %zu: closed, unanswered", where);
  }
  // Replayed: the same record twice.
  {
    FakeConn& c = r.connect(0);
    size_t at = c.in.size();
    c.send(KILO_HOST_MSG_STATUS_REQ, 1);
    std::vector<uint8_t> rec(c.in.begin() + (long)at, c.in.end());
    r.pass();
    CHECK(c.open && count(c.take(), KILO_HOST_MSG_STATUS) == 1);
    c.send_raw(rec);
    r.pass();
    CHECK(!c.open && c.take().empty());
  }
  // Truncated: a record cut short, the next one behind it.
  {
    FakeConn& c = r.connect(0);
    size_t at = c.in.size();
    c.send(KILO_HOST_MSG_STATUS_REQ, 1);
    c.in.resize(c.in.size() - 5);
    c.send(KILO_HOST_MSG_STATUS_REQ, 2);
    r.pass();
    CHECK(!c.open && c.take().empty());
    (void)at;
  }
  // Too long (one past the limit), and shorter than a tag: closed without reading on.
  for (uint32_t len : {(uint32_t)KILO_HOST_RECORD_MAX + noise::kTagLen + 1, noise::kTagLen - 1}) {
    FakeConn& c = r.connect(0);
    c.send_raw({(uint8_t)len, (uint8_t)(len >> 8)});
    r.pass();
    CHECKF(!c.open, "record length %u", len);
  }
  // The largest record is fine: a STATUS_REQ, a BRIDGE_TX (not the
  // controller: dropped unanswered) and KEEPALIVEs, exactly KILO_HOST_RECORD_MAX bytes.
  {
    FakeConn& c = r.connect(0);
    std::vector<uint8_t> big(f, f + n);
    uint8_t fb[KILO_HOST_MAX_FRAME], pad[196] = {};
    uint32_t bn = host_frame_encode(KILO_HOST_MSG_BRIDGE_TX, 0, pad, sizeof pad, fb);
    big.insert(big.end(), fb, fb + bn);
    while (big.size() < KILO_HOST_RECORD_MAX) {
      uint8_t k[8];
      uint32_t kn = host_frame_encode(KILO_HOST_MSG_KEEPALIVE, 0, nullptr, 0, k);
      big.insert(big.end(), k, k + kn);
    }
    CHECK(big.size() == KILO_HOST_RECORD_MAX);
    c.send_record(big.data(), (uint32_t)big.size());
    r.run_ms(5, 1);
    std::vector<HostFrame> got = c.take();
    CHECK(c.open && got.size() == 1 && count(got, KILO_HOST_MSG_STATUS) == 1);
  }
  CHECK(r.server.stats().auth_failed == 7);
}

void test_slots() {
  Rig r;
  // Four idle connections that never say hello hold every client slot ...
  FakeConn* idle[4];
  for (int s = 0; s < 4; s++) {
    idle[s] = &r.listener.connect_raw(s);
    r.run_ms(10, 10);
  }
  // ... yet a laptop gets in at once: the oldest idle one makes room.
  FakeConn& c = r.connect(4);
  CHECK(c.open && r.server.clients() == 1 && !idle[0]->open && idle[1]->open && r.server.stats().evicted == 1);
  // Again and again: the client is never the one closed.
  for (int k = 0; k < 6; k++) {
    int32_t slot = -1;
    for (int s = 0; s < 4; s++)
      if (!r.listener.conns[s].open) slot = s;
    r.listener.connect_raw(slot);
    r.run_ms(10, 10);
    CHECK(c.open && r.server.connections() == 4);
  }
  c.send(KILO_HOST_MSG_STATUS_REQ, 1);
  r.pass();
  CHECK(c.open && count(c.take(), KILO_HOST_MSG_STATUS) == 1);
  // Four clients: a fifth connection is closed at once.
  Rig q;
  for (int s = 0; s < 4; s++) q.connect(s);
  FakeConn& fifth = q.listener.connect(4);
  q.pass();
  CHECK(!fifth.open && q.server.clients() == 4 && q.server.stats().refused == 1);
}

}  // namespace

// CAL: the calibration relayed to the body. The body here is the test: it
// answers CAL_GET with CAL_VALUES and acks CAL_SET / CAL_SAVE / CAL_REVERT.
struct CalBody {
  Rig& r;
  int16_t off[16] = {0, 0, 0, 0, 0, 0, 0, 0, 32, -48, -256, 32, 16, 0, -16, 8};
  uint8_t refuse = KILO_ACK_STATUS_OK;  // the status CAL_SET / CAL_SAVE get
  bool saved_event = true;              // CAL_SAVE: EVENT EEPROM_SAVED follows
  std::vector<uint8_t> seen;            // the body commands, in order
  uint8_t bseq = 0;
  explicit CalBody(Rig& rig) : r(rig) {}
  void answer() {
    for (const Sent& s : r.serial.take_sent()) {
      if (s.type == KILO_MSG_CAL_GET) {
        seen.push_back(s.type);
        r.serial.feed_frame(KILO_MSG_CAL_VALUES, bseq++, off, sizeof off);
      } else if (s.type == KILO_MSG_CAL_SET || s.type == KILO_MSG_CAL_SAVE || s.type == KILO_MSG_CAL_REVERT) {
        seen.push_back(s.type);
        if (s.type == KILO_MSG_CAL_SET && refuse == KILO_ACK_STATUS_OK)
          off[s.payload[0]] = (int16_t)(s.payload[1] | (s.payload[2] << 8));
        kilo_ack_t a = {s.seq, s.type, refuse};
        r.serial.feed_frame(KILO_MSG_ACK, bseq++, &a, sizeof a);
        if (s.type == KILO_MSG_CAL_SAVE && refuse == KILO_ACK_STATUS_OK && saved_event) {
          kilo_event_t e = {KILO_EVENT_EEPROM_SAVED, 0, 0, 1};
          r.server.on_body_event(e);  // the Mind forwards every EVENT (Mind::on_event)
        }
      }
    }
  }
  void run_ms(uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 5) {
      r.clock.advance_ms(5);
      r.pass();
      answer();
    }
  }
};

const HostFrame* cal_status(const std::vector<HostFrame>& fs, uint8_t seq) {
  for (const HostFrame& f : fs)
    if (f.type == KILO_HOST_MSG_CAL_STATUS && f.seq == seq) return &f;
  return nullptr;
}

int16_t status_offset(const HostFrame* f, int j) {
  return (int16_t)(f->payload[2 + 2 * j] | (f->payload[3 + 2 * j] << 8));
}

void test_cal() {
  Rig r;
  CalBody body(r);
  kilo_hello_t h = {};
  h.proto_major = KILO_PROTO_MAJOR;
  h.epoch = 1;
  r.serial.feed_frame(KILO_MSG_HELLO, 0, &h, sizeof h);
  kilo_telemetry_t t = {};
  t.epoch = 1;
  r.serial.feed_frame(KILO_MSG_TELEMETRY, 1, &t, sizeof t);
  FakeConn& a = r.connect(0);
  FakeConn& b = r.connect(1);
  body.run_ms(20);
  a.take();
  b.take();
  // GET: any client
  kilo_host_cal_t get = {KILO_CAL_OP_GET, 0, 0};
  b.send(KILO_HOST_MSG_CAL, 1, &get, sizeof get);
  body.run_ms(50);
  std::vector<HostFrame> fb = b.take();
  const HostFrame* st = cal_status(fb, 1);
  CHECK(st != nullptr && st->payload[0] == KILO_CAL_OP_GET && status_offset(st, 10) == -256 && status_offset(st, 9) == -48);
  // SET from an observer with nothing held: refused; out of range: refused with the range
  kilo_host_cal_t set = {KILO_CAL_OP_SET, 10, 136};
  b.send(KILO_HOST_MSG_CAL, 2, &set, sizeof set);
  kilo_host_cal_t far = {KILO_CAL_OP_SET, 10, 401};
  b.send(KILO_HOST_MSG_CAL, 3, &far, sizeof far);
  kilo_host_cal_t badj = {KILO_CAL_OP_SET, 16, 0};
  b.send(KILO_HOST_MSG_CAL, 4, &badj, sizeof badj);
  body.run_ms(20);
  fb = b.take();
  CHECK(result_code(fb, 2) == KILO_HOST_RESULT_CODE_NOT_CONTROLLER && result_code(fb, 3) == KILO_HOST_RESULT_CODE_BAD_ARG &&
        result_code(fb, 4) == KILO_HOST_RESULT_CODE_BAD_ARG);
  CHECK(body.seen.size() == 1);  // only the GET reached the body
  // a holds the calibration posture; b's SET goes through (the nudge from another window)
  uint8_t take[2] = {1, 0};  // no keepalive: the long waits below send nothing
  kilo_host_posture_t calib = {KILO_POSTURE_CALIB, 0, 0};
  a.send(KILO_HOST_MSG_CONTROL, 1, take, 2);
  a.send(KILO_HOST_MSG_POSTURE, 2, &calib, sizeof calib);
  body.run_ms(20);
  b.send(KILO_HOST_MSG_CAL, 5, &set, sizeof set);
  b.send(KILO_HOST_MSG_CAL, 6, &get, sizeof get);  // while the SET is in flight: BUSY
  body.run_ms(60);
  fb = b.take();
  st = cal_status(fb, 5);
  CHECK(st != nullptr && st->payload[0] == KILO_CAL_OP_SET && status_offset(st, 10) == 136);
  CHECK(result_code(fb, 6) == KILO_HOST_RESULT_CODE_BUSY);
  CHECK(body.seen.size() == 3 && body.seen[1] == KILO_MSG_CAL_SET && body.seen[2] == KILO_MSG_CAL_GET);
  CHECK(r.capture.has("host: client 1: calibration joint 10 offset +8.50 deg"));
  // SAVE: waits for EEPROM_SAVED, then the offsets
  kilo_host_cal_t save = {KILO_CAL_OP_SAVE, 0, 0};
  a.send(KILO_HOST_MSG_CAL, 3, &save, sizeof save);
  body.run_ms(60);
  std::vector<HostFrame> fa = a.take();
  st = cal_status(fa, 3);
  CHECK(st != nullptr && st->payload[0] == KILO_CAL_OP_SAVE && st->payload[1] == 1 && status_offset(st, 10) == 136);
  // no EEPROM_SAVED: answered after the wait, saved 0
  body.saved_event = false;
  a.send(KILO_HOST_MSG_CAL, 4, &save, sizeof save);
  body.run_ms(1500);
  CHECK(cal_status(a.take(), 4) == nullptr);
  body.run_ms(600);
  fa = a.take();
  st = cal_status(fa, 4);
  CHECK(st != nullptr && st->payload[1] == 0 && r.capture.has("no EEPROM_SAVED within 2000 ms"));
  // the body refuses (EEPROM busy): BUSY, with its reason
  body.refuse = KILO_ACK_STATUS_EEPROM_BUSY;
  a.send(KILO_HOST_MSG_CAL, 5, &set, sizeof set);
  body.run_ms(60);
  fa = a.take();
  CHECK(result_code(fa, 5) == KILO_HOST_RESULT_CODE_BUSY && cal_status(fa, 5) == nullptr);
  // REVERT
  body.refuse = KILO_ACK_STATUS_OK;
  kilo_host_cal_t rev = {KILO_CAL_OP_REVERT, 0, 0};
  a.send(KILO_HOST_MSG_CAL, 6, &rev, sizeof rev);
  body.run_ms(60);
  fa = a.take();
  st = cal_status(fa, 6);
  CHECK(st != nullptr && st->payload[0] == KILO_CAL_OP_REVERT && body.seen.back() == KILO_MSG_CAL_GET);
  // a body that never answers: NO_BODY after the link's retries
  Rig r2;
  r2.serial.feed_frame(KILO_MSG_HELLO, 0, &h, sizeof h);
  FakeConn& c = r2.connect(0);
  r2.run_ms(20);
  c.take();
  c.send(KILO_HOST_MSG_CAL, 1, &get, sizeof get);
  r2.run_ms(1000);
  int code = result_code(c.take(), 1);
  CHECKF(code == KILO_HOST_RESULT_CODE_NO_BODY, "code %d: %s", code, r2.capture.lines.empty() ? "" : r2.capture.lines.back().c_str());
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: test_host VECTORS.txt\n");
    return 2;
  }
  test_vectors(argv[1]);
  test_rx_split_and_back_to_back();
  test_logring();
  test_status_and_errors();
  test_control_rules();
  test_keepalive();
  test_telem_rate();
  test_log_tail();
  test_bridge();
  test_reboot();
  test_slow_client_dropped();
  test_runs();
  test_handshake_and_records();
  test_one_x25519_per_pass();
  test_no_hello_timeout();
  test_refusals();
  test_wrong_private_key();
  test_bad_records();
  test_slots();
  test_cal();
  std::printf("  %d host vectors\n", g_vectors);
  return report("host");
}
