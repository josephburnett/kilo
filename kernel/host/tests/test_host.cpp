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
  for (int i = 2; i < (int)LogRing::kLines + 10; i++) logf(ring, LogLevel::Debug, "line %d", i);
  CHECK(ring.next_seq() == LogRing::kLines + 10 && ring.first_seq() == 10);
  CHECK(ring.get(9) == nullptr && ring.get(10) != nullptr && std::string(ring.get(10)->text) == "line 10");
  CHECK(ring.get(LogRing::kLines + 10) == nullptr);
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
  ring.set_watched(true);
  logf(ring.status(), LogLevel::Info, "status %d", 3);
  CHECK(ring.next_seq() == kept + 2 && std::string(ring.get(kept + 1)->text) == "status 3" &&
        next.lines.size() == passed + 3);
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

// ---- walk recordings (WALK_LIST, WALK_FETCH) -------------------------------------------

void fill_walk(record::WalkStore& s, uint32_t records, uint8_t salt) {
  s.open(0);
  uint8_t p[60];
  for (uint32_t i = 0; i < records; i++) {
    for (uint32_t k = 0; k < sizeof p; k++) p[k] = (uint8_t)(i + k + salt);
    s.append(KILO_WALK_RECORD_RX, i * 20000, p, sizeof p);
  }
}

std::vector<uint8_t> store_bytes(const record::WalkStore& s, uint32_t id) {
  std::vector<uint8_t> out(s.find(id)->size);
  uint32_t total;
  CHECK(s.read(id, 0, out.data(), (uint32_t)out.size(), &total) == (int32_t)out.size());
  return out;
}

kilo_host_walk_info_t walk_list(Rig& r, FakeConn& c, uint32_t after, uint8_t seq) {
  c.send(KILO_HOST_MSG_WALK_LIST, seq, &after, sizeof after);
  r.pass();
  kilo_host_walk_info_t w;
  std::memset(&w, 0, sizeof w);
  for (const HostFrame& f : c.take())
    if (f.type == KILO_HOST_MSG_WALK_INFO && f.seq == seq) std::memcpy(&w, f.payload, f.len);
  return w;
}

void test_walks() {
  Rig r;
  FakeConn& a = r.connect(0);
  r.pass();
  a.take();
  // no store attached: UNSUPPORTED
  uint32_t zero = 0;
  a.send(KILO_HOST_MSG_WALK_LIST, 1, &zero, 4);
  r.pass();
  CHECK(result_code(a.take(), 1) == KILO_HOST_RESULT_CODE_UNSUPPORTED);

  std::vector<uint8_t> mem(record::WalkStore::kIndexBytes + 8 + 200000);
  record::WalkStore st;
  st.attach(mem.data(), (uint32_t)mem.size());
  r.server.attach_walks(st);
  fill_walk(st, 400, 1);  // walk 1: 400 x 66 B = 26 KB
  const char* t1 = "gait=trot dir=forward speed=55.6 end=stopped";
  std::memcpy(st.current()->text, t1, std::strlen(t1));
  st.current()->text_len = (uint8_t)std::strlen(t1);
  st.current()->end_us = st.current()->start_us + 8000000;
  st.close();
  fill_walk(st, 10, 2);  // walk 2: still being recorded

  kilo_host_walk_info_t w = walk_list(r, a, 0, 2);
  CHECKF(w.found == 1 && w.id == 1 && w.kept == 2 && w.recording == 2, "found %u id %u kept %u rec %u", w.found,
         (unsigned)w.id, w.kept, (unsigned)w.recording);
  CHECKF(w.bytes == 400u * 66u && w.records == 400 && w.duration_ms == 8000, "%u B %u records %u ms",
         (unsigned)w.bytes, (unsigned)w.records, (unsigned)w.duration_ms);
  CHECK(w.found == 1 && w.id == 1 && w.kept == 2 && w.recording == 2 && w.bytes == 400u * 66u && w.records == 400 &&
        w.duration_ms == 8000 && w.capacity == st.capacity() && w.used == st.used() &&
        std::memcmp(w.text, t1, std::strlen(t1)) == 0 && !(w.flags & KILO_WALK_FLAGS_RECORDING));
  w = walk_list(r, a, 1, 3);
  CHECK(w.found == 1 && w.id == 2 && (w.flags & KILO_WALK_FLAGS_RECORDING));
  w = walk_list(r, a, 2, 4);
  CHECK(w.found == 0 && w.kept == 2 && w.recording == 2);

  kilo_host_walk_fetch_t q = {2, 0};
  a.send(KILO_HOST_MSG_WALK_FETCH, 5, &q, sizeof q);  // still being recorded
  q = {9, 0};
  a.send(KILO_HOST_MSG_WALK_FETCH, 6, &q, sizeof q);  // no such walk
  q = {1, 400u * 66u + 1};
  a.send(KILO_HOST_MSG_WALK_FETCH, 7, &q, sizeof q);  // past its end
  r.pass();
  std::vector<HostFrame> fs = a.take();
  CHECK(result_code(fs, 5) == KILO_HOST_RESULT_CODE_BAD_ARG && result_code(fs, 6) == KILO_HOST_RESULT_CODE_BAD_ARG &&
        result_code(fs, 7) == KILO_HOST_RESULT_CODE_BAD_ARG && count(fs, KILO_HOST_MSG_WALK_DATA) == 0);

  // walk 1, while walk 2 is recorded: one chunk a pass
  std::vector<uint8_t> want = store_bytes(st, 1), got;
  q = {1, 0};
  a.send(KILO_HOST_MSG_WALK_FETCH, 8, &q, sizeof q);
  bool end = false;
  size_t max_per_pass = 0;
  for (int k = 0; k < 2000 && !end; k++) {
    r.pass();
    fs = a.take();
    if (k == 0) CHECK(result_code(fs, 8) == KILO_HOST_RESULT_CODE_OK);
    max_per_pass = std::max(max_per_pass, count(fs, KILO_HOST_MSG_WALK_DATA));
    for (const HostFrame& f : fs) {
      if (f.type != KILO_HOST_MSG_WALK_DATA) continue;
      kilo_host_walk_data_t d;
      std::memcpy(&d, f.payload, f.len);
      CHECK(d.id == 1 && d.total == want.size() && d.offset == got.size());
      if (d.status == KILO_WALK_DATA_STATUS_END) end = true;
      else got.insert(got.end(), d.data, d.data + (f.len - KILO_HOST_WALK_DATA_MIN_LEN));
    }
  }
  CHECK(end && got == want && max_per_pass == 1 && r.server.stats().walk_fetches == 1);

  // walk 2 ends: kFetchChunks a pass, and a client that stops reading only
  // stalls its download (it is not dropped, nothing is lost)
  st.close();
  got.clear();
  end = false;
  a.write_room = 0;
  a.send(KILO_HOST_MSG_WALK_FETCH, 9, &q, sizeof q);
  for (int k = 0; k < 50; k++) r.pass();
  CHECK(a.open && r.server.clients() == 1);
  a.write_room = -1;
  max_per_pass = 0;
  for (int k = 0; k < 2000 && !end; k++) {
    r.pass();
    fs = a.take();
    max_per_pass = std::max(max_per_pass, count(fs, KILO_HOST_MSG_WALK_DATA));
    for (const HostFrame& f : fs) {
      if (f.type != KILO_HOST_MSG_WALK_DATA) continue;
      kilo_host_walk_data_t d;
      std::memcpy(&d, f.payload, f.len);
      CHECK(d.offset == got.size());
      if (d.status == KILO_WALK_DATA_STATUS_END) end = true;
      else got.insert(got.end(), d.data, d.data + (f.len - KILO_HOST_WALK_DATA_MIN_LEN));
    }
  }
  CHECKF(end && got == want && max_per_pass > 1 && max_per_pass <= HostServer::kFetchChunks + 20, "per pass %zu",
         max_per_pass);  // the stalled ring drains in its first passes

  // the walk is dropped from the store during its download: GONE
  a.send(KILO_HOST_MSG_WALK_FETCH, 10, &q, sizeof q);
  r.pass();
  a.take();
  for (uint32_t i = 0; i < 10; i++) {  // the store makes room: walk 1 goes
    fill_walk(st, 400, (uint8_t)(10 + i));
    st.close();
  }
  CHECK(st.find(1) == nullptr);
  bool gone = false;
  for (int k = 0; k < 50 && !gone; k++) {
    r.pass();
    for (const HostFrame& f : a.take())
      if (f.type == KILO_HOST_MSG_WALK_DATA && f.payload[12] == KILO_WALK_DATA_STATUS_GONE) gone = true;
  }
  CHECK(gone);
  // id 0 stops a fetch
  q = {st.after(0)->id, 0};
  a.send(KILO_HOST_MSG_WALK_FETCH, 11, &q, sizeof q);
  kilo_host_walk_fetch_t stop = {0, 0};
  a.send(KILO_HOST_MSG_WALK_FETCH, 12, &stop, sizeof stop);
  r.pass();
  fs = a.take();
  CHECK(result_code(fs, 12) == KILO_HOST_RESULT_CODE_OK);
  r.pass();
  r.pass();
  CHECK(count(a.take(), KILO_HOST_MSG_WALK_DATA) == 0);
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
  test_walks();
  test_handshake_and_records();
  test_one_x25519_per_pass();
  test_no_hello_timeout();
  test_refusals();
  test_wrong_private_key();
  test_bad_records();
  test_slots();
  std::printf("  %d host vectors\n", g_vectors);
  return report("host");
}
