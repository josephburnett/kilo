// The laptop interface (docs/fast-boot.md P2): host framing against
// protocol/vectors/host_frames.json (flattened by flatten_vectors.py), the
// RAM log ring, and HostServer driven through fake connections, a fake UART
// and a fake clock: replies and their seqs, the control rules (one
// controller, observers refused, REST on release, close and keepalive
// timeout), TELEM rate, the log tail, the raw bridge both ways, out-of-step
// streams and back-pressure.
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

// ---- fakes of the network seams ------------------------------------------------

class FakeConn final : public Conn {
 public:
  std::vector<uint8_t> in;   // what the client sent, read from pos on
  size_t pos = 0;
  std::vector<uint8_t> out;  // what the server wrote
  bool open = false;
  bool peer_closed = false;
  int32_t write_room = -1;   // >= 0: take at most this many bytes per write
  HostRx rx;                 // the client's decoder of out

  int32_t read(uint8_t* buf, uint32_t n) override {
    if (!open) return -1;
    size_t k = std::min<size_t>(n, in.size() - pos);
    if (k == 0) return peer_closed ? -1 : 0;
    std::memcpy(buf, in.data() + pos, k);
    pos += k;
    return (int32_t)k;
  }
  int32_t write(const uint8_t* buf, uint32_t n) override {
    if (!open) return -1;
    uint32_t k = write_room >= 0 && (uint32_t)write_room < n ? (uint32_t)write_room : n;
    out.insert(out.end(), buf, buf + k);
    return (int32_t)k;
  }
  void close() override { open = false; }
  bool is_open() const override { return open; }

  void send(uint8_t type, uint8_t seq, const void* payload = nullptr, uint32_t len = 0) {
    uint8_t f[KILO_HOST_MAX_FRAME];
    uint32_t n = host_frame_encode(type, seq, (const uint8_t*)payload, len, f);
    in.insert(in.end(), f, f + n);
  }
  // The frames the server wrote since the last call.
  std::vector<HostFrame> take() {
    std::vector<HostFrame> got;
    for (uint8_t b : out) {
      HostRxStatus st = rx.push(b);
      if (st == HostRxStatus::Frame) got.push_back(rx.frame());
      CHECKF(st == HostRxStatus::Frame || st == HostRxStatus::None, "server wrote a broken frame");
    }
    out.clear();
    return got;
  }
};

class FakeListener final : public Listener {
 public:
  FakeConn conns[5];  // one more than the server serves
  std::deque<int32_t> pending;

  int32_t accept() override {
    if (pending.empty()) return -1;
    int32_t s = pending.front();
    pending.pop_front();
    conns[s].open = true;
    return s;
  }
  Conn* conn(uint32_t slot) override { return &conns[slot]; }
  uint32_t slots() const override { return 5; }

  FakeConn& connect(int32_t slot) {
    conns[slot] = FakeConn();
    pending.push_back(slot);
    return conns[slot];
  }
};

struct Rig {
  FakeSerial serial;
  FakeClock clock;
  CaptureLog capture;
  LogRing ring{clock, &capture};
  SerialLink link{serial, clock, ring};
  FakeListener listener;
  HostServer server{listener, link, serial, clock, ring, ring, 0x1E1405F0u};

  Rig() { link.connect(); }
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
}

void test_status_and_errors() {
  Rig r;
  FakeConn& c = r.listener.connect(0);
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
  // A bad CRC: out of step, closed.
  size_t at = c.in.size();
  c.send(KILO_HOST_MSG_STATUS_REQ, 5);
  c.in[at + 3] ^= 0x40;
  r.pass();
  CHECK(!c.open && r.server.clients() == 0 && r.server.stats().out_of_step == 1);
  // A slot past what the server serves is closed at accept.
  FakeConn& extra = r.listener.connect(4);
  r.pass();
  CHECK(!extra.open && r.server.stats().refused == 1);
}

void test_control_rules() {
  Rig r;
  kilo_telemetry_t t = {};
  t.epoch = 2;  // epoch goals wait for the body's first epoch
  r.serial.feed_frame(KILO_MSG_TELEMETRY, 0, &t, sizeof t);
  FakeConn& a = r.listener.connect(0);
  FakeConn& b = r.listener.connect(1);
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
  FakeConn& a = r.listener.connect(0);
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
  FakeConn& a = r.listener.connect(0);
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
  FakeConn& a = r.listener.connect(0);
  r.pass();  // "client 0 connected" is logged too
  uint8_t sub[3] = {1, 4, KILO_LOG_LEVEL_INFO};
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
  // the last 4 lines: early 3, early 4, (debug: filtered), client connected
  CHECKF(lines.size() == 3, "%zu lines", lines.size());
  if (lines.size() == 3) {
    CHECK(lines[0] == "early 3" && lines[1] == "early 4" && lines[2].find("client 0 connected") != std::string::npos);
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
  FakeConn& a = r.listener.connect(0);
  FakeConn& b = r.listener.connect(1);
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

void test_slow_client_dropped() {
  Rig r;
  FakeConn& a = r.listener.connect(0);
  r.pass();
  a.write_room = 0;  // never drains
  for (int i = 0; i < 40; i++) a.send(KILO_HOST_MSG_STATUS_REQ, (uint8_t)i);  // 40 x 170 B > the 4 KB ring
  r.pass();
  CHECK(!a.open && r.server.clients() == 0);
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
  test_slow_client_dropped();
  std::printf("  %d host vectors\n", g_vectors);
  return report("host");
}
