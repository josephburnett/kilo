#include "net/host_server.h"
#include "util/log.h"
#include "util/mem.h"

namespace kilo {

namespace {

constexpr uint32_t kKeepaliveUs = KILO_HOST_KEEPALIVE_TIMEOUT_MS * 1000u;
constexpr uint32_t kBodyLostUs = KILO_BODY_LOST_MS * 1000u;
constexpr uint32_t kReadChunk = 256;
constexpr uint32_t kReadsPerPoll = 4;  // bounded work per client per pass (1 KB)
constexpr uint32_t kResultTextMax = KILO_HOST_RESULT_MAX_LEN - 2;

const char* name_of(uint8_t type) {
  switch (type) {
    case KILO_HOST_MSG_STATUS_REQ: return "STATUS_REQ";
    case KILO_HOST_MSG_CONTROL: return "CONTROL";
    case KILO_HOST_MSG_POSTURE: return "POSTURE";
    case KILO_HOST_MSG_PARAM_GET: return "PARAM_GET";
    case KILO_HOST_MSG_PARAM_SET: return "PARAM_SET";
    case KILO_HOST_MSG_TELEM_SUB: return "TELEM_SUB";
    case KILO_HOST_MSG_LOG_SUB: return "LOG_SUB";
    case KILO_HOST_MSG_BRIDGE_OPEN: return "BRIDGE_OPEN";
    case KILO_HOST_MSG_BRIDGE_CLOSE: return "BRIDGE_CLOSE";
    default: return "?";
  }
}

}  // namespace

HostServer::HostServer(Listener& listener, SerialLink& link, Serial& serial, Clock& clock, Log& log, LogRing& ring,
                       uint32_t mind_build)
    : listener_(listener),
      link_(link),
      serial_(serial),
      clock_(clock),
      log_(log),
      ring_(ring),
      mind_build_(mind_build),
      start_us_(clock.now_us()) {}

uint32_t HostServer::clients() const {
  uint32_t n = 0;
  for (const Client& c : clients_) n += c.open ? 1u : 0u;
  return n;
}

// ---- the pass --------------------------------------------------------------------

void HostServer::poll() {
  uint64_t now = clock_.now_us();
  accept(now);
  for (uint32_t i = 0; i < kMaxClients; i++)
    if (clients_[i].open) read_client(i, now);
  if (controller_ >= 0) {
    const Client& c = clients_[controller_];
    if (c.keepalive && now - c.last_rx_us > kKeepaliveUs) {
      logf(log_, LogLevel::Warn, "host: client %d silent for %u ms", (int)controller_,
           (unsigned)((now - c.last_rx_us) / 1000u));
      lose_control("keepalive timeout");
    }
  }
  if (bridge_) pump_bridge();
  streams(now);
  for (uint32_t i = 0; i < kMaxClients; i++)
    if (clients_[i].open) flush(i);
}

void HostServer::accept(uint64_t now) {
  for (;;) {
    int32_t slot = listener_.accept();
    if (slot < 0) return;
    Conn* conn = listener_.conn((uint32_t)slot);
    if ((uint32_t)slot >= kMaxClients || clients_[slot].open) {
      conn->close();
      stats_.refused++;
      continue;
    }
    Client& c = clients_[slot];
    mem_zero(&c, sizeof c);
    c.open = true;
    c.conn = conn;
    c.rx.reset();
    c.last_rx_us = now;
    stats_.accepted++;
    logf(log_, LogLevel::Info, "host: client %d connected (%u open)", (int)slot, (unsigned)clients());
  }
}

void HostServer::read_client(uint32_t i, uint64_t now) {
  uint8_t buf[kReadChunk];
  for (uint32_t r = 0; r < kReadsPerPoll; r++) {
    Client& c = clients_[i];
    if (!c.open) return;
    int32_t n = c.conn->read(buf, sizeof buf);
    if (n < 0) {
      drop(i, "closed");
      return;
    }
    if (n == 0) return;
    for (int32_t k = 0; k < n; k++) {
      HostRxStatus st = c.rx.push(buf[k]);
      if (st == HostRxStatus::Frame) {
        handle(i, c.rx.frame(), now);
        if (!c.open) return;
      } else if (st != HostRxStatus::None) {
        stats_.out_of_step++;
        drop(i, st == HostRxStatus::BadCrc ? "bad CRC: out of step" : "bad length: out of step");
        return;
      }
    }
  }
}

void HostServer::drop(uint32_t i, const char* why) {
  Client& c = clients_[i];
  if (!c.open) return;
  c.conn->close();
  c.open = false;
  stats_.closed++;
  logf(log_, LogLevel::Info, "host: client %d %s", (int)i, why);
  if (controller_ == (int32_t)i) lose_control(why);
}

void HostServer::lose_control(const char* why) {
  if (controller_ < 0) return;
  int32_t was = controller_;
  controller_ = -1;
  if (bridge_) close_bridge("the controller let go");
  stats_.control_lost++;
  // Python: the daemon ends a client's hold with REST (kilo/daemon/sessions.py).
  kilo_set_posture_t rest = {0, KILO_POSTURE_REST, 0, 0};  // the link fills the epoch
  bool ok = link_.set_goal(KILO_MSG_SET_POSTURE, (const uint8_t*)&rest, sizeof rest);
  logf(log_, LogLevel::Info, "host: client %d gave up control (%s): %s", (int)was, why,
       ok ? "REST latched" : "REST refused by the link");
}

void HostServer::close_bridge(const char* why) {
  if (!bridge_) return;
  bridge_ = false;
  // The laptop spoke to the body with its own seqs and session: start over,
  // as after a reopened port (HELLO_REQ, goals resent once the body is heard).
  link_.disconnect();
  link_.connect();
  logf(log_, LogLevel::Info, "host: bridge closed (%s); link resumed", why);
}

// ---- requests -------------------------------------------------------------------

void HostServer::handle(uint32_t i, const HostFrame& f, uint64_t now) {
  Client& c = clients_[i];
  c.last_rx_us = now;
  uint8_t kind = kilo_host_msg_kind(f.type);
  uint8_t len_st = kilo_host_payload_len_check(f.type, f.len);
  bool command = kind == KILO_KIND_COMMAND;
  if (len_st == KILO_LEN_UNKNOWN_TYPE || (kind != KILO_KIND_COMMAND && kind != KILO_KIND_STREAM) ||
      f.type >= 0x80) {
    result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_UNKNOWN_TYPE, "unknown type");
    return;
  }
  if (len_st != KILO_LEN_OK) {
    result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_BAD_LEN, "bad payload length");
    return;
  }
  if (command) stats_.requests++;
  bool is_controller = controller_ == (int32_t)i;
  const uint8_t* p = f.payload;
  switch (f.type) {
    case KILO_HOST_MSG_STATUS_REQ: {
      kilo_host_status_t s;
      mem_zero(&s, sizeof s);
      const BodyState& b = link_.state();
      s.host_version = KILO_HOST_VERSION;
      s.mind_build = mind_build_;
      s.uptime_ms = (uint32_t)((now - start_us_) / 1000u);
      s.state = state_flags(now);
      s.controller = controller_ < 0 ? KILO_HOST_CONTROLLER_NONE
                     : is_controller  ? KILO_HOST_CONTROLLER_YOU
                                      : KILO_HOST_CONTROLLER_OTHER;
      s.clients = (uint8_t)clients();
      s.proto_ok = b.proto_ok;
      s.telemetry_age_ms = telemetry_age_ms(now);
      s.reboots = b.reboots;
      s.telemetry_count = b.telemetry_count;
      s.event_count = b.event_count;
      if (b.has_event) s.last_event = b.last_event;
      if (b.has_hello) s.hello = b.hello;
      fill_link_stats(s.link);
      if (b.has_telemetry) s.telemetry = b.telemetry;
      reply(i, f.seq, &s, sizeof s, KILO_HOST_MSG_STATUS);
      return;
    }
    case KILO_HOST_MSG_CONTROL: {
      uint8_t take = p[0], keepalive = p[1];
      if (take > 1 || keepalive > 1) return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_BAD_ARG, "take, keepalive: 0 or 1");
      if (take == 0) {
        if (!is_controller) return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_NOT_CONTROLLER, "not the controller");
        lose_control("released");
        return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_OK, "released: REST");
      }
      if (controller_ >= 0 && !is_controller)
        return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_BUSY, "another client has control");
      if (!is_controller) logf(log_, LogLevel::Info, "host: client %d has control%s", (int)i, keepalive ? " (keepalive)" : "");
      controller_ = (int32_t)i;
      c.keepalive = keepalive != 0;
      return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_OK);
    }
    case KILO_HOST_MSG_KEEPALIVE:
      return;
    case KILO_HOST_MSG_POSTURE: {
      kilo_host_posture_t q;
      mem_copy(&q, p, sizeof q);
      if (!is_controller) return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_NOT_CONTROLLER, "take control first");
      if (bridge_) return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_BRIDGED, "the bridge is open");
      if (q.posture > KILO_POSTURE_ZERO) return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_BAD_ARG, "no such posture");
      if (!link_.connected() || link_.state().proto_ok == 0)
        return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_NO_BODY, "no body to drive");
      kilo_set_posture_t g = {0, q.posture, q.transition_ticks, q.flags};
      if (!link_.set_goal(KILO_MSG_SET_POSTURE, (const uint8_t*)&g, sizeof g))
        return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_NO_BODY, "the link refused the goal");
      logf(log_, LogLevel::Info, "host: client %d: posture %u", (int)i, (unsigned)q.posture);
      return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_OK);
    }
    case KILO_HOST_MSG_PARAM_GET:
      return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_UNSUPPORTED, "params: not yet (P3)");
    case KILO_HOST_MSG_PARAM_SET:
      if (!is_controller) return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_NOT_CONTROLLER, "take control first");
      return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_UNSUPPORTED, "params: not yet (P3)");
    case KILO_HOST_MSG_TELEM_SUB: {
      uint8_t hz = p[0];
      if (hz > KILO_HOST_TELEM_MAX_HZ) return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_BAD_ARG, "hz: 0..50");
      c.telem_period_us = hz == 0 ? 0u : 1000000u / hz;
      c.telem_next_us = now;
      return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_OK);
    }
    case KILO_HOST_MSG_LOG_SUB: {
      uint8_t enable = p[0], tail = p[1], min_level = p[2];
      if (enable > 1 || min_level > KILO_LOG_LEVEL_ERROR)
        return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_BAD_ARG, "enable 0..1, min_level 0..3");
      c.log_on = enable != 0;
      c.log_min = min_level;
      uint32_t next = ring_.next_seq();
      c.log_seq = next - (tail < next - ring_.first_seq() ? tail : next - ring_.first_seq());
      return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_OK);
    }
    case KILO_HOST_MSG_BRIDGE_OPEN:
      if (!is_controller) return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_NOT_CONTROLLER, "take control first");
      if (!link_.connected()) return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_NO_BODY, "the UART is closed");
      if (!bridge_) {
        bridge_ = true;
        logf(log_, LogLevel::Info, "host: bridge open for client %d: link paused", (int)i);
      }
      return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_OK);
    case KILO_HOST_MSG_BRIDGE_TX: {
      if (!bridge_ || !is_controller) return;  // a stream: never answered
      int32_t w = serial_.write(p, f.len);
      if (w < 0) {
        close_bridge("UART write failed");  // the link's next poll() finds the failure and disconnects
        return;
      }
      stats_.bridge_tx_bytes += (uint32_t)w;
      stats_.bridge_tx_short += f.len - (uint32_t)w;
      return;
    }
    case KILO_HOST_MSG_BRIDGE_CLOSE:
      if (!is_controller) return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_NOT_CONTROLLER, "take control first");
      if (!bridge_) return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_NOT_BRIDGED, "no bridge open");
      close_bridge("BRIDGE_CLOSE");
      return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_OK);
    default:
      return result(i, f.seq, f.type, KILO_HOST_RESULT_CODE_UNSUPPORTED, name_of(f.type));
  }
}

// ---- streams --------------------------------------------------------------------

void HostServer::pump_bridge() {
  if (controller_ < 0) return;
  uint32_t i = (uint32_t)controller_;
  for (;;) {
    uint32_t space = room(i);
    if (space <= kHostOverhead) return;  // the controller's ring is full: leave the bytes in the UART
    uint32_t n = space - kHostOverhead;
    if (n > KILO_HOST_BRIDGE_CHUNK_MAX) n = KILO_HOST_BRIDGE_CHUNK_MAX;
    uint8_t buf[KILO_HOST_BRIDGE_CHUNK_MAX];
    int32_t got = serial_.read(buf, n);
    if (got < 0) {
      close_bridge("UART read failed");  // the link's next poll() finds the failure and disconnects
      return;
    }
    if (got == 0) return;
    stats_.bridge_rx_bytes += (uint32_t)got;
    Client& c = clients_[i];
    enqueue(i, KILO_HOST_MSG_BRIDGE_RX, c.out_seq++, buf, (uint32_t)got);
  }
}

void HostServer::streams(uint64_t now) {
  for (uint32_t i = 0; i < kMaxClients; i++) {
    Client& c = clients_[i];
    if (!c.open) continue;
    if (c.telem_period_us != 0 && now >= c.telem_next_us) {
      c.telem_next_us += c.telem_period_us;
      if (c.telem_next_us <= now) c.telem_next_us = now + c.telem_period_us;  // fell behind: no burst
      kilo_host_telem_t t;
      mem_zero(&t, sizeof t);
      const BodyState& b = link_.state();
      t.uptime_ms = (uint32_t)((now - start_us_) / 1000u);
      t.state = state_flags(now);
      t.telemetry_age_ms = telemetry_age_ms(now);
      t.telemetry_count = b.telemetry_count;
      t.event_count = b.event_count;
      if (b.has_event) t.last_event = b.last_event;
      fill_link_stats(t.link);
      if (b.has_telemetry) t.telemetry = b.telemetry;
      if (enqueue(i, KILO_HOST_MSG_TELEM, c.out_seq, &t, sizeof t)) {
        c.out_seq++;
        stats_.telem_sent++;
      } else {
        stats_.telem_skipped++;
      }
    }
    while (c.log_on) {
      if (c.log_seq < ring_.first_seq()) c.log_seq = ring_.first_seq();  // overwritten: the seq gap tells
      const LogEntry* e = ring_.get(c.log_seq);
      if (e == nullptr) break;  // caught up
      if ((uint8_t)e->level < c.log_min) {
        c.log_seq++;
        continue;
      }
      kilo_host_log_line_t l;
      l.seq = e->seq;
      l.t_ms = e->t_ms;
      l.level = (uint8_t)e->level;
      mem_copy(l.text, e->text, e->len);
      if (!enqueue(i, KILO_HOST_MSG_LOG_LINE, c.out_seq, &l, KILO_HOST_LOG_LINE_MIN_LEN + e->len)) break;
      c.out_seq++;
      c.log_seq++;
      stats_.log_sent++;
    }
  }
}

// ---- sending --------------------------------------------------------------------

bool HostServer::enqueue(uint32_t i, uint8_t type, uint8_t seq, const void* payload, uint32_t len) {
  Client& c = clients_[i];
  if (!c.open || room(i) < len + kHostOverhead) return false;
  uint8_t frame[KILO_HOST_MAX_FRAME];
  uint32_t n = host_frame_encode(type, seq, (const uint8_t*)payload, len, frame);
  if (n == 0) return false;
  uint32_t tail = (c.tx_head + c.tx_len) % kTxBytes;
  uint32_t first = kTxBytes - tail < n ? kTxBytes - tail : n;
  mem_copy(c.tx + tail, frame, first);
  mem_copy(c.tx, frame + first, n - first);
  c.tx_len += n;
  return true;
}

void HostServer::reply(uint32_t i, uint8_t seq, const void* payload, uint32_t len, uint8_t type) {
  if (!enqueue(i, type, seq, payload, len)) drop(i, "too slow: its replies do not fit");
}

void HostServer::result(uint32_t i, uint8_t seq, uint8_t request, uint8_t code, const char* text) {
  uint8_t buf[KILO_HOST_RESULT_MAX_LEN];
  buf[0] = request;
  buf[1] = code;
  uint32_t n = 0;
  while (text[n] != '\0' && n < kResultTextMax) {
    buf[2 + n] = (uint8_t)text[n];
    n++;
  }
  reply(i, seq, buf, 2 + n, KILO_HOST_MSG_RESULT);
}

void HostServer::flush(uint32_t i) {
  Client& c = clients_[i];
  while (c.open && c.tx_len > 0) {
    uint32_t seg = kTxBytes - c.tx_head < c.tx_len ? kTxBytes - c.tx_head : c.tx_len;
    int32_t w = c.conn->write(c.tx + c.tx_head, seg);
    if (w < 0) {
      drop(i, "write failed");
      return;
    }
    if (w == 0) return;
    c.tx_head = (c.tx_head + (uint32_t)w) % kTxBytes;
    c.tx_len -= (uint32_t)w;
  }
  if (c.tx_len == 0) c.tx_head = 0;
}

// ---- state ----------------------------------------------------------------------

uint8_t HostServer::state_flags(uint64_t now) const {
  const BodyState& b = link_.state();
  uint8_t s = 0;
  if (link_.connected()) s |= KILO_HOST_STATE_LINK_CONNECTED;
  if (b.alive(now, kBodyLostUs)) s |= KILO_HOST_STATE_BODY_ALIVE;
  if (b.has_hello) s |= KILO_HOST_STATE_HAS_HELLO;
  if (b.has_telemetry) s |= KILO_HOST_STATE_HAS_TELEMETRY;
  if (link_.contact()) s |= KILO_HOST_STATE_CONTACT;
  if (bridge_) s |= KILO_HOST_STATE_BRIDGE;
  if (controller_ >= 0) s |= KILO_HOST_STATE_CONTROLLED;
  return s;
}

uint16_t HostServer::telemetry_age_ms(uint64_t now) const {
  const BodyState& b = link_.state();
  if (!b.has_telemetry || now < b.telemetry_at_us) return 0xFFFFu;
  uint64_t ms = (now - b.telemetry_at_us) / 1000u;
  return ms >= 0xFFFFu ? (uint16_t)0xFFFFu : (uint16_t)ms;
}

void HostServer::fill_link_stats(kilo_host_link_stats_t& o) const {
  const LinkStats& s = link_.stats();
  o.frames_rx = s.frames_rx;
  o.frames_tx = s.frames_tx;
  o.bytes_rx = s.bytes_rx;
  o.bytes_tx = s.bytes_tx;
  o.decode_errors = s.decode_error_total();
  o.crc_errors = s.decode_errors[KILO_FRAME_BAD_CRC];
  o.heartbeats = s.heartbeats;
  o.goal_sends = s.goal_sends;
  o.commands = s.commands;
  o.retries = s.retries;
  o.timeouts = s.timeouts;
  o.nacks = s.nacks;
  o.dup_acks = s.dup_acks;
  o.tx_dropped = s.tx_dropped;
  o.tx_short = s.tx_short;
  o.reconnects = s.reconnects;
  o.disconnects = s.disconnects;
  o.goals_refused = s.goals_refused;
  o.goals_cleared = s.goals_cleared;
  o.hello_polls = s.hello_polls;
}

}  // namespace kilo
