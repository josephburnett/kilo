#include "link/serial_link.h"
#include "link/names.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/mem.h"

namespace kilo {

namespace {

// Bytes taken from the Serial per read, and reads per poll: a cooperative
// task hands the CPU back even when bytes keep coming (250000 baud is
// 25 B/ms, far below what one poll drains).
constexpr uint32_t kReadChunk = 256;
constexpr uint32_t kReadsPerPoll = 16;

bool same_identity(const kilo_hello_t& a, const kilo_hello_t& b) {
  return a.reset_cause == b.reset_cause && a.last_alive_tick == b.last_alive_tick && a.fw_build == b.fw_build;
}

}  // namespace

SerialLink::SerialLink(Serial& serial, Clock& clock, Log& log, const LinkConfig& cfg)
    : serial_(serial), clock_(clock), log_(log), cfg_(cfg) {
  kilo_rx_init(&rx_);
  // The TX seq starts somewhere else on every start (Python: random), so a
  // restarted mind does not repeat the (seq, type) of its previous life
  // inside the body's 500 ms de-dup window. The clock's low bits differ
  // from boot to boot by enough.
  uint64_t t = clock_.now_us();
  seq_ = (uint8_t)(t ^ (t >> 8) ^ (t >> 16));
}

// ---- connection --------------------------------------------------------------

void SerialLink::connect() {
  uint64_t now = clock_.now_us();
  kilo_rx_init(&rx_);
  state_.connected = true;
  if (ever_connected_) stats_.reconnects++;
  ever_connected_ = true;
  hello_solicited_ = true;
  hello_solicited_at_ = now;
  // A (re)opened port may have reset the body (DTR on a USB adapter): no
  // motion goal until it is heard on this connection.
  contact_ = false;
  contact_needs_telemetry_ = ever_contact_;
  for (Goal& g : goals_)
    if (g.used) g.dirty = true;
  hello_on_connect_ = true;  // ask who is there (sent by housekeeping, liveness-gated)
  logf(log_, LogLevel::Info, "link connected");
  if (listener_ != nullptr) listener_->on_connection(true, false);
}

void SerialLink::disconnect() {
  if (!state_.connected) return;
  state_.connected = false;
  stats_.disconnects++;
  logf(log_, LogLevel::Info, "link disconnected");
  if (listener_ != nullptr) listener_->on_connection(false, false);
}

// ---- mind liveness -----------------------------------------------------------

void SerialLink::mark_alive() {
  mind_alive_at_ = clock_.now_us();
  mind_ever_alive_ = true;
}

bool SerialLink::mind_alive() const {
  return mind_ever_alive_ && clock_.now_us() - mind_alive_at_ < cfg_.liveness_window_us;
}

// ---- the loop ----------------------------------------------------------------

void SerialLink::poll() {
  uint64_t now = clock_.now_us();
  for (uint32_t i = 0; i < kReadsPerPoll && state_.connected; i++) {
    uint8_t buf[kReadChunk];
    int32_t n = serial_.read(buf, sizeof buf);
    if (n < 0) {
      logf(log_, LogLevel::Warn, "read failed");
      disconnect();
      break;
    }
    if (n == 0) break;
    stats_.bytes_rx += (uint32_t)n;
    state_.last_rx_us = now;
    for (int32_t k = 0; k < n; k++) {
      kilo_frame_t f;
      uint8_t st = kilo_rx_push(&rx_, buf[k], &f);
      if (st != KILO_FRAME_NONE) handle(st, f, now);  // f points into rx_: handle it before the next push
    }
  }
  request_timer(now);
  housekeeping(now);
}

void SerialLink::housekeeping(uint64_t now) {
  bool alive = state_.alive(now, cfg_.body_lost_us);
  if (alive != was_alive_) {
    was_alive_ = alive;
    if (!alive) {
      // it may be rebooting: no motion goal until it is heard again
      contact_ = false;
      contact_needs_telemetry_ = true;
    }
    logf(log_, LogLevel::Info, "body %s", alive ? "alive" : "lost (no telemetry for 300 ms)");
    if (listener_ != nullptr) listener_->on_connection(state_.connected, alive);
  }
  if (!state_.connected) return;
  bool mind_ok = mind_ever_alive_ && now - mind_alive_at_ < cfg_.liveness_window_us;

  // Goals set or changed by the mind go out now; all of them at 5 Hz.
  bool refresh = mind_ok && (!ever_refresh_ || now - last_refresh_at_ >= cfg_.goal_refresh_us);
  if (refresh) {
    ever_refresh_ = true;
    last_refresh_at_ = now;
  }
  for (Goal& g : goals_) {
    if (!g.used || !(g.dirty || refresh)) continue;
    if (!contact_ && goal_is_motion(g)) continue;  // stays dirty: sent once the body is heard
    bool epoch_goal = is_epoch_goal(g.type);
    if (epoch_goal && g.epoch == kNoEpoch) {
      if (state_.epoch == kNoEpoch) continue;  // wait for the first epoch from HELLO or TELEMETRY
      g.epoch = state_.epoch;
    }
    uint8_t payload[KILO_MAX_PAYLOAD];
    mem_copy(payload, g.payload, g.len);
    if (epoch_goal) payload[0] = (uint8_t)g.epoch;
    g.dirty = false;
    g.sends++;
    if (send_frame(g.type, next_seq(), payload, g.len, now)) stats_.goal_sends++;
    else if (deferred_) g.dirty = true;  // no room: it goes at the next poll
  }

  if (!mind_ok) return;  // a stalled mind sends nothing on its own: let the body's watchdog trip
  if (hello_on_connect_ || (!alive && (!ever_hello_poll_ || now - last_hello_poll_ >= cfg_.hello_poll_us))) {
    hello_on_connect_ = false;
    ever_hello_poll_ = true;
    last_hello_poll_ = now;
    hello_solicited_ = true;
    hello_solicited_at_ = now;
    stats_.hello_polls++;
    send_frame(KILO_MSG_HELLO_REQ, next_seq(), nullptr, 0, now);
    if (deferred_) {  // no room: ask again at the next poll
      hello_on_connect_ = true;
      stats_.hello_polls--;
    }
  }
  if (!ever_tx_ || now - last_tx_at_ >= cfg_.heartbeat_idle_us) {
    if (send_frame(KILO_MSG_HEARTBEAT, next_seq(), nullptr, 0, now)) stats_.heartbeats++;
  }
}

bool SerialLink::send_frame(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len, uint64_t now) {
  if (!state_.connected) {
    stats_.tx_dropped++;
    return false;
  }
  deferred_ = false;
  uint8_t wire[KILO_MAX_WIRE];
  uint8_t n = kilo_frame_encode(type, seq, payload, len, wire);
  if (n == 0) return false;  // len > KILO_MAX_PAYLOAD: callers check lengths first
  if (serial_.tx_free() < n) {
    // Whole frames or nothing: a cut frame costs the body a CRC error and
    // feeds no watchdog. Housekeeping retries at the next poll.
    deferred_ = true;
    stats_.tx_deferred++;
    return false;
  }
  int32_t w = serial_.write(wire, n);
  if (w < 0) {
    logf(log_, LogLevel::Warn, "write failed");
    stats_.tx_dropped++;
    disconnect();
    return false;
  }
  if (w < n) {
    // A partial frame feeds no watchdog: the body drops it as bad CRC at
    // the next delimiter. So it does not count as traffic.
    stats_.tx_short++;
    return false;
  }
  last_tx_at_ = now;
  ever_tx_ = true;
  stats_.frames_tx++;
  stats_.bytes_tx += n;
  return true;
}

// ---- goals and streams -------------------------------------------------------

bool SerialLink::set_goal(uint8_t type, const uint8_t* payload, uint8_t len) {
  if (!is_latched_goal(type) || kilo_payload_len_check(type, payload, len) != KILO_LEN_OK) return false;
  if (state_.proto_ok == 0) return false;  // refusing to drive (logged with the HELLO)
  Goal* g = goals_.take(type);
  if (g == nullptr) return false;
  mem_copy(g->payload, payload, len);
  g->len = len;
  g->dirty = true;
  g->epoch = is_epoch_goal(type) ? state_.epoch : kNoEpoch;
  return true;
}

bool SerialLink::clear_goal(uint8_t type) { return goals_.remove(type); }

const Goal* SerialLink::goal(uint8_t type) { return goals_.find(type); }

bool SerialLink::send_stream(uint8_t type, const uint8_t* payload, uint8_t len) {
  if (!is_stream_out(type) || !contact_ || state_.proto_ok == 0) return false;
  if (kilo_payload_len_check(type, payload, len) != KILO_LEN_OK) return false;
  return send_frame(type, next_seq(), payload, len, clock_.now_us());
}

uint32_t SerialLink::clear_motion_goals(int16_t keep_epoch, const char* reason) {
  // keep_epoch kNoEpoch: every motion goal goes (a reboot). Else the goals
  // of other epochs go, and goals still waiting for a first epoch adopt it.
  char names[96];
  size_t used = 0;
  uint32_t n = 0;
  names[0] = '\0';
  for (Goal& g : goals_) {
    if (!g.used || !is_epoch_goal(g.type)) continue;
    if (keep_epoch == kNoEpoch || (g.epoch != kNoEpoch && g.epoch != keep_epoch)) {
      used += format(names + used, sizeof names - used, "%s%s", n ? "," : "", msg_name(g.type));
      g.used = false;
      n++;
    } else if (g.epoch == kNoEpoch) {
      g.epoch = keep_epoch;
    }
  }
  if (n == 0) return 0;
  stats_.goals_cleared += n;
  logf(log_, LogLevel::Info, "cleared goals %s (%s)", names, reason);
  cleared_count_ += n;
  format(cleared_reason_, sizeof cleared_reason_, "%s", reason);
  return n;
}

void SerialLink::set_contact() {
  if (contact_) return;
  contact_ = true;
  contact_needs_telemetry_ = false;
  ever_contact_ = true;
  for (Goal& g : goals_)
    if (g.used && goal_is_motion(g)) g.dirty = true;
}

void SerialLink::notify_cleared() {
  if (cleared_count_ == 0) return;
  uint32_t n = cleared_count_;
  cleared_count_ = 0;
  if (listener_ != nullptr) listener_->on_goals_cleared(cleared_reason_, n);
}

void SerialLink::notify_reboot() {
  if (!reboot_pending_) return;
  reboot_pending_ = false;
  if (listener_ != nullptr) listener_->on_reboot(reboot_hello_, reboot_why_);
}

// ---- commands ----------------------------------------------------------------

bool SerialLink::request(uint8_t type, const uint8_t* payload, uint8_t len) {
  if (req_.state == RequestState::Waiting) return false;  // one at a time
  if (kilo_msg_kind(type) != KILO_KIND_COMMAND || kilo_payload_len_check(type, payload, len) != KILO_LEN_OK)
    return false;
  if (!state_.connected) return false;
  if (type == KILO_MSG_TRICK_PLAY && !contact_) return false;  // the body has not been heard on this connection
  mem_zero(&req_, sizeof req_);
  req_.state = RequestState::Waiting;
  req_.type = type;
  req_.reply_type = query_reply(type);
  req_.len = len;
  mem_copy(req_.payload, payload, len);
  req_.seq = next_seq();
  stats_.commands++;
  send_request(clock_.now_us());
  return true;
}

bool SerialLink::send_request(uint64_t now) {
  req_.sends++;
  req_.sent_at_us = now;
  if (req_.type == KILO_MSG_HELLO_REQ) {
    hello_solicited_ = true;
    hello_solicited_at_ = now;
  }
  return send_frame(req_.type, req_.seq, req_.payload, req_.len, now);
}

void SerialLink::request_timer(uint64_t now) {
  if (req_.state != RequestState::Waiting || now - req_.sent_at_us < cfg_.ack_timeout_us) return;
  if (req_.sends <= cfg_.retries) {
    stats_.retries++;
    logf(log_, LogLevel::Debug, "%s seq=%u: no answer, retry %u", msg_name(req_.type), req_.seq, req_.sends);
    // Queries are never de-duplicated by the body: a fresh seq. Commands
    // keep theirs, so the body re-acks without running them twice.
    if (req_.reply_type != 0 || kilo_msg_is_query(req_.type)) req_.seq = next_seq();
    send_request(now);
    return;
  }
  req_.state = RequestState::TimedOut;
  stats_.timeouts++;
  logf(log_, LogLevel::Warn, "%s: no answer after %u tries", msg_name(req_.type), req_.sends);
}

void SerialLink::finish(uint8_t status) {
  req_.state = RequestState::Done;
  req_.status = status;
  have_last_done_ = true;
  last_done_seq_ = req_.seq;
  last_done_type_ = req_.type;
  if (status != KILO_ACK_STATUS_OK) stats_.nacks++;
}

// ---- received frames ---------------------------------------------------------

void SerialLink::handle(uint8_t status, const kilo_frame_t& f, uint64_t now) {
  if (status != KILO_FRAME_OK) {
    if (status < sizeof stats_.decode_errors / sizeof stats_.decode_errors[0]) stats_.decode_errors[status]++;
    return;
  }
  stats_.frames_rx++;
  // kilo_frame_decode checked the length of every known type: the payload
  // fills its struct exactly.
  switch (f.type) {
    case KILO_MSG_HELLO: {
      kilo_hello_t h;
      mem_copy(&h, f.payload, sizeof h);
      on_hello(h, now);
      break;
    }
    case KILO_MSG_TELEMETRY: {
      kilo_telemetry_t t;
      mem_copy(&t, f.payload, sizeof t);
      on_telemetry(t, now);
      break;
    }
    case KILO_MSG_EVENT: {
      kilo_event_t e;
      mem_copy(&e, f.payload, sizeof e);
      on_event(e);
      break;
    }
    case KILO_MSG_ACK: {
      kilo_ack_t a;
      mem_copy(&a, f.payload, sizeof a);
      on_ack(a);
      break;
    }
    default:
      break;
  }
  match_reply(f);
}

void SerialLink::match_reply(const kilo_frame_t& f) {
  if (req_.state != RequestState::Waiting || req_.reply_type != f.type) return;
  mem_copy(req_.reply, f.payload, f.len);
  req_.reply_len = f.len;
  finish(KILO_ACK_STATUS_OK);
}

void SerialLink::on_ack(const kilo_ack_t& a) {
  if (req_.state == RequestState::Waiting && a.seq == req_.seq && a.type == req_.type) {
    req_.acked = true;
    // An OK ACK of a query: keep waiting for its data reply.
    if (a.status != KILO_ACK_STATUS_OK || req_.reply_type == 0) finish(a.status);
    return;
  }
  if (have_last_done_ && a.seq == last_done_seq_ && a.type == last_done_type_)
    stats_.dup_acks++;
  else
    stats_.unmatched_acks++;
}

void SerialLink::on_telemetry(const kilo_telemetry_t& t, uint64_t now) {
  int16_t prev_epoch;
  bool backwards = state_.on_telemetry(t, now, &prev_epoch);
  // The reboot or epoch check comes before the contact that re-enables
  // motion goals.
  if (backwards && state_.has_hello) {
    reboot(now, t.epoch, "tick went backwards");
  } else if (prev_epoch != kNoEpoch && t.epoch != prev_epoch) {
    char reason[24];
    format(reason, sizeof reason, "epoch %d->%u", prev_epoch, t.epoch);
    clear_motion_goals(t.epoch, reason);
  } else if (prev_epoch == kNoEpoch) {
    clear_motion_goals(t.epoch, "first epoch");
  }
  set_contact();
  notify_reboot();
  notify_cleared();
  if (listener_ != nullptr) listener_->on_telemetry(t);
}

void SerialLink::on_event(const kilo_event_t& e) {
  state_.last_event = e;
  state_.has_event = true;
  state_.event_count++;
  const char* ev = event_name(e.code);
  if (e.code == KILO_EVENT_IR_KEY)
    logf(log_, LogLevel::Info, "event IR_KEY 0x%04x epoch=%u", (unsigned)(uint16_t)e.arg, e.epoch);
  else
    logf(log_, LogLevel::Info, "event %s arg=%d epoch=%u", ev, e.arg, e.epoch);
  int16_t prev_epoch = state_.epoch;
  state_.epoch = e.epoch;
  bool clearing = is_clearing_event(e.code);
  if (clearing || prev_epoch != e.epoch) {
    // every EVENT carries the post-bump epoch: goals from older epochs go
    char reason[48];
    if (clearing)
      format(reason, sizeof reason, "%s", ev);
    else if (prev_epoch == kNoEpoch)
      format(reason, sizeof reason, "epoch none->%u (%s)", e.epoch, ev);
    else
      format(reason, sizeof reason, "epoch %d->%u (%s)", prev_epoch, e.epoch, ev);
    clear_motion_goals(e.epoch, reason);
    notify_cleared();
  }
  if (e.code == KILO_EVENT_GOAL_REFUSED) {
    uint16_t arg = (uint16_t)e.arg;
    uint8_t gtype = (uint8_t)(arg >> 8), reason = (uint8_t)(arg & 0xFFu);
    stats_.goals_refused++;
    if (reason == KILO_GOAL_REFUSED_REASON_STALE) {
      Goal* g = goals_.find(gtype);
      bool drop = g != nullptr && (g->epoch == kNoEpoch || g->epoch != e.epoch);
      if (drop) goals_.remove(gtype);
      logf(log_, LogLevel::Warn, "goal %s refused as STALE (epoch now %u)%s", msg_name(gtype), e.epoch,
           drop ? "; dropped" : "");
    } else {
      // GOAL_REFUSED reasons reuse the ACK status codes
      logf(log_, LogLevel::Warn, "goal %s refused: %s", msg_name(gtype), ack_status_name(reason));
    }
    if (listener_ != nullptr) listener_->on_refused(gtype, reason);
  }
  if (listener_ != nullptr) listener_->on_event(e);
}

void SerialLink::on_hello(const kilo_hello_t& h, uint64_t now) {
  bool solicited = (hello_solicited_ && now - hello_solicited_at_ < cfg_.hello_solicit_window_us) ||
                   (req_.state == RequestState::Waiting && req_.type == KILO_MSG_HELLO_REQ);
  bool first = !state_.has_hello;
  bool changed = !first && !same_identity(state_.hello, h);
  state_.on_hello(h, now);
  // Clear before contact: no refresh of an old goal can go out between this
  // HELLO and the clear.
  bool rebooted = !first && (!solicited || changed);
  if (rebooted) {
    char why[64];
    format(why, sizeof why, "%s HELLO (reset cause %s)", solicited ? "new" : "unsolicited",
           reset_cause_name(h.reset_cause));
    reboot(now, h.epoch, why);
    reboot_hello_ = &state_.hello;
  } else {
    clear_motion_goals(h.epoch, "hello");
  }
  if (rebooted || !contact_needs_telemetry_) set_contact();
  logf(log_, LogLevel::Info, "HELLO proto %u.%u build %08x reset %s config %s caps 0x%x epoch %u%s",
       h.proto_major, h.proto_minor, (unsigned)h.fw_build, reset_cause_name(h.reset_cause),
       config_state_name(h.config_state), (unsigned)h.caps, h.epoch, solicited ? "" : " (unsolicited)");
  if (state_.proto_ok == 0)
    logf(log_, LogLevel::Error, "protocol major %u != expected %u: refusing to drive; flash the body",
         h.proto_major, KILO_PROTO_MAJOR);
  notify_reboot();
  notify_cleared();
  if (listener_ != nullptr) listener_->on_hello(state_.hello, solicited);
}

void SerialLink::reboot(uint64_t now, int16_t epoch, const char* why) {
  // Motion goals are cleared every time, first (a crash loop must not keep
  // goals set after its first reboot); only the count and the notice are
  // debounced (one reboot's HELLO and tick reset).
  state_.reset_session();
  state_.epoch = epoch;
  clear_motion_goals(kNoEpoch, "reboot");
  if (ever_reboot_ && now - last_reboot_at_ < cfg_.reboot_debounce_us) return;
  ever_reboot_ = true;
  last_reboot_at_ = now;
  state_.reboots++;
  logf(log_, LogLevel::Warn, "body rebooted: %s", why);
  reboot_pending_ = true;
  reboot_hello_ = nullptr;
  format(reboot_why_, sizeof reboot_why_, "%s", why);
}

}  // namespace kilo
