// serial_link: the mind's end of the body link (docs/architecture.md §4,
// §5). A port of kilo/link/serial_link.py to one cooperative, non-blocking
// state machine: the Python reader and writer threads become poll(), called
// from the mind's main loop (a few ms apart; it never blocks).
//
// What poll() does, in order:
// - reads what the Serial has, decodes frames (proto/kilo_frame.h) and
//   handles them: HELLO (protocol check, reboot detection), TELEMETRY (tick
//   unwrap, epoch), EVENT (epoch, GOAL_REFUSED), ACK and query replies for
//   the one command in flight;
// - times out and retries that command: 50 ms, up to 3 retries; commands
//   retry with the same seq (the body's de-dup re-acks without re-running),
//   queries with a fresh one (answered by their data reply, never
//   de-duplicated);
// - housekeeping: sends goals set or changed since the last pass, refreshes
//   every latched goal at 5 Hz, polls HELLO_REQ once per connect and at 2 Hz
//   while the body is lost (no TELEMETRY for 300 ms), and sends a HEARTBEAT
//   after 100 ms without any frame. The refresh, HELLO_REQ and HEARTBEAT are
//   sent only while the mind's liveness stamp (mark_alive()) is < 1 s old,
//   so a stalled mind trips the body's watchdog (stop at 0.5 s, REST at
//   1.5 s, LIMP 10 s later) instead of being masked by the link.
//
// Goals and the epoch rule (§3c, §4, §5): epoch goals (SET_POSTURE,
// SET_HEAD, SET_BODY_POSE) carry the epoch current when the mind set them.
// On any epoch change seen in TELEMETRY, EVENT or HELLO, on EVENT FELL /
// WATCHDOG / LOW_BATTERY, and on a body reboot, motion goals from older
// epochs are cleared (nothing is replayed). GOAL_REFUSED STALE drops the
// refused goal.
//
// Contact: after every connect and after the body was lost, no motion goal
// is sent or refreshed and FEET and TRICK_PLAY are refused until the body
// has been heard on this connection: a HELLO that proves a reboot, or a
// TELEMETRY (after a loss only TELEMETRY's tick can tell a quiet reboot
// apart). A HELLO is a reboot when it is unsolicited, or when its identity
// (reset cause, last_alive_tick, fw_build) differs from the previous one.
//
// A body whose HELLO.proto_major is not KILO_PROTO_MAJOR is never driven:
// set_goal() and send_stream() refuse (kilo/body/client.py's
// ProtocolMismatch). HEARTBEAT, HELLO_REQ and commands still go out.
#ifndef KILO_MIND_LINK_SERIAL_LINK_H
#define KILO_MIND_LINK_SERIAL_LINK_H
#include <stdint.h>
#include "link/body_state.h"
#include "link/consts.h"
#include "link/goals.h"
#include "link/stats.h"
#include "platform.h"
#include "proto/kilo_frame.h"

namespace kilo {

// Notices from the link, called from poll() after the state they report is
// updated. Python counterpart: SerialLink.subscribe() kinds of the same
// names. Override what you need.
class LinkListener {
 public:
  virtual void on_connection(bool /*connected*/, bool /*alive*/) {}
  virtual void on_hello(const kilo_hello_t& /*hello*/, bool /*solicited*/) {}
  virtual void on_telemetry(const kilo_telemetry_t& /*telemetry*/) {}
  virtual void on_event(const kilo_event_t& /*event*/) {}
  // hello: the HELLO that proved it, or null for a tick that went backwards.
  virtual void on_reboot(const kilo_hello_t* /*hello*/, const char* /*why*/) {}
  virtual void on_goals_cleared(const char* /*reason*/, uint32_t /*count*/) {}
  virtual void on_refused(uint8_t /*goal_type*/, uint8_t /*reason*/) {}

 protected:
  ~LinkListener() = default;
};

// The one command in flight (Python's _Pending plus request()'s result).
enum class RequestState : uint8_t {
  Idle,      // none yet
  Waiting,   // sent, waiting for its ACK or reply
  Done,      // answered: status (OK, or the ACK's refusal) and, for a query, reply
  TimedOut,  // no answer after 1 + retries sends
};

struct Request {
  RequestState state;
  uint8_t type;
  uint8_t reply_type;  // query_reply(type): the data reply that answers it, 0 = an ACK does
  uint8_t seq;
  uint8_t sends;
  bool acked;          // an OK ACK of a query arrived (it still waits for the reply)
  uint8_t status;      // Done: KILO_ACK_STATUS_*
  uint8_t len;
  uint8_t reply_len;
  uint64_t sent_at_us;
  uint8_t payload[KILO_MAX_PAYLOAD];
  uint8_t reply[KILO_MAX_PAYLOAD];  // Done with a reply: its payload
};

class SerialLink {
 public:
  SerialLink(Serial& serial, Clock& clock, Log& log, const LinkConfig& cfg = LinkConfig());

  void set_listener(LinkListener* listener) { listener_ = listener; }

  // The port is open (Python _connect): ask who is there, hold motion goals
  // until the body is heard. The owner opens the device; the link calls
  // disconnect() itself when the Serial fails.
  void connect();
  void disconnect();
  bool connected() const { return state_.connected; }

  // Read, handle, retry, housekeeping. Never blocks.
  void poll();

  // The first TX seq. The constructor takes it from the clock's low bits;
  // a platform with a better source (the Pi's hardware RNG: its clock reads
  // nearly the same at every boot) sets it before connect().
  void seed_seq(uint8_t seq) { seq_ = seq; }

  // Stamped by the mind's main loop on every pass (>= 1 Hz).
  void mark_alive();
  bool mind_alive() const;

  // Latch a goal (every goal type but HEARTBEAT), sent at the next poll and
  // refreshed at 5 Hz. payload is the whole message payload; an epoch goal's
  // epoch byte is the link's to fill. False if the type is not a latched
  // goal, the length is wrong, or the body speaks another protocol.
  bool set_goal(uint8_t type, const uint8_t* payload, uint8_t len);
  bool clear_goal(uint8_t type);
  const Goal* goal(uint8_t type);

  // Write one Pi->body stream frame (FEET) now: no queue, ack, refresh or
  // retry. False when not written: not connected, the body not heard on
  // this connection yet, another protocol, or a write error.
  bool send_stream(uint8_t type, const uint8_t* payload, uint8_t len);

  // Send a command or query; poll() completes it (req().state). False
  // if one is still waiting, not connected, not a command, a wrong length,
  // or TRICK_PLAY before the body was heard on this connection.
  bool request(uint8_t type, const uint8_t* payload, uint8_t len);
  const Request& req() const { return req_; }

  const BodyState& state() const { return state_; }
  const LinkStats& stats() const { return stats_; }
  // Whether the body was heard on this connection (motion goals may go out).
  bool contact() const { return contact_; }

 private:
  void handle(uint8_t status, const kilo_frame_t& f, uint64_t now);
  void on_ack(const kilo_ack_t& a);
  void on_telemetry(const kilo_telemetry_t& t, uint64_t now);
  void on_event(const kilo_event_t& e);
  void on_hello(const kilo_hello_t& h, uint64_t now);
  void match_reply(const kilo_frame_t& f);
  void reboot(uint64_t now, int16_t epoch, const char* why);
  uint32_t clear_motion_goals(int16_t keep_epoch, const char* reason);
  void set_contact();
  void request_timer(uint64_t now);
  void housekeeping(uint64_t now);
  bool send_frame(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len, uint64_t now);
  bool send_request(uint64_t now);
  void finish(uint8_t status);
  uint8_t next_seq() { return seq_++; }
  void notify_cleared();
  void notify_reboot();

  Serial& serial_;
  Clock& clock_;
  Log& log_;
  LinkConfig cfg_;
  LinkListener* listener_ = nullptr;

  BodyState state_;
  LinkStats stats_ = {};
  Goals goals_;
  Request req_ = {};
  kilo_rx_t rx_ = {};

  uint8_t seq_;
  bool have_last_done_ = false;  // (seq, type) of the last completed command: its re-ACKs are dups
  uint8_t last_done_seq_ = 0;
  uint8_t last_done_type_ = 0;

  bool mind_ever_alive_ = false;
  uint64_t mind_alive_at_ = 0;
  uint64_t last_tx_at_ = 0;
  bool ever_tx_ = false;
  bool ever_refresh_ = false;
  uint64_t last_refresh_at_ = 0;
  bool ever_hello_poll_ = false;
  uint64_t last_hello_poll_ = 0;
  bool hello_solicited_ = false;  // a HELLO_REQ went out at hello_solicited_at_
  uint64_t hello_solicited_at_ = 0;
  bool deferred_ = false;  // the last send_frame() was held back by tx_free()
  bool ever_reboot_ = false;
  uint64_t last_reboot_at_ = 0;
  bool was_alive_ = false;
  bool ever_connected_ = false;
  bool ever_contact_ = false;
  bool hello_on_connect_ = false;
  bool contact_ = false;
  bool contact_needs_telemetry_ = false;

  // Notices raised while handling a frame, sent once its state is settled
  // (Python's _locked_then_emit).
  uint32_t cleared_count_ = 0;
  char cleared_reason_[48] = {};
  bool reboot_pending_ = false;
  const kilo_hello_t* reboot_hello_ = nullptr;
  char reboot_why_[64] = {};
};

}  // namespace kilo

#endif
