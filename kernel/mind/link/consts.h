// consts: message classes and link timings the link needs beyond what the
// generated header gives (kilo_msg_kind, kilo_msg_is_goal, kilo_msg_is_query,
// kilo_msg_has_epoch). Python counterpart: kilo/link/consts.py and
// LinkConfig in kilo/link/serial_link.py.
#ifndef KILO_MIND_LINK_CONSTS_H
#define KILO_MIND_LINK_CONSTS_H
#include <stdint.h>
#include "proto/kilo_protocol.h"

namespace kilo {

// The data reply that answers a query (HELLO_REQ -> HELLO, ...), 0 for
// none. The C header lacks consts.REPLIES (protocol/gen.py emits it for
// Python only), so it is listed here; kernel/host/tests/test_names.cpp
// checks it against protocol/gen/kilo_protocol.py's QUERY_REPLY.
uint8_t query_reply(uint8_t type);

// A goal latched by the link and refreshed (every goal but HEARTBEAT).
inline bool is_latched_goal(uint8_t type) { return kilo_msg_is_goal(type) && type != KILO_MSG_HEARTBEAT; }
// consts.EPOCH_GOALS = consts.MOTION_GOALS: goals whose payload starts with
// the epoch byte (SET_POSTURE, SET_HEAD, SET_BODY_POSE). Cleared on an
// epoch change.
inline bool is_epoch_goal(uint8_t type) { return kilo_msg_is_goal(type) && kilo_msg_has_epoch(type); }
// A Pi->body stream (FEET): sent as made, never acked, refreshed or retried.
inline bool is_stream_out(uint8_t type) { return type < 0x80 && kilo_msg_kind(type) == KILO_KIND_STREAM; }
// consts.CLEARING_EVENTS: events that clear the motion goals whatever the epoch.
inline bool is_clearing_event(uint8_t code) {
  return code == KILO_EVENT_FELL || code == KILO_EVENT_WATCHDOG || code == KILO_EVENT_LOW_BATTERY;
}

// Timings (serial_link.py LinkConfig), from the spec where it has them.
struct LinkConfig {
  uint32_t heartbeat_idle_us = KILO_HEARTBEAT_IDLE_MS * 1000u;  // HEARTBEAT after this much TX silence
  uint32_t liveness_window_us = KILO_MIND_ALIVE_MS * 1000u;     // autonomous TX only while the mind is alive
  uint32_t goal_refresh_us = 1000000u / KILO_GOAL_REFRESH_HZ;   // latched goals re-sent at 5 Hz
  uint32_t ack_timeout_us = KILO_CMD_RETRY_MS * 1000u;          // a command is retried after 50 ms
  uint8_t retries = KILO_CMD_MAX_RETRIES;                       // ... up to 3 times
  uint32_t hello_poll_us = 1000000u / KILO_HELLO_POLL_HZ;       // HELLO_REQ at 2 Hz while the body is lost
  uint32_t body_lost_us = KILO_BODY_LOST_MS * 1000u;            // no TELEMETRY this long: the body is lost
  uint32_t hello_solicit_window_us = 1000000u;  // a HELLO this soon after a HELLO_REQ answers it
  uint32_t reboot_debounce_us = 3000000u;       // one reboot's HELLO and tick reset count once
};

}  // namespace kilo

#endif
