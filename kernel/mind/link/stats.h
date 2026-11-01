// stats: the link's counters. Python counterpart: LinkStats in
// kilo/body/state.py, without the counters of what is not ported (open
// failures, silence, undecodable payloads: the C codec already checks every
// known type's length, so a frame that decodes always unpacks).
#ifndef KILO_MIND_LINK_STATS_H
#define KILO_MIND_LINK_STATS_H
#include <stdint.h>

namespace kilo {

struct LinkStats {
  uint32_t frames_rx;
  uint32_t frames_tx;
  uint32_t bytes_rx;
  uint32_t bytes_tx;
  // Per kilo_frame_status (index 1..6: bad_cobs, bad_crc, bad_length,
  // bad_payload_len, unknown_type, overflow; link/names.h names them).
  uint32_t decode_errors[7];
  uint32_t heartbeats;
  uint32_t goal_sends;
  uint32_t commands;
  uint32_t retries;
  uint32_t timeouts;
  uint32_t nacks;           // ACK with status != OK
  uint32_t dup_acks;        // re-ACK of an already completed command (the body's de-dup at work)
  uint32_t unmatched_acks;
  uint32_t tx_dropped;      // frames not written: disconnected, or the device failed
  uint32_t tx_short;        // frames the device took only part of (TX buffer full); the body
                            // drops the damaged frame at the next delimiter
  uint32_t reconnects;
  uint32_t disconnects;
  uint32_t goals_refused;   // EVENT GOAL_REFUSED (Python counts them per goal:reason)
  uint32_t goals_cleared;
  uint32_t hello_polls;

  uint32_t decode_error_total() const {
    uint32_t n = 0;
    for (uint32_t e : decode_errors) n += e;
    return n;
  }
};

}  // namespace kilo

#endif
