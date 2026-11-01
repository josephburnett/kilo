// body_state: the mind's view of the body as the link sees it. Python
// counterpart: BodyState in kilo/body/state.py, the part the link updates
// (latest TELEMETRY with the tick unwrapped, the epoch, the last HELLO and
// protocol check, reboots, events). The derived percepts (distance median,
// battery percent, upright) belong to perception (docs/fast-boot.md P3).
#ifndef KILO_MIND_LINK_BODY_STATE_H
#define KILO_MIND_LINK_BODY_STATE_H
#include <stdint.h>
#include "proto/kilo_protocol.h"

namespace kilo {

constexpr int16_t kNoEpoch = -1;  // Python's epoch None: no HELLO, TELEMETRY or EVENT yet

struct BodyState {
  bool connected = false;
  uint64_t last_rx_us = 0;

  bool has_telemetry = false;
  kilo_telemetry_t telemetry{};
  uint64_t telemetry_at_us = 0;
  uint32_t telemetry_count = 0;  // TELEMETRY received so far
  bool has_tick = false;
  uint32_t tick = 0;  // TELEMETRY.tick unwrapped (u16 on the wire)
  uint16_t tick_raw = 0;

  int16_t epoch = kNoEpoch;

  bool has_hello = false;
  kilo_hello_t hello{};
  uint64_t hello_at_us = 0;
  int8_t proto_ok = -1;  // -1 no HELLO yet, else HELLO.proto_major == KILO_PROTO_MAJOR
  uint32_t reboots = 0;

  bool has_event = false;
  kilo_event_t last_event{};
  uint32_t event_count = 0;

  // Record a TELEMETRY. Returns true when the tick went more than half a
  // wrap backwards (the body restarted); *prev_epoch gets the epoch before.
  bool on_telemetry(const kilo_telemetry_t& t, uint64_t now_us, int16_t* prev_epoch);
  void on_hello(const kilo_hello_t& h, uint64_t now_us);
  // Forget the tick history after a body reboot.
  void reset_session();
  // Connected, and a TELEMETRY less than lost_us old.
  bool alive(uint64_t now_us, uint32_t lost_us) const;
};

}  // namespace kilo

#endif
