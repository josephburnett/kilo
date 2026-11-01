#include "link/body_state.h"

namespace kilo {

bool BodyState::on_telemetry(const kilo_telemetry_t& t, uint64_t now_us, int16_t* prev_epoch) {
  bool backwards = false;
  if (!has_tick) {
    tick = t.tick;
    has_tick = true;
  } else {
    uint16_t d = (uint16_t)(t.tick - tick_raw);
    if (d >= 0x8000u) {  // more than half a wrap backwards: the body restarted
      backwards = true;
      tick = t.tick;
    } else {
      tick += d;
    }
  }
  tick_raw = t.tick;
  telemetry = t;
  has_telemetry = true;
  telemetry_at_us = now_us;
  telemetry_count++;
  *prev_epoch = epoch;
  epoch = t.epoch;
  return backwards;
}

void BodyState::on_hello(const kilo_hello_t& h, uint64_t now_us) {
  hello = h;
  has_hello = true;
  hello_at_us = now_us;
  epoch = h.epoch;
  proto_ok = h.proto_major == KILO_PROTO_MAJOR ? 1 : 0;
}

void BodyState::reset_session() {
  has_tick = false;
  tick = 0;
  tick_raw = 0;
}

bool BodyState::alive(uint64_t now_us, uint32_t lost_us) const {
  return connected && has_telemetry && now_us - telemetry_at_us < lost_us;
}

}  // namespace kilo
