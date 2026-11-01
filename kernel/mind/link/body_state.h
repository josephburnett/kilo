// body_state: the mind's view of the body as the link sees it. Python
// counterpart: BodyState in kilo/body/state.py: the part the link updates
// (latest TELEMETRY with the tick unwrapped, the epoch, the last HELLO and
// protocol check, reboots, events) and its derived values (distance median
// and approach speed, battery percent, fallen, upright, moving), which the
// behaviour runner reads. The detectors (near with hysteresis, tilt, tap...)
// are perception/'s.
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

  // ---- derived (state.py properties) ----------------------------------------------
  // Median of the last 3 echoes (0 = no echo is skipped); false before any.
  bool distance_mm(double* mm) const;
  // Closing speed (mm/s, positive = closer) over the last 8 medians; false
  // with fewer than 2 or no time between them.
  bool approach_mm_s(double* v) const;
  // The pack's charge from the resting 2S curve (est.); false without a
  // battery (NO_BATTERY flag) or a voltage.
  bool battery_pct(double* pct) const;
  bool fallen() const;   // mode FALLEN or RECOVERING
  bool moving() const;   // the gait/trick flags, or mode GAIT, TRICK, RECOVERING
  // |pitch| and |roll| under 30 deg and not fallen; -1 before any TELEMETRY.
  int upright() const;

  struct DistPoint {
    uint64_t at_us;
    double median;
  };
  uint16_t dist_[3] = {};  // the last echoes (state.py _dist, maxlen 3)
  uint8_t dist_n_ = 0;
  DistPoint dist_hist_[8] = {};  // (arrival, median) (_dist_hist, maxlen 8)
  uint8_t dist_hist_head_ = 0, dist_hist_n_ = 0;
};

// state.py battery_percent: pack mV on the resting 2S curve (est.).
double battery_percent(uint16_t mv);

}  // namespace kilo

#endif
