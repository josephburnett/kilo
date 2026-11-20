// sched: the 50 Hz deadline (doc §3a). The main loop polls this instead of
// using a timer interrupt: nearly all tick work is I2C, which cannot run in
// an ISR, so an ISR could only set a flag anyway.
//
// The deadline advances by exactly one tick (KILO_TICK_MS, 20 ms) each tick (a fixed grid: a tick
// that starts late does not push the next ones later, so there is no drift).
// If the loop falls more than two ticks behind, it resynchronises instead of
// running a burst of catch-up ticks, and counts an overrun.
#ifndef KILO_CORE_SCHED_H
#define KILO_CORE_SCHED_H
#include <stdint.h>
#include "../proto/kilo_protocol.h"

#define TICK_US ((uint32_t)KILO_TICK_MS * 1000u)

struct Sched {
  uint32_t next;
  uint16_t overruns;
};

void sched_start(Sched& s, uint32_t now_us);
// True if a tick is due at now_us (micros(), wrapping). Advances the grid.
bool sched_due(Sched& s, uint32_t now_us);

#endif
