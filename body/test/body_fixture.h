// The body setup the host tests and harnesses share: a fresh config, an
// upright robot's tick input, and an event log drained from the queue.
//
// fixture_config() is what a cold boot leaves behind before body_init(): the
// EEPROM fake blank, the model defaults in the RAM image, nothing being
// written, and the live params loaded from that image. Callers then boot the
// body their own way (cold or warm, with or without the link).
#ifndef KILO_TEST_BODY_FIXTURE_H
#define KILO_TEST_BODY_FIXTURE_H
#include <vector>
#include "../src/core/body.h"
#include "../src/core/params.h"
#include "fake_eeprom.h"

static inline void fixture_config(Config& cfg, uint8_t state = CFG_MIGRATED) {
  g_ee.reset();
  config_defaults(cfg.img);
  cfg.state = (ConfigState)state;
  cfg.reason = CFG_REASON_NONE;
  cfg.newest = 0xFF;
  cfg.writing = 0;
  cfg.saved = 0;
  params_load(cfg.img);
}

// fixture_config(), then a cold boot with a working IMU (or a warm one from
// the given canonical angles).
static inline void fixture_body(Body& b, Config& cfg, uint8_t state = CFG_MIGRATED,
                                const int16_t* warm = nullptr) {
  fixture_config(cfg, state);
  body_init(b, &cfg, warm, true);
}

// The tick input of a robot standing still and level: a fresh IMU sample
// with gravity straight down (gz = 1 g in the MPU's 16384 counts per g).
static inline TickIn fixture_upright() {
  TickIn in = TickIn();
  in.imu = IMU_FRESH;
  in.att.gz = 16384;
  return in;
}

// Move every queued event into log, oldest first.
static inline void fixture_drain(EventQueue& q, std::vector<Event>& log) {
  const Event* e;
  while ((e = events_peek(q)) != nullptr) {
    log.push_back(*e);
    events_pop(q);
  }
}

#endif
