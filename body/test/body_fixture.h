// The body setup the host tests and harnesses share: a fresh config, an
// upright robot's tick input, and an event log drained from the queue.
//
// fixture_config() is what a cold boot leaves behind before body_init(): the
// EEPROM fake blank, the model defaults in the RAM image, nothing being
// written, and the live params loaded from that image. Callers then boot the
// body their own way (cold or warm, with or without the link).
#ifndef KILO_TEST_BODY_FIXTURE_H
#define KILO_TEST_BODY_FIXTURE_H
#include <string.h>
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

// A POSE payload (core/stream.h) into p[KILO_POSE_LEN]: legs as per leg
// (a, b) pairs LF RF RB LB (a foot's x, z in q2, or with the leg's JOINT
// flag shoulder and knee a16), head pan, tilt, tail a16 (JOINT_OFF: IDLE's).
static inline void fixture_pose(uint8_t* p, uint8_t epoch, uint16_t tick, uint8_t flags, const int16_t legs[8],
                                const int16_t head[3], int16_t pitch = 0, int16_t roll = 0,
                                uint8_t support = 0xF) {
  kilo_pose_t m;
  m.epoch = epoch;
  m.tick = tick;
  m.flags = flags;
  for (int i = 0; i < 8; i++) m.leg[i] = legs[i];
  for (int i = 0; i < 3; i++) m.head[i] = head[i];
  m.pitch = pitch;
  m.roll = roll;
  m.support = support;
  static_assert(sizeof m == KILO_POSE_LEN, "kilo_pose_t");
  memcpy(p, &m, sizeof m);
}

// STAND's legs as a joint-mode POSE's (a, b) pairs: shoulder, knee per leg.
static const int16_t FIXTURE_STAND_LEGS[8] = {30 * 16, 30 * 16, 30 * 16, 30 * 16,
                                              -30 * 16, -30 * 16, -30 * 16, -30 * 16};
#define FIXTURE_ALL_JOINT 0x0Fu  // POSE flags: every leg joint angles

// Move every queued event into log, oldest first.
static inline void fixture_drain(EventQueue& q, std::vector<Event>& log) {
  const Event* e;
  while ((e = events_peek(q)) != nullptr) {
    log.push_back(*e);
    events_pop(q);
  }
}

#endif
