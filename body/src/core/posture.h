// posture: the built-in postures (PROGMEM), from legacy/InstinctNybble.h.
//
// Ids are the SET_POSTURE values of the protocol (Appendix E). LIMP has no
// angles (every channel full-off). STANCE (P2) has no table either: its
// legs come from the IK (core/stream.h: all four feet under the shoulders at
// STANCE's height, 0.80 (L1 + L2) by default, with SET_BODY_POSE and the
// geometric balance on top; the arbiter reads them from stream_joints) and
// its head and tail are STAND's (all 0, which posture_angle returns for it).
// It is IDLE's posture while a stream of feet plays, and the one such a
// stream hands the legs back to. "balance" in
// InstinctNybble.h has exactly the angles of "up" (STAND); on the new body
// it is STAND with the balance flag set.
#ifndef KILO_CORE_POSTURE_H
#define KILO_CORE_POSTURE_H
#include <stdint.h>

enum PostureId : uint8_t {
  POSTURE_LIMP = 0,
  POSTURE_REST = 1,
  POSTURE_SIT = 2,
  POSTURE_STAND = 3,
  POSTURE_STANCE = 4,  // P2: the IK posture (core/stream.h)
  POSTURE_CALIB = 5,
  POSTURE_ZERO = 6,
  POSTURE_COUNT = 7,
  POSTURE_HOLD = 0xFE,  // internal: hold the snapshot (warm boot, fall)
};


// Angle of canonical joint k (0..10) in posture id, a16.
int16_t posture_angle(uint8_t id, uint8_t k);

// Expected body attitude while in the posture (the balance reference), a16:
// the legacy header bytes verbatim. Kilo's attitude IS the legacy ypr
// (doc §3f "Sign conventions", measured 2026-10-10): nose DOWN = +pitch,
// RIGHT side up = +roll, so "sit" (nose up) expects pitch -30 deg, exactly
// as its legacy header says. The gain signs live in balance.h.
int16_t posture_pitch(uint8_t id);
int16_t posture_roll(uint8_t id);

#endif
