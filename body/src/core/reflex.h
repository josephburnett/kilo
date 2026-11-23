// reflex: the fall reflex's pure parts (doc §3g "Fall"). P1.
//
// The detector and the attitude tests are pure functions of the attitude,
// host-tested on their own; body.cpp runs the state machine around them
// (owner, epoch, events). The body does not get up by itself: getting up
// is the mind's (a POSE stream with FALL_OK; decided 2026-11-22, the body's
// built-in get-up `rc` is gone).
//
//   UPRIGHT -> FALLEN   |roll| or |pitch| > FALL_DEG (75 deg) on FALL_SAMPLES
//                       (6) consecutive fresh IMU samples (120 ms at 50 Hz);
//                       one sample never trips it (P0 finding 8: a handled
//                       sample read -70 deg). Epoch bump (a stream ends),
//                       REFLEX holds the pose, EVENT FELL (+-1 roll / +-2
//                       pitch, the angle's sign), a chirp. Then waits until
//                       still: every body rate < 20 deg/s on
//                       FALL_STILL_TICKS (10) samples.
//   FALLEN  -> LYING    still (or the IMU failed): REST, EVENT LYING (the
//                       FELL arg), LIMP after FALL_LIMP_TICKS (2 s), then
//                       the reflex lets go.
//
// Detection is off in LIMP and CALIB, under IMU_FAULT, while a FALL_OK
// stream plays and while another reflex holds the joints (warm-boot hold,
// watchdog REST, low battery). A more important reflex taking the joints
// ends the fall reflex silently. Goals while it holds the joints: SET_POSTURE
// LIMP ends it (the Pi's emergency stop), a POSE with FALL_OK takes the
// joints from it (the mind's get-up); every other motion goal is refused
// FALLEN.
#ifndef KILO_CORE_REFLEX_H
#define KILO_CORE_REFLEX_H
#include <stdint.h>
#include "attitude.h"

#define FALL_DEG 75           // tilt threshold, deg
#define FALL_SAMPLES 6        // consecutive samples over it (120 ms)
#define FALL_STILL_DPS 20     // "still": every body rate below this
#define FALL_STILL_TICKS 10
#define FALL_LIMP_TICKS 100   // LYING: REST, then LIMP 2 s later

enum FallState : uint8_t { FS_UPRIGHT = 0, FS_FALLEN = 1, FS_LYING = 2 };

struct Fall {
  uint8_t state;   // FallState
  uint8_t over;    // consecutive samples over FALL_DEG (UPRIGHT)
  uint8_t still;   // consecutive still samples (FALLEN)
  int8_t axis;     // the FELL arg: +-1 roll, +-2 pitch
  uint16_t timer;  // LYING's LIMP timer
};

// One fresh sample through the detector. Returns 0, or the FELL arg once
// FALL_SAMPLES consecutive samples were over FALL_DEG: a pitch fall (+-2)
// when the body's long axis is within 15 deg of vertical (|pitch| folded
// about 90 deg, since the legacy reflection reports pitch past 90 deg
// upside down), else a roll fall (+-1, the sign of roll: on a side or on
// the back). A sample under the threshold restarts the count.
int8_t fall_detect(uint8_t& over, const Attitude& a);
// Every body rate under FALL_STILL_DPS.
bool fall_still(const Attitude& a);

#endif
