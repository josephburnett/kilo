// client: the goal calls of kilo/body/client.py's BodyClient that the
// remote and the behaviour layer share: posture(), set_balance(), look().
// Each latches a goal on the link (refreshed at 5 Hz, epoch-stamped by
// the link) and returns false where Python raises (an unknown name, a value
// outside the wire range, a body with another protocol, a full goal table).
#ifndef KILO_MIND_REMOTE_CLIENT_H
#define KILO_MIND_REMOTE_CLIENT_H
#include <stdint.h>
#include "link/serial_link.h"
#include "planner/walker.h"

namespace kilo {

// client.py POSTURE_CHOICES -> SET_POSTURE code; "balance" is STAND with the flag. -1: unknown.
int posture_code(const char* name);

// BodyClient.posture(name, balance): balance -1 is None ("balance" turns
// the flag on, every other name off). A posture takes the legs from a walk
// in progress: the walker's stream is aborted first.
bool client_posture(SerialLink& link, planner::Walker* walker, const char* name, int8_t balance = -1,
                    uint8_t transition_ticks = 0);
// BodyClient.set_balance(enable, kp): the gains' other values are the
// body's defaults (Kd 30 ms, dead band 2 deg, clamp 15 deg). kp 0..0.996.
bool client_set_balance(SerialLink& link, bool enable, double kp = KILO_BALANCE_KP_Q8 / 256.0);
// The SET_BALANCE goal is latched with enable set.
bool client_balance_enabled(SerialLink& link);
// BodyClient.look(pan, tilt): SET_HEAD HOLD, the tail not held, 120 deg/s.
bool client_look(SerialLink& link, double pan_deg, double tilt_deg);

}  // namespace kilo

#endif
