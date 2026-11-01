// goals: the latched desired-state goals, at most one per goal type.
// Python counterpart: Goal, SerialLink._goals (a dict by name) and
// _is_motion in kilo/link/serial_link.py.
//
// The payload is stored whole; for an epoch goal its first byte is
// overwritten with the goal's epoch when it is packed for sending, so a goal
// keeps the epoch current when the mind set it and can never outlive an
// epoch bump it did not see.
#ifndef KILO_MIND_LINK_GOALS_H
#define KILO_MIND_LINK_GOALS_H
#include <stdint.h>
#include "link/body_state.h"
#include "proto/kilo_protocol.h"

namespace kilo {

struct Goal {
  bool used;
  bool dirty;      // send at the next housekeeping, refresh or not
  uint8_t type;
  uint8_t len;
  int16_t epoch;   // epoch goals: the epoch they were set under (kNoEpoch: adopt the first one seen)
  uint32_t sends;
  uint8_t payload[KILO_MAX_PAYLOAD];
};

// A goal that moves the body: SET_POSTURE other than REST/LIMP, a
// SET_BALANCE enable, any other epoch goal. Held back while the body has
// not been heard on this connection.
bool goal_is_motion(const Goal& g);

class Goals {
 public:
  // Latched goal types: SET_POSTURE, SET_HEAD, SET_BODY_POSE, SET_BALANCE,
  // SET_EYES, BEEP.
  static constexpr uint32_t kSlots = 6;

  Goal* find(uint8_t type);
  // The slot for type, zeroed and marked used (an existing goal of that
  // type is replaced); null when every slot holds another type.
  Goal* take(uint8_t type);
  bool remove(uint8_t type);
  Goal* begin() { return slots_; }
  Goal* end() { return slots_ + kSlots; }

 private:
  Goal slots_[kSlots] = {};
};

}  // namespace kilo

#endif
