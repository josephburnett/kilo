// body: what the behaviours call on the body, and where the runner gets
// its snapshot. Python counterpart: the duck-typed body of
// kilo/behaviour/library.py, implemented over the real link by
// kilo/behaviour/adapter.py (BodyAdapter); the Mind implements it over its
// SerialLink and TrickPlayer (mind.cpp), the tests with a recording fake.
//
// The defaults refuse (no pure virtuals: the freestanding core does not
// link __cxa_pure_virtual).
#ifndef KILO_MIND_BEHAVIOUR_BODY_H
#define KILO_MIND_BEHAVIOUR_BODY_H
#include <stdint.h>
#include "behaviour/tree.h"

namespace kilo {
namespace behaviour {

class BodyIface {
 public:
  // runner hooks (adapter.snapshot(), on_tick())
  virtual void snapshot(Snapshot& out) { out = Snapshot(); }
  virtual void on_tick() {}
  // Latch a posture (rest, sit, stand, balance = stand with the flag);
  // balance -1 is Python's None (the name decides). False: refused.
  virtual bool posture(const char* /*name*/, int8_t /*balance*/) { return false; }
  virtual bool set_balance(bool /*enable*/) { return false; }
  virtual bool look(double /*pan_deg*/, double /*tilt_deg*/) { return false; }
  // Start a library trick: a handle > 0, or 0 when it cannot start.
  virtual uint32_t play(const char* /*trick*/) { return 0; }
  // Running until the trick is done: Success on TRICK_DONE DONE, Failure otherwise.
  virtual Status play_status(uint32_t /*handle*/) { return Status::Failure; }
  // Abandon it: stopped on the body if still playing (play() cancelled).
  virtual void play_cancel(uint32_t /*handle*/) {}
  // [(midi note, ms), ...]
  virtual void beep(const uint16_t (*/*notes*/)[2], int /*n*/) {}

 protected:
  ~BodyIface() = default;
};

}  // namespace behaviour
}  // namespace kilo

#endif
