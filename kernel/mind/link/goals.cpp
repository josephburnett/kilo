#include "link/goals.h"
#include "link/consts.h"
#include "util/mem.h"

namespace kilo {

bool goal_is_motion(const Goal& g) {
  if (g.type == KILO_MSG_SET_POSTURE) {
    uint8_t posture = g.payload[offsetof(kilo_set_posture_t, posture)];
    return posture != KILO_POSTURE_REST && posture != KILO_POSTURE_LIMP;
  }
  if (g.type == KILO_MSG_SET_BALANCE) return g.payload[offsetof(kilo_set_balance_t, enable)] != 0;
  return is_epoch_goal(g.type);
}

Goal* Goals::find(uint8_t type) {
  for (Goal& g : slots_)
    if (g.used && g.type == type) return &g;
  return nullptr;
}

Goal* Goals::take(uint8_t type) {
  Goal* g = find(type);
  for (Goal* s = slots_; g == nullptr && s != slots_ + kSlots; s++)
    if (!s->used) g = s;
  if (g != nullptr) {
    mem_zero(g, sizeof *g);
    g->used = true;
    g->type = type;
  }
  return g;
}

bool Goals::remove(uint8_t type) {
  Goal* g = find(type);
  if (g == nullptr) return false;
  g->used = false;
  return true;
}

}  // namespace kilo
