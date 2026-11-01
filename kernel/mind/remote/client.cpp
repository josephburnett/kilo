#include "remote/client.h"
#include "util/dmath.h"
#include "util/str.h"

namespace kilo {

int posture_code(const char* name) {
  static const struct {
    const char* name;
    uint8_t code;
  } kNames[] = {{"limp", KILO_POSTURE_LIMP},     {"rest", KILO_POSTURE_REST},   {"stand", KILO_POSTURE_STAND},
                {"sit", KILO_POSTURE_SIT},       {"balance", KILO_POSTURE_STAND}, {"stance", KILO_POSTURE_STANCE},
                {"calib", KILO_POSTURE_CALIB},   {"zero", KILO_POSTURE_ZERO}};
  for (const auto& n : kNames)
    if (str_eq(name, n.name)) return n.code;
  return -1;
}

bool client_posture(SerialLink& link, planner::Walker* walker, const char* name, int8_t balance,
                    uint8_t transition_ticks) {
  int code = posture_code(name);
  if (code < 0 || link.state().proto_ok == 0) return false;
  bool flag = balance == 1;
  if (str_eq(name, "balance") && balance < 0) flag = true;
  // a posture takes the legs from the stream: end it first, so no FEET re-enters GAIT under it
  if (walker != nullptr && walker->active()) walker->abort();
  kilo_set_posture_t g = {0, (uint8_t)code, transition_ticks, (uint8_t)(flag ? KILO_POSTURE_FLAGS_BALANCE : 0)};
  return link.set_goal(KILO_MSG_SET_POSTURE, (const uint8_t*)&g, sizeof g);
}

bool client_set_balance(SerialLink& link, bool enable, double kp) {
  double q = d_round(kp * 256.0);
  if (!(q >= 0 && q <= 255) || link.state().proto_ok == 0) return false;
  kilo_set_balance_t b = {(uint8_t)(enable ? 1 : 0), (uint8_t)q, KILO_BALANCE_KD_MS, KILO_BALANCE_DEADBAND_DEG,
                          KILO_BALANCE_MAX_CORR_DEG};
  return link.set_goal(KILO_MSG_SET_BALANCE, (const uint8_t*)&b, sizeof b);
}

bool client_balance_enabled(SerialLink& link) {
  const Goal* g = link.goal(KILO_MSG_SET_BALANCE);
  return g != nullptr && g->payload[0] != 0;
}

bool client_look(SerialLink& link, double pan_deg, double tilt_deg) {
  const double lim = KILO_HEAD_MAX_DEG * 16.0;
  double pan = d_round(pan_deg * 16.0), tilt = d_round(tilt_deg * 16.0);
  if (!(pan >= -lim && pan <= lim && tilt >= -lim && tilt <= lim) || link.state().proto_ok == 0) return false;
  kilo_set_head_t h = {0, KILO_HEAD_MODE_HOLD, (int16_t)pan, (int16_t)tilt, KILO_HEAD_NOT_HELD, KILO_HEAD_SPEED_DPS};
  return link.set_goal(KILO_MSG_SET_HEAD, (const uint8_t*)&h, sizeof h);
}

}  // namespace kilo
