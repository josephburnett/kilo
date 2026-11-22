#include "intent/estimate.h"
#include "proto/kilo_protocol.h"

namespace kilo {
namespace intent {

namespace {
constexpr int32_t kA16 = 16;
int32_t iabs(int32_t v) { return v < 0 ? -v : v; }
int32_t imax(int32_t a, int32_t b) { return a > b ? a : b; }
}  // namespace

const char* state_name(State s) {
  static const char* const kNames[kStates] = {"UNKNOWN",      "MOVING",        "STANDING", "SITTING", "RESTING",
                                              "ON_LEFT_SIDE", "ON_RIGHT_SIDE", "ON_BACK",  "ON_END"};
  return (int)s < kStates ? kNames[(int)s] : "?";
}

const char* cmd_name(Cmd c) {
  static const char* const kNames[5] = {"none", "stand", "sit", "rest", "limp"};
  return (int)c < 5 ? kNames[(int)c] : "?";
}

Cmd cmd_of_posture(uint8_t posture) {
  switch (posture) {
    case KILO_POSTURE_STAND:
    case KILO_POSTURE_STANCE:
      return Cmd::Stand;
    case KILO_POSTURE_SIT:
      return Cmd::Sit;
    case KILO_POSTURE_REST:
      return Cmd::Rest;
    case KILO_POSTURE_LIMP:
      return Cmd::Limp;
    default:
      return Cmd::None;
  }
}

namespace {
bool cmd_state(Cmd c, State* out) {
  switch (c) {
    case Cmd::Stand:
      *out = State::Standing;
      return true;
    case Cmd::Sit:
      *out = State::Sitting;
      return true;
    case Cmd::Rest:
    case Cmd::Limp:
      *out = State::Resting;
      return true;
    default:
      return false;
  }
}
}  // namespace

int16_t fold_pitch(int16_t pitch) {
  int32_t p = iabs(pitch);
  return (int16_t)(p <= 90 * kA16 ? p : 180 * kA16 - p);
}

bool attitude_class(int16_t pitch, int16_t roll, State* out) {
  if (fold_pitch(pitch) >= kEndA16) {
    *out = State::OnEnd;
    return true;
  }
  int32_t r = iabs(roll);
  if (r > kBackA16) {
    *out = State::OnBack;
    return true;
  }
  if (r >= kSideA16) {
    *out = roll > 0 ? State::OnLeftSide : State::OnRightSide;
    return true;
  }
  return false;
}

Answer classify(int16_t pitch, int16_t roll, Cmd cmd, State prev, const Row* rows, int n) {
  State f;
  if (attitude_class(pitch, roll, &f)) {
    if (f == State::OnEnd || f == State::OnBack) return {f, 90, 0, 0};
    int32_t want = roll > 0 ? 90 * kA16 : -90 * kA16;
    return {f, 90, 0, (int16_t)(roll - want)};
  }
  // the rows that fit: their score (the worse error as a part of its tolerance, x1000) and errors
  constexpr int kMax = 8;
  int fit_row[kMax];
  int32_t fit_score[kMax];
  int32_t fit_dp[kMax], fit_dr[kMax];
  int fits = 0;
  for (int i = 0; i < n && fits < kMax; i++) {
    int32_t dp = (int32_t)pitch - rows[i].pitch, dr = (int32_t)roll - rows[i].roll;
    if (iabs(dp) <= rows[i].tol_pitch && iabs(dr) <= rows[i].tol_roll) {
      fit_row[fits] = i;
      fit_score[fits] = imax(iabs(dp) * 1000 / rows[i].tol_pitch, iabs(dr) * 1000 / rows[i].tol_roll);
      fit_dp[fits] = dp;
      fit_dr[fits] = dr;
      fits++;
    }
  }
  if (fits == 0) return {State::Unknown, 0, 0, 0};
  State asked;
  bool has_asked = cmd_state(cmd, &asked);
  if (has_asked) {
    for (int k = 0; k < fits; k++)
      if (rows[fit_row[k]].state == asked)
        return {asked, (uint8_t)(100 - fit_score[k] / 20), (int16_t)fit_dp[k], (int16_t)fit_dr[k]};
    int best = 0;  // the nearest of the others (the first of equals: the table's order)
    for (int k = 1; k < fits; k++)
      if (fit_score[k] < fit_score[best]) best = k;
    return {rows[fit_row[best]].state, (uint8_t)(50 - fit_score[best] / 40), (int16_t)fit_dp[best],
            (int16_t)fit_dr[best]};
  }
  if (fits == 1)
    return {rows[fit_row[0]].state, (uint8_t)(60 - fit_score[0] / 50), (int16_t)fit_dp[0], (int16_t)fit_dr[0]};
  for (int k = 0; k < fits; k++)
    if (rows[fit_row[k]].state == prev) return {prev, 40, (int16_t)fit_dp[k], (int16_t)fit_dr[k]};
  return {State::Unknown, 0, 0, 0};
}

bool Estimator::feed(const Sample& s) {
  bool still = iabs(s.rate_pitch) < kStillA16 && iabs(s.rate_roll) < kStillA16 && iabs(s.rate_yaw) < kStillA16;
  if (!still) {
    still_ = false;
  } else if (!still_) {
    still_ = true;
    still_since_ = s.t_us;
  }
  settled_ = still_ && s.t_us - still_since_ >= kStillUs;
  Answer ans;
  bool moving_mode = s.mode < 16 && (kMovingModes >> s.mode) & 1u;
  if (moving_mode || !settled_) ans = {State::Moving, 0, 0, 0};
  else ans = classify(s.pitch, s.roll, s.cmd == Cmd::None && s.mode == KILO_MODE_LIMP ? Cmd::Limp : s.cmd, resting_);
  if (!started_) {
    started_ = true;
    est_ = Estimate();
    est_.since_us = s.t_us;
    cand_ = ans.state;
    cand_since_ = s.t_us;
  }
  if (ans.state != cand_) {
    cand_ = ans.state;
    cand_since_ = s.t_us;
  }
  if (ans.state == est_.state) {
    est_.confidence = ans.confidence;
    est_.err_pitch = ans.err_pitch;
    est_.err_roll = ans.err_roll;
    return false;
  }
  if (s.t_us - cand_since_ < kConfirmUs) return false;
  est_.state = ans.state;
  est_.confidence = ans.confidence;
  est_.since_us = s.t_us;
  est_.err_pitch = ans.err_pitch;
  est_.err_roll = ans.err_roll;
  if (ans.state != State::Moving && ans.state != State::Unknown) resting_ = ans.state;
  changes_++;
  return true;
}

}  // namespace intent
}  // namespace kilo
