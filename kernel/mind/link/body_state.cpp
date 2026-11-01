#include "link/body_state.h"

namespace kilo {

bool BodyState::on_telemetry(const kilo_telemetry_t& t, uint64_t now_us, int16_t* prev_epoch) {
  bool backwards = false;
  if (!has_tick) {
    tick = t.tick;
    has_tick = true;
  } else {
    uint16_t d = (uint16_t)(t.tick - tick_raw);
    if (d >= 0x8000u) {  // more than half a wrap backwards: the body restarted
      backwards = true;
      tick = t.tick;
    } else {
      tick += d;
    }
  }
  tick_raw = t.tick;
  telemetry = t;
  has_telemetry = true;
  telemetry_at_us = now_us;
  telemetry_count++;
  if (t.dist_mm != 0) {  // state.py: the echo, then (arrival, median of the last 3)
    if (dist_n_ == 3) {
      dist_[0] = dist_[1];
      dist_[1] = dist_[2];
      dist_n_ = 2;
    }
    dist_[dist_n_++] = t.dist_mm;
    double m;
    distance_mm(&m);
    if (dist_hist_n_ == 8) {
      dist_hist_head_ = (uint8_t)((dist_hist_head_ + 1) % 8);
      dist_hist_n_--;
    }
    dist_hist_[(dist_hist_head_ + dist_hist_n_++) % 8] = {now_us, m};
  }
  *prev_epoch = epoch;
  epoch = t.epoch;
  return backwards;
}

void BodyState::on_hello(const kilo_hello_t& h, uint64_t now_us) {
  hello = h;
  has_hello = true;
  hello_at_us = now_us;
  epoch = h.epoch;
  proto_ok = h.proto_major == KILO_PROTO_MAJOR ? 1 : 0;
}

void BodyState::reset_session() {
  has_tick = false;
  tick = 0;
  tick_raw = 0;
  dist_n_ = 0;
  dist_hist_head_ = dist_hist_n_ = 0;
}

bool BodyState::alive(uint64_t now_us, uint32_t lost_us) const {
  return connected && has_telemetry && now_us - telemetry_at_us < lost_us;
}

// ---- derived ------------------------------------------------------------------------

namespace {

// state.py BATTERY_CURVE: resting 2S Li-ion, pack mV -> percent (est.)
const struct {
  uint16_t mv;
  double pct;
} kBatteryCurve[] = {{6600, 0.0},  {7000, 5.0},  {7200, 12.0}, {7300, 20.0}, {7400, 30.0}, {7500, 40.0},
                     {7600, 50.0}, {7800, 65.0}, {8000, 80.0}, {8200, 90.0}, {8400, 100.0}};
constexpr int kCurveN = sizeof kBatteryCurve / sizeof kBatteryCurve[0];

}  // namespace

double battery_percent(uint16_t mv) {
  if (mv <= kBatteryCurve[0].mv) return kBatteryCurve[0].pct;
  if (mv >= kBatteryCurve[kCurveN - 1].mv) return kBatteryCurve[kCurveN - 1].pct;
  for (int i = 0; i + 1 < kCurveN; i++) {
    double v0 = kBatteryCurve[i].mv, v1 = kBatteryCurve[i + 1].mv;
    if (v0 <= mv && mv <= v1) {
      double p0 = kBatteryCurve[i].pct, p1 = kBatteryCurve[i + 1].pct;
      return p0 + (p1 - p0) * (mv - v0) / (v1 - v0);
    }
  }
  return 0.0;  // unreachable
}

bool BodyState::distance_mm(double* mm) const {
  if (dist_n_ == 0) return false;
  double a = dist_[0];
  if (dist_n_ == 1) {
    *mm = a;
  } else if (dist_n_ == 2) {
    *mm = (a + dist_[1]) / 2;  // statistics.median of two: their mean
  } else {
    double b = dist_[1], c = dist_[2];
    *mm = a > b ? (b > c ? b : (a > c ? c : a)) : (a > c ? a : (b > c ? c : b));
  }
  return true;
}

bool BodyState::approach_mm_s(double* v) const {
  if (dist_hist_n_ < 2) return false;
  const DistPoint& h0 = dist_hist_[dist_hist_head_];
  const DistPoint& hn = dist_hist_[(dist_hist_head_ + dist_hist_n_ - 1) % 8];
  if (hn.at_us <= h0.at_us) return false;
  *v = (h0.median - hn.median) / ((double)(hn.at_us - h0.at_us) / 1e6);
  return true;
}

bool BodyState::battery_pct(double* pct) const {
  if (!has_telemetry || (telemetry.flags & KILO_TELEMETRY_FLAGS_NO_BATTERY) || telemetry.batt_mv == 0) return false;
  *pct = battery_percent(telemetry.batt_mv);
  return true;
}

bool BodyState::fallen() const {
  return has_telemetry && (telemetry.mode == KILO_MODE_FALLEN || telemetry.mode == KILO_MODE_RECOVERING);
}

bool BodyState::moving() const {
  if (!has_telemetry) return false;
  uint8_t m = telemetry.mode;
  return (telemetry.flags & (KILO_TELEMETRY_FLAGS_GAIT | KILO_TELEMETRY_FLAGS_TRICK)) != 0 || m == KILO_MODE_GAIT ||
         m == KILO_MODE_TRICK || m == KILO_MODE_RECOVERING;
}

int BodyState::upright() const {
  if (!has_telemetry) return -1;
  const int16_t lim = 30 * 16;  // state.py UPRIGHT_DEG, in 1/16 deg (|p| / 16 < 30)
  bool up = telemetry.pitch < lim && telemetry.pitch > -lim && telemetry.roll < lim && telemetry.roll > -lim;
  return up && !fallen() ? 1 : 0;
}

}  // namespace kilo
