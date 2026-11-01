// imu: percepts from the TELEMETRY attitude: tilted, picked_up, tap, pet,
// startle. A port of kilo/perception/imu.py with the same detectors,
// thresholds (ImuConfig) and windows. Attitude convention as measured on
// Kilo: nose down = +pitch, right side up = +roll, degrees.
//
// - tilted / level: more than tilt_deg (25) from the reference for
//   tilt_confirm_s (0.5 s), released below tilt_exit_deg (18). The
//   reference is set_reference()'s, else learned once the attitude has been
//   calm for settle_s after start-up and after each mode change.
// - picked_up / put_down: attitude excursion over pickup_window_s >= 12 deg
//   (6 with the servos off) for 0.5 s while the body is not moving itself;
//   released once calm and back near the reference.
// - tap: a short impulse on a still body (vs the median of the preceding
//   window), up within 0.12 s and gone within 0.35 s.
// - pet: gentle periodic rocking over 4 s (1.5-10 deg peak to peak, >= 4
//   half cycles of 0.2-1.5 s, never faster than 30 deg/s).
// - startle: >= 10 deg from calm within 0.2 s, or a gyro rate >= 250 deg/s.
// While the body moves itself (GAIT, TRICK, RECOVERING, CALIB, BOOT, the
// gait/trick flags) and for settle_s after a mode change, only tilt runs.
//
// What differs: the attitude history is a fixed ring of kBuf samples (4.5 s
// at up to 56 Hz; the oldest are dropped beyond, Python keeps all of
// them); the cooldown bookkeeping is per event kind instead of a dict.
#ifndef KILO_MIND_PERCEPTION_IMU_H
#define KILO_MIND_PERCEPTION_IMU_H
#include <stdint.h>
#include "perception/events.h"

namespace kilo {
namespace perception {

struct ImuConfig {
  // tilt
  double tilt_deg = 25.0;
  double tilt_exit_deg = 18.0;
  double tilt_confirm_s = 0.5;
  double max_ref_deg = 20.0;
  // self motion
  double settle_s = 1.0;
  double calm_deg = 2.0;
  // pickup
  double pickup_window_s = 1.0;
  double pickup_deg = 12.0;
  double pickup_unpowered_deg = 6.0;
  double pickup_confirm_s = 0.5;
  int pickup_min_samples = 5;
  double put_down_calm_s = 1.5;
  // impulses
  double pre_window_s = 1.0;
  double pre_gap_s = 0.25;
  double tap_min_deg = 1.5;
  double tap_quiet_deg = 1.5;
  double tap_rise_s = 0.12;
  double tap_return_deg = 0.8;
  double tap_max_s = 0.35;
  double tap_cooldown_s = 0.5;
  double startle_deg = 10.0;
  double startle_rise_s = 0.2;
  double startle_rate_dps = 250.0;
  double startle_cooldown_s = 3.0;
  // pet
  double pet_window_s = 4.0;
  double pet_min_pp = 1.5;
  double pet_max_pp = 10.0;
  double pet_band_frac = 0.25;
  double pet_min_band = 0.5;
  int pet_min_half_cycles = 4;
  double pet_half_period_lo_s = 0.2;
  double pet_half_period_hi_s = 1.5;
  double pet_max_rate_dps = 30.0;
  double pet_cooldown_s = 3.0;
  double pet_hold_s = 1.5;
};

// One attitude sample. Unknown flags (has_flags false) count as powered and
// not moving; mode -1 is unknown.
struct ImuSample {
  double t;
  double pitch, roll;
  bool has_rates;
  double rates[3];  // rate_pitch, rate_roll, rate_yaw (deg/s)
  bool has_flags;
  uint16_t flags;   // KILO_TELEMETRY_FLAGS_*
  int mode;         // KILO_MODE_*, -1 unknown
};

class ImuDetector {
 public:
  explicit ImuDetector(const ImuConfig& cfg = ImuConfig()) : cfg_(cfg) { reset(); }
  void reset();
  // The current posture's expected attitude (fixed), or learn it again.
  void set_reference(double pitch, double roll);
  void learn_reference();
  // The mind moves the body (a posture move...): quiet until t + seconds + settle_s.
  void notify_self_motion(double t, double seconds);
  void feed(const ImuSample& s, PerceptEvents& out);

  bool active() const;
  bool pet() const;
  bool tilted() const { return tilted_; }
  TiltDirection tilt_direction() const { return tilt_dir_; }
  bool picked_up() const { return picked_up_; }
  bool has_attitude() const { return has_att_; }
  double pitch() const { return pitch_; }
  double roll() const { return roll_; }
  double ref_pitch() const { return ref_p_; }
  double ref_roll() const { return ref_r_; }

 private:
  struct S {
    double t, p, r;
  };
  struct D {
    double t, d;
  };
  static constexpr int kBuf = 256;
  static constexpr int kD = 32;
  const S& buf(int i) const { return buf_[(head_ + i) % kBuf]; }
  int window_start(double t, double seconds) const;  // index of the first sample in the last `seconds`
  double excursion(int from) const;                    // from index `from` to the newest
  bool cooldown_ok(Percept p, double t, double cooldown) const;
  void learn(double t);
  void tilt(double t, PerceptEvents& out);
  void pickup(double t, bool powered, PerceptEvents& out);
  bool impulse(double t, double* d);
  bool rose_from(double t, double below, double within) const;
  void startle(double t, bool has_d, double d, bool has_rate, double rate, PerceptEvents& out);
  void tap(double t, bool has_d, double d, PerceptEvents& out);
  void pet_step(double t, PerceptEvents& out);

  ImuConfig cfg_;
  double keep_s_ = 0;
  S buf_[kBuf];
  int head_ = 0, n_ = 0;
  D d_[kD];
  int d_head_ = 0, d_n_ = 0;
  double ref_p_ = 0, ref_r_ = 0;
  bool ref_fixed_ = false;
  bool has_relearn_ = false;
  double relearn_from_ = 0;
  int mode_ = -1;
  double quiet_from_ = 0;
  bool self_moving_ = false;
  bool has_att_ = false;
  double pitch_ = 0, roll_ = 0;
  bool tilted_ = false;
  TiltDirection tilt_dir_ = TiltDirection::None;
  bool has_tilt_since_ = false;
  double tilt_since_ = 0;
  bool picked_up_ = false;
  bool has_pick_since_ = false;
  double pick_since_ = 0;
  enum class TapState : uint8_t { Idle, Cand, Busy } tap_state_ = TapState::Idle;
  double tap_t0_ = 0, tap_peak_ = 0;
  bool pre_quiet_ = false;
  bool has_pet_seen_ = false;
  double pet_seen_ = 0;
  bool has_last_[(int)Percept::Count] = {};
  double last_[(int)Percept::Count] = {};
  // pet scratch (members: too big for a small stack)
  double ts_[kBuf], x_[kBuf], best_[kBuf], tmp_[kBuf];
};

}  // namespace perception
}  // namespace kilo

#endif
