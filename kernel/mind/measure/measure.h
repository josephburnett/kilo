// measure: Kilo measures himself (docs/self-measurement.md), the remote's B1.
// The script's poses (measure/script.h) streamed to the body as FEET at
// 50 Hz with the body's balance off, so the tilt the IMU reads is the
// body's own answer to each pose; every frame and a mark per step go into
// the flight recorder (a run of its own, "measure"), and the laptop fits
// the model to it afterwards (kilo measure fit). Nothing here changes a
// model number.
//
// start() refuses (and nothing moves) when: the body's protocol is not
// ours or its TELEMETRY is older than KILO_BODY_LOST_MS; it is on USB power;
// the battery (at rest, the highest of the last 20 s, as the tricks') is
// under kMinMv (spec battery.trick_min_mv); it is not IDLE in STAND or
// STANCE; it is not level and still (|pitch|, |roll| over kLevelMaxDeg, or
// either moved more than kStillPtpDeg over the last second's TELEMETRY:
// picked up, on a slope, still settling).
//
// A run:
// 1. SET_STREAM telem_div 1 (50 Hz TELEMETRY), its ACK waited for;
// 2. SET_BALANCE off (the goal there before put back at the end), the head
//    released to the posture's, STANCE without the balance flag for
//    kStanceWaitUs; then the posture goal is dropped (a refresh would end GAIT);
// 3. the stream: every 20 ms each script tick up to the body's tick +
//    KILO_FEET_LOOKAHEAD_TICKS goes out as FEET with the rigid model's
//    attitude and support for it (measure/script.h rigid_rest: "the model's
//    prediction" in the recording); TELEMETRY watched for each sweep's tip
//    (measure/script.h: the tilt toward the lifted foot kTipRiseDeg above
//    the lifted hold's and risen kTipJumpDeg within kTipJumpTicks, or
//    kTipUpFraction of the model's tip from the settle hold's during the
//    lifted hold: Script::tip; a "base" line with both baselines and a
//    "tip" line go into the recording) and for trouble;
// 4. the end: once the last tick has played, STAND (which ends GAIT on the
//    body), the balance goal and the TELEMETRY rate put back, and the run
//    closed once the body has left GAIT (or after kEndWaitUs).
//
// It stops at once, into the same end (STAND), on stop() (the remote: any
// key, B1 again among them), on EVENT FELL (the body's fall reflex has the
// joints; STAND is refused until it lets go, as for any posture), on the
// body leaving GAIT or refusing FEET, on a body reboot or epoch change, on
// a tilt over kAbortTiltDeg, on a tilt kAbortOffDeg away from the model's
// prediction (less the offset it had at the start: an IMU offset is
// expected), and on the mind's loop stalling.
#ifndef KILO_MIND_MEASURE_MEASURE_H
#define KILO_MIND_MEASURE_MEASURE_H
#include <stdint.h>
#include "link/serial_link.h"
#include "measure/script.h"
#include "planner/walker.h"
#include "platform.h"

namespace kilo {
namespace measure {

// Who records the run (the Mind: the flight recorder). Not pure (as remote::BehaviourHook).
class MeasureObserver {
 public:
  // The run begins: its header fields ("script=1 tick0=.. ..."), then MARK lines, then its end
  // (the end line; reason DONE, STOPPED, FELL or ABORTED).
  virtual void measure_begin(const char* /*fields*/) {}
  virtual void measure_line(const char* /*line*/) {}
  virtual void measure_end(const char* /*line*/, const char* /*reason*/, bool /*ok*/) {}

 protected:
  ~MeasureObserver() = default;
};

enum class Phase : uint8_t { Idle, SetStream, Stance, Streaming, Finish, Ending };

class Measure {
 public:
  static constexpr uint16_t kMinMv = KILO_BATTERY_TRICK_MIN_MV;
  static constexpr uint64_t kStanceWaitUs = 600000;
  static constexpr uint64_t kSetStreamWaitUs = 1000000;
  static constexpr uint64_t kEndWaitUs = 1500000;
  static constexpr uint64_t kEnterTimeoutUs = 600000;  // GAIT this soon after the first FEET
  static constexpr uint64_t kMindStallUs = 500000;
  static constexpr real kLevelMaxDeg = 10;    // start: |pitch| and |roll| at most this
  static constexpr real kStillPtpDeg = 3;     // ...and moved at most this over the last second
  static constexpr real kAbortTiltDeg = 25;   // stop: a tilt past this (the body's fall reflex: 75)
  static constexpr real kAbortOffDeg = 10;    // stop: this far from the model's prediction (less the start's offset)

  Measure(SerialLink& link, Clock& clock, Log& log, const planner::BodyModel& model = planner::BodyModel());
  void set_observer(MeasureObserver* o) { obs_ = o; }

  // From the owner's LinkListener.
  void on_telemetry(const kilo_telemetry_t& t);
  void on_event(const kilo_event_t& e);

  // Begin a run: null, or why not (nothing sent then). rested: the battery at rest, if known.
  const char* start(bool has_rested, uint16_t rested_mv);
  // Stop now (into STAND); why: for the log and the run's end ("B6 (0x02, stand)").
  void stop(const char* why);
  // The run's work. Never blocks.
  void poll();

  bool active() const { return phase_ != Phase::Idle; }
  Phase phase() const { return phase_; }
  // How the last run ended (null while one runs or before any), and whether it ran to its end.
  const char* ended() const { return ended_; }
  bool ended_ok() const { return ok_; }
  uint32_t ticks_sent() const { return sent_; }
  uint32_t tips() const { return tips_; }
  uint32_t runs() const { return runs_; }
  const Script& script() const { return script_; }
  // The body tick of script tick 0 (valid once streaming).
  int64_t tick0() const { return tick0_; }

 private:
  struct Seen {
    uint64_t rx_us;
    int16_t pitch, roll;
  };
  static constexpr int kSeen = 32;
  static constexpr int kRing = 256;  // script ticks remembered (predictions, steps)

  void abort(const char* why, const char* reason);
  void begin_stream(uint64_t now);
  void wake(uint64_t now);
  void finish(const char* reason, const char* why, bool ok);
  void ending(uint64_t now);
  void line(const char* text);
  void send_stance();
  void watch(const kilo_telemetry_t& t, int64_t tick);

  SerialLink& link_;
  Clock& clock_;
  Log& log_;
  planner::BodyModel model_;
  MeasureObserver* obs_ = nullptr;
  Script script_;
  planner::BodyClock clock_est_;

  Phase phase_ = Phase::Idle;
  uint64_t phase_at_ = 0;
  uint64_t next_wake_ = 0;
  bool req_sent_ = false;
  uint8_t req_seq_ = 0;
  bool restore_divs_ = false;
  uint64_t restore_at_ = 0;
  bool balance_changed_ = false, restore_balance_ = false;
  uint8_t balance_before_[KILO_SET_BALANCE_LEN] = {};

  int16_t epoch_ = 0;
  uint32_t reboots_ = 0;
  int64_t tick0_ = 0;
  int64_t planned_ = 0;   // the next body tick to plan
  int64_t last_tick_ = 0;  // the last body tick planned
  bool gait_seen_ = false;
  uint64_t stream_at_ = 0;
  bool stalled_ = false;
  uint64_t stalled_since_ = 0;
  uint8_t prev_support_ = 0xF;
  uint32_t sent_ = 0, late_ = 0, tips_ = 0, runs_ = 0;

  struct Planned {
    int64_t tick;  // script tick, -1 none
    int16_t step;
    int16_t into;  // ticks into its step
    int16_t pitch_a16, roll_a16;  // the prediction
  };
  Planned ring_[kRing] = {};
  bool have_offset_ = false;
  real off_p_ = 0, off_r_ = 0;  // measured - predicted at the start (the IMU's offset, roughly)
  // the tip's watch (script.py TipWatch): the sweep's two baselines, the tilt above the second
  int base_exp_ = -1;           // the experiment whose settle hold the baseline is of
  real base_p_ = 0, base_r_ = 0;
  uint32_t base_n_ = 0;
  real lift_p_ = 0, lift_r_ = 0;  // the lifted hold's last kTipBaseTicks, averaged
  uint32_t lift_n_ = 0;
  bool base_marked_ = false;    // the "base" line written
  bool tip_fired_ = false;      // this sweep's tip seen
  struct Rel {
    int64_t tick;  // script tick, -1 none
    real v;
  };
  static constexpr int kRel = 16;
  Rel rel_[kRel] = {};
  int last_mark_step_ = -1;

  Seen seen_[kSeen] = {};
  int seen_head_ = 0, seen_n_ = 0;

  const char* abort_why_ = nullptr;  // set from a callback, acted on in poll()
  const char* abort_reason_ = nullptr;
  char abort_buf_[96] = {};
  const char* ended_ = nullptr;
  char ended_buf_[112] = {};
  bool ok_ = false;
  char why_[112] = {};
};

}  // namespace measure
}  // namespace kilo

#endif
