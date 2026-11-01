// walker: the planner's ticks streamed to the body as FEET at 50 Hz. A port
// of mind/kilo/planner/walker.py (BodyClock, Stream, Walker) to one
// cooperative state machine: Walker::poll() runs from the mind's main loop
// (every few ms, never blocking) where Python has a thread per walk.
//
// A walk (Walker::start):
// 1. TELEMETRY at 50 Hz: SET_STREAM telem_div 1, the JOINTS and IMU_RAW
//    divisors kept (stream_divs; put back after the walk).
// 2. The balance off for this walk if asked (SET_BALANCE enable 0, the
//    previous goal restored after); the STANCE posture with the posture's
//    balance flag, for kStanceWaitUs; then the posture goal is dropped (a
//    SET_POSTURE refresh would end GAIT on the body).
// 3. The stream: every 20 ms on a deadline grid (a wake more than two ticks
//    late restarts the grid, no burst), the TELEMETRY and EVENTs since the
//    last wake update the body clock, the error tracker and the planner;
//    then every planned tick up to the body's tick + KILO_FEET_LOOKAHEAD_TICKS
//    goes out through SerialLink::send_stream, so the body's 8-deep queue
//    holds about 5 (ticks already played are planned but not sent).
// 4. The planned stop: the planner steps every leg home and levels; when it
//    is done the STANCE posture ends GAIT (EVENT GAIT_STOPPED), and the
//    posture goal is cleared once the body has left GAIT.
// The stream also ends on GAIT_STOPPED from the body, GOAL_REFUSED for FEET,
// FELL, a reboot, an epoch change, the body leaving GAIT, GAIT not entered
// within 0.6 s, and the mind's loop stalling (mark_alive) for 0.5 s.
//
// The owner forwards its LinkListener's on_telemetry and on_event here
// (the link has one listener); time comes from the Clock it is given.
//
// What differs from Python: start() returns at once (the setup runs in
// poll()); a start during a planned stop is queued and started when that
// stop ends, or after kStartWaitUs with the old stream aborted; no per-tick
// log records (the frame log is the host's), no gc tuning.
#ifndef KILO_MIND_PLANNER_WALKER_H
#define KILO_MIND_PLANNER_WALKER_H
#include <stdint.h>
#include "link/serial_link.h"
#include "planner/errors.h"
#include "planner/plan.h"
#include "platform.h"

namespace kilo {
namespace planner {

constexpr uint32_t kTickUs = KILO_TICK_MS * 1000u;
constexpr int kLookahead = KILO_FEET_LOOKAHEAD_TICKS;
constexpr int kClockWindow = 100;      // telemetry frames in the body-clock estimate (2 s at 50 Hz)
constexpr int kClockResetFrames = 5;   // ...restarted after this many frames in a row a tick below its maximum
constexpr uint64_t kStartWaitUs = 5000000;     // a start during a planned stop waits this long, then aborts it
constexpr uint64_t kEnterTimeoutUs = 600000;   // the body must be in GAIT this soon after the first FEET
constexpr uint64_t kHandoverTimeoutUs = 1000000;  // ...and out of it this soon after the STANCE posture
constexpr uint64_t kMindStallUs = 500000;
constexpr uint64_t kStanceWaitUs = 600000;     // POSTURE_TICKS (0.5 s) and a margin

// The body's tick, estimated from TELEMETRY arrival times (walker.py
// BodyClock): body time = local time + the largest (tick x 20 ms - arrival)
// of the window, the least delayed frame; restarted from the newest frames
// when kClockResetFrames in a row are more than a tick below that maximum
// (the body's tick fell back after a stall of its loop).
class BodyClock {
 public:
  void reset();
  // One TELEMETRY: its u16 tick and arrival (us). Returns the tick unwrapped.
  int64_t add(uint16_t raw_tick, uint64_t rx_us);
  // The body's tick (fractional) at local time t_us; false before any TELEMETRY.
  bool now(uint64_t t_us, double* tick) const;
  int64_t tick() const { return tick_; }
  uint32_t resets() const { return resets_; }

 private:
  int64_t offs_[kClockWindow] = {};  // tick x 20 ms - arrival, us; a ring
  int head_ = 0, n_ = 0;
  bool has_raw_ = false;
  uint16_t raw_ = 0;
  int64_t max_ = 0;
  int below_ = 0;
  int64_t tick_ = 0;
  uint32_t resets_ = 0;
};

// FEET for one planned tick; pitch and roll kept inside the field's range
// (the body refuses a FEET beyond it: BAD_ARG ends the walk).
void pack_feet(uint8_t epoch, const TickPlan& tp, kilo_feet_t* out);

struct WalkGoal {
  const Pattern* pattern = &default_pattern();
  Direction direction = Direction::Forward;
  real speed_mm_s = -1;  // < 0: the pattern's default
  bool balance = true;   // false: the body's balance off for this walk
};

enum class WalkPhase : uint8_t {
  Idle,
  SetStream,  // SET_STREAM telem_div 1 sent, waiting for its ACK
  Stance,     // the STANCE posture, waiting kStanceWaitUs
  Streaming,  // FEET going out (also through the planned stop and the handover)
};

// One walk's stream counters and end (Python's Stream attributes).
struct StreamStats {
  uint32_t sent;
  uint32_t skipped;     // planned ticks already past when planned: not sent
  uint32_t underruns;   // EVENT FEET_UNDERRUN seen
  uint32_t max_behind;  // the most ticks one wake had to send
  uint32_t dropped_rx;  // TELEMETRY/EVENTs dropped between two wakes (queue full)
};

class Walker {
 public:
  Walker(SerialLink& link, Clock& clock, Log& log, const BodyModel& model = BodyModel(),
         const PlannerConfig& config = PlannerConfig());

  // From the owner's LinkListener.
  void on_telemetry(const kilo_telemetry_t& t);
  void on_event(const kilo_event_t& e);

  // Walk, or while walking change pattern, direction and speed at once (a
  // pattern after every leg has stepped home). Null when accepted, else why
  // not (Python's WalkError): nothing changed then.
  const char* start(const WalkGoal& goal);
  // The planned stop (each leg steps home, the body levels, then STANCE).
  void stop();
  // End the stream now, without the planned stop: the body holds the last
  // pose, then settles on its own.
  void abort();
  // The walk's work: setup, the 20 ms stream wakes, teardown. Never blocks.
  void poll();

  bool active() const { return phase_ != WalkPhase::Idle; }
  WalkPhase phase() const { return phase_; }
  // How the last walk ended (null while one runs or before any) and whether
  // it was the planned stop.
  const char* ended() const { return ended_; }
  bool ended_ok() const { return ok_; }
  const StreamStats& stream_stats() const { return stats_; }
  const Planner& planner() const { return planner_; }
  const ErrorTracker& tracker() const { return tracker_; }
  const BodyClock& body_clock() const { return clock_est_; }
  // The divisors the walk puts back (SET_STREAM telem, JOINTS, IMU_RAW);
  // the owner sets them when it changes the streams (client.stream_divs).
  void set_stream_divs(uint8_t telem, uint8_t joints, uint8_t imu_raw);
  // One log line: pattern, speed, what the stream did, the model's error.
  void log_status(LogLevel level) const;

 private:
  struct Tel {
    kilo_telemetry_t t;
    uint64_t rx_us;
  };
  static constexpr int kQueue = 32;
  static constexpr int kPred = 64;

  const char* check_can_walk() const;
  void update(const WalkGoal& goal);
  void begin_setup(const WalkGoal& goal, uint64_t now);
  void setup_step(uint64_t now);
  bool begin_stream(uint64_t now);
  bool wake(uint64_t now);
  void ingest();
  void end(const char* why, bool ok);
  void teardown(bool streamed);
  void posture_stance(bool balance);
  void send_stream_divs_step();

  SerialLink& link_;
  Clock& clock_;
  Log& log_;
  Planner planner_;
  ErrorTracker tracker_;
  BodyClock clock_est_;

  WalkPhase phase_ = WalkPhase::Idle;
  WalkGoal goal_;
  bool queued_ = false;  // a start during a planned stop
  WalkGoal queued_goal_;
  uint64_t queued_at_ = 0;

  // setup and teardown
  uint8_t divs_[3] = {KILO_TELEMETRY_DEFAULT_DIV, 0, 0};
  uint8_t telem_div_before_ = KILO_TELEMETRY_DEFAULT_DIV;
  bool stream_req_sent_ = false;
  uint8_t stream_req_seq_ = 0;  // our SET_STREAM's seq (a restore may be the request before it)
  bool restore_divs_ = false;  // SET_STREAM back to telem_div_before_, when the request slot is free
  bool posture_balance_ = false;
  bool balance_changed_ = false;
  bool restore_balance_ = false;
  uint8_t balance_before_[KILO_SET_BALANCE_LEN] = {};
  uint64_t phase_at_ = 0;

  // TELEMETRY seen, for the body clock at a walk's start (BodyState.history)
  struct Seen {
    uint16_t tick;
    uint64_t rx_us;
  };
  Seen seen_[kClockWindow] = {};
  int seen_head_ = 0, seen_n_ = 0;

  // what arrived since the last wake (the stream's _new_items)
  Tel tel_q_[kQueue] = {};
  int tel_head_ = 0, tel_n_ = 0;
  kilo_event_t ev_q_[kQueue] = {};
  int ev_head_ = 0, ev_n_ = 0;

  // the stream (Python's Stream)
  int16_t epoch_ = 0;
  uint32_t reboots_ = 0;
  StreamStats stats_ = {};
  bool gait_seen_ = false;
  bool has_last_tel_ = false;
  uint8_t last_mode_ = 0;
  bool handover_ = false;
  uint64_t handover_at_ = 0;
  bool started_ = false;
  uint64_t started_at_ = 0;
  bool stalled_ = false;
  uint64_t stalled_since_ = 0;
  uint64_t next_wake_ = 0;
  const char* ended_ = nullptr;  // points at ended_buf_ once ended
  char ended_buf_[96] = {};
  char why_[96] = {};  // an end reason being formatted
  bool ok_ = false;
  struct Pred {
    bool used;
    uint16_t tick;
    Attitude att;
  };
  Pred pred_[kPred] = {};  // u16 tick -> the attitude sent for it (the last 64)
};

}  // namespace planner
}  // namespace kilo

#endif
