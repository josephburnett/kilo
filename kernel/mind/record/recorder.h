// recorder: every walk, recorded tick by tick into the WalkStore, so that
// the error of the model can be studied afterwards (kilo walks --host IP
// --fetch, then kilo log walk FILE). Python counterpart: the daemon's
// --kilolog (kilo/logs/kilolog.py), which kept the raw frames of a whole
// session on the Pi's disk; this keeps the walks only, in RAM, with what
// the planner knew.
//
// A recording (the format is protocol/spec.toml [host] "Walk recordings",
// host.walk_format; records are kilo_walk_record_header_t + payload):
//   MARK "walk id=N format=1 t0_us=... mind_build=... body_build=..."   at t 0
//   RX / TX frames of the pre-roll (KILO_HOST_WALK_PREROLL_MS before the setup)
//   MARK "setup ..."        the goal asked: pattern, direction, speed, balance
//   RX / TX                 every body link frame both ways (TELEMETRY at
//                           50 Hz in the walk, EVENT, JOINTS if streamed;
//                           FEET, goals, commands, HEARTBEAT)
//   MARK "stream ..."       the stream began: the first tick, the timing the
//                           planner chose (speed, stride, period, shift, stance)
//   MARK "config ..."       the PlannerConfig in use (margin, gate, catch, ...)
//   MARK "model ..."        the BodyModel in use (masses, geometry, servo lag)
//   PLAN                    each tick planned (walk_plan: margin, the
//                           attitude without lag, the legs in the air)
//   PLANNER                 the hold, catch and wait counters, when they move
//   MARK "change ..." / "stop ..." / "queued ..."   as they happen
//   MARK "end ok=.. reason=..."  "result ..."  "error ..."   the end: the
//                           stream's counters and the error tracker's estimate
//   RX / TX of the post-roll (KILO_HOST_WALK_POSTROLL_MS), then the walk closes.
// t_us counts from t0 (the walk MARK gives it on the mind's clock), so a FEET
// written at t and the TELEMETRY that reports its tick give the lag.
//
// Cost: a record is a copy of at most 70 bytes into the ring, plus a few
// numbers formatted for a MARK line (a handful per walk); no allocation.
// The pre-roll is a ring of the last kPreRoll frames, filled all the time
// (one 72-byte copy per frame), copied into a walk at its setup.
#ifndef KILO_MIND_RECORD_RECORDER_H
#define KILO_MIND_RECORD_RECORDER_H
#include <stdint.h>
#include "link/serial_link.h"
#include "planner/walker.h"
#include "platform.h"
#include "record/store.h"

namespace kilo {
namespace record {

// The storage the platforms allocate for the recorder: enough for 20
// minutes of walking at 50 Hz and more (kernel/README.md "Walk recordings").
constexpr uint32_t kStoreBytes = 8u << 20;

class WalkRecorder final : public FrameTap, public planner::WalkObserver {
 public:
  static constexpr uint32_t kPreRoll = 256;  // frames kept for the pre-roll (about 2 s of an idle link)
  static constexpr uint64_t kPreRollUs = KILO_HOST_WALK_PREROLL_MS * 1000ull;
  static constexpr uint64_t kPostRollUs = KILO_HOST_WALK_POSTROLL_MS * 1000ull;

  WalkRecorder(Clock& clock, const SerialLink& link);

  // storage: the platform's block (kStoreBytes), the pre-roll carved from
  // its start, the store the rest. Without it nothing is recorded.
  void attach(uint8_t* storage, uint32_t bytes, uint32_t mind_build);
  bool attached() const { return store_.ready(); }
  WalkStore& store() { return store_; }
  const WalkStore& store() const { return store_; }
  // A walk is being recorded (from its setup to the end of its post-roll).
  bool recording() const { return recording_; }

  // Ends a walk's recording once its post-roll is over. Every pass.
  void poll();

  // FrameTap
  void on_frame(bool tx, uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len, uint64_t now) override;
  // planner::WalkObserver
  void on_walk_mark(const planner::Walker& w, planner::WalkMark mark) override;
  void on_walk_tick(const planner::Walker& w, const planner::TickPlan& tp, bool sent) override;

 private:
  struct PreRec {
    uint64_t t_us;
    uint8_t kind, type, seq, len;
    uint8_t payload[KILO_MAX_PAYLOAD];
  };

  void begin(const planner::Walker& w, uint64_t now);
  void finish();
  uint32_t rel(uint64_t now) const;
  bool frame(uint8_t kind, uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len, uint64_t now);
  void mark_line(const char* line, uint64_t now, bool essential = false);
  void set_text(const planner::Walker& w, const char* end);
  void goal_line(const char* word, const planner::WalkGoal& g, uint64_t now);
  void timing_line(const char* word, const planner::Walker& w, uint64_t now);

  Clock& clock_;
  const SerialLink& link_;
  WalkStore store_;
  uint32_t mind_build_ = 0;
  PreRec* pre_ = nullptr;
  uint32_t pre_head_ = 0, pre_n_ = 0;
  bool recording_ = false;
  bool ending_ = false;   // the walk ended: the post-roll runs until end_at_
  uint64_t end_at_ = 0;
  uint64_t t0_ = 0;
  planner::PlannerStats last_ = {};
};

}  // namespace record
}  // namespace kilo

#endif
