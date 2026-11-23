// recorder: the flight recorder. From boot, everything the mind sees and
// does goes into one RAM stream (record/flight.h), whether or not Kilo is
// walking: every body link frame both ways (HEARTBEAT only counted), every
// log line (the LogRing's tap), and during a walk or a trick what the
// mind knew. A run (a walk, a trick) is a span of the stream, opened and
// closed by RUN records and listed in the run index (WALK_LIST); the laptop
// fetches a run's span or any time range (kilo runs --host --fetch, kilo
// record fetch) and analyses it afterwards (kilo log walk|trick FILE).
// A walk's frames also download as a .kilolog (kilo/logs/kilolog.py, the
// raw frame format kilo log reads).
//
// A walk's records (protocol/spec.toml "The flight recorder", host.walk_format):
//   RUN BEGIN, MARK "walk id=N format=2 t0_us_hi=.. t0_us_lo=.. setup_us=0 mind_build=.. body_build=.."
//   MARK "setup ..."        the goal asked: pattern, direction, speed, balance
//   MARK "stream ..."       the stream began: the timing the planner chose
//   MARK "config ...", "model ...", "masses ..."   the planner's config, the body model
//   PLAN                    each tick planned (walk_plan)
//   PLANNER                 the hold, catch and wait counters, when they move
//   MARK "change ..." / "timing ..." / "stop ..." / "queued ..."
//   MARK "result ..." "error ..." "end ok=.. reason=..", RUN END
// A trick's (the trick player, polled):
//   RUN BEGIN, MARK "trick id=N format=2 trick=NAME [stage=S of=N] tid=.. crc=.. frames=.. speed=.. mirror=..
//                    mind_build=.. body_build=.. battery_mv=.. mode=.. by=remote|other"
//                    (the rear-up: S the stage allowed at the start, 1 from the remote, N when another
//                    client plays it whole; N its stages)
//   MARK "frames dur=a,b,.. hold=x,y,.. loop=start,end,repeats"   each keyframe's ticks at 1x
//   MARK "play speed=.."    at the trick's start on the body: the keyframes' times count from here
//   MARK "key ..."          a remote key while the run is open (which key stopped it)
//   MARK "stage allowed=S reached=R of=N"   the rear-up from the remote: A2 allowed a higher stage, or a
//                           stage's top was reached (R 0: none yet)
//   MARK "end reason=DONE|STOPPED|PREEMPTED|FELL|FAILED|REFUSED elapsed_ms=.. expected_ticks=..
//                    battery_mv=.. mode=.. [stage=S of=N [reached=R]] why=..", RUN END
// The self-measurement (C5 twice, measure/measure.h; docs/self-measurement.md): a run of kind TRICK (the
// host protocol has walks and tricks; its label tells it apart), label "measure=self end=REASON":
//   RUN BEGIN, MARK "measure id=N format=1 mind_build=.. body_build=.. script=1 tick0=.. ..."
//   MARK "model ...", "step i=.. exp=.. phase=.. t=.. tick=..", "tip exp=.. seen=.. x=..", "key ..."
//   MARK "end reason=DONE|STOPPED|FELL|ABORTED ticks=.. sent=.. ... why=..", RUN END
// The POSE it sent (each tick's feet and the model's attitude) and the TELEMETRY are in the stream.
// The run's label (WALK_INFO.text): "trick=NAME [stage=S of=N [reached=R]] end=REASON", S the
// highest stage allowed in the run, reached=R only when the play did not get to S's top.
// The frames themselves (TELEMETRY: the measured pitch and roll at every
// tick; the POSE keys and the end posture's SET_POSTURE) are in the stream anyway.
// A run stays RECORDING for KILO_HOST_WALK_POSTROLL_MS after its end, so
// its fetch (from KILO_HOST_WALK_PREROLL_MS before its start) has both rolls.
//
// Cost: an append is a header and a copy of at most 255 bytes (no
// allocation); a MARK line is formatted a handful of times per run.
#ifndef KILO_MIND_RECORD_RECORDER_H
#define KILO_MIND_RECORD_RECORDER_H
#include <stdint.h>
#include "link/serial_link.h"
#include "engine/walker.h"
#include "platform.h"
#include "record/flight.h"
#include "record/metrics.h"
#include "tricks/player.h"

namespace kilo {
namespace record {

// The storage the platforms allocate: hours of the link (about 3 KB/s
// idle, 6 KB/s walking).
constexpr uint32_t kStoreBytes = 64u << 20;

struct Run {
  uint32_t id;          // 1.. in this boot; an earlier boot's: boot << 20 | id
  uint8_t kind;         // KILO_RUN_KIND_*
  uint8_t flags;        // KILO_WALK_FLAGS_*
  uint8_t text_len;
  uint8_t reserved;
  uint32_t feet;        // POSE frames sent (walks)
  uint64_t start_us;    // the mind's clock at its start (a walk's setup, a trick's start)
  uint64_t end_us;      // ... at its end; 0 until then
  char text[KILO_HOST_WALK_TEXT_MAX];  // WALK_INFO.text
};

class Recorder final : public FrameTap, public engine::WalkObserver, public Log {
 public:
  static constexpr uint32_t kMaxRuns = 256;
  static constexpr uint64_t kPreRollUs = KILO_HOST_WALK_PREROLL_MS * 1000ull;
  static constexpr uint64_t kPostRollUs = KILO_HOST_WALK_POSTROLL_MS * 1000ull;

  Recorder(Clock& clock, const SerialLink& link);

  // storage: the platform's block (kStoreBytes). Without it nothing is recorded.
  void attach(uint8_t* storage, uint32_t bytes, uint32_t mind_build);
  bool attached() const { return flight_.ready(); }
  FlightStore& flight() { return flight_; }
  const FlightStore& flight() const { return flight_; }
  void set_metrics(Metrics* m) { metrics_ = m; }

  // Every pass: the trick player's state (a play began, was acknowledged,
  // ended), and the end of runs' post-rolls.
  void poll(const tricks::TrickPlayer& player);
  // The remote refused to start a trick (battery, USB power, the player busy).
  void trick_refused(const char* trick, const char* why);
  // The rear-up's stages in the player's play `play` (remote::TrickNotes::trick_stage): the open run's
  // label and a MARK, or kept for the run the play is about to open.
  void trick_stage(uint32_t play, int allowed, int reached, int of);
  // A remote key's line: a MARK into the open runs (and remembered: a trick
  // started within 5 s of one is "by=remote").
  void key(const char* text);
  // A MARK line into the stream, run or not (the distance guard's zone changes and actions).
  void note(const char* line) { mark_line(line, clock_.now_us()); }
  // The self-measurement's run (above): opened with its header's fields, closed with its end line.
  void measure_begin(const char* fields);
  void measure_end(const char* line, const char* reason, bool ok);

  // A run is open or in its post-roll.
  bool recording() const { return walk_ != 0 || trick_ != 0 || measure_ != 0 || tail_end_us_ != 0; }
  // Moving now: a walk, a trick or a measurement run open (the card saver waits).
  bool in_run() const { return walk_ != 0 || trick_ != 0 || measure_ != 0; }
  uint64_t last_run_end_us() const { return last_end_us_; }
  uint32_t runs_total() const { return next_id_ - 1; }
  // The oldest run with an id above after (0: the oldest kept), or null.
  const Run* run_after(uint32_t after) const;
  const Run* find(uint32_t id) const;
  uint32_t runs_kept() const { return n_runs_; }
  // Runs whose index entries changed since the saver last took them.
  uint32_t runs_changed() const { return runs_changed_; }
  // The blocks a run's fetch covers, from its pre-roll to its post-roll
  // (false: not in RAM, or still recording).
  bool span(const Run& r, uint32_t* first_seq, uint32_t* last_seq) const;

  // Log: every line (LogRing::set_tap)
  void write(LogLevel level, const char* line) override;
  // FrameTap
  void on_frame(bool tx, uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len, uint64_t now) override;
  // engine::WalkObserver
  void on_walk_mark(const engine::Walker& w, engine::WalkMark mark) override;
  void on_walk_tick(const engine::Walker& w, const engine::TickPlan& tp, bool sent) override;

 private:
  Run* begin_run(uint8_t kind, uint64_t now);
  void end_run(uint32_t id, uint64_t now);
  Run* run(uint32_t id);
  void mark_line(const char* line, uint64_t now);
  void run_mark(uint32_t id, uint8_t kind, uint8_t event, uint64_t now);
  void set_text(Run* r, const char* text);
  void walk_text(const engine::Walker& w, const char* end);
  void goal_line(const char* word, const engine::WalkGoal& g, uint64_t now);
  void timing_line(const char* word, const engine::Walker& w, uint64_t now);
  void begin_trick(const tricks::TrickPlayer& p, uint64_t now);
  void end_trick(const char* reason, const char* why, uint32_t elapsed_ms, uint32_t expected, uint64_t now);
  void trick_header(const tricks::Trick& t, uint32_t id, uint8_t speed, bool mirror, uint64_t now);
  void trick_text(Run* r);

  Clock& clock_;
  const SerialLink& link_;
  FlightStore flight_;
  Metrics* metrics_ = nullptr;
  uint32_t mind_build_ = 0;

  Run runs_[kMaxRuns] = {};
  uint32_t head_ = 0, n_runs_ = 0;  // the oldest entry, entries kept
  uint32_t next_id_ = 1;
  uint32_t runs_changed_ = 0;

  uint32_t walk_ = 0;      // the open walk run's id, 0 none
  uint32_t trick_ = 0;     // the open trick run's id
  uint32_t measure_ = 0;   // the open self-measurement run's id
  uint64_t tail_end_us_ = 0;  // the latest post-roll's end (0: none running)
  uint64_t last_end_us_ = 0;
  uint64_t last_key_us_ = 0;
  bool have_key_ = false;

  uint32_t trick_play_ = 0;   // the player's play the trick run is for
  const char* trick_name_ = "";
  // the open trick run's stages (of 0: not staged; reached -1: not known, the play not the remote's)
  int stage_allowed_ = 0, stage_reached_ = -1, stage_of_ = 0;
  uint32_t pending_play_ = 0;  // trick_stage() for a play whose run is not open yet
  int pending_allowed_ = 0, pending_reached_ = 0, pending_of_ = 0;
  bool play_marked_ = false;
  engine::SolverStats last_ = {};
};

}  // namespace record
}  // namespace kilo

#endif
