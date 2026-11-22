// mind: the whole mind core in one object, so a platform's main loop is a
// dozen lines.
//
//   LogRing      the RAM log (every line also goes to the platform's Log)
//   SerialLink   the body link (link/)
//   Walker       the FEET stream (planner/)
//   TrickPlayer  tricks from the compiled-in library (tricks/)
//   Percepts     fed by every TELEMETRY (perception/)
//   Runner       the behaviour layer at 10 Hz, when enabled (behaviour/)
//   RemoteControl  EVENT IR_KEY -> actions, when enabled (remote/)
//   HostServer   the laptop interface, when the platform has a Listener (net/)
//   Recorder     the flight recorder: every frame, log line, walk and trick,
//                with storage from attach_recorder() (record/)
//   Saver        the recorder onto the SD card at safe moments (attach_saver())
//   Metrics      the session's counters (METRICS, kilo metrics --host)
//   Measure      the self-measurement routine (measure/), the remote's B1
//   SoundPlayer  what Kilo says and when (sound/): B2, waking, the
//                behaviour's purrs, the guard's hiss; the output from the
//                platform (MindConfig.sound_output), the sounds the build packed
//   Estimator    the state estimate (intent/estimate.h): STANDING, SITTING,
//                RESTING, on a side, ... from every TELEMETRY; it always runs
//                and only reads (a log line, a MARK and a counter per change)
//   Intent       "be standing / sitting / resting" (intent/intent.h): with
//                MindConfig.intent the remote's B6 and A7, the behaviour's
//                stand, sit and rest and the guard's sit go through it
//                (docs/intent.md); without, nothing calls it
//
//   static Mind mind(serial, clock, platform_log, &listener, cfg);  // ~150 KB: static
//   mind.attach_log(new LogEntry[LogRing::kLines], LogRing::kLines);  // the log's history, once
//   mind.attach_recorder(storage, record::kStoreBytes);  // the flight recorder's 64 MB, once
//   mind.attach_saver(*new record::Saver(...));          // with an SD card
//   mind.start();                // the port is open: HELLO_REQ
//   for (;;) mind.poll();        // a few ms apart; never blocks
//
// One pass of poll(): stamp the mind's liveness, serve the laptop, then
// (unless the raw bridge is open) poll the link, collect the command
// slot's answer for its owner (the trick player, the STATS poll) before
// anyone may send another, and run the walker, the trick player, the
// remote and the behaviour runner.
//
// Liveness (kernel/mind/README.md, the link's open question 3): poll()
// stamps link.mark_alive() once per pass. One cooperative loop runs every
// part, so a pass that returns is the proof that nothing is stuck: a hung
// component never returns and the stamp stops, which is what the body's
// watchdog needs. The one thing a pass cannot prove is that the behaviour
// runner keeps ticking (its 10 Hz grid could stall with the loop alive), so
// with a runner the stamp also needs a tick within kRunnerStallUs (10 s of
// grace before the first).
//
// Who drives the body: the laptop's controller (CONTROL take=1) owns it;
// the remote is ignored and the behaviour runner paused meanwhile. The
// runner runs from the start with MindConfig.behaviour, and the remote's A5
// starts it at run time; while it runs, the remote's B6, A7 and C7 stop it
// (then do their own action) and its other keys are ignored
// (remote/remote.h).
#ifndef KILO_MIND_MIND_H
#define KILO_MIND_MIND_H
#include <stdint.h>
#include "behaviour/alive.h"
#include "behaviour/runner.h"
#include "guard/guard.h"
#include "intent/intent.h"
#include "link/serial_link.h"
#include "measure/measure.h"
#include "net/host_server.h"
#include "perception/percepts.h"
#include "planner/walker.h"
#include "platform.h"
#include "platform_net.h"
#include "power/sleep.h"
#include "record/metrics.h"
#include "record/recorder.h"
#include "record/saver.h"
#include "remote/remote.h"
#include "sound/player.h"
#include "tricks/player.h"
#include "util/logring.h"

namespace kilo {

struct MindConfig {
  uint32_t mind_build = 0;      // STATUS.mind_build (the git hash's first 8 hex digits)
  bool remote = false;          // the IR remote's keys act (the kernel turns it on)
  bool behaviour = false;       // the behaviour runner from the start (the remote's A5 starts it too)
  uint8_t rear_up_key = 0;      // the stage the remote's A2 starts, 1-3, 0 off (KILO_REAR_UP; remote/keys.h): off until the trials
  bool intent = false;          // stand, sit and rest go through the intents (intent/intent.h; KILO_INTENT): off until tried on Kilo
  double balance_kp = -1;       // >= 0: SET_BALANCE on with this Kp from the start
  uint64_t seed = 0;            // the behaviour RNG's seed (logged)
  const behaviour::Personality* personality = nullptr;  // null: behaviour::kPersonality
  uint32_t stats_period_ms = 0;  // > 0: STATS_REQ this often, the answer logged ("body STATS: ...")
  bool record_save = false;     // the card saver on from the start (KILO_RECORD_SAVE; the card's SAVE_ON also turns it on)
  bool light_sleep = false;     // A7: LIMP once still, light sleep after 30 s of nothing (power/sleep.h; KILO_LIGHT_SLEEP)
  bool guard = guard::kDefaultOn;  // the distance guard (guard/guard.h; kilo guard on|off --host at run time)
  // Kilo's sounds (sound/player.h, docs/sound.md): the table the build packed (sound::kSounds) and the
  // platform's output; without either every play request is a counted no-op.
  const sound::Sound* sounds = nullptr;
  uint32_t sound_count = 0;
  sound::SoundOutput* sound_output = nullptr;
  bool sound = sound::kDefaultOn;              // kilo sound on|off --host at run time
  uint8_t sound_volume = sound::kDefaultVolume;
  // The laptop interface's keys (net/host_server.h set_keys(); they outlive
  // the Mind) and the source of its ephemeral keys. Without a Random every
  // connection is refused.
  const AuthorizedKey* authorized_keys = nullptr;
  uint32_t authorized_key_count = 0;
  Random* random = nullptr;
};

class Mind final : public LinkListener,
                   public remote::TrickNotes,
                   public record::MetricsSource,
                   public guard::GuardHost,
                   public sound::SoundHost,
                   public measure::MeasureObserver,
                   public intent::IntentHost,
                   public CalGate {
 public:
  static constexpr uint64_t kRunnerStallUs = 1000000;       // the runner's last tick at most this old
  static constexpr uint64_t kRunnerStartGraceUs = 10000000;  // the grace before its first tick

  // listener: the laptop interface's socket, or null for none.
  Mind(Serial& serial, Clock& clock, Log& platform_log, Listener* listener, const MindConfig& cfg = MindConfig());

  // The port is open: latch the start goals, connect the link.
  void start();
  // One pass. Never blocks.
  void poll();

  // A second LinkListener for the platform (statistics windows); the Mind
  // keeps the link's own.
  void set_tap(LinkListener* tap) { tap_ = tap; }
  // The flight recorder's block (record::kStoreBytes, allocated once by
  // the platform; it outlives the Mind), served to the laptop as WALK_LIST,
  // WALK_FETCH, REC_FETCH. Before start(). Without it nothing is recorded.
  void attach_recorder(uint8_t* storage, uint32_t bytes);
  // The log's history (LogRing::kLines entries, once). Before start().
  void attach_log(LogEntry* lines, uint32_t n) { log_.attach(lines, n); }
  // The card saver (it outlives the Mind; made on recorder() and log()).
  void attach_saver(record::Saver& saver);
  record::Saver* saver() { return saver_; }
  record::Metrics& metrics() { return metrics_; }
  // Nothing moves and nothing needs the loop: the card may be written.
  bool card_safe() const { return card_safe_; }

  LogRing& log() { return log_; }
  SerialLink& link() { return link_; }
  planner::Walker& walker() { return walker_; }
  tricks::TrickPlayer& player() { return player_; }
  perception::Percepts& percepts() { return percepts_; }
  remote::RemoteControl& remote() { return remote_; }
  HostServer& host() { return host_; }
  record::Recorder& recorder() { return recorder_; }
  behaviour::Runner& runner() { return runner_; }
  power::Sleep& sleep() { return sleep_; }
  bool behaviour_active() const { return behaviour_on_ && !runner_.stopped(); }
  bool poweroff_requested() const { return runner_.poweroff_requested(); }
  // A laptop's controller sent REBOOT (and has its OK): restart the platform.
  bool reboot_requested() const { return host_.reboot_requested(); }
  // A laptop client holds control (or the raw bridge is open).
  bool host_controls() const { return listener_ != nullptr && (host_.controller() >= 0 || host_.link_paused()); }
  // Whether the last pass stamped the link's liveness.
  bool stamping() const { return stamping_; }

  // LinkListener
  void on_connection(bool connected, bool alive) override;
  void on_hello(const kilo_hello_t& hello, bool solicited) override;
  void on_telemetry(const kilo_telemetry_t& t) override;
  void on_event(const kilo_event_t& e) override;
  void on_reboot(const kilo_hello_t* hello, const char* why) override;
  void on_goals_cleared(const char* reason, uint32_t count) override;
  void on_refused(uint8_t goal_type, uint8_t reason) override;
  // remote::TrickNotes
  void refused(const char* trick, const char* why) override { recorder_.trick_refused(trick, why); }
  void trick_mark(const char* line) override { recorder_.note(line); }
  void trick_stage(uint32_t play, int allowed, int reached, int of) override {
    recorder_.trick_stage(play, allowed, reached, of);
  }
  bool rested_mv(uint16_t* mv) override;
  // record::MetricsSource
  uint32_t metric_lines() override;
  void metric_line(uint32_t i, char* out, uint32_t n) override;
  // guard::GuardHost
  void sit_and_behave() override;
  const char* not_now() override;
  guard::Still still() override;
  void notice() override;
  bool flinch() override;
  // CalGate: no calibration change over the host channel while a walk or trick runs.
  const char* cal_refusal() override;
  void mark(const char* line) override { recorder_.note(line); }  // guard::GuardHost, sound::SoundHost
  // measure::MeasureObserver: the run into the flight recorder
  void measure_begin(const char* fields) override { recorder_.measure_begin(fields); }
  void measure_line(const char* line) override { recorder_.note(line); }
  void measure_end(const char* line, const char* reason, bool ok) override { recorder_.measure_end(line, reason, ok); }
  measure::Measure& measure() { return measure_; }
  // intent::IntentHost
  void intent_moves() override;
  void intent_mark(const char* line) override { recorder_.note(line); }
  intent::Estimator& estimator() { return estimator_; }
  intent::Intent& intent() { return intent_; }
  // What the mind last asked the body for, as the state estimate's prior (kilo.intent.replay rebuilds it
  // the same way): the SET_POSTURE goal latched; else, after a trick that ended DONE or STOPPED, its end
  // posture; after EVENT RECOVERED stand, after RECOVERY_FAILED rest; nothing while a walk, a trick or the
  // self-measurement runs or after a fall.
  intent::Cmd intent_cmd() const;
  // A posture by name, as the remote's keys, the behaviour layer and the guard ask for it: with
  // MindConfig.intent "stand", "sit" and "rest" are intents (be(STANDING) ...), everything else and
  // everything without it the body's own posture change (client_posture). who: for the log.
  bool posture(const char* name, int8_t balance, const char* who);

  guard::DistanceGuard& guard() { return guard_; }
  sound::SoundPlayer& sound() { return sound_; }
  // Ask for a sound (the events' hooks): not while a trick plays (but the alive layer's small sitting moves) or a
  // laptop holds the body, unless always. why: for the log ("wake", "guard red").
  uint8_t say(const char* name, const char* why, bool always = false);
  behaviour::Alive& alive() { return alive_; }
  // The behaviour runner's sitting variant (the guard started it): every stand the tree asks for is a sit.
  bool sitting_variant() const { return sitting_variant_; }

 private:
  // the behaviour layer's body (behaviour/body.h BodyIface) and percepts (behaviour/runner.h PerceptSource)
  class Body final : public behaviour::BodyIface, public behaviour::PerceptSource {
   public:
    explicit Body(Mind& m) : m_(m) {}
    void snapshot(behaviour::Snapshot& out) override;
    bool posture(const char* name, int8_t balance) override;
    bool set_balance(bool enable) override;
    bool look(double pan, double tilt) override;
    uint32_t play(const char* trick) override;
    behaviour::Status play_status(uint32_t handle) override;
    void play_cancel(uint32_t handle) override;
    void beep(const uint16_t (*notes)[2], int n) override;
    void provide(behaviour::BehaviourPercepts& p, double* startle) override;
    uint32_t unsupported_beeps = 0;

   private:
    Mind& m_;
  };
  // the remote's A5 / B6 / A7 / C7 (remote.h)
  class RunnerHook final : public remote::BehaviourHook {
   public:
    explicit RunnerHook(Mind& m) : m_(m) {}
    bool active() override { return m_.behaviour_active(); }
    bool start(const char** what) override;
    void stop() override;
    bool yields(const remote::Key& key) override;

   private:
    Mind& m_;
    char what_[48] = {};
  };
  // the remote's posture keys (remote.h PostureHook): through posture() above
  class PostureHookImpl final : public remote::PostureHook {
   public:
    explicit PostureHookImpl(Mind& m) : m_(m) {}
    bool posture(const char* name, const char* who) override { return m_.posture(name, -1, who); }

   private:
    Mind& m_;
  };
  // the remote's B1 (remote.h MeasureHook)
  class MeasureHookImpl final : public remote::MeasureHook {
   public:
    explicit MeasureHookImpl(Mind& m) : m_(m) {}
    bool active() override { return m_.measure_.active(); }
    const char* start() override;
    void stop(const char* why) override { m_.measure_.stop(why); }

   private:
    Mind& m_;
  };
  class NoListener final : public Listener {
   public:
    int32_t accept() override { return -1; }
    Conn* conn(uint32_t) override { return nullptr; }
    uint32_t slots() const override { return 0; }
  };

  static bool paused(void* arg) { return static_cast<Mind*>(arg)->host_controls(); }
  static double seconds(void* arg) { return (double)static_cast<Mind*>(arg)->clock_.now_us() / 1e6; }
  bool liveness(uint64_t now);
  void ir_key(const kilo_event_t& e);
  bool forward_key(const remote::Key& key) const;
  bool lying(const kilo_telemetry_t& t) const;
  bool lying_rest();
  double head_held(int field) const;  // the SET_HEAD goal's pan (2) or tilt (4) in degrees, 0 when not held
  void stats(uint64_t now);
  bool safe_for_card(uint64_t now);
  bool body_limp() const;

  Clock& clock_;
  MindConfig cfg_;
  LogRing log_;
  SerialLink link_;
  NoListener no_listener_;
  Listener* listener_;
  HostServer host_;
  planner::Walker walker_;
  tricks::TrickPlayer player_;
  perception::Percepts percepts_;
  Body body_;
  behaviour::Runner runner_;
  RunnerHook runner_hook_;
  remote::RemoteControl remote_;
  measure::Measure measure_;
  MeasureHookImpl measure_hook_;
  intent::Estimator estimator_;
  intent::Intent intent_;
  PostureHookImpl posture_hook_;
  intent::Cmd prior_ = intent::Cmd::None;  // intent_cmd() when no SET_POSTURE goal is latched
  uint32_t state_entered_[intent::kStates] = {};  // how often the estimate entered each state (METRICS "state:")
  void estimate_step(const kilo_telemetry_t& t);
  power::Sleep sleep_;  // light sleep after A7 or lying idle (power/sleep.h)
  guard::DistanceGuard guard_;
  behaviour::Alive alive_;
  bool sitting_variant_ = false;  // the runner's sitting variant (the guard's)
  bool runner_by_guard_ = false;  // the guard started the runner: the remote's walking keys stop it
  sound::SoundPlayer sound_;
  sound::Chatter chatter_;        // the behaviour's own sounds (A5)
  bool was_asleep_ = false;       // light sleep, the last pass
  bool floor_seen_ = false;       // lying_rest(): the ultrasonic read the floor under his nose
  uint64_t floor_seen_us_ = 0;    // ... last at
  bool hello_said_ = false;       // the first HELLO's mew
  uint32_t hisses_said_ = 0;      // the guard's RED reactions hissed at
  bool head_centred_ = false;     // the head held straight ahead for a walk (head_for_walk)
  bool flinch_pending_ = false;   // the guard's lean back, once the sitting move it stopped has ended
  void head_for_walk();
  uint16_t posture_seen_ = 0xFFFFu;  // the SET_POSTURE goal last seen (code, flags): a change moves the body
  void posture_moves();
  void sound_events();
  // Low battery, judged at rest (power/battery.h RestedBattery): the eyes' orange.
  bool tired_ = false;
  void battery_step();
 public:
  bool tired() const { return tired_; }
  // The battery at rest in percent (what the behaviour runner and the eyes judge by); false if none.
  bool rested_pct(double* pct) const;
 private:
  bool trick_playing() const;     // a trick other than the alive layer's sitting moves
  record::Recorder recorder_;
  record::Metrics metrics_;
  record::Saver* saver_ = nullptr;
  uint64_t unsafe_at_us_ = 0;  // the last pass that was not safe for the card
  bool card_safe_ = false;
  LinkListener* tap_ = nullptr;

  // the snapshot's last EVENTs (BodyState.events)
  behaviour::Snapshot::Event events_[behaviour::Snapshot::kEvents] = {};
  int n_events_ = 0;
  uint32_t event_count_ = 0;

  uint64_t started_us_ = 0;
  bool behaviour_on_ = false;     // the runner was started (by the config or by A5)
  uint64_t runner_started_us_ = 0;  // ...at this time: the first tick's grace runs from here
  bool stamping_ = true;
  bool paused_ = false;  // the last pass found the UART taken (bridge or flasher)
  bool stats_asked_ = false;
  uint8_t stats_seq_ = 0;
  uint64_t next_stats_us_ = 0;
};

}  // namespace kilo

#endif
