// mind: the whole mind core in one object, so a platform's main loop is a
// dozen lines. Python counterpart: the daemon (kilo/daemon/core.py) with
// its BodyClient, walker, trick player, SharedPercepts, behaviour runner
// (--behaviour) and IR remote (--remote), and its socket server.
//
//   LogRing      the RAM log (every line also goes to the platform's Log)
//   SerialLink   the body link (link/)
//   Walker       the FEET stream (planner/)
//   TrickPlayer  tricks from the compiled-in library (tricks/)
//   Percepts     fed by every TELEMETRY (perception/)
//   Runner       the behaviour layer at 10 Hz, when enabled (behaviour/)
//   RemoteControl  EVENT IR_KEY -> actions, when enabled (remote/)
//   HostServer   the laptop interface, when the platform has a Listener (net/)
//
//   static Mind mind(serial, clock, platform_log, &listener, cfg);  // ~60 KB: static
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
// grace before the first), as the daemon's pump checks (daemon/behaviour.py
// _stamp_liveness).
//
// Who drives the body: the laptop's controller (CONTROL take=1) owns it;
// the remote is ignored and the behaviour runner paused meanwhile (Python
// refuses client goals while the runner is active unless --override pauses
// it: a host controller is that override). With the runner active the
// remote is ignored (remote_ir.py).
#ifndef KILO_MIND_MIND_H
#define KILO_MIND_MIND_H
#include <stdint.h>
#include "behaviour/runner.h"
#include "link/serial_link.h"
#include "net/host_server.h"
#include "perception/percepts.h"
#include "planner/walker.h"
#include "platform.h"
#include "platform_net.h"
#include "remote/remote.h"
#include "tricks/player.h"
#include "util/logring.h"

namespace kilo {

struct MindConfig {
  uint32_t mind_build = 0;      // STATUS.mind_build (the git hash's first 8 hex digits)
  bool remote = false;          // kilo run --remote (the Pi's KILO_RUN_ARGS turns it on)
  bool behaviour = false;       // kilo run --behaviour
  double balance_kp = -1;       // >= 0: SET_BALANCE on with this Kp from the start (kilo run --balance KP)
  uint64_t seed = 0;            // the behaviour RNG's seed (logged)
  const behaviour::Personality* personality = nullptr;  // null: behaviour::kPersonality
  remote::RemoteConfig remote_cfg;
  uint32_t stats_period_ms = 0;  // > 0: STATS_REQ this often, the answer logged ("body STATS: ...")
};

class Mind final : public LinkListener {
 public:
  static constexpr uint64_t kRunnerStallUs = 1000000;       // daemon/behaviour.py RUNNER_STALL_S
  static constexpr uint64_t kRunnerStartGraceUs = 10000000;  // RUNNER_START_GRACE_S

  // listener: the laptop interface's socket, or null for none.
  Mind(Serial& serial, Clock& clock, Log& platform_log, Listener* listener, const MindConfig& cfg = MindConfig());

  // The port is open: latch the start goals, connect the link.
  void start();
  // One pass. Never blocks.
  void poll();

  // A second LinkListener for the platform (statistics windows); the Mind
  // keeps the link's own.
  void set_tap(LinkListener* tap) { tap_ = tap; }

  LogRing& log() { return log_; }
  SerialLink& link() { return link_; }
  planner::Walker& walker() { return walker_; }
  tricks::TrickPlayer& player() { return player_; }
  perception::Percepts& percepts() { return percepts_; }
  remote::RemoteControl& remote() { return remote_; }
  HostServer& host() { return host_; }
  behaviour::Runner& runner() { return runner_; }
  bool behaviour_active() const { return cfg_.behaviour && !runner_.stopped(); }
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

 private:
  // the behaviour layer's body (behaviour/adapter.py BodyAdapter) and percepts (runner.py PerceptFeed)
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
  void stats(uint64_t now);

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
  remote::RemoteControl remote_;
  LinkListener* tap_ = nullptr;

  // the snapshot's last EVENTs (BodyState.events)
  behaviour::Snapshot::Event events_[behaviour::Snapshot::kEvents] = {};
  int n_events_ = 0;
  uint32_t event_count_ = 0;

  uint64_t started_us_ = 0;
  bool stamping_ = true;
  bool paused_ = false;  // the last pass found the UART taken (bridge or flasher)
  bool stats_asked_ = false;
  uint8_t stats_seq_ = 0;
  uint64_t next_stats_us_ = 0;
};

}  // namespace kilo

#endif
