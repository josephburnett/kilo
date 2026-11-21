// sleep: Kilo's light sleep (docs/power.md). Two parts, on different clocks:
//
//   the servos  off (SET_POSTURE LIMP: every servo output off, the fall
//               reflex off too: he is inert) as soon as he lies still after
//               the remote's A7: the remote's own A7 action (its planned
//               stop, then REST) is done, the body is in no moving mode, and
//               kLimpSettleUs have passed with nothing moving. A laptop
//               connected or asking does not hold this back. The body keeps
//               running: its 50 Hz loop, the IMU, the IR receiver (keys
//               still arrive as EVENT IR_KEY), TELEMETRY at 25/s (the battery
//               trend comes from it), the link watchdog (the mind's heartbeat
//               keeps going). He stays limp until a key that moves him.
//   the Pi      if then nothing happens for kSleepAfterUs (no key, no laptop
//               connected or asking, nothing moving), the lowest power state
//               from which the remote and the laptop still wake him: the
//               platform's main loop idles between passes (Circle: wfi and
//               one pass per kSleepLoopUs instead of a busy 1 ms loop; the
//               loop reads asleep()); the ACT LED off but a blink every 5 s;
//               the Ethernet PHY in its energy-detect power-down while no
//               cable is in (below). A laptop needs the full-speed loop, so
//               this part waits while one is connected. What it cannot switch
//               is in docs/power.md "What light sleep cannot switch off".
//
// He also goes to sleep on his own: awake, lying in REST with the servos
// powered (Activity::lying_rest), and kIdleSleepUs with no remote key at
// all, nothing busy, the body in no moving mode and no laptop holding the
// body (a laptop merely connected or polling does not count): exactly as
// after A7, Drowsy with the servos off at once. A standing Kilo is never
// laid down by this.
//
// States:
//
//   Awake    the default; A7 (a remote key the mind acts on), or kIdleSleepUs
//            lying idle in REST -> Drowsy
//   Drowsy   A7 stands: the servos go off once he lies still (above), and
//            kSleepAfterUs of nothing happening are counted. Any activity
//            restarts the count: a key that does nothing (rows 1-3, B2's
//            meow, a held key's repeats, another remote's key, C7, B5/B7
//            while not walking), a laptop connecting or sending a request, a
//            client connected at all, the walker, a trick, the remote's own
//            action (A7's planned stop), the behaviour runner, the body in
//            GAIT, TRICK, FALLEN, RECOVERING, CALIB or BOOT. None of these
//            powers the servos again. A key that moves Kilo (B6, C5, A4-C4,
//            A6, C6, A5, A2) cancels: back to Awake, and for a walk, a turn,
//            a trick or the behaviour runner the servos first get REST again
//            if they were off (a trick refuses to start from LIMP); a posture
//            key goes there directly. A second A7: the remote rests him again
//            (servos on), so the limp follows again after the settle. A
//            laptop posing him (a posture goal other than REST or LIMP on
//            record while nothing is busy) also calls it off. When the count
//            runs out and the card saver is not writing (it is asked to save
//            what it holds first: "save now", when saving is on), -> Asleep.
//   Asleep   light sleep. Any key or laptop activity wakes the Pi (back to
//            Drowsy: A7 still stands, the servos stay off, so 30 s after the
//            last activity he sleeps again); a key that moves him wakes him
//            to Awake, as from Drowsy.
//
// The sound (Mind::sound_events) follows Asleep only: the output stops on
// Asleep and the wake's mew plays when he leaves it, never for the servos
// alone.
//
// Waking is one pass: the next pass of the loop is a full one again (the
// platform reads asleep()), so a key acts within one sleep period of its
// EVENT (10 ms) plus the body's IR decode, as when awake.
//
// The network (Hardware::net_*): while asleep with an Ethernet link, the
// network stays up (the laptop fetches recordings over the cable). With no
// link the PHY goes to its energy-detect power-down, where it wakes by
// itself when a cable brings a signal; because a laptop's port may be in
// the same mode (two such ports never see each other), every kNetRecheckUs
// the PHY is fully on for kNetLookUs, sending link pulses. A link found
// keeps it on. Waking puts it back to normal.
//
// Off (set_enabled(false), KILO_LIGHT_SLEEP=0): none of it, the servo part
// included: A7 is plain REST and he never limps on his own (the awake
// baseline of docs/power.md). The battery trend is kept either way.
//
// The flight recorder records all of it (the log lines below go into it;
// every TELEMETRY too), and the metrics lines (metric_line) give the state,
// the servos (servos=on|off from TELEMETRY, limps= the LIMPs sent),
// the time in each state, the counts and the battery trend (battery.h):
// `kilo power --host`, `kilo metrics --host`.
#ifndef KILO_MIND_POWER_SLEEP_H
#define KILO_MIND_POWER_SLEEP_H
#include <stdint.h>
#include "link/serial_link.h"
#include "planner/walker.h"
#include "platform.h"
#include "power/battery.h"
#include "record/saver.h"

namespace kilo {
namespace power {

constexpr uint64_t kSleepAfterUs = 30000000;   // A7, then nothing for this long: light sleep
// After A7, nothing moving for this long: the servos off. The remote's REST is a 0.5 s move
// (SET_POSTURE's default POSTURE_TICKS, 25 ticks of 20 ms), the body's own REST 1 s
// (REST_MOVE_TICKS, 50 ticks); the servos need a moment more to arrive.
constexpr uint64_t kLimpSettleUs = 3000000;
constexpr uint64_t kIdleSleepUs = 120000000;   // awake, lying in REST with nothing happening this long: as after A7
constexpr uint32_t kSleepLoopUs = 10000;       // the platform loop's period asleep (awake: 1 ms)
constexpr uint64_t kNetRecheckUs = 10000000;   // asleep with no link: a look for one this often
constexpr uint64_t kNetLookUs = 3000000;       // ... the PHY fully on this long
constexpr uint64_t kFlushWaitUs = 20000000;    // the card saver's save before sleep: waited for at most this long
constexpr uint64_t kSocPeriodUs = 10000000;    // the SoC's clock and temperature read this often

enum class State : uint8_t { Awake = 0, Drowsy = 1, Asleep = 2 };
const char* state_name(State s);
// The PHY: normal, energy-detect power-down, normal for a look (asleep, no link).
enum class Net : uint8_t { On = 0, Save = 1, Look = 2 };
const char* net_name(Net n);

// The SoC as the platform reads it (Circle: the property mailbox).
struct Soc {
  uint32_t arm_hz = 0;      // 0: unknown
  int32_t temp_mc = 0;      // millidegrees C
  uint32_t throttled = 0;   // GET_THROTTLED's bits (under-voltage now/since boot: 0x1 / 0x10000)
};

// What the platform can switch. Not pure (a pure virtual needs
// __cxa_pure_virtual, behaviour/tree.h): the defaults are a platform with
// no network to save (kilo-mind-host).
class Hardware {
 public:
  // There is a network device whose PHY net_save() can set (the Pi: the
  // LAN9514, once the USB scan found it).
  virtual bool net_device() { return false; }
  // An Ethernet link is up.
  virtual bool net_link() { return true; }
  // The PHY's energy-detect power-down on or off: false if it failed.
  virtual bool net_save(bool /*on*/) { return false; }
  // The SoC's clock, temperature, throttling; false if unknown.
  virtual bool soc(Soc* /*out*/) { return false; }

 protected:
  ~Hardware() = default;
};

// What the Mind tells it each pass.
struct Activity {
  bool busy = false;          // the walker, a trick, the remote's action, the behaviour runner
  uint32_t host_events = 0;   // the host server's connections + requests so far: a change is laptop activity
  uint32_t host_clients = 0;  // clients connected now
  bool host_holds = false;    // a laptop client holds (controls) the body
  bool lying_rest = false;    // lying in REST, the servos powered (Mind::lying_rest)
};

class Sleep {
 public:
  Sleep(SerialLink& link, planner::Walker& walker, Clock& clock, Log& log);
  // Off (the default): it only keeps the battery trend; A7 is just rest, no limp.
  void set_enabled(bool on) { enabled_ = on; }
  bool enabled() const { return enabled_; }
  void set_hardware(Hardware* hw) { hw_ = hw; }
  // The card saver: asked to save now before sleep, waited for.
  void set_saver(record::Saver* s) { saver_ = s; }

  // An IR key (EVENT IR_KEY arg), before the remote acts on it. acts: the
  // mind acts on remote keys now (the remote on, no laptop holding the body).
  void key(uint16_t arg, bool acts);
  void on_telemetry(const kilo_telemetry_t& t);
  void poll(const Activity& a);

  State state() const { return state_; }
  bool asleep() const { return state_ == State::Asleep; }
  // The platform loop's period now: kSleepLoopUs asleep, 0 (its own) otherwise.
  uint32_t loop_period_us() const { return asleep() ? kSleepLoopUs : 0; }
  Net net() const { return net_; }
  uint32_t sleeps() const { return sleeps_; }
  // The LIMPs sent (after A7, lying idle, or at sleep), and the sleeps begun by lying idle.
  uint32_t limps() const { return limps_; }
  uint32_t idle_sleeps() const { return idle_sleeps_; }
  // The servos off by light sleep's LIMP (sent, and no key or posture since).
  bool limp() const { return limp_; }
  uint32_t wakes() const { return wakes_; }
  // Time in each state so far, the running one included.
  uint64_t time_in_us(State s) const;
  const BatteryTrend& battery() const { return battery_; }
  // The battery at rest (battery.h RestedBattery), from every TELEMETRY.
  const RestedBattery& rested() const { return rested_; }
  bool rested_mv(uint16_t* mv) const { return rested_.mv(clock_.now_us(), mv); }
  const char* last_wake() const { return last_wake_; }

  // The metrics lines (`kilo power --host`): "power: ...", "power_hw: ...", "battery: ...",
  // "power_segments: ...".
  static constexpr uint32_t kMetricLines = 4;
  void metric_line(uint32_t i, char* out, uint32_t n) const;

 private:
  void set_state(State s, uint64_t now);
  void enter(uint64_t now);
  void wake(State to, const char* why, uint64_t now);
  bool send_limp(const char* why);
  void call_off(const char* why, uint64_t now);
  bool posed_elsewhere();
  void network(uint64_t now);
  void net_set(Net n, uint64_t now);
  bool moving_mode() const;

  SerialLink& link_;
  planner::Walker& walker_;
  Clock& clock_;
  Log& log_;
  Hardware* hw_ = nullptr;
  record::Saver* saver_ = nullptr;
  bool enabled_ = false;

  State state_ = State::Awake;
  uint64_t since_us_ = 0;       // in this state since
  uint64_t quiet_since_us_ = 0; // Drowsy: nothing has happened since
  uint64_t time_us_[3] = {};    // finished stretches per state
  uint32_t host_events_ = 0;
  bool host_seen_ = false;      // host_events_ was set once
  bool flush_asked_ = false;
  uint64_t flush_at_us_ = 0;
  bool limp_pending_ = false;    // Drowsy: A7's LIMP to send once he lies still
  bool limp_ = false;            // the LIMP sent, no key or posture since
  uint64_t still_since_us_ = 0;  // limp_pending_: nothing moving since
  bool idle_on_ = false;         // Awake: lying idle in REST since idle_since_us_
  uint64_t idle_since_us_ = 0;
  uint32_t limps_ = 0, idle_sleeps_ = 0;
  uint32_t sleeps_ = 0, wakes_ = 0, flushes_ = 0;
  char last_wake_[32] = "-";

  Net net_ = Net::On;
  bool net_device_ = false;     // the last pass's Hardware::net_device()
  uint64_t net_next_us_ = 0;    // Save: the next look; Look: its end
  uint32_t net_saves_ = 0, net_looks_ = 0, net_errors_ = 0;

  Soc soc_;
  bool soc_ok_ = false;
  uint64_t soc_next_us_ = 0;

  BatteryTrend battery_;
  RestedBattery rested_;
};

}  // namespace power
}  // namespace kilo

#endif
