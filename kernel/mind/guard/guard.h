// guard: the distance guard (docs/distance-guard.md). Kilo reads the
// ultrasonic range on his head; when a forward walk brings him too close to
// something he stops and sits, and when he is very close he backs off a
// little first. The eyes show the zone: green clear, blue too close, red
// very close.
//
// 1. The filter (DistanceFilter). The body measures every 80 ms (12.5 Hz,
//    TELEMETRY flag DIST_FRESH; measured in the recordings of 2026-11-17).
//    Only fresh readings count. No echo (0), a reading beyond kMaxRangeMm,
//    or one taken with the beam tilted down at the floor (the head held
//    tilted down plus the body's nose-down pitch over kFloorTiltDeg, or
//    Kilo lying in REST or LIMP with the head down) is not a reading. The filtered range is the median of the readings of the
//    last kWindowS (at most 3), and needs two of them: one short echo (a
//    hand waved past, a reflection) never stops Kilo. Without a reading
//    for kStaleS there is no range: the zone is green (nothing seen).
// 2. Zones with hysteresis. RED below kRedMm (left above kRedExitMm); BLUE
//    below blue_mm(v) (left above it + kBlueExitMm), where v is the forward
//    walk's speed: the gap left after the stop (kStopGapMm), the way
//    travelled while the reading comes in and the servos follow
//    (v x kReactS), and the planned stop's own way (v^2 / 2a, a by gait,
//    from the planner: stop_way_mm()). Never below kBlueMinMm (also the
//    threshold standing still, for the eyes).
// 3. Actions, only for a forward walk the mind drives (the remote's or the
//    guard's own; not with a laptop holding control, during a trick, or
//    with Kilo picked up), and only with the guard on:
//    BLUE: the planned stop (Walker::stop), then sit and start the
//    behaviour runner's sitting variant (GuardHost::sit_and_behave).
//    RED: back off: the walk turns backward at kBackSpeedMmS for kBackS
//    (about kBackMm; no rear sensor, so short and slow), then the planned
//    stop, then as BLUE. RED during BLUE's stop backs off too.
//    Then HOLDING: forward keys are refused (refuses()); turning, backward
//    and the posture keys work. It releases after kClearS of green.
// Every zone change and action is a log line and a MARK record in the
// flight recorder ("guard zone=blue mm=287 blue=316 red=150 v=180 ...",
// "guard stop ..."), and counted (metrics line "guard:").
#ifndef KILO_MIND_GUARD_GUARD_H
#define KILO_MIND_GUARD_GUARD_H
#include <stdint.h>
#include "link/serial_link.h"
#include "planner/walker.h"
#include "platform.h"
#include "proto/kilo_protocol.h"

namespace kilo {
namespace guard {

// ---- the thresholds, all in one place --------------------------------------------------
constexpr bool kDefaultOn = true;         // the kernel's default (kilo guard on|off --host)
constexpr double kMaxRangeMm = 2500;      // beyond: not a reading (the recordings: echoes to 3.4 m, outliers)
constexpr double kWindowS = 0.30;         // readings this recent make the median (3 at 12.5 Hz)
constexpr double kStaleS = 0.40;          // no reading for this long: no range (green)
constexpr double kFloorTiltDeg = 12;      // beam tilted down more than this: the floor, not a reading
constexpr double kStopGapMm = 100;        // left between the sensor and the obstacle after the stop
constexpr double kReactS = 0.45;          // reading (80 ms + the median's 80 ms), link, FEET queued ahead (100 ms), servo lag (120 ms)
constexpr double kDecelWalkMmS2 = 75;     // the planned stop's deceleration, by gait (planner, 2026-11-17:
constexpr double kDecelTrotMmS2 = 120;    //   walk 80-97 mm/s stops in 39-57 mm, trot 144-184 in 79-127 mm,
constexpr double kDecelCrawlMmS2 = 8;     //   crawl 22-27 in 23-30 mm), rounded down for a margin. Since the
                                          //   1.5 mm/tick stride slew the walk and trot stop in a third of
                                          //   that (trot 230 in 66 mm; test_guard prints it): kept, the safe side
constexpr double kBlueMinMm = 200;        // BLUE never below this (and standing still)
constexpr double kBlueExitMm = 80;        // BLUE is left this far above its threshold
constexpr double kRedMm = 150;            // RED: back off
constexpr double kRedExitMm = 200;
constexpr double kBackSpeedMmS = 40;      // backing off: slow ...
constexpr double kBackS = 2.0;            // ... and short (the reversal and about kBackMm)
constexpr double kBackMm = 50;
constexpr double kClearS = 1.0;           // green this long releases the hold
// The eyes (SET_EYES all six LEDs; the legacy firmware's dim 0x22 levels, a little brighter).
constexpr uint8_t kEyeLevel = 0x30;

enum class Zone : uint8_t { Green = KILO_GUARD_ZONE_GREEN, Blue = KILO_GUARD_ZONE_BLUE, Red = KILO_GUARD_ZONE_RED };
enum class Action : uint8_t {
  Watch = KILO_GUARD_ACTION_WATCH,
  Stopping = KILO_GUARD_ACTION_STOPPING,
  Backing = KILO_GUARD_ACTION_BACKING,
  Holding = KILO_GUARD_ACTION_HOLDING,
};
const char* zone_name(Zone z);
const char* action_name(Action a);

// The planned stop's way at v mm/s in this gait (v^2 / 2a).
double stop_way_mm(const char* gait, double v_mm_s);
// BLUE's threshold at v (0: not walking forward).
double blue_mm(const char* gait, double v_mm_s);

// Step 1 above.
class DistanceFilter {
 public:
  // One TELEMETRY: its range, whether it is a new reading, and whether the
  // beam pointed at the floor then. t in seconds.
  void feed(double t, uint16_t raw_mm, bool fresh, bool floor);
  bool has() const { return has_; }
  double mm() const { return mm_; }
  uint16_t raw() const { return raw_; }
  uint32_t readings() const { return readings_; }
  uint32_t dropouts() const { return dropouts_; }
  uint32_t floor_ignored() const { return floor_; }

 private:
  struct Pt {
    double t, mm;
  };
  Pt pts_[3] = {};
  int n_ = 0;
  bool has_ = false;
  double mm_ = 0;
  uint16_t raw_ = 0;
  uint32_t readings_ = 0, dropouts_ = 0, floor_ = 0;
};

// Step 2: the zone for a filtered range (has false: none) at thresholds blue, red.
Zone next_zone(Zone now, bool has, double mm, double blue, double red);

// What the guard asks of the Mind.
class GuardHost {
 public:
  // Sit and start the behaviour runner's sitting variant.
  virtual void sit_and_behave() {}
  // A forward walk the guard may act on: the Mind's (remote) walk, no
  // laptop in control, no trick, not picked up.
  virtual bool may_act() { return true; }
  // A MARK line into the flight recorder.
  virtual void mark(const char* /*line*/) {}

 protected:
  ~GuardHost() = default;
};

class DistanceGuard {
 public:
  DistanceGuard(SerialLink& link, planner::Walker& walker, Clock& clock, Log& log);
  void set_host(GuardHost* host) { host_ = host; }
  void set_enabled(bool on);
  bool enabled() const { return on_; }

  // Every TELEMETRY (after the walker has seen it). head_tilt_deg: the
  // tilt the head is held at (0 when not held; - = down). lying: Kilo lies
  // with the head down (REST or LIMP, as after A7 and in light sleep): the
  // beam reads the floor, so no reading counts.
  void on_telemetry(const kilo_telemetry_t& t, double head_tilt_deg, bool lying = false);
  // Every pass: the actions' steps and the eyes' goal.
  void poll();

  // A remote key would walk forward now (remote.h's rules): refused while
  // the guard holds or acts. why: the reason for the log.
  bool holds() const { return on_ && action_ != Action::Watch; }

  Zone zone() const { return zone_; }
  Action action() const { return action_; }
  const DistanceFilter& filter() const { return filter_; }
  double blue_now() const { return blue_; }
  double speed_now() const { return v_; }
  // The eyes: the colour asked for, and whether the body takes SET_EYES.
  Zone eyes() const { return eyes_; }
  bool eyes_driven() const;
  void fill(kilo_host_guard_status_t& s) const;
  // One metrics line ("guard: on=1 zone=green ...").
  void metrics_line(char* out, uint32_t n) const;
  // A remote key the guard let through (B6, A7, C7, a turn, backward):
  // the stop or the backing it was doing is the key's now; it keeps holding.
  void key_took_over();
  // A forward key was refused (the Mind logs it): counted.
  void count_refused() { refused_++; }
  uint32_t refused() const { return refused_; }
  uint32_t stops() const { return stops_; }
  uint32_t pullbacks() const { return pullbacks_; }

 private:
  bool forward_walk(double* v, const char** gait) const;
  void set_action(Action a, const char* why);
  void note(const char* what);
  void eyes_step();

  SerialLink& link_;
  planner::Walker& walker_;
  Clock& clock_;
  Log& log_;
  GuardHost* host_ = nullptr;
  bool on_ = kDefaultOn;
  DistanceFilter filter_;
  Zone zone_ = Zone::Green;
  Action action_ = Action::Watch;
  double blue_ = kBlueMinMm;
  double v_ = 0;           // the forward walk's speed (held through the guard's actions)
  const char* gait_ = "";  // ... and its gait
  uint64_t action_at_ = 0;
  uint64_t deadline_ = 0;
  bool back_started_ = false;  // Backing: the walk goes backward now (kBackS runs from then)
  uint64_t green_since_ = 0;
  bool green_ = true;
  Zone eyes_ = Zone::Green;
  bool eyes_sent_ = false;
  uint32_t zone_changes_ = 0, stops_ = 0, pullbacks_ = 0, refused_ = 0, releases_ = 0;
  double min_mm_ = 0;  // the closest filtered range while walking forward
  bool has_min_ = false;
};

}  // namespace guard
}  // namespace kilo

#endif
