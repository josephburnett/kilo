// guard: the distance guard (docs/distance-guard.md). Kilo reads the
// ultrasonic range on his head; he reacts to what comes close in every state
// but light sleep and a laptop holding him. The eyes show the zone: green
// clear, blue too close, red very close.
//
// 1. The filter (DistanceFilter). The body measures every 80 ms (12.5 Hz,
//    TELEMETRY flag DIST_FRESH). Only fresh readings count. No echo (0) or a
//    reading beyond kMaxRangeMm is not a reading; a reading at or beyond
//    where the beam meets the floor is the floor, not a reading: with the
//    beam tilted down more than kFloorTiltDeg (the head held down plus the
//    body's nose-down pitch) the floor is kSensorHeightMm / sin(tilt) away,
//    and readings from kFloorShare of that on are dropped (a hand closer
//    than the floor still counts); lying in REST or LIMP every reading is
//    the floor. The filtered range is the median of the readings of the last
//    kWindowS (at most 3), and needs two of them: one short echo never acts.
//    Without a reading for kStaleS there is no range (green), except right
//    after something very close (under kRedMm): then no echo means
//    closer than the sensor measures, and the range holds for kCloseHoldS.
// 2. Zones with hysteresis. Two sets of thresholds, each tracked with its
//    own hysteresis on every reading:
//    - walking forward: RED below kRedMm (left above kRedExitMm); BLUE
//      below blue_mm(v) (left above it + kBlueExitMm), v the forward walk's
//      speed: the gap left after the stop (kStopGapMm), the way travelled
//      while the reading comes in and the quick stop starts (v x kReactS),
//      and the quick stop's own way (v^2 / 2a, a by gait: stop_way_mm()).
//      Never below kBlueMinMm. These hold through the guard's own stop and
//      back-off of a forward walk (at the speed it had when the guard acted);
//    - not walking forward (standing, sitting, turning in place, walking
//      backward, after the walk has ended): closer, he does not move into
//      it. BLUE below kBlueStillMm (left above it + kBlueStillExitMm), RED
//      below kRedStillMm (left above kRedStillExitMm). The sensor reads down
//      to about 30 mm (lying it reads the floor at 55-63 mm), so RED at 80 mm
//      is read, not guessed.
//    The zone in force (zone(), the eyes, the reactions) is the walking one
//    while walking forward, else the still one; the walking one at the
//    speed held (kBlueMinMm when none) is also kept while still (walk_zone()):
//    a forward key is refused while it is not green (refuses_forward()), so a
//    walk never starts into something it would stop for at once, and after a
//    stop for something ahead the hold lasts until the way is clear for
//    walking, while he may stand (B6) and stays standing.
// 3. What he does (poll()) in the zone in force, with the guard on and
//    nothing in the way (GuardHost::not_now(): a laptop holds the body, a
//    trick, picked up, light sleep):
//    - walking forward (any gait, also steering): BLUE: the quick stop
//      (Walker::halt), then sit and the behaviour runner's sitting variant
//      (GuardHost::sit_and_behave). RED: back off (the walk turns backward
//      at kBackSpeedMmS for kBackS, about kBackMm; no rear sensor, so short
//      and slow), the quick stop, then as BLUE; the crawl and the long trot
//      only stop (their strides turn round at 0.5 and 0.75 mm a tick: 4 s
//      and 1.5 s before they would go back; the long trot's quick stop has
//      him still in about 0.6 s). The long trot's BLUE is the trot's
//      (stop_way_mm: its quick stop moves him 10 mm at most in the planner,
//      16 in the virtual body, as the trot at 187 to 230 mm/s);
//    - turning in place: RED only (a hand in his face): the quick stop, then
//      sit (a wall seen while turning is not a reason to stop turning);
//    - walking backward: nothing (he is going away; the eyes show it);
//    - standing still: something enters BLUE: he looks at it and sits; RED:
//      a short back-off (as above) and sits;
//    - sitting: BLUE: he looks at it; RED: he leans back (GuardHost::flinch);
//    - lying (REST, after A7): the eyes only.
//    Every RED reaction hisses (hisses()). Then HOLDING: forward keys are
//    refused (refuses_forward()); turning, backward and the posture keys
//    work, and B6 next to what stopped him stands him up for good (only a
//    new entry into the zone in force makes him react). It releases after
//    kClearS of green in the walking zone.
//    The release rule: at most kMaxBacks RED reactions (backing off,
//    leaning back, a stopped turn) until the walking zone has been green
//    for kClearS; after that only the eyes. A hand reaching for him or the floor
//    seen with the head down never backs him off over and over.
// The eyes show the zone; while the zone is green and Kilo is tired they
// are a dim orange instead (the Mind decides: set_tired()).
// The log and the flight recorder: every zone change and action is a line
// and a MARK ("guard zone=blue mm=287 ...", "guard action=stopping ..."),
// and every change of what the guard makes of it (the state Kilo is in and
// why it acts or not) one compact line: "guard decides: zone=red mm=89
// raw=91 seen=reading state=sitting: lean back" or "... state=lying: eyes
// only (lying: the floor)"; the counters in the metrics line "guard:".
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
constexpr double kFloorTiltDeg = KILO_MODEL_SENSOR_ULTRASONIC_FLOOR_TILT_DEG;  // beam tilted down more than this
                                          //   (12): the floor is in view (the body model, spec [model.sensor])
constexpr double kSensorHeightMm = KILO_MODEL_SENSOR_ULTRASONIC_HEIGHT_MM;  // the sensor above the floor, standing
                                          //   (80, a little low: the floor nearer; spec [model.sensor])
constexpr double kFloorShare = 0.75;      // ... readings from this share of the floor's distance on are the floor
constexpr double kCloseHoldS = 1.0;       // no echo right after something very close: the range holds this long
constexpr double kStopGapMm = 100;        // left between the sensor and the obstacle after the stop
constexpr double kReactS = 0.40;          // reading (80 ms + the median's 80 ms), link, the re-planned FEET
                                          //   (40 ms), the servos (120 ms), and a margin
constexpr double kDecelWalkMmS2 = 400;    // the quick stop's deceleration, by gait: its way (the support feet's
constexpr double kDecelTrotMmS2 = 700;    //   travel from the halt; test_guard prints it: walk 72.5 mm/s 4 mm, trot
                                          //   230 13 mm) at least doubled: the floor may need the room
constexpr double kStopSlackMm = 5;        // ... plus this (the swing that lands, at any speed)
constexpr double kCrawlStopMm = 20;       // the crawl's: its lean (up to 18 mm fore and aft) as the legs square up
constexpr double kBlueMinMm = 200;        // walking forward: BLUE never below this (forward keys need this clear)
constexpr double kBlueExitMm = 80;        // BLUE is left this far above its threshold
constexpr double kRedMm = 150;            // walking forward: RED, back off
constexpr double kRedExitMm = 200;
constexpr double kBlueStillMm = 120;      // not walking forward: BLUE below this (2026-11-21: 190 mm sat him down
constexpr double kBlueStillExitMm = 40;   //   after B6 each time), left this far above it
constexpr double kRedStillMm = 80;        // not walking forward: RED below this (the sensor reads to about 30 mm)
constexpr double kRedStillExitMm = 120;
constexpr double kBackSpeedMmS = 40;      // backing off: slow ...
constexpr double kBackS = 2.0;            // ... and short (the reversal and about kBackMm)
constexpr double kBackMm = 50;
constexpr double kClearS = 1.0;           // green this long releases the hold
constexpr int kMaxBacks = 2;              // RED reactions until green for kClearS; then the eyes only
// The eyes (SET_EYES all six LEDs; the legacy firmware's dim 0x22 levels, a little brighter).
constexpr uint8_t kEyeLevel = 0x30;
// Orange (low battery): red at kEyeLevel, green a third of it, no blue.
constexpr uint8_t kOrangeGreen = kEyeLevel / 3;

enum class Zone : uint8_t { Green = KILO_GUARD_ZONE_GREEN, Blue = KILO_GUARD_ZONE_BLUE, Red = KILO_GUARD_ZONE_RED };
enum class Action : uint8_t {
  Watch = KILO_GUARD_ACTION_WATCH,
  Stopping = KILO_GUARD_ACTION_STOPPING,
  Backing = KILO_GUARD_ACTION_BACKING,
  Holding = KILO_GUARD_ACTION_HOLDING,
};
enum class Eyes : uint8_t {
  Green = KILO_GUARD_EYES_GREEN,
  Blue = KILO_GUARD_EYES_BLUE,
  Red = KILO_GUARD_EYES_RED,
  Orange = KILO_GUARD_EYES_ORANGE,
};
const char* zone_name(Zone z);
const char* eyes_name(Eyes e);
const char* action_name(Action a);

// The quick stop's way at v mm/s in this gait (kStopSlackMm + v^2 / 2a; the crawl kCrawlStopMm).
double stop_way_mm(const char* gait, double v_mm_s);
// BLUE's threshold at v (0: not walking forward).
double blue_mm(const char* gait, double v_mm_s);

// Where the beam meets the floor (1. above): 0 when it does not (tilted down
// kFloorTiltDeg or less); readings from the return on are the floor.
double floor_from_mm(double tilt_down_deg);

// What the last fresh reading was.
enum class Seen : uint8_t { None, Reading, NoEcho, TooFar, Floor };
const char* seen_name(Seen s);

// Step 1 above.
class DistanceFilter {
 public:
  // One TELEMETRY: its range, whether it is a new reading, and from where on
  // a reading is the floor (0: no floor in view; < 0: every reading). t in
  // seconds.
  void feed(double t, uint16_t raw_mm, bool fresh, double floor_mm);
  bool has() const { return has_; }
  double mm() const { return mm_; }
  uint16_t raw() const { return raw_; }
  uint32_t readings() const { return readings_; }
  uint32_t dropouts() const { return dropouts_; }
  uint32_t floor_ignored() const { return floor_; }
  Seen last() const { return last_; }
  bool close_hold() const { return hold_; }  // no echo after something very close: holding the range

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
  Seen last_ = Seen::None;
  bool hold_ = false;
  double hold_until_ = 0;
};

// Step 2: the zone for a filtered range (has false: none) at thresholds blue, red; BLUE is left
// above blue + blue_exit, RED above red_exit.
Zone next_zone(Zone now, bool has, double mm, double blue, double red, double blue_exit = kBlueExitMm,
               double red_exit = kRedExitMm);

// Kilo, when the walker is idle (GuardHost::still()).
enum class Still : uint8_t { None, Standing, Sitting, Lying };
const char* still_name(Still s);

// What the guard asks of the Mind.
class GuardHost {
 public:
  // Sit and start the behaviour runner's sitting variant.
  virtual void sit_and_behave() {}
  // Why the guard must not act now (a laptop holds the body, a trick plays,
  // picked up, light sleep), or null.
  virtual const char* not_now() { return nullptr; }
  // How Kilo is when the walker is idle (None: between postures, moving).
  virtual Still still() { return Still::None; }
  // BLUE while still: look straight at it (the head ahead).
  virtual void notice() {}
  // RED while sitting: lean back. False if it could not.
  virtual bool flinch() { return false; }
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

  // The guard holds or acts (not watching).
  bool holds() const { return on_ && action_ != Action::Watch; }
  // A remote key would walk forward now (remote.h's rules): refused while
  // the guard holds or acts, or the walking zone is not green (something
  // closer than a forward walk would stop for). why: the reason for the log
  // ("holding: blue, 110 mm ahead", "175 mm ahead, a forward walk needs 200").
  bool refuses_forward(char* why, uint32_t n) const;

  Zone zone() const { return zone_; }       // the zone in force (the eyes, the reactions)
  Zone walk_zone() const { return walk_zone_; }  // the zone at the walking thresholds
  bool walking_thresholds() const { return walking_; }
  Action action() const { return action_; }
  const DistanceFilter& filter() const { return filter_; }
  double blue_now() const { return blue_; }  // the thresholds in force
  double red_now() const { return red_; }
  double speed_now() const { return v_; }
  // Low battery (the Mind's rule): orange eyes while the zone is green.
  void set_tired(bool tired) { tired_ = tired; }
  bool tired() const { return tired_; }
  // The eyes: the colour asked for, and whether the body takes SET_EYES.
  Eyes eyes() const { return eyes_; }
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
  uint32_t hisses() const { return hisses_; }  // RED reactions (the Mind hisses once for each)
  uint32_t reactions() const { return reactions_; }  // reactions while still or turning (looks, sits, leans back)
  int backs() const { return backs_; }  // RED reactions since the last release (kMaxBacks)
  const char* decision() const { return decided_; }  // the last "guard decides" line's state and reason

 private:
  enum class Walk : uint8_t { None, Forward, Turn, Back, Other, Stopping };
  Walk walk_kind(double* v, const char** gait) const;
  bool forward_walk(double* v, const char** gait) const;
  void quick_stop(uint64_t now, const char* why);
  void back_off(uint64_t now, const char* why);
  bool react_still(uint64_t now, Still s);
  void decide(const char* state, const char* reason);
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
  Zone zone_ = Zone::Green;       // in force: walk_zone_ or still_zone_
  Zone walk_zone_ = Zone::Green;  // at the walking thresholds (blue_walk_, kRedMm)
  Zone still_zone_ = Zone::Green; // at the still thresholds
  bool walking_ = false;          // the walking thresholds are in force
  Action action_ = Action::Watch;
  double blue_ = kBlueStillMm, red_ = kRedStillMm;  // in force
  double blue_walk_ = kBlueMinMm;
  double v_ = 0;           // the forward walk's speed (held through the guard's actions)
  const char* gait_ = "";  // ... and its gait
  uint64_t action_at_ = 0;
  uint64_t deadline_ = 0;
  bool back_started_ = false;  // Backing: the walk goes backward now (kBackS runs from then)
  uint64_t green_since_ = 0;
  bool green_ = true;
  Eyes eyes_ = Eyes::Green;
  bool tired_ = false;
  bool eyes_sent_ = false;
  uint32_t zone_changes_ = 0, stops_ = 0, pullbacks_ = 0, refused_ = 0, releases_ = 0;
  uint32_t hisses_ = 0, reactions_ = 0;
  int backs_ = 0;
  bool blue_new_ = false, red_new_ = false;  // the zone entered BLUE (from green) or RED: not yet reacted to
  char decided_[80] = {};  // the last decision line's state and reason
  bool lying_ = false;
  double floor_mm_ = 0;
  double min_mm_ = 0;  // the closest filtered range while walking forward
  bool has_min_ = false;
};

}  // namespace guard
}  // namespace kilo

#endif
