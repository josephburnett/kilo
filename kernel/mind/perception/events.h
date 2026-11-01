// events: one timestamped percept event from a detector. Python
// counterpart: kilo/perception/events.py (PerceptEvent), whose free-form
// data dict becomes a few named numbers per kind.
#ifndef KILO_MIND_PERCEPTION_EVENTS_H
#define KILO_MIND_PERCEPTION_EVENTS_H
#include <stdint.h>

namespace kilo {
namespace perception {

enum class Percept : uint8_t {
  Near,         // a: distance_mm
  Approaching,  // a: distance_mm, b: speed_mm_s (closing)
  Clear,        // a: distance_mm; no_echo: true when it is the sustained no-echo kind (a unset)
  Tilted,       // direction; a: pitch, b: roll
  Level,        // a: pitch, b: roll
  PickedUp,     // a: pitch, b: roll; servos_powered
  PutDown,      // a: pitch, b: roll
  Tap,          // a: onset (s), b: peak_deg, c: duration_s
  Pet,          // a: pp_deg, b: strokes
  Startle,      // a: delta_deg (has_a), b: rate_dps (has_b)
  Count
};

enum class TiltDirection : uint8_t { None, NoseDown, NoseUp, RightUp, LeftUp };

const char* percept_name(Percept p);              // "near", "picked_up", ... (Python's names)
const char* tilt_direction_name(TiltDirection d);  // "nose_down", ...; "" for None

struct PerceptEvent {
  double t;  // seconds, the caller's time base
  Percept kind;
  TiltDirection direction;
  bool has_a, has_b;
  bool no_echo;
  bool servos_powered;
  double a, b, c;
};

// The events one update produced (at most a zone, an approach, a tilt, a
// pickup, a startle, a tap and a pet).
struct PerceptEvents {
  static constexpr int kMax = 8;
  PerceptEvent e[kMax];
  int n = 0;
  PerceptEvent& add(double t, Percept kind) {
    PerceptEvent& x = e[n < kMax ? n++ : kMax - 1];
    x = PerceptEvent();
    x.t = t;
    x.kind = kind;
    return x;
  }
};

}  // namespace perception
}  // namespace kilo

#endif
