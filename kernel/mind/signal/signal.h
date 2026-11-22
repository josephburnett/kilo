// signal: what Kilo tells the owner. The eyes first (the six lights of the
// distance sensor on his head), and a sound as well, which is heard only if
// a speaker is connected (none is, as of 2026-11-22). The eye language is
// in docs/signals.md; this is its one implementation.
//
// A named Signal is asked for at one call site (Mind::signal()), which plays
// its sound (signal_sound()) and hands it to the eyes here.
//
// The eyes, highest priority first (EyeLanguage::look()):
//   1. steady white: the body is about to be reprogrammed (Signal::Flashing,
//      until the next one-shot). The body cannot drive its lights while its
//      program is written, so the lights keep the last colour they were
//      sent: white, sent before the flasher takes the line;
//   2. the distance guard's blue and red, steady: what is in front of his
//      face always shows, also while walking (guard/guard.h);
//   3. a one-shot: three quick flashes (green: ready or done; purple:
//      refused; red: failed or gave up), or one long white flash (a run's
//      recording is on the SD card). One at a time; one more may wait (the
//      newer replaces it). A one-shot starts only while the guard is green
//      and is dropped if that has not happened within kWaitUs;
//   4. light sleep: dark, a short dim green blink every 5 s;
//   5. busy (a job he must not be disturbed in: settling for and running
//      the self-measurement, the righting experiment): white, blinking
//      slowly;
//   6. the guard's steady colour: green, or orange when he is tired.
//
// Sending (EyeDriver): SET_EYES only on a change of what the eyes should
// show (or when the link's goal is not it: a laptop set another), plus the
// link's own 5 Hz refresh. The body skips a SET_EYES that
// arrives while an echo is being timed (up to 25 ms in every 80, body/src/
// hal/eyes.h); so a change is sent again kRepeatUs and 2 x kRepeatUs later,
// so that a quick flash is not lost or stretched to the next refresh.
#ifndef KILO_MIND_SIGNAL_SIGNAL_H
#define KILO_MIND_SIGNAL_SIGNAL_H
#include <stdint.h>
#include "guard/guard.h"
#include "link/serial_link.h"

namespace kilo {
namespace signals {

enum class Signal : uint8_t {
  Ready,     // green x3, "mew": the body answered after power-on; awake again after light sleep
  Done,      // green x3, "trill": a job finished well (body flashed, measured, righted)
  Refused,   // purple x3, "hiss": not started (a key refused, a flash refused, the build lacks it)
  Failed,    // red x3, "hiss": it went wrong (the body flash failed, the measurement stopped itself)
  GaveUp,    // red x3, "meow": the righting gave up and left him limp on the floor
  Saved,     // one long white flash, no sound: a run's recording is on the SD card
  Flashing,  // steady white, no sound: the body is about to be reprogrammed from the SD card
  GuardRed,  // no pattern (the guard's own red shows), "hiss": the guard pulled him back
};
// "ready", "done", ...
const char* signal_name(Signal s);
// The sound for s (sound/ names), or null for none.
const char* signal_sound(Signal s);

// One SET_EYES: the LED mask and the colour.
struct Look {
  uint8_t mask, r, g, b;
};
bool same(const Look& a, const Look& b);
// "green", "white", "off", ... for the log and the tests ("other" for anything else).
const char* look_name(const Look& l);

constexpr uint8_t kLevel = guard::kEyeLevel;  // every colour's channels at this level
constexpr uint8_t kAll = 0x3F;                // all six LEDs
constexpr Look kOff = {kAll, 0, 0, 0};
constexpr Look kGreen = {kAll, 0, kLevel, 0};
constexpr Look kBlue = {kAll, 0, 0, kLevel};
constexpr Look kRed = {kAll, kLevel, 0, 0};
constexpr Look kOrange = {kAll, kLevel, guard::kOrangeGreen, 0};
constexpr Look kWhite = {kAll, kLevel, kLevel, kLevel};
constexpr Look kPurple = {kAll, kLevel, 0, kLevel};
constexpr Look kDimGreen = {kAll, 0, kLevel / 3, 0};  // light sleep's blink

// What the eyes show when no one-shot plays.
struct State {
  guard::Eyes guard;  // the guard's colour (green, orange, blue, red)
  bool asleep;        // light sleep (power/sleep.h Asleep)
  bool busy;          // a job: the slow white blink
};

class EyeLanguage {
 public:
  static constexpr uint64_t kFlashOnUs = 150000;   // a quick flash: on ...
  static constexpr uint64_t kFlashOffUs = 150000;  // ... then off
  static constexpr int kFlashes = 3;
  static constexpr uint64_t kSavedOnUs = 1000000;  // the long white flash ...
  static constexpr uint64_t kSavedOffUs = 300000;  // ... then dark
  static constexpr uint64_t kWaitUs = 3000000;     // a one-shot waits at most this long for the guard's green
  static constexpr uint64_t kBusyHalfUs = 500000;  // busy: half a second on, half a second off
  static constexpr uint64_t kSleepEveryUs = 5000000;  // light sleep: a blink every 5 s ...
  static constexpr uint64_t kSleepBlinkUs = 100000;   // ... this long

  // A signal at now: a one-shot (Ready .. Saved) queues; Flashing holds steady white until the next
  // one-shot; GuardRed changes nothing here.
  void show(Signal s, uint64_t now);
  // What the eyes show at now (advances the one-shots: call it every pass).
  Look look(const State& st, uint64_t now);
  // How long a one-shot takes.
  static uint64_t length(Signal s);

  bool flashing() const { return flashing_; }
  bool playing() const { return playing_.used && playing_.started; }
  uint32_t shown() const { return shown_; }      // one-shots that started
  uint32_t dropped() const { return dropped_; }  // one-shots never shown (the guard not green for kWaitUs, replaced)

 private:
  struct Shot {
    bool used, started;
    Signal s;
    uint64_t asked, start;
  };
  static Look frame(Signal s, uint64_t t);  // a one-shot t after its start
  Shot playing_ = {}, waiting_ = {};
  bool flashing_ = false;
  uint32_t shown_ = 0, dropped_ = 0;
};

// Sends what EyeLanguage::look() says as the SET_EYES goal.
class EyeDriver {
 public:
  static constexpr uint64_t kRepeatUs = 40000;  // a change goes out again this much later, twice
  explicit EyeDriver(SerialLink& link) : link_(link) {}
  // Every pass. driven: the body takes SET_EYES (HELLO caps EYES); else nothing is sent.
  void poll(const Look& l, bool driven, uint64_t now);
  const Look& sent() const { return sent_; }
  uint32_t changes() const { return changes_; }

 private:
  SerialLink& link_;
  bool has_ = false;
  Look sent_ = {};
  uint64_t last_us_ = 0;
  int repeats_ = 0;
  uint32_t changes_ = 0;
};

}  // namespace signals
}  // namespace kilo

#endif
