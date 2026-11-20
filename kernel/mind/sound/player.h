// player: what Kilo says and when (docs/sound.md). The main loop's side of
// the sounds: play requests by name, the queue, the families, the gap,
// the volume, on and off, light sleep, the counters; the output is behind
// render.h's SoundOutput (the Pi's PWM device, kilo-mind-host's and the
// tests' TimedOutput).
//
// Requests (request(name, why)): the events (Mind), the remote's B2, the
// laptop (SOUND PLAY). A name ending in a digit plays that sound; any other
// name is a family: one of the sounds whose name is it, or it and digits
// (meow -> meow1, meow2, meow3), picked at random (a seeded RNG), never the
// same one twice in a row when there are more.
//
// The queue: one sound plays, one request waits (kQueue 1). A newer
// request replaces the waiting one (newest wins: the last thing that
// happened is what he answers; the replaced one is counted as dropped). The
// waiting one starts once nothing plays and kGapUs has passed since the
// last sound ended, so no run of events can machine-gun sounds; one that
// waited kStaleUs is dropped (a meow seconds after the key is no answer to
// it). A request while off, asleep, for an unknown name or with no sounds
// in the image is refused, counted, and says why (KILO_SOUND_RESULT_*).
//
// Volume 0..100 (kDefaultVolume): the gain is (volume / 100)^2, so the
// steps sound about even. Neither on/off nor the volume is kept over a
// reboot (the distance guard's switch is not either).
//
// Every sound started is a log line and a MARK in the flight recorder
// ("sound play name=meow2 asked=meow why=key:B2 volume=70 ms=1020").
#ifndef KILO_MIND_SOUND_PLAYER_H
#define KILO_MIND_SOUND_PLAYER_H
#include <stdint.h>
#include "behaviour/random.h"
#include "platform.h"
#include "proto/kilo_protocol.h"
#include "sound/render.h"
#include "sound/sound.h"

namespace kilo {
namespace sound {

constexpr uint8_t kDefaultVolume = 70;
constexpr bool kDefaultOn = true;
constexpr uint64_t kGapUs = 1000000;    // silence between two sounds, at least
constexpr uint64_t kStaleUs = 2000000;  // a request not started by then is dropped
constexpr uint32_t kQueue = 1;          // requests that wait while a sound plays

// The gain for a volume (Q16).
uint32_t gain_q16(uint8_t volume);

// Who hears of each sound started (the Mind: a MARK into the flight recorder).
class SoundHost {
 public:
  virtual void mark(const char* /*line*/) {}

 protected:
  ~SoundHost() = default;
};

class SoundPlayer {
 public:
  SoundPlayer(Clock& clock, Log& log, uint64_t seed);
  void set_sounds(const Sound* sounds, uint32_t n);
  void set_output(SoundOutput* out) { out_ = out; }
  void set_host(SoundHost* host) { host_ = host; }

  void set_enabled(bool on);
  bool enabled() const { return on_; }
  // 0..100 (more is 100).
  void set_volume(uint8_t v);
  uint8_t volume() const { return volume_; }

  // Ask for a sound or a family. why: who asked ("key:B2", "wake",
  // "host"), for the log. KILO_SOUND_RESULT_QUEUED, or why not.
  uint8_t request(const char* name, const char* why);
  // Light sleep (the output silenced and stopped) and the wake.
  void suspend(bool asleep);
  bool asleep() const { return asleep_; }
  // Every pass: the output's poll, then the waiting request if its time has come.
  void poll();

  // A sound plays, or one waits.
  bool busy();
  bool playing();
  const Sound* last() const { return last_; }  // playing now, or played last
  uint32_t count() const { return n_; }
  const Sound& sound(uint32_t i) const { return sounds_[i]; }
  // The sound of that exact name, or null.
  const Sound* find(const char* name) const;
  // Total length of the sounds, ms.
  uint32_t total_ms() const;

  uint32_t played() const { return played_; }
  uint32_t requests() const { return requests_; }
  uint32_t unknown() const { return unknown_; }
  uint32_t replaced() const { return replaced_; }
  uint32_t stale() const { return stale_; }
  uint32_t refused_off() const { return off_; }
  uint32_t refused_asleep() const { return asleep_n_; }
  uint32_t refused_none() const { return none_; }
  uint32_t dropped() const { return replaced_ + stale_ + off_ + asleep_n_ + none_ + failed_; }

  void fill(kilo_host_sound_status_t& s, uint8_t result);
  // The metrics line ("sound: on=1 volume=70 sounds=7 ...").
  void metrics_line(char* out, uint32_t n);

 private:
  const Sound* pick(const char* name);
  void begin();

  Clock& clock_;
  Log& log_;
  behaviour::PyRandom rng_;
  const Sound* sounds_ = nullptr;
  uint32_t n_ = 0;
  SoundOutput* out_ = nullptr;
  SoundHost* host_ = nullptr;
  bool on_ = kDefaultOn;
  uint8_t volume_ = kDefaultVolume;
  bool asleep_ = false;

  // the waiting request
  const Sound* pending_ = nullptr;
  uint64_t pending_at_ = 0;
  char asked_[kNameMax + 1] = {};
  char why_[24] = {};

  bool was_playing_ = false;
  bool ever_played_ = false;
  uint64_t ended_at_ = 0;
  const Sound* last_ = nullptr;
  uint32_t played_ = 0, requests_ = 0, unknown_ = 0, replaced_ = 0, stale_ = 0, off_ = 0, asleep_n_ = 0, none_ = 0,
           failed_ = 0;
};

// The behaviour's own sounds (the remote's A5): now and then, while the
// runner sits, a purr, more rarely a trill or a meow; standing or
// exploring, a rare trill or meow. Seeded (the runner's seed), so a run
// replays. poll() says what to ask for now, or null.
class Chatter {
 public:
  enum class Mode : uint8_t { None, Sitting, Standing };
  static constexpr double kFirstMinS = 20, kFirstMaxS = 60;       // after he settles into a mode
  static constexpr double kSitMinS = 45, kSitMaxS = 120;          // between two, sitting
  static constexpr double kStandMinS = 90, kStandMaxS = 240;      // ... standing or exploring

  explicit Chatter(uint64_t seed);
  // now in seconds. quiet: say nothing now (a trick plays): the next one is drawn again.
  const char* poll(double now, Mode mode, bool quiet);
  uint32_t said() const { return said_; }

 private:
  behaviour::PyRandom rng_;
  Mode mode_ = Mode::None;
  double next_at_ = 0;
  uint32_t said_ = 0;
};

}  // namespace sound
}  // namespace kilo

#endif
