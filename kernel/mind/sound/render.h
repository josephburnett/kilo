// render: one sound at a time into the output's sample buffers, and the
// seam to the platform's output (docs/sound.md).
//
// Renderer is the part that runs in the audio interrupt on the Pi (the PWM
// sound device's chunk callback, src/sound.cpp): integer arithmetic only,
// no allocation, no logging, bounded work per frame. Per output frame: the
// stored mu-law sample and the next one decoded (sound.h kUlaw), linearly
// interpolated to the output rate (a 16.16 fixed-point position), times
// the volume's gain and the envelope, around the output's DC level, the
// same value on both channels (the speaker is one; the jack has two).
//
//   the envelope   kFadeInMs up at the start, kFadeOutMs down before the
//                  end, and kFadeOutMs down when stopped: no click at
//                  either end, whatever the recording starts or ends with
//   the DC level   the PWM's silence is the middle of its range, its output
//                  with the device stopped is 0: power(true) ramps the level
//                  from 0 to the middle over kRampMs, power(false) back
//                  down (a slow step, below what the speaker sounds, instead
//                  of a jump: the "pop"); a sound waits for the level
//
// The main loop's calls (start, stop, power) and render() must not
// interleave: on the Pi the output wraps each call in a critical section
// (interrupts off for the few instructions it takes). Nothing here blocks.
//
// SoundOutput is the seam the player (player.h) drives; the Pi's is the
// PWM device (src/sound.cpp), the host's TimedOutput below renders by the
// clock into a scratch buffer, so a sound "plays" for as long as it would
// on the Pi.
#ifndef KILO_MIND_SOUND_RENDER_H
#define KILO_MIND_SOUND_RENDER_H
#include <stdint.h>
#include "platform.h"
#include "sound/sound.h"

namespace kilo {
namespace sound {

constexpr uint32_t kOutRate = 44100;   // the output's rate (the PWM carrier: 44.1 kHz, above hearing)
constexpr uint32_t kFadeInMs = 5;
constexpr uint32_t kFadeOutMs = 15;
constexpr uint32_t kRampMs = 80;       // the DC level's ramp at power on and off
constexpr uint32_t kUnity = 65536;     // gain 1.0 (Q16)

class Renderer {
 public:
  // out_rate: frames a second; max: the output's largest value (the PWM's
  // range - 1); silence is max / 2.
  explicit Renderer(uint32_t out_rate = kOutRate, uint32_t max = 0xFFFF);
  void set_max(uint32_t max);
  uint32_t max() const { return max_; }

  // ---- the main loop's side ----
  // Play s at gain (Q16, 0..kUnity), replacing what plays (cut at once: the
  // player only starts a sound when nothing sounds).
  void start(const Sound& s, uint32_t gain_q16);
  // Fade out what plays (kFadeOutMs).
  void stop();
  // The DC level toward the middle (on) or 0 (off).
  void power(bool on);
  bool powered() const { return powered_; }
  // A sound is playing or fading out (or waits for the level).
  bool sounding() const { return clip_ != nullptr; }
  // Off: no sound, the level at 0 and staying there; the device may stop.
  bool settled_off() const { return !powered_ && level_ == 0 && clip_ == nullptr; }
  // On and at the middle.
  bool at_level() const { return powered_ && level_ == target(); }

  // ---- the output's side (the interrupt) ----
  // frames stereo frames into out (2 words a frame, the channels the
  // same). Returns the words written: 0 once settled_off() (the PWM device
  // then stops its DMA).
  uint32_t render(unsigned* out, uint32_t frames);

  // Counters (read with the main loop's calls).
  uint32_t clips_ended() const { return ended_; }
  uint64_t frames_rendered() const { return frames_; }

 private:
  uint32_t target() const { return powered_ ? half_ << 16 : 0; }
  void end_clip();

  uint32_t rate_, max_, half_;
  uint32_t ramp_step_;            // Q16 level per frame
  uint32_t in_frames_, out_frames_;
  bool powered_ = false;
  uint32_t level_ = 0;            // Q16: the DC level now
  const Sound* clip_ = nullptr;
  uint32_t pos_ = 0;              // the clip's sample now ...
  uint32_t frac_ = 0;             // ... and the fraction past it (16 bits)
  uint32_t step_ = 0;             // 16.16 samples per output frame
  uint32_t fade_at_ = 0;          // the clip's sample where its end fade starts
  uint32_t gain_ = kUnity;
  uint32_t env_ = 0;              // Q16 envelope
  uint32_t in_step_ = 0, out_step_ = 0;
  bool stopping_ = false;
  uint32_t ended_ = 0;
  uint64_t frames_ = 0;
};

// What the platform's output tells the player.
struct OutputStats {
  uint8_t state = 0;            // KILO_SOUND_STATE_*
  uint32_t chunks = 0;          // chunks the device asked for
  uint32_t late = 0;            // ... more than 1.5 chunk periods after the one before
  uint32_t interval_max_us = 0; // the longest time between two chunks while playing
  uint32_t callback_max_us = 0; // the longest a chunk took to fill
  uint32_t callback_mean_us = 0;
  uint32_t starts = 0;          // the device started (stopped mode: once a sound)
  uint32_t failures = 0;        // the device would not start
};

// The seam (player.h drives it). Not pure (a pure virtual needs
// __cxa_pure_virtual, behaviour/tree.h): the defaults are no output.
class SoundOutput {
 public:
  // Play s at gain (Q16): false if the device failed.
  virtual bool play(const Sound& /*s*/, uint32_t /*gain_q16*/) { return false; }
  // A sound plays or fades out.
  virtual bool playing() { return false; }
  // Fade out what plays.
  virtual void stop() {}
  // Light sleep: silence the output and stop the device (true), start it again (false).
  virtual void suspend(bool /*asleep*/) {}
  // Every pass of the main loop.
  virtual void poll() {}
  virtual void stats(OutputStats& out) { out = OutputStats(); }

 protected:
  ~SoundOutput() = default;
};

// kilo-mind-host's output and the tests': a Renderer driven by the clock,
// kChunkFrames at a time into a scratch buffer, as the Pi's device would
// ask for them. tap (if set) sees every chunk.
class TimedOutput final : public SoundOutput {
 public:
  static constexpr uint32_t kChunkFrames = 512;
  using Tap = void (*)(void* arg, const unsigned* words, uint32_t n);

  explicit TimedOutput(Clock& clock, uint32_t max = 0xFFFF, bool keep_running = false);
  void set_tap(Tap tap, void* arg) {
    tap_ = tap;
    tap_arg_ = arg;
  }
  Renderer& renderer() { return r_; }

  bool play(const Sound& s, uint32_t gain_q16) override;
  bool playing() override { return r_.sounding(); }
  void stop() override { r_.stop(); }
  void suspend(bool asleep) override;
  void poll() override;
  void stats(OutputStats& out) override;

 private:
  Clock& clock_;
  Renderer r_;
  bool keep_running_;
  bool running_ = false, asleep_ = false;
  uint64_t t_us_ = 0;   // the clock up to which frames are rendered
  uint64_t frac_ = 0;   // frames owed, x 1e6
  unsigned buf_[kChunkFrames * 2];
  Tap tap_ = nullptr;
  void* tap_arg_ = nullptr;
  uint32_t chunks_ = 0, starts_ = 0;
};

}  // namespace sound
}  // namespace kilo

#endif
