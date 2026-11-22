#include "sound/render.h"
#include "proto/kilo_protocol.h"

namespace kilo {
namespace sound {

Renderer::Renderer(uint32_t out_rate, uint32_t max) : rate_(out_rate) {
  in_frames_ = kFadeInMs * out_rate / 1000;
  out_frames_ = kFadeOutMs * out_rate / 1000;
  if (in_frames_ == 0) in_frames_ = 1;
  if (out_frames_ == 0) out_frames_ = 1;
  in_step_ = kUnity / in_frames_;
  out_step_ = kUnity / out_frames_;
  set_max(max);
}

void Renderer::set_max(uint32_t max) {
  if (max < 2) max = 2;
  if (max > 0xFFFF) max = 0xFFFF;
  max_ = max;
  half_ = max / 2;
  uint32_t ramp_frames = kRampMs * rate_ / 1000;
  ramp_step_ = ramp_frames != 0 ? (half_ << 16) / ramp_frames : (half_ << 16);
  if (ramp_step_ == 0) ramp_step_ = 1;
  if (level_ > target()) level_ = target();
}

void Renderer::start(const Sound& s, uint32_t gain_q16) {
  if (s.samples < 2 || s.data == nullptr || s.rate == 0) return;
  clip_ = &s;
  pos_ = 0;
  frac_ = 0;
  step_ = (uint32_t)(((uint64_t)s.rate << 16) / rate_);
  uint32_t fade = kFadeOutMs * s.rate / 1000;
  fade_at_ = s.samples > fade ? s.samples - fade : 0;
  gain_ = gain_q16 > kUnity ? kUnity : gain_q16;
  env_ = 0;
  stopping_ = false;
}

void Renderer::stop() {
  if (clip_ == nullptr) return;
  if (!at_level()) {  // not sounding yet (the level still ramping): nothing to fade
    end_clip();
    return;
  }
  stopping_ = true;
}

void Renderer::power(bool on) {
  powered_ = on;
  if (!on) end_clip();  // a sound cannot play without the level under it
}

void Renderer::end_clip() {
  if (clip_ == nullptr) return;
  clip_ = nullptr;
  stopping_ = false;
  env_ = 0;
  ended_++;
}

uint32_t Renderer::render(unsigned* out, uint32_t frames) {
  if (settled_off()) return 0;
  const uint32_t tgt = target();
  const int32_t half = (int32_t)half_, top = (int32_t)max_;
  for (uint32_t f = 0; f < frames; f++) {
    int32_t s = 0;
    if (level_ != tgt) {  // the DC ramp first; a sound waits for it
      if (level_ < tgt) level_ = tgt - level_ > ramp_step_ ? level_ + ramp_step_ : tgt;
      else level_ = level_ - tgt > ramp_step_ ? level_ - ramp_step_ : tgt;
    } else if (clip_ != nullptr) {
      const uint32_t i = pos_;
      if (i + 1 >= clip_->samples) {
        end_clip();
      } else {
        if (!stopping_ && i >= fade_at_) stopping_ = true;
        if (stopping_) {
          if (env_ <= out_step_) env_ = 0;
          else env_ -= out_step_;
        } else if (env_ < kUnity) {
          env_ = kUnity - env_ > in_step_ ? env_ + in_step_ : kUnity;
        }
        const uint8_t* d = clip_->data;
        int32_t a = kUlaw.v[d[i]], b = kUlaw.v[d[i + 1]];
        int32_t v = a + (int32_t)(((int64_t)(b - a) * (int32_t)frac_) >> 16);
        v = (int32_t)(((int64_t)v * (int32_t)gain_) >> 16);
        v = (int32_t)(((int64_t)v * (int32_t)env_) >> 16);
        s = v;
        frac_ += step_;
        pos_ += frac_ >> 16;
        frac_ &= 0xFFFF;
        if (stopping_ && env_ == 0) end_clip();
      }
    }
    int32_t u = (int32_t)(level_ >> 16) + ((s * half) >> 15);
    if (u < 0) u = 0;
    else if (u > top) u = top;
    out[2 * f] = (unsigned)u;
    out[2 * f + 1] = (unsigned)u;
  }
  frames_ += frames;
  return frames * 2;
}

// ---- the host's output ----------------------------------------------------------------------

TimedOutput::TimedOutput(Clock& clock, uint32_t max) : clock_(clock), r_(kOutRate, max) {}

bool TimedOutput::play(const Sound& s, uint32_t gain_q16) {
  if (asleep_) return false;
  if (!running_) {
    r_.power(true);
    running_ = true;
    t_us_ = clock_.now_us();
    frac_ = 0;
    starts_++;
  } else if (!r_.powered()) {
    r_.power(true);  // ramping down: back up
  }
  r_.start(s, gain_q16);
  return true;
}

void TimedOutput::suspend(bool asleep) {
  asleep_ = asleep;
  if (asleep) {
    r_.stop();
    r_.power(false);  // ramps down, then render() returns 0: the device stops
  }
}

void TimedOutput::poll() {
  uint64_t now = clock_.now_us();
  if (!running_) return;
  if (now - t_us_ > 1000000) t_us_ = now - 1000000;  // a long stall: at most a second to catch up
  frac_ += (now - t_us_) * kOutRate;
  t_us_ = now;
  uint64_t owed = frac_ / 1000000;
  frac_ %= 1000000;
  while (owed > 0) {
    uint32_t n = owed > kChunkFrames ? kChunkFrames : (uint32_t)owed;
    owed -= n;
    uint32_t w = r_.render(buf_, n);
    chunks_++;
    if (w == 0) {  // settled off: the device stops
      running_ = false;
      return;
    }
    if (tap_ != nullptr) tap_(tap_arg_, buf_, w);
  }
  if (!r_.sounding() && r_.powered()) r_.power(false);  // stopped between sounds
}

void TimedOutput::stats(OutputStats& out) {
  out = OutputStats();
  out.state = asleep_ ? KILO_SOUND_STATE_ASLEEP : r_.sounding() ? KILO_SOUND_STATE_PLAYING : KILO_SOUND_STATE_IDLE;
  out.chunks = chunks_;
  out.starts = starts_;
}

}  // namespace sound
}  // namespace kilo
