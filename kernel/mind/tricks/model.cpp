#include "tricks/model.h"
#include "proto/crc16.h"
#include "util/mem.h"

namespace kilo {
namespace tricks {

const uint8_t kCanonical[KILO_JOINT_COUNT] = {0, 1, 2, 8, 9, 10, 11, 12, 13, 14, 15};

namespace {

const int8_t kLimits[16][2] = KILO_JOINT_LIMITS_INIT;
constexpr uint16_t kUnusedMask = 0x00F8;  // slots 3-7 (model.py UNUSED)
constexpr uint8_t kEasingMask = 0x03;
constexpr uint8_t kEasingReserved = 3;    // Easing.SPLINE_RESERVED

}  // namespace

void trick_begin(const Trick& t, uint8_t trick_id, kilo_trick_begin_t* out) {
  out->trick_id = trick_id;
  out->n_frames = t.n_frames;
  out->loop_start = t.loop_start;
  out->loop_end = t.loop_end;
  out->repeats = t.repeats;
  out->joint_mask = t.joint_mask;
  out->flags = t.flags;
  out->end_posture = t.end_posture;
  out->exp_roll = t.exp_roll;
  out->exp_pitch = t.exp_pitch;
}

uint8_t trick_chunks(const Trick& t) { return (uint8_t)((t.n_frames + kFramesPerMsg - 1) / kFramesPerMsg); }

uint8_t trick_frames_msg(const Trick& t, uint8_t chunk, kilo_trick_frames_t* out) {
  uint8_t first = (uint8_t)(chunk * kFramesPerMsg);
  uint8_t n = (uint8_t)(t.n_frames - first < kFramesPerMsg ? t.n_frames - first : kFramesPerMsg);
  out->first = first;
  out->count = n;
  mem_copy(out->frames, t.frames + first, n * sizeof(kilo_keyframe_t));
  return (uint8_t)KILO_TRICK_FRAMES_LEN_FOR(n);
}

uint16_t trick_crc16(const Trick& t) {
  return kilo_crc16((const uint8_t*)t.frames, (uint16_t)(t.n_frames * sizeof(kilo_keyframe_t)));
}

uint8_t trick_id_for(const Trick& t) {
  kilo_trick_begin_t b;
  trick_begin(t, 0, &b);
  uint16_t crc = KILO_CRC16_INIT;
  const uint8_t* p = (const uint8_t*)&b;
  for (uint32_t i = 1; i < sizeof b; i++) crc = kilo_crc16_update(crc, p[i]);  // begin(0).pack()[1:]
  p = (const uint8_t*)t.frames;
  for (uint32_t i = 0; i < t.n_frames * sizeof(kilo_keyframe_t); i++) crc = kilo_crc16_update(crc, p[i]);
  return (uint8_t)(crc % 254 + 1);
}

const char* trick_problem(const Trick& t) {
  if (t.n_frames < 1 || t.n_frames > kMaxFrames) return "frame count outside 1..24";
  if (!(t.loop_start <= t.loop_end && t.loop_end < t.n_frames)) return "loop outside the frames";
  if (t.joint_mask == 0) return "no joint is driven";
  if (t.joint_mask & kUnusedMask) return "an unused joint slot is driven";
  if (t.frames == nullptr) return "no frames";
  uint16_t mask = 0;
  for (uint8_t i = 0; i < t.n_frames; i++) {
    const kilo_keyframe_t& f = t.frames[i];
    if (f.dur < 1) return "a frame's dur is 0";
    if ((f.flags & kEasingMask) == kEasingReserved) return "easing 3 is reserved";
    for (uint32_t k = 0; k < KILO_JOINT_COUNT; k++) {
      int8_t a = f.ang[k];
      if (a == kAngleUnchanged) continue;
      uint8_t j = kCanonical[k];
      mask = (uint16_t)(mask | (1u << j));
      if (a < kLimits[j][0] || a > kLimits[j][1]) return "an angle outside its joint's limits";
    }
  }
  if (mask != t.joint_mask) return "joint_mask is not the joints the frames drive";
  return nullptr;
}

uint8_t speed_q6(double speed) {
  double x = speed * 64.0;
  if (!(x >= kSpeedMin - 0.5 && x < 255.5)) return 0;
  return (uint8_t)(x + 0.5);
}

uint32_t phase_ticks(uint32_t n, uint8_t speed_q6) {
  if (speed_q6 == 0) speed_q6 = kSpeedUnity;
  return (n * 64u + speed_q6 - 1u) / speed_q6;
}

bool total_ticks(const Trick& t, uint8_t speed_q6, uint32_t* ticks) {
  if (t.repeats == kRepeatForever) return false;
  // player.py play_order(): 0..loop_end, then (plays - 1) x loop_start..loop_end, then the rest
  uint32_t plays = t.repeats > 1 ? t.repeats : 1u;
  uint32_t total = 0;
  for (uint8_t f = 0; f < t.n_frames; f++) {
    uint32_t phase = phase_ticks(t.frames[f].dur, speed_q6) + phase_ticks(t.frames[f].hold, speed_q6);
    total += phase * ((f >= t.loop_start && f <= t.loop_end) ? plays : 1u);
  }
  *ticks = total;
  return true;
}

uint32_t KeyframeTimer::len() const {
  const kilo_keyframe_t& f = t_.frames[frame_];
  return (uint32_t)(hold_ ? f.hold : f.dur) << 6;
}

bool KeyframeTimer::step() {
  if (!running_) return false;
  if (pos_ >= len()) {
    pos_ = 0;
    if (!hold_ && t_.frames[frame_].hold) {
      hold_ = true;
    } else {
      hold_ = false;
      bool again = false;
      if (frame_ == t_.loop_end) {
        plays_ = (uint8_t)(plays_ + 1 < 254 ? plays_ + 1 : 254);
        again = t_.repeats == kRepeatForever || plays_ < t_.repeats;
      }
      if (again) {
        frame_ = t_.loop_start;
      } else {
        frame_++;
        if (frame_ >= t_.n_frames) {
          running_ = false;
          return true;
        }
      }
    }
  }
  pos_ += speed_;
  return false;
}

}  // namespace tricks
}  // namespace kilo
