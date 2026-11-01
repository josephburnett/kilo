// model: a trick as the body stores it, and the body's player timing.
// Python counterparts: kilo/tricks/model.py (Trick.begin, frame_messages,
// commit, crc16, problems) and kilo/tricks/player.py (phase_ticks,
// total_ticks, Player), plus trick_id_for and speed_q6 from
// kilo/body/client.py.
//
// The mind ships its trick library compiled in (tricks/library.gen.cpp,
// made by tricks/gen_library.py from mind/kilo/tricks/library/*.json) and
// only ever uploads it, so a trick here is already in wire form: the
// TRICK_BEGIN fields and the 14 B keyframes exactly as Python's
// Trick.begin() and Keyframe.to_wire() make them (angles rounded half away
// from zero, -128 = unchanged, canonical joint order). Python's float
// angles, JSON, mirror() and speed() stay on the laptop: the body mirrors
// and changes speed itself (TRICK_PLAY flags and speed_q6), so one upload
// serves every variant.
#ifndef KILO_MIND_TRICKS_MODEL_H
#define KILO_MIND_TRICKS_MODEL_H
#include <stdint.h>
#include "proto/kilo_protocol.h"

namespace kilo {
namespace tricks {

constexpr int8_t kAngleUnchanged = -128;
constexpr uint8_t kMaxFrames = KILO_TRICK_CAPACITY;           // 24
constexpr uint8_t kFramesPerMsg = KILO_TRICK_FRAMES_PER_MSG;  // 4
constexpr uint8_t kRepeatForever = 255;
constexpr uint8_t kSpeedUnity = KILO_TRICK_SPEED_UNITY_Q6;    // 64 = 1x
constexpr uint8_t kSpeedMin = 16;                             // 0.25x (client.py SPEED_MIN)

// Wire order of the 11 driven joints as legacy slots (model.py CANONICAL).
extern const uint8_t kCanonical[KILO_JOINT_COUNT];

struct Trick {
  const char* name;    // the Kilo name (library.FIRST_VERSION key)
  const char* legacy;  // the legacy token library.load() also accepts ("up", "wsf")
  uint8_t n_frames;
  uint8_t loop_start;
  uint8_t loop_end;
  uint8_t repeats;      // total plays of the loop: 0 and 1 once, 255 until stopped
  uint16_t joint_mask;  // u16 over the 16 legacy slots
  uint8_t flags;        // KILO_TRICK_FLAGS_*
  uint8_t end_posture;  // KILO_POSTURE_*
  int8_t exp_roll;      // whole degrees, clamped to +-127
  int8_t exp_pitch;
  const kilo_keyframe_t* frames;
};

// TRICK_BEGIN for trick_id.
void trick_begin(const Trick& t, uint8_t trick_id, kilo_trick_begin_t* out);
// TRICK_FRAMES chunks: ceil(n / 4); chunk k's payload into out, its length returned.
uint8_t trick_chunks(const Trick& t);
uint8_t trick_frames_msg(const Trick& t, uint8_t chunk, kilo_trick_frames_t* out);
// CRC-16/CCITT-FALSE over the n x 14 B frame array (TRICK_COMMIT).
uint16_t trick_crc16(const Trick& t);
// client.py trick_id_for: a stable 1..254 tag, crc_hqx(begin(0)[1:] + frames, 0xFFFF) % 254 + 1.
uint8_t trick_id_for(const Trick& t);
// model.py problems(), on the wire form: null when the body takes it, else
// the first problem. Checks the frame count, loop, joint mask, dur, the
// reserved easing and the angles against the spec's joint limits.
const char* trick_problem(const Trick& t);

// client.py speed_q6: round(speed x 64), 0 when outside 16..255 (0.25x..3.98x).
uint8_t speed_q6(double speed);

// ---- player.py: the body's keyframe player timing --------------------------------------

// Ticks a phase of n ticks takes at speed_q6: ceil(n x 64 / speed).
uint32_t phase_ticks(uint32_t n, uint8_t speed_q6 = kSpeedUnity);
// Ticks from TRICK_PLAY to the last phase's end (TRICK_DONE comes a tick
// later); false for a trick that loops until stopped.
bool total_ticks(const Trick& t, uint8_t speed_q6, uint32_t* ticks);

// Tick-exact mirror of the body's kf_step() (player.py Player): step()
// once per body tick, true on the tick the trick finishes.
class KeyframeTimer {
 public:
  explicit KeyframeTimer(const Trick& t, uint8_t speed_q6 = kSpeedUnity) : t_(t), speed_(speed_q6) {}
  bool step();
  bool running() const { return running_; }
  uint8_t frame() const { return frame_; }

 private:
  uint32_t len() const;
  const Trick& t_;
  uint8_t speed_;
  bool running_ = true;
  uint8_t frame_ = 0;
  bool hold_ = false;
  uint8_t plays_ = 0;
  uint32_t pos_ = 0;
};

}  // namespace tricks
}  // namespace kilo

#endif
