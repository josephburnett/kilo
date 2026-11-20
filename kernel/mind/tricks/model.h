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
constexpr uint8_t kSpeedMin = KILO_TRICK_PLAY_SPEED_Q6_MIN;    // 0.25x (spec TRICK_PLAY speed_q6 range)

// Wire order of the 11 driven joints as legacy slots (model.py CANONICAL).
extern const uint8_t kCanonical[KILO_JOINT_COUNT];

// A check the mind makes while the body plays the trick (model.py Guard,
// mind-side only): over the second half of frame `frame`'s hold the
// TELEMETRY nose-up pitch (-pitch) must average inside lo..hi and |roll|
// stay within roll_max (all a16, 1/16 deg), or the player sends TRICK_STOP.
struct TrickGuard {
  uint8_t frame;
  int16_t lo_a16;
  int16_t hi_a16;
  int16_t roll_a16;
  const char* name;
};

struct Trick {
  const char* name;    // the Kilo name (library.names())
  const char* legacy;  // the legacy token library.load() also accepts ("up", "wsf"); "" for an authored trick
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
  uint8_t stop_ticks;   // mind-side: TRICK_STOP's transition for this trick (0 = the body's), Trick.stop_ticks
  const TrickGuard* guards = nullptr;  // mind-side: Trick.guards
  uint8_t n_guards = 0;
  int16_t roll_max_a16 = 0;            // mind-side: Trick.roll_max (0 = none), a16
};

// player.py GuardWatch: fed every TELEMETRY (microseconds since TRICK_PLAY's
// ACK, TELEMETRY pitch and roll, a16), feed() returns why the trick must be
// stopped (once; the text lives in the watch), else null.
class GuardWatch {
 public:
  static constexpr uint8_t kMaxGuards = 8;
  static constexpr uint32_t kEarlyTicks = 2;  // player.py GUARD_EARLY_TICKS
  void reset(const Trick& t, uint8_t speed_q6);
  const char* feed(uint64_t t_us, int16_t pitch_a16, int16_t roll_a16);
  bool fired() const { return fired_; }
  const char* why() const { return why_; }
  // when guard i decides (us since the play), for the tests
  uint64_t decide_us(uint8_t i) const { return i < n_ ? decide_us_[i] : 0; }

 private:
  const Trick* t_ = nullptr;
  uint8_t n_ = 0;
  uint64_t from_us_[kMaxGuards] = {};
  uint64_t decide_us_[kMaxGuards] = {};
  int32_t sum_[kMaxGuards] = {};
  uint16_t count_[kMaxGuards] = {};
  bool done_[kMaxGuards] = {};
  uint8_t over_ = 0;
  bool fired_ = false;
  char why_[96] = {};
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
