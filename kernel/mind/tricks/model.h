// model: a trick as recorded keyframes, and their timing as the mind plays
// them through the POSE stream (tricks/player.h). Python counterparts:
// kilo/tricks/model.py (Trick.begin, crc16, problems, trick_id_for).
//
// The mind ships its trick library compiled in (tricks/library.gen.cpp,
// made by tricks/gen_library.py from mind/kilo/tricks/library/*.json): the
// header fields and the 14 B keyframes exactly as Python's Trick.begin()
// and Keyframe.to_wire() make them (angles rounded half away from zero,
// -128 = unchanged, canonical joint order). Python's float angles, JSON,
// mirror() and speed() stay on the laptop: the player mirrors and changes
// speed as it makes the stream's keys. (Until engine step 4 rewrites them
// as goals: docs/movement-engine.md.)
#ifndef KILO_MIND_TRICKS_MODEL_H
#define KILO_MIND_TRICKS_MODEL_H
#include <stdint.h>
#include "proto/kilo_protocol.h"

namespace kilo {
namespace tricks {

constexpr int8_t kAngleUnchanged = -128;
constexpr uint8_t kMaxFrames = 24;
constexpr uint8_t kRepeatForever = 255;
constexpr uint8_t kSpeedUnity = 64;  // speed in 1/64: 64 = 1x
constexpr uint8_t kSpeedMin = 16;    // 0.25x

// A keyframe (14 B, the order Python's Keyframe.to_wire() writes): dur
// (ticks from the previous frame, >= 1), hold (ticks at the target), flags
// (bits 0-1 easing, bit 2 balance), ang[11] (whole degrees, canonical
// joint order; -128 = unchanged: the previous frame's by index).
struct Keyframe {
  uint8_t dur, hold, flags;
  int8_t ang[KILO_JOINT_COUNT];
};
enum Easing : uint8_t { kLinear = 0, kCosine = 1, kStep = 2, kEasingReserved = 3 };
constexpr uint8_t kEasingMask = 0x03;
constexpr uint8_t kFrameBalance = 0x04;  // the body's balance on while moving to and holding this frame
// Trick flags.
constexpr uint8_t kTrickBalance = 0x01;
constexpr uint8_t kTrickFallOk = 0x02;  // played while the fall reflex holds him: the get-up (righting)

// Wire order of the 11 driven joints as legacy slots (model.py CANONICAL).
extern const uint8_t kCanonical[KILO_JOINT_COUNT];

// A check the mind makes while the trick plays (model.py Guard): over the
// second half of frame `frame`'s hold the TELEMETRY nose-up pitch (-pitch)
// must average inside lo..hi and |roll| stay within roll_max (all a16, 1/16
// deg), or the player stops it.
// push_a16 (0 = none; model.py Guard.push_roll): from the start of the
// frame's move to that window the roll may move at most this far from the
// last reading before the move (two readings in a row past it: stop).
struct TrickGuard {
  uint8_t frame;
  int16_t lo_a16;
  int16_t hi_a16;
  int16_t roll_a16;
  const char* name;
  int16_t push_a16 = 0;
};

struct Trick {
  const char* name;    // the Kilo name (library.names())
  const char* legacy;  // the legacy token library.load() also accepts ("up", "wsf"); "" for an authored trick
  uint8_t n_frames;
  uint8_t loop_start;
  uint8_t loop_end;
  uint8_t repeats;      // total plays of the loop: 0 and 1 once, 255 until stopped
  uint16_t joint_mask;  // u16 over the 16 legacy slots
  uint8_t flags;        // kTrickBalance, kTrickFallOk
  uint8_t end_posture;  // KILO_POSTURE_*: the posture the body takes when it ends
  int8_t exp_roll;      // whole degrees, clamped to +-127: the attitude expected (the balance's reference)
  int8_t exp_pitch;
  const Keyframe* frames;
  uint8_t stop_ticks;   // a stop's transition to the end posture (0 = XFADE_TICKS), Trick.stop_ticks
  const TrickGuard* guards = nullptr;  // Trick.guards
  uint8_t n_guards = 0;
  int16_t roll_max_a16 = 0;            // Trick.roll_max (0 = none), a16
};

// GuardWatch: fed every TELEMETRY (microseconds since the trick's start on
// the body, TELEMETRY pitch and roll, a16), feed() returns why the trick must be
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
  uint64_t move_us_[kMaxGuards] = {};  // the guarded frame's move starts (push_a16)
  int16_t roll0_[kMaxGuards] = {};     // the last roll before it
  bool have_roll0_[kMaxGuards] = {};
  uint8_t push_over_[kMaxGuards] = {};
  int32_t sum_[kMaxGuards] = {};
  uint16_t count_[kMaxGuards] = {};
  bool done_[kMaxGuards] = {};
  uint8_t over_ = 0;
  bool fired_ = false;
  char why_[96] = {};
};

// CRC-16/CCITT-FALSE over the n x 14 B frame array: which version of a
// trick played (the flight recorder's trick lines).
uint16_t trick_crc16(const Trick& t);
// model.py trick_id_for: a stable 1..254 tag, crc_hqx(header(0)[1:] + frames,
// 0xFFFF) % 254 + 1, the header packed as the 11 B the body's store took.
uint8_t trick_id_for(const Trick& t);
// model.py problems(): null when the player can play it, else the first
// problem. Checks the frame count, loop, joint mask (every leg joint is
// the stream's: a trick drives all eight; the head joints it may leave),
// dur, the reserved easing, frame 0 setting every leg joint, and the angles
// against the spec's joint limits.
const char* trick_problem(const Trick& t);

// round(speed x 64), 0 when outside 16..255 (0.25x..3.98x).
uint8_t speed_q6(double speed);

// ---- the timing ---------------------------------------------------------------------------

// Ticks a phase of n ticks takes at speed_q6: ceil(n x 64 / speed).
uint32_t phase_ticks(uint32_t n, uint8_t speed_q6 = kSpeedUnity);
// Ticks from the trick's start to its last key; false for a trick that
// loops until stopped.
bool total_ticks(const Trick& t, uint8_t speed_q6, uint32_t* ticks);

// One key of a trick for the POSE stream: frame `frame`'s pose, reached
// `at` ticks after the trick's start, along `ease` (kLinear or kCosine).
struct TrickKey {
  uint32_t at;
  uint8_t frame;
  uint8_t ease;
  bool hold;  // the end of the frame's hold (the pose unchanged since the last key)
};

// The keys of a trick in play order (frames 0..loop_end, the loop's repeats,
// the rest; a loop until stopped repeats for ever): per frame a key at the
// end of its move (dur, eased as the frame says; a step is a key one tick
// after the last and a hold key at the move's end) and one at the end of
// its hold (if any), each phase ceil(n x 64 / speed) ticks.
class KeySeq {
 public:
  KeySeq(const Trick& t, uint8_t speed_q6 = kSpeedUnity) : t_(&t), speed_(speed_q6) {}
  KeySeq() = default;
  // The next key; false after the last.
  bool next(TrickKey* k);

 private:
  const Trick* t_ = nullptr;
  uint8_t speed_ = kSpeedUnity;
  uint8_t frame_ = 0;
  uint8_t phase_ = 0;  // 0 the move, 1 a step's rest of the move, 2 the hold
  uint8_t plays_ = 0;
  bool done_ = false;
  uint32_t at_ = 0;
};

// Frame f's angle of canonical joint k, a16, with -128 resolved to the last
// frame before it (by index) that sets it, mirrored (L/R legs swapped, pan
// and tail negated) if asked; KILO_HEAD_NOT_HELD when no frame up to f sets
// it (a head joint left to the body's posture).
int16_t trick_angle(const Trick& t, uint8_t f, uint8_t k, bool mirror);

}  // namespace tricks
}  // namespace kilo

#endif
