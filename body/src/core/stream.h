// stream: the mind's stream of poses (POSE, docs/movement-engine.md) and
// STANCE, IDLE's IK posture. The one way the mind moves the joints: a walk,
// the self-measurement, a trick and a get-up are all keys of this stream.
//
// The body is not a puppet that replays angles: it is given poses to
// assume, each with the tick by which to reach it, gets there by itself
// (the IK, the joint limits and the safety slew in the arbiter), corrects
// the error it measures (the balance, body.cpp), keeps a pose when the mind
// goes quiet, and reports what it did (TELEMETRY corr_*, ref_*, support,
// limited, pose_queued; core/link.cpp).
//
// A key (PoseKey, 30 B): the tick, the flags (which legs are joint angles,
// BALANCE, FALL_OK, EASE), the support feet, the 11 values in canonical
// joint order (pan, tilt, tail, the four shoulders LF RF RB LB, the four
// knees), the attitude the mind predicts there. A foot-mode leg keeps its
// foot in the shoulder's and knee's slots: x in the shoulder's, z in the
// knee's, both q2 (1/4 mm). JOINT_OFF (-32768) in a joint value leaves that
// joint to IDLE.
//
// The queue. Up to POSE_DEPTH keys in tick order. A key for tick T arriving
// after tick N was played is queued when 1 <= T - N <= POSE_MAX_AHEAD (one
// for a queued tick replaces it); else, or with the queue full and T later
// than every queued key, it is dropped (STATS pose_dropped). With the queue
// full and T earlier, the latest queued key makes room.
//
// Playing (stream_step, once per tick). `cur` is the pose played; its tick
// is where the segment toward the next key began (the last key reached, or
// for the first key of a stream the tick it arrived, from a snapshot of the
// joints' outputs then: the stream takes the joints from where they are).
// Each tick every value moves the fraction of what is left that makes it
// arrive exactly on the next key's tick: 1 / (ticks left) in a straight
// line, or the step of a cosine ease (EASE). On that tick `cur` becomes the
// key. A leg that is a foot in both stays a foot (x and z interpolate, and
// it gets the IK, SET_BODY_POSE and the geometric balance); a leg that is
// or becomes joint angles moves in joint space (a foot is converted by the
// IK) and is a foot again once the key is reached.
//
// The queue dry with the last key reached: EVENT POSE_UNDERRUN (arg = ticks
// since the last POSE arrived), the pose is held for POSE_HOLD_TICKS, then
// the foot-mode feet whose support bit is clear go down SETTLE_Q2 per tick
// at their x to the ground (the plane through the three support feet when
// three are down, else their mean depth, else STANCE's height), their bit
// set when there. With every leg a foot and every foot down the stream ends
// (IDLE STANCE, STREAM_STOPPED arg 1, body.cpp); with a joint-mode leg it
// keeps holding (until a SET_POSTURE, an epoch bump or a reflex). A key
// that arrives meanwhile is played toward from the held pose.
//
// The balance (doc §3f; body.cpp runs the controller). Every leg a foot:
// geometric, the reference the key's pitch and roll (plus SET_BODY_POSE),
// the corrections lengthening or shortening only the legs whose foot is
// down; each leg's share ramps 1/4 per tick toward 4 while its support bit
// is set and toward 0 while clear (bits of `share`), so a lifting foot lets
// go of it and a landing one takes it back without a step; held while any
// support bit is clear. A joint-mode leg: joint-space corrections (as the
// built-in postures) while the next key has BALANCE, else they decay.
//
// Every played value then goes through the arbiter's safety stage (limits,
// the 10 deg/tick slew): a late, missing or wild key never snaps a joint.
//
// RAM: 5 x 30 B queue + 30 B pose + about 20 B of state, STANCE and the
// body pose. The leg angles are body_tick's, on the stack.
#ifndef KILO_CORE_STREAM_H
#define KILO_CORE_STREAM_H
#include <stdint.h>
#include "../proto/kilo_protocol.h"
#include "joints.h"

#define POSE_DEPTH KILO_POSE_DEPTH
#define POSE_HOLD_TICKS KILO_POSE_HOLD_TICKS
#define POSE_MAX_AHEAD KILO_POSE_MAX_AHEAD_TICKS
#define SETTLE_Q2 8      // a swing foot's descent when settling, 2 mm per tick
#define POSE_JOINT_LEGS 0x0Fu  // PoseKey::flags bits 0-3: the leg is joint angles

enum StreamState : uint8_t {
  ST_OFF = 0,     // no stream: the joints are IDLE's (STANCE or a posture)
  ST_PLAY = 1,    // moving toward the next key
  ST_HOLD = 2,    // the last key reached, none queued: holding it
  ST_SETTLE = 3,  // the hold is over: the swing feet go down
};

struct PoseKey {
  uint16_t tick;
  uint8_t flags;    // KILO_POSE_FLAGS_*
  uint8_t support;  // KILO_SUPPORT_FLAGS_*
  int16_t v[DRIVEN_COUNT];  // canonical order; a foot-mode leg: x in its shoulder's slot, z in its knee's (q2)
  int16_t pitch, roll;      // the attitude the mind predicts, a16
};

struct Stream {
  PoseKey q[POSE_DEPTH];  // in tick order, q[0] next
  PoseKey cur;            // the pose played; cur.tick: where the segment toward q[0] began
  uint8_t n;              // keys queued
  uint8_t state;          // StreamState
  uint8_t held;           // ST_HOLD: ticks held so far
  uint8_t fall_ok;        // the newest key had FALL_OK
  uint8_t support;        // the feet on the ground this tick
  uint8_t ik_cut;         // legs the IK clamped this tick (bits LF RF RB LB)
  uint16_t rx_tick;       // the body's tick when the last POSE was queued
  uint16_t share;         // 4 bits per leg (LF in bits 0-3): its share of the correction, 0..4 quarters
  uint16_t dropped, underruns;  // STATS pose_dropped, pose_underruns
  // STANCE's height and the body pose (SET_BODY_POSE goal and its slewed
  // value: pitch, roll a16; height delta and x-shift mm), applied to STANCE
  // and on top of every foot-mode leg.
  int16_t h;  // STANCE's body height, q2
  int16_t pose_g[2], pose[2];  // [0] pitch, [1] roll
  int8_t dh_g, xs_g, dh, xs;
  uint8_t geo;  // the geometric balance ran this tick: its corrections enter the IK
};

void stream_init(Stream& s);
// STANCE's height when IDLE enters STANCE from another posture, and after a
// settle: 0.80 (L1 + L2) (spec gait.stance_h_256).
void stream_stance_default(Stream& s);
// Latch a SET_BODY_POSE goal (payload after the epoch: pitch, roll i16 a16,
// dheight, xshift i8 mm); pitch and roll are clamped to +-30 deg.
void stream_pose_goal(Stream& s, const uint8_t* p);
// The body pose slews toward its goal (every tick, streaming or not).
void stream_pose_step(Stream& s);

// POSE payload check (feet within x -480..480, z 60..440 q2; pitch and roll
// within +-30 deg; support bits 0-3; known flags). Returns a GoalResult (OK
// or BAD_ARG).
uint8_t stream_check(const uint8_t* p);
// Queue a checked POSE (p = the whole payload) after tick `now` was played.
// out: the joints' outputs now (the first key's starting pose). False if
// dropped (counted).
bool stream_push(Stream& s, const uint8_t* p, uint16_t now, const int16_t out[DRIVEN_COUNT]);
// A checked POSE payload whose legs are all feet.
inline bool stream_feet_key(const uint8_t* p) { return !(p[3] & POSE_JOINT_LEGS); }
// Drop the stream: queue, pose and state (the joints go back to IDLE).
void stream_stop(Stream& s);

// One tick (call with the tick being played, before stream_joints). out:
// the joints' last outputs (where a joint the stream takes over starts).
// Returns STREAM_EV_* bits.
#define STREAM_EV_UNDERRUN 1  // the last key reached with none queued: EVENT POSE_UNDERRUN
#define STREAM_EV_SETTLED 2   // every leg a foot and down: hand the legs to IDLE STANCE
uint8_t stream_step(Stream& s, uint16_t tick, const int16_t out[DRIVEN_COUNT]);

// The stream holds the joints.
inline bool stream_on(const Stream& s) { return s.state != ST_OFF; }
// Every leg of the pose played is a foot (the geometric balance applies).
inline bool stream_feet(const Stream& s) { return !(s.cur.flags & POSE_JOINT_LEGS); }
// The next key (or the pose held) asks for joint-space balance.
bool stream_balance(const Stream& s);
// Keys waiting in the queue.
inline uint8_t stream_queued(const Stream& s) { return s.n; }
// The attitude the joints are told to give the body, a16 ([0] pitch, [1]
// roll): the body pose, plus the stream's prediction while it plays. The
// balance's reference.
int16_t stream_ref(const Stream& s, uint8_t axis);

// This tick's joint angles (a16, canonical order; JOINT_OFF = not the
// stream's) into j[11]: the stream's pose while it plays, else STANCE's legs
// (j[0..2] JOINT_OFF). corr: the balance corrections (roll, pitch, a16),
// used through the IK when s.geo. Sets s.ik_cut.
void stream_joints(Stream& s, int16_t corr_roll, int16_t corr_pitch, int16_t j[DRIVEN_COUNT]);

#endif
