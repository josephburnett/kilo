// alive: small, slow body moves while the behaviour runner idles or
// explores (the remote's A5), so Kilo reads as alive: the owner's request
// of 2026-11-17 ("the head moves ... but the body should shift a little").
// It runs next to the runner, outside its tree (whose choices match the
// Python runner's tick for tick), and only touches the body pose and the
// legs; the tree keeps the head (its look ranges were widened in
// personality.yaml for "a little more head").
//
// Standing (the tree latched STAND without the balance flag): the posture
// becomes STANCE (the body's IK stand, the one walks start from, where
// SET_BODY_POSE applies) and the mind eases SET_BODY_POSE between small
// poses at 10 Hz, cosine, over 1.5-4 s each: a weight shift to one side
// (roll), a lean forward or back (x-shift, a little pitch), lower or
// higher, a shoulder dip (pitch and roll together), a slow sway (roll one
// way, the other, back), the body following the head when it glances (a
// lean toward its pan), and now and then a stretch (nose down, lower,
// held). Every pose keeps the centre of mass at least kMinMarginMm inside
// the four feet (stance_margin_mm; checked for the whole menu by the tests).
// The body slews the pose itself (0.5 deg and 1 mm a tick), faster than
// these ramps, so the ramp is what Kilo shows.
//
// Sitting (SIT latched): SIT has no IK, so each move is a short trick built
// here and played on the legs and tail (kSitMoves in alive.cpp, up to three
// keyframes off SIT and back to it): a lean to one side (the front knees and
// shoulders against each other), the shoulders forward or back, one
// shoulder dipping, a sway, a stretch of the front legs, the hips lowering,
// a tail swish, a front paw lifted (the weight on the other side first),
// a shrug. Livelier since 2026-11-19: larger (4-14 degrees, the tail 15-30),
// quicker frames, and mostly short pauses (0.3-2 s, one in five a rest of
// 3-5 s). Between and during them the head looks around (SET_HEAD, a new
// look every 0.8-3 s, 45 degrees to each side). Every move inside what the
// joints reach and statically stable on Kilo (mind/tests/test_alive_sit.py).
// flinch(): the distance guard's "very close while sitting": shoulders and
// knees back, the head pulled up, the tail up, held 0.9 s, back to SIT.
//
// The original Nybble's tricks (2026-11-22, the owner: "keep those as
// occasional movements in the random mode"): now and then, sitting or
// standing, one of the personality's alive.tricks instead of a small move
// (the library's alive_<name>_sit or alive_<name>_stand, mind/kilo/tricks/
// alive_set.py: from that pose and back to it, held to the body model; the
// engine's instead for the stretch, sitting or standing, front down and rear up
// with the rear's shake, engine/stretch.h, and standing for the face wash, a
// front paw lifted on three feet, engine/paw.h, either side, two to four
// strokes), at
// alive.trick_speed x the original's pace; the wait between two is drawn
// from 0.5 to 1.5 x alive.trick_every_s, and the same trick never comes
// twice in a row. request_trick() (the remote's A5 while the runner runs,
// however it was started) plays one at once: when nothing plays, at the
// next pass (a standing pose move under way is levelled and dropped); while
// a sitting move or another trick plays, it waits for that to end (one
// waits at most; a press then is refused). Not sitting or standing in the
// runner (napping, greeting, a posture still settling), it waits up to
// kTrickWaitS and is dropped. Standing, the trick ends in STAND and the
// layer takes STANCE again. The tricks count as tricks for the rest of
// the mind (Mind::trick_playing: quiet, the distance guard waits for its
// end), and stop() stops them as it stops a sitting move.
//
// Timing and choice come from a seeded RNG (the runner's seed): a pause of
// 1.5-6 s between standing moves, never the same move twice in a row, the
// sizes and sides drawn each time; the tricks' choice and timing (and the
// pause after one) from a second one, so the moves draw what they drew
// before the tricks came. All of it stops at
// once with the runner (the remote's B6, A7, C7: stop()): the pose goes back
// to level, a move's trick or the trick is stopped.
#ifndef KILO_MIND_BEHAVIOUR_ALIVE_H
#define KILO_MIND_BEHAVIOUR_ALIVE_H
#include <stdint.h>
#include "behaviour/personality.h"
#include "behaviour/random.h"
#include "engine/paw.h"
#include "engine/stretch.h"
#include "link/serial_link.h"
#include "planner/model.h"
#include "platform.h"
#include "tricks/player.h"
#include "util/logring.h"

namespace kilo {
namespace behaviour {

constexpr double kMinMarginMm = 20;  // the centre of mass at least this far inside the feet (standing)
constexpr double kTrickWaitS = 10;   // a trick asked for (A5) waits this long for sitting or standing

// A standing pose on top of STANCE (SET_BODY_POSE): degrees (pitch nose
// down +, roll right side up +, as TELEMETRY) and mm.
struct BodyPose {
  double pitch, roll, dh, xs;
};
// How far the centre of mass stays inside the four feet's rectangle in
// STANCE with this pose (mm; the model's lengths and the measured centre of
// mass, the shift taken at full CoM height: an upper bound on the shift).
double stance_margin_mm(const planner::BodyModel& m, const BodyPose& p);

enum class AliveMove : uint8_t { Shift, Lean, Height, Dip, Sway, Follow, Stretch, Hips, Tail, Paw, Shrug, Flinch, kCount };
const char* alive_move_name(AliveMove m);

// A sitting move (alive.cpp kSitMoves): per keyframe, each joint's change from SIT in percent of
// the amplitude (canonical order); sided: mirrored half the time.
struct SitMoveDef {
  AliveMove move;
  const char* name;
  double a_lo, a_hi;  // the amplitude, degrees
  bool sided;
  uint8_t n;  // keyframes before the one back to SIT
  struct {
    int8_t c[KILO_JOINT_COUNT];
  } f[3];
};
const SitMoveDef* sit_move_def(AliveMove m);

class Alive {
 public:
  static constexpr int kMaxSteps = 4;  // poses in one move (a sway: there, the other way, back)

  Alive(SerialLink& link, tricks::TrickPlayer& player, LogRing& log, uint64_t seed,
        const Personality& personality = kPersonality);

  // Every pass. running: the runner runs (not paused, not stopped); calm:
  // its choice is idle or explore; head_pan: the pan the tree holds the
  // head at (0 when not held). now in seconds.
  void poll(double now, bool running, bool calm, double head_pan);
  // At once: level the pose, stop a move's trick (the runner stopped).
  void stop();
  // The guard's lean back while sitting (a trick on the legs, the head and the tail); false if the
  // player is busy or refused it.
  bool flinch();
  // A5 while the runner runs: one of the tricks, now or after what plays (see above). *what: the
  // remote's log line. False: refused (one already waits, or the personality has none).
  bool request_trick(double now, const char** what);
  bool trick_waiting() const { return pending_; }
  // One of ours plays (the player busy with it).
  bool trick_playing() const;
  // The last trick started (its library name, "" before the first), how many, how many failed.
  const char* last_trick() const { return lib_ != nullptr ? lib_->name : ""; }
  uint32_t tricks_played() const { return tricks_; }
  uint32_t trick_failures() const { return trick_failures_; }
  double next_trick_at() const { return trick_at_; }  // the next occasional one (0: not timed yet)
  uint32_t looks() const { return looks_; }

  bool standing() const { return variant_ == Variant::Stand; }
  bool sitting() const { return variant_ == Variant::Sit; }
  uint32_t moves() const { return moves_; }
  uint32_t moves_of(AliveMove m) const { return counts_[(int)m]; }
  AliveMove last_move() const { return last_; }
  const BodyPose& pose() const { return cur_; }  // the pose sent last (standing)
  // The trick the last sitting move played (for the tests).
  const tricks::Trick& sit_trick() const { return trick_; }
  void metrics_line(char* out, uint32_t n) const;
  // For the tests: plan one move as poll() would (standing: the poses, after
  // the margin check; sitting: the trick), without sending anything.
  int plan_stand_move(AliveMove m, double head_pan, BodyPose* out, int cap);
  const tricks::Trick& plan_sit_move(AliveMove m) {
    plan_sit(m);
    return trick_;
  }

 private:
  enum class Variant : uint8_t { None, Stand, Sit };
  struct Step {
    BodyPose to;
    double dur, hold;
  };
  Variant wanted() const;
  void enter(Variant v, double now);
  void leave();
  AliveMove pick(bool standing);
  void plan_stand(AliveMove m, double head_pan);
  void plan_sit(AliveMove m);
  void build_sit(const SitMoveDef& d, double a, bool mirror, bool head);
  void sit_look(double now);
  void send_pose(const BodyPose& p);
  void stand_step(double now);
  int pick_trick();
  bool play_trick(double now);
  bool trick_due(double now) const;

  SerialLink& link_;
  tricks::TrickPlayer& player_;
  LogRing& log_;
  PyRandom rng_;
  planner::BodyModel model_;
  Variant variant_ = Variant::None;
  double entered_ = 0, next_at_ = 0;
  bool pose_sent_ = false;
  bool own_stance_ = false;  // we turned the tree's STAND into STANCE
  AliveMove last_ = AliveMove::kCount;
  // a standing move under way
  Step steps_[kMaxSteps] = {};
  int n_steps_ = 0, step_ = 0;
  double step_at_ = 0, sent_at_ = -1;
  BodyPose from_ = {0, 0, 0, 0}, cur_ = {0, 0, 0, 0};
  double last_pan_ = 0;
  // a sitting move: its trick
  tricks::Keyframe frames_[kMaxSteps] = {};
  tricks::Trick trick_ = {};
  uint32_t play_ = 0;
  uint32_t moves_ = 0, counts_[(int)AliveMove::kCount] = {}, sit_failures_ = 0, looks_ = 0;
  double look_at_ = 0;
  bool looked_ = false;  // the sitting looks moved the head
  // the tricks
  const Personality& pers_;
  PyRandom trick_rng_;
  engine::PawWash paw_;                 // standing, the face wash: the engine's, made when asked
  engine::Stretch stretch_;             // the stretch, standing or sitting: the engine's, made when asked
  const tricks::Trick* lib_ = nullptr;  // the last one started
  uint32_t lib_play_ = 0;
  int last_trick_ = -1, pending_k_ = -1;
  bool pending_ = false, restance_ = false;
  double trick_at_ = 0, pending_at_ = 0;
  uint32_t tricks_ = 0, trick_failures_ = 0;
  char what_[96] = {};
};

}  // namespace behaviour
}  // namespace kilo

#endif
