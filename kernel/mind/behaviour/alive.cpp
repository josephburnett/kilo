#include "behaviour/alive.h"
#include "remote/client.h"
#include "tricks/model.h"
#include "util/dmath.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/logring.h"
#include "util/mem.h"

namespace kilo {
namespace behaviour {

namespace {

constexpr double kSettleS = 1.2;        // after the posture is latched, before the first move
constexpr double kPauseMinS = 1.5, kPauseMaxS = 6.0;  // standing: between two moves
// sitting (livelier, 2026-11-19): mostly short pauses, now and then a longer rest
constexpr double kSitPauseMinS = 0.3, kSitPauseMaxS = 2.0, kSitRestChance = 0.2, kSitRestMinS = 3.0, kSitRestMaxS = 5.0;
constexpr double kLookMinS = 0.8, kLookMaxS = 3.0;  // sitting: a new look this often
constexpr double kSitLookPanDeg = 45;               // ... this far to each side
constexpr double kSitLookTiltLo = -32, kSitLookTiltHi = -5;  // ... around SIT's -20 (- = down)
constexpr double kSendEveryS = 0.1;     // the pose's ramp, sent at 10 Hz
constexpr double kComX = 7.5;           // the centre of mass ahead of the feet's centre at STAND (model.h, measured 2026-11-14)
// SIT as the body holds it (body/src/core/posture.cpp; the rear shoulders' -90 clamped to their -80 limit),
// canonical order: pan, tilt, tail, shoulders LF RF RB LB, knees LF RF RB LB.
const int8_t kSit[KILO_JOINT_COUNT] = {10, -20, -60, 30, 30, -80, -80, 60, 60, 45, 45};
enum : int { kTail = 2, kShLF = 3, kShRF = 4, kShRB = 5, kShLB = 6, kKnLF = 7, kKnRF = 8, kKnRB = 9, kKnLB = 10 };
// legs and tail (legacy slots 2, 8-15): the head stays the tree's
constexpr uint16_t kSitMask = (1u << 2) | 0xFF00u;

double clampd(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }
int irnd(double v) { return (int)(v < 0 ? v - 0.5 : v + 0.5); }
double sgn(double v) { return v < 0 ? -1.0 : 1.0; }

}  // namespace

const char* alive_move_name(AliveMove m) {
  static const char* const kNames[] = {"shift", "lean",  "height", "dip",   "sway",  "follow",
                                       "stretch", "hips", "tail",   "paw",   "shrug", "flinch"};
  return (int)m < (int)AliveMove::kCount ? kNames[(int)m] : "none";
}

double stance_margin_mm(const planner::BodyModel& m, const BodyPose& p) {
  double h = (double)m.stance_height(205) + (p.dh > 0 ? p.dh : 0);  // the CoM's height, at most
  double fore = kComX + h * d_sin(p.pitch * 3.14159265358979 / 180.0) - p.xs;
  double side = h * d_sin(p.roll * 3.14159265358979 / 180.0);
  double mf = (double)m.length_mm / 2 - (fore < 0 ? -fore : fore);
  double ms = (double)m.track_mm / 2 - (side < 0 ? -side : side);
  return mf < ms ? mf : ms;
}

Alive::Alive(SerialLink& link, tricks::TrickPlayer& player, LogRing& log, uint64_t seed)
    : link_(link), player_(player), log_(log), rng_(seed ^ 0xA11FEull) {
  trick_.name = "alive_sit";
  trick_.legacy = "";
  trick_.frames = frames_;
}

Alive::Variant Alive::wanted() const {
  SerialLink& l = link_;
  const Goal* g = l.goal(KILO_MSG_SET_POSTURE);
  if (g == nullptr) return variant_ == Variant::Sit ? Variant::Sit : Variant::None;  // a sit move's trick cleared it
  uint8_t code = g->payload[1], flags = g->payload[3];
  if (code == KILO_POSTURE_STAND && flags == 0) return Variant::Stand;
  if (code == KILO_POSTURE_STANCE && own_stance_) return Variant::Stand;
  if (code == KILO_POSTURE_SIT) return Variant::Sit;
  return Variant::None;
}

void Alive::send_pose(const BodyPose& p) {
  kilo_set_body_pose_t g;
  g.epoch = 0;  // the link stamps it
  g.pitch = (int16_t)irnd(p.pitch * 16);
  g.roll = (int16_t)irnd(p.roll * 16);
  g.dheight_mm = (int8_t)irnd(p.dh);
  g.xshift_mm = (int8_t)irnd(p.xs);
  if (link_.set_goal(KILO_MSG_SET_BODY_POSE, (const uint8_t*)&g, sizeof g)) pose_sent_ = true;
  cur_ = p;
}

void Alive::enter(Variant v, double now) {
  variant_ = v;
  entered_ = now;
  next_at_ = now + kSettleS + rng_.uniform(0, 2.0);
  n_steps_ = 0;
  from_ = cur_ = {0, 0, 0, 0};
  if (v == Variant::Stand && !own_stance_) {
    own_stance_ = client_posture(link_, nullptr, "stance");
    logf(log_, LogLevel::Info, "alive: standing (STANCE, the body pose on top)%s", own_stance_ ? "" : ": refused");
  } else if (v == Variant::Sit) {
    next_at_ = now + 0.6 + rng_.uniform(0, 1.0);  // sitting: the first move soon
    look_at_ = now + rng_.uniform(0.3, 1.5);
    logf(log_, LogLevel::Info, "alive: sitting");
  }
}

void Alive::stop() {
  if (pose_sent_ && (cur_.pitch != 0 || cur_.roll != 0 || cur_.dh != 0 || cur_.xs != 0)) send_pose({0, 0, 0, 0});
  if (looked_) client_look(link_, 0, 0);  // the sitting looks: the head back to straight ahead
  looked_ = false;
  if (player_.busy() && player_.trick() == &trick_) player_.stop();
  n_steps_ = 0;
  variant_ = Variant::None;
  own_stance_ = false;
}

void Alive::leave() {
  bool stance = own_stance_;
  stop();
  if (stance) {  // the tree still thinks STAND: give it back, unless it latched something else since
    const Goal* g = link_.goal(KILO_MSG_SET_POSTURE);
    if (g != nullptr && g->payload[1] == KILO_POSTURE_STANCE) client_posture(link_, nullptr, "stand");
  }
}

AliveMove Alive::pick(bool standing) {
  // weights: shift, lean, height, dip, sway, follow, stretch, hips, tail, paw, shrug, flinch (the guard's only)
  static const double kStand[] = {3, 3, 2, 2, 2, 2, 0.7, 0, 0, 0, 0, 0};
  static const double kSitW[] = {3, 2.5, 0, 2, 2, 0, 1, 1.5, 3, 1.5, 1.5, 0};
  const double* w = standing ? kStand : kSitW;
  double total = 0;
  for (int i = 0; i < (int)AliveMove::kCount; i++)
    if (i != (int)last_) total += w[i];
  double r = rng_.uniform(0, total);
  for (int i = 0; i < (int)AliveMove::kCount; i++) {
    if (i == (int)last_) continue;
    if (r < w[i]) return (AliveMove)i;
    r -= w[i];
  }
  return AliveMove::Shift;
}

int Alive::plan_stand_move(AliveMove m, double head_pan, BodyPose* out, int cap) {
  plan_stand(m, head_pan);
  int n = n_steps_ < cap ? n_steps_ : cap;
  for (int i = 0; i < n; i++) out[i] = steps_[i].to;
  n_steps_ = 0;
  return n;
}

void Alive::plan_stand(AliveMove m, double head_pan) {
  double side = rng_.random() < 0.5 ? -1 : 1;
  double dur = rng_.uniform(1.5, 3.0), hold = rng_.uniform(0.8, 2.5);
  n_steps_ = 0;
  auto add = [&](BodyPose p, double d, double h) {
    if (n_steps_ < kMaxSteps) steps_[n_steps_++] = {p, d, h};
  };
  switch (m) {
    case AliveMove::Shift:
      add({0, side * rng_.uniform(1.5, 3.0), 0, 0}, dur, hold);
      break;
    case AliveMove::Lean:
      add({side * rng_.uniform(0.8, 2.0), 0, 0, side * rng_.uniform(3, 6)}, dur, hold);
      break;
    case AliveMove::Height:
      add({0, 0, side < 0 ? -rng_.uniform(3, 7) : rng_.uniform(1, 2.5), 0}, dur, hold);
      break;
    case AliveMove::Dip:
      add({rng_.uniform(1.5, 3.0), side * rng_.uniform(1.5, 2.5), -rng_.uniform(0, 2), 0}, dur, hold);
      break;
    case AliveMove::Sway: {
      double a = rng_.uniform(1.5, 2.5);
      add({0, side * a, 0, 0}, rng_.uniform(2.0, 3.0), 0.3);
      add({0, -side * a, 0, 0}, rng_.uniform(3.0, 4.0), 0.3);
      break;
    }
    case AliveMove::Follow: {  // lean toward where the head looks (pan + = left, assumed: roll right side up)
      double s = head_pan == 0 ? side : sgn(head_pan);
      add({0, s * clampd(1.0 + (head_pan < 0 ? -head_pan : head_pan) / 25.0, 1.0, 2.5), 0, rng_.uniform(1, 3)}, dur,
          hold);
      break;
    }
    case AliveMove::Stretch:
      add({rng_.uniform(4.0, 5.5), 0, -rng_.uniform(3, 5), -rng_.uniform(3, 5)}, rng_.uniform(2.5, 3.5),
          rng_.uniform(1.5, 2.5));
      break;
    default:
      add({0, side * 2.0, 0, 0}, dur, hold);
      break;
  }
  add({0, 0, 0, 0}, rng_.uniform(1.5, 3.0), 0);  // back to level
  for (int i = 0; i < n_steps_; i++)  // never outside the margin: scale the pose down until it fits
    for (int k = 0; k < 4 && stance_margin_mm(model_, steps_[i].to) < kMinMarginMm; k++) {
      BodyPose& p = steps_[i].to;
      p = {p.pitch / 2, p.roll / 2, p.dh / 2, p.xs / 2};
    }
}

// The sitting moves (2026-11-19, the owner: "he doesn't move nearly enough in the sitting
// position"). Each is up to three keyframes off SIT and then back to it; a frame's change of each
// joint is its percentage of the move's amplitude a (degrees, drawn from a_lo..a_hi), in the
// canonical order: pan, tilt, tail, shoulders LF RF RB LB, knees LF RF RB LB. A sided move is
// mirrored half the time (LF <-> RF, RB <-> LB, the tail negated). The head stays the tree's
// (pan and tilt 0 here; the sitting looks are SET_HEAD, sit_look()), except in the flinch.
// mind/tests/test_alive_sit.py reads this table: every move at its largest amplitude, both sides,
// inside what Kilo's joints reach (the recorded calibration, kilo cal show) and resting on the
// floor with the centre of mass inside the feet (kilo.tricks.stance, the 3-D statics), on Kilo as
// designed (the right hind shoulder recalibrated) and as recorded. Keep it one row a line.
// SIT_MOVES_BEGIN
const SitMoveDef kSitMoves[] = {
    {AliveMove::Shift, "shift", 4, 9, true, 1, {{0, 0, 0, 50, -50, 0, 0, 100, -100, 0, 0}}},
    {AliveMove::Lean, "lean", 5, 12, true, 1, {{0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0}}},
    {AliveMove::Dip, "dip", 5, 10, true, 1, {{0, 0, 30, 50, 0, 0, 0, 100, 0, 0, 0}}},
    {AliveMove::Sway, "sway", 3, 7, true, 2, {{0, 0, 40, 0, 0, 0, 0, 100, -100, 0, 0}, {0, 0, -40, 0, 0, 0, 0, -100, 100, 0, 0}}},
    {AliveMove::Stretch, "stretch", 8, 14, false, 1, {{0, 0, 100, -100, -100, 0, 0, -100, -100, 0, 0}}},
    {AliveMove::Hips, "hips", 4, 10, false, 1, {{0, 0, 50, 0, 0, 0, 0, 0, 0, -100, -100}}},
    {AliveMove::Tail, "tail", 15, 30, true, 3, {{0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, -100, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0}}},
    {AliveMove::Paw, "paw", 20, 28, true, 2, {{0, 0, 30, 0, 0, 0, 0, -30, 30, 0, 0}, {0, 0, 30, 100, 0, 0, 0, -30, 30, 0, 0}}},
    {AliveMove::Shrug, "shrug", 5, 10, false, 2, {{0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0}, {0, 0, 0, 40, 40, 0, 0, 0, 0, 0, 0}}},
    {AliveMove::Flinch, "flinch", 20, 20, false, 1, {{0, 75, 100, -40, -40, 0, 0, -25, -25, 0, 0}}},
};
// SIT_MOVES_END

const SitMoveDef* sit_move_def(AliveMove m) {
  for (const SitMoveDef& d : kSitMoves)
    if (d.move == m) return &d;
  return nullptr;
}

// Plays d on top of SIT into the trick (frames_, then back to SIT). a: the amplitude, mirror: the
// other side; the frames' timing drawn here (quicker for the tail and the flinch).
void Alive::build_sit(const SitMoveDef& d, double a, bool mirror, bool head) {
  int n = 0;
  auto frame = [&](const double dd[KILO_JOINT_COUNT], double dur_s, double hold_s) {
    kilo_keyframe_t& f = frames_[n++];
    mem_zero(&f, sizeof f);
    f.dur = (uint8_t)clampd(dur_s * 50, 1, 255);
    f.hold = (uint8_t)clampd(hold_s * 50, 0, 255);
    f.flags = KILO_EASING_COSINE;
    for (int k = 0; k < (int)KILO_JOINT_COUNT; k++)
      f.ang[k] = (k == 0 || (k == 1 && !head)) ? tricks::kAngleUnchanged : (int8_t)irnd((double)kSit[k] + dd[k]);
  };
  bool quick = d.move == AliveMove::Tail || d.move == AliveMove::Flinch;
  for (int i = 0; i < d.n && i < 3; i++) {
    double dd[KILO_JOINT_COUNT];
    for (int k = 0; k < (int)KILO_JOINT_COUNT; k++) dd[k] = d.f[i].c[k] * a / 100.0;
    if (mirror) {  // LF <-> RF, RB <-> LB, the tail the other way
      double t;
      t = dd[kShLF], dd[kShLF] = dd[kShRF], dd[kShRF] = t;
      t = dd[kShRB], dd[kShRB] = dd[kShLB], dd[kShLB] = t;
      t = dd[kKnLF], dd[kKnLF] = dd[kKnRF], dd[kKnRF] = t;
      t = dd[kKnRB], dd[kKnRB] = dd[kKnLB], dd[kKnLB] = t;
      dd[kTail] = -dd[kTail];
    }
    double dur = d.move == AliveMove::Flinch ? 0.25 : quick ? rng_.uniform(0.25, 0.5) : rng_.uniform(0.5, 1.1);
    double hold = d.move == AliveMove::Flinch ? 0.9 : quick ? rng_.uniform(0.05, 0.3)
                  : i + 1 < d.n ? rng_.uniform(0.2, 0.6) : rng_.uniform(0.3, 1.4);
    frame(dd, dur, hold);
  }
  double zero[KILO_JOINT_COUNT] = {};
  frame(zero, d.move == AliveMove::Flinch ? 1.0 : rng_.uniform(0.6, 1.2), 0);
  trick_.n_frames = (uint8_t)n;
  trick_.loop_start = 0;
  trick_.loop_end = (uint8_t)(n - 1);
  trick_.repeats = 1;
  trick_.joint_mask = head ? (uint16_t)(kSitMask | (1u << 1)) : kSitMask;
  trick_.flags = 0;
  trick_.end_posture = KILO_POSTURE_SIT;
  trick_.exp_roll = 0;
  trick_.exp_pitch = 0;
  trick_.stop_ticks = 0;
}

void Alive::plan_sit(AliveMove m) {
  const SitMoveDef* d = sit_move_def(m);
  if (d == nullptr || m == AliveMove::Flinch) d = sit_move_def(AliveMove::Tail);  // a standing move: a tail flick
  bool mirror = d->sided && rng_.random() < 0.5;
  build_sit(*d, rng_.uniform(d->a_lo, d->a_hi), mirror, false);
}

bool Alive::flinch() {
  if (player_.busy()) return false;
  const SitMoveDef* d = sit_move_def(AliveMove::Flinch);
  build_sit(*d, d->a_hi, false, true);
  const char* why = nullptr;
  const char* problem = tricks::trick_problem(trick_);
  play_ = problem == nullptr ? player_.start(trick_, false, tricks::kSpeedUnity, &why) : 0;
  if (play_ == 0) {
    sit_failures_++;
    logf(log_, LogLevel::Warn, "alive: flinch not played: %s", problem != nullptr ? problem : why != nullptr ? why : "?");
    return false;
  }
  last_ = AliveMove::Flinch;
  moves_++;
  counts_[(int)AliveMove::Flinch]++;
  uint32_t ticks = 0;
  tricks::total_ticks(trick_, tricks::kSpeedUnity, &ticks);
  if (variant_ == Variant::Sit) next_at_ = next_at_ > 0 ? next_at_ + ticks * 0.02 : 0;
  logf(log_.status(), LogLevel::Info, "alive: flinch (leaning back, %u ticks)", (unsigned)ticks);
  return true;
}

// The sitting looks: the head to somewhere new every kLookMinS..kLookMaxS (SET_HEAD; the tree's own
// looks come in between and are as good).
void Alive::sit_look(double now) {
  if (now < look_at_) return;
  double pan = rng_.uniform(-kSitLookPanDeg, kSitLookPanDeg);
  double tilt = rng_.uniform(kSitLookTiltLo, kSitLookTiltHi);
  if (rng_.random() < 0.25) pan = 0;  // back to straight ahead now and then
  look_at_ = now + rng_.uniform(kLookMinS, kLookMaxS);
  if (client_look(link_, pan, tilt)) {
    looked_ = true;
    looks_++;
  }
}

void Alive::stand_step(double now) {
  if (n_steps_ == 0) return;
  const Step& s = steps_[step_];
  double t = now - step_at_;
  BodyPose p = s.to;
  if (t < s.dur) {
    double u = (1 - d_cos(3.14159265358979 * t / s.dur)) / 2;
    p.pitch = from_.pitch + (s.to.pitch - from_.pitch) * u;
    p.roll = from_.roll + (s.to.roll - from_.roll) * u;
    p.dh = from_.dh + (s.to.dh - from_.dh) * u;
    p.xs = from_.xs + (s.to.xs - from_.xs) * u;
  } else if (t >= s.dur + s.hold) {
    from_ = s.to;
    step_at_ = now;
    if (++step_ >= n_steps_) {
      n_steps_ = 0;
      next_at_ = now + rng_.uniform(kPauseMinS, kPauseMaxS);
    }
  }
  if (n_steps_ != 0 && t < s.dur && now - sent_at_ < kSendEveryS) return;
  sent_at_ = now;
  send_pose(p);
}

void Alive::poll(double now, bool running, bool calm, double head_pan) {
  if (!running || !calm) {
    if (variant_ != Variant::None) leave();
    return;
  }
  Variant v = wanted();
  if (v != variant_) {
    if (variant_ != Variant::None) leave();
    if (v == Variant::None) return;
    enter(v, now);
  }
  const BodyState& b = link_.state();
  bool idle = b.has_telemetry && (b.telemetry.mode == KILO_MODE_IDLE || b.telemetry.mode == KILO_MODE_TRICK);
  if (variant_ == Variant::Stand) {
    if (n_steps_ > 0) {
      stand_step(now);
      return;
    }
    if (now < next_at_ || !idle || b.telemetry.mode != KILO_MODE_IDLE) return;
    double pan_jump = head_pan - last_pan_;
    last_pan_ = head_pan;
    AliveMove m = (pan_jump > 10 || pan_jump < -10) && last_ != AliveMove::Follow ? AliveMove::Follow : pick(true);
    plan_stand(m, head_pan);
    step_ = 0;
    step_at_ = now;
    from_ = cur_;
    last_ = m;
    moves_++;
    counts_[(int)m]++;
    char line[96];
    format(line, sizeof line, "alive: %s (standing; pitch %d roll %d dh %d xs %d, tenths)", alive_move_name(m),
           irnd(steps_[0].to.pitch * 10), irnd(steps_[0].to.roll * 10), irnd(steps_[0].to.dh * 10),
           irnd(steps_[0].to.xs * 10));
    logf(log_.status(), LogLevel::Info, "%s", line);
    return;
  }
  // sitting: a move is a trick; the next waits for it and the pause; the head looks around meanwhile
  if (b.has_telemetry && (b.telemetry.mode == KILO_MODE_IDLE || b.telemetry.mode == KILO_MODE_TRICK)) sit_look(now);
  if (now < next_at_ || player_.busy() || !b.has_telemetry || b.telemetry.mode != KILO_MODE_IDLE) return;
  AliveMove m = pick(false);
  plan_sit(m);
  const char* why = nullptr;
  const char* problem = tricks::trick_problem(trick_);
  play_ = problem == nullptr ? player_.start(trick_, false, tricks::kSpeedUnity, &why) : 0;
  uint32_t ticks = 0;
  tricks::total_ticks(trick_, tricks::kSpeedUnity, &ticks);
  double pause = rng_.random() < kSitRestChance ? rng_.uniform(kSitRestMinS, kSitRestMaxS)
                                                : rng_.uniform(kSitPauseMinS, kSitPauseMaxS);
  next_at_ = now + ticks * 0.02 + pause;
  last_ = m;
  if (play_ == 0) {
    sit_failures_++;
    logf(log_, LogLevel::Warn, "alive: %s (sitting) not played: %s", alive_move_name(m),
         problem != nullptr ? problem : why != nullptr ? why : "?");
    return;
  }
  moves_++;
  counts_[(int)m]++;
  logf(log_.status(), LogLevel::Info, "alive: %s (sitting, %u frames, %u ticks)", alive_move_name(m),
       (unsigned)trick_.n_frames, (unsigned)ticks);
}

void Alive::metrics_line(char* out, uint32_t n) const {
  uint32_t len = format(out, n, "alive: moves=%u looks=%u sit_failures=%u", (unsigned)moves_, (unsigned)looks_,
                        (unsigned)sit_failures_);
  for (int i = 0; i < (int)AliveMove::kCount; i++)
    if (counts_[i] != 0) len += format(out + len, n - len, " %s=%u", alive_move_name((AliveMove)i), (unsigned)counts_[i]);
}

}  // namespace behaviour
}  // namespace kilo
