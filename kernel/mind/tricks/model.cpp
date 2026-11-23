#include "tricks/model.h"
#include "link/consts.h"
#include "proto/crc16.h"
#include "util/fmt.h"
#include "util/mem.h"

namespace kilo {
namespace tricks {

const uint8_t kCanonical[KILO_JOINT_COUNT] = {0, 1, 2, 8, 9, 10, 11, 12, 13, 14, 15};

namespace {

const int8_t kLimits[16][2] = KILO_JOINT_LIMITS_INIT;
constexpr uint16_t kUnusedMask = 0x00F8;  // slots 3-7 (model.py UNUSED)
constexpr uint16_t kLegMask = 0xFF00;     // slots 8-15: the stream drives every leg joint

}  // namespace

static_assert(sizeof(Keyframe) == 14, "Keyframe: 14 B");

uint16_t trick_crc16(const Trick& t) {
  return kilo_crc16((const uint8_t*)t.frames, (uint16_t)(t.n_frames * sizeof(Keyframe)));
}

uint8_t trick_id_for(const Trick& t) {
  // header(0) as packed: trick_id, n_frames, loop_start, loop_end, repeats,
  // joint_mask (LE), flags, end_posture, exp_roll, exp_pitch; [1:] skips the id
  const uint8_t h[10] = {t.n_frames, t.loop_start, t.loop_end, t.repeats, (uint8_t)(t.joint_mask & 0xFF),
                         (uint8_t)(t.joint_mask >> 8), t.flags, t.end_posture, (uint8_t)t.exp_roll,
                         (uint8_t)t.exp_pitch};
  uint16_t crc = KILO_CRC16_INIT;
  for (uint32_t i = 0; i < sizeof h; i++) crc = kilo_crc16_update(crc, h[i]);
  const uint8_t* p = (const uint8_t*)t.frames;
  for (uint32_t i = 0; i < t.n_frames * sizeof(Keyframe); i++) crc = kilo_crc16_update(crc, p[i]);
  return (uint8_t)(crc % 254 + 1);
}

const char* trick_problem(const Trick& t) {
  if (t.n_frames < 1 || t.n_frames > kMaxFrames) return "frame count outside 1..24";
  if (!(t.loop_start <= t.loop_end && t.loop_end < t.n_frames)) return "loop outside the frames";
  if (t.joint_mask == 0) return "no joint is driven";
  if (t.joint_mask & kUnusedMask) return "an unused joint slot is driven";
  if ((t.joint_mask & kLegMask) != kLegMask) return "a leg joint is not driven (the stream drives every leg)";
  if (t.frames == nullptr) return "no frames";
  uint16_t mask = 0;
  for (uint8_t i = 0; i < t.n_frames; i++) {
    const Keyframe& f = t.frames[i];
    if (f.dur < 1) return "a frame's dur is 0";
    if ((f.flags & kEasingMask) == kEasingReserved) return "easing 3 is reserved";
    for (uint32_t k = 0; k < KILO_JOINT_COUNT; k++) {
      int8_t a = f.ang[k];
      if (a == kAngleUnchanged) {
        if (i == 0 && k >= 3) return "frame 0 leaves a leg joint unchanged (the stream needs every leg's angle)";
        continue;
      }
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

namespace {
// a16 as "-12.3" (tenths, truncated toward zero)
const char* deg(char* buf, int a16) {
  format_fixed(buf, 12, a16 * 10 / 16, 1);
  return buf;
}
}  // namespace

void GuardWatch::reset(const Trick& t, uint8_t speed_q6) {
  t_ = &t;
  n_ = t.guards != nullptr ? (t.n_guards < kMaxGuards ? t.n_guards : kMaxGuards) : 0;
  over_ = 0;
  fired_ = false;
  why_[0] = 0;
  // when each guarded frame's hold first starts, in play order (player.py play_order; a forever loop once)
  uint32_t plays = t.repeats == kRepeatForever ? 1u : (t.repeats > 1 ? t.repeats : 1u);
  for (uint8_t i = 0; i < n_; i++) {
    sum_[i] = 0;
    count_[i] = 0;
    done_[i] = false;
    have_roll0_[i] = false;
    push_over_[i] = 0;
    uint8_t g = t.guards[i].frame < t.n_frames ? t.guards[i].frame : 0;
    uint32_t acc = 0, h0 = 0, m0 = 0;
    bool found = false;
    auto visit = [&](uint8_t f) {
      if (found) return;
      m0 = acc;
      acc += phase_ticks(t.frames[f].dur, speed_q6);
      if (f == g) {
        h0 = acc;
        found = true;
        return;
      }
      acc += phase_ticks(t.frames[f].hold, speed_q6);
    };
    for (uint8_t f = 0; f <= t.loop_end && f < t.n_frames; f++) visit(f);
    for (uint32_t p = 1; p < plays; p++)
      for (uint8_t f = t.loop_start; f <= t.loop_end; f++) visit(f);
    for (uint32_t f = t.loop_end + 1u; f < t.n_frames; f++) visit((uint8_t)f);
    uint32_t h = phase_ticks(t.frames[g].hold, speed_q6);
    from_us_[i] = ticks_to_us(2u * h0 + h) / 2;  // h0 + h / 2 ticks
    move_us_[i] = ticks_to_us(m0);
    decide_us_[i] = ticks_to_us(h0 + h - (h > kEarlyTicks ? kEarlyTicks : h));
  }
}

const char* GuardWatch::feed(uint64_t t_us, int16_t pitch_a16, int16_t roll_a16) {
  if (t_ == nullptr || fired_) return nullptr;
  char a[12], b[12], c[12];
  int aroll = roll_a16 < 0 ? -roll_a16 : roll_a16;
  if (t_->roll_max_a16 > 0 && aroll > t_->roll_max_a16) {
    if (++over_ >= 2) {
      fired_ = true;
      format(why_, sizeof why_, "roll %s deg past %s", deg(a, roll_a16), deg(b, t_->roll_max_a16));
      return why_;
    }
  } else {
    over_ = 0;
  }
  for (uint8_t i = 0; i < n_; i++) {
    const TrickGuard& g = t_->guards[i];
    if (g.push_a16 <= 0) continue;
    if (t_us < move_us_[i]) {
      roll0_[i] = roll_a16;
      have_roll0_[i] = true;
    } else if (t_us < from_us_[i]) {
      if (!have_roll0_[i]) {
        roll0_[i] = roll_a16;
        have_roll0_[i] = true;
      }
      int d = (int)roll_a16 - roll0_[i];
      push_over_[i] = (d > g.push_a16 || -d > g.push_a16) ? (uint8_t)(push_over_[i] + 1) : (uint8_t)0;
      if (push_over_[i] >= 2) {
        fired_ = true;
        format(why_, sizeof why_, "guard %s: roll %s deg, %s while it moved (limit %s)", g.name, deg(a, roll_a16),
               deg(b, (int16_t)d), deg(c, g.push_a16));
        return why_;
      }
    }
  }
  for (uint8_t i = 0; i < n_; i++) {
    if (done_[i]) continue;
    const TrickGuard& g = t_->guards[i];
    if (t_us >= from_us_[i] && t_us <= decide_us_[i]) {
      if (aroll > g.roll_a16) {
        done_[i] = fired_ = true;
        format(why_, sizeof why_, "guard %s: roll %s deg past %s", g.name, deg(a, roll_a16), deg(b, g.roll_a16));
        return why_;
      }
      sum_[i] += -pitch_a16;
      count_[i]++;
    } else if (t_us > decide_us_[i]) {
      done_[i] = true;
      if (count_[i] == 0) {
        fired_ = true;
        format(why_, sizeof why_, "guard %s: no TELEMETRY during its hold", g.name);
        return why_;
      }
      int32_t avg = sum_[i] / (int32_t)count_[i];
      if (avg < g.lo_a16 || avg > g.hi_a16) {
        fired_ = true;
        format(why_, sizeof why_, "guard %s: nose up %s deg outside %s..%s", g.name, deg(a, (int)avg),
               deg(b, g.lo_a16), deg(c, g.hi_a16));
        return why_;
      }
    }
  }
  return nullptr;
}

bool KeySeq::next(TrickKey* k) {
  if (t_ == nullptr || done_) return false;
  for (;;) {
    const Keyframe& f = t_->frames[frame_];
    uint8_t ease = f.flags & kEasingMask;
    if (phase_ == 0) {  // the move
      uint32_t n = phase_ticks(f.dur, speed_);
      if (ease == kStep && n > 1) {
        at_ += 1;  // the jump on the first tick (the safety slew paces it), then wait (phase 1)
        phase_ = 1;
        *k = {at_, frame_, kLinear, false};
        return true;
      }
      at_ += n;
      phase_ = 2;
      *k = {at_, frame_, ease == kCosine ? kCosine : kLinear, false};
      return true;
    }
    if (phase_ == 1) {  // a step's wait at its target
      at_ += phase_ticks(f.dur, speed_) - 1;
      phase_ = 2;
      *k = {at_, frame_, kLinear, true};
      return true;
    }
    // the hold, then the next frame in play order
    phase_ = 0;
    uint32_t h = phase_ticks(f.hold, speed_);
    bool emit = h > 0;
    if (emit) at_ += h;
    TrickKey hk = {at_, frame_, kLinear, true};
    bool again = false;
    if (frame_ == t_->loop_end) {
      plays_ = (uint8_t)(plays_ + 1 < 254 ? plays_ + 1 : 254);
      again = t_->repeats == kRepeatForever || plays_ < t_->repeats;
    }
    if (again)
      frame_ = t_->loop_start;
    else if (++frame_ >= t_->n_frames)
      done_ = true;
    if (emit) {
      *k = hk;
      return true;
    }
    if (done_) return false;
  }
}

int16_t trick_angle(const Trick& t, uint8_t f, uint8_t k, bool mirror) {
  static const uint8_t kMirror[KILO_JOINT_COUNT] = {0, 1, 2, 4, 3, 6, 5, 8, 7, 10, 9};  // LF<->RF, RB<->LB
  uint8_t src = mirror ? kMirror[k] : k;
  for (int i = f; i >= 0; i--) {
    int8_t v = t.frames[i].ang[src];
    if (v == kAngleUnchanged) continue;
    if (mirror && (k == 0 || k == 2)) v = (int8_t)-v;
    return (int16_t)(v * 16);
  }
  return KILO_HEAD_NOT_HELD;
}

}  // namespace tricks
}  // namespace kilo
