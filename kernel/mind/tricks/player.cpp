#include "tricks/player.h"
#include "link/consts.h"
#include "link/names.h"
#include "proto/kilo_names.h"
#include "util/fmt.h"
#include "util/log.h"

namespace kilo {
namespace tricks {

namespace {

constexpr int kLookahead = KILO_POSE_LOOKAHEAD_TICKS;
constexpr int64_t kTickUs = KILO_TICK_MS * 1000;

int16_t clamp480(int v) { return (int16_t)(v < -480 ? -480 : v > 480 ? 480 : v); }

}  // namespace

const char* done_reason_name(uint8_t reason) {
  switch (reason) {
    case kDone:
      return "DONE";
    case kStopped:
      return "STOPPED";
    case kPreempted:
      return "PREEMPTED";
    case kFell:
      return "FELL";
    default:
      return "?";
  }
}

void TrickPlayer::on_telemetry(const kilo_telemetry_t& t) {
  body_.add(t.tick, link_.state().telemetry_at_us);
  if (step_ == Step::Playing && t.mode == KILO_MODE_STREAM) stream_seen_ = true;
}

void TrickPlayer::on_event(const kilo_event_t& e) {
  if (step_ != Step::Playing) return;
  uint64_t now = clock_.now_us();
  if (e.code == KILO_EVENT_FELL) {
    ended(kFell, now);
  } else if (e.code == KILO_EVENT_STREAM_STOPPED) {
    ended(kPreempted, now);
  } else if (e.code == KILO_EVENT_GOAL_REFUSED && ((uint16_t)e.arg >> 8) == KILO_MSG_POSE) {
    format(error_, sizeof error_, "the body refused POSE: %s", ack_status_name((uint8_t)(e.arg & 0xFF)));
    fail(error_);
  }
}

uint32_t TrickPlayer::start(const Trick& trick, bool mirror, uint8_t speed_q6, const char** why) {
  const char* no = nullptr;
  if (step_ != Step::Idle) no = "a trick is still playing: stop it first";
  else if (speed_q6 < kSpeedMin) no = "speed outside 0.25..3.98";
  else no = trick_problem(trick);
  if (no != nullptr) {
    if (why != nullptr) *why = no;
    logf(log_, LogLevel::Warn, "trick %s: %s", trick.name, no);
    return 0;
  }
  trick_ = &trick;
  mirror_ = mirror;
  speed_ = speed_q6;
  step_ = Step::Starting;
  step_at_ = clock_.now_us();
  played_at_ = 0;
  seq_ = KeySeq(trick, speed_q6);
  have_key_ = seq_.next(&key_);
  all_sent_ = !have_key_;
  sent_ = 0;
  stream_seen_ = false;
  result_ = {};
  result_.state = PlayState::Running;
  result_.play = ++plays_;
  result_.trick_id = trick_id_for(trick);
  uint32_t ticks = 0;
  result_.has_expected = total_ticks(trick, speed_q6, &ticks);
  result_.expected_ticks = ticks;
  watch_.reset(trick, speed_q6);
  return result_.play;
}

void TrickPlayer::stop(int transition_ticks) {
  if (step_ == Step::Starting) {
    fail("stopped");
    return;
  }
  if (step_ != Step::Playing) return;
  uint8_t t = transition_ticks == kTrickStopTicks ? stop_ticks() : (uint8_t)transition_ticks;
  end_posture(t);
  ended(kStopped, clock_.now_us());
}

void TrickPlayer::fail(const char* why) {
  if (step_ == Step::Playing) end_posture(0);  // the body holds the last key: its end posture
  link_.hold_goal(KILO_MSG_SET_POSTURE, false);
  own_goal_ = false;
  step_ = Step::Idle;
  result_.state = PlayState::Failed;
  if (why != error_) format(error_, sizeof error_, "%s", why);
  result_.error = error_;
  logf(log_, LogLevel::Warn, "trick %s: %s", trick_ != nullptr ? trick_->name : "?", error_);
}

int64_t TrickPlayer::body_tick_now(uint64_t now) const {
  double t = 0;
  if (!body_.now(now, &t)) return -1;
  return (int64_t)t;
}

// The end posture as the latched SET_POSTURE goal (the body ends the stream
// with it); 0 ticks = XFADE_TICKS. A posture someone asked for while it
// played (held back meanwhile) goes instead: the body takes that one.
void TrickPlayer::end_posture(uint8_t ticks) {
  if (link_.goal(KILO_MSG_SET_POSTURE) == nullptr) {
    uint8_t p[KILO_SET_POSTURE_LEN] = {0, trick_->end_posture,
                                       ticks != 0 ? ticks : (uint8_t)KILO_PARAM_XFADE_TICKS_DEFAULT, posture_flags_};
    own_goal_ = link_.set_goal(KILO_MSG_SET_POSTURE, p, sizeof p);
  }
  link_.hold_goal(KILO_MSG_SET_POSTURE, false);
}

void TrickPlayer::ended(uint8_t reason, uint64_t now) {
  reason_ = reason;
  result_.elapsed_ms = played_at_ != 0 && now > played_at_ ? (uint32_t)((now - played_at_) / 1000u) : 0;
  if (reason == kDone || reason == kStopped) {
    step_ = Step::Ending;  // wait for the body to leave the stream, then clear the goal
    step_at_ = now;
    return;
  }
  finish();
}

void TrickPlayer::finish() {
  if (step_ == Step::Ending && own_goal_) link_.clear_goal(KILO_MSG_SET_POSTURE);
  own_goal_ = false;
  link_.hold_goal(KILO_MSG_SET_POSTURE, false);
  step_ = Step::Idle;
  result_.state = PlayState::Done;
  result_.reason = reason_;
  logf(log_, LogLevel::Info, "trick %s: %s after %u ms (expected %u ticks, %u keys)", trick_->name,
       done_reason_name(reason_), (unsigned)result_.elapsed_ms, (unsigned)result_.expected_ticks, (unsigned)sent_);
}

// The trick starts on the body's next tick; frame 0's key at its dur from
// there, but never closer than the look-ahead (a key must reach the body
// before its tick).
bool TrickPlayer::begin(uint64_t now) {
  int64_t cur = body_tick_now(now);
  if (cur < 0) return false;
  base_ = cur;
  played_at_ = now;
  const Goal* g = link_.goal(KILO_MSG_SET_POSTURE);
  posture_flags_ = g != nullptr ? g->payload[3] : 0;
  link_.clear_goal(KILO_MSG_SET_POSTURE);  // a refresh would end the stream: postures wait for its end
  link_.hold_goal(KILO_MSG_SET_POSTURE, true);
  own_goal_ = false;
  epoch_ = link_.state().epoch;
  seen_telemetry_ = link_.state().telemetry_count;
  step_ = Step::Playing;
  logf(log_, LogLevel::Info, "trick %s: playing (id %u, speed %u/64%s), from body tick %u", trick_->name,
       result_.trick_id, speed_, mirror_ ? ", mirrored" : "", (unsigned)(base_ & 0xFFFF));
  return true;
}

bool TrickPlayer::send_key(const TrickKey& k) {
  const Trick& t = *trick_;
  const Keyframe& f = t.frames[k.frame];
  kilo_pose_t m = {};
  // (a key closer than the look-ahead could arrive after its tick: it waits, and so do the ones after it)
  int64_t tick = base_ + (int64_t)k.at;
  if (tick < base_ + kLookahead) tick = base_ + kLookahead;
  if (sent_ != 0 && tick <= last_sent_) tick = last_sent_ + 1;
  m.epoch = (uint8_t)epoch_;
  m.tick = (uint16_t)(tick & 0xFFFF);
  m.flags = KILO_POSE_FLAGS_JOINT_LF | KILO_POSE_FLAGS_JOINT_RF | KILO_POSE_FLAGS_JOINT_RB | KILO_POSE_FLAGS_JOINT_LB;
  if (f.flags & kFrameBalance) m.flags |= KILO_POSE_FLAGS_BALANCE;
  if (t.flags & kTrickFallOk) m.flags |= KILO_POSE_FLAGS_FALL_OK;
  if (k.ease == kCosine) m.flags |= KILO_POSE_FLAGS_EASE;
  for (uint8_t l = 0; l < 4; l++) {
    m.leg[2 * l] = trick_angle(t, k.frame, (uint8_t)(3 + l), mirror_);
    m.leg[2 * l + 1] = trick_angle(t, k.frame, (uint8_t)(7 + l), mirror_);
  }
  for (uint8_t j = 0; j < 3; j++) {
    bool driven = (t.joint_mask >> kCanonical[j]) & 1u;
    m.head[j] = driven ? trick_angle(t, k.frame, j, mirror_) : (int16_t)KILO_HEAD_NOT_HELD;
  }
  m.pitch = clamp480(t.exp_pitch * 16);
  m.roll = clamp480((mirror_ ? -t.exp_roll : t.exp_roll) * 16);
  m.support = 0;  // joint angles: no foot model (TELEMETRY support 0 while it plays)
  if (!link_.send_stream(KILO_MSG_POSE, (const uint8_t*)&m, sizeof m)) return false;
  last_sent_ = tick;
  sent_++;
  return true;
}

void TrickPlayer::poll() {
  uint64_t now = clock_.now_us();
  switch (step_) {
    case Step::Idle:
      return;
    case Step::Starting:
      if (!link_.connected()) return fail("no link to the body");
      if (link_.contact() && link_.state().epoch != kNoEpoch && begin(now)) break;
      if (now - step_at_ > kStartWaitUs) fail("the body was not heard (no contact, epoch or TELEMETRY)");
      return;
    case Step::Ending: {
      const BodyState& st = link_.state();
      bool left = st.has_telemetry && st.telemetry_at_us > step_at_ && st.telemetry.mode != KILO_MODE_STREAM;
      if (left || now - step_at_ > kEndWaitUs || !link_.connected()) finish();
      return;
    }
    case Step::Playing:
      break;
  }
  if (!link_.connected()) return fail("no link to the body");
  const BodyState& st = link_.state();
  if (st.epoch != epoch_) return ended(kPreempted, now);  // the body moved on (a reflex, a reboot)
  // The keys due: each once the one before it is within the look-ahead.
  int64_t cur = body_tick_now(now);
  while (have_key_ && (sent_ == 0 || last_sent_ <= cur + kLookahead)) {
    if (!send_key(key_)) return fail("POSE could not be sent");
    have_key_ = seq_.next(&key_);
    all_sent_ = !have_key_;
  }
  // The guards, on every new TELEMETRY.
  if (!watch_.fired() && st.telemetry_count != seen_telemetry_) {
    seen_telemetry_ = st.telemetry_count;
    uint64_t at = st.telemetry_at_us > played_at_ ? st.telemetry_at_us - played_at_ : 0;
    const char* why = watch_.feed(at, st.telemetry.pitch, st.telemetry.roll);
    if (why != nullptr) {
      result_.guard = why;
      logf(log_, LogLevel::Warn, "trick %s: %s: stopping", trick_->name, why);
      end_posture(stop_ticks());
      return ended(kStopped, now);
    }
    // Someone else's goal or a reflex took the joints without an event seen.
    if (stream_seen_ && st.telemetry.mode != KILO_MODE_STREAM) return ended(kPreempted, now);
  }
  // The end: the body's own tick has reached the last key.
  if (all_sent_ && body_.tick() >= last_sent_) {
    end_posture(0);
    return ended(kDone, now);
  }
  if (all_sent_ && now > played_at_ && body_.tick() < last_sent_ &&
      now - played_at_ > (uint64_t)((last_sent_ - base_) * kTickUs) + kDoneMarginUs) {
    format(error_, sizeof error_, "the body did not reach the last key within %u ms", (unsigned)(kDoneMarginUs / 1000u));
    fail(error_);
  }
}

}  // namespace tricks
}  // namespace kilo
