#include "intent/intent.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/str.h"

namespace kilo {
namespace intent {

uint8_t posture_of(State s) {
  switch (s) {
    case State::Sitting:
      return KILO_POSTURE_SIT;
    case State::Resting:
      return KILO_POSTURE_REST;
    default:
      return KILO_POSTURE_STAND;
  }
}

namespace {
const char* posture_word(uint8_t posture) {
  return posture == KILO_POSTURE_SIT ? "sit" : posture == KILO_POSTURE_REST ? "rest" : "stand";
}
}  // namespace

void Intent::mark(const char* line) {
  if (host_ != nullptr) host_->intent_mark(line);
}

void Intent::enter(Phase p, uint64_t now) {
  phase_ = p;
  phase_at_ = now;
}

bool Intent::latch(uint8_t posture, bool balance) {
  kilo_set_posture_t g = {0, posture, 0, (uint8_t)(balance ? KILO_POSTURE_FLAGS_BALANCE : 0)};
  return link_.set_goal(KILO_MSG_SET_POSTURE, (const uint8_t*)&g, sizeof g);
}

bool Intent::be(State goal, int8_t balance, const char* who, const char** why) {
  if (goal_index(goal) < 0) {
    if (why != nullptr) *why = "not a state to ask for";
    return false;
  }
  if (link_.state().proto_ok == 0) {
    if (why != nullptr) *why = "protocol mismatch: refusing to drive";
    return false;
  }
  uint64_t now = clock_.now_us();
  bool again = active() && goal == goal_;
  goal_ = goal;
  balance_ = balance == 1;
  format(who_, sizeof who_, "%s", who != nullptr ? who : "");
  if (again) return true;  // the same goal while it is on its way: nothing new
  n_.asked++;
  ended_ = nullptr;
  reached_ = false;
  if (!active()) {
    steps_ = fails_ = 0;
    asked_at_ = now;
    known_play_ = player_.result().play;
    // a posture takes the legs from the stream: end a walk first, so no FEET re-enters GAIT under the move
    if (walker_.active()) walker_.abort();
    enter(Phase::Settle, now);
  }
  // (a new goal while a step runs: the step ends, then Check goes on toward the new goal)
  logf(log_, LogLevel::Info, "intent: be %s (%s; he is %s)", state_name(goal), who_, state_name(est_.state()));
  char l[96];
  format(l, sizeof l, "intent goal=%s who=%s state=%s", state_name(goal), who_[0] != 0 ? who_ : "-",
         state_name(est_.state()));
  mark(l);
  return true;
}

void Intent::cancel(const char* why) {
  if (!active()) return;
  if (phase_ == Phase::Move && move_started_ && player_.busy()) player_.stop();
  n_.cancelled++;
  logf(log_, LogLevel::Info, "intent: %s dropped (%s)", state_name(goal_), why);
  format(ended_buf_, sizeof ended_buf_, "dropped: %s", why);
  ended_ = ended_buf_;
  reached_ = false;
  char l[96];
  format(l, sizeof l, "intent end=cancelled goal=%s why=%s", state_name(goal_), why);
  mark(l);
  phase_ = Phase::Idle;
}

void Intent::on_event(const kilo_event_t& e) {
  if (phase_ != Phase::Reflex) return;
  if (e.code == KILO_EVENT_RECOVERED) recovered_ = true;
  if (e.code == KILO_EVENT_RECOVERY_FAILED) recovery_failed_ = true;
}

void Intent::finish(bool reached, const char* why) {
  uint64_t now = clock_.now_us();
  unsigned ms = (unsigned)((now - asked_at_) / 1000u);
  reached_ = reached;
  format(ended_buf_, sizeof ended_buf_, "%s", why);
  ended_ = ended_buf_;
  if (reached) {
    n_.reached++;
    logf(log_, LogLevel::Info, "intent: %s reached (%d step%s, %u ms)", state_name(goal_), steps_,
         steps_ == 1 ? "" : "s", ms);
  } else {
    n_.gave_up++;
    logf(log_, LogLevel::Warn, "intent: %s given up: %s (%d step%s, %d failed, %u ms)", state_name(goal_), why, steps_,
         steps_ == 1 ? "" : "s", fails_, ms);
  }
  char l[120];
  format(l, sizeof l, "intent end=%s goal=%s steps=%d failed=%d ms=%u why=%s", reached ? "reached" : "gave_up",
         state_name(goal_), steps_, fails_, ms, why);
  mark(l);
  phase_ = Phase::Idle;
}

void Intent::give_up(State st, const char* why) {
  // leave him in the safest posture he can reach: upright, the body's own REST; fallen, as he lies
  if (!fallen(st) && st != State::Resting) {
    bool ok = latch(KILO_POSTURE_REST, false);
    logf(log_, LogLevel::Warn, "intent: he is left in REST (the body's own change)%s", ok ? "" : ": REFUSED by the link");
  }
  finish(false, why);
}

// The body's own posture change toward `to` (a step of its own: Check follows).
void Intent::body_posture(State to, uint64_t now, const char* why) {
  uint8_t posture = posture_of(to);
  from_ = est_.state();
  to_ = to;
  move_ = nullptr;
  steps_++;
  n_.postures++;
  if (host_ != nullptr) host_->intent_moves();
  bool ok = latch(posture, to == goal_ && balance_);
  logf(log_, ok ? LogLevel::Info : LogLevel::Warn, "intent: step %d: the body's own change to %s (%s)%s", steps_,
       posture_word(posture), why, ok ? "" : ": REFUSED by the link");
  char l[96];
  format(l, sizeof l, "intent step=%d kind=posture to=%s from=%s", steps_, state_name(to), state_name(from_));
  mark(l);
  enter(Phase::Posture, now);
}

bool Intent::reflex_busy() const {
  const BodyState& b = link_.state();
  return b.has_telemetry && (b.telemetry.mode == KILO_MODE_FALLEN || b.telemetry.mode == KILO_MODE_RECOVERING);
}

// The placeholder step from a fallen state: wait for the body's fall reflex (it ends in STAND).
void Intent::wait_reflex(State st, uint64_t now) {
  from_ = st;
  if (!fallen(st)) to_ = State::Standing;
  move_ = nullptr;
  recovered_ = recovery_failed_ = false;
  steps_++;
  n_.reflex_waits++;
  logf(log_, LogLevel::Info, "intent: step %d: he is %s: waiting for the body's fall reflex to right him", steps_,
       fallen(st) ? state_name(st) : "fallen (the body says)");
  char l[96];
  format(l, sizeof l, "intent step=%d kind=wait_reflex from=%s", steps_, state_name(st));
  mark(l);
  enter(Phase::Reflex, now);
}

void Intent::start_edge(const Edge& e, State st, uint64_t now) {
  switch (e.kind) {
    case EdgeKind::Move: {
      from_ = st;
      to_ = e.to;
      move_ = e.move;
      move_started_ = false;
      steps_++;
      enter(Phase::Move, now);
      return;  // poll() starts it once the trick player is free
    }
    case EdgeKind::Posture:
      body_posture(e.to, now, st == State::Unknown ? "no pose the model knows" : "no checked move leads there");
      return;
    case EdgeKind::WaitReflex:
      to_ = e.to;
      wait_reflex(st, now);
      return;
    default:
      give_up(st, "no step leads from there");
  }
}

void Intent::decide(State st, uint64_t now) {
  const BodyState& b = link_.state();
  if (st == goal_) {
    // there: the goal's posture latched, as a posture key does today (the body is in it or takes it)
    bool ok = latch(posture_of(goal_), balance_);
    finish(ok, ok ? "there" : "the link refused SET_POSTURE");
    return;
  }
  if (steps_ >= kMaxSteps) {
    give_up(st, "too many steps");
    return;
  }
  if (fails_ >= kMaxFails) {
    give_up(st, "three steps did not end where the model said");
    return;
  }
  if (b.has_telemetry && b.telemetry.mode == KILO_MODE_LIMP && !fallen(st)) {
    // the servos are off: REST first (a trick is refused in LIMP), then look again
    bool ok = latch(KILO_POSTURE_REST, false);
    logf(log_, ok ? LogLevel::Info : LogLevel::Warn, "intent: the servos are off: REST first%s",
         ok ? "" : ": REFUSED by the link");
    if (host_ != nullptr) host_->intent_moves();
    enter(Phase::Wake, now);
    return;
  }
  int g = goal_index(goal_);
  int idx = (int)st < kStates && g >= 0 ? kNext[(int)st][g] : -1;
  if (idx < 0) {
    give_up(st, "no step leads from there");
    return;
  }
  start_edge(kEdges[idx], st, now);
}

void Intent::poll() {
  if (phase_ == Phase::Idle) return;
  uint64_t now = clock_.now_us();
  State st = est_.state();
  switch (phase_) {
    case Phase::Idle:
      return;
    case Phase::Settle:
      if (reflex_busy()) {  // the reflex has the joints and refuses every goal: wait for it
        wait_reflex(st, now);
        return;
      }
      if (st == State::Moving) {
        if (now - phase_at_ < kSettleWaitUs) return;
        body_posture(goal_, now, "he did not come to rest");  // as a posture key does today
        return;
      }
      decide(st, now);
      return;
    case Phase::Wake:
      if (now - phase_at_ < kWakeUs) return;
      enter(Phase::Settle, now);
      return;
    case Phase::Move: {
      if (!move_started_) {
        if (player_.busy()) {
          if (now - phase_at_ < kPlayerWaitUs) return;
          steps_--;
          body_posture(to_, now, "the trick player stayed busy");
          return;
        }
        const char* why = nullptr;
        if (host_ != nullptr) host_->intent_moves();
        play_ = player_.start(*move_, false, tricks::kSpeedUnity, &why);
        if (play_ == 0) {
          logf(log_, LogLevel::Warn, "intent: the move %s was refused: %s", move_->name, why != nullptr ? why : "?");
          steps_--;
          body_posture(to_, now, "the move was refused");
          return;
        }
        move_started_ = true;
        known_play_ = play_;
        n_.moves++;
        logf(log_, LogLevel::Info, "intent: step %d: %s -> %s by the move %s (planned from the model)", steps_,
             state_name(from_), state_name(to_), move_->name);
        char l[120];
        format(l, sizeof l, "intent step=%d kind=move move=%s from=%s to=%s play=%u", steps_, move_->name,
               state_name(from_), state_name(to_), (unsigned)play_);
        mark(l);
        return;
      }
      if (player_.busy()) return;
      const tricks::PlayResult& r = player_.result();
      bool done = r.play == play_ && r.state == tricks::PlayState::Done && r.reason == KILO_TRICK_DONE_REASON_DONE;
      if (done) {
        // the body is in the move's end posture: latch it, so nothing else pulls him out of it
        latch(move_->end_posture, to_ == goal_ && balance_);
      } else {
        logf(log_, LogLevel::Warn, "intent: the move %s did not finish: %s", move_->name,
             r.play != play_ ? "another play took the trick player"
                             : r.state == tricks::PlayState::Failed ? (r.error != nullptr ? r.error : "failed")
                                                                   : r.guard != nullptr ? r.guard
                                                                                        : tricks::done_reason_name(r.reason));
      }
      enter(Phase::Check, now);
      return;
    }
    case Phase::Posture:
      if (now - phase_at_ < kPostureMinUs) return;
      enter(Phase::Check, now);
      return;
    case Phase::Reflex:
      if (recovery_failed_) {
        give_up(st, "the body's fall reflex could not right him: he needs a hand");
        return;
      }
      if (recovered_ || (!reflex_busy() && !fallen(st) && st != State::Moving && st != State::Unknown)) {
        // righted (by the reflex, which ends in STAND, or by a hand): latch where he is meant to be
        if (recovered_) latch(KILO_POSTURE_STAND, goal_ == State::Standing && balance_);
        enter(Phase::Check, now);
        return;
      }
      if (now - phase_at_ >= kReflexWaitUs) give_up(st, "nothing righted him: he needs a hand");
      return;
    case Phase::Check: {
      if (now - phase_at_ < kCheckMinUs) return;  // the estimate needs this long to follow
      bool timeout = now - phase_at_ >= kCheckWaitUs;
      if (st == State::Moving && !timeout) return;
      if (st != to_) {
        fails_++;
        n_.failed_steps++;
        logf(log_, LogLevel::Warn, "intent: step %d meant to end %s; he is %s", steps_, state_name(to_),
             state_name(st));
      }
      char l[96];
      format(l, sizeof l, "intent step=%d ended=%s meant=%s", steps_, state_name(st), state_name(to_));
      mark(l);
      if (st == State::Moving) {  // never settled: nothing to plan from
        give_up(st, "he did not come to rest");
        return;
      }
      decide(st, now);
      return;
    }
  }
}

}  // namespace intent
}  // namespace kilo
