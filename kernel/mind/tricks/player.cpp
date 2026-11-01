#include "tricks/player.h"
#include "link/names.h"
#include "util/fmt.h"
#include "util/log.h"

namespace kilo {
namespace tricks {

const char* done_reason_name(uint8_t reason) {
  switch (reason) {
    case KILO_TRICK_DONE_REASON_DONE: return "DONE";
    case KILO_TRICK_DONE_REASON_STOPPED: return "STOPPED";
    case KILO_TRICK_DONE_REASON_PREEMPTED: return "PREEMPTED";
    case KILO_TRICK_DONE_REASON_FELL: return "FELL";
    default: return "?";
  }
}

void TrickPlayer::on_event(const kilo_event_t& e) {
  if (e.code != KILO_EVENT_TRICK_DONE) return;
  uint16_t arg = (uint16_t)e.arg;
  // client.py trick_done_since: the first TRICK_DONE for our trick_id after the play went out
  bool ours = (step_ == Step::Playing || (step_ == Step::Play && waiting_)) && (arg & 0xFF) == tid_;
  if (ours && !done_seen_) {
    done_seen_ = true;
    done_reason_ = (uint8_t)(arg >> 8);
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
  tid_ = trick_id_for(trick);
  crc_ = trick_crc16(trick);
  chunk_ = 0;
  forced_ = false;
  replayed_ = false;
  done_seen_ = false;
  step_ = Step::Check;
  step_at_ = clock_.now_us();
  result_ = {};
  result_.state = PlayState::Running;
  result_.play = ++plays_;
  result_.trick_id = tid_;
  uint32_t ticks = 0;
  result_.has_expected = total_ticks(trick, speed_q6, &ticks);
  result_.expected_ticks = ticks;
  timeout_us_ = result_.has_expected ? (uint64_t)(ticks + 1) * KILO_TICK_MS * 1000u + kDoneMarginUs : 0;
  return result_.play;
}

void TrickPlayer::stop(uint8_t transition_ticks) {
  stop_pending_ = true;
  stop_ticks_ = transition_ticks;
  if (step_ != Step::Idle && step_ != Step::Playing) fail("stopped");  // abandoned before TRICK_PLAY was acked
}

void TrickPlayer::fail(const char* why) {
  step_ = Step::Idle;
  result_.state = PlayState::Failed;
  format(error_, sizeof error_, "%s", why);
  result_.error = error_;
  logf(log_, LogLevel::Warn, "trick %s: %s", trick_ != nullptr ? trick_->name : "?", error_);
}

void TrickPlayer::finish(uint8_t reason, uint64_t now) {
  step_ = Step::Idle;
  result_.state = PlayState::Done;
  result_.reason = reason;
  result_.elapsed_ms = (uint32_t)((now - played_at_) / 1000u);
  logf(log_, LogLevel::Info, "trick %s: TRICK_DONE %s after %u ms (expected %u ticks)", trick_->name,
       done_reason_name(reason), (unsigned)result_.elapsed_ms, (unsigned)result_.expected_ticks);
}

bool TrickPlayer::send(uint8_t type, const void* payload, uint8_t len, uint64_t now) {
  if (link_.req().state == RequestState::Waiting || !link_.request(type, (const uint8_t*)payload, len)) {
    if (now - step_at_ > kSlotWaitUs && type != KILO_MSG_TRICK_STOP) {
      format(error_, sizeof error_, "%s could not be sent", msg_name(type));
      fail(error_);
    }
    return false;
  }
  waiting_ = true;
  req_type_ = type;
  req_seq_ = link_.req().seq;
  return true;
}

void TrickPlayer::collect() {
  if (!waiting_) return;
  const Request& r = link_.req();
  if (r.type == req_type_ && r.seq == req_seq_ && r.state == RequestState::Waiting) return;
  waiting_ = false;
  bool mine = r.type == req_type_ && r.seq == req_seq_;
  answered(req_type_, mine && r.state == RequestState::Done, mine ? r.status : 0xFF, clock_.now_us());
}

void TrickPlayer::answered(uint8_t type, bool done, uint8_t status, uint64_t now) {
  bool ok = done && status == KILO_ACK_STATUS_OK;
  if (type == KILO_MSG_TRICK_STOP) {
    if (!ok) logf(log_, LogLevel::Warn, "TRICK_STOP: %s", done ? ack_status_name(status) : "no answer");
    return;
  }
  if (step_ == Step::Idle) return;  // abandoned (stop()) while its command was out
  step_at_ = now;
  const char* what = done ? ack_status_name(status) : "no answer";
  switch (step_) {
    case Step::Check:  // TRICK_COMMIT alone: does the body hold exactly this trick?
      if (ok) {
        cached_ = true;
        cached_tid_ = tid_;
        cached_crc_ = crc_;
        cached_reboots_ = link_.state().reboots;
        step_ = Step::Play;
      } else if (done) {
        forced_ = true;
        step_ = Step::Begin;
      } else {
        fail("TRICK_COMMIT: no answer");
      }
      return;
    case Step::Begin:
      if (!ok) break;
      chunk_ = 0;
      step_ = Step::Frames;
      return;
    case Step::Frames:
      if (!ok) break;
      if (++chunk_ >= trick_chunks(*trick_)) step_ = Step::Commit;
      return;
    case Step::Commit:
      if (!ok) break;
      cached_ = true;
      cached_tid_ = tid_;
      cached_crc_ = crc_;
      cached_reboots_ = link_.state().reboots;
      result_.uploaded = true;
      logf(log_, LogLevel::Info, "trick %s: uploaded (id %u, %u frames, crc %04x)", trick_->name, tid_,
           trick_->n_frames, crc_);
      step_ = Step::Play;
      return;
    case Step::Play:
      if (ok) {
        step_ = Step::Playing;
        played_at_ = now;
        logf(log_, LogLevel::Info, "trick %s: playing (id %u, speed %u/64%s)", trick_->name, tid_, speed_,
             mirror_ ? ", mirrored" : "");
        return;
      }
      if (done && status == KILO_ACK_STATUS_NO_TRICK && !replayed_) {  // the body's trick was replaced
        replayed_ = true;
        cached_ = false;
        forced_ = true;
        step_ = Step::Begin;
        return;
      }
      break;
    default:
      return;
  }
  format(error_, sizeof error_, "%s: %s", msg_name(type), what);
  if (type == KILO_MSG_TRICK_FRAMES || type == KILO_MSG_TRICK_BEGIN || type == KILO_MSG_TRICK_COMMIT) cached_ = false;
  fail(error_);
}

void TrickPlayer::poll() {
  uint64_t now = clock_.now_us();
  if (waiting_) return;
  if (stop_pending_) {
    kilo_trick_stop_t s = {stop_ticks_};
    if (!link_.connected()) stop_pending_ = false;
    else if (send(KILO_MSG_TRICK_STOP, &s, sizeof s, now)) stop_pending_ = false;
    if (waiting_) return;
  }
  if (step_ != Step::Idle && step_ != Step::Playing && !link_.connected()) {
    fail("no link to the body");
    return;
  }
  switch (step_) {
    case Step::Idle:
      return;
    case Step::Check: {
      if (cached_ && cached_tid_ == tid_ && cached_crc_ == crc_ && cached_reboots_ == link_.state().reboots) {
        step_ = Step::Play;  // sent next pass
        return;
      }
      if (forced_) {
        step_ = Step::Begin;
        return;
      }
      kilo_trick_commit_t c = {tid_, crc_};
      send(KILO_MSG_TRICK_COMMIT, &c, sizeof c, now);
      return;
    }
    case Step::Begin: {
      cached_ = false;
      kilo_trick_begin_t b;
      trick_begin(*trick_, tid_, &b);
      send(KILO_MSG_TRICK_BEGIN, &b, sizeof b, now);
      return;
    }
    case Step::Frames: {
      kilo_trick_frames_t f;
      uint8_t len = trick_frames_msg(*trick_, chunk_, &f);
      send(KILO_MSG_TRICK_FRAMES, &f, len, now);
      return;
    }
    case Step::Commit: {
      kilo_trick_commit_t c = {tid_, crc_};
      send(KILO_MSG_TRICK_COMMIT, &c, sizeof c, now);
      return;
    }
    case Step::Play: {
      int16_t epoch = link_.state().epoch;
      if (epoch == kNoEpoch) {
        fail("no epoch from the body yet");
        return;
      }
      if (link_.req().state == RequestState::Waiting) {  // keep the posture goal until TRICK_PLAY can go
        send(KILO_MSG_TRICK_PLAY, nullptr, 0, now);         // (only times the wait out)
        return;
      }
      link_.clear_goal(KILO_MSG_SET_POSTURE);  // the trick's end posture replaces it
      kilo_trick_play_t p = {(uint8_t)epoch, tid_, speed_, (uint8_t)(mirror_ ? KILO_TRICK_PLAY_FLAGS_MIRROR : 0)};
      done_seen_ = false;
      send(KILO_MSG_TRICK_PLAY, &p, sizeof p, now);
      return;
    }
    case Step::Playing:
      if (done_seen_) {
        finish(done_reason_, now);
      } else if (timeout_us_ != 0 && now - played_at_ > timeout_us_) {
        stop_pending_ = true;  // client.py _stop_quietly
        stop_ticks_ = 0;
        format(error_, sizeof error_, "no TRICK_DONE within %u ms", (unsigned)(timeout_us_ / 1000u));
        fail(error_);
      }
      return;
  }
}

}  // namespace tricks
}  // namespace kilo
