#include "behaviour/library.h"
#include "proto/kilo_protocol.h"
#include "util/dmath.h"
#include "util/fmt.h"
#include "util/str.h"

namespace kilo {
namespace behaviour {

namespace {

const uint16_t kSadBeep[3][2] = {{64, 120}, {60, 120}, {55, 300}};  // library.py SAD_BEEP

// Python round(x, 1) (the look angles are never exact halves)
double round1(double x) { return d_round(x * 10.0) / 10.0; }

const char* name_with(char* buf, uint32_t cap, const char* pre, double v, const char* post) {
  char g[16];
  format_g(g, sizeof g, v);
  format(buf, cap, "%s%s%s", pre, g, post);
  return buf;
}

}  // namespace

// ---- actions ----------------------------------------------------------------------------

PostureAction::PostureAction(const char* posture, int8_t balance)
    : Action(name_), posture_(posture), balance_(balance) {
  format(name_, sizeof name_, "posture:%s%s", posture, balance == 1 ? "+bal" : "");
}

Status PostureAction::run(Context& ctx) {
  Scratch& d = ctx.data;
  if (d.has_posture && str_eq(d.posture, posture_) && d.posture_balance == balance_) return Status::Success;
  if (!ctx.body->posture(posture_, balance_)) {
    ctx.error(name(), "the posture was refused");
    return Status::Failure;
  }
  d.has_posture = true;
  d.posture = posture_;
  d.posture_balance = balance_;
  return Status::Success;
}

Status SetBalanceAction::run(Context& ctx) {
  Scratch& d = ctx.data;
  if (d.has_balance && d.balance == enable_) return Status::Success;
  if (!ctx.body->set_balance(enable_)) {
    ctx.error(name(), "SET_BALANCE was refused");
    return Status::Failure;
  }
  d.has_balance = true;
  d.balance = enable_;
  return Status::Success;
}

static Status look_at(Context& ctx, const char* node, double pan, double tilt) {
  if (!ctx.body->look(pan, tilt)) {
    ctx.error(node, "the look was refused");
    return Status::Failure;
  }
  ctx.data.has_head = true;
  ctx.data.head_pan = pan;
  ctx.data.head_tilt = tilt;
  return Status::Success;
}

Status LookAction::run(Context& ctx) { return look_at(ctx, name(), pan_, tilt_); }

Status LookRandom::run(Context& ctx) {
  double p = round1(ctx.rng->uniform(-pan_, pan_));
  double t = round1(ctx.rng->uniform(-tilt_ / 2, tilt_));
  return look_at(ctx, name(), p, t);
}

Status LookToward::run(Context& ctx) {
  const BehaviourPercepts& p = ctx.percepts;
  // float(p.get("stimulus_pan", p.get("face_pan", 0.0)) or 0.0)
  double pan = p.has_stimulus_pan ? p.stimulus_pan : p.has_face_pan ? p.face_pan : 0.0;
  double tilt = p.has_stimulus_tilt ? p.stimulus_tilt : p.has_face_tilt ? p.face_tilt : 0.0;
  return look_at(ctx, name(), pan, tilt);
}

PlayAction::PlayAction(const char* trick) : AsyncAction(name_), trick_(trick) {
  format(name_, sizeof name_, "play:%s", trick);
}

bool PlayAction::start(Context& ctx) {
  body_ = ctx.body;
  handle_ = body_->play(trick_);
  if (handle_ == 0) {
    ctx.error(name(), "the trick could not start");
    return false;
  }
  return true;
}

Status PlayAction::poll(Context& ctx) {
  Status st = ctx.body->play_status(handle_);
  if (st == Status::Failure) ctx.error(name(), "the trick did not end DONE");
  return st;
}

void PlayAction::cancel() {
  if (body_ != nullptr && handle_ != 0) body_->play_cancel(handle_);
  handle_ = 0;
}

Status BeepAction::run(Context& ctx) {
  ctx.body->beep(notes_, n_);
  return Status::Success;
}

Status PoweroffAction::run(Context& ctx) {
  ctx.data.poweroff = true;
  return Status::Success;
}

// ---- safety ------------------------------------------------------------------------------

bool Library::battery_critical(const Context& ctx, const Personality& cfg) {
  const Snapshot& s = ctx.state;
  if (s.has_battery && s.battery_pct <= cfg.critical_battery_pct) return true;
  if (ctx.percepts.battery_critical) return true;
  for (int i = 0; i < s.n_events; i++)
    if (s.events[i].code == KILO_EVENT_LOW_BATTERY && s.events[i].arg >= kCriticalLevel) return true;
  return false;
}

bool Library::fallen_cond(Context& ctx, void* arg) {
  const Personality& cfg = static_cast<Library*>(arg)->cfg_;
  if (ctx.state.fallen) {
    ctx.data.has_settle_until = true;
    ctx.data.settle_until = ctx.now + cfg.fallen_settle_s;
    ctx.data.forget_goals();  // the body dropped our goals
    return true;
  }
  return ctx.data.has_settle_until && ctx.now < ctx.data.settle_until;
}

bool Library::wind_down_cond(Context& ctx, void* arg) {
  const Personality& cfg = static_cast<Library*>(arg)->cfg_;
  if (ctx.data.winding_down) return true;
  if (ctx.data.shutdown_requested || ctx.percepts.shutdown || battery_critical(ctx, cfg)) {
    ctx.data.winding_down = true;
    return true;
  }
  return false;
}

bool Library::low_battery_cond(Context& ctx, void* arg) {
  const Personality& cfg = static_cast<Library*>(arg)->cfg_;
  const Snapshot& s = ctx.state;
  bool low_now = s.mode == KILO_MODE_LOW_BATT || (s.has_battery && s.battery_pct < cfg.low_battery_pct);
  if (low_now) ctx.data.low_battery = true;
  else if (ctx.data.low_battery && (!s.has_battery || s.battery_pct >= cfg.low_battery_pct + kLowBatteryHysteresisPct))
    ctx.data.low_battery = false;
  return ctx.data.low_battery;
}

// ---- the tree ------------------------------------------------------------------------------

Library::Library(const Personality& cfg)
    : cfg_(cfg),
      // idle
      idle_posture_(cfg.idle_posture, -1),
      glance_look_(cfg.idle_look_pan_deg, cfg.idle_look_tilt_deg),
      glance_wait_(1.5, "Wait(1.5s)"),
      glance_center_(0, 0, "look_center"),
      glance_("glance", glance_c_, 3),
      chance_(cfg.idle_look_chance, &glance_, name_with(n_chance_, sizeof n_chance_, "Chance(", cfg.idle_look_chance, ")")),
      force_(&chance_),
      beat_wait_(cfg.idle_look_every_s, name_with(n_idle_wait_, sizeof n_idle_wait_, "Wait(", cfg.idle_look_every_s, "s)")),
      beat_("idle_beat", beat_c_, 2),
      idle_loop_(&beat_, 0, "Forever"),
      idle_seq_("idle_seq", idle_c_, 2),
      // explore
      explore_posture_(cfg.explore_posture, -1),
      scan_look_(cfg.explore_look_pan_deg, cfg.explore_look_tilt_deg),
      scan_wait_(cfg.explore_look_every_s,
                 name_with(n_explore_wait_, sizeof n_explore_wait_, "Wait(", cfg.explore_look_every_s, "s)")),
      scan_("scan", scan_c_, 2),
      scan_repeat_(&scan_, cfg.explore_looks, name_with(n_repeat_, sizeof n_repeat_, "Repeat(", cfg.explore_looks, ")")),
      explore_center_(0, 0, "look_center"),
      explore_seq_("explore_seq", explore_c_, 3),
      // nap
      nap_balance_(false),
      nap_posture_(cfg.nap_posture, -1),
      head_down_(0, -10, "head_down"),
      nap_hold_(1.0, "hold"),
      nap_forever_(&nap_hold_, 0, "Forever"),
      nap_seq_("nap_seq", nap_c_, 4),
      // greet
      greet_stand_("stand", -1),
      greet_play_(cfg.greet_trick),
      greet_center_(0, 0, "look_center"),
      greet_seq_("greet_seq", greet_c_, 4),
      greet_cooldown_(cfg.greet_cooldown_s, &greet_seq_,
                      name_with(n_cooldown_, sizeof n_cooldown_, "Cooldown(", cfg.greet_cooldown_s, "s)")),
      greet_gate_("greet_gate", gate_c_, 2),
      // startle
      startle_stand_("stand", 1),
      startle_wait_(cfg.startle_hold_s, name_with(n_startle_wait_, sizeof n_startle_wait_, "Wait(", cfg.startle_hold_s, "s)")),
      startle_seq_("startle_seq", startle_c_, 3),
      // safety
      sad_beep_(kSadBeep, 3, "sad_beep"),
      sad_force_(&sad_beep_),
      wd_balance_(false),
      wd_rest_("rest", -1),
      wd_settle_(cfg.wind_down_s, "settle"),
      wd_hold_(1.0, "hold"),
      wd_forever_(&wd_hold_, 0, "Forever"),
      wind_down_seq_("wind_down_seq", wd_c_, 6),
      lb_balance_(false),
      lb_rest_("rest", -1),
      lb_hold_(1.0, "hold"),
      lb_forever_(&lb_hold_, 0, "Forever"),
      rest_low_battery_("rest_low_battery", lb_c_, 3),
      root_(behaviours_, safety_, 3, cfg) {
  glance_c_[0] = &glance_look_;
  glance_c_[1] = &glance_wait_;
  glance_c_[2] = &glance_center_;
  beat_c_[0] = &beat_wait_;
  beat_c_[1] = &force_;
  idle_c_[0] = &idle_posture_;
  idle_c_[1] = &idle_loop_;
  scan_c_[0] = &scan_look_;
  scan_c_[1] = &scan_wait_;
  explore_c_[0] = &explore_posture_;
  explore_c_[1] = &scan_repeat_;
  explore_c_[2] = &explore_center_;
  nap_c_[0] = &nap_balance_;
  nap_c_[1] = &nap_posture_;
  nap_c_[2] = &head_down_;
  nap_c_[3] = &nap_forever_;
  greet_c_[0] = &greet_stand_;
  greet_c_[1] = &greet_look_;
  greet_c_[2] = &greet_play_;
  greet_c_[3] = &greet_center_;
  gate_c_[0] = &company_;
  gate_c_[1] = &greet_cooldown_;
  startle_c_[0] = &startle_stand_;
  startle_c_[1] = &startle_look_;
  startle_c_[2] = &startle_wait_;
  wd_c_[0] = &sad_force_;
  wd_c_[1] = &wd_balance_;
  wd_c_[2] = &wd_rest_;
  wd_c_[3] = &wd_settle_;
  wd_c_[4] = &poweroff_;
  wd_c_[5] = &wd_forever_;
  lb_c_[0] = &lb_balance_;
  lb_c_[1] = &lb_rest_;
  lb_c_[2] = &lb_forever_;

  behaviours_[kIdle] = {"idle", &idle_seq_, Activity::Idle, 0.0};
  behaviours_[kExplore] = {"explore", &explore_seq_, Activity::Explore, 0.0};
  behaviours_[kNap] = {"nap", &nap_seq_, Activity::Rest, cfg.nap_min_s};
  behaviours_[kGreet] = {"greet", &greet_gate_, Activity::Social, 0.0};
  behaviours_[kStartle] = {"startle", &startle_seq_, Activity::Active, 0.0};
  safety_[0] = {"fallen", fallen_cond, this, &wait_recover_, Activity::Rest};
  safety_[1] = {"wind_down", wind_down_cond, this, &wind_down_seq_, Activity::Rest};
  safety_[2] = {"low_battery", low_battery_cond, this, &rest_low_battery_, Activity::Rest};
}

}  // namespace behaviour
}  // namespace kilo
