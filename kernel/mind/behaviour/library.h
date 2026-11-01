// library: the behaviours and the safety branches, built once from a
// personality. A port of kilo/behaviour/library.py (build_root), node for
// node and with the same names (the active path in the log is Python's).
//
// Behaviours: idle (sit; every look_every_s maybe glance around), explore
// (a stub until autonomous walking is allowed: stand and look around N
// times), nap (balance off, rest, head down; a longer dwell), greet (when a
// face or petting is present: stand, look at it, play the greet trick;
// cooldown), startle (stand with balance, look toward the stimulus, hold).
// Safety, in priority order: fallen (wait for the body's own recovery, then
// fallen_settle_s), wind_down (critical battery, a CRITICAL LOW_BATTERY
// event or a shutdown request: sad beep, rest, wait wind_down_s, then ask
// for poweroff; latched), low_battery (balance off, rest, until 5 % above
// the threshold). Posture and balance goals are sent on change only (cached
// in ctx.data) and re-asserted after the runner forgets them.
#ifndef KILO_MIND_BEHAVIOUR_LIBRARY_H
#define KILO_MIND_BEHAVIOUR_LIBRARY_H
#include <stdint.h>
#include "behaviour/body.h"
#include "behaviour/selector.h"

namespace kilo {
namespace behaviour {

constexpr double kLowBatteryHysteresisPct = 5.0;  // library.py LOW_BATTERY_HYSTERESIS_PCT
constexpr int kCriticalLevel = 3;                 // BatteryLevel.CRITICAL (EVENT LOW_BATTERY arg)

// ---- the actions (library.py's helpers) ------------------------------------------------

class PostureAction : public Action {
 public:
  PostureAction(const char* posture, int8_t balance);
 protected:
  Status run(Context& ctx) override;
 private:
  const char* posture_;
  int8_t balance_;
  char name_[32];
};

class SetBalanceAction : public Action {
 public:
  explicit SetBalanceAction(bool enable) : Action(enable ? "balance:on" : "balance:off"), enable_(enable) {}
 protected:
  Status run(Context& ctx) override;
 private:
  bool enable_;
};

class LookAction : public Action {
 public:
  LookAction(double pan, double tilt, const char* name) : Action(name), pan_(pan), tilt_(tilt) {}
 protected:
  Status run(Context& ctx) override;
 private:
  double pan_, tilt_;
};

class LookRandom : public Action {
 public:
  LookRandom(double pan, double tilt) : Action("look_around"), pan_(pan), tilt_(tilt) {}
 protected:
  Status run(Context& ctx) override;
 private:
  double pan_, tilt_;
};

class LookToward : public Action {
 public:
  LookToward() : Action("look_toward") {}
 protected:
  Status run(Context& ctx) override;
};

class PlayAction : public AsyncAction {
 public:
  explicit PlayAction(const char* trick);
 protected:
  bool start(Context& ctx) override;
  Status poll(Context& ctx) override;
  void cancel() override;
 private:
  const char* trick_;
  char name_[32];
  BodyIface* body_ = nullptr;
  uint32_t handle_ = 0;
};

class BeepAction : public Action {
 public:
  BeepAction(const uint16_t (*notes)[2], int n, const char* name) : Action(name), notes_(notes), n_(n) {}
 protected:
  Status run(Context& ctx) override;
 private:
  const uint16_t (*notes_)[2];
  int n_;
};

class PoweroffAction : public Action {
 public:
  PoweroffAction() : Action("poweroff") {}
 protected:
  Status run(Context& ctx) override;
};

class WaitRecover : public Action {
 public:
  WaitRecover() : Action("wait_recover") {}
 protected:
  Status run(Context&) override { return Status::Running; }
};

class HasCompany : public Condition {
 public:
  HasCompany() : Condition("company") {}
 protected:
  bool test(Context& ctx) override { return ctx.percepts.face || ctx.percepts.pet; }
};

// ---- the tree ------------------------------------------------------------------------------

class Library {
 public:
  explicit Library(const Personality& cfg);
  UtilitySelector& root() { return root_; }

  static bool battery_critical(const Context& ctx, const Personality& cfg);

 private:
  static bool fallen_cond(Context& ctx, void* arg);
  static bool wind_down_cond(Context& ctx, void* arg);
  static bool low_battery_cond(Context& ctx, void* arg);

  const Personality& cfg_;
  char n_idle_wait_[24], n_chance_[24], n_explore_wait_[24], n_repeat_[24], n_cooldown_[24], n_startle_wait_[24];

  // idle
  PostureAction idle_posture_;
  LookRandom glance_look_;
  Wait glance_wait_;
  LookAction glance_center_;
  Node* glance_c_[3];
  Sequence glance_;
  Chance chance_;
  ForceSuccess force_;
  Wait beat_wait_;
  Node* beat_c_[2];
  Sequence beat_;
  Repeat idle_loop_;
  Node* idle_c_[2];
  Sequence idle_seq_;
  // explore
  PostureAction explore_posture_;
  LookRandom scan_look_;
  Wait scan_wait_;
  Node* scan_c_[2];
  Sequence scan_;
  Repeat scan_repeat_;
  LookAction explore_center_;
  Node* explore_c_[3];
  Sequence explore_seq_;
  // nap
  SetBalanceAction nap_balance_;
  PostureAction nap_posture_;
  LookAction head_down_;
  Wait nap_hold_;
  Repeat nap_forever_;
  Node* nap_c_[4];
  Sequence nap_seq_;
  // greet
  HasCompany company_;
  PostureAction greet_stand_;
  LookToward greet_look_;
  PlayAction greet_play_;
  LookAction greet_center_;
  Node* greet_c_[4];
  Sequence greet_seq_;
  Cooldown greet_cooldown_;
  Node* gate_c_[2];
  Sequence greet_gate_;
  // startle
  PostureAction startle_stand_;
  LookToward startle_look_;
  Wait startle_wait_;
  Node* startle_c_[3];
  Sequence startle_seq_;
  // safety
  WaitRecover wait_recover_;
  BeepAction sad_beep_;
  ForceSuccess sad_force_;
  SetBalanceAction wd_balance_;
  PostureAction wd_rest_;
  Wait wd_settle_;
  PoweroffAction poweroff_;
  Wait wd_hold_;
  Repeat wd_forever_;
  Node* wd_c_[6];
  Sequence wind_down_seq_;
  SetBalanceAction lb_balance_;
  PostureAction lb_rest_;
  Wait lb_hold_;
  Repeat lb_forever_;
  Node* lb_c_[3];
  Sequence rest_low_battery_;

  BehaviourEntry behaviours_[kBehaviours];
  SafetyEntry safety_[3];
  UtilitySelector root_;
};

}  // namespace behaviour
}  // namespace kilo

#endif
