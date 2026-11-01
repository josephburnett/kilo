// selector: the utility root. A port of kilo/behaviour/selector.py:
//
// 1. Safety branches in order (fallen, wind-down, low battery): the first
//    whose condition holds is ticked; if it does not FAIL it wins and the
//    running behaviour is halted.
// 2. Scores: score[b] = sum(weight x term) + noise[b]. Terms are the
//    drives, fatigue (1 - energy), calm (1 - arousal), percepts as 0/1
//    (face, pet, near, approaching, alone), base (1) and products ("a*b").
//    The noise is uniform in +-selector.noise, redrawn every noise_period_s.
// 3. Hysteresis: the running behaviour keeps going unless a challenger
//    scores at least margin more and the current one has run min_dwell_s
//    (or its own longer min_dwell); urgent behaviours skip the dwell; when
//    the current one has finished the best is taken freely.
// 4. The choice is ticked; if it FAILs, the current one, then the rest by
//    score, are tried in the same tick.
// last() holds the choice, its reason and the scores.
#ifndef KILO_MIND_BEHAVIOUR_SELECTOR_H
#define KILO_MIND_BEHAVIOUR_SELECTOR_H
#include <stdint.h>
#include "behaviour/personality.h"
#include "behaviour/tree.h"

namespace kilo {
namespace behaviour {

struct BehaviourEntry {
  const char* name;
  Node* tree;
  Activity activity;
  double min_dwell;  // a longer dwell for this behaviour (nap)
};

struct SafetyEntry {
  const char* name;
  bool (*condition)(Context& ctx, void* arg);
  void* arg;
  Node* tree;
  Activity activity;
};

struct Choice {
  int chosen = -1;  // behaviour index, or kSafetyBase + safety index; -1 none
  const char* name = nullptr;
  char reason[32] = "";
  bool has_status = false;
  Status status = Status::Failure;
  bool has_scores = false;
  double scores[kBehaviours] = {};
  Activity activity = Activity::Idle;
};

class UtilitySelector : public Node {
 public:
  static constexpr int kSafetyBase = 100;
  static constexpr int kMaxSafety = 4;
  // behaviours: exactly kBehaviours, in BehaviourId order.
  UtilitySelector(const BehaviourEntry* behaviours, const SafetyEntry* safety, int n_safety, const Personality& cfg,
                  const char* name = "root");
  void halt() override;
  // selector.score(): the scores this tick (redraws the noise when due).
  void score(Context& ctx, double* out);
  const Choice& last() const { return last_; }
  uint32_t switches() const { return switches_; }
  int current() const { return current_; }
  const char* current_name() const;

 protected:
  Status do_tick(Context& ctx) override;

 private:
  Node* node(int which) const;
  void switch_to(Context& ctx, int which);

  const BehaviourEntry* b_;
  const SafetyEntry* s_;
  int n_safety_;
  const Personality& cfg_;
  int current_ = -1;
  double since_ = 0;
  bool finished_ = true;
  double noise_[kBehaviours] = {};
  bool has_noise_at_ = false;
  double noise_at_ = 0;
  Choice last_;
  uint32_t switches_ = 0;
};

// selector.terms(): the term values from the drives and percepts.
void terms(const Context& ctx, double* out /* kTerms */);

}  // namespace behaviour
}  // namespace kilo

#endif
