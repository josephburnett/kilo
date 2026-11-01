#include "behaviour/selector.h"
#include "util/dmath.h"
#include "util/fmt.h"

namespace kilo {
namespace behaviour {

namespace {

int str_cmp(const char* a, const char* b) {
  while (*a != '\0' && *a == *b) {
    a++;
    b++;
  }
  return (unsigned char)*a - (unsigned char)*b;
}

void set_reason(Choice& c, const char* r) { format(c.reason, sizeof c.reason, "%s", r); }

}  // namespace

const char* behaviour_name(uint8_t b) {
  static const char* const kNames[kBehaviours] = {"idle", "explore", "nap", "greet", "startle"};
  return b < kBehaviours ? kNames[b] : "?";
}

const char* term_name(uint8_t t) {
  static const char* const kNames[kTerms] = {"base", "energy", "fatigue", "curiosity", "sociability", "arousal",
                                             "calm", "face", "pet", "near", "approaching", "alone"};
  return t < kTerms ? kNames[t] : "?";
}

void terms(const Context& ctx, double* t) {
  const Drives& d = ctx.drives;
  const BehaviourPercepts& p = ctx.percepts;
  double face = p.face ? 1.0 : 0.0, pet = p.pet ? 1.0 : 0.0;
  t[kBase] = 1.0;
  t[kEnergy] = d.energy;
  t[kFatigue] = 1.0 - d.energy;
  t[kCuriosity] = d.curiosity;
  t[kSociability] = d.sociability;
  t[kArousal] = d.arousal;
  t[kCalm] = 1.0 - d.arousal;
  t[kFace] = face;
  t[kPet] = pet;
  t[kNear] = p.near ? 1.0 : 0.0;
  t[kApproaching] = p.approaching ? 1.0 : 0.0;
  t[kAlone] = !(p.face || p.pet) ? 1.0 : 0.0;
}

UtilitySelector::UtilitySelector(const BehaviourEntry* behaviours, const SafetyEntry* safety, int n_safety,
                                 const Personality& cfg, const char* name)
    : Node(name), b_(behaviours), s_(safety), n_safety_(n_safety < kMaxSafety ? n_safety : kMaxSafety), cfg_(cfg) {}

const char* UtilitySelector::current_name() const {
  if (current_ < 0) return nullptr;
  return current_ >= kSafetyBase ? s_[current_ - kSafetyBase].name : b_[current_].name;
}

Node* UtilitySelector::node(int which) const {
  return which >= kSafetyBase ? s_[which - kSafetyBase].tree : b_[which].tree;
}

void UtilitySelector::switch_to(Context& ctx, int which) {
  if (which == current_) return;
  if (current_ >= 0) node(current_)->halt();
  current_ = which;
  since_ = ctx.now;
  switches_++;
}

void UtilitySelector::score(Context& ctx, double* out) {
  if (!has_noise_at_ || ctx.now - noise_at_ >= cfg_.sel_noise_period_s) {
    has_noise_at_ = true;
    noise_at_ = ctx.now;
    double amp = cfg_.sel_noise;
    for (int b = 0; b < kBehaviours; b++) noise_[b] = amp != 0.0 ? ctx.rng->uniform(-amp, amp) : 0.0;
  }
  double t[kTerms];
  terms(ctx, t);
  for (int b = 0; b < kBehaviours; b++) {
    PySum s;
    for (int i = 0; i < cfg_.n_weights[b]; i++) {
      const Weight& w = cfg_.weights[b][i];
      double v = t[w.factors[0]];
      if (w.n > 1) {
        v = 1.0;
        for (int f = 0; f < w.n; f++) v *= t[w.factors[f]];
      }
      s.add(w.w * v);
    }
    out[b] = s.value() + noise_[b];
  }
}

Status UtilitySelector::do_tick(Context& ctx) {
  for (int i = 0; i < n_safety_; i++) {
    const SafetyEntry& s = s_[i];
    if (!s.condition(ctx, s.arg)) continue;
    ctx.push(s.name);
    Status st = s.tree->tick(ctx);
    ctx.pop();
    if (st == Status::Failure) {
      if (current_ == kSafetyBase + i) {
        s.tree->halt();
        current_ = -1;
        finished_ = true;
      }
      continue;
    }
    switch_to(ctx, kSafetyBase + i);
    finished_ = st == Status::Success;
    last_ = Choice();
    last_.chosen = kSafetyBase + i;
    last_.name = s.name;
    set_reason(last_, "safety");
    last_.has_status = true;
    last_.status = st;
    last_.activity = s.activity;
    return st;
  }

  double scores[kBehaviours];
  score(ctx, scores);
  int order[kBehaviours];  // sorted(scores, key=lambda b: (-scores[b], b))
  for (int i = 0; i < kBehaviours; i++) {
    int b = i, j = i - 1;
    while (j >= 0 && (scores[order[j]] < scores[b] ||
                      (scores[order[j]] == scores[b] && str_cmp(b_[order[j]].name, b_[b].name) > 0))) {
      order[j + 1] = order[j];
      j--;
    }
    order[j + 1] = b;
  }
  int best = order[0];
  int cur = current_ >= 0 && current_ < kBehaviours ? current_ : -1;
  int first;
  const char* reason;
  if (cur < 0 || finished_) {
    first = best;
    reason = "free";
  } else if (best == cur) {
    first = cur;
    reason = "best";
  } else if (scores[best] >= scores[cur] + cfg_.sel_margin) {
    double dwell = d_max(cfg_.sel_min_dwell_s, b_[cur].min_dwell);
    if (ctx.now - since_ >= dwell) {
      first = best;
      reason = "switch";
    } else if (cfg_.urgent & (1u << best)) {
      first = best;
      reason = "urgent";
    } else {
      first = cur;
      reason = "dwell";
    }
  } else {
    first = cur;
    reason = "margin";
  }
  int cand[kBehaviours + 1];
  int n = 0;
  cand[n++] = first;
  if (cur >= 0 && cur != first && !finished_) cand[n++] = cur;
  for (int i = 0; i < kBehaviours; i++) {
    bool in = false;
    for (int k = 0; k < n; k++) in = in || cand[k] == order[i];
    if (!in) cand[n++] = order[i];
  }
  char why[32];
  format(why, sizeof why, "%s", reason);
  for (int k = 0; k < n; k++) {
    int b = cand[k];
    bool fallback = why[0] == 'f' && why[1] == 'a' && why[2] == 'l' && why[3] == 'l';  // reason.startswith("fallback")
    if (b != first && !fallback) format(why, sizeof why, "fallback(%s)", b_[first].name);
    ctx.push(b_[b].name);
    Status st = b_[b].tree->tick(ctx);
    ctx.pop();
    if (st == Status::Failure) {
      b_[b].tree->halt();
      if (b == current_) finished_ = true;
      continue;
    }
    switch_to(ctx, b);
    finished_ = st == Status::Success;
    last_ = Choice();
    last_.chosen = b;
    last_.name = b_[b].name;
    set_reason(last_, why);
    last_.has_status = true;
    last_.status = st;
    last_.has_scores = true;
    for (int i = 0; i < kBehaviours; i++) last_.scores[i] = scores[i];
    last_.activity = b_[b].activity;
    return Status::Running;  // the root keeps living
  }
  last_ = Choice();
  set_reason(last_, "none");
  last_.has_status = true;
  last_.status = Status::Failure;
  last_.has_scores = true;
  for (int i = 0; i < kBehaviours; i++) last_.scores[i] = scores[i];
  last_.activity = Activity::Idle;
  return Status::Failure;
}

void UtilitySelector::halt() {
  if (current_ >= 0) node(current_)->halt();
  current_ = -1;
  finished_ = true;
  Node::halt();
}

}  // namespace behaviour
}  // namespace kilo
