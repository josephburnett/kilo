#include "engine/posture.h"
#include "tricks/library.h"
#include "util/fmt.h"

namespace kilo {
namespace engine {

using planner::clamp;
using planner::m_fabs;
using planner::m_max;
using planner::m_min;
using planner::round_i;

namespace {

// The four leg groups (canonical joints); the head and tail go with the first stage.
constexpr int kGroups = 4;
const int kGroup[kGroups][2] = {{3, 4}, {7, 8}, {5, 6}, {9, 10}};
const char* const kGroupName[kGroups] = {"front shoulders", "front knees", "hind shoulders", "hind knees"};
// A way to stage a move: each group's stage 0..kWayStages-1, as the code sum of stage(g) x 3^g (0: all at
// once). Only codes whose used stages are 0, 1, .. in order count (the others are the same ways again).
constexpr int kWayStages = 3;
constexpr int kWays = 81;
// Kilo's servos as he is since 2026-11-19: the right hind shoulder re-seated (what the recordings of SIT
// since then show: level, where the EEPROM's zero would roll him 5 degrees onto his right hind foot).
constexpr Servos kServos = Servos::Reseated;

int stage_of(int code, int g) {
  for (int i = 0; i < g; i++) code /= 3;
  return code % 3;
}

// A code whose stages are 0, or 0 and 1, or 0, 1 and 2 (none skipped).
bool canonical(int code) {
  unsigned used = 0;
  for (int g = 0; g < kGroups; g++) used |= 1u << stage_of(code, g);
  return used == 1u || used == 3u || used == 7u;
}

struct Plan {
  Joints at[kMaxStages];  // the joints after each stage
  uint8_t way[2] = {0, 0};  // per move (two through another posture), how it is staged
  int n = 0;
  real margin = -1e9f;
};

// The joints after each stage of moving from a to b staged as `way`.
int stages(const Joints& a, const Joints& b, int way, Joints* out) {
  Joints cur = a;
  for (int k = 0; k < 3; k++) cur.a[k] = b.a[k];
  int n = 0;
  for (int st = 0; st < kWayStages; st++) {
    bool any = false;
    for (int g = 0; g < kGroups; g++)
      if (stage_of(way, g) == st) {
        any = true;
        for (int i = 0; i < 2; i++) cur.a[kGroup[g][i]] = b.a[kGroup[g][i]];
      }
    if (any) out[n++] = cur;
  }
  return n;
}

// The least margin along the stages from `from`; -1e9 when something but kMayCarry touches the floor or
// a topple drops a point more than kDropMaxMm.
real judge(const BodyModel& m, const Joints& from, const Joints* at, int n) {
  Rest cur = rest(m, kServos, from);
  cur.drop = 0;  // (he is there already: the topples from level that find his rest are not his)
  Along al;
  al.add(cur);
  Joints prev = from;
  for (int i = 0; i < n; i++) {
    follow(m, kServos, prev, at[i], &cur, &al);
    prev = at[i];
  }
  if ((al.contacts & ~kMayCarry) != 0 || al.drop > kDropMaxMm) return -1e9f;
  return al.margin;
}

// The best way from a to b straight there (no other posture between); *direct: all at once's margin.
// A way with more stages must keep kTieMm more for each stage it adds.
Plan best_direct(const BodyModel& m, const Joints& a, const Joints& b, real* direct) {
  Plan best;
  real best_score = -1e18f;
  for (int w = 0; w < kWays; w++) {
    if (!canonical(w)) continue;
    Plan p;
    p.n = stages(a, b, w, p.at);
    p.way[0] = (uint8_t)w;
    p.margin = judge(m, a, p.at, p.n);
    if (w == 0) *direct = p.margin;
    real score = p.margin - kTieMm * (real)(p.n - 1);
    if (score > best_score) {
      best = p;
      best_score = score;
    }
  }
  return best;
}

int span_ticks(const Joints& a, const Joints& b, real tempo) {
  real span = 0;
  for (int k = 0; k < kJoints; k++) span = m_max(span, m_fabs(b.a[k] - a.a[k]));
  real t = m_max((real)kStageMinTicks, span / kRateDegPerTick);
  int n = round_i(t / tempo);
  return n < 1 ? 1 : n > 255 ? 255 : n;
}

void way_text(int way, char* buf, size_t n) {
  if (way == 0) {
    format(buf, n, "all at once");
    return;
  }
  size_t at = 0;
  buf[0] = 0;
  for (int st = 0; st < kWayStages; st++) {
    bool first = true;
    for (int g = 0; g < kGroups; g++)
      if (stage_of(way, g) == st && at + 1 < n) {
        at += format(buf + at, n - at, "%s%s", first ? (at ? ", then " : "") : "+", kGroupName[g]);
        first = false;
      }
  }
}

}  // namespace

const char* posture_word(uint8_t posture) {
  return posture == KILO_POSTURE_SIT ? "sit" : posture == KILO_POSTURE_REST ? "rest"
                                           : posture == KILO_POSTURE_STAND ? "stand" : "?";
}

bool posture_joints(uint8_t posture, Joints* j) {
  const char* name = posture == KILO_POSTURE_STAND ? "stand"
                     : posture == KILO_POSTURE_SIT ? "sit"
                     : posture == KILO_POSTURE_REST ? "rest" : nullptr;
  const tricks::Trick* t = name != nullptr ? tricks::find_trick(name) : nullptr;
  if (t == nullptr) return false;
  for (int k = 0; k < kJoints; k++) j->a[k] = (real)t->frames[0].ang[k];
  return true;
}


namespace {

// The way from joints `from` to posture `to` (its joints `tj`): the best straight there, or through another
// posture when that keeps kTieMm more and the straight way does not keep kPostureMarginMm.
void plan_way(const BodyModel& m, const Joints& from, uint8_t to, const Joints& tj, PostureWay* w) {
  Plan best = best_direct(m, from, tj, &w->direct);
  w->via = 0;
  if (best.margin < kPostureMarginMm) {
    const uint8_t kVia[3] = {KILO_POSTURE_STAND, KILO_POSTURE_SIT, KILO_POSTURE_REST};
    for (uint8_t v : kVia) {
      if (v == to) continue;
      Joints mid;
      posture_joints(v, &mid);
      real d0, d1;
      Plan p0 = best_direct(m, from, mid, &d0);
      Plan p1 = best_direct(m, mid, tj, &d1);
      if (p0.n + p1.n > kMaxStages) continue;
      Plan p;
      for (int i = 0; i < p0.n; i++) p.at[p.n++] = p0.at[i];
      for (int i = 0; i < p1.n; i++) p.at[p.n++] = p1.at[i];
      p.way[0] = p0.way[0];
      p.way[1] = p1.way[0];
      p.margin = judge(m, from, p.at, p.n);  // the whole way, the orientation carried through
      if (p.margin > best.margin + kTieMm) {
        best = p;
        w->via = v;
      }
    }
  }
  w->n = best.n;
  for (int i = 0; i < best.n; i++) w->at[i] = best.at[i];
  w->margin = best.margin;
  char a[84], b[84];
  way_text(best.way[0], a, sizeof a);
  if (w->via != 0) {
    way_text(best.way[1], b, sizeof b);
    format(w->text, sizeof w->text, "%s; %s; %s", a, posture_word(w->via), b);
  } else {
    format(w->text, sizeof w->text, "%s", a);
  }
}

int posture_index(uint8_t p) { return p == KILO_POSTURE_REST ? 0 : p == KILO_POSTURE_SIT ? 1 : p == KILO_POSTURE_STAND ? 2 : -1; }

}  // namespace

bool PostureMove::build(const BodyModel& m, const Joints& from, const PostureGoal& g) {
  why_ = nullptr;
  Joints to;
  if (!posture_joints(g.posture, &to)) {
    why_ = "not a posture to move to";
    return false;
  }
  plan_way(m, from, g.posture, to, &way_);
  return timed(m, from, g);
}

bool PostureMove::build_from(const BodyModel& m, uint8_t from, const PostureGoal& g) {
  why_ = nullptr;
  int a = posture_index(from), b = posture_index(g.posture);
  Joints fj, to;
  if (a < 0 || b < 0 || !posture_joints(from, &fj) || !posture_joints(g.posture, &to)) {
    why_ = "not a posture to move from or to";
    return false;
  }
  if (!cached_[a][b]) {
    plan_way(m, fj, g.posture, to, &cache_[a][b]);
    cached_[a][b] = true;
  }
  way_ = cache_[a][b];
  return timed(m, fj, g);
}

// The trick of way_ from `from` at the goal's tempo: each stage along the cosine ease, kStageHoldTicks
// apart, the last held kEndHoldTicks.
bool PostureMove::timed(const BodyModel& m, const Joints& from, const PostureGoal& g) {
  if (way_.margin < kPostureMarginMm || way_.n == 0) {
    why_ = "no way there keeps the weight inside what carries him";
    return false;
  }
  real tempo = clamp(g.tempo, kPostureTempoMin, kPostureTempoMax);
  Joints prev = from;
  for (int i = 0; i < way_.n; i++) {
    tricks::Keyframe& f = frames_[i];
    f.dur = (uint8_t)span_ticks(prev, way_.at[i], tempo);
    int hold = i == way_.n - 1 ? kEndHoldTicks : kStageHoldTicks;
    f.hold = (uint8_t)m_max((real)1, (real)round_i((real)hold / tempo));
    f.flags = tricks::kCosine;
    for (int k = 0; k < kJoints; k++) f.ang[k] = (int8_t)clamp((real)round_i(way_.at[i].a[k]), -127, 127);
    prev = way_.at[i];
  }
  Joints to;
  posture_joints(g.posture, &to);
  Rest end = rest(m, kServos, to);
  trick_ = {};
  trick_.name = g.posture == KILO_POSTURE_SIT ? "be_sitting" : g.posture == KILO_POSTURE_REST ? "be_resting"
                                                                                               : "be_standing";
  trick_.legacy = "";
  trick_.n_frames = (uint8_t)way_.n;
  trick_.loop_start = 0;
  trick_.loop_end = 0;
  trick_.repeats = 0;
  trick_.joint_mask = 0;
  for (int k = 0; k < kJoints; k++) trick_.joint_mask = (uint16_t)(trick_.joint_mask | (1u << tricks::kCanonical[k]));
  trick_.flags = 0;
  trick_.end_posture = g.posture;
  trick_.exp_roll = (int8_t)clamp((real)round_i(telemetry_roll(m, end)), -127, 127);
  trick_.exp_pitch = (int8_t)clamp((real)round_i(telemetry_pitch(m, end)), -127, 127);
  trick_.frames = frames_;
  trick_.stop_ticks = kStopTicks;
  trick_.guards = nullptr;
  trick_.n_guards = 0;
  trick_.roll_max_a16 = (int16_t)round_i(kRollMaxDeg * 16);
  const char* p = tricks::trick_problem(trick_);
  if (p != nullptr) {
    why_ = p;
    return false;
  }
  return true;
}

}  // namespace engine
}  // namespace kilo
