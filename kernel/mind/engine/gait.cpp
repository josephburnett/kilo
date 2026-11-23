#include "engine/gait.h"
#include "util/str.h"

namespace kilo {
namespace engine {

using planner::ceil_i;
using planner::m_exp;
using planner::m_fabs;
using planner::m_max;
using planner::m_min;
using planner::m_sqrt;
using planner::round_i;

// LF RF RB LB. The walk lifts LB, LF, RB, RF; the trot LF+RB, then RF+LB; the crawl as the walk,
// slower, its four-foot time shared evenly. Heights and lifts as the walks the owner saw on the floor
// (2026-11-22: the walk 8 mm at 205/256, the trot 8 mm, the crawl 15 mm at 179/256).
const Gait kGaitTable[kGaits] = {
    // the walk: three shares of the four-foot time before a hind leg's lift, one before a fore leg's
    // (the weight crosses the diagonal there: docs/walking-and-balance.md §16)
    {"walk", 4, 1, {{LB, 0}, {LF, 0}, {RB, 0}, {RF, 0}}, {3, 1, 3, 1}, 10, 48, 54, 8, 205, (real)3.5, 58, 1, true, false,
     false, 0, 6, 16},
    {"trot", 2, 2, {{LF, RB}, {RF, LB}, {0, 0}, {0, 0}}, {1, 1, 0, 0}, kTrotWMin, 2 * (kTrotWMin + 1), 52, 8, 205, -1,
     144, (real)0.75, false, true, false, 0, 0, 16},
    {"crawl", 4, 1, {{LB, 0}, {LF, 0}, {RB, 0}, {RF, 0}}, {1, 1, 1, 1}, 15, (real)127.5, 50, 15, 179, 8, -1,
     (real)0.5, false, false, true, 200, 12, 4},
};

const Gait& default_gait() { return kGaitTable[1]; }

const Gait* find_gait(const char* name) {
  for (const Gait& g : kGaitTable)
    if (str_ieq(name, g.name)) return &g;
  return nullptr;
}

static const char* const kDirectionNames[] = {"forward", "back", "left", "right", "march"};

bool find_direction(const char* name, Direction* out) {
  if (str_ieq(name, "backward") || str_ieq(name, "backwards")) {
    *out = Direction::Back;
    return true;
  }
  for (int i = 0; i < 5; i++)
    if (str_ieq(name, kDirectionNames[i])) {
      *out = (Direction)i;
      return true;
    }
  return false;
}

const char* direction_name(Direction d) { return kDirectionNames[(int)d]; }

int trot_swing_for(real speed_mm_s) {
  real v = m_fabs(speed_mm_s);
  if (v <= kTrotVFrom) return kTrotWMin;
  int w = kTrotWMin + ceil_i((v - kTrotVFrom) / kTrotVStep - (real)1e-9);
  return w < kTrotWMax ? w : kTrotWMax;
}

static real fit_x(int w) {  // the effective time on one diagonal over the pendulum's time constant
  real tau = m_sqrt(kFitComHMm / planner::kGmmS2);
  return ((real)(w - 1) * kTickMs + kFitExtraMs) / (real)1000.0 / tau;
}

real fit_k(int w) {
  real x = fit_x(w);
  real e = m_exp(x), ei = (real)1.0 / e;
  real sh = (e - ei) / 2, ch = (e + ei) / 2;
  return (sh - x) / (ch - (real)1.0);
}

real fit_land_x(int w, real v) { return kFitComXMm + fit_k(w) * v * m_sqrt(kFitComHMm / planner::kGmmS2); }

void offsets(const Gait& g, int w, real cycle, real out[kLegs]) {
  real total = 0;
  for (int k = 0; k < g.n_groups; k++) total += g.gap_share[k];
  real gaps = m_max((real)0, cycle - (real)(g.n_groups * w));
  real t = 0;
  for (int k = 0; k < g.n_groups; k++) {
    t += gaps * g.gap_share[k] / total;
    for (int i = 0; i < g.group_size; i++) out[g.groups[k][i]] = t / cycle;
    t += (real)w;
  }
}

// The crawl's shortest four-foot time before a swing: the body's move over the next three feet.
constexpr int kCrawlShiftMinTicks = 12;

static Timing trot_timing(const Gait& g, real v, bool back, real cadence) {
  real lo = kStrideMinMm / (((real)kTrotWMin + kTrotStancePlus) * kTickS);
  v = m_min(back ? kTrotTopBackMmS : kTrotTopMmS, m_max(lo, v));
  int w = trot_swing_for(v);
  if (cadence > 0 && cadence != 1) {
    int c = round_i((real)w / cadence);
    w = c < kTrotWMin ? kTrotWMin : c > kTrotWMax ? kTrotWMax : c;
  }
  real stance = (real)w + kTrotStancePlus;
  real s = m_max(kStrideMinMm, v * stance * kTickS);
  real cycle = (real)(2 * (w + 1));
  return {s, w, cycle, stance, cycle * kTickMs, s / (stance * kTickS), (real)g.cycle_ticks / cycle};
}

Timing timing(const Gait& g, real speed_mm_s, Direction d, real cadence) {
  real v = m_fabs(speed_mm_s);
  if (g.pendulum) return trot_timing(g, v, d == Direction::Back, cadence);
  real c = cadence > 0 ? m_min(kCadenceMax, m_max(kCadenceMin, cadence)) : (real)1.0;
  int slots = g.n_groups;
  int gap = g.stance_from_speed ? kCrawlShiftMinTicks : 1;
  int w = round_i((real)g.swing_ticks / c);
  int wmin = g.swing_ticks * 4 / 5;
  if (w < wmin) w = wmin;
  real cycle_min = (real)(slots * (w + gap));
  real stride, cycle;
  if (g.stance_from_speed) {
    // the stride as long as it goes, the stance from the speed, slower than the longest stance a
    // shorter stride; the cadence asked scales the stride (a faster cadence: shorter steps)
    real st_max = g.cycle_max_ticks - (real)w;
    stride = m_min(g.stride_mm, g.stride_mm / c);
    real stance = v > 0 ? stride / v / kTickS : st_max;
    if (stance > st_max) {
      stance = st_max;
      stride = m_max(kStrideMinMm, v * stance * kTickS);
    }
    cycle = m_max(cycle_min, stance + (real)w);
  } else {
    // the cadence fixed (the gait's, or the goal's), the stride from the speed; a stride past the
    // longest raises the cadence as far as the swing lets it
    cycle = m_max(cycle_min, g.cycle_ticks / c);
    stride = v * (cycle - (real)w) * kTickS;
    if (stride > g.stride_mm) {
      real need = g.stride_mm / (v * kTickS) + (real)w;
      cycle = m_max(cycle_min, need);
      stride = m_min(g.stride_mm, v * (cycle - (real)w) * kTickS);
    }
    stride = m_max(kStrideMinMm, stride);
    // a long stride: a longer swing, so no joint turns faster than the walks the owner saw
    int wl = planner::ceil_i(stride / kSwingMmPerTick - (real)1e-6);
    if (wl > w) {
      w = wl;
      stride = m_max(kStrideMinMm, m_min(g.stride_mm, v * (cycle - (real)w) * kTickS));
    }
  }
  real stance = cycle - (real)w;
  return {stride, w, cycle, stance, cycle * kTickMs, stride / (stance * kTickS), g.cycle_ticks / cycle};
}

real default_speed(const Gait& g) {
  if (g.default_mm_s >= 0) return g.default_mm_s;
  return g.stride_mm / ((g.cycle_ticks - (real)g.swing_ticks) * kTickS);
}

void speed_range(const Gait& g, real* lo, real* hi) {
  if (g.pendulum) {
    *lo = trot_timing(g, 0, false, 0).speed_mm_s;
    *hi = kTrotTopMmS;
    return;
  }
  *lo = timing(g, 0).speed_mm_s;
  *hi = timing(g, 1e4).speed_mm_s;
}

void side_strides(Direction d, real stride, real out[2], real steer, real cap) {
  switch (d) {
    case Direction::Forward:
    case Direction::Back: {
      real s = d == Direction::Forward ? stride : -stride;
      real w = m_max((real)-1.0, m_min((real)1.0, steer)) * m_fabs(s);
      real left = s - w, right = s + w;
      real m = m_max(m_fabs(left), m_fabs(right));
      if (cap > 0 && m > cap) {
        left = left * cap / m;
        right = right * cap / m;
      }
      out[0] = left;
      out[1] = right;
      return;
    }
    case Direction::March:
      out[0] = out[1] = 0;
      return;
    case Direction::Left:
    case Direction::Right: {
      real t = kTurnFrac * stride;
      out[0] = d == Direction::Left ? -t : t;
      out[1] = d == Direction::Left ? t : -t;
      return;
    }
  }
}

}  // namespace engine
}  // namespace kilo
