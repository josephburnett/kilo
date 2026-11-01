// The C++ planner against the Python one: replays every scenario of
// tests/planner_vectors.txt (made by gen_planner_vectors.py from
// mind/kilo/planner) and compares every planned tick: foot positions and
// support exactly (they are whole mm on the wire), the attitudes within
// 0.01 deg, the margins within 0.05 mm; and the patterns' timing, the leg
// IK, the centre of mass, fit_attitude, stability and the ErrorTracker.
//
// One allowance: the plan has exact .5 mm ties (a trot's foot lands at
// stride / 2 = 12.5), which Python's double and the float port round either
// way by noise. A foot 1 mm off where the port's unrounded value is within
// 0.002 mm of a .5 is counted as a tie, not a failure (and that tick's
// margin, which uses the rounded feet, is not compared). The double twin
// must match with no ties.
//
//   test_planner_vectors VECTORS    (the build also makes a double-precision
//                                    twin, test_planner_vectors_double)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "planner/errors.h"
#include "planner/plan.h"
#include "tests/check.h"

using namespace kilo::planner;

namespace {

struct Dev {
  Dev(const char* w, double t) : what(w), tol(t) {}
  const char* what;
  double tol;
  double max = 0;
  long n = 0, bad = 0;
  std::string worst;
  void see(double got, double want, const std::string& where) {
    double d = std::fabs(got - want);
    if (std::isinf(want) && std::isinf(got) && (want > 0) == (got > 0)) d = 0;
    n++;
    if (d > max) {
      max = d;
      worst = where;
    }
    if (!(d <= tol)) bad++;
    CHECKF(d <= tol, "%s: %s got %.6f want %.6f", what, where.c_str(), got, want);
  }
  void print() const {
    std::printf("  %-14s %6ld values, max deviation %.6f (tolerance %g)%s%s\n", what, n, max, tol,
                bad ? ", OVER: " : "", bad ? std::to_string(bad).c_str() : "");
  }
};

double num(const std::string& s) {
  if (s == "inf") return INFINITY;
  return std::strtod(s.c_str(), nullptr);
}

const Pattern& pat(const std::string& name) {
  const Pattern* p = find_pattern(name.c_str());
  if (!p) {
    std::printf("unknown pattern %s\n", name.c_str());
    std::exit(2);
  }
  return *p;
}

Direction dir(const std::string& name) {
  Direction d;
  if (!find_direction(name.c_str(), &d)) {
    std::printf("unknown direction %s\n", name.c_str());
    std::exit(2);
  }
  return d;
}

}  // namespace

int main(int argc, char** argv) {
  const char* path = argc > 2 ? argv[2] : "tests/planner_vectors.txt";
  if (argc > 1 && std::strstr(argv[1], "planner_vectors")) path = argv[1];
  std::ifstream in(path);
  if (!in) {
    std::printf("cannot open %s\n", path);
    return 2;
  }
  Dev pos{"feet mm", 0.05}, att{"attitude deg", 0.01}, mar{"margin mm", 0.05}, flags{"support/lifting", 0},
      ticks{"tick", 0}, stat{"end stats", 0}, tie_d{"ties (1 mm)", 1}, minm{"min margin mm", 0.05}, timing_d{"timing", 1e-3},
      ik_d{"ik rad", 1e-4}, com_d{"com mm", 1e-3}, fit_d{"fit deg", 0.01}, stab_d{"stability mm", 0.01},
      err_d{"errors deg", 0.01}, est_d{"estimate", 0.0101};

  BodyModel model;
  static Planner planner(model);  // static: the planner is a few KB, kept off the stack
  ErrorTracker tracker;
  std::string scen;
  long tick_lines = 0;
  long line_no = 0;
  std::string line;
  while (std::getline(in, line)) {
    line_no++;
    std::istringstream ss(line);
    std::vector<std::string> f;
    for (std::string w; ss >> w;) f.push_back(w);
    if (f.empty()) continue;
    const std::string& k = f[0];
    std::string where = scen + " line " + std::to_string(line_no);
    if (k == "sr") {
      real lo, hi;
      speed_range(pat(f[1]), model.servo_lag_ms, &lo, &hi);
      timing_d.see(lo, num(f[2]), where);
      timing_d.see(hi, num(f[3]), where);
    } else if (k == "tm") {
      double v = num(f[2]);
      Timing t = v < 0 ? default_timing(pat(f[1])) : timing(pat(f[1]), (real)v);
      const real got[5] = {t.stride_mm, t.shift_ms, t.stance_ms, t.period_ms, t.speed_mm_s};
      for (int i = 0; i < 5; i++) timing_d.see(got[i], num(f[3 + i]), where);
    } else if (k == "ik") {
      real t1, t2;
      model.ik(std::atoi(f[1].c_str()), (real)num(f[2]), (real)num(f[3]), &t1, &t2);
      ik_d.see(t1, num(f[4]), where);
      ik_d.see(t2, num(f[5]), where);
    } else if (k == "com" || k == "fit" || k == "stab") {
      real xs[4], zs[4];
      for (int i = 0; i < 4; i++) {
        xs[i] = (real)num(f[1 + i]);
        zs[i] = (real)num(f[5 + i]);
      }
      if (k == "com") {
        V3 c = model.com(xs, zs);
        com_d.see(c.x, num(f[9]), where);
        com_d.see(c.y, num(f[10]), where);
        com_d.see(c.z, num(f[11]), where);
      } else if (k == "fit") {
        Attitude a = fit_attitude(model, xs, zs, (uint8_t)std::atoi(f[9].c_str()),
                                  {(real)num(f[10]), (real)num(f[11])});
        fit_d.see(a.pitch, num(f[12]), where);
        fit_d.see(a.roll, num(f[13]), where);
      } else {
        real m = stability(model, xs, zs, (uint8_t)std::atoi(f[9].c_str()), (real)num(f[10]), (real)num(f[11]));
        stab_d.see(m, num(f[12]), where);
      }
    } else if (k == "S") {
      scen = f[1];
      double v = num(f[4]);
      planner.start(pat(f[2]), dir(f[3]), v < 0 ? (real)-1 : (real)v, (uint32_t)std::atol(f[5].c_str()));
    } else if (k == "speed") {
      planner.set_speed((real)num(f[1]));
    } else if (k == "dir") {
      planner.set_direction(dir(f[1]));
    } else if (k == "pattern") {
      planner.set_pattern(pat(f[1]));
    } else if (k == "stop") {
      planner.stop();
    } else if (k == "observe") {
      planner.observe((real)num(f[1]), (real)num(f[2]));
    } else if (k == "t") {
      tick_lines++;
      const TickPlan& t = planner.next();
      ticks.see(t.tick, num(f[1]), where);
      bool tie_tick = false;
      for (int i = 0; i < 8; i++) {
        double got = i < 4 ? t.x[i] : t.z[i - 4], want = num(f[2 + i]);
        double raw = i < 4 ? t.x_raw[i] : t.z_raw[i - 4];
        double frac = raw - std::floor(raw);
        if (sizeof(real) == 4 && std::fabs(got - want) == 1 && std::fabs(frac - 0.5) < 0.002) {
          tie_d.see(got, want, where);  // counted, within its tolerance
          tie_tick = true;
          continue;
        }
        pos.see(got, want, where + (i < 4 ? " x" : " z") + std::to_string(i % 4));
      }
      flags.see(t.support, num(f[10]), where + " support");
      att.see(t.pitch, num(f[11]), where + " pitch");
      att.see(t.roll, num(f[12]), where + " roll");
      att.see(t.plan_pitch, num(f[13]), where + " plan_pitch");
      att.see(t.plan_roll, num(f[14]), where + " plan_roll");
      if (!tie_tick) mar.see(t.margin, num(f[15]), where + " margin");
      flags.see(t.lifting, num(f[16]), where + " lifting");
    } else if (k == "end") {
      const PlannerStats& s = planner.stats();
      stat.see(planner.done(), num(f[1]), where + " done");
      stat.see(s.holds, num(f[2]), where + " holds");
      stat.see(s.catches, num(f[3]), where + " catches");
      stat.see(s.waits, num(f[4]), where + " waits");
      minm.see(s.min_margin, num(f[5]), where + " min_margin");
      CHECKF(f[6] == planner.pattern().name, "%s: pattern %s want %s", where.c_str(), planner.pattern().name,
             f[6].c_str());
    } else if (k == "er") {
      tracker.reset();
    } else if (k == "ea") {
      Attitude own;
      bool has_own = f[8] != "-";
      if (has_own) own = {(real)num(f[8]), (real)num(f[9])};
      int16_t v[6];
      for (int i = 0; i < 6; i++) v[i] = (int16_t)std::atoi(f[2 + i].c_str());
      Attitude e = tracker.add(std::atol(f[1].c_str()), v[0], v[1], v[2], v[3], v[4], v[5], has_own ? &own : nullptr);
      err_d.see(e.pitch, num(f[10]), where);
      err_d.see(e.roll, num(f[11]), where);
      err_d.see(tracker.offset().pitch, num(f[12]), where);
      err_d.see(tracker.offset().roll, num(f[13]), where);
    } else if (k == "ee") {
      ErrorEstimate e;
      bool have = tracker.estimate(&e);
      CHECKF(have == (f[1] != "none"), "%s: estimate %d", where.c_str(), (int)have);
      if (have && f.size() >= 9) {
        est_d.see(e.samples, num(f[1]), where + " samples");
        est_d.see(e.offset_pitch_deg, num(f[2]), where + " offset pitch");
        est_d.see(e.offset_roll_deg, num(f[3]), where + " offset roll");
        CHECKF(e.has_lag == (f[4] != "none"), "%s: lag %d", where.c_str(), (int)e.has_lag);
        if (e.has_lag && f[4] != "none") {
          est_d.see(e.lag_ms, num(f[4]), where + " lag");
          est_d.see(e.lag_quality, num(f[5]), where + " lag quality");
        }
        est_d.see(e.spike_deg, num(f[6]), where + " spike");
        est_d.see(e.corr_pitch_deg, num(f[7]), where + " corr pitch");
        est_d.see(e.corr_roll_deg, num(f[8]), where + " corr roll");
      }
    } else {
      CHECKF(false, "line %ld: unknown record %s", line_no, k.c_str());
    }
  }
  std::printf("planner vectors (%s, %s): %ld planned ticks\n", path, sizeof(real) == 8 ? "double" : "float",
              tick_lines);
  for (const Dev* d : {&ticks, &pos, &tie_d, &flags, &att, &mar, &stat, &minm, &timing_d, &ik_d, &com_d, &fit_d, &stab_d,
                       &err_d, &est_d})
    d->print();
  CHECKF(tick_lines > 1000, "only %ld ticks replayed", tick_lines);
  CHECKF(tie_d.n <= 40, "%ld ties: more than float noise at .5 mm explains", tie_d.n);
  return report(sizeof(real) == 8 ? "plan vec dbl" : "plan vectors");
}
