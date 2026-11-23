// The engine's cost per tick on this machine: Solver::next() timed one
// call at a time over long walks of every gait (with speed, direction,
// cadence and gait changes and a stop), as -O2 builds it without sanitizers.
// The Pi 2 B's figure comes from its boot bench (kernel/src/plannerbench.h);
// the tick is 20 ms.
//
//   make -C kernel/host bench
#include <time.h>
#include <algorithm>
#include <cstdio>
#include <vector>
#include "engine/solve.h"

using namespace kilo::engine;

namespace {

double now_us() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1e6 + ts.tv_nsec / 1e3;
}

Solver g_solver;  // static storage: a solver is a few KB
double g_start_max = 0;

void run(const char* name, std::vector<double>* all) {
  std::vector<double> us;
  for (int rep = 0; rep < 4; rep++) {
    Solver& p = g_solver;
    double t0 = now_us();
    p.start(*find_gait(name), Direction::Forward, -1, 1000);
    double start_us = now_us() - t0;
    if (start_us > g_start_max) g_start_max = start_us;
    for (int i = 0; i < 6000; i++) {
      if (i == 1500) p.set_speed(rep % 2 ? 6 : 40);
      if (i == 3000) p.set_direction(Direction::Left);
      if (i == 2200) p.set_cadence((real)1.3);
      if (i == 4000) p.set_gait(*find_gait(rep % 2 ? "walk" : "trot"));
      if (i == 5200) p.stop();
      if (p.done()) break;
      double t1 = now_us();
      p.next();
      us.push_back(now_us() - t1);
    }
  }
  std::sort(us.begin(), us.end());
  double sum = 0;
  for (double v : us) sum += v;
  std::printf("  %-6s %6zu ticks: mean %6.2f us, p50 %6.2f, p99 %6.2f, max %7.2f\n", name, us.size(),
              sum / us.size(), us[us.size() / 2], us[us.size() * 99 / 100], us.back());
  all->insert(all->end(), us.begin(), us.end());
}

}  // namespace

int main() {
  std::printf("engine cost per tick (host):\n");
  std::vector<double> all;
  for (const char* n : {"trot", "walk", "crawl"}) run(n, &all);
  std::sort(all.begin(), all.end());
  std::printf("  all    %6zu ticks: p99 %6.2f us, max %7.2f us (the tick: 20000 us)\n", all.size(),
              all[all.size() * 99 / 100], all.back());
  std::printf("  start: max %.2f us\n", g_start_max);
  return 0;
}
