// The planner's cost per tick on this machine: Planner::next() timed one
// call at a time over long walks of every pattern (with speed, direction
// and pattern changes and a stop), as -O2 builds it without sanitizers.
// Python on the Pi 2 B took at most 4.5 ms a tick (docs/walking-and-balance.md);
// the Pi's figure for this port is still to be measured.
//
//   make -C kernel/host bench
#include <time.h>
#include <algorithm>
#include <cstdio>
#include <vector>
#include "planner/plan.h"

using namespace kilo::planner;

namespace {

double now_us() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1e6 + ts.tv_nsec / 1e3;
}

Planner g_planner;  // static storage: a planner is a few KB
double g_start_max = 0;

void run(const char* name, std::vector<double>* all) {
  std::vector<double> us;
  for (int rep = 0; rep < 4; rep++) {
    Planner& p = g_planner;
    double t0 = now_us();
    p.start(*find_pattern(name), Direction::Forward, -1, 1000);  // the first slot's whole lean solve
    double start_us = now_us() - t0;
    if (start_us > g_start_max) g_start_max = start_us;
    for (int i = 0; i < 6000; i++) {
      if (i == 1500) p.set_speed(rep % 2 ? 6 : 40);
      if (i == 3000) p.set_direction(Direction::Left);
      if (i == 4000) p.set_pattern(*find_pattern(rep % 2 ? "walk" : "trot"));
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
  std::printf("planner cost per tick (%s, host):\n", sizeof(real) == 8 ? "double" : "float");
  std::vector<double> all;
  for (const char* n : {"trot", "walk", "crawl"}) run(n, &all);
  std::sort(all.begin(), all.end());
  std::printf("  all    %6zu ticks: p99 %6.2f us, max %7.2f us (Python on the Pi 2 B: max 4500 us)\n", all.size(),
              all[all.size() * 99 / 100], all.back());
  std::printf("  start (the first lean solved whole, before the stream): max %.2f us\n", g_start_max);
  return 0;
}
