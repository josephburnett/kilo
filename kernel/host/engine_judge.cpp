// engine-judge: one walk planned tick by tick, no body, printed for the judge
// (mind/tests/enginejudge.py plays it on the virtual body's RockingPhysics).
//
//   engine-judge --gait walk|trot|crawl [--speed MM_S] [--dir forward|back|left|right|march]
//                [--steer S] [--ticks N] [--at TICK:CMD]... [--set KEY=VALUE]...
//
//   engine-judge --trick paw_wash [--tempo T] [--strokes N]
//
// prints the trick the engine makes (engine/paw.h): "trick lean_dx D lean_roll R margin M repeats N
// loop A B end STAND", then per frame "frame dur hold flags pan tilt tail sh_LF sh_RF sh_RB sh_LB kn_LF ..
// kn_LB" (degrees), then "guard frame lo hi roll" (a16).
//
// --set changes the solver's config (SolverConfig's fields) or the gait's (margin, roll_max, roll_cost,
// swing, cycle) for a study.
//
// CMD: speed=V, steer=S, dir=D, gait=G, stop, halt. Each line printed:
//   tick support lifting x0 z0 x1 z1 x2 z2 x3 z3 pitch roll plan_pitch plan_roll margin lean_dx lean_roll
// feet in quarter millimetres (the POSE's units), angles in degrees, the margin in mm;
// then "stats holds H catches C waits W min_margin M", and "done TICK" once the planned stop is over.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "engine/paw.h"
#include "engine/solve.h"

using namespace kilo;
using namespace kilo::engine;

namespace {

struct At {
  int tick;
  char cmd[32];
};

int usage() {
  fprintf(stderr, "usage: engine-judge --gait G [--speed V] [--dir D] [--steer S] [--ticks N] [--at T:CMD]...\n");
  return 64;
}

void apply(Solver& s, const char* cmd) {
  if (!strcmp(cmd, "stop")) return s.stop();
  if (!strcmp(cmd, "halt")) return s.halt();
  const char* eq = strchr(cmd, '=');
  if (eq == nullptr) return;
  const char* v = eq + 1;
  if (!strncmp(cmd, "speed", 5)) s.set_speed((real)atof(v));
  else if (!strncmp(cmd, "steer", 5)) s.set_steer((real)atof(v));
  else if (!strncmp(cmd, "dir", 3)) {
    Direction d;
    if (find_direction(v, &d)) s.set_direction(d);
  } else if (!strncmp(cmd, "gait", 4)) {
    const Gait* g = find_gait(v);
    if (g != nullptr) s.set_gait(*g);
  }
}

}  // namespace

int trick(int argc, char** argv) {
  PawGoal g;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!strcmp(argv[i], "--tempo")) g.tempo = (real)atof(argv[i + 1]);
    else if (!strcmp(argv[i], "--strokes")) g.strokes = atoi(argv[i + 1]);
  }
  static PawWash w;
  if (!w.build(planner::BodyModel(), g)) {
    fprintf(stderr, "engine-judge: no paw wash: %s\n", w.why());
    return 1;
  }
  const tricks::Trick& t = w.trick();
  printf("trick lean_dx %.3f lean_roll %.3f margin %.3f repeats %u loop %u %u\n", (double)w.lean_dx(),
         (double)w.lean_roll(), (double)w.margin(), (unsigned)t.repeats, (unsigned)t.loop_start, (unsigned)t.loop_end);
  for (int i = 0; i < t.n_frames; i++) {
    const tricks::Keyframe& f = t.frames[i];
    printf("frame %u %u %u", (unsigned)f.dur, (unsigned)f.hold, (unsigned)f.flags);
    for (int k = 0; k < (int)KILO_JOINT_COUNT; k++) printf(" %d", (int)f.ang[k]);
    printf("\n");
  }
  for (int i = 0; i < t.n_guards; i++)
    printf("guard %u %d %d %d\n", (unsigned)t.guards[i].frame, (int)t.guards[i].lo_a16, (int)t.guards[i].hi_a16,
           (int)t.guards[i].roll_a16);
  return 0;
}

int main(int argc, char** argv) {
  if (argc >= 3 && !strcmp(argv[1], "--trick")) {
    if (strcmp(argv[2], "paw_wash")) return usage();
    return trick(argc - 2, argv + 2);
  }
  const Gait* gait = find_gait("walk");
  Direction dir = Direction::Forward;
  real speed = -1, steer = 0;
  int ticks = 600;
  static At ats[64];
  static Gait custom;
  SolverConfig cfg;
  bool have_custom = false;
  int n_at = 0;
  for (int i = 1; i < argc; i++) {
    const char* a = argv[i];
    const char* v = i + 1 < argc ? argv[i + 1] : nullptr;
    if (v == nullptr) return usage();
    if (!strcmp(a, "--gait")) {
      gait = find_gait(v);
      if (gait == nullptr) return usage();
    } else if (!strcmp(a, "--speed")) {
      speed = (real)atof(v);
    } else if (!strcmp(a, "--dir")) {
      if (!find_direction(v, &dir)) return usage();
    } else if (!strcmp(a, "--steer")) {
      steer = (real)atof(v);
    } else if (!strcmp(a, "--ticks")) {
      ticks = atoi(v);
    } else if (!strcmp(a, "--set")) {
      if (!have_custom) {
        custom = *gait;
        have_custom = true;
      }
      const char* eq = strchr(v, '=');
      if (eq == nullptr) return usage();
      real x = (real)atof(eq + 1);
      size_t n = (size_t)(eq - v);
      if (!strncmp(v, "margin", n)) custom.margin_mm = x;
      else if (!strncmp(v, "roll_max", n)) custom.roll_max_deg = x;
      else if (!strncmp(v, "roll_cost", n)) custom.roll_cost = x;
      else if (!strncmp(v, "swing", n)) custom.swing_ticks = (int)x;
      else if (!strncmp(v, "cycle", n)) custom.cycle_ticks = x;
      else if (!strncmp(v, "dx_max", n)) cfg.dx_max_mm = x;
      else if (!strncmp(v, "dx_step", n)) cfg.dx_step_mm = x;
      else if (!strncmp(v, "roll_step", n)) cfg.roll_step_deg = x;
      else if (!strncmp(v, "smooth", n)) cfg.smooth = x;
      else if (!strncmp(v, "smooth2", n)) cfg.smooth2 = x;
      else if (!strncmp(v, "lean_tol", n)) cfg.lean_tol_mm = x;
      else if (!strncmp(v, "lean", n)) cfg.lean = x != 0;
      else return usage();
    } else if (!strcmp(a, "--at")) {
      if (n_at == 64) return usage();
      const char* c = strchr(v, ':');
      if (c == nullptr) return usage();
      ats[n_at].tick = atoi(v);
      snprintf(ats[n_at].cmd, sizeof ats[n_at].cmd, "%s", c + 1);
      n_at++;
    } else {
      return usage();
    }
    i++;
  }
  if (have_custom) gait = &custom;
  static Solver s(planner::BodyModel(), cfg);
  s.start(*gait, dir, speed, 0, -1, steer);
  for (int t = 0; t < ticks && !s.done(); t++) {
    for (int k = 0; k < n_at; k++)
      if (ats[k].tick == t) apply(s, ats[k].cmd);
    const TickPlan& p = s.next();
    printf("%u %u %u", (unsigned)p.tick, (unsigned)p.support, (unsigned)p.lifting);
    for (int leg = 0; leg < kLegs; leg++) printf(" %d %d", (int)p.xq[leg], (int)p.zq[leg]);
    printf(" %.4f %.4f %.4f %.4f %.3f %.3f %.3f\n", (double)p.pitch, (double)p.roll, (double)p.plan_pitch,
           (double)p.plan_roll, (double)p.margin, (double)p.lean_dx, (double)p.lean_roll);
  }
  const SolverStats& st = s.stats();
  printf("stats holds %u catches %u waits %u min_margin %.3f\n", (unsigned)st.holds, (unsigned)st.catches,
         (unsigned)st.waits, (double)st.min_margin);
  if (s.done()) printf("done %u\n", (unsigned)s.tick());
  return 0;
}
