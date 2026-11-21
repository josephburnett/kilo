// The body's IK (core/ik.cpp, the params' defaults) over the legs' workspace,
// one line per foot: "leg x_mm z_mm ok shoulder_a16 knee_a16". Built and read
// by mind/tests/test_one_model.py, which checks it against the mind's model
// of the same leg (kilo.planner.model.BodyModel.ik). Not a test_*.cpp:
// body/tools/host_tests.sh does not run it; body/tools/build_link_harness.sh
// builds it (second argument ik_dump).
#include <cstdio>
#include "../src/core/ik.h"

int main() {
  for (int leg = 0; leg < 4; leg++)
    for (int x = -60; x <= 60; x += 2)
      for (int z = 20; z <= 95; z += 1) {
        int16_t sh = 0, kn = 0;
        bool ok = ik_leg((uint8_t)leg, (int16_t)(x * 4), (int16_t)(z * 4), &sh, &kn);
        std::printf("%d %d %d %d %d %d\n", leg, x, z, ok ? 1 : 0, sh, kn);
      }
  return 0;
}
