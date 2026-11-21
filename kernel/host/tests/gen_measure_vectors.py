"""Golden vectors for the C++ self-measurement script (kernel/mind/measure/script.cpp),
from the Python one (mind/kilo/measure/script.py script 2, statics.rigid_rest):
every tick of two runs, as lines that kernel/host/tests/test_measure.cpp replays.

    ~/OpenCat/.venv/bin/python kernel/host/tests/gen_measure_vectors.py > kernel/host/tests/measure_vectors.txt

Line format (space separated):

    run NAME                                         a new Script
    tip EXP SEEN                                     tip(SEEN) once the planner is TIP_SEEN_LAG_TICKS past
                                                     SEEN in experiment EXP (script.poses)
    t TICK STEP FIRST X0..X3 Z0..Z3 PITCH ROLL SUPPORT   one tick: its pose and the rigid model's rest
    end TICKS                                        the run's length
"""

import sys

from kilo.measure import script as SC
from kilo.measure import statics as ST
from kilo.planner.model import BodyModel

M = BodyModel()


def f(v: float) -> str:
    return f"{v:.4f}".rstrip("0").rstrip(".")


def phase_start(exp: int, phase: str, tips: dict[int, int]) -> int:
    ticks = SC.poses(tips)
    return next(t.tick for t in ticks if SC.STEPS[t.step].exp == exp and SC.STEPS[t.step].phase == phase)


def run(name: str, tips: dict[int, int], out) -> None:
    out.write(f"run {name}\n")
    for e, seen in sorted(tips.items()):
        out.write(f"tip {e} {seen}\n")
    ticks = SC.poses(tips)
    prev = 0xF
    for t in ticks:
        p, r, prev = ST.rigid_rest(M, t.x, t.z, prev)
        out.write(f"t {t.tick} {t.step} {int(t.first)} {' '.join(map(str, t.x))} {' '.join(map(str, t.z))} "
                  f"{f(p)} {f(r)} {prev}\n")
    out.write(f"end {len(ticks)}\n")


def main() -> None:
    out = sys.stdout
    out.write("# kernel/host/tests/gen_measure_vectors.py: the self-measurement script, tick by tick\n")
    run("no_tips", {}, out)
    tips: dict[int, int] = {}
    E = SC.SCRIPT.index
    for e, phase, k in ((E("edge_lf"), "sweep", 40), (E("edge_rf"), "lifted", 4), (E("edge_lb"), "sweep", 95),
                        (E("edge_lf_toward"), "sweep", 60), (E("edge_lb_toward"), "sweep", 13)):
        tips[e] = phase_start(e, phase, tips) + k  # each in the timeline the tips before it made
    run("tips", tips, out)


if __name__ == "__main__":
    main()
