"""The movement engine's tricks (docs/movement-engine.md, step 4), as the mind makes them
(kernel/mind/engine/paw.h through kernel/host's engine-judge, kilo.engine), judged on the body
model as the random mode's library tricks are (kilo.tricks.alive_set, mind/tests/test_alive_tricks.py):

- the paw wash: a front paw lifted on three feet and the face washed with it, at every tempo and
  number of strokes the mind may ask, either paw: from STAND and back to it, inside the joints'
  limits; the rigid quasi-static model on Kilo as designed and as recorded: only the feet on the
  floor, the centre of mass inside them by at least 3 mm all along, no topple; the head at least
  HEAD_CLEAR_MM from the front legs, either way the pan turns (the check the original paw lick
  failed: it bumped his head on Kilo);
- played on the trick dynamics (kilo.tricks.dynamics: the servos' lag, the legs' give, friction)
  with its guard: no fall and no stop, in the fit and at the corners of what the recordings allow.

Simulation only: nothing here was tried on Kilo.
"""

from __future__ import annotations

import numpy as np
import pytest

from kilo import engine as E
from kilo.tricks import alive_set as A
from kilo.tricks import dynamics as D
from kilo.tricks import rear_up as RU
from kilo.tricks.model import Posture

GEOMETRIES = {"as designed": RU.DESIGN, "as recorded": RU.AS_RECORDED}


@pytest.fixture(scope="module")
def tricks():
    return {(tempo, n): E.paw_wash(tempo, n) for tempo in (0.5, 1.0, 2.0) for n in (1, 3, 8)}


def test_from_stand_and_back(tricks):
    home = A.base_angles("stand")
    for (tempo, n), (t, info) in tricks.items():
        t.validate()
        assert t.frames[-1].angles == home and t.end_posture == Posture.STAND
        assert t.repeats == n and (t.loop_start, t.loop_end) == (3, 4) and not t.balance
        assert t.frames[-1].dur >= A.POSTURE_DUR_TICKS and t.stop_ticks == A.POSTURE_DUR_TICKS
        assert t.guards and t.guards[0].frame == 2 and t.frames[2].hold >= 20  # the guard's still hold
        assert info["margin"] >= 7.0 - 0.06 and info["lean_dx"] < 0 and info["lean_roll"] < 0  # back, onto the right
    quick, slow = tricks[(2.0, 3)][0], tricks[(0.5, 3)][0]
    assert sum(f.dur for f in quick.frames) < sum(f.dur for f in slow.frames) / 2


@pytest.mark.parametrize("geometry", GEOMETRIES)
@pytest.mark.parametrize("side", ["left", "right"])
def test_statically_safe_and_the_head_clear(tricks, geometry, side):
    g3 = GEOMETRIES[geometry]
    for t, _ in (tricks[(1.0, 3)], tricks[(2.0, 1)]):
        t = t if side == "left" else t.mirror()
        c = A.check(t, "stand", g3)
        assert c.problems() == [], c.problems()
        assert c.margin >= 3.0 and c.head >= A.HEAD_CLEAR_MM and c.drop <= A.DROP_MAX_MM
        assert c.contacts <= {"lf_foot", "rf_foot", "rb_foot", "lb_foot"}


def _play(t, p):
    return D.simulate(t, p, start=D.STAND, end=D.STAND, guards=t.guards, roll_max=t.roll_max)


@pytest.mark.parametrize("band", ["fitted", "com_dx+4", "com_dy-3", "slow servos 70+100ms", "softer, further right"])
def test_played_on_the_dynamics(tricks, band):
    p = dict(D.bands())[band]
    t, info = tricks[(1.0, 3)]
    pr = _play(t, p)
    assert pr.fell is None and pr.stopped is None, (pr.fell, pr.stopped)
    up = [c for c in pr.contacts if "lf_foot" not in c]
    assert len(up) * 0.02 > 1.0  # the paw up over a second, on the three other feet
    assert all({"rf_foot", "rb_foot", "lb_foot"} <= c for c in up)
    assert np.max(np.abs(pr.roll)) < abs(info["lean_roll"]) + 2.0


@pytest.mark.slow
def test_played_on_the_dynamics_at_every_corner(tricks):
    for tempo in (0.5, 2.0):
        t, _ = tricks[(tempo, 3)]
        for name, p in D.bands():
            for tt in (t, t.mirror()):
                pr = _play(tt, p)
                assert pr.fell is None and pr.stopped is None, (tempo, name, pr.fell, pr.stopped)
