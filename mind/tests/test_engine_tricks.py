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
  with its guard: no fall and no stop, in the fit and at the corners of what the recordings allow;
- the stretch (front down, rear up, held, the shake of the rear, back), from STAND and from SIT, at
  every tempo: the same checks, and at least as far inside his feet, with the head at least as far
  from the front legs, as the library stretch it replaced.

Simulation only: nothing here was tried on Kilo.
"""

from __future__ import annotations

import numpy as np
import pytest

from kilo import engine as E
from kilo.tricks import alive_set as A
from kilo.tricks import dynamics as D
from kilo.tricks import rear_up as RU
from kilo.tricks import sagittal as S
from kilo.tricks.model import CANONICAL, JOINT_IDS, JOINT_NAMES, Posture

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


# ---- the stretch (kernel/mind/engine/stretch.h) ----------------------------------------------------------
@pytest.fixture(scope="module")
def stretches():
    return {(base, tempo): E.stretch(base, tempo) for base in A.BASES for tempo in (0.5, 0.75, 1.0, 2.0)}


def test_the_stretch_from_its_base_and_back(stretches):
    for (base, tempo), (t, info) in stretches.items():
        t.validate()
        assert t.frames[-1].angles == A.base_angles(base)
        assert t.end_posture == {"sit": Posture.SIT, "stand": Posture.STAND}[base]
        assert t.frames[-1].dur >= A.POSTURE_DUR_TICKS and not t.balance and t.repeats == 0
        assert 22 <= info["pitch"] <= 27  # front down, rear up: the original's 25 deg nose down
    quick, slow = stretches[("stand", 2.0)][0], stretches[("stand", 0.5)][0]
    assert sum(f.dur + f.hold for f in quick.frames) < sum(f.dur + f.hold for f in slow.frames) / 2


@pytest.mark.parametrize("base", A.BASES)
def test_the_stretch_shakes_its_rear_gently(stretches, base):
    """At the end of the hold both hind feet go a few mm forward and back together, the tail with them,
    everything else held (the owner, 2026-11-22)."""
    t, info = stretches[(base, 1.0)]
    hold = t.frames[info["shake_from"] - 1]
    shake = t.frames[info["shake_from"]:-1]
    assert hold.hold >= 40 and len(shake) == 7
    g = S.Geometry()

    def foot(a, leg):
        return S.leg_points(g, False, a[JOINT_IDS[f"{leg}_shoulder"]], a[JOINT_IDS[f"{leg}_knee"]])[2]

    xs = []
    for f in shake:
        for j in CANONICAL:
            if JOINT_NAMES[j] not in ("lb_shoulder", "rb_shoulder", "lb_knee", "rb_knee", "tail"):
                assert f.angles[j] == hold.angles[j]
        (x0, z0), (x1, z1) = foot(hold.angles, "lb"), foot(f.angles, "lb")
        assert foot(f.angles, "rb") == (x1, z1)
        assert abs(z1 - z0) < 1.5
        xs.append(x1 - x0)
    swings = xs[:-1]
    assert all(2.5 < abs(x) < 6.0 for x in swings) and abs(xs[-1]) < 1e-9
    assert all(a * b < 0 for a, b in zip(swings, swings[1:]))


# the library stretch it replaced (alive_set, 2026-11-23): its least margin and the head's least gap
ORIGINAL = {("as designed", "stand"): 34.4, ("as designed", "sit"): 34.4, ("as recorded", "stand"): 32.4,
            ("as recorded", "sit"): 0.1}


@pytest.mark.parametrize("geometry", GEOMETRIES)
@pytest.mark.parametrize("base", A.BASES)
def test_the_stretch_statically_as_safe_as_the_original(stretches, geometry, base):
    """On the rigid model as the random mode's library tricks are held to (alive_set.check): on his feet,
    the centre of mass over them, no topple past DROP_MAX_MM, the head clear of the front legs; and at
    least the library stretch's margin (ORIGINAL) and head gap (27.6 mm)."""
    g3 = GEOMETRIES[geometry]
    for tempo in (0.5, 1.0, 2.0):
        t, _ = stretches[(base, tempo)]
        c = A.check(t, base, g3)
        assert c.problems() == [], c.problems()
        assert c.contacts <= {"lf_foot", "rf_foot", "rb_foot", "lb_foot"}
        assert c.head >= 27.6 and c.margin >= ORIGINAL[(geometry, base)] - 0.05, (c.head, c.margin)


@pytest.mark.parametrize("band", ["fitted", "com_dx+4", "com_dy-3", "slow servos 70+100ms", "softer, further right"])
def test_the_stretch_on_the_dynamics(stretches, band):
    p = dict(D.bands())[band]
    for base in A.BASES:
        t, _ = stretches[(base, 0.75)]  # the random mode's pace
        start = D.STAND if base == "stand" else D.SIT
        pr = D.simulate(t, p, start=start, end=start, roll_max=t.roll_max)
        assert pr.fell is None and pr.stopped is None, (base, pr.fell, pr.stopped)
        assert np.max(np.abs(pr.roll)) < 3.0
        assert max(v[0] for v in pr.torque.values()) < 2.4
