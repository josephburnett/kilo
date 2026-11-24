"""The movement engine's postures as goals (docs/movement-engine.md, step 3), as the mind makes them
(kernel/mind/engine/rest.h and posture.h, through kernel/host's engine-judge, kilo.engine):

- the engine's rest finder (where the body comes to rest on its feet, knees, haunches, belly or tail)
  is the tricks' (kilo.tricks.stance): the same attitude, margin and contacts for the postures and
  for poses on the way between them, with the servos as asked, as Kilo's EEPROM has them and with
  the right hind shoulder re-seated;
- its moves between STAND, SIT and REST, at every tempo it allows: from the posture and exactly to
  the other one, the weight further inside what carries him than the body's own all-at-once change
  all along (the rigid model, Kilo as designed), nothing but the feet, knees, haunches, belly and
  tail on the floor;
- played on the tricks' dynamics (kilo.tricks.dynamics: the servos' lag, the legs' give, friction)
  with the mind's roll limit: no fall, no stop, no leg servo over TORQUE_MAX, ending where the
  state estimate reads the posture asked for, in the fit and at every corner of what the
  recordings allow; and rocking no more than the body's own change does.

Simulation only: nothing here was tried on Kilo.
"""

from __future__ import annotations

import numpy as np
import pytest

from kilo import engine as EN
from kilo.intent import estimate as E
from kilo.tricks import dynamics as D
from kilo.tricks import rear_up as RU
from kilo.tricks import stance as T
from kilo.tricks.library import load
from kilo.tricks.model import CANONICAL, JOINT_NAMES, Keyframe, Trick

POSTURES = ("stand", "sit", "rest")
PAIRS = [(a, b) for a in POSTURES for b in POSTURES if a != b]
STATE = {"stand": E.State.STANDING, "sit": E.State.SITTING, "rest": E.State.RESTING}
CMD = {"stand": E.Cmd.STAND, "sit": E.Cmd.SIT, "rest": E.Cmd.REST}
TEMPOS = (0.5, 1.0, 1.5)
MARGIN_MM = 7.5      # engine/posture.h kPostureMarginMm
ROLL_MAX = 25.0      # ...kRollMaxDeg: the player stops the move past it
TORQUE_MAX = 2.4     # kg cm on a leg servo (rear_up.TORQUE_MAX: 80 % of the estimated stall)
# what may carry him (rest.h kMayCarry; stance's labels)
ALLOWED = set(T.FEET) | set(T.KNEE.values()) | {T.TAIL, "butt_l", "butt_r", "belly_l", "belly_r"}
NAME = {"haunch_l": "butt_l", "haunch_r": "butt_r"}  # the engine's labels -> stance's
SERVOS = {"asked": None, "eeprom": T.KILO_CZP, "reseated": RU.DESIGN_CZP}


def angles(name: str) -> list[float]:
    """The posture's eleven joints (canonical order) as the body holds them."""
    return [float(load(name).frames[0].angles[j]) for j in CANONICAL]


def joints(a) -> dict:
    return {JOINT_NAMES[j]: float(v) for j, v in zip(CANONICAL, a)}


@pytest.fixture(scope="module")
def moves():
    return {(a, b, t): EN.posture(a, b, t) for a, b in PAIRS for t in TEMPOS}


# ---- the rest finder -------------------------------------------------------------------------------
def _poses():
    out = [angles(p) for p in POSTURES]
    for a, b in PAIRS:  # on the way between, all at once
        pa, pb = np.array(angles(a)), np.array(angles(b))
        out += [list(pa + (pb - pa) * s) for s in (0.25, 0.5, 0.75)]
    return out


@pytest.mark.parametrize("servos", SERVOS)
def test_rest_finder_is_the_tricks(servos):
    g3 = T.Geometry3(czp=SERVOS[servos])
    for a in _poses():
        mine = EN.rest(a, servos)
        ref = T.rest(g3, joints(a))
        assert abs(mine["pitch"] - ref.pitch) < 0.1 and abs(mine["roll"] - ref.roll) < 0.1, (a, mine, ref)
        assert abs(mine["margin"] - ref.margin) < 0.2, (a, mine["margin"], ref.margin)
        assert {NAME.get(c, c) for c in mine["contacts"]} == set(ref.contacts), (a, mine, ref.contacts)


def test_sitting_and_lying_as_the_recordings_have_them():
    """TELEMETRY's attitude in SIT and REST, by the engine's model, against Kilo's recordings
    (docs/movement-engine-results.md, "Step 3"): sitting 26.8 nose up (13 runs, 26.1 to 27.2), level;
    lying 1.4 nose down, either level or rolled 7.6 to his right."""
    sit = EN.rest(angles("sit"), "reseated")
    assert abs(-sit["telemetry"][0] - 26.8) < 1.5 and abs(sit["telemetry"][1]) < 2.0
    assert sit["contacts"] == {"lf_foot", "rf_foot", "rb_foot", "lb_foot"}
    rest = EN.rest(angles("rest"), "reseated")
    assert abs(rest["telemetry"][0] - 1.4) < 1.5 and abs(rest["telemetry"][1] + 7.6) < 1.0
    # the haunches: 2.8 mm off the floor sitting (an estimate of the frame's outline; the legs' give
    # may well put them down)
    assert sit["contacts"].isdisjoint({"haunch_l", "haunch_r"})


# ---- the moves ------------------------------------------------------------------------------------
def test_from_the_posture_exactly_to_the_other(moves):
    for (a, b, tempo), (t, info) in moves.items():
        assert t is not None, (a, b, info)
        t.validate()
        assert [t.frames[-1].angles[j] for j in CANONICAL] == angles(b)
        assert info["margin"] >= MARGIN_MM and info["margin"] >= info["direct"], (a, b, info)
        assert not t.balance and t.roll_max == ROLL_MAX
    # the tempo: the same way, twice as long at half the pace
    for a, b in PAIRS:
        slow, quick = moves[(a, b, 0.5)][0], moves[(a, b, 1.0)][0]
        assert [f.angles for f in slow.frames] == [f.angles for f in quick.frames]
        assert abs(sum(f.dur for f in slow.frames) - 2 * sum(f.dur for f in quick.frames)) <= 2


def test_the_ways_the_engine_chose(moves):
    info = {(a, b): moves[(a, b, 1.0)][1] for a, b in PAIRS}
    t = {(a, b): moves[(a, b, 1.0)][0] for a, b in PAIRS}
    # stand <-> sit: all at once, over the body's own 25 ticks
    for k in (("stand", "sit"), ("sit", "stand")):
        assert info[k]["stages"] == 1 and t[k].frames[0].dur == 25
    # up from rest: the hind legs first; down to it: the front first
    assert info[("rest", "stand")]["plan"].startswith("hind shoulders+hind knees, then front")
    assert info[("stand", "rest")]["plan"].startswith("front shoulders+front knees, then hind")
    # between sitting and resting: through standing
    assert info[("sit", "rest")]["via"] == "stand" and info[("rest", "sit")]["via"] == "stand"


def _statics(t: Trick, start: str, g3) -> tuple[float, set]:
    frames = [joints([f.angles[j] for j in CANONICAL]) for f in t.frames]
    ss = T.follow(g3, frames, joints(angles(start)), steps=40)
    return min(s.rest.margin for s in ss), set().union(*(s.rest.contacts for s in ss))


def _all_at_once(a: str, b: str) -> Trick:
    """The body's own posture change: every joint straight to the posture over POSTURE_TICKS, eased."""
    return Trick(f"body_{a}_to_{b}", (Keyframe.from_joints(joints(angles(b)), dur=25, hold=5),))


@pytest.mark.parametrize("geometry", ["as designed", "as recorded"])
def test_statically_further_inside_than_the_bodys_own_change(moves, geometry):
    g3 = RU.DESIGN if geometry == "as designed" else RU.AS_RECORDED
    for a, b in PAIRS:
        t, _ = moves[(a, b, 1.0)]
        m, touched = _statics(t, a, g3)
        old, _ = _statics(_all_at_once(a, b), a, g3)
        assert touched <= ALLOWED, (a, b, touched - ALLOWED)
        assert m >= old - 0.05, (a, b, m, old)
        if geometry == "as designed":
            assert m >= MARGIN_MM - 0.3, (a, b, m)
        else:
            assert m >= 0, (a, b, m)  # the EEPROM's right hind shoulder (before the re-seat): at rest throughout


def _judge(t: Trick, a: str, b: str, p: D.Params) -> dict:
    pr = D.simulate(t, p, start=joints(angles(a)), end=joints(angles(b)), roll_max=ROLL_MAX)
    n = len(pr.t)
    tail = slice(max(n - 25, 0), n)
    nose, roll = float(np.mean(pr.nose_up[tail])), float(np.mean(pr.roll[tail]))
    ans = E.classify(E._a16(-nose), E._a16(roll), CMD[b], STATE[a])
    touched = set().union(*pr.contacts) if pr.contacts else set()
    torque = max((v[0] for v in pr.torque.values()), default=0.0)
    return {"fell": pr.fell, "stopped": pr.stopped, "state": ans.state, "touched": touched, "torque": torque,
            "max_roll": float(np.max(np.abs(pr.roll)))}


def _ok(r: dict, b: str) -> list[str]:
    bad = []
    if r["fell"] is not None:
        bad.append(f"fell at {r['fell']:.2f} s")
    if r["stopped"] is not None:
        bad.append(f"stopped: {r['stopped']}")
    if r["state"] != STATE[b]:
        bad.append(f"ends {r['state'].name}")
    if r["touched"] - ALLOWED:
        bad.append(f"touches {sorted(r['touched'] - ALLOWED)}")
    if r["torque"] > TORQUE_MAX:
        bad.append(f"a servo carries {r['torque']:.2f} kg cm")
    return bad


@pytest.mark.parametrize("tempo", TEMPOS)
def test_played_on_the_dynamics(moves, tempo):
    for a, b in PAIRS:
        t, _ = moves[(a, b, tempo)]
        r = _judge(t, a, b, D.NOMINAL)
        assert _ok(r, b) == [], (a, b, tempo, _ok(r, b))
        assert r["max_roll"] < ROLL_MAX - 10, (a, b, tempo, r["max_roll"])
        if tempo == 1.0:
            # rolls about as much as the body's own change: lying down both end rolled 6 deg onto three
            # feet (REST's left hind knee stops short), the engine's way 1.0 deg past that on the way, the
            # same elsewhere (docs/movement-engine-results.md, "Step 3")
            old = _judge(_all_at_once(a, b), a, b, D.NOMINAL)
            assert r["max_roll"] <= old["max_roll"] + 1.5, (a, b, r["max_roll"], old["max_roll"])


@pytest.mark.slow
def test_played_on_the_dynamics_at_every_corner(moves):
    for tempo in (1.0, 1.5):
        for a, b in PAIRS:
            t, _ = moves[(a, b, tempo)]
            for name, p in D.bands():
                r = _judge(t, a, b, p)
                assert _ok(r, b) == [], (a, b, tempo, name, _ok(r, b))
