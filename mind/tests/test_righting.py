"""kilo.tricks.righting: Kilo lying down in the trick simulator, the recorded falls, and the
criterion the model is held to before a getting-up is designed in it (docs/righting.md)."""

import math
import re
from pathlib import Path

import numpy as np
import pytest

from kilo.calibration import POSTURES
from kilo.tricks import dynamics as D
from kilo.tricks import righting as R

REPO = Path(__file__).resolve().parents[2]


# ---- the body's rc ----------------------------------------------------------------------------------
def test_rc_ticks_are_the_bodys():
    """RC_TICKS is the dur and hold of posture.cpp's three rc rows (their last two bytes)."""
    src = (REPO / "body" / "src" / "core" / "posture.cpp").read_text()
    rows = re.findall(r"\{\{([-0-9, ]+)\},\s*(-?\d+),\s*(-?\d+)\}", src)
    assert len(rows) == 7                                  # rest, sit, stand, calib, rc 0-2
    got = tuple((int(d), int(h)) for _, d, h in rows[4:])
    assert got == R.RC_TICKS
    for i, (angles, _, _) in enumerate(rows[4:]):
        assert tuple(int(v) for v in angles.split(",")) == POSTURES[f"recovery {i + 1}"]
    assert R.RC_MOVES_S == pytest.approx(1.78)


def test_rc_mirrored_swaps_the_sides():
    a, b = R.rc_frames(False), R.rc_frames(True)
    for (ja, da, ha), (jb, db, hb) in zip(a, b):
        assert (da, ha) == (db, hb)
        assert jb["head_pan"] == -ja["head_pan"] and jb["tail"] == -ja["tail"] and jb["head_tilt"] == ja["head_tilt"]
        for x, y in (("lf", "rf"), ("lb", "rb")):
            for part in ("shoulder", "knee"):
                assert jb[f"{x}_{part}"] == ja[f"{y}_{part}"] and jb[f"{y}_{part}"] == ja[f"{x}_{part}"]
    # as written it lifts the RIGHT legs (the body: "pushes with the right legs", for the right side down)
    assert a[1][0]["rf_shoulder"] == -99 and a[1][0]["rb_shoulder"] == 115


# ---- TELEMETRY's attitude as a vector -----------------------------------------------------------------
@pytest.mark.parametrize("pitch,roll,what", [
    (2.0, -0.4, "upright"), (-172.7, -177.5, "on his back"), (178.6, -115.8, "on his right side, rolled toward his back"),
    (176.6, 118.4, "on his left side, rolled toward his back"), (1.9, 51.4, "leaning on his left side, feet down"),
    (0.0, 90.0, "on his left side"), (80.0, 0.0, "nose down"), (-80.0, 0.0, "tail down")])
def test_up_of_and_lying(pitch, roll, what):
    up = R.up_of(pitch, roll)
    assert np.linalg.norm(up) == pytest.approx(1.0)
    assert R.lying(up) == what
    p2, r2 = R.telemetry_of(up)
    assert R.angle(R.up_of(p2, r2), up) < 1e-6          # the round trip (the angles wrap at 180)


def test_up_conventions():
    assert R.up_of(0, 0) == pytest.approx([0, 0, 1])
    assert R.up_of(0, 90)[1] < -0.99                      # left side down: up points to his right (y is left)
    assert R.up_of(30, 0)[0] < 0                          # nose down: up points toward the tail
    assert R.up_of(-180, 180)[2] == pytest.approx(-1.0)   # on his back
    assert R.upright(R.up_of(5, -15)) and not R.upright(R.up_of(5, -25))
    for up in (R.up_of(-172.7, -177.5), R.up_of(176.6, 118.4), np.array([0.0, 0.0, 1.0])):
        Rm = R.r0_for(up)
        assert Rm @ Rm.T == pytest.approx(np.eye(3), abs=1e-9) and np.linalg.det(Rm) == pytest.approx(1.0)
        assert Rm[2] == pytest.approx(up / np.linalg.norm(up))


# ---- the recorded falls ----------------------------------------------------------------------------
def test_the_fixture_is_what_the_docs_say():
    fx = R.fixture()
    falls = fx["falls"]
    assert len(falls) == 9
    tries = [(f, t) for f in falls for t in f["tries"]]
    assert len(tries) == 12
    floor = [(f, t) for f, t in tries if f["floor"]]
    assert len(floor) == 6 and all(t["start_lying"] != "upright" for _, t in floor)
    assert sum(t["righted"] for _, t in floor) == 4
    # every try that began upright, and nothing else that ended upright after a failed try, is in a hand
    assert all(not f["floor"] for f, t in tries if t["start_lying"] == "upright")
    # a pitch fall is not tried
    assert [f["ended"] for f in falls if f["fell_arg"] in (2, -2)] == ["RECOVERY_FAILED"]
    # the two that began alike and ended differently
    a = next(t for f, t in floor if f["uptime_s"] == 119.82 and t["n"] == 1)
    b = next(t for f, t in floor if f["uptime_s"] == 119.82 and t["n"] == 3)
    assert R.angle(R.up_of(*a["start"]), R.up_of(*b["start"])) < 2.0 and a["mirrored"] and b["mirrored"]
    assert not a["righted"] and b["righted"]
    # a successful try is over as soon as the body can say so: 1.78 s of moves and 0.5 s level
    assert all(t["lasted_s"] == pytest.approx(2.24, abs=0.05) for _, t in floor if t["righted"])
    assert "floor" in R.falls_table() or "IN A HAND" in R.falls_table()


# ---- the simulator lying down --------------------------------------------------------------------
def test_simulate_is_unchanged_without_a_lying_start():
    """R0 and kin default to what dynamics.simulate always did (rear_up's checks rest on it:
    python -m kilo.tricks.rear_up --check is the full proof)."""
    fr = [(dict(D.STAND, lf_knee=40.0), 5, 5)]
    a = D.simulate(fr, D.FITTED, t_end=0.3)
    b = D.simulate(fr, D.FITTED, t_end=0.3, R0=None, kin=D.Kin(D.FITTED))
    assert np.array_equal(a.nose_up, b.nose_up) and np.array_equal(a.roll, b.roll)
    assert len(a.up) == len(a.t) and a.up[0][2] > 0.99


def test_kin_lying_has_a_head_that_moves_and_hips():
    kin = R.KinLying(D.FITTED)
    q = dict(D.STAND)
    P, c, I = kin.state(q)
    assert len(P) == kin.n == len(kin.labels) == len(kin.radius)
    assert not any(lb.startswith("nose_") for lb in kin.labels)
    for lb in R.HEAD_LABELS + ("lf_hip", "rb_hip"):
        assert lb in kin.labels
    i = kin.labels.index("muzzle_l")
    j = kin.labels.index("ear_l")
    P2, c2, _ = kin.state(dict(q, head_pan=60.0))
    assert P2[i][1] - P[i][1] > 20.0 and c2 == pytest.approx(c)       # the muzzle swings left; the mass stays put
    P3, _, _ = kin.state(dict(q, head_tilt=-70.0))
    assert P3[i][2] < P[i][2] - 20.0 and P3[j][0] > P[j][0]          # nose down, ears forward
    # the feet, knees and tail are dynamics.Kin's own
    P0, _, _ = D.Kin(D.FITTED).state(q)
    assert P[:9] == pytest.approx(P0[:9])


def test_on_his_back_the_model_lies_as_kilo_lay():
    """The rear-up falls of 2026-11-17 ended on his back, 7 degrees nose up: the model stays there
    (the ears' height was set on it; the side attitudes are in rest_check, slower)."""
    fall = R.fixture()["falls"][0]
    tr = fall["tries"][0]
    up0 = R.up_of(*tr["start"])
    st = R.start_joints(fall, 1)
    pr = D.simulate([(st, 1, 0)], D.FITTED, start=st, settle_s=1.0, t_end=0.1, R0=R.r0_for(up0),
                    kin=R.KinLying(D.FITTED))
    assert R.angle(np.array(pr.up[-1]), up0) < 8.0
    assert R.lying(np.array(pr.up[-1])) == "on his back"


def test_start_joints():
    fx = R.fixture()
    assert R.start_joints(fx["falls"][0], 1) == R.REAR_UP_READY
    b8 = next(f for f in fx["falls"] if f["uptime_s"] == 114.44)
    assert R.start_joints(b8, 1)["rb_shoulder"] == -90            # he was sitting
    b8b = next(f for f in fx["falls"] if f["uptime_s"] == 119.82)
    assert R.start_joints(b8b, 1)["lf_knee"] == 30                # standing
    assert R.start_joints(b8b, 2) == R.rc_frames(True)[-1][0]     # after a try: rc's last frame, as mirrored
    sj = R.stance_joints()
    assert sj["lf_shoulder"] == pytest.approx(sj["rf_shoulder"]) and abs(sj["lf_shoulder"]) < 60


# ---- the criterion ----------------------------------------------------------------------------------
def _r(label, rec, got, floor=True, lying="on his back", rest=3.0, rms=10.0, side=(1.4, 1.4)):
    return R.Replay(label=label, floor=floor, start_lying=lying, mirrored=False, recorded_righted=rec, righted=got,
                    rest_err=rest, trace_rms=rms, side_t=side[1], side_t_rec=side[0], end_lying="upright", torque=1.0)


def test_verdict():
    good = [_r("a", True, True), _r("b", True, True), _r("b8@120s try 1", False, False), _r("b8@120s try 3", True, True)]
    v = R.verdict(good)
    assert v.rest_ok and v.outcome_ok and v.motion_ok and v.knife_edge_flips is None and not v.credible
    banded = {"x": good, "y": [_r("b8@120s try 1", False, True), _r("b8@120s try 3", True, True)]}
    assert R.verdict(good, banded).credible
    # a hand's tries do not count either way
    assert R.verdict(good + [_r("h", True, False, floor=False)]).outcome_ok
    # one recorded getting-up the model does not make
    assert not R.verdict(good + [_r("c", True, False)]).outcome_ok
    assert not R.verdict(good + [_r("c", False, True)]).outcome_ok
    assert not R.verdict(good + [_r("c", True, True, rest=20.0)]).rest_ok
    assert not R.verdict(good + [_r("c", True, True, rms=40.0)]).motion_ok
    assert not R.verdict(good + [_r("c", True, True, side=(1.4, 1.9))]).motion_ok
    # the knife edge that never flips (always or never upright) is not credible
    same = {"x": good, "y": good}
    assert R.verdict(good, {k: [_r("b8@120s try 1", False, False), _r("b8@120s try 3", True, False)] for k in same}).knife_edge_flips is False


def test_bands_cover_the_estimates():
    names = [n for n, _, _ in R.bands()]
    assert "fitted" in names and "mu=0.3" in names and "mu=0.8" in names
    for k in ("hip_r", "knee_r", "top_z", "ear_z", "head_len"):
        assert f"{k} x0.75" in names and f"{k} x1.25" in names
    assert "pan the other way" in names and "no head" in names
    assert math.isclose(R.Lying().hip_r, 18.0)


def test_the_verdict_today():
    """What docs/righting.md reports (2026-11-21), pinned: with the fitted numbers and the
    estimated sizes the model lies as Kilo lay, but rc gets him up in none of the four floor
    tries where it got Kilo up. If a measurement changes this, the doc's verdict changes with it."""
    rs = R.replay_all(workers=1, floor_only=True)
    assert len(rs) == 6 and all(r.floor for r in rs)
    v = R.verdict(rs)
    assert v.rest_ok and not v.outcome_ok and not v.motion_ok and not v.credible
    assert sum(r.righted for r in rs if r.recorded_righted) == 0
    assert sum(r.righted for r in rs if not r.recorded_righted) == 0
    # the one thing it follows: from his right side he goes through "on his side" when Kilo did
    b8 = next(r for r in rs if r.label == "b8@114s try 1")
    assert abs(b8.side_t - b8.side_t_rec) < 0.1
    # the leg servos' load stays far under the stall estimate (3 kg cm)
    assert max(r.torque for r in rs) < 2.0
    assert "upright" in R.table(rs)
