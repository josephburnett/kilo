"""The body model's geometry and sign conventions (kilo.planner.model), its
support margin, and the model's error from the telemetry (kilo.planner.errors).
Pure computation: no body. (The walks themselves are the engine's, in C++:
kernel/host/tests/test_engine.cpp, judged by mind/tests/enginejudge.py.)"""

import math

import pytest

from kilo.planner import errors
from kilo.planner.model import LEGS, LF, RB, RF, LB, BodyModel, fit_attitude, leg_lengths, rotation, stability, world_points
from kilo.protocol import kilo_protocol as P

M = BodyModel()
H = M.stance_height(205)  # 76 mm: STANCE's body height


def test_model_masses_and_reach():
    assert M.total_g == pytest.approx(358.0)  # weighed 2026-11-14, the Pi mounted
    cx, cy, cz = M.com([0] * 4, [H] * 4)
    # as measured by hand (7.5 ahead), moved by the correction the recordings predict best with (-3 mm)
    assert abs(cy) < 1e-9 and cx == pytest.approx(7.5 + M.com_dx_mm, abs=0.2) and cz == pytest.approx(-3.0, abs=0.3)
    assert H == pytest.approx(76.0, abs=0.1)
    assert M.tau_s(80) == pytest.approx(0.090, abs=0.001)  # the 90 ms tip time constant of the doc
    for leg in LEGS:
        for x, z in ((0, H), (20, 70), (-25, 60), (10, 85)):
            t1, t2 = M.ik(leg, x, z)
            assert M.fk(t1, t2) == pytest.approx((x, z), abs=1e-6)
            assert (t2 > 0) == (leg in (LF, RF))  # front knees point back, back knees forward
    assert not M.reachable(0, 95) and M.reachable(0, 90)


def test_leg_lengths_and_attitude_conventions():
    """Pitch + = nose down: the back legs are the longer; roll + = right side
    up: the right legs are the longer (TELEMETRY, the firmware's H_i)."""
    xs = [0.0] * 4
    zs = leg_lengths(M, xs, H, 5.0, 0.0)
    assert zs[RB] > zs[RF] and zs[LB] > zs[LF] and sum(zs) / 4 == pytest.approx(H)
    zs = leg_lengths(M, xs, H, 0.0, 5.0)
    assert zs[RF] > zs[LF] and zs[RB] > zs[LB]
    for p, r in ((3.0, -7.0), (-4.0, 9.0), (0.0, 12.0)):
        zs = leg_lengths(M, [5.0, -3.0, 2.0, -8.0], H, p, r)
        assert fit_attitude(M, [5.0, -3.0, 2.0, -8.0], zs, 0xF) == pytest.approx((p, r), abs=1e-4)
        three = 0xF & ~(1 << RB)
        assert fit_attitude(M, [5.0, -3.0, 2.0, -8.0], zs, three) == pytest.approx((p, r), abs=1e-4)
    # on two feet the plane is free about their line: the previous attitude fills in
    diag = (1 << LF) | (1 << RB)
    assert fit_attitude(M, xs, [H] * 4, diag, prev=(0.0, 0.0)) == pytest.approx((0.0, 0.0), abs=1e-6)
    # the world: a nose-down pitch lowers the front feet, a positive roll the left ones
    R = rotation(10.0, 0.0)
    assert R[2][0] < 0  # height of a point ahead (+x) goes down
    R = rotation(0.0, 10.0)
    assert R[2][1] < 0  # right side up is + roll: a point on the left (+y) goes down
    com, feet = world_points(M, xs, leg_lengths(M, xs, H, 0.0, 10.0), 0.0, 10.0)
    assert feet[RF][2] == pytest.approx(feet[LF][2], abs=1e-6)  # all four on the ground
    feet_y = sum(f[1] for f in feet) / 4
    assert com[1] - feet_y == pytest.approx((H + M.foot_r_mm + M.com(xs, [H] * 4)[2]) * math.sin(math.radians(10)),
                                            abs=0.5)  # right side up: the mass moves left of its feet, h sin(roll)


def test_stability_margin():
    zs = [H] * 4
    four = stability(M, [0.0] * 4, zs, 0xF, 0.0, 0.0)
    assert four == pytest.approx(M.track_mm / 2, abs=0.5)  # the half track
    # on three feet the mass sits near the diagonal's edge (nose heavy: 4.4 mm ahead of the centre)
    for lifted in LEGS:
        m = stability(M, [0.0] * 4, zs, 0xF & ~(1 << lifted), 0.0, 0.0)
        assert abs(m) < 4.0
    # a diagonal pair has no inside
    assert stability(M, [0.0] * 4, zs, (1 << LF) | (1 << RB), 0.0, 0.0) <= 0


def _tel(tick, meas, ref, corr=(0.0, 0.0)):
    return P.Telemetry(tick=tick & 0xFFFF, pitch=round(meas[0] * 16), roll=round(meas[1] * 16),
                       ref_pitch=round(ref[0] * 16), ref_roll=round(ref[1] * 16),
                       corr_pitch=round(corr[0] * 16), corr_roll=round(corr[1] * 16), mode=P.Mode.STREAM)


def test_error_offset_lag_and_spike():
    tr = errors.ErrorTracker()
    for t in range(300):
        ref = 8 * math.sin(2 * math.pi * t / 75)
        meas = 8 * math.sin(2 * math.pi * (t - 2) / 75) + 1.5  # 2 ticks (40 ms) late, 1.5 deg off
        tr.add(t, _tel(t, (0.4, meas), (0.0, ref), (0.0, -0.5)))
    e = tr.estimate()
    assert e["lag_ms"] == 40 and e["lag_quality"] > 0.95
    assert e["offset_roll_deg"] == pytest.approx(1.5, abs=0.2) and e["offset_pitch_deg"] == pytest.approx(0.4, abs=0.05)
    assert e["corr_roll_deg"] == pytest.approx(-0.5, abs=0.05)
    assert tr.offset[0] == pytest.approx(0.4, abs=0.05)
    assert errors.ErrorTracker().estimate() is None
    # a plan that does not move (the trot) gives no lag
    flat = errors.ErrorTracker()
    for t in range(100):
        flat.add(t, _tel(t, (0.0, 0.3), (0.0, 0.0)))
    assert flat.estimate()["lag_ms"] is None
    # from a telemetry history: the newest run in STREAM, ticks unwrapped
    hist = [(t * 0.02, _tel(65500 + t, (0, 8 * math.sin(t / 10)), (0, 8 * math.sin(t / 10)))) for t in range(120)]
    assert errors.from_history(hist, int(P.Mode.STREAM))["lag_ms"] == 0
