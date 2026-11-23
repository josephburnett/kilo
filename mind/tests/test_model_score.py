"""The body model judged by prediction (kilo.measure.score, docs/model-fit-results.md): recorded FEET
replayed through the model and the virtual body against what the tilt sensor read; and the virtual
body's legs with their own servo timing and give (kilo.vbody.physics)."""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from kilo.measure import score as S
from kilo.planner.model import BodyModel
from kilo.vbody.physics import Physics

FIXTURE = Path(__file__).parent / "fixtures" / "model_score_2026_11_22.json"
M = BodyModel()


@pytest.fixture(scope="module")
def segs():
    return S.load_segments([FIXTURE])


def test_the_fixture_round_trips(segs, tmp_path):
    p = S.save_segments(segs, tmp_path / "s.json")
    again = S.load_segments([p])
    assert [s.name for s in again] == [s.name for s in segs]
    for a, b in zip(segs, again):
        assert a.k0 == b.k0 and a.feet.keys() == b.feet.keys()
        assert np.allclose(np.nan_to_num(a.meas), np.nan_to_num(b.meas))


def test_the_model_predicts_the_held_out_recordings(segs):
    """The pinned score of the model in the spec on the fixture (recordings the fit never saw): it may
    get better; worse is a model change that has to say why."""
    preds = [S.replay(s, M) for s in segs]
    t = S.total([S.score(s, p) for s, p in zip(segs, preds)])
    assert t["n"] > 900
    assert t["rms_pitch"] < SCORE_PITCH and t["rms_roll"] < SCORE_ROLL, t
    # ...and better than the planner's own rigid prediction (no lag, no tipping) in both
    plan = S.total([S.score(s, p) for s, p in zip(segs, S.plan_preds(segs, M))])
    assert t["rms_pitch"] < plan["rms_pitch"] and t["rms_roll"] < plan["rms_roll"], (t, plan)


def test_the_imu_offset_is_what_makes_the_pitch(segs):
    """Without the model's IMU offset the pitch is predicted about 2 degrees worse: it is the one number
    every recording shows the same way."""
    seg = max(segs, key=lambda s: s.n)
    flat = replace(M, imu_pitch_deg=0.0, imu_roll_deg=0.0)
    a = S.score(seg, S.replay(seg, M))
    b = S.score(seg, S.replay(seg, flat))
    assert b["rms_pitch"] > a["rms_pitch"] + 1.0


def test_the_search_finds_the_imu_offset_and_its_spread(segs):
    """Started half a degree off, one round of the search goes back toward the model's IMU offset (2
    to 3 deg nose down on these four recordings; a round moves a number at most four of its steps),
    and every boot has its own best value near it."""
    ev = S.Evaluator(segs, procs=1)
    try:
        m0 = replace(M, imu_pitch_deg=1.8)
        out = S.fit(ev, m0, S.Judge(), ["imu_pitch"], rounds=1)
        assert out.cost < out.cost0 and 2.0 < out.values["imu_pitch"] < 3.0, out.values
        sp = S.spread(ev, M, S.Judge(), "imu_pitch", [2.0, 2.15, 2.3, 2.45, 2.6])
        assert len(sp["boots"]) >= 3 and 2.0 < sp["median"] < 2.9 and sp["pm"] < 0.3, sp
    finally:
        ev.close()


def test_values_by_name_round_trip():
    v = S.values_of(M, S.Judge())
    m2, j2 = S.with_values(M, S.Judge(), {"com_x": 1.5, "give_lf": 1.4, "lag_lf": 20.0, "dz_rb": -0.5, "inset": 7})
    w = S.values_of(m2, j2)
    assert (w["com_x"], w["give_lf"], w["lag_lf"], w["dz_rb"], w["inset"]) == (1.5, 1.4, 20.0, -0.5, 7.0)
    assert m2.servo_give[8] == m2.servo_give[12] == 1.4 and m2.servo_lag_d_ms[12] == 20.0
    assert all(w[k] == v[k] for k in v if k not in ("com_x", "give_lf", "lag_lf", "dz_rb", "inset"))


# ---- the virtual body's legs --------------------------------------------------------------------------
def _stand(model: BodyModel, ticks: int = 60, cmd=None, give: bool = True) -> Physics:
    ph = Physics(model, legs_give=give)
    z = model.stance_height(205)
    for k in range(ticks):
        xs, zs = [0.0] * 4, [z] * 4
        if cmd:
            cmd(k, xs, zs)
        ph.command(xs, zs)
        ph.step()
    return ph


def test_rigid_legs_by_default():
    """The walks' virtual body keeps rigid legs unless asked (legs_give); the score's judge asks."""
    ph = _stand(M, give=False)
    assert ph.give is None and ph.sink == [0.0] * 4
    assert S.Judge().legs_give
    assert ph.leg_lag_ms == [M.servo_lag_ms] * 4 and ph.leg_tau_ms == [M.servo_tau_ms] * 4


def test_legs_that_give_sink_toward_the_mass():
    """The centre of mass 5 mm to the left on legs that give: the left feet carry more and sink more,
    so he rolls left side down (roll +), and more with a softer left front leg."""
    soft = replace(M, sag_deg_per_kg_cm=4.0, com_dy_mm=5.0, servo_give=(1.0,) * 16)
    ph = _stand(soft)
    assert ph.sink[0] > ph.sink[1] and ph.sink[3] > ph.sink[2]
    assert ph.roll > 0.05
    lf = list(soft.servo_give)
    lf[8] = lf[12] = 2.0
    ph2 = _stand(replace(soft, servo_give=tuple(lf)))
    assert ph2.roll > ph.roll and ph2.sink[0] > ph.sink[0]
    # rigid legs with the same mass: level
    assert abs(_stand(replace(M, com_dy_mm=5.0), give=False).roll) < 1e-9


def test_a_slower_leg_answers_later():
    """The left front leg told to shorten 6 mm: with its servos 40 ms later the nose goes down later."""
    def drop(k, xs, zs):
        if k >= 10:
            zs[0] -= 6.0

    def first_tilt(model):
        ph = Physics(model, legs_give=False)
        z = model.stance_height(205)
        for k in range(40):
            xs, zs = [0.0] * 4, [z] * 4
            drop(k, xs, zs)
            ph.command(xs, zs)
            ph.step()
            if abs(ph.pitch) > 0.3:
                return k
        return None
    lag = list(M.servo_lag_d_ms)
    lag[8] = lag[12] = 40.0
    a, b = first_tilt(M), first_tilt(replace(M, servo_lag_d_ms=tuple(lag)))
    assert a is not None and b is not None and b - a == 2


def test_a_longer_leg_tilts_the_body():
    """leg_dz_mm: both front legs 2 mm longer than commanded lift the nose (pitch nose down +: negative)."""
    m = replace(M, leg_dz_mm=(2.0, 2.0, 0.0, 0.0))
    ph = _stand(m, give=False)
    assert ph.pitch == pytest.approx(-math.degrees(math.atan(2.0 / m.length_mm)), abs=0.05)


# The pinned score (degrees rms) of the spec's model on the fixture (0.60 / 1.07 on 2026-11-22; the
# model before that day's numbers: 2.57 / 1.26): see docs/model-fit-results.md.
SCORE_PITCH = 0.65
SCORE_ROLL = 1.15
