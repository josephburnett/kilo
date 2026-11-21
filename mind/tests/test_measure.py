"""Kilo measures himself (kilo.measure; docs/self-measurement.md): the
script, the statics, the poses' joint angles, and the fit getting back the
numbers that made a run, on its own measurement model with noise and on
the virtual body's physics (an independent model: rigid legs, a real tip)."""

import json
import math

import numpy as np
import pytest

from kilo.measure import fit as F
from kilo.measure import record as R
from kilo.measure import script as SC
from kilo.measure import simulate as SM
from kilo.measure import statics as ST
from kilo.planner import model as PM
from kilo.planner.model import BodyModel
from kilo.tricks import stance

M = BodyModel()


# ---- the script ----------------------------------------------------------------------------------
def test_script_length_and_order():
    ticks = SC.poses()
    assert len(ticks) == SC.max_ticks() <= 50 * 50  # at most 50 s of poses, no sweep ending early
    names = [SC.EXPERIMENTS[SC.STEPS[t.step].exp].name for t in ticks]
    order = list(dict.fromkeys(names))
    assert order == [e.name for e in SC.EXPERIMENTS]
    assert ticks[0].x == (0, 0, 0, 0) and ticks[0].z == (SC.HOME_Z,) * 4
    assert ticks[-1].x == (0, 0, 0, 0) and ticks[-1].z == (SC.HOME_Z,) * 4  # ends at home


def test_sweeps_start_on_the_tripod_side_of_the_models_crossing():
    cr = SM.crossings()
    for i, st in enumerate(SC.STEPS):
        if st.kind != SC.SWEEP:
            continue
        name = SC.EXPERIMENTS[st.exp].name
        x0 = SC.STEPS[i - 4].dx[0]  # to_start's
        x1 = st.dx[0]
        d = 1 if x1 > x0 else -1
        c = cr[name]
        # the start and the end (no tip ending it early) on either side of the model's crossing, with room
        span = SC.ROLL_SPAN_MM if "roll" in name else SC.SWEEP_SPAN_MM
        before, after = span, span
        if name == "edge_rb":  # the RB shoulder's clamp: it starts further back and ends at RB_MAX_X
            before, after = span + SC.RB_BACK_MM, SC.RB_MAX_X - c
        assert SC.STEPS[i - 4].phase == "to_start"
        assert abs((c - x0) * d - before) <= 1.0, (name, x0, c)
        assert abs((x1 - c) * d - after) <= 1.0 and after >= 5, (name, x1, c)


def test_tip_vectors_are_the_models():
    ticks = SC.poses()
    rp = ST.rigid_path(M, ticks)
    for e, ex in enumerate(SC.EXPERIMENTS):
        if ex.lifted < 0:
            continue
        ks = [k for k, t in enumerate(ticks) if SC.STEPS[t.step].exp == e and SC.STEPS[t.step].phase == "sweep"]
        d = np.array(rp[ks[-1]][:2]) - np.array(rp[ks[0]][:2])
        assert np.allclose(np.round(d * 16), ex.tip_a16, atol=1), (ex.name, d * 16)
        assert rp[ks[0]][2] == 0xF & ~(1 << ex.lifted)  # on the tripod at the start: the lifted foot in the air


def test_runner_tip_ends_the_sweep():
    no = SC.poses()
    start = next(t.tick for t in no if SC.EXPERIMENTS[SC.STEPS[t.step].exp].name == "edge_lf"
                 and SC.STEPS[t.step].phase == "sweep")
    tipped = SC.poses({2: start + 30})
    assert len(tipped) < len(no)
    sweep = [t for t in tipped if SC.STEPS[t.step].exp == 2 and SC.STEPS[t.step].phase == "sweep"]
    x_seen = sweep[30].x[0]
    assert sweep[-1].x[0] == x_seen - SC.TIP_EXTRA_MM  # the front feet's sweep moves x down
    lifted = next(t.tick for t in tipped if SC.STEPS[t.step].exp == 3 and SC.STEPS[t.step].phase == "lifted")
    cut = SC.poses({2: start + 30, 3: lifted + 3})
    sweep3 = [t for t in cut if SC.STEPS[t.step].exp == 3 and SC.STEPS[t.step].phase == "sweep"]
    assert len(sweep3) == SC.TIP_EXTRA_MM * SC.TICKS_PER_MM  # tipped as the foot came up: 3 mm of sweep


def test_every_pose_inside_the_joint_limits_and_servo_pulses():
    """The body clamps every joint (spec joint limits) and every servo pulse (Kilo's calibration,
    stance.KILO_CZP): no pose of the script needs either (the angles from the body's IK, core/ik.h:
    shoulder = -t1, knee = -(t2 - sigma 90))."""
    worst, where = 1e9, None
    seen = set()
    for t in SC.poses():
        key = t.x + t.z
        if key in seen:
            continue
        seen.add(key)
        for leg in PM.LEGS:
            t1, t2 = M.ik(leg, t.x[leg], t.z[leg])
            sigma = 1 if PM.FRONT[leg] else -1
            sh = -math.degrees(t1)
            kn = -(math.degrees(t2) - sigma * 90.0)
            for j, a in ((8 + leg, sh), (12 + leg, kn)):
                lo, hi = stance.reach(j)
                if min(a - lo, hi - a) < worst:
                    worst, where = min(a - lo, hi - a), (j, round(a, 1), lo, hi, t.x, t.z)
            d = math.hypot(t.x[leg], t.z[leg])
            assert M.min_reach_mm + 1 < d < M.reach_mm - 1
    assert worst > 3.0, (worst, where)  # degrees to spare at the tightest


# ---- the statics ----------------------------------------------------------------------------------
def test_compliant_statics_vectorised_and_rigid_agree():
    ticks = SC.poses()
    xs = np.array([t.x for t in ticks], float)
    zs = np.array([t.z for t in ticks], float)
    u, inv = np.unique(np.hstack([xs, zs]), axis=0, return_inverse=True)
    P = ST.Poses(M, u[:, :4], u[:, 4:])
    body = ST.Body(sag=3.0, com_dx=-3, com_dy=2, com_dz=5, leg_dz=(0.4, -0.4, 0.4, -0.4))
    a = ST.rest(P, body)
    assert a.ok.all()
    assert np.allclose(a.loads.sum(1), M.total_g, atol=0.1)  # the feet carry the weight
    # ...and its moments: the loads' centre is under the centre of mass, in the world (the tilt leans it)
    for i in range(0, len(P), 7):
        dz = np.array(body.leg_dz)
        com, feet = PM.world_points(M, P.xs[i], P.zs[i] + dz, a.pitch[i], a.roll[i])
        R = PM.rotation(a.pitch[i], a.roll[i])
        d = np.array(R) @ np.array([body.com_dx, body.com_dy, body.com_dz])
        F = a.loads[i]
        cx = sum(F[k] * feet[k][0] for k in range(4)) / F.sum()
        cy = sum(F[k] * feet[k][1] for k in range(4)) / F.sum()
        assert abs(cx - com[0] - d[0]) < 0.6 and abs(cy - com[1] - d[1]) < 0.6, i  # the legs' sag moves the feet a little
    # stiff legs: the rigid model's path (the C++ prediction)
    c = ST.candidates(P, ST.Body(sag=ST.SAG_MIN))
    p, r, _ = ST.path(c, inv.ravel())
    rp = np.array([x[:2] for x in ST.rigid_path(M, ticks)])
    assert np.abs(rp - np.stack([p, r], 1)).max() < 0.02


def test_sag_tilts_the_tripod_toward_the_lifted_foot():
    P = ST.Poses(M, [[22] * 4], [[SC.HOME_Z - SC.LIFT_MM] + [SC.HOME_Z] * 3])
    stiff, soft = ST.rest(P, ST.Body(sag=0.02)), ST.rest(P, ST.Body(sag=3.0))
    assert stiff.support[0] == soft.support[0] == 0b1110
    assert abs(soft.pitch[0] - stiff.pitch[0]) + abs(soft.roll[0] - stiff.roll[0]) > 0.1


# ---- the fit ----------------------------------------------------------------------------------------
def _truth(**kw):
    v = dict(F.model_values())
    v.update(kw)
    v["tip_ms"] = v["lag_ms"] - v["tau_ms"] * math.log(1.0 - SC.TIP_FRACTION)
    return v


def test_fit_recovers_perturbed_parameters():
    mv = F.model_values()
    truth = _truth(imu_pitch=1.2, imu_roll=-2.0, com_x=mv["com_x"] - 4, com_y=-3.0, com_z=mv["com_z"] + 6, sag=3.0,
                   twist=0.8, lag_ms=60, tau_ms=80)
    run = SM.synth_run(truth, noise=SM.IMU_NOISE_DEG, seed=5)
    res = F.Fitter([run]).run()
    tol = {"imu_pitch": 0.05, "imu_roll": 0.05, "com_x": 1.0, "com_y": 0.5, "com_z": 3.0, "sag": 0.2, "twist": 0.05,
           "lag_ms": 2.0, "tau_ms": 2.0}
    for k, t in tol.items():
        assert abs(res.values[k] - truth[k]) < t, (k, res.values[k], truth[k])
        assert abs(res.values[k] - truth[k]) < 4 * res.sd[k] + 1e-3, (k, res.values[k], truth[k], res.sd[k])
    assert res.significant("com_x") and res.significant("sag") and res.significant("imu_roll")
    assert sum(c["measured_x"] is not None for c in res.crossings) >= 5
    js = F.to_json(res, ["synthetic"])
    assert set(F.ROWS) <= set(js["parameters"]) and js["parameters"]["sag"]["significant"]


def test_fit_on_the_virtual_bodys_physics():
    """An independent model (kilo.vbody.physics: rigid, a real inverted-pendulum tip): the centre of
    mass and the IMU offsets come back."""
    run = SM.physics_run((-4.0, -3.0, 0.0), (1.5, -2.0), seed=3)
    res = F.Fitter([run]).run()
    t = run.meta["truth"]
    assert abs(res.values["imu_pitch"] - 1.5) < 0.1 and abs(res.values["imu_roll"] + 2.0) < 0.1
    assert abs(res.values["com_x"] - t["com_x"]) < 1.0 and abs(res.values["com_y"] - t["com_y"]) < 0.5
    assert abs(res.values["com_z"] - t["com_z"]) < 3 * res.sd["com_z"] + 1.0
    assert res.values["sag"] < 0.3
    assert 40 < res.values["lag_ms"] < 65 and 55 < res.values["tau_ms"] < 90
    assert res.tip_rate["events"] == 6


def test_fit_with_a_turned_run_separates_the_floor():
    mv = F.model_values()
    a = SM.synth_run(_truth(imu_pitch=0.5 + 1.0, imu_roll=-1.0 + 0.6, sag=2.0), seed=1)  # IMU + floor
    b = SM.synth_run(_truth(imu_pitch=0.5 - 1.0, imu_roll=-1.0 - 0.6, sag=2.0), seed=2)  # turned: IMU - floor
    b.heading = -1
    res = F.Fitter([a, b]).run()
    assert abs(res.values["imu_pitch"] - 0.5) < 0.05 and abs(res.values["imu_roll"] + 1.0) < 0.05
    assert abs(res.values["floor_pitch"] - 1.0) < 0.05 and abs(res.values["floor_roll"] - 0.6) < 0.05
    assert abs(res.values["com_x"] - mv["com_x"]) < 1.0


def test_a_walk_error_stream_adds_to_the_servo_response():
    run = SM.synth_run(_truth(lag_ms=60, tau_ms=80, sag=2.0), seed=4, backlash=0.15)
    rng = np.random.default_rng(0)
    n = 3000
    t = np.arange(n)
    u = np.stack([2.5 * np.sin(t / 7.0) + 1.0 * np.sin(t / 2.3), 3.0 * np.sin(t / 5.0 + 1)], 1)
    meas = F.servo(u, 60.0, 80.0) + np.array([0.3, -0.4]) + rng.normal(0, 0.1, (n, 2))
    walk = F.Walk("synthetic walk", u=u, meas=meas, used=np.ones(n, bool))
    alone = F.Fitter([run]).run()
    both = F.Fitter([run], [walk]).run()
    assert both.sd["lag_ms"] < alone.sd["lag_ms"] and both.sd["tau_ms"] < alone.sd["tau_ms"]
    assert abs(both.values["lag_ms"] - 60) < 3 and abs(both.values["tau_ms"] - 80) < 4
    assert abs(both.values["walk_pitch:synthetic walk"] - 0.3) < 0.05


# ---- the CLI and the files -------------------------------------------------------------------------
def test_cli_simulate_write_and_fit(tmp_path, capsys):
    from kilo import cli

    p = tmp_path / "sim.json"
    assert cli.main(["measure", "simulate", "--write", str(p), "--seed", "3"]) == 0
    runs = R.load(p)
    assert len(runs) == 1 and len(runs[0]) > 1500 and "truth" in runs[0].meta
    out = tmp_path / "fit.json"
    assert cli.main(["measure", "fit", str(p), "--out", str(out)]) == 0
    text = capsys.readouterr().out
    assert "com_x" in text and "significant" in text and "Nothing here changes the walks' model" in text
    d = json.loads(out.read_text())
    truth = runs[0].meta["truth"]
    assert abs(d["parameters"]["com_y"]["value"] - truth["com_y"]) < 1.0
    assert d["parameters"]["com_x"]["model"] == pytest.approx(F.model_values()["com_x"], abs=1e-3)


def test_cli_plan(capsys):
    from kilo import cli

    assert cli.main(["measure", "plan"]) == 0
    text = capsys.readouterr().out
    for name in ("level", "steps", "edge_lf", "edge_rb_roll", "dropped", "tail swing"):
        assert name in text
