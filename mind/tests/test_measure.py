"""Kilo measures himself (kilo.measure; docs/self-measurement.md): the
script (script 2, and script 1 still loading), its tip rule, the statics,
the poses' joint angles, and the fit getting back the numbers that made a
run, on its own measurement model with noise and backlash and on the
virtual body's physics (an independent model: rigid legs, a real tip)."""

import json
from dataclasses import replace
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
E = SC.SCRIPT.index


def _phase_start(ticks, exp: int, phase: str) -> int:
    return next(t.tick for t in ticks if SC.STEPS[t.step].exp == exp and SC.STEPS[t.step].phase == phase)


# ---- the script ----------------------------------------------------------------------------------
def test_script_length_and_order():
    ticks = SC.poses()
    assert len(ticks) == SC.max_ticks() <= 90 * 50  # at most 90 s of poses, no sweep ending early
    names = [SC.EXPERIMENTS[SC.STEPS[t.step].exp].name for t in ticks]
    order = list(dict.fromkeys(names))
    assert order == [e.name for e in SC.EXPERIMENTS]
    assert ticks[0].x == (0, 0, 0, 0) and ticks[0].z == (SC.HOME_Z,) * 4
    assert ticks[-1].x == (0, 0, 0, 0) and ticks[-1].z == (SC.HOME_Z,) * 4  # ends at home
    assert all(t.into == 0 for t in ticks if t.first)
    # every experiment ends standing at home for a while (how well he comes back is data), every lifted
    # hold is long enough to be read, and a rolled sweep has the flat sweep of the same foot to stand against
    for e, ex in enumerate(SC.EXPERIMENTS):
        last = [st for st in SC.STEPS if st.exp == e][-1]
        assert last.kind == SC.HOLD and last.dx == SC.ZERO and last.dz == SC.ZERO and last.ticks >= 10, ex.name
        if ex.roll_mm:
            flat = SC.EXPERIMENTS[E(ex.of)]
            assert flat.lifted == ex.lifted and flat.roll_mm == 0
    assert all(st.ticks >= 25 for st in SC.STEPS if st.phase == "lifted")
    assert all(st.ticks >= 25 for st in SC.STEPS if st.phase in ("hold", "up_hold") and st.exp != E("check")
               or st.phase == "up_hold")
    assert len(SC.STEPS) <= 224  # kernel/mind/measure/script.h kMaxSteps


def test_stairs_reach_every_level_from_both_sides():
    for levels in (SC.STAIRS_PITCH, SC.STAIRS_ROLL):
        seen: dict[int, set] = {}
        prev = 0
        for lv in levels:
            seen.setdefault(lv, set()).add("below" if lv > prev else "above")
            prev = lv
        top = max(levels)
        for lv, how in seen.items():
            if abs(lv) != top:
                assert how == {"below", "above"}, (lv, how)
        assert levels[-1] == 0 and {abs(a - b) for a, b in zip((0,) + levels, levels)} >= {SC.STAIR_MM, top}


def test_script_1_is_kept_for_its_recordings():
    s1 = SC.SCRIPTS[1]
    assert s1.version == 1 and len(s1.steps) == 72 and s1.max_ticks() == 2431  # as recorded on 2026-11-21
    assert [e.name for e in s1.experiments] == ["level", "steps", "edge_lf", "edge_rf", "edge_lb", "edge_rb",
                                                "edge_rf_roll", "edge_rb_roll", "level_end"]
    ticks = SC.poses(script=s1)
    assert len(ticks) == 2431 and ticks[-1].x == (0, 0, 0, 0)


# The model the routine's sweeps were placed on (2026-11-21): before the corrections the recordings
# predict best with (docs/model-fit-results.md), which move the crossings by 1 to 3 mm.
DESIGN_MODEL = replace(M, com_dx_mm=0.0, com_dy_mm=0.0, com_dz_mm=0.0, track_mm=65.0)


def test_sweeps_start_on_the_tripod_side_of_the_models_crossing():
    cr = SM.crossings(DESIGN_MODEL)
    now = SM.crossings()
    assert len(cr) == 8
    for i, st in enumerate(SC.STEPS):
        if st.kind != SC.SWEEP:
            continue
        ex = SC.EXPERIMENTS[st.exp]
        name = ex.name
        x0 = SC.STEPS[i - 4].dx[0]  # to_start's
        x1 = st.dx[0]
        d = 1 if x1 > x0 else -1
        c = cr[name]
        # the start and the end (no tip ending it early) on either side of the model's crossing, with room
        span = SC.ROLL_SPAN_MM if ex.roll_mm else SC.SWEEP_SPAN_MM
        before, after = span, span
        if name == "edge_rb":  # the RB shoulder's clamp: it starts further back and ends at RB_MAX_X
            before, after = span + SC.RB_BACK_MM, SC.RB_MAX_X - c
        assert SC.STEPS[i - 4].phase == "to_start"
        assert abs((c - x0) * d - before) <= 1.0, (name, x0, c)
        assert abs((x1 - c) * d - after) <= 1.0 and after >= 5, (name, x1, c)
        # the model as it is now still crosses inside the sweep, with room on both sides
        assert (now[name] - x0) * d >= before - 4 and (x1 - now[name]) * d >= min(after, 5) - 1, (name, now[name])


@pytest.mark.parametrize("version", [1, 2])
def test_tip_vectors_are_the_models(version):
    sd = SC.SCRIPTS[version]
    ticks = SC.poses(script=sd)
    rp = ST.rigid_path(DESIGN_MODEL, ticks)  # the routine's tables were made on it
    for e, ex in enumerate(sd.experiments):
        if ex.lifted < 0:
            continue
        ks = [k for k, t in enumerate(ticks) if sd.steps[t.step].exp == e and sd.steps[t.step].phase == "sweep"]
        d = np.array(rp[ks[-1]][:2]) - np.array(rp[ks[0]][:2])
        assert np.allclose(np.round(d * 16), ex.tip_a16, atol=1), (ex.name, d * 16)
        assert rp[ks[0]][2] == 0xF & ~(1 << ex.lifted)  # on the tripod at the start: the lifted foot in the air


def test_runner_tip_ends_the_sweep():
    no = SC.poses()
    lf, rf = E("edge_lf"), E("edge_rf")
    start = _phase_start(no, lf, "sweep")
    tipped = SC.poses({lf: start + 30})
    assert len(tipped) < len(no)
    sweep = [t for t in tipped if SC.STEPS[t.step].exp == lf and SC.STEPS[t.step].phase == "sweep"]
    x_seen = sweep[30].x[0]
    assert sweep[-1].x[0] == x_seen - SC.TIP_EXTRA_MM  # the front feet's sweep moves x down
    lifted = _phase_start(tipped, rf, "lifted")
    cut = SC.poses({lf: start + 30, rf: lifted + 3})
    sweep3 = [t for t in cut if SC.STEPS[t.step].exp == rf and SC.STEPS[t.step].phase == "sweep"]
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


# ---- the tip rule ---------------------------------------------------------------------------------
def _watch(tilt) -> tuple:
    """Play edge_lf's settle, lift, lifted and sweep through a TipWatch with the tilt toward the
    lifted foot ``tilt(phase, into)`` deg (along the model's tip): what it said, and at which sweep tick."""
    e = E("edge_lf")
    u = np.array(SC.EXPERIMENTS[e].tip_a16, float)
    u /= np.hypot(*u)
    w = SC.TipWatch()
    tick = 0
    for i, st in enumerate(SC.STEPS):
        if st.exp != e or st.phase not in ("settle", "lift", "lifted", "sweep"):
            continue
        for into in range(st.ticks):
            deg = tilt(st.phase, into)
            got = w.see(tick, i, into, 2.4 + deg * u[0], 0.5 + deg * u[1])  # an IMU offset under it all
            tick += 1
            if got is not None:
                return got, st.phase, into
    return None, None, None


def test_a_tripods_lean_is_not_a_tip():
    """Kilo's first run (2026-11-21): LF up leaned the tripod 1.8 deg at once, and it crept on along the
    sweep. Script 1's rule (40 % of the tip from the settle hold's tilt) called that a tip."""
    def lean(phase, into):
        return {"settle": 0.0, "lift": 1.8 * into / 10}.get(phase, 1.8) + (0.5 * into / 240 if phase == "sweep" else 0)

    assert _watch(lean) == (None, None, None)
    # even a creep past the rise is not one: a tip is a jump
    assert _watch(lambda p, i: lean(p, i) + (2.0 * i / 240 if p == "sweep" else 0)) == (None, None, None)


def test_a_tip_is_seen_and_a_foot_that_came_up_tipped():
    def tip(phase, into):  # the same lean, then the rest of the tip in a few ticks (the servos' pace)
        base = {"settle": 0.0, "lift": 1.8 * into / 10}.get(phase, 1.8)
        if phase == "sweep" and into >= 100:
            base += 3.0 * (1 - math.exp(-(into - 100) / 4.0))
        return base

    got, phase, into = _watch(tip)
    assert got == "tip" and phase == "sweep" and 101 <= into <= 106
    got, phase, into = _watch(lambda p, i: {"settle": 0.0, "lift": 4.0 * i / 10}.get(p, 4.0))
    assert got == "up" and phase == "lifted" and into == 0


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
        R_ = PM.rotation(a.pitch[i], a.roll[i])
        d = np.array(R_) @ np.array([body.com_dx, body.com_dy, body.com_dz])
        Fz = a.loads[i]
        cx = sum(Fz[k] * feet[k][0] for k in range(4)) / Fz.sum()
        cy = sum(Fz[k] * feet[k][1] for k in range(4)) / Fz.sum()
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


def test_the_feet_rectangle_sets_the_tilt_per_mm():
    """A longer or wider rectangle of feet tilts less for the same leg lengths; the model's own
    rectangle is the default."""
    P = ST.Poses(M, [[0] * 4, [0] * 4], [[70, 70, 82, 82], [72, 80, 80, 72]])  # the top pitch and roll stairs
    a = ST.rest(P, ST.Body(sag=ST.SAG_MIN))
    b = ST.rest(P, ST.Body(sag=ST.SAG_MIN, length=M.length_mm, track=M.track_mm))
    assert np.allclose(a.pitch, b.pitch) and np.allclose(a.roll, b.roll)
    c = ST.rest(P, ST.Body(sag=ST.SAG_MIN, length=133.0, track=70.0))
    assert a.pitch[0] == pytest.approx(math.degrees(math.atan(12 / M.length_mm)), abs=0.05)
    assert c.pitch[0] == pytest.approx(math.degrees(math.atan(12 / 133)), abs=0.05)
    assert a.roll[1] == pytest.approx(math.degrees(math.atan(8 / M.track_mm)), abs=0.05)
    assert c.roll[1] == pytest.approx(math.degrees(math.atan(8 / 70)), abs=0.05)
    # the x scale: feet splayed 16 mm each way make the rectangle 32 mm longer, or 35.2 at 1.1
    P = ST.Poses(M, [[16, 16, -16, -16]], [[70, 70, 82, 82]])
    d = ST.rest(P, ST.Body(sag=ST.SAG_MIN, x_scale=1.1))
    assert d.pitch[0] == pytest.approx(math.degrees(math.atan(12 / (123 + 35.2))), abs=0.05)


# ---- the fit ----------------------------------------------------------------------------------------
def _truth(**kw):
    v = dict(F.model_values())
    v.update(kw)
    v["tip_ms"] = SM.model_tip_ms(v)
    return v


def test_fit_recovers_perturbed_parameters():
    mv = F.model_values()
    truth = _truth(imu_pitch=1.2, imu_roll=-2.0, com_x=mv["com_x"] - 4, com_y=-3.0, com_z=mv["com_z"] + 6, sag=3.0,
                   twist=0.8, lag_ms=60, tau_ms=80, length=133.0, track=69.0)
    run = SM.synth_run(truth, noise=SM.IMU_NOISE_DEG, seed=5)
    res = F.Fitter([run]).run()
    tol = {"imu_pitch": 0.05, "imu_roll": 0.05, "com_x": 1.0, "com_y": 0.5, "com_z": 3.0, "sag": 0.2, "twist": 0.05,
           "lag_ms": 2.0, "tau_ms": 2.0, "length": 1.0, "track": 0.5}
    for k, t in tol.items():
        assert abs(res.values[k] - truth[k]) < t, (k, res.values[k], truth[k])
        assert abs(res.values[k] - truth[k]) < 4 * res.sd[k] + 1e-3, (k, res.values[k], truth[k], res.sd[k])
    for k in ("com_x", "sag", "imu_roll", "length", "track", "twist"):
        assert res.significant(k), k
    assert not res.at_limit and not res.unmeasured
    # the x scale is not fitted (it would take the height with it): the splay alone checks it
    assert "x_scale" not in res.names and abs(res.x_scale_check["value"] - 1.0) < 0.03
    assert res.verdict("x_scale").startswith("matches: the fit keeps 1.00; the splay alone says")
    assert sum(c["measured_x"] is not None for c in res.crossings) >= 6
    assert all(c["lean_deg"] is not None for c in res.crossings)
    # the held-out poses: the fitted model predicts them, today's model (another rectangle, no twist) does not
    ck = res.check
    assert len(ck["poses"]) >= len(SC.CHECK_POSES) and ck["rms_fitted_deg"] < 0.1 < ck["rms_model_deg"]
    # no backlash in this run: he comes back to home as he left it
    assert res.repeatability["holds"] >= 15 and res.repeatability["sd_roll"] < 0.08
    assert abs(res.hysteresis["pitch"]) < 0.08 and res.hysteresis["pitch_poses"] >= 5
    js = F.to_json(res, ["synthetic"])
    assert set(F.ROWS) <= set(js["parameters"]) and js["parameters"]["sag"]["significant"]
    assert js["parameters"]["length"]["where"] == "LEG_LB_MM" and js["parameters"]["length"]["verdict"].startswith("differs")
    text = F.format_table(res)
    assert "LEG_W_MM" in text and "held-out check" in text and "repeatability" in text


def test_fit_with_backlash_says_how_sure_it_is():
    """Every hold a random 0.3 deg off (what Kilo's first run showed): the fit still finds the
    rectangle, the twist and the height within what it says, and reports the spread it saw."""
    mv = F.model_values()
    truth = _truth(imu_pitch=2.4, imu_roll=0.5, com_x=mv["com_x"] - 3, com_y=-1.0, com_z=mv["com_z"] + 6, sag=2.0,
                   twist=0.7, lag_ms=60, tau_ms=80, length=133.0, track=69.0)
    run = SM.synth_run(truth, backlash=0.3, seed=11)
    res = F.Fitter([run]).run()
    for k in ("length", "track", "twist", "com_x", "com_y", "com_z", "sag"):
        assert abs(res.values[k] - truth[k]) < 3 * res.sd[k], (k, res.values[k], truth[k], res.sd[k])
    assert abs(res.values["length"] - 133) < 6 and abs(res.values["track"] - 69) < 3
    assert abs(res.values["twist"] - 0.7) < 0.3 and res.significant("twist")
    assert 0.15 < res.repeatability["sd_pitch"] < 0.5 and res.sigma_hold_deg >= 0.2
    assert res.sd["imu_pitch"] > 0.02  # not the 0.00 a tick-by-tick spread would claim


def test_the_splay_checks_the_x_scale_on_its_own():
    """Feet that travel 15 % further than commanded: the fit (which keeps the x scale 1) says so from the
    splay poses alone; asked to, it fits it."""
    run = SM.synth_run(_truth(sag=2.0, x_scale=1.15), seed=9)
    res = F.Fitter([run]).run()
    assert "x_scale" not in res.names and res.values["x_scale"] == 1.0
    assert abs(res.x_scale_check["value"] - 1.15) < 0.04 and res.x_scale_check["holds"] >= 4
    assert res.verdict("x_scale").startswith("differs: the fit keeps 1.00")
    free = F.Fitter([run], free_x_scale=True).run()
    assert "x_scale" in free.names and abs(free.values["x_scale"] - 1.15) < 0.03 and free.x_scale_check == {}


def test_limits_and_unmeasured_are_said():
    run = SM.synth_run(_truth(sag=2.0), seed=2)
    f = F.Fitter([run])
    th = f.start()
    assert f.limits(th) == {}
    th[f.names.index("com_z")] = -40.0   # where Kilo's first run ended: the end of the range
    th[f.names.index("sag")] = ST.SAG_MIN  # rigid legs are a value, not a limit
    assert f.limits(th) == {"com_z": True}
    # within one +- of the stop is at the stop too (Kilo's runs of 2026-11-22: -39.0 +- 8.1 against -40)
    th[f.names.index("com_z")] = -39.0
    assert f.limits(th) == {} and f.limits(th, {"com_z": 8.1}) == {"com_z": True}
    assert f.limits(th, {"com_z": 0.5}) == {}
    th[f.names.index("com_z")] = -40.0
    res = F.FitResult(values=f.values(th), sd={n: 0.1 for n in f.names}, model=dict(f.mv), names=list(f.names),
                      cov=np.eye(len(f.names)), rms_tick_deg=0.1, rms_model_deg=None, inflation=1.0, n_ticks=1,
                      crossings=[], tip_rate={}, walks=[], notes=[], iterations=1, at_limit=f.limits(th))
    assert not res.significant("com_z") and res.verdict("com_z").startswith("not measured")
    js = F.to_json(res, [])
    assert js["parameters"]["com_z"]["at_limit"] and not js["parameters"]["com_z"]["significant"]
    assert "at limit" in F.format_table(res)
    # no rolled sweep with its flat partner: the height is left at the model's and said to be unmeasured
    f = F.Fitter([run])
    f.prepare(f.start())
    for sw in run.sweeps:
        if run.script.experiments[sw.exp].roll_mm:
            sw.tip_tick = None
    assert f.height_pairs() == 0


def test_fit_on_the_virtual_bodys_physics():
    """An independent model (kilo.vbody.physics: rigid, a real inverted-pendulum tip): the centre of
    mass and the IMU offsets come back, and the rectangle is the model's."""
    # the centre of mass where it was against the routine's design (4 mm behind the hand measurement)
    run = SM.physics_run((-4.0 - M.com_dx_mm, -3.0, 0.0), (1.5, -2.0), seed=3)
    assert [t["how"] for t in run.meta["tips"]] == ["tip"] * 8
    res = F.Fitter([run]).run()
    t = run.meta["truth"]
    assert abs(res.values["imu_pitch"] - 1.5) < 0.1 and abs(res.values["imu_roll"] + 2.0) < 0.1
    assert abs(res.values["com_x"] - t["com_x"]) < 1.0 and abs(res.values["com_y"] - t["com_y"]) < 0.5
    assert abs(res.values["com_z"] - t["com_z"]) < 3 * res.sd["com_z"] + 1.0
    assert abs(res.values["length"] - M.length_mm) < 2.0 and abs(res.values["track"] - M.track_mm) < 1.0
    assert res.values["sag"] < 0.3
    assert 40 < res.values["lag_ms"] < 65 and 55 < res.values["tau_ms"] < 90
    assert res.tip_rate["events"] == 8
    assert sum(c["measured_x"] is not None for c in res.crossings) == 8


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


def test_a_script_1_run_still_loads_and_fits(tmp_path):
    """Script 1's recordings (Kilo's first run, 2026-11-21) stay readable: the run says which script it
    played, the fit takes its steps and experiments from that one, and uses the lean of a tripod whose
    sweep never tipped."""
    mv = F.model_values()
    truth = _truth(imu_pitch=2.4, imu_roll=0.5, com_x=mv["com_x"] - 3, sag=3.0, twist=0.6, length=133.0, track=69.0)
    run = SM.synth_run(truth, seed=7, script=SC.SCRIPTS[1])
    p = R.save_run(run, tmp_path / "v1.json")
    back = R.load(p)[0]
    assert back.script.version == 1 and len(back) == len(run) <= 2431
    assert {back.exp_name(int(e)) for e in back.exp} >= {"steps", "edge_rb_roll"}
    with pytest.raises(R.RecordError):
        R.script_of({"script": "9"})
    assert R.script_of({}).version == 1  # a header from before the field: script 1
    res = F.Fitter([back]).run()
    assert "x_scale" not in res.names and res.verdict("x_scale").startswith("fixed")  # nothing in it splays the feet
    assert abs(res.values["twist"] - 0.6) < 0.1 and abs(res.values["length"] - 133) < 3
    assert abs(res.values["track"] - 69) < 2
    assert res.check == {} and "held-out check: none" in F.format_table(res)
    assert "roll" in res.hysteresis and res.repeatability["holds"] >= 10


# ---- the CLI and the files -------------------------------------------------------------------------
def test_cli_simulate_write_and_fit(tmp_path, capsys):
    from kilo import cli

    p = tmp_path / "sim.json"
    assert cli.main(["measure", "simulate", "--write", str(p), "--seed", "3"]) == 0
    runs = R.load(p)
    assert len(runs) == 1 and len(runs[0]) > 3000 and "truth" in runs[0].meta and runs[0].script.version == 2
    out = tmp_path / "fit.json"
    assert cli.main(["measure", "fit", str(p), "--out", str(out)]) == 0
    text = capsys.readouterr().out
    assert "com_x" in text and "significant" in text and "Nothing here changes the walks' model" in text
    assert "against the model (protocol/spec.toml)" in text and "LEG_LB_MM" in text
    d = json.loads(out.read_text())
    truth = runs[0].meta["truth"]
    assert d["format"] == 2 and abs(d["parameters"]["com_y"]["value"] - truth["com_y"]) < 1.0
    assert d["parameters"]["com_x"]["model"] == pytest.approx(F.model_values()["com_x"], abs=1e-3)
    assert {"repeatability", "hysteresis", "check"} <= set(d) and "verdict" in d["parameters"]["track"]


def test_cli_plan(capsys):
    from kilo import cli

    assert cli.main(["measure", "plan"]) == 0
    text = capsys.readouterr().out
    for name in ("level", "stairs_pitch", "twist", "splay", "edge_lf", "edge_lf_toward", "edge_lb_toward", "check",
                 "dropped", "tail swing", "script 2"):
        assert name in text
