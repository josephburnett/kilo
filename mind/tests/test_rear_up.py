"""The rear_up trick (docs/rear-up-trick.md): the side-view static model
(kilo.tricks.sagittal) with the measured tail and the owner's photo; the
3-D static model with the servos' real reach (kilo.tricks.stance) held to
the two falls of 2026-11-17 (fixtures/rear_up_2026_11_17.json); the
compliant dynamic model (kilo.tricks.dynamics) held to the plays of
2026-11-19..21 (fixtures/rear_up_2026_11_21.json); the third design
(kilo.tricks.rear_up), its checks in that model and its guards; the
library entries; and the mind's guard watch (kilo.tricks.model.GuardWatch).
(The plays on the real body core's trick store went with the store in
protocol 3.0.)"""

from __future__ import annotations

import json
from dataclasses import replace
import math
from pathlib import Path

import pytest

from kilo.planner.model import BodyModel
from kilo.tricks import dynamics as D, library, rear_up as RU, sagittal as S, stance as T
from kilo.tricks.model import JOINT_LIMITS, MAX_FRAMES, Guard, GuardWatch, Keyframe, Posture, Trick


G = S.Geometry.of(BodyModel().view("tricks_rigid"))  # the side view as of the trials and the photo (2026-11-17)
STAND_P = S.Pose(30, 30, -30, -30, 0)
SIT_P = S.Pose(30, 60, -80, 45, -60, head_pan=10, head_tilt=-20)


def pose(js: dict) -> S.Pose:
    """A symmetric joint dict (kilo.tricks.rear_up) as the side view's Pose."""
    assert js["lb_shoulder"] == js["rb_shoulder"] and js["lb_knee"] == js["rb_knee"]
    return S.Pose(js["lf_shoulder"], js["lf_knee"], js["lb_shoulder"], js["lb_knee"], js["tail"],
                  head_pan=js["head_pan"], head_tilt=js["head_tilt"])


# ---- the side-view model ---------------------------------------------------------------------

def test_model_at_stand_matches_the_planner():
    r = S.rest(G, STAND_P)
    assert r.phi == pytest.approx(0.0, abs=0.01) and set(r.contacts) == {S.FRONT_FOOT, S.HIND_FOOT}
    # the centre of mass is the planner's (54 mm behind the front axles, 3 mm below), whatever the tail split
    b = BodyModel().view("tricks_rigid")  # G's
    want = b.com([2.5, 2.5, -2.5, -2.5], [82.3] * 4)
    cx, cz = S.com(G, STAND_P)
    assert cx == pytest.approx(want[0], abs=0.6) and cz == pytest.approx(want[2], abs=0.6)
    assert S.com(G.with_(tail_g=12), STAND_P) == pytest.approx((cx, cz), abs=1e-6)
    # four feet share the weight as a beam: the shoulders carry next to nothing standing
    L = S.load(G, STAND_P, r)
    assert L.on_rear + L.on_front == pytest.approx(b.total_g)
    assert L.hind_shoulder < 0.1 and L.front_shoulder < 0.1 and L.hind_knee < 0.3


def test_leg_kinematics_match_the_ik_conventions():
    # STAND: every foot under its shoulder, 82 mm down (ik.h: t1 -30, t2 +60 in front, the mirror behind)
    for front, sh, kn in ((True, 30, 30), (False, -30, -30)):
        (hx, _), _, (fx, fz) = S.leg_points(G, front, sh, kn)
        assert abs(fx - hx) < 3 and fz == pytest.approx(-82.3, abs=0.2)


def test_rolls_follow_the_centre_of_mass():
    # the centre of mass pushed far behind the hind feet with no tail: the body rolls back onto its rear end
    g = G.with_(com_dx=-90, tail_len=5)
    r = S.rest(g, STAND_P)
    assert r.rolls and r.rolls[0][1] == "backward" and (S.BUTT in r.contacts or S.BACK in r.contacts)
    # SIT rests nose up on the hind and the front feet: its tail (-60) is turned aside and up, off the floor
    r = S.rest(G, SIT_P, 28)
    assert 25 < r.phi < 32 and {r.rear, r.front} == {S.HIND_FOOT, S.FRONT_FOOT} and r.clear[S.TAIL] > 15


# ---- the tail: the owner's measurements and description of 2026-11-16 (docs/rear-up-trick.md) ---

def test_tail_sweeps_a_cone_lowest_at_centre():
    c = S.tail_cone(G)
    # the axis 40 deg back from vertical, the tail 18 deg below straight back: 68 deg apart, so
    # the 94 mm tail's tip goes round a circle of 87 mm radius centred 35 mm along the axis
    assert c["half_angle"] == pytest.approx(68.0) and c["radius"] == pytest.approx(87.2, abs=0.1)
    assert c["centre_along_axis"] == pytest.approx(35.2, abs=0.1)
    # at 0: 89 mm behind and 29 mm below the pivot, so 98 mm behind and 5 mm below the hind axles
    assert c["tip0_from_pivot"] == pytest.approx((-89.4, -29.0), abs=0.1)
    assert c["tip0_from_hip"] == pytest.approx((-98.4, -5.0), abs=0.1)
    # at 90 deg: 87 mm to the side, 23 mm behind and 27 mm ABOVE the pivot
    assert c["tip90_from_pivot"] == pytest.approx((-22.6, 87.2, 27.0), abs=0.1)
    # the tip is lowest at 0, rises the same either way, and moves sideways (which side is left is not known)
    z = {t: S.tail_point(G, t)[2] for t in (-90, -60, -30, -10, 0, 10, 30, 60, 90)}
    assert min(z, key=z.get) == 0
    for t in (10, 30, 60, 90):
        assert z[t] == pytest.approx(z[-t]) and z[t] > z[0]
        assert S.tail_point(G, t)[1] == pytest.approx(-S.tail_point(G, -t)[1]) and S.tail_point(G, t)[1] > 0
    assert S.tail_point(G.with_(tail_sign=-1), 30)[1] < 0
    # the axis angle does not move the tip at 0 (only the sweep)
    assert S.tail_tip(G.with_(tail_axis_deg=40), 0) == pytest.approx(S.tail_tip(G, 0))


def test_tail_touch_height():
    # the tip meets the floor when the hind axles are 98 sin(pitch) + 5 cos(pitch) mm up
    for phi in (0, 20, 28, 45, 60):
        a = math.radians(phi)
        assert S.tail_touch_height(G, phi) == pytest.approx(98.4 * math.sin(a) + 5.0 * math.cos(a), abs=0.1)
    # turned aside, the tip is higher, so the hips must be lower for it to touch
    assert S.tail_touch_height(G, 28, 30) < S.tail_touch_height(G, 28) - 5


def test_tail_servo_holds_nothing_at_centre():
    """At 0 the tail, its axis and the floor's push all lie in the mid-plane:
    the servo holds no torque, the load goes into its bearing. Turned aside
    under load the push turns it further out (0 is the bottom of its circle)."""
    for st in RU.STAGES:
        spec = replace(RU.UP[st.top].spec, left=None, right=None)  # both sides alike for the side view
        p = pose(RU.solve(spec))
        r = S.rest(G, p, spec.phi)
        assert S.TAIL in r.contacts and S.load(G, p, r).tail == pytest.approx(0.0, abs=1e-9)
    p = pose(RU.solve(replace(RU.UP[RU.stage(1).top].spec, left=None, right=None)))
    side = S.Pose(*(p.astuple()[:4]), 20, p.head_pan, p.head_tilt)
    assert 0.05 < abs(S._tail_torque(G, side, 36.0, 28.0)) / S.G_MM_PER_KG_CM < 0.15


def test_feet_rest_under_their_centre():
    """A round foot supports at the point under its centre: the margins are
    measured there, not at a corner of the polygon drawn for the hull."""
    p = pose(RU.solve(replace(RU.UP[RU.stage(3).top].spec, left=None, right=None)))
    r = S.rest(G, p, 54)
    _, _, (fx, fz) = S.leg_points(G, False, p.back_shoulder, p.back_knee)
    assert r.contacts[S.HIND_FOOT] == pytest.approx(S.to_world(fx, fz, r.phi)[0], abs=1e-6)
    assert r.x_front == pytest.approx(r.contacts[S.HIND_FOOT])
    phi, m = S.tripod(G, p)
    assert phi == pytest.approx(r.phi, abs=0.5) and m == pytest.approx(r.margin_front, abs=0.8)


# ---- the photo of 2026-11-16: Kilo, unpowered, standing free at 54 deg ----------------------

def test_photographed_pose_stands_in_the_model():
    """The pose read off the owner's photo (sagittal.PHOTO_POSE) stands on
    the tail tip and the hind feet in the uncorrected model, at the pitch
    the photo shows, with the hip where the photo's leg angles put it: the
    model needs no correction to agree with the photo."""
    r = S.rest(G, S.PHOTO_POSE, S.PHOTO_PITCH)
    assert {r.rear, r.front} == {S.TAIL, S.HIND_FOOT} and r.phi == pytest.approx(54.0, abs=1.0)
    assert 3.0 < r.margin_front < 7.0  # 5.1 mm: thin, as a limp pose propped by hand would be
    pc = S.photo_check(G)
    assert pc.ok and pc.phi == pytest.approx(r.phi, abs=0.5) and pc.margin == pytest.approx(r.margin_front, abs=0.8)
    # the photo's hind leg: thigh 39 deg and shank 73 deg below level put the hip 82 mm up,
    # where the tail tip touches at 54 deg (98 sin + 5 cos)
    hip = 45 * math.sin(math.radians(39.4)) + 50 * math.sin(math.radians(72.8)) + 6
    assert hip == pytest.approx(S.tail_touch_height(G, 54.0), abs=1.5)


def test_photo_rules_out_forward_and_low():
    """The photo is a bound on the centre of mass: at 54 deg a shift
    com_dx forward and com_dz up moves it 0.59 com_dx - 0.81 com_dz
    forward; past the hind feet (less the reading's tolerance) the
    photographed pose could not have stood."""
    assert not S.photo_check(G.with_(com_dx=10, com_dz=-10)).ok
    assert not S.photo_check(G.with_(com_dz=-14)).ok
    for kw in ({"com_dx": 10}, {"com_dz": -10}, {"com_dx": -10, "com_dz": -10}, {"com_dx": 10, "com_dz": 10},
               {"com_dz": 10}, {"com_dx": -10}):
        assert S.photo_check(G.with_(**kw)).ok, kw
    m0 = S.photo_check(G).margin
    m1 = S.photo_check(G.with_(com_dx=5)).margin
    m2 = S.photo_check(G.with_(com_dz=5)).margin
    assert m0 - m1 == pytest.approx(5 * math.cos(math.radians(54)), abs=0.5)
    assert m2 - m0 == pytest.approx(5 * math.sin(math.radians(54)), abs=0.5)
    # the fitted centre of mass of 2026-11-21 (back, right) is one the photo allows, and so is every band
    for _, p in D.bands():
        assert S.photo_check(p.g3().g).ok


def test_leg_gap():
    assert S.leg_gap(G, STAND_P) > 70
    # the hind foot swung forward onto the front leg folded back: they cross
    assert S.leg_gap(G, S.Pose(80, 50, -80, -45, 0)) == 0.0


# ---- the two falls of 2026-11-17, and the model that explains them ---------------------------------

FIX = json.loads((Path(__file__).parent / "fixtures" / "rear_up_2026_11_17.json").read_text())
OLD = [f["joints"] for f in FIX["frames"]]  # tuck, sink, step, ready as played


def model_series(g3: T.Geometry3, until_s: float = 2.8):
    """The model's TELEMETRY (nose up, roll) and rest along the first design's
    play, timed as the body plays it (cosine easing), every 40 ms."""
    starts, acc = [], 0.0
    for f in FIX["frames"]:
        starts.append(acc)
        acc += (f["dur"] + f["hold"]) * 0.02
    prev = T.joints_of(RU.STAND)
    R = T.rest(g3, prev).R
    out = []
    t = 0.0
    while t <= until_s + 1e-9:
        k = max(i for i in range(len(starts)) if starts[i] <= t)
        s = min(1.0, (t - starts[k]) / (FIX["frames"][k]["dur"] * 0.02))
        e = (1 - math.cos(math.pi * s)) / 2
        a = RU.STAND if k == 0 else OLD[k - 1]
        r = T.rest(g3, T.lerp(a, OLD[k], e), R)
        R = r.R
        out.append((round(t, 2), T.telemetry_pitch(g3, r), r.roll, r))
        t += 0.04
    return out


def measured(t: float, run: int = 0) -> tuple[float, float]:
    rows = FIX["runs"][run]["series_since_play_s_nose_up_roll"]
    row = min(rows, key=lambda r: abs(r[0] - t))
    return row[1], row[2]


def at(series, t):
    return min(series, key=lambda x: abs(x[0] - t))


def test_the_pulse_clamp_stops_the_right_hind_shoulder_at_57_deg():
    """The root cause's first half: the body clamps each servo pulse to
    500..2500 us (joints.cpp, as legacy did). With Kilo's calibration (the
    EEPROM dump) the right hind shoulder's pulse ends at -57.3 deg, the left
    one's goes to the -80 limit; the first design asked -80 and -77. (The
    same clamp stops the left front knee at -32 deg: REST asks -45, which is
    the P1 finding B4, "the left front knee does not retract as far".)"""
    assert T.reach(10) == pytest.approx((-57.3, 100), abs=0.1)
    assert T.reach(11)[0] == -80 and T.reach(12)[0] == pytest.approx(-32.3, abs=0.1)
    assert T.reach(8)[1] == pytest.approx(62.3, abs=0.1)
    assert T.applied(10, -80) == pytest.approx(-57.3, abs=0.1) and T.applied(11, -80) == -80
    assert [f["joints"]["rb_shoulder"] for f in FIX["frames"][1:3]] == [-80, -77]
    # with the right hind shoulder re-seated (the design's calibration) both reach -80
    assert T.reach(10, RU.DESIGN_CZP)[0] == -80


def test_the_side_view_model_could_not_explain_the_falls():
    """The regression: the first design's model (the side view, the left and
    right legs as one, the angles as asked) predicted 24 deg nose up at the
    end of the sink, on the tail; both trials measured 16.4 and 16.6 with the
    tail in the air (the video), then fell over the left rear."""
    poses = [S.Pose(j["lf_shoulder"], j["lf_knee"], j["rb_shoulder"], j["rb_knee"], j["tail"]) for j in OLD]
    smp = S.follow(G, poses, start=STAND_P, steps=12)
    sink = [x.rest for x in smp if x.frame == 1 and x.s == 1.0][0]
    nose, _ = measured(1.40)
    assert S.TAIL in sink.contacts and sink.phi - 1.9 - nose > 5  # off by 6 deg: the old model fails the trial
    # and the 3-D model without the clamp fails it the same way
    no_clamp = model_series(T.Geometry3(czp=None, com_dy=-2.0), 1.40)
    assert at(no_clamp, 1.40)[1] - nose > 5


def test_the_3d_model_with_the_clamp_reproduces_both_trials():
    """The corrected model (each leg on its own, the pulse clamp, 3-D rest)
    against the recordings: STAND and SIT, the tuck and the sink within 1.5
    deg; the right hind leg, 23 deg short of the left, carries the rear: the
    left hind foot is in the air and the tail 2 cm off the floor (the video
    shows both); while the hind feet step the right one pushes the rear up
    (the pitch falls) and the centre of mass ends a few mm from the edge
    between the left front and the right hind foot, the edge Kilo went over
    (nose up and rolling left together)."""
    g3 = RU.AS_RECORDED
    assert T.telemetry_pitch(g3, T.rest(g3, T.joints_of(RU.STAND))) == pytest.approx(FIX["stand_nose_up"], abs=0.2)
    sit = T.rest(g3, T.joints_of(RU.SIT), T.rot(25, 0))
    # SIT is a near tie in the rigid model: on the three feet but the left hind (23.9 deg, as here)
    # or the three but the right front (26.4, rolled 5 deg); Kilo measured 26.9 rolled 2: in between
    assert T.telemetry_pitch(g3, sit) == pytest.approx(FIX["sit_nose_up"], abs=3.0)
    ser = model_series(g3)
    for run in (0, 1):
        for t, tol in ((0.76, 1.5), (1.40, 1.5)):  # the ends of tuck and sink
            assert at(ser, t)[1] == pytest.approx(measured(t, run)[0], abs=tol), (run, t)
        # the step: the pitch falls by more than 6 deg by 2.4 s (measured 8.7 and 8.6)
        assert at(ser, 1.44)[1] - at(ser, 2.40)[1] > 6
        assert measured(1.44, run)[0] - measured(2.40, run)[0] > 6
        # the roll stays small until the step ends, then Kilo went left (roll +)
        assert all(abs(x[2]) < 1.5 for x in ser if x[0] <= 2.6)
        assert max(measured(t, run)[1] for t in (2.9, 3.0)) > 10
    r = at(ser, 1.40)[3]
    assert set(r.contacts) == {T.FOOT["lf"], T.FOOT["rf"], T.FOOT["rb"]}
    assert r.clear[T.FOOT["lb"]] > 5 and r.clear[T.TAIL] > 10
    end = at(ser, 2.76)[3]
    assert set(end.edge) == {T.FOOT["lf"], T.FOOT["rb"]} and end.margin < 6


# ---- the compliant model, held to the plays of 2026-11-19..21 ------------------------------------------

def test_the_fast_kinematics_are_stances():
    frames = [RU.joints_of(f) for f in RU.build(None, 3).frames]
    assert D.check_kinematics(D.NOMINAL, frames) < 0.01
    assert D.check_kinematics(D.NOMINAL, frames) < 0.01


@pytest.mark.slow
def test_the_compliant_model_reproduces_the_rock_back_and_the_coin_flip():
    """Every play of the second design that reached "gather" rocked back
    onto the tail: the nose up passed +5 deg 0.55-0.59 s into the 0.6 s
    move and settled at +22.6..+23.1, roll -0.9..-2.1. The body model does
    the same at the same moment (a rigid model held it on four feet, 15 mm
    inside), settling 3 deg lower (+19.6) and level (the tricks' own fit of
    2026-11-21, its centre of mass 5 mm to the right, had the roll; the
    walks and the self-measurement put it in the middle:
    docs/model-fit-results.md). At "lift_r" 3 of 7 plays went over the
    diagonal (before the right hind shoulder was re-seated); with the
    centre of mass in the middle the model goes over in none."""
    v = D.validate()
    rec = [r for r in v["recorded"].values() if "gather_t5" in r]
    assert len(rec) == 3 and all(0.5 <= r["gather_t5"] <= 0.6 and 22.5 <= r["gather_nose_up"] <= 23.2 for r in rec)
    assert sum(r["lift_r_over"] for r in v["recorded"].values()) == 2  # b8a, t2 of the five in the fixture
    g = v["model_gather"]
    assert 0.5 <= g["gather_t5"] <= 0.62
    assert 18.5 <= g["gather_nose_up"] <= 23.5 and -2.5 <= g["gather_roll"] <= 1.0
    assert v["model_lift_r_over"] == (False, False)
    # rigid, the same gather stays on its four feet
    run = D.fixture()["runs"][2]
    assert D.replay_gather(run, D.NOMINAL.with_(sag=0.0))["gather_nose_up"] < 10


@pytest.mark.slow
def test_the_compliance_does_not_ring_on_three_points():
    """On the tripod the loads add up to the weight and the attitude does
    not depend on the time step (until 2026-11-21 the compliance's own
    motion fed the contact dampers and rang: the loads summed to 1.6 times
    the weight and the roll moved with the step)."""
    t = RU.build(None, 1)
    fr = [(RU.joints_of(f), f.dur, f.hold) for f in t.frames[:RU.top_frame(t) + 1]]
    end = sum(d + h for _, d, h in fr) * 0.02
    out = []
    for dt in (1e-3, 4e-4):
        pr = D.simulate(fr, D.NOMINAL, dt=dt, t_end=end)
        m = pr.window(end - 0.5, end)
        total = sum(v[m].mean() for v in pr.load.values())
        assert total == pytest.approx(D.Kin(D.NOMINAL).total, rel=0.03)
        assert max(v[m].std() for v in pr.load.values()) < 3.0
        out.append((pr.nose_up[m].mean(), pr.roll[m].mean()))
    assert out[0] == pytest.approx(out[1], abs=0.5)


# ---- the third design ------------------------------------------------------------------------------

@pytest.mark.parametrize("st", RU.STAGES, ids=lambda s: s.name)
def test_trick_is_built_and_up_to_date(st):
    t = RU.build(None, st)
    assert library.path_for(st.name).read_text() == t.to_json(), "stale: run python -m kilo.tricks.rear_up"
    assert library.load(st.name) == t and st.name in library.names() and library.legacy_name(st.name) == ""
    assert (t.end_posture, t.stop_ticks, t.balance, t.fall_ok) == (Posture.SIT, st.stop_ticks, False, False)
    assert not any(f.balance for f in t.frames) and t.joint_mask == 0xFF07 and len(t.frames) <= MAX_FRAMES
    assert RU.joints_of(t.frames[-1]) == RU.SIT_FRAME
    assert all(RU.joints_of(f)["tail"] == 0 for f in t.frames[:-1])
    for i, f in enumerate(t.frames):
        for j, v in enumerate(f.angles):
            if v is not None:
                lo, hi = JOINT_LIMITS[j]
                assert lo <= v <= hi and float(v).is_integer()
        if i < len(t.frames) - 1:  # every frame but SIT inside what the servos reach (DESIGN), with a margin
            assert RU.reach_problems(RU.joints_of(f)) == [], i
    # the mind's guards: on the rear, paws up, each stage's top (the remote finds the tops by name)
    names = t.source["keyframes"]
    assert [g.name for g in t.guards] == ["back", "paws", "up", "mid", "top"][:2 + st.number]
    assert [names[g.frame] for g in t.guards] == [g.name for g in t.guards]
    assert all(t.frames[g.frame].hold >= 20 for g in t.guards) and t.roll_max == RU.PLAY_ROLL_MAX
    top = RU.top_frame(t)
    assert top == st.top and t.frames[top].hold == RU.HOLD and names[-3:] == ["settle", "drop", "sit"]
    # the lower stages' tops hold STEP_HOLD: the remote's A1 goes on from there (or he comes down)
    lower = [s.top for s in RU.STAGES if s.number < st.number]
    assert [i for i, f in enumerate(t.frames) if f.hold == RU.STEP_HOLD] == lower
    assert [names[i] for i in lower] == ["up", "mid"][:len(lower)]
    assert len(t.source["predicted_pitch_deg"]) == len(t.frames)


def test_each_stage_goes_the_same_way_up_as_the_one_below():
    ups = [[f.joints for f in RU.frames(st)][:st.top + 1] for st in RU.STAGES]
    for lower, higher in zip(ups, ups[1:]):
        assert len(higher) > len(lower)
        assert higher[:len(lower) - 1] == lower[:-1]  # the lower stage's top is passed through, not held
    assert [RU.UP[st.top].spec.phi for st in RU.STAGES] == [28, 40, 54]


def test_fast_and_cat_like():
    """Paws up within about 2 s and the stage-1 hold by about 3 s (the
    second design: 12.5 s); the head comes up first; a pause before the
    rock back; one paw before the other; both hind legs alike on the tripod
    (the centre of mass is in the middle: the self-measurement of 2026-11-22)."""
    t = RU.build(None, 1)
    names = t.source["keyframes"]
    fr = {n: RU.joints_of(f) for n, f in zip(names, t.frames)}
    st = dict(zip(names, RU.frame_starts(t.frames)))
    assert names[:6] == ["look", "crouch", "back", "paw_l", "paws", "up"]
    assert st["up"] + t.frames[names.index("up")].dur * 0.02 <= 3.0
    assert st["paws"] + t.frames[names.index("paws")].dur * 0.02 <= 2.1
    assert {k for k in RU.JOINTS if fr["look"][k] != RU.STAND[k]} == {"head_tilt"} and fr["look"]["head_tilt"] > 0
    assert t.frames[names.index("crouch")].hold > 0
    assert (fr["paw_l"]["lf_knee"], fr["paw_l"]["rf_knee"]) == (fr["paws"]["lf_knee"], fr["back"]["rf_knee"])
    # the hind feet stay where STAND put them on the way to the rear (they never leave the floor: check())
    up = RU.UP[RU.stage(1).top].spec
    assert up.left is None and up.right is None
    assert (fr["up"]["lb_shoulder"], fr["up"]["lb_knee"]) == (fr["up"]["rb_shoulder"], fr["up"]["rb_knee"])


def _check(st, bands, stops):
    t = RU.build(None, st)
    return t, RU.check(t, bands=[(b, p) for b, p in D.bands(RU.MODEL) if b in bands] if bands else None, stops=stops)


@pytest.mark.slow
def test_stage_1_in_every_band_and_every_stop():
    """The gate for the first floor trial, in the compliant model: in every
    band corner (dynamics.bands: the centre of mass, the legs' give, the
    tail, the friction, slow servos, the frame's rear corner) the play from
    STAND reaches the tripod with the mind's guards on, every guard at
    least a degree inside its window, the roll within 7 deg, the hind feet
    never leaving the floor and sliding under 25 mm, back down sitting on
    its feet; cut short half way into and at the end of every keyframe
    (fitted), it comes down sitting."""
    t, rep = _check(1, None, ("fitted",))
    assert rep.problems == []
    plays = [r for r in rep.runs if r.kind == "play"]
    assert len(plays) == len(D.bands()) + 1 and not any(r.stopped for r in plays)  # and the fitted one from SIT
    for r in plays:
        assert r.paws_at is not None and r.paws_at < 2.7 and r.tripod_at is not None and r.tripod_at < 3.0, r.band
    fit = plays[0]
    pred = t.source["predicted_pitch_deg"][RU.top_frame(t)]
    assert fit.band == "fitted" and fit.hold["nose_up"] == pytest.approx(pred, abs=1.0)
    assert len([r for r in rep.runs if r.kind != "play"]) == 2 * (len(t.frames) - 1)


@pytest.mark.slow
@pytest.mark.parametrize("st", RU.STAGES[1:], ids=lambda s: s.name)
def test_higher_stages_reach_the_top_or_stop_safely(st):
    """Stages 2 and 3, in the fitted model and the corners that test them
    most: the fitted model holds the top; in a corner a guard may stop it,
    and then he comes down sitting (python -m kilo.tricks.rear_up --check
    runs every band and the cut-shorts)."""
    t, rep = _check(st, ("fitted", "softer, further right", "com_dy+3", "sag+1.5", "stiffer, further back"), None)
    assert rep.problems == []
    fit = [r for r in rep.runs if r.band == "fitted"][0]
    assert not fit.stopped and set(fit.guards) == {g.name for g in t.guards}


# ---- the mind's guard watch -------------------------------------------------------------------------

def guard_trick(**kw):
    kf = Keyframe.from_joints({"tail": 0}, dur=10, hold=30)
    return Trick("g", frames=(kf, kf, kf), guards=(Guard(1, (10.0, 14.0), 3.0, "mid"),), roll_max=8.0, **kw).validate()


def feed_all(w, samples):
    for t, nose, roll in samples:
        why = w.feed(t, nose, roll)
        if why:
            return t, why
    return None


def test_guard_watch():
    t = guard_trick()
    # the guarded hold of frame 1 starts at tick 50 and lasts 30: averaged over 65..80, decided at 78
    ok = [(i * 0.04, 12.0, 0.5) for i in range(60)]
    assert feed_all(GuardWatch(t), ok) is None
    low = [(i * 0.04, 8.0 if i * 0.04 > 1.0 else 12.0, 0.5) for i in range(60)]
    t_fire, why = feed_all(GuardWatch(t), low)
    assert why.startswith("guard mid: nose up 8.0") and 1.56 < t_fire < 1.64
    # a roll past the guard's limit inside its window stops at once; past roll_max twice anywhere
    rolly = [(i * 0.04, 12.0, 4.0 if i * 0.04 > 1.4 else 0.0) for i in range(60)]
    assert feed_all(GuardWatch(t), rolly)[1] == "guard mid: roll +4.0 deg past 3"
    tip = [(i * 0.04, 12.0, 9.0 if i >= 5 else 0.0) for i in range(60)]
    assert feed_all(GuardWatch(t), tip) == (0.24, "roll +9.0 deg past 8")
    # no TELEMETRY in the window: stopped
    gap = [(i * 0.04, 12.0, 0.0) for i in range(60) if not 1.25 < i * 0.04 < 1.6]
    assert "no TELEMETRY" in feed_all(GuardWatch(t), gap)[1]
    # at half speed the hold comes twice as late
    w = GuardWatch(t, speed=32)
    assert feed_all(w, [(i * 0.04, 12.0, 0.0) for i in range(120)]) is None
    with pytest.raises(Exception):
        Trick("g", frames=(Keyframe.from_joints({"tail": 0}, dur=10),), guards=(Guard(0, (1, 2), 3),)).validate()


def test_guard_watch_push_roll():
    """push_roll watches the guarded frame's move (frame 1: ticks 40..50,
    then up to its window at 65): the roll may move 2 deg from the last
    reading before the move, two readings in a row past that stop."""
    kf = Keyframe.from_joints({"tail": 0}, dur=10, hold=30)
    t = Trick("g", frames=(kf, kf, kf), guards=(Guard(1, (10.0, 14.0), 3.0, "mid", push_roll=2.0),),
              roll_max=8.0).validate()
    assert Trick.from_json(t.to_json()).guards == t.guards
    grows = [(i * 0.04, 12.0, 2.0 if i * 0.04 < 0.9 else 5.5) for i in range(60)]
    t_fire, why = feed_all(GuardWatch(t), grows)
    assert why == "guard mid: roll +5.5 deg, +3.5 while it moved (limit 2)" and t_fire == pytest.approx(0.96)
    # what he brings into the move is the reference: steady at 2.5, then 1.5 more, is no stop
    assert feed_all(GuardWatch(t), [(i * 0.04, 12.0, 1.0 if i * 0.04 < 0.8 else 2.5) for i in range(60)]) is None
    # one reading alone does not stop it; nor does the change after the window opens (the guard's own roll there)
    assert feed_all(GuardWatch(t), [(i * 0.04, 12.0, 4.5 if i == 25 else 0.0) for i in range(60)]) is None
    # no push_roll in the JSON when it is 0: the other tricks' files are unchanged
    assert "push_roll" not in guard_trick().to_json()


def test_guards_round_trip_through_json():
    t = RU.build(None, 2)
    assert Trick.from_json(t.to_json()).guards == t.guards and Trick.from_json(t.to_json()).roll_max == 8.0
