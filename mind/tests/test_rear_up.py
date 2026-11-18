"""The rear_up trick (docs/rear-up-trick.md): the side-view static model
(kilo.tricks.sagittal) with the measured tail and the owner's photo; the
3-D model with the servos' real reach (kilo.tricks.stance) held to the two
falls of 2026-11-17 (fixtures/rear_up_2026_11_17.json); the second design
(kilo.tricks.rear_up), its checks and its guards; the library entries; the
mind's guard watch (kilo.tricks.player.GuardWatch); and the tricks on the
real body core (kilo.vbody.core)."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from kilo.planner.model import BodyModel
from kilo.tricks import library, rear_up as RU, sagittal as S, stance as T
from kilo.tricks.model import JOINT_LIMITS, MAX_FRAMES, Guard, Keyframe, Posture, Trick
from kilo.tricks.player import GuardWatch

from test_corebody import binary, rig, stand, events  # noqa: F401  (fixtures)

G = S.Geometry()
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
    b = BodyModel()
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
        spec = RU.UP_SPECS[st.top]
        p = pose(RU.solve(spec))
        r = S.rest(G, p, spec.phi)
        assert S.TAIL in r.contacts and S.load(G, p, r).tail == pytest.approx(0.0, abs=1e-9)
    p = pose(RU.solve(RU.UP_SPECS[RU.stage(1).top]))
    side = S.Pose(*(p.astuple()[:4]), 20, p.head_pan, p.head_tilt)
    assert 0.05 < abs(S._tail_torque(G, side, 36.0, 28.0)) / S.G_MM_PER_KG_CM < 0.15


def test_feet_rest_under_their_centre():
    """A round foot supports at the point under its centre: the margins are
    measured there, not at a corner of the polygon drawn for the hull."""
    p = pose(RU.solve(RU.UP_SPECS[RU.stage(3).top]))
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
    # the gate checks only the corners the photo allows (not forward and low)
    labels = [g for g, _ in RU.geometries()]
    assert "com_dx=10,com_dz=-10" not in labels and "com_dx=10,com_dz=10" in labels


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


# ---- the second design ------------------------------------------------------------------------------

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
    # the mind's guards: the gather, the low point (the tail just down), onto the tail, paws up, each hold
    names = t.source["keyframes"]
    assert [names[g.frame] for g in t.guards][:5] == ["gather", "low", "onto", "paws", "up"]
    assert all(t.frames[g.frame].hold >= 20 for g in t.guards) and t.roll_max == RU.PLAY_ROLL_MAX
    top = max(range(len(t.frames)), key=lambda i: t.frames[i].hold)
    assert top == st.top and t.frames[top].hold == RU.HOLD and names[-1] == "sit"


def test_each_stage_goes_the_same_way_up_as_the_one_below():
    ups = [[f.joints for f in RU.frames(st)][:st.top + 1] for st in RU.STAGES]
    for lower, higher in zip(ups, ups[1:]):
        assert len(higher) > len(lower)
        assert higher[:len(lower) - 1] == lower[:-1]  # the lower stage's top is passed through, not held
    assert [RU.UP_SPECS[st.top].phi for st in RU.STAGES] == [28, 40, 54]


def test_the_hind_feet_never_swing_under_the_hips_on_the_tail():
    """Why the second design gathers the feet standing high and then sits
    back over them: from under the hip to 80 mm in front, a hind foot needs
    the hips this high to clear the floor, and on the tail at 24 deg they
    are 45 mm up. Even at the -78 deg the re-seated shoulder allows, the
    swing would scrape; at today's -54 it needs 69 mm."""
    touch = S.tail_touch_height(G, 24)
    assert RU.swing_hip_needed(24, -54) > touch + 20
    assert RU.swing_hip_needed(24, -78) > touch + 3
    # the design steps each hind foot standing 80 mm high, one at a time, the other three feet down
    t = RU.build()
    names = t.source["keyframes"]
    for name, leg, other in (("lift_l", "lb", "rb"), ("lift_r", "rb", "lb")):
        js = RU.joints_of(t.frames[names.index(name)])
        assert (js[f"{leg}_shoulder"], js[f"{leg}_knee"]) != (js[f"{other}_shoulder"], js[f"{other}_knee"])


@pytest.mark.parametrize("st", RU.STAGES, ids=lambda s: s.name)
def test_design_check(st):
    """The gate, in 3-D, over the geometries (the centre of mass 5 mm back
    to 10 mm forward and +-10 mm up or down and +-3 mm sideways, the tail
    +-3 mm and +-5 deg, a tail 5 deg off centre, one side's servos 3 deg
    off): playing from STAND, never both hind feet off the floor unless the
    tail is on it, every sample's support margin and roll, no topple
    dropping more than 8 mm, the tripod held with the stage's margin; or a
    guard stops it, and that stop and every cut-short comes down on the
    feet. On Kilo as recorded (the right hind shoulder short) a guard stops
    it at the gather, before anything leans."""
    t = RU.build(None, st)
    rep = RU.check(t)
    assert rep.problems == []
    nom = [r for g, m, r in rep.results if g == "nominal" and m == "play"][0]
    assert not nom.aborted and nom.hold_margin > {1: 10, 2: 12, 3: 9}[st.number]
    assert set(nom.hold_on) == {T.FOOT["lb"], T.FOOT["rb"], T.TAIL} and nom.margin_min > 6
    assert rep.today.aborted == "gather"
    assert nom.hold_pitch == pytest.approx(t.source["predicted_pitch_deg"][st.top], abs=0.2)


def test_the_guards_see_what_went_wrong_on_2026_11_17():
    """The first guard of the second design is at the gather; played on
    Kilo as recorded the model's roll there is past its limit: stopped."""
    t = RU.build()
    g = t.guards[0]
    r = RU.run_play(t, RU.AS_RECORDED)
    assert r.aborted == g.name and abs(r.guard_seen[g.name][1]) > g.roll_max


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


def test_guards_round_trip_through_json():
    t = RU.build(None, 2)
    assert Trick.from_json(t.to_json()).guards == t.guards and Trick.from_json(t.to_json()).roll_max == 8.0


# ---- on the real body core ---------------------------------------------------------------------

def joints_out(out: list[int]) -> dict:
    """The core's joint outputs (a16, canonical order) as joint name -> deg."""
    return {name: v / 16 for name, v in zip(("head_pan", "head_tilt", "tail", "lf_shoulder", "rf_shoulder",
                                             "rb_shoulder", "lb_shoulder", "lf_knee", "rf_knee", "rb_knee",
                                             "lb_knee"), out)}


def play_with_model_attitude(core, client, until, alive=True, max_ticks=1500):
    """Tick the core, feeding it the attitude the 3-D model (the design's
    calibration) gives for its joint outputs, as TELEMETRY (nose down +,
    right side up +). Returns the nose-up pitches fed."""
    R, fed = None, []
    for _ in range(max_ticks):
        d = core.dump()
        if OFF in d["out"]:
            break
        r = T.rest(RU.DESIGN, joints_out(d["out"]), R)
        R = r.R
        nose = T.telemetry_pitch(RU.DESIGN, r)
        core.set_attitude(pitch_deg=-nose, roll_deg=r.roll)
        fed.append(nose)
        core.step(1, alive=alive)
        if until():
            break
    return fed


OFF = -32768


TOP_PITCH = {"rear_up": (24, 28), "rear_up_2": (36, 40), "rear_up_3": (50, 54)}


@pytest.mark.parametrize("name", RU.NAMES)
def test_rear_up_on_the_body_core(rig, name):
    core, link, client = rig()
    stand(core, client)
    t = library.load(name)
    tid, n = client.start_trick(t)
    fed = play_with_model_attitude(core, client, lambda: client.trick_done_since(n, tid) is not None)
    done = client.trick_done_since(n, tid)
    assert done is not None and done.arg >> 8 == 0  # DONE: no fall, no pre-emption
    lo, hi = TOP_PITCH[name]
    assert not events(link, "FELL") and lo < max(fed) < hi
    assert abs(len(fed) - (t.duration_ticks + 1)) <= 2
    core.step(15)  # the end crossfade (only SIT's back shoulders move: -80 in the frame, -90 clamped to -80)
    d = core.dump()
    assert d["posture"] == int(Posture.SIT) and not d["trick_run"] and link.state.mode == "IDLE"
    assert joints_out(d["out"]) == RU.SIT_FRAME
    # the core reached every keyframe exactly (no limit clamps), slew-limited on the way
    core2, link2, client2 = rig()
    stand(core2, client2)
    tid, n = client2.start_trick(t)
    prev, ends, tick = None, [], 0
    for i, f in enumerate(t.frames):
        tick += f.dur + f.hold
        ends.append(tick)
    seen = {}
    k = 0
    while client2.trick_done_since(n, tid) is None and k < 1200:
        core2.step(1)
        k += 1
        out = core2.dump()["out"]
        if prev is not None:
            assert max(abs(a - b) for a, b in zip(out, prev)) <= 10 * 16  # the safety slew, 10 deg/tick
        prev = out
        seen[k] = out
    for i, e in enumerate(ends):
        if e + 1 in seen:
            want = [round(v * 16) for v in RU.keyframe(RU.joints_of(t.frames[i]), 1).to_wire().ang]
            got = seen[e + 1]
            assert max(abs(a - b) for a, b in zip(got, want)) <= 16, (i, got, want)


@pytest.mark.parametrize("name", RU.NAMES)
def test_rear_up_stop_is_a_slow_straight_crossfade_to_sit(rig, name):
    core, link, client = rig()
    stand(core, client)
    t = library.load(name)
    tid, n = client.start_trick(t)
    hold_i = max(range(len(t.frames)), key=lambda i: t.frames[i].hold)
    top = sum(f.dur for f in t.frames[:hold_i + 1]) + 20  # into the hold
    play_with_model_attitude(core, client, lambda: False, max_ticks=top)
    start = core.dump()["out"]
    client.stop_trick()  # None: the trick's own stop_ticks
    assert core.run_until(lambda: client.trick_done_since(n, tid) is not None, 1) is not None
    assert client.trick_done_since(n, tid).arg >> 8 == 1  # STOPPED
    # SIT (body/src/core/posture.cpp): its back shoulders' -90 is clamped to
    # the -80 limit, so they arrive early and wait (the model clamps every sample the same way)
    sit_raw = [v * 16 for v in (10, -20, -60, 30, 30, -90, -90, 60, 60, 45, 45)]
    ws = []
    for _ in range(t.stop_ticks + 5):
        core.step(1)
        out = core.dump()["out"]
        # every joint the same fraction of the way: a straight line in joint space (what the check assumed)
        w = [(o - s) / (e - s) for i, (o, s, e) in enumerate(zip(out, start, sit_raw)) if abs(e - s) > 200 and i not in (5, 6)]
        assert max(w) - min(w) < 0.06, w
        f = sum(w) / len(w)
        for i in (5, 6):
            assert out[i] == pytest.approx(max(-80 * 16, start[i] + f * (sit_raw[i] - start[i])), abs=24)
        ws.append(f)
    assert ws[0] < 0.1 and ws[t.stop_ticks // 2] < 0.75 and ws[-1] == pytest.approx(1.0, abs=0.01)
    assert core.dump()["posture"] == int(Posture.SIT)


@pytest.mark.parametrize("name", RU.NAMES)
def test_rear_up_leaves_the_fall_detector_armed(rig, name):
    """FALL_OK is off: a real fall backwards past 75 deg during the hold is a fall."""
    core, link, client = rig()
    stand(core, client)
    t = library.load(name)
    tid, n = client.start_trick(t)
    hold_i = max(range(len(t.frames)), key=lambda i: t.frames[i].hold)
    play_with_model_attitude(core, client, lambda: False, max_ticks=sum(f.dur for f in t.frames[:hold_i + 1]) + 10)
    assert not events(link, "FELL")
    core.set_attitude(pitch_deg=-85)  # tipping over backwards
    assert core.run_until(lambda: client.trick_done_since(n, tid) is not None, 1) is not None
    assert client.trick_done_since(n, tid).arg >> 8 == 3  # FELL
    core.step(2)
    assert [e.arg for e in events(link, "FELL")] == [-2]  # a pitch fall, nose up
