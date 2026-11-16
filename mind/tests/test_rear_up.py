"""The rear_up trick (docs/rear-up-trick.md): the side-view static model
(kilo.tricks.sagittal) with the measured tail (it swings sideways round a
cone, lowest at 0) and the owner's photo of Kilo standing at 54 deg, the
design's three stages (28, 40, 54 deg) and their check over the unknowns
(kilo.tricks.rear_up), the library entries, and the tricks on the real
body core (kilo.vbody.core): timing, joint limits and slew, the fall
detector left armed, and the slow stop the remote relies on."""

from __future__ import annotations

import math

import pytest

from kilo.planner.model import BodyModel
from kilo.tricks import library, rear_up as RU, sagittal as S
from kilo.tricks.model import JOINT_LIMITS, MAX_FRAMES, Posture

from test_corebody import binary, rig, stand, events  # noqa: F401  (fixtures)

G = S.Geometry()


# ---- the side-view model ---------------------------------------------------------------------

def test_model_at_stand_matches_the_planner():
    r = S.rest(G, RU.STAND)
    assert r.phi == pytest.approx(0.0, abs=0.01) and set(r.contacts) == {S.FRONT_FOOT, S.HIND_FOOT}
    # the centre of mass is the planner's (54 mm behind the front axles, 3 mm below), whatever the tail split
    b = BodyModel()
    want = b.com([2.5, 2.5, -2.5, -2.5], [82.3] * 4)
    cx, cz = S.com(G, RU.STAND)
    assert cx == pytest.approx(want[0], abs=0.6) and cz == pytest.approx(want[2], abs=0.6)
    assert S.com(G.with_(tail_g=12), RU.STAND) == pytest.approx((cx, cz), abs=1e-6)
    # four feet share the weight as a beam: the shoulders carry next to nothing standing
    L = S.load(G, RU.STAND, r)
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
    r = S.rest(g, RU.STAND)
    assert r.rolls and r.rolls[0][1] == "backward" and (S.BUTT in r.contacts or S.BACK in r.contacts)
    # SIT rests nose up on the hind and the front feet: its tail (-60) is turned aside and up, off the floor
    r = S.rest(G, RU.SIT, 28)
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
        p = RU.solve(spec)
        r = S.rest(G, p, spec.phi)
        assert S.TAIL in r.contacts and S.load(G, p, r).tail == pytest.approx(0.0, abs=1e-9)
    p = RU.solve(RU.UP_SPECS[RU.stage(1).top])
    side = S.Pose(*(p.astuple()[:4]), 20, p.head_pan, p.head_tilt)
    assert 0.05 < abs(S._tail_torque(G, side, 36.0, 28.0)) / S.G_MM_PER_KG_CM < 0.15


def test_feet_rest_under_their_centre():
    """A round foot supports at the point under its centre: the margins are
    measured there, not at a corner of the polygon drawn for the hull."""
    p = RU.solve(RU.UP_SPECS[RU.stage(3).top])
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
    # the gate checks only what the photo allows; it rules out the corner that limited the first design
    kept, excluded = RU.stage_geometries(1)
    assert excluded and all(g.startswith("com_dx=10,com_dz=-10") for g, _ in excluded)
    assert ("nominal", G) in kept and len(kept) + len(excluded) == len(RU.geometries())


def test_trick_frames_are_not_held_to_the_feet_reach_cap():
    """The FEET IK keeps a leg within 0.95 of its length (core/ik.h); a
    trick plays joint angles with only the joint limits and the slew, so
    the design lets the hind legs reach further (the knee bent at least
    MIN_KNEE_BEND), as they do in the photo (0.96)."""
    top = RU.UP_SPECS[RU.stage(3).top]
    p = RU.solve(top)
    _, _, (fx, fz) = S.leg_points(G, False, p.back_shoulder, p.back_knee)
    d = math.hypot(fx + G.body.length_mm / 2, fz)
    assert G.body.reach_mm < d <= 0.985 * (G.body.l1_mm + G.body.l2_mm)
    assert 24 <= abs(-p.back_knee - 90) <= 30  # the knee bend
    _, _, (fx, fz) = S.leg_points(G, False, S.PHOTO_POSE.back_shoulder, S.PHOTO_POSE.back_knee)
    assert math.hypot(fx + G.body.length_mm / 2, fz) > G.body.reach_mm
    with pytest.raises(RU.DesignError, match="out of reach"):
        RU.solve(RU.Spec("x", phi=54, h=None, hind_dx=56))


def test_leg_gap():
    assert S.leg_gap(G, RU.STAND) > 70
    # the hind foot swung forward onto the front leg folded back: they cross
    assert S.leg_gap(G, S.Pose(80, 50, -80, -45, 0)) == 0.0


# ---- the design ----------------------------------------------------------------------------------

@pytest.mark.parametrize("st", RU.STAGES, ids=lambda s: s.name)
def test_trick_is_built_and_up_to_date(st):
    t = RU.build(None, st)
    assert library.path_for(st.name).read_text() == t.to_json(), "stale: run python -m kilo.tricks.rear_up"
    assert library.load(st.name) == t and st.name in library.names() and library.legacy_name(st.name) == ""
    assert (t.end_posture, t.stop_ticks, t.balance, t.fall_ok) == (Posture.SIT, st.stop_ticks, False, False)
    assert not any(f.balance for f in t.frames) and t.joint_mask == 0xFF07 and len(t.frames) <= MAX_FRAMES
    # it ends exactly in SIT (no jump into the end posture), every joint inside its limits
    assert RU.pose_of(t.frames[-1]) == RU.SIT
    # the tail stays at its centre (its lowest point, a strut) until the last move to SIT
    assert all(RU.pose_of(f).tail == 0 for f in t.frames[:-1])
    for f in t.frames:
        for j, v in enumerate(f.angles):
            if v is not None:
                lo, hi = JOINT_LIMITS[j]
                assert lo <= v <= hi and float(v).is_integer()
    # the way down is the way up, reversed, and one frame holds (the top)
    poses = [RU.pose_of(f) for f in t.frames]
    top = max(range(len(t.frames)), key=lambda i: t.frames[i].hold)
    assert poses[:top] == poses[top + 1:-1][::-1] and t.frames[top].hold == RU.HOLD and top == st.top
    assert sum(f.hold > 0 for f in t.frames) == 1
    # slow: cosine moves of at most 40 deg/s on average, 20 on the tripod above 28 deg
    low = RU.stage(1).top  # the moves above stage 1's top, up and down, are the slow ones
    prev = RU.STAND
    for i, (f, p) in enumerate(zip(t.frames, poses)):
        d = max(abs(a - b) for a, b in zip(prev.astuple(), p.astuple()))
        slow = low < i <= st.top or st.top < i <= 2 * st.top - low
        assert d / (f.dur * 0.02) <= (RU.RISE_DPS if slow else RU.MOVE_DPS) + 1e-6, i
        prev = p
    assert {1: (10, 15), 2: (13, 17), 3: (16, 20)}[st.number][0] < t.duration_ticks * 0.02 \
        < {1: (10, 15), 2: (13, 17), 3: (16, 20)}[st.number][1]


def test_each_stage_goes_the_same_way_up_as_the_one_below():
    """A trial of stage 1 tries the start of stages 2 and 3 exactly: each
    stage plays the stage below's way up unchanged, then goes on."""
    ups = [RU.up_poses(None, st) for st in RU.STAGES]
    for lower, higher in zip(ups, ups[1:]):
        assert higher[:len(lower)] == lower and len(higher) > len(lower)
    # stage 1 is the first design, keyframe for keyframe
    assert [RU.pose_of(f) for f in RU.build(None, 1).frames][:6] == ups[0]
    pitch = [RU.UP_SPECS[st.top].phi for st in RU.STAGES]
    assert pitch == [28, 40, 54] and RU.UP_SPECS[RU.stage(3).top].phi == S.PHOTO_PITCH


def test_solve_reproduces_its_spec():
    """A solved keyframe stands where its Spec says: the pitch, and the hind
    feet and (h=None) the tail on the floor, or the hind feet up as asked."""
    for spec in RU.UP_SPECS:
        p = RU.solve(spec)
        r = S.rest(G, p, spec.phi)
        assert r.phi == pytest.approx(spec.phi, abs=1.0), spec.name
        assert r.clear[S.HIND_FOOT] == pytest.approx(spec.hind_lift, abs=1.5), spec.name
        if spec.h is None:
            assert r.clear[S.TAIL] < 1.0, spec.name
        else:
            assert r.clear[S.TAIL] > 10, spec.name
    with pytest.raises(RU.DesignError, match="into the floor"):
        RU.solve(RU.Spec("x", phi=40, h=40, hind_dx=60))  # the tail cannot be turned down out of the way
    with pytest.raises(RU.DesignError, match="out of reach"):
        RU.solve(RU.Spec("x", phi=10, h=120, hind_dx=0))


def test_the_tail_tip_slides_forward_on_the_way_up():
    """On the tripod the hind feet come back towards the tail tip as the
    pitch grows (the centre of mass comes back over the hips): the span
    from the tail tip to the hind feet shrinks by ~60 mm from 24 to 54 deg.
    The tail tip carries the least weight, so it is what slides."""
    spans = []
    for spec in RU.UP_SPECS[4:]:
        p = RU.solve(spec)
        r = S.rest(G, p, spec.phi)
        spans.append(r.contacts[S.HIND_FOOT] - r.contacts[S.TAIL])
        L = S.load(G, p, r)
        assert L.on_rear < 0.25 * G.body.total_g  # the tail's share (the rear support)
    assert all(a >= b for a, b in zip(spans, spans[1:]))
    assert 50 < spans[0] - spans[-1] < 70


@pytest.mark.parametrize("st", RU.STAGES, ids=lambda s: s.name)
def test_a_forward_tip_at_the_hold_is_caught_by_the_paws(st):
    """If Kilo goes forward over its hind feet at the top (a push, a wobble),
    the front paws land and it stops on four feet, not on its nose."""
    p = RU.solve(RU.UP_SPECS[st.top])
    held, tipped = RU.catch(G, p, RU.UP_SPECS[st.top].phi)
    assert {held.rear, held.front} == {S.TAIL, S.HIND_FOOT}
    assert set(tipped.contacts) == {S.HIND_FOOT, S.FRONT_FOOT} and -20 < tipped.phi < 0
    assert held.clear[S.FRONT_FOOT] == pytest.approx({1: 25, 2: 66, 3: 109}[st.number], abs=3)


@pytest.mark.parametrize("st", RU.STAGES, ids=lambda s: s.name)
def test_design_check_over_the_ranges(st):
    """The gate: over RANGES that the photo allows (the centre of mass
    +-10 mm both ways but not forward-and-low, the tail within +-3 mm and
    +-5 deg of the measurements, the outline +-4 mm), playing from STAND or
    SIT and stopped anywhere, Kilo never touches the floor with its back,
    the Pi or its nose, never rolls back more than a 15 mm drop, never
    pitches past 70 deg (the fall detector stays armed), always ends down
    on its feet (a stop to STAND could leave it propped on the tail: hence
    SIT), and no servo holds more than half its estimated stall torque. It
    holds the tripod with at least the stage's margin everywhere."""
    t = RU.build(None, st)
    rep = RU.check(t)
    assert rep.problems == []
    want_pitch = {1: (26, 31), 2: (38, 42), 3: (52, 56)}[st.number]
    nom = [r for g, m, r in rep.results if g == "nominal" and m == "play from STAND"][0]
    assert nom.hold_tripod and want_pitch[0] < nom.hold_pitch < want_pitch[1]
    assert nom.hold_margin > {1: 15, 2: 13, 3: 10}[st.number] and nom.lateral > 20
    plays = [r for g, m, r in rep.results if m.startswith("play")]
    assert plays and all(r.hold_tripod and r.hold_margin >= st.margin_min for r in plays)
    assert all(r.end_supports and S.TAIL not in r.end_supports for g, m, r in rep.results)
    assert rep.worst("lateral", "play", min)[0] > 20
    assert rep.worst("pitch_max", "", max)[0] < 60
    assert max(v for v, _, _ in rep.torque_max().values()) < RU.TORQUE_MAX_KG_CM


@pytest.mark.parametrize("st", RU.STAGES, ids=lambda s: s.name)
def test_torque_at_the_hold(st):
    """The servos at the top, nominal: the higher the hold, the less the
    hind legs lean, the less they hold (the hind knee, which the photo's
    pen had to brace while limp, holds least of all)."""
    spec = RU.UP_SPECS[st.top]
    p = RU.solve(spec)
    L = S.load(G, p, S.rest(G, p, spec.phi))
    assert L.hind_knee < 0.35 * RU.STALL_KG_CM and L.hind_shoulder < 0.45 * RU.STALL_KG_CM
    assert L.tail == pytest.approx(0.0, abs=1e-9)


def test_a_stop_to_stand_would_leave_kilo_on_its_tail():
    """Why the end posture is SIT: the straight joint-space line from the
    tripod to STAND straightens the hind legs with the tail on the floor and
    pitches Kilo further up; with the centre of mass 10 mm back it stays
    there, propped on the tail at about 40 deg."""
    t = RU.build()
    poses = [RU.pose_of(f) for f in t.frames]
    top = RU.stage(1).top
    g = G.with_(com_dx=-10)
    to_stand = S.follow(g, poses[:top + 1] + [RU.STAND], start=RU.STAND)[-1].rest
    assert S.TAIL in to_stand.contacts and to_stand.phi > 35
    to_sit = S.follow(g, poses[:top + 1] + [RU.SIT], start=RU.STAND)[-1].rest
    assert set(to_sit.contacts) >= {S.HIND_FOOT, S.FRONT_FOOT} and S.TAIL not in to_sit.contacts


@pytest.mark.parametrize("st", [RU.STAGES[0], RU.STAGES[2]], ids=lambda s: s.name)
def test_back_is_never_the_support_over_the_plausible_ranges(st):
    """If the reading of the tail measurements is off (PLAUSIBLE, wider):
    what can go wrong is a rear sitting down harder or the paws staying
    down; the back and the Pi still never touch the floor."""
    t = RU.build(None, st)
    geos, _ = RU.stage_geometries(st, RU.PLAUSIBLE, ("com_dx", "tail_len", "tail_droop_deg"))
    rep = RU.check(t, geos)
    for g, m, r in rep.results:
        if m.startswith("rest"):
            continue
        assert not (r.forbidden & {S.BACK, S.PI}), (g, m, r.forbidden)
        assert r.pitch_max < RU.PITCH_MAX, (g, m)


# ---- on the real body core ---------------------------------------------------------------------

def pose_of_out(out: list[int]) -> S.Pose:
    d = [v / 16 for v in out]  # canonical: pan tilt tail, shoulders LF RF RB LB, knees LF RF RB LB
    return S.Pose(d[3], d[7], d[5], d[9], d[2], head_pan=d[0], head_tilt=d[1])


def play_with_model_attitude(core, client, until, alive=True, max_ticks=1500):
    """Tick the core, feeding it the pitch the side-view model gives for its
    joint outputs (TELEMETRY: nose down +, so -phi). Returns the pitches fed."""
    phi, fed = 0.0, []
    for _ in range(max_ticks):
        d = core.dump()
        if OFF in d["out"]:
            break
        r = S.rest(G, pose_of_out(d["out"]), phi)
        phi = r.phi
        core.set_attitude(pitch_deg=-phi)
        fed.append(phi)
        core.step(1, alive=alive)
        if until():
            break
    return fed


OFF = -32768


TOP_PITCH = {"rear_up": (25, 32), "rear_up_2": (37, 43), "rear_up_3": (51, 57)}


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
    assert pose_of_out(d["out"]) == RU.SIT
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
            want = [round(v * 16) for v in RU.keyframe(RU.pose_of(t.frames[i]), 1).to_wire().ang]
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
    # the -80 limit, so they arrive early and wait (rear_up.stop_path models it)
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
