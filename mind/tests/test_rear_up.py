"""The rear_up trick (docs/rear-up-trick.md): the side-view static model
(kilo.tricks.sagittal) with the measured tail (it swings sideways round a
cone, lowest at 0), the design and its check over the unknowns
(kilo.tricks.rear_up), the library entry, and the trick on the real body
core (kilo.vbody.core): timing, joint limits and slew, the fall detector
left armed, and the slow stop the remote relies on."""

from __future__ import annotations

import math

import pytest

from kilo.planner.model import BodyModel
from kilo.tricks import library, rear_up as RU, sagittal as S
from kilo.tricks.model import JOINT_LIMITS, Posture

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


# ---- the tail: the owner's measurements of 2026-11-16, as read in docs/rear-up-trick.md ----------

def test_tail_sweeps_a_cone_lowest_at_centre():
    c = S.tail_cone(G)
    # 94 mm, 58 deg from its axis: a circle of 80 mm radius centred 50 mm along the axis ("~50 mm")
    assert c["half_angle"] == pytest.approx(58.0) and c["radius"] == pytest.approx(79.7, abs=0.1)
    assert c["centre_along_axis"] == pytest.approx(49.8, abs=0.1)
    # at 0: 89 mm behind and 29 mm below the pivot, so 98 mm behind and 5 mm below the hind axles
    assert c["tip0_from_pivot"] == pytest.approx((-89.4, -29.0), abs=0.1)
    assert c["tip0_from_hip"] == pytest.approx((-98.4, -5.0), abs=0.1)
    # at 90 deg: 80 mm to the side and 32 mm ABOVE the pivot
    assert c["tip90_from_pivot"][1:] == pytest.approx((79.7, 32.0), abs=0.1)
    # the tip is lowest at 0, rises the same either way, and moves sideways (which side is left is not known)
    z = {t: S.tail_point(G, t)[2] for t in (-90, -60, -30, -10, 0, 10, 30, 60, 90)}
    assert min(z, key=z.get) == 0
    for t in (10, 30, 60, 90):
        assert z[t] == pytest.approx(z[-t]) and z[t] > z[0]
        assert S.tail_point(G, t)[1] == pytest.approx(-S.tail_point(G, -t)[1]) and S.tail_point(G, t)[1] > 0
    assert S.tail_point(G.with_(tail_sign=-1), 30)[1] < 0
    # the axis angle does not move the tip at 0 (only the sweep)
    assert S.tail_tip(G.with_(tail_axis_deg=30), 0) == pytest.approx(S.tail_tip(G, 0))


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
    p = RU.solve(RU.UP_SPECS[-1])
    r = S.rest(G, p, 28)
    assert S.TAIL in r.contacts and S.load(G, p, r).tail == pytest.approx(0.0, abs=1e-9)
    side = S.Pose(*(p.astuple()[:4]), 20, p.head_pan, p.head_tilt)
    assert 0.05 < abs(S._tail_torque(G, side, 36.0, 28.0)) / S.G_MM_PER_KG_CM < 0.15


def test_hind_legs_cannot_reach_a_steep_tripod():
    """Why the hold is 28 deg and not 60: at 60 deg the tail tip touches with
    the hips 88 mm up, and the hind feet cannot reach forward of the centre
    of mass from there."""
    for dx in range(40, 92, 4):
        try:
            p = RU.solve(RU.Spec("x", phi=60, h=None, hind_dx=dx))
        except RU.DesignError:
            continue
        r = S.rest(G, p, 60)
        assert not ({r.rear, r.front} == {S.TAIL, S.HIND_FOOT} and r.margin > 0), dx


def test_leg_gap():
    assert S.leg_gap(G, RU.STAND) > 70
    # the hind foot swung forward onto the front leg folded back: they cross
    assert S.leg_gap(G, S.Pose(80, 50, -80, -45, 0)) == 0.0


# ---- the design ----------------------------------------------------------------------------------

def test_trick_is_built_and_up_to_date():
    t = RU.build()
    assert library.path_for("rear_up").read_text() == t.to_json(), "stale: run python -m kilo.tricks.rear_up"
    assert library.load("rear_up") == t and "rear_up" in library.names() and library.legacy_name("rear_up") == ""
    assert (t.end_posture, t.stop_ticks, t.balance, t.fall_ok) == (Posture.SIT, 75, False, False)
    assert not any(f.balance for f in t.frames) and t.joint_mask == 0xFF07
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
    assert poses[:top] == poses[top + 1:-1][::-1] and t.frames[top].hold == 100 and top == len(RU.UP_SPECS) - 1
    # slow: cosine moves of at most 40 deg/s on average
    prev = RU.STAND
    for f, p in zip(t.frames, poses):
        d = max(abs(a - b) for a, b in zip(prev.astuple(), p.astuple()))
        assert d / (f.dur * 0.02) <= RU.MOVE_DPS + 1e-6
        prev = p
    assert 10 < t.duration_ticks * 0.02 < 15


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


def test_design_check_over_the_ranges():
    """The gate: over RANGES (the centre of mass +-10 mm both ways, the tail
    within +-3 mm and +-5 deg of the measurements, the outline +-4 mm),
    playing from STAND or SIT and stopped anywhere, Kilo never touches the
    floor with its back, the Pi or its nose, never rolls back more than a
    15 mm drop, never pitches past 70 deg (the fall detector stays armed),
    always ends down on its feet (a stop to STAND could leave it propped on
    the tail: hence SIT), and no servo holds more than half its estimated
    stall torque. It holds the tripod with margin in the nominal geometry
    and at every single range end; at some corners (the centre of mass
    forward and low, the tail drooping) the paws cannot come up and Kilo
    stays on four feet: failing safe, reported as notes."""
    t = RU.build()
    rep = RU.check(t)
    assert rep.problems == []
    nom = [r for g, m, r in rep.results if g == "nominal" and m == "play from STAND"][0]
    assert nom.hold_tripod and 26 < nom.hold_pitch < 31 and nom.hold_margin > 15 and nom.lateral > 20
    singles = [r for g, m, r in rep.results if m.startswith("play") and "," not in g]
    assert singles and all(r.hold_tripod and r.hold_margin > 10 for r in singles)
    plays = [r for g, m, r in rep.results if m.startswith("play")]
    assert sum(r.hold_tripod for r in plays) >= 0.9 * len(plays)
    assert all(r.end_supports and S.TAIL not in r.end_supports for g, m, r in rep.results)
    assert rep.worst("hold_margin", "play", min)[0] > 5
    assert rep.worst("lateral", "play", min)[0] > 20
    assert rep.worst("pitch_max", "", max)[0] < 50
    assert max(v for v, _, _ in rep.torque_max().values()) < RU.TORQUE_MAX_KG_CM


def test_a_stop_to_stand_would_leave_kilo_on_its_tail():
    """Why the end posture is SIT: the straight joint-space line from the
    tripod to STAND straightens the hind legs with the tail on the floor and
    pitches Kilo further up; with the centre of mass 10 mm back it stays
    there, propped on the tail at about 40 deg."""
    t = RU.build()
    poses = [RU.pose_of(f) for f in t.frames]
    top = len(RU.UP_SPECS) - 1
    g = G.with_(com_dx=-10)
    to_stand = S.follow(g, poses[:top + 1] + [RU.STAND], start=RU.STAND)[-1].rest
    assert S.TAIL in to_stand.contacts and to_stand.phi > 35
    to_sit = S.follow(g, poses[:top + 1] + [RU.SIT], start=RU.STAND)[-1].rest
    assert set(to_sit.contacts) >= {S.HIND_FOOT, S.FRONT_FOOT} and S.TAIL not in to_sit.contacts


def test_back_is_never_the_support_over_the_plausible_ranges():
    """If the reading of the tail measurements is off (PLAUSIBLE, wider):
    what can go wrong is a rear sitting down harder or the paws staying
    down; the back and the Pi still never touch the floor."""
    t = RU.build()
    geos = RU.geometries(RU.PLAUSIBLE, corners=("com_dx", "tail_len", "tail_droop_deg"))
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


def test_rear_up_on_the_body_core(rig):
    core, link, client = rig()
    stand(core, client)
    t = library.load("rear_up")
    tid, n = client.start_trick(t)
    fed = play_with_model_attitude(core, client, lambda: client.trick_done_since(n, tid) is not None)
    done = client.trick_done_since(n, tid)
    assert done is not None and done.arg >> 8 == 0  # DONE: no fall, no pre-emption
    assert not events(link, "FELL") and 25 < max(fed) < 32
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


def test_rear_up_stop_is_a_slow_straight_crossfade_to_sit(rig):
    core, link, client = rig()
    stand(core, client)
    t = library.load("rear_up")
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


def test_rear_up_leaves_the_fall_detector_armed(rig):
    """FALL_OK is off: a real fall backwards past 75 deg during the hold is a fall."""
    core, link, client = rig()
    stand(core, client)
    t = library.load("rear_up")
    tid, n = client.start_trick(t)
    hold_i = max(range(len(t.frames)), key=lambda i: t.frames[i].hold)
    play_with_model_attitude(core, client, lambda: False, max_ticks=sum(f.dur for f in t.frames[:hold_i + 1]) + 10)
    assert not events(link, "FELL")
    core.set_attitude(pitch_deg=-85)  # tipping over backwards
    assert core.run_until(lambda: client.trick_done_since(n, tid) is not None, 1) is not None
    assert client.trick_done_since(n, tid).arg >> 8 == 3  # FELL
    core.step(2)
    assert [e.arg for e in events(link, "FELL")] == [-2]  # a pitch fall, nose up
