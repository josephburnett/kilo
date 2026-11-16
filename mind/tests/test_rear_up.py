"""The rear_up trick (docs/rear-up-trick.md): the side-view static model
(kilo.tricks.sagittal), the design and its check over the unknowns
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
    # SIT rests nose up on the tail and the front feet (the estimate of where the tail is)
    r = S.rest(G, RU.SIT, 20)
    assert 15 < r.phi < 30 and {r.rear, r.front} == {S.TAIL, S.FRONT_FOOT}


def test_leg_gap():
    assert S.leg_gap(G, RU.STAND) > 70
    # the hind foot swung forward onto the front leg folded back: they cross
    assert S.leg_gap(G, S.Pose(80, 50, -80, -45, 0)) == 0.0


# ---- the design ----------------------------------------------------------------------------------

def test_trick_is_built_and_up_to_date():
    t = RU.build()
    assert library.path_for("rear_up").read_text() == t.to_json(), "stale: run python -m kilo.tricks.rear_up"
    assert library.load("rear_up") == t and "rear_up" in library.names() and library.legacy_name("rear_up") == ""
    assert (t.end_posture, t.stop_ticks, t.balance, t.fall_ok) == (Posture.STAND, 75, False, False)
    assert not any(f.balance for f in t.frames) and t.joint_mask == 0xFF07
    # it ends exactly in STAND (no jump into the end posture), every joint inside its limits
    assert RU.pose_of(t.frames[-1]) == RU.STAND
    for f in t.frames:
        for j, v in enumerate(f.angles):
            if v is not None:
                lo, hi = JOINT_LIMITS[j]
                assert lo <= v <= hi and float(v).is_integer()
    # the way down is the way up, reversed, and one frame holds (the top)
    poses = [RU.pose_of(f) for f in t.frames]
    top = max(range(len(t.frames)), key=lambda i: t.frames[i].hold)
    assert poses[:top] == poses[top + 1:-1][::-1] and t.frames[top].hold == 100
    # slow: cosine moves of at most 40 deg/s on average
    prev = RU.STAND
    for f, p in zip(t.frames, poses):
        d = max(abs(a - b) for a, b in zip(prev.astuple(), p.astuple()))
        assert d / (f.dur * 0.02) <= RU.MOVE_DPS + 1e-6
        prev = p
    assert 15 < t.duration_ticks * 0.02 < 25


def test_solve_reproduces_its_spec():
    """A solved keyframe stands where its Spec says: the pitch, the hind feet and the tail on the floor."""
    for spec in RU.UP_SPECS:
        p = RU.solve(spec)
        r = S.rest(G, p, spec.phi)
        assert r.phi == pytest.approx(spec.phi, abs=3.0), spec.name
        assert r.clear[S.HIND_FOOT] < 1.5, spec.name
    with pytest.raises(RU.DesignError):
        RU.solve(RU.Spec("x", phi=60, h=120, hind_dx=0))  # the tail cannot reach the floor


def test_design_check_over_the_ranges():
    """The gate: over RANGES (the centre of mass +-10 mm both ways, the tail
    and outline within a measurement), playing from STAND or SIT and stopped
    anywhere, Kilo never touches the floor with its back or the Pi, never
    rolls back more than a 15 mm drop, never pitches past 70 deg (the fall
    detector stays armed), holds the tripod with margin, and no servo holds
    more than half its estimated stall torque."""
    t = RU.build()
    rep = RU.check(t)
    assert rep.problems == []
    nom = [r for g, m, r in rep.results if g == "nominal" and m == "play from STAND"][0]
    assert 55 < nom.hold_pitch < 65 and nom.hold_margin > 30 and nom.lateral > 15
    assert rep.worst("hold_margin", "play", min)[0] > 15
    assert rep.worst("pitch_max", "", max)[0] < RU.PITCH_MAX
    assert max(v for v, _, _ in rep.torque_max().values()) < RU.TORQUE_MAX_KG_CM


def test_back_is_never_the_support_over_the_plausible_ranges():
    """Before the owner's measurements (PLAUSIBLE, wider): what can go wrong
    is a rear sitting down harder (up to ~3 cm) or a forward tip onto the
    paws; the back and the Pi still never touch the floor."""
    t = RU.build()
    geos = RU.geometries(RU.PLAUSIBLE, corners=("com_dx", "tail_len", "tail_zero_deg"))
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
    assert not events(link, "FELL") and 55 < max(fed) < 70
    assert abs(len(fed) - (t.duration_ticks + 1)) <= 2
    core.step(15)  # the end crossfade (nothing moves: the last frame is STAND)
    d = core.dump()
    assert d["posture"] == int(Posture.STAND) and not d["trick_run"] and link.state.mode == "IDLE"
    assert pose_of_out(d["out"]) == RU.STAND
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


def test_rear_up_stop_is_a_slow_straight_crossfade_to_stand(rig):
    core, link, client = rig()
    stand(core, client)
    t = library.load("rear_up")
    tid, n = client.start_trick(t)
    top = sum(f.dur for f in t.frames[:7]) + 20  # into the hold
    play_with_model_attitude(core, client, lambda: False, max_ticks=top)
    start = core.dump()["out"]
    client.stop_trick()  # None: the trick's own stop_ticks
    assert core.run_until(lambda: client.trick_done_since(n, tid) is not None, 1) is not None
    assert client.trick_done_since(n, tid).arg >> 8 == 1  # STOPPED
    stand_out = [v * 16 for v in (0, 0, 0, 30, 30, -30, -30, 30, 30, -30, -30)]
    ws = []
    for _ in range(t.stop_ticks + 5):
        core.step(1)
        out = core.dump()["out"]
        # every joint the same fraction of the way: a straight line in joint space (what the check assumed)
        w = [(o - s) / (e - s) for o, s, e in zip(out, start, stand_out) if abs(e - s) > 200]
        assert max(w) - min(w) < 0.06, w
        ws.append(sum(w) / len(w))
    assert ws[0] < 0.1 and ws[t.stop_ticks // 2] < 0.75 and ws[-1] == pytest.approx(1.0, abs=0.01)
    assert core.dump()["posture"] == int(Posture.STAND)


def test_rear_up_leaves_the_fall_detector_armed(rig):
    """FALL_OK is off: a real fall backwards past 75 deg during the hold is a fall."""
    core, link, client = rig()
    stand(core, client)
    t = library.load("rear_up")
    tid, n = client.start_trick(t)
    play_with_model_attitude(core, client, lambda: False, max_ticks=sum(f.dur for f in t.frames[:7]) + 10)
    assert not events(link, "FELL")
    core.set_attitude(pitch_deg=-85)  # tipping over backwards
    assert core.run_until(lambda: client.trick_done_since(n, tid) is not None, 1) is not None
    assert client.trick_done_since(n, tid).arg >> 8 == 3  # FELL
    core.step(2)
    assert [e.arg for e in events(link, "FELL")] == [-2]  # a pitch fall, nose up
