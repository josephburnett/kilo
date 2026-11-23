"""The trot (kilo.planner.plan Planner, kilo.planner.patterns' trot schedule;
docs/walking-and-balance.md §15: the "long trot" of 2026-11-21, the only trot
since 2026-11-22, the C++ mind's planner checked against it by the golden
vectors). Its plan (reach, joints, the build-up, the placement, the remote's
levels, the changes to and from the other gaits, the stops from every
phase), the fitted pendulum against the floor recordings, and walks in the
virtual body on both physics (Physics and kilo.vbody.rocking.RockingPhysics)."""

import math

import pytest

from kilo.planner import patterns as pat
from kilo.planner.model import LEGS, BodyModel
from kilo.planner.patterns import STANCE_PLUS, TROT, swing_for, trot_timing as timing
from kilo.planner.plan import TROT_FIT, Planner
from kilo.planner.walker import stop_timeout

from stridesim import joint_step_deg, walk

M = BodyModel()
BOUND = 9.7  # deg a tick (485 deg/s): the trot's bound, 0.1 over 9.6 for the FEET's 1 mm rounding at its
#  long strides; under the body's 10 deg/tick slew
# The remote's trot levels (kernel/mind/remote/remote.cpp kTrotSpeedFactors x 144): +1 .. +5
LEVELS = (57.6, 100.8, 144.0, 180.0, 201.6)
STEER_FAST = 0.25  # the remote's steer at +4 and +5 (kTrotFastSteerSteps: the first level only)


def plan(direction="forward", speed=None, ticks=400, **kw):
    p = Planner(M, "trot", direction, speed, tick=1000, **kw)
    return p, [p.next() for _ in range(ticks)]


def check(ticks, what, trot=True):
    """In range and reach, the trot's supports (trot False: the walk's and the crawl's too), every
    joint under BOUND a tick."""
    for t in ticks:
        assert all(-60 <= x <= 60 for x in t.x) and all(30 <= z <= 95 for z in t.z), what
        assert all(M.reachable(t.x[i], t.z[i]) for i in LEGS), what
        assert t.support in (0b0101, 0b1010, 0b1111) or (not trot and bin(t.support).count("1") >= 2), what
    worst = max(joint_step_deg(a, b) for a, b in zip(ticks, ticks[1:]))
    assert worst < BOUND, (what, worst)
    return worst


def test_up_to_150_mm_s_the_swing_is_8_ticks():
    """An 8-tick swing on a 9.5-tick stance (a 350 ms cycle), the stride the speed over that stance."""
    for v in (57.6, 100.8, 144.0, 150.0):
        w, s, vv = timing(v)
        assert w == 8 and s == pytest.approx(v * (8 + STANCE_PLUS) * 0.02) and vv == pytest.approx(v)
        p, _ = plan(speed=v, ticks=300)
        assert p.stance_ticks == pytest.approx(8 + STANCE_PLUS) and p.swing_ticks == 8
    t = pat.timing("trot", 144.0)
    assert (t.stride_mm, t.shift_ms, t.stance_ms, t.period_ms) == (pytest.approx(27.36), 20, 190, 350)


def test_the_swing_and_the_stride_build_up_with_the_speed():
    rows = [timing(v) for v in range(140, 231, 5)]
    assert [swing_for(v) for v in (144, 150, 160, 170, 180, 190, 200, 207)] == [8, 8, 9, 10, 10, 11, 11, 11]
    strides = [s for _, s, _ in rows]
    assert all(b >= a for a, b in zip(strides, strides[1:]))
    assert timing(400)[2] == pytest.approx(TROT.top_mm_s) and timing(400)[1] == pytest.approx(51.75)
    assert timing(400, back=True)[2] == pytest.approx(TROT.top_back_mm_s)
    assert pat.timing("trot", 400, "back").speed_mm_s == pytest.approx(TROT.top_back_mm_s)
    assert pat.speed_range("trot") == (pytest.approx(8 / 0.19), TROT.top_mm_s)
    assert max(strides) / timing(144)[1] > 1.85  # 51.7 against 27.4 mm


@pytest.mark.parametrize("direction", ("forward", "back"))
def test_every_speed_stays_in_reach_and_under_the_joint_bound(direction):
    top = TROT.top_mm_s if direction == "forward" else TROT.top_back_mm_s
    worst = 0.0
    for v in range(40, int(top) + 1, 8):
        _, ticks = plan(direction, v, 260)
        worst = max(worst, check(ticks[40:], (direction, v)))
    assert worst * 50 < 485


@pytest.mark.parametrize("steer", (0.5, 1.0, -1.0))
def test_steering_stays_in_reach_and_under_the_joint_bound(steer):
    for v in (58, 144, 190, 207):
        _, ticks = plan("forward", v, 300, steer=steer)
        check(ticks[40:], (v, steer))


def test_each_pair_lands_where_the_fitted_pendulum_ends_upright():
    """Forward at its top (207 mm/s): 19.1 mm ahead of the shoulders (half the 51.7 mm stride would be
    25.9), where the fitted tip is 0; at half a stride the fitted pendulum would tip it far past the
    clamp."""
    p, ticks = plan("forward", 207, 300)
    w, s, v = timing(207)
    land = TROT_FIT.land_x(w, v)
    assert land == pytest.approx(19.1, abs=0.3) and s / 2 - land > 6
    # the swinging feet's touchdown x (the first tick down after a swing)
    downs = [b.x[leg] for a, b in zip(ticks[100:], ticks[101:]) for leg in LEGS
             if not a.support >> leg & 1 and b.support >> leg & 1]
    assert downs and all(abs(x - land) <= 1 for x in downs)
    assert abs(TROT_FIT.tip_deg(w, v, land, clamp=False)) < 1e-6
    assert abs(TROT_FIT.tip_deg(w, v, s / 2, clamp=False)) > 20


# The recorded trots of the trot this one replaced (mind/tests/stridesim.py fit, ~/kilo-logs of
# 2026-11-15..17: an 8-tick swing landing half a stride ahead): direction, speed, stride, the
# measured tip at a diagonal's end (deg, + toward the front foot in the air)
RECORDED = [("forward", 100.8, 19.2, +1.85), ("forward", 144.0, 27.4, -1.15), ("forward", 165.6, 31.5, -1.75),
            ("forward", 180.0, 34.2, -2.15), ("forward", 187.2, 35.6, -2.15), ("back", 42.1, 8.0, +2.2),
            ("back", 57.6, 10.9, +2.3), ("back", 144.0, 27.4, +2.3)]


def test_the_fitted_pendulum_matches_the_recorded_trots():
    """Within 0.35 deg at each recorded speed (rms 0.15), the sign change between 100 and 144 mm/s
    forward included; the virtual body's Physics gets the forward sign wrong above 144."""
    errs = []
    for d, v, s, tip in RECORDED:
        sv, land = (v, s / 2) if d == "forward" else (-v, -s / 2)
        e = TROT_FIT.tip_deg(8, sv, land) - tip
        errs.append(e)
        assert abs(e) < 0.35, (d, v, e)
    assert math.sqrt(sum(e * e for e in errs) / len(errs)) < 0.2


@pytest.mark.parametrize("speed", (180, 207))
def test_the_long_swing_walks_in_the_virtual_body(speed):
    """8 s on RockingPhysics and on Physics: no fall, no catch, no hold; within 4 deg of its plan (roll
    rms under 2: the long trot on the floor rocked 2.0 rms about its plan, boot 14, kilo model score),
    and less rock than the same strides landing at half a stride (RockingPhysics). Since the body
    model's numbers came from the recordings (2026-11-22: the centre of mass 4.4 mm ahead, the trot's own
    fitted pendulum keeps 3.5; the legs give) the virtual body rocks 1.3 to 1.5 rms here (0.3 before)
    and the landing helps it by a fifth, not by half: the trot is fitted to the floor, the virtual body
    only judges it."""
    r = walk(speed, ticks=400)
    r0 = walk(speed, physics="physics", ticks=400)
    flat = walk(speed, ticks=400, place=False)
    for x in (r, r0):
        assert not x["fell"] and x["catches"] == 0 and x["holds"] == 0, x
        assert x["max_err"] < 4.0 and x["rms_roll"] < 2.0 and x["reach"] and x["fastest"] < 485, x
    assert r["rms_roll"] < 0.85 * flat["rms_roll"], (r, flat)


def test_the_build_up_to_the_long_stride():
    """Speed raised a level at a time while walking (144 -> 207 mm/s): the swing grows a tick per
    slot at most, every tick in reach and under the bound."""
    p = Planner(M, "trot", "forward", 144, tick=0)
    ticks, ws = [], []
    for v in (144, 165, 187, 200, 207):
        p.set_speed(v)
        for _ in range(120):
            ticks.append(p.next())
            ws.append(p.swing_ticks)
    check(ticks, "build-up")
    assert ws[0] == 8 and ws[-1] == 11
    assert all(b - a in (0, 1) for a, b in zip(ws, ws[1:]))


@pytest.mark.parametrize("how", ("halt", "stop"))
def test_it_stops_from_the_long_stride(how):
    """The quick stop and the planned stop from the top (207 mm/s): every square-up step in reach and
    under the bound, the stop done (STANCE) well inside stop_timeout."""
    p = Planner(M, "trot", "forward", 207, tick=0)
    ticks = [p.next() for _ in range(200)]
    getattr(p, how)()
    n = 0
    while not p.done and n < 400:
        ticks.append(p.next())
        n += 1
    assert p.done
    check(ticks, how)
    assert n * 0.02 < stop_timeout(pat.PATTERNS["trot"].period_ms) - 1.0
    assert all(abs(x) <= 5 for x in ticks[-1].x)


def test_the_remotes_levels():
    """A4 and B7 / B5: 57.6 to 201.6 mm/s; +1 .. +3 the 8-tick swing, +4 and +5 10 and 11 ticks
    (strides 41.4 and 50.4 mm), inside the top (207 mm/s)."""
    rows = [timing(v) for v in LEVELS]
    assert [w for w, _, _ in rows] == [8, 8, 8, 10, 11]
    assert [round(s, 1) for _, s, _ in rows[3:]] == [41.4, 50.4]
    assert all(v <= TROT.top_mm_s for v in LEVELS)
    t = pat.timing("trot", 201.6)
    assert t.stance_ms == pytest.approx((11 + STANCE_PLUS) * 20) and t.shift_ms == 20


def _changed(start, v0, script):
    p = Planner(M, start, "forward", v0, tick=0)
    ticks, names, ws = [], [], []
    for item in script:
        if item[0] == "run":
            for _ in range(item[1]):
                ticks.append(p.next())
                names.append(p.pattern.name)
                ws.append(p.swing_ticks)
        else:
            getattr(p, item[0])(*item[1:])
    return p, ticks, names, ws


@pytest.mark.parametrize("start,v0,to,v1", [("walk", 80.0, "trot", 160.0), ("crawl", 18.0, "trot", 160.0),
                                            ("trot", 200.0, "walk", 80.0), ("trot", 200.0, "crawl", 18.0)])
def test_changes_into_and_out_of_the_trot(start, v0, to, v1):
    """Every change in reach and under the joint bound; into the trot at 160 mm/s the swing starts
    at 9 ticks and grows a tick a step; out of it into the walk only once the swing is 8 again."""
    p, ticks, names, ws = _changed(start, v0, [("run", 180), ("set_pattern", to), ("set_speed", v1), ("run", 420)])
    check(ticks[40:], (start, to), trot=False)
    assert names[-1] == to
    i = names.index(to)
    if to == "trot":
        assert ws[i] == 9 and ws[-1] == swing_for(v1)
    elif to == "walk":
        assert ws[i - 1] == 8
    assert i * 0.02 < 180 * 0.02 + (8.0 if "crawl" in (start, to) else 2.0)


@pytest.mark.parametrize("how", ("halt", "stop"))
@pytest.mark.parametrize("steer", (0.0, STEER_FAST))
def test_every_stop_from_every_phase_stays_under_the_bound(how, steer):
    """The quick stop and the planned stop asked at every tick of a cycle at the top level: each
    square-up step from 45 mm behind the shoulder long enough for its joints (a foot near the leg's
    reach turns the knee fast)."""
    for at in range(0, 24):
        p = Planner(M, "trot", "forward", 200.0, tick=0, steer=steer)
        ticks = [p.next() for _ in range(200 + at)]
        getattr(p, how)()
        n = 0
        while not p.done and n < 400:
            ticks.append(p.next())
            n += 1
        assert p.done and n * 0.02 < (1.3 if how == "halt" else 2.1), (at, n)
        check(ticks[150:], (how, at))
