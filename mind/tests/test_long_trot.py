"""The long trot (kilo.planner.longtrot, docs/walking-and-balance.md §15;
on the remote's A1 since 2026-11-21, the C++ mind's planner checked against
LongTrotPlanner by the golden vectors). Its plan (reach, joints, the
build-up, the placement, the stages, the changes to and from the other
gaits, the stops from every phase), the fitted pendulum against the floor
recordings, and walks in the virtual body on both physics (Physics and
kilo.vbody.rocking.RockingPhysics)."""

import math

import pytest

from kilo.planner import patterns as pat
from kilo.planner.longtrot import (LONG_TROT, STAGES, STANCE_PLUS, STEER_MAX, TROT_FIT, LongTrotConfig,
                                   LongTrotPlanner, pattern_timing, swing_for, timing)
from kilo.planner.model import LEGS, BodyModel
from kilo.planner.plan import Planner
from kilo.planner.walker import stop_timeout

from stridesim import joint_step_deg, walk

M = BodyModel()
BOUND = 9.7  # deg a tick (485 deg/s): the trot's own bound above 184 mm/s (tests/test_planner.py fast_bound,
#  9.6) and 0.1 for the FEET's 1 mm rounding at the long trot's strides; under the body's 10 deg/tick slew


def plan(direction="forward", speed=None, ticks=400, **kw):
    p = LongTrotPlanner(M, direction, speed, tick=1000, **kw)
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


def test_the_existing_gaits_do_not_know_it():
    """A separate pattern object: not among patterns.PATTERNS (the Python walker's gaits and the golden
    vectors' statics); LongTrotPlanner knows it by name."""
    assert pat.PATTERN_NAMES == ("walk", "trot", "crawl")
    assert "longtrot" not in pat.PATTERNS and LONG_TROT.name == "longtrot"
    with pytest.raises(ValueError):
        pat.pattern("longtrot")


def test_up_to_150_mm_s_it_is_todays_trot():
    """Same swing (8 ticks), stride and stance (9.5 ticks) as the trot; its feet within 2 mm of the
    trot's (the placement: 1.9 mm further ahead at 58 mm/s, 0.4 mm behind at 144)."""
    for v in (57.6, 100.8, 144.0):
        w, s, vv = timing(v)
        tr = pat.timing("trot", v)
        assert w == 8 and s == pytest.approx(tr.stride_mm) and vv == pytest.approx(tr.speed_mm_s)
        p, lt = plan(speed=v, ticks=300)
        q = Planner(M, "trot", "forward", v, tick=1000)
        tt = [q.next() for _ in range(300)]
        assert p.stance_ticks == pytest.approx(q.stance_ticks) == 8 + STANCE_PLUS
        # once the stride has grown (0.75 mm a tick here, 1.5 in the trot)
        assert max(abs(a.x[i] - b.x[i]) for a, b in zip(lt[100:], tt[100:]) for i in LEGS) <= 2
        assert [a.support for a in lt] == [b.support for b in tt]


def test_the_swing_and_the_stride_build_up_with_the_speed():
    cfg = LongTrotConfig()
    rows = [timing(v) for v in range(140, 231, 5)]
    assert [swing_for(v) for v in (144, 150, 160, 170, 180, 190, 200, 207)] == [8, 8, 9, 10, 10, 11, 11, 11]
    strides = [s for _, s, _ in rows]
    assert all(b >= a for a, b in zip(strides, strides[1:]))
    assert timing(400)[2] == pytest.approx(cfg.top_mm_s) and timing(400)[1] == pytest.approx(51.75)
    assert timing(400, back=True)[2] == pytest.approx(cfg.top_back_mm_s)
    assert max(strides) / timing(144)[1] > 1.85  # 51.7 against 27.4 mm


@pytest.mark.parametrize("direction", ("forward", "back"))
def test_every_speed_stays_in_reach_and_under_the_joint_bound(direction):
    top = LongTrotConfig().top_mm_s if direction == "forward" else LongTrotConfig().top_back_mm_s
    worst = 0.0
    for v in range(140, int(top) + 1, 4):
        _, ticks = plan(direction, v, 260)
        worst = max(worst, check(ticks[40:], (direction, v)))
    assert worst * 50 < 485


@pytest.mark.parametrize("steer", (0.5, 1.0, -1.0))
def test_steering_stays_in_reach_and_under_the_joint_bound(steer):
    for v in (144, 190, 207):
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


# The recorded trots (mind/tests/stridesim.py fit, ~/kilo-logs of 2026-11-15..17): direction, speed,
# stride, the measured tip at a diagonal's end (deg, + toward the front foot in the air)
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
def test_the_long_trot_walks_in_the_virtual_body(speed):
    """8 s on RockingPhysics and on Physics: no fall, no catch, no hold; within 4 deg of its plan (roll
    rms under 1.2), less than half the rock of the same strides landing at half a stride
    (RockingPhysics; the two physics put the best landing 3 mm either side of the fitted one)."""
    r = walk(speed, ticks=400)
    r0 = walk(speed, physics="physics", ticks=400)
    flat = walk(speed, ticks=400, cfg=LongTrotConfig(place=False))
    for x in (r, r0):
        assert not x["fell"] and x["catches"] == 0 and x["holds"] == 0, x
        assert x["max_err"] < 4.0 and x["rms_roll"] < 1.2 and x["reach"] and x["fastest"] < 485, x
    assert r["rms_roll"] < 0.5 * flat["rms_roll"], (r, flat)


def test_the_build_up_from_todays_trot_to_the_long_stride():
    """Speed raised a level at a time while walking (144 -> 207 mm/s): the swing grows a tick per
    slot at most, every tick in reach and under the bound."""
    p = LongTrotPlanner(M, "forward", 144, tick=0)
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
    p = LongTrotPlanner(M, "forward", 207, tick=0)
    ticks = [p.next() for _ in range(200)]
    getattr(p, how)()
    n = 0
    while not p.done and n < 400:
        ticks.append(p.next())
        n += 1
    assert p.done
    check(ticks, how)
    assert n * 0.02 < stop_timeout(LONG_TROT.period_ms) - 1.0
    assert all(abs(x) <= 5 for x in ticks[-1].x)


def _script(p, script):
    out = []
    for item in script:
        if item[0] == "run":
            out += [p.next() for _ in range(item[1])]
        elif item[0] == "until_done":
            n = 0
            while not p.done and n < item[1]:
                out.append(p.next())
                n += 1
        else:
            getattr(p, item[0])(*item[1:])
    return out


@pytest.mark.parametrize("pattern", ("trot", "walk", "crawl"))
def test_in_the_other_gaits_it_is_the_planner(pattern):
    """LongTrotPlanner in the walk, the trot and the crawl plans exactly what Planner plans (the C++
    mind is one planner for all four)."""
    script = [("run", 90), ("set_speed", 60.0), ("set_steer", 0.5), ("run", 60), ("set_pattern", "walk"),
              ("run", 120), ("set_pattern", "crawl"), ("run", 250), ("set_pattern", "trot"), ("run", 300),
              ("halt",), ("until_done", 400)]
    a = _script(Planner(M, pattern, "forward", None, tick=7), script)
    b = _script(LongTrotPlanner(M, "forward", None, tick=7, pattern=pattern), script)
    assert len(a) == len(b) > 800
    assert all(x.x == y.x and x.z == y.z and x.support == y.support and x.pitch == y.pitch and x.roll == y.roll
               for x, y in zip(a, b))


def test_the_remotes_stages():
    """A1 and B7 / B5: 160, 180, 200 mm/s, a swing of 9, 10, 11 ticks, strides 33.6, 41.4, 50.0 mm,
    each inside the study's top (207 mm/s)."""
    rows = [timing(v) for v in STAGES]
    assert [w for w, _, _ in rows] == [9, 10, 11]
    assert [round(s, 1) for _, s, _ in rows] == [33.6, 41.4, 50.0]
    assert all(v <= LongTrotConfig().top_mm_s for v in STAGES) and STEER_MAX <= 0.25
    t = pattern_timing(LONG_TROT, 200.0)
    assert t.stance_ms == pytest.approx((11 + STANCE_PLUS) * 20) and t.shift_ms == 20


def _changed(start, v0, script):
    p = LongTrotPlanner(M, "forward", v0, tick=0, pattern=start)
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


@pytest.mark.parametrize("start,v0,to,v1", [("trot", 144.0, "longtrot", 160.0), ("trot", 230.4, "longtrot", 160.0),
                                            ("walk", 80.0, "longtrot", 160.0), ("crawl", 18.0, "longtrot", 160.0),
                                            ("longtrot", 200.0, "trot", 144.0), ("longtrot", 200.0, "walk", 80.0),
                                            ("longtrot", 200.0, "crawl", 18.0)])
def test_changes_into_and_out_of_the_long_trot(start, v0, to, v1):
    """Every change in reach and under the joint bound; into the long trot the swing starts at 9
    ticks (one more than the trot's) and grows a tick a step; out of it directly only once the
    swing is today's 8 again."""
    p, ticks, names, ws = _changed(start, v0, [("run", 180), ("set_pattern", to), ("set_speed", v1), ("run", 420)])
    check(ticks[40:], (start, to), trot=start in ("trot", "longtrot") and to in ("trot", "longtrot"))
    assert names[-1] == to
    i = names.index(to)
    if to == "longtrot":
        assert ws[i] == 9 and ws[-1] == swing_for(v1)
    elif start == "longtrot" and to != "crawl":
        assert ws[i - 1] == 8
    assert i * 0.02 < 180 * 0.02 + (8.0 if "crawl" in (start, to) else 2.0)


@pytest.mark.parametrize("how", ("halt", "stop"))
@pytest.mark.parametrize("steer", (0.0, STEER_MAX))
def test_every_stop_from_every_phase_stays_under_the_bound(how, steer):
    """The quick stop and the planned stop asked at every tick of a cycle at the top stage: each
    square-up step from 45 mm behind the shoulder long enough for its joints (a foot near the leg's
    reach turns the knee fast)."""
    for at in range(0, 24):
        p = LongTrotPlanner(M, "forward", STAGES[-1], tick=0, steer=steer)
        ticks = [p.next() for _ in range(200 + at)]
        getattr(p, how)()
        n = 0
        while not p.done and n < 400:
            ticks.append(p.next())
            n += 1
        assert p.done and n * 0.02 < (1.3 if how == "halt" else 2.1), (at, n)
        check(ticks[150:], (how, at))
