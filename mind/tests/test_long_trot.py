"""The long trot (kilo.planner.longtrot, docs/walking-and-balance.md §15):
a prototype, not wired to the remote or the C++ mind. Its plan (reach,
joints, the build-up, the placement), the fitted pendulum against the floor
recordings, and walks in the virtual body on both physics (Physics and
kilo.vbody.rocking.RockingPhysics)."""

import math

import pytest

from kilo.planner import patterns as pat
from kilo.planner.longtrot import (LONG_TROT, STANCE_PLUS, TROT_FIT, LongTrotConfig, LongTrotPlanner, swing_for,
                                   timing)
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


def check(ticks, what):
    for t in ticks:
        assert all(-60 <= x <= 60 for x in t.x) and all(30 <= z <= 95 for z in t.z), what
        assert all(M.reachable(t.x[i], t.z[i]) for i in LEGS), what
        assert t.support in (0b0101, 0b1010, 0b1111), what
    worst = max(joint_step_deg(a, b) for a, b in zip(ticks, ticks[1:]))
    assert worst < BOUND, (what, worst)
    return worst


def test_the_existing_gaits_do_not_know_it():
    """A separate pattern object: not among the remote's gaits, the golden vectors' or the walker's."""
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
