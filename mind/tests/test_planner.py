"""The planner (kilo.planner): the body model's geometry and sign
conventions, the patterns' speed mapping, the planned steps (margins,
slew, lead, prediction, speed / direction / pattern changes, the stop, the
gate and the catch) and the error estimates. Pure computation: no body."""

import math
from dataclasses import replace

import pytest

from kilo.planner import errors
from kilo.planner import patterns as pat
from kilo.planner.model import (LB, LEGS, LF, RB, RF, BodyModel, Mass, fit_attitude, leg_lengths, rotation,
                                stability, world_points)
from kilo.planner.plan import Planner, PlannerConfig
from kilo.protocol import kilo_protocol as P

M = BodyModel()
H = M.stance_height(205)  # 76 mm: STANCE's body height


# ---- the body model ------------------------------------------------------------------------

def test_model_masses_and_reach():
    assert M.total_g == pytest.approx(358.0)  # weighed 2026-11-14, the Pi mounted
    cx, cy, cz = M.com([0] * 4, [H] * 4)
    # as measured by hand (7.5 ahead), moved by the correction the recordings predict best with (-3 mm)
    assert abs(cy) < 1e-9 and cx == pytest.approx(7.5 + M.com_dx_mm, abs=0.2) and cz == pytest.approx(-3.0, abs=0.3)
    assert H == pytest.approx(76.0, abs=0.1)
    assert M.tau_s(80) == pytest.approx(0.090, abs=0.001)  # the 90 ms tip time constant of the doc
    for leg in LEGS:
        for x, z in ((0, H), (20, 70), (-25, 60), (10, 85)):
            t1, t2 = M.ik(leg, x, z)
            assert M.fk(t1, t2) == pytest.approx((x, z), abs=1e-6)
            assert (t2 > 0) == (leg in (LF, RF))  # front knees point back, back knees forward
    assert not M.reachable(0, 95) and M.reachable(0, 90)


def test_leg_lengths_and_attitude_conventions():
    """Pitch + = nose down: the back legs are the longer; roll + = right side
    up: the right legs are the longer (TELEMETRY, the firmware's H_i)."""
    xs = [0.0] * 4
    zs = leg_lengths(M, xs, H, 5.0, 0.0)
    assert zs[RB] > zs[RF] and zs[LB] > zs[LF] and sum(zs) / 4 == pytest.approx(H)
    zs = leg_lengths(M, xs, H, 0.0, 5.0)
    assert zs[RF] > zs[LF] and zs[RB] > zs[LB]
    for p, r in ((3.0, -7.0), (-4.0, 9.0), (0.0, 12.0)):
        zs = leg_lengths(M, [5.0, -3.0, 2.0, -8.0], H, p, r)
        assert fit_attitude(M, [5.0, -3.0, 2.0, -8.0], zs, 0xF) == pytest.approx((p, r), abs=1e-4)
        three = 0xF & ~(1 << RB)
        assert fit_attitude(M, [5.0, -3.0, 2.0, -8.0], zs, three) == pytest.approx((p, r), abs=1e-4)
    # on two feet the plane is free about their line: the previous attitude fills in
    diag = (1 << LF) | (1 << RB)
    assert fit_attitude(M, xs, [H] * 4, diag, prev=(0.0, 0.0)) == pytest.approx((0.0, 0.0), abs=1e-6)
    # the world: a nose-down pitch lowers the front feet, a positive roll the left ones
    R = rotation(10.0, 0.0)
    assert R[2][0] < 0  # height of a point ahead (+x) goes down
    R = rotation(0.0, 10.0)
    assert R[2][1] < 0  # right side up is + roll: a point on the left (+y) goes down
    com, feet = world_points(M, xs, leg_lengths(M, xs, H, 0.0, 10.0), 0.0, 10.0)
    assert feet[RF][2] == pytest.approx(feet[LF][2], abs=1e-6)  # all four on the ground
    feet_y = sum(f[1] for f in feet) / 4
    assert com[1] - feet_y == pytest.approx((H + M.foot_r_mm + M.com(xs, [H] * 4)[2]) * math.sin(math.radians(10)),
                                            abs=0.5)  # right side up: the mass moves left of its feet, h sin(roll)


def test_stability_margin():
    zs = [H] * 4
    four = stability(M, [0.0] * 4, zs, 0xF, 0.0, 0.0)
    assert four == pytest.approx(M.track_mm / 2, abs=0.5)  # the half track
    # on three feet the mass sits near the diagonal's edge (nose heavy: 4.4 mm ahead of the centre)
    for lifted in LEGS:
        m = stability(M, [0.0] * 4, zs, 0xF & ~(1 << lifted), 0.0, 0.0)
        assert abs(m) < 4.0
    # a diagonal pair has no inside
    assert stability(M, [0.0] * 4, zs, (1 << LF) | (1 << RB), 0.0, 0.0) <= 0


# ---- patterns and speed --------------------------------------------------------------------------

def test_pattern_tables_and_default_timing():
    # the tables of 2026-11-16 (walk one leg at a time with the lean before, 40 mm / 2550 ms at 17.8,
    # top 21.5; the crawl unchanged) and the trot of 2026-11-22 (test_trot.py: its swing grows with the
    # speed, 8 ticks at 144 mm/s on a 9.5-tick stance)
    t = pat.timing("trot")
    assert (round(t.stride_mm, 2), round(t.period_ms), t.shift_ms) == (27.36, 350, 20)
    assert t.speed_mm_s == pytest.approx(144.0, abs=0.1)
    assert pat.PATTERNS["trot"].groups == ((LF, RB), (RF, LB)) and pat.PATTERNS["trot"].couple_ms == 0
    # the walk (since 2026-11-22): the lateral sequence one leg at a time, the strides centred under
    # the centre of mass, the lift tapering from 8 mm at a 40 mm stride to 6 at 54; its second
    # version: a 200 ms swing, 3 ticks on four feet before a hind swing and 1 before a fore swing,
    # a 1 deg roll and 3 mm fore-aft sway
    wp = pat.PATTERNS["walk"]
    assert wp.groups == ((LB,), (LF,), (RB,), (RF,)) and wp.couple_ms == 0 and wp.centre_com
    assert wp.handover_ticks == (3, 1, 3, 1) and (wp.sway_roll_deg, wp.sway_dx_mm) == (-1.0, 3.0)
    assert [pat.step_height(wp, s) for s in (30, 40, 47, 54)] == [8, 8, 7, 6]
    assert pat.PATTERNS["crawl"].groups == ((LB,), (LF,), (RB,), (RF,))
    assert [pat.PATTERNS[p].swing_ms for p in ("trot", "walk", "crawl")] == [160, 200, 300]
    assert not pat.PATTERNS["walk"].lean and not pat.PATTERNS["trot"].lean and pat.PATTERNS["crawl"].lean
    w = pat.timing("walk")
    assert (round(w.stride_mm, 1), round(w.period_ms), w.shift_ms) == (44.1, 960, 40)
    assert w.speed_mm_s == pytest.approx(58.0, abs=0.1)
    c = pat.timing("crawl")
    assert (c.stride_mm, round(c.period_ms)) == (50, 2550) and c.speed_mm_s == pytest.approx(22.2, abs=0.1)
    assert pat.PATTERNS["crawl"].body_h_256 == 179 and pat.PATTERNS["walk"].body_h_256 == 205
    ranges = {n: pat.speed_range(n) for n in pat.PATTERN_NAMES}
    # the trot's top: 207 mm/s, a 51.8 mm stride on an 11-tick swing, the lift 8 mm at every stride
    assert ranges["trot"] == pytest.approx((42.1, 207.0), abs=0.1)
    assert pat.step_height(pat.PATTERNS["trot"], 50) == 8 and pat.step_height(pat.PATTERNS["crawl"], 50) == 15
    assert [pat.PATTERNS[n].stride_slew_mm for n in ("walk", "trot", "crawl")] == [1.0, 0.75, 0.5]
    assert ranges["walk"] == pytest.approx((10.5, 73.0), abs=0.1)
    # above 63 mm/s (and for a side stride past 48 mm) the walk's swing is 220 ms, its four-foot
    # ticks 2 and 0: the footfalls stay where they were
    assert (pat.swing_at(wp, 63), pat.swing_at(wp, 63.1)) == (200, 220) and wp.handover_long_ticks == (2, 0, 2, 0)
    assert pat.timing("walk", 72.5).shift_ms == 20 and pat.timing("walk", 72.5).period_ms == 960
    assert ranges["crawl"][1] == pytest.approx(26.9, abs=0.1)


@pytest.mark.parametrize("name", pat.PATTERN_NAMES)
def test_timing_clamps(name):
    lo, hi = pat.speed_range(name)
    p = pat.PATTERNS[name]
    for v in (lo, hi, (lo + hi) / 2):
        t = pat.timing(name, v)
        assert t.speed_mm_s == pytest.approx(v, rel=1e-6)
        # (the walk's long swing at its top speeds takes its extra tick from the four-foot time)
        long_ms = pat.swing_at(p, v) - p.swing_ms
        assert t.shift_ms >= pat.shift_min_ms(p) - long_ms - 1e-6 and t.stance_ms <= p.stance_max_ms + 1e-6
    assert pat.timing(name, hi * 3).speed_mm_s == pytest.approx(hi)  # capped
    slow = pat.timing(name, lo / 3)
    assert slow.stride_mm == pat.STRIDE_MIN_MM and slow.speed_mm_s == pytest.approx(lo)
    assert pat.timing(name, -20) == pat.timing(name, 20)  # a magnitude
    if pat.is_trot(p):
        # the trot: one tick on four feet at every speed, the swing growing with it (test_trot.py)
        assert pat.timing(name, lo).shift_ms == pat.timing(name, hi).shift_ms == 20
        assert pat.timing(name, lo).period_ms < pat.timing(name, hi).period_ms
    elif pat.fixed_cadence(p):
        # the walk: one cycle at every speed, the stride alone changes (its four-foot phases stay
        # 1 to 3 ticks, 2 on average, 1 with the long swing at the top: the body's balance loop,
        # which holds for two ticks after a touchdown, barely acts in them, §12, §16)
        assert pat.timing(name, lo).period_ms == pat.timing(name, hi).period_ms
        assert (pat.timing(name, lo).shift_ms, pat.timing(name, hi).shift_ms) == (40, 20)
        assert pat.timing(name, lo).stride_mm < pat.timing(name, hi).stride_mm == pytest.approx(p.stride_mm)
    else:
        # the crawl: the swing never changes with the speed, the shift does
        assert pat.timing(name, lo).period_ms - pat.timing(name, hi).period_ms > 0


def test_the_crawl_shift_covers_the_lean_move():
    # the move (300 ms), the lift 60 ms before its end
    assert pat.shift_min_ms(pat.PATTERNS["crawl"]) == 240
    assert pat.timing("crawl", 1e6).shift_ms == pytest.approx(240)
    # the trot: one tick on four feet (and the swing's last tick: its foot is down); the walk two on
    # average (3 before a hind swing, 1 before a fore swing)
    assert pat.shift_min_ms(pat.PATTERNS["trot"]) == 20 and pat.shift_min_ms(pat.PATTERNS["walk"]) == 40


def test_side_strides():
    assert pat.side_strides("forward", 25) == (25, 25)
    # a steer bends forward and backward walks (+ left: the right side pushes more, the nose turns left)
    assert pat.side_strides("forward", 20, 0.25) == (15, 25)
    assert pat.side_strides("back", 20, 0.5) == (-30, -10)  # backward: the left side pushes back more
    assert pat.side_strides("forward", 20, -1.0) == (40, 0)  # the inner side steps in place
    assert pat.side_strides("forward", 40, 0.5, cap=44) == pytest.approx((44 / 3, 44))  # both scaled to the cap
    assert pat.side_strides("forward", 20, 3.0) == (0, 40)  # at most 1
    assert pat.side_strides("left", 25, 0.5) == (-20, 20) and pat.side_strides("march", 25, 1) == (0, 0)
    assert pat.side_strides("backward", 25) == (-25, -25)
    assert pat.side_strides("left", 25) == (-20, 20)  # turning left: the right side pushes forward
    assert pat.side_strides("right", 12.5) == (10, -10)
    assert pat.side_strides("march", 25) == (0, 0)
    with pytest.raises(ValueError):
        pat.direction("sideways")
    with pytest.raises(ValueError):
        pat.pattern("gallop")


# ---- the plan --------------------------------------------------------------------------------------

def plan(pattern="walk", ticks=600, config=None, **kw):
    p = Planner(M, pattern, tick=1000, config=config, **kw)
    return p, [p.next() for _ in range(ticks)]


def joint_step_deg(a, b):
    worst = 0.0
    for leg in LEGS:
        t = M.ik(leg, a.x[leg], a.z[leg])
        u = M.ik(leg, b.x[leg], b.z[leg])
        worst = max(worst, abs(math.degrees(t[0] - u[0])), abs(math.degrees(t[1] - u[1])))
    return worst


@pytest.mark.parametrize("name", pat.PATTERN_NAMES)
def test_plan_fits_feet_and_the_slew(name):
    _, ticks = plan(name)
    assert [t.tick for t in ticks] == list(range(1000, 1600))
    lo, hi = [int(v) for v in "-60 60".split()]
    for t in ticks:
        assert all(lo <= x <= hi for x in t.x) and all(30 <= z <= 95 for z in t.z)
        assert all(M.reachable(t.x[i], t.z[i]) for i in LEGS)
        assert bin(t.support).count("1") >= (2 if name == "trot" else 3)
        if bin(t.support).count("1") == 2:  # two feet: always a diagonal pair
            assert t.support in (0b0101, 0b1010)
    assert max(joint_step_deg(a, b) for a, b in zip(ticks, ticks[1:])) < M.slew_deg_per_tick


def fast_bound(name: str, v: float, steer: float = 0.0) -> float:
    """The fastest joint a fast gait may ask, deg a tick: 8.3 (415 deg/s; the
    floor runs of 2026-11 asked up to 414 at 180 mm/s and kept up). The
    trot above its 8-tick swing (150 mm/s) and steering at any speed (its
    outer side takes the longer stride): 9.7 (485 deg/s, test_trot.py
    BOUND), under the body's 10 deg/tick slew, 0.1 of it for FEET's 1 mm
    rounding at its long strides. The walk (since 2026-11-22): 9.5 (475
    deg/s), its 54 mm stride in a 220 ms swing with the 3 mm fore-aft sway
    asks up to 452 at its top, 471 backward with the steer at its full."""
    if name == "walk":
        return 9.5
    return 9.7 if name == "trot" and (v > 150.0 or steer) else 8.3


@pytest.mark.parametrize("name", ("walk", "trot"))
@pytest.mark.parametrize("direction", ("forward", "back", "left"))
def test_every_speed_of_the_fast_gaits_stays_inside_the_slew(name, direction):
    """At their slowest, default and top speed every joint of the walk and the
    trot stays inside fast_bound(), under the body's 10 deg/tick slew."""
    lo, hi = pat.speed_range(name)
    for v in (lo, pat.default_speed(name), hi):
        _, ticks = plan(name, 300, direction=direction, speed_mm_s=v)
        assert max(joint_step_deg(a, b) for a, b in zip(ticks, ticks[1:])) < fast_bound(name, v), (v, direction)


@pytest.mark.parametrize("name", pat.PATTERN_NAMES)
@pytest.mark.parametrize("direction", ("forward", "back"))
def test_steering_bends_the_path_inside_the_reach_and_the_slew(name, direction):
    """The remote's A6 / C6 while walking (2026-11-17): the steer shortens
    the inner side's stride and lengthens the outer's, never past the
    pattern's longest stride, so every joint stays as it is at the top
    speed: inside fast_bound() in the walk and the trot, the body's slew in
    the crawl (whose backward swing at its top asked 11.9 deg a tick before
    steering existed: §12.3; the steer does not add to it), every foot in
    reach and inside FEET's range."""
    lo, hi = pat.speed_range(name)
    for v in (pat.default_speed(name), hi):
        for steer in (0.25, 0.5, 1.0, -1.0):
            p = Planner(M, name, direction, v, tick=1000, steer=steer)
            ticks = [p.next() for _ in range(400)]
            sl, sr = p.s_cur
            cap = pat.PATTERNS[name].stride_mm
            assert max(abs(sl), abs(sr)) <= cap + 1e-9
            # the nose turns to the steer's side, forward and backward alike: the right side pushes more
            assert (sr - sl) * steer > 0, (v, steer, p.s_cur)
            # the bend: (right - left) / (right + left) = the steer (the cap scales both together)
            assert abs((sr - sl) / (sr + sl)) == pytest.approx(abs(steer), rel=1e-6)
            for t in ticks:
                assert all(-60 <= x <= 60 for x in t.x) and all(M.reachable(t.x[i], t.z[i]) for i in LEGS)
            worst = max(joint_step_deg(a, b) for a, b in zip(ticks, ticks[1:]))
            bound = fast_bound(name, v, steer) if pat.fixed_cadence(pat.PATTERNS[name]) else (
                12.0 if direction == "back" else M.slew_deg_per_tick)
            assert worst < bound, (v, steer, worst)


@pytest.mark.parametrize("name", ("walk", "trot"))
@pytest.mark.parametrize("to", ("forward", "back"))
def test_a_turn_in_place_goes_straight_within_a_second(name, to):
    """The remote's B7 / B5 while turning in place (2026-11-17): the strides
    move 0.75 mm a tick in the trot (its build-up), so the turn becomes a
    straight trot in under 1.4 s; the walk (1.0 mm a tick, strides up to 44
    mm at 1x) in under 1.7 s, under two of its cycles; every foot in reach
    and every joint inside the bound."""
    p = Planner(M, name, "left", tick=0)
    ticks = [p.next() for _ in range(150)]
    p.set_direction(to)
    n = 0
    while any(abs(a - b) > 1e-9 for a, b in zip(p.s_cur, p.s_tgt)):
        ticks.append(p.next())
        n += 1
    assert n * 20 < (1700 if name == "walk" else 1400), n
    ticks += [p.next() for _ in range(100)]
    check_ticks(ticks, (name, to))
    assert max(joint_step_deg(a, b) for a, b in zip(ticks, ticks[1:])) < fast_bound(name, 0)


@pytest.mark.parametrize("direction", ("forward", "back"))
def test_the_walk_is_the_lateral_sequence_evenly_spaced(direction):
    """The walk (since 2026-11-22): one leg at a time, LB, LF, RB, RF (the
    cat's lateral sequence; backward too), the footfalls nearly evenly spaced
    (220 ms from a hind foot to its fore foot, 260 from a fore foot to the
    other side's hind foot: the extra four-foot time where the weight crosses
    the diagonal; the old walk's came 60 and 180 ms apart), never fewer than
    three feet down; walking forward the centre of mass at least 0.3 mm inside
    the three feet at every tick with a leg up, at every speed but the slowest
    (from the model; 1 to 2 mm from 40 mm/s up: docs/walking-and-balance.md
    §16.8; it was up to 7.5 mm outside them in the old walk); walking backward
    it is outside (2 to 7 mm: backward the walk is its first version, §16.8)."""
    order = (LB, LF, RB, RF)
    lo, hi = pat.speed_range("walk")
    for v in (lo * 2, pat.default_speed("walk"), hi):
        _, ticks = plan("walk", 500, direction=direction, speed_mm_s=v)
        ticks = ticks[150:]
        seen = []
        for t in ticks:
            assert len(t.lifting) <= 1 and bin(t.support).count("1") >= 3
            if t.lifting and (not seen or seen[-1] != t.lifting[0]):
                seen.append(t.lifting[0])
        i = seen.index(order[0])
        assert tuple(seen[i:i + 4]) == order
        downs = [(t.tick, a.lifting[0]) for a, t in zip(ticks, ticks[1:])
                 if bin(t.support).count("1") > bin(a.support).count("1")]
        gaps = {(leg, b - a) for (a, leg), (b, _) in zip(downs, downs[1:])}
        # forward 220 ms after a hind foot, 260 after a fore foot; backward evenly, 240 ms
        want = {(LB, 11), (LF, 13), (RB, 11), (RF, 13)} if direction == "forward" else {(leg, 12) for leg in order}
        assert gaps == want, gaps
        worst = min(t.margin for t in ticks if t.lifting)
        assert worst > ((0.3 if v > lo * 2 else 0.2) if direction == "forward" else -7.5), (v, worst)


@pytest.mark.parametrize("name", ("crawl",))
def test_the_lean_keeps_the_mass_inside_three_feet(name):
    p, ticks = plan(name)
    three = [t for t in ticks if len(t.lifting) == 1]
    # margin_mm 14, linearised, and the move's last three ticks after the lift counted: at least 11 mm
    assert three and min(t.margin for t in three) > 11.0
    _, flat = plan(name, config=PlannerConfig(lean=False))
    assert min(t.margin for t in flat if len(t.lifting) == 1) < 0  # the same steps without it: over the edge
    # the lift starts three ticks before the lean's move ends, in its cosine tail (under a degree
    # of roll left), and the lean is held from there to the touchdown
    lead = p.lead
    assert lead == -3  # 60 ms
    for i, t in enumerate(ticks):
        if t.lifting and not ticks[i - 1].lifting and i - lead < len(ticks):
            held = ticks[i - lead]
            assert max(abs(ticks[j].plan_roll - held.plan_roll) for j in range(i, i - lead)) < 1.0, i
            j = i - lead
            while j + 1 < len(ticks) and ticks[j + 1].lifting:
                j += 1
                # plan_roll is the plane through the feet in the whole mm FEET carries: a foot's z
                # rounding the other way moves it up to 0.6 deg (the 70 mm track), the lean stays
                assert abs(ticks[j].plan_roll - held.plan_roll) < 0.75, (i, j)
    # the side the mass goes to: away from the lifting leg (roll + = right up = mass left)
    for t in three:
        leg = t.lifting[0]
        assert (t.plan_roll < 0) == (leg in (LF, LB)), (t.tick, leg, t.plan_roll)


def test_trot_stays_level_on_diagonal_pairs():
    _, ticks = plan("trot")
    for t in ticks:
        assert t.lifting in ((), (LF, RB), (RF, LB))
        assert abs(t.pitch) < 0.5 and abs(t.roll) < 0.5
    pairs = [t.lifting for t in ticks if t.lifting]
    assert len(set(pairs)) == 2


def test_prediction_is_the_plan_through_the_servos_response():
    """The predicted attitude is the plan's (the plane of the feet commanded)
    a dead time earlier (2.5 ticks, linear between ticks), through a
    first-order response of 70 ms: the servo model fitted to the floor
    recordings of 2026-11-15 (before: a pure 150 ms delay)."""
    _, ticks = plan("crawl", 300)
    lag = M.lag_ticks()
    assert lag == 2.5 and M.servo_tau_ms == 70
    a = 1 - math.exp(-20 / 70)
    y = 0.0
    for i in range(300):
        u0, u1 = ticks[max(0, i - 2)].plan_roll if i >= 2 else 0.0, ticks[i - 3].plan_roll if i >= 3 else 0.0
        y += (u0 + (u1 - u0) * (lag - 2) - y) * a
        assert ticks[i].roll == pytest.approx(y, abs=1e-9), i


def test_speed_changes_mid_walk_are_smooth():
    """Down from the trot's default (144 mm/s) to 90 and 10 (its slowest, 42),
    and up again to its top: every joint inside the body's slew, feet on the
    ground slide at most the top speed's 3.7 mm a tick (plus the 1 mm
    rounding). (At its fixed cadence only the stride moves, 1.5 mm a tick.)"""
    p = Planner(M, "trot", tick=0)
    ticks = [p.next() for _ in range(200)]
    p.set_speed(90)
    ticks += [p.next() for _ in range(200)]
    p.set_speed(10)
    ticks += [p.next() for _ in range(300)]
    assert p.timing().speed_mm_s == pytest.approx(pat.speed_range("trot")[0])  # 42: the trot's slowest
    p.set_speed(250)
    ticks += [p.next() for _ in range(300)]
    assert max(joint_step_deg(a, b) for a, b in zip(ticks, ticks[1:])) < M.slew_deg_per_tick
    hi = pat.speed_range("trot")[1]
    assert max(max(abs(a.x[i] - b.x[i]) for i in LEGS if a.support >> i & 1 and b.support >> i & 1)
               for a, b in zip(ticks, ticks[1:])) <= hi * pat.TICK_MS / 1000 + 1  # feet on the ground never jump


def test_direction_and_pattern_changes():
    p = Planner(M, "trot", tick=0)
    for _ in range(100):
        p.next()
    p.set_direction("back")
    for _ in range(150):  # 1.5 mm a tick (0.5 before 2026-11-17): 37 ticks from +27.4 to -27.4
        p.next()
    assert p.s_cur == pytest.approx([-27.36, -27.36])
    p.set_direction("left")
    for _ in range(130):  # the right side from -27.4 to +21.9
        p.next()
    assert p.s_cur == pytest.approx([-21.888, 21.888])
    p.set_pattern("walk")
    assert p.pattern.name == "trot" and p.changing
    seen = []
    for _ in range(300):
        t = p.next()
        seen.append((p.pattern.name, t.lifting))
    first = next(i for i, (n, _) in enumerate(seen) if n == "walk")
    # trot to walk through the legs' way home (since 2026-11-22: the walk's four one-leg slots; the
    # trot's strides shrink 0.75 mm a tick): 0.7 s from this turn
    assert first * 20 < 800
    assert all(len(lf) <= 1 for n, lf in seen[first:] if n == "walk")
    assert p.s_cur == pytest.approx([-0.8 * 54, 0.8 * 54])  # turning left carried over, the walk's top stride
    # into the crawl (a leaning pattern): through the legs' way home, as before
    p.set_pattern("crawl")
    seen = []
    for _ in range(400):
        t = p.next()
        seen.append((p.pattern.name, t.lifting, list(t.x)))
    first = next(i for i, (n, _, _) in enumerate(seen) if n == "crawl")
    assert all(abs(x) <= 2 for x in seen[first][2])  # every foot home when the crawl starts
    assert all(len(lf) <= 1 for n, lf, _ in seen[first:] if n == "crawl")


def check_ticks(ticks, what=""):
    """Every tick inside FEET's ranges and the legs' reach, every joint
    under the body's 10 deg/tick slew from the tick before (until 2026-11-15
    the bound was 9.5 with the slower swings; the 1 mm rounding of FEET alone
    is up to 2 deg a tick at the knee)."""
    for t in ticks:
        assert all(-60 <= x <= 60 for x in t.x) and all(30 <= z <= 95 for z in t.z), (what, t)
        assert all(M.reachable(t.x[i], t.z[i]) for i in LEGS), (what, t)
        assert abs(t.pitch) <= 30 and abs(t.roll) <= 30
    worst = max(joint_step_deg(a, b) for a, b in zip(ticks, ticks[1:]))
    assert worst < M.slew_deg_per_tick, (what, worst)


@pytest.mark.parametrize("seq", [("walk", "trot", "crawl"), ("trot", "walk", "trot"), ("crawl", "walk", "trot"),
                                 ("walk", "crawl", "walk")])
def test_pattern_changes_one_after_another_stay_in_reach(seq):
    """Two pattern changes in a row (the remote's A3 / C4 / B4 pressed one
    after the other), at several points of the cycle and gaps between them."""
    a, b, c = seq
    for t0 in (100, 157, 211):
        for gap in (10, 60, 140):
            p = Planner(M, a, tick=0)
            ticks = [p.next() for _ in range(t0)]
            p.set_pattern(b)
            ticks += [p.next() for _ in range(gap)]
            p.set_pattern(c)
            ticks += [p.next() for _ in range(500)]
            check_ticks(ticks, (seq, t0, gap))
            assert p.pattern.name == c


def test_the_gate_pauses_the_feet():
    """While the gate holds the lift the plan waits: no foot slides toward the
    end of its range, so the next swing stays inside the slew and the reach
    (the trot at twice its speed slides 2.2 mm a tick)."""
    p = Planner(M, "trot", speed_mm_s=2 * pat.default_speed("trot"), tick=0)
    ticks = [p.next() for _ in range(40)]
    p.observe(6.0, 0.0)
    while p.stats["holds"] == 0:
        ticks.append(p.next())
    held = [ticks[-1]] + [p.next() for _ in range(24)]
    assert p.stats["holds"] == 25 and all(t.x == held[0].x and t.z == held[0].z and not t.lifting for t in held)
    ticks += held[1:]
    ticks.append(p.next())
    assert p.last.lifting  # hold_max: then it lifts anyway
    p.observe(0.0, 0.0)
    ticks += [p.next() for _ in range(100)]
    check_ticks(ticks)


def test_the_stance_time_is_slewed():
    """A speed change moves the feet's slide speed gradually (the stance time
    2 % a tick), not at once."""
    p = Planner(M, "crawl", speed_mm_s=10, tick=0)
    for _ in range(100):
        p.next()
    before = p.stance_ticks
    p.set_speed(pat.default_speed("crawl"))  # faster: a shorter stance
    p.next()
    assert p.stance_ticks == pytest.approx(before / 1.02)
    for _ in range(200):
        p.next()
    assert p.stance_ticks == pytest.approx(p.stance_tgt)


def test_the_predicted_attitude_stays_inside_feet_range():
    """Pitch and roll go out inside FEET's +-30 deg whatever the plan
    predicts (a bad model, a pose offset): the body refuses a FEET beyond it
    (BAD_ARG), which would end the walk."""
    from kilo.planner.walker import pack_feet
    p = Planner(M, "crawl", tick=0)
    p._lagged = lambda: (45.0, -31.0)
    for _ in range(60):  # through the servos' response (70 ms) to the bad prediction
        t = p.next()
    assert (t.pitch, t.roll) == (30.0, -30.0)
    f = P.Feet.unpack(pack_feet(1, replace(t, pitch=45.0, roll=-45.0)))
    assert (f.pitch, f.roll) == (480, -480)


def test_the_lean_is_solved_during_the_swing_before():
    """The walk's lean for each slot is solved a few units a tick during the
    swing before it, so no tick carries a whole solve, and in a steady walk
    no slot waits for it; a change mid-swing is solved again in time."""
    for name in ("crawl",):
        p = Planner(M, name, tick=0)
        for _ in range(1500):
            p.next()
        assert p.stats["waits"] == 0, name
    p = Planner(M, "crawl", tick=0)
    for i in range(1500):
        if i % 97 == 0:
            p.set_speed(5 + (i % 7))
        p.next()
    assert p.stats["waits"] < 40


def test_out_of_the_crawl_the_feet_wait_for_the_level_out():
    """Out of the crawl's lean into the walk the first slot's shift levels the
    body (about 20 ticks): the strides stay at 0 until it ends. With the
    walk's 1.5 mm a tick (1.0 since 2026-11-22) they would otherwise have slid every foot 30 mm
    back before its first lift, and the next swings asked up to 16.6 deg a
    tick (found by test_pattern_changes_one_after_another_stay_in_reach)."""
    p = Planner(M, "crawl", tick=0)
    for _ in range(200):
        p.next()
    p.set_pattern("walk")
    while p.pattern.name != "walk":
        p.next()
    assert p.levelling and p.S > 10
    while p.t < p.S:
        p.next()
        assert p.s_cur == [0.0, 0.0] and all(abs(x) < 1.5 for x in p.xg)  # the feet home, not sliding
    p.next()
    assert not p.levelling or p.t >= p.S


@pytest.mark.parametrize("name", pat.PATTERN_NAMES)
def test_the_stop_comes_home_level(name):
    p = Planner(M, name, tick=0)
    for _ in range(300):
        p.next()
    p.stop()
    n = 0
    while not p.done and n < 1000:
        last = p.next()
        n += 1
    assert p.done and n < 600
    assert all(abs(x) <= 1 for x in last.x) and all(abs(z - H) <= 1 for z in last.z)
    assert abs(last.plan_roll) < 0.5 and abs(last.plan_pitch) < 0.5 and last.support == 0xF


def test_gate_holds_the_lift_and_catch_lands_the_foot():
    p = Planner(M, "crawl", tick=0)
    p.observe(0.0, 8.0)  # the body is 8 deg off the plan: no lift
    ticks = [p.next() for _ in range(40)]
    assert not any(t.lifting for t in ticks) and p.stats["holds"] > 0
    p.observe(0.0, 0.0)
    while not p.last.lifting:
        p.next()
    leg = p.last.lifting[0]  # LB: tipping toward it is + roll (left side down) and - pitch (tail down)
    assert leg == LB
    p.next()
    p.observe(-3.0, 6.0)
    down = [p.next() for _ in range(4)]
    assert p.stats["catches"] == 1
    assert not down[-1].lifting and down[-1].support == 0xF  # down in three ticks, where it was


def test_the_walk_catches_its_swinging_leg():
    """In the walk (one leg at a time) the catch lands the leg the body tips
    toward where it is, in three ticks, and the walk goes on with the next leg."""
    p = Planner(M, "walk", tick=0)
    for _ in range(100):
        p.next()
    while p.last is None or p.last.lifting != (LB,):
        p.next()
    p.next()
    p.observe(0.0, 8.0)  # tipping left: toward LB
    down = [p.next() for _ in range(4)]
    p.observe(0.0, 0.0)
    assert p.stats["catches"] == 1
    assert not down[-1].lifting and down[-1].support == 0xF  # LB down in three ticks, where it was
    rest = [p.next() for _ in range(30)]
    check_ticks(down + rest)
    assert any(t.lifting == (LF,) for t in rest)  # and the walk goes on with the next leg

# ---- the error estimates ----------------------------------------------------------------------------

def _tel(tick, meas, ref, corr=(0.0, 0.0)):
    return P.Telemetry(tick=tick & 0xFFFF, pitch=round(meas[0] * 16), roll=round(meas[1] * 16),
                       ref_pitch=round(ref[0] * 16), ref_roll=round(ref[1] * 16),
                       corr_pitch=round(corr[0] * 16), corr_roll=round(corr[1] * 16), mode=P.Mode.GAIT)


def test_error_offset_lag_and_spike():
    tr = errors.ErrorTracker()
    for t in range(300):
        ref = 8 * math.sin(2 * math.pi * t / 75)
        meas = 8 * math.sin(2 * math.pi * (t - 2) / 75) + 1.5  # 2 ticks (40 ms) late, 1.5 deg off
        tr.add(t, _tel(t, (0.4, meas), (0.0, ref), (0.0, -0.5)))
    e = tr.estimate()
    assert e["lag_ms"] == 40 and e["lag_quality"] > 0.95
    assert e["offset_roll_deg"] == pytest.approx(1.5, abs=0.2) and e["offset_pitch_deg"] == pytest.approx(0.4, abs=0.05)
    assert e["corr_roll_deg"] == pytest.approx(-0.5, abs=0.05)
    assert tr.offset[0] == pytest.approx(0.4, abs=0.05)
    assert errors.ErrorTracker().estimate() is None
    # a plan that does not move (the trot) gives no lag
    flat = errors.ErrorTracker()
    for t in range(100):
        flat.add(t, _tel(t, (0.0, 0.3), (0.0, 0.0)))
    assert flat.estimate()["lag_ms"] is None
    # from a telemetry history: the newest run in GAIT, ticks unwrapped
    hist = [(t * 0.02, _tel(65500 + t, (0, 8 * math.sin(t / 10)), (0, 8 * math.sin(t / 10)))) for t in range(120)]
    assert errors.from_history(hist, int(P.Mode.GAIT))["lag_ms"] == 0


def test_a_heavier_side_shifts_the_lean():
    """The lean comes from the model's masses: with the battery on the right
    the body needs less roll to the right before a left leg lifts."""
    right = replace(M, battery=Mass(150.0, 0.0, -12.0, -15.0))
    _, a = plan("crawl", 200)
    b = Planner(right, "crawl", tick=1000)
    b = [b.next() for _ in range(200)]
    assert min(t.plan_roll for t in b) > min(t.plan_roll for t in a)
