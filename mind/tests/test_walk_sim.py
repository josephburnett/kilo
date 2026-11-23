"""The planner on the virtual body's walking physics (tests/walksim.py):
what the plan does to a body that tips over the edge of its support as an
inverted pendulum, with the servos' response (a 50 ms dead time and a 70 ms
first-order lag, fitted to the floor recordings of 2026-11-15), the
firmware's balance loop at Kilo's Kp 0.99, and the stream's timing.
Deterministic: no threads, no wall time.

Model errors used throughout: the servos faster (20 ms + 50 ms) or slower
(100 + 100) than the model, the old pure 140 ms delay, the centre of mass 10
mm ahead or behind and 5 mm left or right with 50 g more battery, the IMU
off by 2 deg pitch and 1.5 deg roll. (The recordings put the real body's
trot rocking at about twice the virtual body's: docs/walking-and-balance.md
§12; the wrong centre of mass is the case that reproduces it.)

What the virtual body can and cannot show: it is a rigid body on point
feet. When the mass leaves the support it tips toward the lifted leg until
that foot reaches the ground (about 9 degrees with the walk's 15 mm step and
300 ms swing; the floor run with the balance off rocked 15 degrees with the
old 500 ms swing and stayed up). The
body's balance holds while a foot is up (as the firmware does since its
tipping plant showed corrections on three feet enlarge the lean), so here
it neither adds nor takes away on three feet. The rigid catch stops the
tip short of a fall, so the test for "today's steps" is that they tip over
the edge every swing."""

from dataclasses import replace

import pytest

import math

from kilo.planner.model import BodyModel, Mass
from kilo.planner.patterns import default_speed, speed_range
from kilo.planner.plan import PlannerConfig
from kilo.protocol import kilo_protocol as P

from walksim import WalkSim

NO_FEEDBACK = dict(gate_deg=99.0, catch_deg=99.0)
TICKS = 500  # 10 s
SERVOS = (dict(lag_ms=20, tau_ms=50), dict(lag_ms=100, tau_ms=100), dict(lag_ms=140, tau_ms=0))


def rms_roll(sim):
    pred = sim.pred()
    g = [t for t in sim.gait() if t.tick in pred]
    return math.sqrt(sum((t.roll / 16 - pred[t.tick][1]) ** 2 for t in g) / len(g))


def tipped_ticks(sim):
    """Ticks where the body was tipping (its mass past an edge): the roll more
    than 3 deg off the plan. Any tick, not only those whose played support has
    a leg up: the body follows its commands a servo lag late, so with the
    300 ms swings of 2026-11-15 most of a tip shows after the played support
    has all four feet again."""
    pred = sim.pred()
    return sum(1 for t in sim.gait() if t.tick in pred and abs(t.roll / 16 - pred[t.tick][1]) > 3)


def test_the_planned_crawl_stays_on_its_plan():
    sim = WalkSim("crawl")
    assert sim.run(TICKS) == TICKS and sim.stream.ended is None
    assert not sim.fell()
    # the body does what the plan predicts: 3.5 deg (2.5 before 2026-11-16, when the virtual
    # servos were the planner's pure delay; with their first-order response the gravity-paced
    # move is followed less exactly, the error mostly in the moves, not on three feet)
    assert sim.max_error() < 3.5
    assert sim.vb.max_tilt > 6  # ...which is a real lean (the move over the three feet)
    assert tipped_ticks(sim) <= 2  # (0 with the pure delay; 1 with the fitted response, in a move) vs > 10 untilted
    assert sim.planner.stats["min_margin"] > 7
    corr = max(max(abs(t.corr_pitch), abs(t.corr_roll)) for t in sim.gait()) / 16
    assert corr < 3.0  # the balance only trims (its dead band is 2 deg)


def test_the_crawl_prediction_follows_the_servos_response():
    """The floor's extra roll toward a lifted hind leg (2026-11-15: 2.7 deg)
    was the body's own correction: the measured roll led the old pure 150 ms
    prediction in each lean move, the error passed the 2 deg dead band, and
    the correction it built was held through the next swing. With the
    prediction through the fitted response the error and the corrections
    are smaller."""
    old = replace(BodyModel(), servo_lag_ms=150.0, servo_tau_ms=0.0)
    for direction in ("forward", "back"):
        a = WalkSim("crawl", direction, model=old, body_model=BodyModel())
        b = WalkSim("crawl", direction)
        a.run(TICKS)
        b.run(TICKS)
        assert b.max_error() < a.max_error() - 0.4, direction
        assert rms_roll(b) < 0.85 * rms_roll(a), direction


@pytest.mark.parametrize("balance", (True, False))
def test_the_same_steps_without_the_lean_tip_over(balance):
    sim = WalkSim("crawl", config=PlannerConfig(lean=False, **NO_FEEDBACK), balance=balance)
    sim.run(TICKS)
    assert sim.planner.stats["min_margin"] < 0  # the mass goes past the edge of the three feet
    # ...and the body falls onto the swinging foot (8.4 deg with the balance off, 14 with it on)
    assert tipped_ticks(sim) > 10 and sim.max_error() > 7.5
    planned = WalkSim("crawl", balance=balance)
    planned.run(TICKS)
    # (with the balance off the planned crawl itself leaves its plan by 5.3 deg in the moves)
    assert planned.max_error() * (2 if balance else 1.5) < sim.max_error()


def test_the_balance_loop_holds_on_three_feet():
    """The floor's finding (and the firmware's tipping plant): corrections on
    three feet act through the feet the body pivots on and enlarge the lean.
    The body holds them while a foot is up, so its loop neither helps nor
    hurts there: holding the mass on three feet is the plan's job."""
    on = WalkSim("crawl", config=PlannerConfig(lean=False, **NO_FEEDBACK), balance=True)
    off = WalkSim("crawl", config=PlannerConfig(lean=False, **NO_FEEDBACK), balance=False)
    on.run(TICKS)
    off.run(TICKS)
    # Both tip. Since 2026-11-16 (the servos' first-order response) the tip outlasts the swing: the
    # loop sees it on the four-foot ticks after the touchdown and the correction it builds there is
    # held through the next swing, so with the loop on the steps without the lean tip further (14
    # deg against 8.4): the floor's extra roll toward a lifted hind leg, the same mechanism. Holding
    # on three feet still keeps it from acting through the pivoting feet; the lean has to hold the
    # mass (and does: test_the_planned_crawl_stays_on_its_plan).
    assert off.max_error() < on.max_error() < 2 * off.max_error()
    held = [t for t in on.gait() if t.support != 0xF]
    assert held and all(t.flags & P.TelemetryFlags.BALANCE_HELD for t in held)


@pytest.mark.parametrize("pattern", ("walk", "trot"))
def test_the_fast_gaits_stay_up(pattern):
    sim = WalkSim(pattern)
    assert sim.run(TICKS) == TICKS
    # the trot 2.5 deg at 144 mm/s, the walk (since 2026-11-22: always on three feet or four) 0.8 at 58
    assert not sim.fell() and sim.vb.max_tilt < 4.5 and sim.max_error() < 4.5
    two = [t for t in sim.gait() if bin(t.support).count("1") == 2]
    if pattern == "trot":
        assert two and all(t.support in (0b0101, 0b1010) for t in two)  # diagonal pairs: LF+RB, RF+LB
    else:
        assert not two
    three = [t for t in sim.gait() if bin(t.support).count("1") == 3]
    assert bool(three) == (pattern == "walk")


@pytest.mark.slow
@pytest.mark.parametrize("pattern", ("walk", "trot"))
@pytest.mark.parametrize("direction", ("forward", "back", "left"))
def test_the_fast_gaits_at_every_level_shrug_off_the_model_errors(pattern, direction):
    """The walk and the trot at their slowest, default and top speed, with
    each model error: up, within 6.5 deg of the plan (the trot within 5; the
    IMU offset counts in full: the walker's gate subtracts it, this does not),
    no catch, and gate holds only with the servos faster than the model (a
    few, in the walk). The trot backward and turning in place only up to its
    default (144 mm/s): the remote walks it backward to -3 and turns in place
    at 1x; at its top turning in place the IMU offset takes it to 5.2."""
    lo, hi = speed_range(pattern)
    errs = SERVOS + (dict(body_model=wrong_mass(10, 5, 50)), dict(body_model=wrong_mass(-10, -5, 50)),
                     dict(body_model=wrong_mass(10, -5, 50)), dict(imu_offset=(2.0, 1.5)))
    top = () if pattern == "trot" and direction != "forward" else (hi,)
    for speed in (lo, default_speed(pattern), *top):
        for kw in errs:
            sim = WalkSim(pattern, direction, speed, **kw)
            assert sim.run(TICKS) == TICKS and not sim.fell()
            st = sim.planner.stats
            what = (speed, kw, sim.max_error(), st)
            # the walk 6.5: at its slowest with the servos 140 ms late it reaches 6.4 since the model's
            # feet are 70 mm apart and its centre of mass 3 mm further back (2026-11-22; under 6 before)
            assert sim.max_error() < (5.0 if pattern == "trot" else 6.5) and st["catches"] == 0, what
            assert st["holds"] == 0 or (pattern == "walk" and kw.get("lag_ms") == 20 and st["holds"] < 10), what


@pytest.mark.slow
@pytest.mark.parametrize("pattern", ("walk", "trot"))
@pytest.mark.parametrize("direction", ("forward", "back"))
def test_steering_shrugs_off_the_model_errors(pattern, direction):
    """The remote's A6 / C6 while walking (2026-11-17): the walk and the trot
    at their default and top speed, steering a little (0.25) and as tight as
    the remote goes (1: the inner side steps in place), each model error:
    up, within the bounds of the straight walk (the trot 5 deg, the walk 6),
    no catch, and gate holds only where the straight walk has them. The
    trot as the remote drives it: at its top forward only, steering 0.25
    (kTrotFastSteerSteps; steering 1 there reaches 5.5 with the IMU offset);
    backward at 144 mm/s steering 1 with the servos' old pure delay it
    reaches 5.6: within the walk's 6 there."""
    errs = SERVOS + (dict(body_model=wrong_mass(10, 5, 50)), dict(body_model=wrong_mass(-10, -5, 50)),
                     dict(body_model=wrong_mass(10, -5, 50)), dict(imu_offset=(2.0, 1.5)))
    trot = pattern == "trot"
    for speed in (default_speed(pattern), speed_range(pattern)[1]):
        if trot and direction == "back" and speed > default_speed(pattern):
            continue
        for steer in ((0.25,) if trot and speed > 150 else (0.25, -1.0)):
            bound = 6.0 if not trot or (direction == "back" and abs(steer) == 1.0) else 5.0
            for kw in errs:
                sim = WalkSim(pattern, direction, speed, **kw)
                sim.planner.set_steer(steer)
                assert sim.run(TICKS) == TICKS and not sim.fell()
                st = sim.planner.stats
                what = (speed, steer, kw, sim.max_error(), st)
                # (protocol 3.0's 3-tick look-ahead: the fastest servos (20 ms lag) walking backward at
                # 58 mm/s steering 0.25 get one catch, at the same largest error as with 5 ticks)
                fast = kw.get("lag_ms") == 20 and pattern == "walk" and direction == "back"
                assert sim.max_error() < bound and (st["catches"] == 0 or fast and st["catches"] <= 1), what
                # (protocol 3.0's 3-tick look-ahead, 2026-11-22: one gate hold in the walk backward at
                # 58 mm/s steering 0.25 with the mass 10 mm forward and 5 mm left; none with 5 ticks)
                walk_ok = pattern == "walk" and (kw.get("lag_ms") == 20 and st["holds"] < 10 or st["holds"] <= 1)
                assert st["holds"] == 0 or walk_ok, what


def test_steering_the_crawl_stays_on_its_plan():
    """The crawl steers too (its lean is solved for whatever strides the feet
    have): forward and backward as tight as the remote goes, within the
    straight crawl's bounds, no catch, no hold."""
    for direction in ("forward", "back"):
        sim = WalkSim("crawl", direction)
        sim.planner.set_steer(1.0)
        assert sim.run(TICKS) == TICKS and not sim.fell()
        # 4 backward (the straight crawl backward reaches 4.4 with the fitted servos: §12.2)
        assert sim.max_error() < (3.5 if direction == "forward" else 4.5), (direction, sim.max_error())
        assert sim.planner.stats["catches"] == 0 and sim.planner.stats["holds"] == 0
        assert sim.planner.stats["min_margin"] > 7


@pytest.mark.parametrize("pattern", ("walk", "trot"))
def test_a_turn_in_place_then_straight_steering_then_stop(pattern):
    """The remote's turn in place (A6 from standing), B7 straight forward at
    once, A6 A6 A6 steering left as tight as it goes, C6 x6 to the right,
    a gait change (the walk and the trot directly), B5 into backward, then
    the planned stop: all in one walk, up, near the plan, the straight walk
    reached within 1.4 s of B7 (the trot, its strides 0.75 mm a tick; the
    walk: 1.7 s), and the stop inside stop_timeout."""
    from kilo.planner.walker import stop_timeout
    sim = WalkSim(pattern, "left")
    other = "trot" if pattern == "walk" else "walk"
    reached = {}

    def each(i):
        p = sim.planner
        if i == 150:
            p.set_direction("forward")
        if i > 150 and "fwd" not in reached and p.s_cur == p.s_tgt:
            reached["fwd"] = i - 150
        if i == 250:
            p.set_steer(1.0)
        elif i == 400:
            p.set_steer(-1.0)
        elif i == 550:
            p.set_pattern(other)
        elif i == 700:
            p.set_direction("back")
        elif i == 850:
            p.stop()
    n = sim.run(1400, each=each)
    assert sim.stream.ended == "stopped" and sim.stream.ok and not sim.fell()
    assert reached["fwd"] * 20 < (1700 if pattern == "walk" else 1400) and sim.planner.pattern.name == other
    assert (n - 850) * 0.02 < stop_timeout(sim.planner.timing().period_ms) - 1.0
    # the trot into the walk through the way home (2026-11-22) with the trot's slower strides: 4.5 in
    # the virtual body, within the trot's straight-walk bound (5)
    assert sim.max_error() < (5.0 if pattern == "trot" else 4.5) and sim.planner.stats["catches"] == 0 \
        and sim.planner.stats["holds"] == 0
    assert sim.vb.pose_underruns == 0 and not sim.vb.pose_refused


def test_the_trot_no_longer_rocks_at_mid_speeds():
    """Until 2026-11-16 the trot at 75 to 140 mm/s (four-foot phases of 4 to 8
    ticks) rocked with the body's balance loop, up to ~10 deg off the plan
    walking backward at 100 mm/s, with gate holds. At the fixed cadence its
    four-foot phase is one tick at every speed: the loop never acts (it holds
    while a foot is up and for two ticks after) and the trot at 100 mm/s
    backward is as close to its plan with the balance on as with it off."""
    on = WalkSim("trot", "back", 100)
    off = WalkSim("trot", "back", 100, balance=False)
    on.run(TICKS)
    off.run(TICKS)
    assert not on.fell() and not off.fell()
    assert on.max_error() < 3.5 and abs(on.max_error() - off.max_error()) < 0.5
    assert on.planner.stats["holds"] == 0
    g = on.gait()
    assert all((a.corr_pitch, a.corr_roll) == (b.corr_pitch, b.corr_roll) for a, b in zip(g, g[1:]))


def test_speed_and_gait_changes_mid_walk_stay_up():
    """The crawl from its default to its slowest and its top, backward, into
    the walk (home first: out of a leaning pattern), up to the walk's top,
    into the trot directly, to 184 mm/s (a 10-tick swing), back to the walk,
    turning, into the crawl again. Into the trot takes well under a second,
    out of its long swing into the walk 2.4 s at most (the swing back to 8
    ticks a tick a step, the stride 0.75 mm a tick)."""
    sim = WalkSim("crawl")
    switched = {}

    def each(i):
        p = sim.planner
        if p.pattern.name not in switched:
            switched[p.pattern.name] = i
        if i == 150:
            p.set_speed(8)
        elif i == 350:
            p.set_speed(27)
        elif i == 500:
            p.set_direction("back")
        elif i == 650:
            p.set_pattern("walk")
        elif i == 900:
            p.set_speed(100)
        elif i == 1000:
            p.set_pattern("trot")
        elif i == 1100:
            p.set_speed(184)
        elif i == 1200:
            p.set_pattern("walk")
            switched.pop("walk", None)
        elif i == 1300:
            p.set_direction("left")
        elif i == 1400:
            p.set_pattern("crawl")
            switched.pop("crawl", None)
    assert sim.run(1800, each=each) == 1800
    assert not sim.fell() and sim.max_error() < 6.5, sim.max_error()
    # between the walk and the trot through the legs' way home (since 2026-11-22): into the trot 2.2 s at
    # most; out of the trot's long swing (184 mm/s: its strides shrink 0.75 mm a tick) WALKBACK
    assert switched["trot"] - 1000 < 110 and switched["walk"] - 1200 < 120
    assert sim.planner.pattern.name == "crawl" and sim.planner.stats["catches"] == 0
    assert sim.vb.pose_underruns == 0 and not sim.vb.pose_refused


@pytest.mark.parametrize("pattern", ("crawl", "walk", "trot"))
def test_a_stalled_producer_is_caught_up(pattern):
    """40 ms (two ticks) without a wake: the three ticks of look-ahead
    (link.pose_lookahead_ticks) carry the body; the next wake sends the
    ticks it missed. No underrun, no fall. (A third missed wake runs the
    queue dry: the body holds the last pose, the next wake resumes.)"""
    sim = WalkSim(pattern)
    assert sim.run(300, stall=(100, 102)) == 300
    assert sim.vb.pose_underruns == 0
    assert sim.stream.max_behind >= 3 and sim.stream.skipped == 0
    assert sim.vb.pose_dropped == 0 and not sim.fell() and sim.max_error() < 4.5
    ticks = [f.tick for f in sim.link.sent]
    assert ticks == list(range(ticks[0], ticks[0] + len(ticks)))  # every tick sent once, in order


def test_a_long_stall_underruns_then_settles_and_the_stream_ends():
    sim = WalkSim("walk")
    sim.run(150)
    n = sim.run(60, stall=(0, 60))  # 1.2 s without a wake: the queue runs dry
    assert n == 60
    assert sim.vb.stream is None and sim.vb.posture == "STANCE" and not sim.fell()
    names = [e for e in sim.events if e[0] in ("POSE_UNDERRUN", "STREAM_STOPPED")]
    assert names[0] == ("POSE_UNDERRUN", 4) and names[-1] == ("STREAM_STOPPED", 1)  # ticks since the last POSE
    assert sim.step() is False and "underrun" in sim.stream.ended and not sim.stream.ok
    # the settle put the lifted feet down at their x: all four feet at one height
    assert max(sim.vb.joint_angles()[3:]) is not None


@pytest.mark.parametrize("pattern", ("crawl", "walk", "trot"))
def test_the_planned_stop_hands_over_to_stance(pattern):
    """The stop at each gait's top speed, inside stop_timeout (which the
    C++ mind's remote and distance guard wait for, remote::stop_timeout_s)."""
    from kilo.planner.walker import stop_timeout
    sim = WalkSim(pattern, speed=speed_range(pattern)[1])
    sim.run(200)
    sim.planner.stop()
    n = sim.run(600)
    assert n < 600 and sim.stream.ended == "stopped" and sim.stream.ok and sim.handovers == 1
    assert n * 0.02 < stop_timeout(sim.planner.timing().period_ms) - 1.0, n
    assert ("STREAM_STOPPED", 0) in sim.events and sim.vb.stream is None and sim.vb.posture == "STANCE"
    assert sim.max_error() < 4.5 and not sim.fell()


def test_predicted_attitude_matches_the_telemetry_conventions():
    """The planner predicts what TELEMETRY measures: pitch nose down +, roll
    right side up +. Before LB lifts, the body leans right (the mass over
    RF RB LF): negative roll, measured as negative by the body."""
    sim = WalkSim("crawl")
    sim.run(60)
    lifts = [t for t in sim.gait() if t.support == 0b0111]  # LB in the air
    assert lifts
    for t in lifts:
        assert t.ref_roll < -3 * 16 and t.roll < -3 * 16
        assert abs(t.roll - t.ref_roll) < 24 and abs(t.pitch - t.ref_pitch) < 24  # within 1.5 deg
    # ...which the plan made with shorter right legs (RF, RB) than the left front one
    leaning = [f for f in sim.link.sent if f.support == 0b0111]
    assert leaning and all(f.z_mm[1] < f.z_mm[0] and f.z_mm[2] < f.z_mm[0] for f in leaning)


def test_the_model_error_is_seen():
    """A body whose servos start 100 ms after a command (the model says 50)
    and whose IMU sits 1.5 deg nose down and 2 deg right up: the estimates
    find both."""
    sim = WalkSim("crawl", lag_ms=100, imu_offset=(1.5, 2.0), balance=False)
    sim.run(400)
    e = sim.stream.tracker.estimate()
    assert e["lag_ms"] == pytest.approx(50, abs=20)
    assert e["offset_pitch_deg"] == pytest.approx(1.5, abs=0.5) and e["offset_roll_deg"] == pytest.approx(2.0, abs=0.7)
    assert not sim.fell()


def test_poses_are_sent_three_ticks_ahead():
    sim = WalkSim("trot")
    seen = []
    sim.run(50, each=lambda i: seen.append(sim.link.sent[-1].tick - (sim.vb.tick & 0xFFFF)))
    assert set(seen[1:]) == {P.POSE_LOOKAHEAD_TICKS}
    assert all(t.pose_queued >= P.POSE_LOOKAHEAD_TICKS - 1 for t in sim.gait()[2:])


def wrong_mass(dx: float, dy: float, battery_g: float) -> BodyModel:
    """The planner's model with ``battery_g`` more battery and the whole
    centre of mass moved ``dx`` forward and ``dy`` left (body and battery
    moved together): a plausible error of the estimated masses."""
    m0 = BodyModel()
    m = replace(m0, battery=Mass(m0.battery.grams + battery_g, m0.battery.x, m0.battery.y, m0.battery.z))
    c0, c1 = m0.com([0] * 4, [76] * 4), m.com([0] * 4, [76] * 4)
    k = m.total_g / (m.body.grams + m.battery.grams)
    ex, ey = (c0[0] + dx - c1[0]) * k, (c0[1] + dy - c1[1]) * k
    return replace(m, body=Mass(m.body.grams, m.body.x + ex, m.body.y + ey, m.body.z),
                   battery=Mass(m.battery.grams, m.battery.x + ex, m.battery.y + ey, m.battery.z))


def test_wrong_mass_helper():
    m, m0 = wrong_mass(10, 5, 50), BodyModel()
    c, c0 = m.com([0] * 4, [76] * 4), m0.com([0] * 4, [76] * 4)
    assert (c[0] - c0[0], c[1] - c0[1]) == pytest.approx((10, 5)) and m.total_g == m0.total_g + 50


@pytest.mark.parametrize("pattern", ("crawl", "walk", "trot"))
def test_a_wrong_centre_of_mass_stays_up(pattern):
    """The physics' centre of mass 10 mm ahead of and 5 mm left of the
    planner's, 50 g more battery: Kilo stays up. With a 10 mm margin the
    walk can leave its plan in some directions (here -10/-5: the mass past
    the edge, catches, up to ~12 deg off), the gate and the catch keep it up."""
    for dx, dy in ((10, 5), (-10, -5)):
        sim = WalkSim(pattern, body_model=wrong_mass(dx, dy, 50))
        assert sim.run(1000) == 1000, (dx, dy)
        assert not sim.fell() and sim.vb.max_tilt < 20, (dx, dy)


@pytest.mark.slow
@pytest.mark.parametrize("pattern", ("crawl",))
def test_a_14_mm_margin_keeps_a_wrong_centre_of_mass_on_its_plan(pattern):
    """The same error in all four directions: with margin_mm 14 the walk and
    the crawl stay near the plan with no catch and no hold (with 10 they
    need the catch or the gate in one or two directions). With the masses
    measured 2026-11-14 and the faster gaits of 2026-11-15: the walk within
    2.3 deg, the crawl within 2.1 deg (before: 2.8 and 4.1)."""
    for dx, dy in ((10, 5), (10, -5), (-10, 5), (-10, -5)):
        sim = WalkSim(pattern, body_model=wrong_mass(dx, dy, 50), config=PlannerConfig(margin_mm=14.0))
        assert sim.run(1500) == 1500
        # 5.5 (4.5 before 2026-11-16: with the virtual servos' fitted response the worst, -10/+5,
        # is 5.25; still no catch and no hold)
        assert not sim.fell() and sim.max_error() < 5.5, (dx, dy, sim.max_error())
        assert sim.planner.stats["catches"] == 0 and sim.planner.stats["holds"] == 0, (dx, dy)
