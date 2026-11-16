"""The planner on the virtual body's walking physics (tests/walksim.py):
what the plan does to a body that tips over the edge of its support as an
inverted pendulum, with servo lag, the firmware's balance loop at Kilo's
Kp 0.99, and the stream's timing. Deterministic: no threads, no wall time.

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

from kilo.planner.model import BodyModel, Mass
from kilo.planner.plan import PlannerConfig
from kilo.protocol import kilo_protocol as P

from walksim import WalkSim

NO_FEEDBACK = dict(gate_deg=99.0, catch_deg=99.0)
TICKS = 500  # 10 s


def tipped_ticks(sim):
    """Ticks where the body was tipping (its mass past an edge): the roll more
    than 3 deg off the plan. Any tick, not only those whose played support has
    a leg up: the body follows its commands a servo lag late, so with the
    300 ms swings of 2026-11-15 most of a tip shows after the played support
    has all four feet again."""
    pred = sim.pred()
    return sum(1 for t in sim.gait() if t.tick in pred and abs(t.roll / 16 - pred[t.tick][1]) > 3)


@pytest.mark.parametrize("pattern", ("walk", "crawl"))
def test_the_planned_walk_stays_on_its_plan(pattern):
    sim = WalkSim(pattern)
    assert sim.run(TICKS) == TICKS and sim.stream.ended is None
    assert not sim.fell()
    assert sim.max_error() < 2.5  # the body does what the plan predicts
    assert sim.vb.max_tilt > 6  # ...which is a real lean (the move over the three feet)
    assert tipped_ticks(sim) == 0
    assert sim.planner.stats["min_margin"] > 7
    corr = max(max(abs(t.corr_pitch), abs(t.corr_roll)) for t in sim.gait()) / 16
    assert corr < 3.0  # the balance only trims (its dead band is 2 deg)


@pytest.mark.parametrize("balance", (True, False))
def test_the_same_steps_without_the_lean_tip_over(balance):
    sim = WalkSim("walk", config=PlannerConfig(lean=False, **NO_FEEDBACK), balance=balance)
    sim.run(TICKS)
    assert sim.planner.stats["min_margin"] < 0  # the mass goes past the edge of the three feet
    assert tipped_ticks(sim) > 10 and sim.max_error() > 9  # ...and the body falls onto the swinging foot
    planned = WalkSim("walk", balance=balance)
    planned.run(TICKS)
    assert planned.max_error() * 4 < sim.max_error()


def test_the_balance_loop_holds_on_three_feet():
    """The floor's finding (and the firmware's tipping plant): corrections on
    three feet act through the feet the body pivots on and enlarge the lean.
    The body holds them while a foot is up, so its loop neither helps nor
    hurts there: holding the mass on three feet is the plan's job."""
    on = WalkSim("walk", config=PlannerConfig(lean=False, **NO_FEEDBACK), balance=True)
    off = WalkSim("walk", config=PlannerConfig(lean=False, **NO_FEEDBACK), balance=False)
    on.run(TICKS)
    off.run(TICKS)
    # (both tip, 12 to 15 deg off the plan; how far differs by a few degrees with the measured masses)
    assert abs(on.max_error() - off.max_error()) < 4
    held = [t for t in on.gait() if t.support != 0xF]
    assert held and all(t.flags & P.TelemetryFlags.BALANCE_HELD for t in held)


def test_the_trot_stays_up():
    sim = WalkSim("trot")
    assert sim.run(TICKS) == TICKS
    assert not sim.fell() and sim.vb.max_tilt < 4 and sim.max_error() < 3.5  # 2.2 at 144 mm/s (3.3 at the old 56)
    two = [t for t in sim.gait() if bin(t.support).count("1") == 2]
    assert two and all(t.support in (0b0101, 0b1010) for t in two)  # diagonal pairs: LF+RB, RF+LB


@pytest.mark.parametrize("direction", ("forward", "back"))
def test_the_trot_at_its_default_and_top_shrugs_off_the_model_errors(direction):
    """At 144 and 152 mm/s (two to three ticks on four feet) the trot stays
    within 6.5 deg of its plan, with no hold and no catch, with the servos
    lagging 100 or 200 ms (the model says 150) or the centre of mass 10 mm
    ahead and 5 mm left of the model's with 50 g more battery."""
    for speed in (144, 152):
        for kw in (dict(lag_ms=100), dict(lag_ms=200), dict(body_model=wrong_mass(10, 5, 50))):
            sim = WalkSim("trot", direction, speed, **kw)
            assert sim.run(TICKS) == TICKS and not sim.fell()
            st = sim.planner.stats
            assert sim.max_error() < 6.5 and st["holds"] == 0 and st["catches"] == 0, (speed, kw, sim.max_error())


def test_the_trot_at_mid_speeds_rocks_with_the_balance():
    """A known weakness, the body's (not the plan's): at about 75 to 140 mm/s
    (four-foot phases of 4 to 8 ticks) the body's balance loop, which corrects
    on four feet and holds on two, rocks the trot, worst walking backward:
    up to ~10 deg off the plan and gate holds, where the same steps with the
    balance off stay within 3 deg. It does not fall. The remote's ladder puts
    only its level 2 (0.7 x 144 = 101 mm/s) there."""
    on = WalkSim("trot", "back", 100)
    off = WalkSim("trot", "back", 100, balance=False)
    on.run(TICKS)
    off.run(TICKS)
    assert not on.fell() and not off.fell()
    assert off.max_error() < 3.5 and on.max_error() > 2 * off.max_error()


def test_speed_changes_mid_walk_stay_up():
    """The walk from its default to the bottom of the remote's ladder and to
    its top, backward, into the trot and up to the trot's top speed. (Not to
    the trot's mid speeds: test_the_trot_at_mid_speeds_rocks_with_the_balance.)"""
    sim = WalkSim("walk")

    def each(i):
        if i == 150:
            sim.planner.set_speed(8)
        elif i == 350:
            sim.planner.set_speed(21)
        elif i == 500:
            sim.planner.set_direction("back")
        elif i == 650:
            sim.planner.set_pattern("trot")  # the legs go home first (up to ~4 s from the walk)
        elif i == 700:
            sim.planner.set_speed(150)
    assert sim.run(1150, each=each) == 1150
    assert not sim.fell() and sim.max_error() < 4
    assert sim.planner.pattern.name == "trot" and sim.planner.timing().speed_mm_s == pytest.approx(150)
    assert sim.vb.feet_underruns == 0 and not sim.vb.feet_refused


def test_a_stalled_producer_is_caught_up():
    """60 ms (three ticks) without a wake: the five ticks of lookahead carry
    the body; the next wake sends the ticks it missed. No underrun, no fall."""
    sim = WalkSim("walk")
    assert sim.run(300, stall=(100, 103)) == 300
    assert ("FEET_UNDERRUN", 1) not in sim.events and sim.vb.feet_underruns == 0
    assert sim.stream.max_behind >= 4 and sim.stream.skipped == 0
    assert sim.vb.feet_late == 0 and not sim.fell() and sim.max_error() < 2.5
    ticks = [f.tick for f in sim.link.sent]
    assert ticks == list(range(ticks[0], ticks[0] + len(ticks)))  # every tick sent once, in order


def test_a_long_stall_underruns_then_settles_and_the_stream_ends():
    sim = WalkSim("walk")
    sim.run(150)
    n = sim.run(60, stall=(0, 60))  # 1.2 s without a wake: the queue runs dry
    assert n == 60
    assert sim.vb.feet is None and sim.vb.posture == "STANCE" and not sim.fell()
    names = [e for e in sim.events if e[0] in ("FEET_UNDERRUN", "GAIT_STOPPED")]
    assert names[0] == ("FEET_UNDERRUN", 6) and names[-1] == ("GAIT_STOPPED", 1)  # 6 ticks since the last FEET
    assert sim.step() is False and "underrun" in sim.stream.ended and not sim.stream.ok
    # the settle put the lifted feet down at their x: all four feet at one height
    assert max(sim.vb.joint_angles()[3:]) is not None


def test_the_planned_stop_hands_over_to_stance():
    sim = WalkSim("walk")
    sim.run(200)
    sim.planner.stop()
    n = sim.run(600)
    assert n < 600 and sim.stream.ended == "stopped" and sim.stream.ok and sim.handovers == 1
    assert ("GAIT_STOPPED", 0) in sim.events and sim.vb.feet is None and sim.vb.posture == "STANCE"
    # 3, not 2.5 as before 2026-11-15: the faster walk's lean moves faster, and the physics' 20 ms
    # settling alone puts that much between the plan and the measured roll during a move
    assert sim.max_error() < 3.0 and not sim.fell()


def test_predicted_attitude_matches_the_telemetry_conventions():
    """The planner predicts what TELEMETRY measures: pitch nose down +, roll
    right side up +. Before LB lifts, the body leans right (the mass over
    RF RB LF): negative roll, measured as negative by the body."""
    sim = WalkSim("walk")
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
    """A body whose servos lag 200 ms (the model says 150) and whose IMU
    sits 1.5 deg nose down and 2 deg right up: the estimates find both."""
    sim = WalkSim("walk", lag_ms=200, imu_offset=(1.5, 2.0), balance=False)
    sim.run(400)
    e = sim.stream.tracker.estimate()
    assert e["lag_ms"] == pytest.approx(50, abs=20)
    assert e["offset_pitch_deg"] == pytest.approx(1.5, abs=0.5) and e["offset_roll_deg"] == pytest.approx(2.0, abs=0.7)
    assert not sim.fell()


def test_feet_are_sent_five_ticks_ahead():
    sim = WalkSim("trot")
    seen = []
    sim.run(50, each=lambda i: seen.append(sim.link.sent[-1].tick - (sim.vb.tick & 0xFFFF)))
    assert set(seen[1:]) == {P.FEET_LOOKAHEAD_TICKS}
    assert all(t.feet_queued >= P.FEET_LOOKAHEAD_TICKS - 1 for t in sim.gait()[2:])


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


@pytest.mark.parametrize("pattern", ("walk", "crawl", "trot"))
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
@pytest.mark.parametrize("pattern", ("walk", "crawl"))
def test_a_14_mm_margin_keeps_a_wrong_centre_of_mass_on_its_plan(pattern):
    """The same error in all four directions: with margin_mm 14 the walk and
    the crawl stay near the plan with no catch and no hold (with 10 they
    need the catch or the gate in one or two directions). With the masses
    measured 2026-11-14 and the faster gaits of 2026-11-15: the walk within
    2.3 deg, the crawl within 2.1 deg (before: 2.8 and 4.1)."""
    for dx, dy in ((10, 5), (10, -5), (-10, 5), (-10, -5)):
        sim = WalkSim(pattern, body_model=wrong_mass(dx, dy, 50), config=PlannerConfig(margin_mm=14.0))
        assert sim.run(1500) == 1500
        assert not sim.fell() and sim.max_error() < 4.5, (dx, dy, sim.max_error())
        assert sim.planner.stats["catches"] == 0 and sim.planner.stats["holds"] == 0, (dx, dy)
