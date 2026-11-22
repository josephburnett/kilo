"""Golden vectors for the C++ planner (kernel/mind/planner), from the Python
planner (mind/kilo/planner): every scenario's commands and every tick it
plans, as lines that kernel/host/tests/test_planner_vectors.cpp replays.

    .venv/bin/python kernel/host/tests/gen_planner_vectors.py > kernel/host/tests/planner_vectors.txt

Line format (space separated; SPEED -1 is the pattern's default speed):

    sr PATTERN  lo hi                                         patterns.speed_range
    tm PATTERN SPEED  stride shift stance period speed        patterns.timing
    ttm SPEED DIR  stride shift stance period speed swing     patterns.timing("trot", SPEED, DIR), swing_for
    land W V  x                                               plan.TROT_FIT.land_x
    ik LEG X Z  t1 t2                                         BodyModel.ik (radians)
    com X0..X3 Z0..Z3  cx cy cz                               BodyModel.com
    fit X0..X3 Z0..Z3 SUPPORT PP PR  pitch roll               fit_attitude with prev (PP, PR)
    stab X0..X3 Z0..Z3 SUPPORT P R  margin                    stability
    S NAME PATTERN DIRECTION SPEED TICK                       a new Planner
    speed V | dir D | steer K | pattern P | stop | halt | observe EP ER
                                                              a command before the next tick
    t TICK X0..X3 Z0..Z3 SUPPORT PITCH ROLL PLAN_PITCH PLAN_ROLL MARGIN LIFTING
                                                              one Planner.next() (LIFTING: a bit mask)
    end DONE HOLDS CATCHES WAITS MIN_MARGIN PATTERN           the planner after its scenario
    er                                                        a new ErrorTracker
    ea TICK P R RP RR CP CR OWN_P OWN_R  ep er off_p off_r    ErrorTracker.add (TELEMETRY's 1/16 deg
                                                              ints; OWN "-": the reference the body reports)
    ee none | ee N OFF_P OFF_R LAG QUALITY SPIKE CORR_P CORR_R
                                                              ErrorTracker.estimate() (LAG, QUALITY may be none)
"""

import math
import sys

from kilo.planner import errors
from kilo.planner import patterns as pat
from kilo.planner.model import BodyModel, fit_attitude, stability
from kilo.planner.plan import TROT_FIT, Planner

M = BodyModel()


def f(v: float) -> str:
    """4 decimals (the comparison allows 0.01 deg, 0.05 mm), trailing zeros cut."""
    t = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if t in ("-0", "") else t


def line(*parts) -> None:
    sys.stdout.write(" ".join(str(p) for p in parts) + "\n")


def statics() -> None:
    for name in pat.PATTERN_NAMES:
        lo, hi = pat.speed_range(name)
        line("sr", name, f(lo), f(hi))
        for v in (None, lo, hi, (lo + hi) / 2, hi * 3, lo / 3, 0.0, 20.0, 90.0):
            t = pat.timing(name, v)
            line("tm", name, -1 if v is None else repr(float(v)), f(t.stride_mm), f(t.shift_ms), f(t.stance_ms),
                 f(t.period_ms), f(t.speed_mm_s))
    for leg in range(4):
        for x, z in ((0, 76), (20, 70), (-25, 60), (10, 85), (0, 95), (0, 20), (60, 30)):
            t1, t2 = M.ik(leg, x, z)
            line("ik", leg, x, z, f(t1), f(t2))
    cases = [([0, 0, 0, 0], [76, 76, 76, 76]), ([5, -3, 2, -8], [70, 80, 75, 72]),
             ([12, 12, -12, -12], [66, 66, 66, 66]), ([-20, 18, 4, 9], [60, 88, 71, 79])]
    for xs, zs in cases:
        line("com", *xs, *zs, *(f(v) for v in M.com(xs, zs)))
        # not one foot: there the plane rests on the regularisation alone and
        # Python's own answer is rounding noise (2.467 deg for a prev of 2.5)
        for sup in (0xF, 0xE, 0xD, 0xB, 0x7, 0x5, 0xA, 0x0):
            for prev in ((0.0, 0.0), (2.5, -3.0)):
                line("fit", *xs, *zs, sup, f(prev[0]), f(prev[1]),
                     *(f(v) for v in fit_attitude(M, xs, zs, sup, prev)))
            for att in ((0.0, 0.0), (4.0, -6.0)):
                line("stab", *xs, *zs, sup, f(att[0]), f(att[1]), f(stability(M, xs, zs, sup, *att)))


def trot_statics() -> None:
    for v in (0.0, 40.0, 100.0, 144.0, 150.0, 150.5, 160.0, 168.0, 170.0, 180.0, 186.0, 190.0, 200.0, 207.0, 230.0,
              400.0):
        for d in ("forward", "back"):
            t = pat.timing("trot", v, d)
            line("ttm", repr(v), d, f(t.stride_mm), f(t.shift_ms), f(t.stance_ms), f(t.period_ms), f(t.speed_mm_s),
                 pat.swing_for(v))
    for w in range(8, 14):
        for v in (-180.0, 0.0, 144.0, 207.0):
            line("land", w, repr(v), f(TROT_FIT.land_x(w, v)))


def scenario(name, pattern, direction="forward", speed=None, tick=1000, script=()):
    """script: ("run", n) plans n ticks, ("until_done", n) up to n until the
    planner is done, the rest are commands."""
    p = Planner(M, pattern, direction, speed, tick=tick)
    line("S", name, pattern, direction, -1 if speed is None else repr(float(speed)), tick)

    def tick_line():
        t = p.next()
        lifting = sum(1 << leg for leg in t.lifting)
        line("t", t.tick, *t.x, *t.z, t.support, f(t.pitch), f(t.roll), f(t.plan_pitch), f(t.plan_roll),
             f(t.margin), lifting)

    for item in script:
        kind = item[0]
        if kind == "run":
            for _ in range(item[1]):
                tick_line()
        elif kind == "until_done":
            n = 0
            while not p.done and n < item[1]:
                tick_line()
                n += 1
        elif kind == "speed":
            line("speed", repr(float(item[1])))
            p.set_speed(item[1])
        elif kind == "dir":
            line("dir", item[1])
            p.set_direction(item[1])
        elif kind == "steer":
            line("steer", repr(float(item[1])))
            p.set_steer(item[1])
        elif kind == "pattern":
            line("pattern", item[1])
            p.set_pattern(item[1])
        elif kind == "stop":
            line("stop")
            p.stop()
        elif kind == "halt":
            line("halt")
            p.halt()
        elif kind == "observe":
            line("observe", repr(float(item[1])), repr(float(item[2])))
            p.observe(item[1], item[2])
    mm = p.stats["min_margin"]
    line("end", int(p.done), p.stats["holds"], p.stats["catches"], p.stats["waits"],
         "inf" if mm == math.inf else f(mm), p.pattern.name)


def scenarios() -> None:
    # every pattern from stand, speed changes, the stop back to stand
    scenario("trot", "trot", script=[("run", 60), ("speed", 90.0), ("run", 45), ("speed", 10.0), ("run", 60),
                                     ("stop",), ("until_done", 400)])
    scenario("walk", "walk", script=[("run", 200), ("stop",), ("until_done", 700)])
    # the walk (since 2026-11-22): forward, backward, forward again at its top, the stop
    scenario("walk_fwd_back", "walk", script=[("run", 150), ("dir", "back"), ("run", 150), ("dir", "forward"),
                                              ("speed", 73.0), ("run", 120), ("stop",), ("until_done", 700)])
    scenario("crawl", "crawl", script=[("run", 250), ("speed", 4.0), ("run", 50)])
    # directions: back, the turns, march; from a tick near the u16 wrap
    scenario("trot_turns", "trot", "back", tick=65500, script=[("run", 40), ("dir", "left"), ("run", 50),
                                                              ("dir", "right"), ("run", 50), ("dir", "march"),
                                                              ("run", 40), ("dir", "forward"), ("run", 30)])
    scenario("walk_left", "walk", "left", speed=60.0, script=[("run", 120), ("dir", "back"), ("run", 60)])
    scenario("crawl_march", "crawl", "march", script=[("run", 120)])
    # the top speeds (asked past them: capped), the lift in the lean move's tail
    scenario("crawl_top_back", "crawl", "back", speed=40.0, script=[("run", 220)])
    scenario("trot_top_down", "trot", speed=200.0, script=[("run", 60), ("speed", 100.0), ("run", 60)])
    # pattern changes: walk and trot directly at a slot start; into or out of the crawl the legs go
    # home first, out of the crawl's lean level first
    scenario("trot_to_walk", "trot", "left", script=[("run", 50), ("pattern", "walk"), ("run", 200)])
    scenario("walk_to_trot", "walk", script=[("run", 20), ("pattern", "crawl"), ("run", 10), ("pattern", "trot"),
                                             ("run", 80)])
    scenario("walk_to_crawl", "walk", speed=73.0, script=[("run", 37), ("pattern", "crawl"), ("run", 260)])
    scenario("crawl_to_walk", "crawl", script=[("run", 130), ("pattern", "walk"), ("run", 300), ("pattern", "trot"),
                                               ("run", 60)])
    # speed changes while the next slot's lean is being solved
    scenario("crawl_speeds", "crawl", script=[("run", 70), ("speed", 6.0), ("run", 70), ("speed", 11.0),
                                              ("run", 70), ("speed", 40.0), ("run", 40)])
    scenario("walk_speeds", "walk", script=[("run", 50), ("speed", 30.0), ("run", 70), ("speed", 73.0), ("run", 70)])
    # the attitude error: the gate holds the lift, the catch lands the foot
    scenario("crawl_gate_catch", "crawl", script=[("observe", 0.0, 8.0), ("run", 40), ("observe", 0.0, 0.0),
                                                  ("run", 2), ("observe", -3.0, 6.0), ("run", 4),
                                                  ("observe", 0.0, 0.0), ("run", 40)])
    scenario("walk_gate_catch", "walk", script=[("observe", 0.0, 8.0), ("run", 40), ("observe", 0.0, 0.0),
                                                ("run", 2), ("observe", -3.0, 6.0), ("run", 4),
                                                ("observe", 0.0, 0.0), ("run", 40)])
    # steering (the remote's A6 / C6 while walking, 2026-11-17): the levels, both ways, at the top
    # speed (the outer side at the longest stride, the lift tapered), backward, then the stop
    scenario("trot_steer", "trot", script=[("run", 40), ("steer", 0.25), ("run", 40), ("steer", 1.0), ("run", 50),
                                           ("steer", -0.5), ("run", 50), ("speed", 201.6), ("run", 50),
                                           ("dir", "back"), ("run", 70), ("stop",), ("until_done", 400)])
    # a steer through the gait changes, into the crawl and its lean
    scenario("walk_steer_changes", "walk", script=[("steer", 0.5), ("run", 60), ("pattern", "trot"), ("run", 60),
                                                   ("pattern", "crawl"), ("run", 300)])
    scenario("crawl_steer", "crawl", "back", script=[("steer", -1.0), ("run", 260)])
    # a turn in place, then straight on (the remote's B7 while turning)
    scenario("trot_turn_go", "trot", "right", script=[("run", 60), ("dir", "forward"), ("run", 60)])
    # the quick stop (halt: the remote's B6 and C7, the distance guard) at every gait and phase:
    # mid-swing, in the shift, the walk at several points of its slot, during the planned stop, steering,
    # turning, the crawl mid-lean, at the top speeds
    for n in (3, 7, 12, 20, 31):
        scenario(f"trot_halt_{n}", "trot", speed=201.6, script=[("run", 40 + n), ("halt",), ("until_done", 200)])
    for n in (0, 3, 6, 9, 11):
        scenario(f"walk_halt_{n}", "walk", speed=73.0, script=[("run", 96 + n), ("halt",), ("until_done", 200)])
    for n in (0, 9, 21, 40):
        scenario(f"crawl_halt_{n}", "crawl", speed=27.0, script=[("run", 130 + n), ("halt",), ("until_done", 600)])
    scenario("trot_stop_halt", "trot", speed=180.0, script=[("run", 60), ("stop",), ("run", 7), ("halt",),
                                                             ("until_done", 200)])
    scenario("trot_steer_halt", "trot", speed=201.6, script=[("steer", 0.5), ("run", 63), ("halt",), ("halt",),
                                                              ("until_done", 200)])
    scenario("trot_turn_halt", "trot", "left", script=[("run", 45), ("halt",), ("until_done", 200)])
    scenario("walk_back_halt", "walk", "back", script=[("run", 100), ("halt",), ("until_done", 200)])
    # the body tipping toward the legs that square up: caught, they stay where they land (the end comes)
    scenario("crawl_halt_catch", "crawl", script=[("run", 140), ("halt",), ("observe", 0.0, 9.0), ("run", 30),
                                                  ("observe", -8.0, -9.0), ("until_done", 600)])
    scenario("trot_halt_catch", "trot", speed=180.0, script=[("run", 50), ("halt",), ("observe", 9.0, 0.0),
                                                             ("until_done", 200)])
    scenario("crawl_change_halt", "crawl", script=[("run", 90), ("pattern", "walk"), ("run", 30), ("halt",),
                                                   ("until_done", 600)])
    scenario("trot_gate", "trot", speed=2 * pat.default_speed("trot"),
             script=[("run", 40), ("observe", 6.0, 0.0), ("run", 40), ("observe", 0.0, 0.0), ("run", 30)])


def long_scenarios() -> None:
    """The trot at its long swing (the remote's +4 and +5, 180 and 201.6 mm/s; the "long trot" of
    2026-11-21): the build-up from stand, the stops from the long stride at every phase, steering,
    the changes to and from the other gaits, backward, the turns, the gate and the catch."""
    st1, st2, st3 = 160.0, 180.0, 200.0  # 9, 10 and 11-tick swings
    scenario("trot_build", "trot", speed=st1, script=[("run", 120), ("speed", st2), ("run", 80), ("speed", st3),
                                                      ("run", 80), ("speed", 207.0), ("run", 50), ("speed", 400.0),
                                                      ("run", 30), ("speed", st1), ("run", 120), ("stop",),
                                                      ("until_done", 400)])
    for n in (0, 3, 6, 9, 13, 17, 22):
        scenario(f"trot_long_halt_{n}", "trot", speed=st3, script=[("run", 150 + n), ("halt",), ("until_done", 300)])
    for n in (0, 5, 11):
        scenario(f"trot_long_stop_{n}", "trot", speed=207.0, script=[("run", 150 + n), ("stop",),
                                                                     ("until_done", 400)])
    scenario("trot_long_steer", "trot", speed=st3, script=[("run", 120), ("steer", 0.25), ("run", 80),
                                                           ("steer", -0.25), ("run", 80), ("steer", 1.0),
                                                           ("run", 60), ("speed", st1), ("run", 60), ("halt",),
                                                           ("until_done", 300)])
    scenario("trot_long_to_walk", "trot", speed=st2, script=[("run", 150), ("pattern", "walk"), ("speed", 80.0),
                                                             ("run", 250)])
    scenario("trot_long_to_crawl", "trot", speed=st3, script=[("run", 150), ("pattern", "crawl"), ("speed", 18.0),
                                                              ("run", 300)])
    scenario("crawl_to_trot", "crawl", script=[("run", 150), ("pattern", "trot"), ("speed", st1), ("run", 350)])
    scenario("walk_trot_walk", "walk", script=[("run", 80), ("pattern", "trot"), ("speed", st1), ("run", 150),
                                               ("pattern", "walk"), ("run", 10), ("pattern", "trot"), ("run", 100)])
    scenario("trot_long_change_halt", "trot", speed=st3, script=[("run", 150), ("pattern", "walk"), ("run", 12),
                                                                 ("halt",), ("until_done", 300)])
    scenario("trot_long_stop_halt", "trot", speed=st2, script=[("run", 150), ("stop",), ("run", 9), ("halt",),
                                                               ("until_done", 300)])
    # backing off from the long stride (the walk's back-off: the trot itself only stops for the guard)
    scenario("trot_long_reverse", "trot", speed=st3, script=[("run", 150), ("dir", "back"), ("speed", 40.0),
                                                             ("run", 200), ("halt",), ("until_done", 200)])
    scenario("trot_long_back", "trot", "back", speed=180.0, script=[("run", 200), ("stop",), ("until_done", 400)])
    scenario("trot_long_turns", "trot", "left", speed=st1, script=[("run", 80), ("dir", "right"), ("run", 80),
                                                                   ("dir", "forward"), ("run", 80)])
    scenario("trot_long_gate_catch", "trot", speed=st2, script=[("run", 100), ("observe", 0.0, 8.0), ("run", 30),
                                                                ("observe", 0.0, 0.0), ("run", 3),
                                                                ("observe", -3.0, 8.0), ("run", 5),
                                                                ("observe", 0.0, 0.0), ("run", 60), ("halt",),
                                                                ("observe", 9.0, 0.0), ("until_done", 300)])


class _Tel:
    pass


def _est(e) -> None:
    if e is None:
        line("ee", "none")
        return

    def g(k):
        return "none" if e[k] is None else f(e[k])

    line("ee", e["samples"], f(e["offset_pitch_deg"]), f(e["offset_roll_deg"]), g("lag_ms"), g("lag_quality"),
         f(e["spike_deg"]), f(e["corr_pitch_deg"]), f(e["corr_roll_deg"]))


def _add(tr, t, ints, own=None) -> None:
    m = _Tel()
    m.pitch, m.roll, m.ref_pitch, m.ref_roll, m.corr_pitch, m.corr_roll = ints
    ep, er = tr.add(t, m, own)
    line("ea", t, *ints, "-" if own is None else repr(own[0]), "-" if own is None else repr(own[1]),
         f(ep), f(er), f(tr.offset[0]), f(tr.offset[1]))


def tracker() -> None:
    line("er")
    tr = errors.ErrorTracker()
    _est(tr.estimate())
    for t in range(260):
        ref = 8 * math.sin(2 * math.pi * t / 75)
        meas = 8 * math.sin(2 * math.pi * (t - 2) / 75) + 1.5 + (3.0 if t == 200 else 0.0)  # 40 ms late, a spike
        ints = (round(0.4 * 16), round(meas * 16), 0, round(ref * 16), round(0.1 * 16), round(-0.5 * 16))
        # every third sample with the walker's own prediction (the body's balance off)
        own = (0.25 * (t % 5), ref + 0.3) if t % 3 == 0 else None
        _add(tr, t, ints, own)
        if t in (40, 49, 50, 120, 259):
            _est(tr.estimate())
    line("er")
    flat = errors.ErrorTracker()  # a plan that does not move: no lag
    for t in range(100):
        _add(flat, t, (0, 5, 0, 0, 0, 0))
    _est(flat.estimate())


if __name__ == "__main__":
    statics()
    scenarios()
    tracker()
    trot_statics()
    long_scenarios()
