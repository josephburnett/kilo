"""The quick stop (Planner.halt, kilo.planner.plan): the remote's B6 and C7 and the distance guard.

On the virtual body with the walking physics and its momentum on (tests/stopsim.py, which prints the
whole table for docs/walking-and-balance.md §14): at every gait's top speed and at moments spread over
a cycle, the walk ends cleanly ("stopped": the STANCE posture took the legs), nobody falls, the body
stops sooner and nearer than with the planned stop, and the trot's and the walk's within a step.
"""

from __future__ import annotations

import pytest

import stopsim
from kilo.planner import patterns as pat
from kilo.planner.model import BodyModel
from kilo.planner.plan import HALT_HOLD_TICKS, HALT_SQUARE_MM, HALT_TICKS, Planner


def top(pattern: str) -> float:
    return pat.timing(pattern, stopsim.level_speed(pattern, 5)).speed_mm_s


# the walk (since 2026-11-22: one leg at a time, a 220 to 260 ms beat): still within 1.3 s, its legs square up
# in its own beat and the body settles a few mm with each (the old walk's was 0.45 s); its roll
# 4.5 deg beyond the walk's own (the square-up's steps on three feet)
@pytest.mark.parametrize("pattern,limit_s,att_deg", [("trot", 0.6, 3.0), ("walk", 1.3, 4.5), ("crawl", 3.3, 3.0)])
def test_quick_stop_at_top_speed(pattern, limit_s, att_deg):
    v = top(pattern)
    period = int(round(pat.timing(pattern, v).period_ms / 20))
    for at in range(0, period, max(1, period // 6)):
        q = stopsim.one(pattern, v, "forward", "replan", at)
        p = stopsim.one(pattern, v, "forward", "planned", at)
        assert q["ended"] == "stopped" and not q["fell"], (at, q)
        assert q["stopped_s"] <= limit_s and q["way_mm"] <= p["way_mm"] + 1, (at, q, p)
        assert q["dpitch"] <= att_deg and q["droll"] <= att_deg, (at, q)  # beyond the walk's own sway, the model's


def test_the_feet_stop_sliding_in_halt_ticks_and_no_foot_lifts_in_the_hold():
    for pattern in ("trot", "walk", "crawl"):
        for at in range(0, 40, 3):
            p = Planner(BodyModel(), pattern, "forward", top(pattern))
            for _ in range(200 + at):
                p.next()
            p.halt()
            prev = p.next()
            lifts = []
            for t in range(1, HALT_HOLD_TICKS + 40):
                cur = p.next()
                if t < HALT_HOLD_TICKS:
                    lifts.append(cur.lifting)
                if t >= HALT_TICKS + 1 and t < HALT_HOLD_TICKS:  # the strides at 0: support feet still
                    for leg in range(4):
                        if (prev.support & cur.support) >> leg & 1:
                            assert abs(cur.x[leg] - prev.x[leg]) <= 1, (pattern, at, t, leg)
                prev = cur
                if p.done:
                    break
            # in the hold every foot that was in the air is down by the third tick, and none lifts
            assert all(not lf for lf in lifts[3:]), (pattern, at, lifts)
            while not p.done:
                p.next()
            assert all(abs(x) < HALT_SQUARE_MM + 1 for x in p.last.x), (pattern, at, p.last.x)


def test_halt_during_the_planned_stop_and_twice():
    p = Planner(BodyModel(), "trot", "forward", 180.0)
    for _ in range(80):
        p.next()
    p.stop()
    for _ in range(5):
        p.next()
    p.halt()
    p.halt()
    assert p.halting and p.stopping
    n = 0
    while not p.done and n < 200:
        p.next()
        n += 1
    assert p.done and n < 60
