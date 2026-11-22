"""How Kilo stops: the planned stop against the quick stop (docs/walking-and-balance.md §14).

Each case walks the virtual body (tests/walksim.py: the planner, the
stream, the body's FEET player and the walking physics with the servo lag)
at a gait and speed until the walk is steady, then asks for the stop at a
tick of the cycle, at every second tick over a whole cycle, and measures
from that moment:

- ``stopped``: from when on the body stays within 3 mm of where it ends
  (its way over the ground: the physics' feet on the ground moving back
  under it; the crawl's lean moves the body over its feet too);
- ``way``: how far it went from the request until then;
- ``done``: when the walk ended (the planner done and the STANCE posture
  taken), the legs squared up under the shoulders;
- ``pitch`` / ``roll``: how far the attitude goes, from the request to the
  end, outside its range over the cycle before (the walk's own sway), with
  the physics' momentum on (braking shifts the weight forward by a h / g);
- ``a``: the largest deceleration the physics saw while the body travelled
  (it includes each step's own: a landing foot changes the speed);
- ``fell``: whether any case fell (printed, and FELL in the table).

    PYTHONPATH=mind python mind/tests/stopsim.py [--md]

The physics has no sliding feet and no frame flex: the numbers are the
model's (computed), not the floor's.
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from kilo.planner import patterns as pat  # noqa: E402
from kilo.vbody import physics as phys  # noqa: E402
from walksim import WalkSim  # noqa: E402

TICK_S = phys.TICK_S
STILL_MM = 3.0  # "stopped": the body stays this close to where it ends
REPLAN_TICKS = 4  # "replan": the plan LOOKAHEAD (5) ahead, planned again from the body's tick + 2: 4 ticks sooner
MODES = ("planned", "quick", "replan")
WALK_FACTORS = (0.4, 0.7, 1.0, 1.15, 1.25)  # the remote's levels (kernel/mind/remote/remote.cpp)
TROT_FACTORS = (0.4, 0.7, 1.0, 1.25, 1.4)


def level_speed(pattern: str, level: int) -> float:
    f = TROT_FACTORS if pattern == "trot" else WALK_FACTORS
    return f[level - 1] * pat.default_speed(pattern)


def _physics(sim):
    g = sim.vb.feet if sim.vb.feet is not None else sim.vb.xfade
    return None if g is None else g.physics


def one(pattern: str, speed: float, direction: str, mode: str, at: int, warm_s: float = 3.0) -> dict:
    """One stop, ``at`` ticks into the cycle after ``warm_s`` of walking. mode: "planned" (stop()),
    "quick" (halt() as the Python walker does it: from the ticks not yet sent, LOOKAHEAD ahead) or
    "replan" (halt() as the C++ mind does it: the ticks already sent planned again from the body's
    tick + 2, kernel/mind/planner/walker.h; here the halt REPLAN_TICKS wakes before the request)."""
    phys.Physics.INERTIA = True
    try:
        sim = WalkSim(pattern, direction, speed)
        period = int(round(pat.timing(pattern, speed).period_ms / 1000.0 / TICK_S))
        early = REPLAN_TICKS if mode == "replan" else 0
        for _ in range(int(warm_s / TICK_S) + at - early):
            sim.step()
        if mode == "planned":
            sim.planner.stop()
        else:
            sim.planner.halt()
        for _ in range(early):
            sim.step()
        ph = _physics(sim)
        before = [(t.pitch / 16.0, t.roll / 16.0) for t in sim.telemetry[-max(50, period):]]
        d0 = ph.dist_mm
        ph.max_a_body = 0.0
        n0 = len(sim.telemetry)
        pos, acc, ticks, on = [], [], 0, True
        while on and ticks < 800:
            on = sim.step()
            ticks += 1
            p = _physics(sim)
            if p is ph:
                pos.append(p.dist_mm - d0)
                acc.append(p.max_a_body)
                p.max_a_body = 0.0
            elif pos:
                pos.append(pos[-1])
                acc.append(0.0)
        # stopped: from here on the body stays within STILL_MM of where it ends
        final = pos[-1] if pos else 0.0
        stopped = 0
        for i, x in enumerate(pos):
            if abs(x - final) > STILL_MM:
                stopped = i + 1
        way = pos[stopped - 1] if stopped else 0.0
        max_a = max(acc[:stopped + 5], default=0.0)  # the braking (not the square-up's leans)
        stopped_tick = stopped
        after = [(t.pitch / 16.0, t.roll / 16.0) for t in sim.telemetry[n0:]]
        rng = lambda v: (min(v), max(v)) if v else (0.0, 0.0)  # noqa: E731
        bp, br = rng([a[0] for a in before]), rng([a[1] for a in before])
        ap, ar = rng([a[0] for a in after]), rng([a[1] for a in after])
        # how far the stop takes the attitude outside the walk's own range
        dp = max(0.0, ap[1] - bp[1], bp[0] - ap[0])
        dr = max(0.0, ar[1] - br[1], br[0] - ar[0])
        peak_p = max((abs(a[0]) for a in after), default=0.0)
        peak_r = max((abs(a[1]) for a in after), default=0.0)
        return {"stopped_s": stopped_tick * TICK_S, "way_mm": way, "done_s": ticks * TICK_S, "dpitch": dp,
                "droll": dr, "peak_pitch": peak_p, "peak_roll": peak_r, "a_mm_s2": max_a, "fell": sim.fell(),
                "ended": sim.stream.ended}
    finally:
        phys.Physics.INERTIA = False


def case(pattern: str, speed: float, mode: str, direction: str = "forward", step: int = 2) -> dict:
    period = int(round(pat.timing(pattern, speed).period_ms / 1000.0 / TICK_S))
    rs = [one(pattern, speed, direction, mode, at) for at in range(0, period, step)]

    def agg(k, f=max):
        return f(r[k] for r in rs)
    return {"n": len(rs), "stopped_med": statistics.median(r["stopped_s"] for r in rs),
            "stopped_max": agg("stopped_s"), "way_med": statistics.median(r["way_mm"] for r in rs),
            "way_max": agg("way_mm"), "done_med": statistics.median(r["done_s"] for r in rs),
            "done_max": agg("done_s"), "dpitch": agg("dpitch"), "droll": agg("droll"),
            "peak_pitch": agg("peak_pitch"), "peak_roll": agg("peak_roll"), "a": agg("a_mm_s2"),
            "fell": any(r["fell"] for r in rs), "ended": sorted({str(r["ended"]) for r in rs})}


CASES = [("crawl", 1), ("crawl", 3), ("crawl", 5), ("walk", 1), ("walk", 3), ("walk", 5),
         ("trot", 1), ("trot", 3), ("trot", 4), ("trot", 5)]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--md", action="store_true", help="a Markdown table")
    ap.add_argument("--step", type=int, default=2, help="ticks between the request times over a cycle")
    ap.add_argument("--modes", default=",".join(MODES), help="which stops (planned, quick, replan)")
    a = ap.parse_args(argv)
    rows = []
    for pattern, level in CASES:
        v = level_speed(pattern, level)
        v = pat.timing(pattern, v).speed_mm_s
        for mode in a.modes.split(","):
            step = a.step * (3 if pattern == "crawl" else 1)  # the crawl's cycle is 2.5 s: every 6th tick
            r = case(pattern, v, mode, step=step)
            rows.append((pattern, level, v, mode, r))
            if not a.md:
                print(f"{pattern:5} +{level} {v:6.1f} mm/s {mode:7} n={r['n']:3} "
                      f"stopped {r['stopped_med']:.2f}/{r['stopped_max']:.2f} s  way {r['way_med']:5.1f}/"
                      f"{r['way_max']:5.1f} mm  done {r['done_med']:.2f}/{r['done_max']:.2f} s  "
                      f"pitch +{r['dpitch']:.1f} (peak {r['peak_pitch']:.1f})  roll +{r['droll']:.1f} "
                      f"(peak {r['peak_roll']:.1f})  a {r['a'] / 1000:.2f} m/s2  fell {r['fell']}  {r['ended']}",
                      flush=True)
    if a.md:
        print("| Gait, level | Speed | Stop | Stopped (median / worst) | Way (median / worst) | Done (median) | "
              "Pitch, roll beyond the walk's own |")
        print("|---|---|---|---|---|---|---|")
        for pattern, level, v, mode, r in rows:
            print(f"| {pattern} +{level} | {v:.0f} mm/s | {mode} | "
                  f"{r['stopped_med']:.2f} / {r['stopped_max']:.2f} s | {r['way_med']:.0f} / {r['way_max']:.0f} mm | "
                  f"{r['done_med']:.1f} s | +{r['dpitch']:.1f}°, +{r['droll']:.1f}° |"
                  + (" FELL |" if r["fell"] else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
