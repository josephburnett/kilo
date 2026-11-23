"""The smoother-walk study (docs/walking-and-balance.md §16): what the old
walk's stutter is in the floor recordings, and the walk that replaced it
against the old one in the virtual body.

    PYTHONPATH=mind python mind/tests/walkstudy.py floor      # the recorded walks (~/kilo-logs), folded by cycle
    PYTHONPATH=mind python mind/tests/walkstudy.py margins    # the plan's margins from the model, and their sensitivity
    PYTHONPATH=mind python mind/tests/walkstudy.py compare    # old walk against the new, in both physics, with model errors
    PYTHONPATH=mind python mind/tests/walkstudy.py second     # the walk's second version against its first (§16.8)
    PYTHONPATH=mind python mind/tests/walkstudy.py drift      # the heading the gyro saw in every straight walk (§16.9)

The old walk (diagonal couplets, 2026-11-16 to 11-22) lives here only, as
OLD_WALK, and the lateral-sequence walk's first version (boot 16's) as
FIRST_WALK: the baselines the evidence is measured against, not planner
patterns any more. Everything the virtual body says is simulation
(kilo.vbody: a rigid body on point feet; RockingPhysics is the long-trot
study's pendulum-on-the-diagonal physics); ``floor`` and ``drift`` read
the recordings of whichever walk they hold (the old walk's until
2026-11-21, the new walk's from boot 16 on, docs/walk-trial-results.md).
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import statistics
import sys
from collections import defaultdict
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from kilo.planner import patterns as pat  # noqa: E402
from kilo.planner.model import LB, LEG_NAMES, LEGS, LF, RB, RF, BodyModel, Mass  # noqa: E402
from kilo.planner.plan import Planner  # noqa: E402
from kilo.protocol import kilo_protocol as P  # noqa: E402

from stridesim import joint_step_deg, physics_as  # noqa: E402
from walksim import WalkSim  # noqa: E402

M = BodyModel()

# The walk until 2026-11-22: LB, then RF 60 ms later, LB lands, RF lands, then RB and LF the same way.
OLD_WALK = pat.Pattern("oldwalk", ((LB, RF), (RB, LF)), swing_ms=150.0, stride_mm=30.0, step_h_mm=8.0,
                       body_h_256=205, period_ms=460.0, shift_min_ms=20.0, stance_max_ms=310.0, lean=False,
                       couple_ms=60.0, default_mm_s=80.0, stride_slew_mm=1.5)
NEW_WALK = pat.PATTERNS["walk"]
# The lateral-sequence walk as first built (boot 16's floor run): a 220 ms swing and one tick on four
# feet before every swing (a footfall every 240 ms), no sway.
FIRST_WALK = replace(NEW_WALK, name="firstwalk", swing_ms=220.0, shift_min_ms=20.0, stance_max_ms=740.0,
                     handover_ticks=(), sway_roll_deg=0.0, sway_dx_mm=0.0, swing_long_ms=0.0,
                     handover_long_ticks=(), long_from_mm_s=0.0, long_stride_mm=0.0)


def com_moved(dx: float = 0.0, dy: float = 0.0, base: BodyModel = M) -> BodyModel:
    """The body model with its centre of mass moved by moving the body's own mass."""
    b = base.body
    k = base.total_g / b.grams
    return replace(base, body=Mass(b.grams, b.x + dx * k, b.y + dy * k, b.z))


# What the self-measurement of 2026-11-22 found (docs/self-measurement-results.md): the centre of mass
# 3.1 mm behind the hand measurement (in the model since the same day: docs/model-fit-results.md) and the
# feet 132 mm front to back, not 123 (not in the model).
MEASURED = replace(M, length_mm=132.0)


def wide_feet(base: BodyModel = M) -> BodyModel:
    """The feet's rectangle the first self-measurement tilts as if (133 x 69 mm, not 123 x 65)."""
    return replace(base, length_mm=133.0, track_mm=69.0)


# ---- the plan alone -------------------------------------------------------------------------------

def plan_stats(p: pat.Pattern, speed: float, direction: str = "forward", model: BodyModel = M,
               plan_model: BodyModel = M, ticks: int = 400, warm: int = 150) -> dict:
    """The plan's own numbers over ``ticks`` after ``warm``: margins (the centre of mass of ``model``
    against the feet ``plan_model`` planned), footfall intervals, feet on the ground, joints."""
    from kilo.planner.model import fit_attitude, stability
    pl = Planner(plan_model, p, direction, speed)
    plans = [pl.next() for _ in range(warm + ticks)][warm:]
    one_up, two_ticks, prev_sup, falls, fastest = [], 0, None, [], 0.0
    for a, b in zip(plans, plans[1:]):
        fastest = max(fastest, joint_step_deg(a, b, plan_model))
    for tp in plans:
        n_up = 4 - bin(tp.support).count("1")
        if n_up == 1:
            att = fit_attitude(model, tp.x, tp.z, tp.support, (tp.plan_pitch, tp.plan_roll))
            one_up.append(stability(model, tp.x, tp.z, tp.support, att[0], att[1]))
        elif n_up == 2:
            two_ticks += 1
        if prev_sup is not None:
            landed = tp.support & ~prev_sup
            for leg in LEGS:
                if landed >> leg & 1:
                    falls.append((tp.tick, leg))
        prev_sup = tp.support
    gaps = [b[0] - a[0] for a, b in zip(falls, falls[1:])]
    cycle = pl.timing().period_ms / 1000.0
    return {"speed": pl.timing().speed_mm_s, "stride": pl.timing().stride_mm, "min_margin": min(one_up, default=math.nan),
            "two_feet_frac": two_ticks / len(plans), "gaps_ms": sorted(set(g * 20 for g in gaps)),
            "footfalls_s": len(falls) / (len(plans) * 0.02), "fastest": fastest * 50, "cycle_s": cycle}


def margins() -> None:
    print("| walk | speed | stride | cycle | footfall gaps | footfalls/s | on two feet | min margin, model "
          "| CoM 3.1 mm back | CoM 5 mm ahead | CoM 3 mm to the right | feet 133 x 69 | fastest joint |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    cases = [("old", OLD_WALK, v) for v in (32, 56, 80, 97)] + [("new", NEW_WALK, v) for v in (23.2, 40.6, 58, 66.7, 73)]
    for name, p, v in cases:
        st = plan_stats(p, v)
        alt = [plan_stats(p, v, model=m)["min_margin"] for m in
               (com_moved(-3.1), com_moved(5.0), com_moved(0.0, -3.0), wide_feet())]
        print(f"| {name} | {st['speed']:.0f} | {st['stride']:.1f} | {st['cycle_s'] * 1000:.0f} ms "
              f"| {'/'.join(str(int(g)) for g in st['gaps_ms'])} ms | {st['footfalls_s']:.1f} "
              f"| {st['two_feet_frac'] * 100:.0f} % | {st['min_margin']:+.1f} | "
              + " | ".join(f"{a:+.1f}" for a in alt) + f" | {st['fastest']:.0f}°/s |")
    for name, p, v in (("old", OLD_WALK, 80), ("new", NEW_WALK, 58), ("old", OLD_WALK, 40), ("new", NEW_WALK, 40)):
        for d in ("back", "left"):
            st = plan_stats(p, v, d)
            print(f"{name} {d} {v}: min margin {st['min_margin']:+.1f} mm, fastest joint {st['fastest']:.0f}°/s")


# ---- the virtual body ------------------------------------------------------------------------------

def sim_walk(p: pat.Pattern, speed: float, direction: str = "forward", physics: str = "physics",
             ticks: int = 400, warm: int = 150, body_model: BodyModel | None = None, inset_mm: float | None = None,
             **vb_kw) -> dict:
    """``ticks`` of walking after ``warm``; physics "physics" or "rocking" (RockingPhysics with the long-trot
    study's settings; ``inset_mm``: how far inside three feet the mass must be before they hold it)."""
    with physics_as(physics, **({} if inset_mm is None else {"inset_mm": inset_mm})):
        sim = WalkSim(p, direction, speed, body_model=body_model, **vb_kw)
        sim.run(warm)
        n0 = len(sim.telemetry)
        sim.run(ticks)
    pred = sim.pred()
    g = [t for t in sim.telemetry[n0:] if t.mode == P.Mode.STREAM and t.tick in pred]
    roll = [t.roll / 16 - pred[t.tick][1] for t in g]
    pitch = [t.pitch / 16 - pred[t.tick][0] for t in g]
    rr = [abs(t.rate_roll / 16) for t in g]
    rp = [abs(t.rate_pitch / 16) for t in g]
    st = sim.planner.stats
    # the roll's swing per cycle (peak to peak), measured
    n = max(1, int(round(sim.planner.timing().period_ms / 20)))
    cyc = [[t.roll / 16 for t in g[i:i + n]] for i in range(0, len(g) - n + 1, n)]
    p2p = statistics.mean(max(c) - min(c) for c in cyc) if cyc else math.nan
    return {"max_err": max((abs(x) for x in roll + pitch), default=0.0),
            "rms_roll": math.sqrt(sum(r * r for r in roll) / max(1, len(roll))),
            "roll_p2p": p2p, "rate_roll_max": max(rr, default=0.0), "rate_pitch_max": max(rp, default=0.0),
            "rate_rms": math.sqrt(sum(a * a + b * b for a, b in zip(rr, rp)) / max(1, len(rr))),
            "fell": sim.fell(), "holds": st["holds"], "catches": st["catches"], "max_tilt": sim.vb.max_tilt}


ERRORS = {"nominal": {}, "CoM 3.1 mm back (self-measurement)": dict(body_model=com_moved(-3.1)),
          "CoM 5 mm ahead": dict(body_model=com_moved(5.0)), "CoM 3 mm right": dict(body_model=com_moved(0, -3.0)),
          "feet 133 x 69": dict(body_model=wide_feet()),
          "servos slower (100 + 100 ms)": dict(lag_ms=100, tau_ms=100),
          "servos faster (20 + 50 ms)": dict(lag_ms=20, tau_ms=50)}


PHYSICS = (("Physics", "physics", None), ("RockingPhysics, 10 mm", "rocking", None),
           ("RockingPhysics, 3 mm", "rocking", 3.0))


def compare(errors: bool = True) -> None:
    print("| walk | speed | physics | roll swing per cycle | fastest roll | fastest pitch | rate rms | error rms (roll) "
          "| largest error | worst of the model errors | holds / catches / falls (all cases) |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for name, p, v in (("old", OLD_WALK, 80), ("new", NEW_WALK, 58), ("new", NEW_WALK, 73), ("old", OLD_WALK, 32),
                       ("new", NEW_WALK, 23.2)):
        for label, phys, inset in PHYSICS:
            r = sim_walk(p, v, physics=phys, inset_mm=inset)
            worst, bad = r["max_err"], [r["holds"], r["catches"], int(r["fell"])]
            if errors:
                for kw in list(ERRORS.values())[1:]:
                    e = sim_walk(p, v, physics=phys, inset_mm=inset, **kw)
                    worst = max(worst, e["max_err"])
                    bad = [bad[0] + e["holds"], bad[1] + e["catches"], bad[2] + int(e["fell"])]
            print(f"| {name} | {v} | {label} | {r['roll_p2p']:.2f}° | {r['rate_roll_max']:.0f}°/s "
                  f"| {r['rate_pitch_max']:.0f}°/s | {r['rate_rms']:.1f}°/s | {r['rms_roll']:.2f}° | {r['max_err']:.1f}° "
                  f"| {worst:.1f}° | {bad[0]} / {bad[1]} / {bad[2]} |")


# ---- the floor --------------------------------------------------------------------------------------

def floor(paths) -> None:
    """The recorded walks (pattern walk, straight forward, after 0.8 s at a speed), folded by the walk's
    cycle (LB lifts after a four-foot tick); duplicates (runs copied into several files) once."""
    from kilo.logs import flight
    from kilo.logs import walkrec as W
    seen, cyc, runs = set(), defaultdict(list), defaultdict(dict)
    for path in paths:
        try:
            s = flight.load(path)
        except Exception:  # noqa: BLE001 - a file that is not a recording
            continue
        for run in s.runs:
            if run.kind != "walk":
                continue
            w = flight.walk_of(s, run)
            res = w.mark("result")
            segs = []
            for m in w.marks:
                if m.word in ("stream", "timing"):
                    segs.append((int(m.fields["tick"]), m.fields.get("pattern"), m.fields.get("dir"),
                                 float(m.fields.get("speed", 0)), m.fields.get("steer"), m.fields.get("pending")))
                elif m.word == "stop":
                    segs.append((int(m.fields["tick"]), None, None, 0.0, None, None))
            if not any(sg[1] == "walk" for sg in segs):
                continue
            rows = W.tick_table(w)
            key = (rows[0]["tick"] if rows else 0, len(rows))
            if key in seen:
                continue
            seen.add(key)
            leads = [r["lead_ms"] for r in rows if r["lead_ms"] is not None]
            q = [r["pose_queued"] for r in rows if r["mode"] == "STREAM" and r["pose_queued"] is not None]
            runs[os.path.basename(path)] = {
                "holds": res.fields.get("holds") if res else "?", "catches": res.fields.get("catches") if res else "?",
                "underruns": (w.mark("end").fields.get("underruns") if w.mark("end") else "?"),
                "lead": (min(leads), max(leads)) if leads else None, "queued": (min(q), max(q)) if q else None}
            by = {r["tick"]: r for r in rows}

            def label(k: int):
                lab = None
                for i, sg in enumerate(segs):
                    end = segs[i + 1][0] if i + 1 < len(segs) else 1 << 30
                    if sg[0] + 40 <= k < end and sg[5] is None:
                        lab = sg
                return lab
            starts = [b["tick"] for a, b in zip(rows, rows[1:]) if b["lifting"] == "LB" and a["lifting"] == "none"]
            for c0, c1 in zip(starts, starts[1:]):
                lab = label(c0)
                if (lab is None or lab[1] != "walk" or lab[2] != "forward" or lab[4] not in (None, "0.00")
                        or label(c1) != lab or c1 - c0 not in (24, 48)):
                    continue
                seg = [by.get(c0 + i) for i in range(c1 - c0)]
                if all(r is not None and r["meas_roll"] is not None for r in seg):
                    cyc[(lab[3], c1 - c0)].append(seg)
    print("runs with the walk:")
    for k, v in runs.items():
        print(f"  {k}: holds {v['holds']} catches {v['catches']} underruns {v['underruns']} "
              f"FEET lead {v['lead']} ms, queued {v['queued']} ticks")
    for (v, period), cs in sorted(cyc.items()):
        n = len(cs[0])
        print(f"\nwalk forward {v:.0f} mm/s, {len(cs)} cycles of {period} ticks (tick 0: LB lifts)")
        print("  legs up   " + " ".join(f"{(r['lifting'] or '-'):>5s}" for r in cs[0]))
        for key in ("meas_roll", "meas_pitch", "rate_roll", "rate_pitch"):
            mean = [statistics.mean(c[i][key] for c in cs) for i in range(n)]
            print(f"  {key:9s} " + " ".join(f"{m:+5.1f}" for m in mean))
        p2r = [max(r["meas_roll"] for r in c) - min(r["meas_roll"] for r in c) for c in cs]
        p2p = [max(r["meas_pitch"] for r in c) - min(r["meas_pitch"] for r in c) for c in cs]
        rr = [max(abs(r["rate_roll"]) for r in c) for c in cs]
        rms = [math.sqrt(sum(r["rate_roll"] ** 2 + r["rate_pitch"] ** 2 for r in c) / n) for c in cs]
        print(f"  per cycle: roll swing {statistics.mean(p2r):.2f}° (sd {statistics.pstdev(p2r):.2f}), pitch swing "
              f"{statistics.mean(p2p):.2f}°, fastest roll {statistics.mean(rr):.0f}°/s, rate rms "
              f"{statistics.mean(rms):.1f}°/s; the plan predicted a flat body (0° of roll)")


# ---- the second version (§16.8) ---------------------------------------------------------------------

JUDGES = (("RockingPhysics 10 mm", 10.0, None), ("RockingPhysics 10 mm, measured", 10.0, MEASURED),
          ("RockingPhysics 3 mm, measured", 3.0, MEASURED))


def _second_row(args):
    name, p, v = args
    st, sm = plan_stats(p, v), plan_stats(p, v, model=MEASURED)
    cells = [f"{st['min_margin']:+.1f}", f"{sm['min_margin']:+.1f}", f"{st['fastest']:.0f}°/s"]
    for _, inset, bm in JUDGES:
        r = sim_walk(p, v, physics="rocking", inset_mm=inset, body_model=bm)
        cells.append(f"{r['roll_p2p']:.1f}° / {r['rate_roll_max']:.0f} / {r['rate_rms']:.0f}" + (" FELL" if r["fell"] else ""))
    return name, v, cells


def second() -> None:
    """The first version against the second, forward at the remote's levels: the plan's least margin
    by the model and by the measured centre of mass and feet, the fastest joint, and per judge the
    roll swing per cycle / fastest roll (°/s) / roll and pitch rate rms (°/s); then backward."""
    from multiprocessing import Pool
    jobs = [(n, p, v) for v in (23.2, 40.6, 55.0, 58.0, 61.0, 66.7, 72.5) for n, p in (("first", FIRST_WALK),
                                                                                         ("second", NEW_WALK))]
    with Pool(min(8, os.cpu_count() or 1)) as pool:
        rows = pool.map(_second_row, jobs, chunksize=1)
    print("| walk | speed | least margin, model | measured | fastest joint | " + " | ".join(j[0] for j in JUDGES) + " |")
    print("|---|---|---|---|---|" + "---|" * len(JUDGES))
    for name, v, cells in rows:
        print(f"| {name} | {v} | " + " | ".join(cells) + " |")
    for v in (23.2, 40.0, 58.0, 72.5):
        a = [plan_stats(p, v, "back", model=m)["min_margin"] for p in (FIRST_WALK, NEW_WALK) for m in (M, MEASURED)]
        print(f"backward {v}: first {a[0]:+.1f} / {a[1]:+.1f}, second {a[2]:+.1f} / {a[3]:+.1f} (model / measured)")


# ---- the heading (§16.9) ----------------------------------------------------------------------------

def drift(paths) -> None:
    """The gyro's yaw rate (TELEMETRY rate_yaw, the raw gyro about the body's up axis; + = the nose
    turning left) over every straight stretch of every recorded walk, after its first second, less the
    gyro's resting offset (the median of that file's still ticks)."""
    from kilo.logs import flight
    from kilo.logs import walkrec as W
    seen = set()
    print("| recording | gait | direction | speed | steer | seconds | heading change | °/s |")
    print("|---|---|---|---|---|---|---|---|")
    for path in paths:
        try:
            s = flight.load(path)
        except Exception:  # noqa: BLE001 - not a recording
            continue
        still = [m.rate_yaw / 16 for _, m in s.telemetry()
                 if abs(m.rate_roll) < 8 and abs(m.rate_pitch) < 8 and abs(m.rate_yaw) < 48]
        if len(still) < 20:
            continue
        bias = statistics.median(still)
        for run in s.runs:
            if run.kind != "walk":
                continue
            w = flight.walk_of(s, run)
            segs = [(int(m.fields["tick"]), m.fields.get("pattern"), m.fields.get("dir"), m.fields.get("speed"),
                     m.fields.get("steer", "0"), m.fields.get("pending")) for m in w.marks if m.word in ("stream", "timing")]
            segs += [(int(m.fields["tick"]), None, None, None, None, None) for m in w.marks if m.word == "stop"]
            segs.sort()
            tel, last = {}, None
            for f in w.frames:
                if f.direction == "rx" and isinstance(f.msg, P.Telemetry):
                    last = W._unwrap(f.msg.tick, last if last is not None else f.msg.tick)
                    tel[last] = f.msg
            for i, sg in enumerate(segs):
                if sg[1] is None or sg[5] or sg[2] not in ("forward", "back"):
                    continue
                end = segs[i + 1][0] if i + 1 < len(segs) else max(tel, default=0)
                ks = [k for k in range(sg[0] + 50, end) if k in tel]
                if len(ks) < 50 or (sg[0], len(ks)) in seen:
                    continue
                seen.add((sg[0], len(ks)))
                yaw = sum((tel[k].rate_yaw / 16 - bias) * 0.02 for k in ks)
                T = len(ks) * 0.02
                print(f"| {os.path.basename(path)} | {sg[1]} | {sg[2]} | {float(sg[3]):.1f} | {sg[4]} | {T:.1f} "
                      f"| {yaw:+.1f}° | {yaw / T:+.2f} |")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("what", choices=("floor", "margins", "compare", "second", "drift"))
    ap.add_argument("files", nargs="*", help="recordings for floor (default: ~/kilo-logs, from-server too)")
    ap.add_argument("--no-errors", action="store_true", help="compare: the nominal model only")
    a = ap.parse_args(argv)
    if a.what == "floor":
        h = os.path.expanduser("~/kilo-logs")
        floor(a.files or sorted(glob.glob(h + "/*.krec") + glob.glob(h + "/from-server/**/*.krec", recursive=True)))
    elif a.what == "drift":
        h = os.path.expanduser("~/kilo-logs")
        drift(a.files or sorted(glob.glob(h + "/*.krec") + glob.glob(h + "/from-server/**/*.krec", recursive=True)))
    elif a.what == "second":
        second()
    elif a.what == "margins":
        margins()
    else:
        compare(not a.no_errors)
    return 0


if __name__ == "__main__":
    sys.exit(main())
