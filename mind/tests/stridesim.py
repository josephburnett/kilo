"""The long-stride study (docs/walking-and-balance.md §15): the trot's limits,
the fit of its rock to the floor recordings, and the trot's long swing
(kilo.planner.plan, the trot since 2026-11-22) in the virtual body.

    PYTHONPATH=mind python mind/tests/stridesim.py limits     # joints and reach per swing length
    PYTHONPATH=mind python mind/tests/stridesim.py fit        # the recorded rock against the models (~/kilo-logs)
    PYTHONPATH=mind python mind/tests/stridesim.py schedule   # the build-up, speed by speed, in the virtual body
    PYTHONPATH=mind python mind/tests/stridesim.py stops      # the quick stop from the long stride

Two physics: kilo.vbody.physics.Physics (the virtual body's own) and
kilo.vbody.rocking.RockingPhysics (a pendulum on the diagonal's line,
momentum kept across support changes, ROCKING's settings: kilo.measure.score's
judge, the one that predicts the floor recordings best with the body model).
The fitted pendulum (kilo.planner.plan.TROT_FIT) is the model that
matches the recordings; the physics check the mechanics (reach, joints,
clearance, falls, the stop) and the direction of the effects.
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

from kilo.logs import proto2  # noqa: E402
from kilo.planner import patterns as pat  # noqa: E402
from kilo.planner.model import LEGS, BodyModel, Mass  # noqa: E402
from kilo.planner.patterns import TROT, swing_for, trot_timing as timing  # noqa: E402
from kilo.planner.plan import TROT_FIT, PlannerConfig  # noqa: E402
from kilo.protocol import kilo_protocol as P  # noqa: E402
from kilo.vbody import pose as vpose  # noqa: E402
from kilo.vbody import physics as phys  # noqa: E402
from kilo.vbody.rocking import RockingPhysics  # noqa: E402

from walksim import WalkSim  # noqa: E402

M = BodyModel()
# RockingPhysics' settings: the judge that predicts the floor recordings best with the one body model
# (kilo.measure.score.Judge, docs/model-fit-results.md; until 2026-11-22 inset 10, line lag 50)
ROCKING = dict(inset_mm=0.0, line_lag_ms=55.0, legs_give=True)
TICK_S = phys.TICK_S


def joint_step_deg(a, b, model=M) -> float:
    """The fastest joint between two ticks (TickPlans or feet), degrees."""
    ax, az = (a.x, a.z) if hasattr(a, "x") else (a.x_mm, a.z_mm) if hasattr(a, "x_mm") else proto2.feet_of(a)
    bx, bz = (b.x, b.z) if hasattr(b, "x") else (b.x_mm, b.z_mm) if hasattr(b, "x_mm") else proto2.feet_of(b)
    worst = 0.0
    for leg in LEGS:
        t = model.ik(leg, ax[leg], az[leg])
        u = model.ik(leg, bx[leg], bz[leg])
        worst = max(worst, abs(math.degrees(t[0] - u[0])), abs(math.degrees(t[1] - u[1])))
    return worst


class physics_as:
    """with physics_as("rocking"): new POSE players walk on RockingPhysics (ROCKING), or Physics."""

    def __init__(self, which: str, **kw):
        self.which, self.kw = which, kw

    def __enter__(self):
        self.old = vpose.PosePlayer.PHYSICS
        if self.which == "rocking":
            kw = {**ROCKING, **self.kw}
            vpose.PosePlayer.PHYSICS = lambda model, **a: RockingPhysics(model, **kw, **a)
        else:
            vpose.PosePlayer.PHYSICS = phys.Physics
        return self

    def __exit__(self, *exc):
        vpose.PosePlayer.PHYSICS = self.old


def shifted(dx: float = 0.0, dy: float = 0.0, battery_g: float = 0.0) -> BodyModel:
    """The body model with its centre of mass moved (the body's mass moved) and battery added."""
    b, bat = M.body, M.battery
    k = (M.total_g + battery_g) / b.grams
    return replace(M, body=Mass(b.grams, b.x + dx * k, b.y + dy * k, b.z),
                   battery=Mass(bat.grams + battery_g, bat.x, bat.y, bat.z))


# ---- one walk -------------------------------------------------------------------------------------

def walk(speed: float, direction: str = "forward", physics: str = "rocking", place: bool = True,
         ticks: int = 400, body_model: BodyModel | None = None, **vb_kw) -> dict:
    """``ticks`` of trotting at ``speed`` after the build-up from rest (place False: the pairs land at
    half a stride); what the virtual body did."""
    with physics_as(physics):
        sim = WalkSim("trot", direction, speed, body_model=body_model, config=PlannerConfig(trot_place=place), **vb_kw)
        sim.run(150)
        n0 = len(sim.telemetry)
        sent0 = len(sim.link.sent)
        sim.run(ticks)
    pred = sim.pred()
    g = [t for t in sim.telemetry[n0:] if t.mode == P.Mode.STREAM and t.tick in pred]
    err = [abs(t.roll / 16 - pred[t.tick][1]) for t in g] + [abs(t.pitch / 16 - pred[t.tick][0]) for t in g]
    roll = [t.roll / 16 - pred[t.tick][1] for t in g]
    sent = sim.link.sent[sent0:]
    fastest = max((joint_step_deg(a, b) for a, b in zip(sent, sent[1:])), default=0.0) * 50
    ph = sim.vb.stream.physics if sim.vb.stream is not None else None
    clear = [c for c in getattr(ph, "clear_log", [])[-ticks:] if c != math.inf]
    st = sim.planner.stats
    return {"speed": speed, "w": sim.planner.swing_ticks, "stride": max(abs(v) for v in sim.planner.s_cur),
            "max_err": max(err, default=0.0), "rms_roll": math.sqrt(sum(r * r for r in roll) / max(1, len(roll))),
            "fell": sim.fell(), "catches": st["catches"], "holds": st["holds"], "fastest": fastest,
            "x": (min(min(f.x_mm) for f in sent), max(max(f.x_mm) for f in sent)),
            "reach": all(M.reachable(f.x_mm[i], f.z_mm[i]) for f in sent for i in LEGS),
            "max_tilt": sim.vb.max_tilt}


# ---- limits ---------------------------------------------------------------------------------------

def swing_rate(s: float, w: int, lift: float, h: float, back: float = 0.0) -> float:
    """deg/s: the fastest joint of a swing of stride s over w ticks (the planner's cosine profile)."""
    slide = s / (w + 1.5)
    x0, x1 = -s / 2 - back - slide, s / 2 - back
    best = 0.0
    for leg in (0, 2):
        prev = M.ik(leg, x0, h)
        for j in range(1, w + 1):
            sm = (1 - math.cos(math.pi * j / w)) / 2
            cur = M.ik(leg, x0 + (x1 - x0) * sm, h - lift * (1 - math.cos(2 * math.pi * j / w)) / 2)
            best = max(best, abs(cur[0] - prev[0]), abs(cur[1] - prev[1]))
            prev = cur
    return math.degrees(best) * 50


def reached_lift(w: int, lift: float = 8.0, model: BodyModel = M) -> float:
    """The foot's highest lift through the servos' dead time and response (mm) for a w-tick swing."""
    a = model.response_alpha(TICK_S * 1000 / 10)
    y, best = 0.0, 0.0
    cmd = [lift * (1 - math.cos(2 * math.pi * j / w)) / 2 for j in range(w + 1)] + [0.0] * 20
    for k in range(len(cmd) * 10):
        t = k / 10.0 - model.servo_lag_ms / (TICK_S * 1000)
        i = int(math.floor(t))
        u = 0.0 if i < 0 else cmd[min(i, len(cmd) - 1)]
        y += (u - y) * a
        best = max(best, y)
    return best


def limits() -> None:
    h = M.stance_height(pat.PATTERNS["trot"].body_h_256)
    print(f"body height {h:.1f} mm: the foot reaches {math.sqrt(M.reach_mm ** 2 - h ** 2):.1f} mm fore and aft "
          f"(lower: {M.stance_height(190):.1f} mm -> {math.sqrt(M.reach_mm ** 2 - M.stance_height(190) ** 2):.1f})")
    print("today's trot, 8-tick swing: " + ", ".join(
        f"{s} mm {lift} mm lift {swing_rate(s, 8, lift, h):.0f} deg/s" for s, lift in ((27.4, 8), (35, 8), (44, 6))))
    print("| swing | two feet | longest stride at 476 deg/s (8 mm lift) | its speed | lift reached | gain (deg/mm) |")
    print("|---|---|---|---|---|---|")
    for w in range(8, 15):
        s = 10.0
        while swing_rate(s + 0.5, w, 8, h) <= 476 and s < 120:
            s += 0.5
        v = s / ((w + 1.5) * TICK_S)
        print(f"| {w} ticks ({w * 20} ms) | {(w - 1) * 20} ms | {s:.1f} mm | {v:.0f} mm/s | {reached_lift(w):.1f} mm "
              f"| {TROT_FIT.gain_deg_per_mm(w):.1f} |")


# ---- the fit --------------------------------------------------------------------------------------

AIR_LFRB, AIR_RFLB = 0b1010, 0b0101  # POSE support: RF+LB down / LF+RB down


def _recordings(paths):
    from kilo.logs import flight
    for path in paths:
        s = flight.load(path)
        for r in s.runs:
            if r.kind == "walk":
                yield path, flight.walk_of(s, r)


def _replay(w, physics: str):
    """(tick, measured roll, virtual roll, POSE/FEET) of a recorded walk replayed through a POSE player."""
    from kilo.logs import walkrec as W
    from walkreplay import BALANCE, key_at
    if not w.plans:
        return []
    feet, tel = {}, {}
    last = w.plans[0][1].tick
    for f in w.frames:
        if f.msg is None:
            continue
        if f.direction == "tx" and proto2.is_stream(f.msg):
            feet[W._unwrap(f.msg.tick, last)] = f.msg
        elif f.direction == "rx" and isinstance(f.msg, P.Telemetry):
            last = W._unwrap(f.msg.tick, last)
            tel[last] = f.msg
    gt = [k for k, t in tel.items() if t.mode == P.Mode.STREAM]
    if not feet or not gt:
        return []
    out = []
    with physics_as(physics):
        g = vpose.PosePlayer(min(feet) - 1, M)
    for k in range(min(feet), max(gt) + 1):
        if k in feet:
            g.queue.append(key_at(feet[k], k))
        ph = g.physics
        g.tick(k, (ph.pitch, ph.roll), ph.rates, BALANCE, True)
        ph.step()
        t = tel.get(k)
        if t is not None and t.mode == P.Mode.STREAM and k in feet:
            out.append((k, t.roll / 16.0, ph.roll, feet[k]))
    return out


def _slots(rows):
    """Per trot slot: (first two-foot tick, sign, rolls measured and virtual from a tick before it to 10 after)."""
    out = []
    for i in range(1, len(rows) - 12):
        sup, prev = rows[i][3].support, rows[i - 1][3].support
        if sup in (AIR_LFRB, AIR_RFLB) and prev == 0xF:
            seg = rows[i - 1:i + 11]
            if all(seg[k][0] == seg[0][0] + k for k in range(len(seg))):
                out.append((seg[1][0], 1 if sup == AIR_LFRB else -1, [r[1] for r in seg], [r[2] for r in seg]))
    return out


def _fold(slots, j):
    n = len(slots[0][j])
    return [sum(s[1] * (s[j][i] - sum(s[j]) / n) for s in slots) / len(slots) for i in range(n)]


def recorded_rock(paths, physics: str = "rocking", settle: int = 30) -> dict:
    """Straight trots in the recordings, by (pattern, direction, speed, stride): the folded rock (deg,
    + toward the front foot in the air), measured and replayed, deduplicated (runs copied in two
    files). The pattern is "trot" for today's trot (its cycle 350 ms or more; "longtrot" in the
    recordings of 2026-11-21) and "old trot" for the trot replaced on 2026-11-22 (an 8-tick swing in a
    340 ms cycle, landing half a stride ahead: the recordings the pendulum was fitted to)."""
    seen, by = set(), defaultdict(list)
    for path, w in _recordings(paths):
        segs = []
        for m in w.marks:
            if m.word in ("stream", "timing"):
                p = m.fields.get("pattern")
                if p == "longtrot" or (p == "trot" and float(m.fields.get("period_ms", 340)) >= 345):
                    p = "trot"
                elif p == "trot":
                    p = "old trot"
                segs.append((int(m.fields["tick"]), p, m.fields.get("dir"),
                             m.fields.get("steer"), float(m.fields.get("speed", 0)), float(m.fields.get("stride_mm", 0))))
            elif m.word == "stop":
                segs.append((int(m.fields["tick"]), None, None, None, 0.0, 0.0))
        if not segs:
            continue
        for s in _slots(_replay(w, physics)):
            lab = None
            for i, sg in enumerate(segs):
                end = segs[i + 1][0] if i + 1 < len(segs) else 1 << 30
                if sg[0] + settle <= s[0] and s[0] + 10 < end:
                    lab = sg
            if (lab is None or lab[1] not in ("trot", "old trot") or lab[3] not in (None, "0.00")
                    or lab[2] not in ("forward", "back")):
                continue
            key = (s[0], s[1], round(s[2][0], 3))
            if key not in seen:
                seen.add(key)
                by[(lab[1], lab[2], lab[4], lab[5])].append(s)
    res = {}
    for k, ss in by.items():
        if len(ss) >= 4:
            m, v = _fold(ss, 2), _fold(ss, 3)
            # the signed tip at a diagonal's end: half the swing of the folded rock, signed by its end
            a = (max(m) - min(m)) / 2 * (1 if m[-3] > m[0] else -1)
            av = (max(v) - min(v)) / 2 * (1 if v[-3] > v[0] else -1)
            res[k] = {"n": len(ss), "measured": m, "virtual": v, "tip": a, "tip_virtual": av}
    return res


def fit(paths) -> None:
    rows = {}
    for physics in ("physics", "rocking"):
        for k, r in recorded_rock(paths, physics).items():
            rows.setdefault(k, {"n": r["n"], "tip": r["tip"]})[physics] = r["tip_virtual"]
    # today's trot: its swing and its landing from the planner's rules, so the fitted pendulum's
    # prediction there is the trot's (about 0: it lands at the balanced point); the old trot's: 8
    # ticks, half a stride
    print("| trot | stride | slots | tip measured | fitted pendulum | virtual body (Physics) | RockingPhysics |")
    print("|---|---|---|---|---|---|---|")
    for (p, d, v, s), r in sorted(rows.items(), key=lambda kv: (kv[0][0], kv[0][1] != "forward", kv[0][2])):
        sv = v if d == "forward" else -v
        land = s / 2 if d == "forward" else -s / 2
        w = 8
        if p == "trot":
            w = swing_for(v)
            x = TROT_FIT.land_x(w, sv)
            land = min(s / 2 + 6, max(0.0, x)) if sv > 0 else max(-s / 2 - 6, min(0.0, x))
        print(f"| {p} {d} {v:.0f} mm/s ({w} ticks) | {s} mm | {r['n']} "
              f"| {r['tip']:+.2f} | {TROT_FIT.tip_deg(w, sv, land):+.2f} "
              f"| {r.get('physics', float('nan')):+.2f} | {r.get('rocking', float('nan')):+.2f} |")


# ---- the build-up ---------------------------------------------------------------------------------

SPEEDS = (144, 160, 170, 180, 190, 200, 207)
ERRORS = {"nominal": {}, "mass 5 mm ahead": dict(body_model=shifted(5.0)),
          "mass 5 mm behind": dict(body_model=shifted(-5.0)),
          "mass 5 mm left, +50 g": dict(body_model=shifted(0.0, 5.0, 50.0)),
          "servos slower (100 + 100 ms)": dict(lag_ms=100, tau_ms=100),
          "servos faster (20 + 50 ms)": dict(lag_ms=20, tau_ms=50)}


def schedule(errors: bool = True) -> None:
    cfg = TROT
    print("| speed | swing | stride | cycle | lands ahead (half stride) | tip if it landed at half a stride "
          "| gain per mm off | virtual body: error, rocking / Physics | worst of the model errors (rocking) "
          "| fastest joint | feet x |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for v in SPEEDS:
        w, s, vv = timing(v, cfg=cfg)
        land = TROT_FIT.land_x(w, vv)
        r = walk(v)
        r0 = walk(v, physics="physics")
        worst = 0.0
        flags = []
        if errors:
            for name, kw in ERRORS.items():
                if name == "nominal":
                    continue
                e = walk(v, **kw)
                worst = max(worst, e["max_err"])
                if e["fell"] or e["catches"] or e["holds"]:
                    flags.append(f"{name}: fell {e['fell']} catches {e['catches']} holds {e['holds']}")
        print(f"| {v} | {w} ({w * 20} ms) | {s:.1f} | {2 * (w + 1) * 20} ms | {land:.1f} ({s / 2:.1f}) "
              f"| {TROT_FIT.tip_deg(w, vv, s / 2, clamp=False):+.1f} (clamped {TROT_FIT.tip_deg(w, vv, s / 2):+.1f}) "
              f"| {TROT_FIT.gain_deg_per_mm(w):.1f} | {r['max_err']:.1f} / {r0['max_err']:.1f} "
              f"(fell {r['fell'] or r0['fell']}, catches {r['catches'] + r0['catches']}) | {worst:.1f} "
              f"| {r['fastest']:.0f} | {r['x'][0]}..{r['x'][1]} |")
        for f in flags:
            print("    ", f)


def stops(speeds=(144, 180, 207), step: int = 2) -> None:
    """The quick stop (halt) at every ``step``-th tick of a cycle, from the trot at each speed."""
    print("| speed | stride | stopped (median / worst) | way (median / worst) | done (median) | peak tilt | fell |")
    print("|---|---|---|---|---|---|---|")
    for v in speeds:
        w, s, _ = timing(v)
        period = 2 * (w + 1)
        rs = [one_stop(v, at) for at in range(0, period, step)]
        print(f"| {v} | {s:.1f} | {statistics.median(r['stopped_s'] for r in rs):.2f} / "
              f"{max(r['stopped_s'] for r in rs):.2f} s | {statistics.median(r['way_mm'] for r in rs):.0f} / "
              f"{max(r['way_mm'] for r in rs):.0f} mm | {statistics.median(r['done_s'] for r in rs):.1f} s "
              f"| {max(r['tilt'] for r in rs):.1f} deg | {any(r['fell'] for r in rs)} |")


def one_stop(speed: float, at: int, physics: str = "rocking", warm_s: float = 3.0) -> dict:
    phys.Physics.INERTIA = True
    try:
        with physics_as(physics):
            sim = WalkSim("trot", "forward", speed)
            for _ in range(int(warm_s / TICK_S) + at):
                sim.step()
            sim.planner.halt()
            ph = sim.vb.stream.physics
            d0, pos, ticks, on, tilt = ph.dist_mm, [], 0, True, 0.0
            while on and ticks < 600:
                on = sim.step()
                ticks += 1
                p = sim.vb.stream.physics if sim.vb.stream is not None else None
                pos.append(p.dist_mm - d0 if p is ph else (pos[-1] if pos else 0.0))
                if sim.telemetry:
                    t = sim.telemetry[-1]
                    tilt = max(tilt, abs(t.pitch / 16), abs(t.roll / 16))
        final = pos[-1] if pos else 0.0
        stopped = max((i + 1 for i, x in enumerate(pos) if abs(x - final) > 3.0), default=0)
        return {"stopped_s": stopped * TICK_S, "way_mm": pos[stopped - 1] if stopped else 0.0,
                "done_s": ticks * TICK_S, "tilt": tilt, "fell": sim.fell()}
    finally:
        phys.Physics.INERTIA = False


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("what", choices=("limits", "fit", "schedule", "stops"))
    ap.add_argument("files", nargs="*", help="recordings for fit (default: ~/kilo-logs/*walk*.krec)")
    ap.add_argument("--no-errors", action="store_true", help="schedule: the nominal model only")
    a = ap.parse_args(argv)
    if a.what == "limits":
        limits()
    elif a.what == "fit":
        fit(a.files or sorted(glob.glob(os.path.expanduser("~/kilo-logs/*walk*.krec"))))
    elif a.what == "schedule":
        schedule(not a.no_errors)
    else:
        stops()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
