"""The engine's judge (docs/movement-engine.md, step 2; results in
docs/movement-engine-results.md, "Step 2: the solver judged"): the engine's
walks, planned by the kernel's own C++ (kernel/host's engine-judge, no body),
played tick by tick on the virtual body with the judge's physics
(kilo.measure.score.Judge: RockingPhysics, the weight held by three feet as
soon as it is inside them, the support lagging 55 ms, the legs giving as the
model says) and the body's own balance loop (kilo.vbody.pose.PosePlayer).

    PYTHONPATH=mind python mind/tests/enginejudge.py table          # every gait at the remote's speeds
    PYTHONPATH=mind python mind/tests/enginejudge.py changes        # speed changes, stops, quick stops, steer
    PYTHONPATH=mind python mind/tests/enginejudge.py one walk 58    # one walk, its numbers
    PYTHONPATH=mind python mind/tests/enginejudge.py one walk 58 --at 300:stop --at 200:speed=40
    ... --bin PATH                                                  # another engine-judge (another build)

What each number is:

- margin: the plan's least margin (mm) of the centre of mass inside the
  feet on the ground, on the ticks with one foot in the air (the model's).
- roll swing / pitch swing: the measured roll's (pitch's) peak to peak per
  cycle, mean over the cycles (degrees): how much he rocks.
- rate rms: the measured pitch and roll rates' rms (deg/s).
- error rms (pitch / roll) and largest: the measured attitude against the
  one the plan predicted and sent as the balance's reference ("kilo model
  score" style: how well the plan predicts the body).
- speed: how far the body went over the ground in the judged window, per
  second (mm/s).
- fastest joint: the fastest joint between two commanded poses (deg/s; the
  body's slew allows 500).
- cut: ticks on which the body's slew or reach cut the pose.
- worst of the model errors: the largest error over the same walk with the
  model wrong (the centre of mass 3 mm back, 5 mm ahead, 3 mm right; the
  feet 133 x 69 mm; the servos slower and faster), and the falls among them.

All of it is simulation: a rigid body on point feet (docs/righting-results.md
says how far the simulators are from the floor).
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from dataclasses import replace

from kilo import engine as E
from kilo.engine import Row
from kilo.measure.score import Judge
from kilo.planner.model import LEGS, BodyModel, Mass
from kilo.protocol import kilo_protocol as P
from kilo.vbody.pose import Key, PosePlayer
from kilo.vbody.sim import BALANCE_DEFAULTS

BIN = E.BIN
LOOKAHEAD = 3   # the walker's: poses 3 ticks ahead of the body
M = BodyModel()


def plan(gait: str, speed: float | None = None, ticks: int = 600, at=(), direction: str = "forward",
         steer: float = 0.0, binary: str | None = None, sets=()):
    """kilo.engine.plan (the kernel's engine-judge), its rows, stats and done tick."""
    return E.plan(gait, speed, ticks, at, direction, steer, binary, sets)


def body_model(dx: float = 0.0, dy: float = 0.0, base: BodyModel = M) -> BodyModel:
    """The body model with its centre of mass moved by moving the body's own mass."""
    b = base.body
    k = base.total_g / b.grams
    return replace(base, body=Mass(b.grams, b.x + dx * k, b.y + dy * k, b.z))


ERRORS = {"CoM 3 mm back": dict(model=body_model(-3.0)), "CoM 5 mm ahead": dict(model=body_model(5.0)),
          "CoM 3 mm right": dict(model=body_model(0, -3.0)),
          "feet 133 x 69": dict(model=replace(M, length_mm=133.0, track_mm=69.0)),
          "servos slower (100 + 100 ms)": dict(lag_ms=100, tau_ms=100),
          "servos faster (20 + 50 ms)": dict(lag_ms=20, tau_ms=50)}


def play(rows: list[Row], model: BodyModel = M, judge: Judge = Judge(), lag_ms=None, tau_ms=None,
         after: int = 60) -> dict:
    """The plan on the virtual body: per tick the measured attitude, rates, the body's distance."""
    old = PosePlayer.PHYSICS
    PosePlayer.PHYSICS = lambda m, **a: judge.make(m, **a)
    try:
        g = PosePlayer(0, model, lag_ms=lag_ms, tau_ms=tau_ms)
    finally:
        PosePlayer.PHYSICS = old
    bal = dict(BALANCE_DEFAULTS, enable=1, kp_q8=253)
    ph = g.physics
    k0 = 10  # the body's tick of the plan's tick 0 (the planner starts 0)
    out = {"tick": [], "pitch": [], "roll": [], "rp": [], "rr": [], "dist": [], "limited": [], "fell": False}
    n = len(rows) + after
    for k in range(1, k0 + n):
        for r in rows:
            if k < r.tick + k0 <= k + LOOKAHEAD and not any(q.tick == r.tick + k0 for q in g.queue) \
                    and r.tick + k0 > g.cur.tick:
                g.queue.append(Key(r.tick + k0, 0, r.support, list(r.x), list(r.z), pitch=r.pitch, roll=r.roll))
        g.queue.sort(key=lambda q: q.tick)
        g.tick(k, (ph.pitch, ph.roll), ph.rates, bal, True)
        ph.step()
        if k >= k0:
            out["tick"].append(k - k0)
            out["pitch"].append(ph.pitch)
            out["roll"].append(ph.roll)
            out["rp"].append(ph.rates[0])
            out["rr"].append(ph.rates[1])
            out["dist"].append(ph.dist_mm)
            out["limited"].append(g.limited)
        if ph.fallen:
            out["fell"] = True
            break
    return out


def joint_rate(rows: list[Row], model: BodyModel = M) -> float:
    worst = 0.0
    for a, b in zip(rows, rows[1:]):
        for leg in LEGS:
            t = model.ik(leg, a.x[leg], a.z[leg])
            u = model.ik(leg, b.x[leg], b.z[leg])
            worst = max(worst, abs(math.degrees(t[0] - u[0])), abs(math.degrees(t[1] - u[1])))
    return worst * 1000.0 / P.TICK_MS


def judge(rows: list[Row], warm: int = 150, end: int | None = None, errors: bool = True) -> dict:
    """The numbers of the module's docstring over the whole cycles in plan ticks warm..end (a cycle:
    from one lift of the left hind leg to the next)."""
    end = len(rows) if end is None else end
    sim = play(rows)
    lifts = [r.tick for a, r in zip(rows, rows[1:]) if r.lifting & 8 and not a.lifting & 8 and warm <= r.tick < end]
    if len(lifts) >= 2:
        warm, end = lifts[0], lifts[-1]
    w = [i for i, t in enumerate(sim["tick"]) if warm <= t < end and t < len(rows)]
    pred = {r.tick: (r.pitch, r.roll) for r in rows}
    ep = [sim["pitch"][i] - pred[sim["tick"][i]][0] for i in w]
    er = [sim["roll"][i] - pred[sim["tick"][i]][1] for i in w]
    rms = lambda v: math.sqrt(sum(x * x for x in v) / max(1, len(v)))  # noqa: E731
    at = {t: i for i, t in enumerate(sim["tick"])}
    cyc = [[at[t] for t in range(a, b) if t in at] for a, b in zip(lifts, lifts[1:])]
    swing = lambda key: statistics.mean(max(sim[key][i] for i in c) - min(sim[key][i] for i in c) for c in cyc) \
        if cyc else math.nan  # noqa: E731
    one_up = [r.margin for r in rows[warm:end] if bin(r.lifting).count("1") == 1]
    secs = len(w) * P.TICK_MS / 1000.0
    res = {
        "margin": min(one_up, default=math.nan),
        "roll_swing": swing("roll"), "pitch_swing": swing("pitch"),
        "rate_rms": rms([sim["rp"][i] for i in w] + [sim["rr"][i] for i in w]) * math.sqrt(2),
        "err_pitch": rms(ep), "err_roll": rms(er), "err_max": max((abs(x) for x in ep + er), default=0.0),
        "speed": (sim["dist"][w[-1]] - sim["dist"][w[0]]) / secs if w and secs > 0 else math.nan,
        "joint": joint_rate(rows[warm:end]), "cut": sum(1 for i in w if sim["limited"][i]),
        "fell": sim["fell"], "max_tilt": max((max(abs(sim["pitch"][i]), abs(sim["roll"][i])) for i in w), default=0.0),
    }
    if errors:
        worst, falls = res["err_max"], int(res["fell"])
        for kw in ERRORS.values():
            s = play(rows, **kw)
            ww = [i for i, t in enumerate(s["tick"]) if warm <= t < end and t < len(rows)]
            e = [abs(s["pitch"][i] - pred[s["tick"][i]][0]) for i in ww] + \
                [abs(s["roll"][i] - pred[s["tick"][i]][1]) for i in ww]
            worst = max(worst, max(e, default=0.0))
            falls += int(s["fell"])
        res["worst"], res["falls"] = worst, falls
    return res


CASES = [("walk", v) for v in (23.2, 40.6, 58.0, 66.7, 72.5)] + [("trot", v) for v in (57.6, 100.8, 144.0, 180.0, 207.0)] \
    + [("crawl", v) for v in (8.9, 15.5, 22.2, 25.0)]


def fmt(r: dict) -> str:
    return (f"{r['margin']:+.1f} | {r['roll_swing']:.2f} / {r['pitch_swing']:.2f} | {r['rate_rms']:.1f} "
            f"| {r['err_pitch']:.2f} / {r['err_roll']:.2f} | {r['err_max']:.1f} | {r['speed']:.0f} | {r['joint']:.0f} "
            f"| {r['cut']} | {r.get('worst', math.nan):.1f} / {r.get('falls', 0)}" + (" FELL" if r["fell"] else ""))


HEAD = ("| gait | speed | engine | margin | roll / pitch swing | rate rms | error rms p / r | largest | speed "
        "| fastest joint | cut | worst / falls of the model errors |\n|---|---|---|---|---|---|---|---|---|---|---|---|")


def engines(specs) -> dict:
    """NAME=PATH[;KEY=VALUE...] -> {name: (path, sets)}"""
    out = {}
    for spec in specs:
        name, rest = spec.split("=", 1)
        path, *sets = rest.split(";")
        out[name] = (path, sets)
    return out


def table(binaries: dict, cases=None) -> None:
    print(HEAD)
    for gait, v in cases or CASES:
        for name, (b, sets) in binaries.items():
            rows, _, _ = plan(gait, v, ticks=550, binary=b, sets=sets)
            print(f"| {gait} | {v} | {name} | " + fmt(judge(rows)) + " |")


# the changes: (gait, speed, what is asked at which tick)
SCENARIOS = [("walk", 40.6, ["200:speed=72.5"]), ("walk", 58.0, ["300:stop"]), ("walk", 58.0, ["300:halt"]),
             ("walk", 58.0, ["200:steer=1", "350:dir=back"]), ("trot", 100.8, ["200:speed=201.6"]),
             ("trot", 144.0, ["300:stop"]), ("trot", 201.6, ["300:halt"]), ("crawl", 22.2, ["300:stop"]),
             ("walk", 58.0, ["200:gait=trot"])]


def scenarios(binaries: dict) -> None:
    """Each change from tick 0 to its end: the largest tilt and error, the ticks until done, the falls."""
    print("| gait | speed | asked | engine | largest tilt | largest error | error rms p / r | done at tick | fell |")
    print("|---|---|---|---|---|---|---|---|---|")
    for gait, v, at in SCENARIOS:
        for name, (b, sets) in binaries.items():
            rows, _, done = plan(gait, v, ticks=900, at=at, binary=b, sets=sets)
            sim = play(rows)
            pred = {r.tick: (r.pitch, r.roll) for r in rows}
            w = [i for i, t in enumerate(sim["tick"]) if 50 <= t < len(rows)]
            ep = [sim["pitch"][i] - pred[sim["tick"][i]][0] for i in w]
            er = [sim["roll"][i] - pred[sim["tick"][i]][1] for i in w]
            rms = lambda x: math.sqrt(sum(a * a for a in x) / max(1, len(x)))  # noqa: E731
            tilt = max((max(abs(sim["pitch"][i]), abs(sim["roll"][i])) for i in w), default=0.0)
            print(f"| {gait} | {v} | {' '.join(at)} | {name} | {tilt:.1f} | {max(map(abs, ep + er)):.1f} "
                  f"| {rms(ep):.2f} / {rms(er):.2f} | {done if done is not None else '-'} | {sim['fell']} |")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=("table", "one", "changes"))
    ap.add_argument("gait", nargs="?", default="walk")
    ap.add_argument("speed", nargs="?", type=float)
    ap.add_argument("--at", action="append", default=[])
    ap.add_argument("--ticks", type=int, default=550)
    ap.add_argument("--warm", type=int, default=150)
    ap.add_argument("--bin", action="append", default=[],
                    help="NAME=PATH[;KEY=VALUE...] of an engine-judge and its --set (more than one: compared)")
    ap.add_argument("--gaits", default="", help="table: only these gaits (comma separated)")
    a = ap.parse_args(argv)
    bins = engines(a.bin) if a.bin else {"engine": (E.binary(), [])}
    if a.what == "changes":
        scenarios(bins)
        return 0
    if a.what == "table":
        g = a.gaits.split(",") if a.gaits else None
        table(bins, [c for c in CASES if g is None or c[0] in g])
        return 0
    print(HEAD)
    for name, (b, sets) in bins.items():
        rows, stats, done = plan(a.gait, a.speed, ticks=a.ticks, at=a.at, binary=b, sets=sets)
        r = judge(rows, warm=a.warm)
        print(f"| {a.gait} | {a.speed} | {name} | " + fmt(r) + f" | stats {stats} done {done}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
