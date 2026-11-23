"""The righting experiment: getting up from a side or his back by moves reasoned on the body model.

    python -m kilo.intent.righting            design, check and print the report (a few minutes)
    python -m kilo.intent.righting --write    ...and write righting/*.json
    python -m kilo.intent.righting --check    exit 1 if the committed files are not what this makes
    python -m kilo.intent.righting --static   only the static part (seconds)

docs/righting.md is the owner's description. It is an EXPERIMENT: the lying
model (kilo.tricks.righting) does not reproduce the body's rc on the
recorded falls (docs/righting-results.md), so what is shipped here is what
the model says, checked over the numbers it is unsure of, not something
seen to work on Kilo.

THE REASONING, on Kilo's body as the model has it. He rolls about his long
axis (the recordings: the nose stays level within 10 degrees through every
fall and every try of rc), so the question is one-dimensional: his centre of
mass height over the roll angle, for a pose of the joints (``profile``). Its
dips are where he lies still; he rolls downhill. His legs move only in their
own plane (no sideways joint), so lying on a side they can push only toward
his back or his belly.

1. STATIC: no single pose takes him from where he lies on a side (rolled
   26-40 degrees toward his back, as the recordings show) downhill to his
   belly (``static_search``: every combination of leg poses on a grid, in
   the model and its variants). The legs of the low side can lever him
   only up to about 15 degrees past the side (their feet reach 50 mm toward
   his back), and there his centre of mass is still behind the shoulder
   servo he pivots on. So getting up needs momentum: it is a throw, as the
   body's rc is.
2. THE THROW (``side_move``), for his right side down (mirrored for the
   left): (a) WIND: slowly, the low side's legs forward and back along his
   body with the knees bent (their feet clear of the floor), the high side's
   legs reaching toward his back, the head up; (b) KICK and THROW together, as
   fast as the servos go: the low side's knees straighten so their feet hit
   the floor beyond his back and push it away (the floor turns him toward
   his belly), while the high side's legs swing over to the standing pose
   (their weight goes toward his belly) and the head tucks; (c) FOLD: the
   low side's legs straight along his body, out of the way of the roll;
   (d) REST. In the exploration (2026-11-22) the kick alone got him over in
   27 of 95 runs, the throw alone in 12, together in 89 (the report).
3. FROM HIS BACK (``back_move``): the left side's feet kick at the floor
   beyond his back and lift that side; the right legs, as for standing,
   stop him on his right side, and the side move follows
   (kilo.intent.graph RIGHTING).

The head turns (pan) and the tail are left at 0: which way a positive angle
turns them is not known (docs/righting.md, measurements).

THE CHECK. Each move is played in the compliant dynamics with Kilo lying
(kilo.tricks.dynamics + kilo.tricks.righting.KinLying: the moving parts'
momentum, the head's mass on the neck, the shoulder servos and the back's
outline as contacts) from several lying starts, on every variant of
``variants()``. It is shipped when it ends where it means to in at least
SHIP_FRACTION of them, and no leg servo carries more than TORQUE_MAX
(kg cm, 0.1 s mean) in any. The trick is FALL_OK (the body's fall check is off
while it plays), has no roll limit, and ends in REST.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import multiprocessing as mp
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from kilo.calibration import POSTURES
from kilo.tricks import dynamics as D
from kilo.tricks import righting as R
from kilo.tricks.model import Keyframe, Posture, Trick

DIR = Path(__file__).resolve().parent / "righting"
REPORT = DIR / "report.json"
NAMES = ("righting_side", "righting_back")
J = D.JOINTS
STAND = dict(zip(J, map(float, POSTURES["stand"])))
REST = dict(zip(J, map(float, POSTURES["rest"])))
# REST with the head straight and the tail at 0: the moves' last frame (the same mirrored or not; the
# body's own REST, which turns them, follows as the end posture)
REST0 = dict(REST, head_pan=0.0, tail=0.0)
SIT = dict(zip(J, map(float, POSTURES["sit"])))

TORQUE_MAX = 2.4        # kg cm on a leg servo (kilo.intent.moves, rear_up: 80 % of the estimated stall)
SHIP_FRACTION = 0.75    # ends where it means to in at least this share of the variants and starts (set to
                        # 0.75 after the back move came out at 79 %: an experiment's bar, not a proof)
STOP_TICKS = 25         # a stopped move goes to REST over this (0.5 s)
WIND_TICKS, WIND_HOLD = 30, 25
KICK_TICKS, KICK_HOLD = 3, 15      # the servos' safety slew (10 deg a tick) sets the real speed
FOLD_TICKS, FOLD_HOLD = 8, 10
REST_TICKS, REST_HOLD = 25, 25

# Starts: (name, the joints he lies with, the roll angle: atan2(-up_y, up_z), right side down is -90).
# The recordings: on his right side 116 degrees, on his left 118-119, on his back 170-177.
SIDE_STARTS = (("stand, right side -110", STAND, -110.0), ("stand, right side -120", STAND, -120.0),
               ("stand, right side -130", STAND, -130.0), ("sit, right side -116", SIT, -116.0),
               ("rest, right side -120", REST, -120.0))
BACK_STARTS = (("stand, on his back 180", STAND, 180.0), ("stand, on his back +172", STAND, 172.0),
               ("stand, on his back -172", STAND, -172.0), ("rest, on his back -175", REST, -175.0))


def _js(**k) -> dict:
    return {**{j: 0.0 for j in J}, **k}


# ---- the moves ----------------------------------------------------------------------------------
def side_move() -> list[tuple[str, dict, int, int]]:
    """(name, joints, dur, hold) per frame, his right side down."""
    wind = _js(head_tilt=30.0,
               # the low side: along the body, knees bent 70 (feet 17 mm toward the back: clear of the floor)
               rf_shoulder=-90.0, rf_knee=70.0, rb_shoulder=90.0, rb_knee=-70.0,
               # the high side: reaching toward his back (feet 50 mm behind it)
               lf_shoulder=-90.0, lf_knee=0.0, lb_shoulder=90.0, lb_knee=0.0)
    kick = dict(wind, head_tilt=-75.0, rf_knee=0.0, rb_knee=0.0,          # the low feet hit the floor beyond his back
                lf_shoulder=STAND["lf_shoulder"], lf_knee=STAND["lf_knee"],  # the high legs over to standing
                lb_shoulder=STAND["lb_shoulder"], lb_knee=STAND["lb_knee"])
    fold = dict(kick, rf_knee=90.0, rb_knee=-90.0)                          # the low legs straight along the body
    return [("wind", wind, WIND_TICKS, WIND_HOLD), ("kick and throw", kick, KICK_TICKS, KICK_HOLD),
            ("fold", fold, FOLD_TICKS, FOLD_HOLD), ("rest", REST0, REST_TICKS, REST_HOLD)]


def back_move() -> list[tuple[str, dict, int, int]]:
    """From his back onto his right side: the left feet kick at the floor beyond his back."""
    wind = _js(lf_shoulder=-90.0, lf_knee=70.0, lb_shoulder=90.0, lb_knee=-70.0,
               rf_shoulder=STAND["rf_shoulder"], rf_knee=STAND["rf_knee"],     # the right legs as standing: they
               rb_shoulder=STAND["rb_shoulder"], rb_knee=STAND["rb_knee"])     # stop the roll on his right side
    kick = dict(wind, lf_knee=0.0, lb_knee=0.0)
    return [("wind", wind, WIND_TICKS, WIND_HOLD), ("kick", kick, KICK_TICKS, 25),
            ("rest", REST0, REST_TICKS, REST_HOLD)]


MOVES = {"righting_side": (side_move, SIDE_STARTS, "belly"), "righting_back": (back_move, BACK_STARTS, "right side")}


def ends_well(goal: str, rx: float) -> bool:
    """Where the move means to leave him, by the roll angle once settled."""
    if goal == "belly":
        return abs(rx) < 50.0
    return -160.0 < rx < -50.0


# ---- the static picture -------------------------------------------------------------------------
PHI = np.radians(np.arange(-180, 181, 1.0))


def profile(js: dict, p: D.Params = D.NOMINAL, ly: R.Lying | None = None) -> np.ndarray:
    """The centre of mass's height above the floor (mm) for roll angles -180..180 (1 degree steps),
    rolling about his long axis with the nose level, in the pose ``js``."""
    kin = R.KinLying(p, ly)
    P, c, _ = kin.state(kin.clamp(js))
    ups = np.stack([np.zeros_like(PHI), -np.sin(PHI), np.cos(PHI)], 1)
    return (((c - P) @ ups.T) + kin.radius[:, None]).max(0)


def downhill(u: np.ndarray, start_deg: float) -> int:
    """Where he comes to rest rolling downhill from start_deg (degrees)."""
    k = int(round(start_deg)) + 180
    for _ in range(361):
        lo, hi = u[k - 1], u[(k + 1) % 361]
        if u[k] <= lo and u[k] <= hi:
            break
        k = (k + 1) % 361 if hi < lo else k - 1
    return k - 180


def static_search(vs=None) -> dict:
    """Every pose on a grid of the legs (his right side down; head up, level or tucked): in how many
    variants does rolling downhill from 110..130 degrees end belly-side (within 75 of upright)?"""
    vs = vs if vs is not None else variants()
    lof = [(-90, 0), (-90, 30), (-60, 0), (-30, 30), (30, 30), (-90, 90), (60, 60)]
    lob = [(90, 0), (90, -30), (60, 0), (30, -30), (-30, -30), (90, -90), (-60, -60)]
    hif = [(0, 90), (30, 30), (-30, 60), (60, 90), (-60, 100)]
    hib = [(0, -90), (-30, -30), (30, -60), (-60, -90), (60, -100)]
    best = 0
    n = 0
    for (a, b), (c, d), (e, f), (g, h), tilt in itertools.product(lof, lob, hif, hib, (-70, 0, 30)):
        js = _js(head_tilt=float(tilt), rf_shoulder=a, rf_knee=b, rb_shoulder=c, rb_knee=d, lf_shoulder=e, lf_knee=f,
                 lb_shoulder=g, lb_knee=h)
        ok = 0
        for _, p, ly in vs:
            u = profile(js, p, ly)
            ok += all(-75 < downhill(u, s) < 30 for s in (-110, -120, -130))
        best = max(best, ok)
        n += 1
    return {"poses": n, "variants": len(vs), "best_variants_rolled_belly_side": best}


# ---- the check ----------------------------------------------------------------------------------
def variants() -> list[tuple[str, D.Params, R.Lying]]:
    """The numbers the lying model is unsure of, one at a time: the tricks' fit; its centre of mass
    10 mm lower and higher, 3 mm either side; softer legs; each estimate of the outline a quarter
    either way; the outline under which rc's first frame rolls him onto his back as it did on Kilo
    (docs/righting-results.md); the tail the other way; slippery and grippy floors; slow servos."""
    base, ly = D.NOMINAL, R.Lying()
    out = [("fit", base, ly)]
    for k, v in (("com_dz", -10.0), ("com_dz", 10.0), ("com_dy", -3.0), ("com_dy", 3.0), ("sag", 1.5)):
        out.append((f"{k} {v:+g}", base.with_(**{k: getattr(base, k) + v}), ly))
    for k in ("hip_r", "top_z", "top_y", "ear_z"):
        for s in (0.75, 1.25):
            out.append((f"{k} x{s:g}", base, ly.with_(**{k: getattr(ly, k) * s})))
    out.append(("the outline that rolls him as on Kilo", base, ly.with_(top_y=15.0, top_z=30.0, ear_z=55.0)))
    out.append(("tail the other way", base.with_(tail_sign=-1.0), ly))
    out.append(("mu 0.3", base.with_(mu=0.3), ly))
    out.append(("mu 1.0", base.with_(mu=1.0), ly))
    out.append(("slow servos 70+100 ms", base.with_(lag_ms=70.0, tau_ms=100.0), ly))
    return out


def _rx(u) -> float:
    return math.degrees(math.atan2(-u[1], u[2]))


def play(frames, start_js: dict, start_rx: float, p: D.Params, ly: R.Lying, after_s: float = 1.0):
    """(roll angle once settled, the largest leg servo load kg cm, the trace) of the frames in the
    lying model, from start_rx with start_js settled first."""
    a = math.radians(start_rx)
    up0 = np.array([0.0, -math.sin(a), math.cos(a)])
    fr = [(js, d, h) for _, js, d, h in frames]
    total = sum(d + h for _, d, h in fr) * D.TICK_S
    pr = D.simulate(fr, p, start=start_js, settle_s=1.0, t_end=total + after_s, R0=R.r0_for(up0),
                    kin=R.KinLying(p, ly))
    torque = max((v for v, _ in pr.torque.values()), default=0.0)
    return _rx(pr.up[-1]), torque, pr


def _job(a):
    name, start, vname, p, ly = a
    fn, starts, goal = MOVES[name]
    sname, sjs, srx = start
    end, torque, _ = play(fn(), sjs, srx, p, ly)
    return {"move": name, "start": sname, "variant": vname, "end_roll": round(end, 1), "torque": round(torque, 2),
            "ok": ends_well(goal, end)}


def check(workers: int = 4) -> dict:
    vs = variants()
    jobs = [(name, st, vn, p, ly) for name, (_, starts, _) in MOVES.items() for st in starts for vn, p, ly in vs]
    if workers <= 1:
        rows = [_job(j) for j in jobs]
    else:
        with mp.get_context("fork").Pool(workers) as pool:
            rows = pool.map(_job, jobs, chunksize=2)
    out = {}
    for name in MOVES:
        rs = [r for r in rows if r["move"] == name]
        ok = sum(r["ok"] for r in rs)
        torque = max(r["torque"] for r in rs)
        failed = sorted({f"{r['variant']} ({r['start']}: ends at {r['end_roll']:+.0f})" for r in rs if not r["ok"]})
        out[name] = {"runs": len(rs), "ok": ok, "max_torque_kg_cm": torque,
                     "shipped": ok >= SHIP_FRACTION * len(rs) and torque <= TORQUE_MAX, "failed": failed}
    return out


# ---- the files ----------------------------------------------------------------------------------
def build(name: str, rep: dict) -> Trick:
    fn, _, goal = MOVES[name]
    r = rep[name]
    summary = (f"ends {'on his belly' if goal == 'belly' else 'on his right side'} in {r['ok']} of {r['runs']} "
               f"simulated runs (variants x starts), "
               f"leg servos at most {r['max_torque_kg_cm']:.2f} kg cm")
    kfs = tuple(Keyframe.from_joints(js, dur=d, hold=h) for _, js, d, h in fn())
    return Trick(
        name="intent_" + name, frames=kfs, end_posture=Posture.REST, expected_roll=0, expected_pitch=0,
        stop_ticks=STOP_TICKS, fall_ok=True,
        source={"kind": "authored", "design": "kilo.intent.righting", "doc": "docs/righting.md",
                "frames": [f for f, _, _, _ in fn()], "summary": summary, "check": r},
        notes=("the righting EXPERIMENT (docs/righting.md): reasoned on the lying model, which does not reproduce "
               "the body's rc on the recorded falls; nothing here has been seen on Kilo",
               "his right side down; the mind plays it mirrored from his left side",
               "FALL_OK: the body's fall check is off while it plays; it ends in REST"),
    ).validate()


def make(workers: int = 4) -> tuple[dict, dict]:
    rep = check(workers)
    files = {f"{n}.json": build(n, rep).to_json() for n in NAMES}
    doc = {"made_by": "python -m kilo.intent.righting --write", "ship_fraction": SHIP_FRACTION,
           "torque_max_kg_cm": TORQUE_MAX, "variants": [v for v, _, _ in variants()], "moves": rep}
    return doc, files


def load_move(name: str) -> Trick:
    return Trick.from_json((DIR / f"{name}.json").read_text())


def load_report() -> dict:
    return json.loads(REPORT.read_text())


def report_text(doc: dict) -> str:
    out = []
    for name, r in doc["moves"].items():
        out.append(f"{name}: {'SHIPPED' if r['shipped'] else 'NOT shipped'} as the experiment: "
                   f"ends where it means to in {r['ok']} of {r['runs']} runs, leg servos at most "
                   f"{r['max_torque_kg_cm']:.2f} kg cm (limit {TORQUE_MAX:g})")
        for f in r["failed"]:
            out.append(f"    not in: {f}")
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--static", action="store_true")
    ap.add_argument("--workers", type=int, default=4)
    a = ap.parse_args(argv)
    if a.static:
        print(json.dumps(static_search()))
        return 0
    doc, files = make(a.workers)
    print(report_text(doc))
    want = dict(files)
    want[REPORT.name] = json.dumps(doc, indent=1) + "\n"
    if a.write:
        DIR.mkdir(exist_ok=True)
        for old in DIR.glob("*.json"):
            if old.name not in want:
                old.unlink()
        for n, text in want.items():
            (DIR / n).write_text(text)
        print(f"wrote {len(want)} files in {DIR}")
        return 0
    if a.check:
        have = {p.name: p.read_text() for p in DIR.glob("*.json")} if DIR.exists() else {}
        stale = sorted(n for n in set(want) | set(have) if want.get(n) != have.get(n))
        if stale:
            print("stale (run python -m kilo.intent.righting --write): " + ", ".join(stale), file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
