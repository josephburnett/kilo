"""The body model judged by prediction: what Kilo was told, replayed through
the model and the virtual body, against what his tilt sensor read.

Every run of a recording in which the mind streamed feet (FEET in protocol 2
recordings, POSE with four feet since: the walks, trots and crawls, and the
self-measurement routine) is a *segment*: the feet the mind sent for each
body tick, the corrections the body's own balance loop reported adding, and
the pitch and roll the body reported. ``replay`` plays the segment's feet
through the virtual body's POSE player (kilo.vbody.pose,
the recorded corrections in place of its balance loop) on a judge physics
(kilo.vbody.rocking.RockingPhysics by default, its ``inset_mm`` and
``line_lag_ms`` as given) with a body model; the predicted reading is the
virtual body's attitude plus the model's IMU offset. ``score`` compares it
with the reading, tick by tick, in degrees.

The score of a model is the rms of (read - predicted) over the streaming (mode STREAM, GAIT in protocol 2) ticks of
a set of segments, pitch and roll apart. Two sets, fixed by boot
(``HOLDOUT``): the fit may look at the first only; the second says whether
what it found predicts what it never saw.

``fit`` finds the model numbers that predict the fit set best (least
squares on the readings, one number at a time, every evaluation a full
replay, in parallel). ``spread`` gives each number's +-: each boot's own
best value, their median and spread, the honest one here (the residuals are
far from independent tick to tick, so a curvature's +- would be much too
small).

    kilo model score [FILES] [--set NAME=VALUE ...]   the score of the model (or of the model so changed)
    kilo model score --fit PARAM,PARAM,...            find the numbers that predict best, and score them

Recordings are read from ~/kilo-logs (recursively) unless given; the same
boot is in several files, and each block of it is used once.
"""

from __future__ import annotations

import glob
import json
import math
import os
from collections import defaultdict
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np

from kilo.logs import flight, proto2
from kilo.logs import walkrec as W
from kilo.planner.model import BodyModel
from kilo.protocol import kilo_protocol as P

A16 = 16.0
SKIP_TICKS = 25        # each segment's first ticks are not scored (the virtual body starts level, in STANCE)
MAX_TILT_DEG = 20.0    # readings past this are a fall or a hand, not a pose the model is for
SEGMENT_CAP = 1000     # in the fit a segment counts as at most this many ticks (the routine's 4000 would swamp it)

# The held-out recordings: never seen by the fit. Boot 14 (old trot, long trot, crawl, walk), 16 and 17 (the
# new walk and trot of 2026-11-22), and the second self-measurement run of boot 15 (Kilo turned round).
HOLDOUT = ((14, None), (16, None), (17, None), (15, 2))
# Boots left out altogether: boot 3 (2026-11-17) was walked with Kilo held in a hand
# (docs/movement-engine-results.md §3), so what his feet did says nothing about the floor.
EXCLUDE_BOOTS = (3,)


# ---- the recordings, one stream per boot -------------------------------------------------------------
def default_files() -> list[str]:
    h = os.path.expanduser("~/kilo-logs")
    return sorted(glob.glob(h + "/**/*.krec", recursive=True))


def boots(paths) -> dict[int, flight.Stream]:
    """Every block of every file once ((boot, seq): the fullest copy), one Stream per boot."""
    best: dict[tuple[int, int], tuple[int, bytes]] = {}
    for p in paths:
        try:
            _, data = flight.read_file(p)
        except (ValueError, OSError):
            continue
        for b in flight.parse_blocks(data).blocks:
            if b.problem and b.problem != "cut":
                continue
            raw = data[b.offset:b.offset + b.have]
            if (b.boot, b.seq) not in best or len(raw) > best[(b.boot, b.seq)][0]:
                best[(b.boot, b.seq)] = (len(raw), raw)
    by: dict[int, list[tuple[int, bytes]]] = defaultdict(list)
    for (boot, seq), (_, raw) in best.items():
        by[boot].append((seq, raw))
    return {boot: flight.parse_blocks(b"".join(r for _, r in sorted(parts)), {"boot": boot})
            for boot, parts in sorted(by.items())}


# ---- a segment ------------------------------------------------------------------------------------------
@dataclass
class Feet:
    """One FEET as the player reads it."""
    tick: int
    x_mm: tuple
    z_mm: tuple
    pitch: int
    roll: int
    support: int


@dataclass
class Segment:
    name: str                 # "b16 walk 2"
    boot: int
    run: int
    what: str                 # the patterns streamed ("walk", "trot+crawl", ...) or "measure"
    k0: int                   # the first tick (a FEET)
    feet: dict = field(default_factory=dict)   # tick -> Feet
    meas: np.ndarray = None   # (n, 2) pitch, roll read; NaN where none or not GAIT
    corr: np.ndarray = None   # (n, 2) the body's corrections, held between readings
    plan: np.ndarray = None   # (n, 2) the attitude the FEET predicted (the planner's rigid model)

    @property
    def n(self) -> int:
        return len(self.meas)

    @property
    def held_out(self) -> bool:
        return any(self.boot == b and (r is None or self.run == r) for b, r in HOLDOUT)

    def part(self, i0: int, i1: int) -> "Segment":
        """Ticks i0..i1-1 of the segment (a FEET at i0 is needed: the player starts there)."""
        k0, k1 = self.k0 + i0, self.k0 + i1
        s = Segment(f"{self.name} [{i0}:{i1}]", self.boot, self.run, self.what, k0,
                    {k: f for k, f in self.feet.items() if k0 <= k < k1}, self.meas[i0:i1].copy(),
                    self.corr[i0:i1].copy())
        s.plan = _plan_of(s)
        return s

    def to_json(self) -> dict:
        return {"name": self.name, "boot": self.boot, "run": self.run, "what": self.what, "k0": self.k0,
                "feet": [[f.tick - self.k0, *f.x_mm, *f.z_mm, f.pitch, f.roll, f.support] for f in self.feet.values()],
                # TELEMETRY's own units (1/16 deg), so the file is small and exact; null: no reading
                "meas16": [[None if not np.isfinite(v) else int(round(v * A16)) for v in r] for r in self.meas],
                "corr16": np.round(self.corr * A16).astype(int).tolist()}

    @classmethod
    def from_json(cls, d: dict) -> "Segment":
        k0 = int(d["k0"])
        feet = {}
        for r in d["feet"]:
            t = k0 + int(r[0])
            feet[t] = Feet(t, tuple(r[1:5]), tuple(r[5:9]), int(r[9]), int(r[10]), int(r[11]))
        meas = np.array([[np.nan if v is None else v / A16 for v in r] for r in d["meas16"]], float).reshape(-1, 2)
        corr = np.array(d["corr16"], float).reshape(-1, 2) / A16
        s = cls(d["name"], int(d["boot"]), int(d["run"]), d["what"], k0, feet, meas, corr)
        s.plan = _plan_of(s)
        return s


def _plan_of(s: Segment) -> np.ndarray:
    plan = np.full((s.n, 2), np.nan)
    last = None
    for i in range(s.n):
        f = s.feet.get(s.k0 + i)
        if f is not None:
            last = f
        if last is not None:
            plan[i] = (last.pitch / A16, last.roll / A16)
    return plan


def _patterns(marks) -> str:
    out = []
    for m in marks:
        p = m.fields.get("pattern")
        if m.word in ("stream", "timing", "change") and p:
            p = "trot" if p == "longtrot" else p
            if p not in out:
                out.append(p)
    return "+".join(out) or "?"


def segments_of(boot: int, s: flight.Stream, min_ticks: int = 50) -> list[Segment]:
    out = []
    for run in s.runs:
        lo, hi = flight.span_of(s, run)
        feet: dict[int, Feet] = {}
        tel: dict[int, Any] = {}
        last = None
        for f in s.frames:
            if f.msg is None or not lo <= f.t_us <= hi:
                continue
            fm = proto2.feet_of(f.msg) if f.direction == "tx" else None
            if fm is not None and None not in fm[0]:  # a tick of a walk (FEET, or POSE with four feet)
                last = f.msg.tick if last is None else last
                k = W._unwrap(f.msg.tick, last)
                m = f.msg
                feet[k] = Feet(k, tuple(fm[0]), tuple(fm[1]), m.pitch, m.roll, m.support)  # the last one plays
            elif f.direction == "rx" and isinstance(f.msg, P.Telemetry):
                last = f.msg.tick if last is None else last
                last = W._unwrap(f.msg.tick, last)
                tel[last] = f.msg
        gait = [k for k, t in tel.items() if t.mode == P.Mode.STREAM]
        if not feet or len([k for k in gait if k in feet]) < min_ticks:
            continue
        k0, k1 = min(feet), max(gait)
        n = k1 - k0 + 1
        meas = np.full((n, 2), np.nan)
        corr = np.zeros((n, 2))
        c = (0.0, 0.0)
        for i in range(n):
            t = tel.get(k0 + i)
            if t is not None:
                c = (t.corr_pitch / A16, t.corr_roll / A16)
                if t.mode == P.Mode.STREAM:
                    meas[i] = (t.pitch / A16, t.roll / A16)
            corr[i] = c
        begin = run.begin_us if run.begin_us is not None else lo
        end = run.end_us if run.end_us is not None else hi
        marks = [m for m in s.marks if begin <= m.t_us <= end]
        what = "measure" if any(m.word == "measure" for m in marks) else _patterns(marks)
        seg = Segment(f"b{boot} {run.kind} {run.id}", boot, run.id, what, k0, feet, meas, corr)
        seg.plan = _plan_of(seg)
        out.append(seg)
    return out


def load_segments(paths=None) -> list[Segment]:
    """Every segment of the recordings (default: ~/kilo-logs), or of JSON files written by ``save_segments``."""
    paths = list(paths) if paths else default_files()
    js = [p for p in paths if str(p).endswith(".json")]
    out = []
    for p in js:
        d = json.loads(Path(p).read_text())
        out += [Segment.from_json(x) for x in (d["segments"] if isinstance(d, dict) else d)]
    rec = [p for p in paths if not str(p).endswith(".json")]
    if rec:
        for boot, s in boots(rec).items():
            out += segments_of(boot, s)
    return [s for s in out if s.boot not in EXCLUDE_BOOTS]


def save_segments(segs: list[Segment], path) -> Path:
    path = Path(path)
    path.write_text(json.dumps({"format": "kilo-model-segments", "segments": [s.to_json() for s in segs]}))
    return path


# ---- the replay -----------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Judge:
    """The virtual body's physics the model is judged on."""
    physics: str = "rocking"     # "rocking" (RockingPhysics) or "physics" (Physics)
    # the judge's own two numbers, fitted with the model's (docs/model-fit-results.md): the weight is held
    # by three feet as soon as it is inside them, and the feet on the ground lag the plan 55 ms more than
    # the swing (the long-trot study's line_lag_ms, there 50)
    inset_mm: float = 0.0        # how far inside three feet the mass must be before they hold it
    line_lag_ms: float = 55.0    # RockingPhysics' support lag
    legs_give: bool = True       # the legs give as the model says (sag_deg_per_kg_cm, servo_give)

    def make(self, model: BodyModel, **kw):
        from kilo.vbody.physics import Physics
        from kilo.vbody.rocking import RockingPhysics
        if self.physics == "physics":
            return Physics(model, inset_mm=self.inset_mm, legs_give=self.legs_give, **kw)
        return RockingPhysics(model, inset_mm=self.inset_mm, line_lag_ms=self.line_lag_ms, legs_give=self.legs_give,
                              **kw)


def replay(seg: Segment, model: BodyModel, judge: Judge = Judge()) -> np.ndarray:
    """(n, 2) the predicted reading (pitch, roll) for every tick of the segment: the virtual body's
    attitude plus the model's IMU offset."""
    from kilo.vbody.pose import Key, PosePlayer
    old = PosePlayer.PHYSICS
    PosePlayer.PHYSICS = lambda m, **a: judge.make(m, **a)
    try:
        g = PosePlayer(seg.k0 - 1, model)
    finally:
        PosePlayer.PHYSICS = old
    rec = [0.0, 0.0]
    g._balance = lambda *a, **k: g.corr.__setitem__(slice(None), rec)
    out = np.empty((seg.n, 2))
    ph = g.physics
    for i in range(seg.n):
        k = seg.k0 + i
        f = seg.feet.get(k)
        if f is not None:
            g.queue.append(Key(k, 0, f.support, list(f.x_mm), list(f.z_mm), pitch=f.pitch / 16.0, roll=f.roll / 16.0))
        rec = [float(seg.corr[i, 0]), float(seg.corr[i, 1])]
        g.tick(k, (ph.pitch, ph.roll), ph.rates, {}, True)
        ph.step()
        out[i] = (ph.pitch + model.imu_pitch_deg, ph.roll + model.imu_roll_deg)
        if ph.fallen:
            out[i + 1:] = np.nan
            break
    return out


def scored(seg: Segment) -> np.ndarray:
    """The ticks that count: GAIT readings after the first SKIP_TICKS, within MAX_TILT_DEG."""
    ok = np.isfinite(seg.meas).all(1)
    ok[:SKIP_TICKS] = False
    ok &= (np.abs(seg.meas) < MAX_TILT_DEG).all(1)
    return ok


def residuals(seg: Segment, pred: np.ndarray) -> np.ndarray:
    """(m, 2) read - predicted at the ticks that count (a prediction lost to a virtual fall: 30 deg off)."""
    ok = scored(seg)
    r = seg.meas[ok] - pred[ok]
    return np.where(np.isfinite(r), r, 30.0)


def score(seg: Segment, pred: np.ndarray) -> dict:
    r = residuals(seg, pred)
    if not len(r):
        return {"n": 0}
    return {"n": int(len(r)), "rms_pitch": float(np.sqrt((r[:, 0] ** 2).mean())),
            "rms_roll": float(np.sqrt((r[:, 1] ** 2).mean())),
            "bias_pitch": float(r[:, 0].mean()), "bias_roll": float(r[:, 1].mean()),
            "shape_pitch": float(r[:, 0].std()), "shape_roll": float(r[:, 1].std())}


def total(scores: list[dict]) -> dict:
    """The rms over every tick of several segments' scores."""
    n = sum(s["n"] for s in scores)
    if not n:
        return {"n": 0}
    out = {"n": n}
    for k in ("pitch", "roll"):
        out[f"rms_{k}"] = math.sqrt(sum(s["n"] * s[f"rms_{k}"] ** 2 for s in scores) / n)
    out["rms"] = math.sqrt((out["rms_pitch"] ** 2 + out["rms_roll"] ** 2) / 2)
    return out


# ---- the model's numbers by name ------------------------------------------------------------------------
# What a fit may change: name -> (the step for its derivative, a short description)
PARAMS = {
    "com_x": (0.5, "centre of mass forward (mm, the correction com_dx_mm)"),
    "com_y": (0.5, "centre of mass to the left (mm, com_dy_mm)"),
    "com_z": (2.0, "centre of mass up (mm, com_dz_mm)"),
    "length": (1.0, "feet front to back (mm, LEG_LB_MM)"),
    "track": (1.0, "feet left to right (mm, LEG_W_MM)"),
    "imu_pitch": (0.1, "tilt sensor reads this nose down on a level body (deg)"),
    "imu_roll": (0.1, "...and this right side up (deg)"),
    "lag": (5.0, "every servo's dead time (ms)"),
    "tau": (5.0, "every servo's time constant (ms)"),
    "sag": (0.5, "every leg servo's give (deg per kg cm)"),
    "inset": (0.5, "the judge: mm inside three feet before they hold the weight"),
    "line_lag": (10.0, "the judge: the feet on the ground lag the plan this much more than the swing (ms)"),
}
for _i, _leg in enumerate(("lf", "rf", "rb", "lb")):
    PARAMS[f"dz_{_leg}"] = (0.3, f"the {_leg.upper()} leg longer than commanded (mm, leg_dz_mm)")
    PARAMS[f"give_{_leg}"] = (0.25, f"the {_leg.upper()} leg's servos give this many times the others (servo_give)")
    PARAMS[f"lag_{_leg}"] = (10.0, f"the {_leg.upper()} leg's servos start this much later (ms, servo_lag_d_ms)")
LEG_IDX = {"lf": 0, "rf": 1, "rb": 2, "lb": 3}
SHOULDER = (8, 9, 10, 11)
KNEE = (12, 13, 14, 15)


def values_of(model: BodyModel, judge: Judge) -> dict[str, float]:
    v = {"com_x": model.com_dx_mm, "com_y": model.com_dy_mm, "com_z": model.com_dz_mm, "length": model.length_mm,
         "track": model.track_mm, "imu_pitch": model.imu_pitch_deg, "imu_roll": model.imu_roll_deg,
         "lag": model.servo_lag_ms, "tau": model.servo_tau_ms, "sag": model.sag_deg_per_kg_cm,
         "inset": judge.inset_mm, "line_lag": judge.line_lag_ms}
    for leg, i in LEG_IDX.items():
        v[f"dz_{leg}"] = model.leg_dz_mm[i]
        v[f"give_{leg}"] = (model.servo_give[SHOULDER[i]] + model.servo_give[KNEE[i]]) / 2
        v[f"lag_{leg}"] = (model.servo_lag_d_ms[SHOULDER[i]] + model.servo_lag_d_ms[KNEE[i]]) / 2
    return v


def with_values(model: BodyModel, judge: Judge, v: dict[str, float]) -> tuple[BodyModel, Judge]:
    """The model and judge with these numbers (names of PARAMS) in place of theirs."""
    kw: dict[str, Any] = {}
    simple = {"com_x": "com_dx_mm", "com_y": "com_dy_mm", "com_z": "com_dz_mm", "length": "length_mm",
              "track": "track_mm", "imu_pitch": "imu_pitch_deg", "imu_roll": "imu_roll_deg", "lag": "servo_lag_ms",
              "tau": "servo_tau_ms", "sag": "sag_deg_per_kg_cm"}
    for k, f in simple.items():
        if k in v:
            kw[f] = float(v[k])
    dz, give, lag = list(model.leg_dz_mm), list(model.servo_give), list(model.servo_lag_d_ms)
    for leg, i in LEG_IDX.items():
        if f"dz_{leg}" in v:
            dz[i] = float(v[f"dz_{leg}"])
        for name, arr in ((f"give_{leg}", give), (f"lag_{leg}", lag)):
            if name in v:
                arr[SHOULDER[i]] = arr[KNEE[i]] = float(v[name])
    kw.update(leg_dz_mm=tuple(dz), servo_give=tuple(give), servo_lag_d_ms=tuple(lag))
    j = judge
    if "inset" in v:
        j = replace(j, inset_mm=float(v["inset"]))
    if "line_lag" in v:
        j = replace(j, line_lag_ms=float(v["line_lag"]))
    return replace(model, **kw), j


# ---- evaluation, in parallel ----------------------------------------------------------------------------
_SEGS: list[Segment] = []


def _init(segs):
    global _SEGS
    _SEGS = segs


def _one(args):
    i, model, judge = args
    seg = _SEGS[i]
    return replay(seg, model, judge)


class Evaluator:
    """Replays a list of segments for many models, in a pool of ``procs`` processes."""

    def __init__(self, segs: list[Segment], procs: int = 0):
        self.segs = segs
        self.procs = procs or max(1, min(8, (os.cpu_count() or 2) // 2))
        self.pool = None
        if self.procs > 1:
            import multiprocessing as mp
            self.pool = mp.get_context("fork").Pool(self.procs, _init, (segs,))
        else:
            _init(segs)
        # longest first: the pool finishes together
        self.order = sorted(range(len(segs)), key=lambda i: -segs[i].n)

    def predict(self, model: BodyModel, judge: Judge) -> list[np.ndarray]:
        jobs = [(i, model, judge) for i in self.order]
        res = self.pool.map(_one, jobs, chunksize=1) if self.pool else [_one(j) for j in jobs]
        out: list = [None] * len(self.segs)
        for i, r in zip(self.order, res):
            out[i] = r
        return out

    def predict_many(self, cases: list[tuple[BodyModel, Judge]]) -> list[list[np.ndarray]]:
        """Several models at once (one pool pass): per case, per segment."""
        jobs = [(i, m, j) for m, j in cases for i in self.order]
        res = self.pool.map(_one, jobs, chunksize=1) if self.pool else [_one(j) for j in jobs]
        out = []
        n = len(self.order)
        for c in range(len(cases)):
            row: list = [None] * len(self.segs)
            for i, r in zip(self.order, res[c * n:(c + 1) * n]):
                row[i] = r
            out.append(row)
        return out

    def close(self) -> None:
        if self.pool is not None:
            self.pool.close()
            self.pool.join()
            self.pool = None


def weighted_residuals(segs: list[Segment], preds: list[np.ndarray]) -> np.ndarray:
    """All segments' residuals as one vector, each segment weighted to count as at most SEGMENT_CAP ticks."""
    parts = []
    for s, p in zip(segs, preds):
        r = residuals(s, p)
        if len(r):
            parts.append(r.ravel() * math.sqrt(min(1.0, SEGMENT_CAP / len(r))))
    return np.concatenate(parts) if parts else np.zeros(0)


# ---- the fit --------------------------------------------------------------------------------------------
# Where a number may go (outside: not physical, or the judge would not run)
BOUNDS = {"inset": (0.0, 20.0), "sag": (0.0, 10.0), "lag": (20.0, 200.0), "tau": (10.0, 300.0),
          "line_lag": (0.0, 200.0), **{f"give_{leg}": (0.2, 5.0) for leg in LEG_IDX}}


@dataclass
class FitOut:
    names: list
    start: dict
    values: dict
    cost0: float
    cost: float
    log: list                # (round, name, [(value, cost)...]) every point tried


def _cost(ev: Evaluator, model: BodyModel, judge: Judge, cases: list[dict]) -> list[float]:
    preds = ev.predict_many([with_values(model, judge, v) for v in cases])
    out = []
    for p in preds:
        r = weighted_residuals(ev.segs, p)
        out.append(float(r @ r))
    return out


def fit(ev: Evaluator, model: BodyModel, judge: Judge, names: list[str], rounds: int = 2, say=None) -> FitOut:
    """The PARAMS ``names`` that predict ``ev``'s segments best: one number at a time (the others held),
    its step (PARAMS) times -2, -1, +1, +2 tried and a parabola through the best and its neighbours;
    the steps halve each round. Every point is a full replay (minutes in all). A search, not a gradient:
    a tip a tick earlier or later changes a whole stretch, so the cost is too rough for derivatives."""
    v0 = values_of(model, judge)
    cur = {k: v0[k] for k in names}
    best = _cost(ev, model, judge, [cur])[0]
    cost0 = best
    log = []
    for rnd in range(rounds):
        for k in names:
            st = PARAMS[k][0] * 2.0 / (2 ** rnd)
            lo, hi = BOUNDS.get(k, (-math.inf, math.inf))
            x = cur[k]
            xs = sorted({min(hi, max(lo, x + d * st)) for d in (-2, -1, 1, 2)} - {x})
            pts = [(x, best)] + list(zip(xs, _cost(ev, model, judge, [dict(cur, **{k: v}) for v in xs])))
            pts.sort()
            bi = min(range(len(pts)), key=lambda i: pts[i][1])
            if 0 < bi < len(pts) - 1:
                (x1, c1), (x2, c2), (x3, c3) = pts[bi - 1:bi + 2]
                a, b, _ = np.polyfit([x1, x2, x3], [c1, c2, c3], 2)
                if a > 0 and x1 < -b / (2 * a) < x3:
                    xv = float(-b / (2 * a))
                    pts.append((xv, _cost(ev, model, judge, [dict(cur, **{k: xv})])[0]))
            xb, cb = min(pts, key=lambda p: p[1])
            if cb < best:
                cur[k], best = round(float(xb), 3), cb
            log.append((rnd, k, sorted(pts)))
            if say:
                say(f"  round {rnd + 1}: {k} -> {cur[k]:.2f}  (cost {best:.0f}, start {cost0:.0f})")
    return FitOut(list(names), {k: v0[k] for k in names}, dict(cur), cost0, best, log)


def by_boot(segs: list[Segment], preds: list[np.ndarray]) -> dict[str, float]:
    """The cost (weighted_residuals squared) of each boot's segments: for each boot's own best value
    of a number (``kilo model score --fit``'s spread)."""
    out: dict[str, list] = defaultdict(list)
    for s, p in zip(segs, preds):
        out[f"b{s.boot}"].append((s, p))
    res = {}
    for b, sp in out.items():
        r = weighted_residuals([x[0] for x in sp], [x[1] for x in sp])
        res[b] = float(r @ r)
    return res


def spread(ev: Evaluator, model: BodyModel, judge: Judge, name: str, values: list[float]) -> dict:
    """Each boot's own best value of ``name`` around the model's (a parabola through the costs at
    ``values``): {boot: best}, and the median and the +- (the boots' spread over the root of their number)."""
    preds = ev.predict_many([with_values(model, judge, {name: v}) for v in values])
    costs = [by_boot(ev.segs, p) for p in preds]
    best = {}
    for b in costs[0]:
        a, bb, _ = np.polyfit(values, [c[b] for c in costs], 2)
        if a > 0 and min(values) - 2 * (max(values) - min(values)) < -bb / (2 * a) < max(values) + 2 * (
                max(values) - min(values)):
            best[b] = float(-bb / (2 * a))
    vs = sorted(best.values())
    med = float(np.median(vs)) if vs else math.nan
    se = float(np.std(vs) / math.sqrt(len(vs))) if len(vs) > 1 else math.nan
    return {"boots": best, "median": med, "pm": se}


# ---- reports --------------------------------------------------------------------------------------------
def table(segs: list[Segment], preds: dict[str, list[np.ndarray]]) -> str:
    """Per segment and per set: rms pitch / roll (deg) for each named prediction; "plan" is the FEET's own."""
    names = list(preds)
    head = "| segment | what | ticks | " + " | ".join(f"{n}: pitch / roll" for n in names) + " |"
    L = [head, "|---|---|---|" + "---|" * len(names)]
    sets: dict[str, dict[str, list]] = {"fit set": defaultdict(list), "held out": defaultdict(list)}
    for i, s in enumerate(segs):
        cells = []
        n = 0
        for nm in names:
            sc = score(s, preds[nm][i])
            n = sc["n"]
            if not n:
                cells.append("-")
                continue
            sets["held out" if s.held_out else "fit set"][nm].append(sc)
            cells.append(f"{sc['rms_pitch']:.2f} / {sc['rms_roll']:.2f}")
        if n:
            L.append(f"| {s.name}{' (held out)' if s.held_out else ''} | {s.what} | {n} | " + " | ".join(cells) + " |")
    for k, by in sets.items():
        cells = []
        n = 0
        for nm in names:
            t = total(by[nm])
            n = t["n"]
            cells.append(f"**{t['rms_pitch']:.2f} / {t['rms_roll']:.2f}**" if n else "-")
        L.append(f"| **all of the {k}** | | {n} | " + " | ".join(cells) + " |")
    return "\n".join(L)


def plan_preds(segs: list[Segment], model: BodyModel) -> list[np.ndarray]:
    """The FEET's own prediction (the planner's rigid model, no lag), plus the model's IMU offset."""
    return [s.plan + np.array([model.imu_pitch_deg, model.imu_roll_deg]) for s in segs]
