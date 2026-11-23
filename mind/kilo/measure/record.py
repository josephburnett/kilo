"""A self-measurement run from a recording, as kilo.measure.fit reads it.

The C++ routine (kernel/mind/measure/measure.h) records its run in the
flight recorder (kernel/mind/record/recorder.h): a run of kind TRICK whose
label is "measure=self end=REASON", whose first MARK is "measure id=..
script=N tick0=.. ...", then "model ...", a "step i=.. exp=.. phase=.. t=.."
at each step's first tick, for each sweep (script 2) "base ..." with the
settle hold's and the lifted hold's tilt, "tip ..." at each tip it saw and
"end reason=..". ``script`` says which of kilo.measure.script.SCRIPTS the
run played (its step and experiment indices are that script's). The
POSE keys it sent (FEET in protocol 2; the feet and the rigid model's attitude for each body tick)
and the TELEMETRY (the measured pitch, roll, rates and battery) are in the
stream around it. Script tick t is body tick tick0 + t.

Also here: the synthetic runs' file (kilo.measure.simulate, ``kilo measure
simulate --write``: the same arrays as JSON), and a walk's error stream
for the fit's ``--walk`` (kilo.logs.walkrec.tick_table).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from kilo.logs import flight, proto2, walkrec
from kilo.measure import fit as F
from kilo.measure import script as SC
from kilo.protocol import kilo_protocol as P

A16 = 16.0
RUN_FORMAT = "kilo-measure-run"


class RecordError(Exception):
    pass


def measure_runs(s: flight.Stream) -> list[flight.RunSpan]:
    """The stream's self-measurement runs (a "measure" MARK in the run)."""
    out = []
    for r in s.runs:
        if r.begin_us is None:
            continue
        end = r.end_us if r.end_us is not None else (s.t_last_us or r.begin_us)
        if any(m.word == "measure" and r.begin_us <= m.t_us <= end for m in s.marks):
            out.append(r)
    return out


def run_of(s: flight.Stream, span: flight.RunSpan, name: str = "") -> F.Run:
    """One recorded run as a fit.Run: every script tick from 0 to the last POSE."""
    lo, hi = flight.span_of(s, span)
    end = span.end_us if span.end_us is not None else hi
    marks = [m for m in s.marks if span.begin_us <= m.t_us <= end]
    head = next((m for m in marks if m.word == "measure"), None)
    if head is None or "tick0" not in head.fields:
        raise RecordError(f"run {span.id}: no measure MARK with tick0")
    tick0 = int(head.fields["tick0"])
    frames = [f for f in s.frames if lo <= f.t_us <= hi and f.msg is not None]

    def t_of(tick: int) -> int:
        d = (tick - tick0) & 0xFFFF
        return d if d < 0x8000 else d - 0x10000

    feet: dict[int, Any] = {}
    tel: dict[int, Any] = {}
    for f in frames:
        if f.direction == "tx" and proto2.is_stream(f.msg):  # POSE (FEET in protocol 2 recordings)
            feet[t_of(f.msg.tick)] = f.msg
        elif f.direction == "rx" and isinstance(f.msg, P.Telemetry) and f.msg.mode == P.Mode.STREAM:
            tel[t_of(f.msg.tick)] = f.msg
    ts = [t for t in feet if t >= 0]
    if not ts:
        raise RecordError(f"run {span.id}: no POSE (or FEET)")
    n = max(ts) + 1
    x = np.zeros((n, 4))
    z = np.full((n, 4), float(SC.HOME_Z))
    pred = np.full((n, 2), np.nan)
    meas = np.full((n, 2), np.nan)
    rate = np.full((n, 2), np.nan)
    batt = np.full(n, np.nan)
    last = None
    for t in range(n):
        m = feet.get(t)
        if m is not None:
            last = m
            pred[t] = (m.pitch / A16, m.roll / A16)
        if last is not None:  # a tick not sent (late): the body held the pose before
            x[t], z[t] = proto2.feet_of(last)
        tm = tel.get(t)
        if tm is not None:
            meas[t] = (tm.pitch / A16, tm.roll / A16)
            rate[t] = (tm.rate_pitch / A16, tm.rate_roll / A16)
            batt[t] = tm.batt_mv
    # the steps: from each "step" MARK on
    step = np.full(n, -1)
    for m in marks:
        if m.word == "step" and "t" in m.fields and "i" in m.fields:
            t = int(m.fields["t"])
            if 0 <= t < n:
                step[t:] = int(m.fields["i"])
    if (step < 0).any():
        first = int(np.argmax(step >= 0)) if (step >= 0).any() else n
        step[:first] = 0
    script = script_of(head.fields, span.id)
    steps = script.steps
    exp = np.array([steps[i].exp if 0 <= i < len(steps) else -1 for i in step])
    phase = [steps[i].phase if 0 <= i < len(steps) else "?" for i in step]
    meta = {"run": span.id, "header": head.fields, "path": s.meta.get("path")}
    for w in ("model", "end"):
        mk = next((m for m in marks if m.word == w), None)
        if mk is not None:
            meta[w] = mk.fields
    meta["tips"] = [m.fields for m in marks if m.word == "tip"]
    meta["bases"] = [m.fields for m in marks if m.word == "base"]  # script 2: each sweep's two baselines
    meta["keys"] = [m.fields.get("text") for m in marks if m.word == "key"]
    return F.Run(name=name or f"run {span.id}", x=x, z=z, exp=exp, phase=phase, meas=meas, pred=pred,
                 batt_mv=batt, rate=rate, meta=meta, script=script, step=step)


def script_of(fields: dict, run_id: Any = "?") -> SC.ScriptDef:
    """The script a run played, from its header's ``script=`` (1 if it has none)."""
    try:
        v = int(fields.get("script", 1))
    except (TypeError, ValueError):
        v = -1
    if v not in SC.SCRIPTS:
        raise RecordError(f"run {run_id}: script {fields.get('script')!r} is not one this kilo knows "
                          f"({', '.join(map(str, sorted(SC.SCRIPTS)))})")
    return SC.SCRIPTS[v]


def walks_of(s: flight.Stream, name: str = "") -> list[F.Walk]:
    """The stream's walks as error streams (fit.Walk): u = the POSE
    prediction plus the body's correction, ticks in STREAM with three or four
    feet in the plan's support."""
    out = []
    for r in s.runs:
        if r.kind != "walk":
            continue
        try:
            rows = walkrec.tick_table(flight.walk_of(s, r))
        except (ValueError, KeyError):
            continue
        if not rows:
            continue
        k0 = rows[0]["tick"]
        n = rows[-1]["tick"] - k0 + 1
        u = np.full((n, 2), np.nan)
        meas = np.full((n, 2), np.nan)
        for row in rows:
            i = row["tick"] - k0
            if row.get("pred_pitch") is not None:
                cp = row.get("corr_pitch") or 0.0
                cr = row.get("corr_roll") or 0.0
                u[i] = (row["pred_pitch"] + cp, row["pred_roll"] + cr)
            # a tick the servos alone answer: three or four feet down, and the body's balance not
            # correcting from what it measures just then (off, or holding its correction: a foot is up)
            if row.get("meas_pitch") is not None and row.get("mode") == "STREAM":
                sup = row.get("support_plan")
                open_loop = not row.get("balance") or row.get("held")
                if sup is not None and bin(int(sup)).count("1") >= 3 and open_loop:
                    meas[i] = (row["meas_pitch"], row["meas_roll"])
        # the prediction held over a missing tick (the body played the one before)
        for i in range(1, n):
            if not np.isfinite(u[i]).all():
                u[i] = u[i - 1]
        if not np.isfinite(u[0]).all():
            ok = np.isfinite(u).all(1)
            if not ok.any():
                continue
            u[: int(np.argmax(ok))] = u[int(np.argmax(ok))]
        used = np.isfinite(meas).all(1)
        if used.sum() < 100:
            continue
        out.append(F.Walk(name=f"{name or 'walk'}#{r.id}", u=u, meas=np.where(used[:, None], meas, 0.0), used=used))
    return out


# ---- files ----------------------------------------------------------------------------------------
def save_run(run: F.Run, path: str | Path) -> Path:
    """A run as JSON (the synthetic runs: kilo measure simulate --write)."""
    p = Path(path)
    d = {"format": RUN_FORMAT, "name": run.name, "x": run.x.tolist(), "z": run.z.tolist(), "exp": run.exp.tolist(),
         "phase": list(run.phase), "meas": np.where(np.isfinite(run.meas), run.meas, None).tolist(),
         "pred": np.where(np.isfinite(run.pred), run.pred, None).tolist(),
         "batt_mv": np.where(np.isfinite(run.batt_mv), run.batt_mv, None).tolist(), "meta": run.meta,
         "script": run.script.version, "step": None if run.step is None else np.asarray(run.step).tolist()}
    p.write_text(json.dumps(d, default=float) + "\n")
    return p


def _arr(v) -> np.ndarray:
    return np.array([[np.nan if c is None else c for c in r] if isinstance(r, list) else (np.nan if r is None else r)
                     for r in v], float)


def load(path: str | Path, heading: int = 1) -> list[F.Run]:
    """Every self-measurement run in a file: a .krec (kilo runs --fetch,
    kilo measure fetch, kilo record fetch) or a run's JSON (save_run)."""
    p = Path(path)
    if p.suffix == ".json":
        d = json.loads(p.read_text())
        if d.get("format") != RUN_FORMAT:
            raise RecordError(f"{p}: not a {RUN_FORMAT} file")
        script = script_of({"script": d.get("script", 1)}, p.name)
        return [F.Run(name=d.get("name", p.stem), x=_arr(d["x"]), z=_arr(d["z"]), exp=np.array(d["exp"]),
                      phase=list(d["phase"]), meas=_arr(d["meas"]), pred=_arr(d["pred"]), batt_mv=_arr(d["batt_mv"]),
                      heading=heading, meta=d.get("meta", {}), script=script,
                      step=None if d.get("step") is None else np.array(d["step"]))]
    try:
        s = flight.load(p)
    except (OSError, ValueError) as e:
        raise RecordError(f"{p}: {e}") from e
    spans = measure_runs(s)
    if not spans:
        raise RecordError(f"{p}: no self-measurement run in it")
    runs = []
    for sp in spans:
        r = run_of(s, sp, name=f"{p.name}#{sp.id}")
        r.heading = heading
        runs.append(r)
    return runs


def load_walks(path: str | Path) -> list[F.Walk]:
    p = Path(path)
    try:
        s = flight.load(p)
    except (OSError, ValueError) as e:
        raise RecordError(f"{p}: {e}") from e
    return walks_of(s, p.name)


def summary(run: F.Run) -> str:
    """A few lines about a recorded run (kilo runs --fetch prints it)."""
    m = run.meta
    end = m.get("end", {})
    secs = len(run) * SC.TICK_MS / 1000.0
    got = int(np.isfinite(run.meas).all(1).sum())
    b = run.batt_mv[np.isfinite(run.batt_mv)]
    L = [f"  self-measurement: {len(run)} ticks ({secs:.1f} s), {got} with TELEMETRY in STREAM, "
         f"{len(m.get('tips', []))} tips; end {end.get('reason', '?')}: {end.get('why', '?')}"]
    if len(b):
        L.append(f"  battery {b.min():.0f}-{b.max():.0f} mV")
    L.append("  fit it: kilo measure fit " + str(m.get("path") or "FILE"))
    return "\n".join(L)
