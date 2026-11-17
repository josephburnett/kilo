"""A trick run from the flight recorder (kilo.logs.flight): what the trick
did against what its design predicted, and the metrics of the run.

``kilo runs --host --fetch N`` writes a ``.krec`` per run and prints this
analysis for a trick; ``kilo log trick FILE [--csv OUT] [--json] [--plot
OUT.png]`` prints it again (``kilo log run FILE`` picks walk or trick).

What the mind records for a trick (protocol/spec.toml [host] "The flight
recorder"): RUN begin/end around it, the MARK lines ``trick`` (which trick,
its id and CRC, the stage for rear_up, the battery and the body's mode at
the start, who started it), ``frames`` (each keyframe's move and hold in
ticks of 20 ms at 1x), ``play`` (the moment the body acknowledged
TRICK_PLAY: keyframe 0 starts there), ``key`` (each remote key that came
while it ran, with what the remote did), ``end`` (DONE, STOPPED,
PREEMPTED, FELL, FAILED or REFUSED, the time from the play, the battery,
why); and every TELEMETRY (pitch, roll, their rates, the keyframe the body
says it plays, the battery, the mode) and every other frame and log line,
from 2 s before to 2 s after.

The prediction: the trick's design exports the model's pitch at the end of
each keyframe's move (``source.predicted_pitch_deg`` in the trick's library
JSON, kilo.tricks.rear_up). Between keyframes it is interpolated with the
keyframe player's cosine easing over the move, and held over the hold;
before keyframe 0 it is the start posture's (STAND: 0). Pitch here is NOSE
UP + (the design's convention; TELEMETRY's own pitch is nose DOWN +, so
nose_up = -TELEMETRY.pitch). error = measured - predicted, nose up +.

Metrics (``analyze``): per keyframe the measured and predicted pitch at its
end, the mean and largest error over its move and hold; the largest pitch
and roll rates (the IMU's, deg/s) and when; the tip-over onset: the first
time the error stays beyond ``threshold_deg`` (default 8) for ``onset_s``
(default 0.2 s), or the roll beyond it, with the time, the keyframe and the
direction (back: more nose up than the design, forward: less, left/right:
rolled); the lowest battery; how it ended and, if a key stopped it, which.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from kilo.logs import flight
from kilo.protocol import kilo_protocol as P
from kilo.tricks.player import FOREVER, phase_ticks, play_order

TICK_US = 20000
A16 = 16.0
THRESHOLD_DEG = 8.0
ONSET_S = 0.2

CSV_FIELDS = (
    "t_s", "uptime_s", "since_play_s", "mode", "flags", "keyframe", "keyframe_name", "body_keyframe",
    "nose_up_deg", "pred_nose_up_deg", "err_deg", "pitch_deg", "roll_deg", "rate_pitch_dps", "rate_roll_dps",
    "batt_mv", "dist_mm",
)


@dataclass
class Keyframe:
    index: int
    name: str
    start_s: float        # from the play
    move_end_s: float
    end_s: float          # the hold's end
    pred_end: float | None  # nose up, deg
    dur: int
    hold: int


@dataclass
class TrickRun:
    run: flight.RunSpan
    stream: flight.Stream
    start: dict[str, str] = field(default_factory=dict)   # the `trick` MARK
    frames_mark: dict[str, str] = field(default_factory=dict)
    play_us: int | None = None
    end: dict[str, str] = field(default_factory=dict)
    end_us: int | None = None
    keys: list[tuple[int, str]] = field(default_factory=list)  # (t_us, text)
    telemetry: list[tuple[int, Any]] = field(default_factory=list)
    lo_us: int = 0        # the span's start (CSV times count from it)

    @property
    def name(self) -> str:
        return self.start.get("trick", "?")

    @property
    def stage(self) -> str | None:
        return self.start.get("stage")


def trick_run(s: flight.Stream, run: flight.RunSpan | None = None) -> TrickRun:
    if run is None:
        run = next((r for r in s.runs if r.kind == "trick"), None)
        if run is None:
            raise ValueError("no trick in this recording")
    lo, hi = flight.span_of(s, run)
    part = s.between(lo, hi)
    begin = run.begin_us if run.begin_us is not None else lo
    end = run.end_us if run.end_us is not None else hi
    tr = TrickRun(run=run, stream=part, lo_us=lo)
    for m in part.marks:
        if not begin <= m.t_us <= end:
            continue
        if m.word == "trick":
            tr.start = m.fields
        elif m.word == "frames":
            tr.frames_mark = m.fields
        elif m.word == "play" and tr.play_us is None:
            tr.play_us = m.t_us
        elif m.word == "end":
            tr.end, tr.end_us = m.fields, m.t_us
        elif m.word == "key":
            tr.keys.append((m.t_us, m.fields.get("text", "")))
    if not tr.keys:  # an older mind: the remote's log lines
        tr.keys = [(g.t_us, g.text) for g in part.logs if begin <= g.t_us <= end and g.text.startswith("remote ")]
    tr.telemetry = part.telemetry()
    return tr


def _ints(v: str | None) -> list[int]:
    if not v:
        return []
    return [int(x) for x in v.split(",") if x.strip().lstrip("-").isdigit()]


def library_source(name: str) -> dict[str, Any]:
    try:
        from kilo.tricks import library

        return dict(library.load(name).source)
    except Exception:  # noqa: BLE001 - an unknown trick: no prediction
        return {}


def keyframes(tr: TrickRun, source: dict[str, Any] | None = None) -> list[Keyframe]:
    """The keyframes in play order with their times from the play (the body's
    keyframe player: a move of phase_ticks(dur) then a hold of
    phase_ticks(hold), at the run's speed)."""
    src = library_source(tr.name) if source is None else source
    durs, holds = _ints(tr.frames_mark.get("dur")), _ints(tr.frames_mark.get("hold"))
    if not durs:
        return []
    holds = (holds + [0] * len(durs))[:len(durs)]
    loop = _ints(tr.frames_mark.get("loop")) or [0, 0, 0]
    ls, le, rep = (loop + [0, 0, 0])[:3]
    speed = int(tr.start.get("speed", "64") or 64)
    names = list(src.get("keyframes") or [])
    pred = list(src.get("predicted_pitch_deg") or [])
    order = play_order(len(durs), ls, le, rep if rep != FOREVER else 1) if le < len(durs) else list(range(len(durs)))
    out, t = [], 0
    for k in order:
        mv, hd = phase_ticks(durs[k], speed), phase_ticks(holds[k], speed)
        out.append(Keyframe(k, names[k] if k < len(names) else f"frame {k}", t * TICK_US / 1e6,
                            (t + mv) * TICK_US / 1e6, (t + mv + hd) * TICK_US / 1e6,
                            pred[k] if k < len(pred) else None, durs[k], holds[k]))
        t += mv + hd
    return out


def predicted_at(kfs: list[Keyframe], since_play_s: float, start: float = 0.0) -> tuple[float | None, int | None]:
    """(predicted nose up, the keyframe index in play order) at this time
    from the play; (start, None) before it; the last keyframe's after."""
    if not kfs or kfs[0].pred_end is None:
        return None, None
    if since_play_s < 0:
        return start, None
    prev = start
    for i, k in enumerate(kfs):
        if since_play_s < k.end_s:
            if k.pred_end is None:
                return None, i
            if since_play_s < k.move_end_s and k.move_end_s > k.start_s:
                s = (since_play_s - k.start_s) / (k.move_end_s - k.start_s)
                return prev + (k.pred_end - prev) * (1 - math.cos(math.pi * s)) / 2, i
            return k.pred_end, i
        prev = k.pred_end if k.pred_end is not None else prev
    return kfs[-1].pred_end, len(kfs) - 1


def rows(tr: TrickRun, source: dict[str, Any] | None = None) -> list[dict[str, Any]]:
    """One row per TELEMETRY in the run's span (CSV_FIELDS)."""
    kfs = keyframes(tr, source)
    out = []
    for t, m in tr.telemetry:
        since = None if tr.play_us is None else (t - tr.play_us) / 1e6
        pred, ki = predicted_at(kfs, since) if since is not None else (None, None)
        if tr.end_us is not None and t > tr.end_us:  # after the end: the end posture, no prediction
            pred, ki = None, None
        nose = -m.pitch / A16
        r = {
            "t_s": round((t - tr.lo_us) / 1e6, 3), "uptime_s": round(tr.stream.uptime_s(t), 3),
            "since_play_s": None if since is None else round(since, 3),
            "mode": P.Mode(m.mode).name if m.mode in P.Mode._value2member_map_ else str(m.mode),
            "flags": m.flags, "keyframe": None if ki is None else kfs[ki].index,
            "keyframe_name": None if ki is None else kfs[ki].name,
            "body_keyframe": None if m.trick_frame == 0xFF else m.trick_frame,
            "nose_up_deg": round(nose, 3), "pred_nose_up_deg": None if pred is None else round(pred, 3),
            "err_deg": None if pred is None else round(nose - pred, 3),
            "pitch_deg": m.pitch / A16, "roll_deg": m.roll / A16,
            "rate_pitch_dps": m.rate_pitch / A16, "rate_roll_dps": m.rate_roll / A16,
            "batt_mv": m.batt_mv, "dist_mm": m.dist_mm,
        }
        out.append(r)
    return out


def _onset(rs: list[dict[str, Any]], threshold: float, hold_s: float) -> dict[str, Any] | None:
    """The first time the pitch error or the roll stays beyond threshold for hold_s."""
    run_start: dict[str, tuple[float, dict[str, Any]] | None] = {"pitch": None, "roll": None}
    for r in rs:
        if r["since_play_s"] is None or r["err_deg"] is None:
            run_start = {"pitch": None, "roll": None}
            continue
        for axis, v in (("pitch", r["err_deg"]), ("roll", r["roll_deg"])):
            if abs(v) > threshold:
                if run_start[axis] is None:
                    run_start[axis] = (r["t_s"], r)
                t0, r0 = run_start[axis]
                if r["t_s"] - t0 >= hold_s - 1e-9:
                    if axis == "pitch":
                        way = "back (more nose up than the design)" if r0["err_deg"] > 0 else \
                              "forward (less nose up than the design)"
                    else:  # roll + = the right side up: it goes over to its left
                        way = "left" if r0["roll_deg"] > 0 else "right"
                    return {"t_s": r0["t_s"], "since_play_s": r0["since_play_s"], "axis": axis, "direction": way,
                            "keyframe": r0["keyframe"], "keyframe_name": r0["keyframe_name"],
                            "err_deg": r0["err_deg"], "roll_deg": r0["roll_deg"],
                            "nose_up_deg": r0["nose_up_deg"], "pred_nose_up_deg": r0["pred_nose_up_deg"]}
            else:
                run_start[axis] = None
    return None


def analyze(tr: TrickRun, source: dict[str, Any] | None = None, threshold_deg: float = THRESHOLD_DEG,
            onset_s: float = ONSET_S) -> dict[str, Any]:
    src = library_source(tr.name) if source is None else source
    kfs = keyframes(tr, src)
    rs = rows(tr, src)
    out: dict[str, Any] = {
        "path": tr.stream.meta.get("path"), "run": tr.run.id, "trick": tr.name, "stage": tr.stage,
        "start": tr.start, "end": tr.end, "keys": [{"t_s": round((t - tr.lo_us) / 1e6, 3), "text": k}
                                                     for t, k in tr.keys],
        "played": tr.play_us is not None, "has_prediction": bool(kfs and kfs[0].pred_end is not None),
        "telemetry": len(rs), "threshold_deg": threshold_deg, "onset_s": onset_s,
    }
    if tr.play_us is not None:
        out["play_t_s"] = round((tr.play_us - tr.lo_us) / 1e6, 3)
    if tr.end_us is not None:
        out["end_t_s"] = round((tr.end_us - tr.lo_us) / 1e6, 3)
    # stopped by: the last key before the end
    if tr.keys and tr.end.get("reason") in ("STOPPED", "PREEMPTED"):
        out["stopped_by"] = tr.keys[-1][1]
    in_run = [r for r in rs if r["since_play_s"] is not None and r["pred_nose_up_deg"] is not None]
    per = []
    for i, k in enumerate(kfs):
        sel = [r for r in in_run if r["keyframe"] == k.index and k.start_s <= r["since_play_s"] < k.end_s]
        errs = [r["err_deg"] for r in sel]
        last = sel[-1] if sel else None
        per.append({"index": k.index, "name": k.name, "start_s": round(k.start_s, 3), "end_s": round(k.end_s, 3),
                    "move_ticks": k.dur, "hold_ticks": k.hold, "pred_end_deg": k.pred_end,
                    "meas_end_deg": None if last is None else last["nose_up_deg"],
                    "samples": len(sel), "mean_err_deg": round(sum(errs) / len(errs), 2) if errs else None,
                    "max_err_deg": round(max(errs, key=abs), 2) if errs else None,
                    "reached": bool(sel)})
    out["keyframes"] = per
    errs = [r["err_deg"] for r in in_run]
    if errs:
        out["error"] = {"samples": len(errs), "mean_deg": round(sum(errs) / len(errs), 2),
                        "max_deg": round(max(errs, key=abs), 2),
                        "rms_deg": round(math.sqrt(sum(e * e for e in errs) / len(errs)), 2)}
    played = [r for r in rs if r["since_play_s"] is not None and (tr.end_us is None or r["since_play_s"] <= (
        tr.end_us - tr.play_us) / 1e6)]
    if played:
        mp = max(played, key=lambda r: abs(r["rate_pitch_dps"]))
        mr = max(played, key=lambda r: abs(r["rate_roll_dps"]))
        out["peak"] = {"nose_up_deg": max(r["nose_up_deg"] for r in played),
                       "roll_deg": round(max((r["roll_deg"] for r in played), key=abs), 2),
                       "rate_pitch_dps": mp["rate_pitch_dps"], "rate_pitch_t_s": mp["t_s"],
                       "rate_roll_dps": mr["rate_roll_dps"], "rate_roll_t_s": mr["t_s"]}
    out["tip_over"] = _onset(rs, threshold_deg, onset_s)
    batt = [r["batt_mv"] for r in rs if r["batt_mv"]]
    out["battery"] = {"min_mv": min(batt) if batt else None, "start_mv": tr.start.get("battery_mv"),
                      "end_mv": tr.end.get("battery_mv")}
    falls = [f for f in tr.stream.frames if f.direction == "rx" and isinstance(f.msg, P.Event)
             and f.msg.code == P.EventCode.FELL]
    out["fell_events"] = [round((f.t_us - tr.lo_us) / 1e6, 3) for f in falls]
    if tr.telemetry and rs and rs[0]["since_play_s"] is not None:
        pass
    pre = [r for r in rs if r["since_play_s"] is not None and r["since_play_s"] < 0]
    if pre and out["has_prediction"]:
        start = sum(r["nose_up_deg"] for r in pre) / len(pre)
        out["start_nose_up_deg"] = round(start, 2)
        if abs(start) > 5:
            out["start_note"] = (f"it stood at {start:+.1f} deg nose up before the play; the prediction assumes "
                                 "it starts from STAND (0)")
    return out


def format_analysis(a: dict[str, Any]) -> str:
    st, en = a.get("start", {}), a.get("end", {})
    stage = f" (stage {a['stage']})" if a.get("stage") else ""
    L = [f"{a.get('path') or 'trick'}: trick run {a['run']}: {a['trick']}{stage}, mind {st.get('mind_build', '?')}, "
         f"body {st.get('body_build', '?')}, by {st.get('by', '?')}"]
    L.append(f"  start   battery {st.get('battery_mv', '?')} mV, mode {st.get('mode', '?')}, speed "
             f"{st.get('speed', '?')}/64{', mirrored' if st.get('mirror') == '1' else ''}")
    why = f": {en['why']}" if en.get("why") else ""
    L.append(f"  end     {en.get('reason', '?')}{why}; {en.get('elapsed_ms', '?')} ms after the play (expected "
             f"{en.get('expected_ticks', '?')} ticks), battery {en.get('battery_mv', '?')} mV, mode {en.get('mode', '?')}")
    if a.get("stopped_by"):
        L.append(f"  stopped by  {a['stopped_by']}")
    for k in a.get("keys", []):
        L.append(f"  key     {k['t_s']:7.2f}s {k['text']}")
    if not a.get("played"):
        L.append("  not played (no TRICK_PLAY acknowledged)")
        return "\n".join(L)
    if a.get("start_note"):
        L.append(f"  note    {a['start_note']}")
    if not a.get("has_prediction"):
        L.append("  no prediction for this trick (its library entry has no predicted_pitch_deg)")
    L.append("  keyframes (nose up, deg; error = measured - predicted over the move and hold)")
    L.append("     #  name          start    end   predicted measured   mean err  max err")
    for k in a.get("keyframes", []):
        def f(v: Any, fmt: str = "+7.1f") -> str:
            return "      -" if v is None else format(v, fmt)
        L.append(f"    {k['index']:2d}  {k['name'][:12]:12s} {k['start_s']:6.2f} {k['end_s']:6.2f}   {f(k['pred_end_deg'])}"
                 f"  {f(k['meas_end_deg'])}   {f(k['mean_err_deg'], '+7.2f')}  {f(k['max_err_deg'], '+7.2f')}"
                 + ("" if k["reached"] else "  (not reached)"))
    e = a.get("error")
    if e:
        L.append(f"  error   {e['samples']} ticks: mean {e['mean_deg']:+.2f} rms {e['rms_deg']:.2f} largest "
                 f"{e['max_deg']:+.2f} deg")
    pk = a.get("peak")
    if pk:
        L.append(f"  peak    nose up {pk['nose_up_deg']:+.1f} deg, roll {pk['roll_deg']:+.1f} deg; pitch rate "
                 f"{pk['rate_pitch_dps']:+.0f} deg/s at {pk['rate_pitch_t_s']:.2f}s, roll rate "
                 f"{pk['rate_roll_dps']:+.0f} deg/s at {pk['rate_roll_t_s']:.2f}s")
    tip = a.get("tip_over")
    if tip:
        L.append(f"  TIP-OVER onset at {tip['t_s']:.2f}s ({tip['since_play_s']:.2f} s into the play), keyframe "
                 f"{tip['keyframe']} {tip['keyframe_name']}: {tip['axis']} {tip['direction']}; error "
                 f"{tip['err_deg']:+.1f} deg, roll {tip['roll_deg']:+.1f} (beyond {a['threshold_deg']:g} deg for "
                 f"{a['onset_s']:g} s)")
    else:
        L.append(f"  tip-over  none (error and roll within {a['threshold_deg']:g} deg)")
    if a.get("fell_events"):
        L.append("  FELL    body EVENT at " + ", ".join(f"{t:.2f}s" for t in a["fell_events"]))
    b = a.get("battery", {})
    L.append(f"  battery lowest {b.get('min_mv')} mV")
    return "\n".join(L)


def write_csv(rs: list[dict[str, Any]], path: str | Path) -> Path:
    path = Path(path)
    with open(path, "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(CSV_FIELDS)
        for r in rs:
            wr.writerow(["" if r[k] is None else r[k] for k in CSV_FIELDS])
    return path


def plot(rs: list[dict[str, Any]], out: str | Path, title: str = "") -> tuple[Path, str]:
    out = Path(out)
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        p = write_csv(rs, out.with_suffix(".csv"))
        return p, f"wrote CSV {p} (matplotlib is not installed)"
    t = [r["t_s"] for r in rs]

    def col(k: str) -> list[float]:
        return [math.nan if r[k] is None else float(r[k]) for r in rs]
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
    ax[0].plot(t, col("pred_nose_up_deg"), lw=1, label="predicted nose up")
    ax[0].plot(t, col("nose_up_deg"), lw=1, label="measured nose up")
    ax[0].set_ylabel("°")
    ax[1].plot(t, col("err_deg"), lw=1, label="error (nose up)")
    ax[1].plot(t, col("roll_deg"), lw=1, label="roll")
    ax[1].set_ylabel("°")
    ax[2].plot(t, col("rate_pitch_dps"), lw=1, label="pitch rate")
    ax[2].plot(t, col("rate_roll_dps"), lw=1, label="roll rate")
    ax[2].set_ylabel("°/s")
    ax[2].set_xlabel("t (s) since the recording's start")
    for a_ in ax:
        a_.legend(fontsize=7, loc="upper right")
        a_.grid(True, alpha=0.3)
    ax[0].set_title(title, fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out, f"wrote {out}"


def to_json(a: dict[str, Any]) -> str:
    return json.dumps(a, indent=1)
