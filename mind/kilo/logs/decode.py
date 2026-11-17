"""``kilo record decode FILE``: a flight recording (``.krec`` from ``kilo
record fetch`` or ``kilo runs --fetch``, or raw blocks off the SD card's
kilorec/ folder) as readable rows.

    kilo record decode FILE                 the runs, the log lines, the counts
    kilo record decode FILE --csv OUT.csv   one row per TELEMETRY (the columns below)
    kilo record decode FILE --json          one JSON object per line: every TELEMETRY row, and every
                                            log line, key press, body event and command as its own row
    kilo record decode FILE --log           the mind's log lines only
    kilo record decode FILE --plot OUT.png  pitch, roll, battery and the runs over time (CSV without matplotlib)
    --from S --to S                         only seconds S..S of the file (from its start)

The CSV columns (one row per TELEMETRY the body sent, 25 a second, 50
while it walks):

    t_s             seconds since the file's first record
    uptime_s        the mind's clock in seconds (about its uptime)
    mode            the body's mode (IDLE, GAIT, TRICK, FALLEN, LIMP, ...)
    flags           TELEMETRY flags by name, "+" between (SERVOS_POWERED+TRICK+...)
    pitch_deg       pitch, nose DOWN + (as the body reports it)
    nose_up_deg     the same, nose UP + (the trick designs' convention)
    roll_deg        roll, right side up +
    rate_pitch_dps  the IMU's pitch rate, deg/s
    rate_roll_dps   the IMU's roll rate, deg/s
    batt_mv         the battery, mV (filtered by the body)
    dist_mm         the ultrasonic distance, mm (0: no echo)
    body_keyframe   the keyframe the body says it plays (empty: none)
    command         the last command the mind sent the body (FEET tick, SET_POSTURE name, TRICK_PLAY, TRICK_STOP ...)
    run             the run in progress ("walk 3", "trick 4"), empty between runs
    trick           the trick's name during a trick run
    keyframe        the keyframe the trick should be at by the clock (from the play and the keyframes' timing)
    pred_nose_up_deg  the design's predicted pitch then, nose up + (empty: no prediction)
    err_deg         nose_up_deg - pred_nose_up_deg
    key             a remote key that arrived since the last row: "IR_KEY 0x0002" and what the remote did with it
    event           a body EVENT since the last row (FELL, TRICK_DONE, GAIT_STOPPED, WATCHDOG, ...)

--json rows have "type": "telemetry" (the fields above), "log" (t_s,
level, text), "key" (t_s, code, outcome), "event" (t_s, name, arg),
"command" (t_s, name, fields), "mark" (t_s, word, fields), "run" (t_s, id,
kind, event).
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, Iterator

from kilo.logs import flight, trickrec
from kilo.protocol import kilo_protocol as P

CSV_FIELDS = ("t_s", "uptime_s", "mode", "flags", "pitch_deg", "nose_up_deg", "roll_deg", "rate_pitch_dps",
              "rate_roll_dps", "batt_mv", "dist_mm", "body_keyframe", "command", "run", "trick", "keyframe",
              "pred_nose_up_deg", "err_deg", "key", "event")
A16 = 16.0
KEY_OUTCOME_US = 500000  # the remote's line about a key comes within this of the key's EVENT


def _flags(v: int) -> str:
    return "+".join(f.name for f in P.TelemetryFlags if v & f) or "-"


def _mode(v: int) -> str:
    return P.Mode(v).name if v in P.Mode._value2member_map_ else str(v)


def _command(f: Any) -> str:
    m = f.msg
    if m is None:
        return f.name
    if isinstance(m, P.SetPosture):
        try:
            return f"SET_POSTURE {P.Posture(m.posture).name}"
        except ValueError:
            return f"SET_POSTURE {m.posture}"
    if isinstance(m, P.Feet):
        return f"FEET tick {m.tick}"
    if isinstance(m, P.TrickPlay):
        return f"TRICK_PLAY id {m.trick_id} speed {m.speed_q6}"
    return f.name


def keys(s: flight.Stream) -> list[dict[str, Any]]:
    """Every IR_KEY EVENT with what the remote did (its next "remote ..." log line)."""
    remote = [g for g in s.logs if g.text.startswith("remote ")]
    out = []
    for f in s.frames:
        if f.direction == "rx" and isinstance(f.msg, P.Event) and f.msg.code == P.EventCode.IR_KEY:
            line = next((g.text for g in remote if 0 <= g.t_us - f.t_us <= KEY_OUTCOME_US), "")
            out.append({"t_us": f.t_us, "code": f"0x{f.msg.arg & 0xFFFF:04x}", "outcome": line})
    return out


def rows(s: flight.Stream) -> Iterator[dict[str, Any]]:
    """The CSV rows, one per TELEMETRY, in time order."""
    t0 = s.t_first_us or 0
    spans = []
    for r in s.runs:
        lo = r.begin_us if r.begin_us is not None else t0
        hi = r.end_us if r.end_us is not None else (s.t_last_us or lo)
        tr = None
        if r.kind == "trick":
            try:
                tr = trickrec.trick_run(s, r)
            except ValueError:
                tr = None
        spans.append((lo, hi, r, tr, trickrec.keyframes(tr) if tr is not None else []))
    key_list = keys(s)
    ki = 0
    command = ""
    pending_events: list[str] = []
    for f in s.frames:
        if f.direction == "tx":
            command = _command(f)
            continue
        if isinstance(f.msg, P.Event) and f.msg.code != P.EventCode.IR_KEY:
            name = P.EventCode(f.msg.code).name if f.msg.code in P.EventCode._value2member_map_ else str(f.msg.code)
            pending_events.append(name)
            continue
        if not isinstance(f.msg, P.Telemetry):
            continue
        m, t = f.msg, f.t_us
        key_txt = []
        while ki < len(key_list) and key_list[ki]["t_us"] <= t:
            k = key_list[ki]
            key_txt.append(f"IR_KEY {k['code']}" + (f": {k['outcome']}" if k["outcome"] else ""))
            ki += 1
        run = trick = kf = pred = err = None
        for lo, hi, r, tr, kfs in spans:
            if lo <= t <= hi:
                run = r.label
                if tr is not None:
                    trick = tr.name
                    if tr.play_us is not None and kfs:
                        pred, idx = trickrec.predicted_at(kfs, (t - tr.play_us) / 1e6)
                        kf = None if idx is None else kfs[idx].index
                break
        nose = -m.pitch / A16
        if pred is not None:
            err = round(nose - pred, 3)
            pred = round(pred, 3)
        yield {"t_s": round((t - t0) / 1e6, 3), "uptime_s": round(s.uptime_s(t), 3), "mode": _mode(m.mode),
               "flags": _flags(m.flags), "pitch_deg": m.pitch / A16, "nose_up_deg": nose, "roll_deg": m.roll / A16,
               "rate_pitch_dps": m.rate_pitch / A16, "rate_roll_dps": m.rate_roll / A16, "batt_mv": m.batt_mv,
               "dist_mm": m.dist_mm, "body_keyframe": None if m.trick_frame == 0xFF else m.trick_frame,
               "command": command, "run": run, "trick": trick, "keyframe": kf, "pred_nose_up_deg": pred,
               "err_deg": err, "key": "; ".join(key_txt) or None, "event": "+".join(pending_events) or None}
        pending_events = []


def json_rows(s: flight.Stream) -> Iterator[dict[str, Any]]:
    t0 = s.t_first_us or 0
    out: list[tuple[int, dict[str, Any]]] = []
    for r in rows(s):
        out.append((int(round(r["t_s"] * 1e6)) + t0, dict(type="telemetry", **r)))
    for g in s.logs:
        out.append((g.t_us, {"type": "log", "t_s": round((g.t_us - t0) / 1e6, 3), "level": g.tag, "text": g.text}))
    for k in keys(s):
        out.append((k["t_us"], {"type": "key", "t_s": round((k["t_us"] - t0) / 1e6, 3), "code": k["code"],
                                "outcome": k["outcome"]}))
    for f in s.frames:
        t_s = round((f.t_us - t0) / 1e6, 3)
        if f.direction == "rx" and isinstance(f.msg, P.Event) and f.msg.code != P.EventCode.IR_KEY:
            name = P.EventCode(f.msg.code).name if f.msg.code in P.EventCode._value2member_map_ else str(f.msg.code)
            out.append((f.t_us, {"type": "event", "t_s": t_s, "name": name, "arg": f.msg.arg}))
        elif f.direction == "tx":
            fields = P.to_dict(f.msg) if f.msg is not None else {"payload": f.payload.hex()}
            out.append((f.t_us, {"type": "command", "t_s": t_s, "name": f.name, "fields": fields}))
    for m in s.marks:
        out.append((m.t_us, {"type": "mark", "t_s": round((m.t_us - t0) / 1e6, 3), "word": m.word,
                             "fields": m.fields}))
    for t, r in s.run_marks:
        out.append((t, {"type": "run", "t_s": round((t - t0) / 1e6, 3), "id": r.id,
                        "kind": "trick" if r.kind == P.RunKind.TRICK else "walk",
                        "event": "begin" if r.event == P.RunEvent.BEGIN else "end"}))
    out.sort(key=lambda x: x[0])
    for _, d in out:
        yield d


def write_csv(rs: list[dict[str, Any]], path: str | Path) -> Path:
    path = Path(path)
    with open(path, "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(CSV_FIELDS)
        for r in rs:
            wr.writerow(["" if r[k] is None else r[k] for k in CSV_FIELDS])
    return path


def summary(s: flight.Stream) -> str:
    t0, t1 = s.t_first_us, s.t_last_us
    dur = 0.0 if t0 is None or t1 is None else (t1 - t0) / 1e6
    L = [f"{s.meta.get('path', 'recording')}: {len(s.blocks)} blocks, {dur:.1f} s"
         + (f" (mind clock {s.uptime_s(t0):.1f} .. {s.uptime_s(t1):.1f} s)" if t0 is not None and t1 is not None
            else "")]
    tel = sum(1 for f in s.frames if f.direction == "rx" and isinstance(f.msg, P.Telemetry))
    L.append(f"  {tel} TELEMETRY, {len(s.frames) - tel} other frames, {s.skipped} heartbeats counted, "
             f"{len(s.logs)} log lines, {len(s.marks)} marks")
    if s.bad_blocks:
        L.append(f"  {len(s.bad_blocks)} bad blocks skipped: "
                 + ", ".join(f"#{b.seq} ({b.problem})" for b in s.bad_blocks[:8]))
    cut = [b for b in s.blocks if b.problem == "cut"]
    if cut:
        L.append(f"  the last block is cut short ({cut[-1].have} of {cut[-1].used} bytes): read as far as it goes")
    if s.bad_records:
        L.append(f"  {s.bad_records} records did not parse")
    for r in s.runs:
        b = "?" if r.begin_us is None else f"{(r.begin_us - (t0 or 0)) / 1e6:.2f}s"
        e = "running" if r.end_us is None else f"{(r.end_us - (t0 or 0)) / 1e6:.2f}s"
        L.append(f"  run {r.id:3d} {r.kind:5s} {b} .. {e}")
    ks = keys(s)
    for k in ks[:50]:
        L.append(f"  key {(k['t_us'] - (t0 or 0)) / 1e6:8.2f}s {k['code']} {k['outcome']}")
    return "\n".join(L)


def log_lines(s: flight.Stream) -> list[str]:
    t0 = s.t_first_us or 0
    return [f"{(g.t_us - t0) / 1e6:9.3f} {g.tag} {g.text}" for g in s.logs]


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
    ax[0].plot(t, col("nose_up_deg"), lw=1, label="nose up")
    ax[0].plot(t, col("pred_nose_up_deg"), lw=1, label="predicted (tricks)")
    ax[0].plot(t, col("roll_deg"), lw=1, label="roll")
    ax[0].set_ylabel("°")
    ax[1].plot(t, col("batt_mv"), lw=1, label="battery mV")
    ax[2].step(t, [1 if r["run"] else 0 for r in rs], lw=1, where="post", label="in a run")
    ax[2].set_xlabel("t (s) since the file's start")
    for a in ax:
        a.legend(fontsize=7, loc="upper right")
        a.grid(True, alpha=0.3)
    ax[0].set_title(title, fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out, f"wrote {out}"


def main(args: Any) -> int:
    import sys

    try:
        s = flight.load(args.file)
    except (OSError, ValueError) as e:
        print(f"kilo record decode: {e}", file=sys.stderr)
        return 1
    t0 = s.t_first_us or 0
    if args.t_from is not None or args.t_to is not None:
        s = s.between(None if args.t_from is None else t0 + int(args.t_from * 1e6),
                      None if args.t_to is None else t0 + int(args.t_to * 1e6))
    if args.json:
        for d in json_rows(s):
            print(json.dumps(d))
        return 0
    if args.log:
        for ln in log_lines(s):
            print(ln)
        return 0
    print(summary(s))
    if args.csv or args.plot:
        rs = list(rows(s))
        if args.csv:
            print(f"wrote {write_csv(rs, args.csv)} ({len(rs)} rows)", file=sys.stderr)
        if args.plot:
            print(plot(rs, args.plot, title=str(args.file))[1], file=sys.stderr)
    return 0
