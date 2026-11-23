"""Walk recordings from a C++ mind (kernel/mind/record/recorder.h): read,
convert and analyse.

``kilo walks --host IP --fetch`` downloads each recorded walk and writes two
files per walk:

- ``walk-<time>-<id>.kwalk``: what the mind sent, byte for byte, behind a
  small header: magic ``b"KILOWALK\\n"``, u16 LE meta_len, meta (UTF-8 JSON:
  the WALK_INFO fields, the mind's address and build, when it was fetched,
  ``start_wall``: the walk's start as wall time, estimated from the mind's
  uptime), then the records. This is the raw data: every link frame both
  ways with the mind's clock in microseconds, each planned tick, the
  planner's counters, and the MARK lines (protocol/spec.toml [host] "Walk
  recordings").
- ``walk-<time>-<id>.kilolog``: the same frames as a ``.kilolog``
  (kilo.logs.kilolog, version 1, ms stamps), so ``kilo log summary|replay|plot``
  read it.

``kilo log walk FILE.kwalk [--csv OUT] [--plot OUT.png] [--json]`` is the
analysis: per tick (``tick_table``) what the plan predicted and what the
body measured, the difference, the body's correction, the support and held
flags, the legs in the air, the stability margin and how early each POSE
arrived; and a summary (``summarize``): RMS and largest error per axis, the
constant offset (an IMU mounted off level, a mass misplaced), the servo lag
(as the error tracker estimates it, and from the attitude without lag), the
error by gait phase (which legs are in the air), the time the corrections
were held, the smallest margin, holds, catches and waits.

Angles are degrees: pitch nose down +, roll right side up + (TELEMETRY's
convention). error = measured - predicted, where predicted is what the POSE
frame for that tick carried (the attitude the planner expects a servo lag
after the feet move); the body's reference (``ref_*``) is that prediction
while its balance is on.
"""

from __future__ import annotations

import csv
import json
import math
import struct
import time
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from kilo.logs import proto2
from kilo.planner.model import LEG_NAMES
from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P

MAGIC = b"KILOWALK\n"
A16 = 16.0
LEGS = LEG_NAMES
_HDR = struct.Struct("<BBI")
assert _HDR.size == P.WalkRecordHeader.SIZE


# ---- the file ---------------------------------------------------------------------

# A recording file (.kwalk here, .krec in kilo.logs.flight): the magic, the meta's length (u16 LE), the
# meta as compact JSON, then the data.

def write_container(path: str | Path, magic: bytes, meta: dict[str, Any], data: bytes, what: str) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    m = json.dumps(meta, separators=(",", ":"), sort_keys=True).encode()
    if len(m) > 0xFFFF:
        raise ValueError(f"{what} meta too long")
    path.write_bytes(magic + struct.pack("<H", len(m)) + m + bytes(data))
    return path


def has_magic(path: str | Path, magic: bytes) -> bool:
    try:
        with open(path, "rb") as f:
            return f.read(len(magic)) == magic
    except OSError:
        return False


def split_container(data: bytes, magic: bytes) -> tuple[dict[str, Any], bytes] | None:
    """(meta, data) of a container's bytes, None if they do not start with magic and a length."""
    if not data.startswith(magic) or len(data) < len(magic) + 2:
        return None
    (n,) = struct.unpack_from("<H", data, len(magic))
    start = len(magic) + 2
    return json.loads(data[start:start + n].decode()), data[start + n:]


def write_file(path: str | Path, meta: dict[str, Any], records: bytes) -> Path:
    return write_container(path, MAGIC, meta, records, "kwalk")


def is_kwalk(path: str | Path) -> bool:
    return has_magic(path, MAGIC)


def read_file(path: str | Path) -> tuple[dict[str, Any], bytes]:
    got = split_container(Path(path).read_bytes(), MAGIC)
    if got is None:
        raise ValueError(f"{path}: not a walk recording (.kwalk)")
    return got


# ---- records ----------------------------------------------------------------------

@dataclass
class Frame:
    t_us: int
    direction: str  # "rx" (body -> mind) or "tx"
    type: int
    seq: int
    payload: bytes
    msg: Any = None  # decoded, or None (unknown type, bad length)

    @property
    def name(self) -> str:
        return proto2.type_name(self.type, self.payload)


@dataclass
class Mark:
    t_us: int
    word: str
    fields: dict[str, str]

    def num(self, key: str, default: float | None = None) -> float | None:
        v = self.fields.get(key)
        try:
            return float(v) if v is not None else default
        except ValueError:
            return default


@dataclass
class Walk:
    meta: dict[str, Any] = field(default_factory=dict)
    frames: list[Frame] = field(default_factory=list)
    plans: list[tuple[int, Any]] = field(default_factory=list)     # (t_us, P.WalkPlan)
    planner: list[tuple[int, Any]] = field(default_factory=list)   # (t_us, P.WalkPlanner)
    marks: list[Mark] = field(default_factory=list)
    bad: int = 0  # records that did not parse (a cut-off tail)

    def mark(self, word: str) -> Mark | None:
        return next((m for m in self.marks if m.word == word), None)

    def marks_of(self, word: str) -> list[Mark]:
        return [m for m in self.marks if m.word == word]

    @property
    def t0_us(self) -> int:
        m = self.mark("walk")
        if m is None:
            return 0
        return (int(m.fields.get("t0_us_hi", 0)) << 32) | int(m.fields.get("t0_us_lo", 0))


def parse_mark(text: str) -> tuple[str, dict[str, str]]:
    """'end ok=1 streamed=1 reason=the body fell' -> ('end', {...}); a
    ``reason`` takes the rest of the line (it may hold spaces)."""
    word, _, rest = text.partition(" ")
    out: dict[str, str] = {}
    while rest:
        rest = rest.lstrip(" ")
        key, eq, tail = rest.partition("=")
        if not eq:
            break
        if key == "reason":
            out[key] = tail
            break
        value, _, rest = tail.partition(" ")
        out[key] = value
    return word, out


def parse_records(data: bytes, meta: dict[str, Any] | None = None) -> Walk:
    """A walk's records: host.walk_format 1 (a walk's records alone, t_us
    from its t0), or 2 (flight-recorder blocks, kilo.logs.flight: the walk
    run in them, the meta's ``walk.id`` if it names one)."""
    from kilo.logs import flight

    if flight.is_blocks(data):
        s = flight.parse_blocks(data, meta)
        want = ((meta or {}).get("walk") or {}).get("id")
        run = next((r for r in s.runs if r.kind == "walk" and (want is None or r.id == (want & 0xFFFFF))), None)
        return flight.walk_of(s, run)
    w = Walk(meta=dict(meta or {}))
    i, n = 0, len(data)
    while i + _HDR.size <= n:
        kind, ln, t = _HDR.unpack_from(data, i)
        body = data[i + _HDR.size:i + _HDR.size + ln]
        if len(body) < ln:
            w.bad += 1
            break
        i += _HDR.size + ln
        try:
            if kind in (P.WalkRecord.RX, P.WalkRecord.TX):
                typ, seq, payload = body[0], body[1], bytes(body[2:])
                try:
                    msg = proto2.decode_payload(typ, payload)
                except (KeyError, ValueError, P.PayloadError, struct.error):
                    msg = None
                w.frames.append(Frame(t, "rx" if kind == P.WalkRecord.RX else "tx", typ, seq, payload, msg))
            elif kind == P.WalkRecord.PLAN:
                w.plans.append((t, P.WalkPlan.unpack(body)))
            elif kind == P.WalkRecord.PLANNER:
                w.planner.append((t, P.WalkPlanner.unpack(body)))
            elif kind == P.WalkRecord.MARK:
                word, fields = parse_mark(body.decode("ascii", "replace"))
                w.marks.append(Mark(t, word, fields))
            else:
                w.bad += 1
        except (IndexError, P.PayloadError):
            w.bad += 1
    return w


def load(path: str | Path) -> Walk:
    """A .kwalk (format 1 or 2) or a .krec (kilo.logs.flight) holding a walk."""
    from kilo.logs import flight

    if flight.is_krec(path):
        meta, data = flight.read_file(path)
    else:
        meta, data = read_file(path)
    w = parse_records(data, meta)
    w.meta.setdefault("path", str(path))
    return w


# ---- the .kilolog ----------------------------------------------------------------------

def to_kilolog(w: Walk, path: str | Path) -> Path:
    """Every frame as a .kilolog (version 1): rx/tx records with ms since t0.
    The meta carries the walk's MARK lines' first words and its WALK_INFO."""
    from kilo.logs.kilolog import KilologWriter

    meta = {"source": "kilo walks (a C++ mind's walk recording)", "walk": w.meta.get("walk"),
            "start_wall": w.meta.get("start_wall", time.time()), "marks": [m.word for m in w.marks]}
    wr = KilologWriter(path, meta=meta, clock=lambda: 0.0, queue_size=len(w.frames) + 16)
    try:
        for f in w.frames:
            wr.write_wire(C.build_frame(f.type, f.seq, f.payload), f.direction, f.t_us / 1e6)
    finally:
        wr.close()
    return Path(path)


def to_logdata(w: Walk, path: str | Path) -> Any:
    """A kilo.logs.reader.LogData of the walk (TELEMETRY samples and EVENTs),
    so reader.load() takes a .kwalk too."""
    from kilo.link import consts
    from kilo.logs.reader import LogData, LogEvent, sample_from_fields

    data = LogData(Path(path), "kwalk", meta=dict(w.meta))
    for f in w.frames:
        data.kinds[f.direction] += 1
        (data.rx_types if f.direction == "rx" else data.tx_types)[f.name] += 1
        if f.direction != "rx" or f.msg is None:
            continue
        t = f.t_us / 1e6
        if isinstance(f.msg, P.Telemetry):
            data.samples.append(sample_from_fields(t, f.msg))
        elif isinstance(f.msg, P.Event):
            data.events.append(LogEvent(t, consts.EVENTS.get(f.msg.code, str(f.msg.code)), f.msg.arg,
                                        f.msg.tick, f.msg.epoch))
    return data


# ---- per tick ----------------------------------------------------------------------

def _unwrap(raw: int, ref: int) -> int:
    """The tick nearest ref whose low 16 bits are raw."""
    return ref + ((raw - ref + 0x8000) & 0xFFFF) - 0x8000


def _legs(bits: int) -> str:
    names = [LEGS[i] for i in range(4) if bits & (1 << i)]
    return "+".join(names) if names else "none"


TABLE_FIELDS = (
    "tick", "t_ms", "mode", "lifting", "support_plan", "support_body", "held", "balance",
    "pred_pitch", "pred_roll", "meas_pitch", "meas_roll", "err_pitch", "err_roll",
    "plan_pitch", "plan_roll", "ref_pitch", "ref_roll", "corr_pitch", "corr_roll",
    "margin_mm", "pose_sent_ms", "telem_ms", "lead_ms", "pose_queued", "rate_pitch", "rate_roll",
)


def tick_table(w: Walk) -> list[dict[str, Any]]:
    """One row per body tick from the first planned tick to the last
    TELEMETRY: the plan (PLAN, POSE; FEET in protocol 2) joined with what the body reported for
    that tick (TELEMETRY.tick). Missing parts are None (a tick planned but
    skipped, a TELEMETRY dropped)."""
    if not w.plans:
        return []
    ref = w.plans[0][1].tick
    plan: dict[int, tuple[int, Any]] = {}
    for t, p in w.plans:
        plan[p.tick] = (t, p)
    feet: dict[int, tuple[int, Any]] = {}
    tel: dict[int, tuple[int, Any]] = {}
    last = ref
    for f in w.frames:
        if f.msg is None:
            continue
        if f.direction == "tx" and proto2.is_stream(f.msg):
            k = _unwrap(f.msg.tick, last)
            feet.setdefault(k, (f.t_us, f.msg))
        elif f.direction == "rx" and isinstance(f.msg, P.Telemetry):
            k = _unwrap(f.msg.tick, last)
            last = k
            tel[k] = (f.t_us, f.msg)
    first = min(plan)
    end = max([k for k in tel if k >= first] + [max(plan)])
    rows = []
    for k in range(first, end + 1):
        pt = plan.get(k)
        ft = feet.get(k)
        tt = tel.get(k)
        if pt is None and tt is None:
            continue
        p = pt[1] if pt else None
        fm = ft[1] if ft else None
        tm = tt[1] if tt else None
        r: dict[str, Any] = {k2: None for k2 in TABLE_FIELDS}
        r["tick"] = k
        if p is not None:
            r["lifting"] = _legs(p.lifting)
            r["support_plan"] = p.support
            r["plan_pitch"], r["plan_roll"] = p.plan_pitch / A16, p.plan_roll / A16
            r["margin_mm"] = None if p.margin == -32768 else p.margin / 10.0
            r["pred_pitch"], r["pred_roll"] = p.pitch / A16, p.roll / A16
        if fm is not None:  # what the body was sent (clamped to the field's range)
            r["pred_pitch"], r["pred_roll"] = fm.pitch / A16, fm.roll / A16
            r["support_plan"] = fm.support
            r["pose_sent_ms"] = round(ft[0] / 1000, 3)
        if tm is not None:
            flags = P.TelemetryFlags(tm.flags)
            r["t_ms"] = r["telem_ms"] = round(tt[0] / 1000, 3)
            r["mode"] = proto2.mode_name(tm.mode)
            r["meas_pitch"], r["meas_roll"] = tm.pitch / A16, tm.roll / A16
            r["ref_pitch"], r["ref_roll"] = tm.ref_pitch / A16, tm.ref_roll / A16
            r["corr_pitch"], r["corr_roll"] = tm.corr_pitch / A16, tm.corr_roll / A16
            r["rate_pitch"], r["rate_roll"] = tm.rate_pitch / A16, tm.rate_roll / A16
            r["support_body"] = tm.support
            r["held"] = int(bool(flags & P.TelemetryFlags.BALANCE_HELD))
            r["balance"] = int(bool(flags & P.TelemetryFlags.BALANCE_ACTIVE))
            r["pose_queued"] = tm.pose_queued
            if r["pred_pitch"] is not None:
                r["err_pitch"] = r["meas_pitch"] - r["pred_pitch"]
                r["err_roll"] = r["meas_roll"] - r["pred_roll"]
            if r["pose_sent_ms"] is not None:
                r["lead_ms"] = round(r["telem_ms"] - r["pose_sent_ms"], 3)
        elif r["pose_sent_ms"] is not None:
            r["t_ms"] = r["pose_sent_ms"]
        rows.append(r)
    return rows


def write_csv(rows: list[dict[str, Any]], path: str | Path) -> Path:
    path = Path(path)
    with open(path, "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(TABLE_FIELDS)
        for r in rows:
            wr.writerow(["" if r[k] is None else (f"{r[k]:.4g}" if isinstance(r[k], float) else r[k])
                         for k in TABLE_FIELDS])
    return path


# ---- the summary ----------------------------------------------------------------------

def _rms(xs: list[float]) -> float | None:
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else None


def _r(v: float | None, n: int = 2) -> float | None:
    return None if v is None else round(v, n)


def _xcorr_lag(src: dict[int, float], dst: dict[int, float], max_k: int = 25) -> tuple[int | None, float | None]:
    """The shift k (ticks, >= 0) that best lines src[t - k] up with dst[t]
    (Pearson); None when src hardly moves."""
    vals = list(src.values())
    if len(vals) < 20:
        return None, None
    m = sum(vals) / len(vals)
    if math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals)) < 0.3:
        return None, None
    best = (None, -2.0)
    for k in range(0, max_k + 1):
        pairs = [(v, dst[t + k]) for t, v in src.items() if t + k in dst]
        if len(pairs) < 20:
            continue
        ma = sum(a for a, _ in pairs) / len(pairs)
        mb = sum(b for _, b in pairs) / len(pairs)
        cov = sum((a - ma) * (b - mb) for a, b in pairs)
        va = sum((a - ma) ** 2 for a, _ in pairs)
        vb = sum((b - mb) ** 2 for _, b in pairs)
        if va <= 0 or vb <= 0:
            continue
        c = cov / math.sqrt(va * vb)
        if c > best[1] + 1e-9:
            best = (k, c)
    return best[0], (None if best[0] is None else round(best[1], 3))


def summarize(w: Walk, rows: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    from kilo.planner.errors import estimate

    rows = tick_table(w) if rows is None else rows
    walk_m, setup, stream, end = w.mark("walk"), w.mark("setup"), w.mark("stream"), w.mark("end")
    result, err_m, model = w.mark("result"), w.mark("error"), w.mark("model")
    gait = [r for r in rows if r["mode"] == "STREAM" and r["err_pitch"] is not None]
    out: dict[str, Any] = {
        "path": w.meta.get("path"), "walk": walk_m.fields if walk_m else {},
        "setup": setup.fields if setup else {}, "stream": stream.fields if stream else {},
        "config": (w.mark("config").fields if w.mark("config") else {}),
        "model": model.fields if model else {}, "masses": (w.mark("masses").fields if w.mark("masses") else {}),
        "changes": [dict(m.fields, t_s=round(m.t_us / 1e6, 3), what=m.word) for m in w.marks
                    if m.word in ("change", "stop", "queued", "timing")],
        "end": end.fields if end else {}, "result": result.fields if result else {},
        "tracker": err_m.fields if err_m else {},
        "frames": dict(Counter(f"{f.direction} {f.name}" for f in w.frames)),
        "ticks_planned": len(w.plans), "ticks_sent": sum(1 for _, p in w.plans if p.sent),
        "ticks_gait": len(gait), "bad_records": w.bad,
    }
    if w.frames:
        out["duration_s"] = round((w.frames[-1].t_us - w.frames[0].t_us) / 1e6, 3)
    if not gait:
        out["error"] = None
        return out
    ep = [r["err_pitch"] for r in gait]
    er = [r["err_roll"] for r in gait]
    off_p, off_r = sum(ep) / len(ep), sum(er) / len(er)
    out["error"] = {
        "samples": len(gait),
        "rms_pitch": _r(_rms(ep)), "rms_roll": _r(_rms(er)),
        "max_pitch": _r(max(ep, key=abs)), "max_roll": _r(max(er, key=abs)),
        "offset_pitch": _r(off_p), "offset_roll": _r(off_r),
        "rms_pitch_less_offset": _r(_rms([e - off_p for e in ep])),
        "rms_roll_less_offset": _r(_rms([e - off_r for e in er])),
        "meas_mean_pitch": _r(sum(r["meas_pitch"] for r in gait) / len(gait)),
        "meas_mean_roll": _r(sum(r["meas_roll"] for r in gait) / len(gait)),
        "pred_mean_pitch": _r(sum(r["pred_pitch"] for r in gait) / len(gait)),
        "pred_mean_roll": _r(sum(r["pred_roll"] for r in gait) / len(gait)),
    }
    # the error tracker's estimate over the whole walk (kilo.planner.errors: the
    # lag is relative to the model's servo lag, + = the servos slower)
    est = estimate((r["tick"], r["meas_pitch"], r["meas_roll"], r["pred_pitch"], r["pred_roll"],
                    r["corr_pitch"] or 0.0, r["corr_roll"] or 0.0) for r in gait)
    out["estimate"] = est
    lag_model = model.num("servo_lag_ms") if model else None
    # the servo lag outright: the attitude without lag (plan_*) against the measured one
    plan_r = {r["tick"]: r["plan_roll"] for r in rows if r["plan_roll"] is not None}
    plan_p = {r["tick"]: r["plan_pitch"] for r in rows if r["plan_pitch"] is not None}
    meas_r = {r["tick"]: r["meas_roll"] for r in gait}
    meas_p = {r["tick"]: r["meas_pitch"] for r in gait}
    kr, qr = _xcorr_lag(plan_r, meas_r)
    kp, qp = _xcorr_lag(plan_p, meas_p)
    out["lag"] = {
        "model_ms": lag_model,
        "tracker_vs_model_ms": est.get("lag_ms") if est else None,
        "from_plan_roll_ms": None if kr is None else kr * 20, "quality_roll": qr,
        "from_plan_pitch_ms": None if kp is None else kp * 20, "quality_pitch": qp,
    }
    # by gait phase: which legs are in the air
    by: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for r in gait:
        by[r["lifting"] or "?"].append(r)
    out["by_phase"] = {
        k: {"ticks": len(v),
            "mean_pitch": _r(sum(x["err_pitch"] for x in v) / len(v)),
            "mean_roll": _r(sum(x["err_roll"] for x in v) / len(v)),
            "rms_pitch": _r(_rms([x["err_pitch"] for x in v])), "rms_roll": _r(_rms([x["err_roll"] for x in v])),
            "corr_pitch": _r(sum(x["corr_pitch"] or 0 for x in v) / len(v)),
            "corr_roll": _r(sum(x["corr_roll"] or 0 for x in v) / len(v))}
        for k, v in sorted(by.items(), key=lambda kv: (-len(kv[1]), kv[0]))}
    held = [r for r in gait if r["held"]]
    out["held"] = {"ticks": len(held), "seconds": round(len(held) * (P.TICK_MS / 1000), 2),
                   "fraction": round(len(held) / len(gait), 3)}
    out["balance_active_fraction"] = round(sum(1 for r in gait if r["balance"]) / len(gait), 3)
    cp = [r["corr_pitch"] for r in gait if r["corr_pitch"] is not None]
    cr = [r["corr_roll"] for r in gait if r["corr_roll"] is not None]
    out["correction"] = {"mean_pitch": _r(sum(cp) / len(cp)) if cp else None,
                         "mean_roll": _r(sum(cr) / len(cr)) if cr else None,
                         "max_pitch": _r(max(cp, key=abs)) if cp else None,
                         "max_roll": _r(max(cr, key=abs)) if cr else None}
    one_up = [p.margin / 10.0 for _, p in w.plans if p.margin != -32768 and bin(p.lifting).count("1") == 1]
    out["margin"] = {"min_one_leg_up_mm": _r(min(one_up), 1) if one_up else None,
                     "planner_min_mm": result.num("min_margin_mm") if result else None}
    leads = [r["lead_ms"] for r in gait if r["lead_ms"] is not None]
    if leads:
        out["lead_ms"] = {"min": _r(min(leads), 1), "mean": _r(sum(leads) / len(leads), 1), "max": _r(max(leads), 1)}
    if result:
        out["planner"] = {k: int(result.fields[k]) for k in ("holds", "catches", "waits") if k in result.fields}
        out["planner"]["events"] = [{"t_s": round(t / 1e6, 3), "tick": q.tick, "holds": q.holds, "catches": q.catches,
                                     "waits": q.waits} for t, q in w.planner[:200]]
    return out


def format_summary(s: dict[str, Any]) -> str:
    st, su, en, res = s.get("stream", {}), s.get("setup", {}), s.get("end", {}), s.get("result", {})
    wk = s.get("walk", {})
    L = [f"{s.get('path') or 'walk'}: walk {wk.get('id', '?')} (mind {wk.get('mind_build', '?')}, "
         f"body {wk.get('body_build', '?')})"]
    L.append(f"  asked   {su.get('pattern', '?')} {su.get('dir', '?')} speed {su.get('speed_asked', '?')} "
             f"balance {su.get('balance', '?')}")
    if st:
        L.append(f"  walked  {st.get('pattern')} {st.get('dir')} {st.get('speed')} mm/s, stride {st.get('stride_mm')} "
                 f"mm, period {st.get('period_ms')} ms, swing {st.get('swing_ticks')} ticks")
    cfg = s.get("config", {})
    if cfg:
        L.append("  config  " + " ".join(f"{k}={cfg[k]}" for k in ("margin_mm", "gate_deg", "catch_deg", "hold_max",
                                                                     "lean") if k in cfg))
    mo = s.get("model", {})
    if mo:
        tau = f" + {mo['servo_tau_ms']} ms first-order" if "servo_tau_ms" in mo else ""
        L.append(f"  model   servo lag {mo.get('servo_lag_ms')} ms{tau}, total {mo.get('total_g')} g")
    for c in s.get("changes", []):
        L.append(f"  {c['t_s']:7.2f}s {c['what']}: " + " ".join(f"{k}={v}" for k, v in c.items()
                                                             if k not in ("t_s", "what")))
    L.append(f"  end     {en.get('reason', '?')} (ok {en.get('ok', '?')}); sent {res.get('sent', '?')} skipped "
             f"{res.get('skipped', '?')} underruns {res.get('underruns', '?')}; holds {res.get('holds', '-')} catches "
             f"{res.get('catches', '-')} waits {res.get('waits', '-')}")
    L.append(f"  ticks   {s['ticks_planned']} planned, {s['ticks_sent']} sent, {s['ticks_gait']} measured in STREAM"
             + (f"; {s['duration_s']:.1f} s recorded" if "duration_s" in s else ""))
    e = s.get("error")
    if not e:
        L.append("  error   no STREAM telemetry with a prediction")
        return "\n".join(L)
    L.append(f"  error   (measured - predicted, deg, {e['samples']} ticks)")
    L.append(f"          pitch rms {e['rms_pitch']:.2f} max {e['max_pitch']:+.2f} offset {e['offset_pitch']:+.2f} "
             f"(rms less offset {e['rms_pitch_less_offset']:.2f})")
    L.append(f"          roll  rms {e['rms_roll']:.2f} max {e['max_roll']:+.2f} offset {e['offset_roll']:+.2f} "
             f"(rms less offset {e['rms_roll_less_offset']:.2f})")
    L.append(f"          measured mean pitch {e['meas_mean_pitch']:+.2f} roll {e['meas_mean_roll']:+.2f}; "
             f"predicted mean pitch {e['pred_mean_pitch']:+.2f} roll {e['pred_mean_roll']:+.2f}")
    lag = s.get("lag", {})

    def ms(v: Any) -> str:
        return "-" if v is None else f"{v:+.0f} ms"
    L.append(f"  lag     model {lag.get('model_ms')} ms; tracker: {ms(lag.get('tracker_vs_model_ms'))} vs the model; "
             f"from the plan without lag: roll {'-' if lag.get('from_plan_roll_ms') is None else lag['from_plan_roll_ms']}"
             f" ms (r {lag.get('quality_roll')}), pitch "
             f"{'-' if lag.get('from_plan_pitch_ms') is None else lag['from_plan_pitch_ms']} ms (r {lag.get('quality_pitch')})")
    tr = s.get("tracker") or {}
    if tr:
        L.append("  tracker (the mind's, last 4 s) " + " ".join(f"{k}={v}" for k, v in tr.items()))
    L.append("  by phase (legs in the air): ticks, mean / rms error pitch, roll; mean correction pitch, roll")
    for k, v in s.get("by_phase", {}).items():
        L.append(f"    {k:8s} {v['ticks']:5d}  {v['mean_pitch']:+6.2f} / {v['rms_pitch']:5.2f}  "
                 f"{v['mean_roll']:+6.2f} / {v['rms_roll']:5.2f}   corr {v['corr_pitch']:+6.2f} {v['corr_roll']:+6.2f}")
    h, c = s.get("held", {}), s.get("correction", {})
    L.append(f"  held    {h.get('ticks')} ticks ({h.get('seconds')} s, {h.get('fraction', 0):.0%} of STREAM); balance "
             f"active {s.get('balance_active_fraction', 0):.0%}")
    L.append(f"  correction mean pitch {c.get('mean_pitch')} roll {c.get('mean_roll')}, largest pitch "
             f"{c.get('max_pitch')} roll {c.get('max_roll')} deg")
    m = s.get("margin", {})
    L.append(f"  margin  smallest with one leg up {m.get('min_one_leg_up_mm')} mm (the planner's {m.get('planner_min_mm')})")
    if "lead_ms" in s:
        ld = s["lead_ms"]
        L.append(f"  lead    POSE sent to its tick reported: min {ld['min']} mean {ld['mean']} max {ld['max']} ms")
    return "\n".join(L)


def plot(rows: list[dict[str, Any]], out: str | Path, title: str = "") -> tuple[Path, str]:
    """PNG: predicted vs measured pitch and roll, the error, the correction,
    held / legs in the air. CSV instead without matplotlib."""
    out = Path(out)
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        p = write_csv(rows, out.with_suffix(".csv"))
        return p, f"wrote CSV {p} (matplotlib is not installed)"
    t = [(r["t_ms"] or 0) / 1000 for r in rows]

    def col(k: str) -> list[float]:
        return [math.nan if r[k] is None else float(r[k]) for r in rows]
    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(11, 9))
    for i, axis in enumerate(("pitch", "roll")):
        ax[i].plot(t, col(f"pred_{axis}"), lw=1, label="predicted (POSE)")
        ax[i].plot(t, col(f"meas_{axis}"), lw=1, label="measured")
        ax[i].plot(t, col(f"plan_{axis}"), lw=0.8, ls=":", label="plan, no lag")
        ax[i].set_ylabel(f"{axis} °")
        ax[i].legend(fontsize=7, loc="upper right")
        ax[i].grid(True, alpha=0.3)
    ax[2].plot(t, col("err_pitch"), lw=1, label="error pitch")
    ax[2].plot(t, col("err_roll"), lw=1, label="error roll")
    ax[2].plot(t, col("corr_pitch"), lw=0.8, ls="--", label="correction pitch")
    ax[2].plot(t, col("corr_roll"), lw=0.8, ls="--", label="correction roll")
    ax[2].set_ylabel("°")
    ax[2].legend(fontsize=7, loc="upper right")
    ax[2].grid(True, alpha=0.3)
    ax[3].step(t, col("held"), lw=1, where="post", label="held")
    lift = [math.nan if r["lifting"] is None else (0 if r["lifting"] == "none" else len(r["lifting"].split("+")))
            for r in rows]
    ax[3].step(t, lift, lw=1, where="post", label="legs in the air")
    ax[3].plot(t, [math.nan if v is None else v / 10 for v in col("margin_mm")], lw=0.8, label="margin / 10 mm")
    ax[3].set_xlabel("t (s) since the recording's start")
    ax[3].legend(fontsize=7, loc="upper right")
    ax[3].grid(True, alpha=0.3)
    ax[0].set_title(title, fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out, f"wrote {out}"


def info_dict(info: Any) -> dict[str, Any]:
    """A P.HostWalkInfo as plain data, its text split into its key=value fields."""
    d = {k: v for k, v in P.to_dict(info).items() if k != "text"}
    text = bytes(info.text).decode("ascii", "replace")
    word, fields = parse_mark("x " + text.replace("end=", "reason=", 1))
    if "reason" in fields:
        fields["end"] = fields.pop("reason")
    d["text"] = text
    d.update(fields)
    d["flags"] = [f.name for f in P.WalkFlags if info.flags & f]
    return d
