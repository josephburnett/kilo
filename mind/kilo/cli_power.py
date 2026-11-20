"""``kilo power``: light sleep and the battery (docs/power.md; the mind's
side is kernel/mind/power/sleep.h).

    kilo power --host IP [--json]   the power state now, the time in each state, the battery's
                                    trend in mV per minute (the mind keeps 30 min of it)
    kilo power FILE.krec [--json]   the battery's trend in a recording (kilo record fetch), per
                                    stretch: asleep, limp, resting or standing, walking, a trick

Kilo has no current sensor. The battery's voltage falling is the only
measure of the draw there is: the faster it falls, the more is drawn. It
sags under load and climbs back when the load goes, so each stretch's
slope leaves out its first SETTLE_S seconds, and stretches shorter than
MIN_STRETCH_S get no slope.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from typing import Any, Iterable

from kilo import EXIT_ERR, EXIT_OK

SETTLE_S = 60.0          # a stretch's first minute: the voltage settles after the load changed
MIN_STRETCH_S = 90.0     # shorter stretches: no slope
METRIC_GROUPS = ("power", "power_hw", "battery", "power_segments")


def add_parsers(sub: Any) -> None:
    s = sub.add_parser("power", help="light sleep and the battery's trend: over --host, or from a recording",
                       description="Over --host: the mind's power state (awake, drowsy: A7 counting down, asleep), "
                                   "the time in each, and the battery's trend in mV per minute. With a .krec "
                                   "(kilo record fetch): the trend per stretch of the recording (asleep, limp, "
                                   "resting or standing, walking, a trick). docs/power.md.")
    s.add_argument("file", nargs="?", default=None, help="a recording (.krec) instead of --host")
    s.add_argument("--json", action="store_true", help="as JSON")


# ---- the mind's metrics lines ----------------------------------------------------------

def parse_metrics(lines: Iterable[str]) -> dict[str, Any]:
    """The power lines of METRICS as {group: {key: value}} (power_segments: a list)."""
    out: dict[str, Any] = {}
    for ln in lines:
        group, _, rest = ln.partition(":")
        if group not in METRIC_GROUPS:
            continue
        if group == "power_segments":
            segs = []
            for tok in rest.split():
                parts = tok.split(":")
                if len(parts) != 4:
                    continue
                state, dur, mv, slope = parts
                first, _, last = mv.rstrip("mV").partition("-")
                segs.append({"state": state, "s": _num(dur.rstrip("s")), "first_mv": _num(first),
                             "last_mv": _num(last), "mv_per_min": _num(slope)})
            out[group] = segs
            continue
        d = {}
        for tok in rest.split():
            k, eq, v = tok.partition("=")
            if eq:
                d[k] = _num(v)
        out[group] = d
    return out


def _num(v: str) -> Any:
    try:
        return int(v)
    except ValueError:
        try:
            return float(v)
        except ValueError:
            return None if v == "?" else v


def _hms(s: Any) -> str:
    if not isinstance(s, (int, float)):
        return "?"
    s = int(s)
    return f"{s // 3600}:{s // 60 % 60:02d}:{s % 60:02d}"


def _rate(v: Any) -> str:
    return "-" if v is None else f"{v:+.1f} mV/min"


def print_power(d: dict[str, Any]) -> None:
    p, hw, b = d.get("power", {}), d.get("power_hw", {}), d.get("battery", {})
    print(f"state       {p.get('state')} for {_hms(p.get('for_s'))}  (light sleep {p.get('light_sleep')}; "
          f"{p.get('sleeps')} sleeps, {p.get('wakes')} wakes, last wake {p.get('last_wake')})")
    print(f"time        awake {_hms(p.get('awake_s'))}  drowsy {_hms(p.get('drowsy_s'))}  "
          f"asleep {_hms(p.get('asleep_s'))}")
    print(f"battery     {b.get('mv')} mV (lowest {b.get('min_mv')})  trend {_rate(b.get('mv_per_min_1m'))} over 1 min, "
          f"{_rate(b.get('5m'))} 5 min, {_rate(b.get('15m'))} 15 min, {_rate(b.get('30m'))} 30 min")
    print(f"            this state (after its first minute) {_rate(b.get('this_state'))}; "
          f"{b.get('samples')} samples of 10 s")
    for g in d.get("power_segments", []):
        print(f"stretch     {g['state']:7s} {_hms(g['s'])}  {g['first_mv']} -> {g['last_mv']} mV  "
              f"{_rate(g['mv_per_min'])}")
    if hw:
        mhz = hw.get("arm_mhz")
        print(f"Pi          ARM {mhz or '?'} MHz, SoC {hw.get('temp_c')} C, throttled {hw.get('throttled')}"
              f"{' (under-voltage seen)' if _undervolt(hw.get('throttled')) else ''}; Ethernet PHY "
              f"{hw.get('net')} (device {hw.get('net_device')}, {hw.get('net_saves')} power-downs, "
              f"{hw.get('net_looks')} looks, {hw.get('net_errors')} errors); card saves before sleep "
              f"{hw.get('card_flushes')}")


def _undervolt(v: Any) -> bool:
    try:
        return bool(int(str(v), 16) & 0x10001)
    except ValueError:
        return False


def cmd_power(args: argparse.Namespace, hl: Any) -> int:
    from kilo.hostlink import HostRefused

    try:
        lines = hl.metrics()
    except HostRefused as e:
        print(f"kilo power: this mind keeps no metrics ({e.code_name})", file=sys.stderr)
        return EXIT_ERR
    d = parse_metrics(lines)
    if "power" not in d:
        print("kilo power: this mind has no light sleep (an older kernel: kilo update --host)", file=sys.stderr)
        return EXIT_ERR
    if args.json:
        print(json.dumps(d, indent=2))
    else:
        print_power(d)
        print("(this connection woke Kilo if he was asleep; A7 still holds: he sleeps again 30 s after it closes)")
    return EXIT_OK


# ---- a recording -----------------------------------------------------------------------

@dataclass
class Stretch:
    kind: str           # asleep, limp, idle (resting or standing, servos on), walking, trick
    t0_s: float
    t1_s: float
    n: int
    first_mv: int
    last_mv: int
    mean_mv: float
    mv_per_min: float | None   # after SETTLE_S; None if too short

    @property
    def s(self) -> float:
        return self.t1_s - self.t0_s


def kind_of(mode: str, servos: bool, asleep: bool) -> str:
    if asleep:
        return "asleep"
    if mode == "GAIT":
        return "walking"
    if mode in ("TRICK", "FALLEN", "RECOVERING"):
        return "trick"
    if mode == "LIMP" or not servos:
        return "limp"
    return "idle"


def slope_mv_min(pts: list[tuple[float, int]]) -> float | None:
    n = len(pts)
    if n < 3:
        return None
    mt = sum(t for t, _ in pts) / n
    mm = sum(m for _, m in pts) / n
    den = sum((t - mt) ** 2 for t, _ in pts)
    if den <= 0:
        return None
    return sum((t - mt) * (m - mm) for t, m in pts) / den * 60.0


def stretches(samples: Iterable[tuple[float, str, bool, int]], logs: Iterable[tuple[float, str]]) -> list[Stretch]:
    """samples: (t_s, mode name, servos powered, batt_mv); logs: (t_s, text). Contiguous
    stretches of one kind; light sleep from the mind's "power: light sleep (" to "power: awake (" lines."""
    marks = sorted((t, txt) for t, txt in logs if txt.startswith(("power: light sleep (", "power: awake (")))
    out: list[Stretch] = []
    cur: list[tuple[float, int]] = []
    cur_kind = None
    mi = 0
    asleep = False

    def close() -> None:
        if not cur:
            return
        t0 = cur[0][0]
        settled = [p for p in cur if p[0] >= t0 + SETTLE_S]
        rate = slope_mv_min(settled) if cur[-1][0] - t0 >= MIN_STRETCH_S else None
        out.append(Stretch(cur_kind or "?", t0, cur[-1][0], len(cur), cur[0][1], cur[-1][1],
                           sum(m for _, m in cur) / len(cur), None if rate is None else round(rate, 1)))

    for t, mode, servos, mv in samples:
        while mi < len(marks) and marks[mi][0] <= t:
            asleep = marks[mi][1].startswith("power: light sleep (")
            mi += 1
        if not mv:
            continue
        k = kind_of(mode, servos, asleep)
        if k != cur_kind:
            close()
            cur, cur_kind = [], k
        cur.append((t, mv))
    close()
    return out


def summary(st: list[Stretch]) -> dict[str, dict[str, Any]]:
    """Per kind: the time, and the slope over its stretches weighted by their settled length."""
    out: dict[str, dict[str, Any]] = {}
    for g in st:
        d = out.setdefault(g.kind, {"s": 0.0, "weighted": 0.0, "w": 0.0, "mean_mv": []})
        d["s"] += g.s
        d["mean_mv"].append(g.mean_mv)
        if g.mv_per_min is not None:
            w = max(0.0, g.s - SETTLE_S)
            d["weighted"] += g.mv_per_min * w
            d["w"] += w
    return {k: {"s": round(d["s"], 1), "mv_per_min": round(d["weighted"] / d["w"], 1) if d["w"] else None,
                "mean_mv": round(sum(d["mean_mv"]) / len(d["mean_mv"]))} for k, d in out.items()}


def from_file(path: str) -> list[Stretch]:
    from kilo.hostlink import enum_name
    from kilo.logs import flight
    from kilo.protocol import kilo_protocol as P

    s = flight.load(path)
    t0 = s.t_first_us or 0
    samples = [((t - t0) / 1e6, enum_name(P.Mode, m.mode), bool(m.flags & 4), 0 if m.flags & (1 << 9) else m.batt_mv)
               for t, m in s.telemetry()]
    logs = [((g.t_us - t0) / 1e6, g.text) for g in s.logs]
    return stretches(samples, logs)


def offline(args: argparse.Namespace) -> int:
    if not args.file:
        print("kilo power: needs --host IP[:PORT] (the mind's power state), or a recording: kilo power FILE.krec",
              file=sys.stderr)
        return EXIT_ERR
    try:
        st = from_file(args.file)
    except (OSError, ValueError) as e:
        print(f"kilo power: {e}", file=sys.stderr)
        return EXIT_ERR
    summ = summary(st)
    if args.json:
        print(json.dumps({"stretches": [asdict(g) | {"s": round(g.s, 1)} for g in st], "by_kind": summ}, indent=2))
        return EXIT_OK
    print(f"{'kind':8s} {'from s':>8s} {'for':>8s} {'mV first':>8s} {'last':>6s} {'mean':>6s}  slope (after {SETTLE_S:.0f} s)")
    for g in st:
        print(f"{g.kind:8s} {g.t0_s:8.1f} {_hms(g.s):>8s} {g.first_mv:8d} {g.last_mv:6d} {g.mean_mv:6.0f}  "
              f"{_rate(g.mv_per_min)}")
    print("by kind:")
    for k, d in summ.items():
        print(f"  {k:8s} {_hms(d['s'])}  mean {d['mean_mv']} mV  {_rate(d['mv_per_min'])}")
    print("(a slope needs a stretch of 90 s or more; the faster the voltage falls, the more is drawn)")
    return EXIT_OK
