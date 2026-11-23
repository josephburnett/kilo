"""What the floor recordings say about the movement engine (docs/movement-engine.md, Part one A;
results in docs/movement-engine-results.md).

    PYTHONPATH=mind python mind/tests/enginestudy.py inventory   # the boots, builds and runs found
    PYTHONPATH=mind python mind/tests/enginestudy.py link        # 1. how late the link is, how short the look-ahead can be
    PYTHONPATH=mind python mind/tests/enginestudy.py margin      # 2. the plan's support margin against the measured tip
    PYTHONPATH=mind python mind/tests/enginestudy.py balance     # 3. how much the body's balance correction acts
    PYTHONPATH=mind python mind/tests/enginestudy.py lag         # 4. the servo lag against the battery
    PYTHONPATH=mind python mind/tests/enginestudy.py estimate    # 5. the always-on state estimate against what was asked
    PYTHONPATH=mind python mind/tests/enginestudy.py getup       # 6. rc's get-ups: roll speed, time to still, retries
    PYTHONPATH=mind python mind/tests/enginestudy.py tricks      # 7. tricks on schedule, IMU against the model
    PYTHONPATH=mind python mind/tests/enginestudy.py all

Every subcommand takes recordings as arguments (default: every .krec in ~/kilo-logs and
~/kilo-logs/from-server, recursively). The same boot is often in several files (the card's whole
boot, the server's fetch of it, one file per run); every 16 KB block is used once, keyed by
(boot, block number), the fullest copy kept, and each boot is analysed as one stream. So the
numbers can be recomputed after the next floor runs by fetching the card and running this again.

Everything here is a measurement of what the recordings hold (the old walk, the old trot and long
trot, the crawl, the builds of 2026-11-17 to 11-21) except where a line says "model".
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import statistics
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field

from kilo.logs import flight, proto2
from kilo.logs import walkrec as W
from kilo.protocol import kilo_protocol as P

A16 = 16.0
TICK_US = P.TICK_MS * 1000


# ---- the recordings, one stream per boot ------------------------------------------------------

@dataclass
class Boot:
    boot: int
    stream: flight.Stream
    files: list[str] = field(default_factory=list)

    @property
    def builds(self) -> tuple[str, str]:
        for m in self.stream.marks:
            if "mind_build" in m.fields:
                return m.fields.get("mind_build", "?"), m.fields.get("body_build", "?")
        for g in self.stream.logs:
            if "Kilo mind, build " in g.text:
                return g.text.split("build ")[1].split()[0], "?"
        return "?", "?"

    @property
    def name(self) -> str:
        return f"b{self.boot}"


def default_files() -> list[str]:
    h = os.path.expanduser("~/kilo-logs")
    return sorted(glob.glob(h + "/*.krec") + glob.glob(h + "/from-server/**/*.krec", recursive=True))


def boots(paths) -> list[Boot]:
    """Every block of every file once: (boot, seq) -> the fullest copy; one Stream per boot."""
    best: dict[tuple[int, int], tuple[int, bytes, str]] = {}
    for p in paths:
        try:
            meta, data = flight.read_file(p)
        except (ValueError, OSError):
            continue
        s = flight.parse_blocks(data)
        for b in s.blocks:
            if b.problem and b.problem != "cut":
                continue
            raw = data[b.offset:b.offset + b.have]
            k = (b.boot, b.seq)
            if k not in best or len(raw) > best[k][0]:
                best[k] = (len(raw), raw, p)
    by: dict[int, list[tuple[int, bytes, str]]] = defaultdict(list)
    for (boot, seq), (_, raw, p) in best.items():
        by[boot].append((seq, raw, p))
    out = []
    for boot in sorted(by):
        parts = sorted(by[boot])
        s = flight.parse_blocks(b"".join(raw for _, raw, _ in parts), {"boot": boot})
        out.append(Boot(boot, s, sorted({os.path.basename(p) for _, _, p in parts})))
    return out


def walks(bt: Boot):
    """(RunSpan, walkrec.Walk, its tick table) for every walk run of the boot."""
    for run in bt.stream.runs:
        if run.kind != "walk":
            continue
        w = flight.walk_of(bt.stream, run)
        rows = W.tick_table(w)
        if rows:
            yield run, w, rows


def patterns_of(w: W.Walk) -> list[str]:
    out = []
    for m in w.marks:
        if m.word in ("stream", "timing", "change") and m.fields.get("pattern") and m.fields["pattern"] not in out:
            out.append(m.fields["pattern"])
    return out


def pct(xs, q: float) -> float:
    xs = sorted(xs)
    if not xs:
        return math.nan
    i = min(len(xs) - 1, max(0, int(round(q * (len(xs) - 1)))))
    return xs[i]


def spread(xs, unit: str = "", f: str = ".1f") -> str:
    """n, median, the 5th-95th percentile range, min and max."""
    xs = [x for x in xs if x is not None and not math.isnan(x)]
    if not xs:
        return "n 0"
    return (f"n {len(xs)}, median {format(statistics.median(xs), f)}{unit}, 5-95 % "
            f"{format(pct(xs, .05), f)}..{format(pct(xs, .95), f)}, min {format(min(xs), f)} max {format(max(xs), f)}")


# ---- inventory ----------------------------------------------------------------------------------

def inventory(bs: list[Boot]) -> None:
    print("| boot | mind / body build (? : the boot has no run to say) | TELEMETRY | runs | walk patterns | tricks | falls (FELL) |")
    print("|---|---|---|---|---|---|---|")
    for bt in bs:
        s = bt.stream
        tel = sum(1 for f in s.frames if f.direction == "rx" and isinstance(f.msg, P.Telemetry))
        pats: Counter = Counter()
        for run, w, rows in walks(bt):
            for p in patterns_of(w):
                pats[p] += 1
        tricks = Counter(m.fields.get("trick", "?") for m in s.marks if m.word == "trick")
        if any(m.word == "measure" for m in s.marks):
            tricks["self-measurement"] += 1
        fell = sum(1 for f in s.frames if f.direction == "rx" and isinstance(f.msg, P.Event)
                   and f.msg.code == P.EventCode.FELL)
        mb, bb = bt.builds
        mins = ((s.t_last_us or 0) - (s.t_first_us or 0)) / 60e6
        print(f"| {bt.name} | {mb} / {bb} | {tel} over {mins:.1f} min | {len(s.runs)} "
              f"| {', '.join(f'{k} {v}' for k, v in pats.items()) or '-'} "
              f"| {', '.join(f'{k} {v}' for k, v in tricks.items()) or '-'} | {fell} |")


# ---- 1. the link ---------------------------------------------------------------------------------

def link(bs: list[Boot]) -> None:
    """Per walk run in GAIT: how long before its tick's TELEMETRY each FEET went out (``lead``), the
    body's queue, the TELEMETRY's arrival jitter, the gaps between the mind's sends, and the mind's
    loop in ticks (a FEET's tick minus the newest TELEMETRY tick the mind had when it sent it)."""
    lead, queued, jitter, gaps, loop = [], [], [], [], []
    per_build: dict[str, list[float]] = defaultdict(list)
    n_runs, n_ticks = 0, 0
    for bt in bs:
        for run, w, rows in walks(bt):
            g = [r for r in rows if r["mode"] == "STREAM"]
            if len(g) < 25:
                continue
            n_runs += 1
            n_ticks += len(g)
            ld = [r["lead_ms"] for r in g if r["lead_ms"] is not None]
            lead += ld
            per_build[bt.builds[0]] += ld
            queued += [r["pose_queued"] for r in g if r["pose_queued"] is not None]
            # TELEMETRY arrival against the body's tick grid: the lower envelope of (arrival - 20 ms * tick)
            # over a 1 s window is the fastest arrival; above it is jitter (the body's send, the wire, the mind)
            ar = [(r["tick"], r["telem_ms"]) for r in g if r["telem_ms"] is not None]
            for i, (k, t) in enumerate(ar):
                win = ar[max(0, i - 25):i + 25]
                base = min(t2 - (k2 - k) * P.TICK_MS for k2, t2 in win)
                jitter.append(t - base)
            # the mind's sends: gaps between FEET frames, and how far ahead of the newest telemetry
            last_tick = None
            for f in w.frames:
                if f.msg is None:
                    continue
                if f.direction == "rx" and isinstance(f.msg, P.Telemetry):
                    last_tick = f.msg.tick if last_tick is None else W._unwrap(f.msg.tick, last_tick)
                elif f.direction == "tx" and proto2.is_stream(f.msg) and last_tick is not None:
                    j = W._unwrap(f.msg.tick, last_tick)
                    loop.append(j - last_tick)
            sends = sorted(r["pose_sent_ms"] for r in g if r["pose_sent_ms"] is not None)
            gaps += [b - a for a, b in zip(sends, sends[1:]) if b - a < 200]  # longer: another run's stream
    print(f"walk runs in GAIT: {n_runs}, {n_ticks} ticks (old walk, old trot, long trot, crawl; builds 2026-11-17..21)")
    print(f"FEET sent to its tick's TELEMETRY received (lead, ms): {spread(lead)}")
    for b, v in per_build.items():
        print(f"   mind {b}: {spread(v)}")
    print(f"body's queue at each tick (feet_queued, ticks): {dict(sorted(Counter(queued).items()))}")
    print(f"TELEMETRY arrival above the fastest in its second (ms): {spread(jitter)}")
    print(f"gaps between the mind's FEET sends (ms): {spread(gaps)}; over 30 ms: {sum(1 for x in gaps if x > 30)}, "
          f"over 40 ms: {sum(1 for x in gaps if x > 40)}")
    print(f"the mind's loop: FEET tick - newest TELEMETRY tick when sent: {dict(sorted(Counter(loop).items()))}")
    if lead:
        lo = min(lead)
        print(f"read: the earliest any FEET arrived before its tick's telemetry was {lo:.1f} ms with a 100 ms "
              f"look-ahead; {sum(1 for x in lead if x < 60)} of {len(lead)} under 60 ms")


# ---- 2. the support margin against the tip ---------------------------------------------------------

# a lifted leg's corner: (front +1 / back -1, left +1 / right -1). The tripod tips about the diagonal of
# the two feet beside it; the tilt about that line is pitch * NX + roll * NY toward the corner (pitch nose
# down +, roll right side up + = the left side down), the line's normal for a 123 x 65 mm rectangle.
CORNER = {"LF": (1, 1), "RF": (1, -1), "LB": (-1, 1), "RB": (-1, -1)}
NX, NY = 0.47, 0.88
LAG_TICKS = 3   # the servos' 50 + 70 ms: a swing's effect is read this many ticks after it ends
MARGIN_BINS = (-99, -6, -4, -2, 0, 2, 4, 6, 8, 10, 12, 14, 99)


def swings(bs: list[Boot]) -> list[dict]:
    """Every swing of one leg alone (contiguous ticks with exactly that leg up), with the plan's smallest
    margin over it and the measured tilt about the tripod's diagonal toward the lifted corner, of the error
    (measured - predicted), from the tick before the lift to LAG_TICKS after the landing. ``clean``: no
    second leg lifted in that window (the old walk's single-leg windows run into its two-feet phase)."""
    out = []
    for bt in bs:
        for run, w, rows in walks(bt):
            pats = patterns_of(w)
            by = {r["tick"]: r for r in rows}
            g = [r for r in rows if r["mode"] == "STREAM"]
            i = 0
            while i < len(g):
                r = g[i]
                if r["lifting"] not in CORNER or r["margin_mm"] is None:
                    i += 1
                    continue
                j = i
                while j + 1 < len(g) and g[j + 1]["lifting"] == r["lifting"] and g[j + 1]["tick"] == g[j]["tick"] + 1:
                    j += 1
                leg, t0, t1 = r["lifting"], g[i]["tick"], g[j]["tick"]
                win = [by.get(k) for k in range(t0 - 1, t1 + LAG_TICKS + 1)]
                if all(x is not None and x["err_roll"] is not None for x in win):
                    fx, fy = CORNER[leg]

                    def tow(x):
                        return x["err_pitch"] * fx * NX + x["err_roll"] * fy * NY
                    a = tow(win[0])
                    out.append({"pattern": "+".join(pats), "boot": bt.name, "build": bt.builds[0], "leg": leg,
                                "ticks": j - i + 1, "margin": min(g[k]["margin_mm"] for k in range(i, j + 1)),
                                "clean": all(x["lifting"] in ("none", leg) for x in win[1:]),
                                "toward": tow(win[-1]) - a, "swing_abs": max(abs(tow(x) - a) for x in win)})
                i = j + 1
    return out


def margin(bs: list[Boot]) -> None:
    sw = swings(bs)
    print(f"single-leg swings: {len(sw)} ({sum(s['clean'] for s in sw)} clean); patterns "
          f"{dict(Counter(s['pattern'] for s in sw))}")
    print("| plan's margin (mm, + inside) | swings | of them clean | ticks up (median) | mostly | tilt toward the lifted "
          "corner at landing + 60 ms (deg, mean +- se) | largest tilt either way (deg, mean) |")
    print("|---|---|---|---|---|---|---|")
    for b0, b1 in zip(MARGIN_BINS, MARGIN_BINS[1:]):
        v = [s for s in sw if b0 <= s["margin"] < b1]
        if not v:
            continue
        d = [s["toward"] for s in v]
        se = statistics.pstdev(d) / math.sqrt(len(d)) if len(d) > 1 else math.nan
        lo = "below" if b0 == -99 else f"{b0:+d}"
        hi = "above" if b1 == 99 else f"{b1:+d}"
        print(f"| {lo} to {hi} | {len(v)} | {sum(s['clean'] for s in v)} | {statistics.median(s['ticks'] for s in v):.0f} "
              f"| {Counter(s['pattern'] for s in v).most_common(1)[0][0]} | {statistics.mean(d):+.2f} +- {se:.2f} "
              f"| {statistics.mean(s['swing_abs'] for s in v):.2f} |")


# ---- 3. the body's balance correction ---------------------------------------------------------------

def balance(bs: list[Boot]) -> None:
    """corr_pitch / corr_roll and the BALANCE_ACTIVE / BALANCE_HELD flags over every TELEMETRY, by mode."""
    acc: dict[str, dict] = defaultdict(lambda: {"n": 0, "active": 0, "held": 0, "corr": [], "err": []})
    for bt in bs:
        for t, m in bt.stream.telemetry():
            mode = proto2.mode_name(m.mode)
            a = acc[mode]
            a["n"] += 1
            fl = P.TelemetryFlags(m.flags)
            act = bool(fl & P.TelemetryFlags.BALANCE_ACTIVE)
            a["active"] += act
            a["held"] += bool(fl & P.TelemetryFlags.BALANCE_HELD)
            if act:
                a["corr"].append(math.hypot(m.corr_pitch, m.corr_roll) / A16)
                a["err"].append(math.hypot(m.pitch - m.ref_pitch, m.roll - m.ref_roll) / A16)
    print("| mode | TELEMETRY | balance active | held | correction (deg, when active) | over 1 deg | over 3 deg "
          "| attitude - reference (deg, when active) |")
    print("|---|---|---|---|---|---|---|---|")
    for mode, a in sorted(acc.items(), key=lambda kv: -kv[1]["n"]):
        c = a["corr"]
        print(f"| {mode} | {a['n']} | {a['active'] / a['n']:.0%} | {a['held'] / a['n']:.0%} "
              f"| {spread(c, '', '.2f') if c else '-'} | {sum(1 for x in c if x > 1) / max(1, len(c)):.1%} "
              f"| {sum(1 for x in c if x > 3) / max(1, len(c)):.1%} | {spread(a['err'], '', '.2f') if a['err'] else '-'} |")


# ---- 4. the servo lag against the battery -------------------------------------------------------------

def _lag_fine(src: dict, dst: dict, max_k: int = 15) -> tuple[float | None, float | None]:
    """The shift (ms) that best lines src[t - k] up with dst[t], to a fraction of a tick (a parabola
    through the best three), and its correlation; None when src hardly moves."""
    vals = list(src.values())
    if len(vals) < 50 or statistics.pstdev(vals) < 0.3:
        return None, None
    cs = []
    for k in range(max_k + 1):
        pairs = [(v, dst[t + k]) for t, v in src.items() if t + k in dst]
        if len(pairs) < 50:
            cs.append(-2.0)
            continue
        xa, xb = zip(*pairs)
        sa, sb = statistics.pstdev(xa), statistics.pstdev(xb)
        if sa <= 0 or sb <= 0:
            cs.append(-2.0)
            continue
        ma, mb = statistics.mean(xa), statistics.mean(xb)
        cs.append(sum((a - ma) * (b - mb) for a, b in pairs) / (len(pairs) * sa * sb))
    k = max(range(len(cs)), key=lambda i: cs[i])
    frac = 0.0
    if 0 < k < len(cs) - 1:
        den = cs[k - 1] - 2 * cs[k] + cs[k + 1]
        if den < 0:
            frac = 0.5 * (cs[k - 1] - cs[k + 1]) / den
    return (k + frac) * P.TICK_MS, cs[k]


def lag(bs: list[Boot]) -> None:
    """Per walk run: the delay from the plan's attitude without lag (PLAN plan_pitch/plan_roll) to the
    measured one, against the battery over the run."""
    out = []
    for bt in bs:
        for run, w, rows in walks(bt):
            g = [r for r in rows if r["mode"] == "STREAM" and r["meas_roll"] is not None]
            if len(g) < 100:
                continue
            batt = [m.batt_mv for t, m in [(f.t_us, f.msg) for f in w.frames if f.direction == "rx"
                                           and isinstance(f.msg, P.Telemetry)] if m.mode == P.Mode.STREAM and m.batt_mv]
            for axis in ("roll", "pitch"):
                src = {r["tick"]: r[f"plan_{axis}"] for r in rows if r[f"plan_{axis}"] is not None}
                dst = {r["tick"]: r[f"meas_{axis}"] for r in g}
                ms, c = _lag_fine(src, dst)
                if ms is not None and c > 0.5:
                    out.append((bt.name, bt.builds[0], run.id, "+".join(patterns_of(w)), axis, ms, c,
                                statistics.mean(batt) if batt else None, len(g)))
    print("| boot | mind | run | gait | axis | lag (ms) | correlation | battery (mV) | ticks |")
    print("|---|---|---|---|---|---|---|---|---|")
    for b, mb, rid, pat, axis, ms, c, mv, n in out:
        print(f"| {b} | {mb} | {rid} | {pat} | {axis} | {ms:.0f} | {c:.2f} | {mv:.0f} | {n} |")
    for axis in ("roll", "pitch"):
        pts = [(o[7], o[5]) for o in out if o[4] == axis and o[7]]
        if len(pts) >= 3:
            xs, ys = zip(*pts)
            mx, my = statistics.mean(xs), statistics.mean(ys)
            sxx = sum((x - mx) ** 2 for x in xs)
            slope = sum((x - mx) * (y - my) for x, y in pts) / sxx if sxx else math.nan
            r = (sum((x - mx) * (y - my) for x, y in pts) / math.sqrt(sxx * sum((y - my) ** 2 for y in ys))
                 if sxx and statistics.pstdev(ys) else math.nan)
            print(f"{axis}: {spread(ys, ' ms', '.0f')}; battery {min(xs):.0f}..{max(xs):.0f} mV; slope "
                  f"{slope * 100:+.1f} ms per 100 mV, r {r:+.2f}")


# ---- 5. the state estimate ------------------------------------------------------------------------------

def estimate(bs: list[Boot]) -> None:
    """kilo.intent.replay over every boot (one stream each), and the estimate's changes of state."""
    from kilo.intent import estimate as E
    from kilo.intent import replay as R
    total: Counter = Counter()
    wrong, changes, flips = [], Counter(), []
    for bt in bs:
        est = E.Estimator()
        last_state, last_t, run = None, None, None
        for smp, since in R.samples(bt.stream):
            if est.feed(smp):
                st = est.est.state
                changes[(bt.name)] += 1
                if last_state is not None and last_t is not None and smp.t_us - last_t < 1e6:
                    flips.append((bt.name, bt.stream.uptime_s(smp.t_us), last_state.name, st.name))
                last_state, last_t = st, smp.t_us
            judged = smp.mode in (int(P.Mode.IDLE), int(P.Mode.LIMP)) and smp.t_us - since >= R.ASKED_S * 1e6
            if not judged:
                run = None
                continue
            st = est.est.state
            total[(smp.cmd, st)] += 1
            want = E.CMD_STATE.get(smp.cmd)
            if want is not None and st not in (want, E.State.MOVING):
                if run is not None and run[0] == smp.cmd and run[1] == st:
                    run[3] = smp.t_us
                else:
                    run = [smp.cmd, st, smp.t_us, smp.t_us, smp.pitch / 16, smp.roll / 16, bt]
                    wrong.append(run)
            else:
                run = None
    cols = list(E.State)
    print("| asked \\ estimate | " + " | ".join(c.name for c in cols) + " | agree |")
    print("|---|" + "---|" * (len(cols) + 1))
    for cmd in E.Cmd:
        row = [total.get((cmd, c), 0) for c in cols]
        if sum(row):
            want = E.CMD_STATE.get(cmd)
            ok = sum(total.get((cmd, c), 0) for c in cols if want is None or c in (want, E.State.MOVING))
            print(f"| {cmd.name.lower()} | " + " | ".join(str(v) for v in row) + f" | {ok / sum(row):.1%} |")
    print(f"\nwrong stretches (a posture asked, another resting state estimated): {len(wrong)}")
    for cmd, st, t0, t1, p, r, bt in wrong:
        print(f"  {bt.name} {bt.stream.uptime_s(t0):7.1f} s for {(t1 - t0) / 1e6:5.1f} s: asked {cmd.name.lower()}, "
              f"estimate {st.name}, pitch {p:+.1f} roll {r:+.1f}")
    rest_flips = [f for f in flips if "MOVING" not in (f[2], f[3])]
    print(f"changes of state: {sum(changes.values())} over {len(bs)} boots; a change within 1 s of the last: "
          f"{len(flips)}, of them between two resting states (no MOVING): {len(rest_flips)}")
    for b, t, a, c in rest_flips:
        print(f"  {b} {t:7.1f} s {a} -> {c}")
    # after a fall: how he lies (the attitude, kilo.tricks.righting.lying) against the estimate, while
    # FALLEN or RECOVERING and still; and the time from FELL to the first lying state
    from kilo.tricks import righting as RT
    lie: Counter = Counter()
    settle = []
    for bt in bs:
        est = E.Estimator()
        fell_t = None
        for smp, since in R.samples(bt.stream):
            est.feed(smp)
            if smp.mode == int(P.Mode.FALLEN) and fell_t is None:
                fell_t = smp.t_us
            if fell_t is not None and est.est.state in (E.State.ON_LEFT_SIDE, E.State.ON_RIGHT_SIDE, E.State.ON_BACK,
                                                        E.State.ON_END):
                settle.append((smp.t_us - fell_t) / 1e6)
                fell_t = None
            if smp.mode == int(P.Mode.IDLE) and fell_t is not None and smp.t_us - fell_t > 10e6:
                fell_t = None
            if smp.mode in (int(P.Mode.FALLEN), proto2.MODE_RECOVERING) and est.est.state != E.State.MOVING:
                words = RT.lying(RT.up_of(smp.pitch / A16, smp.roll / A16))
                lie[(words.split(",")[0].replace("leaning on his", "on his"), est.est.state.name)] += 1
    print("\nwhile FALLEN or RECOVERING and still: how he lies (from the attitude) -> the estimate, TELEMETRY:")
    for (w, st), n in sorted(lie.items()):
        print(f"  {w:32s} {st:14s} {n}")
    print(f"from the body's FALLEN to the first lying state: {spread(settle, ' s', '.2f')}")


# ---- 6. rc's get-ups --------------------------------------------------------------------------------------

def getup(bs: list[Boot]) -> None:
    """The falls and rc's tries in the recordings (FELL, RECOVERING, RECOVERED, RECOVERY_FAILED): per try
    the roll rate and how long until still (rates under 20 deg/s for 0.2 s), and how the fall ended."""
    from kilo.tricks import righting as RT
    names = {P.EventCode.FELL: "FELL", proto2.EVENT_RECOVERED: "RECOVERED", P.EventCode.LYING: "FAILED"}
    falls = []
    for bt in bs:
        s = bt.stream
        evs = [(f.t_us, names[f.msg.code]) for f in s.frames if f.direction == "rx" and isinstance(f.msg, P.Event)
               and f.msg.code in names]
        tel = s.telemetry()
        for i, (t, e) in enumerate(evs):
            if e != "FELL":
                continue
            end = next(((t2, e2) for t2, e2 in evs[i + 1:] if e2 != "FELL"), None)
            stop = end[0] if end else t + 15e6
            seg = [(t2, m) for t2, m in tel if t <= t2 <= stop + 1.5e6]
            # tries: contiguous RECOVERING stretches
            tries, cur = [], []
            for t2, m in seg:
                if m.mode == proto2.MODE_RECOVERING:
                    cur.append((t2, m))
                elif cur:
                    tries.append(cur)
                    cur = []
            if cur:
                tries.append(cur)
            for k, tr in enumerate(tries):
                t0 = tr[0][0]
                rr = [abs(m.rate_roll) / A16 for _, m in tr]
                rp = [abs(m.rate_pitch) / A16 for _, m in tr]
                after = [(t2, m) for t2, m in seg if t2 > tr[-1][0]]
                full = tr + after[:40]
                still = None
                for a in range(len(full)):
                    w = [m for t2, m in full[a:] if t2 - full[a][0] <= 0.2e6]
                    if (full[a][0] - t0 > 1.78e6 and len(w) >= 3
                            and all(abs(m.rate_roll) / A16 < 20 and abs(m.rate_pitch) / A16 < 20 for m in w)):
                        still = (full[a][0] - t0) / 1e6
                        break
                fm = tr[0][1]
                lm = (after[10][1] if len(after) > 10 else tr[-1][1])
                falls.append((bt.name, s.uptime_s(t), k + 1, len(tries), RT.lying(RT.up_of(fm.pitch / A16, fm.roll / A16)),
                              max(rr), statistics.mean(rr), max(rp), still,
                              RT.lying(RT.up_of(lm.pitch / A16, lm.roll / A16)), end[1] if end else "?"))
    print("| boot | fell at (s) | try | of | lying at its start | fastest roll (deg/s) | mean roll rate | fastest pitch "
          "| still after (s) | after the try | the fall ended |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for f in falls:
        still = "-" if f[8] is None else f"{f[8]:.2f}"
        print(f"| {f[0]} | {f[1]:.1f} | {f[2]} | {f[3]} | {f[4]} | {f[5]:.0f} | {f[6]:.0f} | {f[7]:.0f} | {still} "
              f"| {f[9]} | {f[10]} |")
    for bt in bs:   # falls without any try (the body gives up at once, e.g. nose or tail down)
        s = bt.stream
        evs = [(f.t_us, names[f.msg.code]) for f in s.frames if f.direction == "rx" and isinstance(f.msg, P.Event)
               and f.msg.code in names]
        for i, (t, e) in enumerate(evs):
            if e == "FELL" and not any(r[0] == bt.name and abs(r[1] - s.uptime_s(t)) < 0.05 for r in falls):
                end = next((e2 for t2, e2 in evs[i + 1:] if e2 != "FELL"), "?")
                m = min(s.telemetry(), key=lambda tm: abs(tm[0] - t - 0.5e6))[1]
                print(f"| {bt.name} | {s.uptime_s(t):.1f} | - | 0 | {RT.lying(RT.up_of(m.pitch / A16, m.roll / A16))} "
                      f"| - | - | - | - | - | {end} |")
    # the roll's bursts during the tries: a stretch with |roll rate| over 60 deg/s, how long it lasts and how
    # far it rolls (the time budget of a closed-loop get-up: how long one action's roll takes)
    bursts = []
    for bt in bs:
        cur = []
        for t, m in bt.stream.telemetry():
            if m.mode == proto2.MODE_RECOVERING and abs(m.rate_roll) / A16 > 60:
                cur.append((t, m))
            elif cur:
                if len(cur) >= 2:
                    dr = (cur[-1][1].roll - cur[0][1].roll) / A16
                    dr = (dr + 180) % 360 - 180
                    bursts.append(((cur[-1][0] - cur[0][0]) / 1e6 + 0.04, abs(dr),
                                   max(abs(x.rate_roll) for _, x in cur) / A16))
                cur = []
    if bursts:
        print(f"\nroll bursts in the tries (|roll rate| > 60 deg/s, TELEMETRY at 25 Hz): {len(bursts)}; lasting "
              f"{spread([b[0] for b in bursts], ' s', '.2f')}; rolling {spread([b[1] for b in bursts], ' deg', '.0f')}")
    fx = RT.fixture()
    rows = []
    for fall in fx["falls"]:
        for tr in fall.get("tries", []):
            trace = tr.get("trace") or []
            moving = [x for x in trace if x[0] >= 0]
            if not moving:
                continue
            rr = [abs(x[4]) for x in moving]
            rows.append((fall["file"], fall.get("floor"), tr["n"], tr.get("start_lying"), tr.get("righted"), max(rr),
                         tr.get("lasted_s")))
    print("\nthe fixture's tries (mind/tests/fixtures/falls_2026_11.json): file, on the floor, try, lying, righted, "
          "fastest roll deg/s, lasted s")
    for r in rows:
        print("  ", r)
    fl = [r for r in rows if r[1]]
    print(f"on the floor: {len(fl)} tries, {sum(1 for r in fl if r[4])} righted; fastest roll {spread([r[5] for r in fl], '', '.0f')}")


# ---- 7. tricks -----------------------------------------------------------------------------------------

def tricks(bs: list[Boot]) -> None:
    """Every trick run: elapsed_ms against expected_ticks, and the measured nose-up against the design's
    prediction (kilo.logs.trickrec; the prediction only where the library still holds the trick played)."""
    from kilo.logs import trickrec as T
    print("| boot | run | trick | ended | elapsed ms | expected ms | late ms | battery mV | error rms (deg) | largest "
          "| note |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    late, errs, sched_off = [], [], []
    for bt in bs:
        for run in bt.stream.runs:
            if run.kind != "trick":
                continue
            tr = T.trick_run(bt.stream, run)
            if not tr.start:
                continue
            a = T.analyze(tr)
            en = a["end"]
            el, ex = en.get("elapsed_ms"), en.get("expected_ticks")
            d = None
            if el and ex and en.get("reason") == "DONE":
                d = int(el) - int(ex) * P.TICK_MS
                late.append(d)
            e = a.get("error") or {}
            if e:
                errs.append(e["rms_deg"])
            # on schedule: when the body says it starts each keyframe (TELEMETRY.trick_frame) against the
            # schedule from the frames MARK (kilo.logs.trickrec.keyframes, the player's own timing)
            kfs = T.keyframes(tr, {})
            if tr.play_us is not None and kfs:
                seen, prev = {}, None
                for t, m in tr.telemetry:
                    if m.limited != 0xFF and m.limited != prev and t >= tr.play_us:
                        seen.setdefault(m.limited, (t - tr.play_us) / 1e6)
                    prev = m.limited
                for k in kfs:
                    if k.index in seen and k.index > 0:
                        sched_off.append((seen[k.index] - k.start_s) * 1000)
            print(f"| {bt.name} | {run.id} | {a['label']} | {en.get('reason', '?')} | {el or '-'} "
                  f"| {int(ex) * P.TICK_MS if ex else '-'} | {'-' if d is None else d} "
                  f"| {a['battery'].get('min_mv')} | {e.get('rms_deg', '-')} | {e.get('max_deg', '-')} "
                  f"| {(a.get('source_note') or a.get('start_note') or '')[:60]} |")
    print(f"\nDONE runs, elapsed - expected: {spread(late, ' ms', '.0f')}")
    print(f"runs with a prediction, error rms: {spread(errs, ' deg', '.2f')}")
    print(f"keyframe starts the body reported minus the schedule (TELEMETRY at 25-50 Hz, so +-20..40 ms): "
          f"{spread(sched_off, ' ms', '.0f')}")


# ---- main ----------------------------------------------------------------------------------------

STUDIES = {"inventory": inventory, "link": link, "margin": margin, "balance": balance, "lag": lag,
           "estimate": estimate, "getup": getup, "tricks": tricks}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("what", choices=tuple(STUDIES) + ("all",))
    ap.add_argument("files", nargs="*", help="recordings (default: ~/kilo-logs, from-server too)")
    a = ap.parse_args(argv)
    bs = boots(a.files or default_files())
    for name, fn in STUDIES.items():
        if a.what in (name, "all"):
            if a.what == "all":
                print(f"\n## {name}\n")
            fn(bs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
