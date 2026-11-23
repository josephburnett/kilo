"""The walk's floor recordings (docs/walking-and-balance.md §16, docs/walk-trial-results.md): what
the walk does on the floor, folded by its cycle, and the heading the gyro saw.

    PYTHONPATH=mind python mind/tests/walkstudy.py floor      # the recorded walks (~/kilo-logs), folded by cycle
    PYTHONPATH=mind python mind/tests/walkstudy.py drift      # the heading the gyro saw in every straight walk (§16.9)

Both read the recordings of whichever walk they hold. The engine's walks in
the virtual body are mind/tests/enginejudge.py's (the judge).
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import statistics
import sys
from collections import defaultdict

from kilo.protocol import kilo_protocol as P


# ---- the floor --------------------------------------------------------------------------------------

def floor(paths) -> None:
    """The recorded walks (pattern walk, straight forward, after 0.8 s at a speed), folded by the walk's
    cycle (LB lifts after a four-foot tick); duplicates (runs copied into several files) once."""
    from kilo.logs import flight
    from kilo.logs import walkrec as W
    seen, cyc, runs = set(), defaultdict(list), defaultdict(dict)
    for path in paths:
        try:
            s = flight.load(path)
        except Exception:  # noqa: BLE001 - a file that is not a recording
            continue
        for run in s.runs:
            if run.kind != "walk":
                continue
            w = flight.walk_of(s, run)
            res = w.mark("result")
            segs = []
            for m in w.marks:
                if m.word in ("stream", "timing"):
                    segs.append((int(m.fields["tick"]), m.fields.get("pattern"), m.fields.get("dir"),
                                 float(m.fields.get("speed", 0)), m.fields.get("steer"), m.fields.get("pending")))
                elif m.word == "stop":
                    segs.append((int(m.fields["tick"]), None, None, 0.0, None, None))
            if not any(sg[1] == "walk" for sg in segs):
                continue
            rows = W.tick_table(w)
            key = (rows[0]["tick"] if rows else 0, len(rows))
            if key in seen:
                continue
            seen.add(key)
            leads = [r["lead_ms"] for r in rows if r["lead_ms"] is not None]
            q = [r["pose_queued"] for r in rows if r["mode"] == "STREAM" and r["pose_queued"] is not None]
            runs[os.path.basename(path)] = {
                "holds": res.fields.get("holds") if res else "?", "catches": res.fields.get("catches") if res else "?",
                "underruns": (w.mark("end").fields.get("underruns") if w.mark("end") else "?"),
                "lead": (min(leads), max(leads)) if leads else None, "queued": (min(q), max(q)) if q else None}
            by = {r["tick"]: r for r in rows}

            def label(k: int):
                lab = None
                for i, sg in enumerate(segs):
                    end = segs[i + 1][0] if i + 1 < len(segs) else 1 << 30
                    if sg[0] + 40 <= k < end and sg[5] is None:
                        lab = sg
                return lab
            starts = [b["tick"] for a, b in zip(rows, rows[1:]) if b["lifting"] == "LB" and a["lifting"] == "none"]
            for c0, c1 in zip(starts, starts[1:]):
                lab = label(c0)
                if (lab is None or lab[1] != "walk" or lab[2] != "forward" or lab[4] not in (None, "0.00")
                        or label(c1) != lab or c1 - c0 not in (24, 48)):
                    continue
                seg = [by.get(c0 + i) for i in range(c1 - c0)]
                if all(r is not None and r["meas_roll"] is not None for r in seg):
                    cyc[(lab[3], c1 - c0)].append(seg)
    print("runs with the walk:")
    for k, v in runs.items():
        print(f"  {k}: holds {v['holds']} catches {v['catches']} underruns {v['underruns']} "
              f"FEET lead {v['lead']} ms, queued {v['queued']} ticks")
    for (v, period), cs in sorted(cyc.items()):
        n = len(cs[0])
        print(f"\nwalk forward {v:.0f} mm/s, {len(cs)} cycles of {period} ticks (tick 0: LB lifts)")
        print("  legs up   " + " ".join(f"{(r['lifting'] or '-'):>5s}" for r in cs[0]))
        for key in ("meas_roll", "meas_pitch", "rate_roll", "rate_pitch"):
            mean = [statistics.mean(c[i][key] for c in cs) for i in range(n)]
            print(f"  {key:9s} " + " ".join(f"{m:+5.1f}" for m in mean))
        p2r = [max(r["meas_roll"] for r in c) - min(r["meas_roll"] for r in c) for c in cs]
        p2p = [max(r["meas_pitch"] for r in c) - min(r["meas_pitch"] for r in c) for c in cs]
        rr = [max(abs(r["rate_roll"]) for r in c) for c in cs]
        rms = [math.sqrt(sum(r["rate_roll"] ** 2 + r["rate_pitch"] ** 2 for r in c) / n) for c in cs]
        print(f"  per cycle: roll swing {statistics.mean(p2r):.2f}° (sd {statistics.pstdev(p2r):.2f}), pitch swing "
              f"{statistics.mean(p2p):.2f}°, fastest roll {statistics.mean(rr):.0f}°/s, rate rms "
              f"{statistics.mean(rms):.1f}°/s; the plan predicted a flat body (0° of roll)")


# ---- the heading (§16.9) ----------------------------------------------------------------------------

def drift(paths) -> None:
    """The gyro's yaw rate (TELEMETRY rate_yaw, the raw gyro about the body's up axis; + = the nose
    turning left) over every straight stretch of every recorded walk, after its first second, less the
    gyro's resting offset (the median of that file's still ticks)."""
    from kilo.logs import flight
    from kilo.logs import walkrec as W
    seen = set()
    print("| recording | gait | direction | speed | steer | seconds | heading change | °/s |")
    print("|---|---|---|---|---|---|---|---|")
    for path in paths:
        try:
            s = flight.load(path)
        except Exception:  # noqa: BLE001 - not a recording
            continue
        still = [m.rate_yaw / 16 for _, m in s.telemetry()
                 if abs(m.rate_roll) < 8 and abs(m.rate_pitch) < 8 and abs(m.rate_yaw) < 48]
        if len(still) < 20:
            continue
        bias = statistics.median(still)
        for run in s.runs:
            if run.kind != "walk":
                continue
            w = flight.walk_of(s, run)
            segs = [(int(m.fields["tick"]), m.fields.get("pattern"), m.fields.get("dir"), m.fields.get("speed"),
                     m.fields.get("steer", "0"), m.fields.get("pending")) for m in w.marks if m.word in ("stream", "timing")]
            segs += [(int(m.fields["tick"]), None, None, None, None, None) for m in w.marks if m.word == "stop"]
            segs.sort()
            tel, last = {}, None
            for f in w.frames:
                if f.direction == "rx" and isinstance(f.msg, P.Telemetry):
                    last = W._unwrap(f.msg.tick, last if last is not None else f.msg.tick)
                    tel[last] = f.msg
            for i, sg in enumerate(segs):
                if sg[1] is None or sg[5] or sg[2] not in ("forward", "back"):
                    continue
                end = segs[i + 1][0] if i + 1 < len(segs) else max(tel, default=0)
                ks = [k for k in range(sg[0] + 50, end) if k in tel]
                if len(ks) < 50 or (sg[0], len(ks)) in seen:
                    continue
                seen.add((sg[0], len(ks)))
                yaw = sum((tel[k].rate_yaw / 16 - bias) * 0.02 for k in ks)
                T = len(ks) * 0.02
                print(f"| {os.path.basename(path)} | {sg[1]} | {sg[2]} | {float(sg[3]):.1f} | {sg[4]} | {T:.1f} "
                      f"| {yaw:+.1f}° | {yaw / T:+.2f} |")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("what", choices=("floor", "drift"))
    ap.add_argument("files", nargs="*", help="recordings (default: ~/kilo-logs, from-server too)")
    a = ap.parse_args(argv)
    h = os.path.expanduser("~/kilo-logs")
    paths = a.files or sorted(glob.glob(h + "/*.krec") + glob.glob(h + "/from-server/**/*.krec", recursive=True))
    (floor if a.what == "floor" else drift)(paths)
    return 0


if __name__ == "__main__":
    sys.exit(main())
