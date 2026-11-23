"""A recorded walk replayed through the virtual body (docs/walking-and-balance.md §12.2).

The POSE keys a C++ mind sent (FEET in protocol 2 recordings; a ``.kwalk``,
kilo.logs.walkrec) go tick by tick through the virtual body's POSE player
and physics; the measured attitude
the body reported is compared with the virtual one, the constant pitch
offset removed. Two ways:

- ``corr``: the body's recorded corrections in place of the virtual balance
  loop (the physics alone);
- ``loop``: the virtual balance loop at the recorded gains (physics and loop).

    PYTHONPATH=mind python mind/tests/walkreplay.py FILE.kwalk [--lag MS] [--tau MS]

prints, per mode, the roll and pitch rms of measured - virtual, the plan's
own (measured - predicted) for comparison, and the mean roll residual by
the legs in the air.
"""

from __future__ import annotations

import argparse
import math
from collections import defaultdict

from kilo.logs import proto2
from kilo.logs import walkrec as W
from kilo.planner.model import BodyModel
from kilo.protocol import kilo_protocol as P
from kilo.vbody.pose import Key, PosePlayer

BALANCE = {"kp_q8": 253, "kd_ms": 30, "deadband_deg": 2, "max_corr_deg": 15}  # the gains the recorded walks ran with


def key_at(msg, k: int) -> Key:
    """A recorded POSE (or protocol 2 FEET) of feet as the player's key for unwrapped tick k."""
    x, z = proto2.feet_of(msg)
    return Key(k, 0, int(msg.support), list(x), list(z), pitch=msg.pitch / 16.0, roll=msg.roll / 16.0)


def load(path):
    """(walk, POSE/FEET by unwrapped tick, TELEMETRY by tick, PLAN by tick)."""
    w = W.load(path)
    feet, tel = {}, {}
    last = w.plans[0][1].tick
    for f in w.frames:
        if f.msg is None:
            continue
        if f.direction == "tx" and proto2.is_stream(f.msg):
            feet[W._unwrap(f.msg.tick, last)] = f.msg  # the last for a tick: the body plays it (the quick stop's)
        elif f.direction == "rx" and isinstance(f.msg, P.Telemetry):
            last = W._unwrap(f.msg.tick, last)
            tel[last] = f.msg
    return w, feet, tel, {p.tick: p for _, p in w.plans}


def replay(path, mode="loop", lag_ms=None, tau_ms=None, model=None, skip=25):
    """The comparison as a dict (see the module docstring)."""
    w, feet, tel, plans = load(path)
    model = model or BodyModel()
    k0 = min(feet)
    k1 = max(k for k, t in tel.items() if t.mode == P.Mode.STREAM)
    g = PosePlayer(k0 - 1, model, lag_ms=lag_ms, tau_ms=tau_ms)
    if mode == "corr":
        rec = {"c": [0.0, 0.0]}
        g._balance = lambda *a, **kw: g.corr.__setitem__(slice(None), rec["c"])
    sim = {}
    for k in range(k0, k1 + 1):
        if k in feet:
            g.queue.append(key_at(feet[k], k))
        if mode == "corr" and k in tel:
            rec["c"] = [tel[k].corr_pitch / 16.0, tel[k].corr_roll / 16.0]
        ph = g.physics
        g.tick(k, (ph.pitch, ph.roll), ph.rates, BALANCE, True)
        ph.step()
        sim[k] = (ph.pitch, ph.roll)
    rows = [(k, t.pitch / 16.0, t.roll / 16.0, sim[k][0], sim[k][1], feet[k].pitch / 16.0, feet[k].roll / 16.0,
             plans[k].lifting if k in plans else 0)
            for k, t in sorted(tel.items()) if t.mode == P.Mode.STREAM and k in sim and k in feet][skip:]
    off = sum(r[1] - r[3] for r in rows) / len(rows)

    def rms(v):
        return math.sqrt(sum(x * x for x in v) / len(v))
    by = defaultdict(list)
    for r in rows:
        by[W._legs(r[7])].append(r)
    return {
        "ticks": len(rows), "pitch_offset": off,
        "rms_pitch": rms([r[1] - off - r[3] for r in rows]), "rms_roll": rms([r[2] - r[4] for r in rows]),
        "plan_rms_roll": rms([r[2] - r[6] for r in rows]),
        "by_phase": {k: (len(v), sum(r[2] - r[4] for r in v) / len(v), sum(r[2] - r[6] for r in v) / len(v))
                     for k, v in sorted(by.items())},
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("files", nargs="+")
    ap.add_argument("--lag", type=float, default=None, help="dead time, ms (default: the body model's)")
    ap.add_argument("--tau", type=float, default=None, help="first-order time constant, ms (default: the model's)")
    a = ap.parse_args()
    for f in a.files:
        for mode in ("corr", "loop"):
            r = replay(f, mode, a.lag, a.tau)
            print(f"{f} {mode}: {r['ticks']} ticks, pitch offset {r['pitch_offset']:+.2f}, rms pitch "
                  f"{r['rms_pitch']:.2f} roll {r['rms_roll']:.2f} (the plan's {r['plan_rms_roll']:.2f})")
            for k, (n, sim, plan) in r["by_phase"].items():
                print(f"    {k:8s} {n:5d} ticks  roll residual {sim:+.2f} (the plan's {plan:+.2f})")


if __name__ == "__main__":
    main()
