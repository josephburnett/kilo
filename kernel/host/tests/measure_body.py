#!/usr/bin/env python3
"""The Python virtual body (kilo.vbody.sim) on a pty for the self-measurement
loop in kernel/host/test.sh: its walking physics with the centre of mass
moved and the IMU offset, so the fit (kilo measure fetch) has something to
find; B6 and then B1 pressed on the remote at the times given.

  measure_body.py --dcom -4 -3 0 --imu 1.5 -2 --b6-at 1.5 --b1-at 3.5

Prints "virtual body on /dev/pts/N" and runs until killed. The physics' feet
edge is not inset (kilo.vbody.physics inset_mm: the support polygon as the
feet are): the fit's model has the feet's centres for the edge.
"""
from __future__ import annotations

import argparse
import functools
import sys
import time

from kilo.measure import simulate
from kilo.planner.model import BodyModel
from kilo.vbody import feet as vfeet
from kilo.vbody.physics import Physics
from kilo.vbody.sim import VirtualBody, open_pty


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dcom", type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=("DX", "DY", "DZ"))
    ap.add_argument("--imu", type=float, nargs=2, default=(0.0, 0.0), metavar=("PITCH", "ROLL"))
    ap.add_argument("--lag-ms", type=float, default=50.0)
    ap.add_argument("--tau-ms", type=float, default=70.0)
    ap.add_argument("--b6-at", type=float, default=1.5)
    ap.add_argument("--b1-at", type=float, default=3.5)
    a = ap.parse_args()
    vfeet.FeetPlayer.PHYSICS = functools.partial(Physics, inset_mm=0.0)
    model = simulate.moved_model(BodyModel(), *a.dcom)
    t, path, _slave = open_pty()
    vb = VirtualBody(t, model=model, imu_offset=tuple(a.imu), lag_ms=a.lag_ms, tau_ms=a.tau_ms).start()
    print(f"virtual body on {path} (centre of mass moved {tuple(a.dcom)} mm, IMU off {tuple(a.imu)} deg)", flush=True)
    t0 = time.monotonic()
    pending = sorted([(a.b6_at, 0x02), (a.b1_at, 0x4A)])
    try:
        while True:
            now = time.monotonic() - t0
            while pending and now >= pending[0][0]:
                _, key = pending.pop(0)
                vb.ir_key(key)
                print(f"[{now:6.2f}s] IR key 0x{key:02X}", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        vb.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
