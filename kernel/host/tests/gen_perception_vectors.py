"""Golden vectors for the C++ perception (kernel/mind/perception), from the
Python detectors (mind/kilo/perception) run over the traces of
mind/tests/test_perception.py (the same builders and seeds) and a few more.
kernel/host/tests/test_perception.cpp replays them: the inputs are written
with repr(), so the C++ sees the same doubles, and every event must come out
on the same sample with the same values.

    ~/OpenCat/.venv/bin/python kernel/host/tests/gen_perception_vectors.py > kernel/host/tests/perception_vectors.txt

Lines (space separated; "-" is None):

    U NAME [KEY=VALUE ...]        a new UltrasonicDetector (UltrasonicConfig overrides)
    u T D                         feed(T, D)
    I NAME [KEY=VALUE ...]        a new ImuDetector (ImuConfig overrides)
    r P R                         set_reference(P, R)
    i T P R RATES FLAGS MODE      feed(T, P/16, R/16, RATES "a,b,c" /16, FLAGS (TELEMETRY bits), MODE (code,
                                  -1 = None)): the traces quantised to TELEMETRY's 1/16 degree
    P NAME                        a new Percepts
    p T PITCH ROLL RP RR RY DIST BATT MODE FLAGS   update(Telemetry(...), T) (raw TELEMETRY fields)
    e NAME T V...                 an event the line before produced (its data, rounded as Python rounds it)
    x KEY VALUE                   the state after the scenario (bool 0/1, or a number)
"""

from __future__ import annotations

import math
import random
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "mind"))
sys.path.insert(0, str(REPO / "mind" / "tests"))

import test_perception as tp  # noqa: E402  (the test's trace builders)

from kilo.link import consts  # noqa: E402
from kilo.perception import ImuConfig, ImuDetector, Percepts, UltrasonicConfig, UltrasonicDetector  # noqa: E402
from kilo.protocol.kilo_protocol import Telemetry  # noqa: E402

MODES = {v: k for k, v in consts.MODES.items()}
FLAG_BITS = {name: bit for bit, name in consts.FLAG_BITS.items()}
out = sys.stdout


def r(v) -> str:
    return "-" if v is None else repr(float(v))


def emit(*parts) -> None:
    out.write(" ".join(str(p) for p in parts) + "\n")


def events(evs) -> None:
    for e in evs:
        d = e.data
        if e.name in ("near", "clear"):
            vals = ["no_echo"] if d.get("reason") == "no_echo" else [r(d["distance_mm"])]
        elif e.name == "approaching":
            vals = [r(d["distance_mm"]), r(d["speed_mm_s"])]
        elif e.name == "tilted":
            vals = [d["direction"], r(d["pitch"]), r(d["roll"])]
        elif e.name in ("level", "put_down"):
            vals = [r(d["pitch"]), r(d["roll"])]
        elif e.name == "picked_up":
            vals = [r(d["pitch"]), r(d["roll"]), int(d["servos_powered"])]
        elif e.name == "tap":
            vals = [r(d["onset"]), r(d["peak_deg"]), r(d["duration_s"])]
        elif e.name == "pet":
            vals = [r(d["pp_deg"]), d["strokes"]]
        elif e.name == "startle":
            vals = [r(d["delta_deg"]), r(d["rate_dps"])]
        else:
            raise ValueError(e.name)
        emit("e", e.name, r(e.t), *vals)


def cfg_args(kw) -> list[str]:
    return [f"{k}={v}" for k, v in kw.items()]


# ---- ultrasonic -------------------------------------------------------------------

def us(name, dists, dt=tp.US_DT, **cfg):
    emit("U", name, *cfg_args(cfg))
    det = UltrasonicDetector(UltrasonicConfig(**cfg))
    for i, d in enumerate(dists):
        t = i * dt
        d = None if d is None else float(round(d))  # whole mm, as TELEMETRY.dist_mm
        emit("u", r(t), "-" if d is None else int(d))
        events(det.feed(t, d))
    emit("x", "near", int(det.near))
    emit("x", "clear", int(det.clear))
    emit("x", "approaching", int(det.approaching))
    emit("x", "distance", r(det.distance_mm))


def ultrasonic() -> None:
    noisy = tp.noisy
    for k, base in enumerate((800.0, 300.0, 1500.0)):  # test_quiet_far_and_mid_no_events
        rng = random.Random(1)
        for _ in range(k * 400):  # the test draws all three from one RNG in turn
            rng.uniform(-5, 5)
        us(f"quiet_{int(base)}", [noisy(base, rng) for _ in range(400)])
    rng = random.Random(2)  # test_approach_then_near_then_clear
    seq = [noisy(1200, rng) for _ in range(30)]
    d = 1200.0
    while d > 100:
        d -= 400 * tp.US_DT
        seq.append(noisy(d, rng))
    seq += [noisy(100, rng) for _ in range(20)] + [noisy(800, rng) for _ in range(20)]
    us("approach_near_clear", seq)
    rng = random.Random(3)
    us("slow_drift", [noisy(1000 - 50 * i * tp.US_DT, rng) for i in range(150)])
    rng = random.Random(4)
    us("walk_by", [noisy(2000, rng) for _ in range(30)] + [noisy(500, rng) for _ in range(8)]
       + [noisy(2000, rng) for _ in range(30)])
    rng = random.Random(5)
    seq = []
    for i in range(200):
        seq.append(None if i % 17 == 0 else 60.0 if i % 23 == 0 else noisy(700, rng))
    us("spikes_and_dropouts", seq)
    us("dropouts_while_near", [100.0] * 10 + [None, 100.0, None, None, 100.0] + [None] * 20)
    rng = random.Random(6)
    us("near_hysteresis", [noisy(120, rng) for _ in range(10)] + [noisy(175, rng) for _ in range(100)])
    us("configurable", [1000.0] * 5 + [250.0] * 5, near_mm=300.0, near_exit_mm=350.0)
    seq = []
    for _ in range(3):
        seq += [1500.0] * 10 + [1500 - 300 * i * tp.US_DT for i in range(1, 20)]
    us("approach_cooldown", seq)
    us("zero_and_far_are_dropouts", [500.0] * 10 + [0.0] * 5 + [5000.0] * 20 + [380.0] * 6 + [420.0] * 6)


# ---- IMU ----------------------------------------------------------------------------

def flag_bits(flags) -> str:
    if flags is None:
        return "-"
    return str(sum(1 << FLAG_BITS[f] for f in flags))


def a16(v: float) -> int:
    """TELEMETRY's resolution: the traces are quantised to 1/16 degree as the body sends them."""
    return round(v * 16)


def imu(name, rows, det=None, ref=None, **cfg):
    emit("I", name, *cfg_args(cfg))
    det = det or ImuDetector(ImuConfig(**cfg))
    if ref is not None:
        det.set_reference(*ref)
        emit("r", r(ref[0]), r(ref[1]))
    for t, p, rr, g, flags, mode in rows:
        p16, r16 = a16(p), a16(rr)
        g16 = None if g is None else [a16(x) for x in g]
        rates = "-" if g16 is None else ",".join(str(x) for x in g16)
        emit("i", r(t), p16, r16, rates, flag_bits(flags), MODES[mode] if mode is not None else -1)
        events(det.feed(t, p16 / 16, r16 / 16, None if g16 is None else [x / 16 for x in g16], flags, mode))
    emit("x", "tilted", int(det.tilted))
    emit("x", "picked_up", int(det.picked_up))
    emit("x", "pet", int(det.pet))
    emit("x", "ref_p", r(det.ref[0]))
    emit("x", "ref_r", r(det.ref[1]))


def imus() -> None:
    Trace, ramp, tap_shape = tp.Trace, tp.ramp, tp.tap_shape
    for seed in range(2):
        imu(f"quiet_{seed}", Trace(seed).add(60).rows)
    tr = Trace(1).add(5)
    tr.add(1.5, fp=ramp(-75, 1.0)).add(3, fp=lambda u: -75).add(1.0, fp=lambda u: -75 + 75 * u).add(4)
    tr.add(1.5, fr=ramp(-85, 1.0)).add(3, fr=lambda u: -85).add(1.0, fr=lambda u: -85 + 85 * u).add(4)
    imu("tilt_nose_up_left_up", tr.rows)
    imu("tilt_vs_reference", Trace(2, pitch=-20.0, roll=0.0).add(10).rows, ref=(-20.0, 0.0))
    imu("tilt_vs_learned_reference", Trace(2, pitch=-20.0, roll=0.0).add(10).rows)
    imu("brief_tilt", Trace(3).add(3).add(0.3, fp=lambda u: 30).add(3).rows, pickup_deg=90.0)
    imu("tap", Trace(4).add(3).add(0.2, fp=lambda u: tap_shape(u)).add(2)
        .add(0.2, fr=lambda u: tap_shape(u, peak=-2.5)).add(2).rows)
    imu("pet", Trace(5).add(3).add(6, fp=lambda u: 1.5 * math.sin(2 * math.pi * 1.0 * u)).add(3).rows)

    def wander(u):
        return 8 * math.sin(2 * math.pi * 0.4 * u) + 4 * math.sin(2 * math.pi * 1.1 * u)
    imu("pickup_unpowered", Trace(6).add(3, flags=()).add(4, fp=wander, fr=lambda u: 0.5 * wander(u + 1), flags=())
        .add(4, flags=()).rows)
    imu("startle", Trace(7).add(3).add(0.5, fr=ramp(20, 0.12)).add(3, fr=lambda u: 20 - 20 * min(1, u / 2)).add(2)
        .rows, pickup_deg=90.0)
    rows = [(i * tp.IMU_DT, 0.0, 0.0, (0.0, 0.0, 300.0 if i == 60 else 0.0), {"servos_powered"}, "IDLE")
            for i in range(80)]
    imu("startle_by_gyro", rows)
    imu("self_motion", Trace(8).add(3).add(3, fp=lambda u: 1.5 * math.sin(2 * math.pi * u) + tap_shape(u, 1.0, 4),
                                           flags=("servos_powered", "gait"), mode="GAIT").add(0.5).rows)
    rows = []
    for i in range(40):
        p = -74 if 14 <= i <= 19 else 0.9
        rr = -88 if 24 <= i <= 28 else -2.7
        rows.append((float(i), p, rr, None, None, None))
    imu("low_rate", rows)
    # a pickup with the servos powered and a 50 Hz walk-rate trace with a mode change
    tr = Trace(9).add(3).add(3, fp=wander, fr=lambda u: wander(u + 0.5)).add(5)
    imu("pickup_powered", tr.rows)
    tr = Trace(10).add(2).add(2, mode="TRICK", flags=("servos_powered", "trick")).add(4)
    imu("mode_change_relearns", tr.rows)


# ---- Percepts on raw TELEMETRY -----------------------------------------------------------

def percepts(name, rows, **kw):
    emit("P", name)
    pc = Percepts(**kw)
    for t, tel in rows:
        emit("p", r(t), tel.pitch, tel.roll, tel.rate_pitch, tel.rate_roll, tel.rate_yaw, tel.dist_mm, tel.batt_mv,
             tel.mode, tel.flags)
        events(pc.update(tel, t)["events"])
    d = pc.as_dict()
    for k in ("near", "approaching", "clear", "tilted", "picked_up", "tap", "pet", "startle"):
        emit("x", k, int(bool(d[k])))
    emit("x", "distance", r(d["distance_mm"]))


def percept_runs() -> None:
    rows, d = [], 1000
    for i in range(200):  # test_percepts_from_body_state
        fresh = i % 2 == 0
        if fresh and i > 50:
            d = max(80, d - 30)
        rows.append((i * tp.IMU_DT, Telemetry(tick=i * 2, pitch=14, roll=-43, dist_mm=d, batt_mv=8000, mode=2,
                                              flags=(1 << 2) | ((1 << 11) if fresh else 0))))
    percepts("approach_from_telemetry", rows)
    rows = []  # test_percepts_snapshot_dicts_and_event_hold, as TELEMETRY
    for i in range(120):
        rows.append((i * tp.IMU_DT, Telemetry(tick=i, pitch=0, roll=0, rate_yaw=300 * 16 if i == 60 else 0,
                                              dist_mm=900, mode=2, flags=1 << 2)))
    percepts("startle_hold", rows[:80])
    percepts("startle_hold_expired", rows)
    rows = []  # a tap through Percepts, at 25 Hz, no echo: clear after 1 s
    rng = random.Random(11)
    for i in range(150):
        p = 16 * (0.9 + rng.uniform(-0.3, 0.3) + (3.0 if i == 100 else 1.8 if i in (99, 101) else 0.0))
        rows.append((i * tp.IMU_DT, Telemetry(tick=i, pitch=round(p), roll=-43, dist_mm=0, mode=2,
                                              flags=(1 << 2) | ((1 << 11) if i % 2 else 0))))
    percepts("tap_no_echo", rows)


def main() -> int:
    emit("#", "generated by kernel/host/tests/gen_perception_vectors.py; do not edit")
    ultrasonic()
    imus()
    percept_runs()
    return 0


if __name__ == "__main__":
    sys.exit(main())
