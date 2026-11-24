"""The movement engine's walks, seen from the laptop.

The engine is written once, in C++ (kernel/mind/engine: the step clock and
the solver, docs/movement-engine.md); there is no Python copy of it. The
laptop asks the kernel's own code: ``engine-judge`` (kernel/host, built by
``make -C kernel/host build/engine-judge``, built here when missing) plans a
walk tick by tick with no body and prints every tick. ``plan()`` runs it and
returns the ticks: what the judge (mind/tests/enginejudge.py), the
calibration's table of asked angles (kilo.calibration) and ``kilo model
apply``'s report read.

``pack_pose_feet`` packs a POSE of four feet, as the walker sends them (the
body's tests use it).
"""

from __future__ import annotations

import os
import subprocess
from dataclasses import dataclass

from kilo.protocol import kilo_protocol as P

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HOST = os.path.join(REPO, "kernel", "host")
BIN = os.path.join(HOST, "build", "engine-judge")
HEAD_IDLE = -32768  # a POSE head value: left to IDLE


@dataclass
class Row:
    """One tick of the engine's plan: feet in mm (LF RF RB LB), the attitude it predicts (degrees)."""
    tick: int
    support: int
    lifting: int
    x: list
    z: list
    pitch: float
    roll: float
    plan_pitch: float
    plan_roll: float
    margin: float
    lean_dx: float
    lean_roll: float


_made = False


def binary(path: str | None = None) -> str:
    """The engine-judge to run: ``path``, or this checkout's, made up to date (once a process: make
    rebuilds it when the engine's code or the model's generated header changed)."""
    global _made
    if path is not None:
        return path
    if not _made:
        subprocess.run(["make", "-s", "-C", HOST, f"-j{os.cpu_count() or 2}", "build/engine-judge"], check=True,
                       capture_output=True)
        _made = True
    return BIN


def plan(gait: str, speed: float | None = None, ticks: int = 600, at=(), direction: str = "forward",
         steer: float = 0.0, path: str | None = None, sets=()) -> tuple[list[Row], dict, int | None]:
    """The engine's walk: its ticks, its stats (holds, catches, waits, min_margin) and the tick it was
    done (None: still walking after ``ticks``). ``at``: "TICK:CMD" (speed=V, steer=S, dir=D, gait=G,
    stop, halt); ``sets``: "KEY=VALUE" (engine-judge --set: a study of the solver's numbers)."""
    args = [binary(path), "--gait", gait, "--ticks", str(ticks), "--dir", direction, "--steer", str(steer)]
    if speed is not None:
        args += ["--speed", str(speed)]
    for a in at:
        args += ["--at", a]
    for s in sets:
        args += ["--set", s]
    out = subprocess.run(args, check=True, capture_output=True, text=True).stdout.split("\n")
    rows, stats, done = [], {}, None
    for line in out:
        f = line.split()
        if not f:
            continue
        if f[0] == "stats":
            stats = {f[i]: float(f[i + 1]) for i in range(1, len(f) - 1, 2)}
        elif f[0] == "done":
            done = int(f[1])
        else:
            v = [float(x) for x in f]
            rows.append(Row(int(v[0]), int(v[1]), int(v[2]), [v[3 + 2 * i] / 4 for i in range(4)],
                            [v[4 + 2 * i] / 4 for i in range(4)], v[11], v[12], v[13], v[14], v[15], v[16], v[17]))
    return rows, stats, done


def pack_pose_feet(epoch: int, tick: int, x_mm, z_mm, pitch_a16: int, roll_a16: int, support: int) -> bytes:
    """A POSE key in foot mode: every leg a foot (x, z in mm, per leg LF RF RB LB, sent in quarter
    mm), the head left to IDLE, flags 0."""
    q = [int(round(v * 4)) for pair in zip(x_mm, z_mm) for v in pair]
    return P.Pose._FMT.pack(epoch, tick & 0xFFFF, 0, *q, HEAD_IDLE, HEAD_IDLE, HEAD_IDLE, pitch_a16, roll_a16, support)


def paw_wash(tempo: float = 1.0, strokes: int = 3, path: str | None = None):
    """The engine's paw wash (kernel/mind/engine/paw.h: the left paw lifted on three feet, the face
    washed with it) as a kilo.tricks.model.Trick, and the lean it chose: (trick, {lean_dx, lean_roll,
    margin})."""
    from kilo.tricks.model import Guard, Posture, Trick

    out = subprocess.run([binary(path), "--trick", "paw_wash", "--tempo", str(tempo), "--strokes", str(strokes)],
                         check=True, capture_output=True, text=True).stdout.split("\n")
    lines = [ln.split() for ln in out if ln.strip()]
    guards, info, loop = [], {}, (0, 0, 0)
    for f in lines:
        if f[0] == "trick":
            info = {f[i]: float(f[i + 1]) for i in (1, 3, 5)}
            loop = (int(f[10]), int(f[11]), int(f[8]))
        elif f[0] == "guard":
            guards.append(Guard(int(f[1]), (int(f[2]) / 16, int(f[3]) / 16), int(f[4]) / 16, "three feet"))
    frames = _frames([f for f in lines if f[0] == "frame"])
    t = Trick("paw_wash", tuple(frames), loop_start=loop[0], loop_end=loop[1], repeats=loop[2],
              expected_roll=round(info["lean_roll"]), end_posture=Posture.STAND, stop_ticks=25, guards=tuple(guards),
              roll_max=round(abs(info["lean_roll"]) + 10, 4))
    return t, info


def stretch(base: str = "stand", tempo: float = 1.0, depth: float = 1.0, path: str | None = None):
    """The engine's stretch (kernel/mind/engine/stretch.h: front down, rear up, held, the shake, back) from
    and back to ``base`` (stand or sit) as a kilo.tricks.model.Trick, and what the engine says of it:
    (trick, {margin, pitch, shake_from})."""
    from kilo.tricks.model import Posture, Trick

    out = subprocess.run([binary(path), "--trick", "stretch", "--from", base, "--tempo", str(tempo), "--depth",
                          str(depth)], check=True, capture_output=True, text=True).stdout.split("\n")
    lines = [ln.split() for ln in out if ln.strip()]
    h = lines[0]
    info = {"margin": float(h[2]), "pitch": float(h[4]), "shake_from": int(h[6])}
    frames = _frames([f for f in lines if f[0] == "frame"])
    t = Trick(f"stretch_{base}", tuple(frames), end_posture=Posture(POSTURE_CODE[base]), stop_ticks=25,
              roll_max=15.0)
    return t, info


def _frames(lines):
    """The "frame dur hold flags a0..a10" lines engine-judge prints, as kilo.tricks.model Keyframes."""
    from kilo.tricks.model import CANONICAL, JOINT_SLOTS, Easing, Keyframe

    out = []
    for f in lines:
        dur, hold, flags = int(f[1]), int(f[2]), int(f[3])
        a: list = [None] * JOINT_SLOTS
        for k, j in enumerate(CANONICAL):
            a[j] = int(f[4 + k])
        out.append(Keyframe(tuple(a), dur=dur, hold=hold, easing=Easing(flags & 3), balance=bool(flags & 4)))
    return out


POSTURE_CODE = {"stand": 3, "sit": 2, "rest": 1}


def posture(frm: str, to: str, tempo: float = 1.0, path: str | None = None):
    """The engine's move (kernel/mind/engine/posture.h) from ``frm`` (stand, sit, rest, or eleven joint
    angles in the tricks' canonical order, comma-separated) to ``to`` (stand, sit, rest) as a
    kilo.tricks.model.Trick, and what the engine says of it: (trick, {margin, direct, via, stages,
    plan}); (None, {why}) when it finds no way."""
    from kilo.tricks.model import Posture, Trick

    r = subprocess.run([binary(path), "--posture", frm, to, "--tempo", str(tempo)], capture_output=True, text=True)
    lines = [ln.split() for ln in r.stdout.split("\n") if ln.strip()]
    if r.returncode != 0:
        return None, {"why": r.stdout.strip()}
    h = lines[0]
    info = {"margin": float(h[2]), "direct": float(h[4]), "via": h[6], "stages": int(h[8]),
            "exp_pitch": int(h[10]), "exp_roll": int(h[11]), "plan": " ".join(h[13:])}
    frames = _frames([f for f in lines if f[0] == "frame"])
    t = Trick(f"be_{to}", tuple(frames), end_posture=Posture(POSTURE_CODE[to]), expected_roll=info["exp_roll"],
              expected_pitch=info["exp_pitch"], stop_ticks=40, roll_max=25.0)
    return t, info


def rest(angles, servos: str = "eeprom", path: str | None = None) -> dict:
    """Where the engine's rest finder (kernel/mind/engine/rest.h) puts the body for joint ``angles``
    (eleven, the tricks' canonical order, degrees), with the servos ``asked`` (no pulse clamp),
    ``eeprom`` (Kilo's calibration) or ``reseated`` (the right hind shoulder re-seated): pitch (nose up
    +), roll (left side down +), margin, com_h, topples, telemetry (pitch, roll) and contacts."""
    out = subprocess.run([binary(path), "--rest", servos, *[str(float(a)) for a in angles]], check=True,
                         capture_output=True, text=True).stdout.split()
    v = dict(zip(out[1:12:2], out[2:13:2]))
    i = out.index("contacts")
    return {"pitch": float(v["pitch"]), "roll": float(v["roll"]), "margin": float(v["margin"]),
            "com_h": float(v["com_h"]), "topples": int(v["topples"]),
            "telemetry": (float(out[i - 2]), float(out[i - 1])), "contacts": set(out[i + 1:])}
