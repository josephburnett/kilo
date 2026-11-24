#!/usr/bin/env python3
"""Golden vectors for the C++ state estimate and intent tables (kernel/mind/intent/), from the
Python reference (mind/kilo/intent), on stdout:

    .venv/bin/python kernel/host/tests/gen_intent_vectors.py > kernel/host/tests/intent_vectors.txt

kernel/host/tests/test_intent.cpp reads them (kernel/host/test.sh checks the committed file is
current). Lines:

    row I STATE PITCH ROLL TOL_PITCH TOL_ROLL       the estimate's table (a16)
    c PITCH ROLL CMD PREV -> STATE CONF ERRP ERRR   classify() for one attitude, prior and state before
    seq NAME                                        a fresh estimator; then for every sample
    s T_US PITCH ROLL RP RR RY MODE CMD -> CHANGED STATE CONF SINCE_US ERRP ERRR
    edge I FROM TO KIND POSTURE MOVE MIRROR         the edges ("-" for no move)
    next STATE I0 I1 I2                             the first step to STANDING, SITTING, RESTING
    nextr STATE I0 I1 I2                            ...with the righting experiment
    move NAME CRC16 TRICK_ID TOTAL_TICKS            each righting move, as the trick player plays it
                                                    (standing, sitting, resting: the engine's, made when asked)

States, priors and kinds are their numbers (estimate.py State and Cmd, graph.py Kind).
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "mind"))

from kilo.tricks.model import trick_id_for  # noqa: E402
from kilo.intent import estimate as E  # noqa: E402
from kilo.intent import graph as G  # noqa: E402
from kilo.intent import righting as RT  # noqa: E402
from kilo.protocol import kilo_protocol as P  # noqa: E402


class Lcg:
    """A fixed pseudo-random stream (no Python version in it)."""

    def __init__(self, seed: int):
        self.x = seed & 0xFFFFFFFF

    def next(self) -> int:
        self.x = (self.x * 1664525 + 1013904223) & 0xFFFFFFFF
        return self.x >> 8

    def between(self, lo: int, hi: int) -> int:
        return lo + self.next() % (hi - lo + 1)


def classify_lines() -> list[str]:
    out = []
    angles = [-2880, -2100, -1930, -1920, -1910, -1500, -1440, -961, -960, -959, -645, -644, -600, -470, -454, -320,
              -264, -263, -200, -191, -190, -189, -150,
              -143, -137, -136, -40, 0, 38, 47, 48, 136, 137, 150, 151, 189, 190, 191, 300, 590, 959, 960, 961, 1440,
              1919, 1920, 1921, 2100,
              2880]
    r = Lcg(7)
    cases = [(p, ro) for p in angles for ro in (-2100, -1921, -1920, -1440, -961, -960, -334, -333, -300, -191, -190,
                                                   -150, -143, 0, 47, 48, 150, 151, 190, 191,
                                                   959, 960, 1440, 1920, 1921, 2100)]
    for _ in range(400):
        cases.append((r.between(-2880, 2880), r.between(-2880, 2880)))
    for _ in range(400):   # near the upright rows
        cases.append((r.between(-700, 200), r.between(-320, 200)))
    for i, (p, ro) in enumerate(cases):
        for cmd in E.Cmd:
            prev = E.State((i + int(cmd)) % len(E.State))
            a = E.classify(p, ro, cmd, prev)
            out.append(f"c {p} {ro} {int(cmd)} {int(prev)} -> {int(a.state)} {a.confidence} {a.err_pitch} {a.err_roll}")
    return out


IDLE, GAIT, TRICK, FALLEN, RECOVERING, LIMP = (int(P.Mode.IDLE), int(P.Mode.STREAM), int(P.Mode.STREAM),
                                               int(P.Mode.FALLEN), int(P.Mode.FALLEN), int(P.Mode.LIMP))  # protocol 3: a walk and a trick are both a POSE stream; no RECOVERING


def sequences() -> list[tuple[str, list[E.Sample]]]:
    """Scripted lives: each a list of samples 40 ms apart (TELEMETRY's 25 a second)."""
    seqs = []

    def run(name, parts, seed):
        r = Lcg(seed)
        t = 1_000_000
        out = []
        for n, pitch, roll, rate, mode, cmd in parts:
            for _ in range(n):
                jp, jr = r.between(-3, 3), r.between(-3, 3)
                rates = [r.between(-rate, rate) for _ in range(3)]
                out.append(E.Sample(t, pitch + jp, roll + jr, rates[0], rates[1], rates[2], mode, cmd))
                t += 40_000
        seqs.append((name, out))

    C, S = E.Cmd, 40   # S: a quiet body's rate noise, a16/s
    run("stand sit stand rest", [
        (30, 38, 3, S, IDLE, C.NONE), (30, 38, 3, S, IDLE, C.STAND), (12, -200, 10, 900, IDLE, C.SIT),
        (40, -412, 30, S, IDLE, C.SIT), (10, -100, 0, 800, IDLE, C.STAND), (30, 38, 3, S, IDLE, C.STAND),
        (15, 30, -60, 700, IDLE, C.REST), (40, 12, -27, S, IDLE, C.REST), (30, 12, -27, S, LIMP, C.LIMP)], 1)
    run("a walk and a trick", [
        (25, 38, 3, S, IDLE, C.STAND), (60, 40, 20, 600, GAIT, C.NONE), (20, 35, 6, S, GAIT, C.NONE),
        (25, 35, 6, S, IDLE, C.STAND), (50, -300, 0, 500, TRICK, C.NONE), (30, -410, 30, S, IDLE, C.SIT)], 2)
    run("falls on his left side, the reflex rights him", [
        (25, 38, 3, S, IDLE, C.STAND), (6, 100, 700, 3000, IDLE, C.STAND), (30, 60, 1420, S, FALLEN, C.NONE),
        (40, 0, 400, 2000, RECOVERING, C.NONE), (30, 38, 3, S, IDLE, C.STAND)], 3)
    run("on his right side, on his back, on his nose", [
        (25, 30, -1440, S, IDLE, C.STAND), (5, 0, -1900, 2500, IDLE, C.STAND), (25, 2800, -2700, S, IDLE, C.STAND),
        (5, 900, 0, 2500, IDLE, C.NONE), (25, 1300, 100, S, IDLE, C.NONE), (25, -1350, -60, S, IDLE, C.REST)], 4)
    run("told to stand but he sits; nothing asked; leaning", [
        (30, -430, 20, S, IDLE, C.STAND), (30, 38, 3, S, IDLE, C.NONE), (30, 20, -100, S, IDLE, C.NONE),
        (30, 500, 30, S, IDLE, C.STAND), (30, 38, 700, S, IDLE, C.NONE), (4, 38, 3, 500, IDLE, C.STAND),
        (3, 38, 3, S, IDLE, C.STAND), (4, 38, 3, 500, IDLE, C.STAND), (30, 38, 3, S, IDLE, C.STAND)], 5)
    run("limp at power-on, nothing asked; then on his side under the fall reflex", [
        (30, 20, -30, S, LIMP, C.NONE), (30, 20, -30, S, IDLE, C.NONE), (30, 20, -30, S, IDLE, C.STAND),
        (20, 60, 1400, S, FALLEN, C.NONE), (10, 60, 1400, 1500, RECOVERING, C.NONE),
        (20, 60, 1400, S, RECOVERING, C.NONE), (20, 38, 3, S, IDLE, C.STAND)], 8)
    run("a flicker shorter than the confirmation", [
        (30, 38, 3, S, IDLE, C.STAND), (5, -430, 0, S, IDLE, C.STAND), (20, 38, 3, S, IDLE, C.STAND),
        (8, -430, 0, S, IDLE, C.STAND), (20, 38, 3, S, IDLE, C.STAND)], 6)
    return seqs


def main() -> int:
    out = ["# kernel/host/tests/gen_intent_vectors.py: the state estimate and the intent tables (mind/kilo/intent)"]
    for i, r in enumerate(E.table()):
        out.append(f"row {i} {int(r.state)} {r.pitch} {r.roll} {r.tol_pitch} {r.tol_roll}")
    out += classify_lines()
    for name, samples in sequences():
        out.append(f"seq {name}")
        est = E.Estimator()
        for s in samples:
            ch = est.feed(s)
            e = est.est
            out.append(f"s {s.t_us} {s.pitch} {s.roll} {s.rate_pitch} {s.rate_roll} {s.rate_yaw} {s.mode} {int(s.cmd)} "
                       f"-> {int(ch)} {int(e.state)} {e.confidence} {e.since_us} {e.err_pitch} {e.err_roll}")
    edges = G.edges()
    for i, e in enumerate(edges):
        out.append(f"edge {i} {int(e.src)} {int(e.dst)} {int(e.kind)} {e.posture} {'intent_' + e.move if e.move else '-'} "
                   f"{int(e.mirror)}")
    for s, row in zip(E.State, G.next_table(edges)):
        out.append(f"next {int(s)} " + " ".join(str(v) for v in row))
    for s, row in zip(E.State, G.next_table(edges, righting=True)):
        out.append(f"nextr {int(s)} " + " ".join(str(v) for v in row))
    for name in RT.NAMES:
        t = RT.load_move(name)
        out.append(f"move {t.name} {t.crc16()} {trick_id_for(t)} {t.duration_ticks or 0}")
    sys.stdout.write("\n".join(out) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
