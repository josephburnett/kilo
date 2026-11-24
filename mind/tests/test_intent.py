"""The state estimate and the intent graph (kilo.intent, docs/intent.md); the moves between the
postures are the engine's (mind/tests/test_engine_postures.py). The C++ port is checked against
this reference by kernel/host/tests/test_intent.cpp (intent_vectors.txt)."""

import subprocess
import sys
from pathlib import Path

from kilo.intent import estimate as E
from kilo.intent import graph as G
from kilo.planner.model import BodyModel
from kilo.protocol import kilo_protocol as P

REPO = Path(__file__).resolve().parents[2]
S, C = E.State, E.Cmd
IDLE = int(P.Mode.IDLE)


def a16(deg):
    return E._a16(deg)


# ---- the table ----------------------------------------------------------------------------------
def test_table_is_the_models_and_covers_kilos_recordings():
    rows = {r.state: r for r in E.table()}
    assert set(rows) == {S.STANDING, S.SITTING, S.RESTING}
    # standing level the sensor reads the model's IMU offset (2026-11-22: 2.3 nose down, 0.5 roll), and
    # sitting the model is within 1 deg of what Kilo read (-25.7; the body's own posture.cpp says -30)
    assert (rows[S.STANDING].pitch, rows[S.STANDING].roll) == (a16(BodyModel().imu_pitch_deg),
                                                               a16(BodyModel().imu_roll_deg))
    assert abs(rows[S.SITTING].pitch / 16 + 25.7) < 1.0
    # what Kilo read, still, in each posture he was told to hold (the recordings of 2026-11-17..21:
    # kilo.intent.replay; pitch nose down +, roll): the estimate must name it, and with room to spare
    recorded = {C.STAND: (2.4, 0.2), C.SIT: (-25.7, 2.1), C.REST: (0.7, -1.7)}
    for cmd, (p, r) in recorded.items():
        for dp in (-3.9, 0, 3.9):          # 3 sd of the recordings' spread
            for dr in (-3.9, 0, 3.9):
                ans = E.classify(a16(p + dp), a16(r + dr), cmd, S.UNKNOWN)
                assert ans.state == E.CMD_STATE[cmd], (cmd, dp, dr, ans)
                assert ans.confidence >= 50, (cmd, dp, dr, ans)
    # the tolerance: the model's error against the recordings' means plus three times their spread
    assert rows[S.STANDING].tol_pitch == rows[S.RESTING].tol_roll == a16(E.MODEL_ERROR_DEG + 3 * E.SPREAD_DEG)
    # sitting and standing can never both fit
    assert abs(rows[S.SITTING].pitch - rows[S.STANDING].pitch) > 2 * rows[S.STANDING].tol_pitch


def test_fallen_classes_follow_the_bodys_conventions():
    # roll + is the right side up: he lies on his left (body/src/core/attitude.h)
    assert E.attitude_class(0, a16(90)) == S.ON_LEFT_SIDE
    assert E.attitude_class(0, a16(-90)) == S.ON_RIGHT_SIDE
    assert E.attitude_class(a16(10), a16(61)) == S.ON_LEFT_SIDE
    assert E.attitude_class(a16(10), a16(59)) is None
    # upside down: |roll| near 180, the pitch reflected past 90 (attitude.cpp)
    assert E.attitude_class(a16(175), a16(170)) == S.ON_BACK
    assert E.attitude_class(a16(-178), a16(-175)) == S.ON_BACK
    # on the nose or the tail: the pitch near +-90, whatever the roll reads there
    assert E.attitude_class(a16(80), a16(40)) == S.ON_END
    assert E.attitude_class(a16(-85), a16(-150)) == S.ON_END
    assert E.attitude_class(a16(100), a16(170)) == S.ON_END   # folded: 80 from level
    # the body's fall reflex trips at 75 deg; every attitude it calls a fall is a fallen class here
    for roll in (76, 90, 120, 150, 179, -76, -100, -179):
        assert E.attitude_class(0, a16(roll)) in E.FALLEN
    for pitch in (76, 89, -76, -89):
        assert E.attitude_class(a16(pitch), 0) == S.ON_END


def test_the_posture_asked_for_is_a_prior_never_the_answer():
    # told to stand, on his side
    assert E.classify(0, a16(88), C.STAND, S.STANDING).state == S.ON_LEFT_SIDE
    # told to stand, sitting: SITTING, at half the confidence
    ans = E.classify(a16(-26), 0, C.STAND, S.STANDING)
    assert ans.state == S.SITTING and ans.confidence <= 50
    # level: STANDING and RESTING read alike, the prior decides; with none, the state before
    assert E.classify(a16(2), 0, C.STAND, S.UNKNOWN).state == S.STANDING
    assert E.classify(a16(2), 0, C.REST, S.UNKNOWN).state == S.RESTING
    assert E.classify(a16(2), 0, C.LIMP, S.STANDING).state == S.RESTING
    assert E.classify(a16(2), 0, C.NONE, S.RESTING).state == S.RESTING
    assert E.classify(a16(2), 0, C.NONE, S.SITTING).state == S.UNKNOWN
    # leaning 20 deg nose down, 30 to a side: nothing the model knows
    assert E.classify(a16(20), 0, C.STAND, S.STANDING).state == S.UNKNOWN
    assert E.classify(0, a16(30), C.NONE, S.STANDING).state == S.UNKNOWN


def _feed(est, n, pitch, roll, cmd=C.NONE, rate=0, mode=IDLE, t0=[0]):
    out = []
    for _ in range(n):
        t0[0] += 40_000
        out.append(est.feed(E.Sample(t0[0], a16(pitch), a16(roll), a16(rate), 0, 0, mode, cmd)))
    return out


def test_estimator_settles_confirms_and_logs_each_change_once():
    est = E.Estimator()
    ch = _feed(est, 30, 2.4, 0.2, C.STAND)
    assert est.est.state == S.STANDING and sum(ch) == 2       # UNKNOWN -> MOVING (not yet still) -> STANDING
    # a move: rates
    _feed(est, 10, -12, 0, C.SIT, rate=60)
    assert est.est.state == S.MOVING
    _feed(est, 30, -25.7, 2.1, C.SIT)
    assert est.est.state == S.SITTING and est.est.confidence > 70
    sit = {r.state: r for r in E.table()}[S.SITTING]
    assert abs(est.est.err_pitch / 16 - (-25.7 - sit.pitch / 16)) < 0.2
    # a blip shorter than the confirmation changes nothing
    ch = _feed(est, 5, 2, 0, C.SIT) + _feed(est, 10, -25.7, 2.1, C.SIT)
    assert not any(ch) and est.est.state == S.SITTING
    # walking, a trick: MOVING whatever the attitude
    _feed(est, 20, 0, 0, C.NONE, mode=int(P.Mode.STREAM))
    assert est.est.state == S.MOVING
    # the fall reflex at work, lying still on his side: the side, not MOVING
    _feed(est, 30, 2, -89, C.NONE, mode=int(P.Mode.FALLEN))
    assert est.est.state == S.ON_RIGHT_SIDE
    # limp and level with nothing asked (power-on): RESTING, he cannot be standing
    est2 = E.Estimator()
    _feed(est2, 30, 1, -1, C.NONE, mode=int(P.Mode.LIMP))
    assert est2.est.state == S.RESTING


# ---- the graph ------------------------------------------------------------------------------------
def test_graph_first_steps():
    es = G.edges()
    kind = lambda s, g: es[G.next_edge(s, g, es)].kind if G.next_edge(s, g, es) >= 0 else None  # noqa: E731
    step = lambda s, g: es[G.next_edge(s, g, es)]  # noqa: E731
    # between the three: one step, the engine's move (it goes through another posture itself if need be)
    for a in G.GOALS:
        for b in G.GOALS:
            if a != b:
                assert kind(a, b) == G.Kind.ENGINE and step(a, b).dst == b
    assert not any(e.kind == G.Kind.POSTURE and e.src in G.GOALS for e in es)
    # there already, or moving: no step
    for g in G.GOALS:
        assert G.next_edge(g, g, es) == -1 and G.next_edge(S.MOVING, g, es) == -1
        # fallen: the one placeholder, whatever the goal; unknown: the body's own change
        for s in E.FALLEN:
            assert kind(s, g) == G.Kind.WAIT_REFLEX and step(s, g).dst == S.STANDING
        assert kind(S.UNKNOWN, g) == G.Kind.POSTURE and step(S.UNKNOWN, g).posture == int(G.END[g])
    # every state reaches every goal in at most three steps
    for s in S:
        if s == S.MOVING:
            continue
        for g in G.GOALS:
            cur, n = s, 0
            while cur != g:
                cur = es[G.next_edge(cur, g, es)].dst
                n += 1
                assert n <= 3
    # without the righting experiment the fallen states wait for the body's reflex
    assert G.REFLEX == (G.Kind.WAIT_REFLEX, "", S.STANDING)
    # with it: a side by the side move (mirrored from the left) to RESTING, the back onto the right side,
    # nose or tail down nothing; the rest of the table is the same
    for g in G.GOALS:
        r = lambda s: es[G.next_edge(s, g, es, righting=True)]  # noqa: E731
        assert (r(S.ON_RIGHT_SIDE).kind, r(S.ON_RIGHT_SIDE).move, r(S.ON_RIGHT_SIDE).mirror) == (G.Kind.RIGHT, "righting_side", False)
        assert (r(S.ON_LEFT_SIDE).move, r(S.ON_LEFT_SIDE).mirror, r(S.ON_LEFT_SIDE).dst) == ("righting_side", True, S.RESTING)
        assert (r(S.ON_BACK).move, r(S.ON_BACK).dst) == ("righting_back", S.ON_RIGHT_SIDE)
        assert G.next_edge(S.ON_END, g, es, righting=True) == -1
        for s in S:
            if s not in E.FALLEN:
                assert G.next_edge(s, g, es, righting=True) == G.next_edge(s, g, es)
        for s in (S.ON_RIGHT_SIDE, S.ON_LEFT_SIDE, S.ON_BACK):  # up within four steps
            cur, n = s, 0
            while cur != g:
                cur = es[G.next_edge(cur, g, es, righting=True)].dst
                n += 1
                assert n <= 4


# ---- the generated C++ tables and vectors -------------------------------------------------------------
def test_generated_tables_and_vectors_are_current():
    env = {"PYTHONPATH": str(REPO / "mind"), "PATH": "/usr/bin:/bin"}
    r = subprocess.run([sys.executable, str(REPO / "kernel/mind/intent/gen_intent.py"), "--check"], env=env,
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    r = subprocess.run([sys.executable, str(REPO / "kernel/host/tests/gen_intent_vectors.py")], env=env,
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert r.stdout == (REPO / "kernel/host/tests/intent_vectors.txt").read_text()
