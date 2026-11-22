"""The state estimate, the moves planned from the model and the intent graph (kilo.intent,
docs/intent.md). The C++ port is checked against this reference by kernel/host/tests/test_intent.cpp
(intent_vectors.txt)."""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from kilo.intent import estimate as E
from kilo.intent import graph as G
from kilo.intent import moves as M
from kilo.protocol import kilo_protocol as P
from kilo.tricks import dynamics as D

REPO = Path(__file__).resolve().parents[2]
S, C = E.State, E.Cmd
IDLE = int(P.Mode.IDLE)


def a16(deg):
    return E._a16(deg)


# ---- the table ----------------------------------------------------------------------------------
def test_table_is_the_models_and_covers_kilos_recordings():
    rows = {r.state: r for r in E.table()}
    assert set(rows) == {S.STANDING, S.SITTING, S.RESTING}
    assert rows[S.STANDING].pitch == 0 and rows[S.STANDING].roll == 0
    # SIT: the body's own expected pitch is -30 (posture.cpp); the model's is within 3 deg of it
    assert abs(rows[S.SITTING].pitch / 16 + 30) < 3
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
    assert abs(est.est.err_pitch / 16 - (-25.7 + 28.4)) < 0.2
    # a blip shorter than the confirmation changes nothing
    ch = _feed(est, 5, 2, 0, C.SIT) + _feed(est, 10, -25.7, 2.1, C.SIT)
    assert not any(ch) and est.est.state == S.SITTING
    # walking, a trick: MOVING whatever the attitude
    _feed(est, 20, 0, 0, C.NONE, mode=int(P.Mode.GAIT))
    assert est.est.state == S.MOVING
    # the fall reflex at work, lying still on his side: the side, not MOVING
    _feed(est, 30, 2, -89, C.NONE, mode=int(P.Mode.FALLEN))
    assert est.est.state == S.ON_RIGHT_SIDE
    _feed(est, 30, 2, -89, C.NONE, mode=int(P.Mode.RECOVERING))
    assert est.est.state == S.ON_RIGHT_SIDE
    # limp and level with nothing asked (power-on): RESTING, he cannot be standing
    est2 = E.Estimator()
    _feed(est2, 30, 1, -1, C.NONE, mode=int(P.Mode.LIMP))
    assert est2.est.state == S.RESTING


# ---- the moves --------------------------------------------------------------------------------------
def test_committed_moves_are_what_the_plan_makes():
    """The plan (the staging and its margins: the fast part) and every shipped move's keyframes,
    recomputed, are the committed files'. (`python -m kilo.intent.moves --check` recomputes the
    dynamics too: test_every_shipped_move_in_the_dynamics does it on the model and the fit.)"""
    doc = M.load_plan()
    assert doc["margin_min_mm"] == M.MARGIN_MM
    by_name = {e["move"]: e for e in doc["moves"]}
    assert set(by_name) == {M.move_name(a, b) for a, b in M.pairs()}
    for a, b in M.pairs():
        pl = M.plan(a, b)
        e = by_name[pl.name]
        assert e["stages"] == [list(s) for s in pl.order], pl.name
        assert e["margin_mm"] == pl.margins and e["one_stage_margin_mm"] == pl.direct_margins, pl.name
        margin_ok = pl.margin >= M.MARGIN_MM
        assert e["shipped"] == (margin_ok and not e["dynamics"]["failed"]), pl.name
        if e["shipped"]:
            t = M.load_move(pl.name)
            built = M.build(pl)
            assert t.frames == built.frames and t.end_posture == built.end_posture, pl.name
            assert t.roll_max == M.ROLL_MAX and t.stop_ticks == M.STOP_TICKS and not t.problems()
            # the last keyframe is the body's own posture, inside the joint limits
            assert t.frames[-1].joints == {k: v for k, v in M.joints(b).items()}
        else:
            assert not (M.MOVES_DIR / f"{pl.name}.json").exists() and e["why_not"]


def test_the_plan_orders_the_legs_where_that_keeps_the_mass_further_inside():
    doc = {e["move"]: e for e in M.load_plan()["moves"]}
    # stand <-> sit: all joints at once is already 27 mm inside: nothing to order
    assert len(doc["stand_to_sit"]["stages"]) == 1 and min(doc["stand_to_sit"]["margin_mm"].values()) > 25
    # rest -> stand: all at once comes within 7 mm; the hind legs first keeps 10
    e = doc["rest_to_stand"]
    assert e["stages"][0] == ["hind shoulders", "hind knees"] and len(e["stages"]) == 2
    assert min(e["one_stage_margin_mm"].values()) < M.MARGIN_MM <= min(e["margin_mm"].values())
    # rest <-> sit: no staging keeps 10 mm: not shipped, the graph goes through STANDING
    assert not doc["rest_to_sit"]["shipped"] and not doc["sit_to_rest"]["shipped"]
    assert "comes within" in doc["rest_to_sit"]["why_not"]


@pytest.mark.parametrize("name", M.shipped())
def test_every_shipped_move_in_the_dynamics(name):
    """Played in the compliant dynamics on the model and on the tricks' fit, each shipped move ends
    where the state estimate reads its target, without a fall, a stop or an overloaded servo."""
    e = {x["move"]: x for x in M.load_plan()["moves"]}[name]
    pl = M.plan(S[e["from"]], S[e["to"]])
    for band, p in (("model", D.NOMINAL), ("fitted", D.FITTED)):
        run = M.judge(pl, band, p)
        assert run.ok, (name, band, run.problems)
        assert run.end_state == e["to"]
        assert run.max_roll < M.ROLL_MAX - 10


def test_a_bad_move_is_not_shipped():
    """The check refuses a plan whose margin is short, and a move that ends somewhere else."""
    pl = M.plan(S.RESTING, S.SITTING)
    rep = M.check(pl, which=[])
    assert not rep.ok and "comes within" in rep.why()
    # a "stand to sit" that stops half way does not end SITTING
    good = M.plan(S.STANDING, S.SITTING)
    half = {k: (M.joints(S.STANDING)[k] + v) / 2 for k, v in good.frames[-1].items()}
    bad = M.Plan(good.a, good.b, good.order, [half], good.margins, good.direct_margins)
    run = M.judge(bad, "model", D.NOMINAL)
    assert not run.ok and "not SITTING" in "; ".join(run.problems)


# ---- the graph ------------------------------------------------------------------------------------
def test_graph_first_steps():
    es = G.edges()
    kind = lambda s, g: es[G.next_edge(s, g, es)].kind if G.next_edge(s, g, es) >= 0 else None  # noqa: E731
    step = lambda s, g: es[G.next_edge(s, g, es)]  # noqa: E731
    assert step(S.STANDING, S.SITTING).move == "stand_to_sit"
    assert step(S.RESTING, S.STANDING).move == "rest_to_stand"
    # rest -> sit has no checked move: through STANDING by two that are, not the body's own change
    assert step(S.RESTING, S.SITTING).move == "rest_to_stand" and step(S.SITTING, S.RESTING).move == "sit_to_stand"
    assert not any(e.kind == G.Kind.POSTURE and e.src in G.GOALS for e in es)
    # there already, or moving: no step
    for g in G.GOALS:
        assert G.next_edge(g, g, es) == -1 and G.next_edge(S.MOVING, g, es) == -1
        # fallen: the one placeholder, whatever the goal; unknown: the body's own change
        for s in E.FALLEN:
            assert kind(s, g) == G.Kind.WAIT_REFLEX and step(s, g).dst == S.STANDING
        assert kind(S.UNKNOWN, g) == G.Kind.POSTURE and step(S.UNKNOWN, g).posture == int(M.END[g])
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
    # the righting move's slot is one entry
    assert G.RIGHTING == (G.Kind.WAIT_REFLEX, "", S.STANDING)


def test_without_shipped_moves_the_bodys_own_change_is_the_fallback():
    plan = json.loads(json.dumps(M.load_plan()))
    for e in plan["moves"]:
        e["shipped"] = e["move"] == "stand_to_sit"
    es = G.edges(plan)
    assert es[G.next_edge(S.STANDING, S.SITTING, es)].kind == G.Kind.MOVE
    back = es[G.next_edge(S.SITTING, S.STANDING, es)]
    assert back.kind == G.Kind.POSTURE and back.posture == int(P.Posture.STAND)
    assert es[G.next_edge(S.RESTING, S.SITTING, es)].kind == G.Kind.POSTURE


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


@pytest.mark.skipif(not os.environ.get("KILO_INTENT_FULL"), reason="two minutes: set KILO_INTENT_FULL=1")
def test_moves_check_over_every_band():
    """`python -m kilo.intent.moves --check`: the whole check (21 bands of the dynamics per move, a
    minute or two) makes exactly the committed files. Run it after changing the model or the moves."""
    r = subprocess.run([sys.executable, "-m", "kilo.intent.moves", "--check"], cwd=REPO / "mind",
                       env={"PYTHONPATH": str(REPO / "mind"), "PATH": "/usr/bin:/bin"}, capture_output=True, text=True)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
