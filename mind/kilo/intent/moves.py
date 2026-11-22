"""The moves between Kilo's resting states, planned and checked from the body model.

    python -m kilo.intent.moves            plan every move, check it, print the report
    python -m kilo.intent.moves --write    ...and write moves/*.json and moves/plan.json
    python -m kilo.intent.moves --check    exit 1 if the committed files are not what this makes

A MOVE takes Kilo from one of the body's postures (STAND, SIT, REST: the
joint angles of body/src/core/posture.cpp, which stay the end points) to
another, as keyframes the body's trick player plays. Today the body gets
from one posture to another by turning every joint at once, straight to
its new angle. Here the model chooses the way:

THE PLAN. The eight leg joints are four groups (front shoulders, front
knees, hind shoulders, hind knees). A candidate moves them in one stage
(all together: the body's own way) or in two (some groups first, then the
rest); the head and the tail go with either stage. For each candidate the
3-D rest finder (kilo.tricks.stance.follow) gives, along the whole path,
how far the centre of mass is inside the feet. The plan is the candidate
whose smallest margin is largest, taken over two models: the body model as
it is (``dynamics.NOMINAL``) and its "tricks" view (``dynamics.FITTED``:
the centre of mass and the legs' give as the rear-up recordings fitted
them); a one-stage move wins a tie (within TIE_MM).

THE CHECK (the standard the rear-up is held to, kilo.tricks.rear_up.check):

1. the plan's smallest margin is at least MARGIN_MM in both models;
2. played in the compliant dynamics (kilo.tricks.dynamics.simulate) on the
   model and on every band of ``dynamics.bands()`` (the fit and each thing
   it is unsure of, to its edge), Kilo does not fall, the mind's roll limit
   (ROLL_MAX) does not stop the play, no leg servo carries more than
   TORQUE_MAX, nothing but feet (and knees, the tail) touches the floor,
   and he ends where the state estimate (kilo.intent.estimate) reads the
   target state.

A move that fails is NOT shipped: its entry in moves/plan.json says why,
and the mind uses the body's own posture change for that step
(kilo.intent.graph).

What this does not do: the end postures are still the body's tables (a
model-made STAND would change how he stands; not in this step), and the
plan only orders the joints: it does not search foot paths.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from kilo.calibration import POSTURES
from kilo.intent import estimate as E
from kilo.tricks import dynamics as D
from kilo.tricks import stance as T
from kilo.tricks.model import JOINT_IDS, JOINT_LIMITS, Keyframe, Posture, Trick

MOVES_DIR = Path(__file__).resolve().parent / "moves"
PLAN_FILE = MOVES_DIR / "plan.json"

MARGIN_MM = 10.0        # the centre of mass at least this far inside the support, along the whole path
TIE_MM = 0.5            # a one-stage move wins against a two-stage one no more than this better
RATE_DPS = 90.0         # the fastest joint of a stage turns at this (the body's safety slew: 500)
STAGE_HOLD = 5          # ticks between two stages
END_HOLD = 10           # ticks at the end, before the body takes the end posture over
ROLL_MAX = 25.0         # deg: the mind stops the move past this (Trick.roll_max)
STOP_TICKS = 40         # a stopped move goes to its end posture over this (0.8 s)
TORQUE_MAX = 2.4        # kg cm on a leg servo (rear_up.TORQUE_MAX: 80 % of the estimated stall)
FOLLOW_STEPS = 12       # samples per stage in the rest finder
ALLOWED = set(T.FEET) | set(T.KNEE.values()) | {T.TAIL}

STATES = (E.State.STANDING, E.State.SITTING, E.State.RESTING)
POSTURE = {E.State.STANDING: "stand", E.State.SITTING: "sit", E.State.RESTING: "rest"}
END = {E.State.STANDING: Posture.STAND, E.State.SITTING: Posture.SIT, E.State.RESTING: Posture.REST}
# the postures' expected attitude as body/src/core/posture.cpp has it (roll, pitch; whole degrees)
EXPECTED = {E.State.STANDING: (0, 0), E.State.SITTING: (0, -30), E.State.RESTING: (0, 0)}
JOINTS = D.JOINTS
GROUPS = {"front shoulders": ("lf_shoulder", "rf_shoulder"), "front knees": ("lf_knee", "rf_knee"),
          "hind shoulders": ("rb_shoulder", "lb_shoulder"), "hind knees": ("rb_knee", "lb_knee")}
HEAD = ("head_pan", "head_tilt", "tail")


def joints(state: E.State) -> dict:
    """The posture's joint angles as the body plays them: inside the joint limits (SIT asks the hind
    shoulders for -90; the body's safety stage stops them at -80, posture.cpp)."""
    out = {}
    for k, v in zip(JOINTS, POSTURES[POSTURE[state]]):
        lo, hi = JOINT_LIMITS[JOINT_IDS[k]]
        out[k] = float(min(hi, max(lo, v)))
    return out


def move_name(a: E.State, b: E.State) -> str:
    return f"{POSTURE[a]}_to_{POSTURE[b]}"


def models() -> list[tuple[str, D.Params]]:
    """The two models a plan's margin is taken over."""
    return [("model", D.NOMINAL), ("tricks view", D.FITTED)]


# ---- the plan ------------------------------------------------------------------------------------
def candidates() -> list[tuple]:
    """Every way to stage the four leg groups: one stage, or a first stage of 1 to 3 groups."""
    names = tuple(GROUPS)
    out = [(names,)]
    for k in (1, 2, 3):
        for first in itertools.combinations(names, k):
            out.append((first, tuple(n for n in names if n not in first)))
    return out


def stages(a: dict, b: dict, order: tuple, head_stage: int = 0) -> list[dict]:
    """The joint targets after each stage of ``order``; the head and tail move in ``head_stage``."""
    cur = dict(a)
    out = []
    for i, groups in enumerate(order):
        for g in groups:
            for j in GROUPS[g]:
                cur[j] = b[j]
        if i == min(head_stage, len(order) - 1):
            for j in HEAD:
                cur[j] = b[j]
        out.append(dict(cur))
    return out


def margin_along(frames: list[dict], start: dict, p: D.Params) -> float:
    return min(s.rest.margin for s in T.follow(p.g3(), frames, start, steps=FOLLOW_STEPS))


@dataclass
class Plan:
    a: E.State
    b: E.State
    order: tuple                  # the stages' groups
    frames: list                  # joint dict per stage
    margins: dict                 # model name -> the smallest margin along the path, mm
    direct_margins: dict          # ...of the one-stage move, for comparison

    @property
    def margin(self) -> float:
        return min(self.margins.values())

    @property
    def name(self) -> str:
        return move_name(self.a, self.b)


def plan(a: E.State, b: E.State) -> Plan:
    ja, jb = joints(a), joints(b)
    scored = []
    for order in candidates():
        fr = stages(ja, jb, order)
        ms = {n: round(margin_along(fr, ja, p), 2) for n, p in models()}
        scored.append((min(ms.values()), order, fr, ms))
    direct = scored[0]
    best = max(scored, key=lambda s: s[0])
    if direct[0] >= best[0] - TIE_MM:
        best = direct
    return Plan(a, b, best[1], best[2], best[3], direct[3])


def _dur(a: dict, b: dict) -> int:
    d = max(abs(a[k] - b[k]) for k in a)
    return max(5, int(math.ceil(d / RATE_DPS * 50)))


def keyframes(pl: Plan) -> list[Keyframe]:
    out = []
    prev = joints(pl.a)
    for i, f in enumerate(pl.frames):
        last = i == len(pl.frames) - 1
        out.append(Keyframe.from_joints(f, dur=_dur(prev, f), hold=END_HOLD if last else STAGE_HOLD))
        prev = f
    return out


def build(pl: Plan, report: "Report | None" = None) -> Trick:
    roll, pitch = EXPECTED[pl.b]
    src = {"kind": "authored", "design": "kilo.intent.moves", "doc": "docs/intent.md",
           "from": POSTURE[pl.a], "to": POSTURE[pl.b], "stages": [list(s) for s in pl.order],
           "margin_mm": pl.margins, "one_stage_margin_mm": pl.direct_margins}
    if report is not None:
        src["dynamics"] = report.summary()
    return Trick(
        name="intent_" + pl.name, frames=tuple(keyframes(pl)), end_posture=END[pl.b], expected_roll=roll,
        expected_pitch=pitch, stop_ticks=STOP_TICKS, roll_max=ROLL_MAX, source=src,
        notes=(f"authored: the move from {POSTURE[pl.a].upper()} to {POSTURE[pl.b].upper()}, planned from the body "
               "model (kilo.intent.moves): the order of the leg joints that keeps the centre of mass furthest "
               "inside the feet",
               "checked in the rest finder and in the compliant dynamics over its uncertainty bands",
               f"the mind stops it past {ROLL_MAX:g} deg of roll; balance off"),
    ).validate()


# ---- the check -----------------------------------------------------------------------------------
@dataclass
class Run:
    band: str
    ok: bool
    problems: list
    end_nose_up: float
    end_roll: float
    end_state: str
    max_roll: float
    torque: float


@dataclass
class Report:
    plan: Plan
    runs: list = field(default_factory=list)
    problems: list = field(default_factory=list)   # the plan's own (the margin)

    @property
    def ok(self) -> bool:
        return not self.problems and all(r.ok for r in self.runs)

    def summary(self) -> dict:
        bad = [r for r in self.runs if not r.ok]
        return {"bands": len(self.runs), "failed": [f"{r.band}: {'; '.join(r.problems)}" for r in bad],
                "end_nose_up_deg": [round(min(r.end_nose_up for r in self.runs), 1),
                                    round(max(r.end_nose_up for r in self.runs), 1)],
                "end_roll_deg": [round(min(r.end_roll for r in self.runs), 1),
                                 round(max(r.end_roll for r in self.runs), 1)],
                "max_roll_deg": round(max(r.max_roll for r in self.runs), 1),
                "max_torque_kg_cm": round(max(r.torque for r in self.runs), 2)}

    def why(self) -> str:
        w = list(self.problems)
        w += [f"{r.band}: {'; '.join(r.problems)}" for r in self.runs if not r.ok]
        return "; ".join(w)


def bands() -> list[tuple[str, D.Params]]:
    """The model, then the tricks' fit and each of its bands."""
    return [("model", D.NOMINAL)] + D.bands()


def simulate(pl: Plan, p: D.Params) -> D.Prediction:
    fr = [(k.joints, k.dur, k.hold) for k in keyframes(pl)]
    return D.simulate(fr, p, start=joints(pl.a), end=joints(pl.b), roll_max=ROLL_MAX)


def judge(pl: Plan, band: str, p: D.Params) -> Run:
    pr = simulate(pl, p)
    n = len(pr.t)
    tail = slice(max(n - 25, 0), n)                 # the last half second
    nose, roll = float(np.mean(pr.nose_up[tail])), float(np.mean(pr.roll[tail]))
    cmd = {E.State.STANDING: E.Cmd.STAND, E.State.SITTING: E.Cmd.SIT, E.State.RESTING: E.Cmd.REST}[pl.b]
    ans = E.classify(E._a16(-nose), E._a16(roll), cmd, pl.a)
    problems = []
    if pr.fell is not None:
        problems.append(f"fell at {pr.fell:.2f} s")
    if pr.stopped is not None:
        problems.append(f"stopped at {pr.stopped[0]:.2f} s: {pr.stopped[1]}")
    if ans.state != pl.b:
        problems.append(f"ends {ans.state.name} (nose up {nose:+.1f}, roll {roll:+.1f}), not {pl.b.name}")
    touched = set().union(*pr.contacts) if pr.contacts else set()
    other = sorted(touched - ALLOWED)
    if other:
        problems.append("touches the floor with " + ", ".join(other))
    torque = max((v[0] for v in pr.torque.values()), default=0.0)
    if torque > TORQUE_MAX:
        problems.append(f"a leg servo carries {torque:.2f} kg cm (limit {TORQUE_MAX:g})")
    return Run(band, not problems, problems, nose, roll, ans.state.name, float(np.max(np.abs(pr.roll))), torque)


def check(pl: Plan, which: list | None = None) -> Report:
    rep = Report(pl)
    for n, m in pl.margins.items():
        if m < MARGIN_MM:
            rep.problems.append(f"the centre of mass comes within {m:.1f} mm of the support's edge in the {n} "
                                f"(needs {MARGIN_MM:g})")
    for band, p in (which if which is not None else bands()):
        rep.runs.append(judge(pl, band, p))
    return rep


# ---- everything, and the files -------------------------------------------------------------------
def pairs() -> list[tuple]:
    return [(a, b) for a in STATES for b in STATES if a != b]


def make(which: list | None = None) -> tuple[dict, dict]:
    """(plan.json's content, {file name: Trick JSON text}) for every pair."""
    entries, files = [], {}
    for a, b in pairs():
        pl = plan(a, b)
        rep = check(pl, which)
        e = {"move": pl.name, "from": a.name, "to": b.name, "shipped": rep.ok,
             "stages": [list(s) for s in pl.order], "margin_mm": pl.margins, "one_stage_margin_mm": pl.direct_margins,
             "dynamics": rep.summary() if rep.runs else {}}
        if rep.ok:
            e["trick"] = "intent_" + pl.name
            files[pl.name + ".json"] = build(pl, rep).to_json()
        else:
            e["why_not"] = rep.why()
        entries.append(e)
    doc = {"made_by": "python -m kilo.intent.moves --write", "margin_min_mm": MARGIN_MM, "roll_max_deg": ROLL_MAX,
           "moves": entries}
    return doc, files


def load_plan() -> dict:
    return json.loads(PLAN_FILE.read_text())


def load_move(name: str) -> Trick:
    """The shipped move ``name`` ("stand_to_sit")."""
    return Trick.from_json((MOVES_DIR / f"{name}.json").read_text())


def shipped() -> list[str]:
    return [e["move"] for e in load_plan()["moves"] if e["shipped"]]


def _plan_text(doc: dict) -> str:
    return json.dumps(doc, indent=1) + "\n"


def report_text(doc: dict) -> str:
    out = []
    for e in doc["moves"]:
        st = " then ".join(" + ".join(s) for s in e["stages"])
        m = ", ".join(f"{k} {v:.1f}" for k, v in e["margin_mm"].items())
        d = ", ".join(f"{k} {v:.1f}" for k, v in e["one_stage_margin_mm"].items())
        out.append(f"{e['move']:14s} {'SHIPPED' if e['shipped'] else 'not shipped'}: {st}")
        out.append(f"{'':14s} margin mm: {m} (all joints at once: {d})")
        dy = e.get("dynamics") or {}
        if dy:
            out.append(f"{'':14s} dynamics, {dy['bands']} bands: ends nose up {dy['end_nose_up_deg'][0]:+.1f}.."
                       f"{dy['end_nose_up_deg'][1]:+.1f}, roll {dy['end_roll_deg'][0]:+.1f}..{dy['end_roll_deg'][1]:+.1f}; "
                       f"largest roll on the way {dy['max_roll_deg']:.1f}, torque {dy['max_torque_kg_cm']:.2f} kg cm; "
                       f"{len(dy['failed'])} failed")
        if not e["shipped"]:
            out.append(f"{'':14s} why not: {e['why_not']}")
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args(argv)
    doc, files = make()
    print(report_text(doc))
    want = dict(files)
    want["plan.json"] = _plan_text(doc)
    if a.write:
        MOVES_DIR.mkdir(exist_ok=True)
        for old in MOVES_DIR.glob("*.json"):
            if old.name not in want:
                old.unlink()
        for n, text in want.items():
            (MOVES_DIR / n).write_text(text)
        print(f"wrote {len(want)} files in {MOVES_DIR}")
        return 0
    if a.check:
        have = {p.name: p.read_text() for p in MOVES_DIR.glob("*.json")} if MOVES_DIR.exists() else {}
        stale = sorted(n for n in set(want) | set(have) if want.get(n) != have.get(n))
        if stale:
            print("stale (run python -m kilo.intent.moves --write): " + ", ".join(stale), file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
