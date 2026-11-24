"""Which step leads from the state Kilo is in to the one he is asked for.

The nodes are the states of kilo.intent.estimate, the goals STANDING,
SITTING and RESTING. An EDGE is one step:

  ENGINE       between standing, sitting and resting: the move the movement
               engine works out from the body model when it is asked
               (kernel/mind/engine/posture.h, docs/movement-engine.md step 3:
               from the posture he is in, staged so that his weight stays
               furthest inside what carries him, through another posture
               when that keeps more, at the tempo asked), played by the trick
               player. If the engine finds no way, the mind uses the body's
               own posture change instead and says so;
  POSTURE      the body's own posture change (SET_POSTURE): every joint
               straight to the posture's angle. From UNKNOWN (settled,
               upright enough, but in no pose the model knows);
  WAIT_REFLEX  from a fallen state: wait for the body's fall reflex (it lays
               him down, EVENT LYING: given up, he needs a hand; the body has
               no get-up of its own) or for a hand to right him. What the mind
               does unless it is built with the righting experiment;
  RIGHT        from a side or his back, with the righting experiment
               (KILO_RIGHTING=1, docs/righting.md): a righting move
               (kilo.intent.righting, RIGHTING below), played by the trick
               player as a FALL_OK trick, mirrored from his left side.

``next_edge(state, goal, righting=...)`` is the first edge of the shortest
chain (fewest edges; ENGINE before POSTURE among equals; the table's order
after that), without the RIGHT edges or, with ``righting``, without the
WAIT_REFLEX ones. kernel/mind/intent/gen_intent.py writes the edges and both
next-step tables into the mind (intent.gen.cpp), so the C++ only looks them
up.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

from kilo.intent import estimate as E
from kilo.tricks.model import Posture

State = E.State
GOALS = (State.STANDING, State.SITTING, State.RESTING)
END = {State.STANDING: Posture.STAND, State.SITTING: Posture.SIT, State.RESTING: Posture.REST}


class Kind(IntEnum):
    NONE = 0
    ENGINE = 1
    POSTURE = 2
    WAIT_REFLEX = 3
    RIGHT = 4


@dataclass(frozen=True)
class Edge:
    src: State
    dst: State
    kind: Kind
    move: str = ""          # RIGHT: the move's name ("righting_side")
    posture: int = 0        # POSTURE: the SET_POSTURE code
    mirror: bool = False    # RIGHT: played mirrored (left and right swapped)


# Without the righting experiment a fallen state waits for the body's fall reflex, which ends standing.
REFLEX = (Kind.WAIT_REFLEX, "", State.STANDING)
# With it (kilo.intent.righting): from where, which move, mirrored, and where it means to leave him.
# A side ends on his belly (RESTING; the engine's move takes it on); his back ends on his right side
# (the side move follows). Nose or tail down: nothing (the intent gives up).
RIGHTING = ((State.ON_RIGHT_SIDE, "righting_side", False, State.RESTING),
            (State.ON_LEFT_SIDE, "righting_side", True, State.RESTING),
            (State.ON_BACK, "righting_back", False, State.ON_RIGHT_SIDE))


def edges() -> list[Edge]:
    out = [Edge(a, b, Kind.ENGINE) for a in GOALS for b in GOALS if a != b]
    for s in E.FALLEN:
        out.append(Edge(s, REFLEX[2], REFLEX[0], move=REFLEX[1]))
    for s, name, mirror, to in RIGHTING:
        out.append(Edge(s, to, Kind.RIGHT, move=name, mirror=mirror))
    for b in GOALS:
        out.append(Edge(State.UNKNOWN, b, Kind.POSTURE, posture=int(END[b])))
    return out


def next_edge(state: State, goal: State, es: list[Edge] | None = None, righting: bool = False) -> int:
    """The index (in edges()) of the first step from ``state`` to ``goal``; -1: none (already
    there, MOVING, or no way). ``righting``: the RIGHT edges instead of the WAIT_REFLEX ones."""
    es = es if es is not None else edges()
    if state == goal or state == State.MOVING:
        return -1
    skip = Kind.WAIT_REFLEX if righting else Kind.RIGHT
    # breadth first over the edges, each state's edges in (ENGINE, POSTURE, WAIT_REFLEX, RIGHT), then table order
    order = sorted((i for i in range(len(es)) if es[i].kind != skip), key=lambda i: (int(es[i].kind), i))
    first = {state: -1}
    queue = [state]
    while queue:
        s = queue.pop(0)
        for i in order:
            e = es[i]
            if e.src != s or e.dst in first:
                continue
            first[e.dst] = i if s == state else first[s]
            if e.dst == goal:
                return first[e.dst]
            queue.append(e.dst)
    return -1


def next_table(es: list[Edge] | None = None, righting: bool = False) -> list[list[int]]:
    """next_edge for every state (rows, E.State order) and goal (columns, GOALS order)."""
    es = es if es is not None else edges()
    return [[next_edge(s, g, es, righting) for g in GOALS] for s in State]


def posture_name(code: int) -> str:
    return Posture(code).name.lower()


def describe(es: list[Edge] | None = None) -> str:
    es = es if es is not None else edges()
    out = []
    for i, e in enumerate(es):
        what = {Kind.ENGINE: "the engine's move (made from the model when asked)",
                Kind.POSTURE: f"the body's own change to {posture_name(e.posture)}",
                Kind.WAIT_REFLEX: "wait for the body's fall reflex",
                Kind.RIGHT: f"righting move {e.move}{' mirrored' if e.mirror else ''} (the experiment)"}[e.kind]
        out.append(f"{i:2d} {e.src.name:14s} -> {e.dst.name:13s} {what}")
    for righting in (False, True):
        t = next_table(es, righting)
        out.append(f"first step{' with the righting experiment' if righting else ''}, by state and goal ("
                   + ", ".join(g.name for g in GOALS) + "):")
        for s in State:
            out.append(f"   {s.name:14s} " + " ".join(f"{v:3d}" for v in t[int(s)]))
    return "\n".join(out)


if __name__ == "__main__":
    print(describe())
