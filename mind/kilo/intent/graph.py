"""Which step leads from the state Kilo is in to the one he is asked for.

The nodes are the states of kilo.intent.estimate, the goals STANDING,
SITTING and RESTING. An EDGE is one step:

  MOVE         a move planned and checked from the model (kilo.intent.moves,
               moves/plan.json "shipped"), played by the trick player;
  POSTURE      the body's own posture change (SET_POSTURE): every joint
               straight to the posture's angle. Used only where no chain of
               shipped moves leads to the goal, and from UNKNOWN (settled,
               upright enough, but in no pose the model knows);
  WAIT_REFLEX  from a fallen state: wait for the body's fall reflex to
               right him (EVENT RECOVERED: he stands). A PLACEHOLDER: the
               righting move of step 3b replaces it by changing RIGHTING
               below, the one entry all four fallen states use.

``next_edge(state, goal)`` is the first edge of the shortest chain (fewest
edges; MOVE before POSTURE among equals; the table's order after that).
kernel/mind/intent/gen_intent.py writes the edges and this next-step table
into the mind (intent.gen.cpp), so the C++ only looks it up.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

from kilo.intent import estimate as E
from kilo.intent import moves as M
from kilo.tricks.model import Posture

State = E.State
GOALS = (State.STANDING, State.SITTING, State.RESTING)


class Kind(IntEnum):
    NONE = 0
    MOVE = 1
    POSTURE = 2
    WAIT_REFLEX = 3


@dataclass(frozen=True)
class Edge:
    src: State
    dst: State
    kind: Kind
    move: str = ""          # MOVE: the shipped move's name ("stand_to_sit")
    posture: int = 0        # POSTURE: the SET_POSTURE code


# What gets Kilo up from a fallen state, and where it leaves him. Step 3b's righting move goes here:
# (Kind.MOVE, "the move's name", State.<where it ends>).
RIGHTING = (Kind.WAIT_REFLEX, "", State.STANDING)


def edges(plan: dict | None = None) -> list[Edge]:
    plan = plan if plan is not None else M.load_plan()
    shipped = {(e["from"], e["to"]): e["move"] for e in plan["moves"] if e["shipped"]}
    out = [Edge(State[a], State[b], Kind.MOVE, move=name) for (a, b), name in shipped.items()]
    # the body's own posture change where no chain of shipped moves leads
    reach = {s: {s} for s in GOALS}
    grew = True
    while grew:
        grew = False
        for e in out:
            for s in GOALS:
                if e.src in reach[s] and e.dst not in reach[s]:
                    reach[s].add(e.dst)
                    grew = True
    for a in GOALS:
        for b in GOALS:
            if b not in reach[a]:
                out.append(Edge(a, b, Kind.POSTURE, posture=int(M.END[b])))
    for s in E.FALLEN:
        out.append(Edge(s, RIGHTING[2], RIGHTING[0], move=RIGHTING[1]))
    for b in GOALS:
        out.append(Edge(State.UNKNOWN, b, Kind.POSTURE, posture=int(M.END[b])))
    return out


def next_edge(state: State, goal: State, es: list[Edge] | None = None) -> int:
    """The index (in edges()) of the first step from ``state`` to ``goal``; -1: none (already
    there, MOVING, or no way)."""
    es = es if es is not None else edges()
    if state == goal or state == State.MOVING:
        return -1
    # breadth first over the edges, each state's edges in (MOVE, POSTURE, WAIT_REFLEX), then table order
    order = sorted(range(len(es)), key=lambda i: (int(es[i].kind), i))
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


def next_table(es: list[Edge] | None = None) -> list[list[int]]:
    """next_edge for every state (rows, E.State order) and goal (columns, GOALS order)."""
    es = es if es is not None else edges()
    return [[next_edge(s, g, es) for g in GOALS] for s in State]


def posture_name(code: int) -> str:
    return Posture(code).name.lower()


def describe(es: list[Edge] | None = None) -> str:
    es = es if es is not None else edges()
    out = []
    for i, e in enumerate(es):
        what = {Kind.MOVE: f"move {e.move} (model-planned)", Kind.POSTURE: f"the body's own change to {posture_name(e.posture)}",
                Kind.WAIT_REFLEX: "wait for the body's fall reflex (placeholder)"}[e.kind]
        out.append(f"{i:2d} {e.src.name:14s} -> {e.dst.name:9s} {what}")
    t = next_table(es)
    out.append("first step, by state and goal (" + ", ".join(g.name for g in GOALS) + "):")
    for s in State:
        out.append(f"   {s.name:14s} " + " ".join(f"{v:3d}" for v in t[int(s)]))
    return "\n".join(out)


if __name__ == "__main__":
    print(describe())
