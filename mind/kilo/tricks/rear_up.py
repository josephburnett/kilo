"""rear_up: Kilo sits up on his hind feet and his tail, front paws in the air (A2).

    python -m kilo.tricks.rear_up              write the three stages' JSON, print their checks
    python -m kilo.tricks.rear_up --check      exit 1 if a JSON is stale or a check fails
    python -m kilo.tricks.rear_up --stage 1    only that stage
    python -m kilo.tricks.rear_up --quick      the fitted model only: no bands, no stops

docs/rear-up-trick.md has the design in plain words, the procedure and the
history; docs/rear-up-trick-results.md what happened on Kilo.

THE THIRD DESIGN (2026-11-21). The end pose is the same static TRIPOD as
before: both hind feet forward under the belly, the tail tip on the floor
behind (the tail servo swings it left and right; at 0 its tip is lowest, so
it stays at 0 as a fixed strut), the centre of mass between them, the front
paws up. What is new is the way there, and the model that judges it.

What the plays of 2026-11-19..21 taught (kilo.tricks.dynamics, fitted to
them): Kilo's leg servos give about 4 deg per kg cm under load, and his
centre of mass is about 5.5 mm further back and 5 mm further right than the
planner's model. So on four feet with the hind feet forward he rocks back
onto his tail by himself (the second design's "gather", every time, +23
deg), and on three feet near a diagonal he goes over it (its "lift_r", 3 of
7 times). A rigid model's margin is not a margin: every check here runs in
the compliant model over its uncertainty (dynamics.bands).

The way up (the owner's idea: flex the knees and push him back):

- look: the head comes up (a cat looks before it moves);
- crouch: the hind knees fold, the body goes down and back a little; a
  short pause;
- back: the front legs push the body back over the hind feet, which never
  leave the floor: the centre of mass goes behind them and he rocks back
  onto the frame's rear corners and the tail (on purpose; guarded);
- paw_l, paws: the paws fold up in front, the left one first;
- up: the hind legs lengthen and lift the hips until the tail tip carries
  the rear: the tripod, stage 1, held. The left hind leg is 2.5 mm
  shorter than the right (RIGHT_DROP): he leans left by that, against the
  rightward centre of mass, which otherwise rolls him 2 to 18 deg onto
  his right side on the compliant legs.

Stages 2 and 3 go on on the tripod (rise, mid; climb, top: the hind legs
straighten and draw the feet in as the centre of mass comes back over the
hips; the tail tip slides forward). The way down is the way up backwards:
the tripod frames in reverse to "up", the hind legs fold and he sits back
on his rear ("settle"), the paws drop to the floor in front ("drop"),
SIT. (A forward drop from the tripod straight onto the paws rolled him
up to 8 deg in the corners: the paws cannot reach the floor from 24 deg.)

THREE STAGES, one library entry each (STAGES; the remote's rear-up key
plays the one the build allows at most, KILO_REAR_UP=1, 2 or 3). Each stage
plays the one before it unchanged and then goes on: 1 rear_up holds at "up",
2 rear_up_2 at "mid", 3 rear_up_3 at "top". In a higher stage the lower
stages' tops hold STEP_HOLD (4 s): the mind's remote goes on to the next
stage only when A2 is pressed again in that time, else it stops the trick
there (TRICK_STOP: the straight line to SIT that every stop takes, checked
from every frame); kernel/mind/remote/remote.h. The remote finds the stage
tops by their guards' names ("up", "mid", "top").

Keyframes are given as world-space intent (Spec) and solved into joint
angles. Every hold stays far under 70 deg nose up, so the body's fall
detector (75 deg for 120 ms) stays armed.
"""

from __future__ import annotations

import argparse
import functools
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from . import dynamics as D
from . import sagittal as S
from . import stance as T
from .library import path_for
from .model import CANONICAL, JOINT_IDS, JOINT_LIMITS, JOINT_NAMES, Easing, Guard, Keyframe, Posture, Trick, TICK_MS

END = Posture.SIT
JOINTS = tuple(JOINT_NAMES[j] for j in CANONICAL)


def J(fs, fk, bs, bk, tail=0, pan=0, tilt=0, *, lb=None, rb=None) -> dict:
    """Joint angles (OpenCat degrees) with both sides alike, or a hind leg
    of its own: ``lb`` / ``rb`` = (shoulder, knee)."""
    lbs, lbk = lb if lb is not None else (bs, bk)
    rbs, rbk = rb if rb is not None else (bs, bk)
    return {"head_pan": pan, "head_tilt": tilt, "tail": tail,
            "lf_shoulder": fs, "rf_shoulder": fs, "rb_shoulder": rbs, "lb_shoulder": lbs,
            "lf_knee": fk, "rf_knee": fk, "rb_knee": rbk, "lb_knee": lbk}


# The body's postures (body/src/core/posture.cpp), as asked: SIT's back
# shoulders -90 are held at -80 by the joint limits.
STAND = D.STAND
SIT = D.SIT
REST = J(60, -45, -60, 45, -45, -30, -80)
SIT_FRAME = J(30, 60, -80, 45, -60, 10, -20)  # the trick's last frame: SIT within the joint limits

# ---- the calibration the design needs, and Kilo's as recorded ---------------------------------
# The right hind shoulder's servo zero after re-seating its upper leg (docs section 2).
# The plays since 2026-11-19 read as re-seated (dynamics: SIT and the tail rolls), not confirmed.
DESIGN_CZP = D.DESIGN_CZP
DESIGN = T.Geometry3(czp=DESIGN_CZP)
# Kilo before the re-seat (the EEPROM dump of 2026-10-10: the right hind shoulder stops at -57):
# the rigid model's fit of 2026-11-17, and the trick must stop itself safely on it.
AS_RECORDED = T.Geometry3(czp=T.KILO_CZP, com_dy=-2.0)
REACH_MARGIN = 2.0  # deg: every angle at least this far inside what the servo reaches (DESIGN)
MIN_KNEE_BEND = 25.0  # the hind knee stays bent at least this much (the leg at most 98 % of its length)


@dataclass(frozen=True)
class Spec:
    """One keyframe as intent. phi: body pitch, nose up, deg; h: the hind
    shoulder axles' height above the floor, mm, or None for "on the tail"
    (sagittal.tail_touch_height at phi); hind_dx: the hind feet's floor
    point ahead of the hind axles (world X, mm); ``left`` / ``right``:
    (dx, lift) for one hind foot instead (lift < 0: that leg reaches
    further down); front: the front feet's floor point ahead of the front
    axles, or None for paws up at ``paws`` (OpenCat shoulder, knee); tail:
    its servo angle; head: (pan, tilt; tilt + is up)."""
    name: str
    phi: float
    h: float | None
    hind_dx: float
    front: float | None = None
    paws: tuple = (0.0, 0.0)
    tail: float = 0.0
    left: tuple | None = None
    right: tuple | None = None
    head: tuple = (0.0, 0.0)


class DesignError(ValueError):
    pass


def _leg(g: S.Geometry, front: bool, dxw: float, dzw: float, phi: float) -> tuple[float, float]:
    """The foot's disc centre at world offset (dxw, dzw) from its shoulder
    axle -> OpenCat (shoulder, knee): the planner's IK (core/ik.h) without
    its 0.95 reach cap (trick frames are joint angles), the knee bent at
    least MIN_KNEE_BEND."""
    xb, zb = S.to_world(dxw, dzw, -phi)
    b = g.body
    l1, l2 = b.l1_mm, b.l2_mm
    d = math.hypot(xb, zb)
    d_max = math.sqrt(l1 * l1 + l2 * l2 + 2 * l1 * l2 * math.cos(math.radians(MIN_KNEE_BEND)))
    if not (b.min_reach_mm - 1e-6 <= d <= d_max + 1e-6 and zb < 0):
        raise DesignError(f"foot ({xb:.0f}, {zb:.0f}) mm out of reach")
    c = min(1.0, max(-1.0, (d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2)))
    t2 = math.atan2(math.sqrt(1.0 - c * c) * (1.0 if front else -1.0), c)
    t1 = math.atan2(xb, -zb) - math.atan2(l2 * math.sin(t2), l1 + l2 * math.cos(t2))
    sh = -math.degrees(t1)
    kn = 90.0 - math.degrees(t2) if front else -(math.degrees(t2) + 90.0)
    return sh, kn


def hip_height(spec: Spec, g: S.Geometry | None = None) -> float:
    """The hind axles' height the Spec asks for, mm."""
    g = g or S.Geometry()
    return spec.h if spec.h is not None else S.tail_touch_height(g, spec.phi, spec.tail)


def reach_problems(js: dict, czp=DESIGN_CZP, margin: float = REACH_MARGIN) -> list[str]:
    """Joints outside what their servo reaches (stance.reach), less margin."""
    out = []
    for name, v in js.items():
        lo, hi = T.reach(JOINT_IDS[name], czp)
        if not lo + margin - 1e-9 <= v <= hi - margin + 1e-9:
            out.append(f"{name} {v:g} outside {lo + margin:.1f}..{hi - margin:.1f}")
    return out


def solve(spec: Spec, g: S.Geometry | None = None) -> dict:
    """Joint angles (whole degrees, as the wire carries them) for a Spec."""
    g = g or S.Geometry()
    r = g.body.foot_r_mm
    hx = g.body.length_mm / 2
    h = hip_height(spec, g)
    touch = S.tail_touch_height(g, spec.phi, spec.tail)
    if spec.h is not None and h < touch - 1.0:
        raise DesignError(f"{spec.name}: the tail would be {touch - h:.0f} mm into the floor")
    try:
        legs = {}
        for side, own in (("lb", spec.left), ("rb", spec.right)):
            dx, lift = own if own is not None else (spec.hind_dx, 0.0)
            legs[side] = _leg(g, False, dx, r - h + lift, spec.phi)
        if spec.front is not None:
            _, fz = S.to_world(2 * hx, 0.0, spec.phi)
            fs, fk = _leg(g, True, spec.front, r - (h + fz), spec.phi)
        else:
            fs, fk = spec.paws
    except DesignError as e:
        raise DesignError(f"{spec.name}: {e}") from None
    js = J(fs, fk, 0, 0, spec.tail, spec.head[0], spec.head[1], lb=legs["lb"], rb=legs["rb"])
    js = {k: int(round(v)) for k, v in js.items()}
    bad = [p for p in reach_problems(js) if not p.startswith(("head", "tail"))]
    for name in ("head_tilt", "tail"):
        lo, hi = JOINT_LIMITS[JOINT_IDS[name]]
        if not lo <= js[name] <= hi:
            bad.append(f"{name} {js[name]} outside {lo}..{hi}")
    if bad:
        raise DesignError(f"{spec.name}: " + "; ".join(bad))
    return js


# ---- the design ------------------------------------------------------------------------------
# World places (mm, + forward): STAND's hind feet at HIND_X0, its front feet FRONT_GAP ahead.
HIND_X0 = -68.0
FRONT_GAP = 128.0
TUCK = (-15.0, -25.0)    # the paws folded up in front (the left front knee's pulse ends at -32)
RIGHT_DROP = 2.5         # mm: on the tripod the left hind leg is this much shorter than the right


def on_feet(name: str, ahead: float, h: float, phi: float, gap: float = FRONT_GAP, head=(0.0, 0.0)) -> Spec:
    """The hind feet where STAND put them, ``ahead`` mm in front of the hind
    axles, the axles ``h`` up, pitch ``phi``; the front feet ``gap`` mm in
    front of the hind feet."""
    X = HIND_X0 - ahead
    fx = X + S.to_world(123.0, 0.0, phi)[0]
    return Spec(name, phi=phi, h=h, hind_dx=ahead, front=HIND_X0 + gap - fx, head=head)


def on_tail(name: str, phi: float, ahead: float, head=(0.0, 0.0), paws=TUCK, drop: float = RIGHT_DROP) -> Spec:
    """On the tripod at pitch ``phi``: the hips where the tail tip meets the
    floor, the hind feet ``ahead`` mm in front of the axles, the left one
    ``drop`` mm higher (its leg that much shorter: he leans left by that),
    the paws up."""
    return Spec(name, phi=phi, h=None, hind_dx=ahead, front=None, paws=paws, left=(ahead, drop),
                right=(ahead, 0.0), head=head)


@dataclass(frozen=True)
class Step:
    """One keyframe of the way up: its pose, its move (ticks; or ``rate``,
    deg/s, the largest joint's average speed), its hold, a guard (the
    window of TELEMETRY nose up the mind accepts over the hold's second
    half, and the |roll| limit), and ``tuck``: the paws tucked only on
    that side ("l", "r") or both (None: as the pose says)."""
    spec: Spec
    dur: int = 0
    hold: int = 0
    rate: float = 0.0
    guard: tuple | None = None
    tuck: str | None = None
    pose: dict | None = None      # the joints as given (not solved from spec)

    @property
    def name(self) -> str:
        return self.spec.name


GUARD_HOLD = 20   # ticks a guarded frame on the way up holds (0.4 s; the mind averages its second half)
HOLD = 100        # ticks at a stage's top (2 s)
STEP_HOLD = 200   # ticks at a lower stage's top in a higher stage (4 s): the remote's A2 goes on, else down
RISE_DPS = 30.0   # on the tripod above stage 1: the tail tip slides forward as Kilo rises
GUARD_ROLL = 6.0  # deg: the guards' |roll| limit (TELEMETRY, which reads about +2 where Kilo is level)
PLAY_ROLL_MAX = 8.0

# The guards' windows are the compliant model's readings over every band (check() prints them,
# and holds every band at least GUARD_PAD inside): see docs/rear-up-trick.md section 8.
UP = (
    Step(Spec("look", 0, None, 0), dur=15, pose=dict(STAND, head_tilt=15)),
    Step(on_feet("crouch", 60, 66, 0, head=(0, 10)), dur=18, hold=8),
    Step(on_feet("back", 85, 42, 5, gap=112, head=(0, 0)), dur=18, hold=GUARD_HOLD, guard=(3.5, 12.5, GUARD_ROLL)),
    Step(on_feet("paw_l", 85, 42, 5, gap=112, head=(0, -10)), dur=12, tuck="l"),
    Step(on_feet("paws", 85, 42, 5, gap=112, head=(0, -15)), dur=12, hold=GUARD_HOLD, tuck="lr",
         guard=(2.5, 12.0, GUARD_ROLL)),
    Step(on_tail("up", 28, 78, head=(0, -25)), dur=25, hold=HOLD, guard=(19.5, 27.0, GUARD_ROLL)),
    Step(on_tail("rise", 34, 68, head=(0, -30)), rate=RISE_DPS),
    Step(on_tail("mid", 40, 64, head=(0, -30)), rate=RISE_DPS, hold=HOLD,
         guard=(33.0, 41.0, GUARD_ROLL)),
    Step(on_tail("climb", 47, 55, head=(0, -35)), rate=RISE_DPS),
    Step(on_tail("top", 54, 51, head=(0, -40)), rate=RISE_DPS, hold=HOLD,
         guard=(46.0, 55.0, GUARD_ROLL)),
)
NAMES_UP = tuple(s.name for s in UP)


@dataclass(frozen=True)
class Stage:
    """One library entry: the way up to UP[top], held there, and down.
    ``may_stop``: in a band corner a guard may stop it (it must come down
    safely); stage 1 must reach its top in every band."""
    number: int
    name: str
    top: int            # index into UP of the held frame
    stop_ticks: int     # TRICK_STOP's straight line to SIT, ticks
    may_stop: bool = True


STAGES = (
    Stage(1, "rear_up", top=NAMES_UP.index("up"), stop_ticks=75, may_stop=False),
    Stage(2, "rear_up_2", top=NAMES_UP.index("mid"), stop_ticks=100),
    Stage(3, "rear_up_3", top=NAMES_UP.index("top"), stop_ticks=125),
)
NAMES = tuple(st.name for st in STAGES)
DOWN_TO = "up"      # the way down: the tripod frames in reverse to here, then settle, drop, SIT
SETTLE_TICKS = 25
DROP_TICKS = 15
SIT_TICKS = 25


def stage(n) -> Stage:
    """A stage by number (1..3), library name, or itself."""
    if isinstance(n, Stage):
        return n
    for st in STAGES:
        if n in (st.number, st.name):
            return st
    raise KeyError(f"no rear_up stage {n!r}")


@dataclass(frozen=True)
class Frame:
    name: str
    joints: dict
    dur: int
    hold: int
    guard: tuple | None = None


def _dur(a: dict, b: dict, rate: float) -> int:
    d = max(abs(a[k] - b[k]) for k in JOINTS)
    return max(15, math.ceil(d / rate * 1000.0 / TICK_MS))


def joints_for(step: Step, g: S.Geometry | None = None) -> dict:
    js = dict(step.pose) if step.pose is not None else solve(step.spec, g)
    if step.tuck:
        for side in step.tuck:
            js[f"{side}f_shoulder"], js[f"{side}f_knee"] = int(TUCK[0]), int(TUCK[1])
    return js


def frames(st=1, g: S.Geometry | None = None) -> list[Frame]:
    """The whole motion: up (UP to the stage's top), the tripod frames
    back down to "up" (each as slow as the move up it reverses), down onto
    the rear ("settle"), the paws down ("drop"), SIT."""
    st = stage(st)
    lower = {s.top for s in STAGES if s.number < st.number}
    out = []
    prev = STAND
    ups = []
    for i, s in enumerate(UP[:st.top + 1]):
        js = joints_for(s, g)
        hold = HOLD if i == st.top else STEP_HOLD if i in lower else (s.hold if i < STAGES[0].top else
                                                                     (GUARD_HOLD if s.guard else 0))
        out.append(Frame(s.name, js, s.dur or _dur(prev, js, s.rate), hold, s.guard))
        ups.append(js)
        prev = js
    low = NAMES_UP.index(DOWN_TO)
    for i in range(st.top - 1, low - 1, -1):
        js = ups[i]
        out.append(Frame(f"down:{UP[i].name}", js, _dur(prev, js, UP[i + 1].rate), 0))
        prev = js
    # down the way he came up: the hind legs fold and he sits back on his rear ("settle", the paws
    # still tucked), the paws drop to the floor in front ("drop"), SIT
    by_name = {f.name: f.joints for f in out}
    out.append(Frame("settle", by_name["paws"], SETTLE_TICKS, 0))
    out.append(Frame("drop", by_name["back"], DROP_TICKS, 0))
    out.append(Frame("sit", SIT_FRAME, SIT_TICKS, 0))
    return out


def keyframe(js: dict, dur: int, hold: int = 0) -> Keyframe:
    return Keyframe.from_joints(dict(js), dur=dur, hold=hold, easing=Easing.COSINE)


def joints_of(kf: Keyframe) -> dict:
    return {k: kf.joints[k] for k in JOINTS}


def top_frame(trick: Trick) -> int:
    """The frame a stage holds at its top: its last guarded frame."""
    return trick.guards[-1].frame


def frame_starts(fr: Sequence) -> list[float]:
    """When each frame's move starts (s from the play's first tick), and the end."""
    out, t = [], 0
    for f in fr:
        out.append(t * TICK_MS / 1000)
        t += f.dur + f.hold
    return out + [t * TICK_MS / 1000]


def _play_frames(fr: Sequence[Frame]) -> list:
    return [(f.joints, f.dur, f.hold) for f in fr]


@functools.lru_cache(maxsize=None)
def predicted(st=1) -> tuple:
    """The TELEMETRY nose up the fitted compliant model gives at the end of
    each keyframe (its move and its hold: where he settles), exported with
    the library as "predicted_pitch_deg" for kilo log trick."""
    fr = frames(st)
    pr = D.simulate(_play_frames(fr), D.FITTED, stop_ticks=stage(st).stop_ticks)
    out = []
    for te in frame_starts(fr)[1:]:
        m = pr.window(te - 0.06, te)
        out.append(round(float(pr.nose_up[m].mean()), 1))
    return tuple(out)


def build(g: S.Geometry | None = None, st=1) -> Trick:
    st = stage(st)
    fr = frames(st, g)
    pred = list(predicted(st.number))
    guards = tuple(Guard(frame=i, nose_up=(f.guard[0], f.guard[1]), roll_max=f.guard[2], name=f.name)
                   for i, f in enumerate(fr) if f.guard and f.hold >= GUARD_HOLD)
    top = UP[st.top]
    return Trick(
        name=st.name, frames=tuple(keyframe(f.joints, f.dur, f.hold) for f in fr), end_posture=END,
        stop_ticks=st.stop_ticks, guards=guards, roll_max=PLAY_ROLL_MAX,
        source={"kind": "authored", "design": "kilo.tricks.rear_up", "doc": "docs/rear-up-trick.md",
                "stage": st.number, "design_version": 3, "model": "kilo.tricks.dynamics FITTED",
                "requires": "the right hind shoulder (joint 10) reaching -78 deg: calibration offset +7..+25 "
                            "(kilo cal show), docs/rear-up-trick.md section 2",
                "keyframes": [f.name for f in fr], "predicted_pitch_deg": pred},
        notes=(
            f"authored: rear_up stage {st.number} of {len(STAGES)}, design 3: crouch, push back onto the rear, "
            f"paws up, the hind legs lift him onto the hind feet and the tail tip, {top.spec.phi:g} deg",
            "the hind feet never leave the floor; the tail stays at 0 (its lowest point) as a strut",
            "generated by python -m kilo.tricks.rear_up, judged in the compliant model (kilo.tricks.dynamics) "
            "over its uncertainty bands, every stop included",
            "guards: the mind checks the measured pitch and roll at the marked holds and stops the trick (SIT) "
            "when they are off; predicted_pitch_deg is TELEMETRY's nose up in the fitted model",
            "balance off, fall_ok off: the pitch stays under 70 deg nose up, the fall detector (75 deg) stays armed",
            f"end posture SIT; TRICK_STOP goes to it in a straight line over stop_ticks ({st.stop_ticks})",
        ),
    ).validate()


# ---- the check --------------------------------------------------------------------------------
PITCH_MAX = 65.0       # deg nose up, anywhere (the fall detector trips at 75)
ROLL_STOP = 20.0       # deg |roll| coming down after a stop
GUARD_PAD = 1.0        # deg: every band's reading at least this far inside each guard's window
TAIL_MIN_G = 15.0      # the tail carries at least this at stage 1's top (a margin behind the hind feet)
TAIL_MIN_HIGH_G = 8.0  # ...at stages 2 and 3 (a forward tip there lands on the folded paws)
FOOT_MIN_G = 20.0      # ...and each hind foot this
SLIDE_MAX = 25.0       # mm a hind foot may slide over the whole play
TORQUE_MAX = 2.4       # kg cm on a leg servo (0.1 s mean): 80 % of the ESTIMATED 3 kg cm stall
HIND = ("lb_foot", "rb_foot")
FRONT = ("lf_foot", "rf_foot")
FEET = HIND + FRONT
ALLOWED = set(FEET) | {T.TAIL, "butt_l", "butt_r", "lf_knee", "rf_knee", "lb_knee", "rb_knee"}


def as_recorded() -> D.Params:
    """Kilo before the right hind upper leg was re-seated: it stops at -57 deg."""
    return D.FITTED.with_(czp=T.KILO_CZP)


@dataclass
class Run:
    """One play (or one cut-short) in one band."""
    band: str
    kind: str                        # "play", or "stop at NAME" / "stop in NAME" (half way into its move)
    stopped: str = ""                # what stopped the play (a guard, the roll limit)
    fell: bool = False
    max_nose_up: float = 0.0
    max_roll: float = 0.0            # |roll|, TELEMETRY
    guards: dict = field(default_factory=dict)   # guard name -> (mean nose up, max |roll|) over its window
    hold: dict = field(default_factory=dict)     # at the stage's top: loads (g) and the roll
    tripod_at: float | None = None   # s: on the hind feet and the tail, the front feet off, from then on
    paws_at: float | None = None     # s: both front feet off the floor from then on
    slide: dict = field(default_factory=dict)
    torque: float = 0.0
    torque_at: str = ""
    forbidden: set = field(default_factory=set)
    end: dict = field(default_factory=dict)      # at the end: loads, nose up, roll
    problems: list = field(default_factory=list)


def _off_from(pr: D.Prediction, cond: np.ndarray, until: float) -> float | None:
    """The first time from which ``cond`` holds until ``until``."""
    m = pr.t <= until
    bad = np.nonzero(m & ~cond)[0]
    last_bad = bad[-1] if len(bad) else -1
    if last_bad + 1 >= len(pr.t) or pr.t[last_bad + 1] > until:
        return None
    return float(pr.t[last_bad + 1])


def judge(pr: D.Prediction, trick: Trick, band: str, kind: str = "play") -> Run:
    """The rules a play (or a cut-short) is held to; ``Run.problems`` lists what broke them."""
    may_stop = stage(trick.name).may_stop and not band.startswith("fitted")
    fr = trick.frames
    st = frame_starts(fr)
    r = Run(band, kind, stopped=pr.stopped[1] if pr.stopped else "", fell=pr.fell is not None)
    t_stop = pr.stopped[0] if pr.stopped else 1e9
    r.max_nose_up = float(pr.nose_up.max())
    r.max_roll = float(np.abs(pr.roll).max())
    for gd in trick.guards:
        a = st[gd.frame] + (fr[gd.frame].dur + fr[gd.frame].hold / 2) * TICK_MS / 1000
        b = st[gd.frame] + (fr[gd.frame].dur + fr[gd.frame].hold) * TICK_MS / 1000
        m = pr.window(a, b)
        if m.any() and b <= t_stop + 0.05:
            r.guards[gd.name] = (float(pr.nose_up[m].mean()), float(np.abs(pr.roll[m]).max()))
    top = top_frame(trick)
    hb = st[top + 1]
    ha = hb - fr[top].hold * TICK_MS / 1000 / 2
    if kind == "play" and not pr.stopped:
        m = pr.window(ha, hb)
        r.hold = {k: float(pr.load[k][m].mean()) for k in FEET + (T.TAIL, "lf_knee", "rf_knee")}
        r.hold["roll"] = float(pr.roll[m].mean())
        r.hold["nose_up"] = float(pr.nose_up[m].mean())
        front_off = (pr.load["lf_foot"] < 2) & (pr.load["rf_foot"] < 2) & (pr.load["lf_knee"] < 2) \
            & (pr.load["rf_knee"] < 2)
        r.paws_at = _off_from(pr, front_off, hb)
        tri = front_off & (pr.load["lb_foot"] > 5) & (pr.load["rb_foot"] > 5) & (pr.load[T.TAIL] > 5)
        r.tripod_at = _off_from(pr, tri, hb)
    r.slide = {k: round(v, 1) for k, v in pr.slide.items() if v >= 0.5}
    if pr.torque:
        k, (v, t) = max(pr.torque.items(), key=lambda kv: kv[1][0])
        r.torque, r.torque_at = v, f"{k} at {t:.2f}s"
    for k, v in pr.load.items():
        if k not in ALLOWED and v.max() > 5.0:
            r.forbidden.add(k)
    m = pr.t >= pr.t[-1] - 0.2
    r.end = {k: float(pr.load[k][m].mean()) for k in FEET + (T.TAIL,)}
    r.end["nose_up"], r.end["roll"] = float(pr.nose_up[m].mean()), float(pr.roll[m].mean())
    # the judgement
    p = r.problems
    if r.fell:
        p.append("FELL")
    if r.max_nose_up > PITCH_MAX:
        p.append(f"pitches {r.max_nose_up:.1f} deg nose up (max {PITCH_MAX:g})")
    if r.forbidden:
        p.append(f"touches the floor with {sorted(r.forbidden)}")
    if min(r.end[k] for k in HIND) < 10 or r.end["lf_foot"] + r.end["rf_foot"] < 30 or r.end[T.TAIL] > 5 \
            or abs(r.end["roll"]) > 6:
        p.append(f"does not end sitting on its four feet: {_fmt_loads(r.end)}")
    if kind == "play":
        if r.stopped:
            if not may_stop:
                p.append(f"stopped: {r.stopped}")
            if r.max_roll > ROLL_STOP:
                p.append(f"rolls {r.max_roll:.1f} deg coming down (max {ROLL_STOP:g})")
        else:
            h = r.hold
            tail_min = TAIL_MIN_G if stage(trick.name).number == 1 else TAIL_MIN_HIGH_G
            if min(h[k] for k in HIND) < FOOT_MIN_G or h[T.TAIL] < tail_min or \
                    max(h[k] for k in FRONT + ("lf_knee", "rf_knee")) > 2:
                p.append(f"not on the tripod at the top: {_fmt_loads(h)}")
            for gd in trick.guards if not may_stop else ():
                if gd.name in r.guards:
                    nu, ro = r.guards[gd.name]
                    if not gd.nose_up[0] + GUARD_PAD <= nu <= gd.nose_up[1] - GUARD_PAD:
                        p.append(f"guard {gd.name}: nose up {nu:.1f} within {GUARD_PAD:g} of its window "
                                 f"{gd.nose_up[0]:g}..{gd.nose_up[1]:g}")
                    if ro > gd.roll_max - GUARD_PAD:
                        p.append(f"guard {gd.name}: |roll| {ro:.1f} within {GUARD_PAD:g} of its limit {gd.roll_max:g}")
            if r.max_roll > PLAY_ROLL_MAX - GUARD_PAD and not may_stop:
                p.append(f"rolls {r.max_roll:.1f} deg (the limit {PLAY_ROLL_MAX:g})")
            for k in HIND:
                if r.slide.get(k, 0.0) > SLIDE_MAX:
                    p.append(f"{k} slides {r.slide[k]:.0f} mm (max {SLIDE_MAX:g})")
            if r.torque > TORQUE_MAX:
                p.append(f"{r.torque_at} carries {r.torque:.2f} kg cm (max {TORQUE_MAX:g})")
    elif r.max_roll > ROLL_STOP:
        p.append(f"rolls {r.max_roll:.1f} deg coming down (max {ROLL_STOP:g})")
    return r


def _fmt_loads(d: dict) -> str:
    return ", ".join(f"{k.replace('_foot', '')} {v:.0f}" + ("" if k in ("roll", "nose_up") else " g")
                     for k, v in d.items())


def _sim(job) -> Run:
    trick, band, p, kind, stop_tick, *more = job
    start = more[0] if more else STAND
    fr = [(joints_of(f), f.dur, f.hold) for f in trick.frames]
    if stop_tick is None:
        pr = D.simulate(fr, p, start=start, guards=trick.guards, roll_max=trick.roll_max,
                        stop_ticks=trick.stop_ticks)
    else:
        pr = D.simulate(fr, p, stop_tick=stop_tick, stop_ticks=trick.stop_ticks,
                        t_end=(stop_tick + trick.stop_ticks) * TICK_MS / 1000 + 1.5)
    return judge(pr, trick, band, kind)


def stop_points(trick: Trick) -> list[tuple[str, int]]:
    """(label, tick) of every cut-short checked: half way into each frame's
    move and at its end (then the hold), up to the last frame."""
    out, t = [], 0
    names = trick.source["keyframes"]
    for k, f in enumerate(trick.frames[:-1]):
        out.append((f"stop in {names[k]}", t + max(1, f.dur // 2)))
        out.append((f"stop at {names[k]}", t + f.dur))
        t += f.dur + f.hold
    return out


STOP_BANDS = ("fitted", "stiffer, further back", "softer, further right", "slow servos 70+100ms")


@dataclass
class Report:
    runs: list = field(default_factory=list)
    recorded: Run | None = None      # Kilo before the re-seat: stopped by a guard, or played safely

    @property
    def problems(self) -> list[str]:
        out = [f"{r.band} / {r.kind}: {p}" for r in self.runs for p in r.problems]
        if self.recorded is not None:
            out += [f"as recorded / play: {p}" for p in self.recorded.problems
                    if not p.startswith(("stopped", "not on the tripod"))]
        return out


def check(trick: Trick | None = None, bands: Sequence | None = None, stops: Sequence[str] | None = STOP_BANDS,
          workers: int | None = None) -> Report:
    """The play in every band (default dynamics.bands()), with the mind's
    guards; every cut-short in the bands ``stops``; the fitted play from SIT
    (A2 while he sits); the play on Kilo before the re-seat (it must stop
    itself safely, or play safely)."""
    trick = trick or build()
    bands = list(bands if bands is not None else D.bands())
    jobs = [(trick, b, p, "play", None) for b, p in bands]
    for b, p in bands:
        if stops and b in stops:
            jobs += [(trick, b, p, kind, tick) for kind, tick in stop_points(trick)]
    if any(b == "fitted" for b, _ in bands):  # A2 while he sits (after a stop): the trick starts from SIT
        jobs.append((trick, "fitted, from SIT", D.FITTED, "play", None, SIT))
    jobs.append((trick, "as recorded", as_recorded(), "play", None))
    n = workers or os.cpu_count() or 1
    if n > 1 and len(jobs) > 1:
        with ProcessPoolExecutor(max_workers=n) as ex:
            runs = list(ex.map(_sim, jobs, chunksize=1))
    else:
        runs = [_sim(j) for j in jobs]
    return Report(runs=runs[:-1], recorded=runs[-1])


def summary(rep: Report, trick: Trick) -> str:
    L = []
    secs = trick.duration_ticks * TICK_MS / 1000
    names = trick.source["keyframes"]
    pred = trick.source["predicted_pitch_deg"]
    st = frame_starts(trick.frames)
    L.append(f"{trick.name}: {len(trick.frames)} frames, {secs:.1f} s at 1x, end {trick.end_posture.name}, "
             f"stop_ticks {trick.stop_ticks}, roll_max {trick.roll_max:g}")
    gd = {g.frame: g for g in trick.guards}
    for i, f in enumerate(trick.frames):
        js = joints_of(f)
        g = gd.get(i)
        L.append(f"  {i:2d} {names[i]:11s} {st[i]:5.2f}s move {f.dur * 0.02:4.2f} hold {f.hold * 0.02:4.2f}  "
                 f"front {js['lf_shoulder']:4d} {js['lf_knee']:4d} / {js['rf_shoulder']:4d} {js['rf_knee']:4d}  "
                 f"LB {js['lb_shoulder']:4d} {js['lb_knee']:4d}  RB {js['rb_shoulder']:4d} {js['rb_knee']:4d}  "
                 f"head {js['head_tilt']:4d}  pred {pred[i]:+5.1f}"
                 + (f"  guard {g.nose_up[0]:g}..{g.nose_up[1]:g} |roll|<={g.roll_max:g}" if g else ""))
    plays = [r for r in rep.runs if r.kind == "play"]
    L.append("plays (TELEMETRY nose up / roll; loads at the top's hold, g; s from the start):")
    for r in plays:
        gs = "  ".join(f"{k} {v[0]:+.1f}/{v[1]:.1f}" for k, v in r.guards.items())
        h = r.hold
        top = (f"top {h['nose_up']:+.1f}/{h['roll']:+.1f} lb {h['lb_foot']:.0f} rb {h['rb_foot']:.0f} "
               f"tail {h[T.TAIL]:.0f}") if h else "-"
        tm = (f"paws up {r.paws_at:.2f}s tripod {r.tripod_at:.2f}s" if r.paws_at is not None and
              r.tripod_at is not None else "")
        sl = ",".join(f"{k.replace('_foot', '')} {v:.0f}" for k, v in r.slide.items() if v >= 1) or "-"
        L.append(f"  {r.band:26s} {tm:28s} {top:40s} max|roll| {r.max_roll:4.1f} slid {sl}  "
                 f"servo {r.torque:.2f} ({r.torque_at})  | {gs}" + (f"  STOPPED {r.stopped}" if r.stopped else ""))
    for g in trick.guards:
        vals = [r.guards[g.name][0] for r in plays if g.name in r.guards]
        rolls = [r.guards[g.name][1] for r in plays if g.name in r.guards]
        if vals:
            L.append(f"  guard {g.name:6s} window {g.nose_up[0]:g}..{g.nose_up[1]:g}: the bands read "
                     f"{min(vals):+.1f}..{max(vals):+.1f}, |roll| up to {max(rolls):.1f} (limit {g.roll_max:g})")
    stops = [r for r in rep.runs if r.kind != "play"]
    if stops:
        w = max(stops, key=lambda r: r.max_roll)
        L.append(f"cut short: {len(stops)} runs; largest roll {w.max_roll:.1f} ({w.band} / {w.kind}); highest "
                 f"nose up {max(r.max_nose_up for r in stops):.1f}; all end sitting: "
                 f"{all(not any('end sitting' in p for p in r.problems) for r in stops)}")
    if rep.recorded is not None:
        a = rep.recorded
        L.append(f"Kilo before the re-seat (right hind shoulder stops at -57): "
                 f"{'stopped: ' + a.stopped if a.stopped else 'played to the top'}; max |roll| {a.max_roll:.1f}, "
                 f"ends {_fmt_loads(a.end)}")
    L.append(f"{len(rep.runs) + (rep.recorded is not None)} runs, {len(rep.problems)} problems")
    L += ["  " + p for p in rep.problems[:40]]
    return "\n".join(L)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Build and check the rear_up trick's three stages.")
    ap.add_argument("--check", action="store_true", help="only check: exit 1 if stale or failing")
    ap.add_argument("--stage", type=int, choices=[st.number for st in STAGES], help="only this stage")
    ap.add_argument("--quick", action="store_true", help="the fitted model only (no bands, no stops)")
    args = ap.parse_args(argv)
    bad = False
    for st in STAGES:
        if args.stage is not None and st.number != args.stage:
            continue
        trick = build(None, st)
        rep = check(trick, bands=D.bands()[:1] if args.quick else None, stops=None if args.quick else STOP_BANDS)
        print(summary(rep, trick), flush=True)
        print()
        path = path_for(st.name)
        text = trick.to_json()
        stale = not path.exists() or path.read_text() != text
        bad = bad or bool(rep.problems)
        if args.check:
            if stale:
                print(f"stale: {path.name} (run python -m kilo.tricks.rear_up)")
                bad = True
        elif stale:
            path.write_text(text)
            print(f"wrote {path.name}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
