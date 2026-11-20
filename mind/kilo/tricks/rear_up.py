"""rear_up: Kilo sits back on its hind feet and its tail, front paws off the floor.

    python -m kilo.tricks.rear_up            write the three stages' JSON, print their checks
    python -m kilo.tricks.rear_up --check    exit 1 if a JSON is stale or a check fails
    python -m kilo.tricks.rear_up --stage 3  only that stage
    python -m kilo.tricks.rear_up --search   why it needs the right hind shoulder to reach -78 deg

The design (docs/rear-up-trick.md has the reasoning, the hardware procedure
and the history). Kilo cannot balance on its hind feet: they are round (a
6 mm disc) and the servos answer too slowly to balance it actively. What
works is a static TRIPOD: the hind feet forward under the belly, the tail
tip on the floor behind, the centre of mass between them.

The tail (measured 2026-11-16, kilo.tricks.sagittal) turns LEFT AND RIGHT,
not down: at servo 0 its tip is lowest, 98 mm behind and 5 mm below the
hind shoulder axles. So it stays at 0 as a fixed strut, and the body tips
nose up until the tip meets the floor (sagittal.tail_touch_height).

THE SECOND DESIGN (2026-11-17), after the first fell twice on Kilo
(docs/rear-up-trick-results.md). What the trials taught, and what the
design does about it:

1. The body clamps every servo pulse to 500..2500 us. With Kilo's
   calibration the right hind shoulder stops at -57.3 deg (stance.reach),
   the left one goes to -80. The first design asked -80 and -77: the right
   hind leg stayed 23 deg short, Kilo stood on three feet with the left hind
   foot in the air and the tail 1-2 cm above the floor, and when the hind
   feet stepped it went over the left-rear diagonal. The model is now 3-D
   and applies the clamp (kilo.tricks.stance): every check runs on it.
2. No static path onto the tripod exists without the hind shoulders at
   about -75 deg while the hind feet pass under the hips (``--search``: with
   -54 the swing needs the hips 70 mm up, where the tail is 25 mm off the
   floor, and with all four feet down the centre of mass only gets behind
   the hind feet with the hips 70 mm up). So the trick needs the right hind
   upper leg re-seated on its servo and recalibrated
   (docs/rear-up-trick.md section 2: its calibration offset +7 to +25
   instead of -16). DESIGN is that calibration; AS_RECORDED is Kilo's
   as dumped (2026-10-10), on which the trick must stop itself safely at the
   first guard.
3. A hind foot cannot swing under the hips once the rear is on the tail
   (the hips are 45 mm up; the swing needs 47-50 even at -78 deg). So the
   hind feet step forward while Kilo stands high, ONE AT A TIME with the
   other three feet down (lean, lift_l, place_l, lean_r, lift_r,
   place_r); then the four feet stay planted and the body goes back and
   down over them (gather, sit_back, low) until the tail touches, and on
   back (onto) until the centre of mass is behind the hind feet: the front
   feet carry nothing and the paws come up (paws, up). Both hind feet are
   never off the floor at once.
4. The mind checks the measured attitude at five holds (Guard, kilo.tricks.
   model; gather, low, onto, paws, up) and stops the trick (TRICK_STOP, to
   SIT) when it is off: a hind leg not where it was asked shows as a roll
   at the gather, a tail not down as a pitch off at low and onto.
5. The checks are 3-D: the support polygon (seen from above), the roll,
   every topple, over geometries that include a tail a few degrees off
   centre, one side's servos sagging, and the centre of mass to either side.

THREE STAGES, one library entry each (STAGES; the remote's rear-up key
plays the one the build allows at most, KILO_REAR_UP=1, 2 or 3). Each stage
plays the one before it unchanged and then goes on: 1 rear_up holds at
UP_SPECS "up", 2 rear_up_2 at "mid", 3 rear_up_3 at "top" (the photo's
pose). In a higher stage the lower stages' tops hold STEP_HOLD (4 s): the
mind's remote lets the trick go on to the next stage only when A2 is pressed
again in that time, else it stops it there (TRICK_STOP: the crossfade to SIT
that every stop takes, checked from every frame); kernel/mind/remote/remote.h.

Keyframes are given as world-space intent (Spec) and solved into joint
angles for the model's geometry. Every hold stays far under 70 deg nose
up, so the body's fall detector (75 deg for 120 ms) stays armed.
"""

from __future__ import annotations

import argparse
import itertools
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field, replace
from typing import Sequence

from . import sagittal as S
from . import stance as T
from .library import path_for
from .model import JOINT_IDS, JOINT_LIMITS, Easing, Guard, Keyframe, Posture, Trick, TICK_MS

END = Posture.SIT

JOINTS = ("head_pan", "head_tilt", "tail", "lf_shoulder", "rf_shoulder", "rb_shoulder", "lb_shoulder",
          "lf_knee", "rf_knee", "rb_knee", "lb_knee")


def J(fs, fk, bs, bk, tail=0, pan=0, tilt=0, *, lb=None, rb=None) -> dict:
    """Joint angles (OpenCat degrees) with both sides alike, or a hind leg
    of its own: ``lb`` / ``rb`` = (shoulder, knee)."""
    lbs, lbk = lb if lb is not None else (bs, bk)
    rbs, rbk = rb if rb is not None else (bs, bk)
    return {"head_pan": pan, "head_tilt": tilt, "tail": tail,
            "lf_shoulder": fs, "rf_shoulder": fs, "rb_shoulder": rbs, "lb_shoulder": lbs,
            "lf_knee": fk, "rf_knee": fk, "rb_knee": rbk, "lb_knee": lbk}


# The body's postures (body/src/core/posture.cpp), as asked: SIT's back
# shoulders -90 are held at -80 by the joint limits (the model applies the
# limits and the pulse clamp itself, as the body does every tick).
STAND = J(30, 30, -30, -30, 0)
SIT = J(30, 60, -90, 45, -60, 10, -20)
REST = J(60, -45, -60, 45, -45, -30, -80)
SIT_FRAME = J(30, 60, -80, 45, -60, 10, -20)  # the trick's last frame: SIT within the joint limits

# ---- the calibration the design needs, and Kilo's as recorded ---------------------------------
# The right hind shoulder's CZP after re-seating its upper leg (docs
# section 2): calibration offset +12 (the range it may end in, +7..+25), CZP = 90 +
# 10 - 12 = 88. Its pulse then ends at -85 deg, past the -80 joint limit.
DESIGN_CZP = tuple(88 if j == JOINT_IDS["rb_shoulder"] else v for j, v in enumerate(T.KILO_CZP))
DESIGN = T.Geometry3(czp=DESIGN_CZP)
# Kilo on 2026-11-17: the EEPROM dump's calibration, the centre of mass 2 mm
# to the right (fitted: the tuck and SIT rolled the way it puts them).
AS_RECORDED = T.Geometry3(czp=T.KILO_CZP, com_dy=-2.0)
REACH_MARGIN = 2.0  # deg: every angle at least this far inside what the servo reaches (DESIGN)

# ---- speeds ---------------------------------------------------------------------------------
MOVE_DPS = 40.0   # average joint speed over a cosine move (peak 63 deg/s): slow against the servos
RISE_DPS = 20.0   # on the tripod above stage 1: the tail tip slides forward as Kilo rises
MIN_DUR = 30      # ticks (0.6 s)
GUARD_HOLD = 30   # ticks a guarded frame holds (0.6 s; the mind averages its second half)
MIN_KNEE_BEND = 25.0  # the hind knee stays bent at least this much (the leg at most 98 % of its length)


@dataclass(frozen=True)
class Spec:
    """One keyframe as intent. phi: body pitch, nose up, deg; h: the hind
    shoulder axles' height above the floor, mm, or None for "on the tail"
    (sagittal.tail_touch_height at phi, less ``press`` mm: the legs ask for
    the hips lower than the tail lets them go, so the tail carries the rear
    and the hind feet hover); hind_dx: the hind feet's floor point ahead of
    the hind axles (world X, mm) and hind_lift how far above the floor;
    ``left`` / ``right``: (dx, lift) for one hind foot instead (a step);
    front: the front feet's floor point ahead of the front axles, or None
    for paws up in ``paws`` (OpenCat shoulder, knee); tail: its servo angle
    (0: centre, its lowest point); head: (pan, tilt); hold: ticks held
    after arriving; guard: the mind checks the measured attitude during
    the hold (Guard); rate: the move's average joint speed, deg/s."""
    name: str
    phi: float
    h: float | None
    hind_dx: float
    front: float | None = None
    paws: tuple = (55.0, 20.0)
    press: float = 0.0
    tail: float = 0.0
    hind_lift: float = 0.0
    left: tuple | None = None
    right: tuple | None = None
    head: tuple = (0.0, 0.0)
    hold: int = 0
    guard: bool = False
    guard_roll: float = 0.0  # the guard's |roll| limit, deg (0: GUARD_ROLL)
    rate: float = MOVE_DPS


# The front paws on the tripod: tucked back near the shoulders (the left
# front shoulder's pulse ends at 62 deg on Kilo, so 55, not the first
# design's 60). A forward tip is caught by them (catch()).
PAWS = (55.0, 20.0)
HOLD = 100   # ticks at a stage's top (2 s)
STEP_HOLD = 200  # ticks at a lower stage's top in a higher stage (4 s): the remote's A2 goes on, else down

# The way onto the tripod, as places on the floor (world X, mm, + forward):
# the hind feet end at 0, the front feet stay at FRONT_X the whole time
# (from STAND, where the hind feet are FRONT_X + 128 behind them... i.e. at
# HIND_X0). The body is given by its hind axles (X, height) and its pitch.
FRONT_X = 60.0
HIND_X0 = FRONT_X - 128.0  # STAND: the feet 2.5 mm behind/ahead of their axles, which are 123 apart


def at(name: str, X: float, h: float, phi: float, lb=(0.0, 0.0), rb=(0.0, 0.0), front: float | None = FRONT_X,
       **kw) -> Spec:
    """A Spec from world places: the hind axles at (X, h), pitch phi; each
    hind foot at (world X, lift); the front feet at world X ``front`` (None:
    paws up)."""
    fx = X + S.to_world(123.0, 0.0, phi)[0]
    return Spec(name, phi=phi, h=h, hind_dx=lb[0] - X, left=(lb[0] - X, lb[1]), right=(rb[0] - X, rb[1]),
                front=None if front is None else front - fx, **kw)


H0 = HIND_X0
UP_SPECS = (
    # 1. lean forward over the planted feet, lower a little
    at("lean", -38, 80, 0, lb=(H0, 0), rb=(H0, 0), head=(0, -5)),
    # 2. the left hind foot steps forward to 0 (the other three hold)
    at("lift_l", -38, 80, 0, lb=(-34, 16), rb=(H0, 0), head=(0, -5)),
    at("place_l", -38, 80, 0, lb=(0, 0), rb=(H0, 0), head=(0, -5)),
    # 3. lean further (over the left hind foot and the front feet), and the right one steps
    at("lean_r", -26, 80, 0, lb=(0, 0), rb=(H0, 0), head=(0, -5)),
    at("lift_r", -26, 80, 0, lb=(0, 0), rb=(-34, 28), head=(0, -5)),
    at("place_r", -26, 80, 0, lb=(0, 0), rb=(0, 0), head=(0, -5)),
    # 4. the four feet stay put; the body goes back and down, nose up, until
    #    the tail meets the floor and the centre of mass is behind the hind feet
    at("gather", -54, 70, 6, head=(0, -10), hold=GUARD_HOLD, guard=True, guard_roll=3.0),
    at("sit_back", -54, 57, 13, head=(0, -12)),
    at("low", -54, 46, 24, head=(0, -15), hold=GUARD_HOLD, guard=True),
    at("onto", -72, 44, 24, head=(0, -20), hold=GUARD_HOLD, guard=True),
    # 5. the paws come up: on the tripod; checked
    at("paws", -72, 44, 24, front=None, paws=PAWS, head=(0, -25), hold=GUARD_HOLD, guard=True),
    Spec("up", phi=28, h=None, hind_dx=74, front=None, paws=PAWS, head=(0, -30), hold=HOLD, guard=True),
    # stages 2 and 3: on the tripod the hind legs straighten and draw the
    # feet in as the centre of mass comes back over the hips
    Spec("rise", phi=34, h=None, hind_dx=72, front=None, paws=PAWS, head=(0, -30), rate=RISE_DPS),
    Spec("mid", phi=40, h=None, hind_dx=68, front=None, paws=PAWS, head=(0, -30), hold=HOLD, guard=True,
         rate=RISE_DPS),
    Spec("climb", phi=47, h=None, hind_dx=60, front=None, paws=PAWS, head=(0, -30), rate=RISE_DPS),
    Spec("top", phi=54, h=None, hind_dx=52, front=None, paws=PAWS, head=(0, -30), hold=HOLD, guard=True,
         rate=RISE_DPS),
)


@dataclass(frozen=True)
class Stage:
    """One library entry: the way up to UP_SPECS[top], held there, and down."""
    number: int
    name: str
    top: int            # index into UP_SPECS of the held frame
    stop_ticks: int     # TRICK_STOP's crossfade to SIT, ticks
    margin_min: float   # the support margin at the hold the check holds it to, mm (worst over the geometries)


STAGES = (
    Stage(1, "rear_up", top=11, stop_ticks=75, margin_min=5.0),
    Stage(2, "rear_up_2", top=13, stop_ticks=100, margin_min=5.0),
    Stage(3, "rear_up_3", top=15, stop_ticks=125, margin_min=3.0),
)
# The way down: the tripod frames in reverse back to here (the paws down
# again), then SIT straight from there (the check covers that line too).
DOWN_TO = "onto"
NAMES = tuple(st.name for st in STAGES)

# The guards the trick carries: the window around the model's TELEMETRY
# nose-up pitch (deg) and the roll; and the roll limit over the whole play.
GUARD_TOL = 2.5
GUARD_ROLL = 6.0
PLAY_ROLL_MAX = 8.0


def stage(n) -> Stage:
    """A stage by number (1..3), library name, or itself."""
    if isinstance(n, Stage):
        return n
    for st in STAGES:
        if n in (st.number, st.name):
            return st
    raise KeyError(f"no rear_up stage {n!r}")


class DesignError(ValueError):
    pass


def _leg(g: S.Geometry, front: bool, dxw: float, dzw: float, phi: float) -> tuple[float, float]:
    """The foot's disc centre at world offset (dxw, dzw) from its shoulder
    axle -> OpenCat (shoulder, knee): the planner model's IK (core/ik.h)
    without its 0.95 reach cap (trick frames are joint angles), the knee
    bent at least MIN_KNEE_BEND."""
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
    if spec.h is not None:
        return spec.h
    return S.tail_touch_height(g, spec.phi, spec.tail) - spec.press


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
        raise DesignError(f"{spec.name}: the tail would be {touch - h:.0f} mm into the floor "
                          "(it cannot be turned down: use h=None)")
    try:
        legs = {}
        for side, own in (("lb", spec.left), ("rb", spec.right)):
            dx, lift = own if own is not None else (spec.hind_dx, spec.hind_lift)
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


def up_specs(st=1) -> tuple:
    """The way up for a stage: UP_SPECS to its top; the top held; the guards' holds kept."""
    st = stage(st)
    lower = {s.top for s in STAGES if s.number < st.number}
    out = []
    for i, s in enumerate(UP_SPECS[:st.top + 1]):
        # the top holds; a lower stage's top holds STEP_HOLD (the remote's A2 goes on from it); a
        # guarded frame below it holds only long enough for its guard
        hold = s.hold if i == st.top else STEP_HOLD if i in lower else (GUARD_HOLD if s.guard else 0)
        out.append(replace(s, hold=hold))
    return tuple(out)


def top_frame(trick: Trick) -> int:
    """The frame a stage holds at its top: its last guarded frame."""
    return trick.guards[-1].frame


@dataclass(frozen=True)
class Frame:
    name: str
    joints: dict
    dur: int
    hold: int
    guard: bool
    guard_roll: float = 0.0


def _dur(a: dict, b: dict, rate: float = MOVE_DPS) -> int:
    d = max(abs(a[k] - b[k]) for k in JOINTS)
    return max(MIN_DUR, math.ceil(d / rate * 1000.0 / TICK_MS))


def frames(st=1, g: S.Geometry | None = None) -> list[Frame]:
    """The whole motion: up (UP_SPECS to the stage's top), the same way down
    (each move back as slow as the move up it reverses), SIT."""
    specs = up_specs(st)
    ups = [solve(s, g) for s in specs]
    out = []
    prev = STAND
    for s, js in zip(specs, ups):
        out.append(Frame(s.name, js, _dur(prev, js, s.rate), s.hold, s.guard and s.hold >= GUARD_HOLD,
                         s.guard_roll or GUARD_ROLL))
        prev = js
    low = [s.name for s in specs].index(DOWN_TO)
    for i in range(len(ups) - 2, low - 1, -1):
        js = ups[i]
        out.append(Frame(f"down:{specs[i].name}", js, _dur(prev, js, specs[i + 1].rate), 0, False))
        prev = js
    out.append(Frame("sit", SIT_FRAME, _dur(prev, SIT_FRAME, MOVE_DPS), 0, False))
    return out


def keyframe(js: dict, dur: int, hold: int = 0) -> Keyframe:
    return Keyframe.from_joints(dict(js), dur=dur, hold=hold, easing=Easing.COSINE)


def joints_of(kf: Keyframe) -> dict:
    return {k: kf.joints[k] for k in JOINTS}


def keyframe_names(st=1) -> list[str]:
    return [f.name for f in frames(st)]


# ---- what the model predicts -------------------------------------------------------------------
def play_rests(g3: T.Geometry3, js_frames: Sequence[dict], start: dict = STAND, steps: int = 12):
    """stance.follow from ``start`` through the frames."""
    return T.follow(g3, js_frames, start, steps=steps)


def predicted(fr: Sequence[Frame], g3: T.Geometry3 = DESIGN) -> list[float]:
    """The TELEMETRY nose-up pitch the model predicts at the end of each
    keyframe's move (DESIGN, from STAND): exported with the library
    (source "predicted_pitch_deg") for kilo log trick, and the guards'
    windows are built on it."""
    out = []
    for smp in play_rests(g3, [f.joints for f in fr], steps=16):
        if smp.frame >= 0 and smp.s == 1.0:
            out.append(round(T.telemetry_pitch(g3, smp.rest), 1))
    return out


def build(g: S.Geometry | None = None, st=1) -> Trick:
    st = stage(st)
    fr = frames(st, g)
    pred = predicted(fr)
    guards = tuple(Guard(frame=i, nose_up=(round(pred[i] - GUARD_TOL, 1), round(pred[i] + GUARD_TOL, 1)),
                         roll_max=f.guard_roll, name=f.name) for i, f in enumerate(fr) if f.guard)
    top = UP_SPECS[st.top]
    return Trick(
        name=st.name, frames=tuple(keyframe(f.joints, f.dur, f.hold) for f in fr), end_posture=END,
        stop_ticks=st.stop_ticks, guards=guards, roll_max=PLAY_ROLL_MAX,
        source={"kind": "authored", "design": "kilo.tricks.rear_up", "doc": "docs/rear-up-trick.md",
                "stage": st.number, "design_version": 2,
                "requires": "the right hind shoulder (joint 10) reaching -78 deg: calibration offset +7..+25 "
                            "(kilo cal show), docs/rear-up-trick.md section 2",
                "keyframes": [f.name for f in fr], "predicted_pitch_deg": pred},
        notes=(
            f"authored (not a legacy skill): rear_up stage {st.number} of {len(STAGES)}, design 2: sit back on "
            f"the hind feet and the tail tip, front paws up, {top.phi:g} deg",
            "the tail stays at 0 (its lowest point, it swings sideways) as a strut; the body tips up onto it",
            "the hind feet swing forward one at a time, only after the mind has seen the tail carry the rear",
            "generated by python -m kilo.tricks.rear_up from world-space keyframes and kilo.tricks.stance (3-D)",
            "guards: the mind checks the measured pitch and roll at the marked holds and stops the trick (SIT) "
            "when they are off; predicted_pitch_deg is TELEMETRY's nose up",
            "balance off, fall_ok off: the pitch stays under 70 deg nose up, the fall detector (75 deg) stays armed",
            f"end posture SIT; TRICK_STOP crossfades to it over stop_ticks ({st.stop_ticks})",
        ),
    ).validate()


def build_all(g: S.Geometry | None = None) -> dict:
    return {st.name: build(g, st) for st in STAGES}


# ---- the check ----------------------------------------------------------------------------------
# What the design is held to, over GEOMETRIES (each one thing off the
# nominal, plus the corners of the centre of mass the photo allows):
PITCH_MAX = 70.0        # deg nose up, anywhere: the fall detector trips at 75
ROLL_PLAY = 7.0         # deg, |roll| anywhere while playing (the guards stop at 6, the whole play at 8)
ROLL_STOP = 20.0        # ...while coming down after a stop: cut short while one hind foot is in the air the
                        # straight line to SIT rocks Kilo onto that side (rigid model: up to 16 deg; it ends
                        # on its feet); the rigid model rolls further than Kilo does on three feet (SIT on
                        # Kilo as recorded: model 5.4, measured 2.0)
DROP_PLAY = 8.0         # mm: how far the point that lands may fall in a topple, playing
DROP_BACK_STOP = 15.0   # ...backwards when cut short
DROP_PAWS_STOP = 100.0  # ...when cut short, onto a foot (the paws (from the top of stage 3 the line to SIT
                        # brings them down from up to 9 cm: the "catch" below; the first design: 11 cm)
MARGIN_PLAY = 3.5       # mm: the centre of mass inside the support polygon, every sample of a play
TRANSFER = ("low", "onto", "paws", "down:paws", "down:onto", "sit")  # ...except while the load moves between the
                        # front feet and the tail (judged by the topple's drop and the roll; the hold margin
                        # is judged on its own)
LEG_GAP_MIN = 12.0      # mm between the front and the hind leg of a side
STALL_KG_CM = 3.0       # ESTIMATE (P1S-class servo)
TORQUE_MAX_KG_CM = 0.5 * STALL_KG_CM
HIND_FEET = (T.FOOT["lb"], T.FOOT["rb"])
FRONT_FEET = (T.FOOT["lf"], T.FOOT["rf"])
ALLOWED = T.FEET | {T.TAIL, "butt_l", "butt_r"}


# How far behind the model's the centre of mass may be for the sit-back to
# hold (mm). The path search (--search) finds no way down onto the tail that
# keeps it more than about 10 mm ahead of the hind feet before the tail
# touches, so the design cannot take the full -10 mm the measurement allows
# (with -10 the rear rocks back onto the tail from up to 20 mm).
COM_BACK = -5.0


def _side(sign: float, deg: float, side: str) -> tuple:
    legs = ("lf", "lb") if side == "l" else ("rf", "rb")
    return tuple((f"{leg}_{j}", sign * deg) for leg in legs for j in ("shoulder", "knee"))


def geometries(base: T.Geometry3 = DESIGN) -> list[tuple[str, T.Geometry3]]:
    """The nominal geometry; one thing off it at a time (the centre of mass
    10 mm each way fore-aft and up-down and 3 mm to each side; the tail
    3 mm longer or shorter, drooping 5 deg more or less, 5 deg off centre
    either way; one side's four leg servos 3 deg off either way, which is
    what a calibration error or a sag under load does); and the corners of
    the centre of mass fore-aft and up-down that the photo allows."""
    g = base.g
    out = [("nominal", base)]
    for k, vals in (("com_dx", (COM_BACK, 10)), ("com_dz", (-10, 10)), ("tail_len", (91, 97)),
                    ("tail_droop_deg", (13, 23))):
        for v in vals:
            out.append((f"{k}={v:g}", base.with_(g=g.with_(**{k: v}))))
    for v in (-3, 3):
        out.append((f"com_dy={v:g}", base.with_(com_dy=v)))
    for v in (-5, 5):
        out.append((f"tail_offset={v:g}", base.with_(tail_offset=v)))
    for side in ("l", "r"):
        for v in (-3, 3):
            out.append((f"{side} side {v:+g} deg", base.with_(offsets=_side(1, v, side))))
    for dx, dz in itertools.product((COM_BACK, 10), (-10, 10)):
        gg = g.with_(com_dx=dx, com_dz=dz)
        if S.photo_check(gg).ok:
            out.append((f"com_dx={dx},com_dz={dz}", base.with_(g=gg)))
    return out


@dataclass
class Result:
    """One geometry, one motion."""
    label: str
    pitch_max: float = -90.0
    roll_max: float = 0.0
    drop_max: float = 0.0
    drop_other: float = 0.0              # the largest drop landing on anything but a front foot
    margin_min: float = 1e9
    forbidden: set = field(default_factory=set)
    hold_margin: float | None = None     # the support margin at the stage's hold (None: not reached)
    hold_pitch: float | None = None      # TELEMETRY nose up there
    hold_on: tuple = ()
    aborted: str = ""                    # a guard that stopped the play (its name)
    guard_seen: dict = field(default_factory=dict)  # guard name -> (TELEMETRY nose up, roll) the model gives
    end_on: tuple = ()
    end_phi: float = 0.0
    problems: list = field(default_factory=list)
    notes: list = field(default_factory=list)


def run_play(trick: Trick, g3: T.Geometry3, label: str = "", steps: int = 12) -> Result:
    """The play from STAND with the mind's guards: at a guarded hold the
    model's TELEMETRY must be inside the guard's window, or the play is
    stopped there (TRICK_STOP: the crossfade to SIT) and judged as a stop."""
    res = Result(label)
    fr = [joints_of(f) for f in trick.frames]
    names = trick.source.get("keyframes") or [str(i) for i in range(len(fr))]
    guards = {g.frame: g for g in trick.guards}
    hold_i = top_frame(trick)
    prev = STAND
    r = T.rest(g3, prev)
    R = r.R
    stopped_at = None
    swing = {}
    for k, js in enumerate(fr):
        for i in range(1, steps + 1):
            s = i / steps
            cur = T.lerp(prev, js, s)
            r = T.rest(g3, cur, R)
            R = r.R
            _judge_sample(res, r, names[k], cur, prev, js, playing=True, s=s)
        prev = js
        if k == hold_i:
            res.hold_margin, res.hold_on = r.margin, tuple(sorted(r.contacts))
            res.hold_pitch = T.telemetry_pitch(g3, r)
        if k in guards:
            gd = guards[k]
            tp = T.telemetry_pitch(g3, r)
            res.guard_seen[gd.name] = (round(tp, 1), round(r.roll, 1))
            if not (gd.nose_up[0] <= tp <= gd.nose_up[1] and abs(r.roll) <= gd.roll_max):
                res.aborted = gd.name
                stopped_at = js
                break
    if stopped_at is not None:
        cur = stopped_at
        for i in range(1, steps * 2 + 1):
            s = i / (steps * 2)
            js = T.lerp(cur, SIT, s)
            r = T.rest(g3, js, R)
            R = r.R
            _judge_sample(res, r, "stop", js, cur, SIT, playing=False)
    res.end_on, res.end_phi = tuple(sorted(r.contacts)), r.pitch
    return res


def run_stop(trick: Trick, g3: T.Geometry3, at_frame: int, frac: float, label: str = "", steps: int = 12) -> Result:
    """Cut short ``frac`` of the way into frame ``at_frame`` (TRICK_STOP, the
    watchdog, an epoch bump): the crossfade to SIT from there."""
    res = Result(label)
    fr = [joints_of(f) for f in trick.frames]
    prev = STAND
    r = T.rest(g3, prev)
    R = r.R
    for k in range(at_frame + 1):
        js = fr[k]
        n = steps if k < at_frame else max(1, round(steps * frac))
        for i in range(1, n + 1):
            s = i / steps
            r = T.rest(g3, T.lerp(prev, js, s), R)
            R = r.R
        if k < at_frame:
            prev = js
    at = T.lerp(prev, fr[at_frame], frac)
    for i in range(1, steps * 2 + 1):
        s = i / (steps * 2)
        js = T.lerp(at, SIT, s)
        r = T.rest(g3, js, R)
        R = r.R
        _judge_sample(res, r, "stop", js, at, SIT, playing=False)
    res.end_on, res.end_phi = tuple(sorted(r.contacts)), r.pitch
    return res


def _judge_sample(res: Result, r: T.Rest3, name: str, cur: dict, a: dict, b: dict, playing: bool,
                  s: float = 1.0) -> None:
    res.pitch_max = max(res.pitch_max, r.pitch)
    res.roll_max = max(res.roll_max, abs(r.roll))
    for deg, onto, drop in r.topples:
        res.drop_max = max(res.drop_max, drop)
        if onto not in T.FEET:
            res.drop_other = max(res.drop_other, drop)
    res.forbidden |= set(r.contacts) - ALLOWED
    if playing:
        if name not in TRANSFER:
            res.margin_min = min(res.margin_min, r.margin)
        hind_off = all(h not in r.contacts for h in HIND_FEET)
        if hind_off and T.TAIL not in r.contacts and name not in ("tuck",):
            res.problems.append(f"{name}: both hind feet off the floor and the tail not on it")
        if hind_off and name == "tuck":
            res.problems.append("tuck: both hind feet off the floor")
        for side, leg in (("l", "lb"), ("r", "rb")):
            if name == f"lift_{side}" and s > 0.25 and T.FOOT[leg] in r.contacts:  # (it leaves the floor first)
                res.problems.append(f"{name}: the swinging {leg} foot touches the floor")
                break


def _floor(glabel: str, margin_min: float = MARGIN_PLAY) -> float:
    """The support margin a play must keep in geometry glabel: MARGIN_PLAY,
    except with one side's servos off, where the rigid model stands on
    three feet across a diagonal (Kilo's legs give a little and share the
    load over four; the roll and the topples still have to pass)."""
    return 0.0 if " side " in glabel else min(MARGIN_PLAY, margin_min)


def _judge(res: Result, kind: str, margin_min: float, floor: float = MARGIN_PLAY) -> Result:
    """kind: "play" or "stop"."""
    if res.forbidden:
        res.problems.append(f"touches the floor with {sorted(res.forbidden)}")
    if res.pitch_max > PITCH_MAX:
        res.problems.append(f"pitches {res.pitch_max:.1f} deg nose up (max {PITCH_MAX:g})")
    if T.TAIL in res.end_on or len(set(res.end_on) & T.FEET) < 3 or not set(res.end_on) & set(FRONT_FEET):
        res.problems.append(f"ends on {list(res.end_on)}, not down on its feet")
    if kind == "play" and not res.aborted:
        if res.roll_max > ROLL_PLAY:
            res.problems.append(f"rolls {res.roll_max:.1f} deg (max {ROLL_PLAY:g})")
        if res.drop_max > DROP_PLAY:
            res.problems.append(f"a topple drops {res.drop_max:.1f} mm (max {DROP_PLAY:g})")
        if res.margin_min < floor:
            res.problems.append(f"support margin down to {res.margin_min:.1f} mm (min {floor:g})")
        on_tripod = res.hold_on and set(HIND_FEET) <= set(res.hold_on) and T.TAIL in res.hold_on \
            and not set(FRONT_FEET) & set(res.hold_on)
        if not on_tripod:
            res.problems.append(f"not on the tripod at the hold: on {list(res.hold_on)}")
        elif res.hold_margin < margin_min:
            res.problems.append(f"hold margin {res.hold_margin:.1f} mm (min {margin_min:g})")
    else:
        if res.roll_max > ROLL_STOP:
            res.problems.append(f"rolls {res.roll_max:.1f} deg coming down (max {ROLL_STOP:g})")
        if res.drop_max > DROP_PAWS_STOP:
            res.problems.append(f"the paws drop {res.drop_max:.1f} mm coming down (max {DROP_PAWS_STOP:g})")
        if res.drop_other > DROP_BACK_STOP:
            res.problems.append(f"a topple drops {res.drop_other:.1f} mm onto something other than a foot "
                                f"coming down (max {DROP_BACK_STOP:g})")
        if kind == "play":
            res.notes.append(f"stopped by the guard at {res.aborted}")
    return res


@dataclass
class Report:
    results: list = field(default_factory=list)  # (geometry label, motion label, Result)
    today: Result | None = None                  # the play on Kilo as recorded (AS_RECORDED)
    catch: tuple = ()

    @property
    def problems(self) -> list[str]:
        out = [f"{g} / {m}: {p}" for g, m, r in self.results for p in r.problems]
        if self.today is not None:
            out += [f"as recorded / play: {p}" for p in self.today.problems]
        return out


def _check_one(job) -> list:
    trick, glabel, g3, margin_min, steps = job
    out = [(glabel, "play", _judge(run_play(trick, g3, glabel, steps), "play", margin_min, _floor(glabel, margin_min)))]
    n = len(trick.frames)
    for k in range(n - 1):
        for frac in (0.5, 1.0):
            res = run_stop(trick, g3, k, frac, glabel, steps)
            out.append((glabel, f"stop at {k}{'' if frac == 1.0 else '-'}", _judge(res, "stop", margin_min)))
    return out


def check(trick: Trick | None = None, geos=None, steps: int = 8, stops: bool = True) -> Report:
    """Every geometry (default: geometries()), the play and every cut-short;
    the play on Kilo as recorded must stop itself at a guard, safely; the
    nominal torques; the legs' gap."""
    trick = trick or build()
    st = stage(trick.name)
    geos = geos if geos is not None else geometries()
    rep = Report()
    jobs = [(trick, gl, g3, st.margin_min, steps) for gl, g3 in geos]
    run = _check_one if stops else (lambda j: [(j[1], "play", _judge(run_play(j[0], j[2], j[1], j[4]), "play", j[3],
                                                                   _floor(j[1], j[3])))])
    if stops and len(jobs) > 2 and (os.cpu_count() or 1) > 1:
        with ProcessPoolExecutor(max_workers=os.cpu_count()) as ex:
            for part in ex.map(_check_one, jobs):
                rep.results += part
    else:
        for job in jobs:
            rep.results += run(job)
    today = _judge(run_play(trick, AS_RECORDED, "as recorded", steps), "play", st.margin_min)
    if not today.aborted:
        today.problems.append("Kilo as recorded (the right hind shoulder short) is not stopped by a guard")
    rep.today = today
    # torques and the legs' gap, nominal, on the frames the side view can take (both sides alike)
    for i, f in enumerate(trick.frames):
        js = joints_of(f)
        p = S.Pose(js["lf_shoulder"], js["lf_knee"], js["lb_shoulder"], js["lb_knee"], js["tail"])
        gap = S.leg_gap(S.Geometry(), p)
        if gap < LEG_GAP_MIN:
            rep.results.append(("nominal", f"frame {i}", Result(f"frame {i}", problems=[f"legs {gap:.0f} mm apart"])))
    tq = torques(trick)
    for k, (v, name) in tq.items():
        if v > TORQUE_MAX_KG_CM:
            rep.results.append(("nominal", name, Result(name, problems=[f"{k} {v:.2f} kg cm (max {TORQUE_MAX_KG_CM})"])))
    held, tipped = catch(trick)
    rep.catch = (held.clear[T.FOOT["lf"]], tuple(sorted(tipped.contacts)), tipped.pitch)
    if set(tipped.contacts) - ALLOWED or not set(HIND_FEET) & set(tipped.contacts):
        rep.results.append(("nominal", "catch", Result("catch", problems=[
            f"a forward tip at the hold ends on {sorted(tipped.contacts)}, not caught by the paws"])))
    return rep


def torques(trick: Trick) -> dict:
    """The largest static servo torque (kg cm) per joint over the frames
    where the side view applies (both sides alike), nominal, from the side
    view's load split (sagittal.load)."""
    g = S.Geometry()
    out: dict = {}
    phi = 0.0
    for i, f in enumerate(trick.frames):
        js = joints_of(f)
        if js["lb_shoulder"] != js["rb_shoulder"] or js["lb_knee"] != js["rb_knee"]:
            continue
        p = S.Pose(js["lf_shoulder"], js["lf_knee"], js["lb_shoulder"], js["lb_knee"], js["tail"])
        r = S.rest(g, p, phi)
        phi = r.phi
        L = S.load(g, p, r)
        names = trick.source.get("keyframes") or []
        for k in ("hind_shoulder", "hind_knee", "front_shoulder", "front_knee", "tail"):
            v = getattr(L, k)
            if v > out.get(k, (0.0, ""))[0]:
                out[k] = (v, names[i] if i < len(names) else str(i))
    return out


def catch(trick: Trick, g3: T.Geometry3 = DESIGN):
    """A forward tip at the hold: Kilo pushed forward over its hind feet
    until something else touches. Returns (the rest at the hold, the rest
    from there): the second should be on the hind and the front feet."""
    hold_i = top_frame(trick)
    smp = play_rests(g3, [joints_of(f) for f in trick.frames[:hold_i + 1]], steps=8)
    held = smp[-1].rest
    js = joints_of(trick.frames[hold_i])
    R = held.R
    pitch, roll = T.attitude(R)
    land = pitch
    pts = T.points(g3, js)
    while land > -80.0:
        land -= 0.25
        Rl = T.rot(land, roll)
        W = [(lb, Rl @ p, rr) for lb, p, rr in pts]
        foot = min(w[2] - rr for lb, w, rr in W if lb in HIND_FEET)
        if any(w[2] - rr <= foot + 0.01 for lb, w, rr in W if lb not in HIND_FEET):
            break
    return held, T.rest(g3, js, T.rot(land, roll))


def summary(rep: Report, trick: Trick) -> str:
    lines = []
    secs = trick.duration_ticks * TICK_MS / 1000
    names = trick.source.get("keyframes", [])
    pred = trick.source.get("predicted_pitch_deg", [])
    lines.append(f"{trick.name}: {len(trick.frames)} frames, {secs:.1f} s at 1x, end {trick.end_posture.name}, "
                 f"stop_ticks {trick.stop_ticks}, roll_max {trick.roll_max:g}")
    for i, f in enumerate(trick.frames):
        js = joints_of(f)
        lines.append(f"  {i:2d} {names[i] if i < len(names) else '':12s} dur {f.dur:3d} hold {f.hold:3d}  "
                     f"front {js['lf_shoulder']:4.0f} {js['lf_knee']:4.0f}  "
                     f"LB {js['lb_shoulder']:4.0f} {js['lb_knee']:4.0f}  RB {js['rb_shoulder']:4.0f} {js['rb_knee']:4.0f}  "
                     f"tail {js['tail']:3.0f}  pred {pred[i] if i < len(pred) else '':>5}")
    for gd in trick.guards:
        lines.append(f"  guard at {gd.frame} {gd.name}: nose up {gd.nose_up[0]:g}..{gd.nose_up[1]:g}, "
                     f"|roll| <= {gd.roll_max:g}")
    nom = [r for g, m, r in rep.results if g == "nominal" and m == "play"]
    if nom:
        n = nom[0]
        lines.append(f"nominal: hold at {n.hold_pitch:.1f} deg (TELEMETRY nose up) on {list(n.hold_on)}, "
                     f"margin {n.hold_margin:.1f} mm; support margin down to {n.margin_min:.1f} mm on the way; "
                     f"roll up to {n.roll_max:.1f} deg; topples drop up to {n.drop_max:.1f} mm")
    plays = [(g, r) for g, m, r in rep.results if m == "play"]
    held = [(r.hold_margin, g) for g, r in plays if not r.aborted and r.hold_margin is not None]
    if held:
        lines.append("worst hold margin %.1f mm (%s)" % min(held))
    lines.append("worst support margin while playing %.1f mm (%s)" % min((r.margin_min, g) for g, r in plays
                                                                        if not r.aborted))
    lines.append("worst roll while playing %.1f deg (%s)" % max((r.roll_max, g) for g, r in plays if not r.aborted))
    ab = [(g, r.aborted, r.guard_seen.get(r.aborted)) for g, r in plays if r.aborted]
    lines.append(f"stopped by a guard in {len(ab)} of {len(plays)} geometries" + (":" if ab else ""))
    lines += [f"  {g}: at {a} (the model's nose up, roll: {s})" for g, a, s in ab]
    if rep.today is not None:
        t = rep.today
        lines.append(f"Kilo as recorded (right hind shoulder stops at {T.reach(10)[0]:.1f}): "
                     f"{'stopped by the guard at ' + t.aborted if t.aborted else 'NOT stopped'} "
                     f"(the model there: {t.guard_seen.get(t.aborted)}), ends on {list(t.end_on)}, "
                     f"rolls up to {t.roll_max:.1f} deg, topples up to {t.drop_max:.1f} mm")
    tq = torques(trick)
    lines.append("torque, kg cm, nominal (side view; stall ~%.1f, an estimate):" % STALL_KG_CM)
    for k, (v, name) in sorted(tq.items()):
        lines.append(f"  {k:15} {v:.2f}  ({name})")
    if rep.catch:
        lines.append(f"a forward tip at the hold: the paws fall from {rep.catch[0]:.0f} mm and Kilo ends on "
                     f"{list(rep.catch[1])} at {rep.catch[2]:.1f} deg")
    lines.append(f"{len(rep.results)} runs, {len(rep.problems)} problems")
    lines += ["  " + p for p in rep.problems[:40]]
    return "\n".join(lines)


# ---- --search: why the right hind shoulder must reach further ------------------------------------
def swing_hip_needed(phi: float, sh_min: float, kn_max: float = 44.0, clearance: float = 4.0) -> float:
    """The lowest hind-axle height (mm) from which a hind foot can swing
    from under its hip to 80 mm in front of it without touching the floor,
    with the shoulder no further forward than ``sh_min`` and the knee no
    more folded than ``kn_max``: the highest the foot can be over each
    point on the way, taken at the worst point."""
    g = S.Geometry()
    best: dict = {}
    sh = sh_min
    while sh <= 97:
        kn = -112.0
        while kn <= kn_max:
            _, _, (fx, fz) = S.leg_points(g, False, sh, kn)
            X, Z = S.to_world(fx + g.body.length_mm / 2, fz, phi)
            k = int(round(X / 5)) * 5
            if 0 <= k <= 80 and Z > best.get(k, -1e9):
                best[k] = Z
            kn += 1.0
        sh += 1.0
    return -min(best.values()) + g.body.foot_r_mm + clearance


def search_report() -> str:
    lines = ["The hind foot's swing from under the hip to 80 mm ahead, at 24 deg nose up (the sink):",
             f"  tail touches the floor with the hind axles {S.tail_touch_height(S.Geometry(), 24):.1f} mm up"]
    for sh in (-54, -60, -70, -78):
        lines.append(f"  shoulder to {sh}: the axles must be {swing_hip_needed(24, sh):.1f} mm up for a 4 mm clearance")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Build and check the rear_up trick's three stages.")
    ap.add_argument("--check", action="store_true", help="only check: exit 1 if stale or failing")
    ap.add_argument("--stage", type=int, choices=[st.number for st in STAGES], help="only this stage")
    ap.add_argument("--search", action="store_true", help="the reach the swing needs, and stop")
    args = ap.parse_args(argv)
    if args.search:
        print(search_report())
        return 0
    bad = False
    for st in STAGES:
        if args.stage is not None and st.number != args.stage:
            continue
        trick = build(None, st)
        rep = check(trick)
        print(summary(rep, trick))
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
