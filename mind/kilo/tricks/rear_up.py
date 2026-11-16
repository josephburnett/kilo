"""rear_up: Kilo sits back on its hind feet and its tail, front paws off the floor.

    python -m kilo.tricks.rear_up            write the three stages' JSON, print their checks
    python -m kilo.tricks.rear_up --check    exit 1 if a JSON is stale or a check fails
    python -m kilo.tricks.rear_up --report   the checks over the full uncertainty grid (slow)
    python -m kilo.tricks.rear_up --stage 3  only that stage

The design (docs/rear-up-trick.md has the reasoning and the hardware
procedure). Kilo cannot balance on its hind feet: they are round (a 6 mm
disc), so a feet-only stance has no support length, and the servos answer
too slowly to balance it actively. What works is a static TRIPOD: the hind
feet forward under the belly, the tail tip on the floor behind, the centre
of mass between them.

The tail (measured 2026-11-16, kilo.tricks.sagittal) turns LEFT AND RIGHT,
not down: at servo 0 it points back and 18 deg down and its tip is at its
lowest, 98 mm behind and 5 mm below the hind shoulder axles. So the tail
stays at 0 as a fixed strut, and the tripod is reached by tipping the body
nose up until the tip meets the floor: that happens when the hind axles
are 98 sin(pitch) + 5 cos(pitch) mm up (sagittal.tail_touch_height). The
pitch is then set by how high the hind legs hold the hips.

THREE STAGES, one library entry each, so the trials can stop part way up
(STAGES; the remote's B2 plays the one the build selects, KILO_REAR_UP=1,
2 or 3). Each stage plays the one before it unchanged and then goes on:

  1 rear_up    28 deg: the low sit-up (the first design)
  2 rear_up_2  40 deg
  3 rear_up_3  54 deg: the pose of the owner's photo (sagittal.PHOTO_POSE)

The photo (2026-11-16) showed the unpowered Kilo standing free at 54 deg
(read off the photo; the owner's "about 60" was by eye). The model
already stands in that pose (5 mm of margin), so it needs no correction;
what made the first design stop at 28 deg was (a) the FEET reach cap
(0.95 of the leg's length), which does not apply to trick frames (they are
joint angles: the body only applies the joint limits and the slew), and
(b) the "centre of mass forward AND low" corner of the uncertainty, which
the photo rules out (sagittal.photo_check). Every stage is checked over
the geometries the photo allows. At the top the front paws are tucked
(60, 20) instead of reaching forward as in the photo, which puts the
centre of mass 6.5 mm further behind the hind feet than in the pose Kilo
stood in by itself.

The motion, from STAND (or SIT):
  0 tuck    the hind feet forward a little, the rear lower, front feet down
  1 sink    the rear down onto the tail tip (24 deg nose up) with the hind
            feet still near the hips, so the centre of mass stays ahead of
            them: Kilo rests on the tail and the front feet
  2 step    the hind feet swing forward in the air (10 mm up) to 78 mm
            ahead of their axles
  3 ready   they come down: the centre of mass is now behind them, so the
            front feet carry little
  4 paws    the front paws come up and forward (they hover 25 mm above the
            floor: if the centre of mass is further forward than the model,
            Kilo tips onto them, a short drop)
  5 up      the hind legs lift the hips: 28 deg nose up (stage 1 holds 2 s)
  6 rise    (stages 2, 3) on the tripod, the hind legs straighten and draw
            the feet in, slowly: 34 deg
  7 mid     40 deg (stage 2 holds 2 s)
  8 climb   47 deg
  9 top     54 deg (stage 3 holds 2 s)
  then the same way down to the tuck, and SIT (the end posture).
While the pitch grows on the tripod the distance between the hind feet and
the tail tip shrinks (the centre of mass comes back over the hips, so the
feet must come back with it): the tail tip, which carries a tenth of the
weight, slides forward on the floor (about 60 mm from 24 to 54 deg) and
back on the way down. The model lets contacts slide freely.

Keyframes are given as world-space intent (Spec: body pitch, hip height or
"on the tail", where the hind and front feet stand) and solved into joint
angles for the model's geometry, so new measurements (Geometry) re-solve
the trick: change the defaults in kilo.tricks.sagittal.Geometry and run
this module.

Every hold stays far under 70 deg nose up, so the body's fall detector (75
deg for 120 ms) stays armed: the trick does NOT set FALL_OK.
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
from .model import JOINT_LIMITS, Easing, Keyframe, Posture, Trick, TICK_MS
from .library import path_for

# The end posture, which is also where every way of cutting the trick short
# goes (TRICK_STOP, a watchdog stop, an epoch bump): the body crossfades
# there in joint space from wherever it is. SIT, not STAND: the straight
# line from the tripod to STAND straightens the hind legs while the tail
# tip is still on the floor, which lifts the hips and pitches the body
# further nose up instead of bringing the front feet down; over the ranges
# a quarter of those stops end propped on the tail and the straight hind
# legs at 38-45 deg, front paws in the air. The line to SIT folds the hind
# legs and turns the tail aside (its tip rises off the floor), so the rear
# sinks and Kilo always ends sitting on its four feet.
END = Posture.SIT

# The body's SIT and REST postures (body/src/core/posture.cpp; SIT's back
# shoulders -90 are clamped to the joint limit -80 by the safety stage).
SIT = S.Pose(30, 60, -80, 45, -60, head_pan=10, head_tilt=-20)
STAND = S.Pose(30, 30, -30, -30, 0)
REST = S.Pose(60, -45, -60, 45, -45, head_pan=-30, head_tilt=-80)
END_POSE = SIT

# Joint speed for the move durations: the average over a cosine move; its
# peak is pi/2 times this (63 deg/s), slow against the servos' 0.07 s/60 deg
# and long against the 140 ms lag, so the motion stays close to static.
MOVE_DPS = 40.0
# On the tripod above stage 1 (the rise to 40 and 54 deg) half that: the
# tail tip slides forward as the body rises, and the owner's hand has time.
RISE_DPS = 20.0
MIN_DUR = 30  # ticks (0.6 s)
# The hind legs in trick frames: the body plays a trick's joint angles
# with only the joint limits and the slew (core/keyframe, the arbiter's
# safety stage); the 0.95 reach cap of core/ik.h is for FEET only. The
# design keeps the knee bent at least this much (the leg at most 0.98 of
# its full length), short of straight, where a forward-pointing hind knee
# could snap through backwards. The photo's knee is bent 33 deg.
MIN_KNEE_BEND = 25.0


@dataclass(frozen=True)
class Spec:
    """One keyframe as intent. phi: body pitch, nose up, deg; h: the hind
    shoulder axles' height above the floor, mm, or None for "on the tail":
    the height at which the tail tip just touches the floor at this pitch
    (sagittal.tail_touch_height), less ``press`` mm so that the tail
    carries some load; hind_dx: the hind feet's floor point ahead of the
    hind axles (world X, mm); front: the front feet's floor point ahead of
    the front axles, or None for paws up in ``paws`` (OpenCat shoulder,
    knee); tail: the tail servo's angle (0: centre, its lowest point; the
    trick keeps it there); hind_lift: the hind feet this far above the
    floor (stepping); head: (pan, tilt); hold: ticks held after arriving
    (when this is the stage's top); rate: the move's average joint speed,
    deg/s."""
    name: str
    phi: float
    h: float | None
    hind_dx: float
    front: float | None = None
    paws: tuple = (80.0, 0.0)
    press: float = 0.0
    tail: float = 0.0
    hind_lift: float = 0.0
    head: tuple = (0.0, 0.0)
    hold: int = 0
    rate: float = MOVE_DPS


# Chosen by searching each Spec's numbers against check() (the margins and
# the smallest rolls over the ranges); docs/rear-up-trick.md sections 1 and 7
# have the reasoning.
HOLD = 100  # ticks at a stage's top (2 s)
# The front paws on the tripod, the same at every stage: tucked back near
# the shoulders. Folded right back (80, 120) they would add 2 mm of margin
# at 54 deg, but a forward tip would then go on over them onto the nose;
# reaching forward as in the photo (-29, 15) costs 4 mm. At (60, 20) a
# forward tip is caught by the paws (catch(), the check's "catch").
PAWS = (60.0, 20.0)
UP_SPECS = (
    Spec("tuck", phi=8, h=70, hind_dx=25, front=0, head=(0, -10)),
    # the rear down onto the tail with the hind feet still under the hips:
    # the centre of mass stays ahead of them, so nothing rocks back
    Spec("sink", phi=24, h=None, hind_dx=36, front=-6, head=(0, -15)),
    # on the tail and the front feet, the hind feet step forward in the air
    Spec("step", phi=24, h=None, hind_dx=78, hind_lift=10, front=-6, head=(0, -20)),
    Spec("ready", phi=24, h=None, hind_dx=78, front=-6, head=(0, -20)),
    Spec("paws", phi=24, h=None, hind_dx=78, front=None, paws=PAWS, head=(0, -25)),
    Spec("up", phi=28, h=None, hind_dx=78, front=None, paws=PAWS, head=(0, -30), hold=HOLD),
    # stages 2 and 3: up on the tripod, the hind legs straighten and draw
    # the feet in as the centre of mass comes back over the hips; at 40 and
    # 54 deg the feet are as far forward as MIN_KNEE_BEND lets them (the
    # top: the knee bent 26 deg, the photo's 33 deg)
    Spec("rise", phi=34, h=None, hind_dx=72, front=None, paws=PAWS, head=(0, -30), rate=RISE_DPS),
    Spec("mid", phi=40, h=None, hind_dx=68, front=None, paws=PAWS, head=(0, -30), hold=HOLD, rate=RISE_DPS),
    Spec("climb", phi=47, h=None, hind_dx=60, front=None, paws=PAWS, head=(0, -30), rate=RISE_DPS),
    Spec("top", phi=54, h=None, hind_dx=52, front=None, paws=PAWS, head=(0, -30), hold=HOLD, rate=RISE_DPS),
)


@dataclass(frozen=True)
class Stage:
    """One library entry: the way up to UP_SPECS[top], held there, and down."""
    number: int
    name: str
    top: int            # index into UP_SPECS of the held frame
    stop_ticks: int     # TRICK_STOP's crossfade to SIT, ticks
    margin_min: float   # the tripod margin at the hold the check holds it to, mm (worst over the ranges)


# TRICK_STOP for these tricks (the remote's toggle, C7, B6, A7): crossfade
# to SIT over stop_ticks instead of the body's 10 (0.2 s would drop it).
STAGES = (
    Stage(1, "rear_up", top=5, stop_ticks=75, margin_min=5.0),
    Stage(2, "rear_up_2", top=7, stop_ticks=100, margin_min=5.0),
    Stage(3, "rear_up_3", top=9, stop_ticks=125, margin_min=3.0),
)
NAMES = tuple(st.name for st in STAGES)


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


def solve(spec: Spec, g: S.Geometry | None = None) -> S.Pose:
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
        bs, bk = _leg(g, False, spec.hind_dx, r - h + spec.hind_lift, spec.phi)
        if spec.front is not None:
            _, fz = S.to_world(2 * hx, 0.0, spec.phi)
            fs, fk = _leg(g, True, spec.front, r - (h + fz), spec.phi)
        else:
            fs, fk = spec.paws
    except DesignError as e:
        raise DesignError(f"{spec.name}: {e}") from None
    p = S.Pose(*(round(v) for v in (fs, fk, bs, bk, spec.tail, spec.head[0], spec.head[1])))
    for name, j, v in (("front shoulder", 8, p.front_shoulder), ("front knee", 12, p.front_knee),
                       ("back shoulder", 10, p.back_shoulder), ("back knee", 14, p.back_knee),
                       ("tail", 2, p.tail), ("head tilt", 1, p.head_tilt)):
        lo, hi = JOINT_LIMITS[j]
        if not lo <= v <= hi:
            raise DesignError(f"{spec.name}: {name} {v} outside {lo}..{hi}")
    return p


def up_specs(st=1) -> tuple:
    """The way up for a stage: UP_SPECS to its top, only the top held."""
    st = stage(st)
    return tuple(replace(s, hold=s.hold if i == st.top else 0) for i, s in enumerate(UP_SPECS[:st.top + 1]))


def up_poses(g: S.Geometry | None = None, st=1) -> list:
    return [solve(s, g) for s in up_specs(st)]


def sequence(g: S.Geometry | None = None, st=1) -> tuple[list, list, list]:
    """The whole motion's poses, holds and move rates: up, the hold, the
    same way down (each move back as slow as the move up it reverses), SIT."""
    specs = up_specs(st)
    ups = [solve(s, g) for s in specs]
    holds = [s.hold for s in specs]
    rates = [s.rate for s in specs]
    downs = ups[-2::-1]
    down_rates = rates[:0:-1]  # back from up[i + 1] to up[i] at up[i + 1]'s rate
    return ups + downs + [END_POSE], holds + [0] * len(downs) + [0], rates + down_rates + [MOVE_DPS]


def _dur(a: S.Pose, b: S.Pose, rate: float = MOVE_DPS) -> int:
    d = max(abs(x - y) for x, y in zip(a.astuple(), b.astuple()))
    return max(MIN_DUR, math.ceil(d / rate * 1000.0 / TICK_MS))


def keyframe(p: S.Pose, dur: int, hold: int = 0) -> Keyframe:
    return Keyframe.from_joints({
        "head_pan": p.head_pan, "head_tilt": p.head_tilt, "tail": p.tail,
        "lf_shoulder": p.front_shoulder, "rf_shoulder": p.front_shoulder,
        "rb_shoulder": p.back_shoulder, "lb_shoulder": p.back_shoulder,
        "lf_knee": p.front_knee, "rf_knee": p.front_knee,
        "rb_knee": p.back_knee, "lb_knee": p.back_knee,
    }, dur=dur, hold=hold, easing=Easing.COSINE)


def pose_of(kf: Keyframe) -> S.Pose:
    j = kf.joints
    return S.Pose(j["lf_shoulder"], j["lf_knee"], j["rb_shoulder"], j["rb_knee"], j["tail"],
                  head_pan=j["head_pan"], head_tilt=j["head_tilt"])


def build(g: S.Geometry | None = None, st=1) -> Trick:
    st = stage(st)
    poses, holds, rates = sequence(g, st)
    frames = []
    prev = STAND
    for p, h, rate in zip(poses, holds, rates):
        frames.append(keyframe(p, _dur(prev, p, rate), h))
        prev = p
    top = UP_SPECS[st.top]
    return Trick(
        name=st.name, frames=tuple(frames), end_posture=END, stop_ticks=st.stop_ticks,
        source={"kind": "authored", "design": "kilo.tricks.rear_up", "doc": "docs/rear-up-trick.md",
                "stage": st.number},
        notes=(
            f"authored (not a legacy skill): rear_up stage {st.number} of {len(STAGES)}: sit back on the "
            f"hind feet and the tail tip, front paws up, {top.phi:g} deg",
            "the tail stays at 0 (its lowest point, it swings sideways) as a strut; the body tips up onto it",
            "generated by python -m kilo.tricks.rear_up from world-space keyframes and kilo.tricks.sagittal.Geometry",
            "balance off, fall_ok off: the pitch stays under 70 deg nose up, the fall detector (75 deg) stays armed",
            f"end posture SIT; TRICK_STOP crossfades to it over stop_ticks ({st.stop_ticks})",
        ),
    ).validate()


def build_all(g: S.Geometry | None = None) -> dict:
    return {st.name: build(g, st) for st in STAGES}


# ---- the check ----------------------------------------------------------------------------------
# The unknowns and their ranges. RANGES is what the gate (the test,
# --check) holds the design to: the centre of mass +-10 mm both ways (the
# bound on the measurement and the split into parts), the tail within what
# a ruler and a protractor give around the owner's measurements of
# 2026-11-16 (+-3 mm, +-5 deg), and the outline (still ESTIMATES) +-4 mm.
# The tail's axis angle only matters when the tail turns, which the trick
# does not. PLAUSIBLE is wider, for the case that the tail
# measurements (docs/rear-up-trick.md section 2) are off; --report shows it.
RANGES = {
    "com_dx": (-10.0, 10.0),
    "com_dz": (-10.0, 10.0),
    "tail_len": (91.0, 97.0),
    "tail_droop_deg": (13.0, 23.0),
    "tail_pivot_x": (-73.5, -67.5),
    "tail_pivot_z": (21.0, 27.0),
    "tail_axis_deg": (45.0, 55.0),
    "rear_x": (-92.0, -84.0),
    "belly_z": (-28.0, -20.0),
}
CORNERS = ("com_dx", "com_dz", "tail_len", "tail_droop_deg")
PLAUSIBLE = {
    "com_dx": (-10.0, 10.0),
    "com_dz": (-10.0, 10.0),
    "tail_len": (85.0, 103.0),
    "tail_droop_deg": (8.0, 28.0),
    "tail_g": (3.0, 12.0),
    "tail_pivot_x": (-79.0, -62.0),
    "tail_pivot_z": (16.0, 32.0),
    "rear_x": (-96.0, -80.0),
    "belly_z": (-32.0, -16.0),
}
# What the check holds the design to.
PITCH_MAX = 70.0        # deg nose up, anywhere: the fall detector trips at 75
# A roll (the centre of mass passing a support, the body tipping onto the
# next one) is judged by how far the point that lands falls, mm:
BACK_DROP_MAX = 15.0    # backwards, onto the tail or the rear end: cut short while the hind feet
                        # step, with the centre of mass 10 mm back and up, the rear lands from 10 mm
FWD_DROP_MAX = 15.0     # forwards, onto the front feet, while playing
STOP_FWD_DROP_MAX = 40.0  # forwards when the trick is cut short (the paws come down)
# (the tripod margin at the hold, worst case, is each stage's Stage.margin_min)
LEG_GAP_MIN = 12.0      # mm between the front and the hind leg of a side
# Servo torque. ESTIMATE: Petoi's P1S is sold as 3 kg cm stall (retailers;
# MG92B-class servos are similar). Static holding is kept under half of it.
STALL_KG_CM = 3.0
TORQUE_MAX_KG_CM = 0.5 * STALL_KG_CM


@dataclass
class Result:
    """One geometry, one motion."""
    label: str
    back_roll: float = 0.0        # the largest roll backwards, deg
    fwd_roll: float = 0.0
    back_drop: float = 0.0        # ...and how far its landing point fell, mm
    fwd_drop: float = 0.0
    pitch_max: float = -90.0
    forbidden: set = field(default_factory=set)
    hold_margin: float = 1e9      # the tripod's margin at the hold (the nearer of back and front), -1 off it
    hold_pitch: float = 0.0
    lateral: float | None = None  # at the hold, mm
    torque: dict = field(default_factory=dict)
    pi_clear: float = 1e9         # lowest the Pi or the frame's back corner gets
    hold_tripod: bool = False     # at the hold, on the hind feet and the tail alone
    end_phi: float = 0.0          # where it rests at the end
    end_supports: tuple = ()
    problems: list = field(default_factory=list)
    notes: list = field(default_factory=list)  # reported, not failures


def _run(g: S.Geometry, poses: Sequence[S.Pose], start: S.Pose, label: str, hold_index: int | None,
         steps: int = 24) -> Result:
    # (steps per keyframe: a roll is found up to a sample late, so the drops
    # move by about 1 mm between 24 and 48 steps; 16 overstates them by 3)
    res = Result(label)
    for smp in S.follow(g, poses, start=start, steps=steps, start_phi=_start_phi(start)):
        r = smp.rest
        for a, way, _onto, drop in r.rolls:
            if way == "backward":
                res.back_roll = max(res.back_roll, a)
                res.back_drop = max(res.back_drop, drop)
            else:
                res.fwd_roll = max(res.fwd_roll, a)
                res.fwd_drop = max(res.fwd_drop, drop)
        res.pitch_max = max(res.pitch_max, r.phi)
        res.forbidden |= set(r.contacts) & S.FORBIDDEN
        res.pi_clear = min(res.pi_clear, r.clear[S.PI], r.clear[S.BACK])
        L = smp.load
        for k in ("hind_shoulder", "hind_knee", "front_shoulder", "front_knee", "tail"):
            res.torque[k] = max(res.torque.get(k, 0.0), getattr(L, k))
        res.end_phi, res.end_supports = r.phi, tuple(sorted(r.contacts))
        if hold_index is not None and smp.frame == hold_index and smp.s == 1.0:
            res.hold_tripod = {r.rear, r.front} == {S.TAIL, S.HIND_FOOT}
            res.hold_margin = r.margin if res.hold_tripod else -1.0  # -1: not on the tripod
            res.hold_pitch = r.phi
            res.lateral = S.lateral_margin(g, r, smp.pose)
    return res


def _start_phi(p: S.Pose) -> float:
    return 28.0 if p == SIT else 0.0  # SIT rests on the hind and the front feet, 28 deg nose up


def _judge(res: Result, kind: str, margin_min: float) -> Result:
    """kind: "play" (everything), "stop" (a cut-short: no forbidden
    contact, no roll back; a roll forward onto the front feet is the way
    down and only reported), "info" (reported only)."""
    if kind == "info":
        return res
    if res.forbidden:
        res.problems.append(f"touches the floor with {sorted(res.forbidden)}")
    if res.back_drop > BACK_DROP_MAX:
        res.problems.append(f"rolls back {res.back_roll:.1f} deg, a {res.back_drop:.0f} mm drop (max {BACK_DROP_MAX:g})")
    if res.pitch_max > PITCH_MAX:
        res.problems.append(f"pitches {res.pitch_max:.1f} deg nose up (max {PITCH_MAX:g})")
    if S.TAIL in res.end_supports or S.FRONT_FOOT not in res.end_supports:
        res.problems.append(f"ends on {list(res.end_supports)} at {res.end_phi:.1f} deg, not down on its feet")
    if kind == "play":
        if res.fwd_drop > FWD_DROP_MAX:
            res.problems.append(f"rolls forward {res.fwd_roll:.1f} deg, a {res.fwd_drop:.0f} mm drop (max {FWD_DROP_MAX:g})")
        if not res.hold_tripod:
            # failing safe: the centre of mass is too far forward for the
            # paws to come up, so Kilo stays on four feet (any roll forward
            # on the way is judged above); the test holds the nominal
            # geometry and every single range end to the tripod
            res.notes.append(f"no tripod at the hold ({res.hold_pitch:.1f} deg): the paws stayed down")
        elif res.hold_margin < margin_min:
            res.problems.append(f"hold margin {res.hold_margin:.1f} mm (min {margin_min:g})")
    elif res.fwd_drop > STOP_FWD_DROP_MAX:
        res.problems.append(f"drops forward {res.fwd_roll:.1f} deg, {res.fwd_drop:.0f} mm (max {STOP_FWD_DROP_MAX:g})")
    return res


def geometries(ranges: dict | None = None, corners: Sequence[str] = CORNERS,
               base: S.Geometry | None = None) -> list[tuple[str, S.Geometry]]:
    """The nominal geometry, one at each end of every range, and every
    corner of the ranges named in ``corners``."""
    ranges = RANGES if ranges is None else ranges
    base = base or S.Geometry()
    out = [("nominal", base)]
    for k, (lo, hi) in ranges.items():
        out.append((f"{k}={lo:g}", base.with_(**{k: lo})))
        out.append((f"{k}={hi:g}", base.with_(**{k: hi})))
    for vals in itertools.product(*(ranges[k] for k in corners)):
        kw = dict(zip(corners, vals))
        out.append((",".join(f"{k}={v:g}" for k, v in kw.items()), base.with_(**kw)))
    return out


def photo_filter(geos) -> tuple[list, list]:
    """(kept, excluded): the geometries in which the photographed pose
    stands as the photo shows it (sagittal.photo_check), and the others,
    with the reason."""
    kept, excluded = [], []
    for label, g in geos:
        pc = S.photo_check(g)
        if pc.ok:
            kept.append((label, g))
        else:
            where = (f"{pc.margin:.1f} mm behind" if pc.margin >= 0 else f"{-pc.margin:.1f} mm in front of")
            excluded.append((label, f"the photo's pose would stand at {pc.phi:.1f} deg with the centre of mass "
                                    f"{where} the hind feet"))
    return kept, excluded


def stage_geometries(st, ranges: dict | None = None, corners: Sequence[str] = CORNERS) -> tuple[list, list]:
    """(kept, excluded): what a stage is checked over: the geometries of
    the ranges that the photo allows (the corner "centre of mass forward
    and low", which the photo rules out, is what limited the first
    design). The same for every stage: the photo is evidence about Kilo,
    not about one stage."""
    return photo_filter(geometries(ranges, corners))


def catch(g: S.Geometry, p: S.Pose, phi: float) -> tuple[S.Rest, S.Rest]:
    """A forward failure at pose p: Kilo pushed (or wobbling) forward over
    its hind feet until the front paws touch. Returns (the rest as
    designed, the rest from there): the second should be on the hind and
    the front feet (the paws catch it) or back on the tripod, never on the
    nose (it went on over the paws)."""
    r = S.rest(g, p, phi)
    pts = S.outline(g, p)
    land = r.phi
    while land > -80.0:  # turn nose down over the hind feet until something else meets the floor
        land -= 0.25
        w = [(name, S.to_world(x, z, land)[1]) for name, x, z in pts]
        foot = min(z for name, z in w if name == S.HIND_FOOT)
        if any(z <= foot + 0.01 for name, z in w if name != S.HIND_FOOT):
            break
    return r, S.rest(g, p, land)


# The body's SIT row as the crossfade targets it (body/src/core/posture.cpp):
# the back shoulders at -90, which the safety stage clamps to their -80 limit.
SIT_RAW = S.Pose(30, 60, -90, 45, -60, head_pan=10, head_tilt=-20)
BACK_SHOULDER_MIN = JOINT_LIMITS[10][0]


def stop_path(at: S.Pose) -> list[S.Pose]:
    """The body's crossfade from ``at`` to SIT as straight joint-space
    pieces: every joint on the line to SIT_RAW, the back shoulders held at
    -80 once they reach it."""
    if at.back_shoulder <= BACK_SHOULDER_MIN:
        return [END_POSE]
    f = (at.back_shoulder - BACK_SHOULDER_MIN) / (at.back_shoulder - SIT_RAW.back_shoulder)
    return [at.lerp(SIT_RAW, f), END_POSE]


def motions(trick: Trick, rest: bool = False) -> list[tuple[str, S.Pose, list[S.Pose], int | None, str]]:
    """(label, start pose, poses, index of the held frame, kind for _judge)
    for the trick from STAND and from SIT, and for every way it can be cut
    short half way to and at each keyframe: "stop" is the crossfade to the
    end posture (TRICK_STOP, the link watchdog's stop, any epoch bump);
    "rest" is a reflex taking the joints to REST from there (the low-battery
    reflex; the fall reflex only does it after a real fall), reported only."""
    poses = [pose_of(f) for f in trick.frames]
    hold_i = max(range(len(trick.frames)), key=lambda i: trick.frames[i].hold)
    out = [("play from STAND", STAND, poses, hold_i, "play"), ("play from SIT", SIT, poses, hold_i, "play")]
    for i in range(len(poses) - 1):
        for k in (0.5, 1.0):  # half way to keyframe i, and at it
            prev = STAND if i == 0 else poses[i - 1]
            at = prev.lerp(poses[i], k)
            path = poses[:i] + [at]
            tag = f"{i}{'' if k == 1.0 else '-'}"
            out.append((f"stop at {tag}", STAND, path + stop_path(at), None, "stop"))
            if rest:
                out.append((f"rest at {tag}", STAND, path + [REST], None, "info"))
    return out


@dataclass
class Report:
    results: list = field(default_factory=list)  # (geometry label, motion label, Result)
    excluded: list = field(default_factory=list)  # (geometry label, why): ruled out by the photo
    catch: tuple = ()  # nominal: (paw height at the hold mm, supports after a forward tip, its pitch)

    @property
    def problems(self) -> list[str]:
        return [f"{g} / {m}: {p}" for g, m, r in self.results for p in r.problems]

    def worst(self, attr: str, motion_prefix: str = "", fn=max):
        """The worst value of ``attr`` (one of the hold's over the plays held on the tripod)."""
        held = attr in ("hold_margin", "hold_pitch", "lateral")
        vals = [(getattr(r, attr), g, m) for g, m, r in self.results if m.startswith(motion_prefix)
                and (r.hold_tripod or not held)]
        return fn(vals) if vals else None

    def torque_max(self) -> dict:
        out: dict = {}
        for g, m, r in self.results:
            for k, v in r.torque.items():
                if v > out.get(k, (0.0,))[0]:
                    out[k] = (v, g, m)
        return out


def _check_one(job) -> list:
    trick, glabel, g, rest, margin_min = job
    return [(glabel, mlabel, _judge(_run(g, poses, start, mlabel, hold_i), kind, margin_min))
            for mlabel, start, poses, hold_i, kind in motions(trick, rest)]


def check(trick: Trick | None = None, geos=None, rest: bool = False) -> Report:
    """Every motion (and with ``rest`` the REST reflex paths, reported only)
    on every geometry (by default the stage's: stage_geometries)."""
    trick = trick or build()
    st = stage(trick.name)
    rep = Report()
    if geos is None:
        geos, rep.excluded = stage_geometries(st)
    jobs = [(trick, glabel, g, rest, st.margin_min) for glabel, g in geos]
    if len(jobs) > 4 and (os.cpu_count() or 1) > 1:
        # one geometry per task, over the cores (each is ~20 independent runs)
        with ProcessPoolExecutor(max_workers=os.cpu_count()) as ex:
            for part in ex.map(_check_one, jobs, chunksize=4):
                rep.results += part
    else:
        for job in jobs:
            rep.results += _check_one(job)
    nominal = S.Geometry()
    hold_i = max(range(len(trick.frames)), key=lambda i: trick.frames[i].hold)
    held, tipped = catch(nominal, pose_of(trick.frames[hold_i]), UP_SPECS[st.top].phi)
    rep.catch = (held.clear[S.FRONT_FOOT], tuple(sorted(tipped.contacts)), tipped.phi)
    if set(tipped.contacts) & S.FORBIDDEN or S.HIND_FOOT not in tipped.contacts:
        rep.results.append(("nominal", "catch", Result("catch", problems=[
            f"a forward tip at the hold ends on {sorted(tipped.contacts)}, not caught by the paws"])))
    for k, (v, g, m) in rep.torque_max().items():
        if g == "nominal" and v > TORQUE_MAX_KG_CM:
            rep.results.append(("nominal", m, Result(m, problems=[f"{k} {v:.2f} kg cm (max {TORQUE_MAX_KG_CM})"])))
    for i, f in enumerate(trick.frames):
        gap = S.leg_gap(nominal, pose_of(f))
        if gap < LEG_GAP_MIN:
            rep.results.append(("nominal", f"frame {i}", Result(f"frame {i}", problems=[f"legs {gap:.0f} mm apart"])))
    return rep


def summary(rep: Report, trick: Trick) -> str:
    lines = []
    secs = trick.duration_ticks * TICK_MS / 1000
    lines.append(f"{trick.name}: {len(trick.frames)} frames, {secs:.1f} s at 1x, end {trick.end_posture.name}, "
                 f"stop_ticks {trick.stop_ticks}")
    for i, f in enumerate(trick.frames):
        p = pose_of(f)
        lines.append(f"  {i:2d} dur {f.dur:3d} hold {f.hold:3d}  front {p.front_shoulder:4.0f} {p.front_knee:4.0f}  "
                     f"hind {p.back_shoulder:4.0f} {p.back_knee:4.0f}  tail {p.tail:4.0f}  head {p.head_pan:3.0f} {p.head_tilt:4.0f}")
    nom = [r for g, m, r in rep.results if g == "nominal" and m == "play from STAND"][0]
    lines.append(f"nominal: hold at {nom.hold_pitch:.1f} deg nose up, tripod margin {nom.hold_margin:.1f} mm, "
                 f"lateral {nom.lateral if nom.lateral is None else round(nom.lateral, 1)} mm, "
                 f"rolls back {nom.back_roll:.1f} / forward {nom.fwd_roll:.1f} deg, Pi {nom.pi_clear:.0f} mm up at the lowest")
    lines.append("torque, kg cm (worst over every geometry and motion; stall ~%.1f):" % STALL_KG_CM)
    for k, (v, g, m) in sorted(rep.torque_max().items()):
        lines.append(f"  {k:15} {v:.2f}  ({g}, {m})")
    for attr, fn, what in (("hold_margin", min, "tripod margin at the hold, mm"),
                           ("hold_pitch", max, "hold pitch, deg"), ("lateral", min, "sideways margin at the hold, mm"),
                           ("pitch_max", max, "pitch anywhere, deg"), ("back_roll", max, "roll back, deg"),
                           ("back_drop", max, "drop backwards, mm"), ("fwd_roll", max, "roll forward, deg"),
                           ("fwd_drop", max, "drop forwards, mm"), ("pi_clear", min, "Pi above the floor, mm")):
        w = rep.worst(attr, "play" if attr in ("hold_margin", "hold_pitch", "lateral") else "", fn)
        lines.append(f"worst {what}: {w[0]:.1f} ({w[1]}, {w[2]})")
    if rep.catch:
        lines.append(f"a forward tip at the hold: the paws fall from {rep.catch[0]:.0f} mm and Kilo ends on "
                     f"{list(rep.catch[1])} at {rep.catch[2]:.1f} deg")
    plays = [r for g, m, r in rep.results if m.startswith("play")]
    lines.append(f"held on the tripod in {sum(r.hold_tripod for r in plays)} of {len(plays)} plays")
    if rep.excluded:
        lines.append(f"{len(rep.excluded)} geometries ruled out by the photo (not checked):")
        lines += [f"  {g}: {why}" for g, why in rep.excluded[:12]]
    lines.append(f"{len(rep.results)} runs, {len(rep.problems)} problems")
    lines += ["  " + p for p in rep.problems[:40]]
    notes = [f"{g} / {m}: {n}" for g, m, r in rep.results for n in r.notes]
    if notes:
        lines.append(f"{len(notes)} notes (failing safe)")
        lines += ["  " + n for n in notes[:20]]
    return "\n".join(lines)


def report_extra(trick: Trick) -> None:
    """--report: the wider grids, printed."""
    st = stage(trick.name)
    full_corners = ("com_dx", "com_dz", "tail_len", "tail_droop_deg", "tail_pivot_x", "tail_pivot_z")
    print(f"\n-- {st.name}: every corner of the tail and centre-of-mass ranges together (64; some are expected "
          "to fail safe: the paws stay down):")
    geos, excl = stage_geometries(st, RANGES, full_corners)
    full = check(trick, geos)
    full.excluded = excl
    print(summary(full, trick))
    print(f"\n-- {st.name}: the PLAUSIBLE ranges (wider: in case the reading of the tail measurements is off):")
    geos, excl = stage_geometries(st, PLAUSIBLE, ("com_dx", "tail_len", "tail_droop_deg", "tail_pivot_z"))
    wide = check(trick, geos, rest=True)
    wide.excluded = excl
    print(summary(wide, trick))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Build and check the rear_up trick's three stages.")
    ap.add_argument("--check", action="store_true", help="only check: exit 1 if stale or failing")
    ap.add_argument("--report", action="store_true", help="the full uncertainty grid (slow)")
    ap.add_argument("--stage", type=int, choices=[st.number for st in STAGES], help="only this stage")
    args = ap.parse_args(argv)
    bad = False
    for st in STAGES:
        if args.stage is not None and st.number != args.stage:
            continue
        trick = build(None, st)
        rep = check(trick, rest=args.report)
        print(summary(rep, trick))
        if args.report:
            report_extra(trick)
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
