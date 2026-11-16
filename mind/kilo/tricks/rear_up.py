"""rear_up: Kilo sits back on its hind feet and its tail, front paws off the floor.

    python -m kilo.tricks.rear_up            write tricks/library/rear_up.json, print the check
    python -m kilo.tricks.rear_up --check    exit 1 if the JSON is stale or the check fails
    python -m kilo.tricks.rear_up --report   the check over the full uncertainty grid (slow)

The design (docs/rear-up-trick.md has the reasoning and the hardware
procedure). Kilo cannot balance on its hind feet: they are round (a 6 mm
disc), so a feet-only stance has no support length, and the servos answer
too slowly to balance it actively. What works is a static TRIPOD: the hind
feet stretched forward under the belly, the tail tip on the floor behind,
the centre of mass between them.

The tail (measured 2026-11-16, kilo.tricks.sagittal) turns LEFT AND RIGHT,
not down: at servo 0 it points back and 18 deg down and its tip is at its
lowest, 98 mm behind and 5 mm below the hind shoulder axles. So the tail
stays at 0 as a fixed strut, and the tripod is reached by tipping the body
nose up until the tip meets the floor: that happens when the hind axles
are 98 sin(pitch) + 5 cos(pitch) mm up (sagittal.tail_touch_height). The
pitch is then set by how high the hind legs hold the hips. Above about 55
deg the hind feet cannot reach far enough forward to stay ahead of the
centre of mass, and above about 35 deg the margin left for an uncertain
centre of mass is gone, so the hold is at 28 deg: a low sit-up, not a
meerkat. Every keyframe and every point between keyframes is statically
stable in sagittal's model over the ranges in RANGES, so the trick plays
open loop with the balance off.

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
            floor at the hold: if the centre of mass is further forward
            than the model, Kilo tips onto them, a short drop)
  5 up      the hind legs lift the hips: 28 deg nose up, held 2 s
  then the same way down to the tuck, and SIT (the end posture).

Keyframes are given as world-space intent (Spec: body pitch, hip height or
"on the tail", where the hind and front feet stand) and solved into joint
angles for the model's geometry, so new measurements (Geometry) re-solve
the trick: change the defaults in kilo.tricks.sagittal.Geometry and run
this module.

The hold stays far under 70 deg nose up, so the body's fall detector (75
deg for 120 ms) stays armed: the trick does NOT set FALL_OK.
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from dataclasses import dataclass, field
from typing import Sequence

from kilo.planner.model import LF, RB

from . import sagittal as S
from .model import JOINT_LIMITS, Easing, Keyframe, Posture, Trick, TICK_MS
from .library import path_for

NAME = "rear_up"

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
# TRICK_STOP for this trick (the remote's toggle, C7, B6, A7): crossfade to
# SIT over this many ticks instead of the body's 10 (0.2 s would drop it).
STOP_TICKS = 75

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
MIN_DUR = 30  # ticks (0.6 s)


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
    floor (stepping); head: (pan, tilt); hold: ticks held after arriving."""
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


# Chosen by searching each Spec's numbers against check() (the margins and
# the smallest rolls over the ranges); docs/rear-up-trick.md sections 1 and 6
# have the reasoning.
UP_SPECS = (
    Spec("tuck", phi=8, h=70, hind_dx=25, front=0, head=(0, -10)),
    # the rear down onto the tail with the hind feet still under the hips:
    # the centre of mass stays ahead of them, so nothing rocks back
    Spec("sink", phi=24, h=None, hind_dx=36, front=-6, head=(0, -15)),
    # on the tail and the front feet, the hind feet step forward in the air
    Spec("step", phi=24, h=None, hind_dx=78, hind_lift=10, front=-6, head=(0, -20)),
    Spec("ready", phi=24, h=None, hind_dx=78, front=-6, head=(0, -20)),
    Spec("paws", phi=24, h=None, hind_dx=78, front=None, paws=(60.0, 20.0), head=(0, -25)),
    Spec("up", phi=28, h=None, hind_dx=78, front=None, paws=(60.0, 20.0), head=(0, -30), hold=100),
)


class DesignError(ValueError):
    pass


def _leg(g: S.Geometry, front: bool, dxw: float, dzw: float, phi: float) -> tuple[float, float]:
    """The foot's disc centre at world offset (dxw, dzw) from its shoulder
    axle -> OpenCat (shoulder, knee), through the planner model's IK."""
    xb, zb = S.to_world(dxw, dzw, -phi)
    if not g.body.reachable(xb, -zb):
        raise DesignError(f"foot ({xb:.0f}, {zb:.0f}) mm out of reach")
    t1, t2 = g.body.ik(LF if front else RB, xb, -zb)
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


def up_poses(g: S.Geometry | None = None) -> list[S.Pose]:
    return [solve(s, g) for s in UP_SPECS]


def sequence(g: S.Geometry | None = None) -> tuple[list[S.Pose], list[int]]:
    """The whole motion's poses and holds: up, the hold, the same way down, SIT."""
    ups = up_poses(g)
    holds = [s.hold for s in UP_SPECS]
    downs = ups[-2::-1]
    poses = ups + downs + [END_POSE]
    return poses, holds + [0] * len(downs) + [0]


def _dur(a: S.Pose, b: S.Pose) -> int:
    d = max(abs(x - y) for x, y in zip(a.astuple(), b.astuple()))
    return max(MIN_DUR, math.ceil(d / MOVE_DPS * 1000.0 / TICK_MS))


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


def build(g: S.Geometry | None = None) -> Trick:
    poses, holds = sequence(g)
    frames = []
    prev = STAND
    for p, h in zip(poses, holds):
        frames.append(keyframe(p, _dur(prev, p), h))
        prev = p
    return Trick(
        name=NAME, frames=tuple(frames), end_posture=END, stop_ticks=STOP_TICKS,
        source={"kind": "authored", "design": "kilo.tricks.rear_up", "doc": "docs/rear-up-trick.md"},
        notes=(
            "authored (not a legacy skill): sit back on the hind feet and the tail tip, front paws up, 28 deg",
            "the tail stays at 0 (its lowest point, it swings sideways) as a strut; the body tips up onto it",
            "generated by python -m kilo.tricks.rear_up from world-space keyframes and kilo.tricks.sagittal.Geometry",
            "balance off, fall_ok off: the pitch stays under 45 deg nose up, the fall detector (75 deg) stays armed",
            f"end posture SIT; TRICK_STOP crossfades to it over stop_ticks ({STOP_TICKS})",
        ),
    ).validate()


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
TRIPOD_MARGIN = 5.0     # mm: the centre of mass inside the tripod at the hold, worst case
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


def _judge(res: Result, kind: str) -> Result:
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
        elif res.hold_margin < TRIPOD_MARGIN:
            res.problems.append(f"hold margin {res.hold_margin:.1f} mm (min {TRIPOD_MARGIN:g})")
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


def check(trick: Trick | None = None, geos=None, rest: bool = False) -> Report:
    """Every motion (and with ``rest`` the REST reflex paths, reported only)
    on every geometry."""
    trick = trick or build()
    rep = Report()
    for glabel, g in geos or geometries():
        for mlabel, start, poses, hold_i, kind in motions(trick, rest):
            r = _run(g, poses, start, mlabel, hold_i)
            rep.results.append((glabel, mlabel, _judge(r, kind)))
    nominal = S.Geometry()
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
    plays = [r for g, m, r in rep.results if m.startswith("play")]
    lines.append(f"held on the tripod in {sum(r.hold_tripod for r in plays)} of {len(plays)} plays")
    lines.append(f"{len(rep.results)} runs, {len(rep.problems)} problems")
    lines += ["  " + p for p in rep.problems[:40]]
    notes = [f"{g} / {m}: {n}" for g, m, r in rep.results for n in r.notes]
    if notes:
        lines.append(f"{len(notes)} notes (failing safe)")
        lines += ["  " + n for n in notes[:20]]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Build and check the rear_up trick.")
    ap.add_argument("--check", action="store_true", help="only check: exit 1 if stale or failing")
    ap.add_argument("--report", action="store_true", help="the full uncertainty grid (slow)")
    args = ap.parse_args(argv)
    trick = build()
    rep = check(trick, rest=args.report)
    print(summary(rep, trick))
    if args.report:
        print("\n-- every corner of the tail and centre-of-mass ranges together (64; some are expected to fail "
              "safe: the paws stay down):")
        full = check(trick, geometries(corners=("com_dx", "com_dz", "tail_len", "tail_droop_deg", "tail_pivot_x",
                                                "tail_pivot_z")))
        print(summary(full, trick))
        print("\n-- the PLAUSIBLE ranges (wider: in case the reading of the tail measurements is off):")
        wide = check(trick, geometries(PLAUSIBLE, corners=("com_dx", "tail_len", "tail_droop_deg", "tail_pivot_z")),
                     rest=True)
        print(summary(wide, trick))
    path = path_for(NAME)
    text = trick.to_json()
    stale = not path.exists() or path.read_text() != text
    if args.check:
        if stale:
            print(f"stale: {path.name} (run python -m kilo.tricks.rear_up)")
        return 1 if stale or rep.problems else 0
    if stale:
        path.write_text(text)
        print(f"wrote {path.name}")
    return 1 if rep.problems else 0


if __name__ == "__main__":
    sys.exit(main())
