"""rear_up: Kilo sits up on its haunches, front paws off the floor, propped by the tail.

    python -m kilo.tricks.rear_up            write tricks/library/rear_up.json, print the check
    python -m kilo.tricks.rear_up --check    exit 1 if the JSON is stale or the check fails
    python -m kilo.tricks.rear_up --report   the check over the full uncertainty grid (slow)

The design (docs/rear-up-trick.md has the reasoning and the hardware
procedure). Kilo cannot balance on its hind feet: they are round (a 6 mm
disc), so a feet-only stance has no support length, and the centre of mass
at about 120 mm would have to be caught within about 1 deg with a 140 ms
servo lag against a fall time constant of about 110 ms. The tail is a few
grams, so as a counterweight it moves the centre of mass by 2 mm at most.
What works is a static TRIPOD, like a meerkat or a begging dog: the hind
feet stretched forward under the belly, the tail tip on the floor behind
(and the frame's rear end a few mm above it, as a backstop), the centre of
mass between them. Every keyframe and every point between keyframes is
statically stable in kilo.tricks.sagittal's model, over the ranges of the
unknowns in RANGES, so the trick plays open loop with the balance off.

The motion, from STAND (or SIT):
  0 tuck    the hind legs fold and swing forward, the tail goes down: the
            rear sinks onto the tail, the front feet stay down
  1 crouch  further: hind feet under the belly, body about 12 deg nose up
  2 settle  the hind feet stretched forward (84 mm ahead of their axles)
  3 ready   about 25 deg nose up on the tail, the hind feet and the front
            feet; the centre of mass is now behind the hind feet, so the
            front feet carry almost nothing
  4 paws    the front paws come up: the tripod alone
  5 rise    the hind shoulders turn the body up about the hind feet while
            the tail follows the floor
  6 up      about 60 deg nose up, paws tucked, head tilted down to look
            ahead: held 2 s
  then the same way down, ending in STAND (the end posture).

Keyframes are given as world-space intent (Spec: body pitch, hip height,
where the hind and front feet stand) and solved into joint angles for the
model's geometry, so new measurements (Geometry) re-solve the trick:
change the defaults in kilo.tricks.sagittal.Geometry and run this module.

The hold stays under 70 deg nose up (TELEMETRY pitch > -70) over every
range, so the body's fall detector (75 deg for 120 ms) stays armed: the
trick does NOT set FALL_OK, and a real fall backwards is still a fall.
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
# there in joint space from wherever it is. From the held pose the straight
# joint-space line to STAND keeps the front feet reaching the floor before
# the centre of mass passes the hind feet; the line to SIT (or REST) swings
# the tail down first and drops the front 25-30 deg onto the paws.
END = Posture.STAND
# TRICK_STOP for this trick (the remote's toggle, C7, B6, A7): crossfade to
# STAND over this many ticks instead of the body's 10 (0.2 s would drop it).
STOP_TICKS = 75

# The body's SIT and REST postures (body/src/core/posture.cpp; SIT's back
# shoulders -90 are clamped to the joint limit -80 by the safety stage).
SIT = S.Pose(30, 60, -80, 45, -60, head_pan=10, head_tilt=-20)
STAND = S.Pose(30, 30, -30, -30, 0)
REST = S.Pose(60, -45, -60, 45, -45, head_pan=-30, head_tilt=-80)
END_POSE = STAND

# Joint speed for the move durations: the average over a cosine move; its
# peak is pi/2 times this (63 deg/s), slow against the servos' 0.07 s/60 deg
# and long against the 140 ms lag, so the motion stays close to static.
MOVE_DPS = 40.0
MIN_DUR = 30  # ticks (0.6 s)


@dataclass(frozen=True)
class Spec:
    """One keyframe as intent. phi: body pitch, nose up, deg; h: the hind
    shoulder axles' height above the floor, mm; hind_dx: the hind feet's
    floor point ahead of the hind axles (world X, mm); front: the front
    feet's floor point ahead of the front axles, or None for paws up in
    ``paws`` (OpenCat shoulder, knee); tail_press: degrees the tail is
    turned further down than just touching, so it carries load; head:
    (pan, tilt); hold: ticks held after arriving."""
    name: str
    phi: float
    h: float
    hind_dx: float
    front: float | None = None
    paws: tuple = (75.0, 60.0)
    tail_press: float = 2.0
    head: tuple = (0.0, 0.0)
    hold: int = 0


# Chosen by searching each Spec's numbers against check() (the smallest
# rolls over the ranges); docs/rear-up-trick.md §3 has the reasoning.
UP_SPECS = (
    Spec("tuck", phi=9, h=56, hind_dx=20, front=0, tail_press=0, head=(0, -10)),
    Spec("crouch", phi=14, h=46, hind_dx=70, front=-5, head=(0, -15)),
    Spec("settle", phi=16, h=35, hind_dx=82, front=-12, head=(0, -20)),
    # the tail just touching here (no press): pressing would lift the rear
    # and put the centre of mass back over the hind feet, the liftoff's
    # tightest margin (20 mm nominal)
    Spec("ready", phi=25, h=37, hind_dx=84, front=-10, tail_press=0, head=(0, -25)),
    Spec("paws", phi=25, h=37, hind_dx=84, front=None, paws=(0.0, 60.0), tail_press=0, head=(0, -25)),
    Spec("rise", phi=42, h=40, hind_dx=80, front=None, paws=(45.0, 75.0), head=(0, -35)),
    Spec("up", phi=60, h=40, hind_dx=70, front=None, paws=(75.0, 60.0), head=(0, -45), hold=100),
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


def solve(spec: Spec, g: S.Geometry | None = None) -> S.Pose:
    """Joint angles (whole degrees, as the wire carries them) for a Spec."""
    g = g or S.Geometry()
    r = g.body.foot_r_mm
    hx = g.body.length_mm / 2
    bs, bk = _leg(g, False, spec.hind_dx, r - spec.h, spec.phi)
    if spec.front is not None:
        _, fz = S.to_world(2 * hx, 0.0, spec.phi)
        fs, fk = _leg(g, True, spec.front, r - (spec.h + fz), spec.phi)
    else:
        fs, fk = spec.paws
    # the tail tip on the floor, pointing back and down
    _, pz = S.to_world(g.tail_pivot_x + hx, g.tail_pivot_z, spec.phi)
    zp = spec.h + pz
    if not 0 < zp < g.tail_len:
        raise DesignError(f"{spec.name}: the tail cannot reach the floor (its axle is {zp:.0f} mm up)")
    elev = -math.degrees(math.asin(zp / g.tail_len))
    tail = (elev + spec.phi - g.tail_zero_deg) / g.tail_sign - spec.tail_press * g.tail_sign
    p = S.Pose(*(round(v) for v in (fs, fk, bs, bk, tail, spec.head[0], spec.head[1])))
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
    """The whole motion's poses and holds: up, the hold, the same way down, STAND."""
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
            "authored (not a legacy skill): sit up on the haunches, front paws up, the tail as the rear prop",
            "generated by python -m kilo.tricks.rear_up from world-space keyframes and kilo.tricks.sagittal.Geometry",
            "balance off, fall_ok off: the hold stays under 70 deg nose up, the fall detector (75 deg) stays armed",
            f"end posture STAND; TRICK_STOP crossfades to it over stop_ticks ({STOP_TICKS})",
        ),
    ).validate()


# ---- the check ----------------------------------------------------------------------------------
# The unknowns and their ranges (docs/rear-up-trick.md §2 says how each is
# measured). RANGES is what the gate (the test, --check) holds the design
# to: the centre of mass +-10 mm both ways (the task's bound on the
# measurement and the split), and the tail and the outline to within what
# a ruler and a protractor give, around the values in sagittal.Geometry.
# Those values are ESTIMATES until the owner measures them (M1-M5): before
# that, PLAUSIBLE is the spread of what they could be, and --report shows
# which of its corners the design as solved for the estimates fails.
RANGES = {
    "com_dx": (-10.0, 10.0),
    "com_dz": (-10.0, 10.0),
    "tail_len": (55.0, 65.0),
    "tail_zero_deg": (25.0, 35.0),
    "tail_pivot_x": (-90.0, -82.0),
    "tail_pivot_z": (2.0, 10.0),
    "rear_x": (-92.0, -84.0),
    "belly_z": (-28.0, -20.0),
}
PLAUSIBLE = {
    "com_dx": (-10.0, 10.0),
    "com_dz": (-10.0, 10.0),
    "tail_len": (45.0, 80.0),
    "tail_zero_deg": (15.0, 45.0),
    "tail_g": (3.0, 12.0),
    "tail_pivot_x": (-96.0, -76.0),
    "tail_pivot_z": (-4.0, 16.0),
    "rear_x": (-96.0, -80.0),
    "belly_z": (-32.0, -16.0),
}
# What the check holds the design to.
PITCH_MAX = 70.0        # deg nose up, anywhere: the fall detector trips at 75
# A roll (the centre of mass passing a support, the body tipping onto the
# next one) is judged by how far the point that lands falls, mm:
BACK_DROP_MAX = 15.0    # backwards, onto the tail or the rear end: sitting down (frame 1) with the
                        # centre of mass 10 mm further back, the tail tip lands from 10-12 mm
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
    hold_margin: float = 1e9      # min over the held frame of the tripod's back and front margins
    hold_pitch: float = 0.0
    lateral: float | None = None  # at the hold, mm
    torque: dict = field(default_factory=dict)
    pi_clear: float = 1e9         # lowest the Pi or the frame's back corner gets
    problems: list = field(default_factory=list)


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
        if hold_index is not None and smp.frame == hold_index and smp.s == 1.0:
            res.hold_margin = r.margin
            res.hold_pitch = r.phi
            res.lateral = S.lateral_margin(g, r)
    return res


def _start_phi(p: S.Pose) -> float:
    return 22.0 if p == SIT else 0.0


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
    if kind == "play":
        if res.fwd_drop > FWD_DROP_MAX:
            res.problems.append(f"rolls forward {res.fwd_roll:.1f} deg, a {res.fwd_drop:.0f} mm drop (max {FWD_DROP_MAX:g})")
        if res.hold_margin < TRIPOD_MARGIN:
            res.problems.append(f"hold margin {res.hold_margin:.1f} mm (min {TRIPOD_MARGIN:g})")
    elif res.fwd_drop > STOP_FWD_DROP_MAX:
        res.problems.append(f"drops forward {res.fwd_roll:.1f} deg, {res.fwd_drop:.0f} mm (max {STOP_FWD_DROP_MAX:g})")
    return res


def geometries(ranges: dict | None = None, corners: Sequence[str] = ("com_dx", "com_dz", "tail_len", "tail_zero_deg"),
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
            out.append((f"stop at {tag}", STAND, path + [END_POSE], None, "stop"))
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
        vals = [(getattr(r, attr), g, m) for g, m, r in self.results if m.startswith(motion_prefix)]
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
    for attr, fn, what in (("hold_margin", min, "hold margin, mm"), ("hold_pitch", max, "hold pitch, deg"),
                           ("pitch_max", max, "pitch anywhere, deg"), ("back_roll", max, "roll back, deg"),
                           ("fwd_roll", max, "roll forward, deg"), ("pi_clear", min, "Pi above the floor, mm")):
        w = rep.worst(attr, "play" if attr.startswith("hold") else "", fn)
        lines.append(f"worst {what}: {w[0]:.1f} ({w[1]}, {w[2]})")
    lines.append(f"{len(rep.results)} runs, {len(rep.problems)} problems")
    lines += ["  " + p for p in rep.problems[:40]]
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
        print("\n-- the PLAUSIBLE ranges (before the owner's measurements):")
        wide = check(trick, geometries(PLAUSIBLE, corners=("com_dx", "tail_len", "tail_zero_deg", "tail_pivot_z")),
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
