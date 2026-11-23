"""A 3-D static model of Kilo on flat ground: each leg on its own, the body
free to pitch AND roll, and the joint angles the servos really reach.

Why it exists (docs/rear-up-trick-results.md, 2026-11-17): the side-view
model (kilo.tricks.sagittal) takes the left and right legs of a pair as one
and the joint angles as given. Both were wrong for the rear_up trick:

- The body turns a joint angle into a servo pulse and clamps the pulse to
  500..2500 us (body/src/core/joints.cpp joint_counts). With Kilo's
  calibration (the EEPROM dump of 2026-10-10, body/test/test_kilo_eeprom.cpp)
  the right hind shoulder's pulse reaches 2500 us at -57.3 deg: any angle
  below that (rear_up's sink and step ask for -80 and -77, SIT for -80)
  leaves the right hind leg at -57.3 while the left one goes to -80. The
  legs of a pair are then not one leg, and the body stands on three feet.
- On three feet, or on two feet and the tail, what tips Kilo over is a
  diagonal or a side: a roll, which a side view cannot see.

This model takes each leg's own angles after the body's limits and the
pulse clamp (``applied``), puts the feet at their own side (``track_mm``
apart), the tail tip where its cone puts it (sagittal.tail_point, sideways
too), the frame as a box of corners, and finds how the rigid body rests:
on which points, at what pitch and roll, how far the centre of mass is from
the edge of the support polygon (the area between the points touching the
floor, seen from above), in every direction.

Statics. Quasi-static, rigid, no inertia, contacts that do not slide (the
model does not move the body along the floor; the contacts' positions under
the body come from the joint angles). Along a move the body keeps its
orientation from one sample to the next, is set down on its lowest point,
and then topples about the support edge (or point) the centre of mass is
outside of, by exactly the angle that brings the next point down to the
floor, until the centre of mass is over the support polygon. Each topple is
an event (its angle and how far the point that lands fell).

Frames: the body frame of kilo.planner.model (x forward, y LEFT, z up, mm,
origin midway between the shoulder axles). World: X forward, Y left, Z up,
the floor at Z = 0. Attitude: ``pitch`` NOSE UP positive (as
kilo.tricks.sagittal; TELEMETRY's pitch is the opposite sign), ``roll``
LEFT SIDE DOWN positive (as TELEMETRY: "right side up positive").
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Mapping, Sequence

import numpy as np

from kilo.calibration import angle_window
from kilo.planner.model import BodyModel, Outline

from . import sagittal as S
from .model import JOINT_IDS, JOINT_LIMITS

# ---- what the body does with a joint angle -------------------------------------------------------
# body/src/core/joints.cpp: servo = zero + dir * angle (degrees), counts =
# floor((a2p * servo + b * 1000) / 1000), clamped to 500..2500 (us). The
# zeros, slopes and b come from the legacy EEPROM (config_migrate): Kilo's
# dump of 2026-10-10 (body/test/test_kilo_eeprom.cpp:
# CZP, a2p 0x2ad3 = 10963 for every joint, b 60 x 10). If the owner
# recalibrates (kilo cal set ...), `kilo cal show` gives the new offsets and
# CZP = 90 + dir * mid + dir * offset.
# JOINT_DIR and the a2p, b and pulse range: kilo.calibration's table (angle_window).
KILO_CZP = BodyModel().czp  # by legacy joint id: the body model's (protocol/spec.toml [model.servo])

LEGS = ("lf", "rf", "rb", "lb")
FRONT = {"lf": True, "rf": True, "rb": False, "lb": False}
LEFT = {"lf": True, "rf": False, "rb": False, "lb": True}


def servo_reach(j: int, czp: Sequence[int] = KILO_CZP) -> tuple[float, float]:
    """(lo, hi): the joint angles (deg) between which joint j's pulse is not
    clamped. Outside them the servo stops at the end of the pulse range."""
    return angle_window(j, czp[j])


def reach(j: int, czp: Sequence[int] = KILO_CZP) -> tuple[float, float]:
    """The angles joint j really goes to: the body's joint limits AND the pulse range."""
    lo, hi = JOINT_LIMITS[j]
    slo, shi = servo_reach(j, czp)
    return max(lo, slo), min(hi, shi)


def applied(j: int, angle: float, czp: Sequence[int] = KILO_CZP) -> float:
    """The angle joint j ends at when asked for ``angle``."""
    lo, hi = reach(j, czp)
    return min(hi, max(lo, angle))


def joints_of(p) -> dict:
    """Joint name -> angle from a sagittal.Pose (both sides alike) or a mapping."""
    if isinstance(p, Mapping):
        return dict(p)
    return {"lf_shoulder": p.front_shoulder, "rf_shoulder": p.front_shoulder,
            "rb_shoulder": p.back_shoulder, "lb_shoulder": p.back_shoulder,
            "lf_knee": p.front_knee, "rf_knee": p.front_knee,
            "rb_knee": p.back_knee, "lb_knee": p.back_knee,
            "tail": p.tail, "head_pan": p.head_pan, "head_tilt": p.head_tilt}


def lerp(a: Mapping, b: Mapping, s: float) -> dict:
    return {k: a[k] + (b[k] - a[k]) * s for k in a}


# ---- the geometry --------------------------------------------------------------------------------
@dataclass(frozen=True)
class Geometry3:
    """The side view's geometry (sagittal.Geometry: legs, masses, tail,
    outline) plus what 3-D needs. ``czp``: the servo zeros (None: no pulse
    clamp, the angles as asked, which is what the side view assumed);
    ``offsets``: degrees added to joints after the clamp (a servo that sags
    under load, or a calibration that is off), joint name -> deg;
    ``tail_offset``: the tail servo's angle off what was asked (a tail not
    quite centred); ``half_width``: half the frame's width (ESTIMATE, for
    the corners that may touch in a roll); ``imu_pitch``: what TELEMETRY
    reads minus the model's pitch on four feet (the body model's IMU offset;
    nose up positive); ``com_dy``: the centre of
    mass this far to the left (mm; it was measured fore-aft and up-down
    only, and "taken as centred left to right")."""
    g: S.Geometry = field(default_factory=S.Geometry)
    czp: tuple | None = KILO_CZP
    offsets: tuple = ()
    tail_offset: float = 0.0
    half_width: float = Outline().half_width_mm
    imu_pitch: float = -BodyModel().imu_pitch_deg  # the model's IMU offset, nose up +
    com_dy: float = BodyModel().com_dy_mm

    @classmethod
    def of(cls, body: BodyModel, **kw) -> "Geometry3":
        """The 3-D geometry of ``body`` (the body model or one of its views):
        its servo zeros, width, centre-of-mass correction and IMU pitch
        offset (the model's is TELEMETRY's, nose down +; here nose up +)."""
        d = dict(g=S.Geometry.of(body), czp=body.czp, half_width=body.outline.half_width_mm,
                 imu_pitch=-body.imu_pitch_deg, com_dy=body.com_dy_mm)
        d.update(kw)
        return cls(**d)

    def with_(self, **kw) -> "Geometry3":
        return replace(self, **kw)

    def joint(self, js: Mapping, name: str) -> float:
        a = js[name]
        if self.czp is not None:
            a = applied(JOINT_IDS[name], a, self.czp)
        else:
            lo, hi = JOINT_LIMITS[JOINT_IDS[name]]
            a = min(hi, max(lo, a))
        for n, d in self.offsets:
            if n == name:
                a += d
        if name == "tail":
            a += self.tail_offset
        return a


# Contact labels.
FOOT = {leg: f"{leg}_foot" for leg in LEGS}
KNEE = {leg: f"{leg}_knee" for leg in LEGS}
TAIL = "tail"
FEET = set(FOOT.values())
ALLOWED = FEET | {TAIL, "butt_l", "butt_r"} | set(KNEE.values())


def _leg(g3: Geometry3, js: Mapping, leg: str):
    g = g3.g
    front = FRONT[leg]
    sh = g3.joint(js, f"{leg}_shoulder")
    kn = g3.joint(js, f"{leg}_knee")
    (hx, hz), (kx, kz), (fx, fz) = S.leg_points(g, front, sh, kn)
    y = g.body.track_mm / 2 * (1 if LEFT[leg] else -1)
    return (hx, y, hz), (kx, y, kz), (fx, y, fz)


def points(g3: Geometry3, js: Mapping) -> list[tuple[str, np.ndarray, float]]:
    """(label, body-frame point, radius): everything that may touch the floor."""
    g = g3.g
    r = g.body.foot_r_mm
    out = []
    for leg in LEGS:
        h, k, f = _leg(g3, js, leg)
        out.append((FOOT[leg], np.array(f), r))
        out.append((KNEE[leg], np.array(k), 0.0))
        out.append((f"{leg}_hip", np.array(h), 8.0))  # the shoulder servo's housing, beside the axle
    out.append((TAIL, np.array(S.tail_point(g, g3.joint(js, "tail"))), 0.0))
    w = g3.half_width
    for side, y in (("l", w), ("r", -w)):
        out.append((f"butt_{side}", np.array((g.rear_x, y, g.belly_z)), 0.0))
        out.append((f"back_{side}", np.array((g.rear_x, y, g.back_z)), 0.0))
        out.append((f"pi_{side}", np.array((g.pi_rear_x, y, g.pi_top_z)), 0.0))
        out.append((f"nose_{side}", np.array((g.nose_x, y * 0.5, g.nose_z)), 0.0))
        out.append((f"belly_{side}", np.array((g.body.length_mm / 2, y, g.belly_z)), 0.0))
        out.append((f"top_{side}", np.array((g.body.length_mm / 2, y, g.pi_top_z)), 0.0))
    return out


def masses(g3: Geometry3, js: Mapping) -> list[tuple[str, float, np.ndarray]]:
    """(name, grams, body-frame point); as sagittal.masses with each leg on its own side."""
    g = g3.g
    b = g.body
    frame_g = b.body.grams - g.tail_g
    total = b.total_g
    t0x, _, t0z = S.tail_point(g, 0.0, g.tail_com_frac)
    fx = (b.body.grams * b.body.x - g.tail_g * t0x) / frame_g
    fz = (b.body.grams * b.body.z - g.tail_g * t0z) / frame_g
    dx, dy, dz = g.com_dx, g3.com_dy, g.com_dz  # the centre-of-mass correction (the model's unless given)
    out = [("frame", frame_g, np.array((fx + dx * total / frame_g, dy * total / frame_g,
                                        fz + dz * total / frame_g))),
           ("battery", b.battery.grams, np.array((b.battery.x, 0.0, b.battery.z))),
           ("pi", b.pi.grams, np.array((b.pi.x, 0.0, b.pi.z)))]
    for leg in LEGS:
        h, k, f = (np.array(q) for q in _leg(g3, js, leg))
        out.append((f"{leg}_upper", b.upper_leg_g, h + (k - h) * b.upper_leg_at))
        out.append((f"{leg}_lower", b.lower_leg_g, k + (f - k) * b.lower_leg_at))
    out.append(("tail", g.tail_g, np.array(S.tail_point(g, g3.joint(js, "tail"), g.tail_com_frac))))
    return out


def com(g3: Geometry3, js: Mapping) -> np.ndarray:
    ms = masses(g3, js)
    m = sum(w for _, w, _ in ms)
    return sum(w * p for _, w, p in ms) / m


# ---- attitude ------------------------------------------------------------------------------------
def rot(pitch: float, roll: float) -> np.ndarray:
    """Body -> world for a nose-up ``pitch`` and a left-down ``roll`` (deg):
    the pitch about the body's y axis, then the roll about the world's x."""
    p, r = math.radians(pitch), math.radians(roll)
    Ry = np.array([[math.cos(p), 0.0, -math.sin(p)], [0.0, 1.0, 0.0], [math.sin(p), 0.0, math.cos(p)]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(r), math.sin(r)], [0.0, -math.sin(r), math.cos(r)]])
    return Rx @ Ry


def attitude(R: np.ndarray) -> tuple[float, float]:
    """(pitch nose up, roll left down), deg, of body -> world R (as the IMU
    reads them: the pitch from the body's x axis, the roll from its y axis)."""
    xb = R[:, 0]
    yb = R[:, 1]
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, xb[2]))))
    zb = R[:, 2]
    roll = math.degrees(math.atan2(-yb[2], zb[2]))
    return pitch, roll


def _axis_rot(axis: np.ndarray, ang: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def _hull2(pts: list) -> list:
    """2-D convex hull (CCW) of (x, y, label) tuples; collinear points dropped."""
    P = sorted(set(pts), key=lambda q: (q[0], q[1]))
    if len(P) <= 2:
        return P

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo: list = []
    for q in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], q) <= 1e-9:
            lo.pop()
        lo.append(q)
    up: list = []
    for q in reversed(P):
        while len(up) >= 2 and cross(up[-2], up[-1], q) <= 1e-9:
            up.pop()
        up.append(q)
    return lo[:-1] + up[:-1]


def _poly_margin(poly: list, c) -> tuple[float, tuple]:
    """Signed distance from c to the support polygon's boundary (> 0
    inside; a point or a segment has no inside) and the nearest feature:
    ("edge", a, b) or ("point", a), corners as (x, y, label)."""
    n = len(poly)
    if n == 1:
        return -math.hypot(c[0] - poly[0][0], c[1] - poly[0][1]), ("point", poly[0])
    edges = [(poly[i], poly[(i + 1) % n]) for i in range(n if n > 2 else 1)]
    best_out, feat_out = 1e18, None
    inside = n > 2
    best_in, feat_in = 1e18, None
    for a, b in edges:
        ex, ey = b[0] - a[0], b[1] - a[1]
        L2 = ex * ex + ey * ey
        t = ((c[0] - a[0]) * ex + (c[1] - a[1]) * ey) / L2
        if t <= 0:
            dist, feat = math.hypot(c[0] - a[0], c[1] - a[1]), ("point", a)
        elif t >= 1:
            dist, feat = math.hypot(c[0] - b[0], c[1] - b[1]), ("point", b)
        else:
            dist, feat = math.hypot(c[0] - a[0] - t * ex, c[1] - a[1] - t * ey), ("edge", a, b)
        if dist < best_out:
            best_out, feat_out = dist, feat
        if n > 2:
            d = (ex * (c[1] - a[1]) - ey * (c[0] - a[0])) / math.sqrt(L2)  # left of a->b (CCW): inside
            if d < 0:
                inside = False
            if d < best_in:
                best_in, feat_in = d, ("edge", a, b)
    if inside:
        return best_in, feat_in
    return -best_out, feat_out


@dataclass(frozen=True)
class Rest3:
    """Where the body rests for one pose."""
    R: np.ndarray
    pitch: float              # nose up +, deg (the model's; TELEMETRY nose up = pitch + imu_pitch)
    roll: float               # left side down +, deg
    contacts: dict            # label -> (X, Y) floor point of each support touching the floor
    support: tuple            # the support polygon's corners (X, Y, label), CCW
    com: tuple                # world (X, Y, Z) of the centre of mass, Z above the floor
    margin: float             # mm: the centre of mass inside the support polygon (< 0 never at rest)
    edge: tuple               # labels of the nearest edge's ends (where it would tip first)
    topples: tuple = ()       # (deg, onto label, drop mm) for each topple on the way here
    clear: dict = field(default_factory=dict)  # label -> height above the floor, mm

    @property
    def labels(self) -> set:
        return set(self.contacts)

    def on(self, *labels: str) -> bool:
        return all(lb in self.contacts for lb in labels)


TOUCH_MM = 0.5


def rest(g3: Geometry3, js: Mapping, R0: np.ndarray | None = None, max_topples: int = 40) -> Rest3:
    """The resting state of pose ``js`` starting from orientation R0 (the
    previous sample's; None: level)."""
    js = joints_of(js)
    pts = points(g3, js)
    labels = [lb for lb, _, _ in pts]
    P = np.array([p for _, p, _ in pts])          # body frame, N x 3
    rad = np.array([r for _, _, r in pts])
    cb = com(g3, js)
    R = rot(0.0, 0.0) if R0 is None else R0.copy()
    topples = []
    up = np.array((0.0, 0.0, 1.0))
    while True:
        W = P @ R.T
        floor = float(np.min(W[:, 2] - rad))
        W[:, 2] -= floor
        c = R @ cb
        c[2] -= floor
        idx = np.nonzero(W[:, 2] - rad <= TOUCH_MM)[0]
        sup = _hull2([(float(W[i, 0]), float(W[i, 1]), labels[i]) for i in idx])
        m, feat = _poly_margin(sup, (c[0], c[1]))
        if m >= 0 or len(topples) >= max_topples:
            break
        # Topple about the nearest support edge (or point), toward the centre of mass.
        cc = np.array((c[0], c[1], 0.0))
        p0 = np.array((feat[1][0], feat[1][1], 0.0))
        if feat[0] == "edge":
            axis = np.array((feat[2][0] - feat[1][0], feat[2][1] - feat[1][1], 0.0))
        else:
            d = cc - p0
            axis = np.array((-d[1], d[0], 0.0))
        axis = axis / np.linalg.norm(axis)
        # u1: horizontal, perpendicular to the axis, toward the centre of mass (that side goes down)
        u1 = np.cross(up, axis)
        if np.dot(u1, cc - p0) < 0:
            axis, u1 = -axis, -u1
        # The smallest rotation that brings another sphere down onto the floor: a
        # centre alpha along u1 and beta up from the pivot is at beta cos(t) -
        # alpha sin(t) after turning t, and touches when that is its radius.
        alpha = (W - p0) @ u1
        beta = W[:, 2]
        rho = np.hypot(alpha, beta)
        ok = (alpha > 1e-6) & (rho > rad)
        th = np.full(len(W), math.pi / 2)
        th[ok] = np.arccos(np.minimum(1.0, rad[ok] / rho[ok])) - np.arctan2(alpha[ok], beta[ok])
        th[th < 1e-7] = math.pi / 2
        i = int(np.argmin(th))
        best = float(th[i])
        Rt = _axis_rot(axis, best)
        if (Rt @ u1)[2] > 0:
            Rt = _axis_rot(axis, -best)
        R = Rt @ R
        topples.append((round(math.degrees(best), 1), labels[i], round(float(beta[i] - rad[i]), 1)))
    touch = {labels[i]: (float(W[i, 0]), float(W[i, 1])) for i in idx}
    edge = tuple(q[2] for q in feat[1:])
    hgt = W[:, 2] - rad
    clear = {lb: round(float(h), 1) for lb, h in zip(labels, hgt)}
    pitch, roll = attitude(R)
    return Rest3(R=R, pitch=pitch, roll=roll, contacts=touch, support=tuple(sup), com=tuple(float(v) for v in c),
                 margin=m, edge=edge, topples=tuple(topples), clear=clear)


@dataclass(frozen=True)
class Sample3:
    frame: int
    s: float
    joints: dict
    rest: Rest3


def follow(g3: Geometry3, frames: Sequence[Mapping], start: Mapping, steps: int = 20,
           R0: np.ndarray | None = None) -> list[Sample3]:
    """Rest states along straight joint-space moves (the body's keyframe
    player) from ``start`` through ``frames`` (joint names -> deg, as asked:
    the clamp is applied here)."""
    prev = joints_of(start)
    r = rest(g3, prev, R0)
    out = [Sample3(-1, 1.0, prev, r)]
    R = r.R
    for k, f in enumerate(frames):
        f = joints_of(f)
        for i in range(1, steps + 1):
            s = i / steps
            js = lerp(prev, f, s)
            r = rest(g3, js, R)
            R = r.R
            out.append(Sample3(k, s, js, r))
        prev = f
    return out


def telemetry_pitch(g3: Geometry3, r: Rest3) -> float:
    """The nose-up pitch TELEMETRY would show for rest r."""
    return r.pitch + g3.imu_pitch
