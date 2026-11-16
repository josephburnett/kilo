"""A planar (side view) static model of Kilo on flat ground, for tricks.

Given the 11 joint angles of a keyframe, where does Kilo rest, on what, how
far is its centre of mass from tipping, and what torque does each servo
hold? This is for tricks whose support is not the four feet (rear_up: the
hind feet and the tail), which the planner's model (kilo.planner.model)
does not cover. The masses and leg geometry come from that model; the tail
is the owner's measurement (2026-11-16, read as below); the outline of the
frame and the Pi are ESTIMATES (docs/rear-up-trick.md section 2 lists what
is still to measure), so every check runs over ranges of them.

Frames and conventions:

- Body frame as kilo.planner.model: origin midway between the shoulder
  axles, x forward, z up, mm. Left and right legs are taken as one (the
  tricks here are symmetric), so the model is planar: a rigid outline whose
  shape depends on the joint angles.
- World: X forward, Z up, the floor at Z = 0. ``phi`` is the body's pitch,
  NOSE UP positive (TELEMETRY pitch is the opposite sign: -phi):
  X = x cos(phi) - z sin(phi), Z = x sin(phi) + z cos(phi).
- Legs as body/src/core/ik.h: theta1 from straight down (+ = foot forward),
  theta2 the knee bend; OpenCat angles: front shoulder = -theta1, front knee
  = 90 - theta2; back shoulder = -theta1, back knee = -(theta2 + 90).
- The tail (kilo.tricks.sagittal.tail_point; MEASURED by the owner
  2026-11-16, the reading of the numbers is an INTERPRETATION, see
  docs/rear-up-trick.md §2): a straight 94 mm lever whose servo turns it
  LEFT AND RIGHT, not up and down. The servo's axis lies in the body's
  mid-plane and points backward and up, ``tail_axis_deg`` (40) above
  straight back; at servo 0 the tail points backward and ``tail_droop_deg``
  (18) below straight back. The tip therefore sweeps a cone about the axis
  (58 deg half-angle: a circle of radius 80 mm whose centre is 50 mm along
  the axis), LOWEST at 0 and rising to either side. In the side view the
  tail is a strut that cannot be driven down; turning it moves the tip
  sideways (``tail_sign``: +1 if a positive servo angle swings the tip to
  the left, +y; NOT KNOWN) and up.

Statics. A rigid body on flat ground rests on an edge of the convex hull of
its outline (in the side view) with the centre of mass above that edge. The
model follows a motion by continuity: at each sample it starts from the
hull edge whose ground orientation is nearest the previous one, and if the
centre of mass is past either end it rolls over that end onto the next edge
(an event: a drop of so many degrees, forward or backward), until it rests.
That is quasi-static: no inertia, no servo lag, rigid servos.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Sequence

from kilo.planner.model import BodyModel

# Forces are in grams-force and torques in g mm; 1 kg cm = 1000 g x 10 mm.
G_MM_PER_KG_CM = 10000.0

# Contacts: what may touch the floor. ALLOWED ones are supports a trick may
# use; the others mean the motion has gone wrong.
FRONT_FOOT, HIND_FOOT, TAIL, BUTT, HIND_KNEE, BACK, PI, NOSE, FRONT_KNEE = (
    "front_foot", "hind_foot", "tail", "butt", "hind_knee", "back", "pi", "nose", "front_knee")
ALLOWED = {FRONT_FOOT, HIND_FOOT, TAIL, BUTT, HIND_KNEE, FRONT_KNEE}
FORBIDDEN = {BACK, PI, NOSE}
TOUCH_MM = 1.0  # a point this close to the floor counts as a support


@dataclass(frozen=True)
class Geometry:
    """Everything the side view needs. Defaults: measured where
    kilo.planner.model is, the rest ESTIMATED (see each field)."""
    body: BodyModel = field(default_factory=BodyModel)
    # The centre of mass of the whole robot moved by this much (mm, body
    # frame), to cover the uncertainty of the measurement and of the split
    # into parts (the planner's model puts it 54 mm behind the front axles
    # and 3 mm below them; the walks read +1.7..2.0 deg nose down, which is
    # what a centre of mass a little forward of the model would do).
    com_dx: float = 0.0
    com_dz: float = 0.0
    # ---- the tail: MEASURED 2026-11-16 (the owner, a ruler; docs/rear-up-trick.md §2) ----
    tail_pivot_x: float = -70.5   # the tail servo's axis meets the mid-plane 9 mm behind the back shoulder axles
    tail_pivot_z: float = 24.0    # ...and 24 mm above them
    tail_len: float = 94.0        # pivot to the tip, straight line
    tail_droop_deg: float = 18.0  # at servo 0 the tail points this far below straight back
    tail_axis_deg: float = 40.0   # the servo's axis points backward and this far up (INTERPRETATION)
    tail_sign: float = 1.0        # +1: a positive servo angle swings the tip LEFT (+y). NOT KNOWN
    tail_g: float = 6.0           # the moving part (taken out of the frame's lump): ESTIMATE
    tail_com_frac: float = 0.5
    # ---- the outline: ESTIMATES (docs/rear-up-trick.md T3, T4) ----
    rear_x: float = -88.0         # the back end of the frame
    belly_z: float = -24.0        # the underside (battery pack)
    back_z: float = 34.0          # the top of the frame at the rear (the tail servo, its axis 24 mm up)
    pi_rear_x: float = -55.0      # the Pi's rear edge (board, connectors)
    pi_top_z: float = 36.0        # the Pi's highest point (header pins, USB/Ethernet)
    nose_x: float = 110.0
    nose_z: float = -5.0

    def with_(self, **kw) -> "Geometry":
        return replace(self, **kw)


@dataclass(frozen=True)
class Pose:
    """The joint angles of a planar pose, OpenCat degrees (a keyframe's legs
    as one pair). ``head_tilt`` is carried for the keyframes; the head's
    mass stays in the frame's lump (it moves the centre of mass < 1 mm)."""
    front_shoulder: float
    front_knee: float
    back_shoulder: float
    back_knee: float
    tail: float
    head_pan: float = 0.0
    head_tilt: float = 0.0

    def lerp(self, other: "Pose", s: float) -> "Pose":
        return Pose(*(a + (b - a) * s for a, b in zip(self.astuple(), other.astuple())))

    def astuple(self) -> tuple:
        return (self.front_shoulder, self.front_knee, self.back_shoulder, self.back_knee, self.tail,
                self.head_pan, self.head_tilt)


# ---- kinematics ------------------------------------------------------------------------------
def _rad(d: float) -> float:
    return math.radians(d)


def leg_points(g: Geometry, front: bool, shoulder: float, knee: float):
    """(hip, knee, foot) in the body frame for OpenCat angles."""
    hx = g.body.length_mm / 2 if front else -g.body.length_mm / 2
    t1 = _rad(-shoulder)
    t2 = _rad(90.0 - knee) if front else _rad(-knee - 90.0)
    l1, l2 = g.body.l1_mm, g.body.l2_mm
    kx, kz = hx + l1 * math.sin(t1), -l1 * math.cos(t1)
    fx, fz = kx + l2 * math.sin(t1 + t2), kz - l2 * math.cos(t1 + t2)
    return (hx, 0.0), (kx, kz), (fx, fz)


def _tail_basis(g: Geometry):
    """(a, u, ca, sa): the servo's axis a, the unit vector u from the axis
    toward the tail at servo 0 (both in the mid-plane, body frame (x, z)),
    and the cosine and sine of the cone's half-angle."""
    ax, az = -math.cos(_rad(g.tail_axis_deg)), math.sin(_rad(g.tail_axis_deg))
    dx, dz = -math.cos(_rad(g.tail_droop_deg)), -math.sin(_rad(g.tail_droop_deg))
    ca = ax * dx + az * dz
    sa = math.sqrt(max(0.0, 1.0 - ca * ca))
    ux, uz = (dx - ca * ax) / sa, (dz - ca * az) / sa
    return (ax, az), (ux, uz), ca, sa


def tail_point(g: Geometry, tail: float, frac: float = 1.0) -> tuple[float, float, float]:
    """The point ``frac`` of the way from the tail's pivot to its tip, body
    frame (x forward, y LEFT, z up), at servo angle ``tail``: the tail turns
    about its axis by tail_sign * tail degrees, so the tip goes round a cone
    and is lowest at 0."""
    (ax, az), (ux, uz), ca, sa = _tail_basis(g)
    s = _rad(g.tail_sign * tail)
    r = g.tail_len * frac
    c = r * ca
    w = r * sa
    return (g.tail_pivot_x + c * ax + w * math.cos(s) * ux,
            w * math.sin(s),
            g.tail_pivot_z + c * az + w * math.cos(s) * uz)


def tail_tip(g: Geometry, tail: float) -> tuple[float, float]:
    """The tip in the side view (x, z)."""
    x, _, z = tail_point(g, tail)
    return x, z


def tail_cone(g: Geometry) -> dict:
    """The sweep, for the docs: the circle the tip goes round (its radius
    and its centre, the distance along the axis), and the tip at 0 and at
    +-90 deg relative to the pivot and to the hind shoulder axles."""
    (ax, az), _, ca, sa = _tail_basis(g)
    hx = -g.body.length_mm / 2
    t0 = tail_point(g, 0.0)
    t90 = tail_point(g, 90.0)
    return {"radius": g.tail_len * sa, "centre_along_axis": g.tail_len * ca,
            "half_angle": math.degrees(math.acos(ca)),
            "tip0_from_pivot": (t0[0] - g.tail_pivot_x, t0[2] - g.tail_pivot_z),
            "tip0_from_hip": (t0[0] - hx, t0[2]),
            "tip90_from_pivot": (t90[0] - g.tail_pivot_x, t90[1], t90[2] - g.tail_pivot_z)}


def tail_touch_height(g: Geometry, phi: float, tail: float = 0.0) -> float:
    """The height of the hind shoulder axles above the floor at which the
    tail tip just touches it, the body pitched ``phi`` nose up."""
    hx = -g.body.length_mm / 2
    x, z = tail_tip(g, tail)
    return -to_world(x - hx, z, phi)[1]


def masses(g: Geometry, p: Pose) -> list[tuple[str, float, float, float]]:
    """(name, grams, x, z) lumps in the body frame; they sum to the model's total."""
    b = g.body
    out = []
    # The frame's lump is the planner's body lump less the tail at servo 0
    # (so that at STAND the centre of mass is the measured one), moved so
    # that the whole robot's centre of mass moves by (com_dx, com_dz).
    frame_g = b.body.grams - g.tail_g
    total = b.total_g
    t0x, _, t0z = tail_point(g, 0.0, g.tail_com_frac)
    fx = (b.body.grams * b.body.x - g.tail_g * t0x) / frame_g
    fz = (b.body.grams * b.body.z - g.tail_g * t0z) / frame_g
    out.append(("frame", frame_g, fx + g.com_dx * total / frame_g, fz + g.com_dz * total / frame_g))
    out.append(("battery", b.battery.grams, b.battery.x, b.battery.z))
    out.append(("pi", b.pi.grams, b.pi.x, b.pi.z))
    for front, sh, kn in ((True, p.front_shoulder, p.front_knee), (False, p.back_shoulder, p.back_knee)):
        (hx, hz), (kx, kz), (fx, fz) = leg_points(g, front, sh, kn)
        tag = "front" if front else "hind"
        out.append((f"{tag}_upper", 2 * b.upper_leg_g, hx + (kx - hx) * b.upper_leg_at, hz + (kz - hz) * b.upper_leg_at))
        out.append((f"{tag}_lower", 2 * b.lower_leg_g, kx + (fx - kx) * b.lower_leg_at, kz + (fz - kz) * b.lower_leg_at))
    tx, _, tz = tail_point(g, p.tail, g.tail_com_frac)
    out.append(("tail", g.tail_g, tx, tz))
    return out


def com_y(g: Geometry, p: Pose) -> float:
    """The centre of mass's sideways offset (mm, + left): only the tail moves it."""
    ty = tail_point(g, p.tail, g.tail_com_frac)[1]
    return g.tail_g * (ty - tail_point(g, 0.0, g.tail_com_frac)[1]) / g.body.total_g


def com(g: Geometry, p: Pose) -> tuple[float, float]:
    ms = masses(g, p)
    m = sum(w for _, w, _, _ in ms)
    return sum(w * x for _, w, x, _ in ms) / m, sum(w * z for _, w, _, z in ms) / m


def outline(g: Geometry, p: Pose) -> list[tuple[str, float, float]]:
    """Labelled points that may touch the floor (the feet as discs)."""
    pts: list[tuple[str, float, float]] = []
    r = g.body.foot_r_mm
    for front, sh, kn in ((True, p.front_shoulder, p.front_knee), (False, p.back_shoulder, p.back_knee)):
        _, (kx, kz), (fx, fz) = leg_points(g, front, sh, kn)
        foot = FRONT_FOOT if front else HIND_FOOT
        for i in range(12):
            a = 2 * math.pi * i / 12
            pts.append((foot, fx + r * math.cos(a), fz + r * math.sin(a)))
        pts.append((FRONT_KNEE if front else HIND_KNEE, kx, kz))
    tx, tz = tail_tip(g, p.tail)
    pts.append((TAIL, tx, tz))
    pts.append((BUTT, g.rear_x, g.belly_z))
    pts.append((BACK, g.rear_x, g.back_z))
    pts.append((PI, g.pi_rear_x, g.pi_top_z))
    pts.append((NOSE, g.nose_x, g.nose_z))
    return pts


def to_world(x: float, z: float, phi_deg: float) -> tuple[float, float]:
    c, s = math.cos(_rad(phi_deg)), math.sin(_rad(phi_deg))
    return x * c - z * s, x * s + z * c


# ---- statics ---------------------------------------------------------------------------------
def _hull(pts: Sequence[tuple[str, float, float]]) -> list[tuple[str, float, float]]:
    """Convex hull, counter-clockwise (x right, z up), collinear points dropped."""
    P = sorted(set(pts), key=lambda q: (q[1], q[2]))
    if len(P) < 3:
        return list(P)

    def cross(o, a, b):
        return (a[1] - o[1]) * (b[2] - o[2]) - (a[2] - o[2]) * (b[1] - o[1])

    lower: list = []
    for q in P:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 1e-9:
            lower.pop()
        lower.append(q)
    upper: list = []
    for q in reversed(P):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 1e-9:
            upper.pop()
        upper.append(q)
    return lower[:-1] + upper[:-1]


def _edge_phi(a, b) -> float:
    """The pitch that puts hull edge a->b (counter-clockwise) on the floor."""
    dx, dz = b[1] - a[1], b[2] - a[2]
    nx, nz = dz, -dx  # outward normal of a CCW polygon
    phi = -90.0 - math.degrees(math.atan2(nz, nx))
    return (phi + 180.0) % 360.0 - 180.0


def _ang(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


@dataclass(frozen=True)
class Rest:
    """Where the body rests for one pose."""
    phi: float                 # pitch, nose up +, degrees
    rear: str                  # the rear support (smaller X)
    front: str                 # the front support
    x_rear: float              # world X of the supports and the centre of mass (mm)
    x_front: float
    x_com: float
    z_com: float
    rolls: tuple = ()          # (degrees, "forward"/"backward", onto, drop mm) for each roll on the way here
    clear: dict = field(default_factory=dict)  # height above the floor of each outline point (mm)
    contacts: dict = field(default_factory=dict)  # world X of each support touching the floor

    @property
    def margin_back(self) -> float:
        """How far the centre of mass is ahead of the rear support (mm; < 0 = tips back)."""
        return self.x_com - self.x_rear

    @property
    def margin_front(self) -> float:
        return self.x_front - self.x_com

    @property
    def margin(self) -> float:
        return min(self.margin_back, self.margin_front)

    @property
    def supports(self) -> tuple[str, str]:
        return self.rear, self.front

    def tip_back_deg(self) -> float:
        """How far the body can lean back about the rear support before the
        centre of mass passes over it (deg; the static stability angle)."""
        h = self.z_com
        return math.degrees(math.atan2(self.margin_back, h))

    def tip_front_deg(self) -> float:
        return math.degrees(math.atan2(self.margin_front, self.z_com))


def rest(g: Geometry, p: Pose, phi_prev: float = 0.0, max_rolls: int = 12) -> Rest:
    """The resting state for pose p, starting from the hull edge nearest phi_prev."""
    pts = outline(g, p)
    h = _hull(pts)
    n = len(h)
    edges = [(i, _edge_phi(h[i], h[(i + 1) % n])) for i in range(n)]
    cx, cz = com(g, p)
    i, phi = min(edges, key=lambda e: _ang(e[1], phi_prev))
    rolls = []
    for _ in range(max_rolls):
        a, b = h[i], h[(i + 1) % n]
        xa, _ = to_world(a[1], a[2], phi)
        xb, _ = to_world(b[1], b[2], phi)
        xc, _ = to_world(cx, cz, phi)
        # vertex i or i+1 is the rear support; over it the body rolls onto
        # the neighbouring edge (i-1 shares vertex i, i+1 shares vertex i+1)
        a_is_rear = xa <= xb
        x_rear, x_front = (xa, xb) if a_is_rear else (xb, xa)
        if xc < x_rear - 1e-9:
            way, over_a = "backward", a_is_rear
        elif xc > x_front + 1e-9:
            way, over_a = "forward", not a_is_rear
        else:
            break
        pivot = a if over_a else b
        j, onto = ((i - 1) % n, h[(i - 1) % n]) if over_a else ((i + 1) % n, h[(i + 2) % n])
        nphi = edges[j][1]
        deg = _ang(nphi, phi)
        # how far the landing point falls: its distance from the pivot x sin(angle)
        drop = math.hypot(onto[1] - pivot[1], onto[2] - pivot[2]) * math.sin(math.radians(deg))
        rolls.append((round(deg, 1), way, onto[0], round(drop, 1)))
        i, phi = j, nphi
    a, b = h[i], h[(i + 1) % n]
    za = to_world(a[1], a[2], phi)[1]
    zb = to_world(b[1], b[2], phi)[1]
    floor = min(za, zb)
    xc, zc = to_world(cx, cz, phi)
    # The support: every point within TOUCH_MM of the floor (a third
    # contact between the edge's two ends, as when the front feet are about
    # to lift, widens it; the hull edge alone would not see it).
    clear: dict = {}
    touching = []
    for name, x, z in pts:
        X, Z = to_world(x, z, phi)
        h_ = round(Z - floor, 1)
        clear[name] = min(h_, clear.get(name, h_))
        if Z - floor <= TOUCH_MM:
            touching.append((X, name))
    (rxa, ra), (fxb, fb) = min(touching), max(touching)
    contacts: dict = {}
    for X, name in touching:
        contacts.setdefault(name, []).append(X)
    contacts = {k: sum(v) / len(v) for k, v in contacts.items()}
    return Rest(phi=phi, rear=ra, front=fb, x_rear=rxa, x_front=fxb, x_com=xc, z_com=zc - floor,
                rolls=tuple(rolls), clear=clear, contacts=contacts)


# ---- torques ---------------------------------------------------------------------------------
@dataclass(frozen=True)
class Load:
    """Static servo torques (kg cm, the larger of the two sides is the same:
    symmetric), and the share of the weight on each support."""
    hind_shoulder: float
    hind_knee: float
    front_shoulder: float
    front_knee: float
    tail: float
    on_rear: float  # grams on the rear support
    on_front: float


def _point_world(g, x, z, phi):
    return to_world(x, z, phi)[0]


def load(g: Geometry, p: Pose, r: Rest) -> Load:
    """Static torques for rest r. On two supports the weight is shared as a
    beam's; each leg of a pair takes half its pair's share; a support that
    is not a foot or the tail (the butt, a knee) takes its share through the
    frame. On three or more (statically indeterminate: it depends on the
    servos' compliance) every pair of supports that could carry the robot
    alone is tried and each joint gets its worst case."""
    labels = sorted(r.contacts.items(), key=lambda t: t[1]) or [(r.rear, r.x_rear), (r.front, r.x_front)]
    pairs = [(a, b) for i, a in enumerate(labels) for b in labels[i + 1:]
             if a[1] - 1e-6 <= r.x_com <= b[1] + 1e-6 and a[0] != b[0]]
    if not pairs:
        pairs = [((r.rear, r.x_rear), (r.front, r.x_front))]
    worst = None
    for (ra, xa), (fb, xb) in pairs:
        one = _load_pair(g, p, r, ra, xa, fb, xb)
        worst = one if worst is None else Load(*(max(u, v) for u, v in zip(worst.__dict__.values(), one.__dict__.values())))
    return worst


def _load_pair(g: Geometry, p: Pose, r: Rest, rear: str, x_rear: float, front: str, x_front: float) -> Load:
    W = g.body.total_g
    span = x_front - x_rear
    on_front = W * (r.x_com - x_rear) / span if span > 1e-6 else W / 2
    on_rear = W - on_front
    react = {rear: on_rear, front: on_front} if rear != front else {rear: W}
    phi = r.phi
    ms = {name: (w, x, z) for name, w, x, z in masses(g, p)}
    out = {}
    for front, sh, kn in ((True, p.front_shoulder, p.front_knee), (False, p.back_shoulder, p.back_knee)):
        tag = "front" if front else "hind"
        (hx, hz), (kx, kz), (fx, fz) = leg_points(g, front, sh, kn)
        foot = FRONT_FOOT if front else HIND_FOOT
        R = react.get(foot, 0.0) / 2  # per leg
        Xh, Xk, Xf = (_point_world(g, *q, phi) for q in ((hx, hz), (kx, kz), (fx, fz)))
        wu, ux, uz = ms[f"{tag}_upper"]
        wl, lx, lz = ms[f"{tag}_lower"]
        Xu, Xl = _point_world(g, ux, uz, phi), _point_world(g, lx, lz, phi)
        # moments of the floor force (up) and the link weights (down) about each axle
        knee = R * (Xf - Xk) - wl / 2 * (Xl - Xk)
        shoulder = R * (Xf - Xh) - wl / 2 * (Xl - Xh) - wu / 2 * (Xu - Xh)
        out[f"{tag}_knee"] = abs(knee) / G_MM_PER_KG_CM
        out[f"{tag}_shoulder"] = abs(shoulder) / G_MM_PER_KG_CM
    tail = _tail_torque(g, p, react.get(TAIL, 0.0), phi)
    return Load(hind_shoulder=out["hind_shoulder"], hind_knee=out["hind_knee"],
                front_shoulder=out["front_shoulder"], front_knee=out["front_knee"],
                tail=abs(tail) / G_MM_PER_KG_CM, on_rear=on_rear, on_front=on_front)


def _tail_torque(g: Geometry, p: Pose, on_tip: float, phi: float) -> float:
    """The torque about the tail servo's axis (g mm) from the floor pushing
    up on the tip (``on_tip`` grams) and the tail's own weight. World up, in
    the body frame, is (sin phi, 0, cos phi). Both forces lie in the
    mid-plane, as does the axis, when the tail is at 0: there the servo holds
    nothing and the load goes into its bearing; turned sideways, the load
    pushes the tip further out (0 is the lowest point of its circle)."""
    (ax, az), _, _, _ = _tail_basis(g)
    up = (math.sin(_rad(phi)), 0.0, math.cos(_rad(phi)))
    piv = (g.tail_pivot_x, 0.0, g.tail_pivot_z)

    def moment(pt, f):
        r = (pt[0] - piv[0], pt[1] - piv[1], pt[2] - piv[2])
        F = tuple(f * u for u in up)
        m = (r[1] * F[2] - r[2] * F[1], r[2] * F[0] - r[0] * F[2], r[0] * F[1] - r[1] * F[0])
        return m[0] * ax + m[2] * az

    return moment(tail_point(g, p.tail), on_tip) - moment(tail_point(g, p.tail, g.tail_com_frac), g.tail_g)


def lateral_margin(g: Geometry, r: Rest, p: Pose | None = None) -> float | None:
    """On the hind feet and the tail (a triangle on the floor: the feet
    track_mm apart, the tail tip where the tail's angle puts it, on the
    centre line at 0), how far the centre of mass is inside the triangle,
    mm (the nearest edge; < 0 outside). None for other supports."""
    if {r.rear, r.front} != {TAIL, HIND_FOOT}:
        return None
    ty = tail_point(g, p.tail)[1] if p is not None else 0.0
    cy = com_y(g, p) if p is not None else 0.0
    t = g.body.track_mm / 2
    tri = [(r.x_front, t), (r.x_front, -t), (r.x_rear, ty)] if r.front == HIND_FOOT else \
          [(r.x_rear, t), (r.x_rear, -t), (r.x_front, ty)]
    c = (r.x_com, cy)
    out = []
    for i in range(3):
        (x1, y1), (x2, y2) = tri[i], tri[(i + 1) % 3]
        ox, oy = tri[(i + 2) % 3]
        L = math.hypot(x2 - x1, y2 - y1)
        d = ((x2 - x1) * (c[1] - y1) - (y2 - y1) * (c[0] - x1)) / L
        side = ((x2 - x1) * (oy - y1) - (y2 - y1) * (ox - x1))
        out.append(d if side > 0 else -d)
    # the edge between the feet is the fore-aft margin (Rest.margin); this is the sideways one
    return min(out[1:])


def _seg_dist(p, q, a, b) -> float:
    """Shortest distance between segments pq and ab (2-D)."""
    def pt_seg(c, u, v):
        dx, dz = v[0] - u[0], v[1] - u[1]
        L2 = dx * dx + dz * dz
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((c[0] - u[0]) * dx + (c[1] - u[1]) * dz) / L2))
        return math.hypot(c[0] - u[0] - t * dx, c[1] - u[1] - t * dz)

    def cross(o, a_, b_):
        return (a_[0] - o[0]) * (b_[1] - o[1]) - (a_[1] - o[1]) * (b_[0] - o[0])

    d1, d2 = cross(a, b, p), cross(a, b, q)
    d3, d4 = cross(p, q, a), cross(p, q, b)
    if d1 * d2 < 0 and d3 * d4 < 0:
        return 0.0
    return min(pt_seg(p, a, b), pt_seg(q, a, b), pt_seg(a, p, q), pt_seg(b, p, q))


def leg_gap(g: Geometry, p: Pose) -> float:
    """Closest approach (mm) between the front and the hind leg of one side
    (they move in the same plane; the legs are beside the frame, so the
    frame itself is not in their way). The links are taken as lines; keep
    this above the links' width (about 12 mm)."""
    fh, fk, ff = leg_points(g, True, p.front_shoulder, p.front_knee)
    hh, hk, hf = leg_points(g, False, p.back_shoulder, p.back_knee)
    return min(_seg_dist(a, b, c, d) for a, b in ((fh, fk), (fk, ff)) for c, d in ((hh, hk), (hk, hf)))


# ---- following a motion ----------------------------------------------------------------------
@dataclass(frozen=True)
class Sample:
    frame: int
    s: float
    pose: Pose
    rest: Rest
    load: Load
    lateral: float | None


def follow(g: Geometry, poses: Sequence[Pose], start_phi: float = 0.0, steps: int = 20,
           start: Pose | None = None) -> list[Sample]:
    """Rest states along straight joint-space moves through ``poses`` (the
    keyframe player interpolates every joint with the same easing, so the
    path is a straight line in joint space whatever the easing). ``start``
    is the pose before frame 0 (frame 0 moves from it)."""
    out: list[Sample] = []
    phi = start_phi
    prev = start if start is not None else poses[0]
    r = rest(g, prev, phi)
    phi = r.phi
    out.append(Sample(-1, 1.0, prev, r, load(g, prev, r), lateral_margin(g, r, prev)))
    for k, target in enumerate(poses):
        for i in range(1, steps + 1):
            s = i / steps
            p = prev.lerp(target, s)
            r = rest(g, p, phi)
            phi = r.phi
            out.append(Sample(k, s, p, r, load(g, p, r), lateral_margin(g, r, p)))
        prev = target
    return out
