"""How Kilo rests on flat ground for a pose of his four feet, two ways.

RIGID (``rigid_rest``): the planner's model (kilo.planner.model) as the
walks use it: legs that do not give, the body on the plane through three
or four of its feet, the three that carry the centre of mass when one is
off the ground. This is "the model's prediction" the routine sends with
every pose (FEET pitch/roll) and records; kernel/mind/measure/script.cpp
ports it (kernel/host/tests/measure_vectors.txt checks the two agree).

COMPLIANT (``rest``): what the fit uses. Each leg is a spring along the
body's vertical: its foot sinks ``c`` mm per gram it carries, where ``c``
comes from the servos giving ``sag`` degrees per kg cm of torque (the
by-hand diagnosis of 2026-11-21 found about 3) through the leg's levers:

    c = sag' (x^2 + (L2 sin(t1 + t2))^2)

(x the foot's fore-aft offset from the shoulder, the shoulder servo's
lever; L2 sin(t1 + t2) the knee servo's; sag' the sag in radians per g mm).
The body's pose is the plane of kilo.planner.model,
z_i + r = H - a X_i - b Y_i for a foot on the ground, and the feet carry
F_i = max(0, -gap_i) / c_i; the rest is where the forces carry the weight
and its moments about the centre of mass (horizontal positions in the
world, so a tilted body's higher centre of mass leans over the feet). An
active-set solve per pose: the feet that push, the others in the air.
The feet's lengths may each be off by ``leg_dz`` mm (calibration), the
centre of mass off the model by ``com_d*``, and the IMU reads the body's
tilt plus an offset (applied by the caller, kilo.measure.fit).

Conventions as kilo.planner.model: body x forward, y left, z up, mm;
pitch nose DOWN +, roll RIGHT side up + (TELEMETRY's).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from kilo.planner import model as PM
from kilo.planner.model import LEGS, BodyModel

DEG = math.pi / 180.0
SAG_UNIT = DEG / 1.0e4      # 1 deg per kg cm in radians per g mm
SAG_MIN = 0.02              # deg per kg cm: the stiffest the solver takes (a rigid leg is indeterminate on 4 feet)
TOUCH_MM = 0.1              # rigid_rest: a foot this close to the support plane is on it


@dataclass(frozen=True)
class Body:
    """What the measurement can change about the model (the statics' part
    of kilo.measure.fit's parameters)."""
    com_dx: float = 0.0     # mm the centre of mass is from the model's (body frame)
    com_dy: float = 0.0
    com_dz: float = 0.0
    sag: float = 0.0        # deg per kg cm, every leg servo
    leg_dz: tuple = (0.0, 0.0, 0.0, 0.0)  # mm each leg is longer than commanded, LF RF RB LB
    model: BodyModel = field(default_factory=BodyModel)


# ---- the rigid model (the C++ routine's prediction) ---------------------------------------
def _plane_h(model: BodyModel, xs, zs, leg: int, pitch: float, roll: float) -> float:
    a, b = PM.plane_coeffs(pitch, roll)
    hx, hy = model.hip(leg)
    return zs[leg] + model.foot_r_mm + a * (hx + xs[leg]) + b * hy


def rigid_rest(model: BodyModel, xs, zs, prev: int = 0xF) -> tuple[float, float, int]:
    """(pitch, roll, support bits) the rigid model gives for feet (xs, zs):
    all four feet if they are on one plane (within TOUCH_MM); else the
    tripod he stood on the tick before (``prev``) while its fourth foot is
    above the floor and the centre of mass inside it (tipped onto a foot,
    he stays tipped: ``path``); else such a tripod, the one with the most
    margin (kernel/mind/measure/script.cpp rigid_rest)."""
    p4, r4 = PM.fit_attitude(model, xs, zs, 0xF)
    H = sum(_plane_h(model, xs, zs, leg, p4, r4) for leg in LEGS) / 4.0
    worst = max(abs(_plane_h(model, xs, zs, leg, p4, r4) - H) for leg in LEGS)
    if worst <= TOUCH_MM:
        return p4, r4, 0xF
    best = None
    for off in LEGS:
        sup = 0xF & ~(1 << off)
        p, r = PM.fit_attitude(model, xs, zs, sup)
        legs = [leg for leg in LEGS if leg != off]
        H = _plane_h(model, xs, zs, legs[0], p, r)
        # the fourth foot's leg reaches below the floor: this tripod is not how he stands
        if _plane_h(model, xs, zs, off, p, r) - H > TOUCH_MM:
            continue
        m = PM.stability(model, xs, zs, sup, p, r)
        if sup == prev and m >= 0:
            return p, r, sup
        if best is None or m > best[0]:
            best = (m, p, r, sup)
    if best is None:
        return p4, r4, 0xF
    return best[1], best[2], best[3]


def rigid_path(model: BodyModel, ticks) -> list[tuple[float, float, int]]:
    """rigid_rest along the script's ticks (each with .x, .z), the support carried over."""
    out = []
    prev = 0xF
    for t in ticks:
        p, r, prev = rigid_rest(model, t.x, t.z, prev)
        out.append((p, r, prev))
    return out


# ---- the compliant statics -------------------------------------------------------------------
class Poses:
    """Unique feet poses, prepared once: the feet's body-frame points, each
    leg's compliance lever (mm^2) and the model's centre of mass."""

    def __init__(self, model: BodyModel, xs, zs):
        self.model = model
        self.xs = np.asarray(xs, float).reshape(-1, 4)
        self.zs = np.asarray(zs, float).reshape(-1, 4)
        n = len(self.xs)
        self.hips = [model.hip(leg) for leg in LEGS]
        self.lever2 = np.empty((n, 4))
        self.com0 = np.empty((n, 3))
        for i in range(n):
            for leg in LEGS:
                t1, t2 = model.ik(leg, self.xs[i, leg], self.zs[i, leg])
                self.lever2[i, leg] = self.xs[i, leg] ** 2 + (model.l2_mm * math.sin(t1 + t2)) ** 2
            self.com0[i] = model.com(self.xs[i], self.zs[i])
        self.W = model.total_g

    def __len__(self) -> int:
        return len(self.xs)


@dataclass
class Rest:
    pitch: np.ndarray   # deg (N,)
    roll: np.ndarray
    H: np.ndarray       # mm: the support plane's height at the body origin
    loads: np.ndarray   # g (N, 4)
    support: np.ndarray  # int (N,): bits of the feet that push
    ok: np.ndarray      # bool (N,): the solve converged


def _rot_np(a, b):
    """R = Ry(pitch) Rx(-roll) for planes (a, b) (arrays), as (N, 3, 3)."""
    roll = np.arctan(b)
    pitch = np.arctan(a * np.cos(roll))
    cp, sp, cr, sr = np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    R = np.empty(a.shape + (3, 3))
    R[:, 0, 0], R[:, 0, 1], R[:, 0, 2] = cp, -sp * sr, sp * cr
    R[:, 1, 0], R[:, 1, 1], R[:, 1, 2] = 0.0, cr, sr
    R[:, 2, 0], R[:, 2, 1], R[:, 2, 2] = -sp, -cp * sr, cp * cr
    return R, pitch / DEG, roll / DEG


@dataclass
class Candidates:
    """Every pose's five candidate supports (all four feet, then each
    tripod: the foot CANDIDATE_OFF[j] in the air), solved as springs."""
    pitch: np.ndarray   # deg (N, 5)
    roll: np.ndarray
    H: np.ndarray
    loads: np.ndarray   # g (N, 5, 4)
    bad: np.ndarray     # (N, 5): how far it is from possible (a foot pulling, a foot in the air below the floor)

    def valid(self) -> np.ndarray:
        return self.bad < BAD_TOL


CANDIDATES = ((1, 1, 1, 1), (0, 1, 1, 1), (1, 0, 1, 1), (1, 1, 0, 1), (1, 1, 1, 0))
CANDIDATE_SUPPORT = tuple(sum(1 << i for i in range(4) if c[i]) for c in CANDIDATES)
BAD_TOL = 1e-3


def candidates(poses: Poses, body: Body = Body(), iters: int = 4) -> Candidates:
    """Each candidate support of every pose (module docstring): linear
    springs on the feet it has down, ``iters`` passes of the world
    positions (the tilt moves the centre of mass over the feet)."""
    m = poses.model
    n = len(poses)
    sag = max(body.sag, SAG_MIN)
    k = 1.0 / np.maximum(sag * SAG_UNIT * poses.lever2, 1e-6)  # g per mm
    hips = np.array(poses.hips)
    X = hips[None, :, 0] + poses.xs
    Y = np.broadcast_to(hips[None, :, 1], (n, 4))
    Lg = poses.zs + np.array(body.leg_dz, float)[None, :] + m.foot_r_mm
    com = poses.com0 + np.array([body.com_dx, body.com_dy, body.com_dz])[None, :]
    W = poses.W
    nc = len(CANDIDATES)
    out = Candidates(np.zeros((n, nc)), np.zeros((n, nc)), np.zeros((n, nc)), np.zeros((n, nc, 4)), np.zeros((n, nc)))
    for ci, cand in enumerate(CANDIDATES):
        act = np.broadcast_to(np.array(cand, bool)[None, :], (n, 4))
        kk = np.where(act, k, 0.0)
        a = np.zeros(n)
        b = np.zeros(n)
        H = Lg.mean(1)
        for _ in range(iters):
            R, _, _ = _rot_np(a, b)
            xw = R[:, 0, 0, None] * X + R[:, 0, 1, None] * Y - R[:, 0, 2, None] * Lg
            yw = R[:, 1, 0, None] * X + R[:, 1, 1, None] * Y - R[:, 1, 2, None] * Lg
            cx = np.einsum("nj,nj->n", R[:, 0, :], com)
            cy = np.einsum("nj,nj->n", R[:, 1, :], com)
            rows = np.stack([np.ones((n, 4)), xw - cx[:, None], yw - cy[:, None]], 1)   # (n, 3, 4)
            coef = np.stack([-kk, kk * X, kk * Y], 1)                                    # (n, 3, 4)
            M = np.einsum("nrl,ncl->nrc", rows, coef)
            v = -np.einsum("nrl,nl->nr", rows, kk * Lg)
            v[:, 0] += W
            sol = np.linalg.solve(M + np.eye(3)[None] * 1e-12, v[..., None])[..., 0]
            H, a, b = sol[:, 0], sol[:, 1], sol[:, 2]
        F = np.where(act, (Lg - H[:, None] + a[:, None] * X + b[:, None] * Y) * k, 0.0)
        gap = H[:, None] - a[:, None] * X - b[:, None] * Y - Lg
        out.bad[:, ci] = (np.where(act, np.maximum(-F, 0.0) * 1e-2, 0.0).sum(1)
                          + np.where(~act, np.maximum(-gap, 0.0), 0.0).sum(1))
        _, out.pitch[:, ci], out.roll[:, ci] = _rot_np(a, b)
        out.H[:, ci] = H
        out.loads[:, ci] = np.maximum(F, 0.0)
    return out


def path(c: Candidates, idx, start: int = 0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The rest along a sequence of poses (``idx``: the pose of each tick),
    with the hysteresis a lifted foot gives: the support of the tick before
    is kept while it is possible (tipped onto a foot, the centre of mass
    has moved past the edge and stays tipped until it comes back over it);
    else the possible one (the least impossible if none). Returns pitch,
    roll (deg) and the candidate index per tick."""
    idx = np.asarray(idx)
    valid = c.valid()
    best = np.argmin(c.bad, 1)
    state = start
    ch = np.empty(len(idx), int)
    for t, i in enumerate(idx):
        if not valid[i, state]:
            state = best[i]
        ch[t] = state
    return c.pitch[idx, ch], c.roll[idx, ch], ch


def rest(poses: Poses, body: Body = Body()) -> Rest:
    """The compliant rest of every pose on its own (no history: the
    possible support, all four feet first)."""
    c = candidates(poses, body)
    ch = np.argmin(c.bad, 1)
    r = np.arange(len(poses))
    loads = c.loads[r, ch]
    sup = ((loads > 0) * (1 << np.arange(4))[None, :]).sum(1)
    return Rest(c.pitch[r, ch], c.roll[r, ch], c.H[r, ch], loads, sup, c.bad[r, ch] < BAD_TOL)


# ---- the support edge a lifted foot tips over ---------------------------------------------------
# Lifting foot j leaves the tripod of the other three; the centre of mass crosses toward j over
# the edge between the two feet of the other diagonal: LF and RB tip over RF-LB, RF and LB over LF-RB.
EDGE = {PM.LF: (PM.RF, PM.LB), PM.RB: (PM.RF, PM.LB), PM.RF: (PM.LF, PM.RB), PM.LB: (PM.LF, PM.RB)}


def edge_distance(model: BodyModel, xs, zs, body: Body, lifted: int) -> float:
    """Signed distance (mm) of the centre of mass's projection from the
    edge foot ``lifted`` tips over, + on the lifted foot's side (it tips),
    with the body resting on the other three (compliant statics)."""
    sag = max(body.sag, SAG_MIN)
    X = [model.hip(leg)[0] + xs[leg] for leg in LEGS]
    Y = [model.hip(leg)[1] for leg in LEGS]
    L = [zs[leg] + body.leg_dz[leg] + model.foot_r_mm for leg in LEGS]
    c = []
    for leg in LEGS:
        t1, t2 = model.ik(leg, xs[leg], zs[leg])
        c.append(max(sag * SAG_UNIT * (xs[leg] ** 2 + (model.l2_mm * math.sin(t1 + t2)) ** 2), 1e-6))
    c0 = model.com(xs, zs)
    com = (c0[0] + body.com_dx, c0[1] + body.com_dy, c0[2] + body.com_dz)
    # the tripod alone (the lifted foot not allowed down), whatever the centre of mass does
    p, r = _tripod(X, Y, L, c, com, model.total_g, lifted)
    R = PM.rotation(p, r)
    a, b = EDGE[lifted]

    def world(i):
        return (R[0][0] * X[i] + R[0][1] * Y[i] - R[0][2] * L[i], R[1][0] * X[i] + R[1][1] * Y[i] - R[1][2] * L[i])

    A, B, J = world(a), world(b), world(lifted)
    cx = R[0][0] * com[0] + R[0][1] * com[1] + R[0][2] * com[2]
    cy = R[1][0] * com[0] + R[1][1] * com[1] + R[1][2] * com[2]
    ex, ey = B[0] - A[0], B[1] - A[1]
    n = math.hypot(ex, ey)

    def side(px, py):
        return (ex * (py - A[1]) - ey * (px - A[0])) / n

    return math.copysign(1.0, side(*J)) * side(cx, cy)


def _tripod(X, Y, L, c, com, W, lifted):
    """The attitude on the three feet other than ``lifted`` (the plane
    through their compressed lengths; the loads from the statics of the
    tripod, which is determinate)."""
    legs = [i for i in LEGS if i != lifted]
    a = b = 0.0
    for _ in range(4):
        pitch, roll = PM.attitude_of(a, b)
        R = PM.rotation(pitch, roll)
        P = [(R[0][0] * X[i] + R[0][1] * Y[i] - R[0][2] * L[i], R[1][0] * X[i] + R[1][1] * Y[i] - R[1][2] * L[i])
             for i in legs]
        cx = R[0][0] * com[0] + R[0][1] * com[1] + R[0][2] * com[2]
        cy = R[1][0] * com[0] + R[1][1] * com[1] + R[1][2] * com[2]
        # loads: sum F = W, sum F x = W cx, sum F y = W cy (barycentric)
        M = [[1.0, 1.0, 1.0], [P[0][0], P[1][0], P[2][0]], [P[0][1], P[1][1], P[2][1]]]
        s = PM._solve3(M, [W, W * cx, W * cy])
        if s is None:
            break
        Ls = [L[legs[k]] - c[legs[k]] * s[k] for k in range(3)]
        # the plane through the three compressed feet: L_i' = H - a X_i - b Y_i
        M2 = [[1.0, -X[i], -Y[i]] for i in legs]
        s2 = PM._solve3(M2, Ls)
        if s2 is None:
            break
        a, b = s2[1], s2[2]
    return PM.attitude_of(a, b)
