"""dynamics: Kilo as one compliant body on the floor, played keyframe by keyframe.

    python -m kilo.tricks.dynamics            the model against the recorded plays (fixture), per run
    python -m kilo.tricks.dynamics --check    the fast kinematics against kilo.tricks.stance

The model the rear_up trick is judged in (kilo.tricks.rear_up.check). One
rigid body in 3-D with the masses and the leg and tail geometry of
kilo.tricks.stance, on a flat floor:

- contacts: the four foot discs, the knees, the tail tip, the frame's
  corners (stance.points); each a spring-damper normal force and a
  stick-slip friction (a tangential spring to an anchor that slides when
  the force passes mu N);
- the servos: the keyframe player's commands per 20 ms tick (its easing, the
  body's 10 deg per tick safety slew, the pulse clamp for the calibration in
  ``Params.czp``), a dead time and a first-order response (50 + 70 ms, the
  planner's, fitted to the floor recordings of 2026-11-15), and a
  COMPLIANCE: each leg joint gives ``sag`` degrees per kg cm of the load its
  leg's contacts put on it;
- the moving legs move the centre of mass (the momentum is the whole
  robot's; the legs' own spin is left out);
- the IMU reads the body's pitch and roll plus ``imu_pitch`` / ``imu_roll``.

It does NOT model: the tail servo giving (at 0 it holds nothing), a foot
rolling on its disc, a soft floor, a servo running out of torque (stall ~3
kg cm, an ESTIMATE), backlash.

WHAT IT IS HELD TO (fixtures/rear_up_2026_11_21.json, the second design's
plays of 2026-11-19..21; ``validate``): every play that reached "gather"
rocked back onto the tail, the nose up passing +5 deg 0.5-0.6 s into the
0.6 s move and settling at +22.5..+23.1 with the roll -1..-2; at "lift_r"
(the right hind foot in the air) 3 of 7 plays went over the left-hind /
right-front diagonal (+12..+14 nose up, roll -9..-12) and 4 did not.

FITTED, and what it says about Kilo (2026-11-21):

- Kilo is compliant: under load each leg servo gives about 4 deg per kg
  cm (3.5..4.5 reproduce the rock's timing). On four feet that is a soft
  pendulum: as the load moves onto the hind legs they give, the body
  pitches up, which moves the centre of mass further back over the hind
  feet, which loads them more. A rigid model's margin is eaten by it: the
  second design's "gather" had 15 mm in the rigid model and rocked back.
- the centre of mass is about 5.5 mm further back and 5 mm further RIGHT
  than kilo.planner.model's (the lift_r knife edge ties the two: back or
  right both put it over the diagonal; 3 of 7 went over, so it sits on it);
- the tail droops 16 deg, not 18 (the pitch on the tail);
- the IMU reads +2 deg of roll (left side down) in every trick pose where
  the model is level: an offset or a left leg 2 mm short, which the data
  cannot tell apart; taken as an offset.

Not separated by the data: com_dx, com_dz and sag trade against each other
at gather (``bands`` has the ends of that line); com_dy against a left /
right difference in the legs; the servo lag against sag; the friction.

The numerics (2026-11-21): the joints' compliance moves the contact points;
that motion is NOT fed into the contact dampers (only the commanded joints'
motion is). Fed in, the stiff contact dampers and the compliance form a
loop that rings at the time step on a three-point stance (the loads summed
to 1.6 times the weight and the roll depended on the step).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from kilo.calibration import POSTURES
from kilo.planner.model import BodyModel, view_numbers

from . import model as M
from . import sagittal as S
from . import stance as T

G = 9810.0                        # mm/s^2
TICK_S = M.TICK_MS / 1000.0
JOINTS = tuple(M.JOINT_NAMES[j] for j in M.CANONICAL)   # the 11 driven joints, wire order
LEGS = T.LEGS                      # lf rf rb lb
G_MM_PER_KG_CM = 9810.0 * 10000.0  # g mm^2/s^2 per kg cm (a torque of 1 kg cm)
KIN_EVERY = 2                      # physics steps per kinematics update
# Every number about Kilo's body here is the body model's (kilo.planner.model.BodyModel, from
# protocol/spec.toml [model]); FITTED is its "tricks" view (mind/tests/test_one_model.py).
BODY = BodyModel()
SLEW_DEG_TICK = BODY.slew_deg_per_tick  # the body's safety slew (10 deg a tick)
FIXTURE = Path(__file__).resolve().parents[2] / "tests" / "fixtures" / "rear_up_2026_11_21.json"

# The body's postures (body/src/core/posture.cpp; kilo.calibration.POSTURES is the copy the tests
# tie to it) as joint dicts.
STAND = dict(zip(JOINTS, POSTURES["stand"]))
SIT = dict(zip(JOINTS, POSTURES["sit"]))

# The right hind shoulder's servo zero after its upper leg was re-seated (docs/rear-up-trick.md
# section 2: calibration offset +12, the range it may end in +7..+25): it reaches -80 like the left.
assert M.JOINT_IDS["rb_shoulder"] == 10  # BodyModel.czp_reseated's joint
DESIGN_CZP = BODY.czp_reseated


# ---- parameters -----------------------------------------------------------------------------------
@dataclass(frozen=True)
class Params:
    """What the model is uncertain about. Defaults: kilo.tricks.stance's
    nominal values (the re-seated calibration), stiff servos."""
    com_dx: float = BODY.com_dx_mm  # mm: the whole robot's centre of mass moved forward (body frame; the model's)
    com_dy: float = BODY.com_dy_mm  # ...to the left
    com_dz: float = BODY.com_dz_mm  # ...up
    sag: float = BODY.sag_deg_per_kg_cm  # deg per kg cm: every leg servo's compliance under load
    mu: float = 0.5              # friction, feet and tail on the floor
    lag_ms: float = BODY.servo_lag_ms  # servo dead time (the body model's, fitted 2026-11-15: 50)
    tau_ms: float = BODY.servo_tau_ms  # servo first-order time constant (ditto: 70)
    czp: tuple = DESIGN_CZP      # the servo zeros: the pulse clamp (stance.KILO_CZP: before the re-seat)
    tail_len: float = BODY.tail.len_mm
    tail_droop_deg: float = BODY.tail.droop_deg
    belly_z: float = BODY.outline.belly_z_mm  # the frame's underside at its rear corner (an ESTIMATE)
    offsets: tuple = ()          # (joint, deg): the joint really sits this far off what it is sent
    # TELEMETRY nose up minus the model's, deg (STAND reads -1.9): the "tricks" view's, the tricks' default too
    imu_pitch: float = -view_numbers("tricks")["imu_pitch_deg"]
    imu_roll: float = 0.0
    k_mm: float = 3.0e6          # contact stiffness, g/s^2 (3 N/mm)
    c_mm: float = 1.5e4          # contact damping, g/s

    @classmethod
    def of(cls, body: BodyModel, **kw) -> "Params":
        """The parameters as ``body`` (the body model or one of its views) has them: its
        centre-of-mass correction, the legs' give, the IMU's offsets (TELEMETRY's pitch is nose
        down +, this module's nose up +), the servo response, the tail and the underside; the
        servo zeros as re-seated."""
        d = dict(com_dx=body.com_dx_mm, com_dy=body.com_dy_mm, com_dz=body.com_dz_mm, sag=body.sag_deg_per_kg_cm,
                 lag_ms=body.servo_lag_ms, tau_ms=body.servo_tau_ms, czp=body.czp_reseated,
                 tail_len=body.tail.len_mm, tail_droop_deg=body.tail.droop_deg, belly_z=body.outline.belly_z_mm,
                 imu_pitch=-body.imu_pitch_deg, imu_roll=body.imu_roll_deg)
        d.update(kw)
        return cls(**d)

    def with_(self, **kw) -> "Params":
        return replace(self, **kw)

    def g3(self) -> T.Geometry3:
        g = S.Geometry(com_dx=self.com_dx, com_dz=self.com_dz, tail_len=self.tail_len,
                       tail_droop_deg=self.tail_droop_deg, belly_z=self.belly_z)
        return T.Geometry3(g=g, czp=self.czp, com_dy=self.com_dy, imu_pitch=self.imu_pitch)


NOMINAL = Params()
# Fitted to the recorded plays (``validate``; the module's docstring).
# The numbers are the body model's "tricks" view (protocol/spec.toml [model.views.tricks]: the
# centre of mass 5.5 mm back and 5 mm right, sag 4, the tail's droop 16, an IMU roll offset of 2):
# they disagree with the model's own and stay the tricks' until a self-measurement settles them.
FITTED = Params.of(BODY.view("tricks"))


def bands(p: Params | None = None) -> list[tuple[str, Params]]:
    """The fit and one thing off it at a time, to the edges of what the
    recordings allow and a little past: the corners a design must survive."""
    p = p or FITTED
    out = [("fitted", p)]
    for k, vals in (("com_dx", (-4.0, +4.0)), ("com_dy", (-3.0, +3.0)), ("com_dz", (-10.0, +10.0)),
                    ("sag", (-1.0, +1.5)), ("tail_droop_deg", (-4.0, +4.0)), ("tail_len", (-3.0, +3.0))):
        for d in vals:
            out.append((f"{k}{d:+g}", p.with_(**{k: getattr(p, k) + d})))
    out.append(("mu=0.3", p.with_(mu=0.3)))
    out.append(("mu=1.0", p.with_(mu=1.0)))
    out.append(("slow servos 70+100ms", p.with_(lag_ms=70.0, tau_ms=100.0)))
    out.append(("rear corner 10 mm higher", p.with_(belly_z=p.belly_z + 10.0)))
    # the gather fixes only a mix of the two: the ends of that line
    out.append(("stiffer, further back", p.with_(sag=p.sag - 1.0, com_dx=p.com_dx - 3.0)))
    out.append(("softer, further forward", p.with_(sag=p.sag + 1.0, com_dx=p.com_dx + 3.0)))
    # the rightward centre of mass with the softest legs
    out.append(("softer, further right", p.with_(sag=p.sag + 1.0, com_dy=p.com_dy - 3.0)))
    return out


# ---- kinematics (fast; checked against stance.points / stance.com by check_kinematics) --------------
class Kin:
    """Body-frame points and masses for joint angles, as kilo.tricks.stance
    puts them, but quick enough for every physics step."""

    def __init__(self, p: Params):
        self.p = p
        g3 = p.g3()
        self.g3 = g3
        g = g3.g
        self.g = g
        b = g.body
        self.l1, self.l2, self.r = b.l1_mm, b.l2_mm, b.foot_r_mm
        self.hx = b.length_mm / 2
        self.ty = b.track_mm / 2
        self.upper_g, self.lower_g = b.upper_leg_g, b.lower_leg_g
        self.upper_at, self.lower_at = b.upper_leg_at, b.lower_leg_at
        ms = T.masses(g3, STAND)
        self.fixed_m = [(w, np.array(q, float)) for n, w, q in ms if n in ("frame", "battery", "pi")]
        self.tail_g = g.tail_g
        pts = T.points(g3, STAND)
        self.fixed_pts = [(lb, np.array(q, float), rr) for lb, q, rr in pts
                          if not lb.startswith(("lf_", "rf_", "rb_", "lb_")) and lb != T.TAIL]
        self.labels = ([f"{leg}_foot" for leg in LEGS] + [f"{leg}_knee" for leg in LEGS] + [T.TAIL]
                       + [lb for lb, _, _ in self.fixed_pts])
        self.radius = np.array([self.r] * 4 + [0.0] * 4 + [0.0] + [rr for _, _, rr in self.fixed_pts])
        self.n = len(self.labels)
        self.fixed_P = np.array([q for _, q, _ in self.fixed_pts])
        # the frame lump as a box (the model's frame_box_mm, an ESTIMATE: 130 x 60 x 45 mm), the rest as points
        fw = self.fixed_m[0][0]
        bx, by, bz = b.frame_box_mm
        self.I_box = np.diag([fw / 12 * (by ** 2 + bz ** 2), fw / 12 * (bx ** 2 + bz ** 2),
                              fw / 12 * (bx ** 2 + by ** 2)])
        self.total = sum(w for w, _ in self.fixed_m) + 4 * (self.upper_g + self.lower_g) + self.tail_g
        self.reach = {name: T.reach(M.JOINT_IDS[name], p.czp) if p.czp is not None
                      else M.JOINT_LIMITS[M.JOINT_IDS[name]] for name in JOINTS}

    def clamp(self, js: Mapping) -> dict:
        """The angles the servos are sent after the joint limits and the pulse clamp."""
        return {k: min(self.reach[k][1], max(self.reach[k][0], js[k])) for k in JOINTS}

    def leg(self, leg: str, sh: float, kn: float):
        front = T.FRONT[leg]
        hx = self.hx if front else -self.hx
        t1 = math.radians(-sh)
        t2 = math.radians(90.0 - kn) if front else math.radians(-kn - 90.0)
        kx, kz = hx + self.l1 * math.sin(t1), -self.l1 * math.cos(t1)
        fx, fz = kx + self.l2 * math.sin(t1 + t2), kz - self.l2 * math.cos(t1 + t2)
        y = self.ty if T.LEFT[leg] else -self.ty
        return (hx, y, 0.0), (kx, y, kz), (fx, y, fz)

    def state(self, q: Mapping):
        """(points N x 3, centre of mass (3,), inertia about it 3 x 3) for
        actual joint angles q (no clamp applied here)."""
        feet, knees = [], []
        msum = np.zeros(3)
        mtot = 0.0
        pm = []
        for w, c in self.fixed_m:
            msum += w * c
            mtot += w
            pm.append((w, c))
        for leg in LEGS:
            h, k, f = self.leg(leg, q[f"{leg}_shoulder"], q[f"{leg}_knee"])
            feet.append(f)
            knees.append(k)
            h, k, f = np.array(h), np.array(k), np.array(f)
            u = h + (k - h) * self.upper_at
            lo = k + (f - k) * self.lower_at
            msum += self.upper_g * u + self.lower_g * lo
            mtot += self.upper_g + self.lower_g
            pm += [(self.upper_g, u), (self.lower_g, lo)]
        tc = np.array(S.tail_point(self.g, q["tail"], self.g.tail_com_frac))
        msum += self.tail_g * tc
        mtot += self.tail_g
        pm.append((self.tail_g, tc))
        c = msum / mtot
        I = self.I_box.copy()
        for w, x in pm:
            d = x - c
            I += w * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        P = np.vstack([np.array(feet), np.array(knees), np.array(S.tail_point(self.g, q["tail"]))[None, :],
                       self.fixed_P])
        return P, c, I

    def foot_jac(self, leg: str, sh: float, kn: float):
        """d(foot)/d(shoulder), d(foot)/d(knee), d(knee)/d(shoulder), body frame, per degree."""
        e = 0.5
        _, k0, f0 = self.leg(leg, sh, kn)
        _, k1, f1 = self.leg(leg, sh + e, kn)
        _, _, f2 = self.leg(leg, sh, kn + e)
        return ((np.array(f1) - np.array(f0)) / e, (np.array(f2) - np.array(f0)) / e,
                (np.array(k1) - np.array(k0)) / e)


# ---- the keyframe player -------------------------------------------------------------------------
def as_frames(frames) -> list[tuple[dict, int, int, int]]:
    """(joints, dur, hold, easing) from a Trick, a list of Keyframes, or a
    list of (joints, dur, hold[, easing]) / dicts with those keys. A joint a
    frame leaves out keeps the previous frame's angle."""
    if isinstance(frames, M.Trick):
        frames = frames.frames
    out = []
    for f in frames:
        if isinstance(f, M.Keyframe):
            out.append((f.joints, f.dur, f.hold, int(f.easing)))
        elif isinstance(f, Mapping):
            out.append((f["joints"], f["dur"], f.get("hold", 0), f.get("easing", int(M.Easing.COSINE))))
        else:
            js, dur, hold = f[:3]
            out.append((js, dur, hold, f[3] if len(f) > 3 else int(M.Easing.COSINE)))
    return out


def _ease(kind: int, s: float) -> float:
    if kind == int(M.Easing.LINEAR):
        return s
    if kind == int(M.Easing.COSINE):
        return (1 - math.cos(math.pi * s)) / 2
    return 1.0  # STEP


def starts(frames) -> np.ndarray:
    """When each frame's move starts, and the end, s after the play's first tick."""
    return np.cumsum([0] + [d + h for _, d, h, _ in as_frames(frames)]) * TICK_S


def commands(frames, start: Mapping = STAND, stop_tick: int | None = None, stop_ticks: int = 75,
             end: Mapping = SIT, tail_ticks: int = 100) -> tuple[np.ndarray, list[int]]:
    """The joint targets the body plays per 20 ms tick (n x 11, JOINTS
    order) and the frame per tick (-2 after a stop or the end), then
    ``tail_ticks`` holding the last. With ``stop_tick`` the play is cut there
    and the joints go in a straight line to ``end`` over ``stop_ticks``
    (TRICK_STOP)."""
    fr = as_frames(frames)
    cur = np.array([float(start[k]) for k in JOINTS])
    rows, idx = [], []
    for i, (js, dur, hold, ease) in enumerate(fr):
        to = np.array([float(js[k]) if js.get(k) is not None else cur[n] for n, k in enumerate(JOINTS)])
        for t in range(1, dur + 1):
            rows.append(cur + (to - cur) * _ease(ease, t / dur))
            idx.append(i)
        for _ in range(hold):
            rows.append(to.copy())
            idx.append(i)
        cur = to
    if stop_tick is not None and stop_tick < len(rows):
        rows, idx = rows[:stop_tick], idx[:stop_tick]
        a = rows[-1] if rows else np.array([float(start[k]) for k in JOINTS])
        b = np.array([float(end[k]) for k in JOINTS])
        for t in range(1, stop_ticks + 1):
            rows.append(a + (b - a) * t / stop_ticks)
            idx.append(-2)
    last = rows[-1] if rows else cur
    for _ in range(tail_ticks):
        rows.append(last.copy())
        idx.append(-2)
    return np.array(rows), idx


# ---- the dynamics ---------------------------------------------------------------------------------
@dataclass
class Prediction:
    t: np.ndarray                 # s since the play's first tick
    nose_up: np.ndarray           # what TELEMETRY would read, deg (nose up +)
    roll: np.ndarray              # deg, left side down + (TELEMETRY)
    frame: list                   # keyframe index per sample (-2 after the end / a stop)
    contacts: list                # frozenset of labels carrying > 5 g, per sample
    load: dict                    # label -> array of normal force (g) per sample
    clear: dict                   # label -> array of height above the floor (mm) per sample
    slide: dict                   # label -> total distance slid on the floor, mm
    torque: dict = field(default_factory=dict)  # leg joint -> (largest load on its servo, 0.1 s mean, kg cm; when)
    events: list = field(default_factory=list)  # (t, text)
    fell: float | None = None     # t when |pitch| or |roll| passed 75 deg for 120 ms (the body's fall)
    stopped: tuple | None = None  # (t, why) when the mind's checks stopped the play
    up: list = field(default_factory=list)  # the world's up in the body frame, per sample (kilo.tricks.righting)

    def first(self, label: str, after: float = 0.0) -> float | None:
        for t, c in zip(self.t, self.contacts):
            if t >= after and label in c:
                return float(t)
        return None

    def window(self, a: float, b: float) -> np.ndarray:
        return (self.t >= a) & (self.t <= b)


def _rotvec(w: np.ndarray, dt: float) -> np.ndarray:
    th = float(np.linalg.norm(w)) * dt
    if th < 1e-12:
        return np.eye(3)
    a = w / np.linalg.norm(w)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def _sent(kin: Kin, cmd: np.ndarray, prev: np.ndarray) -> np.ndarray:
    """The per-tick angles after the clamp and the safety slew."""
    out = np.empty_like(cmd)
    lo = np.array([kin.reach[k][0] for k in JOINTS])
    hi = np.array([kin.reach[k][1] for k in JOINTS])
    for i in range(len(cmd)):
        prev = prev + np.clip(np.clip(cmd[i], lo, hi) - prev, -SLEW_DEG_TICK, SLEW_DEG_TICK)
        out[i] = prev
    return out


def simulate(frames, p: Params = FITTED, start: Mapping = STAND, settle_s: float = 0.6,
             stop_tick: int | None = None, stop_ticks: int = 75, end: Mapping = SIT,
             t_end: float | None = None, dt: float = 1e-3, sample_s: float = 0.02,
             guards: Sequence[M.Guard] = (), roll_max: float | None = None,
             R0: np.ndarray | None = None, kin: "Kin | None" = None) -> Prediction:
    """Play ``frames`` (a Trick, Keyframes, or (joints, dur, hold) tuples)
    from ``start`` (held ``settle_s`` first). With ``guards`` / ``roll_max``
    the mind's checks run on the model's own TELEMETRY (a guard: the mean
    nose up over the second half of its frame's hold, the largest |roll|
    there; the roll limit: every sample) and stop the play as the mind
    would (the straight line to ``end`` over ``stop_ticks``). Returns the
    trace, sampled every ``sample_s``. ``R0``: the attitude he starts in (body
    -> world; None: level, on the start posture's lowest points), for a start
    lying down; ``kin``: the kinematics, if not Kin(p) (kilo.tricks.righting's,
    which has the head and the legs' sideways bulk)."""
    kin = kin if kin is not None else Kin(p)
    cmd, idx = commands(frames, start, stop_tick, stop_ticks, end)
    n_ticks = len(cmd)
    fr_all = as_frames(frames)
    gwin = {}  # guard frame -> (first tick, end tick, guard): the second half of its hold
    for gd in guards:
        st = sum(d + h for _, d, h, _ in fr_all[:gd.frame]) + fr_all[gd.frame][1]
        gwin[gd.frame] = (st + fr_all[gd.frame][2] / 2, st + fr_all[gd.frame][2], gd)
    gstate: dict = {}
    start_c = np.array([kin.clamp(start)[k] for k in JOINTS])
    sent = _sent(kin, cmd, start_c)
    lag = p.lag_ms / 1000.0
    tau = max(p.tau_ms / 1000.0, 1e-4)
    jidx = {k: i for i, k in enumerate(JOINTS)}
    sag_v = np.array([p.sag if k.endswith(("shoulder", "knee")) else 0.0 for k in JOINTS])
    off_v = np.zeros(len(JOINTS))
    for name, d in p.offsets:
        off_v[jidx[name]] += d

    # level on the start posture, the lowest point just touching
    q = start_c.copy()
    qd = dict(zip(JOINTS, q + off_v))
    P, c, I = kin.state(qd)
    if R0 is None:
        R = np.eye(3)
        floor = float(np.min(P[:, 2] - kin.radius))
        x = np.array([0.0, 0.0, -floor]) + c
    else:  # lying: in that attitude, the lowest point just touching
        R = np.array(R0, float)
        x = np.array([0.0, 0.0, -float(np.min(((P - c) @ R.T)[:, 2] - kin.radius))])
    v = np.zeros(3)
    w = np.zeros(3)
    m = kin.total
    anchors: list = [None] * kin.n
    slid = np.zeros(kin.n)
    defl = np.zeros(len(JOINTS))
    torque = np.zeros(len(JOINTS))
    torque_t = np.zeros(len(JOINTS))
    load_avg = np.zeros(len(JOINTS))
    t0 = -settle_s
    steps = int(round(((n_ticks * TICK_S if t_end is None else t_end) - t0) / dt))
    out_t, out_nu, out_ro, out_fr, out_c = [], [], [], [], []
    out_up = []
    loads: dict = {lb: [] for lb in kin.labels}
    clears: dict = {lb: [] for lb in kin.labels}
    next_sample = t0
    P_prev = c_prev = None
    Pdot = np.zeros((kin.n, 3))
    cdot = np.zeros(3)
    fall_t = over_since = None
    events = []
    contact_prev: frozenset = frozenset()
    stopped_by = None
    k_n, c_n = p.k_mm, p.c_mm
    for step in range(steps):
        t = t0 + step * dt
        # the servo: the command lag s ago, through the first-order response, plus the compliance
        tk = int(math.floor((t - lag) / TICK_S))
        u = start_c if tk < 0 else sent[min(tk, n_ticks - 1)]
        q = q + (u - q) * (dt / tau)
        if step % KIN_EVERY == 0:
            qd = dict(zip(JOINTS, q + defl + off_v))
            P, c, I = kin.state(qd)
            # the points' velocity from the commanded joints only (the module's docstring: numerics)
            Pq, cq, _ = kin.state(dict(zip(JOINTS, q + off_v))) if p.sag else (P, c, I)
            if P_prev is not None:
                Pdot = (Pq - P_prev) / (dt * KIN_EVERY)
                cdot = (cq - c_prev) / (dt * KIN_EVERY)
            P_prev, c_prev = Pq, cq
            P_k, c_k, I_k = P, c, I
        else:
            frac = (step % KIN_EVERY) * dt
            P, c, I = P_k + Pdot * frac, c_k + cdot * frac, I_k
        rel = (P - c) @ R.T
        Pw = x + rel
        Vw = v + np.cross(w, rel) + (Pdot - cdot) @ R.T
        z = Pw[:, 2] - kin.radius
        F = np.zeros_like(Pw)
        touching = []
        for i in np.nonzero(z < 0.0)[0]:
            N = -k_n * z[i] - c_n * Vw[i, 2]
            if N <= 0.0:
                anchors[i] = None
                continue
            xy = Pw[i, :2]
            if anchors[i] is None:
                anchors[i] = xy.copy()
            Ft = -k_n * (xy - anchors[i]) - c_n * Vw[i, :2]
            fm = float(np.hypot(*Ft))
            lim = p.mu * N
            if fm > lim:
                Ft = Ft * (lim / fm)
                na = xy + Ft / k_n
                slid[i] += float(np.hypot(*(na - anchors[i])))
                anchors[i] = na
            F[i] = (Ft[0], Ft[1], N)
            touching.append(i)
        for i in np.nonzero(z >= 0.0)[0]:
            anchors[i] = None
        Iw = R @ I @ R.T
        a = F.sum(axis=0) / m + np.array([0.0, 0.0, -G])
        alpha = np.linalg.solve(Iw, np.cross(rel, F).sum(axis=0) - np.cross(w, Iw @ w))
        v = v + a * dt
        w = w + alpha * dt
        x = x + v * dt
        R = _rotvec(w, dt) @ R
        if step % 200 == 0:
            U, _, Vt = np.linalg.svd(R)
            R = U @ Vt
        # the load on each leg servo (kg cm, from its leg's contacts: the virtual work), and the
        # compliance: each leg joint gives under it, answering as the servo answers a command (tau)
        if step % KIN_EVERY == 0:
            Fb = F @ R
            load = np.zeros(len(JOINTS))
            rad = 180.0 / math.pi
            for li, leg in enumerate(LEGS):
                fi, ki = li, 4 + li
                if not (F[fi].any() or F[ki].any()):
                    continue
                dfs, dfk, dks = kin.foot_jac(leg, qd[f"{leg}_shoulder"], qd[f"{leg}_knee"])
                load[jidx[f"{leg}_shoulder"]] = (Fb[fi] @ dfs + Fb[ki] @ dks) * rad / G_MM_PER_KG_CM
                load[jidx[f"{leg}_knee"]] = (Fb[fi] @ dfk) * rad / G_MM_PER_KG_CM
            if t >= 0:
                load_avg += (load - load_avg) * (dt * KIN_EVERY / 0.1)   # over ~0.1 s: not an impact's peak
                more = np.abs(load_avg) > torque
                torque[more] = np.abs(load_avg)[more]
                torque_t[more] = t
            if p.sag:
                target = np.clip(load * sag_v, -25.0, 25.0)
                defl = defl + (target - defl) * (dt * KIN_EVERY / tau)
        if t >= next_sample - 1e-9:
            pitch, roll = T.attitude(R)
            tk_now = int(math.floor(t / TICK_S))
            fi = idx[tk_now] if 0 <= tk_now < n_ticks else (-1 if tk_now < 0 else -2)
            cs = frozenset(kin.labels[i] for i in touching if F[i, 2] > 5.0 * G)
            if t >= 0:
                out_t.append(t)
                out_nu.append(pitch + p.imu_pitch)
                out_ro.append(roll + p.imu_roll)
                out_fr.append(fi)
                out_c.append(cs)
                out_up.append(R[2].copy())
                for i, lb in enumerate(kin.labels):
                    loads[lb].append(F[i, 2] / G)
                    clears[lb].append(z[i])
                if cs != contact_prev:
                    gone, new = contact_prev - cs, cs - contact_prev
                    events.append((round(t, 2), ("+" + ",".join(sorted(new)) if new else "")
                                   + (" -" + ",".join(sorted(gone)) if gone else "")))
                contact_prev = cs
                if abs(pitch) > 75 or abs(roll) > 75:
                    over_since = t if over_since is None else over_since
                    if fall_t is None and t - over_since >= 0.12:
                        fall_t = t
                        events.append((round(t, 2), "FELL"))
                else:
                    over_since = None
                if stopped_by is None and fi >= 0:
                    nu, ro = pitch + p.imu_pitch, roll + p.imu_roll
                    if roll_max is not None and abs(ro) > roll_max:
                        stopped_by = (t, f"roll {ro:+.1f} past {roll_max:g}")
                    for gk, (gs, ge, gd) in gwin.items():
                        if gs <= tk_now < ge:
                            gstate.setdefault(gk, []).append((nu, ro))
                            if abs(ro) > gd.roll_max and stopped_by is None:
                                stopped_by = (t, f"guard {gd.name}: roll {ro:+.1f} past {gd.roll_max:g}")
                        elif tk_now >= ge and gstate.get(gk):
                            vals = gstate[gk]
                            mn = sum(a_ for a_, _ in vals) / len(vals)
                            if not (gd.nose_up[0] <= mn <= gd.nose_up[1]) and stopped_by is None:
                                stopped_by = (t, f"guard {gd.name}: nose up {mn:.1f} outside "
                                                 f"{gd.nose_up[0]:g}..{gd.nose_up[1]:g}")
                            gstate[gk] = None
                    if stopped_by is not None:
                        events.append((round(t, 2), "STOP " + stopped_by[1]))
                        cut = tk_now + 1
                        cmd2, idx2 = commands(frames, start, cut, stop_ticks, end)
                        sent = np.vstack([sent[:cut], _sent(kin, cmd2[cut:], sent[cut - 1].copy())])
                        idx, n_ticks = idx2, len(cmd2)
                        if t_end is None:
                            steps = min(steps, int(round((n_ticks * TICK_S - t0) / dt)))
            next_sample += sample_s
        if step >= steps:
            break
    return Prediction(t=np.array(out_t), nose_up=np.array(out_nu), roll=np.array(out_ro), frame=out_fr,
                      contacts=out_c, load={k: np.array(v_) for k, v_ in loads.items()},
                      clear={k: np.array(v_) for k, v_ in clears.items()},
                      slide={kin.labels[i]: float(s) for i, s in enumerate(slid) if s > 0.05},
                      torque={k: (float(v_), float(t_)) for k, v_, t_ in zip(JOINTS, torque, torque_t) if v_ > 0},
                      events=events, fell=fall_t, stopped=stopped_by, up=out_up)


# ---- the recorded plays: what the model is held to --------------------------------------------------
def fixture(path: Path = FIXTURE) -> dict:
    return json.loads(path.read_text())


def replay(run: dict, p: Params = FITTED) -> Prediction:
    """A recorded play (one of fixture()["runs"]) in the model, from the
    posture it started in, with the guards and the roll limit it had."""
    start = SIT if run["start"] == "SIT" else STAND
    guards = [M.Guard.from_dict(g) for g in run["guards"]]
    return simulate(run["frames"], p, start=start, guards=guards, roll_max=run["roll_max"],
                    stop_ticks=run["stop_ticks"], t_end=run["end_s"] + 1.0)


def features(t: np.ndarray, nu: np.ndarray, roll: np.ndarray, run: dict, t0: float = 0.0) -> dict:
    """What the model is held to in a trace of the second design (``t0``:
    when frame 0 would have started on its clock): at "lift_r" whether he
    went over (roll past -6); at "gather" when the nose up passed +5 (s
    after its move started), and the mean nose up and roll over the second
    half of its hold."""
    st = starts(run["frames"]) + t0
    names = run["names"]
    out = {}
    k = names.index("lift_r")
    m = (t >= st[k]) & (t <= st[k + 1] + 0.2)
    if m.any():
        out["lift_r_over"] = bool(roll[m].min() < -6.0)
    k = names.index("gather")
    hold = run["frames"][k]["hold"] * TICK_S
    m = (t >= st[k]) & (t <= st[k + 1])
    if m.any():
        i = np.nonzero(m & (nu > 5.0))[0]
        out["gather_t5"] = float(t[i[0]] - st[k]) if len(i) else None
        mm = (t >= st[k + 1] - hold / 2) & (t <= st[k + 1])
        out["gather_nose_up"] = float(nu[mm].mean())
        out["gather_roll"] = float(roll[mm].mean())
    return out


def replay_gather(run: dict, p: Params = FITTED) -> dict:
    """"gather" alone: from "place_r" (all four feet planted under the
    body; the run's frames before it are not played, so the lift_r knife
    edge does not decide it), its move and hold. Its features."""
    k = run["names"].index("gather")
    frames = run["frames"]
    pr = simulate(frames[k:k + 1], p, start=frames[k - 1]["joints"], t_end=(frames[k]["dur"] + frames[k]["hold"])
                  * TICK_S + 0.05)
    return features(pr.t, pr.nose_up, pr.roll, run, t0=-starts(frames)[k])


def lift_r_over(run: dict, p: Params = FITTED) -> bool:
    """Whether the play to the end of "lift_r" goes over the diagonal in the model."""
    k = run["names"].index("lift_r")
    pr = simulate(run["frames"][:k + 1], p, start=SIT if run["start"] == "SIT" else STAND,
                  t_end=starts(run["frames"])[k + 1] + 0.2)
    return features(pr.t, pr.nose_up, pr.roll, run)["lift_r_over"]


def validate(p: Params = FITTED) -> dict:
    """The recorded plays against the model: per run the recorded features;
    the model's gather (from place_r); and lift_r with the centre of mass at
    FITTED's and 0.5 mm further forward (it sits on the knife edge: 3 of 7
    plays went over)."""
    runs = fixture()["runs"]
    t1 = next(r for r in runs if r["label"] == "t1")
    rec = {}
    for run in runs:
        s = np.array(run["series_t_nose_up_roll"])
        rec[run["label"]] = features(s[:, 0], s[:, 1], s[:, 2], run)
    return {"recorded": rec, "model_gather": replay_gather(t1, p),
            "model_lift_r_over": (lift_r_over(t1, p), lift_r_over(t1, p.with_(com_dx=p.com_dx + 0.5)))}


def check_kinematics(p: Params = NOMINAL, frames: Sequence[Mapping] = ()) -> float:
    """Largest difference (mm) between Kin and stance.points / stance.com."""
    kin = Kin(p)
    g3 = p.g3()
    worst = 0.0
    for js in list(frames) + [STAND, SIT]:
        js = kin.clamp(js)
        P, c, _ = kin.state(js)
        ref = {lb: q for lb, q, _ in T.points(g3, js)}
        for lb, q in zip(kin.labels, P):
            worst = max(worst, float(np.max(np.abs(np.array(ref[lb]) - q))))
        worst = max(worst, float(np.max(np.abs(T.com(g3, js) - c))))
    return worst


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="The compliant model against the recorded rear_up plays.")
    ap.add_argument("--check", action="store_true", help="the fast kinematics against kilo.tricks.stance")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="a parameter off FITTED (e.g. com_dx=-6)")
    args = ap.parse_args(argv)
    if args.check:
        d = check_kinematics()
        print(f"kinematics vs stance: largest difference {d:.3f} mm")
        return 0 if d < 0.01 else 1
    p = FITTED.with_(**{k: float(v) for k, v in (s.split("=", 1) for s in args.set)})
    print(f"parameters: {p}")
    fmt = lambda v: "-" if v is None else (("yes" if v else "no") if isinstance(v, bool) else f"{v:+.2f}")  # noqa: E731
    v = validate(p)
    for label, rec in v["recorded"].items():
        print(f"  recorded {label:4} " + "  ".join(f"{k} {fmt(x)}" for k, x in rec.items()))
    print("  model    gather " + "  ".join(f"{k} {fmt(x)}" for k, x in v["model_gather"].items()))
    a, b = v["model_lift_r_over"]
    print(f"  model    lift_r goes over: {fmt(a)} (FITTED), {fmt(b)} (the centre of mass 0.5 mm further forward)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
