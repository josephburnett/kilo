"""righting: Kilo lying on the floor in the trick simulator, and the body's
``rc`` recovery replayed there against every recorded fall (docs/righting.md).

    python -m kilo.tricks.righting --falls      every recorded fall and each try of rc after it
    python -m kilo.tricks.righting --rest       how the model lies against how Kilo lay
    python -m kilo.tricks.righting --replay     rc in the model from each recorded start, against the recording
    python -m kilo.tricks.righting --bands      ...across the uncertain numbers (a minute or two)
    python -m kilo.tricks.righting --phases     where the model has him at the end of each rc frame, against Kilo

WHAT THIS IS FOR. Getting up after a fall is today a table in the body
(body/src/core/posture.cpp, the three ``rc`` rows; reflex.h). Before a
getting-up can be DESIGNED in the model (as the rear-up was), the model has
to show that it can tell a getting-up that works from one that does not.
The only evidence is the recorded falls (mind/tests/fixtures/falls_2026_11.json:
8 tries of rc from lying down, 4 that ended upright and 4 that did not, and
4 played from upright). ``verdict`` holds the model to them by a criterion
fixed before the first replay (CRITERION below).

WHAT IS ADDED TO kilo.tricks.dynamics FOR IT (``KinLying``, ``Lying``):

- a start in any attitude (dynamics.simulate ``R0``);
- the head: a box on the neck that the pan and tilt servos move, as contact
  points, and its mass (``head_g``, out of the frame's lump) moving with it;
- the moving parts' momentum (dynamics.Kin ``internal``): a leg, the head
  or the tail swung one way turns the frame the other way (2026-11-22);
- how far the shoulder and knee servos stick out sideways from the legs'
  plane, the top edges he lies against on his side and on his back, and a
  higher middle of the back (``ridge_z``, off by default);
- which way the tail swings (dynamics.Params ``tail_sign``), not known.

WHY IT FAILS (2026-11-22, ``--phases`` and docs/righting-results.md): rc's
three frames test three different things. Frame 1 (from a side, he rolls
onto his back on Kilo, 3 of 3) holds only with the back's top edges lower
than estimated (top_z 30); frame 2 (from his back over one side) goes the
way Kilo went only with a higher, rounded middle of the back that the feet
cannot out-reach, and then only unmirrored; frame 3 (from the side up onto
his feet) falls short in nearly every variant. No one outline of the 200
tried does all three: the outline of his back and head, never measured, is
what the outcome hangs on. The friction, the servos' speed and the legs'
give change little.

NONE OF THOSE DIMENSIONS IS MEASURED. ``Lying``'s defaults are estimates,
three of them (``hip_r``, ``ear_z``, ``top_z``) set so that the model lies
the way the recordings show Kilo lying (``rest_check``), which uses up the
recorded resting attitudes: only the outcomes and the motion of the tries
test the model. docs/righting.md lists what to measure.

Conventions: kilo.tricks.dynamics's (body x forward, y LEFT, z up, mm; the
world's z up). ``up`` is the world's up in the body frame, a unit vector:
(0, 0, 1) standing, (0, 0, -1) on his back, (0, -1, 0) on his left side.
TELEMETRY's pitch (nose DOWN +, reflected when upside down) and roll (left
side down +) are turned into it by ``up_of`` (body/src/core/attitude.cpp).
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

from kilo.calibration import POSTURES, mirror
from kilo.planner.model import LF, LB, RB, RF, BodyModel

from . import dynamics as D
from . import model as M
from . import stance as T

FIXTURE = Path(__file__).resolve().parents[2] / "tests" / "fixtures" / "falls_2026_11.json"
JOINTS = D.JOINTS
TICK_S = D.TICK_S

# The body's rc: three rows of body/src/core/posture.cpp (kilo.calibration.POSTURES has the angles,
# tied to the C table by mind/tests/test_calibration.py), each a cosine move over ``dur`` ticks and
# a hold (the rows' last two bytes; tests/test_righting.py reads them from posture.cpp).
RC_TICKS = ((11, 30), (5, 25), (18, 0))
RC_MOVES_S = sum(d + h for d, h in RC_TICKS) * TICK_S   # 1.78 s
CHECK_S = 1.0             # then the body waits this long for upright (FALL_CHECK_TICKS)
LEVEL_DEG = 20.0          # "upright": |pitch| and |roll| under this (FALL_LEVEL_DEG)...
LEVEL_S = 0.5             # ...for this long (FALL_LEVEL_TICKS)

# THE CRITERION, written before the first replay (2026-11-21). The model is credible for designing
# a getting-up only if, with the fitted parameters and nothing tuned on the outcomes:
#   (a) REST: from each recorded lying attitude, with the joints he had, the model stays within
#       REST_TOL_DEG of it (it lies the way Kilo lay);
#   (b) OUTCOME: it ends upright for every recorded try that ended upright (4 from lying, 4 from
#       upright), and does not for at least 3 of the 4 that did not [amended, below]. Two recorded tries began the
#       same way within 1 degree (boot 8, on his left side rolled toward his back, mirrored rc)
#       and ended differently, so for that start the outcome must also flip somewhere inside the
#       bands of the uncertain numbers (he is on a knife edge there, and the model must say so);
#   (c) MOTION: for the tries that ended upright, the model passes through "on his side" (up's z
#       through 0) within SIDE_TOL_S of when Kilo did, and the angle between the model's and the
#       recorded up, rms over the try's moves, is under TRACE_TOL_DEG.
#
# AMENDED after the first replay, the same day, and why: all four tries "from upright" and two of
# the tries from lying down belong to falls around which the mind's picked_up percept or the
# guard's picked-up state fired (the fixture's "floor": false). Kilo was in a hand: the first
# replay showed why it matters (rc's second frame lifts both legs of one side; a Kilo standing
# free must go over on that side, and the recordings stay within 8 degrees of level). A try in a
# hand tests nothing about the floor, so (a), (b) and (c) are judged on the FLOOR tries only:
# 6 from lying down, 4 that ended upright and 2 that did not, and (b) asks for all 4 and both.
# The tries in a hand are still replayed and shown.
REST_TOL_DEG = 15.0
SIDE_TOL_S = 0.25
TRACE_TOL_DEG = 30.0


# ---- TELEMETRY's attitude as a vector ---------------------------------------------------------------
def up_of(pitch: float, roll: float) -> np.ndarray:
    """The world's up in the body frame from TELEMETRY's pitch (nose down +, with the legacy
    reflection when upside down) and roll (left side down +), deg."""
    r = math.radians(roll)
    p = pitch
    if math.cos(r) < 0:                # gz < 0: attitude.cpp reflected the pitch
        p = (180.0 - pitch) if pitch > 0 else (-180.0 - pitch)
    p = math.radians(p)
    # attitude.cpp's g = (sin p, cos p sin r, cos p cos r); the model's up = (-gx, -gy, gz)
    return np.array([-math.sin(p), -math.cos(p) * math.sin(r), math.cos(p) * math.cos(r)])


def telemetry_of(up: np.ndarray) -> tuple[float, float]:
    """(pitch, roll) as the body would report ``up`` (up_of's inverse)."""
    gx, gy, gz = -up[0], -up[1], up[2]
    pitch = math.degrees(math.atan2(gx, math.hypot(gy, gz)))
    roll = math.degrees(math.atan2(gy, gz))
    if gz < 0:
        pitch = 180.0 - pitch if pitch > 0 else -180.0 - pitch
    return pitch, roll


def angle(a: np.ndarray, b: np.ndarray) -> float:
    """Degrees between two up vectors."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))))))


def lying(up: np.ndarray) -> str:
    """How he lies, in words."""
    if abs(up[0]) > math.sin(math.radians(60)):
        return "nose down" if up[0] < 0 else "tail down"
    if up[2] > math.cos(math.radians(30)):
        return "upright"
    if up[2] < -math.cos(math.radians(30)):
        return "on his back"
    side = "left" if up[1] < 0 else "right"
    if up[2] > 0.2:
        return f"leaning on his {side} side, feet down"
    if up[2] < -0.2:
        return f"on his {side} side, rolled toward his back"
    return f"on his {side} side"


def upright(up: np.ndarray) -> bool:
    """The body's own test (fall_level): |pitch| and |roll| under 20 deg."""
    p, r = telemetry_of(up)
    return abs(p) < LEVEL_DEG and abs(r) < LEVEL_DEG


def r0_for(up: np.ndarray) -> np.ndarray:
    """A body -> world rotation whose third row is ``up`` (the heading is free: the body's x axis
    as near the world's as it goes)."""
    z = np.asarray(up, float) / np.linalg.norm(up)
    e = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = e - np.dot(e, z) * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.vstack([x, y, z])


# ---- the geometry the model lacks: ESTIMATES --------------------------------------------------------
@dataclass(frozen=True)
class Lying:
    """What touches the floor when Kilo is not on his feet. Every number is an ESTIMATE (mm, body
    frame; nothing here has been measured: docs/righting.md "What to measure"); ``hip_r``,
    ``ear_z`` and ``top_z`` were set on the recorded resting attitudes (rest_check)."""
    hip_r: float = 18.0        # the shoulder servo and its horn, as a ball about the axle's end (stance has 8)
    knee_r: float = 9.0        # the knee servo, as a ball about the knee's axle (stance has 0)
    top_z: float = 36.0        # the top edges he lies against (the model's pi_top_z), and
    top_y: float = 30.0        # ...their half width (the model's half_width)
    ridge_z: float = 0.0       # a higher middle of the back (the Pi's connectors), at +-ridge_y; 0: none
    ridge_y: float = 10.0
    # the head: a box on the neck. The pan servo's axis is vertical through the neck, the tilt's across it.
    neck_x: float = 72.0       # the neck, ahead of the middle of the shoulders (the front axles are at 61.5)
    neck_z: float = 12.0       # ...and above the axles
    head_len: float = 45.0     # neck to the muzzle's end
    head_half_w: float = 22.0
    ear_z: float = 45.0        # the top of the ears above the neck
    ear_x: float = 12.0        # ...and ahead of it
    chin_z: float = -17.0      # the underside of the muzzle below the neck
    head_g: float = 25.0       # the head's moving part (tilt servo, ultrasonic sensor, shell), out of the frame's lump
    head_com_x: float = 18.0   # ...its centre ahead of the neck
    head_com_z: float = 8.0    # ...and above it
    pan_sign: float = 1.0      # +1: a positive pan turns the nose LEFT. NOT KNOWN
    tilt_sign: float = 1.0     # +1: a positive tilt lifts the nose (REST has -80 and the head down)

    def with_(self, **kw) -> "Lying":
        return replace(self, **kw)

    def head_points(self) -> list[tuple[str, tuple, float]]:
        """(label, point from the neck at pan 0 and tilt 0, radius)."""
        w, L = self.head_half_w, self.head_len
        return [("ear_l", (self.ear_x, w * 0.8, self.ear_z), 0.0), ("ear_r", (self.ear_x, -w * 0.8, self.ear_z), 0.0),
                ("cheek_l", (L * 0.4, w, 8.0), 0.0), ("cheek_r", (L * 0.4, -w, 8.0), 0.0),
                ("muzzle_l", (L, w * 0.6, 0.0), 0.0), ("muzzle_r", (L, -w * 0.6, 0.0), 0.0),
                ("chin_l", (L * 0.8, w * 0.6, self.chin_z), 0.0), ("chin_r", (L * 0.8, -w * 0.6, self.chin_z), 0.0)]


HEAD_LABELS = tuple(lb for lb, _, _ in Lying().head_points())


class KinLying(D.Kin):
    """dynamics.Kin with the head moving on its two servos and the sideways bulk of the leg
    servos: the points of ``Lying``. The fixed nose points of the side view are replaced by the
    head's; the top edges take ``Lying``'s height and width."""

    def __init__(self, p: D.Params, ly: Lying | None = None):
        super().__init__(p)
        self.ly = ly or Lying()
        ly = self.ly
        keep = [i for i, (lb, _, _) in enumerate(self.fixed_pts) if not lb.startswith("nose_")]
        fixed = []
        for i in keep:
            lb, q, rr = self.fixed_pts[i]
            q = q.copy()
            if lb.startswith(("pi_", "top_")):
                q[2] = ly.top_z
                q[1] = math.copysign(ly.top_y, q[1])
            fixed.append((lb, q, rr))
        if ly.ridge_z > 0:  # the ridge: four points over the Pi's length
            for sx in (-40.0, 20.0):
                for sy in (-1.0, 1.0):
                    fixed.append((f"ridge_{'l' if sy > 0 else 'r'}{'f' if sx > 0 else 'b'}",
                                  np.array([sx, sy * ly.ridge_y, ly.ridge_z]), 0.0))
        # the shoulder servos' ends (dynamics.Kin leaves stance's hip points out: on his feet they never touch)
        for leg in D.LEGS:
            hx = self.hx if T.FRONT[leg] else -self.hx
            fixed.append((f"{leg}_hip", np.array([hx, self.ty if T.LEFT[leg] else -self.ty, 0.0]), ly.hip_r))
        self.fixed_pts = fixed
        self.fixed_P = np.array([q for _, q, _ in fixed])
        self.head = [(lb, np.array(q, float), rr) for lb, q, rr in ly.head_points()]
        self.labels = ([f"{leg}_foot" for leg in D.LEGS] + [f"{leg}_knee" for leg in D.LEGS] + [T.TAIL]
                       + [lb for lb, _, _ in fixed] + [lb for lb, _, _ in self.head])
        self.radius = np.array([self.r] * 4 + [ly.knee_r] * 4 + [0.0] + [rr for _, _, rr in fixed]
                               + [rr for _, _, rr in self.head])
        self.n = len(self.labels)
        self.neck = np.array([ly.neck_x, 0.0, ly.neck_z])
        self.head_P = np.array([q for _, q, _ in self.head])
        # the head's mass moves with the head: taken out of the frame's lump where it sits at pan 0, tilt 0
        self.head_c = np.array([ly.head_com_x, 0.0, ly.head_com_z])
        w0, x0 = self.fixed_m[0]
        hp = self.neck + self.head_c
        self.fixed_m[0] = (w0 - ly.head_g, (w0 * x0 - ly.head_g * hp) / (w0 - ly.head_g))
        self.internal = True       # the moving parts' momentum turns the frame (dynamics.simulate)

    def state(self, q: Mapping):
        P, c, I = super().state(q)       # feet, knees, tail, then the side view's fixed points (nose among them)
        n_lead = 9
        a = math.radians(self.ly.pan_sign * q["head_pan"])
        b = math.radians(self.ly.tilt_sign * q["head_tilt"])
        Rz = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
        Ry = np.array([[math.cos(b), 0.0, -math.sin(b)], [0.0, 1.0, 0.0], [math.sin(b), 0.0, math.cos(b)]])
        Rh = Rz @ Ry
        head = self.neck + self.head_P @ Rh.T
        # the head's lump back in: the centre of mass and the inertia about it
        m_h = self.ly.head_g
        h = self.neck + Rh @ self.head_c
        m0 = self.total - m_h
        cn = (m0 * c + m_h * h) / self.total
        d0, dh = c - cn, h - cn
        E3 = np.eye(3)
        I = I + m0 * (np.dot(d0, d0) * E3 - np.outer(d0, d0)) + m_h * (np.dot(dh, dh) * E3 - np.outer(dh, dh))
        self.pm = list(self.pm) + [(m_h, h)]
        return np.vstack([P[:n_lead], self.fixed_P, head]), cn, I


# ---- rc, as the body plays it ----------------------------------------------------------------------
def rc_frames(mirrored: bool) -> list[tuple[dict, int, int]]:
    """The body's rc as (joints, dur, hold) (cosine moves, as the body's reflex mover eases them)."""
    out = []
    for i, (dur, hold) in enumerate(RC_TICKS):
        row = POSTURES[f"recovery {i + 1}"]
        if mirrored:
            row = mirror(row)
        out.append((dict(zip(JOINTS, (float(v) for v in row))), dur, hold))
    return out


def stance_joints(model: BodyModel | None = None) -> dict:
    """STANCE (the feet under the shoulders at the stance height) as joint angles (core/ik.h:
    shoulder = -theta1, knee = -(theta2 - sigma 90))."""
    m = model or BodyModel()
    js = dict(zip(JOINTS, (float(v) for v in POSTURES["stand"])))
    for leg, name in ((LF, "lf"), (RF, "rf"), (RB, "rb"), (LB, "lb")):
        t1, t2 = m.ik(leg, 0.0, m.stance_z_mm())
        sigma = 1 if name in ("lf", "rf") else -1
        js[f"{name}_shoulder"] = -math.degrees(t1)
        js[f"{name}_knee"] = -(math.degrees(t2) - sigma * 90.0)
    return js


# the first rear-up design's "ready" frame: what the legs were doing when he went over on 2026-11-17
REAR_UP_READY = {"head_pan": 0.0, "head_tilt": -20.0, "tail": 0.0, "lf_shoulder": 50.0, "rf_shoulder": 50.0,
                 "rb_shoulder": -65.0, "lb_shoulder": -65.0, "lf_knee": 49.0, "rf_knee": 49.0, "rb_knee": -43.0,
                 "lb_knee": -43.0}


def start_joints(fall: dict, n: int) -> dict:
    """The joints the reflex held when try ``n`` (1..) of ``fall`` began: the pose he fell in for
    the first (the reflex freezes it; the recordings have no joint angles, so it is taken from
    what he was doing), rc's last frame (as that try was mirrored) for the later ones."""
    if n > 1:
        prev = fall["tries"][n - 2]
        return rc_frames(prev["mirrored"])[-1][0]
    mode = fall["mode_before"]
    if mode == "TRICK":
        return dict(REAR_UP_READY)
    before = fall.get("attitude_before") or [0.0, 0.0]
    if mode == "IDLE" and before[0] < -15.0:        # sitting reads about -27 (nose up)
        return dict(zip(JOINTS, (float(v) for v in POSTURES["sit"])))
    if mode == "GAIT" or "crawl" in fall["doing"]:
        return stance_joints()
    return dict(zip(JOINTS, (float(v) for v in POSTURES["stand"])))


# ---- a try in the model ------------------------------------------------------------------------------
@dataclass
class Replay:
    label: str
    floor: bool                   # False: a hand had him (the fixture's picked-up lines): no test
    start_lying: str
    mirrored: bool
    recorded_righted: bool
    righted: bool                 # the model: upright (the body's test) for LEVEL_S before the check ran out
    rest_err: float               # deg: the model's attitude after settling against the recorded start
    trace_rms: float              # deg: model against recording over the moves
    side_t: float | None          # s: when the model's up z last rose through 0 (None: it did not)
    side_t_rec: float | None
    end_lying: str
    torque: float                 # the largest load on a leg servo, kg cm (0.1 s mean)
    t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    up: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))


def _rise_through_zero(t: np.ndarray, z: np.ndarray) -> float | None:
    i = np.nonzero((z[:-1] < 0) & (z[1:] >= 0))[0]
    return float(t[i[-1] + 1]) if len(i) else None


def replay(fall: dict, n: int, p: D.Params = D.FITTED, ly: Lying | None = None, settle_s: float = 1.0) -> Replay:
    """Try ``n`` (1..) of a recorded fall in the model: from the recorded attitude with the joints
    of start_joints, settled, then rc as the body mirrored it; against the recording."""
    tr = fall["tries"][n - 1]
    rec = np.array(tr["trace"], float)
    rec_t = rec[:, 0]
    rec_up = np.array([up_of(a, b) for a, b in rec[:, 1:3]])
    up0 = up_of(*tr["start"])
    kin = KinLying(p, ly)
    start = start_joints(fall, n)
    frames = rc_frames(tr["mirrored"])
    t_end = RC_MOVES_S + CHECK_S
    pr = D.simulate(frames, p, start=start, settle_s=settle_s, t_end=t_end, R0=r0_for(up0), kin=kin)
    up = np.array(pr.up)
    rest_err = angle(up[0], up0)
    # the model's up at the recording's sample times, over the moves
    sel = (rec_t >= 0) & (rec_t <= min(RC_MOVES_S + 0.4, pr.t[-1]))
    k = np.clip(np.searchsorted(pr.t, rec_t[sel]), 0, len(pr.t) - 1)
    errs = np.array([angle(up[i], u) for i, u in zip(k, rec_up[sel])])
    ok = np.array([upright(u) for u in up])
    righted = False
    run = 0.0
    for i in range(1, len(ok)):   # as the body judges it: level for LEVEL_S once the moves are over
        run = run + (pr.t[i] - pr.t[i - 1]) if ok[i] and pr.t[i] >= RC_MOVES_S else 0.0
        if run >= LEVEL_S - 1e-9:
            righted = True
            break
    torque = max((v for v, _ in pr.torque.values()), default=0.0)
    return Replay(label=f"{fall['file'].replace('rec-card-', '').replace('.krec', '')}@{fall['uptime_s']:.0f}s try {n}",
                  floor=bool(fall.get("floor", True)), start_lying=tr["start_lying"], mirrored=tr["mirrored"], recorded_righted=tr["righted"],
                  righted=righted, rest_err=rest_err, trace_rms=float(np.sqrt(np.mean(errs ** 2))) if len(errs) else 0.0,
                  side_t=_rise_through_zero(pr.t, up[:, 2]), side_t_rec=_rise_through_zero(rec_t[rec_t >= 0], rec_up[rec_t >= 0][:, 2]),
                  end_lying=lying(up[-1]), torque=torque, t=pr.t, up=up)


def tries(fx: dict | None = None) -> list[tuple[dict, int]]:
    fx = fx or fixture()
    return [(f, t["n"]) for f in fx["falls"] for t in f["tries"]]


def fixture(path: Path = FIXTURE) -> dict:
    return json.loads(path.read_text())


# ---- falls out of a flight recording -------------------------------------------------------------
def falls_in(path, date: str = "", doing: str = "", owner: str = "nothing reported") -> list[dict]:
    """Every EVENT FELL in a flight recording (.krec) with each try of rc after it, as the
    fixture's "falls" entries: the still attitude each try began in, whether rc was mirrored (the
    body's rule: for roll > 0, flipped on every other try), TELEMETRY through it, how it ended,
    and the picked-up lines of the mind's log around it ("floor": none)."""
    from kilo.logs import flight
    from kilo.protocol import kilo_protocol as P

    s = flight.load(path)
    mode = {int(m): m.name for m in P.Mode}
    ev = [(f.t_us, f.msg) for f in s.frames if f.direction == "rx" and isinstance(f.msg, P.Event)]
    tel = s.telemetry()
    fells = [(t, m.arg) for t, m in ev if m.code == P.EventCode.FELL]
    ends = [(t, P.EventCode(m.code).name) for t, m in ev
            if m.code in (P.EventCode.RECOVERED, P.EventCode.RECOVERY_FAILED)]
    out = []
    for k, (tf, arg) in enumerate(fells):
        te, how = next(((t, h) for t, h in ends if t > tf), (None, None))
        nxt = fells[k + 1][0] if k + 1 < len(fells) else 1 << 62
        if te is None or te > nxt:
            te, how = min(nxt, tel[-1][0]), "none in the recording"
        before = [(t, m) for t, m in tel if tf - 3_000_000 <= t < tf - 1_500_000]
        hand = [f"{(g.t_us - tf) / 1e6:+.2f} s {g.text[:60]}" for g in s.logs
                if tf - 2_000_000 <= g.t_us <= te + 200_000
                and ("picked_up" in g.text or "picked-up" in g.text or "put_down" in g.text)]
        fall = {"file": Path(path).name, "picked_up_lines": hand, "floor": not any("picked" in h for h in hand),
                "date": date, "mind_build": s.meta.get("mind_build"), "uptime_s": round(s.uptime_s(tf), 2),
                "fell_arg": arg, "doing": doing, "owner": owner,
                "mode_before": mode.get(before[-1][1].mode, "?") if before else "?",
                "attitude_before": [round(before[-1][1].pitch / 16, 1), round(before[-1][1].roll / 16, 1)]
                if before else None,
                "ended": how, "ended_after_s": round((te - tf) / 1e6, 2), "tries": []}
        seg = [(t, m) for t, m in tel if tf <= t <= te + 200_000]
        i = n = 0
        while i < len(seg):
            if seg[i][1].mode != P.Mode.RECOVERING:
                i += 1
                continue
            j = i
            while j < len(seg) and seg[j][1].mode == P.Mode.RECOVERING:
                j += 1
            t0, m0 = seg[i]
            p0, r0 = m0.pitch / 16, m0.roll / 16
            mirrored = bool((r0 > 0) ^ (n & 1))
            n += 1
            tr = [(t, m) for t, m in tel if t0 - 400_000 <= t <= min(t0 + 3_000_000, te + 200_000)]
            at_moves = min(tr, key=lambda x: abs(x[0] - (t0 + int(RC_MOVES_S * 1e6))))[1]
            last = seg[j - 1][1]
            moved = None
            kk = j
            while kk < len(seg) and seg[kk][1].mode == P.Mode.FALLEN:
                kk += 1
            if j < kk < len(seg):
                moved = round(angle(up_of(seg[j][1].pitch / 16, seg[j][1].roll / 16),
                                    up_of(seg[kk][1].pitch / 16, seg[kk][1].roll / 16)))
            up_m, up_l = up_of(at_moves.pitch / 16, at_moves.roll / 16), up_of(last.pitch / 16, last.roll / 16)
            g0 = up_of(p0, r0)
            fall["tries"].append({
                "n": n, "starts_after_fell_s": round((t0 - tf) / 1e6, 2), "start": [round(p0, 1), round(r0, 1)],
                "start_gravity": [round(-g0[0], 3), round(-g0[1], 3), round(g0[2], 3)], "start_lying": lying(g0),
                "mirrored": mirrored, "lasted_s": round((seg[j - 1][0] - t0) / 1e6, 2),
                "at_moves_end": [round(at_moves.pitch / 16, 1), round(at_moves.roll / 16, 1)],
                "at_moves_end_lying": lying(up_m), "end": [round(last.pitch / 16, 1), round(last.roll / 16, 1)],
                "end_lying": lying(up_l), "righted": lying(up_l) == "upright", "moved_before_next_try_deg": moved,
                "trace": [[round((t - t0) / 1e6, 2), round(m.pitch / 16, 1), round(m.roll / 16, 1),
                           round(m.rate_pitch / 16), round(m.rate_roll / 16), round(m.rate_yaw / 16)] for t, m in tr]})
            i = j
        out.append(fall)
    return out


def _job(job) -> Replay:
    fall, n, p, ly = job
    r = replay(fall, n, p, ly)
    r.t, r.up = np.zeros(0), np.zeros((0, 3))     # not sent back
    return r


def replay_all(p: D.Params = D.FITTED, ly: Lying | None = None, workers: int = 4,
               floor_only: bool = False) -> list[Replay]:
    jobs = [(f, n, p, ly) for f, n in tries() if f.get("floor", True) or not floor_only]
    return _map(jobs, workers)


def _map(jobs, workers: int) -> list[Replay]:
    if workers <= 1:
        return [_job(j) for j in jobs]
    import multiprocessing as mp
    with mp.get_context("fork").Pool(min(workers, 4)) as pool:
        return pool.map(_job, jobs, chunksize=1)


def bands(p: D.Params | None = None, ly: Lying | None = None) -> list[tuple[str, D.Params, Lying]]:
    """The uncertain numbers one at a time: dynamics.bands (the centre of mass, the legs' give,
    the servos, the friction 0.3 and 1.0), the friction 0.4, 0.6 and 0.8, and each estimate of
    ``Lying`` a quarter either way (its signs both ways)."""
    p = p or D.FITTED
    ly = ly or Lying()
    out = [(name, q, ly) for name, q in D.bands(p)]
    for mu in (0.4, 0.6, 0.8):
        out.append((f"mu={mu}", p.with_(mu=mu), ly))
    for k in ("hip_r", "knee_r", "top_z", "top_y", "head_len", "head_half_w", "ear_z", "neck_x", "neck_z"):
        v = getattr(ly, k)
        for s in (0.75, 1.25):
            out.append((f"{k} x{s:g}", p, ly.with_(**{k: v * s})))
    out.append(("pan the other way", p, ly.with_(pan_sign=-ly.pan_sign)))
    out.append(("tilt the other way", p, ly.with_(tilt_sign=-ly.tilt_sign)))
    out.append(("tail the other way", p.with_(tail_sign=-p.tail_sign), ly))
    out.append(("no head mass", p, ly.with_(head_g=0.0)))
    out.append(("a rounded back (ridge 42 mm at +-14, edges 26)", p, ly.with_(ridge_z=42.0, ridge_y=14.0, top_z=26.0,
                                                                           ear_z=55.0)))
    out.append(("no head", p, ly.with_(head_len=1.0, head_half_w=1.0, ear_z=1.0, ear_x=0.0, chin_z=-1.0)))
    return out


# ---- the verdict -------------------------------------------------------------------------------------
@dataclass
class Verdict:
    rest_ok: bool
    outcome_ok: bool
    motion_ok: bool
    knife_edge_flips: bool | None   # None: the bands were not run
    lines: list

    @property
    def credible(self) -> bool:
        return bool(self.rest_ok and self.outcome_ok and self.motion_ok and self.knife_edge_flips)


KNIFE = ("b8@120s try 1", "b8@120s try 3")   # the two that began alike and ended differently


def verdict(rs: Sequence[Replay], banded: Mapping[str, Sequence[Replay]] | None = None) -> Verdict:
    """CRITERION against the replays with the fitted parameters (``rs``) and, for the knife edge,
    across the bands (``banded``: band name -> its replays)."""
    lines = []
    rs = [r for r in rs if r.floor]
    lyingdown = [r for r in rs if r.start_lying != "upright"]
    worst_rest = max(r.rest_err for r in lyingdown)
    rest_ok = worst_rest <= REST_TOL_DEG
    lines.append(f"(a) rest: the model moved at most {worst_rest:.0f} deg from where Kilo lay "
                 f"(allowed {REST_TOL_DEG:g}): {'met' if rest_ok else 'NOT met'}")
    ups = [r for r in rs if r.recorded_righted]
    downs = [r for r in rs if not r.recorded_righted]
    got_up = sum(r.righted for r in ups)
    stayed = sum(not r.righted for r in downs)
    outcome_ok = got_up == len(ups) and stayed == len(downs)
    lines.append(f"(b) outcome: upright in the model for {got_up} of the {len(ups)} floor tries that ended upright, "
                 f"not upright for {stayed} of the {len(downs)} that did not (needs all of both): "
                 f"{'met' if outcome_ok else 'NOT met'}")
    mot = [r for r in ups if r.start_lying != "upright"]
    bad = [r for r in mot if r.trace_rms > TRACE_TOL_DEG or r.side_t is None or r.side_t_rec is None
           or abs(r.side_t - r.side_t_rec) > SIDE_TOL_S]
    motion_ok = not bad
    lines.append(f"(c) motion: {len(mot) - len(bad)} of the {len(mot)} recorded gettings-up are followed (through "
                 f"his side within {SIDE_TOL_S:g} s, rms under {TRACE_TOL_DEG:g} deg): {'met' if motion_ok else 'NOT met'}")
    flips = None
    if banded is not None:
        outs = [r.righted for name, band in banded.items() for r in band if r.label in KNIFE]
        frac = sum(outs) / max(len(outs), 1)
        flips = 0.1 <= frac <= 0.9
        lines.append(f"(b') the knife edge: upright in {100 * frac:.0f} % of the bands for the start that went both "
                     f"ways on Kilo (needs 10..90 %): {'met' if flips else 'NOT met'}")
    return Verdict(rest_ok, outcome_ok, motion_ok, flips, lines)


def table(rs: Sequence[Replay]) -> str:
    out = ["try                        began                                      rc        Kilo      model     rest  rms   through his side (Kilo / model)  ends (model)                torque",
           "                                                                                                     deg   deg   s                                                            kg cm"]
    for r in rs:
        f = lambda v: "   - " if v is None else f"{v:5.2f}"
        out.append(f"{r.label + ('' if r.floor else ' (hand)'):26s} {r.start_lying:42s} {'mirrored' if r.mirrored else 'as is   '}  "
                   f"{'upright' if r.recorded_righted else 'not    '}   {'upright' if r.righted else 'not    '}  "
                   f"{r.rest_err:5.0f} {r.trace_rms:5.0f}   {f(r.side_t_rec)} / {f(r.side_t)}                    "
                   f"{r.end_lying:27s} {r.torque:4.1f}")
    return "\n".join(out)


def falls_table(fx: dict | None = None) -> str:
    fx = fx or fixture()
    out = []
    for f in fx["falls"]:
        out.append(f"{f['date']}  {f['file']} at {f['uptime_s']:.0f} s: FELL {f['fell_arg']:+d} while {f['doing']} "
                   f"(mode {f['mode_before']}); {f['ended']} after {f['ended_after_s']:.1f} s"
                   + ("" if f.get("floor", True) else "; IN A HAND (" + "; ".join(f["picked_up_lines"])[:70] + ")"))
        for t in f["tries"]:
            moved = t["moved_before_next_try_deg"]
            out.append(f"    try {t['n']}: began {t['start_lying']} (pitch {t['start'][0]:+.0f} roll {t['start'][1]:+.0f}), rc "
                       f"{'mirrored' if t['mirrored'] else 'as written'}: after the moves {t['at_moves_end_lying']}, "
                       f"ended {t['end_lying']}" + (f"; then moved {moved:.0f} deg before the next try"
                                                     if moved else ""))
        if not f["tries"]:
            out.append("    no try: the body does not try from nose or tail down")
    return "\n".join(out)


def rest_check(p: D.Params = D.FITTED, ly: Lying | None = None) -> list[tuple[str, float, tuple, tuple]]:
    """(try, degrees the model moved from the recorded lying attitude while settling, the recorded
    (pitch, roll), the model's) for each try that began lying down."""
    out = []
    for f, n in tries():
        tr = f["tries"][n - 1]
        if tr["start_lying"] == "upright":
            continue
        up0 = up_of(*tr["start"])
        kin = KinLying(p, ly)
        st = start_joints(f, n)
        pr = D.simulate([(st, 1, 0)], p, start=st, settle_s=1.5, t_end=0.1, R0=r0_for(up0), kin=kin)
        up = np.array(pr.up)[-1]
        out.append((f"{f['file'].replace('rec-card-', '').replace('.krec', '')}@{f['uptime_s']:.0f}s try {n}",
                    angle(up, up0), tuple(tr["start"]), tuple(round(v, 1) for v in telemetry_of(up))))
    return out


# ---- rc frame by frame ------------------------------------------------------------------------------
PHASE_ENDS_S = (0.88, 1.42, 1.95)   # just after rc's first, second and third frame (RC_TICKS, the lag)


def roll_angle(u: np.ndarray) -> float:
    """The roll about the long axis, deg: 0 upright, -90 on his right side, +-180 on his back."""
    return math.degrees(math.atan2(-u[1], u[2]))


def phases(p: D.Params = D.FITTED, ly: Lying | None = None, workers: int = 4) -> list[dict]:
    """Per floor try: the roll angle at the end of each rc frame, Kilo's and the model's."""
    fl = [(f, n) for f, n in tries() if f.get("floor", True)]
    rs = [replay(f, n, p, ly) for f, n in fl]
    out = []
    for (f, n), r in zip(fl, rs):
        rec = np.array(f["tries"][n - 1]["trace"], float)
        kilo = [roll_angle(up_of(*rec[int(np.argmin(abs(rec[:, 0] - t))), 1:3])) for t in PHASE_ENDS_S]
        model = [roll_angle(r.up[min(int(np.searchsorted(r.t, t)), len(r.t) - 1)]) for t in PHASE_ENDS_S]
        out.append({"try": r.label, "mirrored": r.mirrored, "kilo": kilo, "model": model})
    return out


def phases_table(rows: list[dict]) -> str:
    out = ["try                 rc        Kilo after frame 1, 2, 3     model after frame 1, 2, 3   (deg: 0 upright, "
           "-90 right side, +-180 back)"]
    for r in rows:
        out.append(f"{r['try']:19s} {'mirrored' if r['mirrored'] else 'as is   '}  "
                   + " ".join(f"{v:+5.0f}" for v in r["kilo"]) + "             "
                   + " ".join(f"{v:+5.0f}" for v in r["model"]))
    return "\n".join(out)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m kilo.tricks.righting", description=__doc__.split("\n\n")[0])
    ap.add_argument("--falls", action="store_true")
    ap.add_argument("--rest", action="store_true")
    ap.add_argument("--replay", action="store_true")
    ap.add_argument("--bands", action="store_true")
    ap.add_argument("--phases", action="store_true")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--extract", nargs="+", metavar="FILE.krec",
                    help="instead: the falls in these flight recordings as JSON (the fixture's entries), to add to "
                         "mind/tests/fixtures/falls_*.json with the date, what he was doing and what you saw")
    ap.add_argument("--date", default="")
    ap.add_argument("--doing", default="")
    ap.add_argument("--saw", default="nothing reported", help="with --extract: what the owner saw")
    a = ap.parse_args(argv)
    if a.extract:
        falls = [f for path in a.extract for f in falls_in(path, a.date, a.doing, a.saw)]
        json.dump({"falls": falls}, sys.stdout, separators=(",", ":"))
        print()
        print(falls_table({"falls": falls}), file=sys.stderr)
        return 0
    if a.phases:
        print(phases_table(phases()))
        return 0
    if a.falls or not (a.rest or a.replay or a.bands):
        print(falls_table())
    if a.rest:
        for label, err, rec, mod in rest_check():
            print(f"{label:20s} Kilo lay at pitch {rec[0]:+7.1f} roll {rec[1]:+7.1f}; the model settles at "
                  f"{mod[0]:+7.1f} {mod[1]:+7.1f}: {err:4.0f} deg apart")
    if a.replay or a.bands:
        rs = replay_all(workers=a.workers)
        print(table(rs))
        banded = None
        if a.bands:
            banded = {}
            for name, p, ly in bands():
                banded[name] = _map([(f, n, p, ly) for f, n in tries() if f.get("floor", True)], a.workers)
                got = "".join("U" if r.righted else "." for r in banded[name])
                print(f"  {name:28s} {got}")
            print("  (one letter per floor try, in the table's order: U upright in the model)")
        v = verdict(rs, banded)
        print("\n".join(v.lines))
        print("credible for designing a getting-up: " + ("yes" if v.credible else
              "no" if banded is not None else "not decided without --bands" if (v.rest_ok and v.outcome_ok and v.motion_ok) else "no"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
