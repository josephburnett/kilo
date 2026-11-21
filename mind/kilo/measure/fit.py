"""The fit: Kilo's measured tilt through the self-measurement routine,
explained by a few numbers about his body (docs/self-measurement.md).

THE MEASUREMENT MODEL. For the feet the routine commanded at each tick:

1. the compliant statics (kilo.measure.statics) give the tilt the body
   rests at, from the centre of mass (com_x, com_y, com_z), the legs' sag
   and their ``twist``: the LF and RB legs together longer than the RF and
   LB ones (a table with one short leg). The other ways the four legs can
   be off are not fitted, because no tilt tells them from something else:
   all four longer is a height; the front pair longer than the back looks
   exactly like an IMU pitch offset, the left pair longer like a roll
   offset;
2. the servos get there late: a dead time ``lag_ms``, then a first order
   response of time constant ``tau_ms`` (the planner's servo model);
3. the IMU reads it plus an offset (imu_pitch, imu_roll), which therefore
   also holds the floor's slope and the legs' front/back and left/right
   difference. A second run with Kilo turned round (``--turned``) takes the
   floor out (floor_pitch, floor_roll).

During a sweep the support the body stands on is taken from the data (on
the tripod before the tip was seen, on the lifted foot after), and the
ticks around the tip are left out of the tick-by-tick part: there the tilt
jumps and the inverted pendulum, not the statics, decides it. Instead each
sweep's tip gives one CROSSING: the feet's x at which the centre of mass
crossed the edge. The sweep moves 1 mm every TICKS_PER_MM ticks; the tilt
passes TIP_FRACTION of the tip ``tip_ms`` after the step that put the
centre of mass over the edge (the servos' delay and the pendulum's growth:
a fitted number, which the sweeps the other way along the same edge pin
down), so the half-way point of that step is the crossing (``crossing_at``).
The model's crossing is where its centre of mass, on the tripod, is over
the edge (statics.edge_distance). The crossings locate the centre of mass:
a 1 mm error in one is about 0.5 mm of the centre of mass across the edge.

THE FIT. Least squares (Levenberg-Marquardt, numerical derivatives) of the
tick residuals (measured minus modelled pitch and roll, the tip windows
left out) and the crossing residuals, each over its own spread. The
uncertainties come from the curvature, the tick residuals' spread taken
from the fit itself and inflated by how many ticks they stay correlated
(servo backlash makes errors that last a whole hold, not one tick). The
simulation check (kilo.measure.simulate) shows how honest the +- are.

WALKING RUNS (``--walk``). A walk's error stream (measured minus the
planner's prediction plus the body's correction) is the same servo
response under walking loads: it adds to lag_ms and tau_ms, with each
walk's own offset free (a walk's mean error mixes the IMU offset with the
planner's lean, so it says nothing about the centre of mass or the IMU).
Only ticks with three or four feet down are used (on two the body pivots
and the prediction is not a servo response) and, when the body's balance
was on, only those where it held its correction (a foot up): elsewhere the
correction follows the measurement itself and the fit would find no delay.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from kilo.measure import script as SC
from kilo.measure import statics as ST
from kilo.planner import model as PM
from kilo.planner.model import BodyModel

TICK_MS = SC.TICK_MS
SIGMA_TICK_DEG = 0.2        # the tick residuals' spread to start with (re-estimated; a walk's its own)
SD_SCALE = 1.3              # the curvature's +- times this: what the simulation check needs (kilo.measure.simulate)
                            # for the truth to be within 2 +- in about 95 % of runs (1.0 left it at 80-90 %)
SIGMA_CROSS_MM = 0.6        # a crossing's least spread: where in its 1 mm step it fell (uniform: 0.29), and the
                            # pendulum's time to tip, which depends on it (simulate check: 0.4 left the +- short)
TIP_WINDOW_BEFORE_MM = 2    # tick residuals: leave out the sweep within this of the crossing
TIP_WINDOW_AFTER = 20       # ...and this many ticks after the tip was seen
TIP_JUMP = 0.3              # a tip: the tilt moves this much of the model's tip
TIP_JUMP_TICKS = 12         # ...from this many ticks before it was seen to half as many after
P_STANCE = 205              # spec gait.stance_h_256


@dataclass(frozen=True)
class Param:
    name: str
    unit: str
    what: str
    step: float             # the numerical derivative's step
    lo: float = -math.inf
    hi: float = math.inf
    statics: bool = True    # it changes the statics (they are cached otherwise)


PARAMS = (
    Param("imu_pitch", "deg", "IMU pitch offset (+ floor slope, + front/back legs)", 0.01, statics=False),
    Param("imu_roll", "deg", "IMU roll offset (+ floor slope, + left/right legs)", 0.01, statics=False),
    Param("com_x", "mm", "centre of mass, ahead of the shoulders' middle", 0.05, -40, 40),
    Param("com_y", "mm", "centre of mass, to the left", 0.05, -20, 20),
    Param("com_z", "mm", "centre of mass, above the shoulder axles", 0.2, -40, 40),
    Param("sag", "deg/kg cm", "leg servos' give under load", 0.05, ST.SAG_MIN, 20),
    Param("twist", "mm", "LF+RB legs longer than RF+LB (each pair by half)", 0.02, -6, 6),
    Param("lag_ms", "ms", "servo dead time (with the IMU's own delay)", 0.5, 0, 300, statics=False),
    Param("tau_ms", "ms", "servo time constant", 0.5, 2, 400, statics=False),
    Param("tip_ms", "ms", "a tip's delay: the step over the edge to 40 % of the tip", 1.0, 0, 1000, statics=False),
)
NAMES = tuple(p.name for p in PARAMS)
ROWS = NAMES


def model_values(model: BodyModel | None = None) -> dict[str, float]:
    """What the walks' model says today for each parameter (tip_ms: its
    servo alone, a step's response reaching TIP_FRACTION; no pendulum)."""
    m = model or BodyModel()
    cx, cy, cz = m.com([0.0] * 4, [float(SC.HOME_Z)] * 4)
    tip = m.servo_lag_ms - m.servo_tau_ms * math.log(1.0 - SC.TIP_FRACTION)
    return {"imu_pitch": 0.0, "imu_roll": 0.0, "com_x": cx, "com_y": cy, "com_z": cz, "sag": 0.0, "twist": 0.0,
            "lag_ms": m.servo_lag_ms, "tau_ms": m.servo_tau_ms, "tip_ms": tip}


def body_of(v: dict[str, float], model: BodyModel) -> ST.Body:
    mv = model_values(model)
    t = 0.5 * v["twist"]
    return ST.Body(com_dx=v["com_x"] - mv["com_x"], com_dy=v["com_y"] - mv["com_y"], com_dz=v["com_z"] - mv["com_z"],
                   sag=v["sag"], leg_dz=(t, -t, t, -t), model=model)


# ---- the data ----------------------------------------------------------------------------------
@dataclass
class Sweep:
    exp: int
    lifted: int
    direction: int              # +1: the feet's x grows
    ticks: np.ndarray           # the run's tick indices of the sweep phase
    z: tuple                    # the legs' z during it
    lifted_ticks: np.ndarray    # the "lifted" hold before it
    after: np.ndarray           # the lower and lowered phases after it
    x0: float = 0.0             # the feet's x before the sweep's first step
    tip_tick: float | None = None  # the tip seen (fractional run tick), None: none in the sweep
    note: str = ""

    def crossing_at(self, tip_ms: float) -> float | None:
        """The crossing (feet's x) if the tip is seen ``tip_ms`` after the
        step over the edge: that step's middle. Step n (1..) starts at the
        sweep's tick 6 (n - 1) + 1 and goes to x0 + dir n."""
        if self.tip_tick is None:
            return None
        j = self.tip_tick - self.ticks[0] + 1 - tip_ms / TICK_MS   # the sweep tick the step began
        n = (j - 1) / SC.TICKS_PER_MM + 1
        return self.x0 + self.direction * (n - 0.5)


@dataclass
class Run:
    """One measurement run as the fit reads it: every script tick."""
    name: str
    x: np.ndarray               # (N, 4) commanded
    z: np.ndarray
    exp: np.ndarray             # (N,) EXPERIMENTS index
    phase: list                 # (N,) the step's phase name
    meas: np.ndarray            # (N, 2) measured pitch, roll (NaN: no TELEMETRY)
    pred: np.ndarray            # (N, 2) the routine's prediction (FEET pitch, roll; NaN if none)
    batt_mv: np.ndarray         # (N,) NaN if none
    rate: np.ndarray | None = None  # (N, 2) TELEMETRY rate_pitch, rate_roll
    heading: int = 1            # -1: turned round (the floor's slope the other way)
    meta: dict = field(default_factory=dict)
    sweeps: list = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.x)


@dataclass
class Walk:
    """A walk's error stream: the planner's prediction plus the body's
    correction (u) and the measured attitude, ticks with 3+ feet down."""
    name: str
    u: np.ndarray               # (N, 2) every tick from the first to the last
    meas: np.ndarray            # (N, 2)
    used: np.ndarray            # (N,) bool


def find_sweeps(run: Run) -> list[Sweep]:
    out = []
    for e, ex in enumerate(SC.EXPERIMENTS):
        if ex.lifted < 0:
            continue
        idx = np.array([i for i in range(len(run)) if run.exp[i] == e and run.phase[i] == "sweep"], int)
        if len(idx) == 0:
            continue
        lt = np.array([i for i in range(len(run)) if run.exp[i] == e and run.phase[i] == "lifted"], int)
        af = np.array([i for i in range(len(run)) if run.exp[i] == e and run.phase[i] in ("lower", "lowered")], int)
        d = 1 if run.x[idx[-1], 0] > run.x[idx[0], 0] else -1
        x0 = float(run.x[idx[0], 0]) - d
        out.append(Sweep(e, ex.lifted, d, idx, tuple(run.z[idx[0]]), lt, af, x0))
    return out


def tip_vector(e: int) -> np.ndarray:
    return np.array(SC.EXPERIMENTS[e].tip_a16, float) / 16.0


def find_tip(run: Run, sw: Sweep) -> None:
    """When the sweep's tip was seen (sw.tip_tick, sw.note): the tilt
    toward the lifted foot, from the settle hold's (all four feet down, as
    the routine takes it), passes TIP_FRACTION of the model's tip (to a
    fraction of a tick, linearly). Already past it at the end of the lifted
    hold: he tipped as the foot came up, and the sweep gives no crossing."""
    u = tip_vector(sw.exp)
    T = float(np.hypot(*u))
    u = u / T
    settle = [i for i in range(len(run)) if run.exp[i] == sw.exp and run.phase[i] == "settle"]
    if not settle or len(sw.lifted_ticks) == 0:
        sw.note = "no settle or lifted hold"
        return
    base = np.nanmean(run.meas[settle], 0)
    if not np.all(np.isfinite(base)):
        sw.note = "no TELEMETRY before the lift"
        return
    if np.nanmean(run.meas[sw.lifted_ticks[-5:]] - base, 0) @ u > SC.TIP_FRACTION * T:
        sw.note = "tipped as the foot came up: the crossing is before the sweep's start"
        return
    ks = np.concatenate([sw.ticks, sw.after])
    proj = (run.meas[ks] - base) @ u
    hit = np.where(proj >= SC.TIP_FRACTION * T)[0]
    if len(hit) == 0:
        sw.note = "no tip in the sweep: the crossing is past its end"
        return
    j = hit[0]
    # a tip is a jump: the tilt moves TIP_JUMP of the model's tip within TIP_JUMP_TICKS around it; a slow lean
    # (legs that give so much, or a foot that does not clear, that he leans onto it) has no crossing to time
    lo_, hi_ = max(j - TIP_JUMP_TICKS, 0), min(j + TIP_JUMP_TICKS // 2, len(proj) - 1)
    if not np.isfinite(proj[lo_]) or not np.isfinite(proj[hi_]) or proj[hi_] - proj[lo_] < TIP_JUMP * T:
        sw.note = "a slow lean, not a tip (the legs give, or the lifted foot did not clear): no crossing"
        return
    k = float(ks[j])
    if j > 0 and np.isfinite(proj[j - 1]) and proj[j] > proj[j - 1]:
        k -= (proj[j] - SC.TIP_FRACTION * T) / (proj[j] - proj[j - 1])
    sw.tip_tick = k


def model_crossing(model: BodyModel, body: ST.Body, sw: Sweep, x_lo: float = -45.0, x_hi: float = 45.0) -> float:
    """The feet's x (all four) at which the model's centre of mass is over
    the sweep's edge, its legs at the sweep's z."""
    def f(x):
        return ST.edge_distance(model, [x] * 4, sw.z, body, sw.lifted)

    a, b = x_lo, x_hi
    fa = f(a)
    fb = f(b)
    if (fa > 0) == (fb > 0):
        return a if abs(fa) < abs(fb) else b
    for _ in range(36):
        c = 0.5 * (a + b)
        fc = f(c)
        if (fc > 0) == (fa > 0):
            a, fa = c, fc
        else:
            b = c
    return 0.5 * (a + b)


# ---- the model's tilt for a run ------------------------------------------------------------------
_OPPOSITE = {PM.LF: PM.RB, PM.RB: PM.LF, PM.RF: PM.LB, PM.LB: PM.RF}


class RunModel:
    """A run prepared for the fit: its unique poses, which support each
    sweep tick may not be on (from the data: not yet tipped before the tip
    was seen, tipped after; the four feet or the other tripod as the statics
    decide), the ticks the tick residuals use."""

    def __init__(self, run: Run, model: BodyModel, tip_ms: float):
        self.run = run
        self.model = model
        keys = np.hstack([run.x, run.z])
        u, inv = np.unique(keys, axis=0, return_inverse=True)
        self.poses = ST.Poses(model, u[:, :4], u[:, 4:])
        self.inv = inv.ravel()
        n = len(run)
        self.exclude = np.full(n, -1)    # a candidate the tick may not be on, -1 none
        use = np.isfinite(run.meas).all(1)
        for sw in run.sweeps:
            off_tripod = 1 + sw.lifted                       # candidate: the lifted foot in the air
            off_tipped = 1 + _OPPOSITE[sw.lifted]           # ...the far foot of the diagonal in the air
            if sw.tip_tick is None:
                if "past its end" in sw.note:
                    self.exclude[sw.ticks] = off_tipped
                    self.exclude[sw.lifted_ticks] = off_tipped
                else:  # tipped as the foot came up (or no data): the lift and the sweep left out
                    use[sw.ticks] = False
                    use[sw.lifted_ticks] = False
                    lift = [i for i in range(n) if run.exp[i] == sw.exp and run.phase[i] == "lift"]
                    use[lift] = False
                continue
            kt = sw.tip_tick
            for i in sw.ticks:
                self.exclude[i] = off_tipped if i < kt else off_tripod
            self.exclude[sw.lifted_ticks] = off_tipped
            cx = sw.crossing_at(tip_ms)
            near = np.abs(run.x[sw.ticks, 0] - cx) <= TIP_WINDOW_BEFORE_MM + 0.5
            use[sw.ticks[near & (sw.ticks < kt)]] = False
            w = np.arange(int(math.floor(kt)) - 3 * SC.TICKS_PER_MM, int(math.ceil(kt)) + TIP_WINDOW_AFTER)
            use[w[(w >= 0) & (w < n)]] = False
        self.use = use
        self._cache: dict = {}

    def statics(self, body: ST.Body) -> np.ndarray:
        key = (body.com_dx, body.com_dy, body.com_dz, body.sag, tuple(body.leg_dz))
        hit = self._cache.get(key)
        if hit is not None:
            return hit
        c = ST.candidates(self.poses, body)
        valid = c.valid()
        n = len(self.run)
        out = np.empty((n, 2))
        state = 0
        for t in range(n):
            i = self.inv[t]
            x = self.exclude[t]
            if state == x or not valid[i, state]:  # the possible one (the least impossible), not the excluded
                bad = c.bad[i].copy()
                if x >= 0:
                    bad[x] = np.inf
                state = int(np.argmin(bad))
            out[t, 0] = c.pitch[i, state]
            out[t, 1] = c.roll[i, state]
        if len(self._cache) > 64:
            self._cache.clear()
        self._cache[key] = out
        return out


def servo(s: np.ndarray, lag_ms: float, tau_ms: float) -> np.ndarray:
    """The servo response to the static tilt sequence s (N, 2): s delayed
    lag_ms (linearly interpolated between ticks, s[0] before the start),
    then the first-order lag of tau_ms, tick by tick."""
    n = len(s)
    d = max(lag_ms, 0.0) / TICK_MS
    t = np.arange(n) - d
    i0 = np.clip(np.floor(t).astype(int), 0, n - 1)
    i1 = np.clip(i0 + 1, 0, n - 1)
    f = np.clip(t - np.floor(t), 0.0, 1.0)[:, None]
    f = np.where((t < 0)[:, None], 0.0, f)
    u = (s[i0] * (1 - f) + s[i1] * f).tolist()
    a = 1.0 - math.exp(-TICK_MS / max(tau_ms, 1e-3))
    yl = [None] * n
    p0, p1 = u[0][0], u[0][1]
    for k in range(n):  # y[k] = y[k-1] + a (u[k] - y[k-1]) (numpy has no lfilter)
        p0 += a * (u[k][0] - p0)
        p1 += a * (u[k][1] - p1)
        yl[k] = (p0, p1)
    return np.array(yl)


# ---- the fit ------------------------------------------------------------------------------------
@dataclass
class FitResult:
    values: dict
    sd: dict
    model: dict
    names: list
    cov: np.ndarray
    rms_tick_deg: float
    rms_model_deg: float | None  # the routine's own (rigid) prediction against the measurement
    inflation: float
    n_ticks: int
    crossings: list
    tip_rate: dict
    walks: list
    notes: list
    iterations: int
    battery_mv: tuple | None = None

    def significant(self, name: str) -> bool:
        return name in self.names and abs(self.values[name] - self.model.get(name, 0.0)) > 2.0 * self.sd[name]


class Fitter:
    def __init__(self, runs: list[Run], walks: list[Walk] | None = None, model: BodyModel | None = None,
                 fixed: dict | None = None):
        self.model = model or BodyModel()
        self.runs = runs
        self.walks = walks or []
        self.fixed = dict(fixed or {})
        if not runs:  # walks alone: they time the servos and say nothing else (the module docstring)
            mv0 = model_values(self.model)
            self.fixed.update({p.name: mv0[p.name] for p in PARAMS if p.name not in ("lag_ms", "tau_ms")})
        self.mv = model_values(self.model)
        self.notes: list[str] = []
        self.floor = len({r.heading for r in runs}) > 1
        names = [p.name for p in PARAMS if p.name not in self.fixed]
        if self.floor:
            names += ["floor_pitch", "floor_roll"]
        for w in self.walks:
            names += [f"walk_pitch:{w.name}", f"walk_roll:{w.name}"]
        self.names = names
        self.steps = {p.name: p.step for p in PARAMS}
        self.bounds = {p.name: (p.lo, p.hi) for p in PARAMS}
        for n in names:
            self.steps.setdefault(n, 0.01)
            self.bounds.setdefault(n, (-math.inf, math.inf))
        self.rms: list[RunModel] = []
        self.sigma_t = SIGMA_TICK_DEG
        self.sigma_w = SIGMA_TICK_DEG
        self.sigma_c = SIGMA_CROSS_MM
        self.n_ticks = 0

    def start(self) -> np.ndarray:
        v = []
        for n in self.names:
            if n == "sag":
                v.append(1.5)
            elif n == "tip_ms":
                v.append(self.mv["tip_ms"] + 60.0)
            else:
                v.append(self.mv.get(n, 0.0))
        return np.array(v, float)

    def values(self, th: np.ndarray) -> dict:
        d = dict(self.mv)
        d.update(self.fixed)
        for n, x in zip(self.names, th):
            d[n] = float(x)
        return d

    def prepare(self, th: np.ndarray) -> None:
        v = self.values(th)
        for r in self.runs:
            r.sweeps = find_sweeps(r)
            for sw in r.sweeps:
                find_tip(r, sw)
        self.rms = [RunModel(r, self.model, v["tip_ms"]) for r in self.runs]
        self.n_ticks = int(sum(rm.use.sum() for rm in self.rms))

    def residuals(self, th: np.ndarray, parts: bool = False):
        v = self.values(th)
        body = body_of(v, self.model)
        res_t, res_c, res_w = [], [], []
        for rm in self.rms:
            r = rm.run
            s = rm.statics(body)
            y = servo(s, v["lag_ms"], v["tau_ms"])
            off = np.array([v["imu_pitch"], v["imu_roll"]])
            if self.floor:
                off = off + r.heading * np.array([v["floor_pitch"], v["floor_roll"]])
            e = (y + off - r.meas)[rm.use]
            res_t.append(e.ravel())
            for sw in r.sweeps:
                cx = sw.crossing_at(v["tip_ms"])
                if cx is None:
                    continue
                res_c.append(model_crossing(self.model, body, sw) - cx)
        for w in self.walks:
            y = servo(w.u, v["lag_ms"], v["tau_ms"])
            off = np.array([v[f"walk_pitch:{w.name}"], v[f"walk_roll:{w.name}"]])
            res_w.append((y + off - w.meas)[w.used].ravel())
        rt = np.concatenate(res_t) if res_t else np.zeros(0)
        rc = np.array(res_c, float)
        rw = np.concatenate(res_w) if res_w else np.zeros(0)
        if parts:
            return rt, rc, rw
        return np.concatenate([rt / self.sigma_t, rc / self.sigma_c, rw / self.sigma_w])

    def jacobian(self, th: np.ndarray, r0: np.ndarray) -> np.ndarray:
        J = np.empty((len(r0), len(th)))
        for j, n in enumerate(self.names):
            h = self.steps[n]
            lo, hi = self.bounds[n]
            t2 = th.copy()
            if th[j] + h > hi:
                h = -h
            t2[j] += h
            J[:, j] = (self.residuals(t2) - r0) / h
        return J

    def clip(self, th: np.ndarray) -> np.ndarray:
        out = th.copy()
        for j, n in enumerate(self.names):
            lo, hi = self.bounds[n]
            out[j] = min(max(out[j], lo), hi)
        return out

    def _lm(self, th: np.ndarray, max_iter: int) -> tuple[np.ndarray, int]:
        lam = 1e-2
        r = self.residuals(th)
        cost = float(r @ r)
        it = 0
        for it in range(1, max_iter + 1):
            J = self.jacobian(th, r)
            A = J.T @ J
            g = J.T @ r
            improved = small = False
            for _ in range(12):
                D = np.diag(np.diag(A)) * lam + np.eye(len(th)) * 1e-9
                try:
                    step = -np.linalg.solve(A + D, g)
                except np.linalg.LinAlgError:
                    lam *= 10
                    continue
                t2 = self.clip(th + step)
                r2 = self.residuals(t2)
                c2 = float(r2 @ r2)
                if c2 < cost:
                    small = cost - c2 < 1e-7 * cost
                    th, r, cost = t2, r2, c2
                    lam = max(lam / 3, 1e-7)
                    improved = True
                    break
                lam *= 4
            if not improved or small:
                break
        return th, it

    def run(self, max_iter: int = 40) -> FitResult:
        th = self.start()
        self.prepare(th)
        th, it1 = self._lm(th, 6)
        # again with the tip windows where the fitted tip delay puts them, and the spreads re-estimated
        self.prepare(th)
        rt, rc, rw = self.residuals(th, parts=True)
        if len(rt):
            self.sigma_t = max(float(np.sqrt(np.mean(rt ** 2))), 0.02)
        if len(rw):
            self.sigma_w = max(float(np.sqrt(np.mean(rw ** 2))), 0.02)
        if len(rc):
            self.sigma_c = max(float(np.sqrt(np.mean(rc ** 2))), SIGMA_CROSS_MM)
        th, it2 = self._lm(th, max_iter)
        return self.result(th, it1 + it2)

    def result(self, th: np.ndarray, iterations: int) -> FitResult:
        rt, rc, rw = self.residuals(th, parts=True)
        s_t = float(np.sqrt(np.mean(rt ** 2))) if len(rt) else 0.1
        s_w = float(np.sqrt(np.mean(rw ** 2))) if len(rw) else 0.1
        infl = self._inflation(th)
        # the crossings' spread from their residuals, allowing for the four numbers they mostly fit (com_x, com_y,
        # com_z, tip_ms)
        s_c = max(SIGMA_CROSS_MM, float(np.sqrt(np.mean(rc ** 2)) * math.sqrt(len(rc) / max(len(rc) - 4, 1))))\
            if len(rc) else SIGMA_CROSS_MM
        self.sigma_t, self.sigma_c, self.sigma_w = s_t * math.sqrt(infl), s_c, s_w * math.sqrt(infl)
        r = self.residuals(th)
        J = self.jacobian(th, r)
        try:
            cov = np.linalg.inv(J.T @ J)
        except np.linalg.LinAlgError:
            cov = np.linalg.pinv(J.T @ J)
        cov = cov * SD_SCALE ** 2
        v = self.values(th)
        sd = {n: float(math.sqrt(max(cov[j, j], 0.0))) for j, n in enumerate(self.names)}
        for n in self.fixed:
            sd[n] = 0.0
        crossings = []
        body = body_of(v, self.model)
        nominal = ST.Body(model=self.model)
        for r_ in self.runs:
            for sw in r_.sweeps:
                cx = sw.crossing_at(v["tip_ms"])
                d = {"run": r_.name, "experiment": SC.EXPERIMENTS[sw.exp].name,
                     "measured_x": None if cx is None else round(cx, 2), "note": sw.note,
                     "model_x": round(model_crossing(self.model, nominal, sw), 2)}
                if cx is not None:
                    d["fitted_x"] = round(model_crossing(self.model, body, sw), 2)
                crossings.append(d)
        pred_rms = None
        errs = []
        batt = []
        for r_ in self.runs:
            ok = np.isfinite(r_.meas).all(1) & np.isfinite(r_.pred).all(1)
            if ok.any():
                errs.append((r_.meas[ok] - r_.pred[ok]).ravel())
            b = r_.batt_mv[np.isfinite(r_.batt_mv)]
            if len(b):
                batt += [float(b.min()), float(b.max())]
        if errs:
            pred_rms = float(np.sqrt(np.mean(np.concatenate(errs) ** 2)))
        return FitResult(values=v, sd=sd, model=dict(self.mv), names=list(self.names), cov=cov, rms_tick_deg=s_t,
                         rms_model_deg=pred_rms, inflation=infl, n_ticks=len(rt) // 2, crossings=crossings,
                         tip_rate=tip_rates(self.runs, v), walks=[w.name for w in self.walks], notes=self.notes,
                         iterations=iterations, battery_mv=(min(batt), max(batt)) if batt else None)

    def _inflation(self, th: np.ndarray) -> float:
        """How many ticks a tick residual stays correlated for (1 + 2 x the
        sum of its autocorrelation), the median over the runs' channels."""
        v = self.values(th)
        body = body_of(v, self.model)
        taus = []
        series = []
        for rm in self.rms:
            s = rm.statics(body)
            y = servo(s, v["lag_ms"], v["tau_ms"])
            series.append((y + np.array([v["imu_pitch"], v["imu_roll"]]) - rm.run.meas, rm.use))
        for w in self.walks:
            y = servo(w.u, v["lag_ms"], v["tau_ms"])
            series.append((y + np.array([v[f"walk_pitch:{w.name}"], v[f"walk_roll:{w.name}"]]) - w.meas, w.used))
        for e, use in series:
            for ch in range(2):
                x = np.where(use, e[:, ch], np.nan)
                x = x - np.nanmean(x)
                ok = np.isfinite(x)
                x0 = np.where(ok, x, 0.0)
                var = float(np.sum(x0 * x0)) / max(ok.sum(), 1)
                if var <= 0:
                    continue
                t = 1.0
                for lag in range(1, 150):
                    both = ok[lag:] & ok[:-lag]
                    if both.sum() < 20:
                        break
                    rho = float(np.sum(x0[lag:] * x0[:-lag])) / both.sum() / var
                    if rho < 0.05:
                        break
                    t += 2 * rho
                taus.append(t)
        return float(np.median(taus)) if taus else 1.0


def tip_rates(runs: list[Run], v: dict) -> dict:
    """The owner's tip rate: how fast each tip grows. The tilt toward the
    lifted foot between 10 % and 70 % of the tip, fitted with an
    exponential e^(t / tau) (an inverted pendulum's), tau in ms. A point
    mass on a pendulum of length l grows so with tau = sqrt(l / g): the
    centre of mass's height over the edge, if Kilo were one."""
    taus = []
    for r in runs:
        for sw in r.sweeps:
            if sw.tip_tick is None or len(sw.lifted_ticks) == 0:
                continue
            u = tip_vector(sw.exp)
            T = float(np.hypot(*u))
            u = u / T
            settle = [i for i in range(len(r)) if r.exp[i] == sw.exp and r.phase[i] == "settle"]
            base = np.nanmean(r.meas[settle], 0)
            ks = np.arange(max(int(sw.tip_tick) - 15, 0), min(int(sw.tip_tick) + 10, len(r)))
            pr = (r.meas[ks] - base) @ u / T
            sel = (pr > 0.1) & (pr < 0.7) & np.isfinite(pr)
            if sel.sum() < 3 or (ks[sel][-1] - ks[sel][0]) > 15:  # not a tip: a slow lean (soft legs)
                continue
            tt = ks[sel] * TICK_MS
            yy = np.log(pr[sel])
            A = np.vstack([tt, np.ones_like(tt)]).T
            slope = np.linalg.lstsq(A, yy, rcond=None)[0][0]
            if slope > 0:
                taus.append(1.0 / slope)
    out: dict[str, Any] = {"events": len(taus)}
    m = BodyModel()
    h = m.stance_height(P_STANCE) + m.foot_r_mm + v.get("com_z", 0.0)
    out["model_ms"] = round(1000.0 * math.sqrt(h / PM.G_MM_S2), 1)
    if taus:
        med = float(np.median(taus))
        out["median_ms"] = round(med, 1)
        out["spread_ms"] = round(float(np.median(np.abs(np.array(taus) - med))) * 1.4826, 1) if len(taus) > 1 else None
        out["pendulum_mm"] = round(PM.G_MM_S2 * (med / 1000.0) ** 2, 1)
        out["each_ms"] = [round(t, 1) for t in taus]
    return out


# ---- the table ------------------------------------------------------------------------------------
WHAT = {p.name: (p.what, p.unit) for p in PARAMS}
WHAT["floor_pitch"] = ("the floor's slope, nose down (turned runs)", "deg")
WHAT["floor_roll"] = ("the floor's slope, left side down (turned runs)", "deg")


def format_table(res: FitResult) -> str:
    L = [f"{'parameter':10s} {'unit':10s} {'model':>8s} {'fitted':>8s} {'+-':>6s}  significant  what"]
    rows = list(ROWS) + [n for n in ("floor_pitch", "floor_roll") if n in res.values]
    for n in rows:
        what, unit = WHAT[n]
        mv = res.model.get(n, 0.0)
        sig = ("yes" if res.significant(n) else "no") if n in res.names else "fixed"
        L.append(f"{n:10s} {unit:10s} {mv:8.2f} {res.values[n]:8.2f} {res.sd.get(n, 0.0):6.2f}  {sig:11s}  {what}")
    L.append("")
    L.append(f"tick residuals: {res.n_ticks} ticks, rms {res.rms_tick_deg:.3f} deg (errors last {res.inflation:.0f} "
             "ticks: the +- allow for it)"
             + (f"; the routine's own rigid prediction: rms {res.rms_model_deg:.2f} deg" if res.rms_model_deg else ""))
    for c in res.crossings:
        if c["measured_x"] is None:
            L.append(f"crossing {c['experiment']:13s} none ({c['note']}); model {c['model_x']:+6.1f} mm")
        else:
            L.append(f"crossing {c['experiment']:13s} measured {c['measured_x']:+6.1f} mm  model {c['model_x']:+6.1f}"
                     f"  fitted {c['fitted_x']:+6.1f}")
    tr = res.tip_rate
    if tr.get("events"):
        L.append(f"tip rate: {tr['events']} tips, e-folding time {tr['median_ms']} ms"
                 + (f" +- {tr['spread_ms']}" if tr.get("spread_ms") else "")
                 + f" (a point mass at the fitted height: {tr['model_ms']} ms; {tr['median_ms']} ms is a point "
                   f"mass {tr['pendulum_mm']} mm up)")
    else:
        L.append("tip rate: no tip to time")
    if res.battery_mv:
        L.append(f"battery {res.battery_mv[0]:.0f}-{res.battery_mv[1]:.0f} mV over the run")
    if res.walks:
        L.append(f"walks in the fit (the servo response only): {', '.join(res.walks)}")
    for n in res.notes:
        L.append(f"note: {n}")
    L.append("significant: more than 2 +- from the model's value. Nothing here changes the walks' model.")
    return "\n".join(L)


def to_json(res: FitResult, inputs: list[str]) -> dict:
    rows = list(ROWS) + [n for n in res.values if n.startswith(("floor_", "walk_"))]
    out = {"format": 1, "inputs": inputs, "parameters": {}, "rms_tick_deg": round(res.rms_tick_deg, 4),
           "rms_model_deg": None if res.rms_model_deg is None else round(res.rms_model_deg, 4),
           "inflation": round(res.inflation, 2), "ticks": res.n_ticks, "crossings": res.crossings,
           "tip_rate": res.tip_rate, "walks": res.walks, "notes": res.notes, "iterations": res.iterations,
           "battery_mv": res.battery_mv}
    for n in rows:
        what, unit = WHAT.get(n, (n, "deg"))
        out["parameters"][n] = {"value": round(res.values[n], 4), "sd": round(res.sd.get(n, 0.0), 4),
                                "model": round(res.model.get(n, 0.0), 4), "unit": unit, "what": what,
                                "fitted": n in res.names, "significant": bool(res.significant(n))}
    idx = {n: i for i, n in enumerate(res.names)}
    core = [n for n in ROWS if n in idx]
    sd = np.sqrt(np.maximum(np.diag(res.cov), 1e-30))
    corr = res.cov / np.outer(sd, sd)
    out["correlation"] = {a: {b: round(float(corr[idx[a], idx[b]]), 3) for b in core} for a in core}
    return out


def write_json(res: FitResult, inputs: list[str], path) -> None:
    with open(path, "w") as f:
        json.dump(to_json(res, inputs), f, indent=1)
        f.write("\n")
