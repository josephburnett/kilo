"""The fit: Kilo's measured tilt through the self-measurement routine,
explained by a few numbers about his body (docs/self-measurement.md).

THE MEASUREMENT MODEL. For the feet the routine commanded at each tick:

1. the compliant statics (kilo.measure.statics) give the tilt the body
   rests at, from the centre of mass (com_x, com_y, com_z), the rectangle
   the feet stand on (``length`` front to back, ``track`` left to right),
   the legs' sag and their ``twist``: the LF and RB legs together longer
   than the RF and LB ones (a table with one short leg). The IMU reads
   only angles, so one length has to be taken as right: the legs' (a mm
   of z commanded is a mm), and the rectangle is measured against it.
   ``x_scale`` (how far a foot really moves per mm of x commanded) stays
   1: the splay resolves it to about 10 %, and in the crossings 10 % of
   x scale looks like 10 mm of height, so fitted together it would take
   the height with it. The splay poses are kept out of the fit and CHECK
   it on their own afterwards (``x_scale_check``). The other ways the
   four legs can be off are not fitted, because no tilt tells them from
   something else: all four longer is a height; the front pair longer than
   the back looks exactly like an IMU pitch offset, the left pair longer
   like a roll offset;
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
crossed the edge. The sweep moves 1 mm every TICKS_PER_MM ticks; the tip
(``find_tip``: found by script.TipWatch's rule, the tilt TIP_RISE_DEG above
the lifted hold's and rising; timed half way up its jump) comes ``tip_ms``
after the step that put the
centre of mass over the edge (the servos' delay and the pendulum's growth:
a fitted number, which the sweeps the other way along the same edge pin
down), so the half-way point of that step is the crossing
(``crossing_at``). The model's crossing is where its centre of mass is
over the edge with the body tilted as the IMU read it just before the tip
(statics.edge_distance: how the tripod really leaned, its legs' give and
play included, is measured, not modelled). The crossings locate
the centre of mass: a 1 mm error in one is about 0.5 mm of the centre of
mass across the edge; its height comes from the rolled sweeps against the
flat sweep of the same foot. A sweep that never tips still gives its
statics: the lean of the tripod as the foot comes up and along the sweep
is what shows the twist and the sag.

THE FIT. Least squares (Levenberg-Marquardt, numerical derivatives) of the
tick residuals (measured minus modelled pitch and roll, the tip windows
and the ``check`` poses left out) and the crossing residuals, each over
its own spread. Two kinds of tick:

- STATIC ticks (a hold, from STATIC_SKIP ticks into it; a sweep, which
  carries its lifted hold's resting along): Kilo does not come to rest at
  the same tilt twice (servo play; ``repeatability`` below), so the error
  is the hold's, not the tick's: a hold counts as ONE observation with
  the spread ``sigma_hold``, at least what the returns to the home pose
  show;
- the other ticks (moves, a hold's first ticks): the tick residuals'
  spread from the fit itself, inflated by how many ticks they stay
  correlated.

The uncertainties come from the curvature. The simulation check
(kilo.measure.simulate) shows how honest the +- are. A value that ends at
a limit of its range is "not measured", whatever its +-.

REPEATABILITY, HYSTERESIS, THE HELD-OUT CHECK. Reported beside the fit:
how much the tilt differs between the returns to the home pose; how
differently a stair is held when reached from above and from below; and,
for script 2's ``check`` poses (not used by the fit), the measured tilt
against today's model and against the fitted one: the fit's own residuals
are not a verdict, these are.

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
SIGMA_TICK_DEG = 0.2        # the moving ticks' spread to start with (re-estimated; a walk's its own)
SIGMA_HOLD_DEG = 0.3        # a hold's spread to start with (re-estimated, never under the home returns')
SIGMA_HOLD_MIN = 0.05       # ...and never under this (the IMU's own steps)
SD_SCALE = 1.3              # the curvature's +- times this: what the simulation check needs (kilo.measure.simulate)
                            # for the truth to be within 2 +- in about 95 % of runs (1.0 left it at 80-90 %)
SIGMA_CROSS_MM = 0.6        # a crossing's least spread: where in its 1 mm step it fell (uniform: 0.29), and the
                            # pendulum's time to tip, which depends on it (simulate check: 0.4 left the +- short)
TIP_WINDOW_BEFORE_MM = 2    # tick residuals: leave out the sweep within this of the crossing
TIP_WINDOW_AFTER = 20       # ...and this many ticks after the tip was seen
STATIC_SKIP = 12            # a hold is static from this many ticks in (the dead time and 3 time constants)
SETTLED_TICKS = 6           # the repeatability and the hysteresis are read from a hold's last ticks (no model
                            # of the servos there to take out what is left of the move)
P_STANCE = 205              # spec gait.stance_h_256
LIMIT_FRACTION = 0.01       # a value within this much of its range from a limit is at the limit: not measured
HOLD_PHASES = ("level", "level_end", "settle", "lifted", "lowered", "home_hold", "hold", "up_hold")
STAIR_EXPERIMENTS = ("steps", "stairs_pitch", "stairs_roll")


@dataclass(frozen=True)
class Param:
    name: str
    unit: str
    what: str
    step: float             # the numerical derivative's step
    lo: float = -math.inf
    hi: float = math.inf
    statics: bool = True    # it changes the statics (they are cached otherwise)
    where: str = "not a number of the model"  # where the model keeps it (protocol/spec.toml)
    lo_is_value: bool = False  # its lower limit is a value it can have (rigid legs), not "not measured"
    fine: float = math.inf  # a +- wider than this resolves nothing worth having: "not resolved", not "matches"


PARAMS = (
    Param("imu_pitch", "deg", "IMU pitch offset (+ floor slope, + front/back legs)", 0.01, statics=False),
    Param("imu_roll", "deg", "IMU roll offset (+ floor slope, + left/right legs)", 0.01, statics=False),
    Param("com_x", "mm", "centre of mass, ahead of the shoulders' middle", 0.05, -40, 40,
          where="[model] stand_com_x_mm, *_mass", fine=3.0),
    Param("com_y", "mm", "centre of mass, to the left", 0.05, -20, 20, where="[model] *_mass (y)", fine=3.0),
    Param("com_z", "mm", "centre of mass, above the shoulder axles", 0.2, -40, 40, where="[model] *_mass (z)",
          fine=6.0),
    Param("length", "mm", "the feet's rectangle, front to back (pitch per mm of leg)", 0.2, 90, 170,
          where="LEG_LB_MM", fine=5.0),
    Param("track", "mm", "the feet's rectangle, left to right (roll per mm of leg)", 0.1, 45, 95,
          where="LEG_W_MM", fine=3.0),
    Param("x_scale", "mm/mm", "a foot's travel per mm of x commanded (against the legs' z)", 0.01, 0.5, 1.5,
          where="LEG_L1_MM, LEG_L2_MM (the IK)", fine=0.1),
    Param("sag", "deg/kg cm", "leg servos' give under load", 0.05, ST.SAG_MIN, 20, lo_is_value=True),
    Param("twist", "mm", "LF+RB legs longer than RF+LB (each pair by half)", 0.02, -6, 6),
    Param("lag_ms", "ms", "servo dead time (with the IMU's own delay)", 0.5, 0, 300, statics=False,
          where="[model] servo_lag_ms", fine=15.0),
    Param("tau_ms", "ms", "servo time constant", 0.5, 2, 400, statics=False, where="[model] servo_tau_ms",
          fine=20.0),
    Param("tip_ms", "ms", "a tip's delay: the step over the edge to half way up the tip", 1.0, 0, 1000,
          statics=False),
)
NAMES = tuple(p.name for p in PARAMS)
ROWS = NAMES
BY_NAME = {p.name: p for p in PARAMS}
TIP_DEG = math.hypot(37, 70) / 16.0  # the model's tip onto a lifted foot (LIFT_MM down): 4.95 deg


def model_values(model: BodyModel | None = None) -> dict[str, float]:
    """What the walks' model says today for each parameter (tip_ms: its
    servo alone, a step's response half way; no pendulum)."""
    m = model or BodyModel()
    cx, cy, cz = m.com([0.0] * 4, [float(SC.HOME_Z)] * 4)
    tip = m.servo_lag_ms + m.servo_tau_ms * math.log(2.0)
    return {"imu_pitch": 0.0, "imu_roll": 0.0, "com_x": cx, "com_y": cy, "com_z": cz, "length": m.length_mm,
            "track": m.track_mm, "x_scale": 1.0, "sag": 0.0, "twist": 0.0,
            "lag_ms": m.servo_lag_ms, "tau_ms": m.servo_tau_ms, "tip_ms": tip}


def body_of(v: dict[str, float], model: BodyModel) -> ST.Body:
    mv = model_values(model)
    t = 0.5 * v["twist"]
    return ST.Body(com_dx=v["com_x"] - mv["com_x"], com_dy=v["com_y"] - mv["com_y"], com_dz=v["com_z"] - mv["com_z"],
                   sag=v["sag"], leg_dz=(t, -t, t, -t), model=model, length=v.get("length", mv["length"]),
                   track=v.get("track", mv["track"]), x_scale=v.get("x_scale", 1.0))


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
    name: str = ""
    tip_a16: tuple = (37, 70)
    lean_deg: float | None = None   # the tripod's lean as the foot came up: toward the lifted foot, deg
    lean_vec: tuple | None = None   # ...as (pitch, roll)
    creep_deg: float | None = None  # the most the tilt rose above the lifted hold's before the tip (or at all)
    tip_deg: float | None = None    # how far he tipped (above the lifted hold's tilt), once tipped
    att: tuple | None = None        # the measured (pitch, roll) just before the tip (the IMU's, offset and all)

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
    exp: np.ndarray             # (N,) the script's experiments index
    phase: list                 # (N,) the step's phase name
    meas: np.ndarray            # (N, 2) measured pitch, roll (NaN: no TELEMETRY)
    pred: np.ndarray            # (N, 2) the routine's prediction (FEET pitch, roll; NaN if none)
    batt_mv: np.ndarray         # (N,) NaN if none
    rate: np.ndarray | None = None  # (N, 2) TELEMETRY rate_pitch, rate_roll
    heading: int = 1            # -1: turned round (the floor's slope the other way)
    meta: dict = field(default_factory=dict)
    sweeps: list = field(default_factory=list)
    script: SC.ScriptDef = SC.SCRIPT  # the script it played (its experiments name ``exp``)
    step: np.ndarray | None = None    # (N,) the script's step index, if known

    def __len__(self) -> int:
        return len(self.x)

    def exp_name(self, e: int) -> str:
        ex = self.script.experiments
        return ex[e].name if 0 <= e < len(ex) else "?"


@dataclass
class Hold:
    """A hold of a run: its ticks, and the static ones (from STATIC_SKIP in;
    a short hold's last three)."""
    exp: int
    phase: str
    ticks: np.ndarray
    static: np.ndarray


def holds_of(run: Run) -> list[Hold]:
    """Every hold of the run, in order (a new one wherever the step, or
    failing that the experiment or the phase, changes)."""
    n = len(run)
    out = []
    i = 0
    while i < n:
        j = i + 1
        while j < n and run.exp[j] == run.exp[i] and run.phase[j] == run.phase[i] and \
                (run.step is None or run.step[j] == run.step[i]):
            j += 1
        if run.phase[i] in HOLD_PHASES or run.phase[i].endswith("_hold"):
            ks = np.arange(i, j)
            skip = min(STATIC_SKIP, max(len(ks) - 3, 0))
            out.append(Hold(int(run.exp[i]), run.phase[i], ks, ks[skip:]))
        i = j
    return out


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
    for e, ex in enumerate(run.script.experiments):
        if ex.lifted < 0:
            continue
        idx = np.array([i for i in range(len(run)) if run.exp[i] == e and run.phase[i] == "sweep"], int)
        if len(idx) == 0:
            continue
        lt = np.array([i for i in range(len(run)) if run.exp[i] == e and run.phase[i] == "lifted"], int)
        af = np.array([i for i in range(len(run)) if run.exp[i] == e and run.phase[i] in ("lower", "lowered")], int)
        d = 1 if run.x[idx[-1], 0] > run.x[idx[0], 0] else -1
        x0 = float(run.x[idx[0], 0]) - d
        out.append(Sweep(e, ex.lifted, d, idx, tuple(run.z[idx[0]]), lt, af, x0, name=ex.name, tip_a16=ex.tip_a16))
    return out


def tip_vector(sw: Sweep) -> np.ndarray:
    return np.array(sw.tip_a16, float) / 16.0


def find_tip(run: Run, sw: Sweep) -> None:
    """When the sweep's tip was seen (sw.tip_tick, sw.note), by script 2's
    rule whatever script the run played (script.TipWatch): the tilt toward
    the lifted foot, above the mean of the lifted hold's last
    TIP_BASE_TICKS, passes TIP_RISE_DEG (to a fraction of a tick,
    linearly) having risen TIP_JUMP_DEG within TIP_JUMP_TICKS. Also the
    tripod's lean as the foot came up (sw.lean_deg, from the settle hold's
    tilt), and how far he tipped (sw.tip_deg). Already TIP_UP_FRACTION of
    the model's tip at the end of the lifted hold: he tipped as the foot
    came up, and the sweep gives no crossing."""
    u = tip_vector(sw)
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
    tail = sw.lifted_ticks[-SC.TIP_BASE_TICKS:]
    lifted = np.nanmean(run.meas[tail], 0)
    if not np.all(np.isfinite(lifted)):
        sw.note = "no TELEMETRY in the lifted hold"
        return
    sw.lean_vec = (float(lifted[0] - base[0]), float(lifted[1] - base[1]))
    sw.lean_deg = float((lifted - base) @ u)
    if sw.lean_deg >= SC.TIP_UP_FRACTION * T:
        sw.note = "tipped as the foot came up: the crossing is before the sweep's start"
        return
    ks = np.concatenate([sw.ticks, sw.after])
    rel = (run.meas[ks] - lifted) @ u
    n_sweep = len(sw.ticks)
    hit = None
    for j in range(len(ks)):
        if not np.isfinite(rel[j]) or rel[j] < SC.TIP_RISE_DEG:
            continue
        old = 0.0 if j < SC.TIP_JUMP_TICKS else rel[j - SC.TIP_JUMP_TICKS]
        if np.isfinite(old) and rel[j] - old >= SC.TIP_JUMP_DEG:
            hit = j
            break
    ok = np.isfinite(rel[:n_sweep])
    if hit is None:
        sw.creep_deg = float(np.max(rel[:n_sweep][ok])) if ok.any() else None
        if sw.creep_deg is not None and sw.creep_deg >= SC.TIP_RISE_DEG:
            sw.note = "a slow lean, not a tip (the legs give, or the lifted foot did not clear): no crossing"
        else:
            sw.note = "no tip in the sweep: the crossing is past its end"
        return
    before = rel[:hit][np.isfinite(rel[:hit])]
    sw.creep_deg = float(np.max(before[:-SC.TIP_JUMP_TICKS])) if len(before) > SC.TIP_JUMP_TICKS else 0.0
    aft = rel[hit + 8:n_sweep]
    aft = aft[np.isfinite(aft)]
    sw.tip_deg = float(np.median(aft)) if len(aft) >= 3 else None
    # how he was leaning just before it (the IMU's reading): where his mass was over the feet then
    w = run.meas[ks[max(hit - 20, 0):max(hit - 7, 0)]]
    w = w[np.isfinite(w).all(1)]
    sw.att = (float(np.median(w[:, 0])), float(np.median(w[:, 1]))) if len(w) >= 3 else None
    # the tip's time: half way up the jump, from the lean just before it to where he came to rest (a lean
    # that crept up before the tip would otherwise make the same threshold come sooner)
    pre = rel[max(hit - 14, 0):max(hit - 6, 0)]
    pre = pre[np.isfinite(pre)]
    lo = float(np.median(pre)) if len(pre) else 0.0
    top = rel[hit:hit + 12]
    top = top[np.isfinite(top)]
    hi = sw.tip_deg if sw.tip_deg is not None and sw.tip_deg > lo + SC.TIP_JUMP_DEG else float(np.max(top))
    half = lo + 0.5 * (hi - lo)
    k = float(ks[hit])
    for j in range(max(hit - SC.TIP_JUMP_TICKS, 1), min(hit + 12, len(ks))):
        if np.isfinite(rel[j]) and np.isfinite(rel[j - 1]) and rel[j - 1] < half <= rel[j]:
            k = float(ks[j]) - (rel[j] - half) / (rel[j] - rel[j - 1])
            break
    sw.tip_tick = k


def model_crossing(model: BodyModel, body: ST.Body, sw: Sweep, x_lo: float = -45.0, x_hi: float = 45.0,
                   att: tuple | None = None) -> float:
    """The feet's x (all four) at which the model's centre of mass is over
    the sweep's edge, its legs at the sweep's z; the body tilted as the
    statics say, or as ``att`` (pitch, roll, deg: what the IMU read just
    before the tip, less its offset)."""
    def f(x):
        return ST.edge_distance(model, [x] * 4, sw.z, body, sw.lifted, att)

    a, b = x_lo, x_hi
    fa = f(a)
    fb = f(b)
    if (fa > 0) == (fb > 0):
        return a if abs(fa) < abs(fb) else b
    for _ in range(26):  # to 1e-6 mm
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
    decide), the ticks the tick residuals use, and which of them are a
    hold's static ticks (``hold_of``: the hold's index, -1 a moving tick)."""

    def __init__(self, run: Run, model: BodyModel, tip_ms: float, use_splay: bool = True):
        self.run = run
        self.model = model
        keys = np.hstack([run.x, run.z])
        u, inv = np.unique(keys, axis=0, return_inverse=True)
        self.poses = ST.Poses(model, u[:, :4], u[:, 4:])
        self.inv = inv.ravel()
        n = len(run)
        self.exclude = np.full(n, -1)    # a candidate the tick may not be on, -1 none
        use = np.isfinite(run.meas).all(1)
        self.check = np.array([run.exp_name(int(e)) == "check" for e in run.exp], bool)
        use &= ~self.check
        self.splay = np.array([run.exp_name(int(e)) == "splay" for e in run.exp], bool)
        if not use_splay:  # the x scale is not fitted: the poses that depend on it are kept for its check
            use &= ~self.splay
        for sw in run.sweeps:
            off_tripod = 1 + sw.lifted                       # candidate: the lifted foot in the air
            off_tipped = 1 + _OPPOSITE[sw.lifted]           # ...the far foot of the diagonal in the air
            lift = np.array([i for i in range(n) if run.exp[i] == sw.exp and run.phase[i] == "lift"], int)
            if sw.tip_tick is None:
                if "came up" in sw.note or sw.lean_deg is None:
                    # tipped as the foot came up (or no data): the lift and the sweep left out
                    use[sw.ticks] = False
                    use[sw.lifted_ticks] = False
                    use[sw.after] = False
                    use[lift] = False
                    continue
                # on the tripod as the foot came up and through its hold: the lean is data
                self.exclude[lift] = off_tipped
                self.exclude[sw.lifted_ticks] = off_tipped
                if "past its end" in sw.note:
                    self.exclude[sw.ticks] = off_tipped
                # else a slow lean: the statics decide tick by tick what he stands on (legs that give put the
                # lifted foot down little by little), and the lean along the sweep is fitted as it is
                continue
            kt = sw.tip_tick
            for i in sw.ticks:
                self.exclude[i] = off_tipped if i < kt else off_tripod
            self.exclude[lift] = off_tipped
            self.exclude[sw.lifted_ticks] = off_tipped
            cx = sw.crossing_at(tip_ms)
            near = np.abs(run.x[sw.ticks, 0] - cx) <= TIP_WINDOW_BEFORE_MM + 0.5
            use[sw.ticks[near & (sw.ticks < kt)]] = False
            w = np.arange(int(math.floor(kt)) - 3 * SC.TICKS_PER_MM, int(math.ceil(kt)) + TIP_WINDOW_AFTER)
            use[w[(w >= 0) & (w < n)]] = False
        self.use = use
        self.holds = holds_of(run)
        # a sweep too is one resting of the legs (the lifted hold's, carried along): its ticks' common error
        # is one observation, like a hold's, however many ticks it lasts
        for sw in run.sweeps:
            self.holds.append(Hold(sw.exp, "sweep", sw.ticks, sw.ticks))
        self.hold_of = np.full(n, -1)
        for h, hd in enumerate(self.holds):
            self.hold_of[hd.static] = h
        self._cache: dict = {}

    def statics(self, body: ST.Body) -> np.ndarray:
        key = (body.com_dx, body.com_dy, body.com_dz, body.sag, tuple(body.leg_dz), body.length, body.track,
               body.x_scale)
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


# ---- what the data says by itself: repeatability, hysteresis ------------------------------------------
def _is_home(run: Run, k: int) -> bool:
    return bool(np.all(run.x[k] == 0) and np.all(run.z[k] == SC.HOME_Z))


def repeatability(runs: list[Run]) -> dict:
    """How differently Kilo stands in the home pose each time he comes back
    to it: over every hold at home (their static ticks' mean), the spread
    (sd) and the range of pitch and of roll, per run, pooled."""
    dev_p, dev_r, rng_p, rng_r, n = [], [], 0.0, 0.0, 0
    for run in runs:
        means = []
        for hd in holds_of(run):
            if len(hd.static) == 0 or not _is_home(run, hd.ticks[0]):
                continue
            m = run.meas[hd.static[-SETTLED_TICKS:]]
            if np.isfinite(m).all(1).sum() >= 2:
                means.append(np.nanmean(m, 0))
        if len(means) < 2:
            continue
        a = np.array(means)
        d = a - a.mean(0)
        dev_p += d[:, 0].tolist()
        dev_r += d[:, 1].tolist()
        rng_p = max(rng_p, float(np.ptp(a[:, 0])))
        rng_r = max(rng_r, float(np.ptp(a[:, 1])))
        n += len(means)
    if n < 3:
        return {"holds": n}
    k = max(n - len(runs), 1)
    return {"holds": n, "sd_pitch": float(math.sqrt(sum(v * v for v in dev_p) / k)),
            "sd_roll": float(math.sqrt(sum(v * v for v in dev_r) / k)), "range_pitch": rng_p, "range_roll": rng_r}


def hysteresis(runs: list[Run]) -> dict:
    """How differently a stair is held when it is reached from above and
    from below: over the stairs (script 1: the steps) the mean of (tilt
    reached from above - tilt reached from below) at the same pose, for
    the pitch stairs' pitch and the roll stairs' roll, deg."""
    out: dict[str, Any] = {}
    for axis, ch in (("pitch", 0), ("roll", 1)):
        diffs = []
        for run in runs:
            seen: dict[tuple, dict] = {}
            prev, at = None, -1
            for hd in holds_of(run):
                if run.exp_name(hd.exp) not in STAIR_EXPERIMENTS:
                    continue
                if hd.exp != at:  # the stairs begin from the home pose
                    prev, at = 0.0, hd.exp
                z = run.z[hd.ticks[0]]
                # the level along this axis, in the angle's sign: nose down +, left side down +
                pitch_lv = (z[2] + z[3] - z[0] - z[1]) / 4.0
                roll_lv = (z[1] + z[2] - z[0] - z[3]) / 4.0
                lv, other = (pitch_lv, roll_lv) if ch == 0 else (roll_lv, pitch_lv)
                if other != 0:  # the other axis's stairs
                    prev = None
                    continue
                m = run.meas[hd.static[-SETTLED_TICKS:]] if len(hd.static) else np.zeros((0, 2))
                if prev is not None and lv != prev and np.isfinite(m).all(1).sum() >= 2:
                    seen.setdefault(tuple(z), {})["above" if lv < prev else "below"] = float(np.nanmean(m[:, ch]))
                prev = lv
            diffs += [d["above"] - d["below"] for d in seen.values() if "above" in d and "below" in d]
        if diffs:
            out[axis] = float(np.mean(diffs))
            out[axis + "_max"] = float(np.max(np.abs(diffs)))
            out[axis + "_poses"] = len(diffs)
    return out


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
    at_limit: dict = field(default_factory=dict)  # name -> True: ended at a limit of its range
    unmeasured: dict = field(default_factory=dict)  # name -> why the run could not say (left at the model's)
    sigma_hold_deg: float = 0.0
    n_holds: int = 0
    repeatability: dict = field(default_factory=dict)
    hysteresis: dict = field(default_factory=dict)
    check: dict = field(default_factory=dict)
    x_scale_check: dict = field(default_factory=dict)  # the splay alone: value, sd (the fit keeps x_scale 1)

    def significant(self, name: str) -> bool:
        if self.at_limit.get(name):
            return False
        return name in self.names and abs(self.values[name] - self.model.get(name, 0.0)) > 2.0 * self.sd[name]

    def verdict(self, name: str) -> str:
        """One of: "fixed", "not measured" (at a limit), "matches" (within
        2 +- of the model's), "not resolved" (within 2 +-, but the +- is
        wider than Param.fine), "differs: X -> Y", or for a number the
        model does not have, "new: Y"."""
        if name in self.unmeasured:
            return f"not measured ({self.unmeasured[name]})"
        if name == "x_scale" and name not in self.names and self.x_scale_check:
            v, sd = self.x_scale_check["value"], self.x_scale_check["sd"]
            how = "differs" if abs(v - 1.0) > 2 * sd else "not resolved" if sd > BY_NAME[name].fine else "matches"
            return f"{how}: the fit keeps 1.00; the splay alone says {v:.2f} +- {sd:.2f}"
        if name not in self.names:
            return "fixed (not fitted)"
        if self.at_limit.get(name):
            return "not measured (the fit ran into its limit)"
        p = BY_NAME.get(name)
        v, sd = self.values[name], self.sd[name]
        if p is None or p.where.startswith("not a number"):
            return f"new: {v:.2f} +- {sd:.2f}"
        mv = self.model.get(name, 0.0)
        if not self.significant(name):
            if sd > p.fine:
                return f"not resolved ({mv:.2f}; measured {v:.2f} +- {sd:.2f}: too wide to say)"
            return f"matches ({mv:.2f}; measured {v:.2f} +- {sd:.2f})"
        return f"differs: {mv:.2f} -> {v:.2f} +- {sd:.2f}"


class Fitter:
    def __init__(self, runs: list[Run], walks: list[Walk] | None = None, model: BodyModel | None = None,
                 fixed: dict | None = None, free_x_scale: bool = False):
        self.model = model or BodyModel()
        self.runs = runs
        self.walks = walks or []
        self.fixed = dict(fixed or {})
        mv0 = model_values(self.model)
        if not runs:  # walks alone: they time the servos and say nothing else (the module docstring)
            self.fixed.update({p.name: mv0[p.name] for p in PARAMS if p.name not in ("lag_ms", "tau_ms")})
        elif not free_x_scale or not any(run.exp_name(int(e)) == "splay" for run in runs for e in np.unique(run.exp)):
            # the splay resolves the x scale to about 10 %, and in the crossings 10 % of x scale looks like 10 mm
            # of height: fitted together it would take the height with it. It stays 1 (the legs' geometry is
            # measured) and the splay CHECKS it on its own afterwards (``x_scale_check``).
            self.fixed.setdefault("x_scale", 1.0)
        self.free_x_scale = "x_scale" not in self.fixed
        self.mv = mv0
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
        self.sigma_h = SIGMA_HOLD_DEG
        self.sigma_w = SIGMA_TICK_DEG
        self.sigma_c = SIGMA_CROSS_MM
        self.n_ticks = 0
        self._cross: dict = {}
        self.unmeasured: dict[str, str] = {}  # name -> why the run cannot say (left at the model's value)
        self.rep = repeatability(runs)
        self.home_sd = max(self.rep.get("sd_pitch", 0.0), self.rep.get("sd_roll", 0.0))

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
        self.rms = [RunModel(r, self.model, v["tip_ms"], self.free_x_scale) for r in self.runs]
        self.n_ticks = int(sum(rm.use.sum() for rm in self.rms))

    def height_pairs(self) -> int:
        """How many rolled sweeps have a crossing AND a crossing from the
        flat sweep of the same foot (script 1: any flat sweep of the same
        diagonal): what the centre of mass's height is read from."""
        n = 0
        for r in self.runs:
            got = {sw.name: sw for sw in r.sweeps if sw.tip_tick is not None}
            for sw in got.values():
                ex = r.script.experiments[sw.exp]
                if ex.roll_mm and (ex.of in got or any(
                        not r.script.experiments[o.exp].roll_mm and ST.EDGE[o.lifted] == ST.EDGE[sw.lifted]
                        for o in got.values())):
                    n += 1
        return n

    def _offset(self, v: dict, r: Run) -> np.ndarray:
        off = np.array([v["imu_pitch"], v["imu_roll"]])
        if self.floor:
            off = off + r.heading * np.array([v["floor_pitch"], v["floor_roll"]])
        return off

    def tick_errors(self, th: np.ndarray) -> list:
        """Per run: (modelled - measured) for every tick, (N, 2)."""
        v = self.values(th)
        body = body_of(v, self.model)
        out = []
        for rm in self.rms:
            y = servo(rm.statics(body), v["lag_ms"], v["tau_ms"])
            out.append(y + self._offset(v, rm.run) - rm.run.meas)
        return out

    def residuals(self, th: np.ndarray, parts: bool = False):
        """The weighted residuals (or with ``parts`` the plain ones: the
        moving ticks', every hold's mean (H, 2), the crossings', the
        walks')."""
        v = self.values(th)
        body = body_of(v, self.model)
        res_t, res_h, res_hw, res_c, res_w = [], [], [], [], []
        for rm, e in zip(self.rms, self.tick_errors(th)):
            r = rm.run
            moving = rm.use & (rm.hold_of < 0)
            res_t.append(e[moving].ravel())
            for h, hd in enumerate(rm.holds):
                ks = hd.static[rm.use[hd.static]]
                if len(ks) == 0:
                    continue
                res_h.append(e[ks].mean(0))
                # every static tick over sigma_hold sqrt(m): the hold weighs as one observation of its mean,
                # and its ticks' scatter about it (the IMU's noise) is not counted against the model
                res_hw.append(e[ks].ravel() / math.sqrt(len(ks)))
            for sw in r.sweeps:
                cx = sw.crossing_at(v["tip_ms"])
                if cx is None:
                    continue
                res_c.append(self._crossing(body, sw, self._att(v, r, sw)) - cx)
        for w in self.walks:
            y = servo(w.u, v["lag_ms"], v["tau_ms"])
            off = np.array([v[f"walk_pitch:{w.name}"], v[f"walk_roll:{w.name}"]])
            res_w.append((y + off - w.meas)[w.used].ravel())
        rt = np.concatenate(res_t) if res_t else np.zeros(0)
        rh = np.array(res_h, float).reshape(-1, 2)
        rc = np.array(res_c, float)
        rw = np.concatenate(res_w) if res_w else np.zeros(0)
        if parts:
            return rt, rh, rc, rw
        rhw = np.concatenate(res_hw) if res_hw else np.zeros(0)
        return np.concatenate([rt / self.sigma_t, rhw / self.sigma_h, rc / self.sigma_c, rw / self.sigma_w])

    def _att(self, v: dict, r: Run, sw: Sweep) -> tuple | None:
        """The body's tilt just before the sweep's tip: the IMU's reading
        less its offset (None: not known, the statics' is used)."""
        if sw.att is None:
            return None
        off = self._offset(v, r)
        return (sw.att[0] - float(off[0]), sw.att[1] - float(off[1]))

    def _crossing(self, body: ST.Body, sw: Sweep, att: tuple | None = None) -> float:
        """model_crossing, remembered (the parameters that change neither
        the statics nor the tilt leave it as it is)."""
        key = (body.com_dx, body.com_dy, body.com_dz, body.sag, tuple(body.leg_dz), body.length, body.track,
               body.x_scale, sw.lifted, sw.z, att)
        c = self._cross.get(key)
        if c is None:
            if len(self._cross) > 4096:
                self._cross.clear()
            c = self._cross[key] = model_crossing(self.model, body, sw, att=att)
        return c

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

    def _spreads(self, th: np.ndarray) -> None:
        """The spreads from the residuals as they are (the moving ticks',
        the holds' means never under the home returns' spread, the walks',
        the crossings')."""
        rt, rh, rc, rw = self.residuals(th, parts=True)
        if len(rt):
            self.sigma_t = max(float(np.sqrt(np.mean(rt ** 2))), 0.02)
        if len(rh):
            self.sigma_h = max(float(np.sqrt(np.mean(rh ** 2))), self.home_sd, SIGMA_HOLD_MIN)
        if len(rw):
            self.sigma_w = max(float(np.sqrt(np.mean(rw ** 2))), 0.02)
        if len(rc):
            self.sigma_c = max(float(np.sqrt(np.mean(rc ** 2))), SIGMA_CROSS_MM)

    def run(self, max_iter: int = 40) -> FitResult:
        th = self.start()
        self.prepare(th)
        if self.runs and "com_z" in self.names and self.height_pairs() == 0:
            # nothing tells the height from the rest: left at the model's, and said so
            self.fixed["com_z"] = self.mv["com_z"]
            self.unmeasured["com_z"] = "no rolled sweep tipped together with the flat sweep of the same foot"
            self.names.remove("com_z")
            th = self.start()
        th, it1 = self._lm(th, 6)
        # again with the tip windows where the fitted tip delay puts them, and the spreads re-estimated
        self.prepare(th)
        self._spreads(th)
        th, it2 = self._lm(th, max_iter)
        return self.result(th, it1 + it2)

    def limits(self, th: np.ndarray) -> dict:
        out = {}
        for j, n in enumerate(self.names):
            lo, hi = self.bounds[n]
            if not (math.isfinite(lo) and math.isfinite(hi)):
                continue
            tol = LIMIT_FRACTION * (hi - lo)
            p = BY_NAME.get(n)
            at_lo = th[j] - lo <= tol and not (p is not None and p.lo_is_value)
            if at_lo or hi - th[j] <= tol:
                out[n] = True
        return out

    def result(self, th: np.ndarray, iterations: int) -> FitResult:
        rt, rh, rc, rw = self.residuals(th, parts=True)
        s_t = float(np.sqrt(np.mean(rt ** 2))) if len(rt) else 0.1
        s_w = float(np.sqrt(np.mean(rw ** 2))) if len(rw) else 0.1
        nh = len(rh)
        # the holds' spread, allowing for the numbers they mostly fit (the offsets, the rectangle, the twist, the sag)
        dof = max(2 * nh - min(len(self.names), 8), 1)
        s_h = max(float(np.sqrt(np.sum(rh ** 2) / dof)) if nh else 0.0, self.home_sd, SIGMA_HOLD_MIN)
        infl = self._inflation(th)
        # the crossings' spread from their residuals, allowing for the four numbers they mostly fit (com_x, com_y,
        # com_z, tip_ms)
        s_c = max(SIGMA_CROSS_MM, float(np.sqrt(np.mean(rc ** 2)) * math.sqrt(len(rc) / max(len(rc) - 4, 1))))\
            if len(rc) else SIGMA_CROSS_MM
        self.sigma_t, self.sigma_c, self.sigma_w = s_t * math.sqrt(infl), s_c, s_w * math.sqrt(infl)
        self.sigma_h = s_h
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
        nominal = body_of(self.mv, self.model)
        for r_ in self.runs:
            for sw in r_.sweeps:
                cx = sw.crossing_at(v["tip_ms"])
                d = {"run": r_.name, "experiment": sw.name,
                     "measured_x": None if cx is None else round(cx, 2), "note": sw.note,
                     "model_x": round(model_crossing(self.model, nominal, sw), 2),
                     "lean_deg": None if sw.lean_deg is None else round(sw.lean_deg, 2),
                     "lean_pitch_roll": None if sw.lean_vec is None else [round(c, 2) for c in sw.lean_vec],
                     "creep_deg": None if sw.creep_deg is None else round(sw.creep_deg, 2),
                     "tip_deg": None if sw.tip_deg is None else round(sw.tip_deg, 2),
                     "model_tip_deg": round(float(np.hypot(*tip_vector(sw))), 2)}
                if cx is not None:
                    d["fitted_x"] = round(self._crossing(body, sw, self._att(v, r_, sw)), 2)
                crossings.append(d)
        notes = list(self.notes)
        if self.runs and "com_z" in self.names:
            pairs = self.height_pairs()
            if pairs < 2:
                notes.append(f"the height (com_z) rests on {pairs} rolled sweep against its flat one: nothing "
                             "checks it; take it as a hint")
        small = [c["experiment"] for c in crossings if c["measured_x"] is not None and c["tip_deg"] is not None
                 and c["tip_deg"] < 0.6 * c["model_tip_deg"]]
        if small:
            notes.append("tipped less than 60 % of the lifted foot's height (the lifted foot may not have cleared "
                         "the floor, or the legs gave): " + ", ".join(small) + "; their crossings are less sure "
                         "than the +- say")
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
        allt = np.concatenate([e[rm.use].ravel() for rm, e in zip(self.rms, self.tick_errors(th))]) \
            if self.rms else np.zeros(0)
        rms_all = float(np.sqrt(np.mean(allt ** 2))) if len(allt) else 0.0
        return FitResult(values=v, sd=sd, model=dict(self.mv), names=list(self.names), cov=cov, rms_tick_deg=rms_all,
                         rms_model_deg=pred_rms, inflation=infl, n_ticks=len(allt) // 2, crossings=crossings,
                         tip_rate=tip_rates(self.runs, v), walks=[w.name for w in self.walks], notes=notes,
                         iterations=iterations, battery_mv=(min(batt), max(batt)) if batt else None,
                         at_limit=self.limits(th), unmeasured=dict(self.unmeasured), sigma_hold_deg=s_h, n_holds=nh, repeatability=dict(self.rep),
                         hysteresis=hysteresis(self.runs), check=self.held_out(th),
                         x_scale_check=self.x_scale_check(th, s_h))

    def x_scale_check(self, th: np.ndarray, sigma_hold: float) -> dict:
        """The splay's holds on their own, the fit's other numbers as they
        are: the x scale that best explains their pitch (one number, least
        squares on the holds' means), and its +- from the holds' spread."""
        if self.free_x_scale:
            return {}
        v = self.values(th)
        num = den = 0.0
        n = 0
        h = 0.05
        for rm in self.rms:
            if not rm.splay.any():
                continue
            off = self._offset(v, rm.run)
            a = rm.statics(body_of(v, self.model)) + off
            b = rm.statics(body_of(dict(v, x_scale=v["x_scale"] + h), self.model)) + off
            for hd in rm.holds:
                ks = hd.static
                if rm.run.exp_name(hd.exp) != "splay" or len(ks) == 0 or _is_home(rm.run, ks[-1]):
                    continue
                m = rm.run.meas[ks]
                ok = np.isfinite(m).all(1)
                if ok.sum() < 2:
                    continue
                e = (m[ok] - a[ks][ok]).mean(0)
                d = ((b[ks][ok] - a[ks][ok]) / h).mean(0)
                num += float(d @ e)
                den += float(d @ d)
                n += 1
        if n < 3 or den <= 0:
            return {}
        return {"value": v["x_scale"] + num / den, "sd": SD_SCALE * sigma_hold / math.sqrt(den), "holds": n}

    def held_out(self, th: np.ndarray) -> dict:
        """Script 2's ``check`` poses, which the fit did not use: each
        hold's measured tilt against today's model and against the fitted
        one (both with the fitted IMU offset: the model has none), and the
        rms of each over the poses."""
        v = self.values(th)
        body = body_of(v, self.model)
        nominal = body_of(self.mv, self.model)
        rows = []
        for rm in self.rms:
            if not rm.check.any():
                continue
            off = self._offset(v, rm.run)
            fitted = rm.statics(body) + off
            today = rm.statics(nominal) + off
            for hd in rm.holds:
                if rm.run.exp_name(hd.exp) != "check" or len(hd.static) == 0:
                    continue
                m = rm.run.meas[hd.static]
                if np.isfinite(m).all(1).sum() < 2:
                    continue
                k = hd.static[-1]
                if _is_home(rm.run, k):
                    continue
                rows.append({"run": rm.run.name, "x": [int(c) for c in rm.run.x[k]], "z": [int(c) for c in rm.run.z[k]],
                             "measured": [round(float(c), 2) for c in np.nanmean(m, 0)],
                             "model": [round(float(c), 2) for c in today[k]],
                             "fitted": [round(float(c), 2) for c in fitted[k]]})
        if not rows:
            return {}
        a = np.array([r["measured"] for r in rows])
        return {"poses": rows,
                "rms_model_deg": round(float(np.sqrt(np.mean((np.array([r["model"] for r in rows]) - a) ** 2))), 3),
                "rms_fitted_deg": round(float(np.sqrt(np.mean((np.array([r["fitted"] for r in rows]) - a) ** 2))), 3)}

    def _inflation(self, th: np.ndarray) -> float:
        """How many ticks a moving tick's residual stays correlated for
        (1 + 2 x the sum of its autocorrelation), the median over the runs'
        channels."""
        v = self.values(th)
        taus = []
        series = []
        for rm, e in zip(self.rms, self.tick_errors(th)):
            series.append((e, rm.use & (rm.hold_of < 0)))
        for w in self.walks:
            y = servo(w.u, v["lag_ms"], v["tau_ms"])
            series.append((y + np.array([v[f"walk_pitch:{w.name}"], v[f"walk_roll:{w.name}"]]) - w.meas, w.used))
        for e, use in series:
            for ch in range(2):
                x = np.where(use, e[:, ch], np.nan)
                if np.isfinite(x).sum() < 20:
                    continue
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
    lifted foot (above the lifted hold's) between 10 % and 60 % of the
    model's tip, fitted with an exponential e^(t / tau) (an inverted
    pendulum's), tau in ms. A point mass on a pendulum of length l grows so
    with tau = sqrt(l / g): the centre of mass's height over the edge, if
    Kilo were one."""
    taus = []
    for r in runs:
        for sw in r.sweeps:
            if sw.tip_tick is None or len(sw.lifted_ticks) == 0:
                continue
            u = tip_vector(sw)
            T = float(np.hypot(*u))
            u = u / T
            base = np.nanmean(r.meas[sw.lifted_ticks[-SC.TIP_BASE_TICKS:]], 0)
            ks = np.arange(max(int(sw.tip_tick) - 15, 0), min(int(sw.tip_tick) + 10, len(r)))
            pr = (r.meas[ks] - base) @ u / T
            sel = (pr > 0.1) & (pr < 0.6) & np.isfinite(pr)
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
    out["model_ms"] = round(1000.0 * math.sqrt(max(h, 1.0) / PM.G_MM_S2), 1)
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
        sig = "not measured" if n in res.unmeasured else "fixed" if n not in res.names else \
            "at limit" if res.at_limit.get(n) else "yes" if res.significant(n) else "no"
        L.append(f"{n:10s} {unit:10s} {mv:8.2f} {res.values[n]:8.2f} {res.sd.get(n, 0.0):6.2f}  {sig:11s}  {what}")
    L.append("")
    L.append("against the model (protocol/spec.toml); the fit changes none of it:")
    for n in ROWS:
        L.append(f"  {n:10s} {BY_NAME[n].where:32s} {res.verdict(n)}")
    L.append("")
    L.append(f"tick residuals: {res.n_ticks} ticks, rms {res.rms_tick_deg:.3f} deg (moving ticks' errors last "
             f"{res.inflation:.0f} ticks; {res.n_holds} holds, each one observation, spread "
             f"{res.sigma_hold_deg:.2f} deg: the +- allow for both)"
             + (f"; the routine's own rigid prediction: rms {res.rms_model_deg:.2f} deg" if res.rms_model_deg else ""))
    rp = res.repeatability
    if rp.get("holds", 0) >= 3:
        L.append(f"repeatability: {rp['holds']} returns to the home pose differ by sd {rp['sd_pitch']:.2f} deg in "
                 f"pitch (range {rp['range_pitch']:.2f}) and {rp['sd_roll']:.2f} in roll (range "
                 f"{rp['range_roll']:.2f}): no pose is known better than that")
    else:
        L.append("repeatability: too few returns to the home pose to say")
    hy = res.hysteresis
    if hy:
        L.append("hysteresis (the same stair reached from above minus from below): " + "; ".join(
            f"{a} {hy[a]:+.2f} deg (at most {hy[a + '_max']:.2f}, {hy[a + '_poses']} poses)"
            for a in ("pitch", "roll") if a in hy))
    for c in res.crossings:
        lean = "" if c.get("lean_deg") is None else f"; lean as the foot came up {c['lean_deg']:+.2f} deg"
        if c["measured_x"] is None:
            L.append(f"crossing {c['experiment']:14s} none ({c['note']}); model {c['model_x']:+6.1f} mm{lean}")
        else:
            tip = "" if c.get("tip_deg") is None else f", tipped {c['tip_deg']:.1f} of {c['model_tip_deg']:.1f} deg"
            L.append(f"crossing {c['experiment']:14s} measured {c['measured_x']:+6.1f} mm  model {c['model_x']:+6.1f}"
                     f"  fitted {c['fitted_x']:+6.1f}{lean}{tip}")
    tr = res.tip_rate
    if tr.get("events"):
        L.append(f"tip rate: {tr['events']} tips, e-folding time {tr['median_ms']} ms"
                 + (f" +- {tr['spread_ms']}" if tr.get("spread_ms") else "")
                 + f" (a point mass at the fitted height: {tr['model_ms']} ms; {tr['median_ms']} ms is a point "
                   f"mass {tr['pendulum_mm']} mm up)")
    else:
        L.append("tip rate: no tip to time")
    ck = res.check
    if ck:
        L.append(f"held-out check ({len(ck['poses'])} poses the fit did not use; pitch, roll in deg, both models "
                 "with the fitted IMU offset):")
        for p in ck["poses"]:
            L.append(f"  x {p['x']} z {p['z']}: measured {p['measured'][0]:+.2f} {p['measured'][1]:+.2f}  today's "
                     f"model {p['model'][0]:+.2f} {p['model'][1]:+.2f}  fitted {p['fitted'][0]:+.2f} "
                     f"{p['fitted'][1]:+.2f}")
        L.append(f"  rms error: today's model {ck['rms_model_deg']:.2f} deg, the fitted model "
                 f"{ck['rms_fitted_deg']:.2f} deg")
    else:
        L.append("held-out check: none (script 1 has no check poses): the fit's own residuals are not a verdict")
    if res.battery_mv:
        L.append(f"battery {res.battery_mv[0]:.0f}-{res.battery_mv[1]:.0f} mV over the run")
    if res.walks:
        L.append(f"walks in the fit (the servo response only): {', '.join(res.walks)}")
    for n in res.notes:
        L.append(f"note: {n}")
    L.append("significant: more than 2 +- from the model's value; at limit: the fit ran into the end of its range, "
             "so the value is not measured. Nothing here changes the walks' model.")
    return "\n".join(L)


def to_json(res: FitResult, inputs: list[str]) -> dict:
    rows = list(ROWS) + [n for n in res.values if n.startswith(("floor_", "walk_"))]
    out = {"format": 2, "inputs": inputs, "parameters": {}, "rms_tick_deg": round(res.rms_tick_deg, 4),
           "rms_model_deg": None if res.rms_model_deg is None else round(res.rms_model_deg, 4),
           "inflation": round(res.inflation, 2), "ticks": res.n_ticks, "holds": res.n_holds,
           "sigma_hold_deg": round(res.sigma_hold_deg, 4), "repeatability": res.repeatability,
           "hysteresis": res.hysteresis, "check": res.check, "crossings": res.crossings,
           "tip_rate": res.tip_rate, "walks": res.walks, "notes": res.notes, "iterations": res.iterations,
           "battery_mv": res.battery_mv}
    for n in rows:
        what, unit = WHAT.get(n, (n, "deg"))
        p = BY_NAME.get(n)
        out["parameters"][n] = {"value": round(res.values[n], 4), "sd": round(res.sd.get(n, 0.0), 4),
                                "model": round(res.model.get(n, 0.0), 4), "unit": unit, "what": what,
                                "fitted": n in res.names, "significant": bool(res.significant(n)),
                                "at_limit": bool(res.at_limit.get(n, False)),
                                "measured": n in res.names and not res.at_limit.get(n, False),
                                "where": p.where if p is not None else "", "verdict": res.verdict(n)}
    out["x_scale_check"] = {k: round(float(x), 4) for k, x in res.x_scale_check.items()}
    idx = {n: i for i, n in enumerate(res.names)}
    core = [n for n in ROWS if n in idx]
    sd = np.sqrt(np.maximum(np.diag(res.cov), 1e-30))
    corr = res.cov / np.outer(sd, sd)
    out["correlation"] = {a: {b: round(float(corr[idx[a], idx[b]]), 3) for b in core} for a in core}
    return out


def write_json(res: FitResult, inputs: list[str], path) -> None:
    with open(path, "w") as f:
        json.dump(to_json(res, inputs), f, indent=1, default=float)
        f.write("\n")
