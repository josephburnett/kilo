"""The self-measurement routine's script: which poses, in which order, for
how long (docs/self-measurement.md). kernel/mind/measure/script.cpp is its
port, tick for tick (kernel/host/tests/measure_vectors.txt, made by
gen_measure_vectors.py from this module, checks they agree).

A pose is where the four feet are, as a POSE key in foot mode gives them: ``x`` mm fore-aft
from under the shoulder (+ forward) and ``z`` mm below it, whole mm (the
routine sends whole mm, times 4 in q2). Every pose is the home pose (x 0, z HOME_Z: the
body's STANCE) plus each step's offsets. Moving the feet back (x < 0) moves
the body forward over them.

SCRIPT 2 (what the routine plays today; EXPERIMENTS, STEPS):

  level          stand still: the IMU's offset (with the floor's slope)
  stairs_pitch   the front legs shorter and the back longer by 2, 4, 6 mm
                 and the other way, each level held, each reached once
                 from below and once from above: how much he tilts per mm
                 (the feet's rectangle: ``length``), how differently he
                 comes to rest from either side (the hysteresis), and, in
                 the 6 mm jumps, the servos' dead time and time constant
  stairs_roll    the same left and right by 2, 4 mm: ``track``
  twist          one diagonal pair of legs longer and the other shorter by
                 1 and 2 mm, both ways, with the body 20 mm forward so that
                 the front feet carry him: the pitch is a V whose point is
                 where his own built-in twist is undone
  splay          nose down, the front feet forward and the back feet back,
                 then both inward, twice: the rectangle longer and shorter,
                 the legs' x scale against their z scale
  edge_lf ..     one foot 5 mm up, held (the lean of the tripod is data),
  edge_rb        then the body swept over the feet until the centre of mass
                 crosses the diagonal and Kilo tips onto the lifted foot
                 (caught at once, it is 5 mm away): the centre of mass
                 fore-aft and sideways
  edge_lf_away,  the same with the body rolled, the mass leaning away from
  edge_lf_toward,the lifted foot or toward it: against the flat sweep of
  edge_rf_away,  the same foot the crossing moves by the height of the
  edge_lb_toward centre of mass times the lean (LF both ways; RF and LB the
                 one way the RB shoulder's reach allows)
  check          six held poses the fit does not use: what the fitted model
                 predicts for them, against today's model, is the verdict
  level_end      stand still again: did the IMU drift

SCRIPT 1 (2026-11-21, the first design; SCRIPTS[1]) is kept so that its
recordings still fit: level, steps, four flat sweeps, two rolled ones on
different feet, level_end.

THE TIP (script 2; ``TipWatch``, ported in kernel/mind/measure/measure.cpp).
Lifting a foot leans the tripod a degree or two by itself (legs that give,
a twist): that is not a tip. The baseline is the mean of the last
TIP_BASE_TICKS of the lifted hold; a sweep has tipped once the tilt toward
the lifted foot is TIP_RISE_DEG above it AND has risen TIP_JUMP_DEG within
the last TIP_JUMP_TICKS (a tip is a jump, a lean is not). Then the sweep
goes on TIP_EXTRA_MM and the foot comes down. Already TIP_UP_FRACTION of
the model's tip from the settle hold's tilt during the lifted hold: he
tipped as the foot came up, and the sweep is only TIP_EXTRA_MM long.
``Runner`` plays the script tick by tick and is told of a tip (``tip``).
"""

from __future__ import annotations

from dataclasses import dataclass

from kilo.planner import model as PM
from kilo.protocol import kilo_protocol as P

SCRIPT_VERSION = 2
HOME_Z = 76                 # mm: STANCE's height (spec gait.stance_h_256 205 of L1 + L2 = 76.07)
LIFT_MM = 5                 # a lifted foot's height: how far Kilo can tip before the floor catches him
TICKS_PER_MM = 6            # a sweep's pace: 1 mm every 6 ticks (8.3 mm/s)
TIP_FRACTION = 0.4          # script 1's rule: tipped once the tilt had moved this much of the model's tip
TIP_EXTRA_MM = 3            # a sweep goes on this far past the tip
TIP_SEEN_LAG_TICKS = 7      # kilo.measure.simulate: the newest TELEMETRY the routine has when it plans a tick
TIP_BASE_TICKS = 10         # script 2: the baseline is the mean of the lifted hold's last ticks
TIP_RISE_DEG = 1.5          # ...a tip: the tilt toward the lifted foot this far above it
TIP_JUMP_DEG = 1.0          # ...and risen this much
TIP_JUMP_TICKS = 8          # ...within this many ticks
TIP_UP_FRACTION = 0.6       # tipped as the foot came up: this much of the model's tip from the settle hold's tilt
LIFTED_TICKS = 25           # the lifted hold (its lean is data)
HOME_HOLD = 16              # an experiment's last hold, back at home (how well he comes back is data)
STEP_PITCH_MM = 6           # the largest pitch stair: front legs this much shorter, back longer (5.6 deg)
STEP_ROLL_MM = 4            # the largest roll stair: left legs shorter, right longer (7.0 deg)
STAIR_MM = 2                # a stair
STAIR_HOLD = 26             # ticks a stair is held (after its STAIR_MOVE ticks of move)
STAIR_MOVE = 2
TWIST_X = -20               # the twist steps' feet x: the body 20 mm forward, the front feet carry him
TWIST_MM = 2                # the largest twist step: LF and RB this much longer, RF and LB shorter
SPLAY_PITCH_MM = 6          # the splay's pitch: front legs shorter, back longer (5.6 deg)
SPLAY_OUT_MM = 16           # front feet this far forward and back feet as far back
SPLAY_IN_MM = 14            # ...and inward (the RB shoulder's reach)
ROLL_SWEEP_MM = 4           # a rolled sweep away from the lifted foot: 7.0 deg
ROLL_SWEEP_B_MM = 3         # ...and toward it (5.3 deg: deeper would leave the mass too near the side)
TICK_MS = P.TICK_MS

MOVE, HOLD, SWEEP = 0, 1, 2
KIND_NAMES = ("move", "hold", "sweep")


@dataclass(frozen=True)
class Experiment:
    name: str
    lifted: int = -1            # the foot a sweep lifts (planner leg index), -1 none
    tip_a16: tuple = (0, 0)     # the model's tip: (pitch, roll) change onto the lifted foot, 1/16 deg
    measures: str = ""
    roll_mm: int = 0            # a sweep's roll (+ left side down)
    of: str = ""                # a rolled sweep: the flat sweep of the same foot


@dataclass(frozen=True)
class Step:
    exp: int                    # EXPERIMENTS index
    phase: str                  # the mark's name for this part
    kind: int                   # MOVE (to the offsets over `ticks`), HOLD, SWEEP (x 1 mm per TICKS_PER_MM toward dx)
    ticks: int                  # MOVE, HOLD: its length; SWEEP: its longest (= |dx - x| * TICKS_PER_MM)
    dx: tuple | None = (0, 0, 0, 0)  # the feet's x offsets at the step's end (LF RF RB LB); None: where they are
    dz: tuple = (0, 0, 0, 0)    # the legs' z offsets (- = shorter: that corner lower, or that foot lifted)


@dataclass(frozen=True)
class ScriptDef:
    version: int
    experiments: tuple
    steps: tuple

    def max_ticks(self) -> int:
        return sum(st.ticks for st in self.steps)

    def index(self, name: str) -> int:
        return next(i for i, e in enumerate(self.experiments) if e.name == name)


def _roll(mm: int) -> tuple:
    """z offsets that roll the body: + left side down (TELEMETRY roll +)."""
    return (-mm, mm, mm, -mm)


def _pitch(mm: int) -> tuple:
    """z offsets that pitch the body: + nose down (TELEMETRY pitch +)."""
    return (-mm, -mm, mm, mm)


def _twist(mm: int) -> tuple:
    """z offsets that twist the legs: + LF and RB longer, RF and LB shorter."""
    return (mm, -mm, mm, -mm)


def _add(a: tuple, b: tuple) -> tuple:
    return tuple(p + q for p, q in zip(a, b))


def _lift(dz: tuple, leg: int) -> tuple:
    return tuple(v - LIFT_MM if i == leg else v for i, v in enumerate(dz))


ZERO = (0, 0, 0, 0)

# The model's crossing for each sweep (kilo.measure.simulate.crossings() computes them; test_measure
# checks these): the feet's x where the centre of mass is over the edge, at the model's centre of mass.
# A sweep starts SWEEP_SPAN_MM on the side where the tripod stands and ends at most as far past it
# (ROLL_SPAN_MM rolled: the feet's reach); room for the centre of mass 7 mm off sideways or 15 mm
# fore-aft (the by-hand fit of 2026-11-21 had it 5 mm back and 5 mm right of its model).
SWEEP_SPAN_MM = 20
ROLL_SPAN_MM = 16
RB_BACK_MM = 6              # edge_rb: its sweep starts this much further back, and ends at RB_MAX_X
RB_MAX_X = 14               # (the RB shoulder's pulse clamp, stance.KILO_CZP: no further forward with RB up)

# the model's crossings (feet x, mm): front feet lifted 8.1, back 8.3; rolled 4 mm away -12.0 (front), 3 mm
# toward +23.2 (LF), -6.8 (LB)
_X_FRONT, _X_BACK, _X_AWAY, _X_LF_TOWARD, _X_LB_TOWARD = 8, 8, -12, 23, -7
# script 1's rolled sweeps (RF 4 mm away, RB 3 mm toward)
_X_ROLL, _X_ROLL_B = -12, -7


def _edge(exp: int, leg: int, x0: int, x1: int, roll_mm: int = 0, lifted_ticks: int = LIFTED_TICKS,
          travel: int = 20, home_hold: int = HOME_HOLD) -> list:
    dz = _roll(roll_mm)
    up = _lift(dz, leg)
    return [
        Step(exp, "to_start", MOVE, travel, (x0,) * 4, dz),
        Step(exp, "settle", HOLD, 10, (x0,) * 4, dz),
        Step(exp, "lift", MOVE, 10, (x0,) * 4, up),
        Step(exp, "lifted", HOLD, lifted_ticks, (x0,) * 4, up),
        Step(exp, "sweep", SWEEP, abs(x1 - x0) * TICKS_PER_MM, (x1,) * 4, up),
        Step(exp, "lower", MOVE, 10, None, dz),   # None: the x where the sweep ended
        Step(exp, "lowered", HOLD, 10, None, dz),
        Step(exp, "home", MOVE, travel, ZERO, ZERO),
        Step(exp, "home_hold", HOLD, home_hold, ZERO, ZERO),
    ]


# ---- script 1 (2026-11-21) ----------------------------------------------------------------------------
EXPERIMENTS_1 = (
    Experiment("level", measures="the IMU's offsets (with the floor's slope)"),
    Experiment("steps", measures="servo dead time and time constant; leg sag"),
    Experiment("edge_lf", PM.LF, (37, 70), "centre of mass x and y (over RF-LB)"),
    Experiment("edge_rf", PM.RF, (37, -70), "centre of mass x and y (over LF-RB)"),
    Experiment("edge_lb", PM.LB, (-37, 70), "centre of mass x and y (over LF-RB, the other way)"),
    Experiment("edge_rb", PM.RB, (-37, -70), "centre of mass x and y (over RF-LB, the other way)"),
    Experiment("edge_rf_roll", PM.RF, (37, -70), "centre of mass height (rolled left side down)", ROLL_SWEEP_MM,
               "edge_rf"),
    Experiment("edge_rb_roll", PM.RB, (-37, -69), "centre of mass height (rolled right side down)",
               -ROLL_SWEEP_B_MM, "edge_rb"),
    Experiment("level_end", measures="the IMU's drift over the run"),
)


def _steps_1() -> list:
    s = [Step(0, "level", HOLD, 75)]
    p, r = STEP_PITCH_MM, STEP_ROLL_MM
    for name, dz in (("pitch_down", (-p, -p, p, p)), ("pitch_level", (0,) * 4), ("pitch_up", (p, p, -p, -p)),
                     ("pitch_level", (0,) * 4), ("roll_left_down", _roll(r)), ("roll_level", (0,) * 4),
                     ("roll_right_down", _roll(-r)), ("roll_level", (0,) * 4)):
        s.append(Step(1, name, MOVE, 2, (0,) * 4, dz))
        s.append(Step(1, name + "_hold", HOLD, 38, (0,) * 4, dz))
    S_ = SWEEP_SPAN_MM
    s += _edge(2, PM.LF, _X_FRONT + S_, _X_FRONT - S_, 0, 15, 25, 10)
    s += _edge(3, PM.RF, _X_FRONT + S_, _X_FRONT - S_, 0, 15, 25, 10)
    s += _edge(4, PM.LB, _X_BACK - S_, _X_BACK + S_, 0, 15, 25, 10)
    s += _edge(5, PM.RB, _X_BACK - S_ - RB_BACK_MM, RB_MAX_X, 0, 15, 25, 10)
    S_ = ROLL_SPAN_MM
    s += _edge(6, PM.RF, _X_ROLL + S_, _X_ROLL - S_, ROLL_SWEEP_MM, 15, 25, 10)
    s += _edge(7, PM.RB, _X_ROLL_B - S_, _X_ROLL_B + S_, -ROLL_SWEEP_B_MM, 15, 25, 10)
    s.append(Step(8, "level_end", HOLD, 50))
    return s


# ---- script 2 -------------------------------------------------------------------------------------------
EXPERIMENTS = (
    Experiment("level", measures="the IMU's offsets (with the floor's slope)"),
    Experiment("stairs_pitch", measures="tilt per mm front to back (length); hysteresis; servo dead time and "
                                        "time constant"),
    Experiment("stairs_roll", measures="tilt per mm left to right (track); hysteresis; the servos' timing"),
    Experiment("twist", measures="the legs' built-in twist (where the pitch's V has its point); their sag"),
    Experiment("splay", measures="the legs' x scale against their z scale"),
    Experiment("edge_lf", PM.LF, (37, 70), "centre of mass x and y (over RF-LB)"),
    Experiment("edge_rf", PM.RF, (37, -70), "centre of mass x and y (over LF-RB)"),
    Experiment("edge_lb", PM.LB, (-37, 70), "centre of mass x and y (over LF-RB, the other way)"),
    Experiment("edge_rb", PM.RB, (-37, -70), "centre of mass x and y (over RF-LB, the other way)"),
    Experiment("edge_lf_away", PM.LF, (37, 70), "centre of mass height (LF up, rolled right side down)",
               -ROLL_SWEEP_MM, "edge_lf"),
    Experiment("edge_lf_toward", PM.LF, (37, 69), "centre of mass height (LF up, rolled left side down)",
               ROLL_SWEEP_B_MM, "edge_lf"),
    Experiment("edge_rf_away", PM.RF, (37, -70), "centre of mass height (RF up, rolled left side down)",
               ROLL_SWEEP_MM, "edge_rf"),
    Experiment("edge_lb_toward", PM.LB, (-37, 69), "centre of mass height (LB up, rolled left side down)",
               ROLL_SWEEP_B_MM, "edge_lb"),
    Experiment("check", measures="nothing: held poses the fit does not use (the held-out check)"),
    Experiment("level_end", measures="the IMU's drift over the run"),
)

# the stairs: each level's half difference (mm), in the order played from home. Every level but the
# ends is reached once from below and once from above; a jump of the whole height is a timing step.
STAIRS_PITCH = (2, 4, 6, 0, -2, -4, -6, 0, 6, 4, 2, 0, -6, -4, -2, 0)
STAIRS_ROLL = (2, 4, 0, -2, -4, 0, 4, 2, 0, -4, -2, 0)
TWIST_STEPS = (1, 2, 1, 0, -1, -2, -1, 0)
# the check poses: (name, dx, dz, lifted foot or -1)
CHECK_POSES = (
    ("pitch_roll", ZERO, _add(_pitch(3), _roll(2)), -1),
    ("pitch_roll_back", ZERO, _add(_pitch(-4), _roll(-3)), -1),
    ("forward_rolled", (-10,) * 4, _roll(3), -1),
    ("back_pitched", (10,) * 4, _pitch(2), -1),
    ("lb_up", (-16,) * 4, ZERO, PM.LB),
    ("rf_up_rolled", (22,) * 4, _roll(2), PM.RF),
)


def _steps_2() -> list:
    E = {e.name: i for i, e in enumerate(EXPERIMENTS)}
    s = [Step(E["level"], "level", HOLD, 75)]
    for name, levels, shape in (("stairs_pitch", STAIRS_PITCH, _pitch), ("stairs_roll", STAIRS_ROLL, _roll)):
        for lv in levels:
            s.append(Step(E[name], "step", MOVE, STAIR_MOVE, ZERO, shape(lv)))
            s.append(Step(E[name], "hold", HOLD, STAIR_HOLD, ZERO, shape(lv)))
    e = E["twist"]
    tx = (TWIST_X,) * 4
    s.append(Step(e, "to_start", MOVE, 20, tx, ZERO))
    s.append(Step(e, "hold", HOLD, STAIR_HOLD, tx, ZERO))
    for k in TWIST_STEPS:
        s.append(Step(e, "step", MOVE, STAIR_MOVE, tx, _twist(k)))
        s.append(Step(e, "hold", HOLD, STAIR_HOLD, tx, _twist(k)))
    s.append(Step(e, "home", MOVE, 20, ZERO, ZERO))
    s.append(Step(e, "home_hold", HOLD, HOME_HOLD, ZERO, ZERO))
    e = E["splay"]
    dz = _pitch(SPLAY_PITCH_MM)
    out = (SPLAY_OUT_MM, SPLAY_OUT_MM, -SPLAY_OUT_MM, -SPLAY_OUT_MM)
    inn = (-SPLAY_IN_MM, -SPLAY_IN_MM, SPLAY_IN_MM, SPLAY_IN_MM)
    s.append(Step(e, "step", MOVE, 4, ZERO, dz))
    s.append(Step(e, "hold", HOLD, STAIR_HOLD, ZERO, dz))
    for dx in (out, inn, out, inn, ZERO):  # each end reached twice: the signal is small (0.3 deg per 10 %)
        s.append(Step(e, "step", MOVE, 12, dx, dz))
        s.append(Step(e, "hold", HOLD, STAIR_HOLD, dx, dz))
    s.append(Step(e, "home", MOVE, 4, ZERO, ZERO))
    s.append(Step(e, "home_hold", HOLD, HOME_HOLD, ZERO, ZERO))
    S_ = SWEEP_SPAN_MM
    # front feet lifted: the tripod stands with the feet forward (the body back), the sweep moves them back
    s += _edge(E["edge_lf"], PM.LF, _X_FRONT + S_, _X_FRONT - S_)
    s += _edge(E["edge_rf"], PM.RF, _X_FRONT + S_, _X_FRONT - S_)
    # back feet lifted: the other way
    s += _edge(E["edge_lb"], PM.LB, _X_BACK - S_, _X_BACK + S_)
    # RB's shoulder reaches no further forward than 14 mm with the foot up (its pulse clamp, stance.KILO_CZP):
    # its sweep starts further back instead
    s += _edge(E["edge_rb"], PM.RB, _X_BACK - S_ - RB_BACK_MM, RB_MAX_X)
    # rolled, against the flat sweep of the same foot. All four feet sweep together and the RB shoulder
    # cannot reach forward with the right side down, which leaves: LF both ways, RF away, LB toward.
    S_ = ROLL_SPAN_MM
    s += _edge(E["edge_lf_away"], PM.LF, _X_AWAY + S_, _X_AWAY - S_, -ROLL_SWEEP_MM)
    s += _edge(E["edge_lf_toward"], PM.LF, _X_LF_TOWARD + S_, _X_LF_TOWARD - S_, ROLL_SWEEP_B_MM)
    s += _edge(E["edge_rf_away"], PM.RF, _X_AWAY + S_, _X_AWAY - S_, ROLL_SWEEP_MM)
    s += _edge(E["edge_lb_toward"], PM.LB, _X_LB_TOWARD - S_, _X_LB_TOWARD + S_, ROLL_SWEEP_B_MM)
    e = E["check"]
    for _name, dx, dz, leg in CHECK_POSES:
        s.append(Step(e, "step", MOVE, 15, dx, dz))
        if leg < 0:
            s.append(Step(e, "hold", HOLD, STAIR_HOLD, dx, dz))
        else:
            s.append(Step(e, "hold", HOLD, 10, dx, dz))
            s.append(Step(e, "up", MOVE, 10, dx, _lift(dz, leg)))
            s.append(Step(e, "up_hold", HOLD, STAIR_HOLD, dx, _lift(dz, leg)))
            s.append(Step(e, "down", MOVE, 10, dx, dz))
    s.append(Step(e, "home", MOVE, 20, ZERO, ZERO))
    s.append(Step(e, "home_hold", HOLD, HOME_HOLD, ZERO, ZERO))
    s.append(Step(E["level_end"], "level_end", HOLD, 50))
    return s


SCRIPTS = {1: ScriptDef(1, EXPERIMENTS_1, tuple(_steps_1())),
           2: ScriptDef(2, EXPERIMENTS, tuple(_steps_2()))}
SCRIPT = SCRIPTS[SCRIPT_VERSION]
STEPS = SCRIPT.steps


def max_ticks(script: ScriptDef = SCRIPT) -> int:
    return script.max_ticks()


@dataclass
class Tick:
    tick: int                   # since the script's first tick (0..)
    step: int                   # STEPS index
    x: tuple                    # foot x mm (whole)
    z: tuple                    # foot z mm
    first: bool                 # the step's first tick (a mark)
    into: int = 0               # ticks into its step (0..)


def _lerp(a: int, b: int, j: int, n: int) -> int:
    """a + (b - a) j / n, rounded half away from zero, in integers (as the C++)."""
    num = (b - a) * j
    q, rem = divmod(abs(num), n)
    if 2 * rem >= n:
        q += 1
    return a + (q if num >= 0 else -q)


class Runner:
    """Plays a script's steps tick by tick (``next()``), as
    kernel/mind/measure/script.cpp Script does. ``tip(tick_seen)`` ends the
    sweep under way early: the tilt had moved enough at that tick (its x
    then + TIP_EXTRA_MM is the end)."""

    def __init__(self, script: ScriptDef = SCRIPT):
        self.script = script
        self.steps = script.steps
        self.i = 0                         # the step
        self.j = 0                         # ticks into it
        self.t = 0                         # ticks since the start
        self.x = [0, 0, 0, 0]
        self.z = [HOME_Z] * 4
        self.start_x = list(self.x)
        self.start_z = list(self.z)
        self.sweep_end = None              # the sweep's end x (its target until a tip)
        self.tipped_at = None              # the tick whose TELEMETRY the tip was seen in (this sweep's)
        self.tip_pending = False           # seen in the lifted hold: the sweep to come is cut short
        self.history: dict[int, int] = {}  # tick -> sweep x (for tip(): the x at the tick seen)
        self.done = False

    def _begin(self) -> None:
        self.start_x, self.start_z = list(self.x), list(self.z)
        st = self.steps[self.i]
        if st.kind == SWEEP:
            self.sweep_end = st.dx[0]
            if self.tip_pending:  # tipped as the foot came up: the sweep is only the TIP_EXTRA_MM
                d = 1 if st.dx[0] > self.start_x[0] else -1
                end = self.start_x[0] + d * TIP_EXTRA_MM
                self.sweep_end = min(end, st.dx[0]) if d > 0 else max(end, st.dx[0])
                self.tip_pending = False
        else:
            self.sweep_end = None
            if st.phase != "lifted":
                self.tipped_at = None
                self.tip_pending = False

    def step(self) -> Step:
        return self.steps[self.i]

    def tip(self, tick_seen: int) -> bool:
        """A tip seen in the TELEMETRY of script tick ``tick_seen``: the
        sweep under way ends TIP_EXTRA_MM past the x it had then; seen in
        the lifted hold before it (he tipped as the foot came up), the
        sweep will be only TIP_EXTRA_MM long. False if neither is under way
        (or it has tipped already)."""
        if self.done or self.tipped_at is not None:
            return False
        st = self.steps[self.i]
        if st.phase == "lifted" and self.i + 1 < len(self.steps) and self.steps[self.i + 1].kind == SWEEP:
            self.tipped_at = tick_seen
            self.tip_pending = True
            return True
        if st.kind != SWEEP:
            return False
        x_then = self.history.get(tick_seen, self.x[0])
        d = 1 if st.dx[0] > self.start_x[0] else -1
        end = x_then + d * TIP_EXTRA_MM
        lim = st.dx[0]
        self.sweep_end = min(end, lim) if d > 0 else max(end, lim)
        self.tipped_at = tick_seen
        return True

    def next(self) -> Tick | None:
        """The next tick's pose, or None when the script is over."""
        if self.done:
            return None
        while True:
            st = self.steps[self.i]
            if self.j == 0:
                self._begin()
            n_needed = st.ticks
            if st.kind == SWEEP:
                n_needed = abs(self.sweep_end - self.start_x[0]) * TICKS_PER_MM
            if self.j < n_needed:
                break
            self.i += 1
            self.j = 0
            if self.i == len(self.steps):
                self.done = True
                return None
        first = self.j == 0
        into = self.j
        self.j += 1
        tx = st.dx if st.dx is not None else tuple(self.start_x)
        tz = tuple(HOME_Z + v for v in st.dz)
        if st.kind == MOVE:
            self.x = [_lerp(self.start_x[k], tx[k], self.j, st.ticks) for k in range(4)]
            self.z = [_lerp(self.start_z[k], tz[k], self.j, st.ticks) for k in range(4)]
        elif st.kind == SWEEP:
            d = 1 if st.dx[0] > self.start_x[0] else -1
            mm = (self.j - 1) // TICKS_PER_MM + 1  # 1 mm at the step's first tick, then every TICKS_PER_MM
            self.x = [self.start_x[k] + d * mm for k in range(4)]
            self.z = list(tz)
            self.history[self.t] = self.x[0]
        else:
            self.z = list(self.z)
        tick = Tick(self.t, self.i, tuple(self.x), tuple(self.z), first, into)
        self.t += 1
        return tick


def poses(tip_at: dict[int, int] | None = None, script: ScriptDef = SCRIPT) -> list[Tick]:
    """Every tick of a run in which no sweep tips (``tip_at``: experiment ->
    the script tick whose TELEMETRY saw the tip, applied when the planner
    is TIP_SEEN_LAG_TICKS past it)."""
    r = Runner(script)
    out = []
    pending = dict(tip_at or {})
    while True:
        st = r.step() if not r.done else None
        if st is not None and st.exp in pending and pending[st.exp] + TIP_SEEN_LAG_TICKS <= r.t:
            if r.tip(pending.pop(st.exp)):
                pass
        t = r.next()
        if t is None:
            return out
        out.append(t)


# ---- the tip, as the routine sees it ---------------------------------------------------------------------
class TipWatch:
    """Script 2's tip rule on the TELEMETRY of one run, tick by tick (the
    module docstring; kernel/mind/measure/measure.cpp Measure::watch is its
    port). ``see`` takes each measured tick in order, with the step it
    belongs to and how far into it, and says "tip" (a tip in the sweep),
    "up" (tipped as the foot came up) or None. ``settle`` and ``lifted``
    are the two baselines of the sweep under way (pitch, roll), None until
    known; ``rel`` the last tilt toward the lifted foot above the lifted
    baseline."""

    def __init__(self, script: ScriptDef = SCRIPT):
        self.script = script
        self.exp = -1
        self.settle = None
        self.settle_n = 0
        self.lifted = None
        self.lifted_n = 0
        self.rels: dict[int, float] = {}
        self.rel = 0.0
        self.fired = False

    def see(self, tick: int, step: int, into: int, pitch: float, roll: float) -> str | None:
        st = self.script.steps[step]
        ex = self.script.experiments[st.exp]
        if ex.lifted < 0:
            return None
        if st.phase == "settle":  # all four feet down, before the lift
            if self.exp != st.exp or self.lifted is not None:
                self.exp, self.settle, self.settle_n = st.exp, (0.0, 0.0), 0
                self.lifted, self.lifted_n, self.rels, self.fired = None, 0, {}, False
            n = self.settle_n
            self.settle = ((self.settle[0] * n + pitch) / (n + 1), (self.settle[1] * n + roll) / (n + 1))
            self.settle_n = n + 1
            return None
        if self.exp != st.exp or self.settle_n == 0 or self.fired or st.phase not in ("lifted", "sweep"):
            return None
        up, ur = ex.tip_a16[0] / 16.0, ex.tip_a16[1] / 16.0
        T = (up * up + ur * ur) ** 0.5
        proj = ((pitch - self.settle[0]) * up + (roll - self.settle[1]) * ur) / T
        if st.phase == "lifted":
            if proj >= TIP_UP_FRACTION * T:
                self.fired = True
                return "up"
            if into >= st.ticks - TIP_BASE_TICKS:
                n = self.lifted_n
                base = self.lifted if self.lifted is not None else (0.0, 0.0)
                self.lifted = ((base[0] * n + pitch) / (n + 1), (base[1] * n + roll) / (n + 1))
                self.lifted_n = n + 1
            return None
        if self.lifted is None:  # no TELEMETRY in the lifted hold's end: no baseline, no tip
            return None
        base = ((self.lifted[0] - self.settle[0]) * up + (self.lifted[1] - self.settle[1]) * ur) / T
        rel = proj - base
        self.rel = rel
        self.rels[tick] = rel
        self.rels.pop(tick - 16, None)
        first = tick - into  # the sweep's first tick: before it the tilt was the baseline
        old = None
        for back in range(TIP_JUMP_TICKS, TIP_JUMP_TICKS + 5):
            k = tick - back
            if k < first:
                old = 0.0
                break
            if k in self.rels:
                old = self.rels[k]
                break
        if old is None:
            return None
        if rel >= TIP_RISE_DEG and rel - old >= TIP_JUMP_DEG:
            self.fired = True
            return "tip"
        return None
