"""The self-measurement routine's script: which poses, in which order, for
how long (docs/self-measurement.md). kernel/mind/measure/script.cpp is its
port, tick for tick (kernel/host/tests/measure_vectors.txt, made by
gen_measure_vectors.py from this module, checks they agree).

A pose is where the four feet are, as FEET gives them: ``x`` mm fore-aft
from under the shoulder (+ forward) and ``z`` mm below it, whole mm (the
FEET fields are integers). Every pose is the home pose (x 0, z HOME_Z: the
body's STANCE) plus each step's offsets. Moving the feet back (x < 0) moves
the body forward over them.

The experiments (EXPERIMENTS; what each one measures in the docs):

  level          stand still: the IMU's offset (with the floor's slope)
  steps          pitch and roll steps, both ways: the servos' dead time and
                 time constant; the size of each step: the legs' sag
  edge_lf ..     one foot 5 mm up, the body swept over the feet until the
  edge_rb        centre of mass crosses the diagonal and Kilo tips onto
                 the lifted foot (caught at once, it is 5 mm away): where
                 the centre of mass is, fore-aft and sideways
  edge_rf_roll,  the same with the body rolled (7 and 5.3 deg), the centre of mass
  edge_rb_roll   leaning away from the lifted foot, then toward it: how high it is
  level_end      stand still again: did the IMU drift

A sweep ends early (``tip``): once the tilt has moved TIP_FRACTION of the
model's tip toward the lifted foot, it goes on TIP_EXTRA_MM and then the
foot comes down. ``Runner`` plays the script tick by tick and asks its
caller whether a tip has been seen (the C++ routine reads TELEMETRY).
"""

from __future__ import annotations

from dataclasses import dataclass

from kilo.planner import model as PM
from kilo.protocol import kilo_protocol as P

HOME_Z = 76                 # mm: STANCE's height (spec gait.stance_h_256 205 of L1 + L2 = 76.07)
LIFT_MM = 5                 # a lifted foot's height: how far Kilo can tip before the floor catches him
TICKS_PER_MM = 6            # a sweep's pace: 1 mm every 6 ticks (8.3 mm/s)
TIP_FRACTION = 0.4          # a sweep has tipped once the tilt moved this much of the model's tip
TIP_EXTRA_MM = 3            # ...and goes on this far past it
TIP_SEEN_LAG_TICKS = 7      # kilo.measure.simulate: the newest TELEMETRY the routine has when it plans a tick
STEP_PITCH_MM = 6           # the pitch steps: the front legs this much shorter and the back longer (5.6 deg)
STEP_ROLL_MM = 4            # the roll steps: the left legs shorter and the right longer (7.0 deg)
ROLL_SWEEP_MM = 4           # the rolled sweep away from the lifted foot: 7.0 deg
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


@dataclass(frozen=True)
class Step:
    exp: int                    # EXPERIMENTS index
    phase: str                  # the mark's name for this part
    kind: int                   # MOVE (to the offsets over `ticks`), HOLD, SWEEP (x 1 mm per TICKS_PER_MM toward dx)
    ticks: int                  # MOVE, HOLD: its length; SWEEP: its longest (= |dx - x| * TICKS_PER_MM)
    dx: tuple = (0, 0, 0, 0)    # the feet's x offsets at the step's end (LF RF RB LB)
    dz: tuple = (0, 0, 0, 0)    # the legs' z offsets (- = shorter: that corner lower, or that foot lifted)


def _roll(mm: int) -> tuple:
    """z offsets that roll the body: + left side down (TELEMETRY roll +)."""
    return (-mm, mm, mm, -mm)


def _lift(dz: tuple, leg: int) -> tuple:
    return tuple(v - LIFT_MM if i == leg else v for i, v in enumerate(dz))


# The model's crossing for each sweep (kilo.measure.simulate.crossings() computes them; test_measure
# checks these): the feet's x where the centre of mass is over the edge, at the model's centre of mass.
# A sweep starts SWEEP_SPAN_MM on the side where the tripod stands and ends at most as far past it
# (ROLL_SPAN_MM rolled: the feet's reach); room for the centre of mass 7 mm off sideways or 15 mm
# fore-aft (the by-hand fit of 2026-11-21 had it 5 mm back and 5 mm right of its model).
SWEEP_SPAN_MM = 20
ROLL_SPAN_MM = 16
RB_BACK_MM = 6              # edge_rb: its sweep starts this much further back, and ends at RB_MAX_X
RB_MAX_X = 14               # (the RB shoulder's pulse clamp, stance.KILO_CZP: no further forward with RB up)


def _edge(exp: int, leg: int, x0: int, x1: int, roll_mm: int = 0) -> list:
    dz = _roll(roll_mm)
    up = _lift(dz, leg)
    return [
        Step(exp, "to_start", MOVE, 25, (x0,) * 4, dz),
        Step(exp, "settle", HOLD, 10, (x0,) * 4, dz),
        Step(exp, "lift", MOVE, 10, (x0,) * 4, up),
        Step(exp, "lifted", HOLD, 15, (x0,) * 4, up),
        Step(exp, "sweep", SWEEP, abs(x1 - x0) * TICKS_PER_MM, (x1,) * 4, up),
        Step(exp, "lower", MOVE, 10, None, dz),   # None: the x where the sweep ended
        Step(exp, "lowered", HOLD, 10, None, dz),
        Step(exp, "home", MOVE, 25, (0,) * 4, (0,) * 4),
        Step(exp, "home_hold", HOLD, 10, (0,) * 4, (0,) * 4),
    ]


EXPERIMENTS = (
    Experiment("level", measures="the IMU's offsets (with the floor's slope)"),
    Experiment("steps", measures="servo dead time and time constant; leg sag"),
    Experiment("edge_lf", PM.LF, (37, 70), "centre of mass x and y (over RF-LB)"),
    Experiment("edge_rf", PM.RF, (37, -70), "centre of mass x and y (over LF-RB)"),
    Experiment("edge_lb", PM.LB, (-37, 70), "centre of mass x and y (over LF-RB, the other way)"),
    Experiment("edge_rb", PM.RB, (-37, -70), "centre of mass x and y (over RF-LB, the other way)"),
    Experiment("edge_rf_roll", PM.RF, (37, -70), "centre of mass height (rolled left side down)"),
    Experiment("edge_rb_roll", PM.RB, (-37, -69), "centre of mass height (rolled right side down)"),
    Experiment("level_end", measures="the IMU's drift over the run"),
)

# the model's crossings (feet x, mm): front feet lifted 8.1, back 8.3, rolled RF -12.0, rolled RB -6.8
_X_FRONT, _X_BACK, _X_ROLL, _X_ROLL_B = 8, 8, -12, -7


def _steps() -> list:
    s = [Step(0, "level", HOLD, 75)]
    p, r = STEP_PITCH_MM, STEP_ROLL_MM
    for name, dz in (("pitch_down", (-p, -p, p, p)), ("pitch_level", (0,) * 4), ("pitch_up", (p, p, -p, -p)),
                     ("pitch_level", (0,) * 4), ("roll_left_down", _roll(r)), ("roll_level", (0,) * 4),
                     ("roll_right_down", _roll(-r)), ("roll_level", (0,) * 4)):
        s.append(Step(1, name, MOVE, 2, (0,) * 4, dz))
        s.append(Step(1, name + "_hold", HOLD, 38, (0,) * 4, dz))
    S_ = SWEEP_SPAN_MM
    # front feet lifted: the tripod stands with the feet forward (the body back), the sweep moves them back
    s += _edge(2, PM.LF, _X_FRONT + S_, _X_FRONT - S_)
    s += _edge(3, PM.RF, _X_FRONT + S_, _X_FRONT - S_)
    # back feet lifted: the other way
    s += _edge(4, PM.LB, _X_BACK - S_, _X_BACK + S_)
    # RB's shoulder reaches no further forward than 14 mm with the foot up (its pulse clamp, stance.KILO_CZP):
    # its sweep starts further back instead
    s += _edge(5, PM.RB, _X_BACK - S_ - RB_BACK_MM, RB_MAX_X)
    # rolled: RF lifted with the left side down (the centre of mass moves left, away from RF)
    S_ = ROLL_SPAN_MM
    s += _edge(6, PM.RF, _X_ROLL + S_, _X_ROLL - S_, ROLL_SWEEP_MM)
    # RB lifted with the right side down (toward RB): the other way along the sweep as the height grows
    s += _edge(7, PM.RB, _X_ROLL_B - S_, _X_ROLL_B + S_, -ROLL_SWEEP_B_MM)
    s.append(Step(8, "level_end", HOLD, 50))
    return s


STEPS = tuple(_steps())


def max_ticks() -> int:
    return sum(st.ticks for st in STEPS)


@dataclass
class Tick:
    tick: int                   # since the script's first tick (0..)
    step: int                   # STEPS index
    x: tuple                    # FEET x_mm
    z: tuple                    # FEET z_mm
    first: bool                 # the step's first tick (a mark)


def _lerp(a: int, b: int, j: int, n: int) -> int:
    """a + (b - a) j / n, rounded half away from zero, in integers (as the C++)."""
    num = (b - a) * j
    q, rem = divmod(abs(num), n)
    if 2 * rem >= n:
        q += 1
    return a + (q if num >= 0 else -q)


class Runner:
    """Plays STEPS tick by tick (``next()``), as kernel/mind/measure/script.cpp
    Script does. ``tip(tick_seen)`` ends the sweep under way early: the
    tilt had moved enough at that tick (its x then + TIP_EXTRA_MM is the
    end)."""

    def __init__(self):
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
        st = STEPS[self.i]
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
        return STEPS[self.i]

    def tip(self, tick_seen: int) -> bool:
        """A tip seen in the TELEMETRY of script tick ``tick_seen``: the
        sweep under way ends TIP_EXTRA_MM past the x it had then; seen in
        the lifted hold before it (he tipped as the foot came up), the
        sweep will be only TIP_EXTRA_MM long. False if neither is under way
        (or it has tipped already)."""
        if self.done or self.tipped_at is not None:
            return False
        st = STEPS[self.i]
        if st.phase == "lifted" and self.i + 1 < len(STEPS) and STEPS[self.i + 1].kind == SWEEP:
            self.tipped_at = tick_seen
            self.tip_pending = True
            return True
        if st.kind != SWEEP:
            return False
        x_then = self.history.get(tick_seen, self.x[0])
        d = 1 if STEPS[self.i].dx[0] > self.start_x[0] else -1
        end = x_then + d * TIP_EXTRA_MM
        lim = STEPS[self.i].dx[0]
        self.sweep_end = min(end, lim) if d > 0 else max(end, lim)
        self.tipped_at = tick_seen
        return True

    def next(self) -> Tick | None:
        """The next tick's pose, or None when the script is over."""
        if self.done:
            return None
        while True:
            st = STEPS[self.i]
            if self.j == 0:
                self._begin()
            n_needed = st.ticks
            if st.kind == SWEEP:
                n_needed = abs(self.sweep_end - self.start_x[0]) * TICKS_PER_MM
            if self.j < n_needed:
                break
            self.i += 1
            self.j = 0
            if self.i == len(STEPS):
                self.done = True
                return None
        first = self.j == 0
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
        tick = Tick(self.t, self.i, tuple(self.x), tuple(self.z), first)
        self.t += 1
        return tick


def poses(tip_at: dict[int, int] | None = None) -> list[Tick]:
    """Every tick of a run in which no sweep tips (``tip_at``: experiment ->
    the script tick whose TELEMETRY saw the tip, applied when the planner
    is TIP_SEEN_LAG_TICKS past it)."""
    r = Runner()
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
