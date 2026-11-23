"""The step planner: one tick of the whole body at a time.

``Planner.next()`` returns the plan for the next tick: where each foot is
(x, z per leg in whole mm, as the POSE keys carry them), the feet on the ground, and the
attitude the body is predicted to have at that tick. The streamer
(kilo.planner.walker) calls it once per tick, about five ticks ahead of the
body, and tells it what the body reports (``observe``) in between.

How a tick is made (docs/walking-and-balance.md §10):

- **Slots.** The pattern's swing groups take turns (kilo.planner.patterns):
  each slot is a four-foot shift of ``shift`` ticks, then a swing of
  ``swing`` ticks per leg (in the walk the pair's second leg lifts
  ``couple_ticks`` after the first; before its lift and after its
  touchdown it slides with the ground). Feet on the ground slide back at their side's stride
  speed (stride / stance time); a swinging foot lifts on a cosine profile
  (step height at mid-swing, zero speed at lift-off and touchdown) from
  where it left the ground to half a stride ahead of its shoulder (the walk:
  plus the model's centre of mass, patterns.Pattern.centre_com). Speed and
  direction change the stride targets, which move at most the pattern's
  stride_slew_mm per tick (1.5 mm in the walk and the trot, 0.5 in the
  crawl), and the stance time, which moves at most 2 % per tick, with each slot's
  shift following the stance time as it is (so a slower speed never leaves
  a swing more than a stride to carry): any change at any tick is smooth. A change between the walk and the
  trot goes straight over at a slot start (``_switch_ok``); into or out
  of the crawl through the legs' way home: the strides go to 0 and each
  leg steps under its shoulder (as for a stop), then the new pattern
  starts from its first group as a walk starts; a second change while
  that runs just changes where it goes.
- **The lean (the crawl).** On three feet the body stands on a triangle
  whose long edge runs under its middle, so before each lift the planner
  moves the centre of mass inside the triangle of the three feet that stay
  down, at least ``margin_mm`` from every edge, for the whole swing: it
  solves for the smallest body move (a fore-aft shift of every foot, a roll
  and a little pitch by leg-length differences) that gives that margin at
  lift-off and at touchdown (the body model's centre of mass, linearised
  and solved twice). The move runs during the shift, paced by gravity (a
  side that drops falls no faster than it can), and the lift starts
  ``lead`` ticks after it ends (patterns.LEAN_LEAD_MS: three ticks BEFORE
  its end, in the cosine's tail, where less than a tenth of the move is
  left; the margin the plan reports counts it); it is held until the foot
  is down again. The lean and the lift go to the servos together, so the
  servo lag delays both alike. The solve is spread over the swing before the slot (a few
  units of work per tick, from where the feet will be when the slot
  starts), so no tick carries a whole solve; if something changed since (a
  command, a catch) it is solved again at the slot's start while the plan
  waits. The trot and the walk need none: their diagonal pairs stand on
  a line, briefly (150 and 90 ms), and no lean helps there.
- **Prediction.** The attitude for tick t is the plane through the support
  feet the planner commanded, delayed by the servos' dead time and passed
  through their first-order response (BodyModel.servo_lag_ms and
  servo_tau_ms, fitted to the floor recordings: the body follows its
  commands that way). It goes in POSE as the balance's reference:
  the body corrects only what the plan did not predict.
- **From the measured state.** ``observe`` gives the error the body
  reports (measured minus predicted, less the steady offset the walker
  learns). A leg does not lift while that error is above ``gate_deg``: the
  plan waits (up to ``hold_max`` ticks: every foot where it is, nothing
  sliding toward the end of its range) until the body follows the plan
  again. A swinging leg comes down at once (in three ticks, where it is)
  when the body tips toward it by more than ``catch_deg``.
- **Stop.** The strides go to 0; each leg still away from under its
  shoulder takes one more step to it; then the body moves back to level at
  the stance height and ``done`` is set: the walker hands over to the
  STANCE posture.
- **The quick stop** (``halt``; the remote's B6 and C7 and the distance
  guard): every foot in the air comes down at once where it is (in three
  ticks, as a catch) and no other lifts; the strides go to 0 in
  HALT_TICKS ticks (the feet on the ground stop sliding: the body stops);
  all four feet stay down until HALT_HOLD_TICKS after the halt; then each
  leg more than HALT_SQUARE_MM from under its shoulder steps to it (the
  crawl at its fastest cadence), the body levels and ``done`` is set, as
  for the stop. A leg caught on its way (the body tipping toward it) stays
  where it landed: the quick stop never steps it again (STANCE sets it).
- **The trot** (docs/walking-and-balance.md §15; the "long trot" of
  2026-11-21, the only trot since 2026-11-22): its swing grows with the
  speed (patterns.swing_for), at most a tick longer than the step before
  (the build-up) and a tick shorter, one tick on four feet; each diagonal
  pair lands where the body, falling about the line between its two feet
  as an inverted pendulum, ends the diagonal upright (TrotFit.land_x:
  x_c + k v tau ahead of its shoulder, the pendulum fitted to the floor
  recordings of the trot); each swing is long enough for the step it
  carries (a square-up from a long stride, a stride still growing or
  shrinking, a steer's outer side), and one that is not the steady trot's
  (a square-up, the way home into the crawl, the swing shortening on the
  way out of the trot) long enough for no joint to turn more than
  TrotConfig.joint_deg_per_tick along its path. Out of the trot directly
  (into the walk) once its swing is back to 8 ticks.
- **The sent attitude** stays inside POSE's +-30 deg (the body refuses a
  POSE beyond it).
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

from kilo.planner import patterns as pat
from kilo.planner.model import (FRONT, G_MM_S2, LEFT, LEGS, BodyModel, _solve3, fit_attitude, hull, leg_lengths,
                                stability, view_numbers, world_points)
from kilo.protocol import kilo_protocol as P

X_RANGE = (-60, 60)   # foot x mm the walk keeps to (protocol 2 FEET's range; POSE takes -120..120)
Z_RANGE = (30, 95)    # foot z mm (FEET's; POSE takes 15..110)
ATT_MAX_DEG = 30.0    # POSE pitch / roll (the body refuses a POSE beyond it: BAD_ARG ends the stream)
HEIGHT_SLEW_MM = 0.5  # per tick
RECENTRED_MM = 1.5    # a stopping leg this close to its target does not step again
CENTRE_SLEW_MM = 0.5  # the strides' centre (patterns.Pattern.centre_com) moves at most this much a tick
SWITCH_SLACK_MM = 4.0  # a pattern change: every foot within the new half stride plus this (_switch_ok)
STANCE_SLEW = 0.02    # the stance time (and with it the feet's slide) changes at most 2 % per tick
LEAN_UNITS_PER_TICK = 4  # units of a lean solve (Planner._solve_steps) per tick
HALT_TICKS = 6        # the quick stop: the strides go to 0 in this many ticks (120 ms) ...
HALT_HOLD_TICKS = 10  # ... and all four feet stay down this long from the halt (200 ms) ...
HALT_SQUARE_MM = 5.0  # ... then a leg further than this from under its shoulder steps to it
HALT_NOT_LIFTED = -1000  # caught[leg] tick for a leg the quick stop keeps down (it slides, never lifts)


# The trot's fitted pendulum: its numbers are the "trot" view of the body model (protocol/spec.toml
# [model.views.trot]): its centre of mass disagrees with the model's own, and stays the trot's until
# a self-measurement settles it (kilo model show). The diagonal's direction is the model's foot rectangle.
_FIT = view_numbers("trot")
_TRACK, _LENGTH = BodyModel().track_mm, BodyModel().length_mm


@dataclass(frozen=True)
class TrotFit:
    """The trot's rock, fitted to the floor recordings of 2026-11-15..17
    (mind/tests/stridesim.py fit): an inverted pendulum about the
    diagonal, from upright at each diagonal's start, as if it stood on the
    diagonal ``extra_ms`` longer than the plan's two-foot ticks (the
    rotation is not stopped at each touchdown), its mass ``com_x_mm`` ahead
    of the shoulders' middle, the tip clamped at ``clamp_deg`` (the falling
    side's foot reaching the ground early)."""
    com_x_mm: float = _FIT["com_x_mm"]
    extra_ms: float = _FIT["extra_ms"]
    clamp_deg: float = _FIT["clamp_deg"]
    com_h_mm: float = _FIT["com_h_mm"]

    @property
    def tau_s(self) -> float:
        return math.sqrt(self.com_h_mm / G_MM_S2)

    def two_foot_s(self, swing_ticks: int) -> float:
        """The effective time on one diagonal: the plan's two-foot ticks (the swing less its last,
        whose foot is down) plus extra_ms."""
        return ((swing_ticks - 1) * pat.TICK_MS + self.extra_ms) / 1000.0

    def k(self, swing_ticks: int) -> float:
        x = self.two_foot_s(swing_ticks) / self.tau_s
        return (math.sinh(x) - x) / (math.cosh(x) - 1.0)

    def gain_deg_per_mm(self, swing_ticks: int) -> float:
        """Tip (deg of roll) at a diagonal's end per mm the pair lands off the balanced point."""
        x = self.two_foot_s(swing_ticks) / self.tau_s
        line, roll = _TRACK / math.hypot(_TRACK, _LENGTH), _LENGTH / math.hypot(_TRACK, _LENGTH)
        return math.degrees(line * (math.cosh(x) - 1.0) * roll / self.com_h_mm)

    def tip_deg(self, swing_ticks: int, speed_mm_s: float, land_mm: float, clamp: bool = True) -> float:
        """The tip at a diagonal's end (deg; + toward the front foot in the air) for a pair landing
        ``land_mm`` ahead of its shoulders, walking at ``speed_mm_s`` (+ forward)."""
        x = self.two_foot_s(swing_ticks) / self.tau_s
        line, roll = _TRACK / math.hypot(_TRACK, _LENGTH), _LENGTH / math.hypot(_TRACK, _LENGTH)
        # the mass starts the diagonal (com_x - land) x 0.467 from its line toward the front foot in
        # the air and crosses it at 0.467 x the speed (both signed: + forward)
        hphi = line * ((self.com_x_mm - land_mm) * (math.cosh(x) - 1.0)
                       + speed_mm_s * self.tau_s * (math.sinh(x) - x))
        a = math.degrees(roll * hphi / self.com_h_mm)
        return self.clamp_deg * math.tanh(a / self.clamp_deg) if clamp else a

    def land_x(self, swing_ticks: int, speed_mm_s: float) -> float:
        """Where a pair walking at ``speed_mm_s`` (+ forward) lands for no tip (mm ahead of its shoulders)."""
        return self.com_x_mm + self.k(swing_ticks) * speed_mm_s * self.tau_s


TROT_FIT = TrotFit()


@dataclass
class TickPlan:
    tick: int                 # unwrapped body tick
    x: list                   # int mm per leg, as sent
    z: list
    support: int              # bit i: leg i on the ground
    pitch: float              # predicted at this tick (the servo lag included), degrees
    roll: float
    plan_pitch: float         # the attitude these feet give (no lag)
    plan_roll: float
    margin: float             # mm of the centre of mass inside the support polygon (these feet)
    lifting: tuple            # the legs in the air this tick


@dataclass
class Lean:
    dx: float = 0.0           # body forward of its feet, mm
    pitch: float = 0.0        # degrees, nose down +
    roll: float = 0.0         # degrees, right side up +

    def lerp(self, other: "Lean", s: float) -> "Lean":
        return Lean(self.dx + (other.dx - self.dx) * s, self.pitch + (other.pitch - self.pitch) * s,
                    self.roll + (other.roll - self.roll) * s)


@dataclass
class PlannerConfig:
    margin_mm: float = 14.0       # the centre of mass this far inside the three feet, every swing (14: a 10 mm / 50 g model error keeps every direction within 3 deg; 10 was enough only for the exact model)
    dx_max_mm: float = 18.0       # the body's fore-aft shift over its feet
    dx_speed_mm_s: float = 100.0  # ...and its fastest move
    roll_max_deg: float = 14.0
    pitch_max_deg: float = 6.0
    # the lean's cost per unit: a mm of shift, a degree of pitch, a degree of roll
    dx_scale_mm: float = 10.0
    pitch_scale_deg: float = 2.0
    roll_scale_deg: float = 8.0
    gate_deg: float = 5.0         # no lift while the body is off the plan by more than this
    hold_max: int = 25            # ...for at most this many ticks (then it lifts anyway)
    catch_deg: float = 7.0        # a swinging leg lands at once when the body tips toward it this far
    lean: bool = True             # False: the same steps without the body's move (comparisons, tests)
    shift_ms: float | None = None  # a fixed four-foot shift (None: from the speed); 0 = the old tables' walk
    trot_place: bool = True       # the trot's pairs land where the falling body will be (False: half a stride ahead)


def smooth(s: float) -> float:
    """0..1 -> 0..1 with zero slope at both ends."""
    s = min(1.0, max(0.0, s))
    return (1.0 - math.cos(math.pi * s)) / 2.0


class _LeanJob:
    """One slot's lean, solved a few units of work at a time
    (Planner._lean_steps): the slot's inputs it was started from, and the
    result once done: (shift ticks, the lean, its move's ticks)."""

    __slots__ = ("pattern", "k", "xg", "s", "stance", "shift", "lean_from", "gen", "result")

    def matches(self, pattern, k: int, xg, s, stance: float, shift: int, lean_from: Lean) -> bool:
        a, b = self.lean_from, lean_from
        return (self.pattern is pattern and self.k == k and self.shift == shift
                and abs(self.stance - stance) < 1e-6
                and all(abs(u - v) < 1e-6 for u, v in zip(self.xg, xg))
                and all(abs(u - v) < 1e-6 for u, v in zip(self.s, s))
                and abs(a.dx - b.dx) < 1e-6 and abs(a.pitch - b.pitch) < 1e-6 and abs(a.roll - b.roll) < 1e-6)


class Planner:
    def __init__(self, model: BodyModel, pattern: str = pat.DEFAULT_PATTERN, direction: str = "forward",
                 speed_mm_s: float | None = None, tick: int = 0, config: PlannerConfig | None = None,
                 start_height: float | None = None, steer: float = 0.0, trot: pat.TrotConfig | None = None,
                 fit: TrotFit = TROT_FIT):
        self.model = model
        self.cfg = config or PlannerConfig()
        self.trot = trot or pat.TROT
        self.fit = fit
        self._w = self.trot.w_min        # the trot: this slot's swing ticks (w_min while not in the trot)
        self._w_tgt = self.trot.w_min    # ... the speed's
        self._land = [0.0, 0.0]          # ... this slot's landing x per side (left, right) ...
        self._slot_s = [0.0, 0.0]        # ... for this stride (the target at the slot's start)
        self.tick = tick
        # the lift's start after the lean's move (ticks; negative: before its end)
        self.lead = int(round(pat.LEAN_LEAD_MS / pat.TICK_MS))
        # a level-out (the stop's, or out of the walk into the trot) is done a
        # servo lag before what comes next: the hand-over to STANCE, a trot's lift
        self.settle = model.settle_ticks(pat.TICK_MS)
        self.pattern = pat.pattern(pattern)
        self.direction = pat.direction(direction)
        self.steer = max(-1.0, min(1.0, float(steer)))  # forward / back: the path bends, + left (side_strides)
        self.speed_mm_s = pat.default_speed(self.pattern) if speed_mm_s is None else abs(float(speed_mm_s))
        self._ws = int(round(pat.swing_at(self.pattern, self.speed_mm_s, self.direction == "back") / pat.TICK_MS))  # the walk: this slot's swing (_latch_swing)
        self._pending_pattern: pat.Pattern | None = None
        self.xg = [0.0] * 4          # foot x in the gait frame (no lean shift)
        self.h = model.stance_height(P.GAIT_STANCE_H_256) if start_height is None else float(start_height)
        self.s_cur = [0.0, 0.0]      # stride now, left and right
        self.s_tgt = [0.0, 0.0]
        self.centre = 0.0            # mm ahead of its shoulder a swing's landing is centred (_centre_target)
        self.stance_ticks = 1.0      # a foot's time on the ground now (slewed toward stance_tgt)
        self.stance_tgt = 1.0
        self.lean = Lean()
        self.lean_from = Lean()
        self.lean_to = Lean()
        self.tr_ticks = 1            # the lean's move takes this many ticks from the slot's start
        self.k = -1                  # slot index into the groups
        self.t = 0                   # tick within the slot (stands still while the plan waits)
        self.S = 0                   # this slot's shift ticks (may grow: the lean's move, a slower speed)
        self.held = 0                # ticks the gate held this slot
        self.swing = [0.0] * 4       # per leg: x at its lift-off
        self.swing_h = [0.0] * 4     # ...and its lift at mid-swing (pat.step_height for its side's stride then)
        self.skip = False            # this slot's group does not step (stopping, already in place)
        self.levelling = False       # this slot's shift levels the body out of the crawl's lean: strides held at 0
        self.caught: dict[int, tuple[int, float]] = {}  # leg -> (tick of the catch, lift at the catch)
        self.stopping = False
        self.halting = False         # the quick stop (halt): implies stopping
        self.halt_t = 0              # ticks planned since the halt
        self._halt_slew = 0.0        # the strides' slew per tick in the quick stop
        self._halt_fast = False      # the square-up's cadence was set
        self._halt_kept: set = set()  # legs caught in the quick stop: they stay where they landed
        self.finishing = False       # stopped: the last move back to level
        self.done = False
        self.err = (0.0, 0.0)        # measured - predicted - offset (pitch, roll), the latest
        self.prev_att = (0.0, 0.0)
        self._lag = model.lag_ticks(pat.TICK_MS)
        self._alpha = model.response_alpha(pat.TICK_MS)
        self._att_hist: list[tuple[float, float]] = [(0.0, 0.0)] * (int(self._lag) + 3)
        self._resp = (0.0, 0.0)      # the predicted attitude: the plan's, delayed, through the servos' response
        self._job: _LeanJob | None = None   # this slot's lean while it is still being solved (the slot waits)
        self._next: _LeanJob | None = None  # the next slot's, solved during this slot's swing
        self.last: TickPlan | None = None
        self.stats = {"holds": 0, "catches": 0, "waits": 0, "min_margin": math.inf}
        self._apply_timing()
        self.stance_ticks = self.stance_tgt
        self._start_slot()
        if self._job is not None:  # the first slot's lean: solved here, on the caller's thread
            self._run_job(self._job, None)
            self._apply_job(self._job)

    # ---- commands ----------------------------------------------------------------------
    @property
    def is_trot(self) -> bool:
        return pat.is_trot(self.pattern)

    @property
    def swing_ticks(self) -> int:
        if self.is_trot:
            return self._w
        if self.pattern.swing_long_ms > 0:  # the walk: this slot's (_latch_swing)
            return self._ws
        return max(2, int(round(self.pattern.swing_ms / pat.TICK_MS)))

    def _latch_swing(self) -> None:
        """At a slot's start: its swing for the speed now (patterns.swing_at), kept to its end; the
        long one also for a side stride longer than Pattern.long_stride_mm (a steer's outer side)."""
        p = self.pattern
        sw = pat.swing_at(p, self.speed_mm_s, self.direction == "back")
        if p.swing_long_ms > 0 and max(abs(self.s_tgt[0]), abs(self.s_tgt[1])) > p.long_stride_mm:
            sw = p.swing_long_ms
        self._ws = max(2, int(round(sw / pat.TICK_MS)))

    @property
    def couple_ticks(self) -> int:
        """A group's second leg lifts this many ticks after its first."""
        return int(round(self.pattern.couple_ms / pat.TICK_MS))

    def _span(self) -> int:
        """A slot's ticks from the first lift to the last touchdown."""
        return self.swing_ticks + self.couple_ticks * (len(self.pattern.groups[0]) - 1)

    def _offset(self, leg: int) -> int:
        """The tick (after the shift) at which this leg of the slot's group lifts."""
        return self._group().index(leg) * self.couple_ticks

    @property
    def changing(self) -> bool:
        """A pattern change is under way (the legs go home first)."""
        return self._pending_pattern is not None

    def timing(self) -> pat.Timing:
        return pat.timing(self.pattern, self.speed_mm_s, self.direction)

    def _homing(self) -> bool:
        """Every leg goes home: a stop, or a change into or out of the crawl."""
        return self.stopping or (self._pending_pattern is not None and not self._direct_change())

    def _apply_timing(self) -> None:
        tm = self.timing()
        if self.is_trot:
            self._apply_trot_timing(tm)
            return
        shift = tm.shift_ms if self.cfg.shift_ms is None else self.cfg.shift_ms
        n = len(self.pattern.groups)
        sw = pat.swing_at(self.pattern, self.speed_mm_s, self.direction == "back")  # the walk: long at its top speeds, backward
        self.stance_tgt = (n * (shift + self.pattern.span_ms - self.pattern.swing_ms + sw) - sw) / pat.TICK_MS
        if self.stopping or (self._pending_pattern is not None and not self._direct_change()):
            self.s_tgt = [0.0, 0.0]
        elif self._pending_pattern is not None:
            # changing: no stride longer than the new pattern's at this speed
            # (it switches once the strides and the feet allow, _switch_ok)
            # (a steer's outer side too: _switch_ok)
            st = min(tm.stride_mm, pat.timing(self._pending_pattern, self.speed_mm_s, self.direction).stride_mm)
            self.s_tgt = list(pat.side_strides(self.direction, st, self.steer, cap=st))
        else:
            self.s_tgt = list(pat.side_strides(self.direction, tm.stride_mm, self.steer, cap=self.pattern.stride_mm))
        self._next = None  # the next slot's lean was solved for the old timing

    def _apply_trot_timing(self, tm: pat.Timing) -> None:
        pend = self._pending_pattern
        direct = pend is not None and self._direct_change()
        # out of the trot directly: the swing back to 8 ticks first
        self._w_tgt = self.trot.w_min if direct else pat.swing_for(tm.speed_mm_s, self.trot)
        self.stance_tgt = self._w_tgt + pat.STANCE_PLUS
        if self._homing():
            self.s_tgt = [0.0, 0.0]
        elif direct:
            st = min(tm.stride_mm, pat.timing(pend, self.speed_mm_s, self.direction).stride_mm)
            self.s_tgt = list(pat.side_strides(self.direction, st, self.steer, cap=st))
        else:
            cap = self.trot.stride_per_tick * self._w_tgt - 0.5  # steering: the outer side's stride too
            self.s_tgt = list(pat.side_strides(self.direction, tm.stride_mm, self.steer, cap=cap))
        self._next = None

    def set_speed(self, speed_mm_s: float) -> None:
        self.speed_mm_s = abs(float(speed_mm_s))
        self._apply_timing()
        if self._job is not None:  # this slot is still waiting for its lean: solve it for the new shift
            self._job = self._new_job(self.k, self.xg, self.s_cur, self.stance_ticks, self.lean_from, self.h)
        elif self.t < self.S:  # still in the shift: a new length, but never shorter than its lean needs
            self.S = max(self.t + 1, self.tr_ticks + (self.lead if self._leaning() else 0),
                         self._shift_for(self.stance_ticks))

    def set_direction(self, direction: str) -> None:
        self.direction = pat.direction(direction)
        self._apply_timing()

    def set_steer(self, steer: float) -> None:
        """Bend the path of a forward or backward walk: -1..1, + left
        (pat.side_strides). Ignored by the turns in place and the march."""
        self.steer = max(-1.0, min(1.0, float(steer)))
        self._apply_timing()

    def set_pattern(self, name: str) -> None:
        """Change the pattern. Between two fixed-cadence patterns (the walk
        and the trot, since 2026-11-16) directly: the strides shorten to what
        the new pattern takes at this speed, the body goes to its height, and
        at the first slot start where they and the feet allow (_switch_ok)
        the new pattern takes over with the group whose feet are furthest
        behind (about half a second). Into or out of a leaning pattern (the
        crawl, whose cycle is 6 to 12 times longer) through the legs' way
        home: the strides go to 0 and each leg steps under its shoulder (as
        for a stop, without the level-out), then the new pattern starts from
        its first group as a walk starts. Asking for the pattern that is
        walking cancels a change still under way; the last pattern asked for
        wins. Ignored once stopping."""
        p = pat.pattern(name)
        if self.stopping:
            return
        if p is self.pattern:
            if self._pending_pattern is not None:
                self._pending_pattern = None
                self._apply_timing()
            return
        self._pending_pattern = p
        self._apply_timing()

    def stop(self) -> None:
        self.stopping = True
        self._pending_pattern = None
        self._apply_timing()

    def halt(self) -> None:
        """The quick stop (the module docstring): feet in the air down where
        they are, the strides to 0 in HALT_TICKS, four feet down until
        HALT_HOLD_TICKS, then the legs square up and the body levels. Also
        during the planned stop (it becomes this); nothing once finishing."""
        if self.done or self.finishing or self.halting:
            return
        self.stopping = True
        self.halting = True
        self.halt_t = 0
        self._halt_kept = set()
        self._pending_pattern = None
        self._halt_slew = max(self.pattern.stride_slew_mm, max(abs(v) for v in self.s_cur) / HALT_TICKS)
        self._apply_timing()
        self._next = None
        W = self.swing_ticks
        if self._job is not None or self.skip or self.t < self.S:
            # every foot is down (the shift, or a slot waiting for its lean): the slot ends now
            self._job = None
            self.skip = True
            self.S = self.t
            return
        for leg in self._group():
            if leg in self.caught:
                continue
            j = self.t - self.S - self._offset(leg) + 1  # the next tick's place in its swing
            if j <= 1:
                self.caught[leg] = (HALT_NOT_LIFTED, 0.0)  # still down: it stays down
            elif j < W:
                self.caught[leg] = (self.t - self.S,
                                    self.swing_h[leg] * (1 - math.cos(2 * math.pi * (j - 1) / W)) / 2)

    def observe(self, err_pitch: float, err_roll: float) -> None:
        """The body's latest error: measured - predicted - the steady offset (degrees)."""
        self.err = (err_pitch, err_roll)

    # ---- the slot machine ----------------------------------------------------------------
    def _leaning(self) -> bool:
        return self.pattern.lean and self.cfg.lean

    def _group(self) -> tuple:
        return self.pattern.groups[self.k]

    def _centre_target(self) -> float:
        """Where the swings' landings are centred, mm ahead of the shoulders: the model's centre of
        mass in a centre_com pattern while it walks on, 0 once stopping or changing pattern."""
        if not self.pattern.centre_com or self.stopping or self._pending_pattern is not None:
            return 0.0
        return self.model.stance_com(self.pattern.body_h_256)[0]

    def _home(self) -> bool:
        tol = self._recentred()
        return (all(abs(self.s_cur[i]) < 0.05 for i in (0, 1))
                and all(abs(self.xg[leg]) < tol or leg in self._halt_kept for leg in LEGS))

    def _recentred(self) -> float:
        """A stopping leg this close to under its shoulder does not step again."""
        return HALT_SQUARE_MM if self.halting else RECENTRED_MM

    def _switch_ok(self, p: pat.Pattern) -> bool:
        """The new pattern can take over at this slot start: the strides are
        no longer than it walks at this speed (+ a tick's slew), every
        foot is within half of that stride (+ SWITCH_SLACK_MM) of its
        shoulder, so its first swings carry no foot further than its own do,
        and the body is at its height (a lower crawl's lean reaches further).
        Out of the trot only once its swing is 8 ticks again."""
        if self.is_trot and self._w > self.trot.w_min:
            return False
        st = pat.timing(p, self.speed_mm_s, self.direction).stride_mm
        return (all(abs(v) <= st + self.pattern.stride_slew_mm for v in self.s_cur)
                and all(abs(x) <= st / 2 + SWITCH_SLACK_MM for x in self.xg)
                and abs(self.h - self.model.stance_height(p.body_h_256)) <= HEIGHT_SLEW_MM)

    def _direct_change(self) -> bool:
        """The change under way goes straight over (no step home): both patterns have a fixed cadence
        and as many groups (the walk's four one-leg slots leave its feet
        spread over four phases, which a two-group gait cannot pick up without a foot waiting a slot
        too long: through the way home, since 2026-11-22)."""
        p = self._pending_pattern
        return (p is not None and pat.fixed_cadence(p) and pat.fixed_cadence(self.pattern)
                and len(p.groups) == len(self.pattern.groups))

    def _first_group(self) -> int:
        """The group to start a new pattern with: the one whose feet are furthest behind."""
        best, score = 0, -math.inf
        for k, g in enumerate(self.pattern.groups):
            sc = -math.inf
            for leg in g:
                s = self.s_cur[0 if LEFT[leg] else 1]
                sc = max(sc, abs(self.xg[leg]) if abs(s) < 0.05 else -math.copysign(1.0, s) * self.xg[leg])
            if sc > score + 1e-9:
                best, score = k, sc
        return best

    def _start_slot(self) -> None:
        hold = self.halting and self.halt_t < HALT_HOLD_TICKS  # the quick stop's hold: no group steps
        self._next_slot()
        self._size_trot_swing(hold)

    def _next_slot(self) -> None:
        self._job = None
        if self._pending_pattern is not None and self._direct_change() and self._switch_ok(self._pending_pattern):
            self.pattern, self._pending_pattern = self._pending_pattern, None
            self._apply_timing()
            self.k = self._first_group() - 1
        elif self._pending_pattern is not None and self._home():
            # every foot is home and still: the new pattern starts from its first group
            self.pattern, self._pending_pattern = self._pending_pattern, None
            self.k = -1
            self._apply_timing()
            self.stance_ticks = self.stance_tgt
        self._latch_swing()
        if self.halting and self.halt_t < HALT_HOLD_TICKS:
            # the quick stop's hold: four feet down, the strides going to 0, the lean as it is
            self.t = 0
            self.held = 0
            self.levelling = False
            self.caught = {}
            self.skip = True
            self._next = None
            self.lean_from = Lean(self.lean.dx, self.lean.pitch, self.lean.roll)
            self.lean_to = self.lean_from
            self.tr_ticks = 1
            self.S = HALT_HOLD_TICKS - self.halt_t
            return
        if self.halting and not self._halt_fast:
            # the square-up: the crawl steps at its fastest cadence (the strides are 0: no slide)
            self._halt_fast = True
            if self._leaning():
                self.speed_mm_s = pat.speed_range(self.pattern.name)[1]
                self._apply_timing()
                self.stance_ticks = self.stance_tgt
        self.k = (self.k + 1) % len(self.pattern.groups)
        self.t = 0
        self.held = 0
        self.levelling = False
        self.caught = {}
        g = self._group()
        tol = self._recentred()
        self.skip = (self.stopping or (self.changing and not self._direct_change())) and all(
            abs(self.s_cur[0]) < 0.05 and abs(self.s_cur[1]) < 0.05 and (abs(self.xg[leg]) < tol or leg in self._halt_kept)
            for leg in g)
        self.lean_from = Lean(self.lean.dx, self.lean.pitch, self.lean.roll)
        nxt, self._next = self._next, None
        if self.stopping and self._home():
            self.finishing = True
        if self.finishing:
            self.lean_to = Lean()
            self.tr_ticks = max(self.settle + 5, self._move_ticks(self.lean_from, self.lean_to, self.h))
            self.S = self.tr_ticks + self.settle
            return
        self.S = self._shift_for(self.stance_ticks)
        if self._leaning() and not self.skip:
            if nxt is not None and nxt.matches(self.pattern, self.k, self.xg, self.s_cur, self.stance_ticks,
                                               self.S, self.lean_from):
                job = nxt
            else:  # something changed since it was solved (a command, a catch): solved now, the slot waits
                job = self._new_job(self.k, self.xg, self.s_cur, self.stance_ticks, self.lean_from, self.h)
            if job.result is not None:
                self._apply_job(job)
            else:
                self._job = job
        else:
            self.lean_to = Lean() if not self._leaning() else self.lean_from
            tr = 0
            if self.lean_to != self.lean_from:  # out of a leaning pattern: level first, as fast as gravity lets
                tr = self._move_ticks(self.lean_from, self.lean_to, self.h)
                self.S = max(self.S, tr + self.settle)
                # ...with the feet still: the strides grow from the first lift on (with the walk's
                # 1.5 mm a tick they would slide the feet 30 mm back in this 20-tick shift)
                self.levelling = True
            self.tr_ticks = max(1, tr, self.S - (self.settle if tr else 0))

    def _size_trot_swing(self, hold: bool) -> None:
        """The trot: this slot's swing, the speed's (going home the shortest), at most a tick longer
        than the last slot's (the build-up) and at most a tick shorter, but always long enough for the
        step it carries (a square-up from a long stride, a stride still growing, steering)."""
        if not self.is_trot:
            self._w = self.trot.w_min
            return
        homing = self._homing()
        w = self.trot.w_min if homing else min(self._w_tgt, self._w + 1)  # the build-up: a tick a step
        n = len(self.pattern.groups)
        group = self.pattern.groups[(self.k + 1) % n] if hold else self._group()
        to = {}
        for leg in group:
            side = 0 if LEFT[leg] else 1
            sc = self.s_cur[side]
            if homing or not self.cfg.trot_place:
                to[leg] = sc / 2.0
            else:
                # the landing at the target stride, or (slowing: the stride shrinks 0.75 mm a tick,
                # the swing a tick a step) at the stride now, whichever is further from the foot
                lt, st = self._land_for(side), self.s_tgt[side]
                lc = sc / 2.0 - (st / 2.0 - lt) * min(1.0, max(0.0, sc / st)) if abs(st) > 1e-6 else sc / 2.0
                to[leg] = lc if abs(lc - self.xg[leg]) > abs(lt - self.xg[leg]) else lt
            d = abs(to[leg] - self.xg[leg]) + abs(sc) / max(1.0, self.stance_ticks)
            w = max(w, int(math.ceil(d / self.trot.step_mm_per_tick - 1e-9)))
        w = max(self.trot.w_min, min(self.trot.w_max + 2, max(w, self._w - 1)))
        if homing or self._pending_pattern is not None:  # not the steady trot: the joints checked
            while w < self.trot.w_max + 2 and not all(self._swing_ok(leg, to[leg], w) for leg in group):
                w += 1
        self._w = w
        for side in (0, 1):
            self._land[side] = self._land_for(side)
            self._slot_s[side] = self.s_tgt[side]

    def _swing_ok(self, leg: int, to: float, w: int) -> bool:
        """A swing of ``w`` ticks from where the foot is to ``to`` (the plan's cosine path, the
        pattern's lift, the height now) turns no joint more than joint_deg_per_tick a tick."""
        lim = math.radians(self.trot.joint_deg_per_tick)
        x0, h, lift = self.xg[leg], self.h, self.pattern.step_h_mm
        a = self.model.ik(leg, x0, h)
        for j in range(1, w + 1):
            b = self.model.ik(leg, x0 + (to - x0) * smooth(j / w),
                              h - lift * (1.0 - math.cos(2.0 * math.pi * j / w)) / 2.0)
            if abs(b[0] - a[0]) > lim or abs(b[1] - a[1]) > lim:
                return False
            a = b
        return True

    def _land_for(self, side: int) -> float:
        """Where this side's trot pair lands at the target stride (mm ahead of its shoulder)."""
        s = self.s_tgt[side]
        if self._homing() or not self.cfg.trot_place or abs(s) < 1e-6:
            return s / 2.0
        v = s / ((self._w_tgt + pat.STANCE_PLUS) * pat.TICK_MS / 1000.0)
        x = self.fit.land_x(self._w_tgt, v)
        # no further out than half a stride + 6 mm; going forward never behind the shoulder, going
        # backward never ahead of it
        return min(s / 2.0 + 6.0, max(0.0, x)) if s > 0 else max(s / 2.0 - 6.0, min(0.0, x))

    def _move_ticks(self, a: Lean, b: Lean, height: float) -> int:
        """Ticks the body needs to move from lean a to lean b on a smooth
        (cosine) profile: a side or an end that drops falls no faster than
        gravity lets it (half of BodyModel.drop_accel, a margin), and the
        fore-aft shift goes at most dx_speed_mm_s."""
        m, cfg = self.model, self.cfg
        h = height + m.foot_r_mm
        need = 0.0
        for d_deg, lever in ((b.roll - a.roll, m.track_mm / 2), (b.pitch - a.pitch, m.length_mm / 2)):
            amp = abs(math.radians(d_deg))
            if amp > 0:
                need = max(need, math.pi * math.sqrt(amp / (0.5 * m.drop_accel(lever, h))))
        need = max(need, math.pi / 2 * abs(b.dx - a.dx) / cfg.dx_speed_mm_s)
        return max(1, int(math.ceil(need / (pat.TICK_MS / 1000.0))))

    def _shift_for(self, stance: float) -> int:
        """A slot's four-foot ticks for the stance time now (ticks): the shift
        follows the stance time as it slews after a speed change, so the feet
        on the ground slide one stride per cycle and the next swing never has
        to carry a foot further than a stride in its fixed time. The trot: one tick."""
        if self.is_trot:
            return 1
        n = len(self.pattern.groups)
        w = self.swing_ticks * pat.TICK_MS
        span = self.pattern.span_ms - self.pattern.swing_ms + w
        s = int(round(((stance * pat.TICK_MS + w) / n - span) / pat.TICK_MS))
        ho = self._handover()
        if ho and self.k >= 0:  # the walk: this slot's share (Pattern.handover_ticks), around the speed's mean
            s += ho[self.k] - int(round(sum(ho) / n))
        return max(0, s)

    def _handover(self) -> tuple:
        """The walk's four-foot ticks before each slot's swing for this slot's swing
        (Pattern.handover_ticks, or handover_long_ticks with the long swing); walking backward
        evenly spaced (their mean every slot: the uneven split is made for the forward
        walk's crossing of the diagonal, and backward it rocked more in the virtual body)."""
        p = self.pattern
        ho = p.handover_long_ticks if self.swing_ticks * pat.TICK_MS > p.swing_ms else p.handover_ticks
        if ho and self.direction == "back":
            m = sum(ho) // len(ho)
            return (m,) * len(ho)
        return ho

    def _side_speed(self, leg: int) -> float:
        """mm per tick a foot on the ground slides back."""
        return self.s_cur[0 if LEFT[leg] else 1] / self.stance_ticks

    def _target_x(self, leg: int) -> float:
        side = 0 if LEFT[leg] else 1
        s = self.s_cur[side]
        if not self.is_trot:
            return s / 2.0 + self.centre
        st = self._slot_s[side]
        if not self.cfg.trot_place or abs(st) < 1e-6:
            return s / 2.0
        # the slot's landing: its shift from half a stride, scaled with the stride as it slews (a
        # stop asked mid-swing keeps it: the swing's path does not jump)
        return s / 2.0 - (st / 2.0 - self._land[side]) * min(1.0, max(0.0, s / st))

    def _slewed(self, s, stance: float, h: float):
        """One tick of the strides, the stance time and the height toward
        their targets (the stance time by at most STANCE_SLEW of itself, so
        a speed change never jumps the feet's slide either)."""
        d = self._halt_slew if self.halting else self.pattern.stride_slew_mm
        tgt = (0.0, 0.0) if self.levelling and self.t < self.S else self.s_tgt
        s = [s[i] + max(-d, min(d, tgt[i] - s[i])) for i in (0, 1)]
        stance = min(stance * (1.0 + STANCE_SLEW), max(stance / (1.0 + STANCE_SLEW), self.stance_tgt))
        h += max(-HEIGHT_SLEW_MM, min(HEIGHT_SLEW_MM, self._height_target() - h))
        return s, stance, h

    # ---- the lean -----------------------------------------------------------------------------
    def _new_job(self, k: int, xg, s, stance: float, lean_from: Lean, h_now: float) -> _LeanJob:
        job = _LeanJob()
        job.pattern, job.k, job.xg, job.s, job.stance = self.pattern, k, tuple(xg), tuple(s), stance
        job.shift, job.lean_from, job.result = self._shift_for(stance), lean_from, None
        group = self.pattern.groups[k]
        speed = [s[0 if LEFT[leg] else 1] / stance for leg in LEGS]
        tgt = [s[0 if LEFT[leg] else 1] / 2.0 for leg in LEGS]
        h = self.model.stance_height(self.pattern.body_h_256)
        job.gen = self._lean_steps(group, list(xg), speed, tgt, h, h_now, job.shift, self.swing_ticks, lean_from)
        return job

    @staticmethod
    def _run_job(job: _LeanJob, units: int | None) -> bool:
        """At most ``units`` units of the job's work (None: all of it). True once it is done."""
        if job.result is None:
            try:
                if units is None:
                    while True:
                        next(job.gen)
                for _ in range(units):
                    next(job.gen)
            except StopIteration as e:
                job.result = e.value
        return job.result is not None

    def _apply_job(self, job: _LeanJob) -> None:
        self.S, lean_to, self.tr_ticks = job.result
        self.lean_to = Lean(lean_to.dx, lean_to.pitch, lean_to.roll)
        self._job = None

    def _lean_steps(self, group: tuple, xg, speed, tgt, h: float, h_now: float, shift: int, swing: int,
                    lean_from: Lean):
        """The slot's shift, lean and move ticks (a generator: one unit of
        work per step, the result as its return value). The lift starts
        ``lead`` ticks after the move's end (negative: before it): when the
        move needs longer than the speed's shift gives, the shift grows and
        the lean is solved again for the later lift-off."""
        S = shift
        lean_to = yield from self._solve_steps(group, xg, speed, tgt, h, S, swing)
        tr = self._move_ticks(lean_from, lean_to, h_now)
        if tr + self.lead > S:
            S = tr + self.lead
            lean_to = yield from self._solve_steps(group, xg, speed, tgt, h, S, swing)
            tr = self._move_ticks(lean_from, lean_to, h_now)
        tr_ticks = max(1, S - self.lead, tr)
        return max(S, tr_ticks + self.lead), lean_to, tr_ticks

    def _solve_steps(self, group: tuple, xg, speed, tgt, h: float, shift: int, swing: int):
        """The smallest body move that keeps the centre of mass margin_mm
        inside the feet that stay down, at lift-off (``shift`` ticks on) and
        at touchdown (``swing`` later), for feet now at ``xg`` (gait frame)
        sliding back ``speed`` mm per tick, the group landing at ``tgt``. A
        generator: one margin evaluation or one part of the quadratic
        program per step."""
        cfg = self.cfg
        support = tuple(leg for leg in LEGS if leg not in group)
        configs = []
        for ahead in (shift, shift + swing):
            xs = [xg[leg] - speed[leg] * ahead for leg in LEGS]
            for leg in group:
                xs[leg] = xg[leg] - speed[leg] * shift if ahead == shift else tgt[leg]
            configs.append(xs)
        scale = (cfg.dx_scale_mm, cfg.pitch_scale_deg, cfg.roll_scale_deg)
        lim = (cfg.dx_max_mm, cfg.pitch_max_deg, cfg.roll_max_deg)
        u = [0.0, 0.0, 0.0]
        for _ in range(2):
            rows = []  # (gradient in u, margin at u) per edge per config
            for xs in configs:
                base = self._edge_margins(xs, h, support, u)
                yield
                grads = []
                for j in range(3):
                    du = list(u)
                    du[j] += 0.5
                    m = self._edge_margins(xs, h, support, du)
                    yield
                    grads.append([(m[e] - base[e]) / 0.5 for e in range(len(base))])
                for e in range(len(base)):
                    rows.append(((grads[0][e], grads[1][e], grads[2][e]), base[e]))
            # minimise sum((d_j / scale_j)^2) s.t. m_e + g_e . d >= margin, d = the step from u
            want = cfg.margin_mm
            step = None
            while step is None and want >= -1e-9:
                step = yield from _qp_steps(rows, want, scale)
                if step is None:
                    want = want - 2.0 if want > 2.0 else (0.0 if want > 0 else -1.0)
            if step is None:
                break
            u = [min(lim[j], max(-lim[j], u[j] + step[j])) for j in range(3)]
        return Lean(u[0], u[1], u[2])

    def _edge_margins(self, xs_gait, h, support: tuple, u) -> list:
        """Signed distance (mm, + inside) of the centre of mass from each edge
        of the support polygon, for the gait-frame feet shifted by lean u
        (three support feet: their triangle, counter-clockwise)."""
        xs = [x - u[0] for x in xs_gait]
        zs = leg_lengths(self.model, xs, h, u[1], u[2])
        com, feet = world_points(self.model, xs, zs, u[1], u[2])
        if len(support) == 3:
            a, b, c = (feet[leg] for leg in support)
            if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) < 0:
                b, c = c, b
            poly = (a, b, c)
        else:
            poly = hull([(feet[leg][0], feet[leg][1]) for leg in support])
        out = []
        n = len(poly)
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]
            ex, ey = b[0] - a[0], b[1] - a[1]
            L = math.hypot(ex, ey) or 1.0
            out.append((ex * (com[1] - a[1]) - ey * (com[0] - a[0])) / L)
        return out

    def _height_target(self) -> float:
        if self.finishing:
            return self.model.stance_height(P.GAIT_STANCE_H_256)
        if self._direct_change():  # changing: to the new pattern's height first (_switch_ok)
            return self.model.stance_height(self._pending_pattern.body_h_256)
        return self.model.stance_height(self.pattern.body_h_256)

    def _predict_slot_end(self):
        """The feet, strides, stance time and height at the end of this
        slot's swing if nothing changes (the same arithmetic as next())."""
        W = self.swing_ticks
        xg = list(self.xg)
        x0 = list(self.swing)
        s, st, h = self.s_cur, self.stance_ticks, self.h
        g = self._group()
        for t in range(self.t, self.S + self._span()):
            s, st, h = self._slewed(s, st, h)
            for leg in LEGS:
                side = 0 if LEFT[leg] else 1
                j = t - self.S - (self._offset(leg) if leg in g else 0) + 1
                if leg in g and 1 <= j <= W:
                    if j == 1:
                        x0[leg] = xg[leg]
                    xg[leg] = x0[leg] + (s[side] / 2.0 - x0[leg]) * smooth(j / W)
                else:
                    xg[leg] -= s[side] / st
        return xg, s, st, h

    # ---- one tick ---------------------------------------------------------------------------------
    def next(self) -> TickPlan:
        """Plan the next tick (``self.tick``) and advance."""
        cfg = self.cfg
        W = self.swing_ticks
        if self.t >= self.S + self._span() or (self.skip and self.t >= self.S) or (self.finishing and self.t >= self.S):
            if self.finishing:
                self.done = True
            else:
                self._start_slot()
        # The plan waits (every foot where it is, the lean held, nothing
        # slewed, the slot's clock stopped) while this slot's lean is still
        # being solved, and while the gate holds the lift: a leg lifts only
        # while the body follows the plan.
        wait = False
        if self._job is not None:
            if self._run_job(self._job, LEAN_UNITS_PER_TICK):
                self._apply_job(self._job)
            else:
                wait = True
                self.stats["waits"] += 1
        if not wait and self.t == self.S and not self.skip and not self.finishing and not self.halting \
                and self.held < cfg.hold_max and max(abs(self.err[0]), abs(self.err[1])) > cfg.gate_deg:
            wait = True
            self.held += 1
            self.stats["holds"] += 1

        g = self._group()
        lifting: tuple = ()
        lift = [0.0, 0.0, 0.0, 0.0]
        swinging = False
        if not wait:
            self.s_cur, self.stance_ticks, self.h = self._slewed(self.s_cur, self.stance_ticks, self.h)
            self.centre += max(-CENTRE_SLEW_MM, min(CENTRE_SLEW_MM, self._centre_target() - self.centre))
            if self.t < self.S or self.skip or self.finishing:
                for leg in LEGS:
                    self.xg[leg] -= self._side_speed(leg)
            else:
                swinging = True
                # each leg of the group swings W ticks from its own lift (the
                # second one couple_ticks after the first); before its lift and
                # after its touchdown it slides with the ground
                js = {leg: self.t - self.S - self._offset(leg) + 1 for leg in g}  # 1..W while it swings
                for leg in LEGS:
                    if leg in g and leg not in self.caught and 1 <= js[leg] <= W:
                        continue
                    if leg in self.caught and self.t - self.S - self.caught[leg][0] < 3:
                        continue  # a caught foot lands where it is (three ticks), then slides
                    self.xg[leg] -= self._side_speed(leg)
                for leg in g:
                    j = js[leg]
                    if leg in self.caught or not 1 <= j < W:
                        continue
                    if self._tip_toward(leg) > cfg.catch_deg:
                        prev_lift = self.swing_h[leg] * (1 - math.cos(2 * math.pi * (j - 1) / W)) / 2
                        self.caught[leg] = (self.t - self.S, prev_lift)
                        self.stats["catches"] += 1
                        if self.halting:
                            self._halt_kept.add(leg)
                        self._next = None  # it lands elsewhere: the next slot's lean is solved when it starts
                for leg in g:
                    j = js[leg]
                    if leg in self.caught:
                        jc, lc = self.caught[leg]
                        lift[leg] = lc * (1 - smooth((self.t - self.S - jc + 1) / 3.0))
                    elif 1 <= j <= W:
                        if j == 1:
                            self.swing[leg] = self.xg[leg]  # where the foot leaves the ground
                            # the lift for the longer of the stride now and the one it grows to (a
                            # swing while the stride grows lands further than its lift-off stride)
                            side = 0 if LEFT[leg] else 1
                            self.swing_h[leg] = pat.step_height(
                                self.pattern, max(abs(self.s_cur[side]), abs(self.s_tgt[side])))
                        x0 = self.swing[leg]
                        self.xg[leg] = x0 + (self._target_x(leg) - x0) * smooth(j / W)
                        lift[leg] = self.swing_h[leg] * (1 - math.cos(2 * math.pi * j / W)) / 2
                if self.caught and all(lift[leg] < 1e-6 and (leg in self.caught or js[leg] >= 1) for leg in g):
                    self.t = self.S + self._span()  # every caught foot is down and no other is still to lift
                    for leg in g:
                        lift[leg] = 0.0
                lifting = tuple(leg for leg in g if lift[leg] > 1e-6)
            # the lean: moves from the slot's start, complete tr_ticks later
            if self.t < self.tr_ticks:
                self.lean = self.lean_from.lerp(self.lean_to, smooth((self.t + 1) / self.tr_ticks))
            else:
                self.lean = Lean(self.lean_to.dx, self.lean_to.pitch, self.lean_to.roll)

        lean = self.lean
        sw = self._sway()
        if sw is not None:
            lean = Lean(lean.dx + sw.dx, lean.pitch + sw.pitch, lean.roll + sw.roll)
        xs = [self.xg[leg] - lean.dx for leg in LEGS]
        zs = leg_lengths(self.model, xs, self.h, lean.pitch, lean.roll)
        xi = [int(round(min(X_RANGE[1], max(X_RANGE[0], xs[leg])))) for leg in LEGS]
        zi = [int(round(min(Z_RANGE[1], max(Z_RANGE[0], zs[leg] - lift[leg])))) for leg in LEGS]
        support = 0
        for leg in LEGS:
            if lift[leg] <= 1e-6:
                support |= 1 << leg
        att = fit_attitude(self.model, xi, zi, support, self.prev_att)
        self.prev_att = att
        self._att_hist.append(att)
        del self._att_hist[0]
        lp, lr = self._lagged()
        a = self._alpha
        self._resp = (self._resp[0] + (lp - self._resp[0]) * a, self._resp[1] + (lr - self._resp[1]) * a)
        pp, pr = self._resp
        pp = min(ATT_MAX_DEG, max(-ATT_MAX_DEG, pp))
        pr = min(ATT_MAX_DEG, max(-ATT_MAX_DEG, pr))
        m = stability(self.model, xi, zi, support, att[0], att[1])
        if len(lifting) == 1:
            self.stats["min_margin"] = min(self.stats["min_margin"], m)
        out = TickPlan(self.tick, xi, zi, support, pp, pr, att[0], att[1], m, lifting)
        self.last = out
        self.tick += 1
        if not wait:
            self.t += 1
        if self.halting:
            self.halt_t += 1
        # The next slot's lean, solved a little every tick of this swing (so
        # no tick carries a whole solve), from where the feet will be when
        # that slot starts; used if nothing changes before then.
        if swinging and not self.caught and self._leaning() and not self.changing and not self.finishing:
            if self._next is None:
                xg, s, st, h = self._predict_slot_end()
                self._next = self._new_job((self.k + 1) % len(self.pattern.groups), xg, s, st,
                                           Lean(self.lean_to.dx, self.lean_to.pitch, self.lean_to.roll), h)
            self._run_job(self._next, LEAN_UNITS_PER_TICK)
        return out

    def _sway(self) -> Lean | None:
        """The walk's sway this tick (patterns.Pattern.sway_*): a roll once a cycle and a fore-aft
        shift twice a cycle, in step with the slots, scaled with the stride walked now against the
        speed's (walking forward only: none in a turn in place or backward, nothing once the strides
        are 0: standing, stopped), and for the fore-aft shift also against the default speed's
        stride (slower, less: the body never moves back over its feet)."""
        p = self.pattern
        if not p.handover_ticks or self.is_trot or (p.sway_roll_deg == 0 and p.sway_dx_mm == 0):
            return None
        s = (self.s_cur[0] + self.s_cur[1]) / 2.0
        if s <= 0.0:  # forward only (backward the first version's walk: fading out as the strides pass 0)
            return None
        tm = self.timing()
        f = min(1.0, s / max(1e-6, tm.stride_mm))
        fx = f * min(1.0, s / pat.timing(p).stride_mm)
        W = self.swing_ticks
        lens = [h + W for h in self._handover()]
        phi = sum(lens[:self.k]) + self.t
        cyc = float(sum(lens))
        roll = p.sway_roll_deg * math.cos(2.0 * math.pi * (phi - p.sway_roll_at) / cyc)
        dx = p.sway_dx_mm * math.cos(4.0 * math.pi * (phi - p.sway_dx_at) / cyc)
        return Lean(dx * fx, 0.0, roll * f)

    def _lagged(self) -> tuple[float, float]:
        """The planned attitude the servos' dead time ago (linear between ticks)."""
        hist = self._att_hist
        n = len(hist) - 1            # hist[n] is this tick
        lag = self._lag
        i = int(math.floor(lag))
        f = lag - i
        a = hist[max(0, n - i)]
        b = hist[max(0, n - i - 1)]
        return a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f

    def _tip_toward(self, leg: int) -> float:
        """How far (degrees) the body is tipping toward this leg beyond the plan."""
        ep, er = self.err
        side = 1.0 if LEFT[leg] else -1.0    # a left leg: the left side going down is +roll
        end = 1.0 if FRONT[leg] else -1.0    # a front leg: nose going down is +pitch
        return side * er + end * ep


def _qp_steps(rows, want: float, scale):
    """min sum((d_j / scale_j)^2) subject to m_e + g_e . d >= want for every
    row (g_e, m_e): enumerate active sets of up to three constraints (the
    problem is three-dimensional, the rows few). A generator (one step per
    active-set size); returns d, or None if infeasible."""
    s0, s1, s2 = scale
    # whiten: d = S w, so the cost is |w|^2 and a row's gradient becomes g S
    W = [(g[0] * s0, g[1] * s1, g[2] * s2, want - m) for g, m in rows]  # a . w >= b

    def feasible(w0, w1, w2) -> bool:
        for a0, a1, a2, b in W:
            if a0 * w0 + a1 * w1 + a2 * w2 < b - 1e-6:
                return False
        return True

    if feasible(0.0, 0.0, 0.0):
        return [0.0, 0.0, 0.0]
    best = None
    best_n = math.inf
    n = len(W)
    for size in (1, 2, 3):
        yield
        for idx in itertools.combinations(range(n), size):
            w = _min_norm_eq([W[i] for i in idx])
            if w is None:
                continue
            nn = w[0] * w[0] + w[1] * w[1] + w[2] * w[2]
            if nn < best_n and feasible(w[0], w[1], w[2]):
                best, best_n = w, nn
    if best is None:
        return None
    return [best[0] * s0, best[1] * s1, best[2] * s2]


def _min_norm_eq(A):
    """The least-norm w with a . w = b for each row (a0, a1, a2, b) of A (1..3 rows)."""
    k = len(A)
    if k == 1:
        a0, a1, a2, b = A[0]
        g = a0 * a0 + a1 * a1 + a2 * a2
        if g < 1e-12:
            return None
        lam = b / g
        return (a0 * lam, a1 * lam, a2 * lam)
    if k == 2:
        (p0, p1, p2, pb), (q0, q1, q2, qb) = A
        g00 = p0 * p0 + p1 * p1 + p2 * p2
        g01 = p0 * q0 + p1 * q1 + p2 * q2
        g11 = q0 * q0 + q1 * q1 + q2 * q2
        det = g00 * g11 - g01 * g01
        if abs(det) < 1e-9:
            return None
        l0 = (pb * g11 - g01 * qb) / det
        l1 = (g00 * qb - g01 * pb) / det
        return (p0 * l0 + q0 * l1, p1 * l0 + q1 * l1, p2 * l0 + q2 * l1)
    G = [[A[i][0] * A[j][0] + A[i][1] * A[j][1] + A[i][2] * A[j][2] for j in range(3)] for i in range(3)]
    lam = _solve3(G, (A[0][3], A[1][3], A[2][3]))
    if lam is None:
        return None
    return tuple(A[0][c] * lam[0] + A[1][c] * lam[1] + A[2][c] * lam[2] for c in range(3))
