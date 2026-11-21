"""The long trot (docs/walking-and-balance.md §15, 2026-11-21): the Python
reference of the C++ mind's long trot (kernel/mind/planner/plan.cpp,
longtrot.cpp; the remote's A1).

The gaits in kilo.planner.patterns are untouched: this is a Planner
subclass with its own pattern object (``LONG_TROT``, "longtrot", not in
``patterns.PATTERNS``); in the walk, the trot and the crawl it plans exactly
what Planner plans. The golden vectors (kernel/host/tests/
gen_planner_vectors.py) are made from it for the long trot's scenarios.

The trot's stride is limited by the joints: the swing must carry the foot
a stride and a bit in its fixed 160 ms, and at 44 mm (with the lift down to
6 mm) that already asks 476 deg/s of the 500 the body allows. A longer
stride needs a longer swing, and a longer swing leaves the body longer on
two diagonal feet, falling about their line as an inverted pendulum. What
the long trot does about both:

- **The swing grows with the speed** (``swing_for``): 8 ticks up to
  ``v_from`` mm/s (today's trot), then a tick more for every ``v_step``
  mm/s, at most ``w_max`` (11 ticks, 220 ms), and at most a tick longer
  than the step before: the stride is built up over a few steps. The
  four-foot phase stays one tick, so the stride is the speed x (swing +
  1.5 ticks): it grows faster than the speed (27.4 mm at 144 mm/s as
  today, 51.7 mm at the top, 207 mm/s). The cadence slows instead of the
  joints speeding up; the top speed is about today's.
- **Each pair lands where the falling body will be** (``TrotFit.land_x``):
  ahead of its shoulder by x_c + k v tau (x_c the centre of mass's
  effective position ahead of the shoulders' middle, tau the pendulum's
  time constant sqrt(h / g), v the side's speed, k = (sinh X - X) /
  (cosh X - 1) with X the effective time on the diagonal over tau). The
  body starts each diagonal with its mass on one side of the line and
  crosses it at the speed's sideways share (0.467 of it); landing there,
  the tip the mass's offset makes and the tip its motion makes cancel by
  the diagonal's end. Half a stride ahead (today's trot) is that point only
  near 125..145 mm/s; at its top the long trot lands 19 mm ahead instead of
  26. x_c and the effective time are fitted to the floor recordings of the
  trot (``TROT_FIT``), not measured.
- **Each swing is long enough for the step it carries**: a square-up from
  a long stride, a stride still growing or shrinking, a steer's outer side;
  and a swing that is not the steady long trot's (a square-up, the way home
  into the crawl, the swing shortening out of the long trot) long enough
  for no joint to turn more than ``joint_deg_per_tick`` along its path (a
  foot 45 mm behind its shoulder after a quick stop is near the leg's
  reach, where the knee turns fast).

Stopping: the planned stop and the quick stop (halt) are the Planner's; the
feet step home to 0, each step's swing as long as its distance and its
joints need, and a stop asked mid-swing keeps that swing's landing.
Directions, steering, speed changes, stop, halt and pattern changes
(LongTrotPlanner's docstring) are supported.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from kilo.planner import patterns as pat
from kilo.planner.model import LB, LEFT, LF, RB, RF, BodyModel, G_MM_S2
from kilo.planner.plan import HALT_HOLD_TICKS, HEIGHT_SLEW_MM, SWITCH_SLACK_MM, Planner, PlannerConfig, smooth


@dataclass(frozen=True)
class TrotFit:
    """The trot's rock, fitted to the floor recordings of 2026-11-15..17
    (mind/tests/stridesim.py fit): an inverted pendulum about the
    diagonal, from upright at each diagonal's start, as if it stood on the
    diagonal ``extra_ms`` longer than the plan's two-foot ticks (the
    rotation is not stopped at each touchdown), its mass ``com_x_mm`` ahead
    of the shoulders' middle, the tip clamped at ``clamp_deg`` (the falling
    side's foot reaching the ground early)."""
    com_x_mm: float = 3.5
    extra_ms: float = 120.0
    clamp_deg: float = 2.3
    com_h_mm: float = 79.0

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
        line, roll = 65 / math.hypot(65, 123), 123 / math.hypot(65, 123)
        return math.degrees(line * (math.cosh(x) - 1.0) * roll / self.com_h_mm)

    def tip_deg(self, swing_ticks: int, speed_mm_s: float, land_mm: float, clamp: bool = True) -> float:
        """The tip at a diagonal's end (deg; + toward the front foot in the air) for a pair landing
        ``land_mm`` ahead of its shoulders, walking at ``speed_mm_s`` (+ forward)."""
        x = self.two_foot_s(swing_ticks) / self.tau_s
        line, roll = 65 / math.hypot(65, 123), 123 / math.hypot(65, 123)
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


STANCE_PLUS = 1.5  # a foot's ground time is the swing + 1.5 ticks (the trot's: 9.5 ticks for its 8-tick swing)


@dataclass(frozen=True)
class LongTrotConfig:
    v_from: float = 150.0          # up to this speed the swing is today's 8 ticks
    v_step: float = 18.0           # ... then a tick more for every v_step mm/s ...
    w_min: int = 8
    w_max: int = 11                # ... up to this (220 ms)
    top_mm_s: float = 207.0        # the fastest forward: 51.7 mm at 11 ticks
    top_back_mm_s: float = 180.0   # ... and backward (its feet land further forward, where the knee works harder)
    step_mm_per_tick: float = 5.4  # a swing that carries a foot d mm takes at least d / this ticks
    stride_per_tick: float = 4.75  # ... and a side's stride is at most this x the swing's ticks less 0.5 mm
    # A swing that is not the steady long trot's (a stop's or the quick stop's square-up from a long
    # stride, the way home into the crawl, the swing shortening out of the long trot) is also as long
    # as its path needs for no joint to turn more than this a tick (the body's slew is 10, 500 deg/s):
    # a foot that starts 45 mm behind its shoulder, near the leg's reach, turns the knee fast.
    joint_deg_per_tick: float = 8.0
    place: bool = True             # land where the falling body will be (False: half a stride ahead, as the trot)
    fit: TrotFit = TROT_FIT


# the lift stays 8 mm (the trot tapers it to 6 at 44 mm: its swing is short); the stride cap is
# per swing length (LongTrotConfig.stride_per_tick)
LONG_TROT = pat.Pattern("longtrot", ((LF, RB), (RF, LB)), swing_ms=160.0, stride_mm=52.0, step_h_mm=8.0,
                        body_h_256=205, period_ms=360.0, shift_min_ms=20.0, stance_max_ms=250.0, lean=False,
                        default_mm_s=144.0, stride_slew_mm=0.75)


def swing_for(speed_mm_s: float, cfg: LongTrotConfig = LongTrotConfig()) -> int:
    """The swing's ticks at a speed (the build-up)."""
    v = abs(speed_mm_s)
    if v <= cfg.v_from:
        return cfg.w_min
    return min(cfg.w_max, cfg.w_min + int(math.ceil((v - cfg.v_from) / cfg.v_step - 1e-9)))


def timing(speed_mm_s: float | None, cfg: LongTrotConfig = LongTrotConfig(), back: bool = False
           ) -> tuple[int, float, float]:
    """(swing ticks, stride mm, speed mm/s) for a speed (default: 144), clamped to today's trot's
    slowest and to top_mm_s (top_back_mm_s backward)."""
    v = LONG_TROT.default_mm_s if speed_mm_s is None else abs(float(speed_mm_s))
    lo = pat.STRIDE_MIN_MM / ((cfg.w_min + STANCE_PLUS) * pat.TICK_MS / 1000.0)
    v = min(cfg.top_back_mm_s if back else cfg.top_mm_s, max(lo, v))
    w = swing_for(v, cfg)
    stance = (w + STANCE_PLUS) * pat.TICK_MS / 1000.0
    s = max(pat.STRIDE_MIN_MM, v * stance)
    return w, s, s / stance


def pattern_timing(p: pat.Pattern, speed_mm_s: float, direction: str = "forward",
                   cfg: LongTrotConfig = LongTrotConfig()) -> pat.Timing:
    """patterns.timing() for the three gaits; for LONG_TROT its own (``timing``: the stride over
    the swing + STANCE_PLUS ticks on the ground, the swing from ``swing_for``)."""
    if p is not LONG_TROT:
        return pat.timing(p, speed_mm_s)
    w, s, v = timing(speed_mm_s, cfg, back=pat.direction(direction) == "back")
    st = (w + STANCE_PLUS) * pat.TICK_MS
    return pat.Timing(s, pat.TICK_MS, st, st + w * pat.TICK_MS, v)


def _pattern(name) -> pat.Pattern:
    return LONG_TROT if (name is LONG_TROT or name == "longtrot") else pat.pattern(name)


# The remote's long-trot stages (kernel/mind/remote/remote.cpp kLongTrotStages, key A1 and then B7 /
# B5; 2026-11-21): forward only, each a swing length of the build-up (9, 10, 11 ticks).
STAGES = (160.0, 180.0, 200.0)
STEER_MAX = 0.25  # the remote steers the long trot at its first steer level only


class LongTrotPlanner(Planner):
    """The planner with the long trot (the module docstring) as a fourth pattern, ``LONG_TROT``
    ("longtrot"). In the walk, the trot and the crawl it is exactly Planner (tick for tick); in
    the long trot its swing grows with the speed and its pairs land where the falling body will
    be. The C++ mind's Planner (kernel/mind/planner/plan.cpp) is this class: the golden vectors
    (kernel/host/tests/gen_planner_vectors.py) are made from it.

    Pattern changes into and out of the long trot: from and to the walk and the trot directly
    (both have a fixed cadence), as between those two: the strides go to what both walk at this
    speed, and the new pattern takes over at a slot start where the strides and the feet allow
    (Planner._switch_ok) and, out of the long trot, once its swing is back to today's 8 ticks
    (a tick shorter each step: the build-up undone). Into the long trot its swing then grows a
    tick a step from 8. To and from the crawl through the legs' way home, as for every change
    into or out of the crawl."""

    def __init__(self, model: BodyModel, direction: str = "forward", speed_mm_s: float | None = None,
                 tick: int = 0, config: PlannerConfig | None = None, long: LongTrotConfig | None = None,
                 start_height: float | None = None, steer: float = 0.0, pattern: str = "longtrot"):
        self.long = long or LongTrotConfig()
        self._w = self.long.w_min        # this slot's swing ticks (w_min while not in the long trot)
        self._w_tgt = self.long.w_min    # the speed's
        self._land = [0.0, 0.0]          # this slot's landing x per side (left, right) ...
        self._slot_s = [0.0, 0.0]        # ... for this stride (the target at the slot's start)
        super().__init__(model, _pattern(pattern), direction, speed_mm_s, tick, config, start_height, steer)

    @property
    def is_long(self) -> bool:
        return self.pattern is LONG_TROT

    # ---- timing ------------------------------------------------------------------------
    @property
    def swing_ticks(self) -> int:
        return self._w if self.is_long else super().swing_ticks

    def timing(self) -> pat.Timing:
        if not self.is_long:
            return super().timing()
        return pattern_timing(LONG_TROT, self.speed_mm_s, self.direction, self.long)

    def _homing(self) -> bool:
        """Every leg goes home: a stop, or a change into or out of the crawl."""
        return self.stopping or (self._pending_pattern is not None and not self._direct_change())

    def _apply_timing(self) -> None:
        pend = self._pending_pattern
        if not self.is_long:
            super()._apply_timing()
            if pend is LONG_TROT and self._direct_change() and not self.stopping:
                # into the long trot: no stride longer than both walk at this speed (as Planner's
                # change, with the long trot's own timing)
                st = min(self.timing().stride_mm, pattern_timing(LONG_TROT, self.speed_mm_s, self.direction,
                                                                 self.long).stride_mm)
                self.s_tgt = list(pat.side_strides(self.direction, st, self.steer, cap=st))
            return
        tm = self.timing()
        direct = pend is not None and self._direct_change()
        # out of the long trot directly: the swing back to today's 8 ticks first
        self._w_tgt = self.long.w_min if direct else swing_for(tm.speed_mm_s, self.long)
        self.stance_tgt = self._w_tgt + STANCE_PLUS
        if self._homing():
            self.s_tgt = [0.0, 0.0]
        elif direct:
            st = min(tm.stride_mm, pattern_timing(pend, self.speed_mm_s, self.direction, self.long).stride_mm)
            self.s_tgt = list(pat.side_strides(self.direction, st, self.steer, cap=st))
        else:
            cap = self.long.stride_per_tick * self._w_tgt - 0.5  # steering: the outer side's stride too
            self.s_tgt = list(pat.side_strides(self.direction, tm.stride_mm, self.steer, cap=cap))
        self._next = None

    def _shift_for(self, stance: float) -> int:
        return 1 if self.is_long else super()._shift_for(stance)

    def set_pattern(self, name: str) -> None:
        super().set_pattern(_pattern(name))

    def _switch_ok(self, p: pat.Pattern) -> bool:
        if self.is_long and self._w > self.long.w_min:
            return False  # out of the long trot once its swing is today's again
        st = pattern_timing(p, self.speed_mm_s, self.direction, self.long).stride_mm
        return (all(abs(v) <= st + self.pattern.stride_slew_mm for v in self.s_cur)
                and all(abs(x) <= st / 2 + SWITCH_SLACK_MM for x in self.xg)
                and abs(self.h - self.model.stance_height(p.body_h_256)) <= HEIGHT_SLEW_MM)

    def _start_slot(self) -> None:
        hold = self.halting and self.halt_t < HALT_HOLD_TICKS  # the quick stop's hold: no group steps
        super()._start_slot()
        if not self.is_long:
            self._w = self.long.w_min
            return
        # this slot's swing: the speed's (going home the shortest), at most a tick longer than the
        # last slot's (the build-up) and at most a tick shorter, but always long enough for the step it
        # carries (a square-up from a long stride, a stride still growing, steering)
        homing = self._homing()
        w = self.long.w_min if homing else min(self._w_tgt, self._w + 1)  # the build-up: a tick a step
        n = len(self.pattern.groups)
        group = self.pattern.groups[(self.k + 1) % n] if hold else self._group()
        to = {}
        for leg in group:
            side = 0 if LEFT[leg] else 1
            sc = self.s_cur[side]
            if homing or not self.long.place:
                to[leg] = sc / 2.0
            else:
                # the landing at the target stride, or (slowing: the stride shrinks 0.75 mm a tick,
                # the swing a tick a step) at the stride now, whichever is further from the foot
                lt, st = self._land_for(side), self.s_tgt[side]
                lc = sc / 2.0 - (st / 2.0 - lt) * min(1.0, max(0.0, sc / st)) if abs(st) > 1e-6 else sc / 2.0
                to[leg] = lc if abs(lc - self.xg[leg]) > abs(lt - self.xg[leg]) else lt
            d = abs(to[leg] - self.xg[leg]) + abs(sc) / max(1.0, self.stance_ticks)
            w = max(w, int(math.ceil(d / self.long.step_mm_per_tick - 1e-9)))
        w = max(self.long.w_min, min(self.long.w_max + 2, max(w, self._w - 1)))
        if homing or self._pending_pattern is not None:  # not the steady long trot: the joints checked
            while w < self.long.w_max + 2 and not all(self._swing_ok(leg, to[leg], w) for leg in group):
                w += 1
        self._w = w
        for side in (0, 1):
            self._land[side] = self._land_for(side)
            self._slot_s[side] = self.s_tgt[side]

    def _swing_ok(self, leg: int, to: float, w: int) -> bool:
        """A swing of ``w`` ticks from where the foot is to ``to`` (the plan's cosine path, the
        pattern's lift, the height now) turns no joint more than joint_deg_per_tick a tick."""
        lim = math.radians(self.long.joint_deg_per_tick)
        x0, h, lift = self.xg[leg], self.h, self.pattern.step_h_mm
        a = self.model.ik(leg, x0, h)
        for j in range(1, w + 1):
            b = self.model.ik(leg, x0 + (to - x0) * smooth(j / w),
                              h - lift * (1.0 - math.cos(2.0 * math.pi * j / w)) / 2.0)
            if abs(b[0] - a[0]) > lim or abs(b[1] - a[1]) > lim:
                return False
            a = b
        return True

    # ---- placement ------------------------------------------------------------------------
    def _land_for(self, side: int) -> float:
        """Where this side's pair lands at the target stride (mm ahead of its shoulder)."""
        s = self.s_tgt[side]
        if self._homing() or not self.long.place or abs(s) < 1e-6:
            return s / 2.0
        v = s / ((self._w_tgt + STANCE_PLUS) * pat.TICK_MS / 1000.0)
        x = self.long.fit.land_x(self._w_tgt, v)
        # no further out than the trot's half stride + 6 mm; going forward never behind the
        # shoulder, going backward never ahead of it
        return min(s / 2.0 + 6.0, max(0.0, x)) if s > 0 else max(s / 2.0 - 6.0, min(0.0, x))

    def _target_x(self, leg: int) -> float:
        if not self.is_long:
            return super()._target_x(leg)
        side = 0 if LEFT[leg] else 1
        s = self.s_cur[side]
        st = self._slot_s[side]
        if not self.long.place or abs(st) < 1e-6:
            return s / 2.0
        # the slot's landing: its shift from half a stride, scaled with the stride as it slews (a
        # stop asked mid-swing keeps it: the swing's path does not jump)
        return s / 2.0 - (st / 2.0 - self._land[side]) * min(1.0, max(0.0, s / st))
