"""Gait patterns and the speed -> timing mapping (docs/walking-and-balance.md
§3, §10; mind/README.md "Walking").

A pattern is a sequence of swing groups: the legs that lift together, in
order. Each group gets a slot of the cycle: a four-foot phase (the "shift":
every foot down, the body moving over the feet that will stay down), then
the swing. So

    period = groups x (shift + swing),   stance = period - swing.

The swing time is fixed per pattern (short enough for the body's tip time
constant on two feet, long enough for the servos: every swing's fastest
joint stays under about 425 deg/s, 15 % above what the floor runs of
2026-11 asked for, under the body's 500 deg/s safety slew). Speed comes
from the stride and the stance time: stance = stride / speed; a slower
speed lengthens the shift, never the swing. Slower than the longest stance
allows, the stride shrinks (down to STRIDE_MIN_MM) instead; faster than
the default stride at the shortest shift allows, the speed is capped.

The shortest shift: the trot's two ticks of four-foot phase (40 ms); the
one-leg-at-a-time patterns (walk, crawl) must move the body over the next
three feet before each lift (kilo.planner.plan), a move that gravity paces
(a side drops no faster than it can fall), so their shift is that move's
typical length, LEAN_MOVE_MS, plus LEAN_LEAD_MS (negative: the lift starts
60 ms before the move's end, in its cosine tail). A move that needs longer
makes its slot longer (kilo.planner.plan), and the feet then travel a
little more than the stride.
"""

from __future__ import annotations

from dataclasses import dataclass

from kilo.planner.model import LB, LF, RB, RF

TICK_MS = 20.0
STRIDE_MIN_MM = 8.0
TURN_FRAC = 0.8          # a turn in place: each side's stride, this fraction of the forward stride
LEAN_MOVE_MS = 300.0     # the body's typical move between two swings (walk, crawl; the roll from one side to the other)
LEAN_LEAD_MS = -60.0     # the lift starts this long after the move's end; negative: 3 ticks before it, in its tail


@dataclass(frozen=True)
class Pattern:
    name: str
    groups: tuple            # tuple of tuples of legs: who swings, in order
    swing_ms: float          # fixed swing time
    stride_mm: float         # default stride
    step_h_mm: float         # foot lift at mid-swing
    body_h_256: int          # body height, /256 of L1 + L2
    period_ms: float         # the cycle at the default speed
    shift_min_ms: float      # the shortest four-foot phase before a swing (without the lean's needs)
    stance_max_ms: float     # the longest stance: slower than that, the stride shrinks
    lean: bool               # one leg at a time: the body moves over the three feet that stay down


# Walk and crawl lift LB, LF, RB, RF (the lateral sequence); the trot the
# diagonal pairs LF+RB, RF+LB. Since 2026-11-15 (the owner: "too slow,
# placing the feet super delicately") every pattern is about 2.3 times as
# fast at its default as before and its top speed 2 to 2.7 times the old
# default: longer strides, shorter swings (walk 500 -> 300 ms, crawl 297 ->
# 300 ms with a lower step), the walk's body 10/256 lower (the reach: its
# longer stride with the lean's roll otherwise asks the legs for more than
# the IK gives), the trot two ticks on four feet at its top speed instead of
# three. The tops: trot 152, crawl 26.9, walk 21.5 mm/s (the walk and the
# crawl: the lean's move paced by gravity, the legs' reach and the servos;
# docs/walking-and-balance.md §10).
PATTERNS: dict[str, Pattern] = {
    "walk": Pattern("walk", ((LB,), (LF,), (RB,), (RF,)), swing_ms=300.0, stride_mm=40.0, step_h_mm=15.0,
                    body_h_256=195, period_ms=2550.0, shift_min_ms=0.0, stance_max_ms=4000.0, lean=True),
    "trot": Pattern("trot", ((LF, RB), (RF, LB)), swing_ms=150.0, stride_mm=35.0, step_h_mm=8.0,
                    body_h_256=205, period_ms=393.0, shift_min_ms=40.0, stance_max_ms=1327.0, lean=False),
    "crawl": Pattern("crawl", ((LB,), (LF,), (RB,), (RF,)), swing_ms=300.0, stride_mm=50.0, step_h_mm=15.0,
                     body_h_256=179, period_ms=2550.0, shift_min_ms=0.0, stance_max_ms=4000.0, lean=True),
}
PATTERN_NAMES = tuple(PATTERNS)
DEFAULT_PATTERN = "trot"
DIRECTIONS = ("forward", "back", "left", "right", "march")
_ALIASES = {"backward": "back", "backwards": "back"}


def pattern(name: str) -> Pattern:
    try:
        return PATTERNS[name.lower()]
    except KeyError:
        raise ValueError(f"unknown gait pattern {name!r} (known: {', '.join(PATTERN_NAMES)})") from None


def direction(name: str) -> str:
    d = _ALIASES.get(name.lower(), name.lower())
    if d not in DIRECTIONS:
        raise ValueError(f"unknown direction {name!r} (known: {', '.join(DIRECTIONS)})")
    return d


@dataclass(frozen=True)
class Timing:
    stride_mm: float   # magnitude
    shift_ms: float
    stance_ms: float
    period_ms: float
    speed_mm_s: float  # what this timing walks at with the stride


def shift_min_ms(p: Pattern) -> float:
    if p.lean:
        return max(p.shift_min_ms, LEAN_MOVE_MS + LEAN_LEAD_MS)
    return p.shift_min_ms


def _stance_limits(p: Pattern) -> tuple[float, float]:
    n = len(p.groups)
    lo = n * (shift_min_ms(p) + p.swing_ms) - p.swing_ms
    return lo, max(lo, p.stance_max_ms)


def default_speed(p: Pattern | str) -> float:
    """The speed (mm/s) of the pattern's table: its stride over its default stance."""
    p = pattern(p) if isinstance(p, str) else p
    return p.stride_mm / (p.period_ms - p.swing_ms) * 1000.0


def speed_range(p: Pattern | str) -> tuple[float, float]:
    """(slowest, fastest) speed in mm/s that timing() gives."""
    p = pattern(p) if isinstance(p, str) else p
    lo, hi = _stance_limits(p)
    return STRIDE_MIN_MM / hi * 1000.0, p.stride_mm / lo * 1000.0


def timing(p: Pattern | str, speed_mm_s: float | None = None) -> Timing:
    """Stride, shift and period for walking at ``speed_mm_s`` (default: the
    pattern's default speed), clamped to speed_range()."""
    p = pattern(p) if isinstance(p, str) else p
    n = len(p.groups)
    st_lo, st_hi = _stance_limits(p)
    v = default_speed(p) if speed_mm_s is None else abs(float(speed_mm_s))
    stride = p.stride_mm
    stance = stride / v * 1000.0 if v > 0 else float("inf")
    if stance > st_hi:
        stance = st_hi
        stride = max(STRIDE_MIN_MM, v * st_hi / 1000.0)
    stance = max(stance, st_lo)
    period = stance + p.swing_ms
    return Timing(stride, period / n - p.swing_ms, stance, period, stride / stance * 1000.0)


def side_strides(direction_name: str, stride: float) -> tuple[float, float]:
    """(left, right) stride in mm (+ = the feet push the body forward) for a direction.

    forward / back: the stride on both sides; left / right: turn in place,
    one side backward and the other forward by TURN_FRAC of the stride;
    march: in place."""
    d = direction(direction_name)
    if d == "forward":
        return stride, stride
    if d == "back":
        return -stride, -stride
    if d == "march":
        return 0.0, 0.0
    t = TURN_FRAC * stride
    return (-t, t) if d == "left" else (t, -t)
