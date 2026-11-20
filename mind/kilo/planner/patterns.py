"""Gait patterns and the speed -> timing mapping (docs/walking-and-balance.md
§3, §10, §12; mind/README.md "Walking").

A pattern is a sequence of swing groups: the legs that lift together, in
order (in the walk the second leg of a pair lifts ``couple_ms`` after the
first). Each group gets a slot of the cycle: a four-foot phase (the
"shift": every foot down), then the swing part (``span_ms``: the swing and
the couple), so

    period = groups x (shift + span),   stance = period - swing.

The swing time is fixed per pattern (short enough for the body's tip time
constant on two feet, long enough for the servos: the walk's fastest
joint stays under 415 deg/s at every speed, the trot's up to 184 mm/s too
and under 480 above it (since 2026-11-17), under the body's 500 deg/s
safety slew).

The walk and the trot have a fixed cadence (``fixed_cadence``): their shift
stays at its shortest, one tick, and the speed sets the stride alone, from
STRIDE_MIN_MM to the pattern's stride. With one tick on four feet the
body's balance loop (which holds for two ticks after every touchdown) never
acts in them; with four to eight it rocked the trot (§11, §12).

The crawl: speed from the stride and the stance time, stance = stride /
speed; a slower speed lengthens the shift, never the swing; slower than
the longest stance allows, the stride shrinks (down to STRIDE_MIN_MM);
faster than its stride at the shortest shift allows, the speed is capped.
Its shortest shift: the body's move over the next three feet before each
lift (kilo.planner.plan), paced by gravity, LEAN_MOVE_MS, plus LEAN_LEAD_MS
(negative: the lift starts 60 ms before the move's end, in its cosine
tail). A move that needs longer makes its slot longer.
"""

from __future__ import annotations

from dataclasses import dataclass

from kilo.planner.model import LB, LF, RB, RF
from kilo.protocol import kilo_protocol as P

TICK_MS = float(P.TICK_MS)
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
    couple_ms: float = 0.0   # a group's second leg lifts (and lands) this long after its first
    default_mm_s: float | None = None  # the default speed (None: the stride over the default stance)
    # The lift tapers for long strides (step_height()): step_h_mm up to a
    # stride of taper_from_mm, then linearly down to step_h_long_mm at
    # stride_mm. None: step_h_mm at every stride.
    step_h_long_mm: float | None = None
    taper_from_mm: float = 0.0
    stride_slew_mm: float = 0.5  # the strides move at most this much a tick toward a new speed, direction or steer

    @property
    def span_ms(self) -> float:
        """A slot's swing part: from the first leg's lift to the last leg's touchdown."""
        return self.swing_ms + self.couple_ms * (len(self.groups[0]) - 1)


# Three patterns (docs/walking-and-balance.md §12, since 2026-11-16):
#
# - crawl: the careful gait, one leg at a time in the lateral sequence LB,
#   LF, RB, RF, statically stable: before each lift the body moves over the
#   three feet that stay down (the lean). 9 to 27 mm/s.
# - walk: a walk with diagonal couplets (Hildebrand's "trot-walk"; Petoi's
#   own walk is one too): LB lifts, RF 60 ms after it, LB lands, RF lands,
#   then RB and LF the same way. Four footfalls a cycle; on two diagonal
#   feet for 90 ms of each 150 ms swing, on three for the 60 ms either side,
#   no lean. The hind leg leads (a diagonal sequence, as primates walk; cats
#   lead with the fore leg) because the centre of mass is ahead of the
#   middle: on three feet with a hind leg up it is inside their triangle, with
#   a fore leg up outside. Fixed cadence (460 ms cycle, duty 0.67); speed
#   from the stride, 26 to 97 mm/s.
# - trot: the diagonal pairs LF+RB, RF+LB, fixed cadence (340 ms cycle, one
#   tick on four feet), speed from the stride, 42 to 232 mm/s (to 184 until
#   2026-11-17: a 35 mm stride). The stride now goes to 44 mm and the lift
#   tapers from 8 mm at 35 mm to 6 mm at 44, which keeps the swing's fastest
#   joint under 9.5 deg a tick (the body's slew is 10): the joint speed of
#   the swing is what limits the trot, not the legs' reach (at the 76 mm
#   stance height a foot reaches 48 mm fore and aft; it goes to 23).
#   docs/walking-and-balance.md §13.
#
# The walk and the trot move their strides 1.5 mm a tick (stride_slew_mm,
# since 2026-11-17; 0.5 before, the crawl's still): a turn in place to
# straight walking, a steer and a speed level take a cycle or two, not two
# seconds. The strides stay inside the pattern's longest one whatever the
# change, so no swing carries a foot further than at the top speed.
#
# Fixed cadence (the patterns without a lean): their four-foot phase stays
# one tick at every speed, so the body's balance loop (which holds its
# corrections while a foot is up and for two ticks after) never acts during
# them: in 4 to 8-tick four-foot phases it caught the tip of one diagonal
# and put it into the next, the rocking of §11.
PATTERNS: dict[str, Pattern] = {
    "walk": Pattern("walk", ((LB, RF), (RB, LF)), swing_ms=150.0, stride_mm=30.0, step_h_mm=8.0,
                    body_h_256=205, period_ms=460.0, shift_min_ms=20.0, stance_max_ms=310.0, lean=False,
                    couple_ms=60.0, default_mm_s=80.0, stride_slew_mm=1.5),
    "trot": Pattern("trot", ((LF, RB), (RF, LB)), swing_ms=150.0, stride_mm=44.0, step_h_mm=8.0,
                    body_h_256=205, period_ms=340.0, shift_min_ms=20.0, stance_max_ms=190.0, lean=False,
                    default_mm_s=144.0, step_h_long_mm=6.0, taper_from_mm=35.0, stride_slew_mm=1.5),
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
    lo = n * (shift_min_ms(p) + p.span_ms) - p.swing_ms
    return lo, max(lo, p.stance_max_ms)


def fixed_cadence(p: Pattern) -> bool:
    """Speed from the stride alone, the cycle fixed (the patterns without a lean)."""
    return not p.lean


def default_speed(p: Pattern | str) -> float:
    """The pattern's default speed (mm/s): its table's, or its stride over its default stance."""
    p = pattern(p) if isinstance(p, str) else p
    if p.default_mm_s is not None:
        return p.default_mm_s
    return p.stride_mm / (p.period_ms - p.swing_ms) * 1000.0


def speed_range(p: Pattern | str) -> tuple[float, float]:
    """(slowest, fastest) speed in mm/s that timing() gives."""
    p = pattern(p) if isinstance(p, str) else p
    lo, hi = _stance_limits(p)
    if fixed_cadence(p):
        hi = lo
    return STRIDE_MIN_MM / hi * 1000.0, p.stride_mm / lo * 1000.0


def timing(p: Pattern | str, speed_mm_s: float | None = None) -> Timing:
    """Stride, shift and period for walking at ``speed_mm_s`` (default: the
    pattern's default speed), clamped to speed_range()."""
    p = pattern(p) if isinstance(p, str) else p
    n = len(p.groups)
    st_lo, st_hi = _stance_limits(p)
    v = default_speed(p) if speed_mm_s is None else abs(float(speed_mm_s))
    if fixed_cadence(p):
        stance = st_lo
        stride = min(p.stride_mm, max(STRIDE_MIN_MM, v * stance / 1000.0))
    else:
        stride = p.stride_mm
        stance = stride / v * 1000.0 if v > 0 else float("inf")
        if stance > st_hi:
            stance = st_hi
            stride = max(STRIDE_MIN_MM, v * st_hi / 1000.0)
        stance = max(stance, st_lo)
    period = stance + p.swing_ms
    return Timing(stride, period / n - p.span_ms, stance, period, stride / stance * 1000.0)


def step_height(p: Pattern, stride: float) -> float:
    """The foot's lift at mid-swing for a swing of ``stride`` (mm, a
    magnitude): step_h_mm, tapering to step_h_long_mm at the longest stride."""
    if p.step_h_long_mm is None or stride <= p.taper_from_mm:
        return p.step_h_mm
    f = min(1.0, (stride - p.taper_from_mm) / (p.stride_mm - p.taper_from_mm))
    return p.step_h_mm + (p.step_h_long_mm - p.step_h_mm) * f


def side_strides(direction_name: str, stride: float, steer: float = 0.0,
                 cap: float | None = None) -> tuple[float, float]:
    """(left, right) stride in mm (+ = the feet push the body forward) for a direction.

    forward / back: the stride on both sides, bent by ``steer`` (-1..1, +
    left: the nose turns left, forward and backward alike): the left side's
    stride is the stride less steer x its size, the right side's the stride
    plus it, both scaled down together when the longer passes ``cap`` (the
    pattern's longest stride), so the bend stays and no foot goes further
    than at the top speed. steer 1: the inner side steps in place. left /
    right: turn in place, one side backward and the other forward by
    TURN_FRAC of the stride; march: in place (steer ignored for these)."""
    d = direction(direction_name)
    if d in ("forward", "back"):
        s = stride if d == "forward" else -stride
        w = max(-1.0, min(1.0, steer)) * abs(s)
        left, right = s - w, s + w
        m = max(abs(left), abs(right))
        if cap is not None and m > cap > 0:
            left, right = left * cap / m, right * cap / m
        return left, right
    if d == "march":
        return 0.0, 0.0
    t = TURN_FRAC * stride
    return (-t, t) if d == "left" else (t, -t)
