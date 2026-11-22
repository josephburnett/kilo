"""Gait patterns and the speed -> timing mapping (docs/walking-and-balance.md
§3, §10, §12; mind/README.md "Walking").

A pattern is a sequence of swing groups: the legs that lift together, in
order (a group's second leg may lift ``couple_ms`` after the first; no
pattern uses that today). Each group gets a slot of the cycle: a four-foot
phase (the "shift": every foot down), then the swing part (``span_ms``: the
swing and the couple), so

    period = groups x (shift + span),   stance = period - swing.

The swing time is fixed in the walk and the crawl (long enough for the
servos: the walk's fastest joint stays under 460 deg/s at every speed,
under the body's 500 deg/s safety slew). The trot's swing grows with the
speed instead (``swing_for``, the trot's schedule below; docs/walking-and-
balance.md §15): 8 ticks up to 150 mm/s, a tick more every 18 mm/s, at
most 11, so its stride can grow to 52 mm without a joint passing that
slew.

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

import math
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
    # The strides centred under the centre of mass: every foot lands half a stride plus the model's
    # centre of mass (BodyModel.stance_com, mm ahead of the shoulders' middle) ahead of its
    # shoulder, so the feet's polygon is centred on it (the walk, since 2026-11-22).
    centre_com: bool = False
    # The four-foot ticks before each group's swing (the walk, since its second version of
    # 2026-11-22: more before a hind leg's swing, where the weight crosses the diagonal the
    # fore leg before it and the hind leg share; fewer before a fore leg's). Their mean is
    # shift_min_ms; () = every slot the same.
    handover_ticks: tuple = ()
    # The sway (the walk): the body's own move in time with the cycle, scaled with the stride
    # (none standing, stopping or turning in place). A roll once a cycle, sway_roll_deg x
    # cos(2 pi (phase - sway_roll_at) / cycle) (negative: the right side down at sway_roll_at),
    # and a fore-aft shift of the body over its feet twice a cycle, sway_dx_mm x cos(4 pi (phase
    # - sway_dx_at) / cycle) (+: the body ahead of its feet); the phase in ticks from the start
    # of the first group's slot (the four-foot ticks before LB's swing).
    sway_roll_deg: float = 0.0
    sway_roll_at: float = 0.0
    sway_dx_mm: float = 0.0
    sway_dx_at: float = 0.0
    # Faster than long_from_mm_s the swing is swing_long_ms with handover_long_ticks (the walk: its
    # longest strides in a 220 ms swing keep every joint under 460 deg/s; the footfalls stay where
    # they are, the lift starts a tick earlier); backward always, its four-foot ticks evenly spread
    # and no sway (the first version's walk). 0: swing_ms at every speed.
    swing_long_ms: float = 0.0
    handover_long_ticks: tuple = ()
    long_from_mm_s: float = 0.0
    long_stride_mm: float = 0.0  # ...and for a slot whose longer side's stride is past this (a steer)

    @property
    def span_ms(self) -> float:
        """A slot's swing part: from the first leg's lift to the last leg's touchdown."""
        return self.swing_ms + self.couple_ms * (len(self.groups[0]) - 1)


# Three patterns (docs/walking-and-balance.md §12, since 2026-11-16):
#
# - crawl: the careful gait, one leg at a time in the lateral sequence LB,
#   LF, RB, RF, statically stable: before each lift the body moves over the
#   three feet that stay down (the lean). 9 to 27 mm/s.
# - walk (since 2026-11-22, §16): the cat's lateral-sequence walk, LB, LF,
#   RB, RF one at a time, a 960 ms cycle. Its second version (§16.8): 3
#   ticks on four feet before a hind leg's swing (where the weight crosses
#   the diagonal the fore leg before it shares with it) and 1 before a fore
#   leg's, a 200 ms swing (220 ms with 2 and 0 above 63 mm/s and for a
#   side stride over 48 mm), so a footfall 220 ms after a hind foot's and
#   260 after a fore foot's; a 1 deg roll and 3 mm fore-aft sway
#   (sway_*). No lean and no couplets: three feet down, never two. With the
#   strides centred under the centre of mass (centre_com) the plan keeps
#   the centre of mass 1 to 2 mm inside the three feet from 40 mm/s up
#   (0.4 at the slowest). Backward the same order, the first version's
#   timing (a 220 ms swing every 240 ms, no sway; §16.6). Fixed cadence; speed from the
#   stride, 11 to 73 mm/s (8 to 54 mm). It replaced the walk with diagonal
#   couplets (LB, then RF 60 ms later; 150 ms swings; 460 ms cycle; 26 to 97
#   mm/s), which stood on two feet 80 ms of every step and had the centre
#   of mass up to 7.5 mm outside the three feet: the stutter (§16.1).
# - trot: the diagonal pairs LF+RB, RF+LB, one tick on four feet, the swing
#   growing with the speed (TrotConfig, swing_for; the "long trot" of
#   2026-11-21, the only trot since 2026-11-22): the stride is the speed x
#   (swing + 1.5 ticks), 42 to 207 mm/s forward (180 backward), the stride
#   up to 51.8 mm, the lift 8 mm at every stride. Each pair lands where the
#   body, falling about its diagonal, ends upright (kilo.planner.plan
#   TrotFit). docs/walking-and-balance.md §15.
#
# The walk moves its strides 1.0 mm a tick (stride_slew_mm; its cycle is
# nearly three times the trot's: it builds up over a cycle), the trot 0.75
# (its build-up), the crawl 0.5: a turn in place to straight walking, a
# steer and a speed level take a cycle or two. The strides stay inside the
# pattern's longest one whatever the change, so no swing carries a foot
# further than at the top speed.
#
# Fixed cadence (the patterns without a lean): their four-foot phase stays
# one tick at every speed, so the body's balance loop (which holds its
# corrections while a foot is up and for two ticks after) never acts during
# them: in 4 to 8-tick four-foot phases it caught the tip of one diagonal
# and put it into the next, the rocking of §11.
PATTERNS: dict[str, Pattern] = {
    "walk": Pattern("walk", ((LB,), (LF,), (RB,), (RF,)), swing_ms=200.0, stride_mm=54.0, step_h_mm=8.0,
                    body_h_256=205, period_ms=960.0, shift_min_ms=40.0, stance_max_ms=760.0, lean=False,
                    default_mm_s=58.0, step_h_long_mm=6.0, taper_from_mm=40.0, stride_slew_mm=1.0,
                    centre_com=True, handover_ticks=(3, 1, 3, 1), sway_roll_deg=-1.0, sway_roll_at=9.0,
                    sway_dx_mm=3.0, sway_dx_at=6.0, swing_long_ms=220.0, handover_long_ticks=(2, 0, 2, 0),
                    long_from_mm_s=63.0, long_stride_mm=48.0),
    # the trot's swing_ms, stride_mm and period_ms are its table entry's (its timing is TrotConfig's)
    "trot": Pattern("trot", ((LF, RB), (RF, LB)), swing_ms=160.0, stride_mm=52.0, step_h_mm=8.0,
                    body_h_256=205, period_ms=360.0, shift_min_ms=20.0, stance_max_ms=250.0, lean=False,
                    default_mm_s=144.0, stride_slew_mm=0.75),
    "crawl": Pattern("crawl", ((LB,), (LF,), (RB,), (RF,)), swing_ms=300.0, stride_mm=50.0, step_h_mm=15.0,
                     body_h_256=179, period_ms=2550.0, shift_min_ms=0.0, stance_max_ms=4000.0, lean=True),
}
PATTERN_NAMES = tuple(PATTERNS)
DEFAULT_PATTERN = "trot"
DIRECTIONS = ("forward", "back", "left", "right", "march")
_ALIASES = {"backward": "back", "backwards": "back"}


def pattern(name: "str | Pattern") -> Pattern:
    """A pattern by name; a Pattern is returned as it is."""
    if isinstance(name, Pattern):
        return name
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


def is_trot(p: Pattern) -> bool:
    """The trot: its swing and its timing are TrotConfig's."""
    return p.name == "trot"


# The trot's schedule (docs/walking-and-balance.md §15). Its stride is limited by the joints: a
# fixed 8-tick swing cannot carry more than about 44 mm under the body's 500 deg/s. So the swing
# grows with the speed, a tick a step at most (the build-up), with one tick on four feet: a foot's
# ground time is the swing + STANCE_PLUS ticks and the stride the speed x that.
STANCE_PLUS = 1.5


@dataclass(frozen=True)
class TrotConfig:
    v_from: float = 150.0          # up to this speed the swing is 8 ticks ...
    v_step: float = 18.0           # ... then a tick more for every v_step mm/s ...
    w_min: int = 8
    w_max: int = 11                # ... up to this (220 ms)
    top_mm_s: float = 207.0        # the fastest forward: 51.8 mm at 11 ticks
    top_back_mm_s: float = 180.0   # ... and backward (its feet land further forward, where the knee works harder)
    step_mm_per_tick: float = 5.4  # a swing that carries a foot d mm takes at least d / this ticks
    stride_per_tick: float = 4.75  # ... and a side's stride is at most this x the swing's ticks less 0.5 mm
    # A swing that is not the steady trot's (a stop's or the quick stop's square-up from a long
    # stride, the way home into the crawl, the swing shortening out of the trot) is also as long
    # as its path needs for no joint to turn more than this a tick (the body's slew is 10, 500 deg/s):
    # a foot that starts 45 mm behind its shoulder, near the leg's reach, turns the knee fast.
    joint_deg_per_tick: float = 8.0


TROT = TrotConfig()


def swing_for(speed_mm_s: float, cfg: TrotConfig = TROT) -> int:
    """The trot's swing ticks at a speed (the build-up's target)."""
    v = abs(speed_mm_s)
    if v <= cfg.v_from:
        return cfg.w_min
    return min(cfg.w_max, cfg.w_min + int(math.ceil((v - cfg.v_from) / cfg.v_step - 1e-9)))


def trot_timing(speed_mm_s: float | None, back: bool = False, cfg: TrotConfig = TROT) -> tuple[int, float, float]:
    """(swing ticks, stride mm, speed mm/s) of the trot at a speed (default: 144), clamped to its
    slowest (STRIDE_MIN_MM over the 8-tick swing's ground time) and to top_mm_s (top_back_mm_s
    backward)."""
    v = PATTERNS["trot"].default_mm_s if speed_mm_s is None else abs(float(speed_mm_s))
    lo = STRIDE_MIN_MM / ((cfg.w_min + STANCE_PLUS) * TICK_MS / 1000.0)
    v = min(cfg.top_back_mm_s if back else cfg.top_mm_s, max(lo, v))
    w = swing_for(v, cfg)
    stance = (w + STANCE_PLUS) * TICK_MS / 1000.0
    s = max(STRIDE_MIN_MM, v * stance)
    return w, s, s / stance


def shift_min_ms(p: Pattern) -> float:
    if p.lean:
        return max(p.shift_min_ms, LEAN_MOVE_MS + LEAN_LEAD_MS)
    return p.shift_min_ms


def is_long(p: Pattern, speed_mm_s: float, back: bool = False) -> bool:
    """The pattern swings swing_long_ms at this speed (Pattern.long_from_mm_s), and always backward
    (the walk backward is its first version's: docs/walking-and-balance.md §16.8)."""
    return p.swing_long_ms > 0 and (back or abs(speed_mm_s) > p.long_from_mm_s)


def swing_at(p: Pattern, speed_mm_s: float, back: bool = False) -> float:
    """The swing time (ms) at a speed: swing_ms, or swing_long_ms above long_from_mm_s and backward."""
    return p.swing_long_ms if is_long(p, speed_mm_s, back) else p.swing_ms


def _stance_limits(p: Pattern, swing_ms: float | None = None) -> tuple[float, float]:
    """(shortest, longest) stance; with ``swing_ms`` (a long swing) the cycle stays the same."""
    n = len(p.groups)
    lo = n * (shift_min_ms(p) + p.span_ms) - p.swing_ms
    if swing_ms is not None:
        lo -= swing_ms - p.swing_ms
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
    if is_trot(p):
        return trot_timing(0.0)[2], TROT.top_mm_s
    lo, hi = _stance_limits(p)
    if fixed_cadence(p):
        hi = lo
    if p.swing_long_ms > 0:  # the top speed on the long swing
        lo = _stance_limits(p, p.swing_long_ms)[0]
    return STRIDE_MIN_MM / hi * 1000.0, p.stride_mm / lo * 1000.0


def timing(p: Pattern | str, speed_mm_s: float | None = None, direction_name: str = "forward") -> Timing:
    """Stride, shift and period for walking at ``speed_mm_s`` (default: the
    pattern's default speed), clamped to speed_range(). The trot's is its
    schedule's (trot_timing: the swing from swing_for, one tick on four feet,
    the ground time the swing + STANCE_PLUS ticks; slower backward)."""
    p = pattern(p) if isinstance(p, str) else p
    if is_trot(p):
        w, s, v = trot_timing(speed_mm_s, back=direction(direction_name) == "back")
        st = (w + STANCE_PLUS) * TICK_MS
        return Timing(s, TICK_MS, st, st + w * TICK_MS, v)
    n = len(p.groups)
    v = default_speed(p) if speed_mm_s is None else abs(float(speed_mm_s))
    sw = swing_at(p, v, direction(direction_name) == "back")
    st_lo, st_hi = _stance_limits(p, sw if sw != p.swing_ms else None)
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
    period = stance + sw
    return Timing(stride, period / n - (p.span_ms - p.swing_ms + sw), stance, period, stride / stance * 1000.0)


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
