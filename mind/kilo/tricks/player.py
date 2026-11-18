"""The body's keyframe player timing, in Python (body/src/core/keyframe.cpp).

Used by the virtual body (to send TRICK_DONE on the same tick as the real
body) and by the mind (to know how long a trick takes, for the play()
timeout). The C and Python versions are kept in step by
mind/tests/test_crosscheck.py and body/test/test_keyframe.cpp.

Timing (doc §3e): each frame is a MOVE of ``dur`` ticks then a HOLD of
``hold`` ticks; at speed ``s`` (Q6, 64 = 1x) a phase of n ticks takes
ceil(n * 64 / s) ticks. After ``loop_end`` the player goes back to
``loop_start`` until that section has been played ``repeats`` times in total
(0 and 1 = once, 255 = until stopped). TRICK_DONE comes on the tick after
the last phase.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

SPEED_UNITY = 64
FOREVER = 255
TICK_S = 0.02


def phase_ticks(n: int, speed_q6: int = SPEED_UNITY) -> int:
    """Ticks a phase of ``n`` ticks takes at ``speed_q6``."""
    return (n * 64 + speed_q6 - 1) // speed_q6


def play_order(n: int, loop_start: int, loop_end: int, repeats: int, max_plays: int | None = None) -> list[int]:
    """Frame indices in play order (``repeats`` 255 is cut at ``max_plays``)."""
    plays = max(repeats, 1) if repeats != FOREVER else (max_plays or 1)
    order = list(range(0, loop_end + 1))
    for _ in range(plays - 1):
        order += list(range(loop_start, loop_end + 1))
    return order + list(range(loop_end + 1, n))


def total_ticks(durs: Sequence[int], holds: Sequence[int], loop_start: int, loop_end: int, repeats: int,
                speed_q6: int = SPEED_UNITY) -> int | None:
    """Ticks from TRICK_PLAY to the last phase's end (None: until stopped).
    TRICK_DONE arrives one tick later."""
    if repeats == FOREVER:
        return None
    return sum(phase_ticks(durs[f], speed_q6) + phase_ticks(holds[f], speed_q6)
               for f in play_order(len(durs), loop_start, loop_end, repeats))


@dataclass
class Player:
    """Tick-exact mirror of kf_step(): call ``step()`` once per body tick."""
    durs: Sequence[int]
    holds: Sequence[int]
    loop_start: int = 0
    loop_end: int = 0
    repeats: int = 0
    speed: int = SPEED_UNITY
    running: bool = field(default=True, init=False)
    frame: int = field(default=0, init=False)
    hold: bool = field(default=False, init=False)
    plays: int = field(default=0, init=False)
    pos: int = field(default=0, init=False)

    def _len(self) -> int:
        return (self.holds[self.frame] if self.hold else self.durs[self.frame]) << 6

    def step(self) -> bool:
        """Advance one tick. True on the tick the trick finishes."""
        if not self.running:
            return False
        if self.pos >= self._len():
            self.pos = 0
            if not self.hold and self.holds[self.frame]:
                self.hold = True
            else:
                self.hold = False
                again = False
                if self.frame == self.loop_end:
                    self.plays = min(self.plays + 1, 254)
                    again = self.repeats == FOREVER or self.plays < self.repeats
                if again:
                    self.frame = self.loop_start
                else:
                    self.frame += 1
                    if self.frame >= len(self.durs):
                        self.running = False
                        return True
        self.pos += self.speed
        return False


# ---- the mind's guards (kilo.tricks.model.Guard, Trick.roll_max) ------------------------------
GUARD_EARLY_TICKS = 2  # a guard decides this many ticks before its hold ends


@dataclass
class GuardWatch:
    """What the mind checks while the body plays a trick: fed every
    TELEMETRY as (seconds since TRICK_PLAY's ACK, nose up deg, roll deg),
    it returns why the trick must be stopped (once), else None.

    - Each Guard: over the second half of its frame's hold (at the play's
      speed, by the body's timing) the nose-up readings must average
      inside the window and every |roll| stay within roll_max; decided
      GUARD_EARLY_TICKS before the hold ends. No reading there: stop.
    - Trick.roll_max: |roll| past it on two readings in a row: stop.
    kernel/mind/tricks/model.cpp GuardWatch does the same; the same cases
    test both (mind/tests/test_rear_up.py test_guard_watch,
    kernel/host/tests/test_tricks.cpp test_guard_watch)."""
    trick: object
    speed: int = SPEED_UNITY
    windows: list = field(default_factory=list, init=False)  # (guard, t_from, t_decide)
    sums: dict = field(default_factory=dict, init=False)     # guard index -> [sum, n]
    done: set = field(default_factory=set, init=False)
    over: int = field(default=0, init=False)
    fired: str | None = field(default=None, init=False)

    def __post_init__(self):
        t = self.trick
        frames = t.frames
        starts = []
        acc = 0
        order = play_order(len(frames), t.loop_start, t.loop_end, t.repeats, max_plays=1)
        first_hold = {}
        for f in order:
            acc += phase_ticks(frames[f].dur, self.speed)
            if f not in first_hold:
                first_hold[f] = acc
            acc += phase_ticks(frames[f].hold, self.speed)
        for i, g in enumerate(t.guards):
            h0 = first_hold[g.frame]
            h = phase_ticks(frames[g.frame].hold, self.speed)
            self.windows.append((g, (h0 + h / 2) * TICK_S, (h0 + h - GUARD_EARLY_TICKS) * TICK_S))
            self.sums[i] = [0.0, 0]

    def feed(self, t_s: float, nose_up: float, roll: float) -> str | None:
        if self.fired is not None:
            return None
        rm = getattr(self.trick, "roll_max", 0) or 0
        self.over = self.over + 1 if rm and abs(roll) > rm else 0
        if self.over >= 2:
            return self._fire(f"roll {roll:+.1f} deg past {rm:g}")
        for i, (g, t_from, t_dec) in enumerate(self.windows):
            if i in self.done:
                continue
            if t_from <= t_s <= t_dec:
                if abs(roll) > g.roll_max:
                    self.done.add(i)
                    return self._fire(f"guard {g.name or g.frame}: roll {roll:+.1f} deg past {g.roll_max:g}")
                self.sums[i][0] += nose_up
                self.sums[i][1] += 1
            elif t_s > t_dec:
                self.done.add(i)
                s, n = self.sums[i]
                if n == 0:
                    return self._fire(f"guard {g.name or g.frame}: no TELEMETRY during its hold")
                avg = s / n
                if not g.nose_up[0] <= avg <= g.nose_up[1]:
                    return self._fire(f"guard {g.name or g.frame}: nose up {avg:.1f} deg outside "
                                      f"{g.nose_up[0]:g}..{g.nose_up[1]:g}")
        return None

    def _fire(self, why: str) -> str:
        self.fired = why
        return why
