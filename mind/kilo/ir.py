"""The Petoi IR remote: key table and the daemon's ``--remote`` key map.

The body decodes the remote (NEC, address 0x00) and relays each key as EVENT
IR_KEY with ``arg = command | 0x100`` for a held-key repeat (docs/architecture.md
§0, §3a). The command bytes come from the legacy firmware's table
(``src/infrared.h``: 0x00FFA25D is address 0x00, ~0xFF, command 0xA2, ~0x5D,
written MSB first as the IRremote library did), and so do the legacy meanings
(its K00..K62 skill tokens), kept here for reference.

Keys are named by grid, as the owner reads the remote: columns A B C left to
right, rows 1-7 bottom to top (B7 is the top middle). ``ACTIONS`` is what
``kilo run --remote`` does with a key (the owner's map, 2026-11-15);
``action_for`` turns an IR_KEY arg into one. The mind kernel's remote
(kernel/mind/remote/) is a port of this module and keeps the same map:

====  ======  =========================================================
Key   Code    ``--remote``
====  ======  =========================================================
A7    0xA2    sleep: stop a walk the planned way, then posture rest
B7    0x62    faster: one speed level up (only while walking)
C7    0xE2    freeze: abort the walk, stop a trick and the behaviour runner;
              then only B6 and A7 act
A6    0x22    turn left (from standing: start the turn at 1x)
B6    0x02    stand and stop: stop a walk the planned way, then posture stand
C6    0xC2    turn right
A5    0xE0    behaviour: start the behaviour runner
B5    0xA8    slower: one speed level down, past the slowest into backward
C5    0x90    the calibration posture (CALIB), after stopping a walk
A4    0x68    trot: start walking forward in it, or change to it
B4    0x98    crawl
C4    0xB0    walk
B2    0x38    rear up (trick ``rear_up``, docs/rear-up-trick.md): stop a walk
              the planned way, then sit up on the haunches, hold, come
              down; pressed again while it plays, it comes down at once
              (TRICK_STOP over the trick's stop_ticks, 1.5 s, to STAND)
A3-C1         nothing (logged as ignored), but B2
====  ======  =========================================================
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any

REPEAT = 0x100


@dataclass(frozen=True)
class Key:
    command: int   # NEC command byte
    label: str     # what is printed on the key (the legacy table's name for it; not shown to the owner)
    legacy: str    # what the legacy firmware did with it
    grid: str = ""  # where the key is: column A B C left to right, row 1-7 bottom to top (B7 = top middle)


# The remote's 21 keys, row by row from the top as on the remote (and in
# src/infrared.h). The owner refers to keys by grid position (the icons on
# the keys are not legible), so every message names a key by grid, e.g. "B7".
KEYS: tuple[Key, ...] = (
    Key(0xA2, "CH-", "rest"),
    Key(0x62, "CH", "forward"),
    Key(0xE2, "CH+", "gyro toggle"),
    Key(0x22, "|<<", "left"),
    Key(0x02, ">>|", "stand"),
    Key(0xC2, ">||", "right"),
    Key(0xE0, "-", "pause/limp"),
    Key(0xA8, "+", "backward"),
    Key(0x90, "EQ", "calibrate"),
    Key(0x68, "0", "stepping"),
    Key(0x98, "100+", "crawl"),
    Key(0xB0, "200+", "walk"),
    Key(0x30, "1", "trot"),
    Key(0x18, "2", "sit"),
    Key(0x7A, "3", "stretch"),
    Key(0x10, "4", "hi"),
    Key(0x38, "5", "push-up"),
    Key(0x5A, "6", "pee"),
    Key(0x42, "7", "wash face"),
    Key(0x4A, "8", "butt-up"),
    Key(0x52, "9", "last skill"),
)
KEYS = tuple(Key(k.command, k.label, k.legacy, f"{'ABC'[i % 3]}{7 - i // 3}") for i, k in enumerate(KEYS))
BY_COMMAND: dict[int, Key] = {k.command: k for k in KEYS}
BY_LABEL: dict[str, Key] = {k.label: k for k in KEYS}
BY_GRID: dict[str, Key] = {k.grid: k for k in KEYS}


def split(arg: int) -> tuple[int, bool]:
    """An IR_KEY arg -> (command byte, held-key repeat)."""
    arg &= 0xFFFF
    return arg & 0xFF, bool(arg & REPEAT)


@dataclass(frozen=True)
class Action:
    kind: str        # "posture" | "gait" | "turn" | "speed" | "freeze" | "behaviour" | "trick" | "none" | "ignore"
    name: str = ""   # the posture, the gait pattern, the turn (left / right), faster / slower, the trick
    label: str = ""  # what the key does, for the log


SPEED_FACTORS = (0.4, 0.7, 1.0, 1.15, 1.25)  # x the gait's default speed, levels 1..5 (and -1..-5 backward)
DEFAULT_LEVEL = 3  # +3: 1x, the gait's spec defaults
_IGNORE = Action("ignore")  # a held-key repeat or another remote's key
_NONE = Action("none", label="none")  # rows 1-3: no action
ACTIONS: dict[str, Action] = {
    "A7": Action("posture", "rest", "sleep"),
    "B7": Action("speed", "faster", "faster"),
    "C7": Action("freeze", "", "freeze"),
    "A6": Action("turn", "left", "turn left"),
    "B6": Action("posture", "stand", "stand"),
    "C6": Action("turn", "right", "turn right"),
    "A5": Action("behaviour", "", "behaviour"),
    "B5": Action("speed", "slower", "slower"),
    "C5": Action("posture", "calib", "calibration posture"),
    "A4": Action("gait", "trot", "trot"),
    "B4": Action("gait", "crawl", "crawl"),
    "C4": Action("gait", "walk", "walk"),
    "B2": Action("trick", "rear_up", "rear up"),
}
ACTIONS.update({k.grid: _NONE for k in KEYS if k.grid not in ACTIONS})

FROZEN = "ignored (frozen: B6 or A7 to go on)"
CANCELLED = "cancelled (C7 froze Kilo)"
# rear_up holds Kilo's weight on two servos per hind leg for seconds: below
# this pack voltage (TELEMETRY batt_mv, filtered) B2 refuses, so that the
# body's low-battery reflex (REST from wherever Kilo is, docs/rear-up-trick.md)
# does not fire half way up. Also refused on USB power (no_battery).
TRICK_MIN_MV = 7400


def describe(arg: int) -> str:
    """``B6 (0x02, stand)``, ``B6 (0x02, stand) repeat``; ``? (0x45)`` for another remote's key."""
    cmd, rep = split(arg)
    k = BY_COMMAND.get(cmd)
    s = f"{k.grid} (0x{cmd:02X}, {ACTIONS[k.grid].label})" if k else f"? (0x{cmd:02X})"
    return s + (" repeat" if rep else "")


def action_for(arg: int) -> Action:
    """What ``--remote`` does with an IR_KEY arg. Repeats (a held key) and
    unknown keys are ``ignore``; rows 1-3 are ``none``."""
    cmd, rep = split(arg)
    k = BY_COMMAND.get(cmd)
    if rep or k is None:
        return _IGNORE
    return ACTIONS[k.grid]


def _stop_key(action: Action) -> bool:
    """B6 and A7: the keys that end a freeze and stop the behaviour runner."""
    return action.kind == "posture" and action.name in ("stand", "rest")


class RemoteControl:
    """Carries out ``--remote`` actions through a BodyClient (blocking: the
    daemon runs ``perform`` in its thread pool, one at a time, and ``freeze``
    beside it). Every call returns the one line the daemon logs.

    State: the gait (trot at start), a signed speed level (+1..+5 forward at
    SPEED_FACTORS x the gait's default speed, -1..-5 backward; no level 0)
    and the heading (straight, left, right). Every stop (B6, A7, C5, C7, a
    walk that ended by itself) puts the level back to +3 and the heading
    straight.

    - A gait key (A4 trot, B4 crawl, C4 walk): not walking, walk forward in
      it at +3. Walking in another gait: change the gait (after every leg has
      stepped home), level and heading kept. Walking in that gait: straight
      forward at the level's magnitude (from a turn or from backward);
      already straight forward, nothing changes.
    - A6 / C6: turn left / right at the level's magnitude; not walking, start
      the turn in the selected gait at 1x.
    - B7 / B5: one level up / down, through ... -2 -1 +1 +2 ..., stopping at
      +5 and -5; during a turn the magnitude only (1..5). Not walking:
      ignored. The line shows the real mm/s (kilo.planner.patterns.timing
      clamps: the trot's top two levels are the same speed).
    - A posture key (B6 stand, A7 rest, C5 calib) first stops the walk the
      planned way (each leg steps home, STANCE) and waits for it, then stops
      a playing trick, then takes the posture.
    - C7 (``freeze``): abort the walk (no planned stop: the body holds the
      last pose for link.feet_hold_ticks, settles the swing feet to the
      ground and stays in STANCE, powered), stop a trick and the behaviour
      runner; then every key but B6 and A7 is ignored until one of them.
      An action under way when C7 comes (a posture key waiting for the
      planned stop) returns ``cancelled`` instead of its posture.
    - A5: start the behaviour runner (``behaviour.start()``) after stopping
      a walk and a trick. While it runs, B6, A7 and C7 stop it first
      (``behaviour.stop()``) and then do their own action, A5 does nothing
      and the other keys are ignored.
    - B2 (``trick``): not playing a trick, stop a walk the planned way and
      start the trick (``start_trick``: the key returns at once, the trick
      plays on). While a trick plays (mode TRICK), B2 stops it: the toggle,
      so a second press brings rear_up down. Refused below TRICK_MIN_MV or
      on USB power.
    - Stopping a trick (B2, B6, A7, C5, a gait key, C7) sends TRICK_STOP over
      the trick's ``stop_ticks`` (BodyClient.stop_trick); a key that then
      takes a posture or walks waits that long first, so the crossfade down
      finishes before the next move starts.

    ``behaviour``: an object with ``active()``, ``start()`` (returns a detail
    for the line, or None; raises when it cannot start) and ``stop()``
    (blocks until the runner has stopped); None: A5 fails.
    """

    def __init__(self, client: Any, behaviour: Any = None):
        self.client = client
        self.behaviour = behaviour
        self.pattern = "trot"
        self.level = DEFAULT_LEVEL
        self.heading = "straight"
        self.frozen = False
        self._walking = False  # the remote started the walk going on
        self._freezes = 0
        self._freeze_lock = threading.Lock()

    # ---- state ---------------------------------------------------------------------------
    def direction(self) -> str:
        """The walk's direction for the level and heading: forward, back, left or right."""
        if self.heading != "straight":
            return self.heading
        return "forward" if self.level > 0 else "back"

    def speed_mm_s(self) -> float:
        """The speed asked for: the level's factor of the gait's default speed (a magnitude)."""
        from kilo.planner.patterns import default_speed

        return SPEED_FACTORS[abs(self.level) - 1] * default_speed(self.pattern)

    def body_walking(self) -> bool:
        st = self.client.walk_status()
        return bool(st is not None and st.get("active")) or self.client.state.mode == "GAIT"

    def walking(self) -> bool:
        """The remote's own walk is going on; once it has ended the level and heading start over."""
        st = self.client.walk_status()
        if self._walking and st is not None and st.get("active"):
            return True
        if self._walking:
            self._reset()
        return False

    def _reset(self) -> None:
        self._walking = False
        self.level = DEFAULT_LEVEL
        self.heading = "straight"

    def _walk_text(self, suffix: str = "") -> str:
        from kilo.planner.patterns import timing

        v = timing(self.pattern, self.speed_mm_s()).speed_mm_s
        return f"walk {self.direction()} ({self.pattern}) {v:.1f} mm/s{suffix}"

    def _go(self) -> str:
        try:
            self.client.walk(self.direction(), self.pattern, self.speed_mm_s())
        except Exception:
            self._reset()
            raise
        self._walking = True
        return self._walk_text()

    def _stop_walking(self) -> None:
        if self.body_walking():
            from kilo.planner.walker import stop_timeout

            st = self.client.walk_status() or {}
            self.client.stop_walk(wait_s=stop_timeout(st.get("period_ms")))

    # ---- keys ----------------------------------------------------------------------------
    def perform(self, action: Action) -> str:
        if action.kind == "freeze":
            return self.freeze()
        gen = self._freezes
        note = ""
        if self.frozen:
            if not _stop_key(action):
                return FROZEN
            self.frozen = False
            note = "unfrozen, "
        b = self.behaviour
        if b is not None and b.active():
            if action.kind == "behaviour":
                return "behaviour runner already running"
            if not _stop_key(action):
                return "ignored (behaviour runner active)"
            b.stop()
            note = "behaviour runner stopped, "
        return note + self._act(action, gen)

    def _stop_trick(self, settle: bool = True) -> None:
        """TRICK_STOP over the trick's stop_ticks; with ``settle``, wait
        until its crossfade to the end posture is over."""
        c = self.client
        t = c.started_trick() if hasattr(c, "started_trick") else None
        c.stop_trick()
        ticks = getattr(t, "stop_ticks", 0) or 0
        if settle and ticks:
            time.sleep(ticks * 0.02 + 0.1)

    def _trick(self, action: Action, gen: int) -> str:
        c = self.client
        if c.state.mode == "TRICK":  # the toggle: a second press brings it down
            self._stop_trick(settle=False)
            return "trick stopped: coming down"
        self._reset()
        self._stop_walking()
        if self._freezes != gen:
            return CANCELLED
        st = c.state
        if "no_battery" in st.flags:
            return f"{action.name} refused: on USB power (no battery)"
        mv = st.batt_mv
        if mv is None or mv < TRICK_MIN_MV:
            return f"{action.name} refused: battery {mv} mV (needs {TRICK_MIN_MV})"
        from kilo.tricks import library

        trick = library.load(action.name)
        c.start_trick(trick)
        secs = (trick.duration_ticks or 0) * 0.02
        return f"trick {action.name} started ({secs:.0f} s; B2 again, B6 or C7 bring it down)"

    def _act(self, action: Action, gen: int) -> str:
        c = self.client
        if action.kind == "trick":
            return self._trick(action, gen)
        if action.kind in ("posture", "behaviour"):
            self._reset()
            self._stop_walking()
            if self._freezes != gen:
                return CANCELLED
            if c.state.mode == "TRICK":
                self._stop_trick()
            if action.kind == "posture":
                c.posture(action.name)
                return f"posture {action.name}"
            if self.behaviour is None:
                return "behaviour runner failed: no behaviour runner"
            detail = self.behaviour.start()
            return "behaviour runner started" + (f" ({detail})" if detail else "")
        if action.kind == "gait":
            if self.walking():
                same = action.name == self.pattern
                was = (self.level, self.heading)
                self.pattern = action.name
                if same:  # that gait already: straight forward at the level's magnitude
                    self.heading = "straight"
                    self.level = abs(self.level)
                    if (self.level, self.heading) == was:
                        return self._walk_text(": unchanged")
                return self._go()
            self._reset()
            self.pattern = action.name
            if c.state.mode == "TRICK":
                self._stop_trick()
            return self._go()
        if action.kind == "turn":
            if self.walking():
                if self.heading == action.name:
                    return self._walk_text(": unchanged")
                self.heading = action.name
                self.level = abs(self.level)
                return self._go()
            self._reset()
            self.heading = action.name
            if c.state.mode == "TRICK":
                self._stop_trick()
            return self._go()
        if action.kind == "speed":
            from kilo.planner.patterns import timing

            if not self.walking():
                return "ignored (not walking)"
            up = action.name == "faster"
            n = len(SPEED_FACTORS)
            lv = self.level
            if self.heading != "straight":  # a turn: the magnitude only
                lv = min(lv + 1, n) if up else max(lv - 1, 1)
            elif up:
                lv = 1 if lv == -1 else min(lv + 1, n)
            else:
                lv = -1 if lv == 1 else max(lv - 1, -n)
            end = ""
            if lv == self.level:
                end = (": already the fastest" if up else ": already the slowest" if self.heading != "straight"
                       else ": already the fastest back")
            self.level = lv
            if not end:
                self._go()
            f = SPEED_FACTORS[abs(lv) - 1]
            v = timing(self.pattern, self.speed_mm_s()).speed_mm_s
            return f"speed {'-' if lv < 0 else ''}{f:g}x ({self.pattern} {v:.1f} mm/s {self.direction()}){end}"
        return "ignored (no action on this key)"

    def freeze(self) -> str:
        """C7: at once (beside an action under way, which then returns ``cancelled``)."""
        with self._freeze_lock:
            if self.frozen:
                return FROZEN
            self.frozen = True  # first: an action under way and the keys after it see it
            self._freezes += 1
            c = self.client
            trick = c.state.mode == "TRICK"
            parts = []
            w = getattr(c, "_walker", None)
            if w is not None and w.active:  # no planned stop: the body holds, settles, stays in STANCE
                w.abort()
                parts.append("walk aborted")
            b = self.behaviour
            if b is not None and b.active():
                b.stop()
                parts.append("behaviour runner stopped")
            if trick:
                self._stop_trick(settle=False)  # over its stop_ticks: rear_up comes down to STAND
                parts.append("trick stopped")
            self._reset()
            return f"frozen ({', '.join(parts) or 'holding the posture'}): only B6 or A7 act now"
