"""Parser for the legacy OpenCat skill tables (legacy/InstinctNybble.h).

Array layout (legacy/skill.h dataLen()/formatSkill(), all int8):

    header  period, roll, pitch, angle_ratio            (4 B)
            + loop_start, loop_end, loop_cycles         (3 B more if period < 0)
    frames  |period| rows of
              period == 1  posture    16 angles (joints 0-15)
              period  > 1  gait        8 angles (joints 8-15 only)
              period  < 0  behaviour  16 angles + speed, delay, trigger axis, trigger angle

- roll/pitch are expectedRollPitch[0]/[1] (the header's roll comes first;
  Skill::info() mislabels them "pitch,roll").
- angle_ratio 2 means every stored angle is half the real one.
- Loop bytes are read as unsigned (`byte loopCycle[3]`).
- speed: transform() is called with speedRatio = speed / 4.0 and moves in
  steps = round(maxDiff / speedRatio) steps, where maxDiff is the largest
  change of ALL 16 joints (the head and the never-driven 3-7 too) from
  where calibratedPWM() last left them; it makes steps + 1 passes
  (s = 0..steps), each writing all 16 joints with cosine easing; speed <= 0
  jumps straight to the frame (one pass).
- delay: delay(|delay| * 50) ms after the frame arrives.
- trigger axis != 0: block until ypr[|axis|] crosses `angle` in the direction
  of sign(axis).
- Behaviour loop: frames loop_start..loop_end are played loop_cycles times in
  total (0 and 1 = once; >= 129 wraps int8 negative = forever). A loop_end of
  0 never loops (`c != 0` in perform()). On a repeat loop_start is entered
  from loop_end's pose, not from the frame before it.
- After a behaviour the last frame is held as a posture.

Mapping onto Kilo keyframes (50 Hz tick, 20 ms) is in `to_trick()` and in
mind/README.md "Tricks".
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Sequence

from kilo.protocol import kilo_protocol as P

from .model import (CANONICAL, JOINT_LIMITS, JOINT_NAMES, JOINT_SLOTS, REPEAT_FOREVER,
                    MAX_FRAMES, TICK_MS, UNUSED, Easing, Keyframe, Posture, Trick, Trigger, _round)

DOF = 16
WALKING_DOF = 8

# Duration of one legacy transform() pass: 16 x (a cos(), float maths,
# calibratedPWM()'s EEPROM reads and one PCA9685 write at 500 kHz I2C), with
# the IR receiver's 50 us timer interrupt taking its share. From the code
# with typical ATmega328P costs: 5.6 / 7.4 / 10.2 ms (fast / central / slow,
# mind/README.md "The original firmware's trick timing"). The central figure; no film of the
# legacy firmware has calibrated it.
LEGACY_STEP_MS = 7.4
# Legacy delay byte unit: delay(|delay| * 50) ms.
LEGACY_DELAY_MS = 50
# The legacy firmware's angleLimit[] (OpenCat.h, in git history), all 16 slots: calibratedPWM()
# clamps every write to them, so they are where a joint is left for the next frame's maxDiff.
LEGACY_ANGLE_LIMITS = ((-120, 120), (-80, 45), (-120, 120), (-120, 120), (-90, 60), (-90, 60), (-90, 90),
                       (-90, 90), (-100, 80), (-100, 80), (-80, 100), (-80, 100), (-60, 120), (-60, 120),
                       (-120, 60), (-120, 60))
# Legacy postures move from wherever the robot is at 2 deg/step; the start is
# unknown, so a posture trick uses the body's default posture transition.
POSTURE_DUR_TICKS = P.PARAMS_BY_NAME["POSTURE_TICKS"].default  # the body's default posture transition
# A legacy posture that is not one of the body's (stretch, butt_up) was held until the next
# command; as a trick it holds this long before the body takes its end posture (STAND).
POSTURE_HOLD_TICKS = 100  # 2 s
# Gait frames: legacy writes one joint per loop() pass, 12 passes a frame
# (head + 8 legs), ≈ one 20 ms tick at Kilo's rate (est.).
GAIT_FRAME_TICKS = 1
# The pose a behaviour is assumed to start from when sizing frame 0's dur.
START_POSTURE = "up"

DEFAULT_PATH = Path(__file__).resolve().parents[3] / "legacy" / "InstinctNybble.h"

POSTURE_TABLES = (("up", Posture.STAND), ("sit", Posture.SIT), ("rest", Posture.REST))


@dataclass
class LegacySkill:
    """One raw skill array, values exactly as stored."""
    name: str
    period: int
    roll: int
    pitch: int
    angle_ratio: int = 1
    loop: tuple | None = None                       # (start, end, cycles), behaviours only
    frames: list = field(default_factory=list)      # stored angles, 16 or 8 per row
    extras: list = field(default_factory=list)      # (speed, delay, trig_axis, trig_angle) per row
    skill_type: str = ""                            # 'I' instinct / 'N' newbility, from skillNameWithType

    @property
    def kind(self) -> str:
        return "posture" if self.period == 1 else ("gait" if self.period > 1 else "behaviour")

    @property
    def frame_size(self) -> int:
        return DOF if self.period == 1 else (WALKING_DOF if self.period > 1 else DOF + 4)

    def angles(self, k: int) -> tuple:
        """Real angles of row k in the 16 joint slots (None = not in the row)."""
        row = [v * self.angle_ratio for v in self.frames[k]]
        if self.period > 1:
            return (None,) * (DOF - WALKING_DOF) + tuple(row)
        return tuple(row)

    @classmethod
    def from_array(cls, name: str, data: list[int], skill_type: str = "") -> "LegacySkill":
        data = [int(v) for v in data]
        if not data:
            raise ValueError(f"{name}: empty array")
        period = data[0]
        if period == 0:
            raise ValueError(f"{name}: period 0")
        hdr = 4 if period > 0 else 7
        size = DOF if period == 1 else (WALKING_DOF if period > 1 else DOF + 4)
        want = hdr + abs(period) * size
        if len(data) != want:
            raise ValueError(f"{name}: {len(data)} values, period {period} needs {want}")
        s = cls(name=name, period=period, roll=data[1], pitch=data[2], angle_ratio=data[3],
                skill_type=skill_type)
        if period < 0:
            s.loop = tuple(v & 0xFF for v in data[4:7])
        for k in range(abs(period)):
            row = data[hdr + k * size: hdr + (k + 1) * size]
            if period < 0:
                s.frames.append(row[:DOF])
                s.extras.append(tuple(row[DOF:]))
            else:
                s.frames.append(row)
        return s

    def to_array(self) -> list[int]:
        out = [self.period, self.roll, self.pitch, self.angle_ratio]
        if self.period < 0:
            out += [v if v < 128 else v - 256 for v in self.loop]
        for k, row in enumerate(self.frames):
            out += list(row)
            if self.period < 0:
                out += list(self.extras[k])
        return out

    def to_c(self) -> str:
        """The array as a C definition in the InstinctNybble.h style."""
        a = self.to_array()
        hdr = 4 if self.period > 0 else 7
        lines = [f"const int8_t {self.name}[] PROGMEM = {{ ", ", ".join(map(str, a[:4])) + ","]
        if self.period < 0:
            lines.append(" " + ", ".join(map(str, a[4:7])) + ", ")
        size = self.frame_size
        for k in range(abs(self.period)):
            row = a[hdr + k * size: hdr + (k + 1) * size]
            lines.append("  " + ", ".join(f"{v:3d}" for v in row) + ",")
        lines.append("};")
        return "\n".join(lines) + "\n"


_ARRAY_RE = re.compile(r"const\s+int8_t\s+(\w+)\s*\[\s*\]\s*PROGMEM\s*=\s*\{(.*?)\}\s*;", re.S)
_NAMES_RE = re.compile(r"skillNameWithType\s*\[\s*\]\s*=\s*\{(.*?)\}\s*;", re.S)


def _strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


def parse_text(text: str) -> dict[str, LegacySkill]:
    """All skill arrays in a legacy Instinct*.h text, by array name, in file order."""
    text = _strip_comments(text)
    types: dict[str, str] = {}
    m = _NAMES_RE.search(text)
    if m:
        for tok in re.findall(r'"(\w+)"', m.group(1)):
            types[tok[:-1]] = tok[-1]
    out: dict[str, LegacySkill] = {}
    for m in _ARRAY_RE.finditer(text):
        name = m.group(1)
        vals = [int(v) for v in m.group(2).replace("\n", " ").split(",") if v.strip()]
        out[name] = LegacySkill.from_array(name, vals, types.get(name, ""))
    return out


def parse_file(path: str | Path = DEFAULT_PATH) -> dict[str, LegacySkill]:
    return parse_text(Path(path).read_text())


# ---------------------------------------------------------------------------
# Mapping to Kilo tricks

def transform_passes(max_diff: float, speed: int) -> int:
    """Passes legacy transform() makes for a frame whose largest joint change
    is ``max_diff`` deg at ``speed``: round(max_diff / (speed/4)) + 1 (the
    Arduino round(), half away from zero); one for speed <= 0 (a jump)."""
    if speed <= 0:
        return 1
    return _round(max_diff / (speed / 4.0)) + 1


def passes_to_ticks(passes: int) -> int:
    """Kilo ticks (20 ms) for ``passes`` legacy passes at LEGACY_STEP_MS, to the nearest tick, >= 1."""
    return max(1, _round(passes * LEGACY_STEP_MS / TICK_MS))


def speed_to_ticks(max_diff: float, speed: int) -> int:
    """Keyframe dur in ticks for a legacy frame: its transform() passes at
    LEGACY_STEP_MS each; speed <= 0 (a jump) -> 1."""
    return 1 if speed <= 0 else passes_to_ticks(transform_passes(max_diff, speed))


def delay_to_ticks(delay: int) -> int:
    """Legacy delay(|delay|·50 ms) → hold ticks, rounded half up."""
    return _round(abs(delay) * LEGACY_DELAY_MS / TICK_MS)


def _legacy_clamp(angles: Sequence) -> tuple:
    return tuple(None if v is None else min(LEGACY_ANGLE_LIMITS[j][1], max(LEGACY_ANGLE_LIMITS[j][0], v))
                 for j, v in enumerate(angles))


def _max_diff(frm: Sequence, to: Sequence) -> float:
    """transform()'s maxDiff: the largest change over all 16 slots."""
    return max((abs(t - f) for f, t in zip(frm, to) if f is not None and t is not None), default=0.0)


def play_order(skill: LegacySkill) -> list[int]:
    """The rows a legacy behaviour plays, in order, as perform() walks them;
    an endless loop (cycles >= 129) is cut after one pass of its section."""
    ls, le, cycles = skill.loop
    repeat = 0 if cycles < 2 or cycles >= 129 else cycles - 1
    out, c = [], 0
    while c < abs(skill.period):
        out.append(c)
        if repeat and c != 0 and c == le:
            repeat -= 1
            c = ls
            continue
        c += 1
    return out


def legacy_timing(skill: LegacySkill, start: Sequence) -> list[tuple[int, float, float]]:
    """(row, move ms, delay ms) for each row a legacy behaviour plays, from
    the 16-slot pose ``start``: what the original firmware takes."""
    cur = _legacy_clamp(start)
    out = []
    for c in play_order(skill):
        target = skill.angles(c)
        speed, delay = skill.extras[c][0], skill.extras[c][1]
        passes = transform_passes(_max_diff(cur, target), speed)
        out.append((c, passes * LEGACY_STEP_MS, abs(delay) * LEGACY_DELAY_MS))
        cur = _legacy_clamp(target)
    return out


def legacy_ms(skill: LegacySkill, start: Sequence) -> float:
    """The original firmware's time for one play of a behaviour from ``start`` (ms)."""
    return sum(m + d for _, m, d in legacy_timing(skill, start))


def _nearest_posture(angles: tuple, tables: dict) -> tuple[Posture, float]:
    best, best_d = Posture.STAND, math.inf
    for name, posture in POSTURE_TABLES:
        ref = tables.get(name)
        if ref is None:
            continue
        d = max(abs(angles[j] - ref[j]) for j in range(8, 16))
        if d < best_d:              # ties keep the earlier (STAND first)
            best, best_d = posture, d
    return best, best_d


def to_trick(skill: LegacySkill, name: str | None = None,
             skills: dict[str, LegacySkill] | None = None,
             source_file: str = "legacy/InstinctNybble.h", start: Sequence | None = None) -> Trick:
    """Map a legacy skill onto a Kilo Trick. `skills` (the whole file) supplies
    the start pose (`up`; `start`, 16 slots, overrides it) and the posture
    tables used for end_posture."""
    skills = skills or {}
    notes: list[str] = []
    tables = {n: skills[n].angles(0) for n, _ in POSTURE_TABLES if n in skills}
    if start is None:
        start = tables.get(START_POSTURE) or (None,) * JOINT_SLOTS
    kind = skill.kind

    dropped = sorted({j for k in range(len(skill.frames)) for j, v in enumerate(skill.angles(k))
                      if j in UNUSED and v})
    if dropped:
        notes.append(f"dropped nonzero values on unused joints {dropped}")

    def angles_of(k: int) -> tuple:
        raw = skill.angles(k)
        a: list = [None] * JOINT_SLOTS
        for j in CANONICAL:
            v = raw[j]
            if v is None:
                continue
            lo, hi = JOINT_LIMITS[j]
            if not lo <= v <= hi:
                clamped.add(JOINT_NAMES[j])
                v = min(hi, max(lo, v))
            a[j] = v
        return tuple(a)

    clamped: set = set()
    loop_start = loop_end = repeats = 0
    if kind == "behaviour":
        # The rows in the order perform() plays them, a finite loop unrolled: on a repeat the
        # loop's first row is entered from its last row's pose, so it moves a different distance
        # (and takes a different time) than the first time; one keyframe per play keeps each exact.
        ls, le, cycles = skill.loop
        order = play_order(skill)
        forever = le != 0 and cycles >= 129
        if forever or len(order) > MAX_FRAMES:
            order = list(range(len(skill.frames)))
            if le != 0 and cycles >= 2:
                loop_start, loop_end = ls, le
                repeats = REPEAT_FOREVER if forever else cycles  # int8 wrap
        timing = []
        cur = _legacy_clamp(start)
        for k in order:
            timing.append(transform_passes(_max_diff(cur, skill.angles(k)), skill.extras[k][0]))
            cur = _legacy_clamp(skill.angles(k))
    else:
        order = list(range(len(skill.frames)))

    frames: list[Keyframe] = []
    for i, k in enumerate(order):
        a = angles_of(k)
        if kind == "posture":
            kf = Keyframe(a, dur=POSTURE_DUR_TICKS, hold=0, easing=Easing.COSINE, balance=True)
        elif kind == "gait":
            kf = Keyframe(a, dur=GAIT_FRAME_TICKS, hold=0, easing=Easing.LINEAR, balance=True)
        else:
            speed, delay, axis, angle = skill.extras[k]
            dur = 1 if speed <= 0 else passes_to_ticks(timing[i])
            hold = delay_to_ticks(delay)
            if dur > 255 or hold > 255:
                notes.append(f"frame {i}: dur {dur} / hold {hold} clipped to 255")
                dur, hold = min(dur, 255), min(hold, 255)
            kf = Keyframe(a, dur=dur, hold=hold,
                          easing=Easing.STEP if speed <= 0 else Easing.COSINE,
                          trigger=Trigger(axis, angle) if axis else None)
        frames.append(kf)
    if clamped:
        notes.append(f"clamped to joint limits (legacy clamps too): {sorted(clamped)}")

    if kind == "posture":
        notes.append(f"posture: dur {POSTURE_DUR_TICKS} ticks (legacy moves from the current pose "
                     "at 2 deg/step), balance on (legacy gyro balance applies to postures)")
    elif kind == "gait":
        loop_start, loop_end, repeats = 0, len(frames) - 1, REPEAT_FOREVER
        notes.append(f"gait: {GAIT_FRAME_TICKS} tick/frame linear (est.), loops until stopped, "
                     "legs only (head bob/wag is the body's)")
    else:
        unrolled = repeats == 0 and len(order) != len(skill.frames)
        notes.append(f"behaviour: dur = round((round(maxDiff/(speed/4)) + 1) * {LEGACY_STEP_MS} ms / 20 ms), "
                     f"maxDiff over all 16 joints from where the last row left them (frame 0 from "
                     f"`{START_POSTURE}`); hold = round(|delay| * 50 ms / 20 ms); loop {list(skill.loop)} -> "
                     + (f"unrolled, rows {order}" if unrolled else
                        f"start {loop_start} end {loop_end} repeats {repeats}")
                     + "; balance off (legacy perform() never balances behaviours)")
        if any(f.trigger for f in frames):
            notes.append("uses IMU-trigger columns: not supported by the body")

    # end_posture: what IDLE holds afterwards. Legacy holds the last frame;
    # Kilo returns to a built-in posture. Behaviours take the nearest one by
    # leg angles; posture tricks only their own (sit, rest, up), else STAND.
    end, d = Posture.STAND, None
    last = skill.angles(len(skill.frames) - 1)
    if kind != "gait":
        end, d = _nearest_posture(last, tables)
        if kind == "posture" and d != 0:
            end, d = Posture.STAND, None
            frames[0] = replace(frames[0], hold=POSTURE_HOLD_TICKS)
            notes.append(f"held {POSTURE_HOLD_TICKS} ticks (legacy held a posture until the next command)")
    if d is None:
        notes.append(f"end_posture {end.name} (default)")
    else:
        notes.append(f"end_posture {end.name} (nearest built-in posture, max leg diff {d:g} deg)")

    fall_ok = abs(skill.roll) > 45 or abs(skill.pitch) > 45
    if fall_ok:
        notes.append("FALL_OK: expected tilt > 45 deg")

    return Trick(
        name=name or skill.name,
        frames=tuple(frames),
        loop_start=loop_start, loop_end=loop_end, repeats=repeats,
        expected_roll=skill.roll, expected_pitch=skill.pitch,
        balance=kind != "behaviour", fall_ok=fall_ok, end_posture=end,
        source={"file": source_file, "legacy_name": skill.name, "kind": kind,
                "period": skill.period, "angle_ratio": skill.angle_ratio,
                **({"loop": list(skill.loop)} if skill.loop else {})},
        notes=tuple(notes),
    )
