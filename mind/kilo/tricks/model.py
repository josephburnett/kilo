"""Trick model: keyframes, mirror, speed, validation and the packed tables.

A trick is a list of keyframes in joint space (§3e), designed here (the
library, kilo.tricks.rear_up, alive_set, righting) and played by the C++
mind as a stream of POSE keys with joint-mode legs (docs/movement-engine.md;
the body has no trick store since protocol 3.0). The kernel's trick tables
are generated from this module (kernel/mind/tricks/gen_library.py,
kernel/mind/intent/gen_intent.py): each trick as an 11 B header
(``Trick.begin``: ``TrickHeader``) and 14 B frames (``Keyframe.to_wire``:
``WireKeyframe``), the layout the body's trick store had, with ``crc16`` and
``trick_id_for`` as their identity.

Joints use the legacy 16-slot numbering (0 head pan, 1 head tilt, 2 tail,
3-7 unused, 8-11 shoulders LF RF RB LB, 12-15 knees LF RF RB LB). A packed
frame carries the 11 driven joints in canonical order, and -128 means
"unchanged" (None here). `joint_mask` is a u16 over the same 16 slots.

Also the mind's timing of a play at a speed (Q6, 64 = 1x: ``phase_ticks``,
``play_order``) and its guard watch (``GuardWatch``; the C++ one is in
kernel/mind/tricks/model.cpp).
"""

from __future__ import annotations

import binascii
import enum
import json
import math
import struct
from dataclasses import dataclass, field, replace
from typing import ClassVar, Iterable, Sequence

from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P

Posture = P.Posture


class Easing(enum.IntEnum):
    """A keyframe's move (packed flags bits 0-1)."""
    LINEAR = 0
    COSINE = 1
    STEP = 2
    SPLINE_RESERVED = 3


class TrickFlags(enum.IntFlag):
    """The header's flags."""
    BALANCE = 1 << 0
    FALL_OK = 1 << 1


class KeyframeFlags(enum.IntFlag):
    """A packed frame's flag above the easing."""
    BALANCE = 1 << 2


class TrickDoneReason(enum.IntEnum):
    """How a play ended."""
    DONE = 0
    STOPPED = 1
    PREEMPTED = 2
    FELL = 3


@dataclass
class WireKeyframe:
    """One packed frame, 14 B: dur, hold, flags (bits 0-1 easing, bit 2
    balance), ang[11] (whole degrees, canonical order, -128 unchanged)."""
    SIZE: ClassVar[int] = 14
    _FMT: ClassVar[struct.Struct] = struct.Struct("<BBB11b")
    dur: int = 0
    hold: int = 0
    flags: int = 0
    ang: list = field(default_factory=lambda: [0] * 11)

    def pack(self) -> bytes:
        return self._FMT.pack(self.dur, self.hold, self.flags, *self.ang)

    @classmethod
    def unpack(cls, data: bytes) -> "WireKeyframe":
        v = cls._FMT.unpack(bytes(data))
        return cls(dur=v[0], hold=v[1], flags=v[2], ang=list(v[3:]))


@dataclass
class TrickHeader:
    """A trick's packed header, 11 B."""
    SIZE: ClassVar[int] = 11
    _FMT: ClassVar[struct.Struct] = struct.Struct("<BBBBBHBBbb")
    trick_id: int = 0
    n_frames: int = 0
    loop_start: int = 0
    loop_end: int = 0
    repeats: int = 0      # total plays of loop_start..loop_end: 0 and 1 = once, 255 = until stopped
    joint_mask: int = 0
    flags: int = 0        # TrickFlags
    end_posture: int = 0  # enum posture
    exp_roll: int = 0     # deg; the balance reference for frames with the balance bit (right side up +)
    exp_pitch: int = 0    # deg (nose down +)

    def pack(self) -> bytes:
        return self._FMT.pack(self.trick_id, self.n_frames, self.loop_start, self.loop_end, self.repeats,
                              self.joint_mask, self.flags, self.end_posture, self.exp_roll, self.exp_pitch)

    @classmethod
    def unpack(cls, data: bytes) -> "TrickHeader":
        return cls(*cls._FMT.unpack(bytes(data)))


KEYFRAME_SIZE = WireKeyframe.SIZE

JOINT_SLOTS = 16
# The joints, from the spec's [[joints]] (P.JOINT_INFO): the driven ones in wire order (0, 1, 2, 8..15;
# spec joint_count = 11), their names (head_pan, ..., lf_shoulder, ..., lb_knee), the slots never driven.
CANONICAL = tuple(j.id for j in P.JOINT_INFO.values() if j.driven)
UNUSED = tuple(j.id for j in P.JOINT_INFO.values() if not j.driven)
ANGLE_UNCHANGED = -128

JOINT_NAMES = {j.id: j.name for j in P.JOINT_INFO.values() if j.driven}
JOINT_IDS = {n: j for j, n in JOINT_NAMES.items()}

# Joint limits in degrees for the driven joints: the spec's [[joints]] (the
# body's joints.cpp reads the same; from the original firmware's angleLimit[] in OpenCat.h).
JOINT_LIMITS = {j.id: (j.lo, j.hi) for j in P.JOINT_INFO.values() if j.driven}

# Mirror rule, from legacy Skill::mirror() (legacy/skill.h): negate head pan (0)
# and tail (2), swap the L/R columns (8,9) (10,11) (12,13) (14,15) (legacy also
# swaps 4-7, which Nybble never drives), and negate the expected roll.
# Head tilt is left alone. Legacy mirrors on a trailing 'R' in the skill name
# (or randomly for 'X'), not on a token: `m` is T_INDEXED_SEQUENTIAL_ASC.
MIRROR_NEGATE = (0, 2)
MIRROR_PAIRS = ((8, 9), (10, 11), (12, 13), (14, 15))

MAX_FRAMES = 24          # frames per trick (the body's old store, docs §3h)
TICK_MS = P.TICK_MS      # 50 Hz body tick
REPEAT_FOREVER = 255
SPEED_UNITY_Q6 = 64      # a play's speed, Q6: 64 = 1x
SPEED_MIN_Q6 = 16        # 0.25x


class TrickError(ValueError):
    """A trick that the body would refuse or play wrongly. `.problems` lists why."""

    def __init__(self, problems: list[str]):
        super().__init__("; ".join(problems))
        self.problems = problems


def _round(a: float) -> int:
    """Round half away from zero (not Python's banker's rounding)."""
    return int(math.floor(abs(a) + 0.5)) * (1 if a >= 0 else -1)


def _num(a: float):
    """Store integral floats as ints (cleaner JSON)."""
    return int(a) if float(a).is_integer() else a


@dataclass(frozen=True)
class Trigger:
    """Legacy IMU trigger: wait until ypr[|axis|] crosses `angle` in the
    direction of sign(axis). Not supported by the body; kept as metadata."""
    axis: int
    angle: int


@dataclass(frozen=True)
class Guard:
    """A check the MIND makes while the body plays a trick (mind-side only,
    not on the wire): over the second half of frame ``frame``'s hold the
    measured attitude (TELEMETRY: nose up = -pitch; roll, left side down +)
    must average inside ``nose_up`` = (lo, hi) degrees, and |roll| must stay
    at most ``roll_max``; otherwise the mind stops the trick (over the
    trick's stop_ticks, down to its end posture). The frame's hold must be
    at least GUARD_MIN_HOLD ticks. ``name`` is for the logs.

    ``push_roll`` (deg, 0 = none) also watches the frame's MOVE: from its
    start up to the hold's second half, the roll may change by at most this
    much from the last reading before the move (the roll he brings into it,
    whatever it is); past it on two readings in a row the mind stops the
    trick. It catches a tip while the legs push, before it gathers speed
    (the rear-up of 2026-11-22: the roll grew 9 deg in 0.35 s)."""
    frame: int
    nose_up: tuple
    roll_max: float
    name: str = ""
    push_roll: float = 0.0

    def to_dict(self) -> dict:
        d = {"frame": self.frame, "name": self.name, "nose_up": [_num(v) for v in self.nose_up],
             "roll_max": _num(self.roll_max)}
        if self.push_roll:
            d["push_roll"] = _num(self.push_roll)
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "Guard":
        return cls(frame=d["frame"], nose_up=tuple(d["nose_up"]), roll_max=d["roll_max"], name=d.get("name", ""),
                   push_roll=d.get("push_roll", 0.0))


GUARD_MIN_HOLD = 20  # ticks (0.4 s): the guard averages the second half of the hold


@dataclass(frozen=True)
class Keyframe:
    """One keyframe. `angles` has 16 slots in degrees, None = not driven by
    this frame (wire -128, "unchanged"). `dur` is ticks to move from the
    previous frame's targets, `hold` ticks to stay after arriving."""
    angles: tuple
    dur: int = 25
    hold: int = 0
    easing: Easing = Easing.COSINE
    balance: bool = False
    trigger: Trigger | None = None

    def __post_init__(self):
        a = tuple(None if v is None else v for v in self.angles)
        if len(a) != JOINT_SLOTS:
            raise ValueError(f"Keyframe needs {JOINT_SLOTS} angle slots, got {len(a)}")
        object.__setattr__(self, "angles", a)
        object.__setattr__(self, "easing", Easing(self.easing))

    @classmethod
    def from_joints(cls, joints: dict, **kw) -> "Keyframe":
        """Build from {joint name or index: degrees}."""
        a: list = [None] * JOINT_SLOTS
        for k, v in joints.items():
            j = JOINT_IDS[k] if isinstance(k, str) else int(k)
            a[j] = v
        return cls(tuple(a), **kw)

    @property
    def joints(self) -> dict:
        """{joint name: degrees} for the driven joints, in canonical order."""
        return {JOINT_NAMES[j]: self.angles[j] for j in CANONICAL if self.angles[j] is not None}

    @property
    def mask(self) -> int:
        return sum(1 << j for j, v in enumerate(self.angles) if v is not None)

    @property
    def flags(self) -> int:
        return int(self.easing) | (int(KeyframeFlags.BALANCE) if self.balance else 0)

    # -- transforms --------------------------------------------------------
    def mirror(self) -> "Keyframe":
        a = list(self.angles)
        for j in MIRROR_NEGATE:
            if a[j] is not None:
                a[j] = -a[j]
        for l, r in MIRROR_PAIRS:
            a[l], a[r] = a[r], a[l]
        return replace(self, angles=tuple(a))

    # -- wire --------------------------------------------------------------
    def to_wire(self) -> WireKeyframe:
        ang = [ANGLE_UNCHANGED if self.angles[j] is None else _round(self.angles[j]) for j in CANONICAL]
        return WireKeyframe(dur=self.dur, hold=self.hold, flags=self.flags, ang=ang)

    def encode(self) -> bytes:
        """The 14 B packed frame (dur, hold, flags, ang[11])."""
        return self.to_wire().pack()

    @classmethod
    def from_wire(cls, kf: WireKeyframe) -> "Keyframe":
        a: list = [None] * JOINT_SLOTS
        for j, v in zip(CANONICAL, kf.ang):
            a[j] = None if v == ANGLE_UNCHANGED else v
        return cls(tuple(a), dur=kf.dur, hold=kf.hold, easing=Easing(kf.flags & 0x03),
                   balance=bool(kf.flags & KeyframeFlags.BALANCE))

    @classmethod
    def decode(cls, data: bytes) -> "Keyframe":
        return cls.from_wire(WireKeyframe.unpack(data))

    # -- JSON --------------------------------------------------------------
    def to_dict(self) -> dict:
        d: dict = {"dur": self.dur, "hold": self.hold, "easing": self.easing.name.lower()}
        if self.balance:
            d["balance"] = True
        if self.trigger is not None:
            d["trigger"] = {"axis": self.trigger.axis, "angle": self.trigger.angle}
        d["joints"] = {k: _num(v) for k, v in self.joints.items()}
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "Keyframe":
        t = d.get("trigger")
        return cls.from_joints(
            d["joints"], dur=d["dur"], hold=d.get("hold", 0),
            easing=Easing[d.get("easing", "cosine").upper()],
            balance=d.get("balance", False),
            trigger=Trigger(t["axis"], t["angle"]) if t else None)


@dataclass(frozen=True)
class Trick:
    """A keyframe trick. `loop_start..loop_end` (inclusive frame indices) is
    played `repeats` times in total: 0 and 1 both mean once, 255 until
    stopped (legacy loopCycle semantics, see README "Tricks")."""
    name: str
    frames: tuple
    loop_start: int = 0
    loop_end: int = 0
    repeats: int = 0
    expected_roll: float = 0
    expected_pitch: float = 0
    balance: bool = False
    fall_ok: bool = False
    end_posture: Posture = Posture.STAND
    # Mind-side only (not in the header): the ticks a stop of this trick
    # takes down to its end posture (0 = the body's XFADE_TICKS, 0.2 s). A
    # trick that must come down slowly when stopped (rear_up) sets it; the
    # C++ mind's trick player uses it.
    stop_ticks: int = 0
    # Mind-side only: checks of the measured attitude while it plays (Guard),
    # and a roll limit for the whole play (deg, 0 = none): past it on two
    # TELEMETRY in a row the mind stops it. GuardWatch below and
    # kernel/mind/tricks/player.cpp apply both.
    guards: tuple = ()
    roll_max: float = 0
    source: dict = field(default_factory=dict, compare=False)
    notes: tuple = field(default=(), compare=False)

    def __post_init__(self):
        object.__setattr__(self, "frames", tuple(self.frames))
        object.__setattr__(self, "notes", tuple(self.notes))
        object.__setattr__(self, "end_posture", Posture(self.end_posture))
        object.__setattr__(self, "guards", tuple(self.guards))

    # -- derived -----------------------------------------------------------
    @property
    def joint_mask(self) -> int:
        """Joints the trick owns: those driven by any frame (u16 over slots)."""
        m = 0
        for f in self.frames:
            m |= f.mask
        return m

    @property
    def flags(self) -> int:
        return ((int(TrickFlags.BALANCE) if self.balance else 0)
                | (int(TrickFlags.FALL_OK) if self.fall_ok else 0))

    @property
    def duration_ticks(self) -> int:
        """Ticks for one pass with the loop section played `repeats` times
        (None when it loops until stopped)."""
        per = [f.dur + f.hold for f in self.frames]
        if self.repeats == REPEAT_FOREVER:
            return None
        extra = max(self.repeats, 1) - 1
        return sum(per) + extra * sum(per[self.loop_start:self.loop_end + 1])

    # -- transforms --------------------------------------------------------
    def mirror(self) -> "Trick":
        """Left/right mirror (legacy Skill::mirror): an involution."""
        return replace(self, frames=tuple(f.mirror() for f in self.frames),
                       expected_roll=-self.expected_roll)

    def speed(self, factor: float) -> "Trick":
        """Play `factor` times faster (2 = twice as fast): dur and hold are
        divided by factor. dur stays >= 1; raises if a value exceeds 255."""
        if factor <= 0:
            raise ValueError("speed factor must be > 0")
        frames = []
        for f in self.frames:
            dur = max(1, _round(f.dur / factor))
            hold = _round(f.hold / factor)
            if dur > 255 or hold > 255:
                raise TrickError([f"{self.name}: speed {factor} overflows a frame (dur {dur}, hold {hold})"])
            frames.append(replace(f, dur=dur, hold=hold))
        return replace(self, frames=tuple(frames))

    # -- validation --------------------------------------------------------
    def problems(self) -> list[str]:
        out: list[str] = []
        n = len(self.frames)
        if not 1 <= n <= MAX_FRAMES:
            out.append(f"{n} frames; the body stores 1..{MAX_FRAMES}")
        if not (0 <= self.loop_start <= self.loop_end < max(n, 1)):
            out.append(f"loop {self.loop_start}..{self.loop_end} outside 0..{n - 1}")
        if not 0 <= self.repeats <= 255:
            out.append(f"repeats {self.repeats} outside 0..255")
        if not 0 <= self.stop_ticks <= 255:
            out.append(f"stop_ticks {self.stop_ticks} outside 0..255")
        if self.joint_mask == 0:
            out.append("no joint is driven")
        for g in self.guards:
            tag = f"guard {g.name or g.frame}"
            if not 0 <= g.frame < n:
                out.append(f"{tag}: frame {g.frame} outside 0..{n - 1}")
            elif self.frames[g.frame].hold < GUARD_MIN_HOLD:
                out.append(f"{tag}: frame {g.frame}'s hold {self.frames[g.frame].hold} is under {GUARD_MIN_HOLD} ticks")
            if not (len(g.nose_up) == 2 and -90 <= g.nose_up[0] < g.nose_up[1] <= 90):
                out.append(f"{tag}: nose_up {g.nose_up} is not lo < hi within +-90")
            if not 0 < g.roll_max <= 90:
                out.append(f"{tag}: roll_max {g.roll_max} outside 0..90")
        if not 0 <= self.roll_max <= 90:
            out.append(f"roll_max {self.roll_max} outside 0..90")
        for i, f in enumerate(self.frames):
            if not 1 <= f.dur <= 255:
                out.append(f"frame {i}: dur {f.dur} outside 1..255")
            if not 0 <= f.hold <= 255:
                out.append(f"frame {i}: hold {f.hold} outside 0..255")
            if f.easing == Easing.SPLINE_RESERVED:
                out.append(f"frame {i}: easing 3 is reserved")
            if f.trigger is not None:
                out.append(f"frame {i}: IMU triggers are not supported by the body")
            for j, v in enumerate(f.angles):
                if v is None:
                    continue
                if j in UNUSED:
                    out.append(f"frame {i}: joint {j} is not a driven joint")
                    continue
                lo, hi = JOINT_LIMITS[j]
                if not lo <= _round(v) <= hi:
                    out.append(f"frame {i}: {JOINT_NAMES[j]} {v} outside {lo}..{hi}")
        return out

    def validate(self) -> "Trick":
        p = self.problems()
        if p:
            raise TrickError([f"{self.name}: {x}" for x in p])
        return self

    # -- packed tables -----------------------------------------------------
    def begin(self, trick_id: int) -> TrickHeader:
        """The packed header. exp_roll/exp_pitch are the balance reference for
        frames with the balance bit (whole degrees, legacy convention)."""
        return TrickHeader(trick_id=trick_id, n_frames=len(self.frames),
                           loop_start=self.loop_start, loop_end=self.loop_end,
                           repeats=self.repeats, joint_mask=self.joint_mask,
                           flags=self.flags, end_posture=int(self.end_posture),
                           exp_roll=max(-127, min(127, _round(self.expected_roll))),
                           exp_pitch=max(-127, min(127, _round(self.expected_pitch))))

    def encode_frames(self) -> bytes:
        """The n_frames x 14 B frame array (what crc16 covers)."""
        return b"".join(f.encode() for f in self.frames)

    def crc16(self) -> int:
        return C.crc16(self.encode_frames())

    @classmethod
    def from_wire(cls, begin: TrickHeader, frames: bytes | Sequence[WireKeyframe],
                  name: str = "", **kw) -> "Trick":
        """Rebuild from a header and the frame array (bytes or WireKeyframes).
        Expected roll/pitch come from the header (whole degrees) unless
        given in `kw`."""
        if isinstance(frames, (bytes, bytearray)):
            if len(frames) != begin.n_frames * KEYFRAME_SIZE:
                raise ValueError(f"frame array is {len(frames)} B, want {begin.n_frames} x 14")
            kfs = [Keyframe.decode(frames[i:i + 14]) for i in range(0, len(frames), 14)]
        else:
            kfs = [Keyframe.from_wire(f) for f in frames]
        if len(kfs) != begin.n_frames:
            raise ValueError(f"{len(kfs)} frames, header says {begin.n_frames}")
        kw.setdefault("expected_roll", begin.exp_roll)
        kw.setdefault("expected_pitch", begin.exp_pitch)
        return cls(name=name, frames=tuple(kfs), loop_start=begin.loop_start,
                   loop_end=begin.loop_end, repeats=begin.repeats,
                   balance=bool(begin.flags & TrickFlags.BALANCE),
                   fall_ok=bool(begin.flags & TrickFlags.FALL_OK),
                   end_posture=Posture(begin.end_posture), **kw)

    # -- JSON --------------------------------------------------------------
    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "source": self.source,
            "notes": list(self.notes),
            "expected": {"roll": _num(self.expected_roll), "pitch": _num(self.expected_pitch)},
            "loop": {"start": self.loop_start, "end": self.loop_end, "repeats": self.repeats},
            "balance": self.balance,
            "fall_ok": self.fall_ok,
            "end_posture": self.end_posture.name,
            **({"stop_ticks": self.stop_ticks} if self.stop_ticks else {}),
            **({"guards": [g.to_dict() for g in self.guards]} if self.guards else {}),
            **({"roll_max": _num(self.roll_max)} if self.roll_max else {}),
            "frames": [f.to_dict() for f in self.frames],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Trick":
        loop = d.get("loop", {})
        exp = d.get("expected", {})
        return cls(name=d["name"], frames=tuple(Keyframe.from_dict(f) for f in d["frames"]),
                   loop_start=loop.get("start", 0), loop_end=loop.get("end", 0),
                   repeats=loop.get("repeats", 0),
                   expected_roll=exp.get("roll", 0), expected_pitch=exp.get("pitch", 0),
                   balance=d.get("balance", False), fall_ok=d.get("fall_ok", False),
                   end_posture=Posture[d.get("end_posture", "STAND")],
                   stop_ticks=d.get("stop_ticks", 0),
                   guards=tuple(Guard.from_dict(g) for g in d.get("guards", ())), roll_max=d.get("roll_max", 0),
                   source=d.get("source", {}), notes=tuple(d.get("notes", ())))

    def to_json(self) -> str:
        """Stable, diff-friendly JSON: one line per keyframe."""
        d = self.to_dict()
        frames = d.pop("frames")
        head = json.dumps(d, indent=2)
        body = ",\n".join("    " + json.dumps(f) for f in frames)
        return head[:-2] + ',\n  "frames": [\n' + body + "\n  ]\n}\n"

    @classmethod
    def from_json(cls, text: str) -> "Trick":
        return cls.from_dict(json.loads(text))


def pose(angles: Iterable) -> tuple:
    """16-slot tuple from a 16-value legacy row, dropping the unused slots."""
    a = list(angles)
    return tuple(None if j in UNUSED else a[j] for j in range(JOINT_SLOTS))


def trick_id_for(trick: Trick) -> int:
    """A stable 1..254 tag for a trick's header and frames (the kernel's
    tables carry it beside the CRC)."""
    data = trick.begin(0).pack()[1:] + trick.encode_frames()
    return binascii.crc_hqx(data, 0xFFFF) % 254 + 1


# ---- the timing of a play at a speed ----------------------------------------------------------
TICK_S = P.TICK_MS / 1000


def phase_ticks(n: int, speed_q6: int = SPEED_UNITY_Q6) -> int:
    """Ticks a phase (a move or a hold) of ``n`` ticks takes at ``speed_q6``: ceil(n * 64 / speed)."""
    return (n * 64 + speed_q6 - 1) // speed_q6


def play_order(n: int, loop_start: int, loop_end: int, repeats: int, max_plays: int | None = None) -> list[int]:
    """Frame indices in play order: after ``loop_end`` back to ``loop_start``
    until that section was played ``repeats`` times in total (0 and 1 =
    once; 255, until stopped, is cut at ``max_plays``)."""
    plays = max(repeats, 1) if repeats != REPEAT_FOREVER else (max_plays or 1)
    order = list(range(0, loop_end + 1))
    for _ in range(plays - 1):
        order += list(range(loop_start, loop_end + 1))
    return order + list(range(loop_end + 1, n))


# ---- the mind's guards (Guard, Trick.roll_max) ------------------------------------------------
GUARD_EARLY_TICKS = 2  # a guard decides this many ticks before its hold ends


@dataclass
class GuardWatch:
    """What the mind checks while a trick plays: fed every TELEMETRY as
    (seconds since the play began, nose up deg, roll deg), it returns why
    the trick must be stopped (once), else None.

    - Each Guard: over the second half of its frame's hold (at the play's
      speed, phase_ticks) the nose-up readings must average inside the
      window and every |roll| stay within roll_max; decided
      GUARD_EARLY_TICKS before the hold ends. No reading there: stop.
    - Trick.roll_max: |roll| past it on two readings in a row: stop.
    - A Guard's push_roll: from the start of its frame's move to its
      window, the roll moved more than that from the last reading before
      the move, on two readings in a row: stop.
    kernel/mind/tricks/model.cpp GuardWatch does the same; the same cases
    test both (mind/tests/test_rear_up.py test_guard_watch,
    kernel/host/tests/test_tricks.cpp test_guard_watch)."""
    trick: object
    speed: int = SPEED_UNITY_Q6
    windows: list = field(default_factory=list, init=False)  # (guard, t_from, t_decide)
    pushes: list = field(default_factory=list, init=False)   # (guard index, t_move): where push_roll watches
    roll0: dict = field(default_factory=dict, init=False)    # guard index -> the roll before its move
    push_over: dict = field(default_factory=dict, init=False)
    sums: dict = field(default_factory=dict, init=False)     # guard index -> [sum, n]
    done: set = field(default_factory=set, init=False)
    over: int = field(default=0, init=False)
    fired: str | None = field(default=None, init=False)

    def __post_init__(self):
        t = self.trick
        frames = t.frames
        acc = 0
        order = play_order(len(frames), t.loop_start, t.loop_end, t.repeats, max_plays=1)
        first_hold, first_move = {}, {}
        for f in order:
            if f not in first_move:
                first_move[f] = acc
            acc += phase_ticks(frames[f].dur, self.speed)
            if f not in first_hold:
                first_hold[f] = acc
            acc += phase_ticks(frames[f].hold, self.speed)
        for i, g in enumerate(t.guards):
            h0 = first_hold[g.frame]
            h = phase_ticks(frames[g.frame].hold, self.speed)
            self.windows.append((g, (h0 + h / 2) * TICK_S, (h0 + h - GUARD_EARLY_TICKS) * TICK_S))
            self.sums[i] = [0.0, 0]
            if g.push_roll:
                self.pushes.append((i, first_move[g.frame] * TICK_S))
                self.push_over[i] = 0

    def feed(self, t_s: float, nose_up: float, roll: float) -> str | None:
        if self.fired is not None:
            return None
        rm = getattr(self.trick, "roll_max", 0) or 0
        self.over = self.over + 1 if rm and abs(roll) > rm else 0
        if self.over >= 2:
            return self._fire(f"roll {roll:+.1f} deg past {rm:g}")
        for i, t_move in self.pushes:
            g, t_from, _ = self.windows[i]
            if t_s < t_move:
                self.roll0[i] = roll
            elif t_s < t_from:
                r0 = self.roll0.setdefault(i, roll)
                self.push_over[i] = self.push_over[i] + 1 if abs(roll - r0) > g.push_roll else 0
                if self.push_over[i] >= 2:
                    return self._fire(f"guard {g.name or g.frame}: roll {roll:+.1f} deg, {roll - r0:+.1f} while "
                                      f"it moved (limit {g.push_roll:g})")
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
