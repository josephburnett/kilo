"""Trick model: keyframes, mirror, speed, validation and the wire encoding.

A trick is a list of keyframes the body plays in joint space (§3e). The Pi
owns the library and uploads a trick into the body's 24-frame RAM store with
TRICK_BEGIN / TRICK_FRAMES / TRICK_COMMIT; the bytes are exactly the
generated `kilo_protocol.Keyframe` (14 B) and `TrickBegin` structs.

Joints use the legacy 16-slot numbering (0 head pan, 1 head tilt, 2 tail,
3-7 unused, 8-11 shoulders LF RF RB LB, 12-15 knees LF RF RB LB). On the wire
a frame carries the 11 driven joints in canonical order, and -128 means
"unchanged" (None here). `joint_mask` is a u16 over the same 16 slots.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, replace
from typing import Iterable, Sequence

from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P

Easing = P.Easing
Posture = P.Posture

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

MAX_FRAMES = P.TRICK_CAPACITY            # 24 (32 before the P1 RAM shrink, docs §3h)
FRAMES_PER_MSG = P.TRICK_FRAMES_PER_MSG  # 4
TICK_MS = P.TICK_MS                      # 50 Hz body tick
REPEAT_FOREVER = 255


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
    at most ``roll_max``; otherwise the mind sends TRICK_STOP (over the
    trick's stop_ticks, down to its end posture). The frame's hold must be
    at least GUARD_MIN_HOLD ticks. ``name`` is for the logs."""
    frame: int
    nose_up: tuple
    roll_max: float
    name: str = ""

    def to_dict(self) -> dict:
        return {"frame": self.frame, "name": self.name, "nose_up": [_num(v) for v in self.nose_up],
                "roll_max": _num(self.roll_max)}

    @classmethod
    def from_dict(cls, d: dict) -> "Guard":
        return cls(frame=d["frame"], nose_up=tuple(d["nose_up"]), roll_max=d["roll_max"], name=d.get("name", ""))


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
        return int(self.easing) | (int(P.KeyframeFlags.BALANCE) if self.balance else 0)

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
    def to_wire(self) -> P.Keyframe:
        ang = [ANGLE_UNCHANGED if self.angles[j] is None else _round(self.angles[j]) for j in CANONICAL]
        return P.Keyframe(dur=self.dur, hold=self.hold, flags=self.flags, ang=ang)

    def encode(self) -> bytes:
        """The 14 B on-wire frame (dur, hold, flags, ang[11])."""
        return self.to_wire().pack()

    @classmethod
    def from_wire(cls, kf: P.Keyframe) -> "Keyframe":
        a: list = [None] * JOINT_SLOTS
        for j, v in zip(CANONICAL, kf.ang):
            a[j] = None if v == ANGLE_UNCHANGED else v
        return cls(tuple(a), dur=kf.dur, hold=kf.hold, easing=Easing(kf.flags & 0x03),
                   balance=bool(kf.flags & P.KeyframeFlags.BALANCE))

    @classmethod
    def decode(cls, data: bytes) -> "Keyframe":
        return cls.from_wire(P.Keyframe.unpack(data))

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
    # Mind-side only (not on the wire): the transition_ticks a TRICK_STOP of
    # this trick asks for (0 = the body's XFADE_TICKS, 0.2 s). A trick that
    # must come down slowly when stopped (rear_up) sets it; the remote
    # (kernel/mind/remote) and BodyClient.play() use it.
    stop_ticks: int = 0
    # Mind-side only: checks of the measured attitude while it plays (Guard),
    # and a roll limit for the whole play (deg, 0 = none): past it on two
    # TELEMETRY in a row the mind sends TRICK_STOP. kilo.body.client.play()
    # and kernel/mind/tricks/player.cpp apply both.
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
        return ((int(P.TrickFlags.BALANCE) if self.balance else 0)
                | (int(P.TrickFlags.FALL_OK) if self.fall_ok else 0))

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

    # -- wire --------------------------------------------------------------
    def begin(self, trick_id: int) -> P.TrickBegin:
        """TRICK_BEGIN. exp_roll/exp_pitch are the balance reference for
        frames with the balance bit (whole degrees, legacy convention)."""
        return P.TrickBegin(trick_id=trick_id, n_frames=len(self.frames),
                            loop_start=self.loop_start, loop_end=self.loop_end,
                            repeats=self.repeats, joint_mask=self.joint_mask,
                            flags=self.flags, end_posture=int(self.end_posture),
                            exp_roll=max(-127, min(127, _round(self.expected_roll))),
                            exp_pitch=max(-127, min(127, _round(self.expected_pitch))))

    def encode_frames(self) -> bytes:
        """The n_frames x 14 B frame array (what TRICK_COMMIT's CRC covers)."""
        return b"".join(f.encode() for f in self.frames)

    def crc16(self) -> int:
        return C.crc16(self.encode_frames())

    def frame_messages(self) -> list[P.TrickFrames]:
        wire = [f.to_wire() for f in self.frames]
        return [P.TrickFrames(first=i, frames=wire[i:i + FRAMES_PER_MSG])
                for i in range(0, len(wire), FRAMES_PER_MSG)]

    def commit(self, trick_id: int) -> P.TrickCommit:
        return P.TrickCommit(trick_id=trick_id, crc16=self.crc16())

    def upload_messages(self, trick_id: int) -> list:
        """TRICK_BEGIN, the TRICK_FRAMES chunks and TRICK_COMMIT, validated."""
        self.validate()
        return [self.begin(trick_id), *self.frame_messages(), self.commit(trick_id)]

    @classmethod
    def from_wire(cls, begin: P.TrickBegin, frames: bytes | Sequence[P.Keyframe],
                  name: str = "", **kw) -> "Trick":
        """Rebuild from a TRICK_BEGIN and the frame array (bytes or Keyframes).
        Expected roll/pitch come from the header (whole degrees) unless
        given in `kw`."""
        if isinstance(frames, (bytes, bytearray)):
            if len(frames) != begin.n_frames * P.Keyframe.SIZE:
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
                   balance=bool(begin.flags & P.TrickFlags.BALANCE),
                   fall_ok=bool(begin.flags & P.TrickFlags.FALL_OK),
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
