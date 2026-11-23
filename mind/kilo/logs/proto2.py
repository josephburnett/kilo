"""Protocol 2 recordings: reading the frames of recordings made before protocol 3.0.

The recordings on the laptop (~/kilo-logs, the evidence of
docs/movement-engine-results.md) were made with protocol 2: the mind
streamed FEET (type 0x11, 16 B) where it now streams POSE (0x11, 31 B), and
the body kept a trick store (TRICK_BEGIN .. TRICK_STOP, 0x20..0x24). This
module is the one place that knows those layouts and numbers, for the
readers of old data files (kilo.logs, kilo.intent.replay, the study scripts);
nothing that talks to a body uses it.

- ``decode_payload(type, payload)``: as ``P.decode_payload``, but type 0x11
  is decoded by its length (16 B: ``Feet``, the old layout; 31 B: P.Pose),
  and 0x20..0x24 as the old trick commands (TRICK_FRAMES as None).
- ``mode_name`` / ``event_name``: names for TELEMETRY modes and EVENT codes
  in either protocol. Numbers protocol 3 kept mean the same (mode 3 GAIT is
  STREAM, event 12 GAIT_STOPPED is STREAM_STOPPED, 19 FEET_UNDERRUN is
  POSE_UNDERRUN, 4 RECOVERY_FAILED is LYING: the fall ended lying, REST,
  then LIMP); the ones it dropped are mode 4 TRICK and 6 RECOVERING, event
  3 RECOVERED and 11 TRICK_DONE.
- ``feet_of(msg)``: the feet (x, z in mm, per leg LF RF RB LB) of a FEET or
  of a POSE in foot mode.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Any, ClassVar

from kilo.protocol import kilo_protocol as P

FEET_TYPE = 0x11
FEET_LEN = 16
OLD_MODES = {4: "TRICK", 6: "RECOVERING"}
OLD_EVENTS = {3: "RECOVERED", 11: "TRICK_DONE"}
MODE_TRICK, MODE_RECOVERING = 4, 6
EVENT_RECOVERED, EVENT_TRICK_DONE = 3, 11


@dataclass
class Feet:
    """0x11 FEET (protocol 2): one tick of the walk's plan."""
    NAME: ClassVar[str] = "FEET"
    _FMT: ClassVar[struct.Struct] = struct.Struct("<BHbbbbBBBBhhB")
    epoch: int = 0
    tick: int = 0
    x_mm: list = field(default_factory=lambda: [0] * 4)  # mm, legs LF RF RB LB
    z_mm: list = field(default_factory=lambda: [0] * 4)
    pitch: int = 0  # a16
    roll: int = 0
    support: int = 0

    @classmethod
    def unpack(cls, data: bytes) -> "Feet":
        v = cls._FMT.unpack(bytes(data))
        return cls(v[0], v[1], list(v[2:6]), list(v[6:10]), v[10], v[11], v[12])


@dataclass
class TrickBegin:
    """0x20 TRICK_BEGIN (protocol 2): the trick header."""
    NAME: ClassVar[str] = "TRICK_BEGIN"
    _FMT: ClassVar[struct.Struct] = struct.Struct("<BBBBBHBBbb")
    trick_id: int = 0
    n_frames: int = 0
    loop_start: int = 0
    loop_end: int = 0
    repeats: int = 0
    joint_mask: int = 0
    flags: int = 0
    end_posture: int = 0
    exp_roll: int = 0
    exp_pitch: int = 0

    @classmethod
    def unpack(cls, data: bytes) -> "TrickBegin":
        return cls(*cls._FMT.unpack(bytes(data)))


@dataclass
class TrickCommit:
    """0x22 TRICK_COMMIT (protocol 2)."""
    NAME: ClassVar[str] = "TRICK_COMMIT"
    _FMT: ClassVar[struct.Struct] = struct.Struct("<BH")
    trick_id: int = 0
    crc16: int = 0

    @classmethod
    def unpack(cls, data: bytes) -> "TrickCommit":
        return cls(*cls._FMT.unpack(bytes(data)))


@dataclass
class TrickPlay:
    """0x23 TRICK_PLAY (protocol 2): speed_q6 64 = 1x, flags bit 0 mirror."""
    NAME: ClassVar[str] = "TRICK_PLAY"
    _FMT: ClassVar[struct.Struct] = struct.Struct("<BBBB")
    epoch: int = 0
    trick_id: int = 0
    speed_q6: int = 0
    flags: int = 0

    @classmethod
    def unpack(cls, data: bytes) -> "TrickPlay":
        return cls(*cls._FMT.unpack(bytes(data)))


@dataclass
class TrickStop:
    """0x24 TRICK_STOP (protocol 2)."""
    NAME: ClassVar[str] = "TRICK_STOP"
    _FMT: ClassVar[struct.Struct] = struct.Struct("<B")
    transition_ticks: int = 0

    @classmethod
    def unpack(cls, data: bytes) -> "TrickStop":
        return cls(*cls._FMT.unpack(bytes(data)))


OLD_TYPES: dict[int, Any] = {0x20: TrickBegin, 0x21: None, 0x22: TrickCommit, 0x23: TrickPlay, 0x24: TrickStop}
OLD_NAMES = {0x11: "FEET", 0x20: "TRICK_BEGIN", 0x21: "TRICK_FRAMES", 0x22: "TRICK_COMMIT", 0x23: "TRICK_PLAY",
             0x24: "TRICK_STOP"}


def decode_payload(msg_type: int, payload: bytes) -> Any:
    """A recorded frame's payload as its message, in either protocol. Raises
    KeyError (unknown type), P.PayloadError or struct.error (bad length) as
    P.decode_payload does; TRICK_FRAMES gives None."""
    if msg_type == FEET_TYPE and len(payload) == FEET_LEN:
        return Feet.unpack(payload)
    if msg_type in OLD_TYPES:
        cls = OLD_TYPES[msg_type]
        return None if cls is None else cls.unpack(payload)
    return P.decode_payload(msg_type, payload)


def type_name(msg_type: int, payload: bytes = b"") -> str:
    """A recorded frame's type name (FEET for a 16 B 0x11)."""
    if msg_type == FEET_TYPE and len(payload) == FEET_LEN:
        return "FEET"
    if msg_type in OLD_NAMES and msg_type != FEET_TYPE:
        return OLD_NAMES[msg_type]
    try:
        return P.MsgId(msg_type).name
    except ValueError:
        return f"0x{msg_type:02X}"


def mode_name(v: int) -> str:
    """A TELEMETRY mode's name, in either protocol."""
    if v in P.Mode._value2member_map_:
        return P.Mode(v).name
    return OLD_MODES.get(v, str(v))


def event_name(code: int) -> str:
    """An EVENT code's name, in either protocol."""
    if code in P.EventCode._value2member_map_:
        return P.EventCode(code).name
    return OLD_EVENTS.get(code, str(code))


def is_proto2(frames: list) -> bool:
    """Whether recorded frames (walkrec.Frame) come from a protocol 2 mind and
    body: a HELLO of major 2, a FEET or a trick command among them."""
    for f in frames:
        if f.direction == "tx" and (f.type in OLD_TYPES or (f.type == FEET_TYPE and len(f.payload) == FEET_LEN)):
            return True
        if f.direction == "rx" and isinstance(f.msg, P.Hello):
            return f.msg.proto_major < 3
    return False


def body_keyframe(m: Any, old: bool) -> int | None:
    """The keyframe the body said it played (protocol 2 TELEMETRY.trick_frame,
    now TELEMETRY.limited's byte; 0xFF none); None in protocol 3."""
    return None if not old or m.limited == 0xFF else m.limited


def is_stream(msg: Any) -> bool:
    """The mind's stream frame of either protocol (FEET or POSE)."""
    return isinstance(msg, (Feet, P.Pose))


def feet_of(msg: Any) -> tuple[list[float], list[float]] | None:
    """(x, z) in mm per leg (LF RF RB LB) of a FEET, or of a POSE's foot-mode
    legs (None for a joint-mode leg); None for anything else."""
    if isinstance(msg, Feet):
        return [float(v) for v in msg.x_mm], [float(v) for v in msg.z_mm]
    if isinstance(msg, P.Pose):
        xs, zs = [], []
        for i in range(4):
            joint = msg.flags >> i & 1
            xs.append(None if joint else msg.leg[2 * i] / 4.0)
            zs.append(None if joint else msg.leg[2 * i + 1] / 4.0)
        return xs, zs
    return None
