"""The flight recorder's data (kernel/mind/record/, protocol/spec.toml [host]
"The flight recorder"): read, split into runs, write.

From boot the mind records, in RAM, every body link frame both ways (all
TELEMETRY, EVENTs, JOINTS; FEET, postures, trick commands; HEARTBEATs only
counted), every log line, and during a walk or a trick what the planner or
the trick player did. It keeps this as BLOCKS of host.record_block_bytes
(16 KiB): a 36-byte header (``P.RecordBlockHeader``: magic "KFR1", the
boot's number, the block's number, its size, the mind's clock at its start,
a CRC-32) and records (``P.WalkRecordHeader``: kind, length, microseconds
since the block's start, then the payload). What the laptop fetches
(``kilo runs --fetch``, ``kilo record fetch``) is these blocks back to back,
exactly as the mind holds them.

    s = flight.load("run-20261116-101500-b7-3.krec")   # or parse_blocks(raw bytes)
    s.runs                                        # [RunSpan]: the walks and tricks in it
    s.frames, s.marks, s.logs, s.plans            # every record, by kind, mind time in us
    w = flight.walk_of(s, s.runs[0])               # a walkrec.Walk: kilo log walk's analysis
    s2 = s.between(t_from_us, t_to_us)            # a time slice

Record kinds (``P.WalkRecord``): RX/TX a body link frame (type, seq,
payload), PLAN/PLANNER a walk's planned tick and the planner's counters,
MARK a text line ("word key=value ..."), LOG one line of the mind's log
(level, text), RUN the start or end of a run (``P.RunMark``: id, kind
walk/trick, begin/end).

The file this writes (``.krec``): magic ``b"KILOREC\\n"``, u16 LE meta
length, meta (UTF-8 JSON: what was fetched, from which mind and when, the
mind's uptime then, ``start_wall``), then the raw block bytes.

Times: every record's time is the mind's clock in microseconds (the block's
t0 + the record's t_us); ``uptime_s`` subtracts the mind's start, which the
meta's ``mind_t0_ms`` gives when known (the clock in ms, modulo 2^32 as
REC_STATUS.clock_ms gives it; else 0: the Pi's clock starts near 0). A
``start_wall`` in the meta maps them to this laptop's clock.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from kilo.logs import walkrec
from kilo.protocol import kilo_protocol as P

FILE_MAGIC = b"KILOREC\n"
BLOCK_MAGIC = P.HOST_RECORD_BLOCK_MAGIC
BLOCK_BYTES = P.HOST_RECORD_BLOCK_BYTES
HDR = P.RecordBlockHeader.SIZE
_REC = struct.Struct("<BBI")
PREROLL_US = P.HOST_WALK_PREROLL_MS * 1000
POSTROLL_US = P.HOST_WALK_POSTROLL_MS * 1000
LEVEL_TAG = "DIWE"
_MAGIC_BYTES = struct.pack("<I", BLOCK_MAGIC)


def is_blocks(data: bytes) -> bool:
    """Whether data starts with a flight-recorder block (host.walk_format 2)."""
    return len(data) >= 4 and data[:4] == _MAGIC_BYTES


# ---- what is in it -----------------------------------------------------------------------

@dataclass
class Block:
    seq: int
    boot: int
    t0_us: int
    used: int
    records: int
    skipped: int
    crc32: int
    offset: int          # where it starts in the data
    have: int            # bytes of it in the data (less than used: cut short)
    problem: str = ""    # "" = good; else why it was skipped (or "cut": read as far as it goes)


@dataclass
class LogRec:
    t_us: int
    level: int
    text: str

    @property
    def tag(self) -> str:
        return LEVEL_TAG[self.level] if self.level < len(LEVEL_TAG) else "?"


@dataclass
class RunSpan:
    id: int
    kind: str            # "walk" or "trick"
    begin_us: int | None  # None: its start is not in the data
    end_us: int | None    # None: still running (or its end is not in the data)

    @property
    def label(self) -> str:
        return f"{self.kind} {self.id}"


@dataclass
class Stream:
    meta: dict[str, Any] = field(default_factory=dict)
    blocks: list[Block] = field(default_factory=list)
    frames: list[walkrec.Frame] = field(default_factory=list)
    marks: list[walkrec.Mark] = field(default_factory=list)
    logs: list[LogRec] = field(default_factory=list)
    plans: list[tuple[int, Any]] = field(default_factory=list)
    planner: list[tuple[int, Any]] = field(default_factory=list)
    run_marks: list[tuple[int, Any]] = field(default_factory=list)  # (t_us, P.RunMark)
    bad_records: int = 0
    garbage: int = 0     # bytes between blocks that were not a block

    @property
    def bad_blocks(self) -> list[Block]:
        return [b for b in self.blocks if b.problem and b.problem != "cut"]

    @property
    def skipped(self) -> int:
        return sum(b.skipped for b in self.blocks if not b.problem or b.problem == "cut")

    @property
    def t_first_us(self) -> int | None:
        ts = [b.t0_us for b in self.blocks if not b.problem or b.problem == "cut"]
        return min(ts) if ts else None

    @property
    def t_last_us(self) -> int | None:
        ts = [x for x in (self.frames[-1].t_us if self.frames else None, self.marks[-1].t_us if self.marks else None,
                          self.logs[-1].t_us if self.logs else None) if x is not None]
        return max(ts) if ts else None

    @property
    def mind_t0_ms(self) -> int:
        """The mind's clock at its start, ms mod 2^32 (for uptimes): the
        meta's, else 0 (the Circle kernel's clock starts near 0)."""
        return int(self.meta.get("mind_t0_ms", 0))

    def uptime_s(self, t_us: int) -> float:
        """Seconds since the mind started (the ms count wraps as REC_STATUS's)."""
        ms = ((t_us // 1000) - self.mind_t0_ms) & 0xFFFFFFFF
        return ms / 1000 + (t_us % 1000) / 1e6

    @property
    def runs(self) -> list[RunSpan]:
        out: dict[int, RunSpan] = {}
        order: list[int] = []
        for t, r in self.run_marks:
            kind = "trick" if r.kind == P.RunKind.TRICK else "walk"
            if r.id not in out:
                out[r.id] = RunSpan(r.id, kind, None, None)
                order.append(r.id)
            if r.event == P.RunEvent.BEGIN:
                out[r.id].begin_us = t
            elif r.event == P.RunEvent.END:
                out[r.id].end_us = t
        return [out[i] for i in order]

    def run(self, run_id: int) -> RunSpan | None:
        return next((r for r in self.runs if r.id == run_id), None)

    def between(self, t_from: int | None, t_to: int | None) -> "Stream":
        """The records with t_from <= t <= t_to (None: no bound)."""
        lo = -1 << 62 if t_from is None else t_from
        hi = 1 << 62 if t_to is None else t_to

        def inside(t: int) -> bool:
            return lo <= t <= hi
        s = Stream(meta=dict(self.meta), blocks=list(self.blocks), bad_records=self.bad_records, garbage=self.garbage)
        s.frames = [f for f in self.frames if inside(f.t_us)]
        s.marks = [m for m in self.marks if inside(m.t_us)]
        s.logs = [g for g in self.logs if inside(g.t_us)]
        s.plans = [p for p in self.plans if inside(p[0])]
        s.planner = [p for p in self.planner if inside(p[0])]
        s.run_marks = [r for r in self.run_marks if inside(r[0])]
        return s

    def telemetry(self) -> list[tuple[int, Any]]:
        return [(f.t_us, f.msg) for f in self.frames
                if f.direction == "rx" and isinstance(f.msg, P.Telemetry)]


def _parse_block_records(s: Stream, body: bytes, t0: int) -> None:
    i, n = 0, len(body)
    while i + _REC.size <= n:
        kind, ln, t = _REC.unpack_from(body, i)
        payload = body[i + _REC.size:i + _REC.size + ln]
        if len(payload) < ln:
            s.bad_records += 1  # cut off: the block was still being written
            return
        i += _REC.size + ln
        ta = t0 + t
        try:
            if kind in (P.WalkRecord.RX, P.WalkRecord.TX):
                typ, seq, data = payload[0], payload[1], bytes(payload[2:])
                try:
                    msg = P.decode_payload(typ, data)
                except (KeyError, ValueError, P.PayloadError):
                    msg = None
                s.frames.append(walkrec.Frame(ta, "rx" if kind == P.WalkRecord.RX else "tx", typ, seq, data, msg))
            elif kind == P.WalkRecord.PLAN:
                s.plans.append((ta, P.WalkPlan.unpack(payload)))
            elif kind == P.WalkRecord.PLANNER:
                s.planner.append((ta, P.WalkPlanner.unpack(payload)))
            elif kind == P.WalkRecord.MARK:
                text = payload.decode("ascii", "replace")
                word, fields = parse_mark(text)
                s.marks.append(walkrec.Mark(ta, word, fields))
            elif kind == P.WalkRecord.LOG:
                s.logs.append(LogRec(ta, payload[0], payload[1:].decode("utf-8", "replace")))
            elif kind == P.WalkRecord.RUN:
                s.run_marks.append((ta, P.RunMark.unpack(payload)))
            else:
                s.bad_records += 1
        except (IndexError, P.PayloadError, struct.error):
            s.bad_records += 1


# MARK words whose last field takes the rest of the line (it may hold spaces).
TAIL_KEYS = ("reason", "why", "text")


def parse_mark(text: str) -> tuple[str, dict[str, str]]:
    """'end reason=FELL elapsed_ms=900 why=the body fell over' -> ('end', {...}):
    ``why`` and ``text`` take the rest of the line; ``reason`` does too unless
    a ``key=`` follows it (a walk's end puts reason last, a trick's first).
    A ``key`` MARK keeps its whole text as ``text``."""
    word, _, rest = text.partition(" ")
    if word == "key":
        return word, {"text": rest}
    out: dict[str, str] = {}
    while rest:
        rest = rest.lstrip(" ")
        key, eq, tail = rest.partition("=")
        if not eq or " " in key:
            break
        if key in ("why", "text"):
            out[key] = tail
            break
        value, _, rest = tail.partition(" ")
        if key == "reason":
            nxt = rest.lstrip(" ").partition(" ")[0]
            if "=" not in nxt and rest.strip():  # a walk's reason: the rest of the line
                out[key] = tail
                break
        out[key] = value
    return word, out


def parse_blocks(data: bytes, meta: dict[str, Any] | None = None) -> Stream:
    """Blocks back to back -> a Stream. A block whose CRC fails or whose
    header is broken is skipped (listed in ``bad_blocks``); a last block cut
    short is read as far as it goes (``problem`` "cut"); bytes that are not
    a block are skipped up to the next block magic (``garbage``)."""
    s = Stream(meta=dict(meta or {}))
    i, n = 0, len(data)
    while i + HDR <= n:
        if data[i:i + 4] != _MAGIC_BYTES:
            j = data.find(_MAGIC_BYTES, i + 1)
            j = n if j < 0 else j
            s.garbage += j - i
            i = j
            continue
        h = P.RecordBlockHeader.unpack(data[i:i + HDR])
        t0 = (h.t0_us_hi << 32) | h.t0_us_lo
        b = Block(h.seq, h.boot, t0, h.used, h.records, h.skipped, h.crc32, i, min(h.used, n - i))
        s.blocks.append(b)
        if h.used < HDR or h.used > BLOCK_BYTES:
            b.problem = f"bad size {h.used}"
            j = data.find(_MAGIC_BYTES, i + 4)
            j = n if j < 0 else j
            s.garbage += j - i - HDR
            i = j
            continue
        body = data[i + HDR:i + h.used]
        if len(body) < h.used - HDR:
            b.problem = "cut"
        elif h.crc32 != 0 and zlib.crc32(body) != h.crc32:
            b.problem = "CRC mismatch"
            i += h.used
            continue
        _parse_block_records(s, body, t0)
        i += h.used
    if 0 < n - i < HDR and i < n:
        s.garbage += n - i
    key = (lambda x: x.t_us)
    s.frames.sort(key=key)
    s.marks.sort(key=key)
    s.logs.sort(key=key)
    s.plans.sort(key=lambda x: x[0])
    s.planner.sort(key=lambda x: x[0])
    s.run_marks.sort(key=lambda x: x[0])
    return s


# ---- runs --------------------------------------------------------------------------------

def span_of(s: Stream, run: RunSpan) -> tuple[int, int]:
    """The run's records: from host.walk_preroll_ms before its start to
    host.walk_postroll_ms after its end (the data's own ends where those
    are missing)."""
    first = s.t_first_us or 0
    last = s.t_last_us or first
    lo = (run.begin_us - PREROLL_US) if run.begin_us is not None else first
    hi = (run.end_us + POSTROLL_US) if run.end_us is not None else last
    return max(lo, first), hi


def walk_of(s: Stream, run: RunSpan | None = None) -> walkrec.Walk:
    """A walk run as a walkrec.Walk (the kilo log walk analysis), its times
    counting from the start of its span. run: the first walk if None."""
    if run is None:
        run = next((r for r in s.runs if r.kind == "walk"), None)
        if run is None:
            raise ValueError("no walk in this recording")
    lo, hi = span_of(s, run)
    part = s.between(lo, hi)
    # only this run's own MARK lines (another run's could sit in the pre- or post-roll)
    begin = run.begin_us if run.begin_us is not None else lo
    end = run.end_us if run.end_us is not None else hi
    w = walkrec.Walk(meta=dict(s.meta))
    w.meta.setdefault("run", run.id)
    w.frames = [walkrec.Frame(f.t_us - lo, f.direction, f.type, f.seq, f.payload, f.msg) for f in part.frames]
    w.plans = [(t - lo, p) for t, p in part.plans]
    w.planner = [(t - lo, p) for t, p in part.planner]
    w.marks = [walkrec.Mark(m.t_us - lo, m.word, m.fields) for m in part.marks if begin <= m.t_us <= end]
    w.bad = s.bad_records + len(s.bad_blocks)
    return w


# ---- the file ----------------------------------------------------------------------------

def write_file(path: str | Path, meta: dict[str, Any], blocks: bytes) -> Path:
    return walkrec.write_container(path, FILE_MAGIC, meta, blocks, "krec")


def is_krec(path: str | Path) -> bool:
    return walkrec.has_magic(path, FILE_MAGIC)


def read_file(path: str | Path) -> tuple[dict[str, Any], bytes]:
    data = Path(path).read_bytes()
    got = walkrec.split_container(data, FILE_MAGIC)
    if got is not None:
        return got
    if is_blocks(data):  # raw blocks (e.g. copied off the SD card's kilorec/ folder)
        return {}, data
    raise ValueError(f"{path}: not a flight recording (.krec, or raw blocks)")


def load(path: str | Path) -> Stream:
    """A .krec, a .kwalk holding format-2 blocks, or raw blocks (a card file)."""
    p = Path(path)
    if walkrec.is_kwalk(p):
        meta, data = walkrec.read_file(p)
    else:
        meta, data = read_file(p)
    s = parse_blocks(data, meta)
    s.meta.setdefault("path", str(p))
    return s
