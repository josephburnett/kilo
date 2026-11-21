"""The flight recorder on the laptop: kilo.logs.flight (blocks, runs, the
.krec file), kilo.logs.trickrec (a trick against its design's prediction,
the tip-over onset), kilo.logs.decode (the readable rows), and the CLI
over a fake mind (kilo runs, record, bench sd, metrics, status, log).
The blocks here are built byte for byte as protocol/spec.toml [host] "The
flight recorder" says; the kernel's own are checked in kernel/host/test.sh."""

from __future__ import annotations

import csv
import json
import math
import socket
import struct
import zlib
from typing import Any

import pytest

from kilo import cli
from kilo.logs import decode, flight, trickrec, walkrec
from kilo.protocol import kilo_protocol as P
from kilo.tricks import library
from kilo.tricks.player import phase_ticks
from test_hostlink import FakeMind
from test_walks import synthetic

T0 = 5_000_000  # the mind's clock at the first block, us
TEL_US = 20000


# ---- building blocks as the mind does --------------------------------------------------

def rec(kind: int, t_us: int, payload: bytes) -> tuple[int, int, bytes]:
    return (t_us, kind, payload)


def frame(kind: int, t_us: int, msg: Any, seq: int = 0) -> tuple[int, int, bytes]:
    return rec(kind, t_us, bytes([msg.ID, seq & 0xFF]) + msg.pack())


def mark(t_us: int, text: str) -> tuple[int, int, bytes]:
    return rec(P.WalkRecord.MARK, t_us, text.encode())


def log(t_us: int, text: str, level: int = 1) -> tuple[int, int, bytes]:
    return rec(P.WalkRecord.LOG, t_us, bytes([level]) + text.encode())


def run_mark(t_us: int, run_id: int, kind: int, event: int) -> tuple[int, int, bytes]:
    return rec(P.WalkRecord.RUN, t_us, P.RunMark(id=run_id, kind=kind, event=event).pack())


def blocks(records: list[tuple[int, int, bytes]], boot: int = 0, seq0: int = 0, crc: bool = True,
           open_last: bool = True, skipped: int = 3) -> bytes:
    """Records (absolute us) into blocks of host.record_block_bytes; each
    block's t0 is its first record's time; the last one open (crc 0)."""
    out, cur, t0, seq = [], [], None, seq0
    size = flight.HDR

    def seal(last: bool) -> None:
        nonlocal cur, size, seq, t0
        body = b"".join(struct.pack("<BBI", k, len(p), t - t0) + p for t, k, p in cur)
        c = 0 if (last and open_last) or not crc else zlib.crc32(body)
        h = P.RecordBlockHeader(magic=flight.BLOCK_MAGIC, boot=boot, seq=seq, used=flight.HDR + len(body),
                                t0_us_lo=t0 & 0xFFFFFFFF, t0_us_hi=t0 >> 32, records=len(cur), skipped=skipped,
                                crc32=c)
        out.append(h.pack() + body)
        seq += 1
        cur, size, t0 = [], flight.HDR, None

    for r in sorted(records, key=lambda x: x[0]):
        n = 6 + len(r[2])
        if size + n > flight.BLOCK_BYTES:
            seal(False)
        if t0 is None:
            t0 = r[0]
        cur.append(r)
        size += n
    if cur:
        seal(True)
    return b"".join(out)


def tel(t_us: int, tick: int, nose_up: float = 0.0, roll: float = 0.0, mode: int = P.Mode.IDLE, batt: int = 7900,
        frame_k: int = 0xFF, rate_pitch: float = 0.0) -> tuple[int, int, bytes]:
    m = P.Telemetry(tick=tick & 0xFFFF, pitch=int(round(-nose_up * 16)), roll=int(round(roll * 16)), mode=int(mode),
                    batt_mv=batt, trick_frame=frame_k, rate_pitch=int(round(rate_pitch * 16)), epoch=4,
                    flags=int(P.TelemetryFlags.SERVOS_POWERED))
    return frame(P.WalkRecord.RX, t_us, m, tick)


def event(t_us: int, code: int, arg: int = 0) -> tuple[int, int, bytes]:
    return frame(P.WalkRecord.RX, t_us, P.Event(code=int(code), arg=arg, tick=0, epoch=4))


# ---- a trick run: rear_up_2, tipping over backwards at "rise" --------------------------

def trick_records(name: str = "rear_up_2", run_id: int = 3, tip_at: str | None = "rise", end: str = "FELL",
                  key: str | None = None, crc: str | None = None) -> tuple[list, float]:
    t = library.load(name)
    src = t.source
    durs = [f.dur for f in t.frames]
    holds = [f.hold for f in t.frames]
    names, pred = src["keyframes"], src["predicted_pitch_deg"]
    out = []
    begin = T0 + 2_000_000
    play = begin + 300_000
    out.append(log(T0 + 1_000_000, "remote B2 (0x0E, rear_up_2): trick rear_up_2 started"))
    out.append(event(begin - 50_000, P.EventCode.IR_KEY, 0x0E))
    out.append(run_mark(begin, run_id, P.RunKind.TRICK, P.RunEvent.BEGIN))
    crc = f"{t.crc16():04x}" if crc is None else crc
    out.append(mark(begin, f"trick id={run_id} format=2 trick={name} stage=2 tid=77 crc={crc} frames={len(durs)} "
                           f"speed=64 mirror=0 mind_build=12345678 body_build=45524f43 battery_mv=7900 mode=IDLE "
                           f"by=remote"))
    out.append(mark(begin + 10, "frames dur=" + ",".join(map(str, durs)) + " hold=" + ",".join(map(str, holds))
                    + f" loop=0,0,0"))
    out.append(mark(play, "play speed=64"))
    # the keyframes' times and the prediction (as trickrec computes it, independently here)
    starts, tt = [], 0
    for d, h in zip(durs, holds):
        starts.append((tt, tt + phase_ticks(d), tt + phase_ticks(d) + phase_ticks(h)))
        tt += phase_ticks(d) + phase_ticks(h)
    total_s = tt * 0.02
    tip_s = None if tip_at is None else starts[names.index(tip_at)][0] * 0.02 + 0.05
    end_s = total_s if tip_s is None else tip_s + 1.0
    tick = 1000
    t_us = T0
    while t_us < play + int((end_s + 2.5) * 1e6):
        s = (t_us - play) / 1e6
        if s < 0:
            nu = 0.0
            fk = 0xFF
        else:
            prev, nu, fk = 0.0, pred[-1], len(durs) - 1
            for k, (a, b, c) in enumerate(starts):
                if s < c * 0.02:
                    fk = k
                    if s < b * 0.02:
                        x = (s - a * 0.02) / ((b - a) * 0.02)
                        nu = prev + (pred[k] - prev) * (1 - math.cos(math.pi * x)) / 2
                    else:
                        nu = pred[k]
                    break
                prev = pred[k]
        rate = 0.0
        if tip_s is not None and s > tip_s:
            nu += 30 * (s - tip_s)  # over backwards
            rate = 30.0
        mode = P.Mode.TRICK if 0 <= s <= end_s else P.Mode.IDLE
        out.append(tel(t_us, tick, nu, mode=mode, batt=7900 - int(max(0, s) * 10), frame_k=fk if mode == P.Mode.TRICK
                       else 0xFF, rate_pitch=rate))
        tick += 1
        t_us += TEL_US
    end_us = play + int(end_s * 1e6)
    if key is not None:
        out.append(event(end_us - 100_000, P.EventCode.IR_KEY, 0x02))
        out.append(mark(end_us - 90_000, f"key {key}"))
        out.append(log(end_us - 90_000, key))
    if end == "FELL":
        out.append(event(end_us - 20_000, P.EventCode.FELL))
    out.append(frame(P.WalkRecord.TX, play - 20_000, P.TrickPlay(epoch=4, trick_id=77, speed_q6=64, flags=0)))
    out.append(mark(end_us, f"end reason={end} elapsed_ms={int(end_s * 1000)} expected_ticks={tt} battery_mv=7850 "
                            f"mode=FALLEN why=the body fell"))
    out.append(run_mark(end_us + 10, run_id, P.RunKind.TRICK, P.RunEvent.END))
    out.append(log(end_us + 20, f"trick {name}: TRICK_DONE {end}"))
    return out, tip_s


def test_blocks_parse_times_crc_and_a_cut_last_block():
    recs = [tel(T0 + i * TEL_US, i, nose_up=i * 0.01) for i in range(2000)] + [log(T0 + 7, "hello")]
    data = blocks(recs)
    s = flight.parse_blocks(data)
    assert len(s.blocks) > 2 and not s.bad_blocks and s.garbage == 0
    tl = s.telemetry()
    assert len(tl) == 2000 and tl[0][0] == T0 and tl[-1][0] == T0 + 1999 * TEL_US
    assert s.logs[0].text == "hello" and s.logs[0].t_us == T0 + 7 and s.logs[0].tag == "I"
    assert s.skipped == 3 * len(s.blocks)
    # a damaged block: skipped, named; the others still read
    bad = bytearray(data)
    bad[flight.HDR + 40] ^= 0xFF
    s2 = flight.parse_blocks(bytes(bad))
    assert [b.problem for b in s2.bad_blocks] == ["CRC mismatch"] and len(s2.telemetry()) < 2000
    # the last block cut short (it was still being written): read as far as it goes
    s3 = flight.parse_blocks(data[:-100])
    assert s3.blocks[-1].problem == "cut" and 1990 < len(s3.telemetry()) < 2000
    # garbage in front: skipped up to the first block
    s4 = flight.parse_blocks(b"\x00" * 17 + data)
    assert s4.garbage == 17 and len(s4.telemetry()) == 2000


def test_mark_fields():
    assert flight.parse_mark("end reason=FELL elapsed_ms=900 why=the body fell over") == (
        "end", {"reason": "FELL", "elapsed_ms": "900", "why": "the body fell over"})
    assert flight.parse_mark("end ok=0 streamed=1 reason=the body left GAIT (mode STANCE)")[1]["reason"] == \
        "the body left GAIT (mode STANCE)"
    assert flight.parse_mark("key remote B6 (0x02, stand): trick stopped") == (
        "key", {"text": "remote B6 (0x02, stand): trick stopped"})


def test_a_walk_in_blocks_reads_as_before():
    """The synthetic walk of test_walks, as format-2 blocks with RUN marks:
    the same summary as from its format-1 records."""
    one = synthetic(300)
    w1 = walkrec.parse_records(one)
    recs = [run_mark(T0 + 1_000_000, 4, P.RunKind.WALK, P.RunEvent.BEGIN)]
    i = 0
    while i + 6 <= len(one):
        k, ln, t = struct.unpack_from("<BBI", one, i)
        recs.append((T0 + 1_000_000 + t, k, one[i + 6:i + 6 + ln]))
        i += 6 + ln
    recs.append(run_mark(recs[-1][0] + 1, 4, P.RunKind.WALK, P.RunEvent.END))
    recs += [tel(T0 + j * TEL_US, j) for j in range(40)]  # the pre-roll's idle telemetry
    data = blocks(recs)
    w2 = walkrec.parse_records(data)
    s1, s2 = walkrec.summarize(w1), walkrec.summarize(w2)
    assert s2["error"]["rms_pitch"] == s1["error"]["rms_pitch"] == 2.0
    assert s2["error"]["offset_roll"] == s1["error"]["offset_roll"] and s2["ticks_planned"] == 300
    assert s2["end"]["reason"].startswith("stopped")


def test_trick_tip_over_is_found_at_its_keyframe():
    recs, tip_s = trick_records()
    s = flight.parse_blocks(blocks(recs))
    assert [r.label for r in s.runs] == ["trick 3"]
    tr = trickrec.trick_run(s)
    assert tr.name == "rear_up_2" and tr.stage == "2" and tr.play_us is not None
    a = trickrec.analyze(tr)
    assert a["has_prediction"] and a["played"]
    kf = {k["name"]: k for k in a["keyframes"]}
    # before the tip the measurement follows the design: the errors are small
    for n in ("look", "crouch", "back", "paw_l", "paws", "up"):
        assert kf[n]["reached"] and abs(kf[n]["max_err_deg"]) < 0.5, (n, kf[n])
    src = library.load("rear_up_2").source
    assert kf["up"]["pred_end_deg"] == src["predicted_pitch_deg"][src["keyframes"].index("up")]
    tip = a["tip_over"]
    assert tip is not None and tip["keyframe_name"] == "rise" and tip["axis"] == "pitch"
    assert tip["direction"].startswith("back")
    assert tip["since_play_s"] == pytest.approx(tip_s + 8 / 30, abs=0.06)  # 8 deg at 30 deg/s
    assert a["end"]["reason"] == "FELL" and a["fell_events"] and a["peak"]["rate_pitch_dps"] == 30.0
    assert a["battery"]["min_mv"] < 7900
    text = trickrec.format_analysis(a)
    assert "TIP-OVER onset" in text and "rise" in text and "FELL: the body fell" in text
    rows = trickrec.rows(tr)
    both = [r for r in rows if r["since_play_s"] is not None and 0 < r["since_play_s"] < tip_s
            and r["body_keyframe"] is not None]
    same = sum(r["body_keyframe"] == r["keyframe"] for r in both)  # the clock's keyframe is the body's (but at edges)
    assert both and same >= 0.95 * len(both)


def test_trick_without_a_tip_and_stopped_by_a_key(tmp_path):
    recs, _ = trick_records(name="rear_up", run_id=5, tip_at=None, end="STOPPED",
                            key="remote B6 (0x02, stand): trick stopped")
    s = flight.parse_blocks(blocks(recs))
    a = trickrec.analyze(trickrec.trick_run(s, s.run(5)))
    assert a["tip_over"] is None and a["end"]["reason"] == "STOPPED"
    assert a["stopped_by"] == "remote B6 (0x02, stand): trick stopped"
    assert abs(a["error"]["max_deg"]) < 0.5
    p = trickrec.write_csv(trickrec.rows(trickrec.trick_run(s)), tmp_path / "t.csv")
    hdr = next(csv.reader(open(p)))
    assert tuple(hdr) == trickrec.CSV_FIELDS


def test_an_old_recording_is_not_read_against_a_new_design():
    """A run of a trick whose library entry has changed since (its CRC in
    the run's `trick` MARK is not the library's: a new design): the
    keyframes are numbered, nothing is predicted, and the analysis says why."""
    recs, _ = trick_records(name="rear_up", run_id=5, tip_at=None, end="STOPPED", crc="beef")
    s = flight.parse_blocks(blocks(recs))
    a = trickrec.analyze(trickrec.trick_run(s, s.run(5)))
    assert not a["has_prediction"] and "not the trick this run played (CRC beef" in a["source_note"]
    assert [k["name"] for k in a["keyframes"]][:2] == ["frame 0", "frame 1"]
    assert "changed since" in trickrec.format_analysis(a)


def test_a_refused_trick():
    recs = [run_mark(T0, 9, P.RunKind.TRICK, P.RunEvent.BEGIN),
            mark(T0, "trick id=9 format=2 trick=rear_up stage=1 tid=5 crc=0 frames=12 speed=64 mirror=0 "
                     "battery_mv=7100 mode=IDLE by=remote"),
            mark(T0 + 5, "end reason=REFUSED elapsed_ms=0 expected_ticks=0 battery_mv=7100 mode=IDLE "
                         "why=battery 7100 mV (needs 7400)"),
            run_mark(T0 + 6, 9, P.RunKind.TRICK, P.RunEvent.END)] + [tel(T0 + i * TEL_US, i) for i in range(10)]
    s = flight.parse_blocks(blocks(recs))
    a = trickrec.analyze(trickrec.trick_run(s))
    assert not a["played"] and a["end"]["why"] == "battery 7100 mV (needs 7400)"
    text = trickrec.format_analysis(a)
    assert "REFUSED: battery 7100 mV (needs 7400)" in text and "not played" in text


def test_the_stage_reached_labels_a_staged_trick():
    # the mind since 2026-11-21: the stage allowed at the start, each rise (A2) and top reached, the end's
    recs = [run_mark(T0, 9, P.RunKind.TRICK, P.RunEvent.BEGIN),
            mark(T0, "trick id=9 format=2 trick=rear_up_3 stage=1 of=3 tid=5 crc=0 frames=12 speed=64 mirror=0 "
                     "battery_mv=7900 mode=IDLE by=remote"),
            mark(T0 + 100, "rear_up: up to stage 1 (of 3)"),
            mark(T0 + 200, "stage allowed=1 reached=1 of=3"),
            mark(T0 + 300, "stage allowed=2 reached=1 of=3"),
            mark(T0 + 400, "end reason=STOPPED elapsed_ms=9000 expected_ticks=900 battery_mv=7800 mode=IDLE "
                           "stage=2 of=3 reached=1 why="),
            run_mark(T0 + 500, 9, P.RunKind.TRICK, P.RunEvent.END)] + [tel(T0 + i * TEL_US, i) for i in range(10)]
    s = flight.parse_blocks(blocks(recs))
    tr = trickrec.trick_run(s)
    assert (tr.stage, tr.stage_of, tr.stage_reached) == ("2", "3", "1")
    a = trickrec.analyze(tr, source={})
    assert "trick run 9: rear_up stage 2 of 3 (reached 1), mind" in trickrec.format_analysis(a)
    # without the end's fields (a run still open): the last stage MARK
    s = flight.parse_blocks(blocks(recs[:5] + recs[6:]))
    assert trickrec.trick_run(s).stage == "2"
    # an older mind: the build's stage only, as it was
    old = [run_mark(T0, 9, P.RunKind.TRICK, P.RunEvent.BEGIN),
           mark(T0, "trick id=9 format=2 trick=rear_up_3 stage=3 tid=5 crc=0 frames=12 speed=64 mirror=0 "
                    "battery_mv=7900 mode=IDLE by=remote"),
           run_mark(T0 + 500, 9, P.RunKind.TRICK, P.RunEvent.END)] + [tel(T0 + i * TEL_US, i) for i in range(10)]
    a = trickrec.analyze(trickrec.trick_run(flight.parse_blocks(blocks(old))), source={})
    assert a["stage"] == "3" and a["stage_of"] is None
    assert "trick run 9: rear_up_3 (stage 3), mind" in trickrec.format_analysis(a)
    lab = trickrec.trick_label
    assert lab("rear_up_3", "1", "3", "1") == "rear_up stage 1 of 3"
    assert lab("rear_up_3", "1", "3", "0") == "rear_up stage 1 of 3 (top not reached)"
    assert lab("rear_up", "1", "1") == "rear_up stage 1 of 1"
    assert lab("rear_up_2", "2") == "rear_up_2 (stage 2)" and lab("sit", None) == "sit"


def test_decode_rows_json_and_the_file(tmp_path, capsys):
    recs, _ = trick_records()
    meta = {"mind": "x", "mind_t0_ms": 1000}
    path = flight.write_file(tmp_path / "r.krec", meta, blocks(recs))
    assert flight.is_krec(path) and not walkrec.is_kwalk(path)
    s = flight.load(path)
    rows = list(decode.rows(s))
    assert len(rows) == len(s.telemetry()) and rows[0]["uptime_s"] == pytest.approx(4.0)
    keyed = [r for r in rows if r["key"]]
    assert keyed and keyed[0]["key"].startswith("IR_KEY 0x000e")
    inrun = [r for r in rows if r["run"] == "trick 3"]
    assert inrun and any(r["pred_nose_up_deg"] is not None for r in inrun)
    assert any(r["event"] == "FELL" for r in rows)
    assert any(r["command"].startswith("TRICK_PLAY") for r in rows)
    js = list(decode.json_rows(s))
    types = {d["type"] for d in js}
    assert {"telemetry", "log", "key", "event", "command", "mark", "run"} <= types
    assert [d["t_s"] for d in js] == sorted(d["t_s"] for d in js)
    # the CLI, offline
    assert cli.main(["record", "decode", str(path), "--csv", str(tmp_path / "o.csv")]) == 0
    out = capsys.readouterr()
    assert "run   3 trick" in out.out and "wrote" in out.err
    assert tuple(next(csv.reader(open(tmp_path / "o.csv")))) == decode.CSV_FIELDS
    assert cli.main(["record", "decode", str(path), "--log"]) == 0
    assert "remote B2" in capsys.readouterr().out
    assert cli.main(["record", "decode", str(path), "--json", "--from", "3", "--to", "3.1"]) == 0
    lines = [json.loads(x) for x in capsys.readouterr().out.splitlines()]
    assert lines and all(3 <= d["t_s"] <= 3.1 + 1e-9 for d in lines)
    # kilo log trick / run on the file
    assert cli.main(["log", "trick", str(path)]) == 0
    assert "TIP-OVER" in capsys.readouterr().out
    assert cli.main(["log", "run", str(path), "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["trick"] == "rear_up_2"
    assert cli.main(["log", "walk", str(path)]) == 1  # no walk in it
    assert "no walk" in capsys.readouterr().err


def test_record_commands_need_host(capsys):
    for argv in (["runs"], ["record", "status"], ["bench", "sd"], ["metrics"]):
        assert cli.main(argv) == 1
        assert "needs --host" in capsys.readouterr().err


# ---- the CLI against a fake mind -------------------------------------------------------

class RecMind(FakeMind):
    """A fake mind with a flight recorder: one walk-less trick run this boot
    (id 3), one run of boot 6 on the card, REC_*, METRICS, the bench."""

    def __init__(self, keys: list[bytes]) -> None:
        super().__init__(keys)
        recs, _ = trick_records()
        self.data = blocks(recs)
        self.fetches: list[Any] = []
        self.history_from = 100  # the first history line's seq: 100 older lines dropped
        self.prev_log = b"1000 I Kilo mind, build 12345678\n2500 W remote B2: refused\n3000 M keys: total=1\n"

    def info(self, i: int) -> Any:
        if i == 0:
            return P.HostWalkInfo(found=1, kept=2, recording=0, capacity=64 << 20, used=len(self.data), id=3,
                                  start_ms=2000, duration_ms=12000, bytes=len(self.data), records=900,
                                  flags=int(P.WalkFlags.TRICK | P.WalkFlags.STREAMED),
                                  text=b"trick=rear_up_2 stage=2 end=FELL")
        return P.HostWalkInfo(found=1, kept=2, recording=0, capacity=64 << 20, used=len(self.data),
                              id=(6 << 20) | 2, start_ms=50000, duration_ms=9000, bytes=0, records=0,
                              flags=int(P.WalkFlags.TRICK | P.WalkFlags.SAVED | P.WalkFlags.EARLIER_BOOT
                                        | P.WalkFlags.ENDED_OK | P.WalkFlags.STREAMED),
                              text=b"trick=rear_up_3 stage=1 of=3 end=DONE")

    def _stream(self, c: socket.socket, wid: int, data: bytes, total: int) -> None:
        off = 0
        while off < len(data):
            chunk = data[off:off + P.HOST_WALK_CHUNK_MAX]
            self._send(c, P.HostWalkData(id=wid, offset=off, total=total, status=0, data=chunk), 0)
            off += len(chunk)
        self._send(c, P.HostWalkData(id=wid, offset=off, total=total, status=int(P.WalkDataStatus.END)), 0)

    def _handle(self, c: socket.socket, seq: int, m: Any) -> None:
        if isinstance(m, P.HostWalkList):
            order = [0, 3, (6 << 20) | 2]
            nxt = next((k for k, v in enumerate(order[1:]) if v > m.after), None)
            if nxt is None:
                self._send(c, P.HostWalkInfo(found=0, kept=2, capacity=64 << 20, used=len(self.data)), seq)
            else:
                self._send(c, self.info(nxt), seq)
        elif isinstance(m, P.HostWalkFetch):
            self._ok(c, seq, m)
            self._stream(c, m.id, self.data, 0 if m.id >> 20 else len(self.data))  # from the card: total unknown
        elif isinstance(m, P.HostRecStatusReq):
            self._send(c, P.HostRecStatus(clock_ms=900_000, boot=7, capacity=64 << 20, used=3 << 20,
                                          block_bytes=16384, blocks=192, first_seq=0, next_seq=191, oldest_ms=1000,
                                          records=50000, skipped=9000, runs=1, card_state=int(P.RecordCardState.IDLE),
                                          save_on=1, saved_seq=180, unsaved_bytes=180_000, card_bytes=5 << 20,
                                          card_files=9, card_boots=2, writes=300, write_max_us=42000), seq)
        elif isinstance(m, P.HostRecFetch):
            self.fetches.append(m)
            if m.what == P.RecFetchWhat.LOG:
                if m.boot != 6:
                    self._ok(c, seq, m, int(P.HostResultCode.BAD_ARG), b"no log for that boot")
                    return
                self._ok(c, seq, m)
                self._stream(c, 0, self.prev_log, 0)
                return
            self._ok(c, seq, m, 0, b"blocks 0..3")
            self._stream(c, 0, self.data, len(self.data))
        elif isinstance(m, P.HostMetricsReq):
            lines = [b"keys: total=4 acted=3 ignored=1 refused=0", b"tricks: started=1 FELL=1"]
            text = lines[m.index] if m.index < len(lines) else b""
            self._send(c, P.HostMetrics(index=m.index, count=len(lines), text=text), seq)
        elif isinstance(m, P.HostRecBench):
            if not self.controller:
                self._ok(c, seq, m, int(P.HostResultCode.NOT_CONTROLLER), b"take control first")
                return
            self._ok(c, seq, m)
            for size in (512, 4096):
                self._send(c, P.HostBenchResult(write_bytes=size, count=m.count, min_us=800, mean_us=1500,
                                                max_us=9000 if size == 512 else 150_000, sync_us=3000), 0)
            self._send(c, P.HostBenchResult(write_bytes=0, close_us=2000, remove_us=1500), 0)
        elif isinstance(m, (P.HostRecSave, P.HostRecDelete)):
            ok = self.controller
            self._ok(c, seq, m, 0 if ok else int(P.HostResultCode.NOT_CONTROLLER), b"ok" if ok else b"take control")
        elif isinstance(m, P.HostLogSub):
            self._ok(c, seq, m)
            n = 5 if m.tail == 255 else m.tail
            base = self.history_from if m.tail == 255 else 0
            for i in range(n):
                s_ = base + i + (2 if i >= 3 else 0)  # a gap of 2 after the third
                self._send(c, P.HostLogLine(seq=s_, t_ms=1000 * i, level=1, text=f"line {i}".encode()), i)
            self._send(c, P.HostLogLine(seq=0x80000000 | 7, t_ms=9000, level=1, text=b"link: status"), 9)
        else:
            super()._handle(c, seq, m)


@pytest.fixture
def rmind(host_key):
    m = RecMind([host_key.public])
    yield m
    m.close()


def test_cli_runs_list_and_fetch(rmind, tmp_path, capsys):
    assert cli.main(["--host", rmind.addr, "runs"]) == 0
    out = capsys.readouterr().out
    assert "1 runs this boot (0 walks, 1 tricks), 1 from earlier boots; RAM 3.0 of 64 MB" in out
    rows = [ln for ln in out.splitlines() if ln[:1].isdigit() or ln.startswith("b6:")]
    assert len(rows) == 2 and "rear_up_2 stage 2" in rows[0] and rows[0].rstrip().endswith("FELL")
    assert rows[1].startswith("b6:2") and "boot 6 at" in rows[1] and "card" in rows[1]
    assert "rear_up stage 1 of 3 " in rows[1]  # the stage reached, not the build's 3
    assert cli.main(["walks", "--host", rmind.addr, "--json"]) == 0
    js = json.loads(capsys.readouterr().out)
    assert js[0]["kind"] == "trick" and js[0]["trick"] == "rear_up_2" and js[1]["boot"] == 6
    assert cli.main(["--host", rmind.addr, "runs", "--fetch", "last", "--out", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    files = sorted(tmp_path.glob("run-*-trick-3.krec"))
    assert len(files) == 1 and "TIP-OVER onset" in out
    meta, data = flight.read_file(files[0])
    assert data == rmind.data and meta["mind_t0_ms"] == 900_000 - 4321 and meta["boot"] == 7
    # an earlier boot's run (from the card: total 0)
    assert cli.main(["--host", rmind.addr, "runs", "--fetch", "b6:2", "--out", str(tmp_path), "-q"]) == 0
    assert sorted(tmp_path.glob("run-b6-*-trick-2.krec"))
    capsys.readouterr()
    assert cli.main(["--host", rmind.addr, "runs", "--delete", "6"]) == 0
    assert "deleting boot 6's recordings" in capsys.readouterr().out
    assert any(isinstance(m, P.HostRecDelete) and m.boot == 6 for m in rmind.got)
    assert cli.main(["--host", rmind.addr, "runs", "--delete", "all"]) == 0
    assert any(isinstance(m, P.HostRecDelete) and m.boot == 0xFFFFFFFF for m in rmind.got)


def test_cli_record_status_fetch_save(rmind, tmp_path, capsys):
    assert cli.main(["--host", rmind.addr, "record", "status"]) == 0
    out = capsys.readouterr().out
    assert "boot 7" in out and "a power cut now loses 176 KB" in out and "IDLE" in out
    assert cli.main(["--host", rmind.addr, "record", "fetch", "--last", "10m", "--out", str(tmp_path)]) == 0
    f = rmind.fetches[-1]
    assert (f.boot, f.from_ms, f.to_ms, f.what) == (0, 300_000, 0xFFFFFFFF, 0)
    out = capsys.readouterr().out
    assert "run   3 trick" in out and sorted(tmp_path.glob("rec-*.krec"))
    assert cli.main(["--host", rmind.addr, "record", "fetch", "--since", "20m", "--until", "5m",
                     "--out", str(tmp_path)]) == 0
    f = rmind.fetches[-1]
    assert (f.from_ms, f.to_ms) == (0, 600_000)  # 20 min ago is before the clock began: from 0
    assert cli.main(["--host", rmind.addr, "record", "fetch", "--boot", "6", "--out", str(tmp_path)]) == 0
    assert rmind.fetches[-1].boot == 6 and sorted(tmp_path.glob("rec-*-b6.krec"))
    capsys.readouterr()
    assert cli.main(["--host", rmind.addr, "record", "save", "on"]) == 0
    assert any(isinstance(m, P.HostRecSave) and m.mode == 1 for m in rmind.got)
    assert "card saver on" in capsys.readouterr().out


def test_cli_bench_metrics_status(rmind, capsys):
    assert cli.main(["--host", rmind.addr, "bench", "sd", "--count", "8"]) == 0
    out = capsys.readouterr().out
    assert "      512 B      8" in out and "slowest 150.0 ms: OVER" in out and "delete 1.50 ms" in out
    assert [m.count for m in rmind.got if isinstance(m, P.HostRecBench)] == [8]
    assert cli.main(["--host", rmind.addr, "metrics"]) == 0
    assert capsys.readouterr().out.splitlines() == ["keys: total=4 acted=3 ignored=1 refused=0",
                                                    "tricks: started=1 FELL=1"]
    assert cli.main(["--host", rmind.addr, "status"]) == 0
    out = capsys.readouterr().out
    assert "recorder    RAM 3.0 of 64 MB" in out and "metrics     tricks: started=1 FELL=1" in out
    assert cli.main(["--host", rmind.addr, "status", "--json"]) == 0
    d = json.loads(capsys.readouterr().out)
    assert d["recorder"]["card_state"] == "IDLE" and len(d["metrics"]) == 2


def test_cli_log_history_dropped_gaps_save_previous(rmind, tmp_path, capsys):
    assert cli.main(["log", "--host", rmind.addr, "--no-follow", "--save", str(tmp_path)]) == 0
    out = capsys.readouterr().out.splitlines()
    assert out[0] == "(the mind dropped 100 older lines; it keeps the last 4096)"
    assert any("2 lines lost here" in ln for ln in out) and out[-1].endswith("link: status")
    saved = sorted(tmp_path.glob("kilo-*.log"))
    assert len(saved) == 1 and saved[0].read_text().splitlines() == out
    # --lines N: no claim about dropped lines (the tail was asked short)
    assert cli.main(["log", "--host", rmind.addr, "-n", "2", "--no-follow"]) == 0
    assert not capsys.readouterr().out.startswith("(the mind dropped")
    # --previous: boot 7 runs now, so boot 6's log from the card
    assert cli.main(["log", "--host", rmind.addr, "--previous"]) == 0
    out = capsys.readouterr().out
    assert "boot 6's log as saved on the SD card: 3 lines" in out and "     2.500 W remote B2: refused" in out
    f = rmind.fetches[-1]
    assert (f.boot, f.what) == (6, P.RecFetchWhat.LOG)
    assert cli.main(["log", "--host", rmind.addr, "--previous", "4"]) == 1
    assert "no saved log for boot 4" in capsys.readouterr().err


def test_parse_duration_and_run_ids():
    from kilo.cli_record import parse_duration, parse_run_id

    assert parse_duration("10m") == 600 and parse_duration("90s") == 90 and parse_duration("2h") == 7200
    assert parse_duration("1.5") == 1.5
    with pytest.raises(ValueError):
        parse_duration("soon")
    assert parse_run_id("7") == 7 and parse_run_id("b6:3") == (6 << 20) | 3
