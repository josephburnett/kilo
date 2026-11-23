"""Walk recordings: kilo.logs.walkrec (the parser, the per-tick table, the
summary, the .kilolog and CSV) on a synthetic recording whose error is
known, and `kilo walks --host` (WALK_LIST, WALK_FETCH) against a fake mind.
The real recorder and server are tested in kernel/host/tests/test_record.cpp
and test_host.cpp, and all of it on the virtual body in kernel/host/test.sh."""

from __future__ import annotations

import csv
import math
import socket
import struct
from typing import Any

import pytest

from kilo import cli
from kilo.hostlink import HostError, HostLink
from kilo.logs import proto2, reader, walkrec
from kilo.engine import pack_pose_feet
from kilo.protocol import kilo_protocol as P
from test_hostlink import FakeMind

TICK_US = 20000
FIRST = 65500  # the body's tick wraps (u16) in the walk


def rec(kind: int, t_us: int, payload: bytes) -> bytes:
    return struct.pack("<BBI", kind, len(payload), t_us) + payload


def mark(t_us: int, text: str) -> bytes:
    return rec(P.WalkRecord.MARK, t_us, text.encode())


def frame(kind: int, t_us: int, msg: Any, seq: int = 0) -> bytes:
    return rec(kind, t_us, bytes([msg.ID, seq & 0xFF]) + msg.pack())


def a16(deg: float) -> int:
    return int(round(deg * 16))


def synthetic(n: int = 400, offset=(2.0, -1.0), lag_plan=10, lag_model=8, lead_us=90000, proto2_feet=False) -> bytes:
    """A walk whose plan rolls on a sine; the POSE prediction is the plan
    lag_model ticks late, the body lag_plan ticks late plus an offset; LF is
    in the air (and the corrections held) a quarter of the time.
    proto2_feet: the stream as protocol 2 recordings have it (FEET, 16 B)."""
    plan = [5.0 * math.sin(2 * math.pi * t / 40) for t in range(-20, n)]  # index t + 20
    out = [mark(0, f"walk id=7 format=1 t0_us_hi=0 t0_us_lo=1000000 setup_us=0 mind_build=1e1405f0 "
                   f"body_build=45524f43 epoch=4 tick_ms=20"),
           mark(10, "setup pattern=walk dir=forward speed_asked=default balance=1"),
           mark(20, "stream tick=65500 pattern=walk dir=forward speed=11.1 speed_set=11.1 stride_mm=30.0 "
                    "period_ms=3200.0 swing_ticks=25 lead=8"),
           mark(30, "config margin_mm=14.0 gate_deg=5.0 catch_deg=7.0 hold_max=25 lean=1"),
           mark(40, f"model servo_lag_ms={lag_model * 20}.0 total_g=358.0"),
           mark(50, "masses body_g=183.0 battery_g=58.0 pi_g=45.0")]
    for t in range(n):
        tick = FIRST + t
        lifting = 1 if t % 40 < 10 else 0
        margin = 100 + (t % 40) * 10 if lifting else -32768
        p_roll, pred_roll, meas_roll = plan[t + 20], plan[t + 20 - lag_model], plan[t + 20 - lag_plan] + offset[1]
        tp = t * TICK_US
        out.append(rec(P.WalkRecord.PLAN, tp, P.WalkPlan(tick=tick, sent=1, lifting=lifting, support=15 & ~lifting,
                                                           margin=margin, plan_pitch=0, plan_roll=a16(p_roll),
                                                           pitch=0, roll=a16(pred_roll)).pack()))
        if proto2_feet:
            stream = struct.pack("<BHbbbbBBBBhhB", 4, tick & 0xFFFF, 0, 0, 0, 0, 60, 60, 60, 60, 0, a16(pred_roll),
                                 15 & ~lifting)
        else:
            stream = pack_pose_feet(4, tick, [0] * 4, [60] * 4, 0, a16(pred_roll), 15 & ~lifting)
        out.append(rec(P.WalkRecord.TX, tp, bytes([0x11, t & 0xFF]) + stream))
        flags = P.TelemetryFlags.STREAM | P.TelemetryFlags.BALANCE_ACTIVE
        if lifting:
            flags |= P.TelemetryFlags.BALANCE_HELD
        tel = P.Telemetry(tick=tick & 0xFFFF, pitch=a16(offset[0]), roll=a16(meas_roll), mode=int(P.Mode.STREAM),
                          flags=int(flags), ref_pitch=0, ref_roll=a16(pred_roll), corr_pitch=-8, corr_roll=a16(-0.5),
                          pose_queued=4, support=15 & ~lifting, epoch=4)
        out.append(frame(P.WalkRecord.RX, tp + lead_us, tel, t))
        if t % 100 == 50:
            out.append(rec(P.WalkRecord.PLANNER, tp, P.WalkPlanner(tick=tick, holds=t // 100, catches=0, waits=1,
                                                                    min_margin=100).pack()))
    te = n * TICK_US + lead_us
    out.append(mark(te, f"result sent={n} skipped=0 underruns=0 max_behind=1 dropped_rx=0 holds=3 catches=0 waits=4 "
                        f"min_margin_mm=10.0 tick={FIRST + n} clock_resets=0"))
    out.append(mark(te, "end ok=1 streamed=1 reason=stopped (no STREAM_STOPPED seen after the STANCE posture)"))
    return b"".join(out)


def test_parse_mark():
    assert walkrec.parse_mark("end ok=0 streamed=1 reason=the body left STREAM (mode STANCE)") == (
        "end", {"ok": "0", "streamed": "1", "reason": "the body left STREAM (mode STANCE)"})
    assert walkrec.parse_mark("stop tick=345") == ("stop", {"tick": "345"})
    assert walkrec.parse_mark("walk") == ("walk", {})


@pytest.mark.parametrize("old", [False, True], ids=["pose", "protocol2_feet"])
def test_the_summary_finds_the_error_put_in(old):
    w = walkrec.parse_records(synthetic(proto2_feet=old))
    tx = [f.msg for f in w.frames if f.direction == "tx"]
    assert all(isinstance(m, proto2.Feet if old else P.Pose) for m in tx) and len(tx) == 400
    assert {f.name for f in w.frames if f.direction == "tx"} == {"FEET" if old else "POSE"}
    assert w.bad == 0 and len(w.plans) == 400 and w.t0_us == 1000000 and len(w.planner) == 4
    rows = walkrec.tick_table(w)
    assert [r["tick"] for r in rows] == list(range(FIRST, FIRST + 400))  # unwrapped across 65535
    s = walkrec.summarize(w, rows)
    e = s["error"]
    assert e["samples"] == 400
    assert e["offset_pitch"] == pytest.approx(2.0, abs=0.01) and e["offset_roll"] == pytest.approx(-1.0, abs=0.05)
    assert e["meas_mean_pitch"] == pytest.approx(2.0) and e["rms_pitch_less_offset"] == pytest.approx(0, abs=1e-6)
    assert e["rms_roll"] > abs(e["offset_roll"])  # the two lags differ: the roll error swings
    lag = s["lag"]
    assert lag["from_plan_roll_ms"] == 200 and lag["quality_roll"] > 0.99  # the body's lag, outright
    assert lag["model_ms"] == 160.0 and lag["tracker_vs_model_ms"] == pytest.approx(40)  # slower than the model
    assert set(s["by_phase"]) == {"none", "LF"} and s["by_phase"]["LF"]["ticks"] == 100
    assert s["held"]["fraction"] == 0.25 and s["held"]["seconds"] == 2.0 and s["balance_active_fraction"] == 1.0
    assert s["correction"]["mean_pitch"] == -0.5 and s["correction"]["mean_roll"] == -0.5
    assert s["margin"]["min_one_leg_up_mm"] == 10.0 and s["margin"]["planner_min_mm"] == 10.0
    assert s["lead_ms"] == {"min": 90.0, "mean": 90.0, "max": 90.0}
    assert s["planner"]["holds"] == 3 and len(s["planner"]["events"]) == 4
    assert s["end"]["reason"].startswith("stopped (no STREAM_STOPPED") and s["stream"]["pattern"] == "walk"
    text = walkrec.format_summary(s)
    assert "pitch rms 2.00" in text and "offset +2.00" in text and "LF" in text and "+40 ms vs the model" in text


def test_files_csv_kilolog_and_the_existing_tools(tmp_path):
    data = synthetic(200)
    kw = walkrec.write_file(tmp_path / "w.kwalk", {"walk": {"id": 7}, "start_wall": 1.0e9}, data)
    assert walkrec.is_kwalk(kw) and walkrec.read_file(kw) == ({"start_wall": 1.0e9, "walk": {"id": 7}}, data)
    w = walkrec.load(kw)
    rows = walkrec.tick_table(w)
    out = walkrec.write_csv(rows, tmp_path / "w.csv")
    got = list(csv.DictReader(open(out)))
    assert len(got) == 200 and set(walkrec.TABLE_FIELDS) == set(got[0])
    assert float(got[0]["err_pitch"]) == pytest.approx(2.0) and got[5]["lifting"] == "LF" and got[5]["held"] == "1"
    # the .kilolog: kilo log summary reads it unchanged
    kl = walkrec.to_kilolog(w, tmp_path / "w.kilolog")
    d = reader.load(kl)
    assert d.format == "kilolog" and len(d.samples) == 200 and d.tx_types["POSE"] == 200
    assert d.meta["walk"] == {"id": 7} and d.meta["start_wall"] == 1.0e9
    assert d.samples[0].t == pytest.approx(0.09, abs=0.002)  # ms since t0
    # ... and the .kwalk itself
    d2 = reader.load(kw)
    assert d2.format == "kwalk" and len(d2.samples) == 200 and d2.samples[0].mode == "STREAM"
    assert cli.main(["log", "summary", str(kl)]) == 0
    assert cli.main(["log", "walk", str(kw), "--csv", str(tmp_path / "x.csv"), "--plot", str(tmp_path / "x.png")]) == 0
    assert (tmp_path / "x.csv").exists() and ((tmp_path / "x.png").exists() or (tmp_path / "x.csv").exists())


def test_a_cut_off_recording_parses_to_its_last_whole_record():
    data = synthetic(50)
    w = walkrec.parse_records(data[:-7])
    assert w.bad == 1 and w.mark("end") is None and len(w.plans) == 50
    assert walkrec.summarize(w)["error"]["samples"] == 50


# ---- kilo walks --host against a fake mind ---------------------------------------------------

class WalkMind(FakeMind):
    """FakeMind with two walks: 1 recorded (the synthetic one), 2 still recording."""

    def __init__(self, keys: list[bytes]) -> None:
        super().__init__(keys)
        self.walk = synthetic(300)
        self.gone_after: int | None = None  # GONE after this many bytes
        self.supported = True

    def info(self, w: int) -> Any:
        i = P.HostWalkInfo(found=1, kept=2, recording=2, capacity=8 << 20, used=len(self.walk) + 999,
                           dropped_walks=0, dropped_records=0, id=w, start_ms=1000 if w == 1 else 9000,
                           duration_ms=6000, bytes=len(self.walk) if w == 1 else 999, records=1500, feet=300,
                           flags=int(P.WalkFlags.STREAMED | P.WalkFlags.ENDED_OK) if w == 1
                           else int(P.WalkFlags.RECORDING | P.WalkFlags.STREAMED),
                           text=b"gait=walk dir=forward speed=11.1 end=stopped" if w == 1
                           else b"gait=trot dir=left speed=55.6")
        return i

    def _handle(self, c: socket.socket, seq: int, m: Any) -> None:
        if isinstance(m, P.HostWalkList) and self.supported:
            if m.after < 2:
                self._send(c, self.info(m.after + 1), seq)
            else:
                self._send(c, P.HostWalkInfo(found=0, kept=2, recording=2, capacity=8 << 20,
                                             used=len(self.walk) + 999), seq)
        elif isinstance(m, P.HostWalkFetch) and self.supported:
            if m.id != 1:
                self._ok(c, seq, m, int(P.HostResultCode.BAD_ARG), b"walk 2 is still being recorded")
                return
            self._ok(c, seq, m)
            off, total = m.offset, len(self.walk)
            while off < total:
                if self.gone_after is not None and off >= self.gone_after:
                    self._send(c, P.HostWalkData(id=1, offset=off, total=total, status=int(P.WalkDataStatus.GONE)), 0)
                    return
                chunk = self.walk[off:off + P.HOST_WALK_CHUNK_MAX]
                self._send(c, P.HostWalkData(id=1, offset=off, total=total, status=0, data=chunk), 0)
                off += len(chunk)
            self._send(c, P.HostWalkData(id=1, offset=total, total=total, status=int(P.WalkDataStatus.END)), 0)
        else:
            super()._handle(c, seq, m)


@pytest.fixture
def wmind(host_key):
    m = WalkMind([host_key.public])
    yield m
    m.close()


def test_hostlink_lists_and_fetches(wmind):
    with HostLink.connect(wmind.addr) as hl:
        ws = hl.walks()
        assert [w.id for w in ws] == [1, 2] and hl.walk_store.found == 0
        seen: list[tuple[int, int]] = []
        data = hl.fetch_walk(1, progress=lambda got, total: seen.append((got, total)))
        assert data == wmind.walk and seen[-1] == (len(data), len(data))
        wmind.gone_after = 1000
        with pytest.raises(HostError, match="dropped from the mind's store"):
            hl.fetch_walk(1)


def test_cli_walks_list_and_fetch(wmind, tmp_path, capsys):
    assert cli.main(["--host", wmind.addr, "walks"]) == 0
    out = capsys.readouterr().out
    assert "2 runs this boot (2 walks, 0 tricks)" in out and "run 2 being recorded" in out
    lines = [ln for ln in out.splitlines() if ln.strip()[:1].isdigit()]
    assert len(lines) == 2 and "walk  walk forward 11.1" in lines[0] and lines[0].rstrip().endswith("stopped")
    assert lines[1].rstrip().endswith("recording") and "trot" in lines[1]
    assert cli.main(["walks", "--host", wmind.addr, "--json"]) == 0
    import json
    js = json.loads(capsys.readouterr().out)
    assert js[0]["gait"] == "walk" and js[0]["end"] == "stopped" and "RECORDING" in js[1]["flags"]
    # --fetch (all): walk 1 written, walk 2 still recording: said, exit 0 (nothing failed)
    assert cli.main(["--host", wmind.addr, "walks", "--fetch", "--out", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    kw = sorted(tmp_path.glob("walk-*-1.kwalk"))
    assert len(kw) == 1 and kw[0].with_suffix(".kilolog").exists() and "walk 1: " in out
    assert "pitch rms 2.00" in out  # the summary
    meta, data = walkrec.read_file(kw[0])
    assert data == wmind.walk and meta["walk"]["id"] == 1 and meta["mind_build"] == "1e1405f0"
    # the start as wall time: the fake's uptime is 4321 ms, walk 1 started at 1000 ms
    assert meta["fetched_wall"] - meta["start_wall"] == pytest.approx(3.321, abs=0.01)
    # walk 2 asked for by id: still being recorded
    assert cli.main(["--host", wmind.addr, "walks", "--fetch", "2", "--out", str(tmp_path)]) == 1
    assert "still being recorded" in capsys.readouterr().err
    assert cli.main(["--host", wmind.addr, "walks", "--fetch", "9", "--out", str(tmp_path)]) == 1
    assert "no run 9 on the mind (kept: 1, 2)" in capsys.readouterr().err


def test_cli_walks_on_a_mind_without_a_recorder(wmind, capsys):
    wmind.supported = False  # the fake answers UNSUPPORTED, as a mind without a store
    assert cli.main(["--host", wmind.addr, "walks"]) == 1
    assert "does not record walks (UNSUPPORTED)" in capsys.readouterr().err


def test_cli_walks_needs_host(capsys):
    assert cli.main(["walks"]) == 1
    assert "needs --host" in capsys.readouterr().err
