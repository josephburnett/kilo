"""The IR remote on the mind: the key table (against the legacy src/infrared.h),
the --remote key map by grid, its RemoteControl actions on a mock client (the
posture keys, the gait keys, the speed ladder into backward, the turns, the
freeze, the behaviour key), the renderings, and the daemon acting on EVENT
IR_KEY from the virtual body. kernel/host/tests/test_remote.cpp checks the
kernel's port the same way."""

import re
import sys
import tempfile
import threading
import time
import types
from pathlib import Path
from types import SimpleNamespace

import pytest

from kilo import ir
from kilo.body.timeline import Timeline
from kilo.cli import render_monitor  # noqa: F401  (import check: the monitor renders IR_KEY)
from kilo.cli import _event_text
from kilo.link import consts

from test_daemon import remote, start_daemon, vbody_pty, sockdir, wait_for  # noqa: F401  (fixtures)

LEGACY = Path(__file__).resolve().parents[2] / "src" / "infrared.h"


def test_key_table_matches_the_legacy_firmware():
    src = LEGACY.read_text()
    full = re.findall(r'case 0x([0-9A-F]{6}): /\*PTLF\(" ([^"]+)"\)', src)
    short = re.findall(r'case 0x([0-9A-F]{2}): /\*PTLF\(" ([^"]+)"\)', src)
    assert len(full) == len(short) == len(ir.KEYS) == 21
    for (code, label), (cmd, label2) in zip(full, short):
        v = int(code, 16)  # 0x00FFA25D: address 0x00, ~0xFF, command, ~command
        assert v >> 16 == 0xFF and (v >> 8) & 0xFF == int(cmd, 16) and ((v >> 8) ^ v) & 0xFF == 0xFF
        assert label == label2
        assert ir.BY_LABEL[label].command == int(cmd, 16)
    assert len(ir.BY_COMMAND) == 21 and ir.BY_COMMAND[0xA2].grid == "A7"


def test_describe():
    assert ir.describe(0x02) == "B6 (0x02, stand)"
    assert ir.describe(0x102) == "B6 (0x02, stand) repeat"
    assert ir.describe(0xA2) == "A7 (0xA2, sleep)"
    assert ir.describe(0x90) == "C5 (0x90, calibration posture)"
    assert ir.describe(0x18) == "B3 (0x18, none)"
    assert ir.describe(0x45) == "? (0x45)"
    assert ir.split(0x1A2) == (0xA2, True)
    ev = consts.EV["IR_KEY"]
    assert ev == 18 and _event_text(ev, 0x162) == "IR_KEY B7 (0x62, faster) repeat"
    assert _event_text(consts.EV["FELL"], -1) == "FELL(-1)"
    for k in ir.KEYS:  # every line names the key by grid, never by its printed legend
        assert ir.describe(k.command).startswith(k.grid + " (0x") and "EQ" not in ir.describe(k.command)


def test_timeline_renders_ir_key():
    lines = []
    tl = Timeline.__new__(Timeline)
    tl.say = lambda text, now=None: lines.append(text)
    tl._on_event(SimpleNamespace(code=consts.EV["IR_KEY"], arg=0x68, tick=5, epoch=2))
    assert lines == ["EVENT IR_KEY A4 (0x68, trot) tick=5 epoch=2"]


EXPECTED = {
    "A7": ("posture", "rest"), "B7": ("speed", "faster"), "C7": ("freeze", ""),
    "A6": ("turn", "left"), "B6": ("posture", "stand"), "C6": ("turn", "right"),
    "A5": ("behaviour", ""), "B5": ("speed", "slower"), "C5": ("posture", "calib"),
    "A4": ("gait", "trot"), "B4": ("gait", "crawl"), "C4": ("gait", "walk"),
    "B2": ("trick", "rear_up"),
}


def test_key_map():
    assert set(ir.ACTIONS) == set(ir.BY_GRID)
    for k in ir.KEYS:
        a = ir.action_for(k.command)
        assert (a.kind, a.name) == EXPECTED.get(k.grid, ("none", "")), k.grid
        assert (a.kind == "none") == (int(k.grid[1]) <= 3 and k.grid != "B2")  # rows 1-3 do nothing but B2
        assert ir.action_for(k.command | ir.REPEAT).kind == "ignore"  # held keys do nothing
    assert ir.action_for(0x45).kind == "ignore"  # another remote
    from kilo.planner.patterns import PATTERNS

    from kilo.tricks import library

    for a in ir.ACTIONS.values():
        if a.kind == "posture":
            assert a.name in consts.POSTURES
        if a.kind == "gait":
            assert a.name in PATTERNS
        if a.kind == "trick":
            assert a.name in library.names()


class FakeWalker:
    def __init__(self, client):
        self.client = client

    @property
    def active(self):
        return bool(self.client.status and self.client.status["active"])

    def abort(self):
        self.client.calls.append(("abort",))
        self.client.status = {"active": False, "period_ms": 600}


class FakeClient:
    def __init__(self, mode="IDLE"):
        self.calls = []
        self.state = SimpleNamespace(mode=mode, flags=set(), batt_mv=8000)
        self.status = None
        self._walker = FakeWalker(self)
        self.started = None

    def start_trick(self, trick, mirror=False, speed=1.0):
        self.calls.append(("start_trick", trick.name))
        self.started = trick
        self.state.mode = "TRICK"
        return 1, 0

    def started_trick(self):
        return self.started

    def walk(self, direction, pattern, speed_mm_s):
        self.calls.append(("walk", direction, pattern, round(speed_mm_s, 1)))
        self.status = {"active": True, "period_ms": 600}

    def stop_walk(self, wait_s=None):
        self.calls.append(("stop_walk",))
        self.status = {"active": False, "period_ms": 600}
        return True

    def walk_status(self):
        return self.status

    def posture(self, name):
        self.calls.append(("posture", name))

    def stop_trick(self, transition_ticks=None):
        self.calls.append(("stop_trick",))
        self.state.mode = "IDLE"


class FakeRunner:
    def __init__(self):
        self.on = False
        self.starts = self.stops = 0

    def active(self):
        return self.on

    def start(self):
        self.on = True
        self.starts += 1
        return None

    def stop(self):
        self.on = False
        self.stops += 1


def rig(mode="IDLE"):
    c = FakeClient(mode)
    rc = ir.RemoteControl(c, behaviour=FakeRunner())
    return c, rc, lambda grid: rc.perform(ir.ACTIONS[grid])


@pytest.mark.parametrize("mode,expected", [
    ("IDLE", [("posture", "stand")]),
    ("TRICK", [("stop_trick",), ("posture", "stand")]),
])
def test_remote_posture(mode, expected):
    c, rc, key = rig(mode)
    assert key("B6") == "posture stand"
    assert c.calls == expected


def test_remote_posture_keys_stop_a_walk_first():
    c, rc, key = rig()
    assert key("A7") == "posture rest"
    assert key("C5") == "posture calib" and c.calls[-1] == ("posture", "calib")  # the calibration posture
    key("A4")
    n = len(c.calls)
    assert key("C5") == "posture calib"  # the planned stop, waited for, then the posture
    assert c.calls[n:] == [("stop_walk",), ("posture", "calib")]


def test_remote_gait_keys():
    c, rc, key = rig()
    assert key("C4") == "walk forward (walk) 17.8 mm/s"  # not walking: forward in that gait at 1x
    assert c.calls[-1] == ("walk", "forward", "walk", 17.8)
    n = len(c.calls)
    assert key("C4") == "walk forward (walk) 17.8 mm/s: unchanged" and len(c.calls) == n  # idempotent
    key("B7")  # +4
    assert key("A4") == "walk forward (trot) 152.2 mm/s"  # another gait: changed, the level kept
    assert c.calls[-1] == ("walk", "forward", "trot", 158.4) and rc.level == 4  # 1.1x asked: the trot's top
    assert key("B4") == "walk forward (crawl) 24.4 mm/s"
    for _ in range(5):
        key("B5")  # +4 +3 +2 +1 -1 -2
    assert rc.level == -2 and c.calls[-1][1] == "back"
    assert key("B4") == "walk forward (crawl) 15.6 mm/s"  # the gait in use: forward at the magnitude
    assert rc.level == 2 and c.calls[-1] == ("walk", "forward", "crawl", 15.6)
    # every stop puts the level back
    assert key("B6") == "posture stand" and rc.level == 3 and rc.heading == "straight"
    assert key("A4") == "walk forward (trot) 144.0 mm/s"
    key("B7")
    c.status = {"active": False}  # the walk ended by itself
    assert key("A4") == "walk forward (trot) 144.0 mm/s" and rc.level == 3
    # a playing trick is stopped before a walk from standing
    c, rc, key = rig("TRICK")
    key("A4")
    assert c.calls == [("stop_trick",), ("walk", "forward", "trot", 144.0)]


def test_remote_speed_ladder():
    c, rc, key = rig()
    assert key("B7") == "ignored (not walking)" and key("B5") == "ignored (not walking)"
    assert c.calls == []  # they never start a walk
    key("A4")
    # since 2026-11-15 the slowest level is about the old default (trot 55.6 mm/s), 1x more than twice it
    assert [key("B5") for _ in range(8)] == [
        "speed 0.7x (trot 100.8 mm/s forward)", "speed 0.4x (trot 57.6 mm/s forward)",
        "speed -0.4x (trot 57.6 mm/s back)", "speed -0.7x (trot 100.8 mm/s back)",
        "speed -1x (trot 144.0 mm/s back)", "speed -1.1x (trot 152.2 mm/s back)",
        "speed -1.2x (trot 152.2 mm/s back)", "speed -1.2x (trot 152.2 mm/s back): already the fastest back"]
    assert rc.level == -5 and c.calls[-1] == ("walk", "back", "trot", 172.8)  # asked 1.2x; the planner caps it
    for _ in range(4):
        key("B7")
    assert rc.level == -1
    assert key("B7") == "speed 0.4x (trot 57.6 mm/s forward)" and c.calls[-1][1] == "forward"
    for _ in range(3):
        key("B7")
    assert key("B7") == "speed 1.2x (trot 152.2 mm/s forward)"
    n = len(c.calls)
    assert key("B7") == "speed 1.2x (trot 152.2 mm/s forward): already the fastest" and len(c.calls) == n
    # a level the planner clamps to the same speed (the trot's top two): the level moves, the real mm/s shown
    assert key("B5") == "speed 1.1x (trot 152.2 mm/s forward)"
    assert key("B5") == "speed 1x (trot 144.0 mm/s forward)"
    assert key("C4") == "walk forward (walk) 17.8 mm/s"


def test_remote_turns():
    c, rc, key = rig()
    assert key("A6") == "walk left (trot) 144.0 mm/s"  # from standing: the turn at 1x
    assert c.calls[-1] == ("walk", "left", "trot", 144.0)
    assert key("A6") == "walk left (trot) 144.0 mm/s: unchanged"
    assert key("C6") == "walk right (trot) 144.0 mm/s"
    assert key("B5") == "speed 0.7x (trot 100.8 mm/s right)"
    assert key("B5") == "speed 0.4x (trot 57.6 mm/s right)"
    assert key("B5") == "speed 0.4x (trot 57.6 mm/s right): already the slowest"  # no sign flip in a turn
    assert key("B7") == "speed 0.7x (trot 100.8 mm/s right)"
    assert key("A4") == "walk forward (trot) 100.8 mm/s" and rc.heading == "straight"  # the gait: straight again
    key("C6")
    assert key("C4") == "walk right (walk) 12.4 mm/s"  # another gait: the turn goes on in it
    key("C4")
    key("B5")
    key("B5")  # +2: +1, -1
    assert rc.level == -1
    assert key("A6") == "walk left (walk) 7.1 mm/s" and rc.level == 1  # backward, then a turn: the magnitude
    assert key("C4") == "walk forward (walk) 7.1 mm/s"


def test_remote_freeze():
    c, rc, key = rig()
    key("A4")
    assert key("C7") == "frozen (walk aborted): only B6 or A7 act now"
    assert c.calls[-1] == ("abort",) and rc.frozen  # no planned stop, no posture, no LIMP
    n = len(c.calls)
    for g in ("B7", "B5", "A4", "B4", "C4", "A6", "C6", "C5", "A5", "C7"):
        assert key(g) == "ignored (frozen: B6 or A7 to go on)", g
    assert len(c.calls) == n and rc.behaviour.starts == 0
    assert key("B6") == "unfrozen, posture stand" and not rc.frozen
    assert key("A4") == "walk forward (trot) 144.0 mm/s"  # from 1x again
    # standing: holds the posture; A7 goes on too
    c, rc, key = rig()
    assert key("C7") == "frozen (holding the posture): only B6 or A7 act now" and c.calls == []
    assert key("A7") == "unfrozen, posture rest"
    # a trick playing: stopped
    c, rc, key = rig("TRICK")
    assert key("C7") == "frozen (trick stopped): only B6 or A7 act now" and c.calls == [("stop_trick",)]


def test_remote_rear_up_key(monkeypatch):
    slept = []
    monkeypatch.setattr(ir.time, "sleep", slept.append)
    c, rc, key = rig()
    assert key("B2") == "trick rear_up started (19 s; B2 again, B6 or C7 bring it down)"
    assert c.calls == [("start_trick", "rear_up")] and c.started.stop_ticks == 75
    # pressed again while it plays: it comes down (TRICK_STOP over its stop_ticks, from the client)
    assert key("B2") == "trick stopped: coming down" and c.calls[-1] == ("stop_trick",) and slept == []
    # B6 while it plays: stopped, the 1.5 s crossfade waited for, then STAND
    key("B2")
    assert key("B6") == "posture stand" and c.calls[-2:] == [("stop_trick",), ("posture", "stand")]
    assert slept == [pytest.approx(75 * 0.02 + 0.1)]
    # C7 while it plays: stopped (it comes down), frozen; B2 is not a way out of a freeze
    key("B2")
    assert key("C7") == "frozen (trick stopped): only B6 or A7 act now" and c.calls[-1] == ("stop_trick",)
    assert key("B2") == "ignored (frozen: B6 or A7 to go on)"
    # walking: the planned stop first, then the trick
    c, rc, key = rig()
    key("A4")
    n = len(c.calls)
    assert key("B2").startswith("trick rear_up started")
    assert c.calls[n:] == [("stop_walk",), ("start_trick", "rear_up")] and rc.level == 3
    # a flat pack or USB power: refused, nothing moves
    for flags, mv, why in ((set(), 7300, "battery 7300 mV (needs 7400)"), ({"no_battery"}, 0, "on USB power (no battery)"),
                           (set(), None, "battery None mV (needs 7400)")):
        c, rc, key = rig()
        c.state.flags, c.state.batt_mv = flags, mv
        assert key("B2") == f"rear_up refused: {why}" and c.calls == []
    # the behaviour runner active: ignored like every key but the stop keys
    c, rc, key = rig()
    key("A5")
    assert key("B2") == "ignored (behaviour runner active)"


def test_remote_freeze_cancels_a_posture_key_waiting_for_the_planned_stop():
    c, rc, key = rig()
    key("A4")
    waiting, release = threading.Event(), threading.Event()

    def slow_stop(wait_s=None):  # the planned stop takes a while
        waiting.set()
        release.wait(2)
        c.status = {"active": False}
        return True

    c.stop_walk = slow_stop
    out = []
    t = threading.Thread(target=lambda: out.append(key("B6")))
    t.start()
    assert waiting.wait(2)
    assert rc.freeze() == "frozen (walk aborted): only B6 or A7 act now"  # beside it, as the daemon runs C7
    release.set()
    t.join(2)
    assert out == ["cancelled (C7 froze Kilo)"] and ("posture", "stand") not in c.calls and rc.frozen


def test_remote_behaviour_key():
    c, rc, key = rig()
    key("A4")
    assert key("A5") == "behaviour runner started" and c.calls[-1] == ("stop_walk",)  # the walk stopped first
    assert key("A5") == "behaviour runner already running" and rc.behaviour.starts == 1
    for g in ("B7", "B5", "A4", "C4", "A6", "C5"):
        assert key(g) == "ignored (behaviour runner active)", g
    assert key("B6") == "behaviour runner stopped, posture stand" and not rc.behaviour.on
    key("A5")
    assert key("A7") == "behaviour runner stopped, posture rest"
    key("A5")
    assert key("C7") == "frozen (behaviour runner stopped): only B6 or A7 act now"
    assert rc.behaviour.stops == 3 and rc.behaviour.starts == 3
    assert ir.RemoteControl(FakeClient()).perform(ir.ACTIONS["A5"]) == "behaviour runner failed: no behaviour runner"


# ---- the daemon with --remote, keys injected by the virtual body -----------------------------

def test_daemon_remote_keys(start_daemon):  # noqa: F811
    d, vb, path, sock = start_daemon(remote=True)
    body = remote(sock)
    try:
        body.wait_ready(3)
        vb.ir_key(0x02)  # B6: stand
        assert wait_for(lambda: vb.posture == "STAND")
        time.sleep(1.5)  # past idle_limp_s (1 s): the remote's posture is held, no idle LIMP
        assert vb.posture == "STAND" and vb.wd_stage == 0
        assert body.conn.call("daemon")["remote"] == "on, last key B6 (0x02, stand)"
        vb.ir_key(0xA2, repeat=True)  # a repeat does nothing
        vb.ir_key(0x18)  # B3: row 3, nothing
        vb.ir_key(0x62)  # B7: not walking, nothing
        time.sleep(0.3)
        assert vb.posture == "STAND" and vb.feet is None
        vb.ir_key(0x90)  # C5: the calibration posture
        assert wait_for(lambda: vb.posture == "CALIB")
        vb.ir_key(0xA2)  # A7: rest
        assert wait_for(lambda: vb.posture == "REST")
        # a client's hold wins over the remote
        body.posture("stand")
        assert wait_for(lambda: vb.posture == "STAND")
        vb.ir_key(0xA2)
        body.spin(0.4)
        assert vb.posture == "STAND"
    finally:
        body.close()


def test_daemon_without_remote_ignores_keys(start_daemon):  # noqa: F811
    d, vb, path, sock = start_daemon()
    body = remote(sock)
    try:
        body.wait_ready(3)
        assert vb.posture == "LIMP"
        vb.ir_key(0x02)
        time.sleep(0.4)
        assert vb.posture == "LIMP"
        assert body.conn.call("daemon")["remote"] == "off"
    finally:
        body.close()


def test_daemon_remote_walks_and_freezes(start_daemon):  # noqa: F811
    d, vb, path, sock = start_daemon(remote=True)
    body = remote(sock)
    try:
        body.wait_ready(3)
        vb.ir_key(0xB0)  # C4: walk forward in the walk gait, from LIMP (the walker takes STANCE first)
        assert wait_for(lambda: vb.feet is not None, 4)
        assert wait_for(lambda: (body.conn.call("daemon")["walk"] or {}).get("pattern") == "walk", 2)
        time.sleep(1.5)  # past idle_limp_s: no idle timeline over the remote's walk
        info = body.conn.call("daemon")
        assert vb.feet is not None and vb.wd_stage == 0
        assert info["walk"]["active"] and info["walk"]["direction"] == "forward" and "WALK" not in info["held_goals"]
        vb.ir_key(0x68)  # A4: trot, changed while walking (the legs go home first: up to ~5.5 s from the walk)
        assert wait_for(lambda: body.conn.call("daemon")["walk"]["changing_to"] == "trot", 1)
        assert wait_for(lambda: body.conn.call("daemon")["walk"]["pattern"] == "trot", 8)
        assert vb.gait_stops == []
        vb.ir_key(0x22)  # A6: turn left
        assert wait_for(lambda: body.conn.call("daemon")["walk"]["direction"] == "left", 2)
        # C7: the stream ends at once; the body holds, settles the swing feet, stays in STANCE (GAIT_STOPPED 1)
        vb.ir_key(0xE2)
        assert wait_for(lambda: vb.feet is None and vb.gait_stops == [1], 3)
        assert vb.posture == "STANCE"
        time.sleep(1.5)  # past idle_limp_s: frozen, not rested or made limp
        assert vb.posture == "STANCE" and vb.feet is None
        vb.ir_key(0x68)  # A4: ignored while frozen
        time.sleep(0.5)
        assert vb.feet is None
        vb.ir_key(0x02)  # B6: stand
        assert wait_for(lambda: vb.posture == "STAND", 3)
        vb.ir_key(0x22)  # A6: turn left in place from standing
        assert wait_for(lambda: vb.feet is not None, 4)
        assert body.conn.call("daemon")["walk"]["direction"] == "left"
        vb.ir_key(0x02)  # B6: stand stops the walk first (the planned stop)
        assert wait_for(lambda: vb.feet is None and vb.posture == "STAND", 6)
        assert vb.gait_stops == [1, 0]
    finally:
        body.close()


def test_daemon_remote_starts_and_stops_the_behaviour_runner(start_daemon, monkeypatch):  # noqa: F811
    calls = []

    def run_behaviour(client, state, stop_event, log_dir):
        calls.append("started")
        client.posture("sit")
        stop_event.wait(10)
        calls.append("stopped")

    fake = types.ModuleType("kilo.behaviour.runner")
    fake.run_behaviour = run_behaviour
    monkeypatch.setitem(sys.modules, "kilo.behaviour.runner", fake)
    d, vb, path, sock = start_daemon(remote=True, log_dir=Path(tempfile.mkdtemp(prefix="kl")))
    body = remote(sock)
    try:
        body.wait_ready(3)
        assert body.conn.call("daemon")["behaviour"] == "off"  # off by default, as before
        vb.ir_key(0xE0)  # A5: start it
        assert wait_for(lambda: calls == ["started"] and vb.posture == "SIT", 3)
        assert body.conn.call("daemon")["behaviour"] == "running"
        vb.ir_key(0x68)  # A4: ignored while it runs
        time.sleep(0.5)
        assert vb.feet is None
        vb.ir_key(0x02)  # B6: stops it, then stands
        assert wait_for(lambda: calls == ["started", "stopped"] and vb.posture == "STAND", 6)
        assert body.conn.call("daemon")["behaviour"] != "running"
        vb.ir_key(0xE0)  # A5 again: a fresh run
        assert wait_for(lambda: calls.count("started") == 2 and vb.posture == "SIT", 3)
        vb.ir_key(0xE2)  # C7: stops it and freezes
        assert wait_for(lambda: calls.count("stopped") == 2, 6)
        time.sleep(0.3)
        assert vb.posture == "SIT"  # frozen in the posture it was in
        body.spin(1.5)  # liveness kept with the runner stopped: no watchdog
        assert vb.wd_stage == 0
    finally:
        body.close()
