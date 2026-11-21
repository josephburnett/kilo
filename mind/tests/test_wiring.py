"""The mind's pieces wired together: wire bytes into a .kilolog, the body
events and percepts the behaviour runner sees."""

import time

from kilo.behaviour.body_events import BodyEvents
from kilo.behaviour.runner import Runner
from kilo.link import consts
from kilo.link.codec import DecodeError, Frame, KiloCodec
from kilo.logs.kilolog import KilologWriter, read_kilolog


# ---- codec and link: wire bytes ----------------------------------------------------------

def test_feed_wire_pairs_each_frame_with_its_wire_bytes():
    from kilo.link.messages import KiloMessages

    c, m = KiloCodec(), KiloMessages()
    a, b = c.encode(m.type_of("HEARTBEAT"), 1, b""), c.encode(m.type_of("HELLO_REQ"), 2, b"")
    junk = b"\x05\x06\x07\x00"
    stream = b"\x00" + a + junk + b
    d = c.new_decoder()
    got = []
    for i in range(0, len(stream), 3):  # arbitrary chunking
        got += d.feed_wire(stream[i:i + 3])
    frames = [(r, w) for r, w in got if isinstance(r, Frame)]
    errors = [(r, w) for r, w in got if isinstance(r, DecodeError)]
    assert [w for _, w in frames] == [a, b]
    assert [r for r, _ in frames] == c.new_decoder().feed(a + b)
    assert len(errors) == 1 and errors[0][1] == junk


def test_link_writes_rx_and_tx_wire_frames(make_link, tmp_path):
    link, peer = make_link()
    w = KilologWriter(tmp_path / "l.kilolog")
    link.wire_logger = w
    for i in range(5):
        peer.telemetry(tick=i + 1)
        time.sleep(0.02)
    assert peer.wait_for(lambda: peer.of("HEARTBEAT"))
    link.stop()
    w.close()
    meta, recs = read_kilolog(w.path)
    recs = list(recs)
    assert all(r.error is None for r in recs)
    rx = [r for r in recs if r.direction == "rx"]
    assert len(rx) == 5 and {r.frame.type for r in rx} == {link.messages.type_of("TELEMETRY")}
    assert any(r.direction == "tx" for r in recs)


# ---- behaviour-side units -------------------------------------------------------------------

def test_body_events_from_snapshots():
    be = BodyEvents()
    ev = lambda tick, name, arg=0, epoch=0: (0.1, {"code": consts.EV[name], "arg": arg, "tick": tick,
                                                   "epoch": epoch})
    assert be.update({"epoch": 0, "connected": True, "alive": True, "events": [ev(1, "BOOT")]}) == []
    out = be.update({"epoch": 1, "connected": True, "alive": True,
                     "events": [ev(1, "BOOT"), ev(50, "FELL", 1, 1)]})
    assert out == [{"type": "epoch", "from": 0, "to": 1},
                   {"type": "event", "name": "FELL", "arg": 1, "epoch": 1, "tick": 50}]
    assert be.update({"epoch": 1, "connected": True, "alive": True,
                      "events": [ev(1, "BOOT"), ev(50, "FELL", 1, 1)]}) == []
    assert be.update({"epoch": 1, "connected": False, "alive": False, "events": []}) == [
        {"type": "connection", "connected": False, "alive": False}]


def test_runner_keeps_a_fall_startle_over_a_quiet_percept_source():
    class Body:
        def __init__(self):
            self.snap = {"epoch": 0, "fallen": False, "events": []}

        def snapshot(self):
            return dict(self.snap)

        def posture(self, name, balance=None):
            pass

        def set_balance(self, enable):
            pass

    body = Body()
    r = Runner(body, None, percepts=lambda: {"startle": 0.0, "pet": False}, log=False, seed=1)
    r.step()
    assert "startle" not in r.ctx.percepts
    body.snap["fallen"] = True
    r.step()
    assert r.ctx.percepts["startle"] == 1.0
