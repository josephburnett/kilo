"""Regression tests for the second review of the mind: goals into a rebooting
body, the runner's pause, the logs off the control threads, a preempted
trick, races."""

import asyncio
import threading
import time

import pytest

from kilo.behaviour.runner import Runner
from kilo.body.client import BodyClient
from kilo.body.state import BodyState
from kilo.link import consts
from kilo.link.codec import DecodeError, KiloCodec
from kilo.link.serial_link import LinkConfig, NotConnected, SerialLink
from kilo.link.transport import loopback_pair
from kilo.logs.kilolog import KilologWriter, read_kilolog
from kilo.protocol import kilo_protocol as P
from kilo.tricks import library
from kilo.vbody.sim import VirtualBody

from conftest import FAST, keep_alive
from test_behaviour import Clock, FakeBody, day_cfg


@pytest.fixture
def rig():
    made = []

    def make(**vb_kw):
        a, b = loopback_pair()
        vb = VirtualBody(b, **vb_kw).start()
        link = SerialLink(lambda: a, config=LinkConfig(**FAST), port_name="loopback")
        link.mark_alive()
        link.start()
        made.append((link, vb))
        return BodyClient(link), vb, a

    yield make
    for link, vb in made:
        link.stop()
        vb.stop()


def _keep(link, until, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        link.mark_alive()
        if until():
            return True
        time.sleep(0.02)
    return until()


# ---- 2, 8: goals into a rebooting body -----------------------------------------------------

def test_reconnect_into_a_rebooting_body_does_not_stand_it_up():
    """The port reopens (USB replug / DTR) and the open resets the body, which
    buffers what it hears during its 0.6 s init: the latched STAND (epoch 0,
    as the rebooted body's) must not go out before the body is heard."""
    a, b = loopback_pair()
    vb = VirtualBody(b, boot_delay=0.6).start()
    opens = []

    def factory():
        if not opens:
            opens.append(a)
            return a
        a2, b2 = loopback_pair()
        vb.t = b2
        vb.reboot(P.ResetCause.WARM_UNEXPECTED)  # the open's DTR edge resets it
        opens.append(a2)
        return a2

    link = SerialLink(factory, config=LinkConfig(**FAST), port_name="loopback")
    client = BodyClient(link)
    link.mark_alive()
    link.start()
    try:
        client.wait_ready(2)
        client.stand()
        assert _keep(link, lambda: vb.posture == "STAND")
        assert vb.epoch == 0
        opens[0].fail()  # replug
        postures = []
        end = time.monotonic() + 2.5
        while time.monotonic() < end:
            link.mark_alive()
            postures.append(vb.posture)
            time.sleep(0.02)
        assert len(opens) == 2 and vb.booting_until is None
        i = postures.index("LIMP")  # the reboot
        assert "STAND" not in postures[i:], postures
        assert "SET_POSTURE" not in link.goals() and link.state.reboots == 1
        assert link.state.reset_cause == "WARM_UNEXPECTED"
        # contact is back: a new goal works
        client.stand()
        assert _keep(link, lambda: vb.posture == "STAND")
    finally:
        link.stop()
        vb.stop()


def test_lost_body_gets_no_motion_goal_until_it_is_heard(rig):
    """Same connection: the body reboots (1 s init). Once telemetry has been
    missing for 300 ms no SET_POSTURE refresh goes out; the boot HELLO is a
    reboot (new identity) and clears the goal before any refresh."""
    client, vb, a = rig()
    client.wait_ready(2)
    client.stand()
    assert _keep(client.link, lambda: vb.posture == "STAND")
    sent = []
    dec = KiloCodec().new_decoder()
    real_write = a.write

    def tap(data):
        for f in dec.feed(data):
            if not isinstance(f, DecodeError):
                sent.append((time.monotonic(), client.link.messages.name_of(f.type)))
        real_write(data)
    a.write = tap
    t0 = time.monotonic()
    vb.reboot(P.ResetCause.WARM_UNEXPECTED, boot_delay=1.0)
    keep_alive(client.link, 1.6)
    assert vb.booting_until is None
    late = [n for t, n in sent if t - t0 > 0.45 and n == "SET_POSTURE"]
    assert late == [], late
    assert any(n == "HELLO_REQ" for t, n in sent)  # still polling while lost
    assert "SET_POSTURE" not in client.link.goals() and client.state.reboots == 1
    with pytest.raises(NotConnected):  # TRICK_PLAY waits for contact too
        client.link._contact = False
        client.link.request("TRICK_PLAY", {"epoch": 0, "trick_id": 1, "speed_q6": 64, "flags": 0})


def test_second_reboot_within_3s_still_clears_goals(make_link):
    link, peer = make_link()
    peer.hello(epoch=0)
    peer.telemetry(tick=100, epoch=0)
    assert link.wait_until(lambda: link.state.hello is not None and link.state.tick, 1)
    keep_alive(link, 0.4)
    peer.hello(epoch=0, reset_cause=3)  # unsolicited: reboot 1
    assert link.wait_until(lambda: link.state.reboots == 1, 1)
    link.set_goal("SET_POSTURE", posture=consts.POSTURES["stand"], transition_ticks=0, flags=0)
    peer.telemetry(tick=5, epoch=0)
    keep_alive(link, 0.4)
    peer.hello(epoch=0, reset_cause=2)  # a crash loop: reboot 2 within 3 s
    assert link.wait_until(lambda: "SET_POSTURE" not in link.goals(), 1)
    assert link.state.reboots == 1  # the count is debounced, the clear is not


def test_solicited_hello_with_a_new_identity_is_a_reboot(make_link):
    link, peer = make_link()
    peer.hello(epoch=0)
    peer.telemetry(tick=100, epoch=0)
    assert link.wait_until(lambda: link.state.hello is not None, 1)
    link.set_goal("SET_POSTURE", posture=consts.POSTURES["stand"], transition_ticks=0, flags=0)
    keep_alive(link, 0.1)
    reboots = []
    link.subscribe("reboot", lambda h, why: reboots.append(why))
    # a HELLO inside the solicitation window, but from a new boot
    link._hello_solicited_at = link.clock()
    peer.hello(epoch=0, reset_cause=3)
    assert link.wait_until(lambda: bool(reboots), 1)
    assert "new HELLO" in reboots[0] and "SET_POSTURE" not in link.goals()


def test_runner_forgets_its_posture_cache_on_reboot_or_reconnect():
    clock = Clock(0.0)
    body = FakeBody(clock)
    body.snap.update(reboots=0, connected=True)
    paused = {"on": False}
    runner = Runner(body, day_cfg(), clock=clock, seed=1, log=False, paused=lambda: paused["on"])

    def steps(n=3):
        for _ in range(n):
            runner.step()
            clock.advance(0.1)

    def forgot(change):
        """Apply ``change`` to the snapshot; True if the next tick dropped the
        cached posture (it is gone, or the behaviour re-sent it)."""
        runner.ctx.data["posture"] = ("cached", None)
        k = len(body.of("posture"))
        body.snap.update(change)
        steps(1)
        return runner.ctx.data.get("posture") != ("cached", None) or len(body.of("posture")) > k

    steps()
    n = len(body.of("posture"))
    assert n >= 1
    steps()
    assert len(body.of("posture")) == n  # cached: sent on change only
    assert not forgot({})  # nothing changed: the cache stays
    assert forgot({"reboots": 1})  # same epoch (0 -> 0 across a reboot is the dangerous case)
    assert forgot({"connected": False})
    assert forgot({"connected": True})
    assert forgot({"epoch": 2})
    # pause (another owner holds the body): nothing ticked; resume halts and re-asserts
    m, t = len(body.calls), body.ticks
    paused["on"] = True
    steps()
    assert len(body.calls) == m and body.ticks > t
    paused["on"] = False
    steps()
    assert len(body.of("posture")) > n


# ---- 6: logs off the control threads ------------------------------------------------------

class _SlowFile:
    def __init__(self):
        self.data = bytearray()
        self.closed = False

    def write(self, b):
        time.sleep(0.02)
        self.data += b

    def flush(self):
        time.sleep(0.05)

    def close(self):
        self.closed = True


def test_kilolog_slow_sink_does_not_delay_callers(tmp_path):
    w = KilologWriter(tmp_path / "x.kilolog")
    real = w._f
    slow = _SlowFile()
    w._f = slow  # before any record: the writer thread picks it up on its first get()
    w._thread.join(0.01)
    t0 = time.monotonic()
    for i in range(50):
        w.write_frame(0x80, i, b"\x01\x02", "rx")
        if i % 10 == 0:
            w.flush()
    took = time.monotonic() - t0
    assert took < 0.05, took  # 50 x 20 ms of disk happen elsewhere
    w.close()
    assert w.records == 50 and w.dropped == 0 and slow.closed
    real.close()
    # a real file round-trips; a full queue drops and counts instead of blocking
    w = KilologWriter(tmp_path / "y.kilolog", queue_size=4)
    w._f, keep = _SlowFile(), w._f
    t0 = time.monotonic()
    for i in range(40):
        w.write_frame(0x80, i, b"", "tx")
    assert time.monotonic() - t0 < 0.05 and w.dropped > 0
    w.close()
    keep.close()
    with KilologWriter(tmp_path / "z.kilolog") as w:
        for i in range(5):
            w.write_frame(0x80, i, b"\x07")
    assert len(list(read_kilolog(tmp_path / "z.kilolog")[1])) == 5


# ---- 7, 13, 16: client ----------------------------------------------------------------------

def test_cancelled_play_stops_the_trick(rig):
    client, vb, _ = rig()
    client.wait_ready(2)
    client.stand()
    assert _keep(client.link, lambda: vb.posture == "STAND")
    t = library.load("sniff")
    from dataclasses import replace
    loop_forever = replace(t, repeats=255)

    async def go():
        task = asyncio.ensure_future(client.play(loop_forever))
        for _ in range(100):
            await asyncio.sleep(0.02)
            if vb.trick.running:
                break
        assert vb.trick.running
        task.cancel()  # Action.halt() when the runner preempts
        with pytest.raises(asyncio.CancelledError):
            await task
    asyncio.run(go())
    assert _keep(client.link, lambda: not vb.trick.running, 1)
    done = [e for _, e in client.state.events if e.code == consts.EV["TRICK_DONE"]]
    assert done and P.TrickDoneReason((done[-1].arg & 0xFFFF) >> 8) == P.TrickDoneReason.STOPPED


def test_waits_do_not_stamp_liveness(rig):
    client, vb, _ = rig()
    client.wait_ready(2)
    link = client.link
    before = link._mind_alive_at
    th = threading.Thread(target=lambda: link.wait_until(lambda: False, 0.3))
    th.start()
    th.join()
    assert link._mind_alive_at == before  # a pool thread waiting is not the mind's loop
    client.stamp_liveness = False  # a client whose waits are not the main loop
    client.cal_save(0.5)
    assert link._mind_alive_at == before
    client.stand()
    assert _keep(link, lambda: vb.posture == "STAND")
    from dataclasses import replace
    t = replace(library.load("sniff"), repeats=255)

    async def go():
        stamp = link._mind_alive_at
        task = asyncio.ensure_future(client.play(t))
        await asyncio.sleep(0.6)
        assert link._mind_alive_at == stamp  # a looping play() does not keep a dead loop alive
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
    asyncio.run(go())


def test_concurrent_uploads_are_serialised(rig):
    client, vb, _ = rig()
    client.wait_ready(2)
    names = ["hi", "sniff", "check_around", "scratch"]
    errors, ids = [], {}

    def up(n):
        try:
            for _ in range(3):
                ids[n] = client.upload_trick(library.load(n), force=True)[0]
        except Exception as e:  # interleaved BEGIN/FRAMES/COMMIT would fail with BAD_ARG/NO_TRICK
            errors.append((n, e))
    ths = [threading.Thread(target=up, args=(n,)) for n in names]
    for th in ths:
        th.start()
    for th in ths:
        th.join(20)
    assert errors == []
    assert vb.trick.state == "ready" and vb.trick.hdr.trick_id in ids.values()
    assert client._trick_cached[0] == vb.trick.hdr.trick_id


# ---- 12: LinkStats race ------------------------------------------------------------------------

def test_link_stats_snapshot_under_concurrent_counter_growth():
    # Two writers add fresh Counter keys (each key a size change) while two readers copy the
    # stats. The growth is bounded (KEYS per writer) so each copy stays cheap, and the readers
    # copy for as long as the writers are adding keys, so the race window is the whole run.
    # (On CPython with the GIL dict(Counter) is a single C call, so the lock matters for
    # free-threaded builds and for any Python-level iteration added to as_dict() later.)
    KEYS = 4000
    st = BodyState()
    link = SerialLink(lambda: None, state=st)
    growing = threading.Event()
    growing.set()
    errors = []
    reads = [0, 0]
    start = threading.Barrier(4)

    def grow(w):
        start.wait()
        for i in range(KEYS):
            link._handle(DecodeError(f"w{w}k{i}"), 0.0)

    def read(r):
        start.wait()
        try:
            while growing.is_set() or reads[r] == 0:
                st.snapshot()
                reads[r] += 1
        except RuntimeError as e:
            errors.append(e)
    writers = [threading.Thread(target=grow, args=(w,)) for w in range(2)]
    readers = [threading.Thread(target=read, args=(r,)) for r in range(2)]
    for t in writers + readers:
        t.start()
    for t in writers:
        t.join(20)
    growing.clear()
    for t in readers:
        t.join(20)
    assert not any(t.is_alive() for t in writers + readers), "a thread did not finish"
    assert errors == []
    assert min(reads) >= 1 and sum(reads) > 20, reads  # the readers really overlapped the growth
    d = st.stats.as_dict()
    assert "lock" not in d and len(d["decode_errors"]) == 2 * KEYS


