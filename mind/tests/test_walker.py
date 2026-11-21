"""The walk's plumbing: the body clock, the stream's ends, SerialLink's
FEET path, and the walk in the log summary."""

import time

import pytest

from kilo.body.state import BodyState
from kilo.logs.analysis import format_summary, gait_summary
from kilo.logs.reader import LogEvent, TelemetrySample
from kilo.planner.model import BodyModel
from kilo.planner.plan import Planner
from kilo.planner.walker import CLOCK_RESET_FRAMES, BodyClock, Stream
from kilo.protocol import kilo_protocol as P

EV = P.EventCode
R = P.GoalRefusedReason
M = P.MsgId


# ---- the body clock and the stream's ends, without a body -----------------------------------------

def test_body_clock_takes_the_least_delayed_frame():
    bc = BodyClock()
    assert bc.now(5.0) is None
    for i, delay in enumerate((0.004, 0.001, 0.009, 0.002)):
        bc.add(100 + i, 10.0 + i * 0.02 + delay)
    # the frame of tick 101 arrived 1 ms after its tick: body time = local + (101 x 20 ms - 10.021)
    assert bc.now(10.021) == pytest.approx(101.0)
    assert bc.now(10.031) == pytest.approx(101.5)
    bc.add(0xFFFF, 0.0)
    assert bc.tick == 0xFFFF and bc.add(2, 0.06) == 0x10002  # unwrapped across the u16 wrap


def test_body_clock_follows_a_body_stall():
    """The body's tick falls back 80 ms (a stall over 40 ms makes its
    scheduler resync): within CLOCK_RESET_FRAMES frames the estimate follows,
    instead of staying ahead for the window's 2 s (FEET more than the
    queue's depth ahead are dropped)."""
    bc = BodyClock()
    t = 10.0
    for i in range(100):
        bc.add(i, t + i * 0.02 + 0.001 + (i % 3) * 0.002)  # a little jitter
    assert bc.now(10.0 + 99 * 0.02 + 0.001) == pytest.approx(99.0)
    stall = 0.08
    for i in range(100, 100 + CLOCK_RESET_FRAMES):
        bc.add(i, t + i * 0.02 + stall + 0.001)
    rx = t + (100 + CLOCK_RESET_FRAMES - 1) * 0.02 + stall + 0.001
    assert bc.now(rx) == pytest.approx(100 + CLOCK_RESET_FRAMES - 1, abs=0.15) and bc.resets == 1
    # a late frame or two (the reader paused) does not restart it
    bc.add(105, rx + 0.02 + 0.05)
    bc.add(106, rx + 0.04 + 0.001)
    assert bc.resets == 1 and bc.now(rx + 0.04 + 0.001) == pytest.approx(106, abs=0.15)


class _Link:
    def __init__(self):
        self.state = BodyState(clock=lambda: 0.0)
        self.state.epoch = 3
        self.logger = None
        self.alive = True
        self.sent = []

    def send_stream(self, name, payload):
        self.sent.append(P.Feet.unpack(payload))
        return True

    def mind_alive(self):
        return self.alive

    def tel(self, tick, rx, mode=P.Mode.GAIT, epoch=3):
        st = self.state
        st.on_telemetry(P.Telemetry(tick=tick, mode=mode, epoch=epoch), rx)

    def event(self, code, arg=0, epoch=3):
        st = self.state
        st.events.append((0.0, P.Event(code=code, arg=arg, epoch=epoch)))
        st.event_count += 1
        st.epoch = epoch


def _stream(link, done=None):
    link.tel(50, 1.001)
    bc = BodyClock()
    bc.add(50, 1.001)
    s = Stream(link, Planner(BodyModel(), "trot", tick=55), 3, bc, done or (lambda: None))
    return s


def test_stream_sends_five_ticks_ahead_and_catches_up():
    link = _Link()
    s = _stream(link)
    assert s.wake(1.005) and [f.tick for f in link.sent] == [55]
    assert link.sent[0].epoch == 3 and link.sent[0].z_mm == [76] * 4
    link.tel(51, 1.021)
    assert s.wake(1.025) and link.sent[-1].tick == 56
    # three wakes missed: the next one sends what the body has not played yet
    link.tel(55, 1.101)
    assert s.wake(1.105) and [f.tick for f in link.sent[2:]] == [57, 58, 59, 60] and s.max_behind == 4
    # far behind: ticks already past are planned (the plan stays continuous) but not sent
    link.tel(70, 1.401)
    s.wake(1.405)
    assert s.skipped == 10 and link.sent[-1].tick == 75


@pytest.mark.parametrize("what,why", [
    (lambda l: l.event(EV.GAIT_STOPPED, 1), "settled after a FEET underrun"),
    (lambda l: l.event(EV.GAIT_STOPPED, 0), "a posture or an epoch bump"),
    (lambda l: l.event(EV.GOAL_REFUSED, M.FEET << 8 | R.FALLEN), "refused FEET: FALLEN"),
    (lambda l: l.event(EV.FELL, 1, epoch=4), "Kilo fell"),
    (lambda l: setattr(l.state, "reboots", 1), "rebooted"),
    (lambda l: setattr(l.state, "epoch", 9), "epoch changed"),
])
def test_stream_ends(what, why):
    link = _Link()
    s = _stream(link)
    assert s.wake(1.005)
    what(link)
    assert s.wake(1.025) is False and why in s.ended and not s.ok


def test_stream_ends_when_gait_never_comes_or_goes_away():
    link = _Link()
    s = _stream(link)
    link.state.history.clear()
    for i in range(40):
        link.tel(51 + i, 1.021 + i * 0.02, mode=P.Mode.IDLE)
        if not s.wake(1.025 + i * 0.02):
            break
    assert "did not enter GAIT" in s.ended
    link = _Link()
    s = _stream(link)
    link.tel(51, 1.021)
    assert s.wake(1.025) and s.gait_seen
    link.tel(52, 1.041, mode=P.Mode.IDLE)
    assert s.wake(1.045) is False and "left GAIT (mode IDLE)" in s.ended


def test_stream_stops_sending_while_the_mind_stalls():
    link = _Link()
    s = _stream(link)
    s.wake(1.005)
    link.alive = False
    n = len(link.sent)
    assert s.wake(1.025) and s.wake(1.3) and len(link.sent) == n
    assert s.wake(1.6) is False and "stalled" in s.ended


def test_stream_handover_on_the_planned_stop():
    link = _Link()
    calls = []
    s = _stream(link, lambda: calls.append("stance"))
    s.planner.stop()
    t, tick = 1.005, 50
    while s.handover_at is None and tick < 400:
        assert s.wake(t)
        tick += 1
        t += 0.02
        link.tel(tick, t - 0.004)
    assert calls == ["stance"]
    n = len(link.sent)
    link.event(EV.GAIT_STOPPED, 0)
    assert s.wake(t + 0.02) is False and s.ended == "stopped" and s.ok and len(link.sent) == n


# ---- SerialLink: FEET is a stream, not a goal ------------------------------------------------------

def test_send_stream_bypasses_the_goal_machinery(make_link):
    link, peer = make_link()
    payload = P.Feet(epoch=0, tick=5, z_mm=[76] * 4).pack()
    assert link.send_stream("FEET", payload) is False  # the body not heard yet: nothing goes out
    peer.hello(epoch=0)
    peer.telemetry(tick=1, epoch=0)
    assert link.wait_until(lambda: link._contact, 1)
    assert link.send_stream("FEET", payload) is True
    assert peer.wait_for(lambda: peer.of("FEET"), 1)
    assert "FEET" not in link.goals()
    time.sleep(0.5)
    assert len(peer.of("FEET")) == 1  # never refreshed, never retried
    assert link.send_stream("SET_POSTURE", payload) is False  # only Pi->body streams
    link.go_silent()
    assert link.send_stream("FEET", payload) is False


# ---- the log summary --------------------------------------------------------------------------------

def test_log_summary_renders_the_walk():
    ss = [TelemetrySample(t=i * 0.02, tick=i, mode="GAIT", flags={"gait"}, pitch_deg=0.0, roll_deg=0.5,
                          ref_pitch_deg=0.0, ref_roll_deg=0.0, corr_pitch_deg=0.0, corr_roll_deg=0.0)
          for i in range(100)]
    ss += [TelemetrySample(t=2.0 + i * 0.02, tick=100 + i, mode="IDLE", flags=set()) for i in range(10)]
    events = [LogEvent(2.0, "GAIT_STOPPED", 1, 100, 0), LogEvent(1.9, "FEET_UNDERRUN", 1, 95, 0),
              LogEvent(0.0, "GOAL_REFUSED", M.FEET << 8 | R.STALE, 0, 0)]
    g = gait_summary(ss, events)
    assert (g["walking_s"], g["stops"], g["stopped_underrun"], g["underruns"], g["refused"]) == (
        2.0, 1, 1, 1, ["FEET: STALE"])
    assert g["error"]["offset_roll_deg"] == 0.5
    assert gait_summary(ss[100:], []) is None
    text = format_summary({"path": "x", "format": "kilolog", "duration_s": 2.2, "samples": 110, "gait": g,
                           "events": [(0.0, "GOAL_REFUSED", M.FEET << 8 | R.STALE)]})
    assert "gait: walking 2.0s, 1 GAIT_STOPPED (1 after an underrun), 1 FEET_UNDERRUN, refused FEET: STALE" in text
    assert "model error: offset pitch +0.0° roll +0.5°" in text
