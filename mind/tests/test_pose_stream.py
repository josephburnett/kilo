"""The walk's plumbing on the laptop: SerialLink's POSE path and the walk in
the log summary. (The walker itself is the C++ mind's: kernel/host/tests/test_walker.cpp.)"""

import time

from kilo.logs.analysis import format_summary, gait_summary
from kilo.logs.reader import LogEvent, TelemetrySample
from kilo.protocol import kilo_protocol as P

R = P.GoalRefusedReason
M = P.MsgId


def test_send_stream_bypasses_the_goal_machinery(make_link):
    link, peer = make_link()
    payload = P.Pose(epoch=0, tick=5, leg=[0, 304] * 4).pack()
    assert link.send_stream("POSE", payload) is False  # the body not heard yet: nothing goes out
    peer.hello(epoch=0)
    peer.telemetry(tick=1, epoch=0)
    assert link.wait_until(lambda: link._contact, 1)
    assert link.send_stream("POSE", payload) is True
    assert peer.wait_for(lambda: peer.of("POSE"), 1)
    assert "POSE" not in link.goals()
    time.sleep(0.5)
    assert len(peer.of("POSE")) == 1  # never refreshed, never retried
    assert link.send_stream("SET_POSTURE", payload) is False  # only Pi->body streams
    link.go_silent()
    assert link.send_stream("POSE", payload) is False


def test_log_summary_renders_the_walk():
    ss = [TelemetrySample(t=i * 0.02, tick=i, mode="STREAM", flags={"stream"}, pitch_deg=0.0, roll_deg=0.5,
                          ref_pitch_deg=0.0, ref_roll_deg=0.0, corr_pitch_deg=0.0, corr_roll_deg=0.0)
          for i in range(100)]
    ss += [TelemetrySample(t=2.0 + i * 0.02, tick=100 + i, mode="IDLE", flags=set()) for i in range(10)]
    events = [LogEvent(2.0, "STREAM_STOPPED", 1, 100, 0), LogEvent(1.9, "POSE_UNDERRUN", 1, 95, 0),
              LogEvent(0.0, "GOAL_REFUSED", M.POSE << 8 | R.STALE, 0, 0)]
    g = gait_summary(ss, events)
    assert (g["walking_s"], g["stops"], g["stopped_underrun"], g["underruns"], g["refused"]) == (
        2.0, 1, 1, 1, ["POSE: STALE"])
    assert g["error"]["offset_roll_deg"] == 0.5
    assert gait_summary(ss[100:], []) is None
    text = format_summary({"path": "x", "format": "kilolog", "duration_s": 2.2, "samples": 110, "gait": g,
                           "events": [(0.0, "GOAL_REFUSED", M.POSE << 8 | R.STALE)]})
    assert "gait: walking 2.0s, 1 STREAM_STOPPED (1 after an underrun), 1 POSE_UNDERRUN, refused POSE: STALE" in text
    assert "model error: offset pitch +0.0° roll +0.5°" in text
