"""The virtual body's POSE contract (protocol 3.0, spec POSE; kilo.vbody.pose),
driven synchronously: HELLO 3.0, POSE never acked, the gating refusals, the
queue (tick order, the drops: past, too far ahead, no room; STATS
pose_dropped), the stream on the first accepted key, the move toward each
key (on its tick, straight or eased, across a missing key), joint-mode
legs, the hold and the settle on an underrun, the ends (SET_POSTURE, an
epoch bump, a reflex: STREAM_STOPPED 0; the settle: 1), FALL_OK and the
TELEMETRY fields."""

import pytest

from kilo.planner.walker import pack_pose_feet
from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P
from kilo.vbody.sim import VirtualBody

EV = P.EventCode
R = P.GoalRefusedReason
M = P.MsgId
F = P.PoseFlags
STANCE = [0, 0, 0, 0], [76, 76, 76, 76]
OFF = -32768


class Body:
    def __init__(self, **kw):
        self.now = 100.0
        self.vb = VirtualBody(None, clock=lambda: self.now, **kw)
        self.vb.telem_div = 1
        self.vb.posture = "STANCE"
        self.seq = 0
        self.vb.take_sent()

    def raw(self, typ, payload):
        self.seq = (self.seq + 1) & 0xFF
        self.vb.process(C.build_frame(typ, self.seq, payload), self.now)
        return [(t, P.decode_payload(t, p)) for t, _, p in self.vb.take_sent()]

    def send(self, name, **fields):
        cls = P.MESSAGES[M[name]]
        return self.raw(cls.ID, cls(**fields).pack())

    def feet(self, ahead=1, x=None, z=None, pitch=0, roll=0, support=0xF, epoch=None, tick=None):
        """A POSE in foot mode (whole mm, as the walker sends)."""
        x = STANCE[0] if x is None else x
        z = STANCE[1] if z is None else z
        t = (self.vb.tick + ahead) & 0xFFFF if tick is None else tick
        return self.raw(M.POSE, pack_pose_feet(self.vb.epoch if epoch is None else epoch, t, x, z, pitch, roll,
                                               support))

    def pose(self, ahead=1, flags=0, leg=None, head=(OFF, OFF, OFF), pitch=0, roll=0, support=0xF, epoch=None):
        """A POSE with any legs (q2 feet or a16 joint angles)."""
        leg = [0, 304] * 4 if leg is None else leg
        return self.send("POSE", epoch=self.vb.epoch if epoch is None else epoch,
                         tick=(self.vb.tick + ahead) & 0xFFFF, flags=flags, leg=list(leg), head=list(head),
                         pitch=pitch, roll=roll, support=support)

    def tick(self, n=1):
        out = []
        for _ in range(n):
            self.now += 0.02
            self.vb._tick(self.now)
            out += [(t, P.decode_payload(t, p)) for t, _, p in self.vb.take_sent()]
        return out


def events(out):
    return [(m.code, m.arg) for t, m in out if t == M.EVENT]


def telemetry(out):
    return [m for t, m in out if t == M.TELEMETRY]


def test_hello_is_protocol_3_and_pose_is_never_acked():
    vb = VirtualBody(None)
    vb._boot()
    hello = [P.decode_payload(t, p) for t, _, p in vb.take_sent() if t == M.HELLO][0]
    assert (hello.proto_major, hello.proto_minor, hello.pose_depth) == (3, 0, P.POSE_DEPTH)
    b = Body()
    out = b.feet(3)
    assert out == []  # no ACK, no event
    assert b.vb.stream is not None and b.vb.mode == P.Mode.STREAM


def test_pose_gating():
    b = Body()
    assert events(b.feet(epoch=9)) == [(EV.GOAL_REFUSED, M.POSE << 8 | R.STALE)]
    assert b.vb.stream is None
    assert b.feet(epoch=9) == []  # rate-limited: one per type per second
    b.now += 1.1
    b.vb.batt_level = P.BatteryLevel.LOW
    assert events(b.feet()) == [(EV.GOAL_REFUSED, M.POSE << 8 | R.LOW_BATT)]
    u = Body(config_state=P.ConfigState.DEFAULTED)
    assert events(u.feet()) == [(EV.GOAL_REFUSED, M.POSE << 8 | R.UNCALIBRATED)]
    f = Body()
    f.vb.trigger_fall(1)
    f.vb.take_sent()
    assert events(f.feet()) == [(EV.GOAL_REFUSED, M.POSE << 8 | R.FALLEN)]
    r = Body()
    assert events(r.feet(pitch=481)) == [(EV.GOAL_REFUSED, M.POSE << 8 | R.BAD_ARG)]  # +-30 deg (stream_check)
    for bad in (dict(roll=-481), dict(z=[76, 76, 76, 111]), dict(z=[14, 76, 76, 76]), dict(x=[121, 0, 0, 0]),
                dict(support=0x1F)):
        r.now += 1.1
        assert events(r.feet(**bad)) == [(EV.GOAL_REFUSED, M.POSE << 8 | R.BAD_ARG)], bad
    r.now += 1.1
    assert events(r.pose(flags=0x80)) == [(EV.GOAL_REFUSED, M.POSE << 8 | R.BAD_ARG)]  # an unknown flag
    r.now += 1.1
    assert events(r.pose(flags=F.JOINT_LF, leg=[OFF, 0] + [0, 304] * 3)) == [
        (EV.GOAL_REFUSED, M.POSE << 8 | R.BAD_ARG)]  # a joint angle of -32768
    assert r.feet(pitch=480, roll=-480, x=[120, -120, 0, 0], z=[110, 15, 76, 76]) == [] and r.vb.stream is not None
    for posture in ("LIMP", "CALIB"):
        k = Body()
        k.vb.posture = posture
        assert events(k.feet()) == [(EV.GOAL_REFUSED, M.POSE << 8 | R.BUSY)] and k.vb.stream is None
    p1 = Body(p2=False)
    assert events(p1.feet()) == [(EV.GOAL_REFUSED, M.POSE << 8 | R.UNKNOWN_TYPE)]


def test_queue_tick_order_replacement_and_drops():
    b = Body()
    b.feet(2, x=[2, 2, 2, 2])
    b.feet(1, x=[1, 1, 1, 1])  # earlier: goes before it
    b.feet(2, x=[3, 3, 3, 3])  # the same tick again: replaces it
    b.feet(0)                  # the tick just played: dropped
    b.feet(P.POSE_MAX_AHEAD_TICKS + 1)  # too far ahead: dropped
    assert b.vb.pose_dropped == 2 and [k.tick for k in b.vb.stream.queue] == [1, 2]
    t = telemetry(b.tick())[0]
    assert t.mode == P.Mode.STREAM and t.pose_queued == 1 and t.support == 0xF
    assert b.vb.stream.x == [1, 1, 1, 1]
    b.tick()
    assert b.vb.stream.x == [3, 3, 3, 3]
    stats = [m for t, m in b.send("STATS_REQ") if t == M.STATS][0]
    assert stats.pose_dropped == 2 and stats.pose_underruns == 0
    # full: a later key is dropped, an earlier one makes the latest give way
    q = Body()
    for i in range(1, P.POSE_DEPTH + 1):
        q.feet(2 * i)
    q.feet(2 * P.POSE_DEPTH + 2)
    assert q.vb.pose_dropped == 1 and q.vb.stream.queue[-1].tick == 2 * P.POSE_DEPTH
    q.feet(3)
    assert q.vb.pose_dropped == 1 and [k.tick for k in q.vb.stream.queue] == [2, 3, 4, 6, 8]
    # the u16 tick wraps
    w = Body()
    w.vb.tick = 0xFFFE
    w.feet(tick=0x0001)
    assert w.vb.stream is not None and w.vb.pose_dropped == 0


def test_keys_far_apart_are_a_straight_line_or_an_ease():
    b = Body()
    b.feet(1)
    b.tick()
    b.feet(10, x=[10, 10, 10, 10])
    xs = []
    for _ in range(10):
        b.tick()
        xs.append(b.vb.stream.cur.a[0])
    assert xs == pytest.approx([1.0 * i for i in range(1, 11)])  # 1 mm a tick, there on its tick
    e = Body()
    e.feet(1)
    e.tick()
    e.pose(10, flags=F.EASE, leg=[40, 304] * 4)  # x 10 mm in q2
    xs = []
    for _ in range(10):
        e.tick()
        xs.append(e.vb.stream.cur.a[0])
    assert xs[-1] == pytest.approx(10.0) and xs[0] < 0.5 and xs[4] == pytest.approx(5.0, abs=0.01)
    assert xs[0] < xs[5] - xs[4]  # slow at the start, faster in the middle


def test_the_slew_limits_a_jump():
    b = Body()
    for i in range(1, 6):
        b.feet(i, x=[40, 40, 40, 40], z=[60, 60, 60, 60])
    t = telemetry(b.tick())[0]
    x = b.vb.stream.x[0]
    assert 0 < x < 40 and t.limited == 0x0F  # one tick moves each joint at most 10 degrees: TELEMETRY.limited
    b.tick(6)
    assert b.vb.stream.x[0] == 40  # ...and it gets there
    assert telemetry(b.tick())[0].limited == 0


def test_underrun_holds_then_settles():
    b = Body()
    for i in range(1, 4):
        b.feet(i, z=[76, 76, 76, 60], support=0b0111)  # LB in the air
    out = b.tick(3)
    assert events(out) == []
    out = b.tick()  # dry: the hold starts
    g = b.vb.stream
    assert events(out) == [(EV.POSE_UNDERRUN, 4)] and g.state == "hold"  # 4 ticks since the POSE arrived
    held = list(g.cur.b)
    out = b.tick(P.POSE_HOLD_TICKS - 1)  # POSE_HOLD_TICKS dry ticks in all: the last pose, held
    assert events(out) == [] and g.state == "hold" and g.cur.b == held == [76, 76, 76, 60]
    b.tick()
    assert g.state == "settle" and g.cur.b == [76, 76, 76, 62]  # then LB comes down, 2 mm a tick
    out = b.tick(6)
    assert events(out) == [] and g.cur.b[3] == 74
    out = b.tick()  # within 2 mm: on the ground, every bit set: the stream ends that tick
    assert events(out) == [(EV.STREAM_STOPPED, 1)]
    assert b.vb.stream is None and b.vb.posture == "STANCE" and b.vb.mode == P.Mode.IDLE
    assert b.vb.stream_stops == [1] and b.vb.pose_underruns == 1
    # a POSE during the hold resumes the play
    r = Body()
    r.feet(1)
    r.tick(2)
    assert r.vb.stream.state == "hold"
    r.feet(1)
    r.tick()
    assert r.vb.stream.state == "play" and r.vb.stream is not None


def test_a_posture_or_an_epoch_bump_ends_the_stream():
    b = Body()
    b.feet(1)
    b.tick()
    out = b.send("SET_POSTURE", epoch=b.vb.epoch, posture=P.Posture.SIT)
    assert events(out) == [(EV.STREAM_STOPPED, 0)] and b.vb.stream is None and b.vb.posture == "SIT"
    e = Body()
    e.feet(1)
    e.feet(2)
    e.tick()
    e.vb.trigger_fall(-1)
    out = [(t, P.decode_payload(t, p)) for t, _, p in e.vb.take_sent()]
    assert (EV.STREAM_STOPPED, 0) in events(out) and e.vb.stream is None
    stopped = [m for t, m in out if t == M.EVENT and m.code == EV.STREAM_STOPPED][0]
    assert stopped.epoch == e.vb.epoch  # the post-bump epoch


def test_telemetry_carries_reference_correction_and_support():
    b = Body()
    b.vb.balance.update(enable=1)
    for i in range(1, 6):
        b.feet(i, z=[80, 72, 72, 80], roll=-5 * 16, support=0b0111)
    t = telemetry(b.tick())[0]
    assert (t.ref_pitch, t.ref_roll, t.support) == (0, -80, 0b0111)
    assert t.flags & P.TelemetryFlags.STREAM and t.flags & P.TelemetryFlags.BALANCE_ACTIVE
    assert t.owners & 0xF == P.Owner.STREAM and t.owners >> 4 == P.Owner.IDLE  # the head left to IDLE
    t = telemetry(b.tick(3))[-1]
    assert t.pose_queued == 1
    # a foot up: the corrections are held (frozen), flag BALANCE_HELD
    assert (t.corr_pitch, t.corr_roll) == (0, 0) and t.flags & P.TelemetryFlags.BALANCE_HELD
    # four feet down, the body off its reference: it corrects (after a 2-tick hold-over)
    for i in range(1, 6):
        b.feet(i, roll=-5 * 16)
    b.tick(5)
    for i in range(1, 3):
        b.feet(i, roll=-5 * 16)
    four = telemetry(b.tick(2))
    assert four[-1].corr_roll != 0 and not four[-1].flags & P.TelemetryFlags.BALANCE_HELD
    # the balance off: the reference reads 0
    b.vb.balance.update(enable=0)
    b.feet(1, roll=-5 * 16)
    assert telemetry(b.tick())[0].ref_roll == 0


def test_a_missing_key_is_moved_across():
    """No POSE for this tick but a later one queued: the pose moves toward
    it (stream.cpp advance), no event, the hold's count does not run."""
    b = Body()
    b.feet(1, x=[5, 5, 5, 5])
    b.feet(3, x=[9, 9, 9, 9])
    b.tick()
    out = b.tick()  # tick 2: nothing for it, tick 3 queued: half-way
    assert events(out) == [] and b.vb.stream.state == "play" and b.vb.stream.cur.a == [7, 7, 7, 7]
    assert b.vb.pose_underruns == 0
    b.tick()
    assert b.vb.stream.cur.a == [9, 9, 9, 9]
    # in the hold, a key queued further ahead ends the hold: the pose moves toward it
    h = Body()
    h.feet(1)
    h.tick(2)  # tick 2: dry, the hold starts (one held tick)
    assert h.vb.stream.state == "hold" and h.vb.stream.held == 1
    h.feet(5, x=[4, 4, 4, 4])
    out = h.tick(5)
    assert h.vb.stream.state == "play" and events(out) == [] and h.vb.stream.cur.a == [4, 4, 4, 4]


def test_joint_mode_legs_move_in_joint_space_and_never_settle():
    """A stream with joint-mode legs (a trick, a get-up): the leg goes there
    in joint space, the head follows its values, and when the keys stop the
    pose is held (no settle, no end)."""
    b = Body()
    b.vb.posture = "SIT"
    sh, kn = 45 * 16, 20 * 16  # (STANCE's foot is 39, 16)
    leg = [sh, kn] + [0, 304] * 3
    out = b.pose(5, flags=F.JOINT_LF, leg=leg, head=(20 * 16, OFF, OFF))
    assert out == [] and b.vb.posture == "SIT"  # (only a stream of feet makes it STANCE)
    b.vb.wd_armed = False  # (no heartbeats here)
    t = telemetry(b.tick(5))[-1]
    g = b.vb.stream
    assert g.angles_of(0) == (45.0, 20.0) and g.angles_of(1) is None
    assert t.owners >> 4 == P.Owner.STREAM and b.vb.joint_angles()[0] == 20.0
    x, z = g.foot_of(g.cur, 0)
    assert g._angles(0, x, z) == pytest.approx((45.0, 20.0))  # the physics' foot is those angles
    out = b.tick(P.POSE_HOLD_TICKS + 30)
    assert [e for e in events(out) if e[0] == EV.STREAM_STOPPED] == [] and b.vb.mode == P.Mode.STREAM
    # a foot-mode key again: the leg goes back to a foot through joint space, then is a foot
    b.feet(4)
    b.tick(4)
    assert g.angles_of(0) is None and g.cur.a[0] == pytest.approx(0.0)


def test_fall_ok_takes_the_joints_from_the_fall_reflex():
    b = Body()
    b.vb.trigger_fall(1)
    b.vb.take_sent()
    assert b.vb.mode == P.Mode.FALLEN
    out = b.pose(5, flags=F.FALL_OK | F.JOINT_LF | F.JOINT_RF | F.JOINT_RB | F.JOINT_LB, leg=[0, 0] * 4)
    assert out == [] and not b.vb.fallen and b.vb.stream is not None and b.vb.stream.fall_ok
    assert telemetry(b.tick())[0].mode == P.Mode.STREAM


def test_the_fall_reflex_lies_down_then_goes_limp():
    b = Body()
    b.vb.posture = "STAND"
    b.vb.wd_armed = False  # (no heartbeats here)
    b.vb.trigger_fall(-1)
    out = [(t, P.decode_payload(t, p)) for t, _, p in b.vb.take_sent()]
    assert (EV.FELL, -1) in events(out)
    out = b.tick(51)  # 1 s FALLEN, holding the pose
    assert events(out) == [(EV.LYING, -1)] and b.vb.posture == "REST" and b.vb.mode == P.Mode.FALLEN
    out = b.tick(152)  # REST (1 s), LIMP 2 s later, the reflex lets go
    assert not b.vb.fallen and b.vb.posture == "LIMP" and telemetry(out)[-1].mode == P.Mode.LIMP


def test_the_settle_puts_the_foot_on_the_plane_of_the_support_feet():
    """A leaning body on three feet: the lifted foot goes to the plane
    through them (stream.cpp ground_plane), not their mean depth."""
    b = Body()
    for i in range(1, 3):
        b.feet(i, z=[70, 80, 84, 60], support=0b0111)  # LB up; LF 70, RF 80, RB 84: the plane gives 74 under LB
    b.tick(2 + 1 + P.POSE_HOLD_TICKS)
    assert b.vb.stream.state == "settle"
    out = b.tick(8)
    assert events(out) == [(EV.STREAM_STOPPED, 1)]
    assert b.vb.xfade is not None and b.vb.xfade.cur.b == [70, 80, 84, 74]  # the mean would be 78
    # two support feet: their mean depth
    t = Body()
    t.feet(1, z=[70, 60, 82, 60], support=0b0101)
    t.tick(2 + P.POSE_HOLD_TICKS)
    assert events(t.tick(10)) == [(EV.STREAM_STOPPED, 1)]
    assert t.vb.xfade.cur.b == [70, 76, 82, 76]


def test_the_end_of_the_stream_crossfades_to_stance():
    """STREAM_STOPPED, then IDLE STANCE takes the legs over XFADE_TICKS from
    where they are (feet x 0, STANCE's height, no lean), on the physics."""
    b = Body()
    for i in range(1, 6):
        b.feet(i, x=[20, 20, -20, -20], z=[70, 82, 82, 70])
    b.tick(5)
    out = b.send("SET_POSTURE", epoch=b.vb.epoch, posture=P.Posture.STANCE)
    assert events(out) == [(EV.STREAM_STOPPED, 0)] and b.vb.mode == P.Mode.IDLE
    g = b.vb.xfade
    assert g is not None and g.x == [20, 20, -20, -20]
    xs = []
    for _ in range(10):
        b.tick()
        xs.append(list(g.x))
    assert all(abs(a) <= 20 for x in xs for a in x) and abs(xs[4][0]) > 1  # gradually, not at once
    assert all(abs(a) < 0.5 for a in xs[-1]) and all(abs(z - 76) < 0.5 for z in g.z)
    assert b.vb.xfade is None  # done: STANCE as before


def test_feet_from_rest_or_sit_power_the_legs_in_stance():
    for posture in ("REST", "SIT"):
        b = Body()
        b.vb.posture = posture
        b.feet(1)
        t = telemetry(b.tick())[0]
        assert b.vb.posture == "STANCE" and t.mode == P.Mode.STREAM and t.flags & P.TelemetryFlags.SERVOS_POWERED


def test_a_reflex_ends_the_stream_at_once():
    b = Body()
    for i in range(1, 6):
        b.feet(i)
    b.tick()
    b.vb.wd_stage = 2  # the watchdog's REST: a reflex holds the joints
    out = b.tick()
    assert events(out) == [(EV.STREAM_STOPPED, 0)] and b.vb.stream is None and b.vb.xfade is None
