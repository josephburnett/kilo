"""The virtual body's POSE player: the mind's stream of poses as the firmware
plays it (protocol/spec.toml POSE, docs/movement-engine.md, doc §3f;
body/src/core/stream.h and body.cpp body_tick), on kilo.vbody.physics.

- **Queue.** Up to ``POSE_DEPTH`` keys in tick order. A key for tick T
  arriving after tick N was played is queued when 1 <= T - N <=
  ``POSE_MAX_AHEAD`` (a second one for a queued tick replaces it); else, or
  with the queue full and T later than every queued key, it is dropped and
  counted (STATS pose_dropped). With the queue full and T earlier, the
  latest queued key makes room.
- **Play.** The played pose ``cur`` moves toward the next key so that it
  arrives exactly on the key's tick: each tick every value moves 1 / (ticks
  left) of what is left (a straight line), or the step of a cosine ease
  (flag EASE), from the last key reached (or, the queue having been empty,
  from the pose held when the key arrived). So keys every tick are played
  on their tick, a missing one is interpolated across, and keys far apart
  are a slow move. Then every joint moves at most ``slew_deg_per_tick``
  toward it (the body's safety slew): nothing snaps. A leg cut by the slew
  or the reach is reported in TELEMETRY.limited.
- **Legs.** A foot-mode leg is a foot (x, z, from q2 to mm). A joint-mode
  leg (its JOINT flag) is two joint angles; a leg that is or becomes joint
  angles moves in joint space (a foot converted by the model's IK), and the
  physics gets its foot from the model's forward kinematics. (The sim does
  not simulate a joint-mode leg's balance in joint space: the correction
  decays while any leg is joint angles.) The head values are kept for
  JOINTS (-32768: left to IDLE).
- **Underrun** (the last key reached, none queued): EVENT POSE_UNDERRUN
  (arg = ticks since the last POSE arrived; STATS pose_underruns) and the
  pose is held; after ``POSE_HOLD_TICKS`` the body settles: every foot-mode
  foot whose support bit is clear goes down (or up) ``SETTLE_MM`` per tick
  at its x to the ground, its bit set when it is there. The ground under a
  lifted foot is the plane through the three support feet (stream.cpp
  ground_plane, in its integers, kept in 30..95 mm); with fewer down or the
  three on one line their mean depth; with none STANCE's height. The tick
  the last bit is set, with every leg a foot, the stream ends
  (``settled``: EVENT STREAM_STOPPED arg 1); with a joint-mode leg it keeps
  holding. A key that arrives meanwhile is played toward from the held pose.
- **The end of a stream** (settled, a SET_POSTURE STANCE, or an epoch
  bump): IDLE STANCE takes the legs over ``XFADE_TICKS`` ticks (the
  arbiter's crossfade, from the output at the hand-over, in joint space, on
  a cosine) to the STANCE feet (x 0, STANCE's height, no lean), the safety
  slew after it (``handover``). A reflex takes the legs at once instead
  (the sim drops the player).
- **Balance** (doc §3f, as body/src/core/balance.h and the geometric loop
  of body.cpp): with the balance on (SET_BALANCE enable or the posture
  flag) and every leg a foot, per axis the ground slope s = measured -
  correction - reference, e = deadband(s), u = Kp e + Kd rate, the
  correction low-passes toward -u by 1/4 per tick, moves at most 1 deg per
  tick and is clamped; the reference is the pose's pitch/roll. It is held
  (frozen) while any foot is up and for 2 ticks after all four are down
  (body.cpp: on three feet the corrections act through the feet a tipping
  body pivots on); with the balance off it decays. While the stream plays
  it acts through the support feet only, each leg's share ramping a quarter
  per tick as its support bit changes: z_i += share_i (end_i (Lb/2)
  sin(pitch corr) + side_i (W/2) sin(roll corr)) (end -1 front, +1 back;
  side -1 left, +1 right); in STANCE (the crossfade) through all four.

Where it differs from the firmware: each stream starts a new physics,
standing level on STANCE's feet, and its first key is moved toward from
those feet, not from the joints' outputs in joint space;
tests/test_corebody.py lists it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from kilo.planner.model import FRONT, LEFT, LEGS, BodyModel
from kilo.protocol import kilo_protocol as P
from kilo.vbody.physics import Physics

POSE_DEPTH = P.POSE_DEPTH
POSE_HOLD_TICKS = P.POSE_HOLD_TICKS
POSE_MAX_AHEAD = P.POSE_MAX_AHEAD_TICKS
BAL_HOLD_TICKS = 2
SETTLE_MM = 2.0  # a lifted foot's descent per tick when settling (stream.h SETTLE_Q2)
JOINT_LEGS = 0x0F
HEAD_IDLE = -32768
F = P.PoseFlags

PLAYING = ("play", "hold", "settle")


def _mm(v: float) -> int:
    """A q2 length to the nearest whole mm, as the firmware rounds ((q2 + 2) >> 2)."""
    return math.floor(v + 0.5)


def ease_cos(i: int, n: int) -> float:
    """fx_ease_cos: 0..1 over i = 0..n, zero slope at both ends."""
    if i >= n:
        return 1.0
    return (1.0 - math.cos(math.pi * i / n)) / 2.0


@dataclass
class Key:
    """A POSE key (or the pose played): per leg its two values, a foot (x, z
    mm) or, with its bit in ``flags``, joint angles (shoulder, knee deg)."""
    tick: int                       # unwrapped body tick
    flags: int = 0
    support: int = 0xF
    a: list = field(default_factory=lambda: [0.0] * 4)   # x mm, or the shoulder deg
    b: list = field(default_factory=lambda: [0.0] * 4)   # z mm, or the knee deg
    head: list = field(default_factory=lambda: [HEAD_IDLE] * 3)  # a16, HEAD_IDLE = left to IDLE
    pitch: float = 0.0              # deg
    roll: float = 0.0

    def copy(self) -> "Key":
        return Key(self.tick, self.flags, self.support, list(self.a), list(self.b), list(self.head), self.pitch,
                   self.roll)

    def joint(self, leg: int) -> bool:
        return bool(self.flags >> leg & 1)


def key_of(msg: Any, tick: int) -> Key:
    """A POSE message as a Key at unwrapped ``tick``."""
    k = Key(tick, int(msg.flags), int(msg.support), head=list(msg.head), pitch=msg.pitch / 16.0, roll=msg.roll / 16.0)
    for leg in LEGS:
        u, v = msg.leg[2 * leg], msg.leg[2 * leg + 1]
        if k.joint(leg):
            k.a[leg], k.b[leg] = u / 16.0, v / 16.0
        else:
            k.a[leg], k.b[leg] = u / 4.0, v / 4.0
    return k


class PosePlayer:
    # the walking physics new players use: Physics, or a callable with its signature (the long-trot
    # study puts kilo.vbody.rocking's there, mind/tests/stridesim.py)
    PHYSICS = Physics

    def __init__(self, tick: int, model: BodyModel, lag_ms: float | None = None, tau_ms: float | None = None):
        self.model = model
        self.queue: list[Key] = []          # in tick order, queue[0] next
        self.state = "play"                 # play, hold, settle; xfade once the stream has ended
        self.held = 0                       # hold: ticks held
        self.stance_h = model.stance_height(P.GAIT_STANCE_H_256)
        self.x = [0.0] * 4                  # commanded feet (after the slew): STANCE's
        self.z = [self.stance_h] * 4
        self.cur = Key(tick, 0, 0xF, list(self.x), list(self.z))  # the pose played
        self.physics = PosePlayer.PHYSICS(model, lag_ms=lag_ms, feet=(self.x, self.z), tau_ms=tau_ms)
        self.corr = [0.0, 0.0]              # pitch, roll correction (deg)
        self._lp = [0.0, 0.0]
        self._hold = 0
        self.support = 0xF
        self.share = [1.0] * 4              # each leg's share of the correction, 0..1 in quarters
        self.rx_tick = tick                 # the body's tick when the last POSE was queued
        self.fall_ok = False                # the newest key had FALL_OK
        self.limited = 0                    # legs the slew or the reach cut this tick
        self._xsrc: list | None = None      # xfade: the joint angles at the hand-over
        self._xi = 0
        self._xn = 0

    @property
    def playing(self) -> bool:
        """The stream holds the joints (stream_on)."""
        return self.state in PLAYING

    @property
    def ref(self) -> tuple[float, float]:
        """The attitude the pose played predicts (the balance's reference)."""
        return (self.cur.pitch, self.cur.roll) if self.playing else (0.0, 0.0)

    @property
    def feet_only(self) -> bool:
        """Every leg of the pose played is a foot."""
        return not self.cur.flags & JOINT_LEGS

    @property
    def balance_bit(self) -> bool:
        """The next key (or the pose held) has BALANCE (stream_balance)."""
        k = self.queue[0] if self.queue else self.cur
        return bool(k.flags & F.BALANCE)

    # ---- queue ---------------------------------------------------------------------------
    def offer(self, msg: Any, now_tick: int) -> bool:
        """Queue a POSE (its u16 tick against the body's unwrapped tick, the
        tick last played). False: dropped (past, too far ahead, no room)."""
        ahead = (msg.tick - now_tick) & 0xFFFF
        q = self.queue
        i = 0
        while i < len(q) and q[i].tick - now_tick < ahead:
            i += 1
        same = i < len(q) and q[i].tick == now_tick + ahead
        if ahead == 0 or ahead > POSE_MAX_AHEAD or (not same and i == POSE_DEPTH):
            return False
        was_empty = not q
        k = key_of(msg, now_tick + ahead)
        if same:
            q[i] = k
        else:
            if len(q) == POSE_DEPTH:
                q.pop()  # the latest key makes room
            q.insert(i, k)
        if was_empty:
            self.cur.tick = now_tick  # the move toward it starts now, from the pose held
        self.fall_ok = bool(msg.flags & F.FALL_OK)
        self.rx_tick = now_tick
        return True

    def queued(self) -> int:
        return len(self.queue)

    # ---- the end of the stream -------------------------------------------------------------------
    def handover(self, ticks: int) -> None:
        """The stream is over: IDLE STANCE crossfades the legs from where they are over ``ticks``."""
        m = self.model
        self.state = "xfade"
        self.queue.clear()
        self._xsrc = [m.ik(leg, self.x[leg], self.z[leg]) for leg in LEGS]
        self._xi, self._xn = 0, max(1, int(ticks))
        self.support = 0xF
        self.cur.flags &= ~JOINT_LEGS
        self.cur.pitch = self.cur.roll = 0.0

    @property
    def xfade_done(self) -> bool:
        return self.state == "xfade" and self._xi >= self._xn

    # ---- joint space --------------------------------------------------------------------------
    def _angles(self, leg: int, x: float, z: float) -> tuple[float, float]:
        """A foot as joint angles (deg, OpenCat convention; core/ik.h before the per-joint dir)."""
        t1, t2 = self.model.ik(leg, x, z)
        sigma = 1.0 if FRONT[leg] else -1.0
        return -math.degrees(t1), -(math.degrees(t2) - sigma * 90.0)

    def _foot(self, leg: int, sh: float, kn: float) -> tuple[float, float]:
        """Joint angles (deg) as a foot (the inverse of _angles)."""
        sigma = 1.0 if FRONT[leg] else -1.0
        return self.model.fk(math.radians(-sh), math.radians(sigma * 90.0 - kn))

    def foot_of(self, k: Key, leg: int) -> tuple[float, float]:
        return self._foot(leg, k.a[leg], k.b[leg]) if k.joint(leg) else (k.a[leg], k.b[leg])

    def angles_of(self, leg: int) -> tuple[float, float] | None:
        """The pose played's joint angles for a joint-mode leg (None: a foot)."""
        return (self.cur.a[leg], self.cur.b[leg]) if self.cur.joint(leg) else None

    # ---- one tick ----------------------------------------------------------------------------
    def _advance(self, now_tick: int) -> None:
        """stream.cpp advance(): one tick toward queue[0]."""
        c, k = self.cur, self.queue[0]
        m, i = k.tick - c.tick, now_tick - c.tick
        w = 1.0
        if i < m:
            if k.flags & F.EASE:
                e0, e1 = ease_cos(i - 1, m), ease_cos(i, m)
                w = (e1 - e0) / (1.0 - e0) if e0 < 1.0 else 1.0
            else:
                w = 1.0 / (m - i + 1)
        for j in range(3):
            if k.head[j] == HEAD_IDLE:
                c.head[j] = HEAD_IDLE
            elif c.head[j] == HEAD_IDLE:
                c.head[j] = k.head[j]  # (the firmware starts from the joint's output)
            else:
                c.head[j] += (k.head[j] - c.head[j]) * w
        for leg in LEGS:
            ta, tb = k.a[leg], k.b[leg]
            if c.joint(leg) or k.joint(leg):  # in joint space
                if not k.joint(leg):
                    ta, tb = self._angles(leg, ta, tb)
                if not c.joint(leg):
                    c.a[leg], c.b[leg] = self._angles(leg, c.a[leg], c.b[leg])
                c.flags |= 1 << leg
            c.a[leg] += (ta - c.a[leg]) * w
            c.b[leg] += (tb - c.b[leg]) * w
        c.pitch += (k.pitch - c.pitch) * w
        c.roll += (k.roll - c.roll) * w
        self.support = c.support & k.support
        if i < m:
            return
        self.cur = k.copy()  # reached: the next segment starts here
        self.support = k.support
        self.queue.pop(0)

    def tick(self, now_tick: int, measured: tuple[float, float], rates: tuple[float, float], balance: dict,
             balance_on: bool) -> tuple[str | None, int]:
        """Play tick ``now_tick`` (stream_step, then the balance and the legs).
        Returns (what happened, arg): ("underrun", ticks since the last
        POSE), ("settled", 0), or (None, 0)."""
        happened: tuple[str | None, int] = (None, 0)
        if self.state != "xfade":
            if self.queue:
                self.state = "play"
                self.held = 0
                self._advance(now_tick)
            else:
                if self.state == "play":  # the last key reached, none queued: hold
                    self.state = "hold"
                    happened = ("underrun", now_tick - self.rx_tick)
                if self.state == "hold":
                    self.held += 1
                    if self.held > POSE_HOLD_TICKS:
                        self.state = "settle"
                if self.state == "settle" and self._settle():
                    happened = ("settled", 0)
        if self.playing:  # each leg's share of the correction: a quarter per tick toward its support bit
            for leg in LEGS:
                up = 0.25 if self.support >> leg & 1 else -0.25
                self.share[leg] = min(1.0, max(0.0, self.share[leg] + up))
        self._balance(measured, rates, balance, balance_on and (self.feet_only or not self.playing))
        cp, cr = math.radians(self.corr[0]), math.radians(self.corr[1])
        half_l, half_w = self.model.length_mm / 2, self.model.track_mm / 2
        if self.playing:
            tx, tz = [], []
            for leg in LEGS:
                x, z = self.foot_of(self.cur, leg)
                tx.append(x)
                tz.append(z)
            share = [0.0 if self.cur.joint(leg) else self.share[leg] for leg in LEGS]
        else:  # STANCE (the crossfade after the stream): all four correct
            tx, tz = [0.0] * 4, [self.stance_h] * 4
            share = [1.0] * 4
        for leg in LEGS:
            end = -1.0 if FRONT[leg] else 1.0
            side = -1.0 if LEFT[leg] else 1.0
            tz[leg] += share[leg] * (end * half_l * math.sin(cp) + side * half_w * math.sin(cr))
        if self.state == "xfade" and self._xi < self._xn:  # the arbiter's crossfade, joint space
            self._xi += 1
            e = ease_cos(self._xi, self._xn)
            m = self.model
            for leg in LEGS:
                a1, a2 = self._xsrc[leg]
                b1, b2 = m.ik(leg, tx[leg], tz[leg])
                tx[leg], tz[leg] = m.fk(a1 + (b1 - a1) * e, a2 + (b2 - a2) * e)
        self._slew(tx, tz)
        self.physics.command(self.x, self.z)
        return happened

    def _foot_xy2(self, leg: int) -> tuple[int, int]:
        """stream.cpp foot_xy2: the foot in the body's plane in doubled mm (X
        forward from the centre, Y to the right)."""
        lb, w = int(round(self.model.length_mm)), int(round(self.model.track_mm))
        return (lb if FRONT[leg] else -lb) + math.floor(2 * self.cur.a[leg] + 0.5), (-w if LEFT[leg] else w)

    def _ground_plane(self, leg: int, abc: list) -> int:
        """stream.cpp ground_plane: the depth (mm, 30..95) of the plane through
        the three support feet ``abc`` at foot ``leg``, in the firmware's
        integers; -1 when the three are on one line."""
        zs = self.cur.b
        X, Y = [0] * 4, [0] * 4
        for k, lg in enumerate((*abc, leg)):
            X[k], Y[k] = self._foot_xy2(lg)
        z = [_mm(zs[lg]) for lg in abc]
        for k in (1, 2, 3):
            X[k] -= X[0]
            Y[k] -= Y[0]
        d = X[1] * Y[2] - Y[1] * X[2]
        if not d:
            return -1
        num = (X[3] * Y[2] - Y[3] * X[2]) * (z[1] - z[0]) + (X[1] * Y[3] - Y[1] * X[3]) * (z[2] - z[0])
        if d < 0:
            d, num = -d, -num
        q = min(127, (abs(num) + d // 2) // d)
        q = z[0] + (-q if num < 0 else q)
        return 30 if q < 30 else 95 if q > 95 else q

    def _settle(self) -> bool:
        """stream.cpp settle(): every foot-mode foot off the ground goes toward
        the ground at its x by SETTLE_MM a tick, its bit set when it is there.
        True when every leg is a foot and all four are down. The ground: the
        plane through the support feet when three are down, else (or a
        degenerate triangle) their mean depth, STANCE's default height when
        none are down."""
        c = self.cur
        zs = c.b
        down = [leg for leg in LEGS if c.support >> leg & 1]
        n = len(down)
        mean = (sum(_mm(zs[leg]) for leg in down) + n // 2) // n if n else int(self.stance_h)
        done = self.feet_only
        for leg in LEGS:
            if (c.support | c.flags) >> leg & 1:
                continue
            p = self._ground_plane(leg, down) if n == 3 else -1
            g = mean if p < 0 else p
            z = zs[leg]
            if z + SETTLE_MM < g:
                zs[leg] = z + SETTLE_MM
                done = False
            elif z > g + SETTLE_MM:
                zs[leg] = z - SETTLE_MM
                done = False
            else:
                zs[leg] = float(g)
                c.support |= 1 << leg
        self.support = c.support
        return done

    def _slew(self, tx, tz) -> None:
        m = self.model
        lim = math.radians(m.slew_deg_per_tick)
        self.limited = 0
        for leg in LEGS:
            a1, a2 = m.ik(leg, self.x[leg], self.z[leg])
            b1, b2 = m.ik(leg, tx[leg], tz[leg])
            n1 = a1 + max(-lim, min(lim, b1 - a1))
            n2 = a2 + max(-lim, min(lim, b2 - a2))
            if (n1, n2) == (b1, b2) and m.reachable(tx[leg], tz[leg]):
                self.x[leg], self.z[leg] = tx[leg], tz[leg]
            else:
                self.x[leg], self.z[leg] = m.fk(n1, n2)
                self.limited |= 1 << leg

    def _balance(self, measured, rates, bal: dict, on: bool) -> None:
        """body.cpp's geometric loop and balance.h's step on both axes (pitch, roll)."""
        if self.playing and self.support & 0xF != 0xF:  # a foot up: held
            self._hold = BAL_HOLD_TICKS + 1
        elif self._hold:
            self._hold -= 1
        if on and self._hold:
            return
        kp = bal["kp_q8"] / 256.0
        kd = bal["kd_ms"] / 1000.0
        db = float(bal["deadband_deg"])
        mx = float(bal["max_corr_deg"])
        ref = self.ref
        for i in (0, 1):
            if on:
                # the geometric loop (body.cpp): the ground slope, the correction's own effect removed
                e = measured[i] - self.corr[i] - ref[i]
                e = 0.0 if abs(e) <= db else e - math.copysign(db, e)
                u = kp * e + kd * rates[i]
                target = max(-mx, min(mx, -u))
            else:
                target = 0.0
            self._lp[i] += (target - self._lp[i]) / 4.0
            self.corr[i] += max(-1.0, min(1.0, self._lp[i] - self.corr[i]))
            self.corr[i] = max(-mx, min(mx, self.corr[i]))
