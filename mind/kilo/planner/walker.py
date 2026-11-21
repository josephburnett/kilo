"""Walking: the planner's ticks streamed to the body as FEET, 50 Hz.

``Stream`` is one walk's producer, woken every 20 ms (``Stream.wake``) by
its driver: the walking simulations of the tests (tests/walksim.py) here,
the C++ mind's walker on Kilo (kernel/mind/planner/walker.h, a port of it).

Every wake (docs/walking-and-balance.md §10 "Stream timing"):

1. New TELEMETRY (BodyState.history, by count) updates the body's clock
   estimate: body time = local time + the largest (tick x 20 ms - arrival
   time) of the last two seconds (the least delayed frame is the best
   estimate; the window follows the drift between the body's crystal and
   the Pi's clock), restarted when five frames in a row are more than a
   tick below it (the body's tick fell back after a stall: BodyClock). In GAIT each one also gives the model's error to the
   ErrorTracker (offset, lag, spike) and to the planner (``observe``).
2. New EVENTs: GAIT_STOPPED (arg 0: a posture or an epoch bump took the
   legs; arg 1: the body settled after an underrun), FEET_UNDERRUN
   (counted), GOAL_REFUSED for FEET, FELL end the stream; so do a reboot, an
   epoch change, and the mind's loop stalling (``mind_alive``) for 0.5 s.
3. Every planned tick from the last one sent up to the body's tick + 5
   (link.feet_lookahead_ticks, 100 ms) goes out at once, so a late wake
   (a garbage-collector pause, an SD-card stall) is caught up, and ticks
   the body has already played are skipped (counted, not sent).

The stream never writes to disk: its per-tick records go to the frame
log's queue (kilo.log.JsonlLogger.log only enqueues).

Stop: when the planner's stop is done (each leg stepped back under its
shoulder, the body levelled at the stance height) the stream calls
``on_done``; its driver then latches the STANCE posture, which ends GAIT on
the body (EVENT GAIT_STOPPED arg 0).
"""

from __future__ import annotations

import logging
import math
from collections import deque
from typing import Any, Callable

from kilo.link import consts
from kilo.planner import patterns as pat
from kilo.planner.errors import ErrorTracker
from kilo.planner.plan import Planner, TickPlan
from kilo.protocol import kilo_protocol as P

log = logging.getLogger("kilo.walk")

TICK_S = P.TICK_MS / 1000.0
LOOKAHEAD = P.FEET_LOOKAHEAD_TICKS
CLOCK_WINDOW = 100            # telemetry frames in the body-clock estimate (2 s at 50 Hz)
CLOCK_RESET_FRAMES = 5        # ...restarted after this many frames in a row a tick below its maximum
ENTER_TIMEOUT_S = 0.6         # the body must be in GAIT this soon after the first FEET
HANDOVER_TIMEOUT_S = 1.0      # ...and out of it this soon after the STANCE posture
MIND_STALL_S = 0.5
_FEET = P.Feet._FMT           # the FEET payload layout (generated)
_A16 = 16.0
_ATT_RANGE = (-480, 480)      # FEET pitch / roll, 1/16 deg (protocol/spec.toml: +-30 deg)


def stop_timeout(period_ms: float | None = None) -> float:
    """How long a planned stop may take: the stride's ramp-down, a cycle to
    step each leg home, the level-out and a margin (default: the slowest
    pattern's cycle). The ramp-down is the stride at 0.5 mm a tick: 2 s
    for the longest stride (the crawl's 50 mm), which the 3.5 s covers
    with the level-out; the faster trot's stop took 2.04 s of a 2.26 s
    limit before."""
    period = period_ms or max(x.period_ms for x in pat.PATTERNS.values())
    return 2 * period / 1000.0 + 3.5


class BodyClock:
    """The body's tick, estimated from TELEMETRY arrival times.

    Body time = local time + the largest (tick x 20 ms - arrival) of the
    window: the least delayed frame. The body's tick can also fall back
    against real time: a stall of its loop over 40 ms makes the scheduler
    resync (body/src/core/sched.cpp), so every later frame's offset is
    smaller by the stall. The window's old maximum would then put the
    estimate ahead of the body for the window's length (FEET sent more than
    the queue's depth ahead are dropped), so once ``CLOCK_RESET_FRAMES``
    frames in a row are all more than a tick below the maximum, the window
    restarts from them. (A burst of late frames after a pause of the mind's
    reader can do the same; the next prompt frame puts the maximum back.)"""

    def __init__(self, window: int = CLOCK_WINDOW):
        self._offs: deque = deque(maxlen=window)
        self._raw: int | None = None
        self._max = -math.inf
        self._below = 0
        self.tick: int | None = None   # unwrapped tick of the newest TELEMETRY
        self.resets = 0

    def add(self, raw_tick: int, rx_time: float) -> int:
        if self._raw is None:
            self.tick = raw_tick
        else:
            d = (raw_tick - self._raw) & 0xFFFF
            if d >= 0x8000:  # backwards: a restart (the stream ends on the reboot anyway)
                self._offs.clear()
                self._below = 0
                self.tick = raw_tick
            else:
                self.tick += d
        self._raw = raw_tick
        off = self.tick * TICK_S - rx_time
        full = len(self._offs) == self._offs.maxlen
        dropped = self._offs[0] if full else None
        self._offs.append(off)
        if dropped is not None and dropped >= self._max:
            self._max = max(self._offs)
        elif off > self._max or len(self._offs) == 1:
            self._max = off
        if off < self._max - TICK_S:
            self._below += 1
            if self._below >= CLOCK_RESET_FRAMES:  # the body's tick fell back: start again from these
                recent = list(self._offs)[-CLOCK_RESET_FRAMES:]
                self._offs.clear()
                self._offs.extend(recent)
                self._max = max(recent)
                self._below = 0
                self.resets += 1
        else:
            self._below = 0
        return self.tick

    def now(self, t: float) -> float | None:
        """The body's tick at local time t (fractional), None before any TELEMETRY."""
        if not self._offs:
            return None
        return (t + self._max) / TICK_S


def pack_feet(epoch: int, tp: TickPlan) -> bytes:
    """FEET for one planned tick; pitch and roll kept inside the field's
    range (the body refuses a FEET beyond it, BAD_ARG, which ends the walk)."""
    x, z = tp.x, tp.z
    lo, hi = _ATT_RANGE
    return _FEET.pack(epoch, tp.tick & 0xFFFF, x[0], x[1], x[2], x[3], z[0], z[1], z[2], z[3],
                      min(hi, max(lo, int(round(tp.pitch * _A16)))), min(hi, max(lo, int(round(tp.roll * _A16)))),
                      tp.support)


class Stream:
    """One walk: plan, send, watch. ``wake(now)`` is one 20 ms step; it
    returns False once the stream is over (``ended`` says why, ``ok`` whether
    it was the planned stop)."""

    def __init__(self, link: Any, planner: Planner, epoch: int, clock: BodyClock,
                 on_done: Callable[[], None], tracker: ErrorTracker | None = None):
        self.link = link
        self.state = link.state
        self.planner = planner
        self.epoch = epoch
        self.clock = clock
        self.on_done = on_done
        self.tracker = tracker or ErrorTracker()
        st = self.state
        with st.lock:
            self._tel_seen = st.telemetry_count
            self._ev_seen = st.event_count
            self._reboots = st.reboots
        self.sent = 0
        self.skipped = 0            # planned ticks already past when planned: not sent
        self.underruns = 0          # EVENT FEET_UNDERRUN seen
        self.max_behind = 0         # the most ticks one wake had to catch up
        self.last_sent: int | None = None
        self.gait_seen = False
        self.handover_at: float | None = None
        self.started_at: float | None = None
        self.stalled_since: float | None = None
        self.ended: str | None = None
        self.ok = False
        self.last_tel: Any = None
        self.pred: dict[int, tuple[float, float]] = {}  # u16 tick -> the attitude sent for it (the last 64)

    # ---- what the body reports ----------------------------------------------------------
    def _new_items(self) -> tuple[list, list]:
        st = self.state
        with st.lock:
            nt = st.telemetry_count - self._tel_seen
            ne = st.event_count - self._ev_seen
            tel = list(st.history)[-nt:] if nt > 0 else []
            evs = list(st.events)[-ne:] if ne > 0 else []
            self._tel_seen = st.telemetry_count
            self._ev_seen = st.event_count
        return tel, evs

    def _end(self, why: str, ok: bool = False) -> bool:
        if self.ended is None:
            self.ended, self.ok = why, ok
            (log.info if ok else log.warning)("walk ended: %s", why)
            self._log("walk_end", reason=why, ok=ok, sent=self.sent, skipped=self.skipped,
                      underruns=self.underruns)
        return False

    def _log(self, kind: str, **fields: Any) -> None:
        lg = self.link.logger
        if lg is not None:
            lg.log(kind, **fields)

    def ingest(self) -> None:
        tel, evs = self._new_items()
        EV = consts.EV
        for _, e in evs:
            if e.code == EV["GAIT_STOPPED"]:
                if self.handover_at is not None:
                    self._end("stopped", ok=True)
                elif e.arg == 1:
                    self._end("the body settled after a FEET underrun (the stream was late)")
                else:
                    self._end("the body stopped the gait (a posture or an epoch bump took the legs)")
            elif e.code == EV["FEET_UNDERRUN"]:
                self.underruns += 1
                self._log("walk_underrun", ticks=e.arg)
            elif e.code == EV["GOAL_REFUSED"]:
                gtype, reason = (e.arg & 0xFFFF) >> 8, e.arg & 0xFF
                if gtype == int(P.MsgId.FEET):
                    self._end(f"the body refused FEET: {consts.REFUSE_REASON.get(reason, reason)}")
            elif e.code == EV["FELL"]:
                self._end("Kilo fell")
        gait = int(P.Mode.GAIT)
        for rx, m in tel:
            tick = self.clock.add(m.tick, rx)
            self.last_tel = m
            if m.mode == gait:
                self.gait_seen = True
                ep, er = self.tracker.add(tick, m, self.pred.get(m.tick))
                self.planner.observe(ep, er)
            elif self.gait_seen and self.handover_at is None and self.ended is None:
                self._end(f"the body left GAIT (mode {consts.MODES.get(m.mode, m.mode)})")

    # ---- one wake ------------------------------------------------------------------------------
    def wake(self, now: float) -> bool:
        if self.ended is not None:
            return False
        self.ingest()
        st = self.state
        if self.ended is not None:
            return False
        if st.reboots != self._reboots:
            return self._end("the body rebooted")
        if st.epoch != self.epoch:
            return self._end(f"the body's epoch changed ({self.epoch} -> {st.epoch})")
        if self.handover_at is not None:
            if self.gait_seen and self.last_tel is not None and self.last_tel.mode != int(P.Mode.GAIT):
                return self._end("stopped", ok=True)
            if now - self.handover_at > HANDOVER_TIMEOUT_S:
                return self._end("stopped (no GAIT_STOPPED seen after the STANCE posture)", ok=True)
            return True
        if not self.link.mind_alive():
            if self.stalled_since is None:
                self.stalled_since = now
            elif now - self.stalled_since > MIND_STALL_S:
                return self._end("the mind's main loop stalled")
            return True  # nothing goes out while the mind looks dead: the body holds, then settles
        self.stalled_since = None
        body = self.clock.now(now)
        if body is None:
            return True
        cur = int(math.floor(body))
        if self.started_at is None:
            self.started_at = now
        elif not self.gait_seen and now - self.started_at > ENTER_TIMEOUT_S:
            return self._end("the body did not enter GAIT")
        p = self.planner
        target = cur + LOOKAHEAD
        behind = 0
        while p.tick <= target and not p.done:
            tp = p.next()
            if tp.tick <= cur:
                self.skipped += 1
                continue
            behind += 1
            if self.link.send_stream("FEET", pack_feet(self.epoch, tp)):
                self.sent += 1
                self.last_sent = tp.tick
                self.pred[tp.tick & 0xFFFF] = (tp.pitch, tp.roll)
                self.pred.pop((tp.tick - 64) & 0xFFFF, None)
            if self.link.logger is not None:
                self._log("plan", tick=tp.tick, x=tp.x, z=tp.z, pitch=round(tp.pitch, 2), roll=round(tp.roll, 2),
                          support=tp.support, margin=round(tp.margin, 1))
        self.max_behind = max(self.max_behind, behind)
        if p.done and self.handover_at is None:
            # every planned tick is out; the STANCE posture ends GAIT after them
            self.handover_at = now
            self.on_done()
        return True

