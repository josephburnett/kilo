"""The IR remote under the daemon (``kilo run --remote``, off by default; the owner's request 2026-10-11).

The body relays the Petoi remote as EVENT IR_KEY; the daemon maps keys to
actions on its own BodyClient (``kilo.ir.ACTIONS``, by grid: B6 stand and
stop, A7 sleep, C5 the calibration posture, A4 / B4 / C4 walk forward in
trot / crawl / walk, B7 / B5 faster / slower (into backward; turning in
place, straight forward / backward), A6 / C6 turn in place from standing and
steer while walking, C7 freeze, A5 the behaviour runner, A2 the rear-up trick
when switched on; rows 1-3 do nothing; held-key repeats and unknown keys are
ignored). The key table and what each action does are in
kilo.ir and mind/README.md "IR remote".

The event arrives on the loop (``_on_ir_key``, a link subscriber); the
action itself runs in the thread pool, one at a time (``_remote_lock``,
``kilo.ir.RemoteControl``), because it may wait for a planned stop. C7 runs
beside the action under way (it aborts the walk that action may be waiting
for, and that action then gives up). A posture or walk set by the remote is
held like the idle latch: nobody owns it and the idle REST/LIMP timeline is
stopped, so it stays until the next key or a client goal; a freeze stops the
timeline too, so the frozen STANCE is not rested or made limp behind the
owner's back. Keys are logged and ignored while the port is released and
while a client holds the posture, a trick or a walk. A5 starts the behaviour
runner (``_RunnerHook``); while it runs the remote's B6, A7 and C7 stop it,
its other keys are ignored. ``daemon`` reports ``remote`` (on/off and the
last key).
"""

from __future__ import annotations

import logging
from typing import Any

from kilo.daemon.sessions import BODY_HOLDS
from kilo.link import consts

log = logging.getLogger("kilo.daemon")


class _RunnerHook:
    """The behaviour runner as kilo.ir.RemoteControl starts and stops it (from a pool thread)."""

    def __init__(self, daemon: Any):
        self.d = daemon

    def active(self) -> bool:
        return self.d.behaviour_active

    def start(self) -> str | None:
        return self.d._start_behaviour_from_pool()

    def stop(self) -> None:
        self.d._stop_behaviour_from_pool()


class RemoteIRMixin:
    def _start_remote(self) -> None:
        from kilo.ir import RemoteControl

        self._remote_control = RemoteControl(self.client, behaviour=_RunnerHook(self))
        self.link.subscribe("event", self._on_ir_key)

    def _on_ir_key(self, msg: Any) -> None:
        """Link event callback (on the loop): EVENT IR_KEY -> a remote action."""
        if msg.code != consts.EV["IR_KEY"]:
            return
        assert self._on_loop()
        from kilo import ir

        text = ir.describe(msg.arg)
        action = ir.action_for(msg.arg)
        if action.kind == "ignore":
            log.debug("remote %s: ignored", text)
            return
        if action.kind == "none":
            log.info("remote %s: ignored (no action on this key)", text)
            return
        if self.port_released or self._shutting_down:
            return
        owner = next((o for o in map(self.goal_owner.get, BODY_HOLDS) if o is not None), None)
        if owner is not None:
            log.info("remote %s: ignored (client %d holds the body)", text, owner.id)
            return
        if action.kind in ("posture", "gait", "turn", "freeze", "trick"):
            # held like the idle latch: no owner, and no idle timeline over it
            # (for a trick above all: the timeline's LIMP stage would drop
            # Kilo mid-trick)
            self.goal_owner["WALK" if action.kind in ("gait", "turn") else "SET_POSTURE"] = None
            self.idle_since = self.idle_stage = None
        self.remote_last = text
        if action.kind == "freeze":
            self.pool.submit(self._remote_freeze, text)
        else:
            self.pool.submit(self._remote_perform, action, text)

    def _remote_perform(self, action: Any, text: str) -> None:
        """Pool thread: one remote action at a time."""
        with self._remote_lock:
            try:
                did = self._remote_control.perform(action)
                log.info("remote %s: %s", text, did)
            except Exception as e:  # LinkError, WalkError, a runner that would not start: log, keep serving
                log.warning("remote %s: %s failed: %s", text, action.kind, e)

    def _remote_freeze(self, text: str) -> None:
        """Pool thread: C7, beside the action under way (not behind the lock)."""
        try:
            log.info("remote %s: %s", text, self._remote_control.freeze())
        except Exception as e:
            log.warning("remote %s: freeze failed: %s", text, e)
