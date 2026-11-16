"""The behaviour runner under the daemon (``kilo run --behaviour``).

Starting it. If ``kilo.behaviour.runner`` imports, the daemon calls
``run_behaviour(client, state, stop_event, log_dir)`` in its own thread (an
``async def`` is awaited on the loop instead): ``client`` is the daemon's
BodyClient, ``state`` its BodyState, ``stop_event`` a threading.Event set at
shutdown (return promptly once it is set), ``log_dir`` a pathlib.Path for
the runner's own logs (``--log-dir``, default ``~/kilo-logs``). When the
function also takes a ``percepts`` keyword it gets the daemon's
SharedPercepts (so its ``notify_self_motion()`` calls quiet the IMU detectors
for everyone, and ``kilo monitor --percepts`` sees the same percepts); a
``paused`` keyword gets the ``--override`` Event. An import failure or an
exception is logged and the daemon carries on without it.

Liveness. The runner stamps the link's liveness on each tick (as source
"mind"). The pump stamps as "daemon" every 0.2 s, but with a runner only
while the runner has ticked within ``runner_stall_s`` (1 s; 10 s of grace
for its first tick), so a hung runner trips the body's watchdog (logged
once: "behaviour runner has not ticked ...").

Runner vs client holds. While the runner is active, ``posture``,
``set_balance``, ``walk``, ``pose``, ``trick_upload`` and ``trick_play``
from clients are refused (``BEHAVIOUR_ACTIVE``) unless the request carries
``override`` (``kilo --override ...``): the runner is then paused (its
behaviours halted, nothing ticked) until that client's hold ends, and
resumes re-asserting its own goals. ``trick_stop``, ``stop_walk`` and
``release`` are always allowed.

The IR remote (kilo.daemon.remote_ir). Its A5 starts the runner at run time
(``_start_behaviour_from_pool``; ``--behaviour`` keeps its meaning: off
unless given), and its B6, A7 and C7 stop it (``_stop_behaviour_from_pool``:
the stop event, then the thread joined, so the runner's ``root.halt()`` has
cancelled what it ran); every other key is ignored while it runs. Each start
is a fresh ``run_behaviour`` call with its own stop event.
"""

from __future__ import annotations

import asyncio
import inspect
import logging
import threading
import time
from typing import Any

from kilo.link.serial_link import LinkError

log = logging.getLogger("kilo.daemon")

RUNNER_STALL_S = 1.0          # the pump stops stamping when the runner has not ticked for this long
RUNNER_START_GRACE_S = 10.0   # ... or has not ticked at all this long after it started
BEHAVIOUR_ACTIVE = ("behaviour runner active; stop it with `kilo daemon stop` (or `sudo systemctl stop kilo`) "
                    "and run `kilo run` without --behaviour, or pass --override (`kilo --override ...`) to "
                    "pause it while this command holds the body")


class BehaviourActive(LinkError):
    pass


class BehaviourMixin:
    def _start_behaviour(self) -> None:
        try:
            from kilo.behaviour.runner import run_behaviour  # type: ignore[import-not-found]
        except Exception as e:  # ImportError or a broken module
            self.behaviour_state = f"unavailable: {e}"
            log.warning("behaviour runner not started: %s", e)
            return
        self.log_dir.mkdir(parents=True, exist_ok=True)
        if self.behaviour_stop.is_set():  # a run before this one was stopped (the remote's B6 / A7 / C7)
            self.behaviour_stop = threading.Event()
        args = (self.client, self.client.state, self.behaviour_stop, self.log_dir)
        kwargs: dict[str, Any] = {}
        try:
            params = inspect.signature(run_behaviour).parameters
            if "percepts" in params:
                kwargs["percepts"] = self.percepts
            if "paused" in params:
                kwargs["paused"] = self.behaviour_paused
        except (TypeError, ValueError):
            pass
        self.behaviour_state = "running"
        self._behaviour_started_at = time.monotonic()
        # a stamp from a run before this one is not a tick of this one
        self._behaviour_stamp0 = self.link.alive_stamps.get("mind") if self.link is not None else None
        self._runner_stamping = True
        if inspect.iscoroutinefunction(run_behaviour):
            async def runner() -> None:
                try:
                    await run_behaviour(*args, **kwargs)
                    self.behaviour_state = "finished"
                except asyncio.CancelledError:
                    raise
                except Exception as e:
                    self.behaviour_state = f"failed: {e}"
                    log.exception("behaviour runner failed")
            self._behaviour_task = asyncio.create_task(runner())
        else:
            def runner_thread() -> None:
                try:
                    run_behaviour(*args, **kwargs)
                    self.behaviour_state = "finished"
                except Exception as e:
                    self.behaviour_state = f"failed: {e}"
                    log.exception("behaviour runner failed")
            self._behaviour_thread = threading.Thread(target=runner_thread, name="kilo-behaviour", daemon=True)
            self._behaviour_thread.start()
        log.info("behaviour runner started")

    @property
    def behaviour_active(self) -> bool:
        return self.behaviour_state == "running"

    async def _stop_behaviour(self) -> None:
        self.behaviour_stop.set()
        if self._behaviour_task is not None:
            try:
                await asyncio.wait_for(self._behaviour_task, 5.0)
            except (asyncio.TimeoutError, asyncio.CancelledError, Exception):
                self._behaviour_task.cancel()
        if self._behaviour_thread is not None:
            await self.loop.run_in_executor(None, self._behaviour_thread.join, 5.0)
            if self._behaviour_thread.is_alive():
                log.warning("behaviour runner did not stop within 5 s")
        self._behaviour_task = self._behaviour_thread = None
        if self.behaviour_state == "running":
            self.behaviour_state = "stopped"

    def _start_behaviour_from_pool(self) -> str | None:
        """Pool thread (the remote's A5): start the runner on the loop (it
        logs its seed and log file itself). Raises when it did not start."""
        async def start() -> None:
            self._start_behaviour()
        asyncio.run_coroutine_threadsafe(start(), self.loop).result(5.0)
        if not self.behaviour_active:
            raise LinkError(f"behaviour runner not started: {self.behaviour_state}")
        return None

    def _stop_behaviour_from_pool(self) -> None:
        """Pool thread (the remote's B6 / A7 / C7): stop the runner and wait for it."""
        asyncio.run_coroutine_threadsafe(self._stop_behaviour(), self.loop).result(10.0)

    def _stamp_liveness(self) -> None:
        """Stamp the link's liveness for the daemon loop; with a behaviour
        runner, only while the runner ticks too (it stamps as "mind" on
        every tick), so a hung runner trips the body's watchdog."""
        link = self.link
        if self.behaviour_active and self._behaviour_started_at is not None:
            last = link.alive_stamps.get("mind")
            if last is not None and last == self._behaviour_stamp0:
                last = None
            if last is None:
                ok = time.monotonic() - self._behaviour_started_at < RUNNER_START_GRACE_S
                idle = time.monotonic() - self._behaviour_started_at
            else:
                idle = link.clock() - last
                ok = idle < self.runner_stall_s
            if not ok:
                if self._runner_stamping:
                    self._runner_stamping = False
                    log.warning("behaviour runner has not ticked for %.1f s: no longer stamping the link's "
                                "liveness (the body's watchdog takes over)", idle)
                return
            if not self._runner_stamping:
                self._runner_stamping = True
                log.warning("behaviour runner ticking again: stamping liveness")
        link.mark_alive("daemon")

    # ---- the behaviour runner vs client holds (--override) -----------------------------------
    def _check_runner(self, s: Any, override: bool) -> None:
        """Refuse a client goal while the behaviour runner drives the body,
        or (``override``) pause the runner until this client's hold ends."""
        if not self.behaviour_active:
            return
        if not override:
            raise BehaviourActive(BEHAVIOUR_ACTIVE)
        if s.id not in self.pausers:
            self.pausers.add(s.id)
            if not self.behaviour_paused.is_set():
                log.info("client %d: --override, behaviour runner paused", s.id)
            self.behaviour_paused.set()

    def _resume_runner_for(self, s: Any) -> None:
        if s.id in self.pausers:
            self.pausers.discard(s.id)
            if not self.pausers:
                self.behaviour_paused.clear()
                log.info("client %d: hold ended, behaviour runner resumed", s.id)
