"""Golden vectors for the C++ behaviour layer (kernel/mind/behaviour), from
the Python runner (mind/kilo/behaviour) driving a scripted body: the
scenarios of mind/tests/test_behaviour.py (a plausible day; fall, startle,
low battery and wind-down; a shutdown request) and a few more (the packaged
personality left alone, drive noise, a pause). kernel/host/tests/test_behaviour.cpp
replays them with the same seed and must make the same choice, reason and
active path on every tick, the same drives and scores, and the same calls
on the body. Also the RNG: the first draws of random.Random(seed).

    ~/OpenCat/.venv/bin/python kernel/host/tests/gen_behaviour_vectors.py > kernel/host/tests/behaviour_vectors.txt

The runner is stepped directly (runner.step()) with the clock at exactly
T0 + k x 0.1 s, and the asyncio loop runs between ticks (so a trick's
coroutine progresses as under run()). The fake body's play() resolves once
the body has seen two more ticks, so the action succeeds on the third tick
after it started.

Lines (space separated, "-" = None):

    R SEED V1 V2 ...                random.Random(SEED).random() x n
    G SEED MU SIGMA V1 V2 ...       random.Random(SEED).gauss(MU, SIGMA) x n
    S NAME SEED T0                  a new scenario
    c KEY VALUE                     its personality (flattened; strings as they are)
    w BEHAVIOUR N F0 F1 F2 W        a weight: N factors (term indices), its weight
    s KEY VALUE                     the body's snapshot changes before the next tick (epoch, mode, battery,
                                    fallen, moving, dist, approach; event CODE ARG appends an EVENT)
    f KEY VALUE                     the percept source: face 0/1, face_pan V (no source until the first f)
    q 0|1                           paused
    h                               request_shutdown()
    T K                             tick K (clock T0 + K x 0.1)
    k CHOSEN REASON STATUS PATH...  that tick's choice and active path, when any of it changed
    d E C S A                       that tick's drives (every 10th tick and on a k line)
    o I E N G S                     that tick's scores in BEHAVIOURS order ("-": none)
    b NAME ARGS...                  a call on the body during that tick
    P                               the runner stopped for poweroff on that tick
"""

from __future__ import annotations

import asyncio
import random
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "mind"))

from kilo.behaviour import personality as pers  # noqa: E402
from kilo.behaviour.runner import Runner  # noqa: E402
from kilo.link import consts  # noqa: E402

MODES = {v: k for k, v in consts.MODES.items()}
out = sys.stdout


def emit(*parts) -> None:
    out.write(" ".join(str(p) for p in parts) + "\n")


def r(v) -> str:
    return "-" if v is None else repr(float(v))


def rng_lines() -> None:
    for seed in (0, 1, 5, 42, 3141592653, 2**40 + 7):
        g = random.Random(seed)
        emit("R", seed, *(repr(g.random()) for _ in range(8)))
    g = random.Random(9)
    emit("G", 9, "0.0", "0.25", *(repr(g.gauss(0.0, 0.25)) for _ in range(7)))


def personality_lines(cfg) -> None:
    def walk(prefix, d):
        for k, v in d.items():
            key = f"{prefix}{k}"
            if key == "weights":
                continue
            if isinstance(v, dict):
                walk(key + ".", v)
            elif isinstance(v, list):
                emit("c", key, ",".join(v))
            elif isinstance(v, str):
                emit("c", key, v)
            else:
                emit("c", key, repr(float(v)) if not isinstance(v, bool) else int(v))
    walk("", cfg)
    for b in pers.BEHAVIOURS:
        for term, w in cfg["weights"][b].items():
            fs = [pers.TERMS.index(f) for f in term.split("*")]
            emit("w", b, len(fs), *(fs + [0] * (3 - len(fs))), repr(float(w)))


class Body:
    """test_behaviour.py's FakeBody, with a play that resolves on body ticks."""

    def __init__(self):
        self.snap = {"epoch": 1, "mode": "IDLE", "battery_pct": 90.0, "fallen": False, "moving": False,
                     "dist_mm": None, "approach_mm_s": None, "events": []}
        self.ticks = 0
        self.calls = []

    def snapshot(self):
        return dict(self.snap)

    def on_tick(self):
        self.ticks += 1

    def posture(self, name, balance=None):
        self.calls.append(("posture", name, "-" if balance is None else int(balance)))

    def set_balance(self, enable):
        self.calls.append(("set_balance", int(enable)))

    def look(self, pan, tilt):
        self.calls.append(("look", repr(float(pan)), repr(float(tilt))))

    def beep(self, notes):
        self.calls.append(("beep", len(notes)))

    def play(self, trick, mirror=False):
        self.calls.append(("play", trick))
        start = self.ticks

        async def go():
            while self.ticks < start + 2:
                await asyncio.sleep(0)
        return go()


def day_cfg():
    return pers.from_dict({
        "drives": {"initial": {"energy": 0.75, "curiosity": 0.2, "sociability": 0.3, "arousal": 0.0},
                   "energy": {"drain_active": 0.01, "drain_idle": 0.002, "recover_rest": 0.01}},
        "behaviours": {"explore": {"looks": 8}},
    })


def scenario(name, cfg, seed, t0, ticks, script):
    """script(k, body, runner, emit_change) is called before tick k to change the inputs."""
    emit("S", name, seed, repr(float(t0)))
    personality_lines(cfg)
    body = Body()
    clock = {"t": t0}
    source = {"on": False, "p": {}}
    paused = {"on": False}

    def percepts():
        return dict(source["p"])

    runner = Runner(body, cfg, percepts=lambda: percepts() if source["on"] else {}, clock=lambda: clock["t"],
                    seed=seed, log=False, paused=lambda: paused["on"])

    def set_snap(key, value):
        if key == "event":
            code, arg = value
            body.snap["events"] = list(body.snap["events"]) + [(0.0, {"code": code, "arg": arg, "tick": 0,
                                                                       "epoch": body.snap["epoch"]})]
            emit("s", "event", code, arg)
            return
        body.snap[key] = value
        v = value
        if key == "mode":
            v = MODES[value]
        elif isinstance(value, bool):
            v = int(value)
        elif value is not None:
            v = repr(float(value)) if key != "epoch" else value
        emit("s", {"battery_pct": "battery", "dist_mm": "dist", "approach_mm_s": "approach"}.get(key, key),
             "-" if v is None else v)

    def set_src(key, value):
        source["on"] = True
        source["p"][key] = value
        emit("f", key, int(value) if isinstance(value, bool) else repr(float(value)))

    def pause(on):
        paused["on"] = on
        emit("q", int(on))

    hooks = {"snap": set_snap, "src": set_src, "pause": pause,
             "shutdown": lambda: (runner.request_shutdown(), emit("h"))}

    async def main():
        last = None
        for k in range(ticks):
            clock["t"] = t0 + k * 0.1
            script(k, clock["t"], hooks)
            emit("T", k)
            body.calls.clear()
            rec = runner.step()
            for c in body.calls:
                emit("b", *c)
            if rec.get("paused"):
                key = ("paused",)
            else:
                key = (rec["chosen"], rec["reason"], rec["status"], tuple(rec["path"]))
            if key != last:
                if rec.get("paused"):
                    emit("k", "paused")
                else:
                    emit("k", rec["chosen"] or "-", rec["reason"], rec["status"] or "-", *rec["path"])
                last = key
                changed = True
            else:
                changed = False
            if not rec.get("paused") and (changed or k % 10 == 0):
                d = runner.ctx.drives
                emit("d", repr(d.energy), repr(d.curiosity), repr(d.sociability), repr(d.arousal))
                sc = runner.root.last.scores
                emit("o", *(repr(sc[b]) if b in sc else "-" for b in pers.BEHAVIOURS))
            if runner.stopped_by == "poweroff" or (runner._stop and runner._poweroff_done):
                emit("P")
                break
            for _ in range(20):
                await asyncio.sleep(0)

    asyncio.run(main())


def main() -> int:
    emit("#", "generated by kernel/host/tests/gen_behaviour_vectors.py; do not edit")
    rng_lines()

    def day(k, t, h):  # test_runner_plausible_day: a face from 1360 s for 20 s
        if abs(t - 1360.0) < 0.05:
            h["src"]("face", True)
            h["src"]("face_pan", 12.0)
        if abs(t - 1380.0) < 0.05:
            h["src"]("face", False)
    scenario("plausible_day", day_cfg(), 42, 1000.0, 4200, day)

    def fall(k, t, h):  # test_runner_fall_startle_low_battery_wind_down
        snap = h["snap"]
        if k == 100:
            snap("fallen", True)
            snap("mode", "FALLEN")
            snap("epoch", 2)
            snap("event", (consts.EV["FELL"], 1))
        if k == 120:
            snap("fallen", False)
            snap("mode", "IDLE")
        if k == 200:
            snap("dist_mm", 150.0)
            snap("approach_mm_s", 600.0)
        if k == 205:
            snap("dist_mm", None)
            snap("approach_mm_s", None)
        if k == 300:
            snap("battery_pct", 15.0)
        if k == 600:
            snap("battery_pct", 6.0)
    scenario("fall_startle_low_battery_wind_down", day_cfg(), 1, 0.0, 6000, fall)

    def shutdown(k, t, h):  # test_runner_shutdown_request_and_stop
        if k == 30:
            h["shutdown"]()
    scenario("shutdown_request", day_cfg(), 1, 0.0, 200, shutdown)

    def quiet(k, t, h):  # the packaged personality on its own; a CRITICAL LOW_BATTERY event at the end
        if k == 2950:
            h["snap"]("event", (consts.EV["LOW_BATTERY"], 3))
    scenario("packaged_alone", pers.load(), 7, 50.0, 3000, quiet)

    def noisy(k, t, h):  # drive noise (gauss), a pause, petting, an approach seen by the source
        if k == 100:
            h["pause"](True)
        if k == 140:
            h["pause"](False)
        if k == 300:
            h["src"]("pet", True)
        if k == 360:
            h["src"]("pet", False)
        if k == 500:
            h["src"]("startle", 1.0)
            h["src"]("approaching", True)
        if k == 503:
            h["src"]("startle", 0.0)
            h["src"]("approaching", False)
    cfg = pers.from_dict({"drives": {"noise": 0.05}, "selector": {"min_dwell_s": 3.0}})
    scenario("noise_pause_pet", cfg, 3, 10.0, 1500, noisy)
    return 0


if __name__ == "__main__":
    sys.exit(main())
