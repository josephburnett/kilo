"""The routine in simulation: what each experiment's signal is, and whether
the fit gets back what made the data (the identifiability check).

    python -m kilo.measure.simulate plan         the experiments, their signals against the IMU, the time
    python -m kilo.measure.simulate check [-n N] the fit on N runs made from perturbed parameters
    (also ``kilo measure plan`` / ``kilo measure simulate``)

Two ways to make a run's data:

- ``synth_run``: the fit's own measurement model (kilo.measure.fit: the
  compliant statics, the servo response, the IMU offset) with parameters
  of our choosing, plus the IMU's noise (TELEMETRY's 1/16 deg steps and
  IMU_NOISE_DEG), and if asked a backlash: every hold a random offset of
  its own (servos come to rest a little differently each time). The
  routine's sweep ends early on the tip it sees, as the C++ does.
- ``physics_run``: the virtual body's walking physics (kilo.vbody.physics:
  rigid legs, a real inverted-pendulum tip, its own servo lag), with the
  centre of mass moved: a model the fit is NOT, for an honest check of
  the crossings and the tip rate.

IMU noise: today's recordings (~/kilo-logs/2026-11-21-f3020d5e, Kilo
standing still): pitch and roll come in steps of 1/16 deg and their
standard deviation over 2 s of standing is 0 to 0.03 deg (the DMP's
output is filtered); the rates' 0.05 deg/s.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

from kilo.measure import fit as F
from kilo.measure import script as SC
from kilo.measure import statics as ST
from kilo.planner import model as PM
from kilo.planner.model import BodyModel, Mass

IMU_NOISE_DEG = 0.03        # sd of a TELEMETRY pitch or roll standing still (2026-11-21)
IMU_STEP_DEG = 1.0 / 16     # TELEMETRY's resolution
RESOLVE_DEG = 3 * IMU_STEP_DEG  # an experiment is kept only if its signal is at least this (0.19 deg)


def quantise(x):
    return np.round(np.asarray(x) * 16.0) / 16.0


# ---- making runs -------------------------------------------------------------------------------------
class _Servo:
    """fit.servo, one tick at a time."""

    def __init__(self, lag_ms: float, tau_ms: float):
        self.d = max(lag_ms, 0.0) / SC.TICK_MS
        self.a = 1.0 - math.exp(-SC.TICK_MS / max(tau_ms, 1e-3))
        self.s: list = []
        self.y = None

    def step(self, s: np.ndarray) -> np.ndarray:
        self.s.append(np.asarray(s, float))
        n = len(self.s) - 1
        t = n - self.d
        if t < 0:
            u = self.s[0]
        else:
            i0 = int(math.floor(t))
            f = t - i0
            i1 = min(i0 + 1, n)
            u = self.s[i0] * (1 - f) + self.s[i1] * f
        if self.y is None:
            self.y = u.copy()
        self.y = self.y + self.a * (u - self.y)
        return self.y


class _Statics:
    """The compliant statics pose by pose with the hysteresis (statics.path), cached."""

    def __init__(self, body: ST.Body):
        self.body = body
        self.cache: dict = {}
        self.state = 0

    def __call__(self, x, z) -> np.ndarray:
        key = tuple(x) + tuple(z)
        c = self.cache.get(key)
        if c is None:
            P = ST.Poses(self.body.model, [list(x)], [list(z)])
            c = ST.candidates(P, self.body)
            self.cache[key] = c
        if not c.valid()[0, self.state]:
            self.state = int(np.argmin(c.bad[0]))
        return np.array([c.pitch[0, self.state], c.roll[0, self.state]])


def _play(source, imu: tuple, noise: float, backlash: float, seed: int, name: str, meta: dict) -> F.Run:
    """Play the script against ``source(x, z) -> (pitch, roll)`` (one call
    per tick, in order), the routine's tip detection on the measurement
    (as kernel/mind/measure/script.cpp: TELEMETRY TIP_SEEN_LAG_TICKS old)."""
    rng = np.random.default_rng(seed)
    r = SC.Runner()
    xs, zs, exps, phases, meas, pred = [], [], [], [], [], []
    hold_off = np.zeros(2)
    base = None
    base_n = 0
    prev_sup = 0xF
    model = BodyModel()
    while True:
        if not r.done:
            st = SC.STEPS[r.i]
            ex = SC.EXPERIMENTS[st.exp]
            seen = r.t - SC.TIP_SEEN_LAG_TICKS
            if r.tipped_at is None and base is not None and seen >= 0 and \
                    phases[seen] in ("lifted", "sweep") and exps[seen] == st.exp and st.phase in ("lifted", "sweep"):
                u = np.array(ex.tip_a16, float) / 16.0
                T = float(np.hypot(*u))
                if float((meas[seen] - base) @ (u / T)) >= SC.TIP_FRACTION * T:
                    r.tip(seen)
        t = r.next()
        if t is None:
            break
        st = SC.STEPS[t.step]
        if t.first and backlash > 0 and st.kind == SC.HOLD:
            hold_off = rng.normal(0.0, backlash, 2)
        y = source(t.x, t.z)
        m = quantise(np.asarray(y) + np.asarray(imu) + hold_off + rng.normal(0.0, noise, 2))
        p, rr, prev_sup = ST.rigid_rest(model, t.x, t.z, prev_sup)
        xs.append(t.x), zs.append(t.z), exps.append(st.exp), phases.append(st.phase)
        meas.append(m), pred.append((p, rr))
        if st.phase == "settle":  # the routine's baseline: the settle hold's telemetry, all four feet down
            base = m if t.first else (base * base_n + m) / (base_n + 1)
            base_n = 1 if t.first else base_n + 1
        elif st.phase == "to_start":
            base, base_n = None, 0
    n = len(xs)
    return F.Run(name=name, x=np.array(xs, float), z=np.array(zs, float), exp=np.array(exps), phase=phases,
                 meas=np.array(meas), pred=quantise(np.array(pred)), batt_mv=np.full(n, 7800.0), meta=meta)


def synth_run(truth: dict, model: BodyModel | None = None, noise: float = IMU_NOISE_DEG, backlash: float = 0.0,
              seed: int = 0, name: str = "synthetic") -> F.Run:
    """A run made by the fit's own measurement model with ``truth``
    (kilo.measure.fit names; missing ones: the model's values)."""
    model = model or BodyModel()
    v = dict(F.model_values(model))
    v.update(truth)
    statics = _Statics(F.body_of(v, model))
    servo = _Servo(v["lag_ms"], v["tau_ms"])

    def source(x, z):
        return servo.step(statics(x, z))

    return _play(source, (v["imu_pitch"], v["imu_roll"]), noise, backlash, seed, name, {"truth": v})


def moved_model(model: BodyModel, dx: float, dy: float, dz: float) -> BodyModel:
    """The model with its centre of mass moved (dx, dy, dz) mm: the frame's lump moves."""
    b = model.body
    k = model.total_g / b.grams
    return BodyModel(**{**model.__dict__, "body": Mass(b.grams, b.x + dx * k, b.y + dy * k, b.z + dz * k)})


def physics_run(dcom=(0.0, 0.0, 0.0), imu=(0.0, 0.0), lag_ms: float = 50.0, tau_ms: float = 70.0,
                noise: float = IMU_NOISE_DEG, seed: int = 0, name: str = "physics") -> F.Run:
    """A run on the virtual body's walking physics (rigid legs, a real tip),
    its centre of mass moved by dcom and the feet's edge not inset."""
    from kilo.vbody.physics import Physics

    model = moved_model(BodyModel(), *dcom)
    ph = Physics(model, lag_ms=lag_ms, tau_ms=tau_ms, inset_mm=0.0, feet=([0.0] * 4, [float(SC.HOME_Z)] * 4))

    def source(x, z):
        ph.command(list(map(float, x)), list(map(float, z)))
        ph.step()
        return np.array([ph.pitch, ph.roll])

    v = dict(F.model_values())
    v.update(imu_pitch=imu[0], imu_roll=imu[1], com_x=v["com_x"] + dcom[0], com_y=v["com_y"] + dcom[1],
             com_z=v["com_z"] + dcom[2], sag=0.0, lag_ms=lag_ms, tau_ms=tau_ms)
    return _play(source, imu, noise, 0.0, seed, name, {"truth": v, "physics": True})


# ---- the plan: each experiment's signal -------------------------------------------------------------
def crossings(model: BodyModel | None = None, body: ST.Body | None = None) -> dict[str, float]:
    """The model's crossing of each sweep (the feet's x)."""
    model = model or BodyModel()
    body = body or ST.Body(model=model)
    run = synth_run({}, model, noise=0.0)
    run.sweeps = F.find_sweeps(run)
    return {SC.EXPERIMENTS[sw.exp].name: F.model_crossing(model, body, sw) for sw in run.sweeps}


def _static_signal(run: F.Run, dv: dict, phases=None) -> np.ndarray:
    """How much the static tilt changes, tick by tick, when the parameters
    change by dv from the model's (the sweeps on the tripod until the
    model's crossing): (N, 2) deg."""
    model = BodyModel()
    v0 = F.model_values(model)
    v1 = dict(v0)
    for k, d in dv.items():
        v1[k] = v0[k] + d
    out = []
    for v in (v0, v1):
        rm = F.RunModel(run, model, v0["tip_ms"])
        out.append(rm.statics(F.body_of(v, model)))
    return out[1] - out[0]


def plan() -> str:
    """The experiments in order: what moves, how long, the parameter each
    measures and its predicted signal against the IMU."""
    model = BodyModel()
    run = synth_run({}, model, noise=0.0)
    run.sweeps = F.find_sweeps(run)
    for sw in run.sweeps:
        F.find_tip(run, sw)
    L = []
    ticks_by_exp = {}
    for e in run.exp:
        ticks_by_exp[int(e)] = ticks_by_exp.get(int(e), 0) + 1
    total = len(run)
    L.append(f"IMU: steps of {IMU_STEP_DEG:.4f} deg, noise sd {IMU_NOISE_DEG} deg standing still; an experiment "
             f"is kept if its signal is at least {RESOLVE_DEG:.2f} deg")
    L.append(f"{'experiment':14s} {'s':>5s}  what moves / measures / signal")
    # signals
    steps_sig = np.abs(_static_signal(run, {"sag": 3.0}))[run.exp == 1].max()
    lag_sig = _lag_signal(run)
    cr0 = crossings(model)
    crx = crossings(model, F.body_of(dict(F.model_values(model), com_x=F.model_values(model)["com_x"] + 1), model))
    cry = crossings(model, F.body_of(dict(F.model_values(model), com_y=1.0), model))
    crz = crossings(model, F.body_of(dict(F.model_values(model), com_z=F.model_values(model)["com_z"] + 5), model))
    for e, ex in enumerate(SC.EXPERIMENTS):
        s = ticks_by_exp.get(e, 0) * SC.TICK_MS / 1000.0
        if ex.name == "level":
            sig = "the offsets themselves (degrees); 1/16 deg resolved"
            what = "nothing: standing at STANCE"
        elif ex.name == "level_end":
            sig = "a drift since the start"
            what = "nothing"
        elif ex.name == "steps":
            what = (f"front/back legs -+{SC.STEP_PITCH_MM} mm (pitch 5.6 deg), left/right -+{SC.STEP_ROLL_MM} mm "
                    "(roll 7.0 deg), each way, 0.8 s each")
            sig = (f"a 5.6 / 7.0 deg step: lag {lag_sig:.2f} deg per ms of dead time; sag 3: "
                   f"{steps_sig:.2f} deg on the step's size")
        else:
            lifted = PM.LEG_NAMES[ex.lifted]
            sw = next(w for w in run.sweeps if w.exp == e)
            x0, x1 = run.x[sw.ticks[0], 0], run.x[sw.ticks[-1], 0]
            rolled = {"edge_rf_roll": " rolled 7.0 deg left side down", "edge_rb_roll": " rolled 5.3 deg right side down"}.get(ex.name, "")
            what = (f"{lifted} up {SC.LIFT_MM} mm{rolled}, feet swept {x0:+.0f} -> {x1:+.0f} mm "
                    f"(crossing {cr0[ex.name]:+.1f})")
            n = ex.name
            sig = (f"tip {np.hypot(*ex.tip_a16) / 16:.1f} deg; crossing moves {crx[n] - cr0[n]:+.2f} mm per mm of "
                   f"com_x, {cry[n] - cr0[n]:+.2f} per mm of com_y, {(crz[n] - cr0[n]) / 5:+.2f} per mm of com_z")
        L.append(f"{ex.name:14s} {s:5.1f}  {what}")
        L.append(f"{'':14s} {'':5s}  measures {ex.measures}; signal: {sig}")
    L.append(f"total {total * SC.TICK_MS / 1000:.1f} s of poses (no sweep ended early; each ends about 2 s sooner "
             f"when it tips near the model's crossing), plus about 1.5 s to start and 0.6 s to stand up after")
    L.append("dropped (signal below the IMU's resolution for the parameter, or none):")
    for name, why in dropped().items():
        L.append(f"  {name}: {why}")
    return "\n".join(L)


def _lag_signal(run: F.Run) -> float:
    """deg per ms of dead time at the steps' fastest change."""
    model = BodyModel()
    rm = F.RunModel(run, model, F.model_values(model)["tip_ms"])
    s = rm.statics(F.body_of(F.model_values(model), model))
    a = F.servo(s, 50.0, 70.0)
    b = F.servo(s, 51.0, 70.0)
    return float(np.abs(a - b)[run.exp == 1].max())


def dropped() -> dict[str, str]:
    """The candidates the routine leaves out, and their signals."""
    model = BodyModel()
    out = {}
    # fore-aft shift of the body, all feet down: the legs' sag shows as a pitch from the load moving
    P = ST.Poses(model, [[-20] * 4, [20] * 4], [[SC.HOME_Z] * 4] * 2)
    r = ST.rest(P, ST.Body(sag=3.0))
    out["shift (body 20 mm forward / back, four feet)"] = (
        f"sag 3 gives {abs(r.pitch[1] - r.pitch[0]):.2f} deg between them: under {RESOLVE_DEG:.2f}; the steps "
        "and the tripods see the sag better")
    P = ST.Poses(model, [[0] * 4, [0] * 4], [[60] * 4, [90] * 4])
    r = ST.rest(P, ST.Body(sag=3.0))
    out["raise / lower (all legs together)"] = (
        f"{abs(r.pitch[1] - r.pitch[0]):.3f} deg: no parameter of the model changes the tilt (a common length "
        "error is only a height)")
    # the tail: 6 g, its centre 47 mm out, swung 60 deg: the whole centre of mass moves ~0.8 mm
    shift = 6.0 * 47.0 * (math.sin(math.radians(30)) * 2) / model.total_g
    out["tail swing"] = (f"moves the centre of mass {shift:.1f} mm: on four feet {0.0:.2f} deg (a rigid "
                         "stance does not tilt), with sag 3 under 0.01 deg; its droop shows only when the tail "
                         "touches the floor")
    out["per-leg steps"] = "one leg's step on four feet rocks the body onto a diagonal: the pitch and roll steps time the same servos with a clean signal"
    return out


# ---- the identifiability check ------------------------------------------------------------------------
SPREAD = {"imu_pitch": (-3, 3), "imu_roll": (-3, 3), "com_x": (-6, 6), "com_y": (-4, 4), "com_z": (-10, 10),
          "sag": (0.5, 4.0), "twist": (-1.0, 1.0), "lag_ms": (30, 80), "tau_ms": (40, 110)}
OFFSET = {"com_x", "com_z"}  # spread around the model's value; the others: absolute


def random_truth(rng) -> dict:
    mv = F.model_values()
    v = {}
    for k, (lo, hi) in SPREAD.items():
        v[k] = rng.uniform(lo, hi) + (mv[k] if k in OFFSET else 0.0)
    return v


def check(trials: int = 20, seed: int = 1, backlash: float = 0.0, verbose: bool = False) -> dict:
    """Fit ``trials`` synthetic runs; per parameter the error's mean and
    rms, the mean +- the fit reported, and how often the truth was within
    2 +-."""
    rng = np.random.default_rng(seed)
    errs: dict[str, list] = {k: [] for k in F.ROWS}
    sds: dict[str, list] = {k: [] for k in F.ROWS}
    t0 = time.time()
    for i in range(trials):
        truth = random_truth(rng)
        truth["tip_ms"] = truth["lag_ms"] - truth["tau_ms"] * math.log(1.0 - SC.TIP_FRACTION)  # no pendulum here
        run = synth_run(truth, noise=IMU_NOISE_DEG, backlash=backlash, seed=int(rng.integers(1 << 30)))
        res = F.Fitter([run]).run()
        for k in F.ROWS:
            errs[k].append(res.values[k] - truth[k])
            sds[k].append(res.sd[k])
        if verbose:
            print(f"trial {i + 1}/{trials} ({time.time() - t0:.0f} s): " +
                  " ".join(f"{k} {res.values[k] - truth[k]:+.2f}" for k in F.ROWS), file=sys.stderr)
    out = {}
    for k in F.ROWS:
        e = np.array(errs[k])
        s = np.array(sds[k])
        out[k] = {"bias": float(e.mean()), "rms": float(np.sqrt(np.mean(e ** 2))), "sd_reported": float(s.mean()),
                  "within_2sd": float(np.mean(np.abs(e) <= 2 * s))}
    out["_trials"] = trials
    out["_backlash"] = backlash
    out["_seconds"] = time.time() - t0
    return out


def format_check(c: dict) -> str:
    L = [f"{c['_trials']} synthetic runs (IMU noise {IMU_NOISE_DEG} deg, 1/16 deg steps"
         + (f", backlash {c['_backlash']} deg per hold" if c["_backlash"] else "") + f"; {c['_seconds']:.0f} s)",
         f"{'parameter':10s} {'unit':10s} {'bias':>7s} {'rms err':>8s} {'+- said':>8s} {'within 2+-':>10s}"]
    for k in F.ROWS:
        d = c[k]
        unit = F.WHAT[k][1]
        L.append(f"{k:10s} {unit:10s} {d['bias']:+7.3f} {d['rms']:8.3f} {d['sd_reported']:8.3f} "
                 f"{100 * d['within_2sd']:9.0f}%")
    return "\n".join(L)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m kilo.measure.simulate", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("plan")
    c = sub.add_parser("check")
    c.add_argument("-n", "--trials", type=int, default=20)
    c.add_argument("--seed", type=int, default=1)
    c.add_argument("--backlash", type=float, default=0.0)
    a = ap.parse_args(argv)
    if a.cmd == "plan":
        print(plan())
    else:
        print(format_check(check(a.trials, a.seed, a.backlash, verbose=True)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
