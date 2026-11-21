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
  its own (servos come to rest a little differently each time; the first
  run on Kilo, 2026-11-21, had the returns to the home pose differ by sd
  0.2 to 0.5 deg). The routine's sweep ends early on the tip it sees
  (script.TipWatch), as the C++ does.
- ``physics_run``: the virtual body's walking physics (kilo.vbody.physics:
  rigid legs, a real inverted-pendulum tip, its own servo lag), with the
  centre of mass moved: a model the fit is NOT, for an honest check of
  the crossings and the tip rate.

IMU noise: the recordings of 2026-11-21 (~/kilo-logs/2026-11-21-f3020d5e,
Kilo standing still): pitch and roll come in steps of 1/16 deg and their
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
BACKLASH_DEG = 0.3          # what a hold's own offset is like on Kilo (the first run's returns to home)


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


def _play(source, imu: tuple, noise: float, backlash: float, seed: int, name: str, meta: dict,
          script: SC.ScriptDef = SC.SCRIPT) -> F.Run:
    """Play the script against ``source(x, z) -> (pitch, roll)`` (one call
    per tick, in order), the routine's tip detection on the measurement
    (script.TipWatch, as kernel/mind/measure/measure.cpp: TELEMETRY
    TIP_SEEN_LAG_TICKS old)."""
    rng = np.random.default_rng(seed)
    r = SC.Runner(script)
    watch = SC.TipWatch(script)
    xs, zs, exps, phases, steps, intos, meas, pred = [], [], [], [], [], [], [], []
    hold_off = np.zeros(2)
    prev_sup = 0xF
    model = BodyModel()
    fed = 0  # the measured ticks the watch has seen so far
    tips = []
    while True:
        if not r.done:
            seen = r.t - SC.TIP_SEEN_LAG_TICKS
            while fed <= seen:
                got = watch.see(fed, steps[fed], intos[fed], float(meas[fed][0]), float(meas[fed][1]))
                if got is not None and r.tip(fed):
                    tips.append({"exp": script.experiments[exps[fed]].name, "seen": fed, "how": got})
                fed += 1
        t = r.next()
        if t is None:
            break
        st = script.steps[t.step]
        if t.first and backlash > 0 and st.kind == SC.HOLD:
            hold_off = rng.normal(0.0, backlash, 2)
        y = source(t.x, t.z)
        m = quantise(np.asarray(y) + np.asarray(imu) + hold_off + rng.normal(0.0, noise, 2))
        p, rr, prev_sup = ST.rigid_rest(model, t.x, t.z, prev_sup)
        xs.append(t.x), zs.append(t.z), exps.append(st.exp), phases.append(st.phase)
        steps.append(t.step), intos.append(t.into)
        meas.append(m), pred.append((p, rr))
    n = len(xs)
    meta = dict(meta, tips=tips)
    return F.Run(name=name, x=np.array(xs, float), z=np.array(zs, float), exp=np.array(exps), phase=phases,
                 meas=np.array(meas), pred=quantise(np.array(pred)), batt_mv=np.full(n, 7800.0), meta=meta,
                 script=script, step=np.array(steps))


def synth_run(truth: dict, model: BodyModel | None = None, noise: float = IMU_NOISE_DEG, backlash: float = 0.0,
              seed: int = 0, name: str = "synthetic", script: SC.ScriptDef = SC.SCRIPT) -> F.Run:
    """A run made by the fit's own measurement model with ``truth``
    (kilo.measure.fit names; missing ones: the model's values)."""
    model = model or BodyModel()
    v = dict(F.model_values(model))
    v.update(truth)
    statics = _Statics(F.body_of(v, model))
    servo = _Servo(v["lag_ms"], v["tau_ms"])

    def source(x, z):
        return servo.step(statics(x, z))

    return _play(source, (v["imu_pitch"], v["imu_roll"]), noise, backlash, seed, name, {"truth": v}, script)


def moved_model(model: BodyModel, dx: float, dy: float, dz: float) -> BodyModel:
    """The model with its centre of mass moved (dx, dy, dz) mm: the frame's lump moves."""
    b = model.body
    k = model.total_g / b.grams
    return BodyModel(**{**model.__dict__, "body": Mass(b.grams, b.x + dx * k, b.y + dy * k, b.z + dz * k)})


def physics_run(dcom=(0.0, 0.0, 0.0), imu=(0.0, 0.0), lag_ms: float = 50.0, tau_ms: float = 70.0,
                noise: float = IMU_NOISE_DEG, seed: int = 0, name: str = "physics",
                script: SC.ScriptDef = SC.SCRIPT) -> F.Run:
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
    return _play(source, imu, noise, 0.0, seed, name, {"truth": v, "physics": True}, script)


# ---- the plan: each experiment's signal -------------------------------------------------------------
def crossings(model: BodyModel | None = None, body: ST.Body | None = None,
              script: SC.ScriptDef = SC.SCRIPT) -> dict[str, float]:
    """The model's crossing of each sweep (the feet's x)."""
    model = model or BodyModel()
    body = body or ST.Body(model=model)
    run = synth_run({}, model, noise=0.0, script=script)
    run.sweeps = F.find_sweeps(run)
    return {sw.name: F.model_crossing(model, body, sw) for sw in run.sweeps}


def _static_signal(run: F.Run, dv: dict) -> np.ndarray:
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
    full = SC.poses()
    L = []
    by_exp: dict[int, int] = {}
    for t in full:
        e = SC.STEPS[t.step].exp
        by_exp[e] = by_exp.get(e, 0) + 1
    L.append(f"script {SC.SCRIPT_VERSION}. IMU: steps of {IMU_STEP_DEG:.4f} deg, noise sd {IMU_NOISE_DEG} deg "
             f"standing still; Kilo comes back to a pose within about {BACKLASH_DEG} deg; an experiment is kept if "
             f"its signal is at least {RESOLVE_DEG:.2f} deg")
    L.append(f"{'experiment':15s} {'s':>5s}  what moves / measures / signal")

    def sig(exp: str, dv: dict, ch: int | None = None) -> float:
        s = np.abs(_static_signal(run, dv))[run.exp == SC.SCRIPT.index(exp)]
        return float(s.max() if ch is None else s[:, ch].max())

    lag_sig = _lag_signal(run)
    mv = F.model_values(model)
    cr0 = crossings(model)
    crx = crossings(model, F.body_of(dict(mv, com_x=mv["com_x"] + 1), model))
    cry = crossings(model, F.body_of(dict(mv, com_y=1.0), model))
    crz = crossings(model, F.body_of(dict(mv, com_z=mv["com_z"] + 5), model))
    for e, ex in enumerate(SC.EXPERIMENTS):
        s = by_exp.get(e, 0) * SC.TICK_MS / 1000.0
        n = ex.name
        if n == "level":
            what = "nothing: standing at STANCE"
            sg = "the offsets themselves (degrees); 1/16 deg resolved"
        elif n == "level_end":
            what = "nothing"
            sg = "a drift since the start"
        elif n == "stairs_pitch":
            what = (f"front legs shorter and back longer by 2, 4, {SC.STEP_PITCH_MM} mm (to 5.6 deg) and the other "
                    f"way; each level held {SC.STAIR_HOLD * SC.TICK_MS / 1000:.2f} s, reached from below and from above")
            sg = (f"{sig(n, {'length': 10.0}, 0):.2f} deg per 10 mm of length at the top stair; a {SC.STEP_PITCH_MM} "
                  f"mm jump: lag {lag_sig:.2f} deg per ms of dead time; sag 3: {sig(n, {'sag': 3.0}):.2f} deg")
        elif n == "stairs_roll":
            what = f"left legs shorter and right longer by 2, {SC.STEP_ROLL_MM} mm (to 7.0 deg) and the other way"
            sg = (f"{sig(n, {'track': 5.0}, 1):.2f} deg per 5 mm of track at the top stair; sag 3: "
                  f"{sig(n, {'sag': 3.0}):.2f} deg")
        elif n == "twist":
            what = (f"feet {-SC.TWIST_X} mm back (the body forward); LF and RB longer, RF and LB shorter by 1, "
                    f"{SC.TWIST_MM} mm and the other way")
            sg = (f"a built-in twist of 0.5 mm moves the V's point by as much: {sig(n, {'twist': 0.5}):.2f} deg "
                  f"at a step; sag 3: {sig(n, {'sag': 3.0}):.2f} deg")
        elif n == "splay":
            what = (f"nose down ({SC.SPLAY_PITCH_MM} mm, 5.6 deg), then the front feet {SC.SPLAY_OUT_MM} mm forward "
                    f"and the back as far back, then {SC.SPLAY_IN_MM} mm inward, twice")
            sg = (f"{sig(n, {'x_scale': 0.1}, 0):.2f} deg per 10 % of x scale: a check to about 10 %, on its own "
                  "(the fit keeps the x scale 1 and leaves these poses out)")
        elif n == "check":
            what = (f"{len(SC.CHECK_POSES)} held poses: pitch with roll, both ways; forward and rolled; back and "
                    "pitched; LB up; RF up and rolled")
            sg = "none wanted: the fit does not use them, it is tested on them"
        else:
            lifted = PM.LEG_NAMES[ex.lifted]
            sw = next(w for w in run.sweeps if w.exp == e)
            st = next(q for q in SC.STEPS if q.exp == e and q.kind == SC.SWEEP)
            x0, x1 = sw.x0 + sw.direction, st.dx[0]
            rolled = ""
            if ex.roll_mm:
                side = "left" if ex.roll_mm > 0 else "right"
                rolled = f" rolled {abs(ex.roll_mm)} mm ({abs(_roll_deg(ex.roll_mm)):.1f} deg) {side} side down"
            what = (f"{lifted} up {SC.LIFT_MM} mm{rolled}, held {SC.LIFTED_TICKS * SC.TICK_MS / 1000:.1f} s, feet "
                    f"swept {x0:+.0f} -> {x1:+.0f} mm (crossing {cr0[n]:+.1f})")
            sg = (f"tip {np.hypot(*ex.tip_a16) / 16:.1f} deg; crossing moves {crx[n] - cr0[n]:+.2f} mm per mm of "
                  f"com_x, {cry[n] - cr0[n]:+.2f} per mm of com_y, {(crz[n] - cr0[n]) / 5:+.2f} per mm of com_z")
            if ex.of:
                sg += (f"; against {ex.of}: {((crz[n] - cr0[n]) - (crz[ex.of] - cr0[ex.of])) / 5:+.2f} mm per mm "
                       "of com_z")
        L.append(f"{n:15s} {s:5.1f}  {what}")
        L.append(f"{'':15s} {'':5s}  measures {ex.measures}; signal: {sg}")
    L.append(f"total {len(full) * SC.TICK_MS / 1000:.1f} s of poses if no sweep tips; {len(run) * SC.TICK_MS / 1000:.1f}"
             " s when each tips at the model's crossing; plus about 1.5 s to start and 0.6 s to stand up after")
    L.append(f"a tip: the tilt toward the lifted foot {SC.TIP_RISE_DEG} deg above the lifted hold's (its last "
             f"{SC.TIP_BASE_TICKS} ticks) and risen {SC.TIP_JUMP_DEG} deg within {SC.TIP_JUMP_TICKS} ticks; "
             f"{100 * SC.TIP_UP_FRACTION:.0f} % of the model's tip during the lifted hold: tipped as the foot came up")
    L.append("dropped (signal below the IMU's resolution for the parameter, or none):")
    for name, why in dropped().items():
        L.append(f"  {name}: {why}")
    return "\n".join(L)


def _roll_deg(mm: int) -> float:
    m = BodyModel()
    return math.degrees(math.atan(2.0 * mm / m.track_mm))


def _lag_signal(run: F.Run) -> float:
    """deg per ms of dead time at the stairs' fastest change."""
    model = BodyModel()
    rm = F.RunModel(run, model, F.model_values(model)["tip_ms"])
    s = rm.statics(F.body_of(F.model_values(model), model))
    a = F.servo(s, 50.0, 70.0)
    b = F.servo(s, 51.0, 70.0)
    return float(np.abs(a - b)[run.exp == SC.SCRIPT.index("stairs_pitch")].max())


def dropped() -> dict[str, str]:
    """The candidates the routine leaves out, and their signals."""
    model = BodyModel()
    out = {}
    # fore-aft shift of the body, all feet down: the legs' sag shows as a pitch from the load moving
    P = ST.Poses(model, [[-20] * 4, [20] * 4], [[SC.HOME_Z] * 4] * 2)
    r = ST.rest(P, ST.Body(sag=3.0))
    out["shift (body 20 mm forward / back, four feet)"] = (
        f"sag 3 gives {abs(r.pitch[1] - r.pitch[0]):.2f} deg between them: under {RESOLVE_DEG:.2f}; the tripods "
        "and the twist see the sag better")
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
    out["per-leg steps"] = ("one leg's step on four feet rocks the body onto a diagonal: the pitch and roll "
                            "stairs time the same servos with a clean signal")
    out["rolled sweeps of RF toward, LB away, RB"] = (
        "all four feet sweep together, and with the right side down the RB shoulder (its pulse clamp) cannot "
        "reach as far forward as those crossings: LF both ways, RF away and LB toward are what its reach allows")
    return out


# ---- the identifiability check ------------------------------------------------------------------------
SPREAD = {"imu_pitch": (-3, 3), "imu_roll": (-3, 3), "com_x": (-6, 6), "com_y": (-4, 4), "com_z": (-10, 10),
          "length": (-4, 12), "track": (-2, 7), "x_scale": (-0.02, 0.02),
          "sag": (0.5, 4.0), "twist": (-1.0, 1.0), "lag_ms": (30, 80), "tau_ms": (40, 110)}
OFFSET = {"com_x", "com_z", "length", "track", "x_scale"}  # spread around the model's value; the others: absolute


def random_truth(rng) -> dict:
    mv = F.model_values()
    v = {}
    for k, (lo, hi) in SPREAD.items():
        v[k] = rng.uniform(lo, hi) + (mv[k] if k in OFFSET else 0.0)
    return v


def model_tip_ms(v: dict) -> float:
    """The tip's delay in the fit's own model (no pendulum): the servo's
    response half way."""
    return v["lag_ms"] + v["tau_ms"] * math.log(2.0)


def check(trials: int = 20, seed: int = 1, backlash: float = 0.0, verbose: bool = False) -> dict:
    """Fit ``trials`` synthetic runs; per parameter the error's mean and
    rms, the mean +- the fit reported, and how often the truth was within
    2 +-."""
    rng = np.random.default_rng(seed)
    rows = [k for k in F.ROWS if k != "tip_ms"]
    errs: dict[str, list] = {k: [] for k in rows}
    sds: dict[str, list] = {k: [] for k in rows}
    limits = {k: 0 for k in rows}
    t0 = time.time()
    for i in range(trials):
        truth = random_truth(rng)
        truth["tip_ms"] = model_tip_ms(truth)  # no pendulum here
        run = synth_run(truth, noise=IMU_NOISE_DEG, backlash=backlash, seed=int(rng.integers(1 << 30)))
        res = F.Fitter([run]).run()
        for k in rows:
            if k == "x_scale" and res.x_scale_check:  # not fitted: what the splay alone says
                errs[k].append(res.x_scale_check["value"] - truth[k])
                sds[k].append(res.x_scale_check["sd"])
                continue
            if k not in res.names or res.at_limit.get(k):  # not measured, and said so: not counted as an error
                limits[k] += 1
                continue
            errs[k].append(res.values[k] - truth[k])
            sds[k].append(res.sd[k])
        if verbose:
            nc = sum(c["measured_x"] is not None for c in res.crossings)
            print(f"trial {i + 1}/{trials} ({time.time() - t0:.0f} s, {nc} crossings): " +
                        " ".join(f"{k} {res.values[k] - truth[k]:+.2f}" for k in rows if k != "x_scale"), file=sys.stderr)
    out = {}
    for k in rows:
        e = np.array(errs[k] or [math.nan])
        s = np.array(sds[k] or [math.nan])
        out[k] = {"bias": float(e.mean()), "rms": float(np.sqrt(np.mean(e ** 2))), "sd_reported": float(s.mean()),
                  "within_2sd": float(np.mean(np.abs(e) <= 2 * s)), "at_limit": limits[k]}
    out["_trials"] = trials
    out["_backlash"] = backlash
    out["_seconds"] = time.time() - t0
    return out


def format_check(c: dict) -> str:
    L = [f"{c['_trials']} synthetic runs (IMU noise {IMU_NOISE_DEG} deg, 1/16 deg steps"
         + (f", backlash {c['_backlash']} deg per hold" if c["_backlash"] else "") + f"; {c['_seconds']:.0f} s)",
         f"{'parameter':10s} {'unit':10s} {'bias':>7s} {'rms err':>8s} {'+- said':>8s} {'within 2+-':>10s}"]
    for k in F.ROWS:
        if k not in c:
            continue
        d = c[k]
        unit = F.WHAT[k][1]
        L.append(f"{k:10s} {unit:10s} {d['bias']:+7.3f} {d['rms']:8.3f} {d['sd_reported']:8.3f} "
                 f"{100 * d['within_2sd']:9.0f}%" + (f"  ({d['at_limit']} not measured)" if d.get("at_limit") else ""))
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
