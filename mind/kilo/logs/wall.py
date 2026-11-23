"""``kilo log wall FILE.krec``: how far Kilo really went, measured against a
wall with the distance sensor in his eyes (docs/wall-walk.md).

A forward walk toward a flat wall is recorded like any other walk (``kilo
runs --fetch``): every new reading of the ultrasonic sensor (TELEMETRY
``dist_mm`` with the DIST_FRESH flag, every 80 ms, stamped with the body's
tick), the head's hold (SET_HEAD: the mind holds it straight ahead while it
walks), the body's pitch, the gyro's yaw rate, the walk's gait and speed
(the ``stream`` / ``timing`` marks) and every foot of the plan (POSE, one a
tick: the step clock). From that this reports, per walk:

- the distance travelled: the wall's distance standing still before the
  walk less the one standing still after it (or, where he did not stand
  still long enough to measure, the straight-line fit's at that moment);
- what the plan commanded: how far the feet on the floor pushed the body
  (POSE: the support feet's travel under the hips, tick by tick);
- the share of that he really went (``ground_frac``; the slip is 1 less
  it), from the two still ends and, as a check, fitted over every reading;
- per stretch at one speed: the measured speed against the commanded one,
  and the distance per step cycle;
- the heading (the gyro's yaw rate less its resting offset; + = the nose
  turning left) and the sideways drift it implies;
- an error estimate, the terms named.

The readings are corrected with the body model: the beam's tilt (the body's
pitch less what the IMU reads standing level, ``[model.correction]
imu_pitch_deg``; the head is held level, so its tilt is 0, and readings
with the head not held straight are not used), the sensor's place on the
body (the model's nose, ``[model.outline]``), which moves with the pitch,
and the heading (a reading along a beam turned by psi from the wall's
normal is the distance / cos psi). Readings past 2.5 m, no-echo readings,
the floor (``[model.sensor]``, as the distance guard counts it) and
single-reading jumps (more than 40 mm off the median of their five
neighbours) are left out.

What it cannot know: whether he started square to the wall (taken as up to
3 degrees off), the speed of sound the sensor assumes (taken as right to
1.5 %, about 10 degrees C of air temperature; ``--tape START END``, the
owner's tape measure from the wall to the front of the eyes before and
after, replaces it), and whether the wall echoes cleanly.
"""

from __future__ import annotations

import json
import math
import statistics
from dataclasses import asdict, dataclass, field
from typing import Any

from kilo.logs import flight, proto2, walkrec
from kilo.planner.model import BodyModel
from kilo.protocol import kilo_protocol as P

A16 = 16.0
TICK_S = 0.02                # the body's tick (TELEMETRY.tick, POSE.tick)
MAX_RANGE_MM = 2500          # the distance guard's: farther is not a wall in a room
JUMP_MM = 40.0               # a reading this far off its neighbours' median is a stray echo
HEAD_SETTLE_S = 0.3          # the head reaches straight ahead this long after the hold (120 deg/s, a 28 deg look)
STILL_RATE_DPS = 8.0         # pitch and roll rates under this: the body is still (the gyro's offset, walkstudy.py)
STILL_WINDOW_S = 1.5         # still readings this long before the walk's first step are the start
END_WINDOW_S = 1.5           # ...and up to this long after its last one (until the posture changes)
STEADY_SKIP_S = 0.5          # a stretch at one speed is measured from this long after it starts
MAX_HEADING_DEG = 15.0       # turned further than this from the start, a flat wall's echo is not trusted
START_SQUARE_DEG = 3.0       # how far off square to the wall he may have started (the procedure's lining-up)
SOUND_SCALE = 0.015          # the speed of sound the sensor assumes: right to this share (about 10 deg C)
BIAS_FLOOR_DPS = 0.05        # the gyro's offset known no better than this (it wanders)
MIN_STILL = 3                # readings to call a still end measured


@dataclass
class Reading:
    t: float                 # s, the body's clock (tick * 20 ms) from the walk's first commanded step
    tick: int                # the body's tick (unwrapped)
    raw_mm: int
    pitch_deg: float         # TELEMETRY pitch, nose down +
    heading_deg: float       # since the first step, + = left
    x_cmd_mm: float          # how far the plan had pushed the body by then (servo lag included)
    wall_mm: float = 0.0     # the body origin's distance to the wall, corrected
    use: bool = True
    why: str = ""            # why not used


@dataclass
class Stretch:
    pattern: str
    speed_set: float         # mm/s the walk was set to (backward: -)
    steer: float
    period_ms: float
    t_from: float
    t_to: float
    n: int = 0
    speed_mm_s: float | None = None      # measured against the wall
    speed_se: float | None = None
    feet_mm_s: float | None = None       # what the plan's feet pushed in the same time
    per_cycle_mm: float | None = None    # measured, per step cycle
    feet_per_cycle_mm: float | None = None


@dataclass
class WallWalk:
    run: int
    pattern: str
    speed_set: float
    steered: bool
    seconds: float                        # first step to last
    cycles: float
    readings: int = 0
    used: int = 0
    dropped: dict[str, int] = field(default_factory=dict)
    noise_mm: float | None = None         # one reading's noise (robust, from neighbouring readings)
    fit_rms_mm: float | None = None       # the readings about the one-wall fit
    quality: str = ""                     # "" = a wall measurement; else why it is not one
    start_mm: float | None = None         # the wall, from the body origin, before
    start_how: str = ""
    start_n: int = 0
    end_mm: float | None = None
    end_how: str = ""
    end_n: int = 0
    travel_mm: float | None = None
    travel_err_mm: float | None = None
    err_terms: dict[str, float] = field(default_factory=dict)
    commanded_mm: float = 0.0             # the support feet's push, first step to last
    commanded_speed_mm: float = 0.0       # the set speed x the time walked (the stretches' sum)
    ground_frac: float | None = None      # travel / commanded (1 = no slip)
    ground_frac_err: float | None = None
    slip: float | None = None
    fit_ground_frac: float | None = None  # the same, fitted over every reading
    fit_ground_frac_se: float | None = None
    model_ground_frac: float | None = None
    heading_deg: float | None = None      # at the last step, + = left
    heading_err_deg: float | None = None
    gyro_bias_dps: float | None = None
    drift_mm: float | None = None         # sideways at the end, + = left
    pitch_corr_mm: float = 0.0            # the largest pitch/place correction made to one reading
    heading_corr_mm: float = 0.0          # ...and heading correction
    stretches: list[Stretch] = field(default_factory=list)
    tape: dict[str, float] | None = None
    notes: list[str] = field(default_factory=list)


# ---- the pieces ------------------------------------------------------------------------------

def _median_off(xs: list[float]) -> tuple[float, float]:
    m = statistics.median(xs)
    mad = statistics.median([abs(x - m) for x in xs]) if len(xs) > 1 else 0.0
    return m, 1.4826 * mad


def _linfit(xs: list[float], ys: list[float]) -> tuple[float, float, float, float] | None:
    """y = a + b x by least squares: (a, b, se of b, rms residual)."""
    n = len(xs)
    if n < 3:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return None
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    a = my - b * mx
    res = [y - a - b * x for x, y in zip(xs, ys)]
    s2 = sum(r * r for r in res) / max(1, n - 2)
    return a, b, math.sqrt(s2 / sxx), math.sqrt(s2)


def _robust_fit(xs: list[float], ys: list[float]) -> tuple[float, float, float, float] | None:
    f = _linfit(xs, ys)
    for _ in range(2):
        if f is None:
            return None
        a, b, _, s = f
        keep = [(x, y) for x, y in zip(xs, ys) if abs(y - a - b * x) <= max(3 * s, 5.0)]
        if len(keep) == len(xs):
            break
        xs, ys = [k[0] for k in keep], [k[1] for k in keep]
        f = _linfit(xs, ys)
    return f


def local_noise(xs: list[float]) -> float:
    """One reading's noise from its neighbours: the robust spread of the second differences (a steady
    approach cancels in them; white noise gives sqrt(6) times its SD)."""
    d2 = [xs[i - 1] - 2 * xs[i] + xs[i + 1] for i in range(1, len(xs) - 1)]
    if len(d2) < 3:
        return 0.0
    return 1.4826 * statistics.median([abs(d) for d in d2]) / math.sqrt(6)


def _walk_feet(msg: Any) -> bool:
    """A walk's tick of the plan: a FEET (protocol 2 recordings) or a POSE with four feet."""
    f = proto2.feet_of(msg)
    return f is not None and None not in f[0]


def commanded_push(feet: dict[int, Any]) -> dict[int, float]:
    """tick -> how far the plan had pushed the body forward by then (mm): each tick, the mean
    backward travel under the hips of the feet that are on the floor at both ends of it."""
    out: dict[int, float] = {}
    x = 0.0
    prev = None
    for k in sorted(feet):
        m = feet[k]
        if prev is not None and k == prev[0] + 1:
            pm = prev[1]
            legs = [i for i in range(4) if (m.support >> i) & 1 and (pm.support >> i) & 1]
            if legs:
                px, mx = proto2.feet_of(pm)[0], proto2.feet_of(m)[0]
                x += sum(px[i] - mx[i] for i in legs) / len(legs)
        out[k] = x
        prev = (k, m)
    return out


def correct(raw_mm: float, pitch_deg: float, heading_deg: float, model: BodyModel) -> tuple[float, float, float]:
    """A reading as the body origin's distance to a flat wall ahead: (distance, the pitch/place
    correction in it, the heading correction in it), mm. The beam tilts with the body (pitch
    less the IMU's level reading; the head is held level), its sensor sits at the model's nose."""
    e = math.radians(pitch_deg - model.imu_pitch_deg)
    nx, nz = model.outline.nose_x_mm, model.outline.nose_z_mm
    along = raw_mm * math.cos(e) + nx * math.cos(e) + nz * math.sin(e)
    psi = math.radians(heading_deg)
    d = along * math.cos(psi)
    return d, along - (raw_mm + nx), d - along


def _floor(raw_mm: float, pitch_deg: float, model: BodyModel) -> bool:
    """The distance guard's floor rule: tilted down past the floor's angle, a reading from 3/4 of
    the floor's distance on is the floor."""
    tilt = pitch_deg - model.imu_pitch_deg
    if tilt <= model.ultrasonic_floor_tilt_deg:
        return False
    return raw_mm >= 0.75 * model.ultrasonic_height_mm / math.sin(math.radians(tilt))


# ---- one walk --------------------------------------------------------------------------------

def analyse_walk(w: walkrec.Walk, run_id: int, model: BodyModel | None = None,
                 tape: tuple[float, float] | None = None) -> WallWalk | None:
    """One recorded walk (flight.walk_of). None: not a forward walk with planned steps."""
    m = model or BodyModel()
    setup = w.mark("setup")
    stream = w.mark("stream")
    if stream is None or (stream.fields.get("dir") or (setup.fields.get("dir") if setup else "")) != "forward":
        return None
    # the body's clock: telemetry ticks unwrapped, and mind time per tick
    tel: list[tuple[int, int, Any]] = []
    last = None
    for f in w.frames:
        if f.direction == "rx" and isinstance(f.msg, P.Telemetry):
            last = walkrec._unwrap(f.msg.tick, last if last is not None else f.msg.tick)
            tel.append((f.t_us, last, f.msg))
    if len(tel) < 10:
        return None
    ref = tel[0][1]
    feet: dict[int, Any] = {}
    for f in w.frames:
        if f.direction == "tx" and _walk_feet(f.msg):
            feet[walkrec._unwrap(f.msg.tick, ref)] = f.msg   # a replanned tick: the last sent wins
    if len(feet) < 5:
        return None
    push = commanded_push(feet)
    ks = sorted(push)
    moving = [k for k in ks if abs(push[k] - push.get(k - 1, push[k])) > 0.01]
    if not moving:
        return None
    k0, k1 = moving[0] - 1, moving[-1]
    lag_ticks = (m.servo_lag_ms + m.servo_tau_ms) / 20.0

    def x_at(tick: float) -> float:
        """the push the body had at tick (it follows the plan servo lag + tau late)."""
        k = math.floor(tick - lag_ticks)
        if k <= ks[0]:
            return push[ks[0]]
        if k >= ks[-1]:
            return push[ks[-1]]
        while k not in push:
            k -= 1
        return push[k]

    # the walk's stretches at one setting
    marks = sorted([mk for mk in w.marks if mk.word in ("stream", "timing", "stop")],
                   key=lambda mk: int(mk.fields.get("tick", 0)))
    stretches: list[Stretch] = []
    for i, mk in enumerate(marks):
        if mk.word == "stop":
            break
        kt = walkrec._unwrap(int(mk.fields["tick"]), ref)
        nxt = marks[i + 1] if i + 1 < len(marks) else None
        ke = walkrec._unwrap(int(nxt.fields["tick"]), ref) if nxt else k1
        sign = -1.0 if mk.fields.get("dir") == "back" else 1.0
        stretches.append(Stretch(mk.fields.get("pattern", "?"),
                                 sign * float(mk.fields.get("speed_set", mk.fields.get("speed", 0))),
                                 float(mk.fields.get("steer", 0) or 0), float(mk.fields.get("period_ms", 0) or 0),
                                 (kt - k0) * TICK_S, (ke - k0) * TICK_S))
    pattern = stretches[0].pattern if stretches else stream.fields.get("pattern", "?")
    ww = WallWalk(run=run_id, pattern=pattern, speed_set=stretches[0].speed_set if stretches else 0.0,
                  steered=any(s.steer for s in stretches) or any(mk.word == "change" and "steer" in mk.fields
                                                                 for mk in w.marks),
                  seconds=(k1 - k0) * TICK_S,
                  cycles=sum((s.t_to - s.t_from) * 1000 / s.period_ms for s in stretches if s.period_ms > 0),
                  commanded_mm=push[k1] - push[k0],
                  commanded_speed_mm=sum(s.speed_set * (s.t_to - s.t_from) for s in stretches))
    ww.model_ground_frac = m.ground_frac_of(pattern)

    # the gyro's resting offset: the still telemetry before the first step
    still_rates = [msg.rate_yaw / A16 for t, k, msg in tel if k < k0
                   and abs(msg.rate_pitch) < STILL_RATE_DPS * A16 and abs(msg.rate_roll) < STILL_RATE_DPS * A16]
    heading: dict[int, float] = {}
    bias_err = None
    if len(still_rates) >= 20:
        bias, sd = _median_off(still_rates)
        bias_err = max(BIAS_FLOOR_DPS, 1.2533 * sd / math.sqrt(len(still_rates)))
        ww.gyro_bias_dps = round(bias, 3)
        h = 0.0
        prev_k = None
        for t, k, msg in tel:
            if k > k0 and prev_k is not None:
                h += (msg.rate_yaw / A16 - bias) * (k - prev_k) * TICK_S
            heading[k] = h if k > k0 else 0.0
            prev_k = k
    else:
        ww.notes.append("the gyro's resting offset is not known (under 0.4 s still before the walk): "
                        "no heading, no heading correction")

    # the head: straight from when. Held straight (SET_HEAD mode 1, pan 0, tilt 0): HEAD_SETTLE_S after
    # the first such hold. Not held: it follows the posture (STAND's and STANCE's head is straight),
    # HEAD_HOLD_MS after the last other hold (the body's own release) or at once after a release.
    settle = int(HEAD_SETTLE_S * 1e6)
    head_from: list[tuple[int, int]] = []   # (t_us of the change, straight from t_us)
    prev_key = None
    for f in w.frames:
        if f.direction == "tx" and isinstance(f.msg, P.SetHead):
            h = f.msg
            key = (h.mode, h.pan, h.tilt)
            if key == prev_key:
                continue
            prev_key = key
            if h.mode == 1 and h.pan == 0 and h.tilt == 0:
                head_from.append((f.t_us, f.t_us + settle))
            elif h.mode == 0:
                head_from.append((f.t_us, f.t_us + settle))
            else:                          # held elsewhere: refreshed while it looks; straight once let go
                head_from.append((f.t_us, -1))   # until the next change (below)
    # a look held elsewhere is straight HEAD_HOLD_MS after its last refresh, if nothing came before
    heads = [(f.t_us, (f.msg.mode, f.msg.pan, f.msg.tilt)) for f in w.frames
             if f.direction == "tx" and isinstance(f.msg, P.SetHead)]
    for i, (tc, ok) in enumerate(head_from):
        if ok == -1:
            nxt = head_from[i + 1][0] if i + 1 < len(head_from) else None
            refs = [t for t, _ in heads if t >= tc and (nxt is None or t < nxt)]
            head_from[i] = (tc, max(refs) + P.HEAD_HOLD_MS * 1000 + settle)

    def head_straight(t_us: int, held: bool) -> bool:
        ok_from = None
        for tc, ok in head_from:
            if tc > t_us:
                break
            ok_from = ok
        if ok_from is None:
            return not held                # no command in the data: straight if the posture has it
        return t_us >= ok_from

    # the posture before the walk: STAND reached (its 0.5 s move and a settle); STANCE as the walk
    # starts is fine (STAND's feet stand 2.5 mm ahead of and behind the hips alike: the body does
    # not move forward going to STANCE, only 6 mm down)
    t_first_feet = min(f.t_us for f in w.frames if f.direction == "tx" and _walk_feet(f.msg))
    posture_ok_from = None
    last_p = None
    for f in w.frames:
        if f.t_us >= t_first_feet:
            break
        if f.direction == "tx" and isinstance(f.msg, P.SetPosture) and f.msg.posture != last_p:
            last_p = f.msg.posture
            if last_p == P.Posture.STAND:
                posture_ok_from = f.t_us + 1_000_000
            elif last_p != P.Posture.STANCE:
                posture_ok_from = None

    # the posture after the walk: readings end when another posture than STANCE / STAND starts
    t_last_feet = max(f.t_us for f in w.frames if f.direction == "tx" and _walk_feet(f.msg))
    t_cut = None
    for f in w.frames:
        if f.direction == "tx" and isinstance(f.msg, P.SetPosture) and f.t_us > t_last_feet \
                and f.msg.posture not in (P.Posture.STANCE, P.Posture.STAND):
            t_cut = f.t_us
            break

    # the readings
    rs: list[Reading] = []
    for t, k, msg in tel:
        if not msg.flags & P.TelemetryFlags.DIST_FRESH:
            continue
        r = Reading(t=(k - k0) * TICK_S, tick=k, raw_mm=msg.dist_mm, pitch_deg=msg.pitch / A16,
                    heading_deg=heading.get(k, 0.0), x_cmd_mm=x_at(k) - push[k0])
        if t_cut is not None and t >= t_cut - int(m.servo_lag_ms * 1000):
            r.use, r.why = False, "after (posture)"
        elif r.t < -STILL_WINDOW_S or r.t > ww.seconds + END_WINDOW_S:
            r.use, r.why = False, "outside"
        elif msg.dist_mm == 0:
            r.use, r.why = False, "no echo"
        elif msg.dist_mm > MAX_RANGE_MM:
            r.use, r.why = False, "too far"
        elif not head_straight(t, bool(msg.flags & P.TelemetryFlags.HEAD_HELD)):
            r.use, r.why = False, "head not straight"
        elif k < k0 and (posture_ok_from is None or t < posture_ok_from or msg.mode != P.Mode.IDLE
                         or abs(msg.rate_pitch) >= STILL_RATE_DPS * A16 or abs(msg.rate_roll) >= STILL_RATE_DPS * A16):
            r.use, r.why = False, "not standing still"
        elif _floor(msg.dist_mm, r.pitch_deg, m):
            r.use, r.why = False, "floor"
        elif abs(r.heading_deg) > MAX_HEADING_DEG:
            r.use, r.why = False, "turned too far"
        if r.why == "outside":
            continue
        rs.append(r)
        if r.use:
            r.wall_mm, pc, hc = correct(r.raw_mm, r.pitch_deg, r.heading_deg, m)
            ww.pitch_corr_mm = max(ww.pitch_corr_mm, abs(pc))
            ww.heading_corr_mm = max(ww.heading_corr_mm, abs(hc))
    good = [r for r in rs if r.use]
    for i, r in enumerate(good):           # single-reading jumps
        nb = [g.wall_mm for g in good[max(0, i - 2):i + 3]]
        if len(nb) >= 3 and abs(r.wall_mm - statistics.median(nb)) > JUMP_MM:
            r.use, r.why = False, "jump"
    ww.readings = len(rs)
    for r in rs:
        if not r.use:
            ww.dropped[r.why] = ww.dropped.get(r.why, 0) + 1
    good = [r for r in rs if r.use]
    ww.used = len(good)
    if len(good) < 6:
        ww.notes.append("too few readings to measure")
        return ww

    # the fit: wall = a - k * push, over every reading
    fit = _robust_fit([r.x_cmd_mm for r in good], [r.wall_mm for r in good])
    ww.noise_mm = round(local_noise([r.wall_mm for r in good]), 1)
    sigma = max(ww.noise_mm, 1.0)
    if fit is not None:
        a, b, se, s = fit
        ww.fit_ground_frac, ww.fit_ground_frac_se = round(-b, 4), round(se, 4)
        ww.fit_rms_mm = round(s, 1)
        if s > 2.5 * sigma + 10:
            ww.quality = (f"the readings do not follow one flat wall ahead (they stray {s:.0f} mm from it, one "
                          f"reading's noise is {sigma:.0f} mm): something else was in the beam, or he turned")

    still = [r for r in good if r.t <= 0]
    after = [r for r in good if r.t >= ww.seconds + lag_ticks * TICK_S + 0.1]
    err: dict[str, float] = {}
    for side, pts in (("start", still), ("end", after)):
        if len(pts) >= MIN_STILL:
            v, _ = _median_off([p.wall_mm for p in pts])
            how, e = "still", 1.2533 * sigma / math.sqrt(len(pts))
        elif fit is not None:
            x = 0.0 if side == "start" else ww.commanded_mm
            v = fit[0] + fit[1] * x
            n = len(good)
            how, e = "fit", math.hypot(sigma / math.sqrt(n), fit[2] * ww.commanded_mm / 2)
        else:
            continue
        setattr(ww, f"{side}_mm", round(v, 1))
        setattr(ww, f"{side}_how", how)
        setattr(ww, f"{side}_n", len(pts))
        err[f"{side} reading"] = e
    if ww.start_mm is None or ww.end_mm is None:
        ww.notes.append("no wall distance at one end")
        return ww
    ww.travel_mm = round(ww.start_mm - ww.end_mm, 1)
    err["speed of sound"] = SOUND_SCALE * abs(ww.travel_mm)
    err["square to the wall at the start"] = ww.start_mm * (1 - math.cos(math.radians(START_SQUARE_DEG)))
    if heading:
        hk = heading.get(max(k for k in heading if k <= k1), 0.0)
        ww.heading_deg = round(hk, 1)
        ww.heading_err_deg = round(bias_err * ww.seconds, 2)
        err["heading"] = ww.end_mm * abs(math.sin(math.radians(hk))) * math.radians(ww.heading_err_deg)
        # sideways: the body went along its heading at the pace the readings say
        frac = ww.travel_mm / ww.commanded_mm if ww.commanded_mm else 1.0
        y, prev = 0.0, push[k0]
        for k in range(k0 + 1, k1 + 1):
            if k in push:
                y += (push[k] - prev) * frac * math.sin(math.radians(heading.get(k, 0.0) if k in heading else hk))
                prev = push[k]
        ww.drift_mm = round(y, 1)
    else:
        err["heading (unknown)"] = abs(ww.travel_mm) * 0.02
    err["pitch and sensor place"] = ww.pitch_corr_mm / 2
    if tape is not None:
        t0, t1 = tape
        sensor_start = ww.start_mm - m.outline.nose_x_mm
        sensor_end = ww.end_mm - m.outline.nose_x_mm
        scale = (t0 - t1) / ww.travel_mm if ww.travel_mm else None
        ww.tape = {"start_mm": t0, "end_mm": t1, "travel_mm": t0 - t1,
                   "sensor_start_mm": round(sensor_start, 1), "sensor_end_mm": round(sensor_end, 1),
                   "sensor_scale": round(scale, 4) if scale else None}
        err.pop("speed of sound", None)
    ww.err_terms = {k: round(v, 1) for k, v in err.items()}
    ww.travel_err_mm = round(math.sqrt(sum(v * v for v in err.values())), 1)
    if ww.commanded_mm > 0:
        ww.ground_frac = round(ww.travel_mm / ww.commanded_mm, 3)
        ww.ground_frac_err = round(ww.travel_err_mm / ww.commanded_mm, 3)
        ww.slip = round(1 - ww.ground_frac, 3)

    # the stretches at one speed
    for s in stretches:
        pts = [r for r in good if s.t_from + STEADY_SKIP_S <= r.t <= s.t_to]
        s.n = len(pts)
        if s.n >= 8 and s.t_to - s.t_from - STEADY_SKIP_S >= 1.5:
            f = _robust_fit([p.t for p in pts], [p.wall_mm for p in pts])
            if f is not None:
                s.speed_mm_s, s.speed_se = round(-f[1], 1), round(f[2], 1)
            fx = _linfit([p.t for p in pts], [p.x_cmd_mm for p in pts])
            if fx is not None:
                s.feet_mm_s = round(fx[1], 1)
            if s.period_ms and s.speed_mm_s is not None:
                s.per_cycle_mm = round(s.speed_mm_s * s.period_ms / 1000, 1)
            if s.period_ms and s.feet_mm_s is not None:
                s.feet_per_cycle_mm = round(s.feet_mm_s * s.period_ms / 1000, 1)
    ww.stretches = stretches
    if ww.steered:
        ww.notes.append("steered: the commanded push is the four feet's mean; the heading correction assumes "
                        "a flat wall square to the start")
    if ww.dropped.get("turned too far"):
        ww.quality = ww.quality or f"turned more than {MAX_HEADING_DEG:.0f} degrees from the start"
        ww.notes.append(f"turned more than {MAX_HEADING_DEG:.0f} degrees: those readings left out")
    if any(s.speed_set <= 0 for s in stretches):
        ww.quality = ww.quality or "walked backward or stood for part of it"
    return ww


def analyse(s: flight.Stream, run: int | None = None, model: BodyModel | None = None,
            tape: tuple[float, float] | None = None) -> list[WallWalk]:
    out = []
    for r in s.runs:
        if r.kind != "walk" or (run is not None and r.id != run):
            continue
        if r.begin_us is None or r.end_us is None:
            continue                       # not whole in this file (kilo runs --fetch has each whole)
        ww = analyse_walk(flight.walk_of(s, r), r.id, model, tape)
        if ww is not None:
            out.append(ww)
    return out


# ---- printing --------------------------------------------------------------------------------

def _f(v: float | None, fmt: str = ".0f", unit: str = "") -> str:
    return "-" if v is None else f"{v:{fmt}}{unit}"


def format_walk(w: WallWalk) -> str:
    lines = [f"walk {w.run}: {w.pattern} at {w.speed_set:g} mm/s{' (steered)' if w.steered else ''}, "
             f"{w.seconds:.1f} s, {w.cycles:.1f} step cycles; readings {w.used} used of {w.readings}"
             + (f" (left out: {', '.join(f'{k} {v}' for k, v in sorted(w.dropped.items()))})" if w.dropped else "")]
    if w.quality:
        lines.append(f"  NOT A WALL MEASUREMENT: {w.quality}")
    if w.noise_mm is not None:
        lines.append(f"  one reading's noise {w.noise_mm:.1f} mm; the readings about the one-wall fit "
                     f"{_f(w.fit_rms_mm, '.1f')} mm")
    lines.append(f"  wall before {_f(w.start_mm)} mm ({w.start_how}, {w.start_n} readings), after {_f(w.end_mm)} mm "
                 f"({w.end_how}, {w.end_n} readings)  [from the body's origin, corrected]")
    if w.travel_mm is not None:
        lines.append(f"  travelled  {w.travel_mm:.0f} +- {_f(w.travel_err_mm)} mm   "
                     f"(" + ", ".join(f"{k} {v:g}" for k, v in w.err_terms.items()) + ")")
    lines.append(f"  commanded  {w.commanded_mm:.0f} mm by the feet on the floor; {w.commanded_speed_mm:.0f} mm by "
                 "the set speed x the time")
    if w.ground_frac is not None:
        lines.append(f"  went {w.ground_frac:.3f} +- {w.ground_frac_err:.3f} of the feet's push (slip {w.slip:+.3f})"
                     f"; fitted over every reading {_f(w.fit_ground_frac, '.3f')} +- {_f(w.fit_ground_frac_se, '.3f')}"
                     + (f"; the model says {w.model_ground_frac:g}" if w.model_ground_frac is not None else ""))
    if w.ground_frac is not None and not w.quality and w.model_ground_frac is not None:
        lines.append(f"  for the model: [model.travel] {w.pattern}_ground_frac = {w.ground_frac:.2f} (protocol/spec.toml; "
                     "the mean of a few such walks, and say where it comes from in [model.about])")
    if w.heading_deg is not None:
        lines.append(f"  heading at the last step {w.heading_deg:+.1f} +- {w.heading_err_deg:.1f} deg (+ = left; gyro "
                     f"offset {w.gyro_bias_dps:+.2f} deg/s), sideways drift about {_f(w.drift_mm, '+.0f')} mm")
    lines.append(f"  corrections at most: pitch and sensor place {w.pitch_corr_mm:.1f} mm, heading "
                 f"{w.heading_corr_mm:.1f} mm")
    for s in [s for s in w.stretches if s.speed_mm_s is not None]:
        lines.append(f"  stretch {s.t_from:5.1f}..{s.t_to:5.1f} s {s.pattern} set {s.speed_set:g} mm/s"
                     f"{f' steer {s.steer:+g}' if s.steer else ''}: measured {_f(s.speed_mm_s, '.1f')} +- "
                     f"{_f(s.speed_se, '.1f')} mm/s, feet {_f(s.feet_mm_s, '.1f')} mm/s; per cycle "
                     f"({s.period_ms:g} ms) {_f(s.per_cycle_mm, '.1f')} mm measured, "
                     f"{_f(s.feet_per_cycle_mm, '.1f')} mm feet ({s.n} readings)")
    short = sum(1 for s in w.stretches if s.speed_mm_s is None)
    if short:
        lines.append(f"  ({short} stretch{'es' if short > 1 else ''} at one setting too short to measure on its own)")
    if w.tape:
        t = w.tape
        lines.append(f"  tape: {t['start_mm']:g} -> {t['end_mm']:g} mm, travel {t['travel_mm']:g} mm; the sensor "
                     f"read {t['sensor_start_mm']:g} -> {t['sensor_end_mm']:g} mm (wall to the eyes); "
                     f"its scale {_f(t['sensor_scale'], '.4f')}")
    for n in w.notes:
        lines.append(f"  note: {n}")
    return "\n".join(lines)


def main(args: Any) -> int:
    import sys

    try:
        s = flight.load(args.file)
    except (OSError, ValueError) as e:
        print(f"cannot read {args.file}: {e}", file=sys.stderr)
        return 1
    ws = analyse(s, args.run, tape=tuple(args.tape) if args.tape else None)
    if not ws:
        print(f"{args.file}: no forward walk with planned steps in it", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps([asdict(w) for w in ws], indent=1))
    else:
        print("\n".join(format_walk(w) for w in ws))
    return 0
