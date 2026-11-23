"""kilo log wall (kilo.logs.wall, docs/wall-walk.md): how far Kilo went,
measured with the distance sensor against a wall ahead.

Synthetic recordings built byte for byte as the mind records (the blocks of
test_flight): he stands, the walk starts, the plan's FEET push the body, the
body goes a known share of that push a few ticks late, turns slowly, pitches
with the steps, and the sensor reads the wall from where the body is (with
noise, stray echoes and missed ones); then the quick stop and a sit. The
analysis must find the travel, the share, the heading and the speed it was
built with. Then the real recordings of 2026-11-22 if they are on this
laptop (~/kilo-logs; not a wall walk: the tool must say so)."""

from __future__ import annotations

import math
import random
from pathlib import Path

import pytest

from kilo import cli
from kilo.logs import flight, wall
from kilo.planner.model import BodyModel
from kilo.protocol import kilo_protocol as P
from test_flight import T0, blocks, frame, mark, run_mark

M = BodyModel()
HELD = int(P.TelemetryFlags.SERVOS_POWERED | P.TelemetryFlags.HEAD_HELD)


def t_of(k: int) -> int:
    return T0 + k * 20000 + 3000


def wall_walk(frac: float = 0.8, v: float = 58.0, seconds: float = 8.0, wall_mm: float = 900.0,
              turn_dps: float = 1.0, bias_dps: float = -0.6, noise: float = 4.0, jumps: float = 0.06,
              zeros: float = 0.01, lag: int = 5, seed: int = 3, other_after: float | None = None,
              look_at: float | None = None) -> tuple[list, dict]:
    """A forward walk at v toward a flat wall wall_mm from the body's origin. Returns (records, truth)."""
    rnd = random.Random(seed)
    recs: list = []
    period = 48                                    # ticks (0.96 s), one leg swings a quarter of it
    stride = v * 0.75 * period * 0.02              # each leg's stance pushes the body this far
    kb, k0 = 100, 130                              # the walk's run begins; its first step
    n = int(seconds / 0.02)
    k_stop, k_last = k0 + n, k0 + n + 50
    nx, nz = M.outline.nose_x_mm, M.outline.nose_z_mm

    def foot(leg: int, k: int) -> tuple[int, bool]:
        if k < k0 or k > k_stop:
            k = min(max(k, k0), k_stop)
        ph = ((k - k0) + leg * period // 4) % period / period
        if ph < 0.75:
            return round(stride / 2 - ph / 0.75 * stride), True
        return round(-stride / 2 + (ph - 0.75) / 0.25 * stride), False

    feet = {}
    for k in range(k0, k_last + 1):
        xs, sup = [], 0
        for leg in range(4):
            x, s = foot(leg, k)
            if k >= k_stop:
                s = True
            xs.append(x)
            sup |= (1 << leg) if s else 0
        feet[k] = P.Feet(epoch=4, tick=k & 0xFFFF, x_mm=xs, z_mm=[76] * 4, pitch=0, roll=0, support=sup)
    push = wall.commanded_push(feet)
    # the body: frac of the push, lag ticks late, along a heading that turns at turn_dps
    xw, prev, psi = 0.0, 0.0, 0.0
    walls, heads = {}, {}
    for k in range(0, k_last + 150):
        p = frac * push.get(min(max(k - lag, k0), k_last), 0.0)
        if k0 < k <= k_stop:
            psi += turn_dps * 0.02
        xw += (p - prev) * math.cos(math.radians(psi))
        prev = p
        walls[k], heads[k] = wall_mm - xw, psi
    truth = {"travel": walls[k0 - 1] - walls[k_last], "push": push[k_last] - push[k0], "heading": heads[k_stop],
             "speed": frac * v * math.cos(math.radians(turn_dps * seconds / 2))}

    recs.append(frame(P.WalkRecord.TX, t_of(2), P.SetPosture(epoch=4, posture=int(P.Posture.STAND))))
    recs.append(run_mark(t_of(kb), 1, P.RunKind.WALK, P.RunEvent.BEGIN))
    recs.append(mark(t_of(kb), f"walk id=1 format=2 t0_us_hi=0 t0_us_lo={t_of(kb)} tick_ms=20"))
    recs.append(mark(t_of(kb), f"setup pattern=walk dir=forward speed_asked={v} balance=1"))
    recs.append(frame(P.WalkRecord.TX, t_of(kb + 1), P.SetPosture(epoch=4, posture=int(P.Posture.STANCE))))
    for k in range(kb + 2, k_last + 1, 10):
        recs.append(frame(P.WalkRecord.TX, t_of(k), P.SetHead(epoch=4, mode=1, pan=0, tilt=0, tail=-32768,
                                                               max_speed_dps=120)))
    if look_at is not None:                        # the head turned away for a moment
        kl = k0 + int(look_at / 0.02)
        recs.append(frame(P.WalkRecord.TX, t_of(kl), P.SetHead(epoch=4, mode=1, pan=400, tilt=-300, tail=-32768,
                                                                max_speed_dps=120)))
        recs.append(frame(P.WalkRecord.TX, t_of(kl + 25), P.SetHead(epoch=4, mode=1, pan=0, tilt=0, tail=-32768,
                                                                     max_speed_dps=120)))
    recs.append(mark(t_of(k0 - 2), f"stream tick={k0} pattern=walk dir=forward speed={v} speed_set={v} "
                                   f"stride_mm={stride:.1f} period_ms={period * 20}.0"))
    for k, f in feet.items():
        recs.append(frame(P.WalkRecord.TX, t_of(k) - 100000, f))
    recs.append(mark(t_of(k_stop), f"stop tick={k_stop} quick=1"))
    recs.append(frame(P.WalkRecord.TX, t_of(k_last + 1), P.SetPosture(epoch=4, posture=int(P.Posture.STANCE))))
    recs.append(frame(P.WalkRecord.TX, t_of(k_last + 2), P.SetPosture(epoch=4, posture=int(P.Posture.SIT))))
    recs.append(mark(t_of(k_last + 2), "end ok=1 streamed=1 reason=stopped"))
    recs.append(run_mark(t_of(k_last + 2), 1, P.RunKind.WALK, P.RunEvent.END))
    for k in range(0, k_last + 150):
        sitting = k > k_last + 4
        moving = k0 <= k <= k_stop
        e = (-26.0 if sitting else 1.0 + (1.5 * math.sin(2 * math.pi * 2 * k / period) if moving else 0.0))
        psi = heads[k]
        d = walls[k]
        if other_after is not None and k > k0 + other_after / 0.02:
            d -= 250                               # something else came into the beam
        er = math.radians(e - M.imu_pitch_deg)
        r = (d / math.cos(math.radians(psi)) - nx * math.cos(er) - nz * math.sin(er)) / math.cos(er)
        r += rnd.gauss(0, noise)
        u = rnd.random()
        if u < jumps:
            r += 300
        elif u < jumps + zeros:
            r = 0
        flags = HELD if kb + 2 <= k <= k_last + 20 else int(P.TelemetryFlags.SERVOS_POWERED)
        if k % 4 == 0:
            flags |= int(P.TelemetryFlags.DIST_FRESH)
        rate_yaw = (turn_dps if k0 < k <= k_stop else 0.0) + bias_dps + rnd.gauss(0, 0.3)
        mode = P.Mode.GAIT if k0 <= k <= k_last else P.Mode.IDLE
        m = P.Telemetry(tick=k & 0xFFFF, pitch=round(e * 16), roll=0, rate_pitch=0, rate_roll=0,
                        rate_yaw=round(rate_yaw * 16), dist_mm=max(0, round(r)), batt_mv=7900, mode=int(mode),
                        flags=flags, trick_frame=0xFF, epoch=4)
        recs.append(frame(P.WalkRecord.RX, t_of(k), m, k))
    return recs, truth


def krec(tmp_path: Path, recs: list, name: str = "w.krec") -> Path:
    return flight.write_file(tmp_path / name, {"mind": "test"}, blocks(recs))


def analyse(tmp_path: Path, **kw) -> tuple[wall.WallWalk, dict]:
    recs, truth = wall_walk(**kw)
    ws = wall.analyse(flight.load(krec(tmp_path, recs)))
    assert len(ws) == 1
    return ws[0], truth


# ---- the pieces ------------------------------------------------------------------------------

def test_a_reading_corrected_for_pitch_place_and_heading():
    # level and square: the reading plus the sensor's place ahead of the origin
    d, pc, hc = wall.correct(500, M.imu_pitch_deg, 0, M)
    assert d == pytest.approx(500 + M.outline.nose_x_mm) and pc == pytest.approx(0) and hc == 0
    # nose down 3 degrees: the beam's horizontal part, the nose a little lower and back
    d, pc, hc = wall.correct(500, M.imu_pitch_deg + 3, 0, M)
    e = math.radians(3)
    assert d == pytest.approx((500 + M.outline.nose_x_mm) * math.cos(e) + M.outline.nose_z_mm * math.sin(e))
    assert -2 < pc < 0
    # turned 10 degrees: the beam reads the wall 1/cos further
    d, _, hc = wall.correct(500, M.imu_pitch_deg, 10, M)
    assert d == pytest.approx((500 + M.outline.nose_x_mm) * math.cos(math.radians(10)))
    assert hc < -9


def test_the_noise_of_one_reading_from_its_neighbours():
    rnd = random.Random(1)
    xs = [900 - 3.2 * i + rnd.gauss(0, 5) for i in range(400)]
    assert wall.local_noise(xs) == pytest.approx(5, rel=0.15)
    assert wall.local_noise([900 - 3.2 * i for i in range(50)]) == pytest.approx(0, abs=1e-9)


def test_the_push_is_the_support_feet_travel():
    feet = {k: P.Feet(tick=k, x_mm=[10 - k, 10 - k, 0, 0], support=0b0011) for k in range(10)}
    push = wall.commanded_push(feet)
    assert push[9] == pytest.approx(9)
    # a foot in the air does not push; a foot that just landed starts pushing on its next tick
    feet[5] = P.Feet(tick=5, x_mm=[5, 30, 0, 0], support=0b0001)
    push = wall.commanded_push(feet)
    assert push[9] == pytest.approx(9)


# ---- a whole walk ----------------------------------------------------------------------------

def test_a_walk_toward_a_wall_gives_the_travel_the_share_and_the_heading(tmp_path):
    w, truth = analyse(tmp_path)
    assert not w.quality, w.quality
    assert w.start_how == "still" and w.start_n >= 10
    assert w.end_how == "still" and w.end_n >= 6      # the quick stop's still second before the sit
    assert w.travel_mm == pytest.approx(truth["travel"], abs=max(5.0, w.travel_err_mm))
    assert w.commanded_mm == pytest.approx(truth["push"])
    assert w.ground_frac == pytest.approx(truth["travel"] / truth["push"], abs=0.02)
    assert w.fit_ground_frac == pytest.approx(0.8, abs=0.03)
    assert w.heading_deg == pytest.approx(truth["heading"], abs=0.5)
    assert w.gyro_bias_dps == pytest.approx(-0.6, abs=0.1)
    assert w.drift_mm > 0                                # turned left: went left
    assert w.dropped.get("jump", 0) >= 3 and w.dropped.get("no echo", 0) >= 1
    assert w.dropped.get("after (posture)", 0) >= 10   # the sit's readings are not the wall
    (s,) = [s for s in w.stretches if s.speed_mm_s is not None]
    assert s.speed_mm_s == pytest.approx(truth["speed"], rel=0.06)
    assert s.feet_mm_s == pytest.approx(58.0, rel=0.03)
    assert s.per_cycle_mm == pytest.approx(s.speed_mm_s * 0.96, rel=1e-3)
    assert 1 < w.travel_err_mm < 20 and "speed of sound" in w.err_terms
    assert w.model_ground_frac == 1.0


def test_no_slip_and_a_fast_walk(tmp_path):
    w, truth = analyse(tmp_path, frac=1.0, v=72.5, turn_dps=0.0, seed=8)
    assert not w.quality
    assert w.ground_frac == pytest.approx(1.0, abs=0.02) and abs(w.heading_deg) < 0.5
    assert w.travel_mm == pytest.approx(truth["travel"], abs=max(5.0, w.travel_err_mm))


def test_the_tape_measure_replaces_the_speed_of_sound(tmp_path):
    recs, truth = wall_walk(seed=5)
    path = krec(tmp_path, recs)
    (w,) = wall.analyse(flight.load(path), tape=(790.0, 790.0 - truth["travel"]))
    assert "speed of sound" not in w.err_terms
    assert w.tape["sensor_scale"] == pytest.approx(1.0, abs=0.02)
    assert w.tape["sensor_start_mm"] == pytest.approx(900 - M.outline.nose_x_mm, abs=5)


def test_something_else_in_the_beam_is_not_a_wall_measurement(tmp_path):
    w, _ = analyse(tmp_path, other_after=4.0)
    assert "do not follow one flat wall" in w.quality


def test_turning_too_far_is_said(tmp_path):
    w, _ = analyse(tmp_path, turn_dps=4.0, seconds=8.0)
    assert w.dropped.get("turned too far", 0) > 0 and "turned more than" in w.quality


def test_readings_with_the_head_turned_away_are_left_out(tmp_path):
    w, truth = analyse(tmp_path, look_at=3.0)
    assert w.dropped.get("head not straight", 0) >= 4
    assert w.travel_mm == pytest.approx(truth["travel"], abs=max(5.0, w.travel_err_mm))


def test_the_cli(tmp_path, capsys):
    recs, _ = wall_walk()
    path = krec(tmp_path, recs)
    assert cli.main(["log", "wall", str(path)]) == 0
    out = capsys.readouterr().out
    assert "travelled" in out and "of the feet's push" in out and "[model.travel] walk_ground_frac" in out
    assert cli.main(["log", "wall", str(path), "--json", "--run", "1"]) == 0
    import json
    (d,) = json.loads(capsys.readouterr().out)
    assert d["run"] == 1 and d["travel_mm"] > 0
    assert cli.main(["log", "wall", str(path), "--run", "7"]) == 1


# ---- the real recordings (not a wall walk) ---------------------------------------------------

REAL = Path.home() / "kilo-logs" / "rec-card-b16.krec"


@pytest.mark.skipif(not REAL.exists(), reason="the recordings of 2026-11-22 are not on this laptop")
def test_the_recorded_walk_of_boot_16_is_read_and_said_not_to_be_a_wall_walk():
    (w,) = wall.analyse(flight.load(REAL))
    # he stood still with his head straight before it (STAND, then the walk's own hold) ...
    assert w.start_how == "still" and w.start_n >= 10
    # ... but steered, turned past 15 degrees and read things other than one wall
    assert w.quality and w.dropped.get("turned too far", 0) > 100
    assert 3 < w.noise_mm < 12
