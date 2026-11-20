"""The alive layer's sitting moves (kernel/mind/behaviour/alive.cpp kSitMoves) against Kilo's joints
and the sit's statics.

The table is read from the C++ source (one row a line between SIT_MOVES_BEGIN and SIT_MOVES_END), so
this checks what the mind plays. Each move at its smallest and largest amplitude, both sides of a
sided one, every keyframe and the way back to SIT:

- every joint it changes inside what that joint reaches with Kilo's recorded calibration (the body's
  joint limits and the servo pulse range, kilo.tricks.stance.reach: what `kilo cal show` computes;
  joints 8, 9, 11, 12 and 14 are short of travel there);
- along the moves (straight joint-space lines, as the body's keyframe player) the body rests with
  the centre of mass inside its support (kilo.tricks.stance.follow): on Kilo with the right hind
  shoulder recalibrated (rear_up.DESIGN, which the rear-up needed) at least MARGIN_MM, rolling at
  most ROLL_MAX_DEG, no topple dropping more than DROP_MAX_MM; on Kilo as recorded (its right hind
  shoulder short, SIT on three feet already) no worse than SIT itself rocks.

The model is rigid and quasi-static (no inertia): a real leg gives a little and the moves are slow
(0.25 s at the quickest, the tail), so it is a static check, not a dynamic one.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from kilo.tricks import rear_up as RU
from kilo.tricks import stance as T

SRC = Path(__file__).resolve().parents[2] / "kernel" / "mind" / "behaviour" / "alive.cpp"
NAMES = ("head_pan", "head_tilt", "tail", "lf_shoulder", "rf_shoulder", "rb_shoulder", "lb_shoulder",
         "lf_knee", "rf_knee", "rb_knee", "lb_knee")
IDS = (0, 1, 2, 8, 9, 10, 11, 12, 13, 14, 15)
MIRROR = {"lf_shoulder": "rf_shoulder", "rf_shoulder": "lf_shoulder", "rb_shoulder": "lb_shoulder",
          "lb_shoulder": "rb_shoulder", "lf_knee": "rf_knee", "rf_knee": "lf_knee", "rb_knee": "lb_knee",
          "lb_knee": "rb_knee"}
MARGIN_MM = 2.0
ROLL_MAX_DEG = 8.0
DROP_MAX_MM = 6.0


def _source() -> str:
    return SRC.read_text()


def sit() -> dict:
    m = re.search(r"const int8_t kSit\[KILO_JOINT_COUNT\] = \{([^}]*)\};", _source())
    return dict(zip(NAMES, (float(v) for v in m.group(1).split(","))))


def moves() -> list[dict]:
    src = _source()
    table = src[src.index("// SIT_MOVES_BEGIN"):src.index("// SIT_MOVES_END")]
    out = []
    for line in table.splitlines():
        m = re.match(r'\s*\{AliveMove::(\w+), "(\w+)", ([\d.]+), ([\d.]+), (true|false), (\d+), \{(.*)\}\},\s*$', line)
        if not m:
            continue
        frames = [[int(v) for v in f.split(",")] for f in re.findall(r"\{([-\d, ]+)\}", m.group(7))]
        assert len(frames) == int(m.group(6)) and all(len(f) == len(NAMES) for f in frames), line
        out.append({"name": m.group(2), "lo": float(m.group(3)), "hi": float(m.group(4)),
                    "sided": m.group(5) == "true", "frames": frames})
    return out


def poses(mv: dict, a: float, mirror: bool) -> list[dict]:
    base = sit()
    out = []
    for f in mv["frames"]:
        js = dict(base)
        for name, c in zip(NAMES, f):
            d = c * a / 100.0
            if mirror and name in MIRROR:
                name = MIRROR[name]
            if mirror and name == "tail":
                d = -d
            js[name] = base[name] + d
        out.append(js)
    return out


def cases():
    for mv in moves():
        for a in (mv["lo"], mv["hi"]):
            for mirror in ((False, True) if mv["sided"] else (False,)):
                yield pytest.param(mv, a, mirror, id=f"{mv['name']}-{a:g}{'-mirrored' if mirror else ''}")


def test_the_table_is_read():
    names = [m["name"] for m in moves()]
    assert names == ["shift", "lean", "dip", "sway", "stretch", "hips", "tail", "paw", "shrug", "flinch"]
    assert sit() == dict(zip(NAMES, (10, -20, -60, 30, 30, -80, -80, 60, 60, 45, 45)))


@pytest.mark.parametrize("mv,a,mirror", list(cases()))
def test_inside_what_the_joints_reach(mv, a, mirror):
    base = sit()
    for js in poses(mv, a, mirror):
        for name, j in zip(NAMES, IDS):
            if js[name] == base[name]:
                continue  # unchanged (SIT's own, the right hind shoulder's -80 among them)
            lo, hi = T.reach(j)  # the limits and the pulse range, Kilo's recorded calibration
            assert lo <= js[name] <= hi, f"{name} (joint {j}) {js[name]:+.1f} outside {lo:+.1f}..{hi:+.1f}"


def _worst(g3, mv, a, mirror):
    frames = poses(mv, a, mirror) + [sit()]
    samples = T.follow(g3, frames, sit(), steps=12, R0=T.rest(g3, sit()).R)[1:]  # from SIT at rest
    margin = min(s.rest.margin for s in samples)
    roll = max(abs(s.rest.roll) for s in samples)
    drop = max((d for s in samples for _, _, d in s.rest.topples), default=0.0)
    allowed = all(set(s.rest.contacts) <= T.ALLOWED for s in samples)
    return margin, roll, drop, allowed


@pytest.mark.parametrize("mv,a,mirror", list(cases()))
def test_statically_stable_on_kilo_as_designed(mv, a, mirror):
    margin, roll, drop, allowed = _worst(RU.DESIGN, mv, a, mirror)
    assert allowed, "rests on something but the feet, the tail or the bottom"
    assert margin >= MARGIN_MM and roll <= ROLL_MAX_DEG and drop <= DROP_MAX_MM, (margin, roll, drop)


@pytest.mark.parametrize("mv,a,mirror", list(cases()))
def test_no_worse_than_sit_on_kilo_as_recorded(mv, a, mirror):
    """The right hind shoulder as recorded stops at -57 deg: SIT itself stands on three feet and
    rocks onto the fourth (rear_up.AS_RECORDED). The moves may rock it the same way, no further."""
    margin, roll, drop, allowed = _worst(RU.AS_RECORDED, mv, a, mirror)
    sit_rest = T.rest(RU.AS_RECORDED, sit())
    assert allowed and margin >= 0.0 and drop <= DROP_MAX_MM, (margin, roll, drop)
    assert roll <= max(ROLL_MAX_DEG, abs(sit_rest.roll) + 6.0), roll


def test_the_sitting_looks_are_inside_the_head_s_reach():
    src = _source()
    pan = float(re.search(r"kSitLookPanDeg = ([\d.]+)", src).group(1))
    lo, hi = (float(v) for v in re.search(r"kSitLookTiltLo = (-?[\d.]+), kSitLookTiltHi = (-?[\d.]+)", src).groups())
    for j, (a, b) in ((0, (-pan, pan)), (1, (lo, hi))):
        r = T.reach(j)
        assert r[0] <= a and b <= r[1], (j, r)
