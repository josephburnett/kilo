"""Joint calibration: which angles each joint can reach with its offset, and
which angles Kilo's postures, gaits and tricks ask of it (`kilo cal show`,
docs/calibration.md).

What the body does with a joint angle (body/src/core/joints.cpp,
config.cpp): the servo angle is ``zero + dir * angle`` with ``zero = 90 +
dir * mid + dir * offset`` (degrees; ``dir`` and ``mid`` per joint from the
legacy firmware, ``offset`` the calibration that CAL_SET sets), the pulse is
``a2p * servo / 1000 + b`` microseconds, clamped to 500..2500. Beyond the
clamp the servo goes no further: an angle the clamp cuts is not reached. The
body's joint limits clamp first. So a joint reaches

    [max(limit_lo, pulse_lo(offset)), min(limit_hi, pulse_hi(offset))]

and a larger offset moves the whole pulse window by the offset, the joint's
angles the same way (+ offset: the joint at angle 0 sits further toward +).

a2p and b are Kilo's (the legacy EEPROM dump of 2026-10-10, kept by the body
since; kilo.tricks.stance): 10.963 us per degree for every joint, b 600 us.
The protocol does not report them; they change only with a new EEPROM.

What is asked (``ASKED``): every angle the body is sent, after the joint
limits, from

- the postures (body/src/core/posture.cpp: REST, SIT, STAND, CALIB;
  ``POSTURES`` below, checked against the C table by
  mind/tests/test_calibration.py),
- the gaits: the planner (kilo.planner) at every remote level the walk, the
  trot and the crawl have, forward, backward, turning and steering as the
  remote drives them (the trot backward to -3 and at +4, +5 steering one
  level; each trot ended by the quick stop, whose square-up starts from
  the furthest back its feet go), through
  the body's IK (body/src/core/ik.h: shoulder = -theta1, knee = -(theta2 -
  sigma 90)),
- the trick library (kilo.tricks.library, the tricks the mind ships),

computed by ``compute_asked()`` (a few seconds) and kept here as a table;
the test recomputes it. Not in it: the head's look commands and the alive
layer's small moves (both stay well inside the postures' ranges).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence

from kilo.tricks.model import CANONICAL as DRIVEN
from kilo.tricks.model import JOINT_LIMITS
from kilo.tricks.model import JOINT_NAMES as NAMES

# ---- the body's joint table (body/src/core/joints.cpp JOINTS: dir, mid) --------------------------
JOINT_DIR = {0: 1, 1: -1, 2: 1, 8: 1, 9: -1, 10: -1, 11: 1, 12: -1, 13: 1, 14: 1, 15: -1}
JOINT_MID = {0: 0, 1: 15, 2: 0, 8: 10, 9: 10, 10: -10, 11: -10, 12: -30, 13: -30, 14: 30, 15: 30}
JOINT_CHANNEL = {0: 12, 1: 11, 2: 4, 8: 14, 9: 9, 10: 6, 11: 1, 12: 15, 13: 8, 14: 7, 15: 0}
LIMITS = {j: (float(lo), float(hi)) for j, (lo, hi) in JOINT_LIMITS.items()}
WHAT = {"head_pan": "head pan", "head_tilt": "head tilt", "tail": "tail",
        "lf_shoulder": "left front upper leg", "rf_shoulder": "right front upper leg",
        "rb_shoulder": "right hind upper leg", "lb_shoulder": "left hind upper leg",
        "lf_knee": "left front lower leg", "rf_knee": "right front lower leg",
        "rb_knee": "right hind lower leg", "lb_knee": "left hind lower leg"}

# servo = zero + dir * angle (degrees), pulse = (A2P * servo + B_US * 1000) / 1000 us, clamped to
# PULSE_MIN..PULSE_MAX (joints.cpp; a2p and b from Kilo's legacy EEPROM, kilo.tricks.stance).
A2P = 10963            # us per servo degree x 1000 (every joint)
B_US = 600
PULSE_MIN, PULSE_MAX = 500, 2500
OFFSET_MAX = 25.0      # the body's CAL_SET range, degrees (+-400 a16)

# ---- the postures (body/src/core/posture.cpp POSTURES, canonical order: pan, tilt, tail,
# shoulders LF RF RB LB, knees LF RF RB LB) ---------------------------------------------------------
POSTURES = {
    "rest": (-30, -80, -45, 60, 60, -60, -60, -45, -45, 45, 45),
    "sit": (10, -20, -60, 30, 30, -90, -90, 60, 60, 45, 45),
    "stand": (0, 0, 0, 30, 30, -30, -30, 30, 30, -30, -30),
    "calib": (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
}


def offset_from_czp(j: int, czp: float) -> float:
    """The calibration offset (degrees) of a legacy calibrated zero position (CZP = zero_deg)."""
    return JOINT_DIR[j] * (czp - 90.0) - JOINT_MID[j]


def zero_deg(j: int, offset_deg: float) -> float:
    """The servo angle at joint angle 0 (config.cpp: 90 + dir mid + dir offset)."""
    return 90.0 + JOINT_DIR[j] * JOINT_MID[j] + JOINT_DIR[j] * offset_deg


def pulse_us(j: int, angle_deg: float, offset_deg: float) -> float:
    """The pulse for joint angle angle_deg, before the 500..2500 clamp."""
    return A2P * (zero_deg(j, offset_deg) + JOINT_DIR[j] * angle_deg) / 1000.0 + B_US


def angle_window(j: int, zero: float) -> tuple[float, float]:
    """The joint angles at 500 and at 2500 us, lower first, for joint j whose
    servo angle at joint angle 0 is zero (outside them the pulse is clamped)."""
    s_lo = (PULSE_MIN - B_US) * 1000.0 / A2P   # servo degrees at 500 us
    s_hi = (PULSE_MAX - B_US) * 1000.0 / A2P   # ...at 2500 us
    d = JOINT_DIR[j]
    a, b = (s_lo - zero) * d, (s_hi - zero) * d
    return min(a, b), max(a, b)


def pulse_window(j: int, offset_deg: float) -> tuple[float, float]:
    """The joint angles at 500 and at 2500 us, lower first."""
    return angle_window(j, zero_deg(j, offset_deg))


def reach(j: int, offset_deg: float) -> tuple[float, float]:
    """The angles joint j really goes to: its limits and the pulse window."""
    lo, hi = LIMITS[j]
    plo, phi = pulse_window(j, offset_deg)
    return max(lo, plo), min(hi, phi)


def offsets_covering(j: int, lo: float, hi: float) -> tuple[float, float] | None:
    """The offsets (degrees) whose pulse window covers [lo, hi], within the
    body's +-25; None if none does. The window moves with the offset: its
    angles are those at offset 0 plus the offset times -1 (it slides
    against the offset)."""
    plo0, phi0 = pulse_window(j, 0.0)
    # pulse_window(j, o) = (plo0 - o, phi0 - o): covering needs plo0 - o <= lo and phi0 - o >= hi
    o_min, o_max = plo0 - lo, phi0 - hi
    o_min, o_max = max(o_min, -OFFSET_MAX), min(o_max, OFFSET_MAX)
    return (o_min, o_max) if o_min <= o_max else None


@dataclass(frozen=True)
class Asked:
    lo: float
    hi: float
    lo_by: str  # what asks the lowest angle
    hi_by: str


# ---- what is asked ---------------------------------------------------------------------------------
def _clamp(j: int, a: float) -> float:
    lo, hi = LIMITS[j]
    return min(hi, max(lo, a))


def _note(acc: dict, j: int, a: float, by: str) -> None:
    a = _clamp(j, a)
    cur = acc.get(j)
    if cur is None:
        acc[j] = [a, a, by, by]
        return
    if a < cur[0]:
        cur[0], cur[2] = a, by
    if a > cur[1]:
        cur[1], cur[3] = a, by


# The remote's speed levels (x the gait's spec speed) and steer levels: kernel/mind/remote/remote.cpp
# kSpeedFactors, kTrotSpeedFactors, kSteerLevels; the trot backward only to -3 (kTrotBackSteps) and
# steering one level at +4 and +5 (kTrotLongFrom, kTrotFastSteerSteps).
SPEED_FACTORS = (0.4, 0.7, 1.0, 1.15, 1.25)
TROT_SPEED_FACTORS = (0.4, 0.7, 1.0, 1.25, 1.4)
STEER_LEVELS = (0.25, 0.5, 1.0)


def _gait_cases():
    for gait in ("walk", "trot", "crawl"):
        for lv, f in enumerate(TROT_SPEED_FACTORS if gait == "trot" else SPEED_FACTORS, start=1):
            trot_long = gait == "trot" and lv >= 4
            steers = STEER_LEVELS[:1] if trot_long else STEER_LEVELS
            yield gait, "forward", f, 0.0, f"{gait} +{lv}"
            if not trot_long:
                yield gait, "back", f, 0.0, f"{gait} -{lv}"
            yield gait, "left", f, 0.0, f"{gait} turn {lv}"
            for s in steers:
                yield gait, "forward", f, s, f"{gait} +{lv} steer {s:g}"
                yield gait, "forward", f, -s, f"{gait} +{lv} steer {-s:g}"
                if not trot_long:
                    yield gait, "back", f, -s, f"{gait} -{lv} steer {s:g}"


CATEGORIES = ("postures", "gaits", "tricks")


def compute_asked(ticks: int = 200) -> dict[str, dict[int, Asked]]:
    """Per category (postures, gaits, tricks): every joint's lowest and highest
    asked angle (degrees, after the joint limits) and what asks it."""
    from kilo.planner import patterns as pat
    from kilo.planner.model import BodyModel
    from kilo.planner.plan import Planner
    from kilo.tricks import library

    cats: dict[str, dict] = {c: {} for c in CATEGORIES}
    acc = cats["postures"]
    for name, row in POSTURES.items():
        for k, j in enumerate(DRIVEN):
            _note(acc, j, row[k], name.upper())
    acc = cats["gaits"]
    model = BodyModel()
    for gait, d, f, steer, label in _gait_cases():
        p = Planner(model, gait, d, pat.default_speed(gait) * f, steer=steer)
        plans = [p.next() for _ in range(ticks)]
        if gait == "trot":  # the trot's quick stop from its long stride squares up from near the reach
            p.halt()
            while not p.done and len(plans) < 3 * ticks:
                plans.append(p.next())
        for tp in plans:
            for leg in range(4):
                t1, t2 = model.ik(leg, float(tp.x[leg]), float(tp.z[leg]))
                sigma = 1.0 if leg < 2 else -1.0
                _note(acc, 8 + leg, -math.degrees(t1), label)
                _note(acc, 12 + leg, -(math.degrees(t2) - sigma * 90.0), label)
    acc = cats["tricks"]
    for name, trick in library.load_all().items():
        for fr in trick.frames:
            for j, a in enumerate(fr.angles):
                if a is not None and j in LIMITS:
                    _note(acc, j, float(a), name)
    return {c: {j: Asked(round(float(v[0]), 1), round(float(v[1]), 1), v[2], v[3]) for j, v in sorted(a.items())}
            for c, a in cats.items()}


# compute_asked() as of this table (mind/tests/test_calibration.py checks it is current;
# python -m kilo.calibration prints it)
ASKED: dict[str, dict[int, Asked]] = {
    'postures': {
        0: Asked(-30.0, 10.0, 'REST', 'SIT'),
        1: Asked(-80.0, 0.0, 'REST', 'STAND'),
        2: Asked(-60.0, 0.0, 'SIT', 'STAND'),
        8: Asked(0.0, 60.0, 'CALIB', 'REST'),
        9: Asked(0.0, 60.0, 'CALIB', 'REST'),
        10: Asked(-80.0, 0.0, 'SIT', 'CALIB'),
        11: Asked(-80.0, 0.0, 'SIT', 'CALIB'),
        12: Asked(-45.0, 60.0, 'REST', 'SIT'),
        13: Asked(-45.0, 60.0, 'REST', 'SIT'),
        14: Asked(-30.0, 45.0, 'STAND', 'REST'),
        15: Asked(-30.0, 45.0, 'STAND', 'REST'),
    },
    'gaits': {
        8: Asked(-9.3, 75.3, 'crawl -5', 'crawl -5'),
        9: Asked(-10.8, 75.8, 'crawl -1 steer 0.5', 'crawl -4'),
        10: Asked(-80.0, -5.2, 'crawl -5', 'trot +5 steer 0.25'),
        11: Asked(-79.9, -2.8, 'crawl -2', 'trot +5 steer -0.25'),
        12: Asked(-16.1, 48.4, 'crawl -1 steer 1', 'crawl -5'),
        13: Asked(-16.5, 50.3, 'crawl turn 4', 'crawl -1 steer 0.5'),
        14: Asked(-34.5, 21.0, 'walk -2 steer 1', 'crawl +1 steer 0.5'),
        15: Asked(-35.6, 19.3, 'trot +5 steer -0.25', 'crawl +4'),
    },
    'tricks': {
        0: Asked(-45.0, 95.0, 'wash_face', 'wash_face'),
        1: Asked(-80.0, 45.0, 'rest', 'stretch'),
        2: Asked(-86.0, 70.0, 'hi', 'check_around'),
        8: Asked(-87.0, 80.0, 'wash_face', 'wash_face'),
        9: Asked(-75.0, 80.0, 'stretch', 'butt_up'),
        10: Asked(-80.0, 95.0, 'sit', 'push_up'),
        11: Asked(-80.0, 95.0, 'sit', 'push_up'),
        12: Asked(-60.0, 85.0, 'butt_up', 'wash_face'),
        13: Asked(-60.0, 82.0, 'butt_up', 'wash_face'),
        14: Asked(-75.0, 60.0, 'push_up', 'hi'),
        15: Asked(-75.0, 45.0, 'push_up', 'sit'),
    },
}


@dataclass(frozen=True)
class JointRange:
    joint: int
    name: str
    offset: float
    reach: tuple[float, float]               # what it reaches with this offset
    asked: dict[str, Asked]                  # by category
    short: dict[str, tuple[float, float]]    # by category: degrees short below, above (0: reached)
    covering: tuple[float, float] | None     # the offsets that reach the postures and gaits (None: none)

    def ok(self, category: str) -> bool:
        lo, hi = self.short.get(category, (0.0, 0.0))
        return lo <= 0.05 and hi <= 0.05


def ranges(offsets_a16: Sequence[int], asked: Mapping[str, Mapping[int, Asked]] | None = None) -> list[JointRange]:
    """Per driven joint: its reach with the offset given (CAL_VALUES, a16) against what is asked."""
    asked = ASKED if asked is None else asked
    out = []
    for j in DRIVEN:
        off = offsets_a16[j] / 16.0
        r = reach(j, off)
        a = {c: asked[c][j] for c in CATEGORIES if j in asked.get(c, {})}
        short = {c: (max(0.0, r[0] - v.lo), max(0.0, v.hi - r[1])) for c, v in a.items()}
        need = [v for c, v in a.items() if c != "tricks"]
        cov = offsets_covering(j, min(v.lo for v in need), max(v.hi for v in need)) if need else None
        out.append(JointRange(j, NAMES[j], off, r, a, short, cov))
    return out


def _span(a: Asked | None) -> str:
    return f"{a.lo:+5.0f} .. {a.hi:+4.0f}" if a else "-"


def range_lines(offsets_a16: Sequence[int], asked: Mapping[str, Mapping[int, Asked]] | None = None) -> list[str]:
    """`kilo cal show`'s table: offset, reach, what the postures, gaits and tricks ask, and every shortfall."""
    rs = ranges(offsets_a16, asked)
    lines = [f"{'joint':<17} {'offset':>7}  {'reaches':>15}   {'postures':>12}  {'gaits':>12}  {'tricks':>12}"]
    notes = []
    for r in rs:
        flag = "" if all(r.ok(c) for c in r.asked) else "  SHORT"
        lines.append(f"{r.joint:2d} {r.name:<14} {r.offset:+7.2f}  {r.reach[0]:+6.1f} .. {r.reach[1]:+6.1f}   "
                     f"{_span(r.asked.get('postures')):>12}  {_span(r.asked.get('gaits')):>12}  "
                     f"{_span(r.asked.get('tricks')):>12}{flag}")
        for c in CATEGORIES:
            if c not in r.asked or r.ok(c):
                continue
            a, (sl, sh) = r.asked[c], r.short[c]
            if sl > 0.05:
                notes.append(f"joint {r.joint} ({WHAT[r.name]}): {a.lo:+.0f} ({a.lo_by}) stops at {r.reach[0]:+.1f}")
            if sh > 0.05:
                notes.append(f"joint {r.joint} ({WHAT[r.name]}): {a.hi:+.0f} ({a.hi_by}) stops at {r.reach[1]:+.1f}")
        if any(not r.ok(c) for c in r.asked if c != "tricks"):
            notes.append(f"   joint {r.joint}: " + (f"an offset of {r.covering[0]:+.1f} .. {r.covering[1]:+.1f} "
                                                     "reaches all the postures and gaits ask"
                                                     if r.covering else "no offset within +-25 reaches all the "
                                                     "postures and gaits ask"))
    if notes:
        lines.append("")
        lines.append("short of what is asked (the servo stops at the end of its pulse range, 500 or 2500 us):")
        lines += [f"  {n}" for n in notes]
    return lines


def main() -> None:  # python -m kilo.calibration: print the ASKED table to paste above
    print("ASKED: dict[str, dict[int, Asked]] = {")
    for c, a in compute_asked().items():
        print(f"    {c!r}: {{")
        for j, v in a.items():
            print(f"        {j}: Asked({v.lo}, {v.hi}, {v.lo_by!r}, {v.hi_by!r}),")
        print("    },")
    print("}")


if __name__ == "__main__":
    main()
