"""kilo.calibration (each joint's reach with its offset, what is asked of it)
and `kilo cal` (cli_cal) on a fake host link: the tables against the body's
C sources, the numbers behind docs/calibration.md and docs/rear-up-trick.md
section 2, the nudge and its range check."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import pytest

from kilo import calibration as C
from kilo import cli_cal
from kilo.protocol import kilo_protocol as P
from kilo.tricks import stance as T

BODY = Path(__file__).resolve().parents[2] / "body" / "src" / "core"


def kilo_offsets_a16() -> list[int]:
    """Kilo's calibration as recorded (the EEPROM dump of 2026-10-10, kilo.tricks.stance)."""
    offs = [0] * 16
    for j in C.DRIVEN:
        offs[j] = round(C.offset_from_czp(j, T.KILO_CZP[j]) * 16)
    return offs


# ---- the tables against the body's sources ------------------------------------------------------
def test_joint_table_matches_joints_cpp():
    src = (BODY / "joints.cpp").read_text()
    rows = re.findall(r"\{(\d+), (-?\d+), (-?\d+), KILO_JOINT_\w+_LO, KILO_JOINT_\w+_HI\},\s*// (\d+)", src)
    got = {int(j): (int(ch), int(d), int(m)) for ch, d, m, j in rows}
    for j in C.DRIVEN:
        assert got[j] == (C.JOINT_CHANNEL[j], C.JOINT_DIR[j], C.JOINT_MID[j]), j


def test_postures_match_posture_cpp():
    src = (BODY / "posture.cpp").read_text()
    rows = [tuple(int(x) for x in r.split(",")) for r in re.findall(r"\{\{([-\d, ]+)\}, -?\d+, -?\d+\}", src)]
    assert rows == [C.POSTURES[n] for n in ("rest", "sit", "stand", "calib", "recovery 1", "recovery 2",
                                           "recovery 3")]


def test_reach_is_the_stance_models():
    offs = kilo_offsets_a16()
    for j in C.DRIVEN:
        assert C.reach(j, offs[j] / 16) == pytest.approx(T.reach(j), abs=1e-6)


def test_asked_table_is_current():
    assert C.compute_asked() == C.ASKED, "stale: python -m kilo.calibration, paste into kilo/calibration.py"


# ---- the numbers in the docs --------------------------------------------------------------------
def test_the_right_hind_shoulder():
    offs = kilo_offsets_a16()
    assert offs[10] == -16 * 16 and offs[11] == 2 * 16
    assert C.reach(10, -16) == pytest.approx((-57.3, 100.0), abs=0.05)  # SIT and the rear-up ask -80
    # what reaches the joint limit forward (-80) and the recovery's +100 at once: +6.7 .. +9.1
    assert C.offsets_covering(10, -80, 100) == pytest.approx((6.69, 9.12), abs=0.01)
    assert C.offsets_covering(10, -80, 0) == pytest.approx((6.69, 25.0), abs=0.01)
    # offset 0 is not the servo's centre for this joint: 1696 us at angle 0; 1500 us is at +17.9
    assert C.pulse_us(10, 0, 0) == pytest.approx(1696.3, abs=0.1)
    assert C.pulse_us(10, 0, 17.9) == pytest.approx(1500, abs=1)
    # the target of the re-seat, +16: angle 0 at 1521 us; any final offset +7 .. +25 reaches -80
    assert C.pulse_us(10, 0, 16) == pytest.approx(1520.9, abs=0.1)
    for o in (7, 16, 25):
        assert C.reach(10, o)[0] == -80.0
    assert C.reach(10, 25)[1] == pytest.approx(84.1, abs=0.05) and C.reach(10, 7)[1] == 100.0


def test_show_flags_every_short_joint():
    text = "\n".join(C.range_lines(kilo_offsets_a16()))
    assert "joint 12 (left front lower leg): -45 (REST) stops at -32.3" in text
    assert "joint 10 (right hind upper leg): -80 (SIT) stops at -57.3" in text
    assert "joint 10: an offset of +6.7 .. +9.1 reaches all the postures and gaits ask" in text
    flagged = {int(m) for m in re.findall(r"^ ?(\d+) \w+ .*SHORT$", text, re.M)}
    assert flagged == {8, 9, 10, 11, 12, 14}
    ok = list(C.ranges([0] * 16))
    assert all(r.reach[0] <= r.reach[1] for r in ok)


# ---- kilo cal on a fake host link ---------------------------------------------------------------
class FakeHost:
    def __init__(self, offs):
        self.offs = list(offs)
        self.calls = []

    def cal(self, op, joint=0, offset_a16=0):
        self.calls.append((int(op), joint, offset_a16))
        if op == P.CalOp.SET:
            self.offs[joint] = offset_a16
        return P.HostCalStatus(op=int(op), saved=1 if op == P.CalOp.SAVE else 0, offset=list(self.offs))


def ns(**kw):
    return argparse.Namespace(**{"cal_cmd": "show", "no_wait": False, **kw})


def test_cli_nudge_and_range(capsys):
    h = FakeHost(kilo_offsets_a16())
    assert cli_cal.cmd_cal_host(ns(cal_cmd="set", joint=10, degrees=16.0), h) == 0
    assert cli_cal.cmd_cal_host(ns(cal_cmd="nudge", joint=10, degrees=-2.5), h) == 0
    out = capsys.readouterr().out
    assert "joint 10 (right hind upper leg): offset -16.00 -> +16.00 deg (not saved: kilo cal save --host)" in out
    assert "offset +16.00 -> +13.50 deg" in out and "reaches -80.0 .. +95.6" in out
    assert h.offs[10] == round(13.5 * 16)
    # past +25: refused before anything is sent, with the range and what still fits
    n = len(h.calls)
    assert cli_cal.cmd_cal_host(ns(cal_cmd="nudge", joint=10, degrees=12.0), h) == 1
    err = capsys.readouterr().err
    assert "outside the body's offset range -25 .. +25 deg (now +13.50; a nudge of at most +11.50 fits)" in err
    assert [c[0] for c in h.calls[n:]] == [P.CalOp.GET]
    assert cli_cal.cmd_cal_host(ns(cal_cmd="set", joint=5, degrees=1.0), h) == 1
    assert "not a driven joint" in capsys.readouterr().err
    assert cli_cal.cmd_cal_host(ns(cal_cmd="save"), h) == 0
    assert "saved to the body's EEPROM" in capsys.readouterr().out
    assert cli_cal.cmd_cal_host(ns(cal_cmd="show"), h) == 0
    assert "10 rb_shoulder" in capsys.readouterr().out
