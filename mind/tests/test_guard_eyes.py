"""The distance guard's lines in `kilo guard` and `kilo status --host`: the
eyes orange for low battery, and why (kernel/mind/guard/guard.h, docs/distance-guard.md)."""

from __future__ import annotations

from kilo.cli_host import guard_dict, guard_lines
from kilo.protocol import kilo_protocol as P


def status(eyes: int, zone: int = P.GuardZone.GREEN) -> P.HostGuardStatus:
    return P.HostGuardStatus(enabled=1, zone=zone, action=0, eyes=eyes, eyes_driven=1, dist_mm=0, raw_mm=0,
                             blue_mm=200, red_mm=150, speed_mm_s=0, zone_changes=0, stops=0, pullbacks=0, refused=0)


def test_orange_says_low_battery():
    g = status(P.GuardEyes.ORANGE)
    lines = guard_lines(g)
    assert lines[0].startswith("guard       on  zone green ")
    assert lines[1].startswith("eyes        orange: low battery, Kilo is tired") and lines[1].endswith("(on the eyes)")
    d = guard_dict(g)
    assert d["eyes"] == "orange" and d["zone"] == "green"


def test_the_zones_colours():
    assert guard_lines(status(P.GuardEyes.BLUE, P.GuardZone.BLUE))[1] == "eyes        blue (on the eyes)"
    assert guard_dict(status(P.GuardEyes.RED, P.GuardZone.RED))["eyes"] == "red"
