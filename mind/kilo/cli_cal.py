"""`kilo cal`: joint calibration, over --host (the C++ mind relays CAL to the
body). docs/calibration.md.

    kilo cal [show] [--host H]           the 16 offsets, and per joint what it reaches with its
                                         offset against what the postures, gaits and tricks ask
    kilo cal set JOINT DEG [--host H]    one joint's absolute offset (not saved)
    kilo cal nudge JOINT +-DEG [--host H]  change one joint's offset by DEG (not saved): with the
                                         calibration posture held, the joint moves at once
    kilo cal save [--host H]             write the calibration to the body's EEPROM
    kilo cal revert [--host H]           back to the saved calibration

Offsets are degrees, -25 .. +25 (the body's range); a + offset turns the
joint toward its + angles (a shoulder: the upper leg back, toward the tail).
Set / nudge / save / revert need control, or the calibration
posture held by any client (`kilo posture calib --host H` in another
window), and no walk or trick running.
"""

from __future__ import annotations

import argparse
import sys
from typing import Any, Callable

from kilo import EXIT_ERR, EXIT_OK
from kilo import calibration as C
from kilo.protocol import kilo_protocol as P



def add_parser(sub: Any) -> None:
    s = sub.add_parser("cal", help="joint calibration (show, set, nudge, save, revert)")
    cs = s.add_subparsers(dest="cal_cmd", metavar="ACTION")
    parsers = [cs.add_parser("show", help="the 16 offsets and each joint's reach against what is asked (default)")]
    c = cs.add_parser("set", help="set one joint's absolute offset (not saved)")
    c.add_argument("joint", type=int)
    c.add_argument("degrees", type=float, help=f"absolute offset in degrees (-{C.OFFSET_MAX:g} .. +{C.OFFSET_MAX:g})")
    parsers.append(c)
    c = cs.add_parser("nudge", help="change one joint's offset by DEG (not saved); moves a held pose at once")
    c.add_argument("joint", type=int)
    c.add_argument("degrees", type=float, help="the change in degrees, + or - (e.g. -2: write it as -- -2 or =-2)")
    parsers.append(c)
    parsers.append(cs.add_parser("save", help="write the calibration to the body's EEPROM"))
    parsers.append(cs.add_parser("revert", help="reload the saved calibration"))
    for p in parsers:  # `kilo cal show --host H`: the option after the action
        p.add_argument("--host", default=argparse.SUPPRESS, metavar="IP[:PORT]", help="a C++ mind (kilo.local, IP)")
        p.add_argument("--key", default=argparse.SUPPRESS, metavar="PATH", help=argparse.SUPPRESS)


def _who(j: int) -> str:
    n = C.NAMES.get(j)
    return f"joint {j} ({C.WHAT[n]})" if n in C.WHAT else f"joint {j}"


def check_offset(j: int, new_deg: float, now_deg: float | None = None) -> str | None:
    """Why new_deg cannot be set on joint j (None: it can)."""
    if not 0 <= j < 16:
        return f"joint {j}: the joints are 0..15 (kilo cal show lists them)"
    if j not in C.DRIVEN:
        return f"joint {j}: not a driven joint on Kilo (driven: {', '.join(map(str, C.DRIVEN))})"
    if abs(new_deg) > C.OFFSET_MAX + 1e-9:
        msg = (f"{_who(j)}: {new_deg:+.2f} deg is outside the body's offset range "
               f"-{C.OFFSET_MAX:g} .. +{C.OFFSET_MAX:g} deg")
        if now_deg is not None:
            room = (C.OFFSET_MAX - now_deg) if new_deg > 0 else (-C.OFFSET_MAX - now_deg)
            msg += f" (now {now_deg:+.2f}; a nudge of at most {room:+.2f} fits)"
        return msg + ". A leg that needs more sits a spline tooth off: re-seat it (docs/calibration.md)."
    return None


def joint_line(offsets_a16: list[int], j: int) -> str:
    r = next(x for x in C.ranges(offsets_a16) if x.joint == j)
    short = []
    for c, a in r.asked.items():
        sl, sh = r.short[c]
        if sl > 0.05:
            short.append(f"{a.lo:+.0f} ({a.lo_by})")
        if sh > 0.05:
            short.append(f"{a.hi:+.0f} ({a.hi_by})")
    return (f"{_who(j)}: offset {r.offset:+.2f} deg, reaches {r.reach[0]:+.1f} .. {r.reach[1]:+.1f}"
            + (f"; not reached: {', '.join(short)}" if short else "; reaches all the postures, gaits and tricks ask"))


def show(offsets_a16: list[int]) -> None:
    print("offsets (degrees; + turns a joint toward its + angles: a shoulder's upper leg back, toward the tail)")
    print("\n".join(C.range_lines(offsets_a16)))
    print("reaches: the angles the joint goes to with its offset (its limits and the 500..2500 us pulse range)")


def run(args: argparse.Namespace, get: Callable[[], list[int]], set_: Callable[[int, int], list[int]],
        save: Callable[[], bool], revert: Callable[[], list[int]], where: str) -> int:
    """The actions. get() -> offsets (a16); set_(j, a16) -> offsets after."""
    cmd = args.cal_cmd or "show"
    if cmd == "show":
        show(get())
        return EXIT_OK
    if cmd in ("set", "nudge"):
        j = args.joint
        now = None
        if cmd == "nudge" or 0 <= j < 16:
            offs = get()
            now = offs[j] / 16.0 if 0 <= j < 16 else None
        new = args.degrees if cmd == "set" else (now or 0.0) + args.degrees
        why = check_offset(j, new, now)
        if why is not None:
            print(f"kilo cal {cmd}: {why}", file=sys.stderr)
            return EXIT_ERR
        offs = set_(j, int(round(new * 16)))
        was = f"{now:+.2f} -> " if now is not None else ""
        print(f"{_who(j)}: offset {was}{offs[j] / 16:+.2f} deg (not saved: kilo cal save{where})")
        print(joint_line(offs, j))
        return EXIT_OK
    if cmd == "save":
        ok = save()
        print("saved to the body's EEPROM" if ok else "save queued, but no EEPROM_SAVED came within 2 s")
        return EXIT_OK if ok else EXIT_ERR
    if cmd == "revert":
        revert()
        print("reverted to the saved calibration")
        return EXIT_OK
    print(f"kilo cal: unknown action {cmd}", file=sys.stderr)
    return EXIT_ERR


def cmd_cal_host(args: argparse.Namespace, hl: Any) -> int:
    """Over --host: CAL relayed by the mind."""
    def get() -> list[int]:
        return list(hl.cal(P.CalOp.GET).offset)

    def set_(j: int, a16: int) -> list[int]:
        return list(hl.cal(P.CalOp.SET, j, a16).offset)

    def save() -> bool:
        return bool(hl.cal(P.CalOp.SAVE).saved)

    def revert() -> list[int]:
        return list(hl.cal(P.CalOp.REVERT).offset)

    return run(args, get, set_, save, revert, " --host")

