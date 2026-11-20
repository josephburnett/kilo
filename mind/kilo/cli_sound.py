"""``kilo sound``: Kilo's sounds, over --host (a C++ mind; docs/sound.md).

    kilo sound [status] --host IP [--json]   on or off, the volume, the sounds in the image, what
                                             played last, the counters since boot
    kilo sound list --host IP [--json]       the sounds built into the image: name, length, rate
    kilo sound play NAME --host IP           play a sound (meow2) or one of a family (meow)
    kilo sound on|off --host IP              sounds on or off (off: every play request is refused)
    kilo sound volume N --host IP            the volume, 0..100 (the mind starts at 70)

Neither on/off nor the volume survives a reboot: the mind starts on, at 70.
A play request waits while another sound plays, and starts at least a
second after it ends; a newer request replaces a waiting one.
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any

from kilo import EXIT_ERR, EXIT_OK
from kilo.hostlink import enum_name
from kilo.protocol import kilo_protocol as P

ACTIONS = ("status", "list", "play", "on", "off", "volume")
NAME_MAX = P.HostSound.MAX_LEN - 2  # SOUND.name: 15 bytes

RESULT_TEXT = {
    P.SoundResult.OK: "",
    P.SoundResult.QUEUED: "queued",
    P.SoundResult.UNKNOWN: "no such sound in the image (kilo sound list)",
    P.SoundResult.OFF: "sounds are off (kilo sound on)",
    P.SoundResult.NO_SOUNDS: "this image has no sounds (none were packed from kernel/sounds, or it was built "
                             "with KILO_SOUND=0)",
    P.SoundResult.ASLEEP: "Kilo is in light sleep: the speaker is off (any key wakes him)",
}


def add_parser(sub: Any) -> None:
    s = sub.add_parser("sound", help="Kilo's sounds over --host: status, list, play NAME, on, off, volume N "
                                     "(docs/sound.md)",
                       description="Kilo's sounds on a C++ mind (docs/sound.md): what is in the image, play one, "
                                   "switch them on or off, set the volume. Neither on/off nor the volume survives "
                                   "a reboot.")
    s.add_argument("action", nargs="?", choices=ACTIONS, default="status")
    s.add_argument("value", nargs="?", default=None, help="play: the sound or family; volume: 0..100")
    s.add_argument("--json", action="store_true")


def status_dict(st: Any) -> dict[str, Any]:
    d = P.to_dict(st)
    d["result"] = enum_name(P.SoundResult, st.result).lower()
    d["state"] = enum_name(P.SoundState, st.state).lower()
    d["last"] = bytes(st.last).decode("ascii", "replace")
    return d


def status_lines(st: Any) -> list[str]:
    """`kilo status --host`'s line and `kilo sound`'s."""
    state = enum_name(P.SoundState, st.state).lower()
    last = bytes(st.last).decode("ascii", "replace") or "-"
    note = "  (no sound output: built with KILO_SOUND=0)" if st.state == P.SoundState.NONE else ""
    return [f"sound       {'on' if st.enabled else 'OFF'}  volume {st.volume}  {st.sounds} sounds in the image  "
            f"{state}  last {last}{note}"]


def count_line(st: Any) -> str:
    return (f"sound count played {st.played}  requests {st.requests}  unknown {st.unknown}  dropped {st.dropped}  "
            f"late chunks {st.late}  longest callback {st.callback_max_us} us")


def info_dict(i: Any) -> dict[str, Any]:
    return {"name": bytes(i.name).decode("ascii", "replace"), "rate": i.rate, "samples": i.samples,
            "seconds": round(i.samples / i.rate, 3) if i.rate else 0.0}


def _play_value(args: argparse.Namespace) -> str | None:
    v = args.value
    if not v:
        return None
    return v.strip().lower()


def cmd_sound(args: argparse.Namespace, hl: Any) -> int:
    action = args.action
    if action == "list":
        infos = hl.sound_list()
        rows = [info_dict(i) for i in infos]
        if args.json:
            print(json.dumps(rows, indent=2))
        elif not rows:
            print("no sounds in this image (kernel/sounds/*.wav are packed by `make -C kernel`; docs/sound.md)")
        else:
            for r in rows:
                print(f"{r['name']:<15} {r['seconds']:6.2f} s  {r['rate']} Hz")
            print(f"{len(rows)} sounds, {sum(r['seconds'] for r in rows):.1f} s")
        return EXIT_OK
    if action == "play":
        name = _play_value(args)
        if not name:
            print("kilo sound play: name a sound or a family (kilo sound list)", file=sys.stderr)
            return EXIT_ERR
        if len(name.encode()) > NAME_MAX:
            print(f"kilo sound play: {name!r}: names are at most {NAME_MAX} characters",
                  file=sys.stderr)
            return EXIT_ERR
        st = hl.sound(P.SoundOp.PLAY, name=name)
    elif action == "volume":
        try:
            vol = int(args.value)
        except (TypeError, ValueError):
            vol = -1
        if not 0 <= vol <= 100:
            print("kilo sound volume: a number 0..100", file=sys.stderr)
            return EXIT_ERR
        st = hl.sound(P.SoundOp.VOLUME, volume=vol)
    elif action in ("on", "off"):
        st = hl.sound(P.SoundOp.ON if action == "on" else P.SoundOp.OFF)
    else:
        st = hl.sound(P.SoundOp.STATUS)
    if args.json:
        print(json.dumps(status_dict(st), indent=2))
    else:
        if action == "play":
            text = RESULT_TEXT.get(st.result, enum_name(P.SoundResult, st.result))
            print(f"sound       {args.value}: {text}")
        print("\n".join(status_lines(st)))
        print(count_line(st))
    if action == "play" and st.result != P.SoundResult.QUEUED:
        return EXIT_ERR
    return EXIT_OK
