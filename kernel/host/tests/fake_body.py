#!/usr/bin/env python3
"""A minimal fake body on a pty, for the cases the virtual body cannot play
(it is the real core and always speaks this checkout's protocol).

  fake_body.py --silent            open a pty and never send anything
  fake_body.py --proto-major 1     answer HELLO_REQ with a HELLO of that
                                   protocol major and stream TELEMETRY at 25 Hz

Prints "fake body on /dev/pts/N" and runs until killed. Uses protocol/'s own
Python codec and generated module (stdlib only).
"""
from __future__ import annotations

import argparse
import os
import select
import sys
import time
import tty
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(REPO / "protocol" / "py"), str(REPO / "protocol" / "gen")]
import kilo_link_codec as C  # noqa: E402
import kilo_protocol as P  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--silent", action="store_true")
    ap.add_argument("--proto-major", type=int, default=P.PROTO_MAJOR)
    args = ap.parse_args()
    master, slave = os.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    print(f"fake body on {os.ttyname(slave)}", flush=True)
    dec = C.StreamDecoder()
    seq = tick = 0
    next_telemetry = time.monotonic()
    while True:
        r, _, _ = select.select([master], [], [], 0.01)
        data = os.read(master, 256) if r else b""
        if args.silent:
            continue
        out = b""
        for f in dec.feed(data):
            if isinstance(f, C.Frame) and f.type == P.MsgId.HELLO_REQ:
                hello = P.Hello(proto_major=args.proto_major, proto_minor=0, fw_build=0x46414B45,  # "FAKE"
                                reset_cause=P.ResetCause.COLD, max_payload=P.MAX_PAYLOAD,
                                joint_count=P.JOINT_COUNT)
                out += C.build_frame(P.MsgId.HELLO, seq & 0xFF, hello.pack())
                seq += 1
        if time.monotonic() >= next_telemetry:
            next_telemetry += 0.04
            out += C.build_frame(P.MsgId.TELEMETRY, seq & 0xFF, P.Telemetry(tick=tick & 0xFFFF).pack())
            seq += 1
            tick += 2
        if out:
            os.write(master, out)


if __name__ == "__main__":
    sys.exit(main())
