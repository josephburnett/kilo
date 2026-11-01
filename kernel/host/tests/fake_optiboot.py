#!/usr/bin/env python3
"""A body that can be flashed, on a pty: the virtual body (kilo.vbody.sim's
VirtualBody) until its RESET line is pulled, then optiboot speaking STK500v1,
then the virtual body again as the new firmware. For kilo-mind-host
--reset-fifo (kernel/host/posix.h FifoResetGpio) and `kilo flash --host`
(kernel/host/test.sh).

  fake_optiboot.py --reset-fifo PATH [--build HEX] [--new-build HEX]
                   [--no-bootloader] [--listen-ms MS] [--image-out FILE]

RESET is a named pipe this program creates: 'L' pulls the line low (the
firmware stops), 'H' releases it. Then, as the Uno's optiboot:
- it listens --listen-ms after the release (start-up delay and 3 LED
  flashes, 440 ms); until then the 328P's RX FIFO keeps the first three
  bytes and loses the rest;
- GET_SYNC, ENTER_PROGMODE and the others: INSYNC OK; LOAD_ADDRESS (word
  address); PROG_PAGE stores the page; READ_PAGE reads it back; READ_SIGN
  is 1E 95 0F; LEAVE_PROGMODE answers and starts the firmware;
- a command not ended by CRC_EOP, or 1 s without a byte (its watchdog),
  also starts the firmware.
The firmware starts 0.3 s later (its init) as a fresh virtual body. A fake
cannot run AVR code, so after a programming session it reports --new-build
as HELLO.fw_build; the reset cause follows the body's reset record
(docs/architecture.md §3a): FLASHED when the build changed, else
WARM_UNEXPECTED. --no-bootloader: RESET just restarts the firmware.
--image-out: after LEAVE_PROGMODE, the programmed bytes (up to the last page
written) go to FILE.

Prints "fake optiboot body on /dev/pts/N", then one line per event.
"""
from __future__ import annotations

import argparse
import os
import select
import sys
import threading
import time
import tty
from collections import deque

from kilo.protocol import kilo_protocol as P
from kilo.vbody.sim import FW_BUILD, VirtualBody

INSYNC, OK, CRC_EOP = 0x14, 0x10, 0x20
GET_SYNC, LOAD_ADDRESS, PROG_PAGE, READ_PAGE, READ_SIGN, LEAVE = 0x30, 0x55, 0x64, 0x74, 0x75, 0x51
SIGNATURE = bytes([0x1E, 0x95, 0x0F])
APP_BOOT_S = 0.3


class Gate:
    """The virtual body's transport: bytes from the pty when the main loop
    feeds them, writes straight to the pty."""

    def __init__(self, master: int):
        self.master = master
        self.buf = bytearray()
        self.cond = threading.Condition()

    def feed(self, data: bytes) -> None:
        with self.cond:
            self.buf += data
            self.cond.notify_all()

    def read(self, n: int) -> bytes:
        with self.cond:
            if not self.buf:
                self.cond.wait(0.02)
            out = bytes(self.buf[:n])
            del self.buf[:n]
            return out

    def write(self, data: bytes) -> None:
        try:
            os.write(self.master, data)
        except BlockingIOError:
            pass

    def close(self) -> None:
        pass


class Body:
    def __init__(self, master: int, args: argparse.Namespace):
        self.master = master
        self.args = args
        self.build = args.build
        self.mode = "app"
        self.vb: VirtualBody | None = None
        self.gate: Gate | None = None
        self.app_at: float | None = None
        self.cause = P.ResetCause.COLD
        self.flash = bytearray(b"\xff" * 32768)
        self.top = 0
        self.programmed = False
        self.rx = bytearray()
        self.boot_at = 0.0
        self.last_rx = 0.0
        self.addr = 0
        self.start_app(time.monotonic(), delay=0.0)

    def say(self, text: str) -> None:
        print(text, flush=True)

    # ---- the firmware ----------------------------------------------------------------
    def start_app(self, now: float, delay: float = APP_BOOT_S) -> None:
        was = self.build
        if self.programmed:
            self.build = self.args.new_build
        if self.vb is not None or delay > 0:  # a reset, not power-on
            self.cause = P.ResetCause.FLASHED if self.build != was else P.ResetCause.WARM_UNEXPECTED
        self.programmed = False
        self.mode = "app"
        self.app_at = now + delay

    def poll_app(self, now: float) -> None:
        if self.app_at is not None and now >= self.app_at:
            self.app_at = None
            self.gate = Gate(self.master)
            self.vb = VirtualBody(self.gate, fw_build=self.build, reset_cause=int(self.cause)).start()
            self.say(f"firmware: build {self.build:08x} reset {P.ResetCause(self.cause).name}")

    def stop_app(self) -> None:
        if self.vb is not None:
            self.vb.stop()
        self.vb = None
        self.gate = None
        self.app_at = None

    # ---- RESET ----------------------------------------------------------------------
    def reset(self, level: str, now: float) -> None:
        if level == "L":
            self.stop_app()
            self.mode = "reset"
            self.say("RESET low")
        elif level == "H" and self.mode == "reset":
            self.say("RESET released")
            if self.args.no_bootloader:
                self.start_app(now)
            else:
                self.mode = "boot"
                self.boot_at = self.last_rx = now
                self.rx.clear()

    # ---- optiboot -------------------------------------------------------------------
    def listening(self, now: float) -> bool:
        return now >= self.boot_at + self.args.listen_ms / 1000

    def boot_rx(self, data: bytes, now: float) -> None:
        for b in data:
            if self.listening(now) or len(self.rx) < 3:  # the 328P's FIFO and shift register
                self.rx.append(b)
        self.last_rx = now
        self.parse(now)

    def poll_boot(self, now: float) -> None:
        self.parse(now)
        if self.mode == "boot" and now > max(self.last_rx, self.boot_at + self.args.listen_ms / 1000) + 1.0:
            self.say("optiboot: watchdog, starting the firmware")
            self.start_app(now)

    def send(self, data: bytes) -> None:
        os.write(self.master, data)

    def parse(self, now: float) -> None:
        if not self.listening(now):
            return
        while self.mode == "boot" and self.rx:
            cmd = self.rx[0]
            need = 2
            if cmd == LOAD_ADDRESS:
                need = 4
            elif cmd == PROG_PAGE:
                need = 5 + (self.rx[1] << 8 | self.rx[2]) if len(self.rx) >= 3 else 1 << 20
            elif cmd == READ_PAGE:
                need = 5
            if len(self.rx) < need:
                return
            c = bytes(self.rx[:need])
            del self.rx[:need]
            if c[-1] != CRC_EOP:
                self.say(f"optiboot: command {cmd:02x} not ended by CRC_EOP: watchdog reset")
                self.start_app(now)
                return
            if cmd == GET_SYNC:
                self.say("optiboot: GET_SYNC")
                self.send(bytes([INSYNC, OK]))
            elif cmd == LOAD_ADDRESS:
                self.addr = (c[1] | c[2] << 8) * 2
                self.send(bytes([INSYNC, OK]))
            elif cmd == PROG_PAGE:
                n = c[1] << 8 | c[2]
                self.flash[self.addr:self.addr + n] = c[4:4 + n]
                self.top = max(self.top, self.addr + n)
                self.programmed = True
                self.send(bytes([INSYNC, OK]))
            elif cmd == READ_PAGE:
                n = c[1] << 8 | c[2]
                self.send(bytes([INSYNC]) + bytes(self.flash[self.addr:self.addr + n]) + bytes([OK]))
            elif cmd == READ_SIGN:
                self.send(bytes([INSYNC]) + SIGNATURE + bytes([OK]))
            elif cmd == LEAVE:
                self.send(bytes([INSYNC, OK]))
                self.say(f"optiboot: LEAVE_PROGMODE, {self.top // 128} pages programmed")
                if self.args.image_out:
                    with open(self.args.image_out, "wb") as f:
                        f.write(self.flash[:self.top])
                self.start_app(now + 0.016)  # WATCHDOG_16MS
                return
            else:
                self.send(bytes([INSYNC, OK]))

    # ---- the pty --------------------------------------------------------------------
    def rx_pty(self, data: bytes, now: float) -> None:
        if self.mode == "app" and self.gate is not None:
            self.gate.feed(data)
        elif self.mode == "boot":
            self.boot_rx(data, now)
        # reset, or the firmware still starting: lost


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--reset-fifo", required=True)
    ap.add_argument("--build", type=lambda x: int(x, 16), default=FW_BUILD)
    ap.add_argument("--new-build", type=lambda x: int(x, 16), default=None)
    ap.add_argument("--no-bootloader", action="store_true")
    ap.add_argument("--listen-ms", type=float, default=440.0)
    ap.add_argument("--image-out", default=None)
    args = ap.parse_args()
    if args.new_build is None:
        args.new_build = args.build
    if not os.path.exists(args.reset_fifo):
        os.mkfifo(args.reset_fifo)
    # O_RDWR: the pipe never reads as closed while the mind is not attached.
    fifo = os.open(args.reset_fifo, os.O_RDWR | os.O_NONBLOCK)
    master, slave = os.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    os.set_blocking(master, False)
    print(f"fake optiboot body on {os.ttyname(slave)} (RESET {args.reset_fifo})", flush=True)
    body = Body(master, args)
    try:
        while True:
            r, _, _ = select.select([master, fifo], [], [], 0.002)
            now = time.monotonic()
            if fifo in r:
                for level in os.read(fifo, 64).decode("ascii", "replace"):
                    body.reset(level, now)
            if master in r:
                try:
                    data = os.read(master, 4096)
                except (BlockingIOError, OSError):
                    data = b""
                if data:
                    body.rx_pty(data, now)
            if body.mode == "boot":
                body.poll_boot(now)
            elif body.mode == "app":
                body.poll_app(now)
    except KeyboardInterrupt:
        pass
    finally:
        body.stop_app()
    return 0


if __name__ == "__main__":
    sys.exit(main())
