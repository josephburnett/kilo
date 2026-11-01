#!/usr/bin/env python3
"""Send one file to a TFTP server (RFC 1350 write request, octet mode).

    tftp_put.py HOST FILE [--name REMOTE] [--port 69] [--timeout 1.0] [--retries 5]

Used by netboot.sh when no tftp client is installed. 512-byte blocks, no
option extensions: the Circle netboot loader (sample 38) speaks plain
RFC 1350 and accepts only names "kernel*.img". Each packet is resent after
--timeout seconds without an answer, at most --retries times. Exit status 0
when the last block is acknowledged.
"""

import argparse
import socket
import struct
import sys
from pathlib import Path

OP_WRQ, OP_DATA, OP_ACK, OP_ERROR = 2, 3, 4, 5
BLOCK = 512


class TftpError(Exception):
    pass


def put(host: str, data: bytes, remote: str, port: int = 69,
        timeout: float = 1.0, retries: int = 5, log=lambda msg: None) -> int:
    """Send data as remote; return the number of DATA packets sent (with resends)."""
    addr = socket.getaddrinfo(host, port, socket.AF_INET, socket.SOCK_DGRAM)[0][4]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    sent = 0
    try:
        packet = struct.pack("!H", OP_WRQ) + remote.encode() + b"\0octet\0"
        dest = addr            # the server answers from a new port (its TID)
        tid = None
        block = 0              # the block we wait to see acknowledged
        nblocks = len(data) // BLOCK + 1   # a full last block needs an empty one after it
        tries = 0
        sock.sendto(packet, dest)
        while True:
            try:
                reply, src = sock.recvfrom(4 + BLOCK)
            except socket.timeout:
                tries += 1
                if tries > retries:
                    raise TftpError(f"no answer for block {block} after {retries} retries")
                log(f"timeout, resending block {block}")
                sock.sendto(packet, dest)
                if packet[1] == OP_DATA:
                    sent += 1
                continue
            if src[0] != addr[0] or (tid is not None and src != tid):
                # RFC 1350 §4: answer a stranger with error 5 and carry on.
                sock.sendto(struct.pack("!HH", OP_ERROR, 5) + b"unknown transfer ID\0", src)
                continue
            if len(reply) < 4:
                continue
            op, num = struct.unpack("!HH", reply[:4])
            if op == OP_ERROR:
                msg = reply[4:].split(b"\0", 1)[0].decode(errors="replace")
                raise TftpError(f"server error {num}: {msg}")
            if op != OP_ACK or num != block % 65536:
                continue       # duplicate or stale ACK: wait, do not resend (Sorcerer's Apprentice)
            if tid is None:
                tid = src
                dest = src
            if block == nblocks:
                return sent
            block += 1
            chunk = data[(block - 1) * BLOCK: block * BLOCK]
            packet = struct.pack("!HH", OP_DATA, block % 65536) + chunk
            tries = 0
            sock.sendto(packet, dest)
            sent += 1
    finally:
        sock.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("host")
    ap.add_argument("file", type=Path)
    ap.add_argument("--name", help="remote file name (default: the local name)")
    ap.add_argument("--port", type=int, default=69)
    ap.add_argument("--timeout", type=float, default=1.0)
    ap.add_argument("--retries", type=int, default=5)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)
    data = args.file.read_bytes()
    log = (lambda msg: print(msg, file=sys.stderr)) if args.verbose else (lambda msg: None)
    try:
        put(args.host, data, args.name or args.file.name, args.port,
            args.timeout, args.retries, log)
    except (TftpError, OSError) as e:
        print(f"tftp_put: {e}", file=sys.stderr)
        return 1
    print(f"sent {len(data)} bytes to {args.host}:{args.port} as {args.name or args.file.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
