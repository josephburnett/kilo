"""The mind's end of the host channel's secure layer for the Python tests'
fake minds (test_hostlink.py, test_flash_image.py), built from the Python
noiseprotocol package as kilo.hostsecure is, but on the responder's side:
the clear hello and its key check, the KN handshake, records both ways.
The real responder is kernel/mind/net/host_secure.cpp (tested in
kernel/host/tests/test_host.cpp, and against kilo.hostsecure in
kernel/host/test.sh)."""

from __future__ import annotations

import socket
import warnings

from noise.connection import Keypair, NoiseConnection

from kilo.protocol import kilo_protocol as P

TAG = 16


def _exact(c: socket.socket, n: int) -> bytes | None:
    buf = b""
    while len(buf) < n:
        chunk = c.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


class Responder:
    """One accepted connection. handshake() first; then recv()/send()."""

    def __init__(self, c: socket.socket, keys: list[bytes]):
        self.c = c
        self.keys = keys
        self.noise: NoiseConnection | None = None
        self.client_key: bytes | None = None
        self.records = 0  # records received (the first, empty one included)

    def _answer(self, status: int) -> None:
        self.c.sendall(P.HOST_HELLO_MAGIC.encode() + bytes([P.HOST_VERSION, status, 0, 0]))

    def handshake(self, status: int = P.HostHelloStatus.OK) -> bool:
        """Read the hello and message 1, answer (status: force a refusal),
        then wait for the client's first record. False: refused or failed."""
        first = _exact(self.c, P.HOST_HELLO_LEN + 32)
        if first is None or first[:4] != P.HOST_HELLO_MAGIC.encode() or first[4] != P.HOST_VERSION:
            return False
        hello, msg1 = first[:P.HOST_HELLO_LEN], first[P.HOST_HELLO_LEN:]
        key = hello[8:40]
        if status == P.HostHelloStatus.OK and key not in self.keys:
            status = P.HostHelloStatus.KEY_UNKNOWN
        if status != P.HostHelloStatus.OK:
            self._answer(status)
            return False
        n = NoiseConnection.from_name(P.HOST_NOISE_PROTOCOL.encode())
        n.set_as_responder()
        n.set_keypair_from_public_bytes(Keypair.REMOTE_STATIC, key)
        n.set_prologue(P.HOST_NOISE_PROLOGUE.encode() + hello)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            n.start_handshake()
        n.read_message(msg1)
        msg2 = bytes(n.write_message())
        self._answer(P.HostHelloStatus.OK)
        self.c.sendall(msg2)
        self.noise, self.client_key = n, key
        return self.recv() is not None  # the first record proves the client holds the key

    def recv(self) -> bytes | None:
        """A record's plaintext; None on close or a record that does not open."""
        head = _exact(self.c, 2)
        if head is None:
            return None
        n = int.from_bytes(head, "little")
        if not TAG <= n <= P.HOST_RECORD_MAX + TAG:
            return None  # as the mind: a bad length closes the connection
        body = _exact(self.c, n)
        if body is None:
            return None
        try:
            plain = bytes(self.noise.decrypt(body))
        except Exception:
            return None
        self.records += 1
        return plain

    def send(self, plain: bytes) -> None:
        """Records holding plain (cut at the record limit)."""
        for i in range(0, max(1, len(plain)), P.HOST_RECORD_MAX):
            ct = self.noise.encrypt(bytes(plain[i:i + P.HOST_RECORD_MAX]))
            self.c.sendall(len(ct).to_bytes(2, "little") + ct)

    def send_forged(self) -> None:
        """A record that does not authenticate (made with the wrong key)."""
        self.c.sendall((TAG + 6).to_bytes(2, "little") + bytes(TAG + 6))
