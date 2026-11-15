"""The host channel's secure layer, the laptop's side (protocol/spec.toml
[host] comments; kernel/README.md "Who may talk to Kilo: keys").

On a fresh TCP connection to the mind, ``open_channel()`` sends the clear
hello (magic, version, this laptop's static public key) and Noise message 1,
reads the mind's clear answer and message 2 (Noise_KN_25519_ChaChaPoly_BLAKE2b,
the prologue host.noise_prologue + the hello), and sends the first record
(empty) that proves to the mind that this laptop holds the private key.
The ``Channel`` it returns carries the host frames in records (``len u16 LE
| ciphertext | tag``, at most host.record_max plaintext bytes).

The Noise code is the ``noiseprotocol`` package, on purpose not the mind's
(kernel/mind/net/noise.*): if either strays from the spec, they stop
agreeing. Nothing cryptographic is written here.
"""

from __future__ import annotations

import socket
import threading
import warnings

from kilo.hostkey import KEYS_FILE, HostKey
from kilo.protocol import kilo_protocol as P

TAG_LEN = 16
# A connection never gets near this (2^62 records); the mind closes there too.
NONCE_LIMIT = 1 << 62
MSG1_LEN, MSG2_LEN = 32, 48


class ChannelError(Exception):
    """The secure layer failed (the connection is unusable)."""


class KeyUnknown(ChannelError):
    def __init__(self, key: HostKey):
        self.key = key
        super().__init__(
            f"the mind does not know this laptop's key ({key.fingerprint}, {key.path}). Add `kilo key show`'s line "
            f"to {KEYS_FILE}, rebuild the kernel (make -C kernel) and install it: `kilo update --host` from a "
            "laptop the mind already knows, or the SD card in a card reader")


class HandshakeFailed(ChannelError):
    pass


def _noise():
    try:
        from noise.connection import Keypair, NoiseConnection
    except ImportError:
        raise ChannelError("the Python package noiseprotocol is missing: pip install noiseprotocol "
                           "(the mind package's dependency)") from None
    return Keypair, NoiseConnection


def hello_bytes(key: HostKey) -> bytes:
    return P.HOST_HELLO_MAGIC.encode("ascii") + bytes([P.HOST_VERSION, 0, 0, 0]) + key.public


def _recv_exact(sock: socket.socket, n: int, what: str) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except socket.timeout:
            raise ChannelError(f"no {what} from the mind in time") from None
        except OSError as e:
            raise ChannelError(f"{what}: {e}") from None
        if not chunk:
            if not buf and what == "answer to the hello":
                raise ChannelError("the mind closed the connection without answering the hello: it may be an "
                                   "older mind (host channel version 1, no keys), or not a Kilo mind at all")
            raise ChannelError(f"the mind closed the connection during the {what}")
        buf += chunk
    return bytes(buf)


class Channel:
    """Records both ways over a connected socket, after the handshake.
    send() may be called from several threads (it locks); recv() from one."""

    def __init__(self, sock: socket.socket, noise) -> None:
        self._sock = sock
        self._noise = noise
        self._send_lock = threading.Lock()
        self._sent = 0
        self._received = 0

    def send(self, frames: bytes) -> None:
        """Whole host frames (concatenated) into as few records as fit."""
        with self._send_lock:
            out = bytearray()
            i = 0
            while True:
                j = i
                while j < len(frames):
                    flen = 2 + int.from_bytes(frames[j:j + 2], "little")
                    if j + flen - i > P.HOST_RECORD_MAX:
                        break
                    j += flen
                if j == i and i < len(frames):
                    raise ValueError("a frame longer than a record")
                out += self._seal(bytes(frames[i:j]))
                i = j
                if i >= len(frames):
                    break
            self._sock.sendall(out)

    def _seal(self, plain: bytes) -> bytes:
        if self._sent >= NONCE_LIMIT:
            raise ChannelError("the send nonce is used up: reconnect")
        ct = self._noise.encrypt(plain)
        self._sent += 1
        return len(ct).to_bytes(2, "little") + ct

    def recv(self) -> bytes | None:
        """The next record's plaintext (may be empty); None when the mind
        closed the connection between records. ChannelError on a record
        that does not authenticate, a bad length, or a cut record."""
        head = self._sock.recv(2)
        if not head:
            return None
        if len(head) == 1:
            head += _recv_exact(self._sock, 1, "record")
        n = int.from_bytes(head, "little")
        if not TAG_LEN <= n <= P.HOST_RECORD_MAX + TAG_LEN:
            raise ChannelError(f"a record of {n} bytes from the mind (not {TAG_LEN}..{P.HOST_RECORD_MAX + TAG_LEN})")
        ct = _recv_exact(self._sock, n, "record")
        if self._received >= NONCE_LIMIT:
            raise ChannelError("the receive nonce is used up: reconnect")
        try:
            plain = self._noise.decrypt(ct)
        except Exception:  # noiseprotocol's NoiseInvalidMessage (InvalidTag underneath)
            raise ChannelError("a record from the mind did not authenticate (tampered or out of step)") from None
        self._received += 1
        return bytes(plain)


def open_channel(sock: socket.socket, key: HostKey, timeout: float) -> Channel:
    """The hello and the KN handshake on a connected socket (blocking, at most
    ``timeout`` per read); the first record sent. Raises KeyUnknown when the
    mind does not know the key, HandshakeFailed when message 2 does not
    authenticate, ChannelError otherwise."""
    Keypair, NoiseConnection = _noise()
    hello = hello_bytes(key)
    n = NoiseConnection.from_name(P.HOST_NOISE_PROTOCOL.encode("ascii"))
    n.set_as_initiator()
    n.set_keypair_from_private_bytes(Keypair.STATIC, key.private)
    n.set_prologue(P.HOST_NOISE_PROLOGUE.encode("ascii") + hello)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        n.start_handshake()
    msg1 = bytes(n.write_message())
    assert len(msg1) == MSG1_LEN
    old = sock.gettimeout()
    sock.settimeout(timeout)
    try:
        sock.sendall(hello + msg1)
        reply = _recv_exact(sock, P.HOST_REPLY_LEN, "answer to the hello")
        if reply[:4] != P.HOST_HELLO_MAGIC.encode("ascii"):
            raise ChannelError("the answer to the hello is not a Kilo mind's")
        status = reply[5]
        if status == P.HostHelloStatus.KEY_UNKNOWN:
            raise KeyUnknown(key)
        if status == P.HostHelloStatus.BAD_VERSION:
            raise ChannelError(f"the mind speaks host channel version {reply[4]}, this kilo version "
                               f"{P.HOST_VERSION}: update the one that is behind")
        if status == P.HostHelloStatus.REFUSED:
            raise ChannelError("the mind refused the handshake for now (it has no randomness: see its log); "
                               "try again")
        if status != P.HostHelloStatus.OK:
            raise ChannelError(f"the mind answered the hello with status {status}")
        msg2 = _recv_exact(sock, MSG2_LEN, "handshake")
        try:
            n.read_message(msg2)
        except Exception:  # noiseprotocol raises InvalidTag (cryptography) or its own errors here
            raise HandshakeFailed(
                f"the handshake failed: the mind's answer did not authenticate. This private key ({key.path}) "
                f"is not the one whose public key ({key.fingerprint}) the mind knows, or something on the "
                "network changed the handshake") from None
        if not n.handshake_finished:
            raise HandshakeFailed("the handshake did not finish")
        ch = Channel(sock, n)
        sock.sendall(ch._seal(b""))  # the first record: now the mind counts this laptop as a client
    except OSError as e:
        raise ChannelError(f"handshake: {e}") from None
    finally:
        sock.settimeout(old)
    return ch
