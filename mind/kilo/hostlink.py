"""The laptop's end of the host channel (docs/fast-boot.md §5, phase P2).

The mind (the Circle kernel on the Pi, or ``kilo-mind-host --listen`` on the
laptop, kernel/mind/net/host_server.h) serves one TCP port; ``kilo --host
<ip[:port]>`` talks to it through ``HostLink``. The messages and framing are
in protocol/spec.toml ``[host]`` and ``[[host_messages]]``; the dataclasses
(``P.HostStatus``, ...) and the framing (``C.build_host_frame``,
``C.HostStreamDecoder``) are generated and vendored like the body link's.

    hl = HostLink.connect("169.254.75.73")      # port 5300 unless given
    st = hl.status()                            # P.HostStatus
    hl.take_control(keepalive=True)             # then POSTURE; a background thread sends KEEPALIVE
    hl.command(P.HostPosture(posture=P.Posture.STAND))
    hl.subscribe_telem(10, on_telem)            # P.HostTelem at 10 Hz
    hl.subscribe_log(on_line, tail=20)          # P.HostLogLine: the last 20, then live
    hl.flash(image, manifest, on_status)        # P4: flash the body through the mind (controller)
    hl.reboot()                                 # restart the mind (controller): OK, then it goes away
    hl.close()                                  # the mind latches REST if this was the controller

Every command gets exactly one reply carrying its seq (STATUS for
STATUS_REQ, RESULT otherwise); ``command()`` raises ``HostRefused`` for a
RESULT whose code is not OK. TELEM, LOG_LINE, BRIDGE_RX and FLASH_STATUS
arrive on the reader thread and go to the handlers.

``BridgeTransport`` is a kilo.link transport over the raw bridge, so the
whole Python link stack (``SerialLink``, ``BodyClient``) can talk to the body
through the mind while the mind's own link is paused.
"""

from __future__ import annotations

import socket
import threading
import time
from typing import Any, Callable

from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P

DEFAULT_PORT = P.HOST_PORT
CONNECT_TIMEOUT_S = 3.0
REPLY_TIMEOUT_S = 3.0
# A whole flash: two attempts of reset, sync and 252 pages programmed and
# read back at 115200 (about 10 s each on a real 328P), then up to 8 s for
# the new firmware's HELLO. Far more than that is a mind that stopped.
FLASH_TIMEOUT_S = 60.0
# After REBOOT's OK: how long `kilo reboot` waits for the connection to drop (wait_closed()).
REBOOT_CLOSE_WAIT_S = 2.0


class HostError(Exception):
    """The host channel failed: no connection, the stream out of step, no reply."""


class HostRefused(HostError):
    """The mind answered a command with RESULT code != OK."""

    def __init__(self, request: int, code: int, text: str):
        self.request = request
        self.code = code
        self.text = text
        try:
            name = P.HostMsgId(request).name
        except ValueError:
            name = f"type {request:#04x}"
        try:
            code_name = P.HostResultCode(code).name
        except ValueError:
            code_name = str(code)
        self.code_name = code_name
        super().__init__(f"{name}: {code_name}" + (f" ({text})" if text else ""))


def parse_address(addr: str) -> tuple[str, int]:
    """'ip', 'ip:port' or 'host:port' -> (host, port); the port defaults to host.port (5300)."""
    host, sep, port = addr.rpartition(":")
    if not sep:
        return addr, DEFAULT_PORT
    if not host or not port.isdigit() or not 0 < int(port) < 65536:
        raise ValueError(f"bad --host {addr!r}: want IP or IP:PORT")
    return host, int(port)


class _Pending:
    def __init__(self) -> None:
        self.done = threading.Event()
        self.reply: Any = None


class HostLink:
    """One connection to the mind's host channel. Thread-safe for requests."""

    def __init__(self, sock: socket.socket, address: str = ""):
        self.address = address
        self._sock = sock
        self._dec = C.HostStreamDecoder()
        self._send_lock = threading.Lock()
        self._lock = threading.Lock()
        self._seq = 0
        self._pending: dict[int, _Pending] = {}
        self._closed = threading.Event()
        self.error: str | None = None  # why the reader stopped
        self.on_telem: Callable[[Any], None] | None = None
        self.on_log: Callable[[Any], None] | None = None
        self.on_bridge: Callable[[bytes], None] | None = None
        self.on_flash: Callable[[Any], None] | None = None
        self._flash_last: Any = None
        self._flash_cond = threading.Condition()
        self._keepalive: threading.Thread | None = None
        self._reader = threading.Thread(target=self._read_loop, name="kilo-hostlink", daemon=True)
        self._reader.start()

    @classmethod
    def connect(cls, addr: str, timeout: float = CONNECT_TIMEOUT_S) -> "HostLink":
        host, port = parse_address(addr)
        try:
            sock = socket.create_connection((host, port), timeout=timeout)
        except OSError as e:
            raise HostError(f"cannot reach the mind at {host}:{port}: {e}") from None
        sock.settimeout(None)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return cls(sock, f"{host}:{port}")

    # ---- requests --------------------------------------------------------------------
    def request(self, msg: Any, timeout: float = REPLY_TIMEOUT_S) -> Any:
        """Send a command and return its reply (P.HostStatus, P.HostResult, ...)."""
        if msg.KIND != P.Kind.COMMAND:
            raise ValueError(f"{msg.NAME} is not a command")
        with self._lock:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFF
            pend = self._pending[seq] = _Pending()
        try:
            self._send(msg, seq)
            if not pend.done.wait(timeout):
                raise HostError(self.error or f"no reply to {msg.NAME} within {timeout:g} s")
            if pend.reply is None:
                raise HostError(self.error or "connection closed")
            return pend.reply
        finally:
            with self._lock:
                self._pending.pop(seq, None)

    def command(self, msg: Any, timeout: float = REPLY_TIMEOUT_S) -> Any:
        """request(), raising HostRefused unless the reply is RESULT OK (or the query's own reply)."""
        r = self.request(msg, timeout)
        if isinstance(r, P.HostResult) and r.code != P.HostResultCode.OK:
            raise HostRefused(r.request, r.code, r.text.decode("ascii", "replace"))
        return r

    def send_stream(self, msg: Any) -> None:
        """KEEPALIVE, BRIDGE_TX: one-way, never answered."""
        self._send(msg, 0)

    def status(self) -> Any:
        return self.request(P.HostStatusReq())

    def take_control(self, keepalive: bool = True) -> None:
        """Become the controller. With keepalive the mind drops control (and
        latches REST) after host.keepalive_timeout_ms of silence; a thread
        here sends KEEPALIVE every host.keepalive_send_ms until close()."""
        self.command(P.HostControl(take=1, keepalive=int(keepalive)))
        if keepalive and self._keepalive is None:
            self._keepalive = threading.Thread(target=self._keepalive_loop, name="kilo-hostlink-ka", daemon=True)
            self._keepalive.start()

    def release_control(self) -> None:
        self.command(P.HostControl(take=0, keepalive=0))

    def subscribe_telem(self, hz: float, handler: Callable[[Any], None] | None = None) -> None:
        if handler is not None:
            self.on_telem = handler
        self.command(P.HostTelemSub(hz=max(0, min(P.HOST_TELEM_MAX_HZ, int(round(hz))))))

    def subscribe_log(self, handler: Callable[[Any], None] | None = None, tail: int = 20,
                      min_level: int = P.LogLevel.DEBUG) -> None:
        if handler is not None:
            self.on_log = handler
        self.command(P.HostLogSub(enable=1, tail=max(0, min(255, tail)), min_level=int(min_level)))

    def flash(self, image: bytes, build: int, crc32: int, persist: bool = False,
              on_status: Callable[[Any], None] | None = None, timeout: float = FLASH_TIMEOUT_S) -> Any:
        """Flash the body through the mind (docs/fast-boot.md §6): take
        control (keepalive), FLASH_BEGIN, the image in FLASH_DATA chunks,
        FLASH_COMMIT, then wait for FLASH_STATUS DONE or FAILED and return
        that P.HostFlashStatus. ``on_status`` sees every FLASH_STATUS. Raises
        HostRefused if the mind refuses (IMAGE_BAD: the mind's own size or
        CRC check failed; the FAILED status that says why went to
        ``on_status`` first), HostError on a lost connection or no end in
        ``timeout``. Control stays taken: the caller releases it."""
        with self._flash_cond:
            self._flash_last = None
        self.on_flash = on_status
        self.take_control(keepalive=True)
        flags = int(P.FlashFlags.PERSIST) if persist else 0
        self.command(P.HostFlashBegin(build=build, image_size=len(image), crc32=crc32, flags=flags))
        step = P.HOST_FLASH_CHUNK_MAX
        for off in range(0, len(image), step):
            self.send_stream(P.HostFlashData(offset=off, data=bytes(image[off:off + step])))
        self.command(P.HostFlashCommit())  # OK: flashing has started
        end = time.monotonic() + timeout
        with self._flash_cond:
            while True:
                last = self._flash_last
                if last is not None and last.phase in (P.FlashPhase.DONE, P.FlashPhase.FAILED):
                    return last
                if self._closed.is_set():
                    raise HostError(self.error or "connection closed while flashing")
                left = end - time.monotonic()
                if left <= 0:
                    raise HostError(f"no end of the flash within {timeout:g} s")
                self._flash_cond.wait(min(left, 0.2))

    def reboot(self) -> Any:
        """REBOOT (controller only: take control first): the RESULT OK, or
        HostRefused. The mind then restarts; see wait_closed()."""
        return self.command(P.HostReboot())

    def wait_closed(self, timeout: float = REBOOT_CLOSE_WAIT_S) -> bool:
        """Wait up to ``timeout`` for the connection to drop; whether it did.
        After REBOOT, kilo-mind-host closes it as it exits, while a Pi that
        reboots just goes silent (no FIN): no close is no error there."""
        return self._closed.wait(timeout)

    # ---- plumbing --------------------------------------------------------------------
    def _send(self, msg: Any, seq: int) -> None:
        data = C.encode_host_message(msg, seq)
        with self._send_lock:
            if self._closed.is_set():
                raise HostError(self.error or "connection closed")
            try:
                self._sock.sendall(data)
            except OSError as e:
                raise HostError(f"send: {e}") from None

    def _keepalive_loop(self) -> None:
        period = P.HOST_KEEPALIVE_SEND_MS / 1000.0
        while not self._closed.wait(period):
            try:
                self.send_stream(P.HostKeepalive())
            except HostError:
                return

    def _read_loop(self) -> None:
        try:
            while True:
                data = self._sock.recv(4096)
                if not data:
                    self.error = "the mind closed the connection"
                    break
                for f in self._dec.feed(data):
                    self._dispatch(f)
        except C.HostFrameError as e:
            self.error = f"host stream out of step: {e}"
        except OSError as e:
            if not self._closed.is_set():
                self.error = f"receive: {e}"
        finally:
            self._closed.set()
            with self._lock:
                for p in self._pending.values():
                    p.done.set()
            with self._flash_cond:
                self._flash_cond.notify_all()

    def _dispatch(self, f: Any) -> None:
        try:
            msg = P.decode_host_payload(f.type, f.payload)
        except (KeyError, P.PayloadError):
            return  # a newer mind's message: ignore
        if msg.KIND == P.Kind.EVENT:  # a reply
            with self._lock:
                pend = self._pending.get(f.seq)
            if pend is not None:
                pend.reply = msg
                pend.done.set()
        elif isinstance(msg, P.HostTelem):
            if self.on_telem is not None:
                self.on_telem(msg)
        elif isinstance(msg, P.HostLogLine):
            if self.on_log is not None:
                self.on_log(msg)
        elif isinstance(msg, P.HostBridgeRx):
            if self.on_bridge is not None:
                self.on_bridge(msg.data)
        elif isinstance(msg, P.HostFlashStatus):
            if self.on_flash is not None:
                self.on_flash(msg)
            with self._flash_cond:
                self._flash_last = msg
                self._flash_cond.notify_all()

    @property
    def closed(self) -> bool:
        return self._closed.is_set()

    def close(self) -> None:
        if not self._closed.is_set():
            self._closed.set()
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self._sock.close()
        self._reader.join(timeout=1.0)

    def __enter__(self) -> "HostLink":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()


class BridgeTransport:
    """A kilo.link transport (kilo/link/transport.py) over the raw bridge:
    BRIDGE_OPEN on creation (it takes control, with keepalive), BRIDGE_CLOSE
    and the hand-back of control on close(). ``SerialLink(lambda:
    BridgeTransport(hl))`` then runs the Python link stack against the body
    through the mind."""

    def __init__(self, hl: HostLink):
        self.hl = hl
        self._buf = bytearray()
        self._cond = threading.Condition()
        hl.on_bridge = self._on_rx
        hl.take_control(keepalive=True)
        hl.command(P.HostBridgeOpen())

    def _on_rx(self, data: bytes) -> None:
        with self._cond:
            self._buf += data
            self._cond.notify_all()

    def read(self, n: int) -> bytes:
        from kilo.link.transport import READ_TIMEOUT_S

        with self._cond:
            if not self._buf:
                if self.hl.closed:
                    raise OSError(self.hl.error or "bridge closed")
                self._cond.wait(READ_TIMEOUT_S)
            out = bytes(self._buf[:n])
            del self._buf[:n]
            return out

    def write(self, data: bytes) -> None:
        for i in range(0, len(data), P.HOST_BRIDGE_CHUNK_MAX):
            try:
                self.hl.send_stream(P.HostBridgeTx(data=bytes(data[i:i + P.HOST_BRIDGE_CHUNK_MAX])))
            except HostError as e:
                raise OSError(str(e)) from None

    def close(self) -> None:
        if self.hl.closed:
            return
        try:
            self.hl.command(P.HostBridgeClose())
        except HostError:
            pass

