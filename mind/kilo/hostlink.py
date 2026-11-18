"""The laptop's end of the host channel (docs/fast-boot.md §5, phase P2).

The mind (the Circle kernel on the Pi, or ``kilo-mind-host --listen`` on the
laptop, kernel/mind/net/host_server.h) serves one TCP port; ``kilo --host
<ip[:port]>`` talks to it through ``HostLink``. The messages and framing are
in protocol/spec.toml ``[host]`` and ``[[host_messages]]``; the dataclasses
(``P.HostStatus``, ...) and the framing (``C.build_host_frame``,
``C.HostStreamDecoder``) are generated and vendored like the body link's.

    hl = HostLink.connect("169.254.75.73")      # port 5300 unless given; this laptop's key (kilo.hostkey)
    st = hl.status()                            # P.HostStatus
    hl.take_control(keepalive=True)             # then POSTURE; a background thread sends KEEPALIVE
    hl.command(P.HostPosture(posture=P.Posture.STAND))
    hl.subscribe_telem(10, on_telem)            # P.HostTelem at 10 Hz
    hl.subscribe_log(on_line, tail=20)          # P.HostLogLine: the last 20, then live
    hl.flash(image, manifest, on_status)        # P4: flash the body through the mind (controller)
    hl.update(kernel_image, progress=...)       # write a new mind kernel onto its SD card (controller)
    hl.reboot()                                 # restart the mind (controller): OK, then it goes away
    hl.walks()                                  # [P.HostWalkInfo]: the walks the mind recorded (any client)
    hl.fetch_walk(3, progress=...)              # run 3's flight-recorder blocks, bytes (WALK_FETCH, WALK_DATA)
    hl.rec_status()                             # P.HostRecStatus: the flight recorder and the card saver
    hl.rec_fetch(from_ms, to_ms, boot=0)        # the blocks of a time range (REC_FETCH), bytes
    hl.rec_fetch(0, 0xFFFFFFFF, boot=6, what=P.RecFetchWhat.LOG)   # boot 6's saved log, text bytes
    hl.metrics()                                # [str]: the session metrics lines (METRICS_REQ)
    hl.guard(mode)                              # the distance guard: 0 off, 1 on, 2 ask (GUARD_STATUS)
    hl.bench(count=32)                          # [P.HostBenchResult]: the SD card's write times (controller)
    hl.rec_save(1); hl.rec_delete(boot)         # the card saver on; earlier boots' recordings off the card
    hl.close()                                  # the mind latches REST if this was the controller

Every command gets exactly one reply carrying its seq (STATUS for
STATUS_REQ, WALK_INFO for WALK_LIST, RESULT otherwise); ``command()`` raises
``HostRefused`` for a RESULT whose code is not OK. TELEM, LOG_LINE,
BRIDGE_RX, FLASH_STATUS, UPDATE_STATUS and WALK_DATA arrive on the reader
thread and go to the handlers.

Every byte goes through the secure layer (kilo.hostsecure): the hello
with this laptop's public key, the Noise KN handshake, then records. A
mind that does not know the key raises ``HostKeyRefused``; no key file
(``kilo key init``), or one others can read, raises ``HostKeyProblem``.

``BridgeTransport`` is a kilo.link transport over the raw bridge, so the
whole Python link stack (``SerialLink``, ``BodyClient``) can talk to the body
through the mind while the mind's own link is paused.
"""

from __future__ import annotations

import socket
import threading
import time
from typing import Any, Callable

from kilo import hostkey, hostsecure
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
# A whole kernel update after UPDATE_COMMIT: up to host.update_max_image (4 MB)
# written to the SD card and read back. A 512 KB kernel takes seconds.
UPDATE_TIMEOUT_S = 120.0
# A walk download: no WALK_DATA for this long is a mind that stopped sending.
# (A fetch from the SD card waits for a moment the body is still: longer.)
WALK_IDLE_TIMEOUT_S = 5.0
REC_IDLE_TIMEOUT_S = 30.0
# The SD card bench: a few hundred writes, a pass each, some slow.
BENCH_TIMEOUT_S = 120.0


def has_kernel_marker(image: bytes) -> bool:
    """Whether image holds host.update_marker, the mind kernel's identity
    (kernel/src/kernel.cpp): what the mind checks before it installs an
    image without FORCE."""
    return P.HOST_UPDATE_MARKER.encode("ascii") in image


class HostError(Exception):
    """The host channel failed: no connection, the stream out of step, no reply."""


class HostKeyProblem(HostError):
    """This laptop's key file is missing or unusable; the message says what to do."""


class HostKeyRefused(HostError):
    """The mind does not know this laptop's key, or the handshake with it failed."""


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

    def __init__(self, sock: socket.socket, channel: hostsecure.Channel, address: str = "",
                 key: hostkey.HostKey | None = None):
        self.address = address
        self.key = key
        self._sock = sock
        self._channel = channel
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
        self.on_update: Callable[[Any], None] | None = None
        self._walk: dict[str, Any] | None = None  # the download fetch_walk() runs
        self.walk_store: Any = None  # the last WALK_INFO with found 0: the store's state
        self._flash_last: Any = None
        self._update_last: Any = None
        self._bench: list[Any] | None = None
        self._flash_cond = threading.Condition()
        self._keepalive: threading.Thread | None = None
        self._reader = threading.Thread(target=self._read_loop, name="kilo-hostlink", daemon=True)
        self._reader.start()

    @classmethod
    def connect(cls, addr: str, timeout: float = CONNECT_TIMEOUT_S,
                key: hostkey.HostKey | str | None = None) -> "HostLink":
        """Connect and run the handshake with ``key`` (a HostKey, or a key
        file's path; default: kilo.hostkey.key_path(), i.e. $KILO_HOST_KEY or
        ~/.config/kilo/host_key)."""
        host, port = parse_address(addr)
        if not isinstance(key, hostkey.HostKey):
            try:
                key = hostkey.load(key)
            except hostkey.HostKeyError as e:
                raise HostKeyProblem(str(e)) from None
        try:
            sock = socket.create_connection((host, port), timeout=timeout)
        except OSError as e:
            raise HostError(f"cannot reach the mind at {host}:{port}: {e}") from None
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            channel = hostsecure.open_channel(sock, key, timeout)
        except (hostsecure.KeyUnknown, hostsecure.HandshakeFailed) as e:
            sock.close()
            raise HostKeyRefused(str(e)) from None
        except hostsecure.ChannelError as e:
            sock.close()
            raise HostError(f"{host}:{port}: {e}") from None
        sock.settimeout(None)
        return cls(sock, channel, f"{host}:{port}", key)

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

    def send_streams(self, msgs: list[Any]) -> None:
        """Several stream messages at once (FLASH_DATA, UPDATE_DATA): packed
        into as few records as fit, in order."""
        self._send_raw(b"".join(C.encode_host_message(m, 0) for m in msgs))

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
        self.send_streams([P.HostFlashData(offset=off, data=bytes(image[off:off + step]))
                           for off in range(0, len(image), step)])
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

    def update(self, image: bytes, force: bool = False, progress: Callable[[Any], None] | None = None,
               timeout: float = UPDATE_TIMEOUT_S) -> Any:
        """Write a new mind kernel onto the mind's SD card: take control
        (keepalive), UPDATE_BEGIN (size, zlib.crc32, FORCE), the image in
        UPDATE_DATA chunks, UPDATE_COMMIT, then wait for UPDATE_STATUS DONE
        or FAILED and return that P.HostUpdateStatus. ``progress`` sees every
        UPDATE_STATUS. After DONE the mind reboots into the new kernel (the
        connection drops). Raises HostRefused if the mind refuses (IMAGE_BAD:
        its size, CRC or marker check failed, and the FAILED status that
        says why went to ``progress`` first; UNSUPPORTED: no kernel store),
        HostError on a lost connection or no end within ``timeout``."""
        import zlib

        with self._flash_cond:
            self._update_last = None
        self.on_update = progress
        self.take_control(keepalive=True)
        flags = int(P.UpdateFlags.FORCE) if force else 0
        self.command(P.HostUpdateBegin(image_size=len(image), crc32=zlib.crc32(image), flags=flags))
        step = P.HOST_UPDATE_CHUNK_MAX
        batch = 64  # frames per send: about 13 KB, a dozen records
        for first in range(0, len(image), step * batch):
            self.send_streams([P.HostUpdateData(offset=off, data=bytes(image[off:off + step]))
                               for off in range(first, min(len(image), first + step * batch), step)])
        self.command(P.HostUpdateCommit(), timeout=max(REPLY_TIMEOUT_S, 10.0))  # OK: writing has started
        end = time.monotonic() + timeout
        with self._flash_cond:
            while True:
                last = self._update_last
                if last is not None and last.phase in (P.UpdatePhase.DONE, P.UpdatePhase.FAILED):
                    return last
                if self._closed.is_set():
                    raise HostError(self.error or "connection closed while updating")
                left = end - time.monotonic()
                if left <= 0:
                    raise HostError(f"no end of the update within {timeout:g} s")
                self._flash_cond.wait(min(left, 0.2))

    def walks(self) -> list[Any]:
        """Every walk the mind has recorded, oldest first (P.HostWalkInfo with
        found 1; each also carries the store's state). WALK_LIST, one walk a
        request; the last answer (found 0) is kept as ``walk_store``. A mind
        without a recorder: HostRefused (UNSUPPORTED, or UNKNOWN_TYPE from an
        older mind)."""
        out, after = [], 0
        while True:
            r = self.command(P.HostWalkList(after=after))
            if not isinstance(r, P.HostWalkInfo):
                raise HostError(f"WALK_LIST: unexpected reply {type(r).__name__}")
            if not r.found:
                self.walk_store = r
                return out
            out.append(r)
            after = r.id

    def fetch_walk(self, walk_id: int, progress: Callable[[int, int], None] | None = None,
                   idle_timeout: float = WALK_IDLE_TIMEOUT_S) -> bytes:
        """Run ``walk_id``'s recording: the flight-recorder blocks around it
        (kilo.logs.flight; a mind of host.walk_format 1 sends a walk's records;
        kilo.logs.walkrec reads both). ``progress(received, total)`` after
        each chunk (total 0: not known, from the card). Raises HostRefused
        (BAD_ARG: no such run, or still recording), HostError (the mind
        dropped it meanwhile: GONE; a gap; no data for ``idle_timeout`` s; the
        connection lost)."""
        return self._fetch(P.HostWalkFetch(id=walk_id, offset=0), walk_id, progress, idle_timeout)

    def rec_fetch(self, from_ms: int, to_ms: int = 0xFFFFFFFF, boot: int = 0, what: int = P.RecFetchWhat.BLOCKS,
                  progress: Callable[[int, int], None] | None = None,
                  idle_timeout: float = REC_IDLE_TIMEOUT_S) -> bytes:
        """REC_FETCH: the flight recorder's blocks from from_ms to to_ms (the
        mind's clock, REC_STATUS.clock_ms; boot 0 = this boot), or (what LOG)
        an earlier boot's saved log as text. Raises HostRefused (BAD_ARG:
        nothing there; UNKNOWN_TYPE: an older mind), HostError as fetch_walk."""
        return self._fetch(P.HostRecFetch(boot=boot, from_ms=max(0, int(from_ms)),
                                          to_ms=min(0xFFFFFFFF, int(to_ms)), what=int(what)),
                           0, progress, idle_timeout)

    def rec_status(self) -> Any:
        """P.HostRecStatus (HostRefused UNKNOWN_TYPE from a mind without a flight recorder)."""
        r = self.command(P.HostRecStatusReq())
        if not isinstance(r, P.HostRecStatus):
            raise HostError(f"REC_STATUS_REQ: unexpected reply {type(r).__name__}")
        return r

    def metrics(self) -> list[str]:
        """The session metrics, one ``group: key=value ...`` line each."""
        out: list[str] = []
        i = 0
        while i < 256:
            r = self.command(P.HostMetricsReq(index=i))
            if not isinstance(r, P.HostMetrics):
                raise HostError(f"METRICS_REQ: unexpected reply {type(r).__name__}")
            if i >= r.count:
                break
            out.append(bytes(r.text).decode("ascii", "replace"))
            i += 1
        return out

    def guard(self, mode: int = 2) -> Any:
        """GUARD: 0 off, 1 on, 2 only ask. P.HostGuardStatus (HostRefused UNKNOWN_TYPE from a mind without it)."""
        r = self.command(P.HostGuard(mode=int(mode)))
        if not isinstance(r, P.HostGuardStatus):
            raise HostError(f"GUARD: unexpected reply {type(r).__name__}")
        return r

    def cal(self, op: int = 0, joint: int = 0, offset_a16: int = 0) -> Any:
        """CAL (P.CalOp GET, SET, SAVE, REVERT), relayed to the body: P.HostCalStatus with
        the 16 offsets after it (HostRefused: the mind's or the body's refusal)."""
        # SAVE waits for the body's EEPROM_SAVED (host.cal_save_wait_ms) before answering
        wait = REPLY_TIMEOUT_S + (P.HOST_CAL_SAVE_WAIT_MS / 1000.0 if op == P.CalOp.SAVE else 0.0)
        r = self.command(P.HostCal(op=int(op), joint=int(joint), offset_a16=int(offset_a16)), timeout=wait)
        if not isinstance(r, P.HostCalStatus):
            raise HostError(f"CAL: unexpected reply {type(r).__name__}")
        return r

    def bench(self, count: int = 32, timeout: float = BENCH_TIMEOUT_S,
              on_result: Callable[[Any], None] | None = None) -> list[Any]:
        """REC_BENCH (controller, the body limp): BENCH_RESULT per write
        size, the last with write_bytes 0. on_result(each) as they come."""
        with self._flash_cond:
            self._bench = []
        try:
            self.command(P.HostRecBench(count=max(1, min(256, int(count)))))
            end = time.monotonic() + timeout
            seen = 0
            while True:
                with self._flash_cond:
                    got = list(self._bench or [])
                    if len(got) <= seen and not (got and got[-1].write_bytes == 0):
                        if self._closed.is_set():
                            raise HostError(self.error or "connection closed during the bench")
                        left = end - time.monotonic()
                        if left <= 0:
                            raise HostError(f"no end of the bench within {timeout:g} s")
                        self._flash_cond.wait(min(left, 0.2))
                        continue
                for r in got[seen:]:
                    if on_result is not None:
                        on_result(r)
                seen = len(got)
                if got and got[-1].write_bytes == 0:
                    return got
        finally:
            with self._flash_cond:
                self._bench = None

    def rec_save(self, mode: int) -> str:
        """REC_SAVE (controller): 0 off, 1 on (remembered on the card), 2 save now. The RESULT's text."""
        r = self.command(P.HostRecSave(mode=int(mode)))
        return r.text.decode("ascii", "replace") if isinstance(r, P.HostResult) else ""

    def rec_delete(self, boot: int) -> str:
        """REC_DELETE (controller): an earlier boot's recordings off the card (0xFFFFFFFF: all earlier boots')."""
        r = self.command(P.HostRecDelete(boot=int(boot)))
        return r.text.decode("ascii", "replace") if isinstance(r, P.HostResult) else ""

    def _fetch(self, msg: Any, walk_id: int, progress: Callable[[int, int], None] | None,
               idle_timeout: float) -> bytes:
        st: dict[str, Any] = {"id": walk_id, "buf": bytearray(), "total": None, "end": None, "error": None,
                              "progress": progress, "last": time.monotonic()}
        with self._flash_cond:
            self._walk = st
        try:
            self.command(msg)
            with self._flash_cond:
                while st["end"] is None:
                    if self._closed.is_set():
                        raise HostError(self.error or "connection closed during the download")
                    if time.monotonic() - st["last"] > idle_timeout:
                        raise HostError(f"walk {walk_id}: no data for {idle_timeout:g} s "
                                        f"({len(st['buf'])} of {st['total']} bytes)")
                    self._flash_cond.wait(0.2)
            if st["error"]:
                raise HostError(st["error"])
            return bytes(st["buf"])
        finally:
            with self._flash_cond:
                self._walk = None

    def _on_walk_data(self, m: Any) -> None:
        with self._flash_cond:
            st = self._walk
            if st is None or m.id != st["id"] or st["end"] is not None:
                return
            st["last"] = time.monotonic()
            st["total"] = m.total
            if m.status == P.WalkDataStatus.GONE:
                st["error"] = f"walk {m.id} was dropped from the mind's store during the download (it needed room)"
                st["end"] = "gone"
            elif m.offset != len(st["buf"]):
                st["error"] = f"walk {m.id}: data at offset {m.offset}, expected {len(st['buf'])}"
                st["end"] = "gap"
            elif m.status == P.WalkDataStatus.END:
                if m.total and m.total != len(st["buf"]):
                    st["error"] = f"walk {m.id}: END after {len(st['buf'])} of {m.total} bytes"
                st["end"] = "end"
            else:
                st["buf"] += m.data
            self._flash_cond.notify_all()
            cb, got = st["progress"], len(st["buf"])
        if cb is not None and m.status == P.WalkDataStatus.DATA:
            cb(got, m.total)

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
        self._send_raw(C.encode_host_message(msg, seq))

    def _send_raw(self, frames: bytes) -> None:
        with self._send_lock:
            if self._closed.is_set():
                raise HostError(self.error or "connection closed")
            try:
                self._channel.send(frames)
            except OSError as e:
                raise HostError(f"send: {e}") from None
            except hostsecure.ChannelError as e:
                raise HostError(str(e)) from None

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
                data = self._channel.recv()
                if data is None:
                    self.error = "the mind closed the connection"
                    break
                for f in self._dec.feed(data):
                    self._dispatch(f)
        except C.HostFrameError as e:
            self.error = f"host stream out of step: {e}"
        except hostsecure.ChannelError as e:
            if not self._closed.is_set():
                self.error = str(e)
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
        elif isinstance(msg, P.HostWalkData):
            self._on_walk_data(msg)
        elif isinstance(msg, P.HostBenchResult):
            with self._flash_cond:
                if self._bench is not None:
                    self._bench.append(msg)
                self._flash_cond.notify_all()
        elif isinstance(msg, P.HostUpdateStatus):
            if self.on_update is not None:
                self.on_update(msg)
            with self._flash_cond:
                self._update_last = msg
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

