"""kilo.hostlink and kilo.cli_host against a fake mind: a TCP server speaking
the host channel (protocol/spec.toml [host_messages]) with the generated
dataclasses and framing, the way kernel/mind/net/host_server.cpp does, over
the secure layer (the hello, Noise KN, records: tests/noise_mind.py, built
from noiseprotocol as the mind's end).
The real server is tested in kernel/host/tests/test_host.cpp, and both
together (with the virtual body) in kernel/host/test.sh."""

from __future__ import annotations

import json
import socket
import threading
import time
import zlib
from typing import Any

import pytest

from kilo import cli, hostkey
from kilo.hostlink import (BridgeTransport, HostError, HostKeyProblem, HostKeyRefused, HostLink, HostRefused,
                           parse_address)
from kilo.kernel_image import has_marker
from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P
from noise_mind import Responder


class FakeMind:
    """One-connection-at-a-time host channel server. Records what it got."""

    def __init__(self, keys: list[bytes]) -> None:
        self.keys = keys  # the laptops' public keys this mind knows
        self.hello_status = P.HostHelloStatus.OK  # another status: refuse every hello so
        self.forge_on: int | None = None  # answer this type with a record that does not authenticate
        self._resp: dict[socket.socket, Responder] = {}
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(4)
        self.addr = f"127.0.0.1:{self.srv.getsockname()[1]}"
        self.got: list[Any] = []
        self.bridge_tx = bytearray()
        self.controller = False
        self.garbage_on: int | None = None  # reply to this type with bytes out of step
        self.reboot_closes = True  # after REBOOT's OK, close the connection (kilo-mind-host exiting)
        # UPDATE_*: the image as received, and how the fake answers COMMIT.
        self.update_store = True       # False: no kernel store (kilo-mind-host without --card)
        self.update_fail: int | None = None  # an UPDATE_ERROR the write ends in (after CHECKING)
        self.update_stays_down = False  # after DONE, stop listening (the mind never comes back)
        self.update_begin: Any = None
        self.update_data = bytearray()
        self._conns: list[socket.socket] = []
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self) -> None:
        while True:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            self._conns.append(c)
            threading.Thread(target=self._serve, args=(c,), daemon=True).start()

    def _send(self, c: socket.socket, msg: Any, seq: int) -> None:
        self._resp[c].send(C.encode_host_message(msg, seq))

    def _serve(self, c: socket.socket) -> None:
        dec = C.HostStreamDecoder()
        r = self._resp[c] = Responder(c, self.keys)
        try:
            if not r.handshake(self.hello_status):
                c.close()
                return
            while True:
                data = r.recv()
                if data is None:
                    return
                for f in dec.feed(data):
                    msg = P.decode_host_payload(f.type, f.payload)
                    self.got.append(msg)
                    self._handle(c, f.seq, msg)
        except OSError:
            return

    def _ok(self, c: socket.socket, seq: int, msg: Any, code: int = 0, text: bytes = b"") -> None:
        self._send(c, P.HostResult(request=msg.ID, code=code, text=text), seq)

    def _handle(self, c: socket.socket, seq: int, m: Any) -> None:
        if self.garbage_on == m.ID:
            self._resp[c].send(b"\x01\x00\xff")  # a good record, but a frame of len 1: out of step
            return
        if self.forge_on == m.ID:
            self._resp[c].send_forged()
            return
        if isinstance(m, P.HostStatusReq):
            st = P.HostStatus(host_version=P.HOST_VERSION, mind_build=0x1E1405F0, uptime_ms=4321,
                              state=int(P.HostState.LINK_CONNECTED | P.HostState.BODY_ALIVE | P.HostState.HAS_HELLO
                                        | P.HostState.HAS_TELEMETRY), clients=1, proto_ok=1, telemetry_age_ms=8,
                              telemetry_count=100)
            st.hello.proto_major, st.hello.fw_build = 2, 0x45524F43
            st.telemetry.mode, st.telemetry.batt_mv, st.telemetry.pitch = int(P.Mode.IDLE), 7400, -32
            st.link.frames_rx = 120
            self._send(c, st, seq)
        elif isinstance(m, P.HostControl):
            self.controller = bool(m.take)
            self._ok(c, seq, m)
        elif isinstance(m, P.HostPosture):
            self._ok(c, seq, m, 0 if self.controller else int(P.HostResultCode.NOT_CONTROLLER), b"take control first")
        elif isinstance(m, P.HostTelemSub):
            self._ok(c, seq, m)
            for i in range(3):
                t = P.HostTelem(uptime_ms=i, state=int(P.HostState.HAS_TELEMETRY), telemetry_count=i)
                t.telemetry.mode, t.telemetry.tick = int(P.Mode.LIMP), 10 + i
                self._send(c, t, i)
        elif isinstance(m, P.HostLogSub):
            self._ok(c, seq, m)
            for i in range(m.tail):
                self._send(c, P.HostLogLine(seq=i, t_ms=1000 * i + 5, level=1, text=f"line {i}".encode()), i)
        elif isinstance(m, P.HostBridgeOpen) or isinstance(m, P.HostBridgeClose):
            self._ok(c, seq, m)
        elif isinstance(m, P.HostReboot):
            if not self.controller:
                self._ok(c, seq, m, int(P.HostResultCode.NOT_CONTROLLER), b"take control first")
                return
            self._ok(c, seq, m, 0, b"rebooting")
            if self.reboot_closes:
                c.shutdown(socket.SHUT_RDWR)
        elif isinstance(m, P.HostUpdateBegin):
            if not self.update_store:
                self._ok(c, seq, m, int(P.HostResultCode.UNSUPPORTED), b"no kernel store (kilo-mind-host: --card DIR)")
                return
            self.update_begin, self.update_data = m, bytearray()
            self._ok(c, seq, m)
        elif isinstance(m, P.HostUpdateData):
            if m.offset == len(self.update_data):
                self.update_data += m.data
        elif isinstance(m, P.HostUpdateCommit):
            self._update_commit(c, seq, m)
        elif isinstance(m, P.HostBridgeTx):
            self.bridge_tx += m.data
            self._send(c, P.HostBridgeRx(data=m.data.upper()), 0)
        elif m.KIND == P.Kind.COMMAND:
            self._ok(c, seq, m, int(P.HostResultCode.UNSUPPORTED))

    def _update_commit(self, c: socket.socket, seq: int, m: Any) -> None:
        b, data = self.update_begin, bytes(self.update_data)
        n = len(data)

        def status(phase: Any, error: int = 0, done: int = 0, card: int = 0) -> Any:
            return P.HostUpdateStatus(phase=int(phase), error=error, received=n, image_size=b.image_size, done=done,
                                      crc32=zlib.crc32(data), card_crc32=card)

        error = 0
        if n < b.image_size:
            error = P.UpdateError.SHORT
        elif zlib.crc32(data) != b.crc32:
            error = P.UpdateError.CRC_MISMATCH
        elif not b.flags & P.UpdateFlags.FORCE and not has_marker(data):
            error = P.UpdateError.NOT_A_KERNEL
        if error:  # as HostServer: the FAILED status first, then RESULT IMAGE_BAD
            self._send(c, status(P.UpdatePhase.FAILED, int(error)), 0)
            self._ok(c, seq, m, int(P.HostResultCode.IMAGE_BAD), f"{P.UpdateError(error).name}: test".encode())
            return
        self._ok(c, seq, m)
        self._send(c, status(P.UpdatePhase.CHECKING), 1)
        for k, done in enumerate(range(0, n + 1, max(1, n // 4))):
            self._send(c, status(P.UpdatePhase.WRITING, done=done), 2 + k)
            if self.update_fail == P.UpdateError.WRITE_FAILED and done > 0:
                self._send(c, status(P.UpdatePhase.FAILED, int(self.update_fail), done=done), 20)
                return
        self._send(c, status(P.UpdatePhase.VERIFYING, done=n), 30)
        self._send(c, status(P.UpdatePhase.INSTALLING, done=n, card=zlib.crc32(data)), 31)
        self._send(c, status(P.UpdatePhase.DONE, done=n, card=zlib.crc32(data)), 32)
        if self.update_stays_down:  # stop listening (shutdown: a blocked accept() keeps a closed socket open)
            self.srv.shutdown(socket.SHUT_RDWR)
            self.srv.close()
        c.shutdown(socket.SHUT_RDWR)  # it reboots (kilo-mind-host: exits 3)

    def names(self) -> list[str]:
        return [m.NAME for m in self.got]

    def close(self) -> None:
        self.srv.close()
        for c in self._conns:
            try:
                c.close()
            except OSError:
                pass


@pytest.fixture
def mind(host_key):
    m = FakeMind([host_key.public])
    yield m
    m.close()


def wait_until(pred, timeout: float = 2.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.01)
    return pred()


def test_parse_address():
    assert parse_address("169.254.75.73") == ("169.254.75.73", P.HOST_PORT) == ("169.254.75.73", 5300)
    assert parse_address("127.0.0.1:41000") == ("127.0.0.1", 41000)
    for bad in ("1.2.3.4:", ":80", "1.2.3.4:x", "1.2.3.4:70000"):
        with pytest.raises(ValueError):
            parse_address(bad)


def test_status_command_and_refusal(mind):
    with HostLink.connect(mind.addr) as hl:
        st = hl.status()
        assert isinstance(st, P.HostStatus) and st.mind_build == 0x1E1405F0 and st.hello.fw_build == 0x45524F43
        with pytest.raises(HostRefused) as ei:
            hl.command(P.HostPosture(posture=int(P.Posture.STAND)))
        assert ei.value.code_name == "NOT_CONTROLLER" and "take control first" in str(ei.value)
        hl.take_control(keepalive=False)
        assert hl.command(P.HostPosture(posture=int(P.Posture.STAND))).code == 0
        with pytest.raises(HostRefused, match="PARAM_GET: UNSUPPORTED"):
            hl.command(P.HostParamGet(id=1))
        with pytest.raises(ValueError):
            hl.request(P.HostKeepalive())  # a stream, not a command


def test_streams(mind):
    with HostLink.connect(mind.addr) as hl:
        telem, lines = [], []
        hl.subscribe_telem(10, telem.append)
        hl.subscribe_log(lines.append, tail=4)
        assert wait_until(lambda: len(telem) == 3 and len(lines) == 4)
        assert [t.telemetry.tick for t in telem] == [10, 11, 12]
        assert [ln.text for ln in lines] == [b"line 0", b"line 1", b"line 2", b"line 3"]
    sub = next(m for m in mind.got if isinstance(m, P.HostTelemSub))
    assert sub.hz == 10


def test_keepalive_while_in_control(mind, monkeypatch):
    monkeypatch.setattr(P, "HOST_KEEPALIVE_SEND_MS", 20)
    with HostLink.connect(mind.addr) as hl:
        hl.take_control(keepalive=True)
        assert wait_until(lambda: mind.names().count("KEEPALIVE") >= 3)
    ctl = next(m for m in mind.got if isinstance(m, P.HostControl))
    assert (ctl.take, ctl.keepalive) == (1, 1)


def test_out_of_step_closes_and_fails_the_request(mind):
    mind.garbage_on = P.HostMsgId.STATUS_REQ
    hl = HostLink.connect(mind.addr)
    with pytest.raises(HostError, match="out of step"):
        hl.status()
    assert hl.closed
    hl.close()


def test_connect_refused():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    with pytest.raises(HostError, match="cannot reach the mind"):
        HostLink.connect(f"127.0.0.1:{port}", timeout=1.0)


def test_bridge_transport(mind):
    with HostLink.connect(mind.addr) as hl:
        tr = BridgeTransport(hl)
        assert mind.names()[:2] == ["CONTROL", "BRIDGE_OPEN"]
        tr.write(b"x" * 450)  # three BRIDGE_TX frames (200 B max each)
        assert wait_until(lambda: len(mind.bridge_tx) == 450)
        assert mind.names().count("BRIDGE_TX") == 3
        got = b""
        end = time.monotonic() + 2
        while len(got) < 450 and time.monotonic() < end:
            got += tr.read(1000)
        assert got == b"X" * 450
        assert tr.read(10) == b""  # nothing: returns after the read timeout
        tr.close()
        assert "BRIDGE_CLOSE" in mind.names()


# ---- the CLI over --host ----------------------------------------------------------------
def test_cli_status(mind, capsys):
    assert cli.main(["--host", mind.addr, "status"]) == 0
    out = capsys.readouterr().out
    assert "build 1e1405f0" in out and "build 45524f43" in out and "mode        IDLE" in out and "frames_rx=120" in out
    assert cli.main(["status", "--host", mind.addr, "--json"]) == 0
    d = json.loads(capsys.readouterr().out)
    assert d["mind_build"] == "1e1405f0" and d["telemetry"]["batt_mv"] == 7400 and "BODY_ALIVE" in d["state"]
    assert d["last_event"] is None


def test_cli_posture_holds_then_releases(mind, capsys):
    assert cli.main(["--host", mind.addr, "posture", "limp", "--for", "0.3"]) == 0
    assert "holding limp" in capsys.readouterr().out
    ctl = [m for m in mind.got if isinstance(m, P.HostControl)]
    pos = [m for m in mind.got if isinstance(m, P.HostPosture)]
    assert [(c.take, c.keepalive) for c in ctl] == [(1, 1), (0, 0)]
    assert pos[0].posture == P.Posture.LIMP


def test_cli_local_commands_refuse_host(mind, capsys):
    assert cli.main(["--host", mind.addr, "key", "show"]) == 1
    assert "runs on this machine, not over --host" in capsys.readouterr().err
    assert mind.got == []  # refused here, before any request reached the mind


def test_cli_monitor_and_log(mind, capsys):
    assert cli.main(["--host", mind.addr, "monitor", "--json", "--duration", "0.3", "--rate", "5"]) == 0
    recs = [json.loads(x) for x in capsys.readouterr().out.splitlines()]
    assert [r["tick"] for r in recs] == [10, 11, 12] and recs[0]["mode"] == "LIMP"
    assert cli.main(["log", "--host", mind.addr, "--lines", "2", "--duration", "0.3"]) == 0
    out = capsys.readouterr().out.splitlines()
    assert out == ["     0.005 I line 0", "     1.005 I line 1"]
    assert cli.main(["--host", mind.addr, "log", "-n", "1", "--duration", "0.2"]) == 0
    assert capsys.readouterr().out.splitlines() == ["     0.005 I line 0"]


def test_reboot(mind):
    with HostLink.connect(mind.addr) as hl:
        with pytest.raises(HostRefused, match="REBOOT: NOT_CONTROLLER"):
            hl.reboot()
        assert not hl.closed
        hl.take_control(keepalive=False)
        assert hl.reboot().code == P.HostResultCode.OK
        assert hl.wait_closed(2.0) and hl.closed


def test_cli_reboot(mind, capsys):
    assert cli.main(["reboot", "--host", mind.addr]) == 0
    out = capsys.readouterr().out
    assert "REBOOT: OK (rebooting)" in out and "the mind closed the connection" in out
    ctl = next(m for m in mind.got if isinstance(m, P.HostControl))
    assert (ctl.take, ctl.keepalive) == (1, 0) and mind.names() == ["CONTROL", "REBOOT"]
    assert cli.main(["--host", mind.addr, "reboot"]) == 0  # --host before the command too


def test_cli_reboot_without_a_close_is_still_done(mind, capsys, monkeypatch):
    # A Pi that reboots sends no FIN: the OK is the answer.
    from kilo import cli_host

    monkeypatch.setattr(cli_host, "REBOOT_CLOSE_WAIT_S", 0.1)
    mind.reboot_closes = False
    assert cli.main(["reboot", "--host", mind.addr]) == 0
    assert "no close within 0.1 s" in capsys.readouterr().out


def test_cli_reboot_needs_host(capsys):
    assert cli.main(["reboot"]) == 1
    assert "needs --host" in capsys.readouterr().err


def test_cli_host_unreachable(capsys):
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    assert cli.main(["--timeout", "1", "--host", f"127.0.0.1:{port}", "status"]) == 1
    assert "cannot reach the mind" in capsys.readouterr().err


# ---- kernel updates (UPDATE_*) ----------------------------------------------------------
MARKER = P.HOST_UPDATE_MARKER.encode()


def kernel_bytes(n: int = 1000, marker: bool = True, key: bool = True) -> bytes:
    """A fake mind kernel: the marker, and this laptop's public key built in
    (as kernel/keys/gen_keys.py puts the authorized keys into a real one)."""
    body = bytes((i * 7) % 251 for i in range(n))
    if marker:
        body = body[:400] + MARKER + body[400 + len(MARKER):]
    if marker and key:
        pub = hostkey.load().public
        body = body[:600] + pub + body[600 + len(pub):]
    return body


def test_has_marker():
    assert has_marker(kernel_bytes()) and not has_marker(kernel_bytes(marker=False))
    assert P.HOST_UPDATE_MARKER == "KILO-MIND-KERNEL-1"
    assert not has_marker(MARKER[:-1])


def test_update(mind):
    img = kernel_bytes(1000)
    seen: list[Any] = []
    with HostLink.connect(mind.addr) as hl:
        st = hl.update(img, progress=seen.append)
        assert st.phase == P.UpdatePhase.DONE and st.card_crc32 == zlib.crc32(img)
        assert hl.wait_closed(2.0)
    assert bytes(mind.update_data) == img
    b = mind.update_begin
    assert (b.image_size, b.crc32, b.flags) == (1000, zlib.crc32(img), 0)
    data = [m for m in mind.got if isinstance(m, P.HostUpdateData)]
    assert len(data) == 5 and all(len(m.data) <= P.HOST_UPDATE_CHUNK_MAX for m in data)
    assert mind.names()[:2] == ["CONTROL", "UPDATE_BEGIN"] and mind.names()[-1] == "UPDATE_COMMIT"
    phases = [P.UpdatePhase(x.phase).name for x in seen]
    assert phases[0] == "CHECKING" and phases[-1] == "DONE" and "WRITING" in phases and "VERIFYING" in phases


def test_update_refused(mind):
    seen: list[Any] = []
    with HostLink.connect(mind.addr) as hl:
        with pytest.raises(HostRefused) as ei:
            hl.update(kernel_bytes(marker=False), progress=seen.append)
        assert ei.value.code_name == "IMAGE_BAD" and "NOT_A_KERNEL" in ei.value.text
        assert seen and seen[-1].phase == P.UpdatePhase.FAILED and seen[-1].error == P.UpdateError.NOT_A_KERNEL
        mind.update_store = False
        with pytest.raises(HostRefused, match="UPDATE_BEGIN: UNSUPPORTED"):
            hl.update(kernel_bytes())
    with HostLink.connect(mind.addr) as hl:  # FORCE: no marker needed
        mind.update_store = True
        assert hl.update(kernel_bytes(marker=False), force=True).phase == P.UpdatePhase.DONE
    assert mind.update_begin.flags == int(P.UpdateFlags.FORCE)


def test_cli_update(mind, tmp_path, capsys):
    img = tmp_path / "kernel7.img"
    img.write_bytes(kernel_bytes(3000))
    assert cli.main(["update", "--host", mind.addr, "--image", str(img), "--wait", "5"]) == 0
    out = capsys.readouterr().out
    assert "update: mind build 1e1405f0; sending" in out and "(3000 bytes, crc32" in out
    assert "update: writing kernel7.new: 3000 of 3000 bytes" in out
    assert "update: DONE: kernel7.img on the SD card is the new kernel (3000 bytes" in out
    assert "update: the mind is rebooting into it" in out
    assert "update: the mind is back after" in out and "mind build 1e1405f0  (the same build as before)" in out
    assert bytes(mind.update_data) == img.read_bytes()
    assert cli.main(["--host", mind.addr, "update", "--image", str(img), "--wait", "0"]) == 0
    assert "the mind is back" not in capsys.readouterr().out


def test_cli_update_refuses_locally(mind, tmp_path, capsys):
    loader = tmp_path / "loader.img"
    loader.write_bytes(kernel_bytes(2000, marker=False))
    assert cli.main(["update", "--host", mind.addr, "--image", str(loader)]) == 1
    err = capsys.readouterr().err
    assert "has no KILO-MIND-KERNEL-1" in err and "--force" in err
    empty = tmp_path / "empty.img"
    empty.write_bytes(b"")
    assert cli.main(["update", "--host", mind.addr, "--image", str(empty)]) == 1
    assert cli.main(["update", "--host", mind.addr, "--image", str(tmp_path / "missing.img")]) == 1
    assert "cannot read" in capsys.readouterr().err
    assert mind.got == []  # nothing reached the mind
    assert cli.main(["update", "--host", mind.addr, "--image", str(loader), "--force", "--wait", "0"]) == 0
    cap = capsys.readouterr()
    assert "with --force" in cap.out and bytes(mind.update_data) == loader.read_bytes()
    assert "is not the mind kernel (no KILO-MIND-KERNEL-1); installed anyway (--force)" in cap.err


def test_cli_update_failures(mind, tmp_path, capsys):
    img = tmp_path / "kernel7.img"
    img.write_bytes(kernel_bytes(3000))
    mind.update_fail = int(P.UpdateError.WRITE_FAILED)
    assert cli.main(["update", "--host", mind.addr, "--image", str(img)]) == 1
    err = capsys.readouterr().err
    assert "kilo update: FAILED: WRITE_FAILED: writing kernel7.new failed; the card's kernel7.img is unchanged" in err
    mind.update_fail = None
    mind.update_store = False
    assert cli.main(["update", "--host", mind.addr, "--image", str(img)]) == 1
    assert "UNSUPPORTED (no kernel store (kilo-mind-host: --card DIR))" in capsys.readouterr().err


def test_cli_update_mind_does_not_come_back(mind, tmp_path, capsys):
    img = tmp_path / "kernel7.img"
    img.write_bytes(kernel_bytes(1000))
    mind.update_stays_down = True
    assert cli.main(["update", "--host", mind.addr, "--image", str(img), "--wait", "0.5"]) == 1
    cap = capsys.readouterr()
    assert "update: DONE" in cap.out and "did not come back within 0.5 s" in cap.err


def test_cli_update_needs_host(capsys):
    assert cli.main(["update"]) == 1
    assert "needs --host" in capsys.readouterr().err


# ---- the secure layer (kilo.hostsecure, kilo.hostkey) --------------------------------------

def test_a_key_the_mind_does_not_know(host_key, tmp_path):
    m = FakeMind([bytes(32)])  # knows another key only
    try:
        with pytest.raises(HostKeyRefused, match="the mind does not know this laptop's key "
                                                 f"\\({host_key.fingerprint}, .*kernel/keys/authorized_keys"):
            HostLink.connect(m.addr)
        assert m.got == []  # nothing was served
    finally:
        m.close()


def test_cli_with_a_key_the_mind_does_not_know(host_key, capsys):
    m = FakeMind([bytes(32)])
    try:
        assert cli.main(["--host", m.addr, "status"]) == 1
        err = capsys.readouterr().err
        assert err.startswith("kilo status: the mind does not know this laptop's key") and err.count("\n") == 1
        assert "kilo update --host" in err and "card reader" in err and "make -C kernel" in err
    finally:
        m.close()


def test_cli_without_a_key(mind, tmp_path, capsys, monkeypatch):
    monkeypatch.setenv("KILO_HOST_KEY", str(tmp_path / "none"))
    assert cli.main(["--host", mind.addr, "status"]) == 1
    err = capsys.readouterr().err
    assert "no host key at" in err and "kilo key init" in err and err.count("\n") == 1
    with pytest.raises(HostKeyProblem):
        HostLink.connect(mind.addr)
    assert mind.got == []


def test_cli_key_option(mind, tmp_path, capsys):
    other, _ = hostkey.init(tmp_path / "other_key")
    assert cli.main(["--key", str(other.path), "--host", mind.addr, "status"]) == 1  # the mind knows the default
    assert "does not know this laptop's key" in capsys.readouterr().err
    mind.keys.append(other.public)
    assert cli.main(["status", "--host", mind.addr, "--key", str(other.path)]) == 0


def test_the_wrong_private_key(mind, host_key, tmp_path):
    # A key file whose private key is not the listed public key's: the
    # mind's message 2 does not authenticate here.
    other, _ = hostkey.init(tmp_path / "other")
    impostor = hostkey.HostKey(other.private, host_key.public, "impostor", other.path)
    with pytest.raises(HostKeyRefused, match="the handshake failed: the mind's answer did not authenticate"):
        HostLink.connect(mind.addr, key=impostor)
    assert mind.got == []


def test_a_forged_record_from_the_mind(mind):
    mind.forge_on = P.HostMsgId.STATUS_REQ
    hl = HostLink.connect(mind.addr)
    with pytest.raises(HostError, match="did not authenticate"):
        hl.status()
    assert hl.closed
    hl.close()


def test_the_mind_refuses_the_hello(mind):
    mind.hello_status = P.HostHelloStatus.BAD_VERSION
    with pytest.raises(HostError, match="host channel version"):
        HostLink.connect(mind.addr)
    mind.hello_status = P.HostHelloStatus.REFUSED
    with pytest.raises(HostError, match="refused the handshake for now"):
        HostLink.connect(mind.addr)


def test_an_older_mind_closes_the_hello():
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)

    def serve() -> None:
        c, _ = srv.accept()
        c.recv(100)
        c.close()  # a version-1 mind: the hello is not a frame it knows

    threading.Thread(target=serve, daemon=True).start()
    try:
        with pytest.raises(HostError, match="without answering the hello: it may be an older mind"):
            HostLink.connect(f"127.0.0.1:{srv.getsockname()[1]}")
    finally:
        srv.close()


def test_frames_are_packed_into_records(mind, host_key):
    # 5000 bytes of UPDATE_DATA in 200-byte frames: several frames a record,
    # none over host.record_max (the Responder would refuse a longer one).
    img = kernel_bytes(5000)
    with HostLink.connect(mind.addr) as hl:
        assert hl.update(img).phase == P.UpdatePhase.DONE
    assert bytes(mind.update_data) == img and mind.names().count("UPDATE_DATA") == 25
    records = sum(r.records for r in mind._resp.values())
    assert records < 25, f"{records} records for 25 UPDATE_DATA frames and a few commands"


def test_every_host_command_takes_key(mind, tmp_path, capsys, monkeypatch):
    # --key before or after the command, for each --host command; $KILO_HOST_KEY points nowhere.
    other, _ = hostkey.init(tmp_path / "other")
    mind.keys[:] = [other.public]
    monkeypatch.setenv("KILO_HOST_KEY", str(tmp_path / "none"))
    k = str(other.path)
    assert cli.main(["log", "--host", mind.addr, "--key", k, "--lines", "1", "--duration", "0.2"]) == 0
    assert cli.main(["--key", k, "--host", mind.addr, "log", "-n", "1", "--duration", "0.2"]) == 0
    assert cli.main(["--host", mind.addr, "posture", "stand", "--for", "0.1", "--key", k]) == 0
    assert cli.main(["--host", mind.addr, "monitor", "--duration", "0.2", "--key", k]) == 0
    img = tmp_path / "kernel7.img"
    body = bytearray(kernel_bytes(3000, key=False))
    body[900:932] = other.public  # the other key built in
    img.write_bytes(bytes(body))
    assert cli.main(["update", "--host", mind.addr, "--image", str(img), "--wait", "0", "--key", k]) == 0
    capsys.readouterr()
    assert cli.main(["--host", mind.addr, "status"]) == 1  # without --key: $KILO_HOST_KEY, which is missing
    assert "kilo key init" in capsys.readouterr().err


def test_a_refused_send_uses_no_nonce(mind):
    from kilo.protocol import kilo_link_codec as C2
    with HostLink.connect(mind.addr) as hl:
        ch = hl._channel
        sent = ch._sent
        good = C2.encode_host_message(P.HostStatusReq(), 1)
        bad = (2000).to_bytes(2, "little") + bytes(2000)  # a frame no record can hold
        with pytest.raises(ValueError):
            ch.send(good + bad)
        assert ch._sent == sent
        assert hl.status().mind_build == 0x1E1405F0  # the stream is still in step
