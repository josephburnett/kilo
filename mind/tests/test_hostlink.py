"""kilo.hostlink and kilo.cli_host against a fake mind: a TCP server speaking
the host channel (protocol/spec.toml [host_messages]) with the generated
dataclasses and framing, the way kernel/mind/net/host_server.cpp does.
The real server is tested in kernel/host/tests/test_host.cpp, and both
together (with the virtual body) in kernel/host/test.sh."""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any

import pytest

from kilo import cli
from kilo.hostlink import BridgeTransport, HostError, HostLink, HostRefused, parse_address
from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P


class FakeMind:
    """One-connection-at-a-time host channel server. Records what it got."""

    def __init__(self) -> None:
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
        c.sendall(C.encode_host_message(msg, seq))

    def _serve(self, c: socket.socket) -> None:
        dec = C.HostStreamDecoder()
        try:
            while True:
                data = c.recv(4096)
                if not data:
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
            c.sendall(b"\x01\x00\xff")  # len 1: out of step
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
        elif isinstance(m, P.HostBridgeTx):
            self.bridge_tx += m.data
            self._send(c, P.HostBridgeRx(data=m.data.upper()), 0)
        elif m.KIND == P.Kind.COMMAND:
            self._ok(c, seq, m, int(P.HostResultCode.UNSUPPORTED))

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
def mind():
    m = FakeMind()
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


def test_cli_unavailable_commands(mind, capsys):
    assert cli.main(["--host", mind.addr, "pca"]) == 1
    assert "not available over --host yet" in capsys.readouterr().err
    assert cli.main(["--host", mind.addr, "posture", "balance"]) == 1
    assert cli.main(["--host", mind.addr, "monitor", "--passive"]) == 1
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
