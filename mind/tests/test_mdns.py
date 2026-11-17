"""kilo.mdns, the resolver that finds Kilo by name, and `kilo find`: the
wire format, and fake responders on loopback (unicast: no datagram leaves
this machine; conftest.py keeps the real network out of every test). The
mind's own responder is tested in kernel/host/tests/test_mdns.cpp, and the
two against each other in kernel/host/test.sh (kilo find --ask against
build/mdns-host)."""

from __future__ import annotations

import json
import socket
import struct
import threading

import pytest

from kilo import cli, mdns


# ---- a fake responder ---------------------------------------------------------------------
def rr(name: str, rtype: int, rdata: bytes, ttl: int = 10) -> bytes:
    return mdns.encode_name(name) + struct.pack(">HHIH", rtype, mdns.CLASS_IN, ttl, len(rdata)) + rdata


class FakeResponder:
    """Answers legacy unicast queries as the mind kernel does (ID and questions
    echoed); `service=False` answers only A, as Circle's own responder did."""

    def __init__(self, address: str = "192.168.88.9", host: str = "kilo.local", service: bool = True,
                 port: int = 5300, build: str = "2d7789e1", answer_name: bool = True):
        self.address, self.host, self.service, self.port, self.build = address, host, service, port, build
        self.answer_name = answer_name
        self.queries: list[mdns.Message] = []
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.addr = self.sock.getsockname()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self) -> None:
        while True:
            try:
                data, src = self.sock.recvfrom(2048)
            except OSError:
                return
            q = mdns.parse_message(data)
            self.queries.append(q)
            inst = self.host.split(".")[0] + "." + mdns.SERVICE
            answers, extra = [], []
            a = rr(self.host, mdns.TYPE_A, socket.inet_aton(self.address))
            for name, qtype, _ in q.questions:
                if name.lower() == self.host and qtype in (mdns.TYPE_A, mdns.TYPE_ANY) and self.answer_name:
                    answers.append(a)
                if name.lower() == mdns.SERVICE and self.service:
                    answers.append(rr(mdns.SERVICE, mdns.TYPE_PTR, mdns.encode_name(inst)))
                    extra.append(rr(inst, mdns.TYPE_SRV, struct.pack(">HHH", 0, 0, self.port)
                                    + mdns.encode_name(self.host)))
                    txt = f"build={self.build}".encode()
                    extra.append(rr(inst, mdns.TYPE_TXT, bytes([len(txt)]) + txt))
                    extra.append(a)
            if not answers:
                continue
            qbytes = data[12:]
            head = struct.pack(">HHHHHH", q.id, 0x8400, len(q.questions), len(answers), 0, len(extra))
            self.sock.sendto(head + qbytes + b"".join(answers + extra), src)

    def close(self) -> None:
        self.sock.close()


@pytest.fixture
def responder():
    rs: list[FakeResponder] = []

    def make(**kw):
        r = FakeResponder(**kw)
        rs.append(r)
        return r

    yield make
    for r in rs:
        r.close()


# ---- the wire format ----------------------------------------------------------------------
def test_query_layout():
    q = mdns.build_query([("kilo.local", mdns.TYPE_A), (mdns.SERVICE, mdns.TYPE_PTR)], qid=0x1234)
    assert q[:12] == struct.pack(">HHHHHH", 0x1234, 0, 2, 0, 0, 0)
    assert q[12:24] == b"\x04kilo\x05local\x00" and q[24:28] == struct.pack(">HH", 1, 1)
    m = mdns.parse_message(q)
    assert not m.is_response and m.questions == [("kilo.local", 1, 1), ("_kilo._tcp.local", 12, 1)]
    # QU: the top bit of the class; Circle's own responder ignored such questions, so the default is off
    assert mdns.parse_message(mdns.build_query([("kilo.local", 1)], qu=True)).questions[0][2] == 0x8001


def test_names_compression_and_malformed():
    data = b"\x04kilo\x05local\x00" + b"\x05_kilo\x04_tcp\xc0\x05"
    assert mdns.read_name(data, 0) == ("kilo.local", 12)
    assert mdns.read_name(data, 12) == ("_kilo._tcp.local", len(data))
    for bad in (b"\xc0\x00", b"\xc0\x05\x00", b"\x04ki", b"\x40abc"):
        with pytest.raises(ValueError):
            mdns.read_name(bad, 0)
    with pytest.raises(ValueError):
        mdns.parse_message(b"\x00\x01")
    with pytest.raises(ValueError):
        mdns.encode_name("a..b")


def test_parse_records():
    inst = "kilo._kilo._tcp.local"
    msg = (struct.pack(">HHHHHH", 0, 0x8400, 0, 4, 0, 0)
           + rr("kilo.local", mdns.TYPE_A, bytes([192, 168, 88, 9]))
           + rr(mdns.SERVICE, mdns.TYPE_PTR, mdns.encode_name(inst))
           + rr(inst, mdns.TYPE_SRV, struct.pack(">HHH", 0, 0, 5300) + mdns.encode_name("kilo.local"))
           + rr(inst, mdns.TYPE_TXT, b"\x0ebuild=2d7789e1"))
    m = mdns.parse_message(msg)
    assert m.is_response
    assert [r.value for r in m.records] == ["192.168.88.9", inst, (5300, "kilo.local"), ["build=2d7789e1"]]


# ---- resolving against fake responders ----------------------------------------------------
def test_resolve_unicast_with_the_service(responder):
    r = responder()
    found = mdns.resolve(targets=[r.addr], timeout=1.0)
    assert len(found) == 1
    f = found[0]
    assert (f.name, f.address, f.port, f.build, f.instance) == ("kilo.local", "192.168.88.9", 5300, "2d7789e1",
                                                                "kilo._kilo._tcp.local")
    assert f.host_arg == "192.168.88.9" and f.via.startswith("unicast from 127.0.0.1")
    # one legacy query with both questions
    assert [q[0] for q in r.queries[0].questions] == ["kilo.local", "_kilo._tcp.local"]


def test_resolve_a_only_responder(responder):
    """A kernel with Circle's responder: the address, no port or build."""
    r = responder(service=False)
    f = mdns.resolve(targets=[r.addr], timeout=1.0)
    assert [(x.address, x.port, x.build) for x in f] == [("192.168.88.9", None, None)]


def test_find_a_kilo_that_took_another_name(responder):
    """Another host had kilo.local: the mind took kilo-2; the service query finds it."""
    r = responder(host="kilo-2.local", port=5301)
    found = mdns.resolve(targets=[r.addr], timeout=0.6)
    assert [(x.name, x.host_arg) for x in found] == [("kilo-2.local", "192.168.88.9:5301")]


def test_nothing_answers(responder):
    r = responder(answer_name=False, service=False)
    assert mdns.resolve(targets=[r.addr], timeout=0.3) == []
    assert len(r.queries) == 1  # no resend inside 0.5 s


def test_the_network_is_off_in_tests():
    with pytest.raises(mdns.MdnsError, match="disabled"):
        mdns.resolve()


def test_host_address(monkeypatch, capsys):
    assert mdns.host_address("192.168.88.9") == "192.168.88.9"
    assert mdns.host_address("10.0.0.1:5301") == "10.0.0.1:5301"
    assert mdns.host_address("example.com") == "example.com"
    seen = []

    def fake_find(name, timeout=mdns.TIMEOUT_S):
        seen.append(name)
        return mdns.Found(name, "192.168.88.8", port=5300, build="abcd1234")

    monkeypatch.setattr(mdns, "find", fake_find)
    assert mdns.host_address("kilo.local") == "192.168.88.8"
    assert mdns.host_address("Kilo.Local:5302") == "192.168.88.8:5302"
    assert mdns.host_address(None) == "192.168.88.8"
    assert seen == ["kilo.local", "Kilo.Local", "kilo.local"]
    err = capsys.readouterr().err
    assert "kilo: found kilo.local at 192.168.88.8, build abcd1234" in err
    assert "kilo: no --host: found kilo.local at 192.168.88.8" in err


# ---- the CLI ------------------------------------------------------------------------------
def test_cli_find(responder, capsys):
    r = responder()
    assert cli.main(["find", "--ask", f"{r.addr[0]}:{r.addr[1]}", "--timeout", "1"]) == 0
    out = capsys.readouterr().out
    assert "kilo.local  192.168.88.9  port 5300  build 2d7789e1" in out
    assert "kilo status --host kilo.local" in out
    assert cli.main(["find", "--ask", f"{r.addr[0]}:{r.addr[1]}", "--json"]) == 0
    d = json.loads(capsys.readouterr().out)
    assert d == [{"name": "kilo.local", "address": "192.168.88.9", "port": 5300, "build": "2d7789e1",
                  "host": "192.168.88.9", "via": d[0]["via"]}]


def test_cli_find_build_over_the_host_channel(responder, capsys, monkeypatch):
    """No TXT in the answer: the build comes from STATUS (here: no mind there, so `?`)."""
    r = responder(service=False, address="127.0.0.1")
    from kilo import hostlink

    monkeypatch.setattr(hostlink, "DEFAULT_PORT", 1)  # nothing listens: STATUS fails at once
    assert cli.main(["find", "--ask", f"{r.addr[0]}:{r.addr[1]}", "--timeout", "0.5"]) == 0
    cap = capsys.readouterr()
    assert "kilo.local  127.0.0.1  port ? (no service answer)  build ?" in cap.out
    assert "STATUS failed" in cap.err


def test_cli_find_nothing(capsys):
    assert cli.main(["find"]) == 1
    assert "kilo find: mDNS is disabled here" in capsys.readouterr().err


def test_cli_host_local_name_is_resolved(responder, capsys, monkeypatch):
    """--host kilo.local goes through the resolver; the connection then uses the address found."""
    free = socket.socket()
    free.bind(("127.0.0.1", 0))
    port = free.getsockname()[1]
    free.close()  # a port nothing listens on: the connection fails, naming it
    r = responder(address="127.0.0.1", port=port)
    real = mdns.resolve
    monkeypatch.setattr(mdns, "resolve", lambda name, timeout=1.5, service=True, targets=None:
                        real(name, 0.5, service, targets=[r.addr]))
    assert cli.main(["status", "--host", "kilo.local"]) == 1
    err = capsys.readouterr().err
    assert f"kilo: found kilo.local at 127.0.0.1:{port}, build 2d7789e1" in err
    assert f"127.0.0.1:{port}" in err.splitlines()[-1]
    # a host-only command without --host finds it the same way
    assert cli.main(["walks"]) == 1
    assert f"kilo: no --host: found kilo.local at 127.0.0.1:{port}" in capsys.readouterr().err
