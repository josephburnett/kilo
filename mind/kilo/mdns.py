"""Finding Kilo on the network by name: a small multicast DNS (mDNS) resolver,
standard library only, for machines that cannot resolve ``.local`` names
themselves (a Debian server without avahi-daemon or libnss-mdns).

Kilo's address comes from DHCP and changes. The mind kernel answers mDNS
queries (kernel/mind/net/mdns.h) for its name, ``kilo.local``, and for its
laptop service, ``_kilo._tcp.local`` (the instance ``kilo._kilo._tcp.local``:
port and build). mDNS is DNS sent to everyone on the local network at once
(the multicast group 224.0.0.251, UDP port 5353); the host that owns the name
answers.

``resolve()`` asks both questions on every IPv4 interface that is up and can
multicast, twice over:

- from a socket of its own on an ephemeral port (a "legacy unicast" query):
  the kernel answers straight back to it, with the port and build;
- from port 5353 (shared with any other mDNS software, SO_REUSEADDR /
  SO_REUSEPORT, and joined to the group), one question a datagram: the
  answers come on the group. Kernels before this resolver (Circle's own
  responder) answer only this way, and only for ``kilo.local``.

It waits up to ``timeout`` (1.5 s) and returns what answered. Finding gives
only an address: the host channel's Noise handshake still checks that the
mind at that address holds a key this laptop trusts (kilo.hostlink).

    kilo find                         the address and build
    kilo status --host kilo.local     any --host command takes a .local name
    kilo walks                        every command that talks to Kilo finds it without --host
"""

from __future__ import annotations

import errno
import os
import random
import select
import socket
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Iterable

GROUP = "224.0.0.251"
PORT = 5353
DEFAULT_NAME = "kilo.local"
SERVICE = "_kilo._tcp.local"
TIMEOUT_S = 1.5
RESEND_S = 0.5  # the queries again if nothing answered by then
SETTLE_S = 0.3  # after the first address, this long for the rest (the build, another Kilo)

TYPE_A, TYPE_PTR, TYPE_TXT, TYPE_SRV, TYPE_ANY = 1, 12, 16, 33, 255
CLASS_IN, CLASS_FLUSH = 1, 0x8000

# Tests set this (or $KILO_MDNS_DISABLE=1, for the CLI in a subprocess): no
# datagram leaves this machine unless `targets` is given
# (mind/tests/conftest.py), so a test run never finds, or reboots, the real Kilo.
NETWORK_DISABLED = False


class MdnsError(Exception):
    pass


# ---- the wire format ------------------------------------------------------------------
def encode_name(name: str) -> bytes:
    out = b""
    for label in name.rstrip(".").split("."):
        raw = label.encode("utf-8")
        if not 0 < len(raw) < 64:
            raise ValueError(f"bad DNS name {name!r}")
        out += bytes([len(raw)]) + raw
    return out + b"\0"


def build_query(questions: Iterable[tuple[str, int]], qid: int = 0, qu: bool = False) -> bytes:
    """A standard query: header, then each (name, type) in class IN (QU: ask for a unicast answer)."""
    qs = list(questions)
    out = struct.pack(">HHHHHH", qid, 0, len(qs), 0, 0, 0)
    for name, qtype in qs:
        out += encode_name(name) + struct.pack(">HH", qtype, CLASS_IN | (CLASS_FLUSH if qu else 0))
    return out


def read_name(data: bytes, off: int) -> tuple[str, int]:
    """The name at off (compression followed) and the offset after it."""
    labels: list[str] = []
    end = None
    hops = 0
    while True:
        if off >= len(data):
            raise ValueError("name runs past the end")
        n = data[off]
        if n == 0:
            off += 1
            break
        if n & 0xC0 == 0xC0:
            if off + 1 >= len(data) or hops > 16:
                raise ValueError("bad name pointer")
            to = ((n & 0x3F) << 8) | data[off + 1]
            if to >= off:
                raise ValueError("name pointer forward")
            if end is None:
                end = off + 2
            off, hops = to, hops + 1
            continue
        if n & 0xC0 or off + 1 + n > len(data):
            raise ValueError("bad label")
        labels.append(data[off + 1:off + 1 + n].decode("utf-8", "replace"))
        off += 1 + n
    return ".".join(labels), end if end is not None else off


@dataclass
class Record:
    name: str
    type: int
    cls: int
    ttl: int
    data: bytes
    value: object = None  # A: "a.b.c.d"; PTR: name; SRV: (port, target); TXT: [str]


@dataclass
class Message:
    id: int
    flags: int
    questions: list[tuple[str, int, int]] = field(default_factory=list)
    records: list[Record] = field(default_factory=list)  # answers, authority, additional

    @property
    def is_response(self) -> bool:
        return bool(self.flags & 0x8000)


def parse_message(data: bytes) -> Message:
    """A DNS message; ValueError if malformed."""
    if len(data) < 12:
        raise ValueError("short message")
    qid, flags, qd, an, ns, ar = struct.unpack_from(">HHHHHH", data)
    m = Message(qid, flags)
    off = 12
    for _ in range(qd):
        name, off = read_name(data, off)
        if off + 4 > len(data):
            raise ValueError("short question")
        qtype, qclass = struct.unpack_from(">HH", data, off)
        off += 4
        m.questions.append((name, qtype, qclass))
    for _ in range(an + ns + ar):
        name, off = read_name(data, off)
        if off + 10 > len(data):
            raise ValueError("short record")
        rtype, rclass, ttl, rdlen = struct.unpack_from(">HHIH", data, off)
        off += 10
        rd = data[off:off + rdlen]
        if len(rd) != rdlen:
            raise ValueError("short record data")
        r = Record(name, rtype, rclass, ttl, rd)
        if rtype == TYPE_A and rdlen == 4:
            r.value = socket.inet_ntoa(rd)
        elif rtype == TYPE_PTR:
            r.value = read_name(data, off)[0]
        elif rtype == TYPE_SRV and rdlen >= 7:
            r.value = (struct.unpack_from(">H", data, off + 4)[0], read_name(data, off + 6)[0])
        elif rtype == TYPE_TXT:
            strings, i = [], 0
            while i < rdlen:
                n = rd[i]
                strings.append(rd[i + 1:i + 1 + n].decode("utf-8", "replace"))
                i += 1 + n
            r.value = strings
        m.records.append(r)
        off += rdlen
    return m


# ---- the interfaces ---------------------------------------------------------------------
def interfaces() -> list[str]:
    """IPv4 addresses of the interfaces that are up and can multicast, loopback left out."""
    found: list[str] = []
    try:
        import fcntl

        SIOCGIFFLAGS, SIOCGIFADDR = 0x8913, 0x8915  # Linux
        IFF_UP, IFF_LOOPBACK, IFF_MULTICAST = 0x1, 0x8, 0x1000
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            for _, name in socket.if_nameindex():
                req = struct.pack("256s", name.encode()[:15])
                try:
                    flags = struct.unpack_from("H", fcntl.ioctl(s.fileno(), SIOCGIFFLAGS, req), 16)[0]
                    if not flags & IFF_UP or flags & IFF_LOOPBACK or not flags & IFF_MULTICAST:
                        continue
                    addr = socket.inet_ntoa(fcntl.ioctl(s.fileno(), SIOCGIFADDR, req)[20:24])
                except OSError:
                    continue  # no IPv4 address
                if addr not in found:
                    found.append(addr)
    except (ImportError, OSError, AttributeError):
        pass
    if not found and sys.platform != "linux":
        # elsewhere (macOS): the interface the route to the group leaves by (connect sends nothing)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect((GROUP, PORT))
                addr = s.getsockname()[0]
                if addr != "0.0.0.0":
                    found.append(addr)
        except OSError:
            pass
    return found


# ---- resolving --------------------------------------------------------------------------
@dataclass
class Found:
    """One Kilo that answered."""
    name: str                  # its host name ("kilo.local"; "kilo-2.local" if two share the network)
    address: str
    port: int | None = None    # the laptop service's port (SRV), if it answered for the service
    build: str | None = None   # the mind's build (TXT build=), if it answered for the service
    instance: str | None = None
    via: str = ""              # "unicast" or "multicast", and the interface

    @property
    def host_arg(self) -> str:
        """What --host takes."""
        return self.address if self.port in (None, 5300) else f"{self.address}:{self.port}"


class _Source:
    """What one address said: its A records, the service's SRV and TXT, and
    every record name in its answers (for -v)."""

    def __init__(self, via: str):
        self.via = via
        self.addr: dict[str, str] = {}             # host name -> address
        self.srv: dict[str, tuple[int, str]] = {}  # _kilo._tcp instance -> (port, host name)
        self.txt: dict[str, list[str]] = {}        # _kilo._tcp instance -> TXT strings
        self.names: list[str] = []


def _is_instance(n: str) -> bool:
    return n.endswith("." + SERVICE)


class _Collect:
    """The answers, by the address they came from. Only records that name the
    asked host (kilo.local) or an instance of the _kilo._tcp service count:
    the multicast socket hears every mDNS answer on the network (a printer's
    SRV on port 443, a phone's own name), and none of those is a Kilo."""

    def __init__(self, name: str):
        self.name = name.lower().rstrip(".")
        self.src: dict[tuple[str, int], _Source] = {}  # by (address, port) the answer came from

    def feed(self, data: bytes, via: str, src: tuple[str, int] = ("", 0)) -> None:
        try:
            m = parse_message(data)
        except (ValueError, struct.error):
            return
        if not m.is_response:
            return
        s = self.src.get(src)
        if s is None:
            s = self.src[src] = _Source(via)
        for r in m.records:
            n = r.name.lower().rstrip(".")
            if n not in s.names:
                s.names.append(n)
            if r.type == TYPE_A and r.value:
                s.addr[n] = str(r.value)
            elif r.type == TYPE_SRV and _is_instance(n) and isinstance(r.value, tuple):
                s.srv[n] = (r.value[0], r.value[1].lower().rstrip("."))
            elif r.type == TYPE_TXT and _is_instance(n) and isinstance(r.value, list):
                s.txt[n] = r.value

    def _found(self) -> tuple[list[Found], list[str]]:
        found: list[Found] = []
        ignored: list[str] = []
        for (addr, _), s in self.src.items():
            got: list[Found] = []
            hosts = ([self.name] if self.name in s.addr else []) + [t for _, t in s.srv.values()]
            for host in dict.fromkeys(hosts):
                if host not in s.addr:
                    continue
                f = Found(host, s.addr[host], via=s.via)
                for inst, (port, target) in s.srv.items():
                    if target == host:
                        f.port, f.instance = port, inst
                        for t in s.txt.get(inst, []):
                            if t.startswith("build="):
                                f.build = t[len("build="):]
                got.append(f)
            for f in got:  # the same Kilo from another port (unicast and multicast): one entry, merged
                same = next((g for g in found if (g.name, g.address) == (f.name, f.address)), None)
                if same is None:
                    found.append(f)
                elif same.build is None and f.build is not None:
                    same.port, same.build, same.instance = f.port, f.build, f.instance
            if not got:
                more = ", ..." if len(s.names) > 6 else ""
                ignored.append(f"{addr or '?'}: not a Kilo's answer ({', '.join(s.names[:6]) or 'no records'}{more})")
        # A Kilo's answer carries the service's build. When one does, an answer
        # for the name alone from elsewhere (another device claiming kilo.local) is set aside.
        if any(f.build for f in found):
            for f in [f for f in found if f.port is None]:
                found.remove(f)
                ignored.append(f"{f.address}: answered for {f.name} but not for {SERVICE}: set aside, "
                               "a Kilo with a build answered")
        found.sort(key=lambda f: (f.name != self.name, f.build is None))
        return found, ignored

    def results(self) -> list[Found]:
        return self._found()[0]

    def ignored(self) -> list[str]:
        return self._found()[1]

    def complete(self) -> bool:
        """The asked name has an address, a port and a build."""
        return any(f.name == self.name and f.build for f in self.results())


def _reuse(s: socket.socket) -> None:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass


def resolve(name: str = DEFAULT_NAME, timeout: float = TIMEOUT_S, service: bool = True,
            targets: list[tuple[str, int]] | None = None, ignored: list[str] | None = None) -> list[Found]:
    """Every Kilo that answered for `name` (and, with service, for the
    _kilo._tcp service) within `timeout` seconds, those with the service's
    build first; empty if none. Answers that name neither (other devices on
    the network) are left out; `ignored`, if given, gets a line for each.

    targets: send the queries there (unicast) instead of to the group on
    every interface, and listen only for unicast answers (tests: a fake
    responder on 127.0.0.1)."""
    name = name.rstrip(".")
    if targets is None and (NETWORK_DISABLED or os.environ.get("KILO_MDNS_DISABLE") == "1"):
        raise MdnsError("mDNS is disabled here (tests)")
    col = _Collect(name)
    qid = random.randrange(1, 0x10000)
    legacy_q = build_query([(name, TYPE_A)] + ([(SERVICE, TYPE_PTR)] if service else []), qid)
    group_qs = [build_query([(name, TYPE_A)])] + ([build_query([(SERVICE, TYPE_PTR)])] if service else [])
    socks: list[tuple[socket.socket, str]] = []   # (socket, what it is for the `via` text)
    sends: list[tuple[socket.socket, bytes, tuple[str, int], str | None]] = []
    try:
        if targets is not None:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("127.0.0.1" if all(t[0].startswith("127.") for t in targets) else "0.0.0.0", 0))
            socks.append((s, "unicast"))
            sends += [(s, legacy_q, t, None) for t in targets]
        else:
            ifaces = interfaces()
            if not ifaces:
                raise MdnsError("no network interface that can multicast is up")
            for ip in ifaces:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                try:
                    s.bind((ip, 0))
                    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(ip))
                    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
                except OSError:
                    s.close()
                    continue
                socks.append((s, f"unicast on {ip}"))
                sends.append((s, legacy_q, (GROUP, PORT), None))
            # port 5353 and the group: the answers of responders that only multicast
            m = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                _reuse(m)
                m.bind(("", PORT))
                m.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
                joined = 0
                for ip in ifaces:
                    try:
                        m.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                     socket.inet_aton(GROUP) + socket.inet_aton(ip))
                        joined += 1
                    except OSError as e:
                        if e.errno != errno.EADDRINUSE:  # already a member
                            continue
                        joined += 1
                if joined:
                    socks.append((m, "multicast"))
                    sends += [(m, q, (GROUP, PORT), ip) for ip in ifaces for q in group_qs]
                else:
                    m.close()
            except OSError:
                m.close()  # port 5353 taken without sharing: the unicast queries only
            if not socks:
                raise MdnsError("cannot open a UDP socket for mDNS")

        def send_all() -> None:
            for s, q, to, ifip in sends:
                try:
                    if ifip is not None:
                        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(ifip))
                    s.sendto(q, to)
                except OSError:
                    pass  # that interface cannot reach the group: the others may

        start = time.monotonic()
        send_all()
        resent = False
        settle_end: float | None = None
        by_fd = {s.fileno(): (s, via) for s, via in socks}
        while True:
            now = time.monotonic()
            end = start + timeout if settle_end is None else min(start + timeout, settle_end)
            if now >= end or col.complete():
                break
            if not resent and now - start >= RESEND_S and not col.results():
                send_all()
                resent = True
            wait = min(end, start + RESEND_S) - now if not resent else end - now
            ready, _, _ = select.select(list(by_fd), [], [], max(0.0, wait))
            for fd in ready:
                s, via = by_fd[fd]
                try:
                    data, src = s.recvfrom(9000)
                except OSError:
                    continue
                col.feed(data, f"{via} from {src[0]}", (src[0], src[1]))
            if settle_end is None and any(f.name == col.name for f in col.results()):
                settle_end = time.monotonic() + SETTLE_S
    finally:
        for s, _ in socks:
            s.close()
    if ignored is not None:
        ignored += col.ignored()
    return col.results()


def is_mdns_name(host: str) -> bool:
    return host.lower().rstrip(".").endswith(".local")


def find(name: str = DEFAULT_NAME, timeout: float = TIMEOUT_S) -> Found:
    """The Kilo answering for `name` (one with the service's build first),
    else the only one the service query found (a Kilo that had to take
    another name); MdnsError if none."""
    found = resolve(name, timeout)
    for f in found:
        if f.name == name.lower().rstrip("."):
            return f
    if len(found) == 1:
        return found[0]
    if found:
        raise MdnsError(f"no answer for {name}, but {len(found)} Kilos answered for {SERVICE}: "
                        + ", ".join(f"{f.name} ({f.address})" for f in found) + "; pick one with --host NAME")
    raise MdnsError(f"no answer for {name} within {timeout:g} s (mDNS on "
                    + (", ".join(interfaces()) or "no interface") + "): is Kilo on, his network cable in, and "
                    "this machine on the same network? Otherwise give his address: --host IP")


def host_address(arg: str | None, timeout: float = TIMEOUT_S, announce: bool = True) -> str:
    """--host's value as an address HostLink.connect takes: a .local name (or
    none at all: kilo.local) is looked up by mDNS and the address found is
    printed to stderr; anything else is returned as it is."""
    if arg is not None:
        host, sep, port = arg.rpartition(":")
        if not sep or not port.isdigit():
            host, port = arg, ""
        if not is_mdns_name(host):
            return arg
        name = host
    else:
        name, port = DEFAULT_NAME, ""
    f = find(name, timeout)
    addr = f.address + (f":{port}" if port else (f":{f.port}" if f.port not in (None, 5300) else ""))
    if announce:
        what = "no --host: found" if arg is None else "found"
        build = f", build {f.build}" if f.build else ""
        print(f"kilo: {what} {f.name} at {addr}{build}", file=sys.stderr, flush=True)
    return addr

