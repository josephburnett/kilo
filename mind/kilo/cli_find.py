"""`kilo find`: Kilo's address on the network, found by name (kilo.mdns).

    kilo find [--name kilo.local] [--timeout S] [--json] [--ask IP[:PORT]]

Prints each Kilo that answered: its name, address, the laptop service's
port and the mind's build. The build comes from the answer (the mind's TXT
record); a mind that answers only for its name (a kernel before this
command) has no build in it, so then `kilo find` asks the mind itself over
the host channel (STATUS: needs this laptop's key, as every --host command).
Finding gives only an address; the connection's handshake still checks
that the mind knows this laptop's key.

--ask sends the query to one address (unicast) instead of the whole
network: for a network that drops multicast, or the tests.

Other devices on the network answer mDNS too (each other's questions, or
announcements of their own services); only answers that name kilo.local or
the _kilo._tcp service count. `kilo find -v` lists the ones left out.
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any

from kilo import mdns

EXIT_OK, EXIT_ERR = 0, 1


def add_parser(sub: Any) -> None:
    s = sub.add_parser("find", help="find Kilo on the network by name (mDNS): his address and build",
                       description="Ask the network who is kilo.local (mDNS, the way .local names are found) and "
                                   "print the address and build of each Kilo that answers. Every --host command "
                                   "also takes --host kilo.local, and the host-only commands (walks, reboot, "
                                   "update) find Kilo this way when --host is left out.")
    s.add_argument("--name", default=mdns.DEFAULT_NAME, help="the name to look for (default kilo.local)")
    s.add_argument("--timeout", type=float, default=mdns.TIMEOUT_S, dest="find_timeout",
                   help=f"seconds to wait for answers (default {mdns.TIMEOUT_S:g})")
    s.add_argument("--json", action="store_true")
    s.add_argument("-v", "--verbose", action="count", default=0, dest="find_verbose",
                   help="also list the answers left out (other devices on the network)")
    s.add_argument("--ask", metavar="IP[:PORT]", default=None,
                   help="ask this address directly (unicast, port 5353 unless given) instead of the network")


def _build_over_host(f: mdns.Found, key: str | None) -> str | None:
    from kilo.hostlink import HostError, HostLink

    try:
        with HostLink.connect(f.host_arg, timeout=2.0, key=key) as hl:
            return f"{hl.status().mind_build:08x}"
    except (HostError, OSError, ValueError) as e:
        print(f"kilo find: {f.address}: no build in the answer, and STATUS failed: {e}", file=sys.stderr)
        return None


def cmd_find(args: argparse.Namespace) -> int:
    name = args.name if mdns.is_mdns_name(args.name) else args.name + ".local"
    targets = None
    if args.ask:
        host, sep, port = args.ask.rpartition(":")
        targets = [(host, int(port))] if sep and port.isdigit() else [(args.ask, mdns.PORT)]
    ignored: list[str] = []
    try:
        found = mdns.resolve(name, args.find_timeout, targets=targets, ignored=ignored)
    except mdns.MdnsError as e:
        print(f"kilo find: {e}", file=sys.stderr)
        return EXIT_ERR
    if getattr(args, "verbose", 0) or getattr(args, "find_verbose", 0):  # other devices' answers (mDNS is heard by everyone): only with -v
        for line in ignored:
            print(f"kilo find: ignored {line}", file=sys.stderr)
    if not found:
        print(f"kilo find: no answer for {name} within {args.find_timeout:g} s"
              + (f" from {args.ask}" if args.ask else
                 f" (asked on {', '.join(mdns.interfaces()) or 'no interface'})")
              + ": is Kilo on, his network cable in, and this machine on the same network?", file=sys.stderr)
        return EXIT_ERR
    for f in found:
        if f.build is None:
            f.build = _build_over_host(f, getattr(args, "key", None))
    if args.json:
        print(json.dumps([{"name": f.name, "address": f.address, "port": f.port, "build": f.build,
                           "host": f.host_arg, "via": f.via} for f in found], indent=1))
        return EXIT_OK
    for f in found:
        port = f"port {f.port}" if f.port else "port ? (no service answer)"
        print(f"{f.name}  {f.address}  {port}  build {f.build or '?'}  ({f.via})")
    if len(found) == 1:
        print(f"use: kilo status --host {found[0].name}   (or --host {found[0].host_arg})")
    return EXIT_OK
