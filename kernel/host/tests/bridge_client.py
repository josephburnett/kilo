#!/usr/bin/env python3
"""The raw bridge end to end (kernel/host/test.sh): through kilo-mind-host's
host channel, open the bridge and run the Python body link (kilo.link's
SerialLink and BodyClient over kilo.hostlink.BridgeTransport) against the
body behind the mind: HELLO, TELEMETRY and a STATS query. Then close the
bridge and check that the mind's own link took the body back.

    bridge_client.py 127.0.0.1:PORT

Prints one line per step; exit 0 on success.
"""
from __future__ import annotations

import sys
import time

from kilo.body.client import BodyClient
from kilo.hostlink import BridgeTransport, HostLink
from kilo.link.serial_link import SerialLink
from kilo.protocol import kilo_protocol as P


def main() -> int:
    addr = sys.argv[1]
    hl = HostLink.connect(addr)
    link = SerialLink(lambda: BridgeTransport(hl), port_name=f"bridge:{addr}")
    link.mark_alive()
    link.start()
    client = BodyClient(link)
    try:
        hello = client.wait_ready(3.0)
        print(f"bridge: HELLO proto {hello.proto_major}.{hello.proto_minor} build {hello.fw_build:08x}", flush=True)
        st = hl.status()
        assert st.state & P.HostState.BRIDGE, "STATUS does not say BRIDGE"
        stats = client.stats()
        print(f"bridge: STATS uptime {stats.uptime_s} s, telemetry here {client.state.telemetry_count}", flush=True)
        assert client.state.telemetry_count > 0
    finally:
        link.stop()  # closes the transport: BRIDGE_CLOSE
    hl.release_control()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        st = hl.status()
        if not st.state & P.HostState.BRIDGE and st.state & P.HostState.CONTACT:
            print(f"bridge closed: the mind's link has the body again (reconnects {st.link.reconnects})", flush=True)
            hl.close()
            return 0
        time.sleep(0.1)
    print(f"FAILED: the mind's link did not take the body back (state {st.state:#x})", flush=True)
    hl.close()
    return 1


if __name__ == "__main__":
    sys.exit(main())
