"""A version-1 client against a version-2 mind (kernel/host/test.sh): one
plaintext STATUS_REQ frame, no hello. The mind must close the connection
and answer nothing.

    python kernel/host/tests/plaintext_client.py 127.0.0.1:PORT
"""

import socket
import sys

from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P

host, port = sys.argv[1].rsplit(":", 1)
s = socket.create_connection((host, int(port)), timeout=5)
s.sendall(C.encode_host_message(P.HostStatusReq(), 7))
got = b""
while True:
    chunk = s.recv(4096)  # the mind closes at once; 5 s would be a mind that waits
    if not chunk:
        break
    got += chunk
assert got == b"", f"the mind answered a plaintext frame: {got!r}"
print("   a plaintext STATUS_REQ: the connection closed, nothing answered")
