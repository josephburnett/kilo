"""Known-answer vectors for the C++ Noise layer (kernel/mind/net/noise.*),
from implementations that are not it: the Python `noiseprotocol` package
(an independent Noise implementation over `cryptography`), Python's own
hmac/hashlib for HMAC-BLAKE2b and HKDF, and the published cacophony and snow
vectors (noise_published_vectors.json, checked here against noiseprotocol
too). kernel/host/tests/test_noise.cpp replays them byte for byte.

    .venv/bin/python kernel/host/tests/gen_noise_vectors.py > kernel/host/tests/noise_vectors.txt

Line format (space separated, hex; "-" is empty):

    noise NAME PROLOGUE INIT_STATIC_PRIV INIT_EPHEMERAL_PRIV RESP_EPHEMERAL_PRIV HASH
                              a KN handshake (HASH "-": the source gives none)
    msg I|R PAYLOAD CIPHERTEXT    the next message, from the initiator (I) or the responder (R):
                              the first two are the handshake's, the rest transport messages
    end
    hmac KEY DATA OUT         HMAC-BLAKE2b(KEY, DATA) (Python hmac + hashlib.blake2b)
    hkdf CK IKM OUT1 OUT2     Noise's HKDF over it, two outputs
    fp PUBLIC FINGERPRINT     the first 8 bytes of BLAKE2b-512(PUBLIC) in hex (kilo key show, the mind's log)
"""

from __future__ import annotations

import hashlib
import hmac
import json
import sys
import warnings
from pathlib import Path

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import x25519
from noise.connection import Keypair, NoiseConnection

# noiseprotocol warns when an ephemeral key is given: here that is the point.
warnings.filterwarnings("ignore", "One of ephemeral keypairs is already set")

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "mind"))

from kilo.protocol import kilo_protocol as P  # noqa: E402

NAME = b"Noise_KN_25519_ChaChaPoly_BLAKE2b"
assert NAME.decode() == P.HOST_NOISE_PROTOCOL


def hx(b: bytes) -> str:
    return b.hex() if b else "-"


def det(label: str, n: int = 32) -> bytes:
    """Fixed bytes for a vector, from its label."""
    return hashlib.blake2b(label.encode(), digest_size=n).digest()


def public(priv: bytes) -> bytes:
    return x25519.X25519PrivateKey.from_private_bytes(priv).public_key().public_bytes(
        serialization.Encoding.Raw, serialization.PublicFormat.Raw)


def hello(pub: bytes) -> bytes:
    """The client's clear hello (spec [host]): magic, version, flags, reserved, key."""
    return P.HOST_HELLO_MAGIC.encode() + bytes([P.HOST_VERSION, 0, 0, 0]) + pub


def run(name: str, prologue: bytes, s: bytes, e_i: bytes, e_r: bytes, messages: list[tuple[str, bytes]]) -> list[str]:
    """One handshake and its transport messages through noiseprotocol, as lines."""
    i = NoiseConnection.from_name(NAME)
    r = NoiseConnection.from_name(NAME)
    i.set_as_initiator()
    r.set_as_responder()
    i.set_keypair_from_private_bytes(Keypair.STATIC, s)
    i.set_keypair_from_private_bytes(Keypair.EPHEMERAL, e_i)
    r.set_keypair_from_public_bytes(Keypair.REMOTE_STATIC, public(s))
    r.set_keypair_from_private_bytes(Keypair.EPHEMERAL, e_r)
    i.set_prologue(prologue)
    r.set_prologue(prologue)
    i.start_handshake()
    r.start_handshake()
    out = []
    for k, (who, payload) in enumerate(messages):
        a, b = (i, r) if who == "I" else (r, i)
        if k < 2:
            assert who == "IR"[k], "KN: -> e, then <- e, ee, se"
            ct = bytes(a.write_message(payload))
            assert bytes(b.read_message(ct)) == payload
        else:
            ct = a.encrypt(payload)
            assert b.decrypt(ct) == payload
        out.append(f"msg {who} {hx(payload)} {hx(ct)}")
    assert i.get_handshake_hash() == r.get_handshake_hash()
    head = f"noise {name} {hx(prologue)} {s.hex()} {e_i.hex()} {e_r.hex()} {i.get_handshake_hash().hex()}"
    return [head, *out, "end"], out


def published() -> list[str]:
    data = json.loads((HERE / "noise_published_vectors.json").read_text())
    lines = []
    for n, v in enumerate(data["vectors"]):
        assert v["protocol_name"] == NAME.decode()
        assert v["init_prologue"] == v["resp_prologue"]
        s = bytes.fromhex(v["init_static"])
        assert bytes.fromhex(v["resp_remote_static"]) == public(s)
        msgs = [("IR"[k % 2], bytes.fromhex(m["payload"])) for k, m in enumerate(v["messages"])]
        got, out = run(f"{v['source'].split('.')[0]}-{n}", bytes.fromhex(v["init_prologue"]), s,
                       bytes.fromhex(v["init_ephemeral"]), bytes.fromhex(v["resp_ephemeral"]), msgs)
        # noiseprotocol agrees with the published ciphertexts (and hash): the vectors are what they say.
        for line, m in zip(out, v["messages"]):
            assert line.split()[3] == m["ciphertext"], (v["source"], line)
        if "handshake_hash" in v:
            assert got[0].split()[6] == v["handshake_hash"]
        else:
            head = got[0].split()
            got[0] = " ".join(head[:6] + ["-"])
        lines += got
    return lines


def ours() -> list[str]:
    """The handshake as the host channel runs it: our prologue (with a hello),
    empty handshake payloads, then records of up to host.record_max bytes both
    ways, several in a row one way too."""
    lines = []
    for n in range(3):
        s = det(f"static {n}")
        prologue = P.HOST_NOISE_PROLOGUE.encode() + hello(public(s))
        msgs = [("I", b""), ("R", b"")]
        sizes = [0, 1, 6, 17, 246, 1000, P.HOST_RECORD_MAX]
        for k, size in enumerate(sizes):
            msgs.append(("I" if (k + n) % 3 else "R", det(f"payload {n} {k}", 64) * (size // 64) +
                         det(f"tail {n} {k}", 64)[:size % 64]))
        msgs += [("R", det(f"burst {n} {k}", 40)) for k in range(4)]
        msgs += [("I", b""), ("I", det(f"after {n}", 7))]
        got, _ = run(f"kilo-{n}", prologue, s, det(f"init ephemeral {n}"), det(f"resp ephemeral {n}"), msgs)
        lines += got
    return lines


def macs() -> list[str]:
    lines = []
    for klen, dlen in [(0, 0), (1, 3), (32, 0), (64, 1), (64, 64), (128, 200), (129, 5), (300, 1000)]:
        key, data = det(f"k {klen}", 64) * (klen // 64 + 1), det(f"d {dlen}", 64) * (dlen // 64 + 1)
        key, data = key[:klen], data[:dlen]
        lines.append(f"hmac {hx(key)} {hx(data)} {hmac.new(key, data, hashlib.blake2b).hexdigest()}")
    for ilen in [0, 32, 64, 100]:
        ck, ikm = det(f"ck {ilen}", 64), (det(f"ikm {ilen}", 64) * 2)[:ilen]
        temp = hmac.new(ck, ikm, hashlib.blake2b).digest()
        o1 = hmac.new(temp, b"\x01", hashlib.blake2b).digest()
        o2 = hmac.new(temp, o1 + b"\x02", hashlib.blake2b).digest()
        lines.append(f"hkdf {ck.hex()} {hx(ikm)} {o1.hex()} {o2.hex()}")
    for n in range(4):
        pub = public(det(f"fp {n}"))
        lines.append(f"fp {pub.hex()} {hashlib.blake2b(pub).hexdigest()[:16]}")
    return lines


def main() -> int:
    print("# Generated by kernel/host/tests/gen_noise_vectors.py: do not edit.")
    for line in published() + ours() + macs():
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
