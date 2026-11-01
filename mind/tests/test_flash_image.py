"""Flashing the body through a C++ mind (docs/fast-boot.md §6, P4), the
laptop's side: kilo.tools.body_image (Intel HEX -> body.bin, the manifest),
`kilo flash --sd`, and `kilo flash --host` with HostLink.flash against a
fake mind. The mind's side is kernel/host/tests/test_flash.cpp; both
together, with a fake optiboot, are in kernel/host/test.sh."""

from __future__ import annotations

import json
import random
import shutil
import socket
import subprocess
import threading
import zlib
from pathlib import Path
from typing import Any

import pytest

from kilo import cli
from kilo.hostlink import HostLink, HostRefused
from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P
from kilo.tools import body_image as BI
from kilo.tools.flash import FlashError, describe_flash_status, flash_sd, looks_like_boot_card


def ihex(data: bytes, base: int = 0, width: int = 16, ext: int | None = None) -> str:
    """Intel HEX of data at base (type 00 records, optional 04 record first, 01 EOF)."""
    lines = []

    def rec(kind: int, addr: int, payload: bytes) -> str:
        r = bytes([len(payload), addr >> 8 & 0xFF, addr & 0xFF, kind]) + payload
        return ":" + (r + bytes([-sum(r) & 0xFF])).hex().upper()

    if ext is not None:
        lines.append(rec(0x04, 0, bytes([ext >> 8, ext & 0xFF])))
    for off in range(0, len(data), width):
        lines.append(rec(0x00, base + off, data[off:off + width]))
    lines.append(rec(0x01, 0, b""))
    return "\n".join(lines) + "\n"


# ---- body_image -------------------------------------------------------------------------

def test_hex_to_bin():
    data = random.Random(1).randbytes(1000)
    assert BI.hex_to_bin(ihex(data)) == data
    assert BI.hex_to_bin(ihex(data, width=32)) == data
    # a gap reads as erased flash; an extended linear address of 0 changes nothing
    assert BI.hex_to_bin(ihex(b"\x01\x02", base=4, ext=0)) == b"\xff" * 4 + b"\x01\x02"
    bad = ihex(b"\x01\x02\x03").splitlines()
    with pytest.raises(BI.ImageError, match="checksum"):
        BI.hex_to_bin(bad[0][:-2] + "00\n")
    with pytest.raises(BI.ImageError, match="no ':'"):
        BI.hex_to_bin("0300000001020300\n")
    with pytest.raises(BI.ImageError, match="no data"):
        BI.hex_to_bin(":00000001FF\n")


AVR_OBJCOPY = next(iter(sorted(Path.home().glob(".arduino15/packages/arduino/tools/avr-gcc/*/bin/avr-objcopy"))),
                   None) or shutil.which("avr-objcopy")


@pytest.mark.skipif(AVR_OBJCOPY is None, reason="no avr-objcopy (arduino-cli's avr-gcc)")
def test_hex_to_bin_matches_avr_objcopy(tmp_path):
    """The same bytes as `avr-objcopy -I ihex -O binary`, which the design names."""
    data = random.Random(2).randbytes(31234)
    (tmp_path / "in.bin").write_bytes(data)
    subprocess.run([AVR_OBJCOPY, "-I", "binary", "-O", "ihex", "in.bin", "body.hex"], cwd=tmp_path, check=True)
    subprocess.run([AVR_OBJCOPY, "-I", "ihex", "-O", "binary", "body.hex", "out.bin"], cwd=tmp_path, check=True)
    ours = BI.hex_to_bin((tmp_path / "body.hex").read_text())
    assert ours == (tmp_path / "out.bin").read_bytes() == data


def test_manifest_round_trip_and_checks(tmp_path):
    data = random.Random(3).randbytes(5000)
    m = BI.make_manifest(data, 0x3D441EC0, commit="3d441ec0" + "0" * 32)
    d = json.loads(m.to_json())
    assert d == {"format": 1, "build": "3d441ec0", "size": 5000, "crc32": f"{zlib.crc32(data):08x}",
                 "commit": "3d441ec0" + "0" * 32}
    assert BI.Manifest.from_json(m.to_json()) == m
    # what the mind writes back to SD (kernel/mind/flash/image.cpp format_manifest)
    assert BI.Manifest.from_json('{"format": 1, "build": "deadbeef", "size": 7, "crc32": "0000000a"}\n') == \
        BI.Manifest(0xDEADBEEF, 7, 10)
    for text, why in [("[]", "format"), ('{"format": 2}', "format"), ('{"format": 1, "build": "1"}', "no size, crc32"),
                      ('{"format": 1, "build": "1", "size": 1, "crc32": "00000000"}', "8 hex"),
                      ('{"format": 1, "build": 5, "crc32": "00000000", "size": 1}', "8 hex"),
                      ('{"format": 1, "build": "00000001", "crc32": "00000000"}', "no size"),
                      ('{"format": 1, "build": "00000001", "crc32": "00000000", "size": "1"}', "size must"), ("{", "not JSON")]:
        with pytest.raises(BI.ImageError, match=why):
            BI.Manifest.from_json(text)
    BI.check_image(data, m)
    for bad, why in [(BI.Manifest(1, 5000, m.crc32 ^ 1), "crc32 mismatch"), (BI.Manifest(0, 5000, m.crc32), "build 0"),
                     (BI.Manifest(1, 4999, m.crc32), "the manifest says 4999"),
                     (BI.Manifest(1, BI.MAX_IMAGE + 1, 0), "1..32256")]:
        with pytest.raises(BI.ImageError, match=why):
            BI.check_image(data, bad)
    b, j = BI.write_image(tmp_path, data, m)
    assert (b.name, j.name) == ("body.bin", "body.json")
    assert BI.read_image(b) == (data, m)
    j.write_text(BI.Manifest(1, 5000, 0).to_json())
    with pytest.raises(BI.ImageError, match="crc32 mismatch"):
        BI.read_image(b)


# ---- kilo flash --sd -----------------------------------------------------------------------

def test_flash_sd(tmp_path, capsys):
    data = random.Random(4).randbytes(3000)
    img = tmp_path / "build"
    BI.write_image(img, data, BI.make_manifest(data, 0x12345678))
    card = tmp_path / "card"
    card.mkdir()
    assert "has no kernel7.img or config.txt" in looks_like_boot_card(card, force_dir=True)
    (card / "kernel7.img").write_bytes(b"k")
    assert "has no config.txt" in looks_like_boot_card(card, force_dir=True)
    (card / "config.txt").write_text("")
    assert looks_like_boot_card(card, force_dir=True) is None
    assert "not a mount point" in looks_like_boot_card(card)  # tmp_path is no FAT mount
    assert "not a directory" in looks_like_boot_card(tmp_path / "nope", force_dir=True)

    rc = cli.main(["flash", "--sd", str(card), "--force-dir", "--image", str(img / "body.bin")])
    assert rc == 0, capsys.readouterr().err
    assert (card / "body.bin").read_bytes() == data
    assert BI.Manifest.from_json((card / "body.json").read_text()).build == 0x12345678
    assert "build 12345678" in capsys.readouterr().out
    assert cli.main(["flash", "--sd", str(card), "--image", str(img / "body.bin")]) == 1
    assert "not a mount point" in capsys.readouterr().err
    with pytest.raises(FlashError, match="not a Kilo boot card"):
        flash_sd(tmp_path / "build", data, BI.make_manifest(data, 1), force_dir=True)


def test_flash_cli_refusals(capsys):
    assert cli.main(["flash", "--rollback"]) == 1
    assert "not yet" in capsys.readouterr().err
    assert cli.main(["flash", "--persist"]) == 1
    assert "for --host or --sd" in capsys.readouterr().err
    assert cli.main(["flash", "--host", "127.0.0.1:1", "--legacy"]) == 1
    assert "not with --legacy" in capsys.readouterr().err


# ---- kilo flash --host --------------------------------------------------------------------

class FlashMind:
    """A host channel server doing what HostServer does with the flash messages:
    controller check, the image in order, CRC at COMMIT, then statuses to DONE
    (or FAILED with `fail_with`)."""

    def __init__(self, fail_with: int | None = None, fw_build: int | None = None):
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(2)
        self.addr = f"127.0.0.1:{self.srv.getsockname()[1]}"
        self.fail_with = fail_with
        self.fw_build = fw_build
        self.image = bytearray()
        self.begin: Any = None
        self.got: list[str] = []
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self) -> None:
        while True:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            threading.Thread(target=self._serve, args=(c,), daemon=True).start()

    def _serve(self, c: socket.socket) -> None:
        dec = C.HostStreamDecoder()
        controller = False
        out_seq = 0

        def send(msg: Any, seq: int) -> None:
            c.sendall(C.encode_host_message(msg, seq))

        def result(seq: int, req: int, code: int = 0, text: bytes = b"") -> None:
            send(P.HostResult(request=req, code=code, text=text), seq)

        def status(**kw: Any) -> None:
            nonlocal out_seq
            send(P.HostFlashStatus(**kw), out_seq)
            out_seq += 1

        try:
            while True:
                data = c.recv(65536)
                if not data:
                    return
                for f in dec.feed(data):
                    m = P.decode_host_payload(f.type, f.payload)
                    self.got.append(m.NAME)
                    if isinstance(m, P.HostControl):
                        controller = bool(m.take)
                        result(f.seq, m.ID)
                    elif isinstance(m, P.HostStatusReq):
                        st = P.HostStatus(state=int(P.HostState.HAS_HELLO | P.HostState.BODY_ALIVE))
                        st.hello.fw_build = self.fw_build if self.fw_build is not None else self.begin.build
                        st.hello.reset_cause = int(P.ResetCause.FLASHED)
                        send(st, f.seq)
                    elif isinstance(m, P.HostFlashBegin):
                        if not controller:
                            result(f.seq, m.ID, int(P.HostResultCode.NOT_CONTROLLER))
                            continue
                        self.begin, self.image = m, bytearray()
                        result(f.seq, m.ID)
                    elif isinstance(m, P.HostFlashData):
                        assert m.offset == len(self.image)
                        self.image += m.data
                    elif isinstance(m, P.HostFlashCommit):
                        b = self.begin
                        pages = (b.image_size + 127) // 128
                        if zlib.crc32(self.image) != b.crc32:
                            status(phase=int(P.FlashPhase.FAILED), error=int(P.FlashError.CRC_MISMATCH),
                                   received=len(self.image))
                            result(f.seq, m.ID, int(P.HostResultCode.IMAGE_BAD), b"CRC_MISMATCH")
                            continue
                        result(f.seq, m.ID)
                        common = dict(attempt=1, pages=pages, received=len(self.image), build=b.build)
                        status(phase=int(P.FlashPhase.RESETTING), **common)
                        if self.fail_with is not None:
                            status(phase=int(P.FlashPhase.FAILED), error=self.fail_with, stk_cmd=0x30,
                                   **{**common, "attempt": 2})
                            continue
                        for ph in (P.FlashPhase.PROGRAMMING, P.FlashPhase.VERIFYING):
                            for pg in range(0, pages + 1, 7):
                                status(phase=int(ph), page=pg, **common)
                        status(phase=int(P.FlashPhase.DONE), page=pages, fw_build=b.build,
                               reset_cause=int(P.ResetCause.FLASHED), **common)
                    elif m.KIND == P.Kind.COMMAND:
                        result(f.seq, m.ID, int(P.HostResultCode.UNSUPPORTED))
        except OSError:
            return

    def close(self) -> None:
        self.srv.close()


@pytest.fixture
def image(tmp_path):
    data = random.Random(5).randbytes(31234)
    m = BI.make_manifest(data, 0x3D441EC0)
    BI.write_image(tmp_path, data, m)
    return tmp_path / "body.bin", data, m


def test_hostlink_flash(image):
    _, data, m = image
    mind = FlashMind()
    try:
        with HostLink.connect(mind.addr) as hl:
            seen: list[Any] = []
            st = hl.flash(data, m.build, m.crc32, persist=True, on_status=seen.append)
            assert st.phase == P.FlashPhase.DONE and st.fw_build == m.build
            assert bytes(mind.image) == data and mind.begin.flags == int(P.FlashFlags.PERSIST)
            assert seen[0].phase == P.FlashPhase.RESETTING and seen[-1] == st
            with pytest.raises(HostRefused) as ei:
                hl.flash(data, m.build, m.crc32 ^ 1, on_status=seen.append)
            assert ei.value.code == P.HostResultCode.IMAGE_BAD
            assert seen[-1].phase == P.FlashPhase.FAILED and seen[-1].error == P.FlashError.CRC_MISMATCH
    finally:
        mind.close()


def test_cli_flash_host(image, capsys):
    path, data, m = image
    mind = FlashMind()
    try:
        assert cli.main(["flash", "--host", mind.addr, "--image", str(path), "--persist"]) == 0
        out = capsys.readouterr().out
        assert "flash: programming done, 245 pages" in out and "flash: verifying done, 245 pages" in out
        assert "verified: HELLO build 3d441ec0 reset FLASHED; the mind keeps the image on SD" in out
        assert mind.got[-1] == "CONTROL" and bytes(mind.image) == data  # control released at the end
    finally:
        mind.close()


def test_cli_flash_host_failures(image, tmp_path, capsys):
    path, data, m = image
    mind = FlashMind(fail_with=int(P.FlashError.SYNC_TIMEOUT))
    try:
        assert cli.main(["--host", mind.addr, "flash", "--image", str(path)]) == 1
        assert "FAILED: failed (attempt 2): SYNC_TIMEOUT (STK500 command 30)" in capsys.readouterr().err
    finally:
        mind.close()
    mind = FlashMind(fw_build=0x11111111)  # DONE, but STATUS reports another build
    try:
        assert cli.main(["--host", mind.addr, "flash", "--image", str(path)]) == 1
        assert "verify FAILED: the mind's link reports build 11111111" in capsys.readouterr().err
    finally:
        mind.close()
    # A manifest that does not match: refused before connecting.
    (tmp_path / "bad.json").write_text(BI.Manifest(m.build, m.size, m.crc32 ^ 1).to_json())
    assert cli.main(["--host", "127.0.0.1:1", "flash", "--image", str(path), "--manifest",
                     str(tmp_path / "bad.json")]) == 1
    assert "crc32 mismatch" in capsys.readouterr().err


def test_describe_flash_status():
    st = P.HostFlashStatus(phase=int(P.FlashPhase.PROGRAMMING), page=3, pages=245, attempt=2)
    assert describe_flash_status(st) == "programming 3/245 (attempt 2)"
    st = P.HostFlashStatus(phase=int(P.FlashPhase.DONE), fw_build=0xAB, reset_cause=int(P.ResetCause.FLASHED))
    assert describe_flash_status(st) == "done; HELLO build 000000ab reset FLASHED"
