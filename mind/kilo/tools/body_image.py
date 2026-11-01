"""The body image the C++ mind flashes (docs/fast-boot.md §6, phase P4).

Two files, next to each other in the build directory and on the SD card:

``body.bin``
    The firmware's flash bytes from address 0, exactly what
    ``avr-objcopy -O binary`` makes of the build: the mind never parses
    Intel HEX. Made here from arduino-cli's ``body.ino.hex`` (``hex_to_bin``),
    so no AVR binutils are needed; gaps (there are none in an AVR build)
    would read 0xFF, as erased flash.
``body.json``
    The manifest, one flat JSON object::

        {"format": 1, "build": "3d441ec0", "size": 31234, "crc32": "cbf43926", "commit": "3d441ec0..."}

    ``build`` is the ``-DKILO_BUILD`` value (8 hex digits: the body's
    ``HELLO.fw_build`` once it runs the image), ``size`` the bytes of
    body.bin, ``crc32`` its ``zlib.crc32``. ``commit`` (optional) is for
    people. JSON rather than TOML because Python writes it with the
    standard library (tomllib only reads) and the mind's parser for one flat
    object is a few dozen lines (kernel/mind/flash/image.cpp,
    ``parse_manifest``), which also writes the same format back to SD.

The image must fit the ATmega328P's flash less optiboot: P.HOST_FLASH_MAX_IMAGE
(32256) bytes.
"""

from __future__ import annotations

import json
import zlib
from dataclasses import dataclass
from pathlib import Path

from kilo.protocol import kilo_protocol as P

IMAGE_FILE = "body.bin"
MANIFEST_FILE = "body.json"
MANIFEST_FORMAT = 1
MAX_IMAGE = P.HOST_FLASH_MAX_IMAGE


class ImageError(Exception):
    pass


@dataclass
class Manifest:
    build: int
    size: int
    crc32: int
    commit: str | None = None

    def to_json(self) -> str:
        d: dict[str, object] = {"format": MANIFEST_FORMAT, "build": f"{self.build:08x}", "size": self.size,
                                "crc32": f"{self.crc32:08x}"}
        if self.commit:
            d["commit"] = self.commit
        return json.dumps(d, indent=2) + "\n"

    @classmethod
    def from_json(cls, text: str) -> "Manifest":
        try:
            d = json.loads(text)
        except json.JSONDecodeError as e:
            raise ImageError(f"manifest: not JSON: {e}") from None
        if not isinstance(d, dict) or d.get("format") != MANIFEST_FORMAT:
            raise ImageError(f"manifest: not format {MANIFEST_FORMAT}")
        missing = [k for k in ("build", "size", "crc32") if k not in d]
        if missing:
            raise ImageError(f"manifest: no {', '.join(missing)}")
        try:
            build, crc = d["build"], d["crc32"]
            if not (isinstance(build, str) and isinstance(crc, str) and len(build) == len(crc) == 8):
                raise ValueError("build and crc32 must be 8 hex digits in a string")
            size = d["size"]
            if not isinstance(size, int) or isinstance(size, bool):
                raise ValueError("size must be a number")
            return cls(build=int(build, 16), size=size, crc32=int(crc, 16), commit=d.get("commit"))
        except ValueError as e:
            raise ImageError(f"manifest: {e}") from None


def make_manifest(data: bytes, build: int, commit: str | None = None) -> Manifest:
    return Manifest(build=build, size=len(data), crc32=zlib.crc32(data), commit=commit)


def check_image(data: bytes, m: Manifest) -> None:
    """The checks the mind makes (FLASH_BEGIN, FLASH_COMMIT), before anything is sent."""
    if not 0 < m.size <= MAX_IMAGE:
        raise ImageError(f"size {m.size}: must be 1..{MAX_IMAGE} bytes (32 KB less optiboot)")
    if m.build == 0:
        raise ImageError("build 0: the image has no build hash")
    if len(data) != m.size:
        raise ImageError(f"{IMAGE_FILE} is {len(data)} bytes, the manifest says {m.size}")
    crc = zlib.crc32(data)
    if crc != m.crc32:
        raise ImageError(f"crc32 mismatch: {IMAGE_FILE} is {crc:08x}, the manifest says {m.crc32:08x}")


def hex_to_bin(text: str) -> bytes:
    """Intel HEX (record types 00 data, 01 EOF, 02/04 extended address) to
    the flash bytes from address 0, as ``avr-objcopy -I ihex -O binary``
    (gaps read 0xFF)."""
    mem: dict[int, int] = {}
    base = 0
    for n, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ImageError(f"hex line {n}: no ':'")
        try:
            rec = bytes.fromhex(line[1:])
        except ValueError:
            raise ImageError(f"hex line {n}: not hex") from None
        if len(rec) < 5 or len(rec) != rec[0] + 5:
            raise ImageError(f"hex line {n}: bad length")
        if sum(rec) & 0xFF:
            raise ImageError(f"hex line {n}: bad checksum")
        count, addr, kind, payload = rec[0], rec[1] << 8 | rec[2], rec[3], rec[4:4 + rec[0]]
        if kind == 0x00:
            for i, b in enumerate(payload):
                mem[base + addr + i] = b
        elif kind == 0x01:
            break
        elif kind == 0x02 and count == 2:
            base = (payload[0] << 8 | payload[1]) << 4
        elif kind == 0x04 and count == 2:
            base = (payload[0] << 8 | payload[1]) << 16
        elif kind in (0x03, 0x05):
            continue  # start address: not flash content
        else:
            raise ImageError(f"hex line {n}: record type {kind:02x}")
    if not mem:
        raise ImageError("hex: no data")
    out = bytearray(b"\xff" * (max(mem) + 1))
    for a, b in mem.items():
        out[a] = b
    return bytes(out)


def write_image(out_dir: Path, data: bytes, m: Manifest) -> tuple[Path, Path]:
    """body.bin and body.json into out_dir. The old manifest goes first and
    the new one last, so a reader never pairs a manifest with half an image
    (the mind also checks the CRC before it flashes)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    bin_path, man_path = out_dir / IMAGE_FILE, out_dir / MANIFEST_FILE
    man_path.unlink(missing_ok=True)
    bin_path.write_bytes(data)
    man_path.write_text(m.to_json())
    return bin_path, man_path


def read_image(image: Path, manifest: Path | None = None) -> tuple[bytes, Manifest]:
    """body.bin and its manifest (default: body.json next to it), checked."""
    manifest = manifest or image.with_name(MANIFEST_FILE)
    try:
        data = image.read_bytes()
        m = Manifest.from_json(manifest.read_text())
    except OSError as e:
        raise ImageError(str(e)) from None
    check_image(data, m)
    return data, m
