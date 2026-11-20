"""`kilo card PATH` (kilo.cli_card) against a temporary directory standing for
Kilo's SD card: which folders it takes, the kernel and the body written with
kernel7.old kept and body-result.txt removed, the checks before (the marker,
this laptop's key, build options), the builds it runs, --status reading what
the mind wrote (kernel/mind/flash/cardflash.h), and --fetch copying the
flight recorder's files in the network fetch's layout. The mind's side is
kernel/host/tests/test_flash.cpp; both together in kernel/host/test.sh."""

from __future__ import annotations

import random
import subprocess
from pathlib import Path
from typing import Any

import pytest

from kilo import cli, cli_card
from kilo.logs import flight
from kilo.protocol import kilo_protocol as P
from kilo.tools import body_image as BI
from test_flight import T0, blocks, log, trick_records

MARK = P.HOST_UPDATE_MARKER.encode()


@pytest.fixture
def no_key(tmp_path, monkeypatch):
    monkeypatch.setenv("KILO_HOST_KEY", str(tmp_path / "no-such-key"))


def make_card(root: Path, kernel: bytes = b"old " + MARK + b"\0" * 64) -> Path:
    card = root / "card"
    card.mkdir()
    (card / "config.txt").write_text("kernel=kernel7.img\n")
    (card / "kernel7.img").write_bytes(kernel)
    return card


def body_files(root: Path, build: int = 0x3D441EC0, size: int = 3000, seed: int = 1) -> tuple[Path, BI.Manifest]:
    data = random.Random(seed).randbytes(size)
    m = BI.make_manifest(data, build)
    d = root / f"body-{seed}"
    BI.write_image(d, data, m)
    return d / "body.bin", m


def kcard(*argv: str) -> int:
    return cli.main(["card", *argv])


# ---- which folder ----------------------------------------------------------------------------

def test_check_card_refusals(tmp_path):
    with pytest.raises(cli_card.CardError, match="not a folder"):
        cli_card.check_card(tmp_path / "nope", force=True)
    bare = tmp_path / "bare"
    bare.mkdir()
    with pytest.raises(cli_card.CardError, match="has no kernel7.img or config.txt"):
        cli_card.check_card(bare, force=False, force_dir=True)
    card = make_card(tmp_path)
    assert cli_card.check_card(card, force=False, force_dir=True) == []
    # Not mounted: a plain folder is not the card's partition.
    with pytest.raises(cli_card.CardError, match="not a mount point|not the top of a mounted drive"):
        cli_card.check_card(card, force=False)
    # A Raspberry Pi OS card: its kernel7.img is Linux.
    (card / "cmdline.txt").write_text("console=serial0\n")
    with pytest.raises(cli_card.CardError, match="Raspberry Pi OS"):
        cli_card.check_card(card, force=False, force_dir=True)
    assert "Raspberry Pi OS" in cli_card.check_card(card, force=True, force_dir=True)[0]
    (card / "cmdline.txt").unlink()
    # The netboot loader's card: no marker, no sign of Kilo.
    (card / "kernel7.img").write_bytes(b"circle loader")
    with pytest.raises(cli_card.CardError, match="not Kilo's mind kernel"):
        cli_card.check_card(card, force=False, force_dir=True)
    (card / "kilorec").mkdir()
    assert cli_card.check_card(card, force=False, force_dir=True) == []


# ---- writing ---------------------------------------------------------------------------------

def test_write_kernel_and_body(tmp_path, no_key, capsys):
    card = make_card(tmp_path)
    (card / "body-result.txt").write_text("kilo-body-result 1\noutcome: failed\nimage: none\n")
    kernel = tmp_path / "kernel7.img"
    kernel.write_bytes(b"new " + MARK + b"\1" * 5000)
    body, m = body_files(tmp_path)
    assert kcard(str(card), "--force-dir", "--kernel", str(kernel), "--image", str(body)) == 0
    out, err = capsys.readouterr()
    assert (card / "kernel7.img").read_bytes() == kernel.read_bytes()
    assert (card / "kernel7.old").read_bytes().startswith(b"old ")
    assert not (card / "kernel7.new").exists()
    assert (card / "body.bin").read_bytes() == body.read_bytes()
    assert BI.Manifest.from_json((card / "body.json").read_text()) == m
    assert not (card / "body-result.txt").exists()
    assert "removed body-result.txt (it said: failed, none)" in out
    assert "the previous one is kernel7.old" in out
    assert f"compares body.json (build {m.build:08x})" in out and f"kilo card {card} --status" in out
    assert "this laptop has no key" in err  # a warning, not a refusal

    # Again with the same kernel: left alone, kernel7.old not touched.
    (card / "kernel7.old").write_bytes(b"older")
    assert kcard(str(card), "--force-dir", "--kernel", str(kernel), "--no-body") == 0
    assert "already this image" in capsys.readouterr().out
    assert (card / "kernel7.old").read_bytes() == b"older"


def test_kernel_checks(tmp_path, host_key, capsys):
    card = make_card(tmp_path)
    loader = tmp_path / "loader.img"
    loader.write_bytes(b"no marker here")
    assert kcard(str(card), "--force-dir", "--kernel", str(loader), "--no-body") == 1
    assert "not the mind kernel" in capsys.readouterr().err
    # Without this laptop's key built in: refused (the network route would lock it out), --force writes it.
    other = tmp_path / "other.img"
    other.write_bytes(b"x" + MARK + b"\0" * 32)
    assert kcard(str(card), "--force-dir", "--kernel", str(other), "--no-body") == 1
    assert f"does not have this laptop's key ({host_key.fingerprint})" in capsys.readouterr().err
    assert (card / "kernel7.img").read_bytes().startswith(b"old ")
    assert kcard(str(card), "--force-dir", "--kernel", str(other), "--no-body", "--force") == 0
    assert (card / "kernel7.img").read_bytes() == other.read_bytes()
    # With it: said so.
    good = tmp_path / "good.img"
    good.write_bytes(b"y" + MARK + host_key.public)
    assert kcard(str(card), "--force-dir", "--kernel", str(good), "--no-body") == 0
    assert f"this laptop's key ({host_key.fingerprint}) is built in" in capsys.readouterr().out


def test_refusals_before_the_card_is_touched(tmp_path, no_key, capsys):
    card = make_card(tmp_path)
    before = sorted(p.name for p in card.iterdir())
    assert kcard(str(card), "--force-dir", "KILO REAR=1", "--no-body") == 1
    assert "not a NAME=VALUE build option" in capsys.readouterr().err
    assert kcard(str(card), "--force-dir", "KILO_REAR_UP=1", "--no-build") == 1
    assert "need a kernel build" in capsys.readouterr().err
    assert kcard(str(card), "--force-dir", "--no-kernel", "--no-body") == 1
    body, _ = body_files(tmp_path)
    bad = tmp_path / "bad"
    bad.mkdir()
    (bad / "body.bin").write_bytes(body.read_bytes()[:-10])  # half a copy, with the whole one's manifest
    (bad / "body.json").write_text((body.parent / "body.json").read_text())
    assert kcard(str(card), "--force-dir", "--no-kernel", "--image", str(bad / "body.bin")) == 1
    assert "manifest says" in capsys.readouterr().err
    assert kcard(str(card), "--force-dir", "--no-kernel", "--no-build",
                 "--build-dir", str(tmp_path / "nobuild")) == 1
    assert "no earlier body build" in capsys.readouterr().err
    assert sorted(p.name for p in card.iterdir()) == before


def test_builds_with_options(tmp_path, no_key, monkeypatch, capsys):
    """The kernel's make with the options passed on, the body's arduino-cli build, then the card."""
    card = make_card(tmp_path)
    kernel_out = tmp_path / "kbuild" / "kernel7.img"
    monkeypatch.setattr(cli_card, "DEFAULT_KERNEL", kernel_out)
    sketch = tmp_path / "body"
    sketch.mkdir()
    (sketch / "body.ino").write_text("// sketch\n")
    build_dir = tmp_path / "bbuild"
    data = random.Random(5).randbytes(600)
    calls: list[list[str]] = []

    def run(cmd: list[str], check: bool = True) -> Any:
        calls.append(cmd)
        if cmd[0] == "make":
            kernel_out.parent.mkdir(parents=True, exist_ok=True)
            kernel_out.write_bytes(b"built " + MARK + " ".join(cmd).encode())
        elif cmd[0] == "arduino-cli":
            out = Path(cmd[cmd.index("--output-dir") + 1])
            hexlines = []
            for off in range(0, len(data), 16):
                chunk = data[off:off + 16]
                r = bytes([len(chunk), off >> 8 & 0xFF, off & 0xFF, 0]) + chunk
                hexlines.append(":" + (r + bytes([-sum(r) & 0xFF])).hex().upper())
            (out / "body.ino.hex").write_text("\n".join(hexlines + [":00000001FF"]) + "\n")
        elif "size_check.sh" in cmd[0]:
            pass
        else:
            raise AssertionError(cmd)
        return subprocess.CompletedProcess(cmd, 0)

    args = cli.parse_args(["card", str(card), "KILO_REAR_UP=1", "KILO_RECORD_SAVE=1", "--force-dir",
                           "--sketch", str(sketch), "--build-dir", str(build_dir)])
    assert cli_card.cmd_card(args, run=run) == 0
    make = calls[0]
    assert make[:3] == ["make", "-C", str(cli_card.KERNEL_DIR)] and make[-2:] == ["KILO_REAR_UP=1", "KILO_RECORD_SAVE=1"]
    assert calls[1][0] == "arduino-cli" and any("-DKILO_BUILD=0x" in c for c in calls[1])
    assert (card / "kernel7.img").read_bytes() == kernel_out.read_bytes()
    assert (card / "body.bin").read_bytes() == data
    m = BI.Manifest.from_json((card / "body.json").read_text())
    assert m.size == 600 and m.build != 0
    out = capsys.readouterr().out
    assert "$ make -C" in out and "At the next power-on:" in out


# ---- what the mind left ----------------------------------------------------------------------

RESULT = """kilo-body-result 1
outcome: {outcome}
image: build {build:08x} size {size} crc32 {crc:08x}
why: {why}
body before: build 56424f44
body after: build {build:08x} reset FLASHED
error: none
battery: 7810 mV
mind: build 1234abcd, 4100 ms after the link came up
next boot: nothing to do: the body runs the card's build
log:
  3.100 I boot flash check: the body runs build 56424f44, the card has build {build:08x} (3000 bytes), battery 7810 mV: flashing
  7.900 I flash: done in 4700 ms: HELLO build {build:08x} reset FLASHED
"""


def test_status(tmp_path, no_key, capsys):
    card = make_card(tmp_path)
    body, m = body_files(tmp_path)
    assert kcard(str(card), "--force-dir", "--no-kernel", "--image", str(body)) == 0
    capsys.readouterr()
    assert kcard(str(card), "--force-dir", "--status") == 0
    out = capsys.readouterr().out
    assert "the mind kernel" in out and "body.bin matches it" in out
    assert "body-result.txt: none: the Pi has not booted from this card" in out
    assert "kilorec/: none (saving to the card is off" in out

    (card / "body-result.txt").write_text(RESULT.format(outcome="flashed", build=m.build, size=m.size, crc=m.crc32,
                                                        why="flashed build 3d441ec0"))
    rec = card / "kilorec"
    rec.mkdir()
    (rec / "SAVE_ON").write_text("")
    (rec / "b000002-s00000000.krc").write_bytes(b"\0" * 2048)
    (rec / "b000002-s00000064.krc").write_bytes(b"\0" * 1024)
    (rec / "b000002.log").write_text("".join(f"{1000 * i} I line {i}\n" for i in range(60)) + "61000 M power: x\n")
    (rec / "b000001.log").write_text("5 I older boot\n")
    assert kcard(str(card), "--force-dir", "--status", "--lines", "5") == 0
    out = capsys.readouterr().out
    assert "body-result.txt: outcome FLASHED (about the body.json on the card now)" in out
    assert "  body after: build 3d441ec0 reset FLASHED" in out
    assert "flash: done in 4700 ms" in out
    assert "saving on; recordings of 2 boots" in out
    assert "boot 2: 2 block files, 0.0 MB, a saved log" in out
    assert "the last 5 lines of boot 2's saved log" in out
    assert "line 59" in out and "line 54" not in out and "power: x" not in out
    # A result about an earlier image.
    other_body, _ = body_files(tmp_path, build=0x11112222, seed=2)
    BI.write_image(card, other_body.read_bytes(), BI.read_image(other_body)[1])
    assert kcard(str(card), "--force-dir", "--status") == 0
    assert "(about ANOTHER image than the body.json on the card now)" in capsys.readouterr().out


def test_fetch_layout_and_analysis(tmp_path, capsys):
    card = make_card(tmp_path)
    rec = card / "kilorec"
    rec.mkdir()
    records, _ = trick_records(run_id=3)
    records.append(log(T0 + 500_000, "an early line"))
    data = blocks(records, boot=4)
    half = data.index(flight._MAGIC_BYTES, len(data) // 2) if data.count(flight._MAGIC_BYTES) > 1 else len(data)
    (rec / "b000004-s00000000.krc").write_bytes(data[:half])
    if half < len(data):
        (rec / "b000004-s00000001.krc").write_bytes(data[half:])
    (rec / "b000004.log").write_text("1000 I hello\n")
    out_dir = tmp_path / "logs"
    assert kcard(str(card), "--force-dir", "--fetch", "--out", str(out_dir)) == 0
    out = capsys.readouterr().out
    assert "1 runs" in out
    whole = out_dir / "rec-card-b4.krec"
    meta, raw = flight.read_file(whole)
    assert raw == data and meta["boot"] == 4 and meta["what"] == "blocks"
    runs = sorted(out_dir.glob("run-b4-*-trick-3.krec"))
    assert len(runs) == 1
    s = flight.load(runs[0])
    assert [r.id for r in s.runs] == [3] and s.runs[0].kind == "trick"
    assert (out_dir / "kilo-card-b4.log").read_text() == "1000 I hello\n"
    # The analysis commands take it as they take a network fetch.
    assert cli.main(["log", "trick", str(runs[0])]) == 0
    assert "trick run 3: rear_up_2" in capsys.readouterr().out
    # Fetching again writes the same names.
    assert kcard(str(card), "--force-dir", "--fetch", "--out", str(out_dir)) == 0
    assert len(list(out_dir.glob("*.krec"))) == 2


def test_fetch_and_status_refuse_a_foreign_folder(tmp_path, capsys):
    d = tmp_path / "usb"
    d.mkdir()
    assert kcard(str(d), "--force-dir", "--status") == 1
    assert "not a Kilo boot card" in capsys.readouterr().err
    assert kcard(str(d), "--force-dir", "--status", "KILO_REAR_UP=1", "--force") == 1
