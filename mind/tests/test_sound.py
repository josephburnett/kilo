"""Kilo's sounds: the pack tool (kernel/tools/pack_sounds.py: WAVs into the
kernel's table, the size report, the budget) on synthetic tones written
here, and `kilo sound` (kilo.cli_sound) and the status line on a fake host
link. kernel/host/tests/test_sound.cpp tests the player and the renderer;
kernel/host/test.sh runs the CLI against kilo-mind-host."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import re
import struct
import sys
import wave
from pathlib import Path

import pytest

from kilo import cli, cli_sound
from kilo.cli_host import HOST_COMMANDS, HOST_ONLY, wants_host
from kilo.protocol import kilo_protocol as P

REPO = Path(__file__).resolve().parents[2]


def _load_pack():
    spec = importlib.util.spec_from_file_location("pack_sounds", REPO / "kernel" / "tools" / "pack_sounds.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["pack_sounds"] = mod  # its dataclass looks itself up there
    spec.loader.exec_module(mod)
    return mod


pack = _load_pack()


def tone(path: Path, seconds: float = 0.2, rate: int = 22050, hz: float = 440, amp: float = 0.25,
         channels: int = 1, width: int = 2) -> Path:
    n = int(seconds * rate)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(channels)
        w.setsampwidth(width)
        w.setframerate(rate)
        frames = bytearray()
        for i in range(n):
            v = amp * math.sin(2 * math.pi * hz * i / rate)
            for _ in range(channels):
                if width == 1:
                    frames.append(int(128 + v * 127))
                elif width == 2:
                    frames += struct.pack("<h", int(v * 32767))
                elif width == 3:
                    frames += int(v * 8388607).to_bytes(3, "little", signed=True)
                else:
                    frames += struct.pack("<i", int(v * 2147483647))
        w.writeframes(bytes(frames))
    return path


# ---- mu-law ---------------------------------------------------------------------------------
def test_ulaw_known_values_and_round_trip():
    # the kernel's table (sound.h kUlaw) at its ends
    assert pack.ulaw_decode(0xFF) == 0 and pack.ulaw_decode(0x7F) == 0
    assert pack.ulaw_decode(0x80) == 32124 and pack.ulaw_decode(0x00) == -32124
    assert pack.ulaw_encode(0) == 0xFF and pack.ulaw_encode(32767) == 0x80 and pack.ulaw_encode(-32768) == 0x00
    for s in range(-32768, 32768, 97):
        back = pack.ulaw_decode(pack.ulaw_encode(s))
        assert abs(back - s) <= abs(s) / 16 + 8, (s, back)
    # monotonic: a louder sample never encodes quieter
    vals = [pack.ulaw_decode(pack.ulaw_encode(s)) for s in range(-32768, 32768, 31)]
    assert all(b >= a for a, b in zip(vals, vals[1:]))


# ---- packing --------------------------------------------------------------------------------
def test_pack_a_directory(tmp_path):
    src = tmp_path / "sounds"
    src.mkdir()
    tone(src / "meow1.wav", 0.5)
    tone(src / "Meow2.WAV", 0.4, rate=44100, channels=2)  # upper case, 44.1 kHz stereo: mono 22050
    tone(src / "mew.wav", 0.25, rate=11025, width=1)  # 8-bit 11025 Hz: kept at its rate
    tone(src / "hiss.wav", 0.3, width=3)
    tone(src / "trill.wav", 0.3, width=4)
    (src / "SOURCES.md").write_text("where they came from\n")  # not a sound
    sounds = pack.pack_dir(src)
    by = {s.name: s for s in sounds}
    assert sorted(by) == ["hiss", "meow1", "meow2", "mew", "trill"]
    assert by["meow1"].rate == 22050 and len(by["meow1"].data) == int(0.5 * 22050)
    assert by["meow2"].rate == 22050 and abs(len(by["meow2"].data) - int(0.4 * 22050)) <= 1
    assert by["meow2"].source == "44100 Hz stereo 16-bit"
    assert by["mew"].rate == 11025 and by["mew"].source == "11025 Hz mono 8-bit"
    # normalised: the peak at -1 dBFS (the tones are at -12 dBFS: gain about 3.5)
    peak = max(abs(pack.ulaw_decode(b)) for b in by["meow1"].data) / 32768
    assert abs(20 * math.log10(peak) + 1) < 0.6 and 3.3 < by["meow1"].gain < 3.8
    out = tmp_path / "sounds.gen.cpp"
    assert pack.main([str(src), str(out)]) == 0
    text = out.read_text()
    assert 'const Sound kSounds[5] = {' in text and "const uint32_t kSoundCount = 5u;" in text
    assert '{"meow1", 22050u, 11025u, kData_meow1},' in text
    assert re.search(r"static const uint8_t kData_mew\[2756\] = \{", text)
    # unchanged: not rewritten (the build relinks only when the sounds change)
    mtime = out.stat().st_mtime_ns
    assert pack.main([str(src), str(out)]) == 0 and out.stat().st_mtime_ns == mtime


def test_pack_nothing(tmp_path, capsys):
    out = tmp_path / "gen.cpp"
    assert pack.main([str(tmp_path / "missing"), str(out), "--shown-as", "kernel/sounds"]) == 0
    text = out.read_text()
    assert "const uint32_t kSoundCount = 0u;" in text and "kSounds[1]" in text
    assert "0 sounds, 0.00 s, 0 KB of samples in the image (budget 1536 KB): none in kernel/sounds" in \
        capsys.readouterr().out
    (tmp_path / "empty").mkdir()
    assert pack.main([str(tmp_path / "empty"), str(out)]) == 0


def test_the_report(tmp_path, capsys):
    src = tmp_path / "s"
    src.mkdir()
    tone(src / "purr.wav", 1.0)
    assert pack.main([str(src), str(tmp_path / "o.cpp")]) == 0
    out = capsys.readouterr().out
    assert re.search(r"SOUND +purr +1\.00 s +22050 Hz +22050 B +\(from 22050 Hz mono 16-bit, peak -12\.\d dBFS", out)
    assert "1 sounds, 1.00 s, 22 KB of samples in the image (budget 1536 KB)" in out
    assert pack.main([str(src), str(tmp_path / "o.cpp")]) == 0
    assert "SOUND  purr" not in capsys.readouterr().out  # unchanged: the summary only
    assert pack.main([str(src), str(tmp_path / "o.cpp"), "--report"]) == 0
    assert "SOUND  purr" in capsys.readouterr().out


def test_over_the_budget_fails_and_says_what_is_largest(tmp_path, capsys):
    src = tmp_path / "s"
    src.mkdir()
    tone(src / "purr.wav", 2.0)
    tone(src / "mew.wav", 0.2)
    out = tmp_path / "o.cpp"
    assert pack.main([str(src), str(out), "--budget", "40000"]) == 1
    err = capsys.readouterr().err
    assert "over the budget of 39 KB" in err and "the largest: purr 43 KB (2.0 s), mew" in err
    assert not out.exists()  # nothing half-written: the build stops


@pytest.mark.parametrize("name,why", [("Big-Meow.wav", "is not one kilo takes"),
                                      ("a_name_much_too_long.wav", "is not one kilo takes")])
def test_bad_names(tmp_path, capsys, name, why):
    src = tmp_path / "s"
    src.mkdir()
    tone(src / name)
    assert pack.main([str(src), str(tmp_path / "o.cpp")]) == 1
    assert why in capsys.readouterr().err


def test_bad_files(tmp_path, capsys):
    src = tmp_path / "s"
    src.mkdir()
    (src / "broken.wav").write_bytes(b"RIFF....WAVEfmt nonsense")
    assert pack.main([str(src), str(tmp_path / "o.cpp")]) == 1
    assert "not a PCM WAV file kilo can read" in capsys.readouterr().err
    (src / "broken.wav").unlink()
    tone(src / "slow.wav", rate=4000)
    assert pack.main([str(src), str(tmp_path / "o.cpp")]) == 1
    assert "4000 Hz; 8000 to 48000 Hz only" in capsys.readouterr().err
    (src / "slow.wav").unlink()
    tone(src / "a.wav")
    tone(src / "A.WAV")
    assert pack.main([str(src), str(tmp_path / "o.cpp")]) == 1
    assert "the same name as" in capsys.readouterr().err


def test_no_normalise_keeps_the_level(tmp_path):
    src = tmp_path / "s"
    src.mkdir()
    tone(src / "quiet.wav", amp=0.01)
    s = pack.pack_dir(src, normalise=False)[0]
    assert s.gain == 1.0
    loud = pack.pack_dir(src)[0]
    assert loud.gain == pack.GAIN_MAX  # at most 4x: a quiet recording's noise stays quiet


# ---- kilo sound on a fake host link ---------------------------------------------------------
class FakeHost:
    def __init__(self, names=("hiss", "meow1", "meow2", "purr")):
        self.names = list(names)
        self.on, self.volume, self.last = 1, 70, b""
        self.calls = []

    def sound(self, op=0, volume=0, name=""):
        self.calls.append((op, volume, name))
        result = P.SoundResult.OK
        if op == P.SoundOp.ON:
            self.on = 1
        elif op == P.SoundOp.OFF:
            self.on = 0
        elif op == P.SoundOp.VOLUME:
            self.volume = volume
        elif op == P.SoundOp.PLAY:
            if not self.on:
                result = P.SoundResult.OFF
            elif name in self.names or any(n.rstrip("0123456789") == name for n in self.names):
                result, self.last = P.SoundResult.QUEUED, name.encode()
            else:
                result = P.SoundResult.UNKNOWN
        return P.HostSoundStatus(result=result, enabled=self.on, volume=self.volume, sounds=len(self.names),
                                 state=P.SoundState.IDLE, played=3, requests=5, unknown=1, dropped=1, late=0,
                                 callback_max_us=41, last=self.last)

    def sound_list(self):
        return [P.HostSoundInfo(index=i, count=len(self.names), rate=22050, samples=11025 * (i + 1),
                                name=n.encode()) for i, n in enumerate(self.names)]


def ns(**kw):
    d = {"action": "status", "value": None, "json": False}
    d.update(kw)
    return argparse.Namespace(**d)


def test_cli_sound_actions(capsys):
    h = FakeHost()
    assert cli_sound.cmd_sound(ns(), h) == 0
    out = capsys.readouterr().out
    assert "sound       on  volume 70  4 sounds in the image  idle  last -" in out
    assert "sound count played 3  requests 5  unknown 1  dropped 1  late chunks 0  longest callback 41 us" in out
    assert cli_sound.cmd_sound(ns(action="play", value="Meow"), h) == 0
    assert h.calls[-1] == (P.SoundOp.PLAY, 0, "meow")
    assert "sound       Meow: queued" in capsys.readouterr().out
    assert cli_sound.cmd_sound(ns(action="play", value="bark"), h) == 1
    assert "bark: no such sound in the image (kilo sound list)" in capsys.readouterr().out
    assert cli_sound.cmd_sound(ns(action="off"), h) == 0
    assert "sound       OFF  volume 70" in capsys.readouterr().out
    assert cli_sound.cmd_sound(ns(action="play", value="purr"), h) == 1
    assert "purr: sounds are off (kilo sound on)" in capsys.readouterr().out
    assert cli_sound.cmd_sound(ns(action="on"), h) == 0
    assert cli_sound.cmd_sound(ns(action="volume", value="35"), h) == 0
    assert h.calls[-1] == (P.SoundOp.VOLUME, 35, "") and "volume 35" in capsys.readouterr().out
    for bad in ("101", "-1", "loud", None):
        assert cli_sound.cmd_sound(ns(action="volume", value=bad), h) == 1
        assert "a number 0..100" in capsys.readouterr().err
    assert cli_sound.cmd_sound(ns(action="play"), h) == 1
    assert "name a sound or a family" in capsys.readouterr().err
    assert cli_sound.cmd_sound(ns(action="play", value="x" * 16), h) == 1
    assert "at most 15 characters" in capsys.readouterr().err


def test_cli_sound_list_and_json(capsys):
    h = FakeHost()
    assert cli_sound.cmd_sound(ns(action="list"), h) == 0
    out = capsys.readouterr().out
    assert "hiss              0.50 s  22050 Hz" in out and "4 sounds, 5.0 s" in out
    assert cli_sound.cmd_sound(ns(action="list", json=True), h) == 0
    rows = json.loads(capsys.readouterr().out)
    assert rows[1] == {"name": "meow1", "rate": 22050, "samples": 22050, "seconds": 1.0}
    assert cli_sound.cmd_sound(ns(json=True), h) == 0
    d = json.loads(capsys.readouterr().out)
    assert d["enabled"] == 1 and d["volume"] == 70 and d["state"] == "idle" and d["result"] == "ok"
    empty = FakeHost(names=())
    assert cli_sound.cmd_sound(ns(action="list"), empty) == 0
    assert "no sounds in this image" in capsys.readouterr().out


def test_status_line_without_an_output():
    st = P.HostSoundStatus(result=0, enabled=1, volume=70, sounds=0, state=P.SoundState.NONE, played=0,
                           requests=2, unknown=0, dropped=2, late=0, callback_max_us=0, last=b"")
    assert cli_sound.status_lines(st) == ["sound       on  volume 70  0 sounds in the image  none  last -  "
                                          "(no sound output: built with KILO_SOUND=0)"]


def test_kilo_sound_is_a_host_command():
    a = cli.parse_args(["sound", "play", "meow", "--host", "10.0.0.2"])
    assert (a.cmd, a.action, a.value, a.host) == ("sound", "play", "meow", "10.0.0.2")
    assert "sound" in HOST_ONLY and HOST_COMMANDS["sound"] is cli_sound.cmd_sound
    assert wants_host(cli.parse_args(["sound"]))  # finds kilo.local without --host
    assert cli.parse_args(["sound", "volume", "50"]).value == "50"
