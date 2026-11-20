#!/usr/bin/env python3
"""Pack Kilo's sounds into the kernel: every kernel/sounds/*.wav into a
generated C++ table (kernel/mind/sound/sound.h's kSounds: name, sample
rate, length, samples) and a size report. kernel/Makefile runs it on every
build (kernel/host/Makefile too, for kilo-mind-host); it rewrites the table
only when it changes, and stops the build, saying why, when a file cannot
be used or the sounds do not fit the budget. docs/sound.md.

    python3 kernel/tools/pack_sounds.py DIR OUT.cpp [--budget BYTES] [--report]

What it takes: uncompressed PCM WAV files (8, 16, 24 or 32-bit, mono or
stereo, 8000 to 48000 Hz); the name is the file name without .wav,
lower case: a-z, 0-9 and _, at most 15 characters. A missing or empty
directory is no error: the table is empty, and every play request is a
quiet no-op the mind counts.

What it stores, per sound: one channel (stereo is averaged), at the file's
rate, or 22050 Hz if the file's is higher (averaged down: the speaker
does not reach above 11 kHz); its peak brought to -1 dBFS (gain at most
4x, so a quiet recording's noise is not made loud), so every sound plays
about as loud at the same volume; then 8-bit mu-law (ITU-T G.711), one
byte a sample. At 22050 Hz that is 21.5 KB a second, half of 16-bit PCM;
mu-law keeps about 13 bits near silence, more than the Pi's PWM output
resolves. The budget (default 1.5 MB of samples) is the most the sounds
may add to the image; over it the build fails with what is largest,
rather than leaving any sound out.

Standard library only (the kernel build may run without the mind's venv),
plus this checkout's generated protocol table (mind/kilo/protocol, stdlib only).
"""

from __future__ import annotations

import argparse
import math
import re
import struct
import sys
import wave
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "mind"))
from kilo.protocol import kilo_protocol as P  # noqa: E402

NAME_MAX = P.HostSound.MAX_LEN - 2  # 15: kernel/mind/sound/sound.h kNameMax, SOUND.name (kilo.cli_sound)
NAME_RE = re.compile(r"^[a-z0-9_]{1,%d}$" % NAME_MAX)
MAX_RATE = 22050               # stored at most this
MIN_RATE, TOP_RATE = 8000, 48000  # files outside this are refused
PEAK = 10 ** (-1 / 20)         # -1 dBFS
GAIN_MAX = 4.0
BUDGET = 1536 * 1024           # bytes of samples
MAX_SOUNDS = 255               # SOUND_STATUS.sounds is a u8


class PackError(Exception):
    pass


@dataclass
class Packed:
    name: str
    rate: int
    data: bytes                # mu-law, one byte a sample
    source: str                # "44100 Hz stereo 16-bit"
    peak_db: float             # the file's peak, dBFS
    gain: float

    @property
    def seconds(self) -> float:
        return len(self.data) / self.rate


# ---- mu-law (G.711) ------------------------------------------------------------------------
_BIAS, _CLIP = 0x84, 32635


def ulaw_encode(sample: int) -> int:
    """One 16-bit sample to its mu-law byte (as Sun's g711.c / Craig Reese's ulaw.c)."""
    sign = 0x80 if sample < 0 else 0
    if sample < 0:
        sample = -sample
    sample = min(sample, _CLIP) + _BIAS
    exponent = max(0, ((sample >> 7) & 0xFF).bit_length() - 1)
    mantissa = (sample >> (exponent + 3)) & 0x0F
    return ~(sign | (exponent << 4) | mantissa) & 0xFF


def ulaw_decode(byte: int) -> int:
    """The byte back to a 16-bit sample (sound.h kUlaw)."""
    u = ~byte & 0xFF
    mag = (((u & 0x0F) << 3) + _BIAS) << ((u & 0x70) >> 4)
    return _BIAS - mag if u & 0x80 else mag - _BIAS


_ENCODE = bytes(ulaw_encode(s) for s in range(-32768, 32768))


# ---- reading ---------------------------------------------------------------------------------
def _read(path: Path) -> tuple[list[float], int, str]:
    """The file's samples as one channel in -1..1, its rate, and what it was."""
    try:
        with wave.open(str(path), "rb") as w:
            ch, width, rate, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
            raw = w.readframes(n)
    except (wave.Error, EOFError, OSError) as e:
        raise PackError(f"{path}: not a PCM WAV file kilo can read ({e}); save it as 16-bit PCM WAV") from None
    if ch not in (1, 2):
        raise PackError(f"{path}: {ch} channels; mono or stereo only")
    if not MIN_RATE <= rate <= TOP_RATE:
        raise PackError(f"{path}: {rate} Hz; {MIN_RATE} to {TOP_RATE} Hz only")
    if width not in (1, 2, 3, 4):
        raise PackError(f"{path}: {8 * width}-bit samples; 8, 16, 24 or 32-bit PCM only")
    frames = len(raw) // (width * ch)
    if frames < 2:
        raise PackError(f"{path}: no samples")
    if width == 1:  # unsigned
        vals = [(b - 128) / 128.0 for b in raw[:frames * ch]]
    elif width == 2:
        vals = [v / 32768.0 for v in struct.unpack(f"<{frames * ch}h", raw[:frames * ch * 2])]
    elif width == 3:
        vals = [int.from_bytes(raw[i:i + 3], "little", signed=True) / 8388608.0 for i in range(0, frames * ch * 3, 3)]
    else:
        vals = [v / 2147483648.0 for v in struct.unpack(f"<{frames * ch}i", raw[:frames * ch * 4])]
    if ch == 2:
        vals = [(vals[i] + vals[i + 1]) / 2 for i in range(0, len(vals), 2)]
    what = f"{rate} Hz {'stereo' if ch == 2 else 'mono'} {8 * width}-bit"
    return vals, rate, what


def _downsample(vals: list[float], rate: int, to: int) -> list[float]:
    """rate -> to (to < rate): each output sample the mean of the input samples it covers."""
    ratio = rate / to
    out = []
    n = int(len(vals) / ratio)
    for k in range(n):
        a, b = int(k * ratio), max(int(k * ratio) + 1, int((k + 1) * ratio))
        seg = vals[a:min(b, len(vals))]
        out.append(sum(seg) / len(seg))
    return out


def pack_file(path: Path, normalise: bool = True) -> Packed:
    name = path.stem.lower()
    if not NAME_RE.match(name):
        raise PackError(f"{path}: the name {name!r} is not one kilo takes: a-z, 0-9 and _, at most {NAME_MAX} "
                        "characters (rename the file)")
    vals, rate, what = _read(path)
    if rate > MAX_RATE:
        vals, rate = _downsample(vals, rate, MAX_RATE), MAX_RATE
    peak = max(abs(v) for v in vals)
    gain = 1.0
    if normalise and peak > 0:
        gain = min(GAIN_MAX, PEAK / peak)
    data = bytes(_ENCODE[max(-32768, min(32767, int(round(v * gain * 32767)))) + 32768] for v in vals)
    peak_db = 20 * math.log10(peak) if peak > 0 else float("-inf")
    return Packed(name, rate, data, what, peak_db, gain)


def pack_dir(src: Path, normalise: bool = True) -> list[Packed]:
    if not src.is_dir():
        return []
    files = sorted(p for p in src.iterdir() if p.is_file() and p.suffix.lower() == ".wav")
    out: list[Packed] = []
    seen: dict[str, Path] = {}
    for p in files:
        k = p.stem.lower()
        if k in seen:
            raise PackError(f"{p}: the same name as {seen[k]} ({k})")
        seen[k] = p
        out.append(pack_file(p, normalise))
    if len(out) > MAX_SOUNDS:
        raise PackError(f"{src}: {len(out)} sounds; at most {MAX_SOUNDS}")
    return out


def check_budget(sounds: list[Packed], budget: int) -> None:
    total = sum(len(s.data) for s in sounds)
    if total <= budget:
        return
    big = sorted(sounds, key=lambda s: -len(s.data))[:5]
    largest = ", ".join(f"{s.name} {len(s.data) / 1024:.0f} KB ({s.seconds:.1f} s)" for s in big)
    raise PackError(f"the sounds take {total / 1024:.0f} KB, over the budget of {budget / 1024:.0f} KB "
                    f"(about {budget / MAX_RATE:.0f} s at {MAX_RATE} Hz); the largest: {largest}. "
                    "Shorten or remove some, or save them at a lower rate (11025 Hz halves a sound).")


# ---- writing ---------------------------------------------------------------------------------
def render(sounds: list[Packed], source: str) -> str:
    rows = [
        f"// Generated by kernel/tools/pack_sounds.py from {source}: do not edit.",
        f"// {len(sounds)} sounds, {sum(s.seconds for s in sounds):.2f} s, "
        f"{sum(len(s.data) for s in sounds)} bytes of mu-law samples.",
        '#include "sound/sound.h"',
        "",
        "namespace kilo {",
        "namespace sound {",
        "",
    ]
    for s in sounds:
        rows.append(f"// {s.name}: {s.seconds:.3f} s at {s.rate} Hz (from {s.source}, gain {s.gain:.2f})")
        rows.append(f"static const uint8_t kData_{s.name}[{len(s.data)}] = {{")
        for i in range(0, len(s.data), 24):
            rows.append("    " + ",".join(f"0x{b:02x}" for b in s.data[i:i + 24]) + ",")
        rows.append("};")
        rows.append("")
    if sounds:
        rows.append(f"const Sound kSounds[{len(sounds)}] = {{")
        for s in sounds:
            rows.append(f'    {{"{s.name}", {s.rate}u, {len(s.data)}u, kData_{s.name}}},')
        rows.append("};")
    else:
        rows.append("const Sound kSounds[1] = {{\"\", 0u, 0u, nullptr}};  // none: kSoundCount is 0")
    rows.append(f"const uint32_t kSoundCount = {len(sounds)}u;")
    rows += ["", "}  // namespace sound", "}  // namespace kilo", ""]
    return "\n".join(rows)


def report_lines(sounds: list[Packed], budget: int) -> list[str]:
    lines = []
    for s in sounds:
        peak = f"{s.peak_db:.1f}" if s.peak_db != float("-inf") else "silent"
        lines.append(f"  SOUND  {s.name:<15} {s.seconds:6.2f} s  {s.rate:5d} Hz  {len(s.data):7d} B  "
                     f"(from {s.source}, peak {peak} dBFS, gain x{s.gain:.2f})")
    return lines


def summary(sounds: list[Packed], budget: int) -> str:
    total = sum(len(s.data) for s in sounds)
    return (f"  SOUNDS {len(sounds)} sounds, {sum(s.seconds for s in sounds):.2f} s, {total / 1024:.0f} KB of "
            f"samples in the image (budget {budget / 1024:.0f} KB)")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("src", type=Path, help="the sounds' directory (kernel/sounds)")
    ap.add_argument("out", type=Path, help="the generated table (.cpp)")
    ap.add_argument("--budget", type=int, default=BUDGET, help=f"bytes of samples at most (default {BUDGET})")
    ap.add_argument("--report", action="store_true", help="every sound's line, even when nothing changed")
    ap.add_argument("--no-normalise", action="store_true", help="keep each file's own level")
    ap.add_argument("--shown-as", default=None, help="the directory's name in messages")
    a = ap.parse_args(argv)
    shown = a.shown_as or str(a.src)
    try:
        sounds = pack_dir(a.src, not a.no_normalise)
        check_budget(sounds, a.budget)
    except PackError as e:
        print(f"pack_sounds: {e}", file=sys.stderr)
        return 1
    text = render(sounds, shown)
    old = a.out.read_text() if a.out.exists() else None
    changed = old != text
    if changed:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(text)
    if changed or a.report:
        for ln in report_lines(sounds, a.budget):
            print(ln)
    print(summary(sounds, a.budget) + ("" if sounds else f": none in {shown}"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
