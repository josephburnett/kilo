"""`kilo flash`: build the body firmware and flash it through the C++ mind
(docs/fast-boot.md §6, phase P4), or put it on the mind's boot card.

1. Build (``prepare_image``, unless ``--image``): ``arduino-cli compile --fqbn
   arduino:avr:uno --output-dir DIR`` with the size flags of
   body/tools/build_flags.sh and ``-DKILO_BUILD=0x<git hash>`` (the first 8
   hex digits of the commit, reported by the body as HELLO.fw_build), then
   ``body/tools/size_check.sh`` (the size/symbol harness) on that ELF; stop
   on failure. The HEX becomes ``body.bin`` + ``body.json``
   (kilo.tools.body_image) in the build dir.
2. ``kilo flash --host IP [--persist]`` (``flash_host``): the image is
   streamed over the host channel (``HostLink.flash``: FLASH_BEGIN,
   FLASH_DATA, FLASH_COMMIT); the mind resets the body into optiboot and
   programs it over STK500v1 (kernel/mind/flash/flasher.h), reporting
   FLASH_STATUS, and ends with the new firmware's HELLO.fw_build.
   ``--persist`` asks the mind to keep the image on its SD card afterwards.
3. ``kilo flash --sd MOUNT``: the same two files onto the boot card's FAT
   partition (checked and written by kilo.cli_card, as ``kilo card``), which
   the mind compares with HELLO.fw_build at boot.

``--image body.bin [--manifest body.json]`` skips the build for either.
"""

from __future__ import annotations

import functools
import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Sequence

MIND_DIR = Path(__file__).resolve().parents[2]
REPO_DIR = MIND_DIR.parent
DEFAULT_SKETCH = REPO_DIR / "body"
DEFAULT_BUILD_DIR = Path.home() / ".cache" / "kilo-build"
FQBN = "arduino:avr:uno"


class FlashError(Exception):
    pass


def git_commit(repo: Path = REPO_DIR) -> str | None:
    """The checkout's commit (40 hex digits), None when git cannot say."""
    try:
        out = subprocess.run(["git", "-C", str(repo), "rev-parse", "HEAD"],
                             capture_output=True, text=True, check=True, timeout=10)
        return out.stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def git_hash(repo: Path = REPO_DIR) -> str | None:
    """The build hash: the commit's first 8 hex digits = HELLO.fw_build u32."""
    commit = git_commit(repo)
    return commit[:8] if commit else None


def run_command(cmd: Sequence[str], what: str, run: Callable[..., Any] = subprocess.run,
                say: Callable[[str], None] = print, error: type[Exception] = FlashError,
                dry_run: bool = False) -> None:
    """Print and run cmd (unless dry_run); error(...) if it fails, naming it what."""
    say("$ " + " ".join(cmd))
    if dry_run:
        return
    try:
        run(list(cmd), check=True)
    except subprocess.CalledProcessError as e:
        raise error(f"{what} failed (exit {e.returncode})") from None
    except OSError as e:
        raise error(f"{what}: {e}") from None


# The size flags for the body build (doc §3h), defined once for this and
# body/tools/size_check.sh and stack_check.sh, which source it.
BUILD_FLAGS_FILE = REPO_DIR / "body" / "tools" / "build_flags.sh"


@functools.cache
def build_flags() -> tuple[str, str]:
    """(CFLAGS, LDFLAGS) from BUILD_FLAGS_FILE."""
    flags = dict(re.findall(r'^([A-Z]+)="([^"]*)"$', BUILD_FLAGS_FILE.read_text(), re.M))
    return flags["CFLAGS"], flags["LDFLAGS"]


def compile_cmd(sketch: Path, out_dir: Path, build_hash: str | None) -> list[str]:
    cflags, ldflags = build_flags()
    cpp = cflags + (f" -DKILO_BUILD=0x{build_hash}" if build_hash else "")
    return ["arduino-cli", "compile", "--fqbn", FQBN, "--output-dir", str(out_dir),
            "--build-property", f"compiler.cpp.extra_flags={cpp}",
            "--build-property", f"compiler.c.extra_flags={cflags}",
            "--build-property", f"compiler.c.elf.extra_flags={ldflags}",
            str(sketch)]


def build_body(sketch: Path, build_dir: Path, size_check: bool, sh: Callable[[Sequence[str], str], None],
               say: Callable[[str], None], dry_run: bool = False) -> int | None:
    """Step 1: arduino-cli compile with the build hash, then the size/symbol
    harness on that ELF. Returns the build hash (HELLO.fw_build to expect),
    None when git cannot say."""
    if not (sketch / f"{sketch.name}.ino").exists() and not dry_run:
        raise FlashError(f"no sketch at {sketch} (use --sketch or --no-build --hex)")
    build_dir.mkdir(parents=True, exist_ok=True)
    build_hash = git_hash()
    sh(compile_cmd(sketch, build_dir, build_hash), "arduino-cli compile")
    size_sh = REPO_DIR / "body" / "tools" / "size_check.sh"
    if size_check:
        if size_sh.exists():
            # Check the ELF that was just built (no second compile).
            sh([str(size_sh), "--elf", str(build_dir / f"{sketch.name}.ino.elf")], "size/symbol harness")
        else:
            say(f"warning: {size_sh} not found, size/symbol harness skipped")
    return int(build_hash, 16) if build_hash else None


# ---- through a C++ mind (P4) --------------------------------------------------------------

@dataclass
class ImageOptions:
    """Where the image comes from: a build (default) or --image/--manifest."""
    image: Path | None = None
    manifest: Path | None = None
    sketch: Path = DEFAULT_SKETCH
    build_dir: Path = DEFAULT_BUILD_DIR
    size_check: bool = True


def image_options(args: Any, image: Path | None = None) -> ImageOptions:
    """ImageOptions from `kilo flash --host|--sd` or `kilo card` arguments (image: instead of args.image)."""
    return ImageOptions(image=image if image is not None else args.image, manifest=args.manifest,
                        sketch=args.sketch or DEFAULT_SKETCH, build_dir=args.build_dir or DEFAULT_BUILD_DIR,
                        size_check=not getattr(args, "no_size_check", False))


def prepare_image(opts: ImageOptions, say: Callable[[str], None] = print,
                  run: Callable[..., Any] = subprocess.run) -> tuple[bytes, Any]:
    """body.bin and its manifest, checked: the given files, or a fresh build
    (compile, size harness, HEX -> body.bin, body.json in the build dir)."""
    from kilo.tools import body_image as BI

    try:
        if opts.image is not None:
            data, m = BI.read_image(opts.image, opts.manifest)
            say(f"image {opts.image}: {m.size} bytes, build {m.build:08x}, crc32 {m.crc32:08x}")
            return data, m
        build = build_body(opts.sketch, opts.build_dir, opts.size_check,
                           lambda cmd, what: run_command(cmd, what, run, say), say)
        if build is None:
            raise FlashError("no build hash (git rev-parse failed): the mind could not check the HELLO; "
                             "use --image/--manifest")
        hex_path = opts.build_dir / f"{opts.sketch.name}.ino.hex"
        data = BI.hex_to_bin(hex_path.read_text())
        m = BI.make_manifest(data, build, commit=git_commit())
        BI.check_image(data, m)
        bin_path, man_path = BI.write_image(opts.build_dir, data, m)
        say(f"image {bin_path}: {m.size} bytes, build {m.build:08x}, crc32 {m.crc32:08x} ({man_path.name})")
        return data, m
    except (BI.ImageError, OSError) as e:
        raise FlashError(str(e)) from None


def describe_flash_status(st: Any) -> str:
    from kilo.hostlink import enum_name as name
    from kilo.protocol import kilo_protocol as P

    phase = name(P.FlashPhase, st.phase)
    out = f"{phase.lower()}"
    if st.phase in (P.FlashPhase.PROGRAMMING, P.FlashPhase.VERIFYING):
        out += f" {st.page}/{st.pages}"
    if st.attempt > 1:
        out += f" (attempt {st.attempt})"
    if st.phase == P.FlashPhase.FAILED:
        out += f": {name(P.FlashError, st.error)}" + (f" (STK500 command {st.stk_cmd:02x})" if st.stk_cmd else "")
    if st.fw_build:
        out += f"; HELLO build {st.fw_build:08x} reset {name(P.ResetCause, st.reset_cause)}"
    return out


def flash_host(hl: Any, data: bytes, manifest: Any, persist: bool = False,
               say: Callable[[str], None] = print, timeout: float | None = None) -> Any:
    """Flash through the mind's host channel; prints progress (phase
    changes, every ~10% of pages). Returns the final P.HostFlashStatus.
    Raises FlashError for a refusal or a lost connection."""
    from kilo.hostlink import FLASH_TIMEOUT_S, HostError, HostRefused
    from kilo.protocol import kilo_protocol as P

    shown: dict[str, int] = {"phase": -1, "tenth": -1}

    paged = (P.FlashPhase.PROGRAMMING, P.FlashPhase.VERIFYING)

    def on_status(st: Any) -> None:
        tenth = st.page * 10 // st.pages if st.pages else 0
        if st.phase != shown["phase"] and shown["phase"] in paged and st.phase > shown["phase"]:
            # The flasher moves on only once every page is done; the last
            # page's status may have been merged into the next phase's.
            say(f"flash: {P.FlashPhase(shown['phase']).name.lower()} done, {st.pages} pages")
        if st.phase != shown["phase"] or (st.phase in paged and tenth != shown["tenth"]):
            shown["phase"], shown["tenth"] = st.phase, tenth
            say("flash: " + describe_flash_status(st))

    say(f"sending {manifest.size} bytes to the mind at {hl.address}" + (" (persist to SD)" if persist else ""))
    t0 = time.monotonic()
    try:
        st = hl.flash(data, manifest.build, manifest.crc32, persist=persist, on_status=on_status,
                      timeout=timeout or FLASH_TIMEOUT_S)
    except HostRefused as e:
        raise FlashError(f"the mind refused: {e}") from None
    except HostError as e:
        raise FlashError(str(e)) from None
    if st.phase != shown["phase"]:
        say("flash: " + describe_flash_status(st))
    say(f"flash ended in {time.monotonic() - t0:.1f} s")
    return st
