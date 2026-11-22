"""`kilo card PATH`: update Kilo through its SD card in this laptop's card
reader, with no network (docs/laptop-setup.md "The card route").

    kilo card PATH [NAME=VALUE ...] [--no-build] [--kernel IMG] [--image BIN [--manifest JSON]]
                   [--no-kernel] [--no-body] [--force]
    kilo card PATH --status [--lines N]
    kilo card PATH --fetch [--out DIR]

PATH is the card's boot partition (FAT) as this laptop mounted it.

Writing (the default): builds the mind kernel (``make -C kernel``, NAME=VALUE
passed on, e.g. KILO_REAR_UP=1) and the body firmware (as ``kilo flash``
builds it: arduino-cli, the size harness, body.bin and body.json), checks
the kernel (the mind kernel's marker; this laptop's public key built in, if
it has a key), then on the card:

- ``kernel7.new`` written and read back, then renamed over
  ``kernel7.img``; a ``kernel7.old`` or ``kernel7.bad`` left from earlier
  removed: no previous kernel is kept (a kernel that misbehaves is
  replaced by building the wanted commit and writing it again; docs/laptop-
  setup.md). What ``kilo update --host`` has the mind do;
- ``body.json`` removed, ``body.bin`` written and read back, ``body.json``
  written last (so a manifest never pairs with half an image), and
  ``body-result.txt`` removed: at the next boot the mind decides afresh
  (kernel/mind/flash/cardflash.h) and writes a new one;
- then everything is flushed to the card (fsync, sync).

``--status`` reads what the mind left: body-result.txt (what the boot-time
flash did), the last lines of the newest saved log (kilorec/bNNNNNN.log,
when the flight recorder saves to the card), the recordings. ``--fetch``
copies the recordings to ~/kilo-logs in the layout the network fetch uses
(rec-card-bN.krec per boot, run-bN-<s>s-<kind>-<id>.krec per run, a
.kilolog per walk), for ``kilo log run|walk|trick FILE`` and ``kilo record
decode FILE``.

A PATH that does not look like Kilo's boot card is refused unless --force:
it must hold kernel7.img and config.txt, no Raspberry Pi OS files, and some
sign of Kilo (a mind kernel, kilorec/, body.json, body-result.txt).
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time
import zlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from kilo import EXIT_ERR, EXIT_OK
from kilo import kernel_image as KI
from kilo.kernel_image import DEFAULT_KERNEL, KERNEL_DIR, KERNEL_FILE, KERNEL_NEW, KERNEL_STALE, has_marker
from kilo.log import DEFAULT_DIR_TEXT
from kilo.protocol import kilo_protocol as P
from kilo.tools.body_image import IMAGE_FILE, MANIFEST_FILE

RESULT_FILE, RESULT_NEW = "body-result.txt", "body-result.new"  # kernel/mind/flash/cardflash.h
RESULT_MAGIC = "kilo-body-result 1"
REC_DIR = "kilorec"
PI_OS_SIGNS = ("cmdline.txt", "overlays")  # a Raspberry Pi OS boot partition: never Kilo's
MAKE_VAR = re.compile(r"^[A-Z][A-Z0-9_]*=\S*$")
_KRC = re.compile(r"^b(\d+)-s(\d+)\.krc$")
_LOG = re.compile(r"^b(\d+)\.log$")


class CardError(Exception):
    pass


# ---- parser ---------------------------------------------------------------------------------

def add_parser(sub: Any) -> None:
    s = sub.add_parser(
        "card", help="update Kilo through its SD card in this laptop's reader: the mind kernel and the body "
                     "firmware (flashed by the mind at its next boot); --status, --fetch read what it left",
        usage="kilo card PATH [NAME=VALUE ...] [--no-build] [--kernel IMG] [--image BIN [--manifest JSON]] "
              "[--no-kernel] [--no-body] [--force] | PATH --status [--lines N] | PATH --fetch [--out DIR]",
        description="PATH is Kilo's SD card (its FAT boot partition) mounted on this laptop. Without --status or "
                    "--fetch: build the mind kernel (make -C kernel, with the NAME=VALUE options, e.g. "
                    "KILO_REAR_UP=1) and the body firmware (as kilo flash does), check them, and copy them onto "
                    "the card with read-back: kernel7.img (no previous one kept) and body.bin + "
                    "body.json. At its next boot the Pi runs the new kernel, which flashes the body if the body "
                    "runs another build and writes what happened to body-result.txt. --status reads that, the "
                    "saved log and the recordings; --fetch copies the recordings to ~/kilo-logs.")
    s.add_argument("path", type=Path, metavar="PATH", help="the card's boot partition, mounted")
    s.add_argument("make_vars", nargs="*", metavar="NAME=VALUE",
                   help="kernel build options for make -C kernel (kernel/Makefile), e.g. KILO_REAR_UP=1")
    g = s.add_mutually_exclusive_group()
    g.add_argument("--status", action="store_true", help="what the mind left on the card: the body flash's "
                                                         "result, the saved log's last lines, the recordings")
    g.add_argument("--fetch", action="store_true",
                   help=f"copy the card's recordings to --out (default {DEFAULT_DIR_TEXT})")
    s.add_argument("--out", default=DEFAULT_DIR_TEXT, metavar="DIR",
                   help=f"--fetch: where to (default {DEFAULT_DIR_TEXT})")
    s.add_argument("--lines", type=int, default=40, metavar="N", help="--status: the saved log's last N lines")
    s.add_argument("--no-build", action="store_true",
                   help="do not build: the last kernel (kernel/build/kernel7.img) and the last body build "
                        "(~/.cache/kilo-build/body.bin)")
    s.add_argument("--kernel", type=Path, metavar="IMG", help="this kernel image instead of building one")
    s.add_argument("--image", type=Path, metavar="BIN", help="this body.bin instead of building the body")
    s.add_argument("--manifest", type=Path, metavar="JSON", help="the manifest of --image (default: body.json "
                                                                 "next to it)")
    s.add_argument("--no-kernel", action="store_true", help="leave the card's kernel as it is")
    s.add_argument("--no-body", action="store_true", help="leave the card's body firmware as it is")
    s.add_argument("--sketch", type=Path, help=argparse.SUPPRESS)
    s.add_argument("--build-dir", type=Path, help=argparse.SUPPRESS)
    s.add_argument("--force", action="store_true",
                   help="write to a PATH that does not look like Kilo's card, a kernel without the mind kernel's "
                        "marker, or one without this laptop's key")
    s.add_argument("--force-dir", action="store_true", help=argparse.SUPPRESS)  # tests: a plain directory


# ---- what the card is -----------------------------------------------------------------------

def _mounted_fat(path: Path) -> str | None:
    """Why path is not the root of a mounted FAT filesystem, or None."""
    real = path.resolve()
    mounts = Path("/proc/mounts")
    if mounts.exists():  # Linux: the mount table says the filesystem type too
        fstype = None
        try:
            for line in mounts.read_text().splitlines():
                parts = line.split()
                if len(parts) >= 3 and Path(parts[1].replace("\\040", " ")) == real:
                    fstype = parts[2]
        except OSError:
            return None
        if fstype is None:
            return f"{path} is not a mount point (mount the card's boot partition and give its folder)"
        if fstype in ("9p", "drvfs"):  # WSL: a Windows drive (sudo mount -t drvfs E: /mnt/e); its FAT is not visible
            return None
        if fstype not in ("vfat", "msdos"):
            return f"{path} is {fstype}, not FAT: not a Raspberry Pi boot partition"
        return None
    if not os.path.ismount(real):  # macOS /Volumes/NAME, Windows E:\
        return f"{path} is not the top of a mounted drive (give the card's folder itself)"
    return None


def check_card(path: Path, force: bool, force_dir: bool = False) -> list[str]:
    """CardError if path is not Kilo's boot card (unless force); the warnings that force let through."""
    if not path.is_dir():
        raise CardError(f"{path}: not a folder (is the card in the reader and mounted?)")
    problems: list[str] = []
    if not force_dir:
        why = _mounted_fat(path)
        if why:
            problems.append(why)
    missing = [n for n in (KERNEL_FILE, "config.txt") if not (path / n).is_file()]
    if missing:
        problems.append(f"{path} has no {' or '.join(missing)}: not a Kilo boot card")
    pi_os = [n for n in PI_OS_SIGNS if (path / n).exists()]
    if pi_os:
        problems.append(f"{path} has {' and '.join(pi_os)}: a Raspberry Pi OS card, not Kilo's")
    if not missing:
        signs = [(path / REC_DIR).is_dir(), (path / MANIFEST_FILE).exists(), (path / RESULT_FILE).exists()]
        if not any(signs):
            try:
                with open(path / KERNEL_FILE, "rb") as f:
                    signs.append(has_marker(f.read()))
            except OSError:
                pass
        if not any(signs):
            problems.append(f"{path}/{KERNEL_FILE} is not Kilo's mind kernel and the card has no {REC_DIR}/, "
                            "body.json or body-result.txt: not Kilo's card? (the netboot loader's card: --force)")
    if problems and not force:
        raise CardError("; ".join(problems) + "; refusing (--force to write anyway)")
    return problems


# ---- writing --------------------------------------------------------------------------------

def write_verified(path: Path, data: bytes) -> None:
    """data into path, flushed to the card, then read back and compared. On
    Linux the cached pages are dropped first, so the read comes from the
    card; elsewhere it may come from this computer's memory."""
    with open(path, "wb") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
        if hasattr(os, "posix_fadvise"):
            os.posix_fadvise(f.fileno(), 0, 0, os.POSIX_FADV_DONTNEED)
    back = path.read_bytes()
    if back != data:
        raise CardError(f"{path}: read back {len(back)} bytes, crc32 {zlib.crc32(back):08x}; wrote {len(data)}, "
                        f"crc32 {zlib.crc32(data):08x}: the card did not keep what was written")


def _sync() -> None:
    if hasattr(os, "sync"):
        os.sync()


def install_kernel(card: Path, data: bytes, say: Callable[[str], None]) -> bool:
    """kernel7.img := data: kernel7.new written and read back, then renamed over kernel7.img; a
    kernel7.old or kernel7.bad left from earlier removed (no previous kernel is kept). False if it was
    already there."""
    img, new = card / KERNEL_FILE, card / KERNEL_NEW
    stale = [n for n in KERNEL_STALE if (card / n).exists()]
    for n in stale:
        (card / n).unlink()
    gone = f"; removed {' and '.join(stale)}" if stale else ""
    if img.is_file() and img.read_bytes() == data:
        say(f"kernel: {img} is already this image ({len(data)} bytes); left as it is{gone}")
        return False
    write_verified(new, data)
    os.replace(new, img)
    say(f"kernel: wrote {img} ({len(data)} bytes, crc32 {zlib.crc32(data):08x}, read back){gone}")
    return True


def read_result(card: Path) -> dict[str, str] | None:
    """body-result.txt's key lines (outcome, image, why, ...) and its log, or None if there is none or it is
    not one."""
    try:
        text = (card / RESULT_FILE).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    lines = text.splitlines()
    if not lines or lines[0].strip() != RESULT_MAGIC:
        return None
    d: dict[str, str] = {}
    log: list[str] = []
    in_log = False
    for line in lines[1:]:
        if in_log:
            log.append(line.strip())
        elif line.strip() == "log:":
            in_log = True
        else:
            k, sep, v = line.partition(": ")
            if sep:
                d[k.strip()] = v.strip()
    d["log"] = "\n".join(x for x in log if x)
    return d


def install_body(card: Path, data: bytes, m: Any, say: Callable[[str], None]) -> None:
    from kilo.tools import body_image as BI

    man, img = card / BI.MANIFEST_FILE, card / BI.IMAGE_FILE
    man.unlink(missing_ok=True)
    write_verified(img, data)
    write_verified(man, m.to_json().encode())
    say(f"body: wrote {img} ({m.size} bytes, crc32 {m.crc32:08x}) and {man.name} (build {m.build:08x}), read back")
    old = read_result(card)
    (card / RESULT_NEW).unlink(missing_ok=True)
    if (card / RESULT_FILE).exists():
        (card / RESULT_FILE).unlink()
        said = f" (it said: {old.get('outcome', '?')}, {old.get('image', '?')})" if old else ""
        say(f"body: removed {RESULT_FILE}{said}: the mind decides afresh at the next boot")


def kernel_build_id() -> str | None:
    try:
        return (KERNEL_DIR / "build" / "build-id").read_text().strip() or None
    except OSError:
        return None


@dataclass
class Plan:
    kernel: bytes | None = None
    kernel_path: Path | None = None
    body: tuple[bytes, Any] | None = None
    warnings: list[str] = field(default_factory=list)


def check_kernel(data: bytes, path: Path, force: bool, key_path: str | None, say: Callable[[str], None]) -> list[str]:
    """The checks `kilo update` makes (kilo.kernel_image.check_kernel). CardError, or the warnings."""
    from kilo import hostkey

    warnings: list[str] = []
    try:
        key = hostkey.load(key_path)
    except hostkey.HostKeyMissing:
        key = None
        warnings.append("this laptop has no key (kilo key init): the card route needs none, but the network route "
                        "(kilo --host) will refuse this laptop until its key is in kernel/keys/authorized_keys "
                        "and the kernel is rebuilt")
    except hostkey.HostKeyError as e:
        key = None
        warnings.append(f"this laptop's key cannot be used ({e}); the card route needs none")
    errors, more = KI.check_kernel(data, path, force, key)
    if errors:
        raise CardError(errors[0])
    if key is not None and key.public in data:
        say(f"kernel: this laptop's key ({key.fingerprint}) is built in: kilo --host will work from here")
    return more + warnings


def prepare(args: argparse.Namespace, run: Callable[..., Any] = subprocess.run,
            say: Callable[[str], None] = print) -> Plan:
    """Build (or take) and check both images, before the card is touched."""
    from kilo.tools import flash as F

    plan = Plan()
    bad = [v for v in args.make_vars if not MAKE_VAR.match(v)]
    if bad:
        raise CardError(f"not a NAME=VALUE build option: {' '.join(bad)} (e.g. KILO_REAR_UP=1)")
    if args.no_kernel and args.no_body:
        raise CardError("--no-kernel and --no-body: nothing to write")
    if args.make_vars and (args.no_build or args.kernel is not None or args.no_kernel):
        raise CardError("kernel build options need a kernel build (not with --no-build, --kernel or --no-kernel)")
    if not args.no_kernel:
        if args.kernel is not None:
            path = args.kernel
        else:
            path = DEFAULT_KERNEL
            if not args.no_build:
                F.run_command(["make", "-C", str(KERNEL_DIR), f"-j{os.cpu_count() or 2}", *args.make_vars],
                              "make -C kernel", run, say, error=CardError)
        try:
            data = path.read_bytes()
        except OSError as e:
            hint = " (build it: make -C kernel, or drop --no-build)" if args.kernel is None else ""
            raise CardError(f"cannot read {path}: {e.strerror}{hint}") from None
        plan.warnings += check_kernel(data, path, args.force, getattr(args, "key", None), say)
        plan.kernel, plan.kernel_path = data, path
    if not args.no_body:
        image = args.image
        if image is None and args.no_build:
            image = (args.build_dir or F.DEFAULT_BUILD_DIR) / IMAGE_FILE
            if not image.exists():
                raise CardError(f"no earlier body build at {image}: drop --no-build (or give --image)")
        try:
            plan.body = F.prepare_image(F.image_options(args, image), say=say, run=run)
        except F.FlashError as e:
            raise CardError(f"body: {e}") from None
    return plan


def write_card(card: Path, plan: Plan, say: Callable[[str], None] = print) -> None:
    try:
        if plan.kernel is not None:
            install_kernel(card, plan.kernel, say)
        if plan.body is not None:
            install_body(card, plan.body[0], plan.body[1], say)
        _sync()
    except OSError as e:
        raise CardError(f"writing to {card}: {e}") from None


def next_boot_text(plan: Plan, built_kernel: bool) -> list[str]:
    out = ["At the next power-on:"]
    if plan.kernel is not None:
        build = kernel_build_id() if built_kernel else None
        out.append("- the Pi starts the new mind kernel" + (f" (build {build})" if build else "")
                   + " (no previous kernel is kept: to go back, build that commit and write it).")
    if plan.body is not None:
        m = plan.body[1]
        out.append(f"- Kilo's eyes stay dark until he is ready, then turn steady green. Before that, about 3 s "
                   f"in, the mind compares body.json (build {m.build:08x}) with the build the body reports. If "
                   "they differ it flashes the body: Kilo's eyes turn steady white and the Pi's green ACT light "
                   "gives two short flashes a second for about 10-20 s while Kilo's board restarts, then his "
                   "eyes turn steady green (it worked, ready). Three quick red flashes first mean it failed; "
                   "three purple flashes (no white) mean refused: battery low, or the files damaged; then "
                   "green, with the old firmware. White that stays, with the ACT light blinking slowly, means "
                   "the board has no working firmware: read the card. The same build: dark, then green. A "
                   "sound as well only if a speaker is connected (docs/signals.md).")
        out.append(f"- either way the mind writes what happened to {RESULT_FILE} on the card. Afterwards put the "
                   "card back in this laptop and run: kilo card PATH --status")
    return out


# ---- reading what the mind left ------------------------------------------------------------

@dataclass
class BootFiles:
    boot: int
    blocks: list[tuple[int, Path]] = field(default_factory=list)  # (first block seq, file)
    log: Path | None = None

    @property
    def block_bytes(self) -> int:
        return sum(p.stat().st_size for _, p in self.blocks)


def recordings(card: Path) -> dict[int, BootFiles]:
    rec = card / REC_DIR
    boots: dict[int, BootFiles] = {}
    if not rec.is_dir():
        return boots
    for p in rec.iterdir():
        m = _KRC.match(p.name)
        if m:
            boots.setdefault(int(m.group(1)), BootFiles(int(m.group(1)))).blocks.append((int(m.group(2)), p))
            continue
        m = _LOG.match(p.name)
        if m:
            boots.setdefault(int(m.group(1)), BootFiles(int(m.group(1)))).log = p
    for b in boots.values():
        b.blocks.sort()
    return dict(sorted(boots.items()))


def log_tail(path: Path, n: int) -> list[str]:
    """The last n log lines of a saved log ("t_ms L text"; the "M" metrics lines left out)."""
    out: list[str] = []
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return out
    for line in text.splitlines():
        t, _, rest = line.partition(" ")
        tag, _, msg = rest.partition(" ")
        if tag == "M" or not t.isdigit():
            continue
        out.append(f"{int(t) / 1000:9.3f} {tag} {msg}")
    return out[-n:] if n > 0 else []


def status_lines(card: Path, lines: int = 40) -> list[str]:
    from kilo.tools import body_image as BI

    out: list[str] = []
    img = card / KERNEL_FILE
    if img.is_file():
        data = img.read_bytes()
        out.append(f"kernel7.img: {len(data)} bytes, crc32 {zlib.crc32(data):08x}, "
                   + ("the mind kernel" if has_marker(data) else "NOT the mind kernel (the netboot loader?)")
                   + "".join(f"; {n} is left from before (the next write removes it)" for n in KERNEL_STALE
                             if (card / n).exists()))
    else:
        out.append("kernel7.img: MISSING: the Pi cannot boot this card")
    man = None
    try:
        man = BI.Manifest.from_json((card / BI.MANIFEST_FILE).read_text())
    except OSError:
        out.append("body.json: none: no body firmware on the card (the mind flashes nothing)")
    except BI.ImageError as e:
        out.append(f"body.json: unreadable ({e})")
    if man is not None:
        try:
            BI.check_image((card / BI.IMAGE_FILE).read_bytes(), man)
            good = "body.bin matches it"
        except (OSError, BI.ImageError) as e:
            good = f"body.bin does NOT match it: {e}"
        out.append(f"body.json: build {man.build:08x}, {man.size} bytes, crc32 {man.crc32:08x}; {good}")
    r = read_result(card)
    if r is None:
        if (card / RESULT_FILE).exists():
            out.append(f"{RESULT_FILE}: there, but not a result file the mind wrote")
        elif man is not None:
            out.append(f"{RESULT_FILE}: none: the Pi has not booted from this card since kilo card wrote it (or it "
                       "could not write to the card)")
    else:
        about = ""
        if man is not None:
            now = f"build {man.build:08x} size {man.size} crc32 {man.crc32:08x}"
            about = (" (about the body.json on the card now)" if r.get("image") == now
                     else " (about ANOTHER image than the body.json on the card now)")
        out.append(f"{RESULT_FILE}: outcome {r.get('outcome', '?').upper()}{about}")
        for k in ("image", "why", "body before", "body after", "error", "battery", "mind", "next boot"):
            if k in r:
                out.append(f"  {k}: {r[k]}")
        if r.get("log"):
            out.append("  its log:")
            out += ["    " + x for x in r["log"].splitlines()]
    boots = recordings(card)
    save_on = (card / REC_DIR / "SAVE_ON").exists()
    if not boots:
        out.append(f"{REC_DIR}/: " + ("no recordings yet" if (card / REC_DIR).is_dir() else "none")
                   + ("" if save_on else " (saving to the card is off: kilo record save on --host, or build with "
                                         "KILO_RECORD_SAVE=1)"))
        return out
    out.append(f"{REC_DIR}/: saving {'on' if save_on else 'off'}; recordings of {len(boots)} boots "
               "(kilo card PATH --fetch copies them):")
    for b in boots.values():
        out.append(f"  boot {b.boot}: {len(b.blocks)} block files, {b.block_bytes / 1048576:.1f} MB"
                   + (", a saved log" if b.log else ""))
    last = next((b for b in reversed(list(boots.values())) if b.log), None)
    if last is not None and lines > 0:
        tail = log_tail(last.log, lines)  # type: ignore[arg-type]
        out.append(f"the last {len(tail)} lines of boot {last.boot}'s saved log ({last.log}):")
        out += ["  " + x for x in tail]
    return out


def fetch(card: Path, out: Path, say: Callable[[str], None] = print) -> list[Path]:
    """Every boot's recordings as rec-card-bN.krec, each run in them as run-bN-<s>s-<kind>-<id>.krec (+ a
    .kilolog for a walk), each saved log as kilo-card-bN.log: the names an earlier boot's fetch over the
    network gives."""
    from kilo.logs import flight, walkrec

    written: list[Path] = []
    out.mkdir(parents=True, exist_ok=True)
    now = time.time()
    for b in recordings(card).values():
        if b.log is not None:
            dst = out / f"kilo-card-b{b.boot}.log"
            dst.write_bytes(b.log.read_bytes())
            written.append(dst)
            say(f"boot {b.boot}: saved log -> {dst}")
        if not b.blocks:
            continue
        data = b"".join(p.read_bytes() for _, p in b.blocks)
        meta = {"mind": f"sd card {card}", "source": "kilo card --fetch", "boot": b.boot, "what": "blocks",
                "format": P.HOST_WALK_FORMAT, "fetched_wall": round(now, 3)}
        rec = flight.write_file(out / f"rec-card-b{b.boot}.krec", meta, data)
        written.append(rec)
        s = flight.parse_blocks(data, meta)
        say(f"boot {b.boot}: {len(data)} bytes, {len(s.blocks)} blocks, {len(s.runs)} runs -> {rec}"
            + (f" ({len(s.bad_blocks)} damaged blocks skipped)" if s.bad_blocks else ""))
        good = [k for k in s.blocks if not k.problem or k.problem == "cut"]
        for run in s.runs:
            lo, hi = flight.span_of(s, run)
            # the blocks its span touches: a block runs from its t0 to the next one's
            parts = [data[k.offset:k.offset + k.have] for i, k in enumerate(good)
                     if k.t0_us <= hi and (i + 1 == len(good) or good[i + 1].t0_us >= lo)]
            start_s = int((run.begin_us if run.begin_us is not None else lo) / 1e6)
            stem = out / f"run-b{b.boot}-{start_s}s-{run.kind}-{run.id}"
            rmeta = dict(meta, kind=run.kind, run=run.id)
            path = flight.write_file(stem.with_suffix(".krec"), rmeta, b"".join(parts))
            written.append(path)
            extra = ""
            if run.kind == "walk":
                try:
                    part = flight.parse_blocks(b"".join(parts), rmeta)
                    kl = walkrec.to_kilolog(flight.walk_of(part, part.run(run.id)), stem.with_suffix(".kilolog"))
                    written.append(kl)
                    extra = f" (+ {kl.name})"
                except (ValueError, KeyError) as e:
                    extra = f" (no .kilolog: {e})"
            end = "" if run.end_us is not None else " (its end is not on the card: cut off)"
            say(f"  {run.kind} {run.id}: -> {path}{extra}{end}")
    if not written:
        say(f"no recordings on {card} ({REC_DIR}/ is empty or missing)")
    return written


# ---- the command ----------------------------------------------------------------------------

def cmd_card(args: argparse.Namespace, run: Callable[..., Any] = subprocess.run) -> int:
    card: Path = args.path
    try:
        warnings = check_card(card, args.force, args.force_dir)
    except CardError as e:
        print(f"kilo card: {e}", file=sys.stderr)
        return EXIT_ERR
    for w in warnings:
        print(f"kilo card: warning: {w} (--force)", file=sys.stderr)
    if args.status or args.fetch:
        if args.make_vars:
            print("kilo card: build options are for writing the card, not --status or --fetch", file=sys.stderr)
            return EXIT_ERR
        if args.status:
            print("\n".join(status_lines(card, args.lines)))
            return EXIT_OK
        try:
            fetch(card, Path(args.out).expanduser())
        except OSError as e:
            print(f"kilo card --fetch: {e}", file=sys.stderr)
            return EXIT_ERR
        return EXIT_OK
    try:
        plan = prepare(args, run=run)
        for w in plan.warnings:
            print(f"kilo card: warning: {w}", file=sys.stderr)
        write_card(card, plan)
    except CardError as e:
        print(f"kilo card: {e}", file=sys.stderr)
        return EXIT_ERR
    built = plan.kernel is not None and args.kernel is None and not args.no_build
    print("\n".join(next_boot_text(plan, built)).replace("kilo card PATH", f"kilo card {card}"))
    print(f"Unmount (eject) {card} before taking the card out.")
    return EXIT_OK
