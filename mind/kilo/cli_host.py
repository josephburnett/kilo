"""`kilo --host <ip[:port]> COMMAND`: the CLI over the mind's host channel
(docs/fast-boot.md §5, kilo.hostlink) instead of a serial port or the daemon.

What a C++ mind (the Circle kernel, or kernel/host's kilo-mind-host) serves
today, and so what works here:

    kilo --host IP status [--json]                 the mind, the body's HELLO and state, link counters
    kilo --host IP posture NAME [--transition T] [--for S]   hold a posture; REST when it ends
    kilo --host IP monitor [--rate HZ] [--duration S] [--json]   live telemetry
    kilo log --host IP [--lines N] [--level L] [--duration S] [--no-follow] [--save [DIR]]
                                                   the mind's log: all it kept (or the last N lines), then live
    kilo log --host IP --previous [BOOT] [--save [DIR]]   an earlier boot's log, saved on the SD card
    kilo --host IP flash [--persist] [--image body.bin [--manifest body.json]]   build and flash the body (P4)
    kilo reboot --host IP                          restart the mind (on the Pi: back to the SD card's boot,
                                                   e.g. the netboot loader); the body gets REST first
    kilo update --host IP [--image PATH] [--force] [--wait S]   write a new kernel onto the mind's SD card;
                                                   it reboots into it, and this waits for it to come back
    kilo runs --host IP [--json] | --fetch ... | --delete ...   the walks and tricks recorded (kilo.cli_record;
                                                   kilo walks is the same), record, bench sd, metrics

``--host`` may also follow the command (``kilo status --host IP``). Every
other command prints that it is not available over --host yet.

Every connection needs this laptop's key (kilo.hostkey: ``kilo key init``
once, its line in kernel/keys/authorized_keys, the kernel rebuilt and
installed); ``--key PATH`` or ``$KILO_HOST_KEY`` picks another key file. No
key file, or a mind that does not know the key: one line saying what to
do, exit status 1.

``posture`` takes control with keepalive and releases it at the end (Ctrl-C,
``--for``); the mind then latches REST, as the daemon does when a hold ends.
If this process dies instead, the mind sees the socket close (or, if the
machine vanished, two seconds without a keepalive) and does the same.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import zlib
from pathlib import Path
from typing import Any

from kilo import cli_record, hostkey
from kilo.hostlink import REBOOT_CLOSE_WAIT_S, HostError, HostKeyRefused, HostLink, HostRefused, has_kernel_marker
from kilo.protocol import kilo_protocol as P

EXIT_OK, EXIT_ERR = 0, 1
LEVEL_TAG = "DIWE"
# `kilo update`'s default image: the mind kernel `make -C kernel` builds in this checkout.
DEFAULT_KERNEL_IMAGE = Path(__file__).resolve().parents[2] / "kernel" / "build" / "kernel7.img"
# After an update: how often `kilo update` tries to reach the rebooted mind.
UPDATE_POLL_S = 0.5


def _flags(value: int, cls: Any) -> list[str]:
    return [f.name for f in cls if value & f]


def _name(cls: Any, v: int) -> str:
    try:
        return cls(v).name
    except ValueError:
        return str(v)


def status_dict(st: Any) -> dict[str, Any]:
    """`kilo --host status --json`: the STATUS reply as plain data."""
    d = P.to_dict(st)
    d["state"] = _flags(st.state, P.HostState)
    d["controller"] = _name(P.HostController, st.controller)
    d["mind_build"] = f"{st.mind_build:08x}"
    if P.HostState.HAS_HELLO & st.state:
        d["hello"]["fw_build"] = f"{st.hello.fw_build:08x}"
    else:
        d["hello"] = None
    if not P.HostState.HAS_TELEMETRY & st.state:
        d["telemetry"] = None
    if st.event_count == 0:
        d["last_event"] = None
    return d


def print_status(st: Any) -> None:
    s, h, t = st.state, st.hello, st.telemetry
    up = st.uptime_ms / 1000
    print(f"mind        build {st.mind_build:08x}  up {up:.1f} s  host channel v{st.host_version}  "
          f"clients {st.clients}  controller {_name(P.HostController, st.controller)}")
    print(f"link        connected={bool(s & P.HostState.LINK_CONNECTED)} alive={bool(s & P.HostState.BODY_ALIVE)} "
          f"contact={bool(s & P.HostState.CONTACT)}" + ("  BRIDGED (link paused)" if s & P.HostState.BRIDGE else ""))
    if s & P.HostState.HAS_HELLO:
        print(f"firmware    proto {h.proto_major}.{h.proto_minor}  build {h.fw_build:08x}  reset "
              f"{_name(P.ResetCause, h.reset_cause)}  config {_name(P.ConfigState, h.config_state)}"
              + ("" if st.proto_ok else "  PROTOCOL MISMATCH: the mind refuses to drive"))
    else:
        print("firmware    no HELLO yet")
    if s & P.HostState.HAS_TELEMETRY:
        age = "" if st.telemetry_age_ms == 0xFFFF else f"  age {st.telemetry_age_ms} ms"
        print(f"mode        {_name(P.Mode, t.mode)}  epoch {t.epoch}  flags "
              f"{','.join(_flags(t.flags, P.TelemetryFlags)) or '-'}{age}")
        print(f"attitude    pitch {t.pitch / 16:+7.2f}°  roll {t.roll / 16:+7.2f}°")
        print(f"battery     {t.batt_mv} mV")
        print(f"distance    {t.dist_mm or '-'} mm")
    else:
        print("mode        no telemetry yet")
    if st.event_count:
        e = st.last_event
        print(f"events      {st.event_count}, last {_name(P.EventCode, e.code)}({e.arg}) at tick {e.tick}")
    print(f"body        reboots {st.reboots}  telemetry frames {st.telemetry_count}")
    print("link stats  " + " ".join(f"{k}={v}" for k, v in P.to_dict(st.link).items() if v))


# ---- commands ---------------------------------------------------------------------
def _extras(hl: HostLink) -> tuple[Any, list[str]]:
    """REC_STATUS and the metrics lines, or (None, []) from a mind without them."""
    rs, lines = None, []
    try:
        rs = hl.rec_status()
    except (HostRefused, HostError):
        pass
    try:
        lines = hl.metrics()
    except (HostRefused, HostError):
        pass
    return rs, lines


def cmd_status(args: argparse.Namespace, hl: HostLink) -> int:
    st = hl.status()
    rs, lines = _extras(hl)
    if args.json:
        d = status_dict(st)
        if rs is not None:
            d["recorder"] = P.to_dict(rs)
            d["recorder"]["card_state"] = _name(P.RecordCardState, rs.card_state)
        if lines:
            d["metrics"] = lines
        print(json.dumps(d, indent=2))
    else:
        print_status(st)
        if rs is not None:
            print(cli_record.recorder_line(rs))
        for ln in lines:
            print(f"metrics     {ln}")
    return EXIT_OK


def cmd_posture(args: argparse.Namespace, hl: HostLink) -> int:
    for flag, attr in (("--pca", "pca"), ("--then-watch", "then_watch")):
        if getattr(args, attr, None) not in (None, False):
            return _unavailable(f"posture {flag}")
    try:
        posture = P.Posture[args.name.upper()]
    except KeyError:
        return _unavailable(f"posture {args.name}")
    hl.take_control(keepalive=True)
    hl.command(P.HostPosture(posture=int(posture), transition_ticks=args.transition, flags=0))
    print(f"holding {args.name}" + (f" for {args.hold:g} s" if args.hold else " (Ctrl-C to release)"), flush=True)
    end = None if args.hold is None else time.monotonic() + args.hold
    try:
        while end is None or time.monotonic() < end:
            if hl.closed:
                print(f"kilo posture: lost the mind: {hl.error}", file=sys.stderr)
                return EXIT_ERR
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    hl.release_control()  # the mind latches REST
    print("released: the mind latched REST")
    return EXIT_OK


def telem_record(m: Any) -> dict[str, Any]:
    """One `kilo --host monitor --json` line."""
    t = m.telemetry
    has = bool(m.state & P.HostState.HAS_TELEMETRY)
    return {"time": round(time.time(), 3), "mind_ms": m.uptime_ms, "alive": bool(m.state & P.HostState.BODY_ALIVE),
            "tick": t.tick if has else None, "mode": _name(P.Mode, t.mode) if has else None,
            "epoch": t.epoch if has else None, "pitch_deg": t.pitch / 16 if has else None,
            "roll_deg": t.roll / 16 if has else None, "batt_mv": t.batt_mv if has else None,
            "age_ms": None if m.telemetry_age_ms == 0xFFFF else m.telemetry_age_ms,
            "frames_rx": m.link.frames_rx, "decode_errors": m.link.decode_errors}


def cmd_monitor(args: argparse.Namespace, hl: HostLink) -> int:
    for flag, attr in (("--passive", "passive"), ("--percepts", "percepts")):
        if getattr(args, attr, False):
            return _unavailable(f"monitor {flag}")
    table = sys.stdout.isatty() and not args.json

    def show(m: Any) -> None:
        r = telem_record(m)
        if table:
            sys.stdout.write("\x1b[H\x1b[2J" + "\n".join(f"{k:>14}  {v}" for k, v in r.items())
                             + "\n(Ctrl-C to quit)\n")
            sys.stdout.flush()
        else:
            print(json.dumps(r), flush=True)

    hl.subscribe_telem(min(max(args.rate, 1.0), P.HOST_TELEM_MAX_HZ), show)
    return _run_for(args.duration, hl)


STATUS_BIT = 0x80000000
ALL_LINES = 255  # LOG_SUB tail: everything the mind kept
LOG_IDLE_S = 0.5  # --no-follow: the history is over once no line came for this long


def _log_file(save: str | None, suffix: str = "") -> Any:
    if save is None:
        return None
    d = Path(save).expanduser()
    d.mkdir(parents=True, exist_ok=True)
    path = d / f"kilo-{time.strftime('%Y%m%d-%H%M%S')}{suffix}.log"
    print(f"(saving to {path})", file=sys.stderr, flush=True)
    return open(path, "w", encoding="utf-8")


def cmd_log(args: argparse.Namespace, hl: HostLink) -> int:
    """`kilo log --host`: the history the mind kept (all of it by default),
    how many older lines it dropped, then live. Status lines (bit 31 of the
    seq: the once-a-second counters, percepts, held keys) are live only and
    not counted. --save copies what is printed to a file; --no-follow stops
    once the history is shown; --previous prints an earlier boot's log from
    the SD card."""
    if getattr(args, "previous", None) is not None:
        return _previous_log(args, hl)
    out = _log_file(args.save)
    st = {"first": True, "last": None, "lost": 0, "at": time.monotonic(), "lines": 0}
    explicit = args.lines is not None
    tail = ALL_LINES if not explicit else args.lines

    def emit(text: str) -> None:
        print(text, flush=True)
        if out is not None:
            out.write(text + "\n")
            out.flush()

    def show(m: Any) -> None:
        st["at"] = time.monotonic()
        status = bool(m.seq & STATUS_BIT)
        if not status:
            if st["first"]:
                st["first"] = False
                if not explicit and m.seq > 0:
                    emit(f"(the mind dropped {m.seq} older lines; it keeps the last {P.HOST_LOG_RING_LINES})")
            elif st["last"] is not None and m.seq > st["last"] + 1:
                gap = m.seq - st["last"] - 1
                st["lost"] += gap
                emit(f"({gap} lines lost here: the ring overwrote them before they were sent)")
            st["last"] = m.seq
            st["lines"] += 1
        tag = LEVEL_TAG[m.level] if m.level < len(LEVEL_TAG) else "?"
        emit(f"{m.t_ms // 1000:6d}.{m.t_ms % 1000:03d} {tag} {m.text.decode('utf-8', 'replace')}")

    try:
        hl.subscribe_log(show, tail=tail, min_level=P.LogLevel[args.level.upper()])
        if args.no_follow:
            end = None if args.duration is None else time.monotonic() + args.duration
            while time.monotonic() - st["at"] < LOG_IDLE_S and not hl.closed:
                if end is not None and time.monotonic() >= end:
                    break
                time.sleep(0.05)
            return EXIT_OK
        return _run_for(args.duration, hl)
    finally:
        if out is not None:
            out.close()


def _previous_log(args: argparse.Namespace, hl: HostLink) -> int:
    try:
        rs = hl.rec_status()
    except HostRefused as e:
        print(f"kilo log --previous: this mind keeps no log on its card ({e.code_name})", file=sys.stderr)
        return EXIT_ERR
    boot = args.previous if args.previous > 0 else rs.boot - 1
    if rs.boot == 0 and args.previous <= 0:
        print("kilo log --previous: the mind has not read its SD card (no card, or not yet): which boot? "
              "(--previous BOOT)", file=sys.stderr)
        return EXIT_ERR
    if boot <= 0:
        print(f"kilo log --previous: this is boot {rs.boot}, the first the card knows: no earlier log", file=sys.stderr)
        return EXIT_ERR
    try:
        data = hl.rec_fetch(0, 0xFFFFFFFF, boot=boot, what=P.RecFetchWhat.LOG)
    except HostRefused as e:
        print(f"kilo log --previous: no saved log for boot {boot} on the card ({e})", file=sys.stderr)
        return EXIT_ERR
    out = _log_file(args.save, f"-boot{boot}")
    text = data.decode("utf-8", "replace")
    print(f"(boot {boot}'s log as saved on the SD card: {text.count(chr(10))} lines; this is boot {rs.boot})")
    try:
        for ln in text.splitlines():
            t, _, rest = ln.partition(" ")
            try:
                ms = int(t)
                ln = f"{ms // 1000:6d}.{ms % 1000:03d} {rest}"
            except ValueError:
                pass
            print(ln)
            if out is not None:
                out.write(ln + "\n")
    finally:
        if out is not None:
            out.close()
    return EXIT_OK


def _run_for(duration: float | None, hl: HostLink) -> int:
    end = None if duration is None else time.monotonic() + duration
    try:
        while end is None or time.monotonic() < end:
            if hl.closed:
                print(f"kilo: lost the mind: {hl.error}", file=sys.stderr)
                return EXIT_ERR
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    return EXIT_OK


def _unavailable(what: str) -> int:
    print(f"kilo {what}: not available over --host yet (the C++ mind serves status, posture, monitor, log, flash, "
          "reboot, update, runs, record, bench, metrics)",
          file=sys.stderr)
    return EXIT_ERR


def cmd_flash(args: argparse.Namespace, hl: HostLink) -> int:
    """`kilo flash --host IP`: build (or --image), send, flash through the
    mind, then check the body's HELLO in STATUS. The image is prepared in
    run_host before connecting (a build takes minutes; the mind would drop
    a silent connection's keepalive)."""
    from kilo.tools.flash import FlashError, describe_flash_status, flash_host

    data, m = args.flash_image
    try:
        st = flash_host(hl, data, m, persist=args.persist, timeout=args.flash_timeout)
    except FlashError as e:
        print(f"kilo flash: {e}", file=sys.stderr)
        return EXIT_ERR
    if st.phase != P.FlashPhase.DONE:
        print(f"kilo flash: FAILED: {describe_flash_status(st)}", file=sys.stderr)
        _release(hl)
        return EXIT_ERR
    # The end check: what the mind's link now knows of the body.
    s = hl.status()
    ok = bool(s.state & P.HostState.HAS_HELLO) and s.hello.fw_build == m.build
    seen = (f"build {s.hello.fw_build:08x} reset {_name(P.ResetCause, s.hello.reset_cause)}"
            if s.state & P.HostState.HAS_HELLO else "no HELLO")
    _release(hl)
    if not ok:
        print(f"kilo flash: verify FAILED: the mind's link reports {seen}, want build {m.build:08x}", file=sys.stderr)
        return EXIT_ERR
    print(f"verified: HELLO {seen}" + ("; the mind keeps the image on SD" if args.persist else ""))
    return EXIT_OK


def cmd_reboot(args: argparse.Namespace, hl: HostLink) -> int:
    """`kilo reboot --host IP`: take control, REBOOT, print the answer. The
    connection dropping afterwards is the reboot, not an error."""
    hl.take_control(keepalive=False)
    r = hl.reboot()
    text = r.text.decode("ascii", "replace")
    print(f"REBOOT: {_name(P.HostResultCode, r.code)}" + (f" ({text})" if text else ""), flush=True)
    if hl.wait_closed(REBOOT_CLOSE_WAIT_S):
        print("the mind closed the connection: it is restarting")
    else:
        print(f"no close within {REBOOT_CLOSE_WAIT_S:g} s (a rebooting Pi goes silent): it is restarting")
    return EXIT_OK


# What each UPDATE_STATUS error means for the card (kernel/mind/update/updater.h).
UPDATE_ERROR_HELP = {
    "BAD_SIZE": "the image is empty or larger than the mind's buffer",
    "BAD_OFFSET": "the image arrived out of order",
    "SHORT": "the image arrived incomplete",
    "CRC_MISMATCH": "the image arrived damaged",
    "NOT_A_KERNEL": f"no {P.HOST_UPDATE_MARKER} in it: not the mind kernel (--force installs it anyway)",
    "WRITE_FAILED": "writing kernel7.new failed; the card's kernel7.img is unchanged",
    "VERIFY_FAILED": "kernel7.new read back differs; the card's kernel7.img is unchanged",
    "INSTALL_FAILED": "a rename failed; the card's kernel7.img is still the old kernel",
    "CARD_BROKEN": "the card has NO kernel7.img now: rewrite it in a card reader (docs/fast-boot-first-boot.md)",
}


def describe_update_status(st: Any) -> str:
    err = _name(P.UpdateError, st.error)
    help_ = UPDATE_ERROR_HELP.get(err, "")
    return f"{err}" + (f": {help_}" if help_ else "") + f" ({st.received} of {st.image_size} bytes received)"


def cmd_update(args: argparse.Namespace, hl: HostLink) -> int:
    """`kilo update --host IP`: the image (read in run_host), the mind's build
    before, the update with progress lines, then the wait for the rebooted
    mind and its build."""
    data: bytes = args.update_image
    before = hl.status().mind_build
    print(f"update: mind build {before:08x}; sending {args.image_path} ({len(data)} bytes, crc32 "
          f"{zlib.crc32(data):08x}){' with --force' if args.force else ''}", flush=True)
    shown: dict[str, int] = {}

    def progress(st: Any) -> None:
        # A phase's last slice and the next phase's start arrive as one status
        # (the latest): each phase's first status says the one before is done.
        phase = _name(P.UpdatePhase, st.phase)
        first = phase not in shown
        if first and phase == "CHECKING":
            print(f"update: the mind checked the image (size, crc32 {st.crc32:08x}"
                  f"{'' if args.force else ', marker'}): writing it to the SD card", flush=True)
        elif first and phase == "VERIFYING":
            print(f"update: kernel7.new written ({st.image_size} bytes); reading it back", flush=True)
        elif first and phase == "INSTALLING":
            print(f"update: kernel7.new read back: {st.done} bytes, crc32 {st.card_crc32:08x}; installing: "
                  "kernel7.img -> kernel7.old, kernel7.new -> kernel7.img", flush=True)
        if phase in ("WRITING", "VERIFYING") and st.done > 0:
            step = st.done * 4 // max(1, st.image_size)  # quarters
            if shown.get(phase) != step:
                verb = "writing kernel7.new" if phase == "WRITING" else "reading kernel7.new back"
                print(f"update: {verb}: {st.done} of {st.image_size} bytes", flush=True)
            shown[phase] = step
        shown.setdefault(phase, -1)

    try:
        st = hl.update(data, force=args.force, progress=progress)
    except HostRefused as e:
        if e.code == P.HostResultCode.IMAGE_BAD and e.text:
            print(f"kilo update: FAILED: the mind refused the image: {e.text}", file=sys.stderr)
        else:
            print(f"kilo update: the mind refused: {e}", file=sys.stderr)
        _release(hl)
        return EXIT_ERR
    if st.phase != P.UpdatePhase.DONE:
        print(f"kilo update: FAILED: {describe_update_status(st)}", file=sys.stderr)
        _release(hl)
        return EXIT_ERR
    print(f"update: DONE: kernel7.img on the SD card is the new kernel ({st.image_size} bytes, crc32 "
          f"{st.card_crc32:08x} read back); the previous one is kernel7.old", flush=True)
    print("update: the mind is rebooting into it", flush=True)
    hl.wait_closed(REBOOT_CLOSE_WAIT_S)
    hl.close()
    if args.wait <= 0:
        return EXIT_OK
    return _wait_for_mind(args.host, args.wait, before, getattr(args, "key", None))


def _wait_for_mind(addr: str, wait_s: float, before: int, key: Any = None) -> int:
    """Poll the mind's port until it answers STATUS again (a Pi reboots and
    boots the card in a few seconds); print its build."""
    start = time.monotonic()
    while True:
        left = wait_s - (time.monotonic() - start)
        if left <= 0:
            print(f"kilo update: the mind did not come back within {wait_s:g} s (port {addr}); the card has the "
                  "new kernel: check the Pi (its LED, `kilo status --host`)", file=sys.stderr)
            return EXIT_ERR
        try:
            with HostLink.connect(addr, timeout=min(1.0, max(0.1, left)), key=key) as hl:
                build = hl.status().mind_build
        except HostKeyRefused as e:
            print(f"kilo update: the mind is back but refuses this laptop: {e}", file=sys.stderr)
            return EXIT_ERR
        except (HostError, OSError):
            time.sleep(min(UPDATE_POLL_S, max(0.0, left)))
            continue
        took = time.monotonic() - start
        same = "  (the same build as before)" if build == before else f"  (was {before:08x})"
        print(f"update: the mind is back after {took:.1f} s: mind build {build:08x}{same}")
        return EXIT_OK


def _prepare_update(args: argparse.Namespace) -> int | None:
    """Read and check the image for `kilo update` (args.update_image); an exit code on failure."""
    path = Path(args.image) if args.image is not None else DEFAULT_KERNEL_IMAGE
    args.image_path = path
    try:
        data = path.read_bytes()
    except OSError as e:
        hint = " (build it: make -C kernel)" if args.image is None else ""
        print(f"kilo update: cannot read {path}: {e.strerror}{hint}", file=sys.stderr)
        return EXIT_ERR
    if not 0 < len(data) <= P.HOST_UPDATE_MAX_IMAGE:
        print(f"kilo update: {path} is {len(data)} bytes; the mind takes 1..{P.HOST_UPDATE_MAX_IMAGE}",
              file=sys.stderr)
        return EXIT_ERR
    if not args.force and not has_kernel_marker(data):
        print(f"kilo update: {path} has no {P.HOST_UPDATE_MARKER}: it is not the mind kernel (the netboot loader "
              "is not either). To install it anyway: --force", file=sys.stderr)
        return EXIT_ERR
    if not args.force:
        # The kernel has its authorized keys in its image as raw bytes
        # (build/gen/authorized_keys.h): one without this laptop's would lock it out.
        try:
            key = hostkey.load(getattr(args, "key", None))
        except hostkey.HostKeyError:
            key = None  # the connection says what is wrong with it
        if key is not None and key.public not in data:
            print(f"kilo update: {path} does not have this laptop's key ({key.fingerprint}) built in: the new "
                  f"kernel would refuse this laptop. Add `kilo key show`'s line to {hostkey.KEYS_FILE} and rebuild "
                  "(make -C kernel); to install it anyway: --force", file=sys.stderr)
            return EXIT_ERR
    args.update_image = data
    return None


def _release(hl: HostLink) -> None:
    try:
        hl.release_control()  # the mind latches REST
    except HostError:
        pass


def _prepare_flash(args: argparse.Namespace) -> int | None:
    """Build or read the image for `kilo flash --host` (args.flash_image); an exit code on failure."""
    from kilo.cli import flash_image_options
    from kilo.tools.flash import FlashError, prepare_image

    if args.legacy or args.hex or args.sd is not None or args.rollback:
        print("kilo flash --host: not with --legacy, --hex, --sd or --rollback (use --image/--manifest)",
              file=sys.stderr)
        return EXIT_ERR
    try:
        args.flash_image = prepare_image(flash_image_options(args))
    except FlashError as e:
        print(f"kilo flash: {e}", file=sys.stderr)
        return EXIT_ERR
    return None


HOST_COMMANDS = {"status": cmd_status, "posture": cmd_posture, "monitor": cmd_monitor, "log": cmd_log,
                 "flash": cmd_flash, "reboot": cmd_reboot, "update": cmd_update, **cli_record.HOST_COMMANDS}


def parse_log_argv(argv: list[str], host: str | None, key: str | None = None) -> argparse.Namespace:
    """`kilo log` keeps its argv for kilo.logs (summary|replay|plot); over --host it is the live tail."""
    p = argparse.ArgumentParser(prog="kilo log --host", description="the mind's log: the last lines, then live")
    p.add_argument("--host", default=host, required=host is None, help="the mind's address, IP[:PORT]")
    p.add_argument("--key", default=key, metavar="PATH", help="this laptop's key file (kilo key init)")
    p.add_argument("--lines", "-n", type=int, default=None, help="lines of history first (default: all the mind "
                                                                 f"kept, up to {P.HOST_LOG_RING_LINES})")
    p.add_argument("--level", choices=[lv.name.lower() for lv in P.LogLevel], default="debug")
    p.add_argument("--duration", type=float, default=None, help="stop after S seconds (default: Ctrl-C)")
    p.add_argument("--no-follow", action="store_true", help="stop once the history is shown")
    p.add_argument("--save", nargs="?", const=cli_record.DEFAULT_OUT, default=None, metavar="DIR",
                   help=f"also write what is shown to DIR/kilo-<time>.log (default DIR {cli_record.DEFAULT_OUT})")
    p.add_argument("--previous", nargs="?", type=int, const=0, default=None, metavar="BOOT",
                   help="an earlier boot's log from the SD card (default: the boot before this one)")
    return p.parse_args(argv)


def wants_host(args: argparse.Namespace) -> bool:
    return bool(getattr(args, "host", None)) or (args.cmd == "log" and "--host" in (args.log_argv or []))


def run_host(args: argparse.Namespace) -> int:
    """main() for `kilo --host ...`."""
    if args.cmd == "log":
        log_args = parse_log_argv(list(args.log_argv or []), getattr(args, "host", None), getattr(args, "key", None))
        args.host, args.lines, args.level, args.duration = (log_args.host, log_args.lines, log_args.level,
                                                            log_args.duration)
        args.no_follow, args.save, args.previous = log_args.no_follow, log_args.save, log_args.previous
        args.key = log_args.key
    fn = HOST_COMMANDS.get(args.cmd)
    if fn is None:
        return _unavailable(args.cmd)
    if args.cmd == "flash":
        err = _prepare_flash(args)
        if err is not None:
            return err
    if args.cmd == "update":
        err = _prepare_update(args)
        if err is not None:
            return err
    try:
        hl = HostLink.connect(args.host, timeout=args.timeout, key=getattr(args, "key", None))
    except (HostError, ValueError) as e:
        print(f"kilo {args.cmd}: {e}", file=sys.stderr)
        return EXIT_ERR
    try:
        return fn(args, hl)
    except HostRefused as e:
        print(f"kilo {args.cmd}: the mind refused: {e}", file=sys.stderr)
        return EXIT_ERR
    except HostError as e:
        print(f"kilo {args.cmd}: {e}", file=sys.stderr)
        return EXIT_ERR
    finally:
        hl.close()

