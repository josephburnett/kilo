"""`kilo --host <ip[:port]> COMMAND`: the CLI over the mind's host channel
(docs/fast-boot.md §5, kilo.hostlink) instead of a serial port or the daemon.

What a C++ mind (the Circle kernel, or kernel/host's kilo-mind-host) serves
today, and so what works here:

    kilo --host IP status [--json]                 the mind, the body's HELLO and state, link counters
    kilo --host IP posture NAME [--transition T] [--for S]   hold a posture; REST when it ends
    kilo --host IP monitor [--rate HZ] [--duration S] [--json]   live telemetry
    kilo log --host IP [--lines N] [--level L] [--duration S]    the mind's log: the last N lines, then live
    kilo --host IP flash [--persist] [--image body.bin [--manifest body.json]]   build and flash the body (P4)
    kilo reboot --host IP                          restart the mind (on the Pi: back to the SD card's boot,
                                                   e.g. the netboot loader); the body gets REST first

``--host`` may also follow the command (``kilo status --host IP``). Every
other command prints that it is not available over --host yet.

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
from typing import Any

from kilo.hostlink import REBOOT_CLOSE_WAIT_S, HostError, HostLink, HostRefused
from kilo.protocol import kilo_protocol as P

EXIT_OK, EXIT_ERR = 0, 1
LEVEL_TAG = "DIWE"


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
def cmd_status(args: argparse.Namespace, hl: HostLink) -> int:
    st = hl.status()
    if args.json:
        print(json.dumps(status_dict(st), indent=2))
    else:
        print_status(st)
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


def cmd_log(args: argparse.Namespace, hl: HostLink) -> int:
    def show(m: Any) -> None:
        tag = LEVEL_TAG[m.level] if m.level < len(LEVEL_TAG) else "?"
        print(f"{m.t_ms // 1000:6d}.{m.t_ms % 1000:03d} {tag} {m.text.decode('utf-8', 'replace')}", flush=True)

    hl.subscribe_log(show, tail=args.lines, min_level=P.LogLevel[args.level.upper()])
    return _run_for(args.duration, hl)


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
          "reboot)",
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
                 "flash": cmd_flash, "reboot": cmd_reboot}


def parse_log_argv(argv: list[str], host: str | None) -> argparse.Namespace:
    """`kilo log` keeps its argv for kilo.logs (summary|replay|plot); over --host it is the live tail."""
    p = argparse.ArgumentParser(prog="kilo log --host", description="the mind's log: the last lines, then live")
    p.add_argument("--host", default=host, required=host is None, help="the mind's address, IP[:PORT]")
    p.add_argument("--lines", "-n", type=int, default=20, help="lines of history first (default 20; the mind keeps "
                                                               f"{P.HOST_LOG_RING_LINES})")
    p.add_argument("--level", choices=[lv.name.lower() for lv in P.LogLevel], default="debug")
    p.add_argument("--duration", type=float, default=None, help="stop after S seconds (default: Ctrl-C)")
    return p.parse_args(argv)


def wants_host(args: argparse.Namespace) -> bool:
    return bool(getattr(args, "host", None)) or (args.cmd == "log" and "--host" in (args.log_argv or []))


def run_host(args: argparse.Namespace) -> int:
    """main() for `kilo --host ...`."""
    if args.cmd == "log":
        log_args = parse_log_argv(list(args.log_argv or []), getattr(args, "host", None))
        args.host, args.lines, args.level, args.duration = (log_args.host, log_args.lines, log_args.level,
                                                            log_args.duration)
    fn = HOST_COMMANDS.get(args.cmd)
    if fn is None:
        return _unavailable(args.cmd)
    if args.cmd == "flash":
        err = _prepare_flash(args)
        if err is not None:
            return err
    try:
        hl = HostLink.connect(args.host, timeout=args.timeout)
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

