"""The `kilo` command line (docs/architecture.md Appendix G): the laptop's
tool for Kilo's C++ mind, over its host channel (kilo.cli_host), and for
what is done on this machine (keys, SD cards, downloaded recordings).

    kilo [--host IP[:PORT]] status [--json]   the mind, the body's HELLO and state, link counters
    kilo monitor [--rate HZ] [--duration S] [--json]   live telemetry
    kilo posture {limp,rest,sit,stand,stance,calib,zero} [--transition TICKS] [--for S]
    kilo cal show | set JOINT DEGREES | nudge JOINT DEGREES | save | revert   (kilo.cli_cal)
    kilo guard [on|off|status]   the distance guard (docs/distance-guard.md)
    kilo flash [--persist] [--image body.bin [--manifest body.json]] | --sd MOUNT   the body firmware, through
                                   the mind or onto its boot card (docs/fast-boot.md §6)
    kilo reboot   restart the mind (the Pi: back to its SD card's boot, e.g. the netboot loader)
    kilo update [--image kernel7.img] [--force] [--wait S]   a new kernel onto the mind's SD card
    kilo card PATH [NAME=VALUE ...] [--no-build] | --status | --fetch   the kernel and the body firmware onto Kilo's
                                   SD card in this laptop's reader; what the mind left there (kilo.cli_card)
    kilo runs [--fetch [N|all|last] [--out DIR]] [--delete BOOT|all]   the walks and tricks the
                                   mind recorded; download them (kilo walks: the same)
    kilo record status|fetch [--last 10m|--all ...]|save on|off|now   the flight recorder (kilo.cli_record)
    kilo record decode FILE [--csv OUT] [--json] [--log] [--plot PNG]   a downloaded recording, readable
    kilo bench sd   the SD card's write times;   kilo metrics   the session's counters
    kilo power | FILE.krec   light sleep and the battery's trend (kilo.cli_power, docs/power.md)
    kilo sound [status|list|play NAME|on|off|volume N]   Kilo's sounds (kilo.cli_sound, docs/sound.md)
    kilo measure plan|fit FILE...|fetch|simulate   Kilo's self-measurement, B1 (kilo.cli_measure,
                                   docs/self-measurement.md)
    kilo log summary|replay|plot|walk|trick|run FILE ...   (walk, trick, run: a .krec from kilo runs --fetch)
    kilo log [--lines N] [--previous [BOOT]] ... --host IP   the mind's log (kilo.cli_host)
    kilo key init [--name NAME] | show   this laptop's key for the mind (kilo.hostkey); --key PATH / $KILO_HOST_KEY
    kilo find [--name kilo.local] [--json]   Kilo's address and build, found by name (mDNS; kilo.cli_find)

Every command that talks to Kilo takes ``--host IP[:PORT]`` (before or after
the command; a .local name such as kilo.local is found by mDNS) and, without
it, finds kilo.local by mDNS.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Sequence

from kilo import EXIT_ERR, EXIT_OK, __version__
from kilo.protocol import kilo_protocol as P

EXIT_USAGE = 2


# ---- argument parsing -------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="kilo", description="Kilo mind command line")
    p.add_argument("--version", action="version", version=f"kilo {__version__}")
    p.add_argument("--timeout", type=float, default=3.0, help="seconds to wait for the mind")
    p.add_argument("-v", "--verbose", action="count", default=0)
    p.add_argument("--host", metavar="IP[:PORT]",
                   help=f"the C++ mind's host channel (port {P.HOST_PORT}; kilo.cli_host; a .local "
                        "name such as kilo.local is found by mDNS, and so is kilo.local when --host is left "
                        "out); needs this laptop's key (kilo key init)")
    p.add_argument("--key", metavar="PATH", default=None,
                   help="this laptop's private key for --host (default $KILO_HOST_KEY, else "
                        "~/.config/kilo/host_key; kilo key init makes it)")
    sub = p.add_subparsers(dest="cmd", required=True, metavar="COMMAND")

    s = sub.add_parser("status", help="the mind, the body's HELLO and state, link counters")
    s.add_argument("--json", action="store_true")

    s = sub.add_parser("monitor", help="live telemetry table")
    s.add_argument("--rate", type=float, default=10.0, help="refresh rate in Hz (table and JSON lines)")
    s.add_argument("--duration", type=float, default=None, help="stop after S seconds")
    s.add_argument("--json", action="store_true",
                   help="JSON lines instead of the table (the default when stdout is not a terminal)")

    s = sub.add_parser("posture", help="hold a posture (until Ctrl-C or --for); the mind latches REST after")
    s.add_argument("name", choices=[n.lower() for n in P.Posture.__members__])
    s.add_argument("--transition", type=int, default=0, metavar="TICKS",
                   help="transition length in 20 ms ticks (0 = body default, 25)")
    s.add_argument("--for", dest="hold", type=float, default=None, metavar="S",
                   help="hold for S seconds, then exit")

    __import__("kilo.cli_cal").cli_cal.add_parser(sub)  # kilo cal (cli_cal.py)

    s = sub.add_parser("guard", help="the distance guard: on, off, or how it stands (docs/distance-guard.md)")
    s.add_argument("mode", nargs="?", choices=["on", "off", "status"], default="status")
    s.add_argument("--json", action="store_true")

    __import__("kilo.cli_sound").cli_sound.add_parser(sub)  # kilo sound (cli_sound.py)

    s = sub.add_parser("reboot", help="restart the mind: the body gets REST, then the Pi reboots "
                                      "(on the netboot card: the loader waits for the next kernel)")

    s = sub.add_parser(
        "update", help="write a new kernel onto the mind's SD card; the Pi boots it from the card "
                       "from then on (the mind reboots into it now), then wait for the mind to come back",
        usage="kilo update [--host IP[:PORT]] [--image PATH] [--force] [--wait S]",
        description="Send a mind kernel image to the mind. The mind checks its size, CRC-32 and the "
                    f"mind kernel's marker ({P.HOST_UPDATE_MARKER}), writes it to its SD card as kernel7.new, "
                    "reads it back, renames kernel7.img to kernel7.old and kernel7.new to kernel7.img, and reboots "
                    "into it. A failure before the renames leaves the card as it was. Before sending, this CLI "
                    "checks that the image has this laptop's public key built in (kilo key show): a kernel without "
                    "it would refuse this laptop afterwards. To put the netboot loader back on the card (it has no "
                    "marker and no keys): kilo update --host IP --image kernel/build/netboot/kernel7.img --force.")
    s.add_argument("--image", type=Path, default=None, metavar="PATH",
                   help="the kernel image (default: kernel/build/kernel7.img of this checkout, `make -C kernel`)")
    s.add_argument("--force", action="store_true",
                   help="install an image without the marker or without this laptop's key: how the netboot "
                        "loader goes back on the card (--image kernel/build/netboot/kernel7.img --force)")
    s.add_argument("--wait", type=float, default=30.0, metavar="S",
                   help="seconds to wait for the mind to come back after its reboot and show its build "
                        "(default 30; 0: do not wait)")

    __import__("kilo.cli_record").cli_record.add_parsers(sub)  # kilo runs (walks), record, bench, metrics
    __import__("kilo.cli_power").cli_power.add_parsers(sub)  # kilo power: light sleep, the battery trend
    __import__("kilo.cli_measure").cli_measure.add_parser(sub)  # kilo measure: the self-measurement (B1)

    s = sub.add_parser("flash", help="build the body firmware and flash it through the mind, or put it on the "
                                     "mind's boot card (--sd)")
    s.add_argument("--sketch", type=Path, help="sketch directory (default: <repo>/body)")
    s.add_argument("--build-dir", type=Path, help="arduino-cli output dir (default ~/.cache/kilo-build)")
    s.add_argument("--no-size-check", action="store_true")
    s.add_argument("--force", action="store_true",
                   help="with --sd: write to a MOUNT that does not look like Kilo's card")
    s.add_argument("--sd", type=Path, metavar="MOUNT",
                   help="copy body.bin + body.json to the mind's boot card (its FAT partition, mounted here): "
                        "the mind flashes the body at boot if HELLO.fw_build differs; the card is checked "
                        "and the files read back as kilo card does")
    s.add_argument("--image", type=Path, metavar="BIN",
                   help="flash this body.bin instead of building (manifest: --manifest, default body.json next "
                        "to it)")
    s.add_argument("--manifest", type=Path, metavar="JSON", help="the manifest of --image")
    s.add_argument("--persist", action="store_true",
                   help="the mind also keeps the image on its SD card after a good flash")
    s.add_argument("--flash-timeout", type=float, default=None, metavar="S",
                   help="seconds to wait for the flash to end (default 60)")
    s.add_argument("--force-dir", action="store_true", help=argparse.SUPPRESS)  # --sd: tests, a plain dir
    s.add_argument("--rollback", action="store_true",
                   help="flash the last good image kept on the mind's SD (not yet: docs/fast-boot.md §6)")

    s = sub.add_parser(
        "key", help="this laptop's key for --host: init (make it, once) or show (the line for "
                    "kernel/keys/authorized_keys)",
        description="The mind talks over the network (kilo --host) only to laptops whose public key is built into "
                    "its kernel (kernel/keys/authorized_keys). `kilo key init` makes this laptop's key pair in "
                    "~/.config/kilo/host_key (or --key PATH, $KILO_HOST_KEY) if there is none, never overwriting "
                    "one, and prints the public line; `kilo key show` prints that line again (stdout) and the "
                    "key's fingerprint (stderr). Add the line with `kilo key show >> kernel/keys/authorized_keys`, "
                    "rebuild (make -C kernel) and install the kernel (kilo update --host from a laptop the mind "
                    "knows, or the SD card).")
    s.add_argument("action", choices=("init", "show"))
    s.add_argument("--name", default=None, help="init: the key's name in the public line (default user@hostname)")

    s = sub.add_parser("log", help="log tools: summary|replay|plot|walk|trick|run FILE ... (kilo.logs); "
                                   "with --host: the mind's log", add_help=False)
    s.add_argument("log_argv", nargs=argparse.REMAINDER, metavar="summary|replay|plot FILE ...")
    __import__("kilo.cli_find").cli_find.add_parser(sub)  # kilo find (cli_find.py)
    __import__("kilo.cli_card").cli_card.add_parser(sub)  # kilo card PATH (cli_card.py)
    seen: set[int] = set()
    for name, sp in sub.choices.items():  # `kilo status --host IP` too (`kilo log --host`: kilo.cli_host)
        if name not in ("log", "find") and id(sp) not in seen:  # an alias (walks: runs) is the same parser
            seen.add(id(sp))
            sp.add_argument("--host", default=argparse.SUPPRESS, metavar="IP[:PORT]", help=argparse.SUPPRESS)
            sp.add_argument("--key", default=argparse.SUPPRESS, metavar="PATH", help=argparse.SUPPRESS)
    return p


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = build_parser()
    argv = list(sys.argv[1:] if argv is None else argv)
    if "log" in argv and "--host" in argv:
        # `kilo log --host IP ...`, `kilo --host IP log -n 5`: argparse's REMAINDER
        # will not start with an option, so the tail goes to kilo.cli_host as it is.
        i = argv.index("log")
        args = parser.parse_args(argv[:i + 1])
        args.log_argv = list(argv[i + 1:])
        return args
    args = parser.parse_args(argv)
    if args.cmd == "cal" and args.cal_cmd is None:
        args.cal_cmd = "show"
    return args


# ---- the commands run here ---------------------------------------------------------
def cmd_log(args: argparse.Namespace) -> int:
    """`kilo log summary|replay|plot|walk|trick|run FILE ...` (kilo.logs)."""
    from kilo.logs.__main__ import build_parser as log_parser, main as log_main

    if not args.log_argv or args.log_argv[0] in ("-h", "--help"):
        log_parser().print_help()
        return 0 if args.log_argv else EXIT_USAGE
    return log_main(list(args.log_argv))


def cmd_key(args: argparse.Namespace) -> int:
    """`kilo key init|show`: this laptop's key for the host channel (kilo.hostkey)."""
    from kilo import hostkey

    try:
        if args.action == "init":
            key, made = hostkey.init(args.key, args.name)
        else:
            key, made = hostkey.load(args.key), False
    except hostkey.HostKeyError as e:
        print(f"kilo key {args.action}: {e}", file=sys.stderr)
        return EXIT_ERR
    except OSError as e:
        print(f"kilo key {args.action}: {e}", file=sys.stderr)
        return EXIT_ERR
    if args.action == "init":
        state = "made" if made else "already there, not changed"
        print(f"kilo key: {key.path} {state}; fingerprint {key.fingerprint}", file=sys.stderr)
        print(f"kilo key: add this line to {hostkey.KEYS_FILE} (kilo key show >> {hostkey.KEYS_FILE}), then "
              "rebuild and install the kernel:", file=sys.stderr)
    else:
        print(f"kilo key: {key.path}, fingerprint {key.fingerprint}", file=sys.stderr)
    print(key.line())
    return EXIT_OK


def cmd_flash_sd(args: argparse.Namespace) -> int:
    """`kilo flash --sd MOUNT`: body.bin + body.json onto the mind's boot card, checked and written as
    `kilo card MOUNT --no-kernel` does (kilo.cli_card)."""
    from kilo.cli_card import CardError, Plan, check_card, write_card
    from kilo.tools.flash import FlashError, image_options, prepare_image

    try:
        for w in check_card(args.sd, args.force, args.force_dir):
            print(f"kilo flash: warning: {w} (--force)", file=sys.stderr)
        data, m = prepare_image(image_options(args))
        write_card(args.sd, Plan(body=(data, m)))
    except (CardError, FlashError) as e:
        print(f"kilo flash: {e}", file=sys.stderr)
        return EXIT_ERR
    print(f"At its next boot the mind flashes the body unless HELLO.fw_build is already {m.build:08x}. "
          f"Unmount (eject) {args.sd} before taking the card out.")
    return EXIT_OK


LOCAL_COMMANDS = {
    "log": cmd_log, "key": cmd_key, "flash": cmd_flash_sd,
    "record": lambda args: __import__("kilo.cli_record").cli_record.offline(args),
    "find": lambda args: __import__("kilo.cli_find").cli_find.cmd_find(args),
    "power": lambda args: __import__("kilo.cli_power").cli_power.offline(args),
    "measure": lambda args: __import__("kilo.cli_measure").cli_measure.local(args),
    "card": lambda args: __import__("kilo.cli_card").cli_card.cmd_card(args)}


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    level = logging.WARNING if args.verbose == 0 else logging.INFO if args.verbose == 1 else logging.DEBUG
    logging.basicConfig(level=level, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    from kilo.cli_host import run_host, wants_host

    if wants_host(args):
        return run_host(args)
    return LOCAL_COMMANDS[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
