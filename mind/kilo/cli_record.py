"""The flight recorder over --host (kilo.logs.flight; kernel/README.md "The
flight recorder"): runs, time ranges, the card saver, the SD bench, metrics.

    kilo runs --host IP [--json]                    this boot's and earlier boots' walks and tricks
    kilo runs --host IP --fetch [N|all|last] [--out DIR]   download them (DIR/run-....krec) and analyse each
    kilo runs --host IP --delete BOOT|all           an earlier boot's recordings off the SD card
    kilo walks ...                                  the same (its old name)
    kilo record status --host IP                    the RAM ring, what a power cut would lose, the card
    kilo record fetch --host IP [--last 10m | --since 20m [--until 5m] | --from S --to S | --all] [--boot N]
                      [--out DIR]                   a time range of everything recorded (DIR/rec-....krec)
    kilo record save on|off|now --host IP           the card saver (takes control: the mind latches REST after)
    kilo record decode FILE [--csv OUT] [--json] [--log] [--plot PNG] [--from S --to S]   (no Kilo needed)
    kilo bench sd --host IP [--count N]             the SD card's write times (Kilo limp; takes control)
    kilo metrics --host IP                          the session's counters since boot

Downloads go to ~/kilo-logs unless --out says otherwise.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path
from typing import Any

from kilo import EXIT_ERR, EXIT_OK
from kilo.hostlink import HostError, HostLink, HostRefused, enum_name
from kilo.log import DEFAULT_DIR_TEXT
from kilo.logs import flight, trickrec, walkrec
from kilo.protocol import kilo_protocol as P

DEFAULT_OUT = DEFAULT_DIR_TEXT
EARLIER_SHIFT = 20
ALL_BOOTS = 0xFFFFFFFF


# ---- parsers ----------------------------------------------------------------------------

def add_parsers(sub: Any) -> None:
    s = sub.add_parser(
        "runs", aliases=["walks"],
        help="the walks and tricks a C++ mind recorded, over --host: list, download (--fetch), delete",
        usage="kilo runs --host IP[:PORT] [--json] | --fetch [N|all|last] [--out DIR] [--quiet] | --delete BOOT|all",
        description="The mind records everything from boot (the flight recorder: every link frame, every log "
                    "line); a run is a walk or a trick in it. Without --fetch: the list, this boot's from RAM "
                    "and earlier boots' from the SD card. --fetch downloads a run (2 s before to 2 s after) as "
                    "DIR/run-<time>-<kind>-<id>.krec (and a .kilolog for a walk) and prints its analysis "
                    "(kilo log run FILE prints it again). --delete removes an earlier boot's recordings from "
                    "the card.")
    s.add_argument("--fetch", nargs="?", const="all", default=None, metavar="N|all|last",
                   help="download run N, the last run, or every run that has ended (the default with no value)")
    s.add_argument("--out", default=DEFAULT_OUT, metavar="DIR", help=f"where --fetch writes (default {DEFAULT_OUT})")
    s.add_argument("--json", action="store_true", help="the list as JSON")
    s.add_argument("--quiet", "-q", action="store_true", help="--fetch: no analysis per run")
    s.add_argument("--delete", metavar="BOOT|all", default=None,
                   help="delete an earlier boot's recordings from the SD card (takes control)")

    s = sub.add_parser("record", help="the flight recorder over --host: status, fetch a time range, the card "
                                      "saver; decode a recording (no Kilo needed)")
    rs = s.add_subparsers(dest="rec_cmd", required=True, metavar="ACTION")
    rs.add_parser("status", help="the RAM ring, what a power cut would lose, the SD card saver")
    f = rs.add_parser("fetch", help="download a time range of everything recorded")
    g = f.add_mutually_exclusive_group()
    g.add_argument("--last", metavar="DUR", help="the last DUR (10m, 90s, 2h)")
    g.add_argument("--since", metavar="DUR", help="from DUR ago (with --until DUR ago, default now)")
    g.add_argument("--all", action="store_true", help="everything there is (the default)")
    g.add_argument("--from", dest="t_from", type=float, metavar="S", help="from the mind's clock S seconds")
    f.add_argument("--until", metavar="DUR", help="with --since: up to DUR ago")
    f.add_argument("--to", dest="t_to", type=float, metavar="S", help="with --from: up to the clock's S seconds")
    f.add_argument("--boot", type=int, default=0, help="an earlier boot's recordings on the SD card (its number)")
    f.add_argument("--out", default=DEFAULT_OUT, metavar="DIR", help=f"where it writes (default {DEFAULT_OUT})")
    sv = rs.add_parser("save", help="the SD card saver: on (remembered on the card), off, or save now")
    sv.add_argument("mode", choices=["on", "off", "now"])
    d = rs.add_parser("decode", help="a .krec as rows: --csv, --json, --log, --plot (kilo.logs.decode)")
    d.add_argument("file")
    d.add_argument("--csv", metavar="OUT", help="one row per TELEMETRY")
    d.add_argument("--json", action="store_true", help="every row (telemetry, log, key, event, command, ...) as JSON")
    d.add_argument("--log", action="store_true", help="the mind's log lines only")
    d.add_argument("--plot", metavar="OUT", help="pitch, roll, battery, runs (PNG; CSV without matplotlib)")
    d.add_argument("--from", dest="t_from", type=float, metavar="S", help="from S seconds of the file")
    d.add_argument("--to", dest="t_to", type=float, metavar="S", help="to S seconds of the file")

    s = sub.add_parser("bench", help="measure over --host: `bench sd` (the SD card's write times)")
    bs = s.add_subparsers(dest="bench_cmd", required=True, metavar="WHAT")
    b = bs.add_parser("sd", help="the SD card's write times: Kilo limp (REST), takes control")
    b.add_argument("--count", type=int, default=32, help="writes of each size (default 32)")

    sub.add_parser("metrics", help="the session's counters since boot, over --host")


def offline(args: argparse.Namespace) -> int:
    """These commands without --host."""
    if args.cmd == "record" and args.rec_cmd == "decode":
        from kilo.logs import decode

        return decode.main(args)
    what = {"runs": "the runs a C++ mind recorded", "walks": "the runs a C++ mind recorded",
            "record": "the mind's flight recorder", "bench": "the mind's SD card",
            "metrics": "the mind's session metrics"}.get(args.cmd, args.cmd)
    print(f"kilo {args.cmd}: needs --host IP[:PORT] ({what}: the Circle kernel or kilo-mind-host); "
          "a downloaded recording: kilo log run FILE, kilo record decode FILE", file=sys.stderr)
    return EXIT_ERR


# ---- helpers ----------------------------------------------------------------------------

def parse_duration(text: str) -> float:
    """'10m' / '90s' / '2h' / '1.5' (seconds) -> seconds."""
    m = re.fullmatch(r"\s*([0-9]*\.?[0-9]+)\s*([smhd]?)\s*", text or "")
    if not m:
        raise ValueError(f"bad duration {text!r}: e.g. 90s, 10m, 2h")
    return float(m.group(1)) * {"": 1, "s": 1, "m": 60, "h": 3600, "d": 86400}[m.group(2)]


def parse_run_id(text: str) -> int:
    """'7' (this boot's run 7, or a full WALK_INFO id) or 'b6:3' (boot 6's run 3)."""
    if ":" in text:
        b, n = text.split(":", 1)
        return (int(b.lstrip("b")) << EARLIER_SHIFT) | int(n)
    return int(text)


def _rec_status(hl: HostLink) -> Any | None:
    try:
        return hl.rec_status()
    except (HostRefused, HostError):
        return None


def _meta(hl: HostLink, st: Any, rs: Any | None, now: float) -> dict[str, Any]:
    m = {"mind": hl.address, "mind_build": f"{st.mind_build:08x}", "fetched_wall": round(now, 3),
         "uptime_ms": st.uptime_ms, "format": P.HOST_WALK_FORMAT}
    if rs is not None:
        m["clock_ms"] = rs.clock_ms
        m["boot"] = rs.boot
        # the clock at the mind's start: uptimes in the analysis count from it
        # the clock at the mind's start, in ms mod 2^32 as REC_STATUS gives it
        # (the host program's clock can be past 49.7 days of ms): uptimes in the
        # analysis count from it
        m["mind_t0_ms"] = (rs.clock_ms - st.uptime_ms) & 0xFFFFFFFF
    return m


def boot_of(info: Any) -> int:
    return info.id >> EARLIER_SHIFT if info.flags & P.WalkFlags.EARLIER_BOOT else 0


def local_id(info: Any) -> int:
    return info.id & ((1 << EARLIER_SHIFT) - 1)


def describe(info: Any) -> dict[str, Any]:
    d = walkrec.info_dict(info)
    d["kind"] = "trick" if info.flags & P.WalkFlags.TRICK else "walk"
    d["boot"] = boot_of(info) or None
    d["run"] = local_id(info)
    d["saved"] = bool(info.flags & P.WalkFlags.SAVED)
    return d


def _when(info: Any, uptime_ms: int, now: float) -> float | None:
    if info.flags & P.WalkFlags.EARLIER_BOOT:
        return None
    return now - max(0, uptime_ms - info.start_ms) / 1000.0


def _row(info: Any, when: float | None) -> str:
    d = describe(info)
    flags = d["flags"]
    state = "recording" if "RECORDING" in flags else d.get("end") or "-"
    if "TRUNCATED" in flags:
        state += " (TRUNCATED: its start is gone)"
    if d["kind"] == "trick":
        if d.get("of"):  # the stage allowed in the run: "rear_up stage 1 of 3"
            what = trickrec.trick_label(d.get("trick", "?"), d.get("stage"), d.get("of"), d.get("reached"))
        else:  # an older mind: the build's stage
            what = d.get("trick", "?") + (f" stage {d['stage']}" if d.get("stage") else "")
    else:
        what = f"{d.get('gait', '?')} {d.get('dir', '?')} {d.get('speed', '?')}"
    started = (time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(when)) if when is not None
               else f"boot {d['boot']} at {info.start_ms / 1000:8.1f}s")
    boot = "" if d["boot"] is None else f"b{d['boot']}:"
    return (f"{boot}{d['run']:<4d} {started:21s} {d['kind']:5s} {what:28s} {info.duration_ms / 1000:7.1f} s  "
            f"{info.bytes / 1024:8.1f} KB  {'card' if d['saved'] else 'RAM ':4s}  {state}")


# ---- kilo runs --------------------------------------------------------------------------

def cmd_runs(args: argparse.Namespace, hl: HostLink) -> int:
    st = hl.status()
    now = time.time()
    if args.delete is not None:
        return _delete(args, hl)
    try:
        runs = hl.walks()
    except HostRefused as e:
        if e.code in (P.HostResultCode.UNSUPPORTED, P.HostResultCode.UNKNOWN_TYPE):
            print(f"kilo {args.cmd}: this mind (build {st.mind_build:08x}) does not record walks ({e.code_name})",
                  file=sys.stderr)
            return EXIT_ERR
        raise
    store = hl.walk_store
    rs = _rec_status(hl)
    if args.fetch is None:
        if args.json:
            print(json.dumps([dict(describe(w), start_wall=None if _when(w, st.uptime_ms, now) is None
                                   else round(_when(w, st.uptime_ms, now), 3)) for w in runs], indent=1))
            return EXIT_OK
        this = [w for w in runs if not w.flags & P.WalkFlags.EARLIER_BOOT]
        earlier = len(runs) - len(this)
        walks = sum(1 for w in this if not w.flags & P.WalkFlags.TRICK)
        line = (f"mind build {st.mind_build:08x}, up {st.uptime_ms / 1000:.0f} s: {len(this)} runs this boot "
                f"({walks} walks, {len(this) - walks} tricks)" + (f", {earlier} from earlier boots" if earlier else ""))
        if rs is not None:
            line += f"; RAM {rs.used / 1048576:.1f} of {rs.capacity / 1048576:.0f} MB"
        else:
            line += f"; {store.used / 1048576:.2f} of {store.capacity / 1048576:.2f} MB used"
        if store is not None and store.dropped_walks:
            line += f", {store.dropped_walks} older dropped for room"
        if store is not None and store.recording:
            line += f", run {store.recording} being recorded"
        print(line)
        if runs:
            print("run  started (this laptop)   kind  what                         duration        size  "
                  "kept  end")
            for w in runs:
                print(_row(w, _when(w, st.uptime_ms, now)))
        return EXIT_OK
    ended = [w for w in runs if not w.flags & P.WalkFlags.RECORDING]
    if args.fetch == "all":
        chosen = ended
        for w in runs:
            if w.flags & P.WalkFlags.RECORDING:
                print(f"run {w.id}: still being recorded; fetch it once it has ended", file=sys.stderr)
    elif args.fetch == "last":
        this = [w for w in ended if not w.flags & P.WalkFlags.EARLIER_BOOT] or ended
        chosen = this[-1:]
        if not chosen:
            print("kilo runs: no run has ended yet", file=sys.stderr)
            return EXIT_ERR
    else:
        try:
            want = parse_run_id(args.fetch)
        except ValueError:
            print(f"kilo runs: --fetch {args.fetch!r}: a run id, last or all", file=sys.stderr)
            return EXIT_ERR
        chosen = [w for w in runs if w.id == want]
        if not chosen:
            print(f"kilo runs: no run {args.fetch} on the mind (kept: "
                  f"{', '.join(str(w.id) for w in runs) or 'none'})", file=sys.stderr)
            return EXIT_ERR
    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    status = EXIT_OK
    for w in chosen:
        if w.flags & P.WalkFlags.RECORDING:
            print(f"run {w.id}: still being recorded; fetch it once it has ended", file=sys.stderr)
            status = EXIT_ERR
            continue
        if fetch_one(hl, w, st, rs, now, out, quiet=args.quiet) != EXIT_OK:
            status = EXIT_ERR
    return status


def fetch_one(hl: HostLink, w: Any, st: Any, rs: Any | None, now: float, out: Path, quiet: bool = False) -> int:
    d = describe(w)
    when = _when(w, st.uptime_ms, now)
    t = time.monotonic()
    try:
        data = hl.fetch_walk(w.id)
    except (HostError, HostRefused) as e:
        print(f"run {w.id}: {e}", file=sys.stderr)
        return EXIT_ERR
    took = time.monotonic() - t
    meta = dict(_meta(hl, st, rs, now), walk=walkrec.info_dict(w), kind=d["kind"])
    if when is not None:
        meta["start_wall"] = round(when, 3)
    if d["boot"]:
        meta["boot"] = d["boot"]
        meta.pop("mind_t0_ms", None)  # another boot's clock
    if not flight.is_blocks(data):  # a mind of host.walk_format 1: a walk's records, as before
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(when if when is not None else now))
        stem = out / f"walk-{stamp}-{w.id}"
        meta["format"] = 1
        kw = walkrec.write_file(stem.with_suffix(".kwalk"), meta, data)
        rec = walkrec.parse_records(data, meta)
        rec.meta["path"] = str(kw)
        kl = walkrec.to_kilolog(rec, stem.with_suffix(".kilolog"))
        print(f"walk {w.id}: {len(data)} bytes in {took:.1f} s -> {kw} (+ {kl.name})", flush=True)
        if not quiet:
            print(walkrec.format_summary(walkrec.summarize(rec)), flush=True)
        return EXIT_OK
    stamp = (time.strftime("%Y%m%d-%H%M%S", time.localtime(when)) if when is not None
             else f"b{d['boot']}-{int(w.start_ms / 1000)}s")
    stem = out / f"run-{stamp}-{d['kind']}-{d['run']}"
    path = flight.write_file(stem.with_suffix(".krec"), meta, data)
    s = flight.parse_blocks(data, meta)
    s.meta["path"] = str(path)
    extra = ""
    run = s.run(d["run"])
    if d["kind"] == "walk" and run is not None:
        wk = flight.walk_of(s, run)
        kl = walkrec.to_kilolog(wk, stem.with_suffix(".kilolog"))
        extra = f" (+ {kl.name})"
    print(f"{d['kind']} {w.id}: {len(data)} bytes in {took:.1f} s -> {path}{extra}", flush=True)
    if s.bad_blocks:
        print(f"  {len(s.bad_blocks)} damaged blocks skipped", flush=True)
    if not quiet:
        print(analysis_text(s, run, d["kind"]), flush=True)
    return EXIT_OK


def analysis_text(s: flight.Stream, run: flight.RunSpan | None, kind: str) -> str:
    if run is None:
        return "  (the run's start and end marks are not in the data)"
    if kind == "trick":
        return trickrec.format_analysis(trickrec.analyze(trickrec.trick_run(s, run)))
    w = flight.walk_of(s, run)
    return walkrec.format_summary(walkrec.summarize(w))


def _delete(args: argparse.Namespace, hl: HostLink) -> int:
    target = ALL_BOOTS if args.delete == "all" else None
    if target is None:
        try:
            target = int(args.delete.lstrip("b"))
        except ValueError:
            print(f"kilo runs --delete {args.delete!r}: a boot number or all", file=sys.stderr)
            return EXIT_ERR
    try:
        hl.take_control(keepalive=False)
        text = hl.rec_delete(target)
    except HostRefused as e:
        print(f"kilo runs --delete: the mind refused: {e}", file=sys.stderr)
        return EXIT_ERR
    finally:
        _release(hl)
    print(f"deleting {'every earlier boot' if target == ALL_BOOTS else f'boot {target}'}'s recordings from the "
          f"SD card" + (f": {text}" if text else "") + " (at the next moment Kilo is still)")
    return EXIT_OK


def _release(hl: HostLink) -> None:
    try:
        hl.release_control()
    except HostError:
        pass


# ---- kilo record ------------------------------------------------------------------------

CARD_STATE_HELP = {"NONE": "no SD card", "OFF": "saving is off (kilo record save on)", "IDLE": "on, up to date or "
                   "waiting for Kilo to be still", "SAVING": "writing", "BACKOFF": "a write was slow: waiting",
                   "FAILED": "the card refused: saving stopped (kilo record save on to retry)"}


def recorder_line(rs: Any) -> str:
    """One line: RAM used, what a power cut would lose, the card."""
    card = enum_name(P.RecordCardState, rs.card_state)
    s = (f"recorder    RAM {rs.used / 1048576:.1f} of {rs.capacity / 1048576:.0f} MB "
         f"({(rs.clock_ms - rs.oldest_ms) / 60000:.0f} min kept); a power cut now loses "
         f"{rs.unsaved_bytes / 1024:.0f} KB; card {card}")
    if rs.card_state not in (P.RecordCardState.NONE,):
        s += f" ({rs.card_bytes / 1048576:.1f} MB, {rs.card_boots} earlier boots, slowest write " \
             f"{rs.write_max_us / 1000:.0f} ms)"
    return s


def cmd_record(args: argparse.Namespace, hl: HostLink) -> int:
    if args.rec_cmd == "decode":
        from kilo.logs import decode

        return decode.main(args)
    if args.rec_cmd == "save":
        mode = {"off": 0, "on": 1, "now": 2}[args.mode]
        try:
            hl.take_control(keepalive=False)
            text = hl.rec_save(mode)
        except HostRefused as e:
            print(f"kilo record save: the mind refused: {e}", file=sys.stderr)
            return EXIT_ERR
        finally:
            _release(hl)
        print(f"card saver {args.mode}" + (f": {text}" if text else "")
              + " (taking control made the mind latch REST)")
        return EXIT_OK
    try:
        rs = hl.rec_status()
    except HostRefused as e:
        print(f"kilo record: this mind has no flight recorder ({e.code_name})", file=sys.stderr)
        return EXIT_ERR
    if args.rec_cmd == "status":
        print_rec_status(rs)
        return EXIT_OK
    return _record_fetch(args, hl, rs)


def print_rec_status(rs: Any) -> None:
    card = enum_name(P.RecordCardState, rs.card_state)
    print(f"flight recorder  boot {rs.boot or '? (card not read)'}, clock {rs.clock_ms / 1000:.1f} s, "
          f"{rs.runs} runs this boot")
    print(f"  RAM     {rs.used / 1048576:.2f} of {rs.capacity / 1048576:.1f} MB, {rs.blocks} blocks of "
          f"{rs.block_bytes // 1024} KB (#{rs.first_seq}..#{rs.next_seq}), oldest at {rs.oldest_ms / 1000:.1f} s, "
          f"{rs.dropped_blocks} overwritten")
    print(f"  records {rs.records}, heartbeats counted {rs.skipped}")
    print(f"  card    {card}: {CARD_STATE_HELP.get(card, '')}; saving {'on' if rs.save_on else 'off'}")
    print(f"          saved up to block #{rs.saved_seq}; a power cut now loses {rs.unsaved_bytes / 1024:.0f} KB")
    print(f"          {rs.card_bytes / 1048576:.2f} MB in {rs.card_files} files, {rs.card_boots} earlier boots")
    print(f"          {rs.writes} writes, slowest {rs.write_max_us / 1000:.1f} ms, {rs.slow_writes} over "
          f"{P.HOST_RECORD_SLOW_WRITE_MS} ms, {rs.card_errors} errors")


def _record_fetch(args: argparse.Namespace, hl: HostLink, rs: Any) -> int:
    now_ms = rs.clock_ms
    try:
        if args.last:
            lo, hi = now_ms - parse_duration(args.last) * 1000, 0xFFFFFFFF
        elif args.since:
            lo = now_ms - parse_duration(args.since) * 1000
            hi = 0xFFFFFFFF if not args.until else now_ms - parse_duration(args.until) * 1000
        elif args.t_from is not None:
            lo = args.t_from * 1000
            hi = 0xFFFFFFFF if args.t_to is None else args.t_to * 1000
        else:
            lo, hi = 0, 0xFFFFFFFF
    except ValueError as e:
        print(f"kilo record fetch: {e}", file=sys.stderr)
        return EXIT_ERR
    if args.boot and (args.last or args.since):
        print("kilo record fetch: --last/--since are for this boot; an earlier boot: --all or --from/--to",
              file=sys.stderr)
        return EXIT_ERR
    st = hl.status()
    now = time.time()
    t = time.monotonic()
    try:
        data = hl.rec_fetch(int(max(0, lo)), int(hi), boot=args.boot)
    except HostRefused as e:
        print(f"kilo record fetch: the mind refused: {e}", file=sys.stderr)
        return EXIT_ERR
    took = time.monotonic() - t
    meta = dict(_meta(hl, st, rs, now), range_ms=[int(max(0, lo)), int(hi)], what="blocks")
    if args.boot:
        meta["boot"] = args.boot
        meta.pop("mind_t0_ms", None)
    out = Path(args.out).expanduser()
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
    path = flight.write_file(out / (f"rec-{stamp}" + (f"-b{args.boot}" if args.boot else "") + ".krec"), meta, data)
    print(f"{len(data)} bytes in {took:.1f} s -> {path}", flush=True)
    from kilo.logs import decode

    s = flight.parse_blocks(data, meta)
    s.meta["path"] = str(path)
    print(decode.summary(s))
    return EXIT_OK


# ---- kilo bench sd ----------------------------------------------------------------------

def cmd_bench(args: argparse.Namespace, hl: HostLink) -> int:
    print("SD card write bench: Kilo must be limp (REST or no servo power); taking control", flush=True)
    print(" write size  count   min ms  mean ms   max ms  sync ms", flush=True)

    def show(r: Any) -> None:
        if r.write_bytes:
            print(f"{r.write_bytes:9d} B {r.count:6d} {r.min_us / 1000:8.2f} {r.mean_us / 1000:8.2f} "
                  f"{r.max_us / 1000:8.2f} {r.sync_us / 1000:8.2f}" + (f"  ERROR {r.error}" if r.error else ""),
                  flush=True)
    try:
        hl.take_control(keepalive=True)
        res = hl.bench(count=args.count, on_result=show)
    except HostRefused as e:
        print(f"kilo bench sd: the mind refused: {e}", file=sys.stderr)
        return EXIT_ERR
    finally:
        _release(hl)
    last = res[-1]
    print(f"close {last.close_us / 1000:.2f} ms, delete {last.remove_us / 1000:.2f} ms"
          + (f"; FAILED: card error {last.error}" if last.error else ""))
    worst = max([r.max_us for r in res] + [r.sync_us for r in res] + [last.close_us])
    verdict = ("under" if worst < P.HOST_RECORD_SLOW_WRITE_MS * 1000 else "OVER")
    print(f"slowest {worst / 1000:.1f} ms: {verdict} the saver's {P.HOST_RECORD_SLOW_WRITE_MS} ms limit (the body's "
          "watchdog is 500 ms)")
    return EXIT_ERR if last.error else EXIT_OK


# ---- kilo metrics -----------------------------------------------------------------------

def cmd_metrics(args: argparse.Namespace, hl: HostLink) -> int:
    try:
        lines = hl.metrics()
    except HostRefused as e:
        print(f"kilo metrics: this mind keeps no metrics ({e.code_name})", file=sys.stderr)
        return EXIT_ERR
    for ln in lines:
        print(ln)
    return EXIT_OK


HOST_COMMANDS = {"runs": cmd_runs, "walks": cmd_runs, "record": cmd_record, "bench": cmd_bench,
                 "metrics": cmd_metrics}
