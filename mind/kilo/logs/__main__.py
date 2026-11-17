"""python -m kilo.logs: offline log tools (until ``kilo log ...`` is wired into cli.py).

    python -m kilo.logs summary FILE [--json]
    python -m kilo.logs replay FILE [--json] [--with-body-events]
    python -m kilo.logs plot FILE --fields pitch,roll[,dist,batt,rate_pitch,mode,balance...]
                              [--out PATH.png|PATH.csv] [--csv]
    python -m kilo.logs walk FILE.kwalk|FILE.krec [--run ID] [--csv OUT] [--plot OUT.png] [--json]
    python -m kilo.logs trick FILE.krec [--run ID] [--csv OUT] [--plot OUT.png] [--json] [--threshold DEG]
    python -m kilo.logs run FILE.krec [--run ID] ...   walk or trick, whichever the run is

FILE is a link JSONL log (``kilo --log``), a ``kilo monitor`` JSONL capture,
a ``.kilolog`` (optionally ``.gz``), or a walk recording (``.kwalk``, from
``kilo walks --host IP --fetch``; kilo.logs.walkrec), or a flight recording
(``.krec``, from ``kilo runs --fetch`` or ``kilo record fetch``;
kilo.logs.flight). ``walk`` analyses one walk: predicted against measured
attitude, tick by tick; ``trick`` one trick run against its design's
predicted pitch (kilo.logs.trickrec); ``run`` either.
"""

from __future__ import annotations

import argparse
import json
import sys

from kilo.logs import analysis, reader


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="python -m kilo.logs", description="Kilo log tools")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("summary", help="duration, rates, attitude, battery, modes, events")
    s.add_argument("file")
    s.add_argument("--json", action="store_true")
    r = sub.add_parser("replay", help="feed the log through Percepts and print its events")
    r.add_argument("file")
    r.add_argument("--json", action="store_true")
    r.add_argument("--with-body-events", action="store_true", help="interleave the body's EVENTs")
    pl = sub.add_parser("plot", help="plot fields over time (CSV without matplotlib)")
    pl.add_argument("file")
    pl.add_argument("--fields", default="pitch,roll", help="comma-separated (default pitch,roll)")
    pl.add_argument("--out", help="output .png or .csv (default next to the log)")
    pl.add_argument("--csv", action="store_true", help="write CSV even if matplotlib is available")
    wk = sub.add_parser("walk", help="a walk recording (.kwalk): predicted vs measured, per tick and summed up")
    wk.add_argument("file")
    wk.add_argument("--csv", metavar="OUT", help="write the per-tick table as CSV")
    wk.add_argument("--plot", metavar="OUT", help="plot predicted vs measured, error, correction, held (PNG)")
    wk.add_argument("--json", action="store_true", help="the summary as JSON")
    wk.add_argument("--run", type=int, default=None, help="in a .krec with several runs: this one")
    for name, hlp in (("trick", "a trick run (.krec): measured vs predicted pitch per keyframe, tip-over, the end"),
                      ("run", "a run (.krec): the walk or trick analysis, whichever it is")):
        t = sub.add_parser(name, help=hlp)
        t.add_argument("file")
        t.add_argument("--run", type=int, default=None, help="this run (default: the first of its kind)")
        t.add_argument("--csv", metavar="OUT", help="write the per-tick table as CSV")
        t.add_argument("--plot", metavar="OUT", help="plot it (PNG; CSV without matplotlib)")
        t.add_argument("--json", action="store_true", help="the analysis as JSON")
        t.add_argument("--threshold", type=float, default=8.0, metavar="DEG",
                       help="tricks: the error that counts as a tip-over (default 8 deg for 0.2 s)")
    return p


def run_main(args: argparse.Namespace) -> int:
    """`trick` and `run` (and `walk` on a .krec)."""
    from kilo.logs import flight, trickrec

    try:
        s = flight.load(args.file)
    except (OSError, ValueError) as e:
        print(f"cannot read {args.file}: {e}", file=sys.stderr)
        return 1
    want = {"trick": "trick", "walk": "walk"}.get(args.cmd)
    runs = [r for r in s.runs if (args.run is None or r.id == args.run) and (want is None or r.kind == want)]
    if not runs:
        print(f"{args.file}: no {want or 'run'}{'' if args.run is None else f' {args.run}'} in it (runs: "
              f"{', '.join(r.label for r in s.runs) or 'none'})", file=sys.stderr)
        return 1
    run = runs[0]
    if run.kind == "walk":
        from kilo.logs import walkrec

        w = flight.walk_of(s, run)
        return _walk_out(args, w)
    tr = trickrec.trick_run(s, run)
    a = trickrec.analyze(tr, threshold_deg=getattr(args, "threshold", 8.0))
    print(trickrec.to_json(a) if args.json else trickrec.format_analysis(a))
    rows = trickrec.rows(tr)
    if args.csv:
        print(f"wrote {trickrec.write_csv(rows, args.csv)} ({len(rows)} ticks)", file=sys.stderr)
    if args.plot:
        print(trickrec.plot(rows, args.plot, title=str(args.file))[1], file=sys.stderr)
    return 0


def _walk_out(args: argparse.Namespace, w) -> int:
    from kilo.logs import walkrec

    rows = walkrec.tick_table(w)
    s = walkrec.summarize(w, rows)
    print(json.dumps(s, indent=1) if args.json else walkrec.format_summary(s))
    if args.csv:
        print(f"wrote {walkrec.write_csv(rows, args.csv)} ({len(rows)} ticks)", file=sys.stderr)
    if args.plot:
        _, note = walkrec.plot(rows, args.plot, title=str(args.file))
        print(note, file=sys.stderr)
    return 0


def walk_main(args: argparse.Namespace) -> int:
    from kilo.logs import flight, walkrec

    if flight.is_krec(args.file) or getattr(args, "run", None) is not None:
        return run_main(args)
    try:
        w = walkrec.load(args.file)
    except (OSError, ValueError) as e:
        print(f"cannot read {args.file}: {e}", file=sys.stderr)
        return 1
    rows = walkrec.tick_table(w)
    s = walkrec.summarize(w, rows)
    print(json.dumps(s, indent=1) if args.json else walkrec.format_summary(s))
    if args.csv:
        print(f"wrote {walkrec.write_csv(rows, args.csv)} ({len(rows)} ticks)", file=sys.stderr)
    if args.plot:
        _, note = walkrec.plot(rows, args.plot, title=str(args.file))
        print(note, file=sys.stderr)
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.cmd == "walk":
        return walk_main(args)
    if args.cmd in ("trick", "run"):
        return run_main(args)
    try:
        data = reader.load(args.file)
    except (OSError, ValueError) as e:
        print(f"cannot read {args.file}: {e}", file=sys.stderr)
        return 1
    if args.cmd == "summary":
        s = analysis.summary(data)
        print(json.dumps(s, indent=1) if args.json else analysis.format_summary(s))
        return 0
    if args.cmd == "replay":
        evs = analysis.replay(data)
        rows = [(e.t, "percept", e.name, e.data) for e in evs]
        if args.with_body_events:
            rows += [(e.t, "body", e.name, {"arg": e.arg}) for e in data.events]
            rows.sort(key=lambda r: r[0])
        if args.json:
            for t, src, name, d in rows:
                print(json.dumps({"t": round(t, 3), "src": src, "name": name, **d}))
        else:
            print(f"{data.path}: {len(data.samples)} samples, {len(evs)} percept events")
            for t, src, name, d in rows:
                extra = " ".join(f"{k}={v}" for k, v in d.items() if v is not None)
                print(f"{t:9.3f}  {src:7s} {name:12s} {extra}")
        return 0
    try:
        path, note = analysis.plot(data, args.fields.split(","), args.out, force_csv=args.csv)
    except ValueError as e:
        print(f"plot: {e}", file=sys.stderr)
        return 2
    print(note)
    return 0


if __name__ == "__main__":
    sys.exit(main())
