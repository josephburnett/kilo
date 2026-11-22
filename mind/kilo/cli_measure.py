"""``kilo measure``: Kilo's self-measurement (docs/self-measurement.md; the
routine is the remote's C5 pressed twice, kernel/mind/measure/).

    kilo measure plan                         the experiments, their signals against the IMU, the time
    kilo measure fit FILE [FILE...] [--turned FILE...] [--walk FILE...] [--out PATH] [--json]
                                              fit the model to recorded runs; a table, and the numbers to PATH
    kilo measure fetch --host IP [--out DIR]  download the last self-measurement run and fit it
    kilo measure simulate [-n N] [--backlash DEG] [--write FILE] [--physics]
                                              the fit on runs made in simulation (the identifiability check)

The fit never changes the model the walks use: it writes its numbers to a
file (default next to the first recording, <name>.fit.json) for the owner
to decide on.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

from kilo import EXIT_ERR, EXIT_OK


def add_parser(sub: Any) -> None:
    s = sub.add_parser("measure", help="Kilo's self-measurement (C5 twice): plan, fit a recorded run, fetch the last one, "
                                       "check the fit in simulation",
                       description=__doc__.split("\n\n")[0])
    ms = s.add_subparsers(dest="measure_cmd", required=True, metavar="ACTION")
    ms.add_parser("plan", help="the experiments in order: what moves, how long, what each measures, its signal")
    f = ms.add_parser("fit", help="fit the model to recorded self-measurement runs (.krec) or simulated ones (.json)")
    f.add_argument("files", nargs="*", metavar="FILE")
    f.add_argument("--turned", nargs="+", default=[], metavar="FILE",
                   help="runs with Kilo turned round (180 deg) on the same floor: takes the floor's slope out of "
                        "the IMU offsets")
    f.add_argument("--walk", nargs="+", default=[], metavar="FILE",
                   help="walk recordings (.krec) whose error stream adds to the servo's dead time and time constant")
    f.add_argument("--out", metavar="PATH", help="where the fitted numbers go (default FILE.fit.json)")
    f.add_argument("--json", action="store_true", help="print the numbers as JSON instead of the table")
    g = ms.add_parser("fetch", help="download the last self-measurement run over --host and fit it")
    g.add_argument("--out", default=None, metavar="DIR", help="where it is written (default ~/kilo-logs)")
    g.add_argument("--no-fit", action="store_true", help="only download")
    m = ms.add_parser("simulate", help="the identifiability check: the fit on runs made from perturbed parameters")
    m.add_argument("-n", "--trials", type=int, default=20)
    m.add_argument("--seed", type=int, default=1)
    m.add_argument("--backlash", type=float, default=0.0, metavar="DEG",
                   help="each hold a random offset of this sd (servos resting a little differently each time)")
    m.add_argument("--write", metavar="FILE", help="instead: one simulated run into FILE (.json) for kilo measure fit")
    m.add_argument("--physics", action="store_true",
                   help="with --write: on the virtual body's walking physics (rigid, a real tip), its centre of "
                        "mass 4 mm back and 3 mm right, the IMU 1.5 / -2 deg off")


def _fit(files: list[str], turned: list[str], walks: list[str], out: str | None, as_json: bool) -> int:
    import json

    from kilo.measure import fit as F
    from kilo.measure import record as R

    runs = []
    try:
        for f in files:
            runs += R.load(f, heading=1)
        for f in turned:
            runs += R.load(f, heading=-1)
        ws = []
        for f in walks:
            got = R.load_walks(f)
            if not got:
                print(f"kilo measure fit: {f}: no walk with 100 ticks on three or four feet: left out", file=sys.stderr)
            ws += got
    except R.RecordError as e:
        print(f"kilo measure fit: {e}", file=sys.stderr)
        return EXIT_ERR
    if not runs and not ws:
        print("kilo measure fit: no self-measurement run and no walk to fit", file=sys.stderr)
        return EXIT_ERR
    res = F.Fitter(runs, ws).run()
    first = (list(files) + list(turned) + list(walks))[0]
    path = Path(out) if out else Path(first).with_suffix(".fit.json")
    inputs = list(files) + [f"turned:{f}" for f in turned] + [f"walk:{f}" for f in walks]
    F.write_json(res, inputs, path)
    if as_json:
        print(json.dumps(F.to_json(res, inputs), indent=1))
    else:
        print(F.format_table(res))
        print(f"the numbers: {path}")
    return EXIT_OK


def local(args: argparse.Namespace) -> int:
    """The actions that need no Kilo."""
    c = args.measure_cmd
    if c == "plan":
        from kilo.measure import simulate

        print(simulate.plan())
        return EXIT_OK
    if c == "fit":
        return _fit(args.files, args.turned, args.walk, args.out, args.json)
    if c == "simulate":
        from kilo.measure import record, simulate

        if args.write:
            if args.physics:
                run = simulate.physics_run((-4.0, -3.0, 0.0), (1.5, -2.0), seed=args.seed)
            else:
                rng = __import__("numpy").random.default_rng(args.seed)
                run = simulate.synth_run(simulate.random_truth(rng), backlash=args.backlash, seed=args.seed)
            p = record.save_run(run, args.write)
            print(f"wrote {p}: {len(run)} ticks; the truth: " +
                  " ".join(f"{k}={v:.2f}" for k, v in run.meta["truth"].items()))
            return EXIT_OK
        print(simulate.format_check(simulate.check(args.trials, args.seed, args.backlash)))
        return EXIT_OK
    print("kilo measure fetch: needs --host IP[:PORT] (or Kilo found by name)", file=sys.stderr)
    return EXIT_ERR


def cmd_host(args: argparse.Namespace, hl: Any) -> int:
    """``kilo measure fetch --host``: the last self-measurement run, downloaded and fitted."""
    if args.measure_cmd != "fetch":
        return local(args)
    import time

    from kilo import cli_record
    from kilo.protocol import kilo_protocol as P

    st = hl.status()
    runs = hl.walks()
    ours = [w for w in runs if bytes(w.text).decode("ascii", "replace").startswith("measure")
            and not w.flags & P.WalkFlags.RECORDING]
    if not ours:
        print("kilo measure fetch: no self-measurement run that has ended on the mind (press C5 twice; a run is "
              "fetchable 2 s after it ends)", file=sys.stderr)
        return EXIT_ERR
    w = ours[-1]
    out = Path(args.out or cli_record.DEFAULT_OUT).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    before = set(out.glob("run-*-measure-*.krec"))
    rs = cli_record._rec_status(hl)
    if cli_record.fetch_one(hl, w, st, rs, time.time(), out, quiet=True) != EXIT_OK:
        return EXIT_ERR
    new = sorted(set(out.glob("run-*-measure-*.krec")) - before, key=lambda p: p.stat().st_mtime)
    if not new:
        new = sorted(out.glob(f"run-*-measure-{cli_record.local_id(w)}.krec"), key=lambda p: p.stat().st_mtime)
    if not new:
        print("kilo measure fetch: the download did not leave a file", file=sys.stderr)
        return EXIT_ERR
    if args.no_fit:
        return EXIT_OK
    return _fit([str(new[-1])], [], [], None, False)
