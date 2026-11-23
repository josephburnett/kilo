"""The state estimate replayed over flight recordings (.krec): what it would
have said, against what Kilo had been told to hold.

    python -m kilo.intent.replay FILE.krec ...            the confusion table over the files
    python -m kilo.intent.replay --changes FILE.krec      every change of state in one file

The prior is rebuilt as the mind builds it (kernel/mind/mind.cpp
Mind::intent_cmd): the SET_POSTURE last sent, or the end posture of the
trick last played (protocol 2 recordings), or nothing while a POSE (FEET)
stream plays.

The confusion table counts TELEMETRY, by what was asked (rows) and what
the estimate says (columns), over the stretches where the answer can be
judged: the body IDLE (or LIMP), the same posture asked for at least
ASKED_S. "Wrong" lists each stretch where a posture was asked and the
estimate named another resting state.
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter

from kilo.intent import estimate as E
from kilo.logs import flight, proto2
from kilo.protocol import kilo_protocol as P

ASKED_S = 3.0   # a posture counts as held once it has been the one asked for this long


def samples(s: flight.Stream):
    """(Sample, asked since us) for every TELEMETRY of the stream."""
    cmd, since = E.Cmd.NONE, 0
    trick_end = None
    for f in s.frames:
        m = f.msg
        if f.direction == "tx":
            if isinstance(m, P.SetPosture):
                c = E.cmd_of_posture(m.posture)
                if c != cmd:
                    cmd, since = c, f.t_us
            elif isinstance(m, proto2.TrickBegin):
                trick_end = m.end_posture
            elif isinstance(m, proto2.TrickPlay) or proto2.is_stream(m):
                if cmd != E.Cmd.NONE:
                    cmd, since = E.Cmd.NONE, f.t_us
            continue
        if isinstance(m, P.Event):
            c = None
            if m.code == proto2.EVENT_TRICK_DONE and trick_end is not None:
                # DONE or STOPPED: he is in the trick's end posture; FELL or PREEMPTED: nobody knows
                ok = ((m.arg & 0xFFFF) >> 8) in (0, 1)
                c = E.cmd_of_posture(trick_end) if ok else E.Cmd.NONE
            elif m.code == proto2.EVENT_RECOVERED:
                c = E.Cmd.STAND      # protocol 2: the body's get-up ended in STAND
            elif m.code == P.EventCode.LYING:
                c = E.Cmd.REST       # the fall ended lying: REST, then LIMP (protocol 2: RECOVERY_FAILED)
            elif m.code == P.EventCode.FELL:
                c = E.Cmd.NONE
            if c is not None and c != cmd:
                cmd, since = c, f.t_us
            continue
        if isinstance(m, P.Telemetry):
            mode = int(P.Mode.STREAM) if m.mode == proto2.MODE_TRICK else m.mode  # protocol 2: a trick played
            yield E.Sample(f.t_us, m.pitch, m.roll, m.rate_pitch, m.rate_roll, m.rate_yaw, mode, cmd), since


def replay(path: str, changes: bool = False):
    s = flight.load(path)
    est = E.Estimator()
    table: Counter = Counter()
    wrong = []
    run = None   # (asked, said, t0, t1, pitch, roll)
    for smp, since in samples(s):
        if est.feed(smp) and changes:
            e = est.est
            print(f"{s.uptime_s(smp.t_us):9.2f} s  state: {e.state.name} (pitch {smp.pitch / 16:+.1f} roll "
                  f"{smp.roll / 16:+.1f}, asked {smp.cmd.name.lower()}, confidence {e.confidence})")
        judged = smp.mode in (int(P.Mode.IDLE), int(P.Mode.LIMP)) and smp.t_us - since >= ASKED_S * 1e6
        if not judged:
            run = None
            continue
        st = est.est.state
        table[(smp.cmd, st)] += 1
        want = E.CMD_STATE.get(smp.cmd)
        bad = want is not None and st not in (want, E.State.MOVING)
        if bad:
            if run is not None and run[0] == smp.cmd and run[1] == st:
                run[3] = smp.t_us
            else:
                run = [smp.cmd, st, smp.t_us, smp.t_us, smp.pitch / 16, smp.roll / 16]
                wrong.append((path, s, run))
        else:
            run = None
    return table, wrong


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("files", nargs="+")
    ap.add_argument("--changes", action="store_true")
    a = ap.parse_args(argv)
    total: Counter = Counter()
    wrong = []
    for f in a.files:
        try:
            t, w = replay(f, a.changes)
        except Exception as e:  # a damaged or foreign file: say so and go on
            print(f"{f}: {e}", file=sys.stderr)
            continue
        total.update(t)
        wrong += w
    cols = [st for st in E.State]
    print("asked \\ estimate  " + " ".join(f"{c.name[:12]:>12}" for c in cols))
    for cmd in E.Cmd:
        row = [total.get((cmd, c), 0) for c in cols]
        if sum(row):
            print(f"{cmd.name.lower():17s} " + " ".join(f"{v:12d}" for v in row))
    print(f"\nwrong: {len(wrong)} stretches (a posture asked, another resting state estimated)")
    for path, s, (cmd, st, t0, t1, p, r) in wrong:
        print(f"  {path.split('/')[-1]} {s.uptime_s(t0):8.1f} s for {(t1 - t0) / 1e6:5.1f} s: asked {cmd.name.lower()}, "
              f"estimate {st.name}, pitch {p:+.1f} roll {r:+.1f} at its start")
    return 0


if __name__ == "__main__":
    sys.exit(main())
