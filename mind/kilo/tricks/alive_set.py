"""The original Nybble's tricks as occasional moves of the random mode (the remote's A5).

The owner, 2026-11-22: bring back a few of the original tricks (the stretch, licking the paw,
peeing...) "as occasional movements in the random mode": while the behaviour runner idles
(sitting) or explores (standing), the alive layer (kernel/mind/behaviour/alive.h) plays one of
them now and then, and A5 pressed while it runs plays one at once.

Each trick of SET comes in two variants, one for each pose the alive layer holds:

    alive_<name>_sit     from SIT, back to SIT (end posture SIT)
    alive_<name>_stand   from STAND, back to STAND (end posture STAND; the alive layer's STANCE is
                         the same legs, the IK stand the walks start from)

built from the legacy skill (legacy/InstinctNybble.h) with the importer's timing
(kilo.tricks.legacy: the original firmware's own, a finite loop unrolled), so:

1. ``via``: for a trick whose original start is far from the base, first the move to its
   natural start (STAND or SIT: the body's own posture, at the original's posture rate,
   2 deg a pass), checked like the rest;
2. the trick's rows, the first one sized from where step 1 left the legs (the original's
   transform() from that pose), each later one from the row before, as the original;
3. a posture skill (str, buttUp: held until the next command by the original) moves there at the
   original's posture rate and holds POSTURE_HOLD_TICKS;
4. back to the base exactly, at the original's posture rate but never quicker than the body's
   posture change (POSTURE_DUR_TICKS), so the end posture the body takes afterwards is no move.
   Stopped half way (B6, A7, C7, the runner leaving the calm behaviours), it goes back over
   POSTURE_DUR_TICKS too (stop_ticks), not the body's 0.2 s crossfade.

The alive layer plays them at the personality's ``alive.trick_speed`` (personality.yaml).

Every variant is held to the one body model (``check()``, mind/tests/test_alive_tricks.py): along
every move, sampled every 2 degrees of the largest joint change, the rigid quasi-static 3-D model
(kilo.tricks.stance) on Kilo as designed (the right hind shoulder recalibrated) and as recorded
(it stops at -57 deg) rests on the feet, the tail, the bottom or the knees only, with the centre of
mass over its support, and never topples by more than DROP_MAX_MM at once (a rock onto another
foot, as the alive layer's own sitting moves are allowed). A lean on three feet (the pee: 16 deg of
roll, the centre of mass 15 mm inside them) is the trick, not a fault. What the model cannot see:
inertia, the legs' give, a foot sliding. The tricks left out are in DROPPED, with why.

    python -m kilo.tricks.alive_set [--check]      (from mind/; then kernel/mind/tricks/gen_library.py)
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path

from . import legacy
from . import stance as T
from .library import FIRST_VERSION, LIBRARY_DIR, path_for
from .model import CANONICAL, JOINT_LIMITS, JOINT_NAMES, Easing, Keyframe, Posture, Trick

# The tricks of the random mode, by their library name, in the order the personality lists them.
SET = ("stretch", "wash_face", "pee", "sniff", "check_around")
# Original tricks looked at and left out (name: why), for the docs and the tests.
DROPPED = {
    "butt_up": ("its front shoulders ask for 90 deg; the limit is 80 and the left one's servo stops at 62, "
                "so the model has him on three feet, rolled 18 deg, the right belly 8 mm off the floor; "
                "the stretch is the same front-low, rear-high move without that"),
    "scratch": ("lifting the hind leg tips him 7.7 mm onto a foot at once on Kilo as recorded (the right "
                "hind shoulder stops at -57 deg), over the 6 mm a sitting move may; 5.9 mm with that "
                "shoulder recalibrated"),
    "hi": "a wave, not a cat's idle move; the greet behaviour plays it",
    "push_up": "not a cat's move; the model puts his right belly on the floor (34 deg of roll)",
}
BASES = ("sit", "stand")
# Where a trick's original rows start from, when that is not the base (the move there first).
# The legacy skills all start from `up`; wash_face sits first itself (its first row is a sit).
VIA = {
    ("stretch", "sit"): "stand",
    ("pee", "sit"): "stand",
    ("sniff", "sit"): "stand",
    ("check_around", "sit"): "stand",
}

POSTURE_SPEED = 8  # the original's posture transform: speedRatio 2 (2 deg a pass), as a behaviour row's speed
POSTURE_HOLD_TICKS = legacy.POSTURE_HOLD_TICKS
POSTURE_DUR_TICKS = legacy.POSTURE_DUR_TICKS

# The model check's bounds (rigid, quasi-static: see the module doc).
DROP_MAX_MM = 6.0   # the largest single topple: a rock onto another foot, not a fall
DEG_PER_SAMPLE = 2.0
REFINE = 8

_SKILLS = legacy.parse_file()
# The bases as the body holds them (its joint limits applied: SIT's rear shoulders -90 -> -80), 16
# slots, and the legacy rows (16 slots, joints 3-7 too) the original's maxDiff starts from.
_BASE_TRICK = {"sit": "sit", "stand": "stand"}
_BASE_LEGACY = {"sit": "sit", "stand": "up"}
END_POSTURE = {"sit": Posture.SIT, "stand": Posture.STAND}


def name_of(trick: str, base: str) -> str:
    return f"alive_{trick}_{base}"


def names() -> list[str]:
    return [name_of(t, b) for t in SET for b in BASES]


def base_angles(base: str) -> tuple:
    """The base pose as the body holds it (16 slots, None on the never-driven 3-7)."""
    a = Trick.from_json(path_for(_BASE_TRICK[base]).read_text()).frames[0].angles
    return tuple(a)


def _base_legacy(base: str) -> tuple:
    return _SKILLS[_BASE_LEGACY[base]].angles(0)


def _clamped(angles) -> tuple:
    out = [None] * len(angles)
    for j in CANONICAL:
        v = angles[j]
        if v is not None:
            lo, hi = JOINT_LIMITS[j]
            out[j] = min(hi, max(lo, v))
    return tuple(out)


def _posture_ticks(frm, to) -> int:
    """The original's posture move from ``frm`` to ``to`` (16-slot legacy poses), in ticks."""
    return legacy.speed_to_ticks(legacy._max_diff(frm, to), POSTURE_SPEED)


def build(trick: str, base: str) -> Trick:
    skill = _SKILLS[FIRST_VERSION[trick]]
    via = VIA.get((trick, base))
    frames: list[Keyframe] = []
    notes = [f"the random mode's {trick} (legacy `{skill.name}`) from {base.upper()} back to {base.upper()}: "
             "kilo.tricks.alive_set"]
    here = _base_legacy(base)  # where the original's maxDiff starts
    if via is not None:
        to = _base_legacy(via)
        frames.append(Keyframe(_clamped(to), dur=max(POSTURE_DUR_TICKS, _posture_ticks(here, to)),
                               easing=Easing.COSINE))
        notes.append(f"first to {via.upper()} (the original's start), at the posture rate, at least "
                     f"{POSTURE_DUR_TICKS} ticks")
        here = to
    if skill.kind == "posture":
        to = skill.angles(0)
        frames.append(Keyframe(_clamped(to), dur=_posture_ticks(here, to), hold=POSTURE_HOLD_TICKS,
                               easing=Easing.COSINE))
        notes.append(f"the posture at the original's posture rate (2 deg a pass of {legacy.LEGACY_STEP_MS} ms), "
                     f"held {POSTURE_HOLD_TICKS} ticks")
        last = to
    else:
        core = legacy.to_trick(skill, name=trick, skills=_SKILLS, start=here)
        assert core.repeats == 0, f"{trick}: its loop is not unrolled"
        frames += [replace(f, balance=False) for f in core.frames]
        order = legacy.play_order(skill)
        last = skill.angles(order[-1])
        notes.append(f"rows {order} timed as the original plays them from there "
                     f"({legacy.LEGACY_STEP_MS} ms a pass)")
    home = base_angles(base)
    frames.append(Keyframe(home, dur=max(POSTURE_DUR_TICKS, _posture_ticks(last, _base_legacy(base))),
                           easing=Easing.COSINE))
    notes.append(f"back to {base.upper()} at the posture rate, at least {POSTURE_DUR_TICKS} ticks; "
                 f"end posture {base.upper()} (no move after)")
    notes.append(f"stopped (B6, A7, C7, the random mode leaving), back to {base.upper()} over "
                 f"{POSTURE_DUR_TICKS} ticks, not the body's 0.2 s crossfade")
    return Trick(name=name_of(trick, base), frames=tuple(frames), end_posture=END_POSTURE[base],
                 expected_roll=0, expected_pitch=0, balance=False, stop_ticks=POSTURE_DUR_TICKS,
                 source={"file": "legacy/InstinctNybble.h", "legacy_name": skill.name, "kind": "alive",
                         "trick": trick, "base": base},
                 notes=tuple(notes)).validate()


def build_all() -> dict[str, Trick]:
    return {name_of(t, b): build(t, b) for t in SET for b in BASES}


# ---- the model check ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Check:
    margin: float      # mm, the least of the centre of mass inside the support (>= 0: at rest)
    roll: float        # deg, the largest |roll|
    pitch: tuple       # deg (lo, hi), nose up +
    drop: float        # mm, the largest single topple
    contacts: frozenset  # everything that touched the floor

    def problems(self) -> list[str]:
        out = []
        bad = sorted(self.contacts - T.ALLOWED)
        if bad:
            out.append(f"rests on {bad}")
        if self.margin < 0:
            out.append(f"the centre of mass {-self.margin:.1f} mm outside the support")
        if self.drop > DROP_MAX_MM:
            out.append(f"topples {self.drop:.1f} mm at once (> {DROP_MAX_MM:g})")
        return out


def _joints(angles) -> dict:
    return {JOINT_NAMES[j]: float(angles[j]) for j in CANONICAL if angles[j] is not None}


def check(t: Trick, base: str, g3: T.Geometry3) -> Check:
    """The rigid quasi-static model along the trick, from its base at rest."""
    start = _joints(base_angles(base))
    cur = dict(start)
    r = T.rest(g3, start)
    R = r.R
    margin, roll, drop = r.margin, abs(r.roll), 0.0
    pitch = [r.pitch, r.pitch]
    contacts = set(r.contacts)
    for f in t.frames:
        nxt = {**cur, **_joints(f.angles)}
        span = max(abs(T.applied(T.JOINT_IDS[k], nxt[k], g3.czp) - T.applied(T.JOINT_IDS[k], cur[k], g3.czp))
                   for k in nxt)
        n = max(2, math.ceil(span / DEG_PER_SAMPLE))
        for i in range(1, n + 1):
            R_before = R
            r = T.rest(g3, T.lerp(cur, nxt, i / n), R)
            d = max([0.0] + [x for _, _, x in r.topples])
            if d > DROP_MAX_MM:
                # A tilt that is only quick (the body turning steadily as a leg straightens) shows as
                # a long topple between two samples; a real topple stays as long however finely the
                # move is sampled. Look again between the two samples, REFINE times as finely.
                Rf, d = R_before, 0.0
                for k in range(1, REFINE + 1):
                    rf = T.rest(g3, T.lerp(cur, nxt, (i - 1 + k / REFINE) / n), Rf)
                    Rf = rf.R
                    d = max([d] + [x for _, _, x in rf.topples])
            R = r.R
            margin = min(margin, r.margin)
            roll = max(roll, abs(r.roll))
            pitch = [min(pitch[0], r.pitch), max(pitch[1], r.pitch)]
            drop = max(drop, d)
            contacts |= set(r.contacts)
        cur = nxt
    return Check(margin, roll, tuple(pitch), drop, frozenset(contacts))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Write the random mode's tricks into the library.")
    ap.add_argument("--check", action="store_true", help="only report stale files")
    args = ap.parse_args(argv)
    files = {path_for(n): t.to_json() for n, t in build_all().items()}
    stale = [p for p, text in files.items() if not p.exists() or p.read_text() != text]
    if args.check:
        for p in stale:
            print(f"stale: {p.name}")
        return 1 if stale else 0
    LIBRARY_DIR.mkdir(exist_ok=True)
    for p in stale:
        p.write_text(files[p])
        print(f"wrote {p.name}")
    print(f"{len(files)} tricks, {len(stale)} changed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
