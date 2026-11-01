#!/usr/bin/env python3
"""Generate kernel/mind/behaviour/personality.gen.cpp from a personality YAML.

The C++ mind cannot parse YAML, so the laptop does: kilo.behaviour.personality
loads the file (merged over its DEFAULTS and validated, exactly as the Python
runner loads it) and this script writes the result as the const table
kPersonality (behaviour/personality.h). Doubles are written with repr(), so
the C++ has Python's values bit for bit.

    ~/OpenCat/.venv/bin/python kernel/mind/behaviour/gen_personality.py [PERSONALITY.yaml]
    ~/OpenCat/.venv/bin/python kernel/mind/behaviour/gen_personality.py --check   (kernel/host/test.sh)

Without a path it reads mind/kilo/behaviour/personality.yaml (the packaged
one, $KILO_PERSONALITY's default). `personality_struct(cfg)` is also used by
kernel/host/tests/gen_behaviour_vectors.py.
"""

from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "mind"))

from kilo.behaviour import personality as pers  # noqa: E402

OUT = HERE / "personality.gen.cpp"
MAX_WEIGHTS = 8
MAX_FACTORS = 3


def d(v) -> str:
    v = float(v)
    s = repr(v)
    return s if ("." in s or "e" in s or "inf" in s or "nan" in s) else s + ".0"


def weights(cfg) -> tuple[str, str]:
    counts, rows = [], []
    for b in pers.BEHAVIOURS:
        items = list(cfg["weights"][b].items())
        if len(items) > MAX_WEIGHTS:
            raise SystemExit(f"weights.{b}: more than {MAX_WEIGHTS} terms")
        counts.append(str(len(items)))
        cells = []
        for term, w in items:
            fs = [pers.TERMS.index(f) for f in term.split("*")]
            if len(fs) > MAX_FACTORS:
                raise SystemExit(f"weights.{b}.{term}: more than {MAX_FACTORS} factors")
            fs += [0] * (MAX_FACTORS - len(fs))
            cells.append(f"{{{len(term.split('*'))}, {{{', '.join(map(str, fs))}}}, {d(w)}}}  /* {term} */")
        rows.append("{" + ", ".join(cells) + "}  /* " + b + " */")
    return "{" + ", ".join(counts) + "}", "{\n        " + ",\n        ".join(rows) + "}"


def personality_struct(cfg) -> str:
    dr, s, sf, pc, bh = cfg["drives"], cfg["selector"], cfg["safety"], cfg["percepts"], cfg["behaviours"]
    ini, en, cu, so, ar = dr["initial"], dr["energy"], dr["curiosity"], dr["sociability"], dr["arousal"]
    counts, table = weights(cfg)
    urgent = sum(1 << pers.BEHAVIOURS.index(u) for u in s["urgent"])
    fields = [
        ("name", f'"{cfg["name"]}"'),
        ("drives.initial.energy", d(ini["energy"])), ("drives.initial.curiosity", d(ini["curiosity"])),
        ("drives.initial.sociability", d(ini["sociability"])), ("drives.initial.arousal", d(ini["arousal"])),
        ("drives.energy.drain_active", d(en["drain_active"])), ("drives.energy.drain_idle", d(en["drain_idle"])),
        ("drives.energy.recover_rest", d(en["recover_rest"])),
        ("drives.energy.battery_full_pct", d(en["battery_full_pct"])),
        ("drives.energy.battery_empty_pct", d(en["battery_empty_pct"])),
        ("drives.curiosity.growth", d(cu["growth"])), ("drives.curiosity.satisfy", d(cu["satisfy"])),
        ("drives.sociability.growth", d(so["growth"])), ("drives.sociability.satisfy", d(so["satisfy"])),
        ("drives.arousal.startle", d(ar["startle"])), ("drives.arousal.approach", d(ar["approach"])),
        ("drives.arousal.half_life_s", d(ar["half_life_s"])), ("drives.noise", d(dr["noise"])),
        ("weights: terms per behaviour", counts), ("weights", table),
        ("selector.noise", d(s["noise"])), ("selector.noise_period_s", d(s["noise_period_s"])),
        ("selector.margin", d(s["margin"])), ("selector.min_dwell_s", d(s["min_dwell_s"])),
        ("selector.urgent: " + ", ".join(s["urgent"]), f"0x{urgent:02X}"),
        ("safety.low_battery_pct", d(sf["low_battery_pct"])),
        ("safety.critical_battery_pct", d(sf["critical_battery_pct"])),
        ("safety.wind_down_s", d(sf["wind_down_s"])), ("safety.fallen_settle_s", d(sf["fallen_settle_s"])),
        ("percepts.near_mm", d(pc["near_mm"])), ("percepts.approach_mm_s", d(pc["approach_mm_s"])),
        ("behaviours.idle.posture", f'"{bh["idle"]["posture"]}"'),
        ("behaviours.idle.look_every_s", d(bh["idle"]["look_every_s"])),
        ("behaviours.idle.look_chance", d(bh["idle"]["look_chance"])),
        ("behaviours.idle.look_pan_deg", d(bh["idle"]["look_pan_deg"])),
        ("behaviours.idle.look_tilt_deg", d(bh["idle"]["look_tilt_deg"])),
        ("behaviours.explore.posture", f'"{bh["explore"]["posture"]}"'),
        ("behaviours.explore.look_every_s", d(bh["explore"]["look_every_s"])),
        ("behaviours.explore.looks", str(int(bh["explore"]["looks"]))),
        ("behaviours.explore.look_pan_deg", d(bh["explore"]["look_pan_deg"])),
        ("behaviours.explore.look_tilt_deg", d(bh["explore"]["look_tilt_deg"])),
        ("behaviours.nap.posture", f'"{bh["nap"]["posture"]}"'), ("behaviours.nap.min_s", d(bh["nap"]["min_s"])),
        ("behaviours.greet.trick", f'"{bh["greet"]["trick"]}"'),
        ("behaviours.greet.cooldown_s", d(bh["greet"]["cooldown_s"])),
        ("behaviours.startle.hold_s", d(bh["startle"]["hold_s"])),
    ]
    return "{\n" + "\n".join(f"    {v},  // {k}" for k, v in fields) + "\n}"


def render(path: Path | None) -> str:
    cfg = pers.load(path)
    src = (path or pers.DEFAULT_PATH).resolve()
    try:
        src = src.relative_to(REPO)
    except ValueError:
        pass
    return ("// Generated by kernel/mind/behaviour/gen_personality.py from " + str(src) + ".\n"
            "// Do not edit: change the YAML and re-run the script (kernel/host/test.sh runs it with --check).\n"
            '#include "behaviour/personality.h"\n\n'
            "namespace kilo {\nnamespace behaviour {\n\n"
            f"const Personality kPersonality = {personality_struct(cfg)};\n\n"
            "}  // namespace behaviour\n}  // namespace kilo\n")


def main(argv: list[str]) -> int:
    check = "--check" in argv
    paths = [a for a in argv if not a.startswith("--")]
    text = render(Path(paths[0]) if paths else None)
    if check:
        if not OUT.exists() or OUT.read_text() != text:
            print(f"{OUT.relative_to(REPO)} is stale: run kernel/mind/behaviour/gen_personality.py", file=sys.stderr)
            return 1
        return 0
    OUT.write_text(text)
    print(f"wrote {OUT.relative_to(REPO)} (sizeof(Personality) is about 1 KB on a 32-bit target)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
