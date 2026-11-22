"""``kilo model``: Kilo's one body model (docs/architecture.md "One model").

    kilo model show [--fit FILE.fit.json]   every number of the model in plain words: what it is, its
                                            unit, where it came from; then the numbers that disagree
                                            (what a part of the mind fitted for itself, against the
                                            model; with --fit, against a self-measurement too)
    kilo model apply FILE.fit.json [--dry-run] [--no-rear-up]
                                            write what a self-measurement found (kilo measure fit)
                                            into protocol/spec.toml, regenerate everything made from
                                            it, and say what changed for Kilo

Everything about Kilo's body is in protocol/spec.toml ([model] and the LEG_*
params); the code reads it through the generated bindings and
kilo.planner.model.BodyModel. ``apply`` is the only way a measurement gets
in: it edits that one file and regenerates the bindings, the vendored
copies, the planner's and the other golden vectors, the rear-up's keyframes
and the mind's trick table, the intent layer's moves (stand, sit, rest; the
righting experiment's) and its state estimate's table. It never commits and never touches Kilo or his
card: the owner looks at ``git diff``, runs the test suites and decides.
``--dry-run`` does all of it in a temporary copy of the checkout and changes
nothing here.

What a fit's numbers become (only those the fit marks significant, not at a
limit of the fit, and inside what is plausible for Kilo):

    com_x, com_y, com_z   [model.correction] com_d*_mm: moved by (fitted - the model's then)
    sag                   [model.correction] sag_deg_per_kg_cm
    twist                 [model.correction] leg_dz_mm: +t/2 on LF and RB, -t/2 on RF and LB
    imu_pitch, imu_roll   [model.correction] imu_*_deg (with the floor's slope, unless the fit had a
                          turned run)
    lag_ms, tau_ms        [model] servo_lag_ms, servo_tau_ms (whole ms)
    lag_d_ms:JOINT, tau_d_ms:JOINT, give:JOINT
                          one servo's own (JOINT a joint name, e.g. lag_d_ms:lb_knee): [model.correction]
                          servo_lag_d_ms, servo_tau_d_ms (ms, later / slower than the robot's) and
                          servo_give (times the legs' sag), at that joint's legacy id
    length, track         the LEG_LB_MM and LEG_W_MM params' defaults (whole mm). These are also the
                          BODY's parameters: its firmware changes, and a body that has saved its own
                          values keeps them until they are set (kilo param).
    anything else         no place in the model: listed, not applied
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from kilo import EXIT_ERR, EXIT_OK

REPO = Path(__file__).resolve().parents[2]
SPEC = Path("protocol") / "spec.toml"
LEG_NAMES = ("left front", "right front", "right back", "left back")
# view key -> the fit parameter that measures the same thing
VIEW_FIT = {"com_x_mm": "com_x", "com_dx_mm": "com_x", "com_dy_mm": "com_y", "com_dz_mm": "com_z",
            "com_h_mm": "com_z", "sag_deg_per_kg_cm": "sag", "imu_pitch_deg": "imu_pitch",
            "imu_roll_deg": "imu_roll"}
# What is plausible for Kilo, by what is written: outside it a fitted number is left out (a fit that ran into
# its own limit, or a run that went wrong).
PLAUSIBLE = {"com": (-20.0, 20.0), "sag": (0.0, 10.0), "twist": (-4.0, 4.0), "imu": (-10.0, 10.0),
             "lag": (10.0, 200.0), "tau": (10.0, 300.0), "length": (100.0, 150.0), "track": (55.0, 80.0),
             "lag_d": (-40.0, 150.0), "tau_d": (-50.0, 200.0), "give": (0.2, 5.0)}
# per-servo fit parameters "KIND:joint" -> ([model.correction] key, digits)
PER_SERVO = {"lag_d_ms": ("servo_lag_d_ms", 0), "tau_d_ms": ("servo_tau_d_ms", 0), "give": ("servo_give", 2)}


def add_parser(sub: Any) -> None:
    s = sub.add_parser("model", help="Kilo's one body model: show it in plain words, or apply a self-measurement "
                                     "to it",
                       description=__doc__.split("\n\n")[0])
    ms = s.add_subparsers(dest="model_cmd", required=True, metavar="ACTION")
    sh = ms.add_parser("show", help="every number of the model, where it came from, and the numbers that disagree")
    sh.add_argument("--fit", metavar="FILE", help="a self-measurement's numbers (kilo measure fit: FILE.fit.json) "
                                                  "to compare with")
    ap = ms.add_parser("apply", help="write a self-measurement's numbers into protocol/spec.toml, regenerate, and "
                                     "report what changed for Kilo (never commits, never touches the card)")
    ap.add_argument("fit", metavar="FILE.fit.json")
    ap.add_argument("--dry-run", action="store_true", help="in a temporary copy of the checkout: nothing changes here")
    ap.add_argument("--no-rear-up", action="store_true",
                    help="skip rebuilding and checking the rear-up trick (several minutes)")
    ap.add_argument("--repo", type=Path, default=None, help=argparse.SUPPRESS)


# ---- the model, in words -------------------------------------------------------------------------
def _value(key: str) -> Any:
    from kilo.protocol import kilo_protocol as P

    return getattr(P, "MODEL_" + key.replace(".", "_").upper())


def _num(v: Any) -> str:
    if isinstance(v, (tuple, list)):
        return "(" + ", ".join(_num(x) for x in v) + ")"
    if isinstance(v, float):
        return f"{v:g}"
    return str(v)


SECTIONS = (("", "Masses, reach and servo timing"), ("correction", "Corrections from a self-measurement"),
            ("servo", "Servo zeros (calibration)"), ("tail", "The tail"), ("outline", "The frame's outline"),
            ("inertia", "Inertia"), ("sensor", "The distance sensor"))
LEG_PARAMS = (("LEG_L1_MM", "upper leg, shoulder to knee"), ("LEG_L2_MM", "lower leg, knee to the foot's centre"),
              ("LEG_LB_MM", "front feet to back feet (taken as the shoulders' spacing)"),
              ("LEG_W_MM", "left feet to right feet"))


def load_fit(path: str | Path) -> dict:
    d = json.loads(Path(path).read_text())
    if not isinstance(d, dict) or not isinstance(d.get("parameters"), dict):
        raise ValueError(f"{path}: not a fit file (kilo measure fit writes FILE.fit.json)")
    return d


def show(fit: dict | None = None) -> str:
    from kilo.planner.model import VIEWS, BodyModel
    from kilo.protocol import kilo_protocol as P

    m = BodyModel()
    L = ["Kilo's body model (protocol/spec.toml; the code reads it from there and nowhere else)", ""]

    def row(name: str, val: str, unit: str, what: str, src: str) -> None:
        L.append(f"  {name:<28} {val + (' ' + unit if unit else ''):<26} {what}  [{src}]")

    L.append("Size")
    for name, what in LEG_PARAMS:
        p = P.PARAMS_BY_NAME[name]
        row(name, _num(p.default), "mm", what, "measured 2026-10-11")
    for prefix, title in SECTIONS:
        keys = [k for k in P.MODEL_ABOUT if (k.split(".")[0] if "." in k else "") == prefix]
        if not keys:
            continue
        L += ["", title]
        for k in keys:
            what, unit, src = P.MODEL_ABOUT[k]
            row(k.split(".")[-1], _num(_value(k)), unit, what, src)
    cx, cy, ch = m.stance_com()
    L += ["", "Worked out from the above, standing ready to walk",
          f"  weight {m.total_g:.0f} g; centre of mass {cx:.1f} mm ahead of the middle of the shoulders, "
          f"{cy:+.1f} mm to the left, {ch:.1f} mm above the floor"]
    L += ["", "NUMBERS THAT DISAGREE",
          "  Parts of the mind that fitted a number for themselves and still use it instead of the model's.",
          "  Each stays until a self-measurement settles it; then its table is deleted from the spec."]
    fp = (fit or {}).get("parameters", {})
    for name in VIEWS:
        what, _, src = P.MODEL_ABOUT[f"views.{name}"]
        L += ["", f"  {name}: {what}  [{src}]"]
        for k, v, base in m.disagreements(name):
            if base is None:
                L.append(f"    {k:<20} {v:g}  (its own: the model has no such number)")
                continue
            line = f"    {k:<20} {v:g}  against the model's {base:.2f}  (off by {v - base:+.2f})"
            f = _fit_as(k, fp, m)
            if f is not None:
                line += f"; the measurement says {f[0]:.2f} +- {f[1]:.2f}"
            L.append(line)
    if fit is not None:
        L += ["", "THE MEASUREMENT AGAINST THE MODEL  (" + ", ".join(Path(p).name for p in fit.get("inputs", [])) + ")"]
        for ch_ in plan_changes(fit, (REPO / SPEC).read_text()):
            L.append("  " + ch_.line())
        L += ["  (kilo model apply FILE.fit.json --dry-run shows what applying it would change for Kilo)"]
    return "\n".join(L)


def _fit_as(view_key: str, fp: dict, m: Any) -> tuple[float, float] | None:
    """A fit's number in the terms of a view's key: (value, +-), or None."""
    p = fp.get(VIEW_FIT.get(view_key, ""))
    if not p or not _usable(p):
        return None
    v, sd = float(p["value"]), float(p.get("sd", 0.0))
    lo, hi = PLAUSIBLE["com"]
    if VIEW_FIT[view_key].startswith("com_") and not lo <= v - float(p.get("model", v)) <= hi:
        return None  # a fit that ran into its limit says nothing
    if view_key in ("com_dx_mm", "com_dy_mm", "com_dz_mm"):  # a correction: fitted minus the model's then, plus ours
        cur = {"com_dx_mm": m.com_dx_mm, "com_dy_mm": m.com_dy_mm, "com_dz_mm": m.com_dz_mm}[view_key]
        return cur + v - float(p["model"]), sd
    if view_key == "com_h_mm":  # above the floor: the fit's com_z is above the shoulder axles
        return m.stance_height(205) + m.foot_r_mm + v, sd
    return v, sd


def _usable(p: dict) -> bool:
    return bool(p.get("fitted", True)) and not p.get("at_limit", False)


# ---- what a fit changes ----------------------------------------------------------------------------
@dataclass
class Change:
    fit: str                 # the fit's parameter
    where: str               # the place in protocol/spec.toml, in words
    old: Any = None
    new: Any = None
    sd: float = 0.0
    why_not: str = ""        # set: not applied, and why

    @property
    def applied(self) -> bool:
        return not self.why_not and self.new != self.old

    def line(self) -> str:
        if self.why_not:
            return f"{self.fit:<10} not applied: {self.why_not}"
        if self.new == self.old:
            return f"{self.fit:<10} {self.where}: stays {_num(self.old)}"
        return f"{self.fit:<10} {self.where}: {_num(self.old)} -> {_num(self.new)}  (+- {self.sd:.2f})"


def _section(text: str, table: str) -> tuple[int, int]:
    m = re.search(rf"^\[{re.escape(table)}\]\s*$", text, re.M)
    if not m:
        raise ValueError(f"protocol/spec.toml: no [{table}]")
    nxt = re.compile(r"^\[", re.M).search(text, m.end())
    return m.end(), nxt.start() if nxt else len(text)


def toml_get(text: str, table: str, key: str) -> str:
    a, b = _section(text, table)
    m = re.search(rf"^{re.escape(key)} = (.*)$", text[a:b], re.M)
    if not m:
        raise ValueError(f"protocol/spec.toml: no {key} in [{table}]")
    return m.group(1).strip()


def toml_set(text: str, table: str, key: str, value: str) -> str:
    a, b = _section(text, table)
    body, n = re.subn(rf"^{re.escape(key)} = .*$", f"{key} = {value}", text[a:b], count=1, flags=re.M)
    if n != 1:
        raise ValueError(f"protocol/spec.toml: no {key} in [{table}]")
    return text[:a] + body + text[b:]


def _param_span(text: str, name: str) -> tuple[int, int]:
    m = re.search(rf'^name = "{re.escape(name)}"\s*$', text, re.M)
    if not m:
        raise ValueError(f"protocol/spec.toml: no param {name}")
    d = re.compile(r"^default = (.*)$", re.M).search(text, m.end())
    nxt = re.compile(r"^\[\[", re.M).search(text, m.end())
    if not d or (nxt and d.start() > nxt.start()):
        raise ValueError(f"protocol/spec.toml: param {name} has no default")
    return d.start(1), d.end(1)


def param_get(text: str, name: str) -> str:
    a, b = _param_span(text, name)
    return text[a:b].strip()


def param_set(text: str, name: str, value: str) -> str:
    a, b = _param_span(text, name)
    return text[:a] + value + text[b:]


def about_set(text: str, key: str, source: str) -> str:
    """[model.about]'s source string of ``key`` (its third)."""
    a, b = _section(text, "model.about")
    k = f'"{key}"' if "." in key else key
    body, n = re.subn(rf'^({re.escape(k)} = \[".*", ".*", )".*"\]$', lambda m: f'{m.group(1)}"{source}"]',
                      text[a:b], count=1, flags=re.M)
    if n != 1:
        raise ValueError(f"protocol/spec.toml: no {key} in [model.about]")
    return text[:a] + body + text[b:]


def _floats(s: str) -> list[float]:
    return [float(x) for x in s.strip("[]").split(",")]


def _joint_id(name: str) -> int | None:
    """A joint's legacy id 0..15 from its name (lb_knee), or None."""
    from kilo.tricks.model import JOINT_IDS

    return JOINT_IDS.get(name)


def plan_changes(fit: dict, spec_text: str) -> list[Change]:
    """What applying ``fit`` would write, parameter by parameter (nothing is written here)."""
    fp = fit["parameters"]
    out: list[Change] = []

    def take(name: str, kind: str, value: float) -> str:
        """Why parameter ``name`` is not applied ('' if it is)."""
        p = fp[name]
        if not p.get("fitted", True):
            return "the fit held it fixed"
        if p.get("at_limit", False):
            return "the fit ran into its limit: not measured"
        if not p.get("significant", True):
            return (f"within the measurement's error of the model's ({float(p['value']):.2f} +- "
                    f"{float(p.get('sd', 0.0)):.2f} against {float(p.get('model', 0.0)):.2f})")
        lo, hi = PLAUSIBLE[kind]
        if not lo <= value <= hi:
            return f"{value:.1f} is outside what is plausible for Kilo ({lo:g} to {hi:g}): not measured"
        return ""

    for name, key in (("com_x", "com_dx_mm"), ("com_y", "com_dy_mm"), ("com_z", "com_dz_mm")):
        if name not in fp:
            continue
        p = fp[name]
        old = float(toml_get(spec_text, "model.correction", key))
        new = round(old + float(p["value"]) - float(p["model"]), 1)
        out.append(Change(name, f"centre of mass correction {key}", old, new, float(p.get("sd", 0.0)),
                          take(name, "com", new)))
    for name, key, kind, nd in (("sag", "sag_deg_per_kg_cm", "sag", 1), ("imu_pitch", "imu_pitch_deg", "imu", 1),
                                ("imu_roll", "imu_roll_deg", "imu", 1)):
        if name not in fp:
            continue
        p = fp[name]
        new = round(float(p["value"]), nd)
        out.append(Change(name, f"correction {key}", float(toml_get(spec_text, "model.correction", key)), new,
                          float(p.get("sd", 0.0)), take(name, kind, new)))
    if "twist" in fp:
        p = fp["twist"]
        t = float(p["value"])
        new = [round(s * t / 2, 2) + 0.0 for s in (1, -1, 1, -1)]
        out.append(Change("twist", "leg lengths' correction leg_dz_mm (" + ", ".join(LEG_NAMES) + ")",
                          _floats(toml_get(spec_text, "model.correction", "leg_dz_mm")), new,
                          float(p.get("sd", 0.0)), take("twist", "twist", t)))
    for name, key, kind in (("lag_ms", "servo_lag_ms", "lag"), ("tau_ms", "servo_tau_ms", "tau")):
        if name not in fp:
            continue
        p = fp[name]
        new = int(round(float(p["value"])))
        out.append(Change(name, f"servo timing {key}", int(float(toml_get(spec_text, "model", key))), new,
                          float(p.get("sd", 0.0)), take(name, kind, new)))
    for kind, (key, nd) in PER_SERVO.items():
        old = _floats(toml_get(spec_text, "model.correction", key))
        new, sds, why = list(old), [], []
        for name in fp:
            k, _, joint = name.partition(":")
            if k != kind:
                continue
            j = _joint_id(joint)
            p = fp[name]
            v = round(float(p["value"]), nd) + 0.0
            w = "no such joint" if j is None else take(name, kind.split("_ms")[0], v)
            if w:
                why.append(f"{joint}: {w}")
                continue
            new[j] = v
            sds.append(float(p.get("sd", 0.0)))
        if sds or why:
            out.append(Change(kind, f"each servo's {key} (by joint id)", old, new, max(sds, default=0.0),
                              "" if sds else "; ".join(why)))
    for name, param in (("length", "LEG_LB_MM"), ("track", "LEG_W_MM")):
        if name not in fp:
            continue
        p = fp[name]
        new = int(round(float(p["value"])))
        out.append(Change(name, f"the feet's spacing {param} (also the body's own parameter)",
                          int(float(param_get(spec_text, param))), new, float(p.get("sd", 0.0)),
                          take(name, name, new)))
    known = {c.fit for c in out}
    for name in fp:
        if name not in known and name.partition(":")[0] not in known:
            out.append(Change(name, "", why_not="no place in the model (the measurement's own number)"))
    return out


def edit_spec(text: str, changes: list[Change], source: str) -> str:
    """protocol/spec.toml with the applied changes written (comments and layout kept)."""
    for c in changes:
        if not c.applied:
            continue
        if c.fit in ("com_x", "com_y", "com_z"):
            key = {"com_x": "com_dx_mm", "com_y": "com_dy_mm", "com_z": "com_dz_mm"}[c.fit]
            text = about_set(toml_set(text, "model.correction", key, repr(float(c.new))), f"correction.{key}", source)
        elif c.fit in ("sag", "imu_pitch", "imu_roll"):
            key = {"sag": "sag_deg_per_kg_cm", "imu_pitch": "imu_pitch_deg", "imu_roll": "imu_roll_deg"}[c.fit]
            text = about_set(toml_set(text, "model.correction", key, repr(float(c.new))), f"correction.{key}", source)
        elif c.fit == "twist":
            text = about_set(toml_set(text, "model.correction", "leg_dz_mm",
                                      "[" + ", ".join(repr(float(v)) for v in c.new) + "]"),
                             "correction.leg_dz_mm", source)
        elif c.fit in ("lag_ms", "tau_ms"):
            key = {"lag_ms": "servo_lag_ms", "tau_ms": "servo_tau_ms"}[c.fit]
            text = about_set(toml_set(text, "model", key, str(int(c.new))), key, source)
        elif c.fit in PER_SERVO:
            key = PER_SERVO[c.fit][0]
            text = about_set(toml_set(text, "model.correction", key,
                                      "[" + ", ".join(repr(float(v)) for v in c.new) + "]"),
                             f"correction.{key}", source)
        elif c.fit in ("length", "track"):
            text = param_set(text, {"length": "LEG_LB_MM", "track": "LEG_W_MM"}[c.fit], str(int(c.new)))
    return text


# ---- what the model means for Kilo (run in the tree it describes) ---------------------------------------
def metrics() -> dict:
    """Numbers that say what this checkout's model means for Kilo: where his centre of mass is
    standing, and for every walk the least distance his centre of mass keeps inside his feet
    while at least three are down (the planner asks for PlannerConfig.margin_mm), and the largest
    lean it plans. Run by ``apply`` in the tree before and after (a subprocess: the model is read
    at import)."""
    from kilo.planner import patterns as pat
    from kilo.planner.model import VIEWS, BodyModel
    from kilo.planner.plan import Planner

    m = BodyModel()
    cx, cy, ch = m.stance_com()
    out: dict = {"stance_com": [round(cx, 2), round(cy, 2), round(ch, 2)], "total_g": round(m.total_g, 1),
                 "walks": {}, "views": {}}
    for name in pat.PATTERN_NAMES:
        p = Planner(m, name, "forward", None, tick=1000)
        worst, pitch, roll = None, 0.0, 0.0
        for _ in range(600):
            t = p.next()
            if bin(t.support).count("1") >= 3:
                worst = t.margin if worst is None else min(worst, t.margin)
            pitch, roll = max(pitch, abs(t.plan_pitch)), max(roll, abs(t.plan_roll))
        out["walks"][name] = {"static": bool(pat.pattern(name).lean),
                              "min_margin_mm": None if worst is None else round(worst, 2),
                              "max_pitch_deg": round(pitch, 2), "max_roll_deg": round(roll, 2)}
    for name in VIEWS:
        out["views"][name] = [[k, v, b] for k, v, b in m.disagreements(name)]
    return out


def _run(tree: Path, args: list[str], cwd: Path | None = None, stdout: Path | None = None,
         timeout: float = 3600) -> subprocess.CompletedProcess:
    env = dict(os.environ, PYTHONPATH=str(tree / "mind"))
    r = subprocess.run([sys.executable, *args], cwd=str(cwd or tree), env=env, capture_output=True, text=True,
                       timeout=timeout)
    if stdout is not None and r.returncode == 0:
        stdout.write_text(r.stdout)
    return r


def _metrics_of(tree: Path) -> dict:
    r = _run(tree, ["-c", "import json; from kilo.cli_model import metrics; print(json.dumps(metrics()))"],
             cwd=tree / "mind")
    if r.returncode != 0:
        raise RuntimeError("the planner failed on this model:\n" + r.stderr[-2000:])
    return json.loads(r.stdout)


VECTORS = ("planner", "measure", "behaviour", "perception", "intent")
# The intent layer's moves and the righting experiment's, planned and checked on the model (committed JSON that
# kernel/mind/intent/gen_intent.py reads): re-planned before the intent tables are generated.
INTENT_PLANS = ("kilo.intent.moves", "kilo.intent.righting")


def regenerate(tree: Path, rear_up: bool, say=print) -> tuple[list[str], str]:
    """Everything made from the spec, rebuilt in ``tree``. Returns (what failed, the rear-up's verdict)."""
    failed: list[str] = []

    def step(what: str, r: subprocess.CompletedProcess) -> bool:
        if r.returncode != 0:
            failed.append(f"{what}: " + (r.stderr or r.stdout).strip().splitlines()[-1][:300])
        return r.returncode == 0

    say("  the bindings (protocol/gen.py) and their copies (protocol/vendor.py)")
    if not step("protocol/gen.py", _run(tree, ["protocol/gen.py"])):
        return failed, "not run"
    step("protocol/vendor.py", _run(tree, ["protocol/vendor.py"]))
    verdict = "not run (--no-rear-up)"
    if rear_up:
        say("  the rear-up's keyframes and its checks (several minutes)")
        r = _run(tree, ["-m", "kilo.tricks.rear_up"], cwd=tree / "mind")
        last = [ln for ln in r.stdout.strip().splitlines() if "runs," in ln and "problems" in ln]
        verdict = ("passes" if r.returncode == 0 else "FAILS") + (": " + "; ".join(last) if last else "")
        if r.returncode != 0 and not last:
            failed.append("kilo.tricks.rear_up: " + (r.stderr or r.stdout).strip().splitlines()[-1][:300])
    say("  the mind's trick table (kernel/mind/tricks/gen_library.py)")
    step("gen_library.py", _run(tree, ["kernel/mind/tricks/gen_library.py"]))
    for mod in INTENT_PLANS:
        say(f"  the moves {mod} plans and checks on the model (a minute or two)")
        step(mod, _run(tree, ["-m", mod, "--write"], cwd=tree / "mind"))
    say("  the state estimate's table and the intents (kernel/mind/intent/gen_intent.py)")
    step("gen_intent.py", _run(tree, ["kernel/mind/intent/gen_intent.py"]))
    for name in VECTORS:
        gen = tree / "kernel" / "host" / "tests" / f"gen_{name}_vectors.py"
        if gen.exists():
            say(f"  the {name} golden vectors")
            step(gen.name, _run(tree, [str(gen)], stdout=gen.with_name(f"{name}_vectors.txt")))
    return failed, verdict


def _copy_tree(src: Path, dst: Path) -> None:
    """The checkout's files (tracked, and untracked ones git does not ignore) into dst."""
    try:
        names = subprocess.run(["git", "-C", str(src), "ls-files", "-co", "--exclude-standard", "-z"], check=True,
                               capture_output=True, timeout=60).stdout.decode().split("\0")
    except (OSError, subprocess.SubprocessError):
        names = []
    if names:
        for n in names:
            f = src / n
            if n and f.is_file():
                (dst / n).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(f, dst / n)
        return
    shutil.copytree(src, dst, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns(".git", ".venv", "build", "__pycache__", ".pytest_cache", "circle"))


def apply(fit_path: str | Path, repo: Path = REPO, dry_run: bool = False, rear_up: bool = True, say=print) -> int:
    fit = load_fit(fit_path)
    spec0 = (repo / SPEC).read_text()
    changes = plan_changes(fit, spec0)
    say(f"What the measurement ({', '.join(Path(p).name for p in fit.get('inputs', [])) or Path(fit_path).name}) "
        "changes in the model:")
    for c in changes:
        say("  " + c.line())
    todo = [c for c in changes if c.applied]
    if not todo:
        say("Nothing to apply: the model stays as it is.")
        return EXIT_OK
    if any(c.fit in ("length", "track") for c in todo):
        say("  NOTE: LEG_LB_MM / LEG_W_MM are also the body's parameters: the body firmware changes with them "
            "(flash it), and a body that saved its own values keeps those until they are set.")
    source = f"self-measurement {Path(fit_path).name}, applied {datetime.date.today().isoformat()}"
    before = _metrics_of(repo)
    tmp = None
    tree = repo
    try:
        if dry_run:
            tmp = tempfile.mkdtemp(prefix="kilo-model-")
            tree = Path(tmp)
            _copy_tree(repo, tree)
        (tree / SPEC).write_text(edit_spec(spec0, changes, source))
        say("Regenerating what is made from the model" + (" (in a temporary copy: --dry-run)" if dry_run else "")
            + ":")
        failed, verdict = regenerate(tree, rear_up, say)
        after = _metrics_of(tree)
        changed = _changed_files(tree) if not dry_run else []
    finally:
        if tmp is not None:
            shutil.rmtree(tmp, ignore_errors=True)
    say("")
    say(report(before, after, changes, verdict))
    for f in failed:
        say(f"FAILED: {f}")
    if dry_run:
        say("Dry run: nothing was changed. Without --dry-run the same is written into this checkout.")
    else:
        say("Written (not committed): " + (", ".join(changed) if changed else "protocol/spec.toml and what is "
                                                                             "made from it"))
        say("Next: git diff; the suites (protocol/test.sh, kernel/host/test.sh, body/tools/host_tests.sh, "
            "body/tools/size_check.sh, pytest in mind/); then decide. Nothing has gone to Kilo.")
    return EXIT_ERR if failed else EXIT_OK


def _changed_files(tree: Path) -> list[str]:
    try:
        r = subprocess.run(["git", "-C", str(tree), "status", "--short"], capture_output=True, text=True, timeout=60)
        return [ln[3:] for ln in r.stdout.splitlines() if ln.strip()]
    except (OSError, subprocess.SubprocessError):
        return []


def report(before: dict, after: dict, changes: list[Change], rear_up: str) -> str:
    L = ["What changes for Kilo:"]
    b, a = before["stance_com"], after["stance_com"]
    L.append(f"  centre of mass, standing: {b[0]:.1f} -> {a[0]:.1f} mm ahead of the shoulders' middle, "
             f"{b[1]:+.1f} -> {a[1]:+.1f} mm to the left, {b[2]:.1f} -> {a[2]:.1f} mm above the floor")
    L.append("  each walk: the least his centre of mass stays inside his feet with three or more down "
             "(the planner asks for 14 mm), and the most it leans him")
    for name, w in after["walks"].items():
        w0 = before["walks"].get(name, {})

        def f(v):
            return "-" if v is None else f"{v:.1f}"
        kind = "" if w.get("static") else "  (a rocking gait: it does not keep him inside by design)"
        L.append(f"    {name:<9} margin {f(w0.get('min_margin_mm'))} -> {f(w['min_margin_mm'])} mm; "
                 f"pitch {f(w0.get('max_pitch_deg'))} -> {f(w['max_pitch_deg'])} deg, "
                 f"roll {f(w0.get('max_roll_deg'))} -> {f(w['max_roll_deg'])} deg" + kind)
    L.append(f"  the rear-up trick's check: {rear_up}")
    sd = {c.fit: c.sd for c in changes if not c.why_not}
    L.append("  numbers that disagree (the views), against the model now:")
    for name, rows in after["views"].items():
        close, far = [], []
        for k, v, base in rows:
            if base is None:
                continue
            s = sd.get(VIEW_FIT.get(k, ""))
            (close if s is not None and abs(v - base) <= max(2 * s, 1e-9) else far).append(
                f"{k} {v:g} vs {base:.2f}")
        if far:
            L.append(f"    {name}: still off: " + "; ".join(far))
        if close:
            L.append(f"    {name}: now within the measurement's error (2 +-): " + "; ".join(close)
                     + ("  -> this view can be deleted" if not far else ""))
        if not far and not close:
            L.append(f"    {name}: nothing to compare (its numbers are its own)")
    return "\n".join(L)


def local(args: argparse.Namespace) -> int:
    try:
        if args.model_cmd == "show":
            print(show(load_fit(args.fit) if args.fit else None))
            return EXIT_OK
        return apply(args.fit, repo=args.repo or REPO, dry_run=args.dry_run, rear_up=not args.no_rear_up)
    except (OSError, ValueError, RuntimeError) as e:
        print(f"kilo model: {e}", file=sys.stderr)
        return EXIT_ERR
