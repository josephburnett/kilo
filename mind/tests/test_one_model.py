"""One model (docs/architecture.md "One model"): every number that describes
Kilo's body is in protocol/spec.toml and reaches code through the generated
bindings and kilo.planner.model.BodyModel.

- no body number typed again in the code that used to have its own;
- the body's fixed-point IK (body/src/core/ik.cpp) and BodyModel.ik are the
  same leg;
- the VIEWS (numbers a part of the mind fitted for itself that disagree with
  the model) are exactly the ones listed here, with how far each is from the
  model: adding, changing or removing one is an edit of this file too.
"""

from __future__ import annotations

import math
import re
import shutil
import subprocess
from pathlib import Path

import pytest

from kilo.measure import script as measure_script
from kilo.planner import longtrot as LT
from kilo.planner import model as PM
from kilo.planner.model import BodyModel
from kilo.protocol import kilo_protocol as P
from kilo.tricks import dynamics as D
from kilo.tricks import rear_up as RU
from kilo.tricks import sagittal as S
from kilo.tricks import stance as T

REPO = Path(__file__).resolve().parents[2]
M = BodyModel()


def code(path: str) -> str:
    """A source file without its comments and docstrings' prose is too much to ask of a regex:
    the patterns below are assignments and calls, which prose does not contain."""
    return (REPO / path).read_text()


# ---- (a) no body number typed outside the spec again ----------------------------------------------
# file -> patterns that were there before 2026-11-21 (each a hand-typed copy of a model number)
TYPED_AGAIN = {
    "mind/kilo/tricks/rear_up.py": [r"to_world\(123", r"com_dy=-?\d", r"math\.atan2\(xb, -zb\)"],
    "mind/kilo/tricks/dynamics.py": [r"lag_ms: float = \d", r"tau_ms: float = \d", r"SLEW_DEG_TICK = \d",
                                     r"FITTED = Params\(com_dx=", r"tail_len: float = \d",
                                     r"tail_droop_deg: float = \d", r"belly_z: float = -?\d", r"130 \*\* 2",
                                     r"imu_pitch: float = -?\d", r"^STAND = \{", r"^SIT = \{", r"\b88 if j =="],
    "mind/kilo/tricks/sagittal.py": [r"tail_\w+: float = -?\d", r"(rear_x|belly_z|back_z|pi_rear_x|pi_top_z|nose_x"
                                     r"|nose_z): float = -?\d"],
    "mind/kilo/tricks/stance.py": [r"KILO_CZP = \(", r"half_width: float = \d", r"imu_pitch: float = -?\d"],
    "mind/kilo/planner/longtrot.py": [r"(com_x_mm|extra_ms|clamp_deg|com_h_mm): float = \d", r"hypot\(65, 123\)"],
    "kernel/mind/planner/longtrot.h": [r"kFit\w+ = \(real\)\d", r"kFit\w+ = \d"],
    "kernel/mind/guard/guard.h": [r"kSensorHeightMm = \d", r"kFloorTiltDeg = \d"],
}


@pytest.mark.parametrize("path", sorted(TYPED_AGAIN))
def test_no_body_number_is_typed_again(path):
    text = code(path)
    for pat in TYPED_AGAIN[path]:
        m = re.search(pat, text, re.M)
        assert m is None, f"{path}: {m.group(0)!r}: a body number belongs in protocol/spec.toml [model]"


def test_the_consumers_read_the_model():
    # the tricks' geometry: the model's tail, outline, servo zeros
    g = S.Geometry()
    assert (g.tail_pivot_x, g.tail_pivot_z, g.tail_len, g.tail_droop_deg, g.tail_axis_deg, g.tail_sign, g.tail_g,
            g.tail_com_frac) == (M.tail.pivot_x_mm, M.tail.pivot_z_mm, M.tail.len_mm, M.tail.droop_deg,
                                 M.tail.axis_deg, M.tail.sign, M.tail.g, M.tail.com_frac)
    assert (g.rear_x, g.belly_z, g.back_z, g.pi_rear_x, g.pi_top_z, g.nose_x, g.nose_z) == (
        M.outline.rear_x_mm, M.outline.belly_z_mm, M.outline.back_z_mm, M.outline.pi_rear_x_mm,
        M.outline.pi_top_z_mm, M.outline.nose_x_mm, M.outline.nose_z_mm)
    assert S.Geometry.of(M) == g and T.Geometry3().half_width == M.outline.half_width_mm
    assert T.KILO_CZP == tuple(P.MODEL_SERVO_CZP) == M.czp
    assert D.DESIGN_CZP == M.czp_reseated and D.DESIGN_CZP[10] == P.MODEL_SERVO_RESEATED_RB_SHOULDER_CZP
    assert [j for j, (a, b) in enumerate(zip(D.DESIGN_CZP, T.KILO_CZP)) if a != b] == [10]
    # the dynamics: the model's servo response and slew; its defaults are the model's corrections
    assert (D.NOMINAL.lag_ms, D.NOMINAL.tau_ms, D.SLEW_DEG_TICK) == (M.servo_lag_ms, M.servo_tau_ms,
                                                                     M.slew_deg_per_tick)
    assert (D.NOMINAL.com_dx, D.NOMINAL.com_dy, D.NOMINAL.com_dz, D.NOMINAL.sag) == (
        M.com_dx_mm, M.com_dy_mm, M.com_dz_mm, M.sag_deg_per_kg_cm)
    # ...and FITTED is the "tricks" view, RU.AS_RECORDED the "tricks_rigid" one
    v = PM.view_numbers("tricks")
    f = D.FITTED
    assert (f.com_dx, f.com_dy, f.sag, f.tail_droop_deg, f.imu_pitch, f.imu_roll) == (
        v["com_dx_mm"], v["com_dy_mm"], v["sag_deg_per_kg_cm"], v["tail_droop_deg"], -v["imu_pitch_deg"],
        v["imu_roll_deg"])
    assert f == D.NOMINAL.with_(com_dx=f.com_dx, com_dy=f.com_dy, sag=f.sag, tail_droop_deg=f.tail_droop_deg,
                                imu_roll=f.imu_roll)
    assert RU.AS_RECORDED.com_dy == PM.view_numbers("tricks_rigid")["com_dy_mm"]
    # the long trot's pendulum is the "longtrot" view
    v = PM.view_numbers("longtrot")
    assert (LT.TROT_FIT.com_x_mm, LT.TROT_FIT.com_h_mm, LT.TROT_FIT.extra_ms, LT.TROT_FIT.clamp_deg) == (
        v["com_x_mm"], v["com_h_mm"], v["extra_ms"], v["clamp_deg"])
    # the measurement's home height is the model's STANCE
    assert M.stance_z_mm() == measure_script.HOME_Z


def test_kilo_czp_is_the_body_tests_table():
    src = (REPO / "body" / "test" / "test_kilo_eeprom.cpp").read_text()
    m = re.search(r"CZP\[16\] = \{([\d, ]+)\}", src)
    assert m and tuple(int(x) for x in m.group(1).split(",")) == tuple(P.MODEL_SERVO_CZP)


def test_a_view_replaces_the_models_numbers():
    v = M.view("tricks")
    assert (v.com_dx_mm, v.com_dy_mm, v.sag_deg_per_kg_cm, v.tail.droop_deg, v.imu_roll_deg) == (
        -5.5, -5.0, 4.0, 16.0, 2.0)
    assert v.com([0.0] * 4, [76.0] * 4)[0] == pytest.approx(M.com([0.0] * 4, [76.0] * 4)[0] - 5.5)
    assert D.Params.of(v) == D.FITTED
    assert T.Geometry3.of(v).com_dy == -5.0 and S.Geometry.of(v).com_dx == -5.5
    assert M.view("longtrot") == M  # its numbers are the pendulum's own: nothing of BodyModel changes


# ---- (c) the views, and how far each is from the model --------------------------------------------
# view -> {key: (the view's number, the model's or None where it has no counterpart)}. Removing a
# view from protocol/spec.toml (its user then reads the model) or changing the model fails here
# until this table says the same: one visible edit.
VIEWS = {
    "longtrot": {"com_x_mm": (3.5, 7.43), "com_h_mm": (79.0, 79.24), "extra_ms": (120.0, None),
                 "clamp_deg": (2.3, None)},
    "tricks": {"com_dx_mm": (-5.5, 0.0), "com_dy_mm": (-5.0, 0.0), "sag_deg_per_kg_cm": (4.0, 0.0),
               "tail_droop_deg": (16.0, 18.0), "imu_pitch_deg": (1.9, 0.0), "imu_roll_deg": (2.0, 0.0)},
    "tricks_rigid": {"com_dy_mm": (-2.0, 0.0)},
}


def test_the_views_are_the_listed_ones():
    assert set(PM.VIEWS) == set(VIEWS) == set(P.MODEL_VIEWS)
    for name, want in VIEWS.items():
        got = {k: (v, b) for k, v, b in M.disagreements(name)}
        assert set(got) == set(want), name
        for k, (v, b) in want.items():
            assert got[k][0] == v, (name, k)
            if b is None:
                assert got[k][1] is None, (name, k)
            else:
                assert got[k][1] == pytest.approx(b, abs=0.01), (name, k)


def test_every_model_number_says_where_it_comes_from():
    for key, (what, unit, source) in P.MODEL_ABOUT.items():
        assert what and source, key


# ---- (b) the body's IK is the model's leg ----------------------------------------------------------
IK_TOL_DEG = 0.15     # the body's fixed point: 1/16 deg outputs, Q12 cosines (its own test allows 0.25 against
                      # its double reference, body/test/test_ik.cpp)
EDGE_MM = 0.75        # points this close to either model's reach are left out: the body's cap is 243/256 of
                      # L1 + L2 (90.0 mm), the model's 0.95 (90.25 mm)


@pytest.fixture(scope="module")
def body_ik(tmp_path_factory):
    if shutil.which("g++") is None or shutil.which("gcc") is None:
        pytest.skip("no g++/gcc")
    out = tmp_path_factory.mktemp("ik") / "ik_dump"
    subprocess.run(["bash", str(REPO / "body" / "tools" / "build_link_harness.sh"), str(out), "ik_dump"],
                   check=True, capture_output=True, timeout=600)
    rows = subprocess.run([str(out)], check=True, capture_output=True, text=True, timeout=60).stdout.split("\n")
    return [tuple(int(v) for v in r.split()) for r in rows if r]


def test_the_bodys_ik_is_the_models_leg(body_ik):
    assert len(body_ik) > 10000
    worst, where, n = 0.0, None, 0
    for leg, x, z, ok, sh_a16, kn_a16 in body_ik:
        d = math.hypot(x, z)
        if not (M.min_reach_mm + EDGE_MM < d < 90.0 - EDGE_MM):
            continue
        assert ok == 1, (leg, x, z)
        t1, t2 = M.ik(leg, x, z)
        sigma = 1 if PM.FRONT[leg] else -1
        # core/ik.h: OpenCat shoulder = -theta1, knee = -(theta2 - sigma 90)
        sh, kn = -math.degrees(t1), -(math.degrees(t2) - sigma * 90.0)
        e = max(abs(sh - sh_a16 / 16.0), abs(kn - kn_a16 / 16.0))
        n += 1
        if e > worst:
            worst, where = e, (leg, x, z, sh, sh_a16 / 16.0, kn, kn_a16 / 16.0)
    assert n > 5000
    assert worst <= IK_TOL_DEG, (worst, where)


def test_the_bodys_reach_cap_is_the_models_within_a_quarter_mm():
    full = M.l1_mm + M.l2_mm
    assert abs(P.MODEL_REACH_FRAC * full - (int(full * 4) * 243 // 256) / 4.0) <= 0.26


# ---- kilo model ------------------------------------------------------------------------------------
FIT = Path(__file__).parent / "fixtures" / "self_measure_2026_11_21.fit.json"  # the first run on Kilo (format 1)


def test_model_show_says_where_every_number_comes_from_and_what_disagrees():
    from kilo import cli_model

    text = cli_model.show(cli_model.load_fit(FIT))
    for word in ("LEG_LB_MM", "stand_com_x_mm", "the owner's ruler, 2026-11-16", "NUMBERS THAT DISAGREE",
                 "longtrot", "tricks_rigid", "com_x_mm             3.5  against the model's 7.43",
                 "the measurement says 4.35 +- 0.31"):
        assert word in text, word
    assert "42.07" not in text  # the first run's height ran into the fit's limit: it says nothing


def test_model_changes_from_the_first_run_and_from_a_script_2_fit():
    import json
    import tomllib

    from kilo import cli_model as CM

    spec = (REPO / "protocol" / "spec.toml").read_text()
    ch = {c.fit: c for c in CM.plan_changes(CM.load_fit(FIT), spec)}
    assert ch["com_x"].applied and ch["com_x"].new == pytest.approx(M.com_dx_mm - 3.1, abs=0.05)
    assert "plausible" in ch["com_z"].why_not      # -40: the fit's limit (format 1 has no at_limit flag)
    assert "error" in ch["lag_ms"].why_not and "no place" in ch["tip_ms"].why_not
    # the keys script 2's fit adds, and its at_limit flag
    fit = json.loads(FIT.read_text())
    fp = fit["parameters"]
    fp["length"] = {"value": 133.2, "sd": 0.6, "model": 123.0, "fitted": True, "significant": True}
    fp["track"] = {"value": 68.7, "sd": 0.5, "model": 65.0, "fitted": True, "significant": True}
    fp["x_scale"] = {"value": 0.98, "sd": 0.02, "model": 1.0, "fitted": True, "significant": False}
    fp["com_z"] = dict(fp["com_z"], value=-6.0, at_limit=True)
    fp["sag"] = dict(fp["sag"], value=2.6, sd=0.4, significant=True)
    fp["lag_ms"] = dict(fp["lag_ms"], value=61.4, sd=2.0, significant=True)
    changes = CM.plan_changes(fit, spec)
    ch = {c.fit: c for c in changes}
    assert (ch["length"].new, ch["track"].new, ch["lag_ms"].new, ch["sag"].new) == (133, 69, 61, 2.6)
    assert "limit" in ch["com_z"].why_not and "no place" in ch["x_scale"].why_not
    new = tomllib.loads(CM.edit_spec(spec, changes, "self-measurement test, applied today"))
    old = tomllib.loads(spec)
    params = {p["name"]: p["default"] for p in new["params"]}
    assert (params["LEG_LB_MM"], params["LEG_W_MM"]) == (133, 69)
    c = new["model"]["correction"]
    assert (c["com_dx_mm"], c["com_dy_mm"], c["com_dz_mm"], c["sag_deg_per_kg_cm"]) == (
        pytest.approx(M.com_dx_mm - 3.1, abs=0.05), pytest.approx(M.com_dy_mm - 0.7, abs=0.05),
        old["model"]["correction"]["com_dz_mm"], 2.6)
    assert new["model"]["servo_lag_ms"] == 61 and new["model"]["servo_tau_ms"] == old["model"]["servo_tau_ms"]
    assert new["model"]["about"]["correction.com_dx_mm"][2] == "self-measurement test, applied today"
    assert new["model"]["about"]["correction.com_dz_mm"] == old["model"]["about"]["correction.com_dz_mm"]
    assert new["model"]["views"] == old["model"]["views"]  # a view is deleted by hand, never by a fit


@pytest.mark.slow
def test_model_apply_dry_run_changes_nothing_and_reports():
    """The first run's fit through the whole of ``kilo model apply --dry-run`` (the rear-up's
    rebuild left out: minutes): the spec, the bindings and the vectors are rebuilt in a temporary
    copy, the report says what changed for Kilo, and this checkout is untouched."""
    from kilo import cli_model as CM

    watched = [REPO / "protocol" / "spec.toml", REPO / "mind" / "kilo" / "protocol" / "kilo_protocol.py",
               REPO / "kernel" / "host" / "tests" / "planner_vectors.txt"]
    before = [p.read_bytes() for p in watched]
    lines: list[str] = []
    rc = CM.apply(FIT, dry_run=True, rear_up=False, say=lines.append)
    text = "\n".join(lines)
    assert rc == 0, text
    assert [p.read_bytes() for p in watched] == before
    assert "centre of mass, standing: 7.4 -> 4.3 mm ahead" in text
    assert "crawl" in text and "longtrot: still off: com_x_mm 3.5 vs 4.33" in text
    assert "Dry run: nothing was changed" in text and "FAILED" not in text
