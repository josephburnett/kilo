"""The random mode's tricks (kilo.tricks.alive_set): the original Nybble's skills from SIT and from
STAND and back, held to the one body model, with the original's timing; and the personality's list.

- the library files are what alive_set writes, each in the library (kilo.tricks.library.AUTHORED);
- every variant starts from its base and ends exactly on it (the end posture the body takes then
  is no move), within the joint limits, with no IMU trigger and no loop left;
- its rows are the legacy rows in the order the original plays them, each timed as the original
  would from where the one before left the legs (kilo.tricks.legacy.legacy_timing), to the tick;
- the rigid quasi-static model (kilo.tricks.stance, alive_set.check) on Kilo as designed and as
  recorded: only feet, tail, bottom and knees on the floor, the centre of mass over the support, no
  topple over DROP_MAX_MM; the tricks left out (alive_set.DROPPED) fail it, or are not cat moves;
- the head at least HEAD_CLEAR_MM from the front legs, either way the pan turns (the original paw
  lick fails that: it bumped his head on Kilo); the stretch's shake is small and in the hold;
- personality.yaml's alive.tricks are the set's, and the personality refuses others.
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from kilo.behaviour import personality as pers
from kilo.tricks import alive_set as A
from kilo.tricks import legacy, library
from kilo.tricks import rear_up as RU
from kilo.tricks.model import CANONICAL, JOINT_IDS, JOINT_NAMES, TICK_MS, Keyframe, Posture

GEOMETRIES = {"as designed": RU.DESIGN, "as recorded": RU.AS_RECORDED}
CASES = A.cases()  # the standing face wash is the engine's: test_engine_tricks.py


def test_library_files_are_current():
    assert A.main(["--check"]) == 0, "run python -m kilo.tricks.alive_set (from mind/)"
    assert [n for n in library.AUTHORED if n.startswith("alive_")] == A.names()
    assert all(library.AUTHORED[n] == "kilo.tricks.alive_set" for n in A.names())
    for n, t in A.build_all().items():
        assert library.load(n) == t


@pytest.mark.parametrize("trick,base", CASES)
def test_from_the_base_and_back_to_it(trick, base):
    t = library.load(A.name_of(trick, base))
    t.validate()
    home = A.base_angles(base)
    assert t.frames[-1].angles == home and t.end_posture == {"sit": Posture.SIT, "stand": Posture.STAND}[base]
    assert t.repeats == 0 and not t.balance and not any(f.trigger for f in t.frames)
    assert all(f.angles[j] is not None for f in t.frames for j in CANONICAL)  # every joint, every frame
    assert t.frames[-1].dur >= A.POSTURE_DUR_TICKS  # back no quicker than the body's own posture change
    assert t.stop_ticks == A.POSTURE_DUR_TICKS  # and as slowly when stopped half way
    if (trick, base) in A.VIA:
        assert t.frames[0].angles == A.base_angles(A.VIA[(trick, base)]) and t.frames[0].dur >= A.POSTURE_DUR_TICKS


@pytest.mark.parametrize("trick,base", CASES)
def test_timed_as_the_original_plays_it(trick, base):
    """Each legacy row as long as the original's transform() and delay() from where it starts."""
    skill = legacy.parse_file()[library.FIRST_VERSION[trick]]
    t = library.load(A.name_of(trick, base))
    via = A.VIA.get((trick, base))
    start = skill_start = legacy.parse_file()[{"sit": "sit", "stand": "up"}[via or base]].angles(0)
    core = t.frames[(1 if via else 0):-1]
    if skill.kind == "posture":
        f, *shake = core
        assert f.dur == legacy.speed_to_ticks(legacy._max_diff(start, skill.angles(0)), A.POSTURE_SPEED)
        # held as long as the original, the shake (if any) the end of the hold, back where it started
        assert f.hold + sum(s.dur + s.hold for s in shake) == A.POSTURE_HOLD_TICKS
        if trick in A.SHAKES:
            assert len(shake) == 2 * A.SHAKES[trick].cycles + 1 and shake[-1].angles == f.angles
        else:
            assert shake == []
        return
    timing = legacy.legacy_timing(skill, skill_start)
    assert len(core) == len(timing)
    order = legacy.play_order(skill)
    for f, (row, move_ms, delay_ms) in zip(core, timing):
        assert abs(f.dur * TICK_MS - move_ms) <= TICK_MS / 2 or f.dur == 1, (row, f.dur, move_ms)
        assert abs(f.hold * TICK_MS - delay_ms) <= TICK_MS / 2, (row, f.hold, delay_ms)
    for f, row in zip(core, order):  # the original's rows, the head changed only where HEAD_ROWS says
        want = A._clamped(skill.angles(row))
        head = A.HEAD_ROWS.get(trick, {}).get(row)
        if head is not None:
            want = A._with_head(Keyframe(want), head).angles
        assert all(f.angles[j] == want[j] for j in CANONICAL), (row, f.joints)


@pytest.mark.parametrize("base", A.BASES)
def test_the_stretch_shakes_its_rear_gently(base):
    """Both hind feet forward and back together, a few mm, at the servos' pace; the rest held."""
    sh = A.SHAKES["stretch"]
    assert 2.0 <= sh.amp_mm <= 6.0 and sh.half_ticks >= 4 and 2 <= sh.cycles <= 4 and sh.tail_deg <= 30
    t = library.load(A.name_of("stretch", base))
    hold = next(f for f in t.frames if f.hold)  # the stretch itself
    shake = t.frames[t.frames.index(hold) + 1:-1]
    g = A.S.Geometry()
    def foot(a, leg):
        return A.S.leg_points(g, False, a[JOINT_IDS[f"{leg}_shoulder"]], a[JOINT_IDS[f"{leg}_knee"]])[2]
    xs = []
    for f in shake:
        for j in CANONICAL:  # only the hind legs and the tail move
            if JOINT_NAMES[j] not in ("lb_shoulder", "rb_shoulder", "lb_knee", "rb_knee", "tail"):
                assert f.angles[j] == hold.angles[j]
        (x0, z0), (x1, z1) = foot(hold.angles, "lb"), foot(f.angles, "lb")
        assert foot(f.angles, "rb") == (x1, z1)  # both hind feet together
        assert abs(z1 - z0) < 1.5  # forward and back, not up and down
        xs.append(x1 - x0)
    swings = xs[:-1]
    assert all(abs(abs(x) - sh.amp_mm) < 1.5 for x in swings) and abs(xs[-1]) < 1e-9
    assert all(a * b < 0 for a, b in zip(swings, swings[1:]))  # forward, back, forward ...


@pytest.mark.parametrize("trick,base", CASES)
def test_the_head_keeps_clear_of_the_front_legs(trick, base):
    """The head never within HEAD_CLEAR_MM of a front leg, whichever way a positive pan turns."""
    c = A.check(library.load(A.name_of(trick, base)), base, RU.DESIGN)
    assert c.head >= A.HEAD_CLEAR_MM, c.head_where


def test_the_original_paw_lick_bumps_the_head():
    """What the owner saw (2026-11-22): the original's head turns into the raised paw."""
    t = A.build("wash_face", "sit")
    orig = replace(t, frames=tuple(replace(f, angles=A._clamped(legacy.parse_file()["wsf"].angles(row)))
                               if row is not None else f for f, row in zip(t.frames, _rows("wash_face"))))
    c = A.check(orig, "sit", RU.DESIGN)
    assert c.head < 3.0 and "the head within" in " ".join(c.problems())  # 2.1 mm on the 2026-11-22 model
    assert A.check(t, "sit", RU.DESIGN).head >= 11.5


def _rows(trick):
    """Each frame's legacy row (None: the move there or back)."""
    order = legacy.play_order(legacy.parse_file()[library.FIRST_VERSION[trick]])
    return order + [None]


@pytest.mark.parametrize("geometry", list(GEOMETRIES))
@pytest.mark.parametrize("trick,base", CASES)
def test_statically_safe_on_the_body_model(trick, base, geometry):
    c = A.check(library.load(A.name_of(trick, base)), base, GEOMETRIES[geometry])
    assert c.problems() == [], c


def test_the_dropped_ones_fail_the_model_or_are_no_cat_moves():
    assert set(A.DROPPED).isdisjoint(A.SET)
    assert {"butt_up", "scratch"} <= set(A.DROPPED)
    # butt_up: the asymmetric front shoulders put him on three feet, rolled
    c = A.check(A.build("butt_up", "stand"), "stand", RU.AS_RECORDED)
    assert c.roll > 15 and "lb_foot" in c.contacts
    rest = A.check(A.build("butt_up", "stand"), "stand", RU.DESIGN)
    assert rest.roll > 15
    # scratch: the hind leg coming up tips him over the sitting moves' bound, as recorded
    assert any("topples" in p for p in A.check(A.build("scratch", "sit"), "sit", RU.AS_RECORDED).problems())


def test_the_personality_names_the_set():
    cfg = pers.load()
    tricks = cfg["alive"]["tricks"]
    assert tricks and set(tricks) <= set(A.SET) and len(set(tricks)) == len(tricks)
    assert cfg["alive"]["trick_every_s"] >= 20 and 0 < cfg["alive"]["trick_speed"] <= 1.0
    for bad in (["scratch"], ["stretch", "stretch"], [], "stretch"):
        with pytest.raises(pers.PersonalityError, match="alive.tricks"):
            pers.from_dict({"alive": {"tricks": bad}})
    with pytest.raises(pers.PersonalityError, match="trick_speed"):
        pers.from_dict({"alive": {"trick_speed": 0.1}})
