"""The state estimate: which resting state Kilo is in, from what the body
reports and the body model. kernel/mind/intent/estimate.cpp is its port
(kernel/host/tests/intent_vectors.txt, made by gen_intent_vectors.py from
this module, checks that the two agree sample for sample).

WHAT IT READS. Every TELEMETRY: pitch and roll (1/16 deg, "a16"; pitch
nose DOWN +, roll RIGHT side up +, with the body's reflection of the pitch
past 90 deg when he is upside down: body/src/core/attitude.h), the three
rates, the body's mode; and what the mind last asked for (``Cmd``: a
posture, or nothing it knows the attitude of).

HOW IT DECIDES, in this order:

1. MOVING while the body is in a moving mode (GAIT, TRICK, BOOT) or has
   not been still (every rate under STILL_DPS) for STILL_US. (FALLEN and
   RECOVERING are not moving modes: while the fall reflex has him he is
   judged by his rates and his attitude like any other time, so a Kilo
   lying still on his side between two tries reads ON_x_SIDE.)
2. From the attitude alone, by where gravity points in his frame:
   ON_END   the nose or the tail down: |pitch|, folded about 90 deg, at
            least END_DEG;
   ON_BACK  |roll| over BACK_DEG;
   ON_LEFT_SIDE / ON_RIGHT_SIDE   |roll| from SIDE_DEG to BACK_DEG, by its
            sign (roll + is the right side up: he lies on his left).
   The body's fall reflex trips at 75 deg (reflex.h FALL_DEG) and calls a
   fall "pitch" within 15 deg of vertical; these classes are wider (60 deg)
   because they name where he has come to rest, not when to react.
3. Otherwise he is upright, and the model says which upright state: the
   table (``table()``) has, for STANDING, SITTING and RESTING, the pitch
   and roll the body model predicts for that posture (the 3-D rest finder
   kilo.tricks.stance.rest on the posture's joints) and a tolerance. A
   state FITS when both errors are inside its tolerance. The posture last
   asked for is a prior, never the answer:
     - it fits: that state;
     - it does not, others do: the nearest of those (he is not where he
       was told to be: half the confidence);
     - nothing was asked and several fit (STANDING and RESTING read alike:
       both are level): the state he was in before if it is one of them,
       else UNKNOWN;
     - nothing fits: UNKNOWN (leaning on something, held at an angle).
   With nothing asked and the body LIMP (the servos off, as at power-on)
   the prior is LIMP: he cannot be standing.
4. A new answer must hold for CONFIRM_US before it replaces the old one.

THE TABLE comes from the model's own numbers (``BodyModel()``: its
centre of mass, legs and IMU offset, the one model the tricks use too).
The tolerance is

    MODEL_ERROR_DEG + 3 x SPREAD_DEG

MODEL_ERROR_DEG = 8: the largest difference between the model's attitude
and the mean of what Kilo's recordings show for a posture he was told to
hold (2026-11-17..21, python -m kilo.intent.replay: REST rolls 9.0 deg in
the model, where the left front knee stops short of its angle, and -1.7 on
Kilo: 7.3; SIT 28.4 nose up against 25.7; STAND 0 against 2.4 nose down,
the IMU's offset). SPREAD_DEG = 1.3: how much one posture's reading varies
from one time to the next in those recordings (the sd of REST's roll, the
largest; the first self-measurement's repeatability within one run, 0.16
deg of pitch and 0.45 of roll, is inside it). When a self-measurement has
put the IMU's offsets and the legs' give into the model, MODEL_ERROR_DEG
is the number to bring down.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from functools import lru_cache

from kilo.protocol import kilo_protocol as P

A16 = 16


class State(IntEnum):
    UNKNOWN = 0
    MOVING = 1
    STANDING = 2
    SITTING = 3
    RESTING = 4
    ON_LEFT_SIDE = 5
    ON_RIGHT_SIDE = 6
    ON_BACK = 7
    ON_END = 8


UPRIGHT = (State.STANDING, State.SITTING, State.RESTING)
FALLEN = (State.ON_LEFT_SIDE, State.ON_RIGHT_SIDE, State.ON_BACK, State.ON_END)


class Cmd(IntEnum):
    """What the mind last asked the body for, as far as the attitude goes."""
    NONE = 0     # nothing with a known resting attitude (a trick's pose, CALIB, nothing yet)
    STAND = 1    # STAND or STANCE (also after a walk: the body ends in STANCE)
    SIT = 2
    REST = 3
    LIMP = 4     # the servos off: he lies as he lay


CMD_STATE = {Cmd.STAND: State.STANDING, Cmd.SIT: State.SITTING, Cmd.REST: State.RESTING, Cmd.LIMP: State.RESTING}

# ---- the constants (generated into kernel/mind/intent/intent.gen.h) ----------------------------
STILL_DPS = 20              # "still": every body rate under this (the body's FALL_STILL_DPS)
STILL_US = 400_000          # ...for this long
CONFIRM_US = 300_000        # a new answer holds this long before it is the state
END_DEG = 60                # ON_END: the long axis within 30 deg of vertical
SIDE_DEG = 60               # ON_x_SIDE: |roll| from here ...
BACK_DEG = 120              # ... to here; past it ON_BACK
MODEL_ERROR_DEG = 8.0       # the model against the recordings' means (the module's docstring)
SPREAD_DEG = 1.3            # one posture's reading from one time to the next (sd, the recordings' largest)
MOVING_MODES = (int(P.Mode.BOOT), int(P.Mode.GAIT), int(P.Mode.TRICK))
LIMP_MODE = int(P.Mode.LIMP)

POSTURE_OF = {State.STANDING: "stand", State.SITTING: "sit", State.RESTING: "rest"}


def _a16(deg: float) -> int:
    v = abs(deg) * A16
    return int(v + 0.5) * (1 if deg >= 0 else -1)


@dataclass(frozen=True)
class Row:
    """One upright state's attitude in the model, as TELEMETRY reports it (a16), and its tolerance."""
    state: State
    pitch: int
    roll: int
    tol_pitch: int
    tol_roll: int
    margin_mm: float = 0.0     # the model's: the centre of mass inside the support (for the docs)
    contacts: tuple = ()


@lru_cache(maxsize=None)
def table() -> tuple:
    """The upright states' rows, from the body model (the module's docstring)."""
    from kilo.calibration import POSTURES
    from kilo.planner.model import BodyModel
    from kilo.tricks import dynamics as D
    from kilo.tricks import stance as T

    body = BodyModel()
    g3 = T.Geometry3.of(body, czp=body.czp_reseated)
    tol_p = tol_r = _a16(MODEL_ERROR_DEG + 3 * SPREAD_DEG)
    rows = []
    for st in UPRIGHT:
        js = dict(zip(D.JOINTS, POSTURES[POSTURE_OF[st]]))
        r = T.rest(g3, js)
        nose_up = T.telemetry_pitch(g3, r)          # nose up +, with the model's IMU pitch offset
        roll = r.roll + body.imu_roll_deg           # left side down + = TELEMETRY's roll
        rows.append(Row(st, _a16(-nose_up), _a16(roll), tol_p, tol_r, round(r.margin, 1), tuple(sorted(r.contacts))))
    return tuple(rows)


# ---- the attitude's class ----------------------------------------------------------------------
def fold(pitch_a16: int) -> int:
    """|pitch| folded about 90 deg (the body reflects it past 90 when he is upside down)."""
    p = abs(pitch_a16)
    return p if p <= 90 * A16 else 180 * A16 - p


def attitude_class(pitch: int, roll: int) -> State | None:
    """The fallen state the attitude alone names, or None: upright."""
    if fold(pitch) >= END_DEG * A16:
        return State.ON_END
    r = abs(roll)
    if r > BACK_DEG * A16:
        return State.ON_BACK
    if r >= SIDE_DEG * A16:
        return State.ON_LEFT_SIDE if roll > 0 else State.ON_RIGHT_SIDE
    return None


@dataclass(frozen=True)
class Answer:
    state: State
    confidence: int       # 0..100
    err_pitch: int        # a16: measured minus the model's for the state (0 where the model has none)
    err_roll: int


def classify(pitch: int, roll: int, cmd: Cmd, prev: State, rows: tuple | None = None) -> Answer:
    """The state a still body's attitude says (steps 2 and 3 of the module's docstring)."""
    fallen = attitude_class(pitch, roll)
    if fallen is not None:
        if fallen == State.ON_END:
            return Answer(fallen, 90, 0, 0)
        if fallen == State.ON_BACK:
            return Answer(fallen, 90, 0, 0)
        want = 90 * A16 if roll > 0 else -90 * A16
        return Answer(fallen, 90, 0, roll - want)
    rows = rows if rows is not None else table()
    fits = []   # (score, row order, row, dp, dr): score = the worse error as a part of its tolerance, x1000
    for i, row in enumerate(rows):
        dp, dr = pitch - row.pitch, roll - row.roll
        if abs(dp) <= row.tol_pitch and abs(dr) <= row.tol_roll:
            score = max(abs(dp) * 1000 // row.tol_pitch, abs(dr) * 1000 // row.tol_roll)
            fits.append((score, i, row, dp, dr))
    if not fits:
        return Answer(State.UNKNOWN, 0, 0, 0)
    asked = CMD_STATE.get(cmd)
    for score, _, row, dp, dr in fits:
        if row.state == asked:
            return Answer(row.state, 100 - score // 20, dp, dr)
    if asked is None:
        if len(fits) == 1:
            score, _, row, dp, dr = fits[0]
            return Answer(row.state, 60 - score // 50, dp, dr)
        for score, _, row, dp, dr in fits:
            if row.state == prev:
                return Answer(row.state, 40, dp, dr)
        return Answer(State.UNKNOWN, 0, 0, 0)
    score, _, row, dp, dr = min(fits, key=lambda f: (f[0], f[1]))
    return Answer(row.state, 50 - score // 40, dp, dr)


# ---- the estimator -------------------------------------------------------------------------------
@dataclass(frozen=True)
class Sample:
    """One TELEMETRY as the estimate reads it, with the mind's clock and what was last asked."""
    t_us: int
    pitch: int
    roll: int
    rate_pitch: int
    rate_roll: int
    rate_yaw: int
    mode: int
    cmd: Cmd = Cmd.NONE


@dataclass
class Estimate:
    state: State = State.UNKNOWN
    confidence: int = 0
    since_us: int = 0
    err_pitch: int = 0
    err_roll: int = 0


class Estimator:
    """feed() every TELEMETRY; True when the state changed."""

    def __init__(self, rows: tuple | None = None):
        self.rows = rows if rows is not None else table()
        self.est = Estimate()
        self.resting = State.UNKNOWN     # the last upright or fallen state held (the prior for "as before")
        self._still_since = None
        self._cand = State.UNKNOWN
        self._cand_since = 0
        self._started = False

    def feed(self, s: Sample) -> bool:
        lim = STILL_DPS * A16
        still = abs(s.rate_pitch) < lim and abs(s.rate_roll) < lim and abs(s.rate_yaw) < lim
        if not still:
            self._still_since = None
        elif self._still_since is None:
            self._still_since = s.t_us
        settled = self._still_since is not None and s.t_us - self._still_since >= STILL_US
        if s.mode in MOVING_MODES or not settled:
            ans = Answer(State.MOVING, 0, 0, 0)
        else:
            cmd = Cmd.LIMP if s.cmd == Cmd.NONE and s.mode == LIMP_MODE else s.cmd
            ans = classify(s.pitch, s.roll, cmd, self.resting, self.rows)
        if not self._started:
            self._started = True
            self.est = Estimate(State.UNKNOWN, 0, s.t_us, 0, 0)
            self._cand, self._cand_since = ans.state, s.t_us
        if ans.state != self._cand:
            self._cand, self._cand_since = ans.state, s.t_us
        if ans.state == self.est.state:
            self.est.confidence, self.est.err_pitch, self.est.err_roll = ans.confidence, ans.err_pitch, ans.err_roll
            return False
        if s.t_us - self._cand_since < CONFIRM_US:
            return False
        self.est = Estimate(ans.state, ans.confidence, s.t_us, ans.err_pitch, ans.err_roll)
        if ans.state not in (State.MOVING, State.UNKNOWN):
            self.resting = ans.state
        return True


def cmd_of_posture(posture: int) -> Cmd:
    """The prior a SET_POSTURE code gives."""
    return {int(P.Posture.STAND): Cmd.STAND, int(P.Posture.STANCE): Cmd.STAND, int(P.Posture.SIT): Cmd.SIT,
            int(P.Posture.REST): Cmd.REST, int(P.Posture.LIMP): Cmd.LIMP}.get(int(posture), Cmd.NONE)
