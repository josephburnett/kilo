"""The virtual body's physics while it walks (kilo.vbody.sim plays FEET).

A rigid body on four point feet over flat ground, with the masses and
geometry of kilo.planner.model.BodyModel, at 2 ms steps:

- **Servo lag.** The feet follow the slew-limited commands ``lag_ms``
  later (a dead time) through a first-order response of time constant
  ``tau_ms`` (both from the body model unless given; fitted to the floor
  recordings of 2026-11-15: 50 ms + 70 ms; either may differ from the model
  the planner uses: the error the mind then sees is that difference).
- **Standing.** The feet within ``contact_mm`` of the lowest one carry the
  body. On three or more of them with the centre of mass's projection
  inside their polygon, the body sits on the plane through them: the
  attitude follows the legs' lengths kinematically (a 20 ms settling).
- **Tipping.** On fewer than three, or with the centre of mass outside
  their polygon, the body is an inverted pendulum about the nearest edge of
  that polygon (the line of two feet, or a single foot): it turns toward
  the side the mass is on with an angular acceleration g d / l^2 (d the
  mass's horizontal distance past the edge, l its distance from the edge),
  until a foot on that side reaches the ground and the mass is supported
  again, which stops it (the landing absorbs the motion). From a few
  degrees past the edge at 80 mm this goes 30 degrees in about 0.2 s, as
  the floor traces of 2026-10-14 show.
- **Fallen.** Past ``fall_deg`` of pitch or roll the body reports a fall.
- **Momentum** (``inertia``, off by default; on for the stopping study,
  mind/tests/stopsim.py). The body's speed over the ground is the speed
  at which its feet on the ground move back under it; when it changes, the
  body's inertia acts at the centre of mass like a shift of it by
  ``-a h / g`` (a the body's acceleration, h the centre of mass's height):
  a body that brakes leans its weight toward its front feet. The
  acceleration is filtered over ``inertia_ms`` (the frame and the feet are
  not rigid). Without it the model is quasi-static: the speed itself never
  matters, only where the feet are.

- **Each servo** (the model's [model.correction] servo_lag_d_ms,
  servo_tau_d_ms): a leg's foot follows with its own dead time and time
  constant, the mean of its shoulder's and its knee's (the attitude cannot
  tell the two joints of a leg apart). All alike: one lag, as before.
- **Legs that give** (``legs_give``, off by default: the walks' virtual body
  is rigid; kilo.measure.score's judge turns it on, the model's
  sag_deg_per_kg_cm and servo_give). Each foot on the ground sinks F / k under its load F, with
  k from the servos' give through the leg's levers as in
  kilo.measure.statics (c = sag (g_shoulder x^2 + g_knee (L2 sin(t1+t2))^2)).
  The loads come from the statics of the feet on the ground (a spring solve
  on four, determinate on three, the lever rule on two); the sink follows
  them over ``settle_s``. The body then stands on the sunk feet.

Yaw and sliding are not modelled; the ground is flat and level.
"""

from __future__ import annotations

import math
from collections import deque

from kilo.planner.model import LEGS, BodyModel, G_MM_S2, _solve3, hull, margin, rotation
from kilo.protocol import kilo_protocol as P

SUBSTEPS = 10
SAG_UNIT = math.pi / 180.0 / 1.0e4  # 1 deg per kg cm in radians per g mm (kilo.measure.statics)
# each leg's servos by legacy joint id (LF RF RB LB): its shoulder and its knee
SHOULDER = (8, 9, 10, 11)
KNEE = (12, 13, 14, 15)
TICK_S = P.TICK_MS / 1000


class Physics:
    def __init__(self, model: BodyModel | None = None, lag_ms: float | None = None, contact_mm: float = 0.6,
                 fall_deg: float = 35.0, settle_s: float = 0.02, feet=None, inset_mm: float = 5.0,
                 tau_ms: float | None = None, inertia: bool | None = None, inertia_ms: float = 20.0,
                 legs_give: bool = False):
        self.model = model or BodyModel()
        self.inertia = Physics.INERTIA if inertia is None else inertia
        self.inertia_ms = inertia_ms
        self.v_body = 0.0      # mm/s forward over the ground (inertia)
        self.a_body = 0.0      # mm/s^2, filtered
        self.max_a_body = 0.0  # the largest |a_body| seen
        self.dist_mm = 0.0     # how far the body went over the ground (forward +)
        self._shift = 0.0      # the inertia's shift of the centre of mass, mm forward
        self._prev = None      # xs of the last substep
        self._age = [0, 0, 0, 0]  # substeps each foot has been on the ground
        self.lag_ms = self.model.servo_lag_ms if lag_ms is None else float(lag_ms)
        self.tau_ms = self.model.servo_tau_ms if tau_ms is None else float(tau_ms)
        m = self.model
        # each leg's own dead time and time constant (the mean of its two servos' differences)
        self.leg_lag_ms = [self.lag_ms + (m.servo_lag_d_ms[SHOULDER[leg]] + m.servo_lag_d_ms[KNEE[leg]]) / 2
                           for leg in LEGS]
        self.leg_tau_ms = [self.tau_ms + (m.servo_tau_d_ms[SHOULDER[leg]] + m.servo_tau_d_ms[KNEE[leg]]) / 2
                           for leg in LEGS]
        # legs that give (``legs_give``: the model's sag and servo_give): each servo's give in rad per g mm
        # (None: rigid legs, the virtual body's default)
        self.give = None
        if legs_give and m.sag_deg_per_kg_cm > 0:
            self.give = [(m.joint_sag(SHOULDER[leg]) * SAG_UNIT, m.joint_sag(KNEE[leg]) * SAG_UNIT) for leg in LEGS]
        self.sink = [0.0] * 4  # mm each foot on the ground has sunk under its load
        self.contact_mm = contact_mm
        self.fall_deg = fall_deg
        self.settle_s = settle_s
        self.inset_mm = inset_mm
        self.pitch = 0.0       # degrees, TELEMETRY conventions
        self.roll = 0.0
        self.rate = 0.0        # tipping rate, rad/s, about self.axis
        self.axis_n = None     # the tipping direction (horizontal unit vector), None when standing
        self.fallen = False
        self.contacts = 0      # bits: feet on the ground
        self.margin = 0.0      # mm of the centre of mass inside the contacts' polygon
        self.max_tilt = 0.0
        x0 = [0.0] * 4 if feet is None else list(feet[0])
        z0 = [self.model.stance_height(P.GAIT_STANCE_H_256)] * 4 if feet is None else list(feet[1])
        n = int(round(max(self.leg_lag_ms) / (TICK_S * 1000))) + 2
        self._cmd: deque = deque([(tuple(x0), tuple(z0))] * n, maxlen=n)
        self.xs, self.zs = list(x0), list(z0)
        self._resp = (list(x0), list(z0))  # the first-order response's state (the delayed commands filtered)
        self.rates = (0.0, 0.0)  # pitch, roll rate, deg/s (last tick)

    INERTIA = False  # the default for new Physics (the stopping study turns it on)

    def command(self, xs, zs) -> None:
        """This tick's commanded feet (after the body's slew limit and its corrections)."""
        self._cmd.append((tuple(xs), tuple(zs)))

    def _feet_at(self, frac: float):
        """The actual feet ``frac`` of a tick into the current tick: each leg's commands its lag ago."""
        n = len(self._cmd) - 1
        xs, zs = [0.0] * 4, [0.0] * 4
        for k in LEGS:
            lag_ticks = max(0.0, self.leg_lag_ms[k] / (TICK_S * 1000) - frac)  # no servo answers early
            i = int(math.floor(lag_ticks))
            f = lag_ticks - i
            a = self._cmd[max(0, n - i)]
            b = self._cmd[max(0, n - i - 1)]
            xs[k] = a[0][k] + (b[0][k] - a[0][k]) * f
            zs[k] = a[1][k] + (b[1][k] - a[1][k]) * f
        return xs, zs

    def _alphas(self, dt: float, tau=None) -> list:
        """Each leg's first-order step for a substep of ``dt`` s (``tau``: one time constant for all)."""
        out = []
        for leg in LEGS:
            t = self.leg_tau_ms[leg] if tau is None else tau
            out.append(1.0 - math.exp(-dt * 1000.0 / t) if t > 0 else 1.0)
        return out

    def _legs_now(self, yx, yz, dt: float) -> None:
        """The legs this substep: the servos' response, each leg as much longer as the model's leg_dz_mm
        says, less each foot's sink under its load (legs that give)."""
        self.xs = list(yx)
        dz = self.model.leg_dz_mm
        if self.give is None:
            self.zs = [yz[leg] + dz[leg] for leg in LEGS] if any(dz) else list(yz)
            return
        zl = [yz[leg] + dz[leg] for leg in LEGS]
        target = self._loads_sink(yx, zl)
        a = min(1.0, dt / self.settle_s)
        for leg in LEGS:
            self.sink[leg] += (target[leg] - self.sink[leg]) * a
        self.zs = [zl[leg] - self.sink[leg] for leg in LEGS]

    def _loads_sink(self, xs, zs) -> list:
        """How far each foot on the ground (self.contacts, the last substep's) sinks under its share of
        the weight: the statics of the feet down (body frame, the tilt's effect on the levers left out)."""
        m = self.model
        down = [leg for leg in LEGS if self.contacts >> leg & 1] or list(LEGS)
        cx, cy, _ = m.com(xs, zs)
        W = m.total_g
        X, Y, c = [0.0] * 4, [0.0] * 4, [0.0] * 4
        for leg in LEGS:
            hx, hy = m.hip(leg)
            X[leg], Y[leg] = hx + xs[leg], hy
            t1, t2 = m.ik(leg, xs[leg], zs[leg])
            gs, gk = self.give[leg]
            c[leg] = max(gs * xs[leg] ** 2 + gk * (m.l2_mm * math.sin(t1 + t2)) ** 2, 1e-6)  # mm per g
        F = [0.0] * 4
        if len(down) >= 3:
            # springs on a plane: F_i = (L_i - H + a X_i + b Y_i) / c_i carrying W at (cx, cy)
            kk = [1.0 / c[leg] if leg in down else 0.0 for leg in LEGS]
            L = [zs[leg] for leg in LEGS]
            rows = ((1.0,) * 4, X, Y)
            coef = ([-k for k in kk], [k * x for k, x in zip(kk, X)], [k * y for k, y in zip(kk, Y)])
            M = tuple(tuple(sum(rows[r][i] * coef[q][i] for i in LEGS) for q in range(3)) for r in range(3))
            v = [-sum(rows[r][i] * kk[i] * L[i] for i in LEGS) for r in range(3)]
            v[0] += W
            v[1] += W * cx
            v[2] += W * cy
            sol = _solve3(M, v)
            if sol is not None:
                H, a, b = sol
                F = [max(0.0, (L[i] - H + a * X[i] + b * Y[i]) * kk[i]) for i in LEGS]
        elif len(down) == 2:
            i, j = down
            ex, ey = X[j] - X[i], Y[j] - Y[i]
            L2 = ex * ex + ey * ey or 1.0
            t = max(0.0, min(1.0, ((cx - X[i]) * ex + (cy - Y[i]) * ey) / L2))
            F[i], F[j] = W * (1 - t), W * t
        else:
            F[down[0]] = W
        return [F[leg] * c[leg] for leg in LEGS]

    def step(self) -> None:
        """One body tick (20 ms) in SUBSTEPS steps."""
        p0, r0 = self.pitch, self.roll
        dt = TICK_S / SUBSTEPS
        al = self._alphas(dt)
        for k in range(SUBSTEPS):
            ux, uz = self._feet_at((k + 1) / SUBSTEPS)
            yx, yz = self._resp
            for leg in LEGS:
                a = al[leg]
                yx[leg] += (ux[leg] - yx[leg]) * a
                yz[leg] += (uz[leg] - yz[leg]) * a
            self._legs_now(yx, yz, dt)
            self._motion(dt)
            self._substep(dt)
            if self.fallen:
                break
        self.rates = ((self.pitch - p0) / TICK_S, (self.roll - r0) / TICK_S)
        self.max_tilt = max(self.max_tilt, abs(self.pitch), abs(self.roll))

    def _motion(self, dt: float) -> None:
        """The body's speed over the ground (its feet on the ground move back
        under it), its acceleration, and the inertia's shift of the mass."""
        prev, self._prev = self._prev, list(self.xs)
        for leg in LEGS:  # substeps on the ground (self.contacts: the last substep's)
            self._age[leg] = self._age[leg] + 1 if self.contacts & (1 << leg) else 0
        if prev is None:
            return
        # the feet down for a tick and more carry the body: a foot that just landed (at its
        # swing's end it moves with the body, not yet with the ground) or is about to lift is not the ground's
        down = [leg for leg in LEGS if self._age[leg] >= SUBSTEPS]
        if not down:
            return
        v = -sum(self.xs[leg] - prev[leg] for leg in down) / len(down) / dt
        self.dist_mm += v * dt
        a = (v - self.v_body) / dt
        self.v_body = v
        k = min(1.0, dt * 1000.0 / self.inertia_ms) if self.inertia_ms > 0 else 1.0
        self.a_body += (a - self.a_body) * k
        self.max_a_body = max(self.max_a_body, abs(self.a_body))
        if self.inertia:
            h = self.model.com(self.xs, self.zs)[2] + max(self.zs) + self.model.foot_r_mm
            self._shift = -self.a_body * h / G_MM_S2

    def _world(self, pitch: float, roll: float):
        m = self.model
        R = rotation(pitch, roll)
        cx, cy, cz = m.com(self.xs, self.zs)
        cx += self._shift  # the inertia (_motion): 0 without it
        com = _rot(R, cx, cy, cz)
        feet = []
        for leg in LEGS:
            hx, hy = m.hip(leg)
            feet.append(_rot(R, hx + self.xs[leg], hy, -(self.zs[leg] + m.foot_r_mm)))
        return com, feet

    def _substep(self, dt: float) -> None:
        com, feet = self._world(self.pitch, self.roll)
        low = min(f[2] for f in feet)
        contacts = [leg for leg in LEGS if feet[leg][2] - low < self.contact_mm]
        if len(contacts) >= 3:
            # standing on the plane through them, if the mass is inside
            fit = self._plane(contacts)
            if fit is not None:
                fp, fr = fit
                com2, feet2 = self._world(fp, fr)
                low2 = min(feet2[leg][2] for leg in contacts)
                below = [leg for leg in LEGS if leg not in contacts and feet2[leg][2] < low2 - self.contact_mm]
                if not below:
                    poly = hull([(feet2[leg][0], feet2[leg][1]) for leg in contacts])
                    mg = margin((com2[0], com2[1]), poly) - self.inset_mm
                    if mg >= 0:
                        a = min(1.0, dt / self.settle_s)
                        self.pitch += (fp - self.pitch) * a
                        self.roll += (fr - self.roll) * a
                        self.rate = 0.0
                        self.axis_n = None
                        self._set_contacts(contacts, mg)
                        return
        # tipping about the nearest edge of the contacts toward the mass
        pts = [(feet[leg][0], feet[leg][1]) for leg in contacts]
        poly = hull(pts)
        c = (com[0], com[1])
        mg = margin(c, poly)
        ex, ey, d = _edge_out(c, poly)
        if len(poly) >= 3:
            mg -= self.inset_mm
            d += self.inset_mm
        if d <= 0.0:
            # on its feet but the plane fit was refused (a foot below): settle as it is
            self.rate = 0.0
            self.axis_n = None
            self._set_contacts(contacts, mg)
            return
        height = com[2] - low
        l2 = height * height + d * d
        if self.axis_n is None or ex * self.axis_n[0] + ey * self.axis_n[1] < 0.5:
            self.rate = 0.0 if self.axis_n is None else self.rate * max(0.0, ex * self.axis_n[0]
                                                                       + ey * self.axis_n[1])
        self.axis_n = (ex, ey)
        self.rate += G_MM_S2 * d / l2 * dt
        delta = math.degrees(self.rate * dt)
        # tilting toward n lowers the side n points to: pitch + n_x, roll + n_y (model docstring)
        self.pitch += delta * ex
        self.roll += delta * ey
        self._set_contacts(contacts, mg)
        if abs(self.pitch) > self.fall_deg or abs(self.roll) > self.fall_deg:
            self.fallen = True

    def _set_contacts(self, contacts, mg: float) -> None:
        b = 0
        for leg in contacts:
            b |= 1 << leg
        self.contacts = b
        self.margin = mg

    def _plane(self, contacts):
        """The attitude with these feet on one plane (least squares on four)."""
        from kilo.planner.model import fit_attitude

        b = 0
        for leg in contacts:
            b |= 1 << leg
        return fit_attitude(self.model, self.xs, self.zs, b, (self.pitch, self.roll), reg=1e-6)


def _rot(R, x, y, z):
    return (R[0][0] * x + R[0][1] * y + R[0][2] * z,
            R[1][0] * x + R[1][1] * y + R[1][2] * z,
            R[2][0] * x + R[2][1] * y + R[2][2] * z)


def _edge_out(p, poly):
    """The outward direction (unit, horizontal) from the support toward the
    point p and p's distance past it: for a polygon the edge it is beyond
    (0 distance inside), for a segment or a point the perpendicular."""
    n = len(poly)
    if n == 1:
        dx, dy = p[0] - poly[0][0], p[1] - poly[0][1]
        d = math.hypot(dx, dy)
        return (dx / d, dy / d, d) if d > 1e-9 else (1.0, 0.0, 0.0)
    if n == 2:
        a, b = poly
        ex, ey = b[0] - a[0], b[1] - a[1]
        L2 = ex * ex + ey * ey
        t = max(0.0, min(1.0, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / L2))
        qx, qy = a[0] + t * ex, a[1] + t * ey
        dx, dy = p[0] - qx, p[1] - qy
        d = math.hypot(dx, dy)
        if d < 1e-6:  # balanced on the line: the least push decides (here: none)
            return (-ey / math.sqrt(L2), ex / math.sqrt(L2), 0.0)
        return dx / d, dy / d, d
    best = None
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        ex, ey = b[0] - a[0], b[1] - a[1]
        L = math.hypot(ex, ey)
        out = (ey / L, -ex / L)  # right of a->b is outside for a CCW polygon
        d = (p[0] - a[0]) * out[0] + (p[1] - a[1]) * out[1]
        if best is None or d > best[2]:
            best = (out[0], out[1], d)
    return best
