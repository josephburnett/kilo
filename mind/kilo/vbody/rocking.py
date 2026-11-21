"""The virtual body's physics for the trot's two-foot phases (docs/walking-and-balance.md §15).

``RockingPhysics`` is kilo.vbody.physics.Physics (the same servo model,
masses, feet, momentum switch and interface) with the two-foot phases
changed where the trot's floor recordings showed Physics wrong (it misses
how the rock changes with the speed):

- **Two feet: an inverted pendulum about their line, and only about it.**
  While the body stands on a diagonal pair, both feet stay on the ground
  (the attitude along their line follows the legs, as on three feet) and
  the body turns freely about the line under gravity, with an angular
  acceleration g u / (h^2 + u^2): u the centre of mass's horizontal
  distance from the line (signed: it can cross the line and come back), h
  its height. The mass's own motion across the line (the body walks
  forward over a line that runs obliquely under it) enters through u,
  recomputed from the feet every 2 ms. Physics takes the feet within
  ``contact_mm`` of the lowest and on a diagonal often stood on one of them
  (a pivot 60 mm from the mass); here a diagonal partner within
  ``pair_mm`` stands too.
- **A third foot that does not hold the mass yet** (the mass less than
  ``inset_mm`` inside the three) leaves the body rocking on its diagonal;
  on three or four feet that hold it the body sits on them (the rotation
  stops, as in Physics).
- **Momentum across the support changes.** From one diagonal straight to
  the other, the part of the rotation about the new line carries over
  (``carry`` x the projection).
- **Two optional fitted terms** (mind/tests/stridesim.py fit):
  ``line_lag_ms``, the support lagging the plan by that much more than the
  swing (the mass sits back over a line further forward by the speed x the
  lag), and ``swing_tau_ms``, a faster response for the legs in the air.
- **Clearance**: each tick, the lowest lifted foot's height above the
  ground (``clear_log``).

The rest is Physics': the servo dead time and first-order response, the
body's slew (in the FEET player), falls, no foot slip, no yaw.
"""

from __future__ import annotations

import math

from kilo.planner.model import LEGS, G_MM_S2, fit_attitude, hull, margin
from kilo.vbody.physics import Physics


class RockingPhysics(Physics):
    def __init__(self, *a, carry: float = 1.0, pair_mm: float = 3.0, line_lag_ms: float = 0.0,
                 swing_tau_ms: float | None = None, **kw):
        super().__init__(*a, **kw)
        self.line_lag_ms = line_lag_ms
        self.swing_tau_ms = swing_tau_ms
        self.carry = carry      # the share of the rotation about the new line kept when the support changes
        self.pair_mm = pair_mm  # a diagonal partner this close to the lowest foot stands on the ground too
        self.pair = None        # the two feet the body rocks on, or None
        self.n = (1.0, 0.0)     # the current line's unit normal (horizontal), rate > 0 tips toward it
        self.min_clear = math.inf  # the lowest swinging foot's height above the ground (mm), this tick
        self.clear_log: list = []  # per tick
        self.rock_rate = 0.0    # rad/s about the current line (signed along n)

    def _motion(self, dt: float) -> None:
        super()._motion(dt)
        inertia = self._shift if self.inertia else 0.0
        # the support lags the plan by line_lag_ms more than the swing (fitted): the mass sits further
        # back over a line that is further forward, by the speed times that lag
        self._shift = inertia - self.v_body * self.line_lag_ms / 1000.0

    def step(self) -> None:
        """One tick: Physics.step with a faster response (swing_tau_ms) for the legs whose delayed
        command is lifted (more than 1 mm shorter than the longest)."""
        self.min_clear = math.inf
        if self.swing_tau_ms is None:
            super().step()
        else:
            self._step_split()
        self.clear_log.append(self.min_clear)

    def _step_split(self) -> None:
        from kilo.vbody.physics import SUBSTEPS, TICK_S
        p0, r0 = self.pitch, self.roll
        dt = TICK_S / SUBSTEPS
        a_s = 1.0 - math.exp(-dt * 1000.0 / self.tau_ms) if self.tau_ms > 0 else 1.0
        a_w = 1.0 - math.exp(-dt * 1000.0 / self.swing_tau_ms) if self.swing_tau_ms > 0 else 1.0
        for k in range(SUBSTEPS):
            ux, uz = self._feet_at((k + 1) / SUBSTEPS)
            top = max(uz)
            yx, yz = self._resp
            for leg in LEGS:
                a = a_w if uz[leg] < top - 1.0 or yz[leg] < top - 1.0 else a_s
                yx[leg] += (ux[leg] - yx[leg]) * a
                yz[leg] += (uz[leg] - yz[leg]) * a
            self.xs, self.zs = list(yx), list(yz)
            self._motion(dt)
            self._substep(dt)
            if self.fallen:
                break
        self.rates = ((self.pitch - p0) / TICK_S, (self.roll - r0) / TICK_S)
        self.max_tilt = max(self.max_tilt, abs(self.pitch), abs(self.roll))

    def _substep(self, dt: float) -> None:
        m = self.model
        com, feet = self._world(self.pitch, self.roll)
        low = min(f[2] for f in feet)
        contacts = [leg for leg in LEGS if feet[leg][2] - low < self.contact_mm]
        # a diagonal partner of a single lowest foot, close to the ground: both stand (the legs are not rigid)
        if len(contacts) == 1:
            k = contacts[0]
            partner = (k + 2) % 4
            if feet[partner][2] - low < self.pair_mm:
                contacts = sorted((k, partner))
        cbits = 0
        for leg in contacts:
            cbits |= 1 << leg
        lifted = [leg for leg in LEGS if leg not in contacts]
        if lifted:
            self.min_clear = min(self.min_clear, min(feet[leg][2] - low for leg in lifted))
        if len(contacts) >= 3:
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
                        self.rock_rate = 0.0
                        self.pair = None
                        self.rate = 0.0
                        self.axis_n = None
                        self._set_contacts(contacts, mg)
                        return
        if len(contacts) == 2 and (contacts[0] + 2) % 4 == contacts[1]:
            self._rock(tuple(contacts), cbits, dt)
            return
        if len(contacts) == 3 and self.pair is not None and all(leg in contacts for leg in self.pair):
            # a third foot touching that does not hold the mass yet (less than inset_mm inside): the
            # body goes on rocking on its diagonal
            b = (1 << self.pair[0]) | (1 << self.pair[1])
            self._rock(self.pair, b, dt)
            return
        # one foot, two on a side, or the mass outside three: Physics' tipping about the nearest edge
        self.pair = None
        self.rock_rate = 0.0
        super()._substep(dt)

    def _rock(self, pair, cbits: int, dt: float) -> None:
        m = self.model
        # the attitude along the line follows the legs: both feet on the ground, the rotation about
        # the line kept (the least change from the attitude now)
        fp, fr = fit_attitude(m, self.xs, self.zs, cbits, (self.pitch, self.roll), reg=1e-6)
        com, feet = self._world(fp, fr)
        a, b = feet[pair[0]], feet[pair[1]]
        ex, ey = b[0] - a[0], b[1] - a[1]
        L = math.hypot(ex, ey) or 1.0
        n = (-ey / L, ex / L)
        if self.pair != pair:
            if self.pair is not None:  # straight from the other diagonal: the rotation about the new line
                self.rock_rate *= self.carry * (self.n[0] * n[0] + self.n[1] * n[1])
            elif self.axis_n is not None:  # from Physics' tipping
                self.rock_rate = self.rate * (self.axis_n[0] * n[0] + self.axis_n[1] * n[1])
            else:  # from standing: at rest
                self.rock_rate = 0.0
            self.pair = pair
        elif (self.n[0] * n[0] + self.n[1] * n[1]) < 0:
            self.rock_rate = -self.rock_rate
        self.n = n
        u = (com[0] - a[0]) * n[0] + (com[1] - a[1]) * n[1]
        h = com[2] - min(a[2], b[2])
        self.rock_rate += G_MM_S2 * u / (h * h + u * u) * dt
        d = math.degrees(self.rock_rate * dt)
        self.pitch = fp + d * n[0]
        self.roll = fr + d * n[1]
        self.rate = 0.0
        self.axis_n = None
        self._set_contacts(list(pair), -abs(u))
        if abs(self.pitch) > self.fall_deg or abs(self.roll) > self.fall_deg:
            self.fallen = True
