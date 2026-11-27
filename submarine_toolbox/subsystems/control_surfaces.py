"""Control surfaces: rudder (yaw) and stern planes (pitch).

Rate-limited actuators; lift ~ 0.5 rho u^2 A CL_delta * delta applied at
a stern lever arm. Sign conventions:
    rudder  > 0  -> yaw to starboard (heading increases)
    planes  > 0  -> nose up (boat rises when moving ahead)
"""

from __future__ import annotations

import numpy as np

from submarine_toolbox.core.subsystem import Subsystem

DEG = np.pi / 180.0

class ControlSurfaces(Subsystem):
    name = "control_surfaces"

    def __init__(self, sim, rudder_area=6.0, plane_area=8.0, cl_delta=3.0,
                 x_rudder=-25.0, x_planes=-22.0,
                 max_rudder=30 * DEG, max_planes=25 * DEG, rate=7 * DEG):
        super().__init__(sim)
        self.Ar, self.Ap, self.cl = rudder_area, plane_area, cl_delta
        self.xr, self.xp = x_rudder, x_planes
        self.max_r, self.max_p, self.rate = max_rudder, max_planes, rate
        self.rudder = 0.0
        self.planes = 0.0

        self._tau = (0.0,) * 6

    @staticmethod
    def _slew(actual, cmd, limit, rate, dt):
        cmd = -limit if cmd < -limit else limit if cmd > limit else cmd
        s = rate * dt
        err = cmd - actual
        return actual + (-s if err < -s else s if err > s else err)

    def update(self, dt: float) -> None:
        rate = self.rate * self.get("hydraulics.effectiveness", 1.0)
        r0, p0 = self.rudder, self.planes
        self.rudder = self._slew(self.rudder, self.get("cmd.rudder", 0.0),
                                 self.max_r, rate, dt)
        self.planes = self._slew(self.planes, self.get("cmd.planes", 0.0),
                                 self.max_p, rate, dt)
        if self.rate > 0.0:
            self.put("hydraulics.usage",
                     (abs(self.rudder - r0) + abs(self.planes - p0))
                     / (self.rate * dt))

        u = self.get("state.u", 0.0)
        q = 0.5 * self.sim.params.rho * u * abs(u)
        Lr = q * self.Ar * self.cl * self.rudder
        Lp = q * self.Ap * self.cl * self.planes

        self._tau = (0.0, -Lr, Lp, 0.0, -self.xp * Lp, -self.xr * Lr)

        self.put("act.rudder", self.rudder)
        self.put("act.planes", self.planes)

    def tau(self) -> np.ndarray:
        return self._tau
