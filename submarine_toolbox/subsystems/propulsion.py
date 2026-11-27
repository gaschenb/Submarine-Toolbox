"""Main propulsion: single shaft, fixed-pitch propeller.

Thrust/torque via open-water coefficients: T = (1-t) Kt rho n|n| D^4,
Q = Kq rho n|n| D^5. First-order shaft lag. Publishes electrical load
for the power subsystem; clamps to zero if the electrical bus is dead.
"""

from __future__ import annotations

import math

from submarine_toolbox.core.subsystem import Subsystem

class Propulsion(Subsystem):
    name = "propulsion"

    def __init__(self, sim, D=3.0, Kt=0.30, Kq=0.05, t_ded=0.10,
                 max_rpm=200.0, shaft_tau=8.0, motor_eff=0.90):
        super().__init__(sim)
        self.D, self.Kt, self.Kq = D, Kt, Kq
        self.t_ded = t_ded
        self.max_rpm = max_rpm
        self.shaft_tau = shaft_tau
        self.motor_eff = motor_eff
        self.rpm = 0.0
        self._thrust = 0.0

    def update(self, dt: float) -> None:
        lim = self.max_rpm * self.get("damage.propulsion_limit", 1.0)
        rpm_cmd = min(max(float(self.get("cmd.rpm", 0.0)), -lim), lim)
        if self.get("power.dead", False):
            rpm_cmd = 0.0
        self.rpm += (rpm_cmd - self.rpm) * dt / self.shaft_tau

        rho = self.sim.params.rho
        n = self.rpm / 60.0
        thrust = (1.0 - self.t_ded) * self.Kt * rho * n * abs(n) * self.D ** 4
        torque = self.Kq * rho * n * abs(n) * self.D ** 5
        power = 2.0 * math.pi * abs(n) * abs(torque) / self.motor_eff

        depth = max(self.get("state.depth", 0.0), 0.0)
        cavitating = abs(self.rpm) > (60.0 + 6.0 * depth)

        self._thrust = thrust
        self.put("propulsion.rpm", self.rpm)
        self.put("propulsion.thrust", thrust)
        self.put("propulsion.torque", torque)
        self.put("propulsion.power", power)
        self.put("propulsion.cavitating", bool(cavitating))

    def tau(self):
        return (self._thrust, 0.0, 0.0, 0.0, 0.0, 0.0)
