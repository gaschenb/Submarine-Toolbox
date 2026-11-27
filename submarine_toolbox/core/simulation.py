"""Fixed-timestep simulation core.

Owns the vessel state, the shared bus, and the ordered subsystem list.
Each step: subsystems update (writing commands/telemetry to the bus and
producing body forces), then the 6-DOF state is integrated with RK4.
"""

from __future__ import annotations

import math

import numpy as np

from submarine_toolbox.core.subsystem import Subsystem
from submarine_toolbox.dynamics.vessel import VesselParams
from submarine_toolbox.dynamics.rigid_body import RigidBodyDynamics

_TWO_PI = 2.0 * math.pi

class Simulation:
    def __init__(self, params: VesselParams | None = None, dt: float = 0.02):
        self.params = params or VesselParams()
        self.dynamics = RigidBodyDynamics(self.params)
        self.dt = dt
        self.time = 0.0

        self.eta = np.array([0.0, 0.0, 15.0, 0.0, 0.0, 0.0])
        self.nu = np.zeros(6)

        self.bus: dict = {
            "autopilot.enabled": True,
            "order.heading": 0.0,
            "order.depth": 15.0,
            "order.speed": 2.5,
            "order.rpm": 0.0,
            "order.rudder": 0.0,
            "order.planes": 0.0,
        }
        self.subsystems: list = []
        self._forcing: list = []
        self._by_name: dict = {}

    def add(self, subsystem_cls, *args, **kwargs):
        sub = subsystem_cls(self, *args, **kwargs)
        self.subsystems.append(sub)
        self._by_name.setdefault(sub.name, sub)
        if type(sub).tau is not Subsystem.tau:
            self._forcing.append(sub)
        return sub

    def get_subsystem(self, name: str):
        return self._by_name.get(name)

    def step(self, n: int = 1):
        for _ in range(n):
            self._single_step()

    def _single_step(self):
        dt = self.dt
        bus = self.bus
        p = self.params
        for sub in self.subsystems:
            sub.update(dt)

        tau = [0.0] * 6
        for sub in self._forcing:
            t = sub.tau()
            for i in range(6):
                tau[i] += t[i]
        W_b = bus.get("ballast.weight", p.m * p.g)
        xg_b = bus.get("ballast.xg", 0.0)
        zg = bus.get("ballast.zg", p.zg)
        W = W_b + bus.get("damage.flood_weight", 0.0)
        xg = (xg_b * W_b + bus.get("damage.flood_moment_x", 0.0)) / W

        f = self.dynamics.rhs
        s = self.eta.tolist() + self.nu.tolist()
        h = 0.5 * dt
        k1 = f(s, tau, W, xg, zg)
        k2 = f([a + h * b for a, b in zip(s, k1)], tau, W, xg, zg)
        k3 = f([a + h * b for a, b in zip(s, k2)], tau, W, xg, zg)
        k4 = f([a + dt * b for a, b in zip(s, k3)], tau, W, xg, zg)
        c = dt / 6.0
        s = [a + c * (b1 + 2.0 * b2 + 2.0 * b3 + b4)
             for a, b1, b2, b3, b4 in zip(s, k1, k2, k3, k4)]

        cur = bus.get("env.current_ned")
        if cur is not None:
            s[0] += cur[0] * dt
            s[1] += cur[1] * dt
            s[2] += cur[2] * dt

        grounded = False
        top = -p.R * 0.8
        if s[2] < top:
            s[2] = top
            s[8] = max(s[8], 0.0)
        floor = bus.get("env.seafloor_depth", 400.0) - p.R
        if s[2] >= floor:
            s[2] = floor
            s[6] = s[7] = s[8] = 0.0
            s[9] *= 0.5
            s[10] *= 0.5
            s[11] *= 0.5
            grounded = True
        for i in (3, 4, 5):
            s[i] = (s[i] + math.pi) % _TWO_PI - math.pi
        bus["state.grounded"] = grounded

        self.eta = np.array(s[:6])
        self.nu = np.array(s[6:])
        self.time += dt

        bus["state.depth"] = s[2]
        bus["state.heading"] = s[5]
        bus["state.pitch"] = s[4]
        bus["state.roll"] = s[3]
        bus["state.speed"] = math.sqrt(s[6] * s[6] + s[7] * s[7] + s[8] * s[8])
        bus["state.u"] = s[6]
        bus["time"] = self.time
