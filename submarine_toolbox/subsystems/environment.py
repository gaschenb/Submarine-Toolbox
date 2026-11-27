"""Ocean environment: sound velocity profile, uniform current, sea state,
ambient noise level, procedural bathymetry.

The current is applied as a kinematic drift by the simulation core. The
SVP is a mixed-layer / thermocline / deep-isothermal model. The seafloor
is an unbounded procedural field sampled under and ahead of the boat.
"""

from __future__ import annotations

import math

import numpy as np

from submarine_toolbox.core.subsystem import Subsystem
from submarine_toolbox.world.bathymetry import Bathymetry

class Environment(Subsystem):
    name = "environment"

    SAMPLE_PERIOD = 0.5
    AHEAD = np.array([0.0, 250.0, 500.0, 1000.0, 1500.0, 2000.0])

    def __init__(self, sim, layer_depth=60.0, sea_state=3,
                 current_speed=0.3, current_dir_deg=45.0,
                 seafloor_depth=480.0, procedural=True, seed=7):
        super().__init__(sim)
        self.layer_depth = layer_depth
        self.sea_state = sea_state
        self.current_speed = current_speed
        self.current_dir = math.radians(current_dir_deg)
        self.procedural = procedural
        self.bathy = Bathymetry(seed=seed, mean=seafloor_depth)
        self._flat = seafloor_depth
        self._acc = self.SAMPLE_PERIOD
        self._cur_key = None
        self._floor = seafloor_depth

    @property
    def seafloor_depth(self):
        return self._floor

    @seafloor_depth.setter
    def seafloor_depth(self, v):
        self._flat = float(v)
        self.bathy.mean = float(v)

    def floor_at(self, n, e):
        if self.procedural:
            return self.bathy.depth(n, e)
        return np.full(np.shape(n), self._flat)

    def sound_speed(self, z):
        """c(z) [m/s]: mixed layer, thermocline gradient, deep isothermal."""
        z = np.asarray(z, dtype=float)
        c0, g_th, g_deep, z_deep = 1520.0, -0.35, 0.017, 700.0
        c_th = c0 + g_th * (z - self.layer_depth)
        c_bot = c0 + g_th * (z_deep - self.layer_depth)
        c_dp = c_bot + g_deep * (z - z_deep)
        return np.where(z <= self.layer_depth, c0,
                        np.where(z <= z_deep, c_th, c_dp))

    @property
    def ambient_nl(self) -> float:
        return 55.0 + 5.0 * self.sea_state

    def _sample_floor(self):
        eta = self.sim.eta
        psi = eta[5]
        n = eta[0] + self.AHEAD * math.cos(psi)
        e = eta[1] + self.AHEAD * math.sin(psi)
        d = self.floor_at(n, e)
        self._floor = float(d[0])
        self.put("env.seafloor_depth", self._floor)
        self.put("env.floor_ahead", d)
        self.put("env.min_floor_ahead", float(d.min()))

    def update(self, dt: float) -> None:
        key = (self.current_speed, self.current_dir)
        if key != self._cur_key:
            self._cur_key = key
            self.put("env.current_ned", (
                self.current_speed * math.cos(self.current_dir),
                self.current_speed * math.sin(self.current_dir), 0.0))
        self.put("env.sea_state", self.sea_state)
        self.put("env.ambient_nl", self.ambient_nl)
        self.put("env.layer_depth", self.layer_depth)
        self._acc += dt
        if self._acc >= self.SAMPLE_PERIOD:
            self._acc = 0.0
            self._sample_floor()
