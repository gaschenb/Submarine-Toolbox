"""Navigation: inertial position estimate with drift, GPS fixes at PD.

INS error model: velocity-bias random walk plus white noise. When any
mast is raised at periscope depth, GPS is available and (in auto mode)
snaps the estimate back to a few meters.
"""

from __future__ import annotations

import numpy as np

from submarine_toolbox.core.subsystem import Subsystem

class Navigation(Subsystem):
    name = "navigation"

    SIGMA_W = 0.05
    SIGMA_B = 2e-4
    GPS_SIGMA = 5.0
    GPS_PERIOD = 10.0

    def __init__(self, sim, seed=None):
        super().__init__(sim)
        self.rng = np.random.default_rng(seed)
        self.err = np.zeros(2)
        self.bias = self.rng.normal(0.0, 0.02, 2)
        self.auto_fix = True
        self._since_fix = 1e9
        self._noise = np.empty((0, 4))
        self._k = 0

    def gps_fix(self):
        self.err = self.rng.normal(0.0, self.GPS_SIGMA, 2)
        self.bias *= 0.2
        self._since_fix = 0.0

    def update(self, dt: float) -> None:
        if self._k >= len(self._noise):
            self._noise = self.rng.standard_normal((4096, 4))
            self._k = 0
        z = self._noise[self._k]
        self._k += 1
        sq = np.sqrt(dt)
        self.bias += z[:2] * (self.SIGMA_B * sq)
        self.err += self.bias * dt + z[2:] * (self.SIGMA_W * sq)
        self._since_fix += dt

        gps_avail = self.get("masts.any_raised", False) \
            and self.get("state.depth", 99.0) <= 18.0
        if gps_avail and self.auto_fix and self._since_fix > self.GPS_PERIOD:
            self.gps_fix()

        est = self.sim.eta[:2] + self.err
        self.put("nav.est_ne", est)
        self.put("nav.err_true_m", float(np.linalg.norm(self.err)))
        self.put("nav.gps_available", gps_avail)
        self.put("nav.since_fix", self._since_fix)
