"""Base class for all submarine subsystems.

Subsystems communicate through the simulation bus (a flat key/value dict)
and may contribute generalized forces tau = [X, Y, Z, K, M, N] in the
body frame (SNAME convention).
"""

from __future__ import annotations

import numpy as np

class Subsystem:
    """Abstract subsystem.

    Lifecycle per simulation step:
        update(dt)  -> advance internal state, read/write sim.bus
        tau()       -> 6-vector of body-frame forces/moments (or zeros)
    """

    name: str = "subsystem"

    def __init__(self, sim):
        self.sim = sim

    def update(self, dt: float) -> None:  # pragma: no cover - interface
        pass

    def tau(self) -> np.ndarray:
        return np.zeros(6)

    def get(self, key: str, default=0.0):
        return self.sim.bus.get(key, default)

    def put(self, key: str, value) -> None:
        self.sim.bus[key] = value
