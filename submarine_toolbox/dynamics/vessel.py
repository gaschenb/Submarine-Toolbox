"""Notional vessel parameters (~1500 t diesel-electric SSK).

All hydrodynamic coefficients are plausible orders of magnitude, not
identified from any real hull. SNAME body frame: x fwd, y stbd, z down.
"""

from dataclasses import dataclass
import numpy as np

@dataclass
class VesselParams:
    L: float = 60.0
    R: float = 3.0
    m: float = 1.5e6
    rho: float = 1025.0
    g: float = 9.81

    Ix: float = 7.0e6
    Iy: float = 4.5e8
    Iz: float = 4.5e8

    Xud: float = 7.5e4
    Yvd: float = 1.35e6
    Zwd: float = 1.35e6
    Kpd: float = 7.0e5
    Mqd: float = 3.6e8
    Nrd: float = 3.6e8

    d_lin: tuple = (5.0e3, 1.0e5, 1.0e5, 1.0e6, 5.0e7, 5.0e7)
    d_quad: tuple = (2.15e3, 2.0e5, 2.0e5, 1.0e7, 5.0e8, 5.0e8)

    zg: float = 0.30
    zb: float = 0.0
    xb: float = 0.0

    test_depth: float = 250.0
    crush_depth: float = 350.0

    mbt_capacity: float = 1.5e5
    var_capacity: float = 2.0e4
    trim_arm: float = 0.05

    def __post_init__(self):
        self.volume = self.m / self.rho
        self.hull_mass = self.m - self.mbt_capacity - 0.5 * self.var_capacity

    @property
    def M(self) -> np.ndarray:
        """Total (rigid + added) mass matrix, diagonal approximation."""
        return np.diag([
            self.m + self.Xud,
            self.m + self.Yvd,
            self.m + self.Zwd,
            self.Ix + self.Kpd,
            self.Iy + self.Mqd,
            self.Iz + self.Nrd,
        ])
