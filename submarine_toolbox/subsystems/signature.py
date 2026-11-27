"""Own-ship radiated noise (broadband source level, dB re 1 uPa @ 1 m).

Components: hotel/machinery base, shaft/machinery (rpm), flow noise
(speed), cavitation, snorkeling diesels, ballast-blow transient.
Also estimates the counter-detection radius against a nominal threat
passive sonar.
"""

from __future__ import annotations

import math

import numpy as np

from submarine_toolbox.core.subsystem import Subsystem

def _db_sum(levels):
    return 10.0 * np.log10(np.sum(10.0 ** (np.asarray(levels) / 10.0)))

_K = 20.0 / math.log(10.0)

def solve_range_for_tl(tl_target: float, alpha_db_per_m: float,
                       r0: float | None = None) -> float:
    """Solve 20 log10(r) + alpha r = TL for r (Newton in ln r)."""
    if tl_target <= 0:
        return 1.0
    x = math.log(r0) if r0 else min(tl_target / _K, 30.0)
    for _ in range(40):
        r = math.exp(x)
        f = _K * x + alpha_db_per_m * r - tl_target
        dx = f / (_K + alpha_db_per_m * r)
        x = max(x - dx, 0.0)
        if abs(dx) < 1e-10:
            break
    return math.exp(x)

class Signature(Subsystem):
    name = "signature"

    BASE_HOTEL = 88.0
    ALPHA = 1.0e-4
    THREAT_FOM_OFFSET = 65.0 - 15.0 + 10.0

    def __init__(self, sim):
        super().__init__(sim)
        self._rcd = None

    def update(self, dt: float) -> None:
        rpm = abs(self.get("propulsion.rpm", 0.0))
        u = abs(self.get("state.u", 0.0))

        comps = {
            "hotel": self.BASE_HOTEL,
            "machinery": 92.0 + 18.0 * math.log10(1.0 + rpm / 20.0),
            "flow": 78.0 + 22.0 * math.log10(1.0 + u / 1.5),
        }
        if self.get("propulsion.cavitating", False):
            comps["cavitation"] = comps["machinery"] + 28.0
        if self.get("power.snorkeling", False):
            comps["snorkel_diesels"] = 135.0
        if self.get("ballast.blowing", False):
            comps["blow_transient"] = 150.0
        if self.get("damage.pumping", False):
            comps["bilge_pumps"] = 118.0

        sl = 10.0 * math.log10(sum(10.0 ** (v / 10.0) for v in comps.values()))
        r_cd = solve_range_for_tl(sl - self.THREAT_FOM_OFFSET, self.ALPHA,
                                  self._rcd)
        self._rcd = r_cd

        self.put("signature.sl", float(sl))
        self.put("signature.components", comps)
        self.put("signature.counter_detection_m", float(r_cd))
