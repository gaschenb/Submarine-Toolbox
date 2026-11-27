"""Endurance: diesel fuel, provisions, fresh water.

Fuel burns with generator output (specific fuel consumption); the power
plant loses diesel charging when tanks run dry. Crew consumes food and
water on wall-clock rates; the distiller replenishes water while
powered. Publishes days-remaining estimates.
"""

from __future__ import annotations

from submarine_toolbox.core.subsystem import Subsystem

DAY = 86400.0

class Endurance(Subsystem):
    name = "endurance"

    SFC = 0.22 / 3.6e6
    WATER_USE = 6.0 / DAY
    DISTILLER_MAKE = 10.0 / DAY
    DISTILLER_KW = 30.0
    FOOD_RATE = 1.0 / DAY

    def __init__(self, sim, fuel_t=80.0, food_days=90.0, water_t=10.0,
                 water_cap_t=12.0):
        super().__init__(sim)
        self.fuel = fuel_t * 1e3
        self.fuel0 = self.fuel
        self.food_days = food_days
        self.water = water_t
        self.water_cap = water_cap_t
        self.distiller_on = True

    def update(self, dt: float) -> None:
        gen_w = self.get("power.gen_kw", 0.0) * 1e3
        self.fuel = max(self.fuel - gen_w * self.SFC * dt, 0.0)

        self.food_days = max(self.food_days - self.FOOD_RATE * dt, 0.0)

        powered = not (self.get("power.dead", False)
                       or self.get("power.shed", False))
        run_dist = self.distiller_on and powered and \
            self.water < self.water_cap
        dw = -self.WATER_USE + (self.DISTILLER_MAKE if run_dist else 0.0)
        self.water = min(max(self.water + dw * dt, 0.0), self.water_cap)
        self.put("load.distiller",
                 self.DISTILLER_KW * 1e3 if run_dist else 0.0)

        self.put("endurance.fuel_t", self.fuel / 1e3)
        self.put("endurance.fuel_pct", 100.0 * self.fuel / self.fuel0)
        self.put("endurance.fuel_ok", self.fuel > 0.0)
        self.put("endurance.food_days", self.food_days)
        self.put("endurance.water_t", self.water)
        self.put("endurance.water_days",
                 self.water / (self.WATER_USE * DAY))
