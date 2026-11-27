"""Hydraulic plant: pressure, fluid inventory, electric pump, leaks.

Control surfaces publish their usage; the plant bleeds pressure with
demand and the pump restores it (drawing electrical load). A leak
casualty drains fluid until isolated; available pressure is capped by
fluid remaining. Actuator consumers scale their slew rates by
`hydraulics.effectiveness`.
"""

from __future__ import annotations


from submarine_toolbox.core.subsystem import Subsystem

class Hydraulics(Subsystem):
    name = "hydraulics"

    P_NOM = 207.0
    PUMP_RATE = 4.0
    IDLE_BLEED = 0.05
    USE_BLEED = 3.0
    LEAK_RATE = 0.0008
    PUMP_KW = 25.0
    STANDBY_KW = 3.0

    def __init__(self, sim):
        super().__init__(sim)
        self.pressure = self.P_NOM
        self.fluid = 1.0
        self.pump_on = True
        self.leaking = False
        self.reserve_available = True

    def start_leak(self):
        self.leaking = True

    def isolate_leak(self):
        self.leaking = False

    def refill_from_reserve(self):
        if self.reserve_available and not self.leaking:
            self.fluid = max(self.fluid, 0.8)
            self.reserve_available = False

    def update(self, dt: float) -> None:
        if self.leaking:
            self.fluid = max(self.fluid - self.LEAK_RATE * dt, 0.0)

        usage = self.get("hydraulics.usage", 0.0)
        self.pressure -= (self.IDLE_BLEED + self.USE_BLEED * usage) * dt

        powered = not (self.get("power.dead", False)
                       or self.get("power.shed", False))
        p_max = self.P_NOM * self.fluid
        pumping = self.pump_on and powered and self.pressure < p_max - 1.0
        if pumping:
            self.pressure += self.PUMP_RATE * dt
        self.pressure = min(max(self.pressure, 0.0), p_max)

        self.put("load.hydraulics",
                 (self.PUMP_KW if pumping else self.STANDBY_KW) * 1e3)
        self.put("hydraulics.pressure", self.pressure)
        self.put("hydraulics.fluid", self.fluid)
        self.put("hydraulics.leaking", self.leaking)
        self.put("hydraulics.reserve", self.reserve_available)
        self.put("hydraulics.effectiveness",
                 min(max(self.pressure / 150.0, 0.0), 1.0))
