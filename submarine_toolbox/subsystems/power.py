"""Electrical plant: main battery + diesel generators (snorkel-limited).

Consumers publish loads on `load.*` bus keys [W]; the plant sums these
with propulsion and hotel load. Battery voltage sags with SOC and load.
Load shedding trips non-vital consumers below SOC thresholds (consumers
honor the `power.shed` flag). Snorkel state comes from the mast manager.
"""

from __future__ import annotations

from submarine_toolbox.core.subsystem import Subsystem

class Power(Subsystem):
    name = "power"

    V_NOM = 440.0
    R_INT = 4.0e-3
    SHED_SOC = 0.10

    def __init__(self, sim, capacity_kwh=5000.0, hotel_kw=150.0,
                 diesel_kw=2500.0, snorkel_depth=14.0):
        super().__init__(sim)
        self.capacity_j = capacity_kwh * 3.6e6
        self.energy_j = self.capacity_j
        self.hotel_w = hotel_kw * 1e3
        self.diesel_w = diesel_kw * 1e3
        self.snorkel_depth = snorkel_depth
        self._nkeys = -1
        self._load_keys = ()

    @property
    def soc(self) -> float:
        return self.energy_j / self.capacity_j

    def update(self, dt: float) -> None:
        bus = self.sim.bus
        if len(bus) != self._nkeys:
            self._nkeys = len(bus)
            self._load_keys = tuple((k, k[5:]) for k in bus
                                    if k.startswith("load."))
        ext = {name: bus[k] for k, name in self._load_keys}
        prop = self.get("propulsion.power", 0.0)
        load = prop + self.hotel_w + sum(ext.values())

        depth = self.get("state.depth", 0.0)
        snorkeling = self.get("masts.snorkel", False) \
            and depth <= self.snorkel_depth
        gen = self.diesel_w if snorkeling \
            and self.get("endurance.fuel_ok", True) else 0.0

        self.energy_j += (gen - load) * dt
        self.energy_j = min(max(self.energy_j, 0.0), self.capacity_j)

        ocv = self.V_NOM * (0.88 + 0.12 * self.soc)
        current = load / max(ocv, 1.0)
        voltage = max(ocv - current * self.R_INT, 0.0)

        self.put("power.soc", self.soc)
        self.put("power.load_kw", load / 1e3)
        self.put("power.gen_kw", gen / 1e3)
        self.put("power.loads_detail",
                 dict(propulsion=prop / 1e3, hotel=self.hotel_w / 1e3,
                      **{k: v / 1e3 for k, v in ext.items()}))
        self.put("power.voltage", voltage)
        self.put("power.snorkeling", snorkeling)
        self.put("power.shed", self.soc < self.SHED_SOC)
        self.put("power.dead", self.soc <= 1e-4)
