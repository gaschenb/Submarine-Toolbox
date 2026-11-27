"""Atmosphere control: O2 / CO2 balance for the pressure hull.

Crew consumes O2 and produces CO2; the O2 generator (electrolysis) and
CO2 scrubbers counteract while powered. Ventilation while surfaced or
snorkeling restores the atmosphere exponentially. Equipment publishes
its electrical load and respects load shedding.
"""

from __future__ import annotations

from submarine_toolbox.core.subsystem import Subsystem

O2_NORM = 20.9
CO2_NORM = 0.04

class Atmosphere(Subsystem):
    name = "atmosphere"

    def __init__(self, sim, volume_m3=2000.0, crew=40,
                 o2_burn_lpm_pp=0.5, co2_make_lpm_pp=0.4,
                 o2_gen_lpm=25.0, scrubber_lpm=30.0,
                 o2_gen_kw=40.0, scrubber_kw=20.0,
                 o2_bank_m3=200.0):
        super().__init__(sim)
        self.V = volume_m3
        self.crew = crew
        self.o2_burn = crew * o2_burn_lpm_pp / 60e3
        self.co2_make = crew * co2_make_lpm_pp / 60e3
        self.o2_gen_rate = o2_gen_lpm / 60e3
        self.scrub_rate = scrubber_lpm / 60e3
        self.o2_gen_kw, self.scrubber_kw = o2_gen_kw, scrubber_kw
        self.o2 = O2_NORM
        self.co2 = CO2_NORM
        self.o2_gen_on = True
        self.scrubber_on = True
        self.o2_bank = o2_bank_m3
        self.bleed_bank = False

    def update(self, dt: float) -> None:
        shed = self.get("power.shed", False) or self.get("power.dead", False)
        equip_ok = self.get("damage.atmo_equipment_ok", True)
        gen_run = self.o2_gen_on and not shed and equip_ok
        scrub_run = self.scrubber_on and not shed and equip_ok

        fire_o2 = self.get("damage.fire_o2_rate", 0.0)
        do2 = -self.o2_burn - fire_o2
        dco2 = self.co2_make + fire_o2
        if gen_run:
            do2 += self.o2_gen_rate
        if scrub_run:
            dco2 -= min(self.scrub_rate,
                        max(self.co2 - CO2_NORM, 0.0) / 100.0 * self.V / 60.0)
        if self.bleed_bank and self.o2_bank > 0.0 and self.o2 < O2_NORM:
            bleed = min(self.o2_gen_rate * 2.0, self.o2_bank / dt)
            do2 += bleed
            self.o2_bank -= bleed * dt

        self.o2 += do2 / self.V * 100.0 * dt
        self.co2 += dco2 / self.V * 100.0 * dt

        if self.get("power.snorkeling", False) or \
                self.get("state.depth", 99.0) < 3.0:
            k = dt / 600.0
            self.o2 += (O2_NORM - self.o2) * k
            self.co2 += (CO2_NORM - self.co2) * k

        self.o2 = min(max(self.o2, 0.0), 25.0)
        self.co2 = min(max(self.co2, 0.0), 10.0)

        load = (self.o2_gen_kw if gen_run else 0.0) \
            + (self.scrubber_kw if scrub_run else 0.0)
        self.put("load.atmosphere", load * 1e3)

        status = "NORMAL"
        if self.o2 < 17.0 or self.co2 > 2.0:
            status = "DANGER"
        elif self.o2 < 19.5 or self.co2 > 0.5:
            status = "CAUTION"
        self.put("atmo.o2", self.o2)
        self.put("atmo.co2", self.co2)
        self.put("atmo.o2_bank", self.o2_bank)
        self.put("atmo.status", status)
        self.put("atmo.equipment_ok", equip_ok)
