"""Damage control: compartment flooding and fire.

Flooding: breach inflow Q = Cd A sqrt(2 g depth); bilge pumps derate
with depth (head limit); open watertight doors let water equalize into
adjacent compartments. Flood water adds weight and a pitch moment that
the dynamics core aggregates with ballast.

Fire: per-compartment intensity grows while boat O2 lasts, consumes O2
and produces CO2 (coupled into the atmosphere subsystem), and
accumulates equipment damage -- engine-room damage limits propulsion,
machinery-space damage knocks out atmosphere equipment. Suppression is
a crew action; fires also die below ~15% O2.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from submarine_toolbox.core.subsystem import Subsystem

COMPARTMENT_SPEC = [
    ("FWD", 300.0, 20.0),
    ("CONTROL", 250.0, 8.0),
    ("MACHINERY", 350.0, -2.0),
    ("ENGINE", 400.0, -15.0),
    ("AFT", 200.0, -24.0),
]

BREACH_AREAS = {"small": 0.005, "large": 0.05}

@dataclass
class Compartment:
    name: str
    volume: float
    x: float
    flood: float = 0.0
    breach_area: float = 0.0
    fire: float = 0.0
    fire_damage: float = 0.0
    pump_on: bool = True
    suppressing: bool = False

class DamageControl(Subsystem):
    name = "damage"

    CD = 0.6
    PUMP_RATE = 0.05
    PUMP_HEAD = 150.0
    DOOR_FLOW = 0.02
    FIRE_O2_RATE = 0.02
    SUPPRESS_RATE = 0.25
    DMG_ENGINE_FULL = 150.0
    DMG_MACH_FULL = 90.0

    def __init__(self, sim):
        super().__init__(sim)
        self.compartments = [Compartment(n, v, x)
                             for n, v, x in COMPARTMENT_SPEC]
        self.doors_shut = True
        self._crush_acc = 0.0
        self._rng = np.random.default_rng()
        self._named = {c.name: c for c in self.compartments}

    def by_name(self, name: str) -> Compartment:
        return self._named[name]

    def breach(self, name: str, size: str = "small"):
        self.by_name(name).breach_area += BREACH_AREAS[size]

    def patch(self, name: str):
        self.by_name(name).breach_area = 0.0

    def ignite(self, name: str):
        c = self.by_name(name)
        c.fire = max(c.fire, 0.1)

    def update(self, dt: float) -> None:
        depth = max(self.get("state.depth", 0.0), 0.0)
        o2 = self.get("atmo.o2", 20.9)
        powered = not self.get("power.dead", False)

        p = self.sim.params
        hull_state = "OK"
        if depth > p.crush_depth:
            hull_state = "FAILING"
            self._crush_acc += dt
            if self._crush_acc > 10.0:
                self._crush_acc = 0.0
                victim = self._rng.choice(self.compartments)
                victim.breach_area += BREACH_AREAS["large"]
        elif depth > p.test_depth:
            hull_state = "OVER TEST DEPTH"

        pump_rate = self.PUMP_RATE * max(1.0 - depth / self.PUMP_HEAD, 0.0)
        pumping = False
        fire_o2 = 0.0

        for c in self.compartments:
            if c.breach_area > 0.0:
                q = self.CD * c.breach_area * math.sqrt(
                    2.0 * 9.81 * max(depth, 1.0))
                c.flood = min(c.flood + q * dt, c.volume)
            if c.pump_on and powered and c.flood > 0.0 and pump_rate > 0.0:
                c.flood = max(c.flood - pump_rate * dt, 0.0)
                pumping = True

            if c.fire > 0.0:
                if c.suppressing:
                    c.fire = max(c.fire - self.SUPPRESS_RATE * dt, 0.0)
                elif o2 > 16.0:
                    c.fire = min(c.fire + 0.15 * c.fire
                                 * (o2 - 16.0) / 5.0 * dt, 1.0)
                else:
                    c.fire = max(c.fire - 0.05 * dt, 0.0)
                if c.flood > 0.5 * c.volume:
                    c.fire = 0.0
                c.fire_damage += c.fire * dt
                fire_o2 += self.FIRE_O2_RATE * c.fire
                if c.fire < 1e-3:
                    c.fire = 0.0
                    c.suppressing = False

        if not self.doors_shut:
            for a, b in zip(self.compartments[:-1], self.compartments[1:]):
                fa, fb = a.flood / a.volume, b.flood / b.volume
                dv = self.DOOR_FLOW * (fa - fb) * min(a.volume, b.volume) * dt
                dv = min(max(dv, -b.flood), a.flood)
                a.flood -= dv
                b.flood += dv

        rho = self.sim.params.rho
        w = sum(c.flood for c in self.compartments) * rho * 9.81
        mx = sum(c.flood * c.x for c in self.compartments) * rho * 9.81

        eng = self.by_name("ENGINE")
        prop_limit = min(max(
            1.0 - eng.fire_damage / self.DMG_ENGINE_FULL
            - eng.flood / eng.volume, 0.0), 1.0)
        mach = self.by_name("MACHINERY")
        atmo_ok = mach.fire_damage < self.DMG_MACH_FULL \
            and mach.flood < 0.5 * mach.volume

        self.put("damage.flood_weight", w)
        self.put("damage.flood_moment_x", mx)
        self.put("damage.fire_o2_rate", fire_o2)         # [m3/s]
        self.put("damage.propulsion_limit", prop_limit)
        self.put("damage.atmo_equipment_ok", atmo_ok)
        self.put("damage.pumping", pumping)
        self.put("damage.hull_state", hull_state)
        self.put("damage.any_fire",
                 any(c.fire > 0.0 for c in self.compartments))
        self.put("damage.status", [
            dict(name=c.name, flood_pct=100.0 * c.flood / c.volume,
                 breach=c.breach_area > 0.0, fire=c.fire,
                 damage=c.fire_damage, pump=c.pump_on,
                 suppressing=c.suppressing)
            for c in self.compartments])
