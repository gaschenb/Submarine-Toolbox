"""Masts (periscope / ESM / radar / snorkel) and mast-borne sensors.

Masts can only be raised at periscope depth or shallower; going deep
forces them down. The surface-search radar detects surface contacts to
the radar horizon but is counter-detectable by their ESM at ~1.5x that
range (one-way vs two-way propagation). Own ESM passively intercepts
contacts' navigation radars.
"""

from __future__ import annotations

import numpy as np

from submarine_toolbox.core.subsystem import Subsystem

PD_LIMIT = 18.0

def radar_horizon(h1: float, h2: float) -> float:
    """4/3-earth radar horizon [m] for antenna heights h1, h2 [m]."""
    return 4120.0 * (np.sqrt(max(h1, 0.1)) + np.sqrt(max(h2, 0.1)))

class Masts(Subsystem):
    name = "masts"

    ATTRS = ("periscope", "esm", "radar", "snorkel", "comms")

    def __init__(self, sim):
        super().__init__(sim)
        self.periscope = False
        self.esm = False
        self.radar = False
        self.snorkel = False
        self.comms = False
        self.radar_tx = False
        self._prev = {a: False for a in self.ATTRS}

    def update(self, dt: float) -> None:
        if self.get("hydraulics.effectiveness", 1.0) < 0.3:
            for a in self.ATTRS:
                if getattr(self, a) and not self._prev[a]:
                    setattr(self, a, False)

        deep = self.get("state.depth", 0.0) > PD_LIMIT
        if deep:
            for a in self.ATTRS:
                setattr(self, a, False)
        if not self.radar:
            self.radar_tx = False
        self._prev = {a: getattr(self, a) for a in self.ATTRS}

        for a in self.ATTRS:
            self.put(f"masts.{a}", getattr(self, a))
        self.put("masts.radar_tx", self.radar_tx)
        self.put("masts.any_raised", any(getattr(self, a)
                                         for a in self.ATTRS))

class RadarESM(Subsystem):
    name = "radar_esm"

    MAST_HEIGHT = 2.0
    RADAR_KW = 15.0
    UPDATE_PERIOD = 1.0

    HEIGHTS = {"merchant": 25.0, "warship": 18.0, "fishing": 8.0,
               "submarine": 0.0}

    def __init__(self, sim, seed=None):
        super().__init__(sim)
        self.rng = np.random.default_rng(seed)
        self._acc = 0.0

    def update(self, dt: float) -> None:
        self._acc += dt
        if self._acc < self.UPDATE_PERIOD:
            return
        self._acc = 0.0

        shed = self.get("power.shed", False)
        own = self.sim.eta
        truth = self.get("contacts.truth", [])

        radar_on = self.get("masts.radar_tx", False) and not shed
        detections, counter = [], []
        if radar_on:
            for c in truth:
                h = self.HEIGHTS.get(c["ctype"], 10.0)
                if h <= 0.0:
                    continue
                d = c["pos"] - own[:3]
                r = float(np.hypot(d[0], d[1]))
                rh = radar_horizon(self.MAST_HEIGHT, h)
                if r <= rh:
                    detections.append(dict(
                        cid=c["cid"],
                        range=r + self.rng.normal(0.0, 20.0),
                        bearing=float(np.arctan2(d[1], d[0]))
                        + self.rng.normal(0.0, np.radians(0.5))))
                if c.get("has_esm", False) and r <= 1.5 * rh:
                    counter.append(c["cid"])
        self.put("load.radar", self.RADAR_KW * 1e3 if radar_on else 0.0)
        self.put("radar.contacts", detections)
        self.put("radar.counter_detected_by", counter)

        esm_on = self.get("masts.esm", False) and not shed
        intercepts = []
        if esm_on:
            for c in truth:
                if not c.get("radar_on", False):
                    continue
                h = self.HEIGHTS.get(c["ctype"], 10.0)
                d = c["pos"] - own[:3]
                r = float(np.hypot(d[0], d[1]))
                if r <= 1.5 * radar_horizon(self.MAST_HEIGHT, h):
                    snr = 40.0 - 20.0 * np.log10(max(r, 100.0) / 1e3)
                    intercepts.append(dict(
                        cid=c["cid"],
                        bearing=float(np.arctan2(d[1], d[0]))
                        + self.rng.normal(0.0, np.radians(2.0)),
                        emitter=f"{c['ctype']}_nav_radar",
                        strength=float(snr)))
        self.put("esm.intercepts", intercepts)
