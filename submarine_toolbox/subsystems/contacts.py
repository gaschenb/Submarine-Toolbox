"""Ambient traffic: surface ships and submarines for the sonar picture.

Contacts move in straight lines with occasional random course changes.
The traffic field is unbounded: contacts that fall beyond the despawn
radius from own ship are recycled onto a ring ahead of nothing in
particular, so density around the boat stays constant wherever it goes.
State is held in arrays and published as arrays plus a truth list.
"""

from __future__ import annotations

import numpy as np

from submarine_toolbox.core.subsystem import Subsystem

CONTACT_TYPES = {
    "merchant": dict(sl=165.0, ts=25.0, spd=7.0, depth=0.0,
                     p_radar=0.95, esm=False),
    "warship": dict(sl=145.0, ts=15.0, spd=9.0, depth=0.0,
                    p_radar=0.7, esm=True),
    "fishing": dict(sl=152.0, ts=12.0, spd=4.0, depth=0.0,
                    p_radar=0.6, esm=False),
    "submarine": dict(sl=125.0, ts=10.0, spd=3.0, depth=120.0,
                      p_radar=0.0, esm=True),
}
_TYPES = list(CONTACT_TYPES)
_WEIGHTS = [0.4, 0.15, 0.3, 0.15]

class ContactManager(Subsystem):
    name = "contacts"

    PERIOD = 0.1

    def __init__(self, sim, n_contacts=6, spawn_radius=25e3,
                 respawn_ring=(35e3, 45e3), despawn_radius=55e3, seed=None):
        super().__init__(sim)
        self.rng = np.random.default_rng(seed)
        self.spawn_radius = spawn_radius
        self.respawn_ring = respawn_ring
        self.despawn_radius = despawn_radius
        n = n_contacts
        self.cid = np.zeros(n, dtype=int)
        self.ctype = [""] * n
        self.pos = np.zeros((n, 3))
        self.course = np.zeros(n)
        self.speed = np.zeros(n)
        self.sl = np.zeros(n)
        self.ts = np.zeros(n)
        self.radar_on = np.zeros(n, dtype=bool)
        self.has_esm = np.zeros(n, dtype=bool)
        self.turn_timer = np.full(n, 120.0)
        self._next_id = 1
        self._acc = 0.0
        own = sim.eta[:2]
        for i in range(n):
            self._spawn(i, own, 3e3, spawn_radius)
        self._publish()

    def _spawn(self, i, own, rmin, rmax):
        rng = self.rng
        ct = str(rng.choice(_TYPES, p=_WEIGHTS))
        spec = CONTACT_TYPES[ct]
        r = rng.uniform(rmin, rmax)
        b = rng.uniform(0, 2 * np.pi)
        depth = spec["depth"] if ct != "submarine" else rng.uniform(60, 200)
        self.cid[i] = self._next_id
        self._next_id += 1
        self.ctype[i] = ct
        self.pos[i] = (own[0] + r * np.cos(b), own[1] + r * np.sin(b), depth)
        self.course[i] = rng.uniform(0, 2 * np.pi)
        self.speed[i] = spec["spd"] * rng.uniform(0.7, 1.2)
        self.sl[i] = spec["sl"] + rng.uniform(-3, 3)
        self.ts[i] = spec["ts"]
        self.radar_on[i] = rng.random() < spec["p_radar"]
        self.has_esm[i] = spec["esm"]
        self.turn_timer[i] = 120.0

    def _publish(self):
        self.put("contacts.cid", self.cid.copy())
        self.put("contacts.pos", self.pos.copy())
        self.put("contacts.sl", self.sl.copy())
        self.put("contacts.ts", self.ts.copy())
        self.put("contacts.ctype", list(self.ctype))
        self.put("contacts.truth", [
            dict(cid=int(self.cid[i]), ctype=self.ctype[i],
                 pos=self.pos[i].copy(), course=float(self.course[i]),
                 speed=float(self.speed[i]), sl=float(self.sl[i]),
                 ts=float(self.ts[i]), radar_on=bool(self.radar_on[i]),
                 has_esm=bool(self.has_esm[i]))
            for i in range(len(self.cid))])

    def update(self, dt: float) -> None:
        self._acc += dt
        if self._acc < self.PERIOD:
            return
        h, self._acc = self._acc, 0.0
        self.pos[:, 0] += self.speed * np.cos(self.course) * h
        self.pos[:, 1] += self.speed * np.sin(self.course) * h
        self.turn_timer -= h
        due = self.turn_timer <= 0.0
        k = int(due.sum())
        if k:
            self.turn_timer[due] = self.rng.uniform(180.0, 900.0, k)
            self.course[due] += self.rng.uniform(-0.6, 0.6, k)
        own = self.sim.eta[:2]
        far = np.hypot(self.pos[:, 0] - own[0], self.pos[:, 1] - own[1]) \
            > self.despawn_radius
        for i in np.flatnonzero(far):
            self._spawn(i, own, *self.respawn_ring)
        self._publish()
