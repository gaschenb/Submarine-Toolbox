"""Passive and active sonar.

Passive:  SNR_excess = SL_tgt - TL - NL_total + DI - DT
Active:   SNR_excess = SL_ping - 2 TL + TS - NL_total + DI - DT

TL = 20 log10(r) + alpha r  (+ cross-layer penalty). Self noise from the
signature subsystem is power-summed with ambient. Bow array is baffled
astern unless the towed array is deployed. Bearing/range measurements
carry SNR-dependent noise. Evaluated vectorized over all contacts.
"""

from __future__ import annotations

import numpy as np

from submarine_toolbox.core.subsystem import Subsystem
from submarine_toolbox.subsystems.signature import _db_sum, solve_range_for_tl

DEG = np.pi / 180.0

def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def transmission_loss(r, alpha, own_z, tgt_z, layer_depth, penalty=12.0,
                      r_trans=1000.0):
    """Hybrid spreading: spherical to r_trans, cylindrical beyond, plus
    absorption and a cross-layer penalty. Scalar or array r / tgt_z."""
    r = np.maximum(r, 1.0)
    tl = np.where(r <= r_trans, 20.0 * np.log10(r),
                  20.0 * np.log10(r_trans) + 10.0 * np.log10(r / r_trans))
    tl = tl + alpha * r
    cross = (own_z < layer_depth) != (np.asarray(tgt_z) < layer_depth)
    tl = tl + penalty * cross
    return float(tl) if np.ndim(tl) == 0 else tl

def _geometry(sim):
    own = sim.eta
    pos = sim.bus.get("contacts.pos")
    if pos is None or len(pos) == 0:
        return None
    d = pos[:, :2] - own[:2]
    return own, pos, np.hypot(d[:, 0], d[:, 1]), np.arctan2(d[:, 1], d[:, 0])

class PassiveSonar(Subsystem):
    name = "passive_sonar"

    DI = 20.0
    DT = 10.0
    ALPHA = 1.0e-4
    BAFFLE_HALF = 30 * DEG
    UPDATE_PERIOD = 0.25
    SELF_NOISE_FLOOR = 55.0

    def __init__(self, sim, seed=None):
        super().__init__(sim)
        self.rng = np.random.default_rng(seed)
        self.towed_deployed = False
        self._acc = 0.0
        self._meas = {}

    def _noise_level(self):
        ambient = self.get("env.ambient_nl", 70.0)
        self_n = max(self.get("signature.sl", 100.0) - 45.0,
                     self.SELF_NOISE_FLOOR)
        return _db_sum([ambient, self_n])

    def update(self, dt: float) -> None:
        self._acc += dt
        if self._acc < self.UPDATE_PERIOD:
            return
        self._acc = 0.0

        nl = self._noise_level()
        self.put("sonar.noise_level", float(nl))
        self.put("sonar.towed_deployed", self.towed_deployed)
        g = _geometry(self.sim)
        if g is None:
            self.put("sonar.contacts", [])
            return
        own, pos, r, brg = g
        cid = self.get("contacts.cid")
        ctype = self.get("contacts.ctype")
        di = self.DI + (3.0 if self.towed_deployed else 0.0)
        tl = transmission_loss(r, self.ALPHA, own[2], pos[:, 2],
                               self.get("env.layer_depth", 60.0))
        snr = self.get("contacts.sl") - tl - nl + di - self.DT \
            + self.rng.normal(0.0, 1.5, len(r))
        ok = snr > 0.0
        if not self.towed_deployed:
            ok &= np.abs(wrap_pi(brg - own[5] - np.pi)) >= self.BAFFLE_HALF
        sigma = np.clip(6.0 / (1.0 + snr / 5.0), 0.3, 6.0) * DEG
        meas = brg + self.rng.normal(0.0, 1.0, len(r)) * sigma

        out, held = [], {}
        for i in np.flatnonzero(ok):
            k = int(cid[i])
            m = float(meas[i])
            prev = self._meas.get(k, m)
            m = prev + 0.5 * float(wrap_pi(m - prev))
            held[k] = m
            s = float(snr[i])
            out.append(dict(cid=k, bearing=m, snr=s,
                            classified=ctype[i] if s > 12.0 else "unknown",
                            truth_range=float(r[i])))
        self._meas = held
        self.put("sonar.contacts", out)

class ActiveSonar(Subsystem):
    name = "active_sonar"

    SL_PING = 220.0
    DI = 15.0
    DT = 15.0
    ALPHA = 2.5e-4

    def __init__(self, sim, seed=None):
        super().__init__(sim)
        self.rng = np.random.default_rng(seed)
        self._ping_requested = False

    def ping(self):
        self._ping_requested = True

    def update(self, dt: float) -> None:
        if not self._ping_requested:
            return
        self._ping_requested = False

        echoes = []
        g = _geometry(self.sim)
        if g is not None:
            own, pos, r, brg = g
            cid = self.get("contacts.cid")
            nl = self.get("sonar.noise_level", 75.0)
            tl = transmission_loss(r, self.ALPHA, own[2], pos[:, 2],
                                   self.get("env.layer_depth", 60.0))
            snr = self.SL_PING - 2.0 * tl + self.get("contacts.ts") - nl \
                + self.DI - self.DT
            ok = np.flatnonzero(snr > 0.0)
            r_sig = np.maximum(0.005 * r, 15.0) / (1.0 + snr / 20.0)
            b_sig = np.clip(3.0 / (1.0 + snr / 10.0), 0.2, 3.0) * DEG
            n_r = self.rng.normal(0.0, 1.0, len(r))
            n_b = self.rng.normal(0.0, 1.0, len(r))
            echoes = [dict(cid=int(cid[i]),
                           range=float(r[i] + n_r[i] * r_sig[i]),
                           bearing=float(brg[i] + n_b[i] * b_sig[i]),
                           snr=float(snr[i])) for i in ok]

        r_cd = solve_range_for_tl(self.SL_PING - (65.0 - 15.0 + 10.0),
                                  self.ALPHA)
        self.put("sonar.echoes", echoes)
        self.put("sonar.ping_time", self.sim.time)
        self.put("sonar.ping_counter_detection_m", float(r_cd))
