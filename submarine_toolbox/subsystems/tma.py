"""Bearing-only target motion analysis.

Collects (time, own-position estimate, measured bearing) samples for the
operator-selected passive contact and batch-fits a constant-velocity
solution X = [N0, E0, vN, vE] by Levenberg-Marquardt on the bearing
residuals. Multiple range hypotheses initialize the solver. Own-ship
maneuvers across legs are required for observability, exactly as at sea.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from submarine_toolbox.core.subsystem import Subsystem

def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def solve_bearings_lm(ts, own_ne, bearings, x0, iters=400, lam=1e-3):
    """LM fit of constant-velocity track(s) to a bearing series.

    x0 may be (4,) or a batch (K, 4) of initializations; the K problems
    are solved simultaneously. Returns (X, rms_rad) with matching shape.
    """
    X = np.array(x0, dtype=float, ndmin=2)
    single = np.ndim(x0) == 1
    K = len(X)
    dt = ts - ts[0]
    on, oe = own_ne[:, 0], own_ne[:, 1]
    eye = np.eye(4)

    def residuals(X):
        N = X[:, :1] + X[:, 2:3] * dt - on
        E = X[:, 1:2] + X[:, 3:4] * dt - oe
        return wrap_pi(bearings - np.arctan2(E, N)), N, E

    e, N, E = residuals(X)
    cost = (e * e).sum(1)
    lam = np.full(K, float(lam))
    active = np.arange(K)
    for _ in range(iters):
        if not len(active):
            break
        a = active
        Na, Ea, ea, la = N[a], E[a], e[a], lam[a]
        rho2 = np.maximum(Na * Na + Ea * Ea, 1.0)
        dN, dE = -Ea / rho2, Na / rho2
        J = np.stack([dN, dE, dN * dt, dE * dt], axis=2)
        Jt = J.transpose(0, 2, 1)
        A = Jt @ J + la[:, None, None] * eye
        g = (Jt @ ea[..., None])[..., 0]
        try:
            step = np.linalg.solve(A, g[..., None])[..., 0]
        except np.linalg.LinAlgError:
            break
        Xn = X[a] + step
        en, Nn, En = residuals(Xn)
        cn = (en * en).sum(1)
        ca = cost[a]
        better = cn < ca
        conv = better & (ca - cn < 1e-12 * np.maximum(ca, 1e-30))
        b = a[better]
        X[b], e[b], N[b], E[b], cost[b] = \
            Xn[better], en[better], Nn[better], En[better], cn[better]
        la = np.where(better, np.maximum(la * 0.3, 1e-9),
                      np.minimum(la * 4.0, 1e6))
        lam[a] = la
        active = a[~conv & (better | (la < 1e6))]
    rms = np.sqrt(cost / e.shape[1])
    if single:
        return X[0], float(rms[0])
    return X, rms

class TMA(Subsystem):
    name = "tma"

    SAMPLE_PERIOD = 2.0
    SOLVE_PERIOD = 5.0
    MIN_SAMPLES = 25
    MAX_SPEED = 15.0
    INIT_RANGES = (1e3, 2e3, 4e3, 8e3, 16e3, 24e3)
    FULL_EVERY = 6

    def __init__(self, sim):
        super().__init__(sim)
        self.track_cid = None
        self.buffer = deque(maxlen=900)
        self._acc_s = 0.0
        self._acc_v = 0.0
        self._prev = None
        self._n_solve = 0

    def set_track(self, cid):
        if cid != self.track_cid:
            self.track_cid = cid
            self.buffer.clear()
            self._prev = None
            self.put("tma.solution", None)

    def update(self, dt: float) -> None:
        if self.track_cid is None:
            return
        self._acc_s += dt
        self._acc_v += dt

        if self._acc_s >= self.SAMPLE_PERIOD:
            self._acc_s = 0.0
            for c in self.get("sonar.contacts", []):
                if c["cid"] == self.track_cid:
                    own = self.get("nav.est_ne", self.sim.eta[:2])
                    self.buffer.append((self.sim.time,
                                        float(own[0]), float(own[1]),
                                        float(c["bearing"])))
                    break

        if self._acc_v >= self.SOLVE_PERIOD and \
                len(self.buffer) >= self.MIN_SAMPLES:
            self._acc_v = 0.0
            self._solve()

    def _solve(self):
        d = np.array(self.buffer)
        ts, own_ne, brgs = d[:, 0], d[:, 1:3], d[:, 3]

        cands = []
        if self._prev is not None:
            Xp, t0 = self._prev
            cands.append([Xp[0] + Xp[2] * (ts[0] - t0),
                          Xp[1] + Xp[3] * (ts[0] - t0), Xp[2], Xp[3]])
        if self._prev is None or self._n_solve % self.FULL_EVERY == 0:
            r0 = np.asarray(self.INIT_RANGES)
            x0 = np.column_stack([own_ne[0, 0] + r0 * np.cos(brgs[0]),
                                  own_ne[0, 1] + r0 * np.sin(brgs[0]),
                                  0.0 * r0, 0.0 * r0])
            X, rms = solve_bearings_lm(ts, own_ne, brgs, x0, iters=60)
            rms = np.where(np.hypot(X[:, 2], X[:, 3]) < self.MAX_SPEED,
                           rms, np.inf)
            if np.isfinite(rms).any():
                cands.append(X[int(np.argmin(rms))])
        self._n_solve += 1
        if not cands:
            return
        X, rms = solve_bearings_lm(ts, own_ne, brgs, np.array(cands))
        rms = np.where(np.hypot(X[:, 2], X[:, 3]) < self.MAX_SPEED, rms, np.inf)
        k = int(np.argmin(rms))
        if not np.isfinite(rms[k]):
            self._prev = None
            return
        best, best_rms = X[k], float(rms[k])
        self._prev = (best.copy(), ts[0])
        t_now = self.sim.time
        pos = np.array([best[0] + best[2] * (t_now - ts[0]),
                        best[1] + best[3] * (t_now - ts[0])])
        own_now = np.asarray(self.get("nav.est_ne", self.sim.eta[:2]))
        rel = pos - own_now
        self.put("tma.solution", dict(
            cid=self.track_cid,
            range=float(np.hypot(*rel)),
            bearing=float(np.arctan2(rel[1], rel[0])),
            course=float(np.arctan2(best[3], best[2])),
            speed=float(np.hypot(best[2], best[3])),
            rms_deg=float(np.degrees(best_rms)),
            n_samples=len(self.buffer),
            leg_span_s=float(ts[-1] - ts[0])))
