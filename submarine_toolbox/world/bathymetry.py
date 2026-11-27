"""Procedural, unbounded bathymetry: deterministic hashed value-noise fBm.

depth(n, e) is a pure function of position and seed, so the simulation
and the viewer agree everywhere without storing any terrain.
"""

from __future__ import annotations

import numpy as np

_M = np.uint64(0xFFFFFFFF)

def _hash01(ix, iy, seed):
    with np.errstate(over="ignore"):
        h = (ix.astype(np.uint64) * np.uint64(374761393)
             + iy.astype(np.uint64) * np.uint64(668265263)
             + np.uint64(seed * 2654435761 & 0xFFFFFFFF)) & _M
        h = ((h ^ (h >> np.uint64(13))) * np.uint64(1274126177)) & _M
        h ^= h >> np.uint64(16)
    return h.astype(np.float64) * (1.0 / 4294967296.0)

def value_noise(x, y, seed=0):
    """Smooth value noise in [0, 1) on a unit lattice."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    x0, y0 = np.floor(x), np.floor(y)
    fx, fy = x - x0, y - y0
    ix, iy = x0.astype(np.int64), y0.astype(np.int64)
    sx = fx * fx * fx * (fx * (fx * 6.0 - 15.0) + 10.0)
    sy = fy * fy * fy * (fy * (fy * 6.0 - 15.0) + 10.0)
    v00 = _hash01(ix, iy, seed)
    v10 = _hash01(ix + 1, iy, seed)
    v01 = _hash01(ix, iy + 1, seed)
    v11 = _hash01(ix + 1, iy + 1, seed)
    a = v00 + sx * (v10 - v00)
    b = v01 + sx * (v11 - v01)
    return a + sy * (b - a)

def fbm(x, y, octaves=5, seed=0, lacunarity=2.03, gain=0.5):
    """Fractal sum, normalized to roughly [-1, 1]."""
    out = 0.0
    amp, norm = 1.0, 0.0
    for k in range(octaves):
        out = out + amp * (2.0 * value_noise(x, y, seed + 97 * k) - 1.0)
        norm += amp
        x = x * lacunarity + 17.3
        y = y * lacunarity - 9.1
        amp *= gain
    return out / norm

class Bathymetry:
    def __init__(self, seed=7, mean=480.0, relief=230.0, wavelength=9000.0,
                 seamount_height=300.0, min_depth=45.0):
        self.seed = seed
        self.mean = mean
        self.relief = relief
        self.wavelength = wavelength
        self.seamount_height = seamount_height
        self.min_depth = min_depth

    def depth(self, n, e):
        """Seafloor depth [m, positive down] at NED (n, e); vectorized."""
        n = np.asarray(n, dtype=float)
        e = np.asarray(e, dtype=float)
        L = self.wavelength
        base = fbm(n / L, e / L, 5, self.seed)
        detail = fbm(n / 900.0, e / 900.0, 3, self.seed + 31)
        m = value_noise(n / (1.7 * L) + 0.5, e / (1.7 * L) + 0.5, self.seed + 71)
        m = np.clip((m - 0.62) / 0.3, 0.0, 1.0)
        mount = m * m * (3.0 - 2.0 * m) * self.seamount_height
        d = self.mean - self.relief * base - 18.0 * detail - mount
        return np.maximum(d, self.min_depth)
