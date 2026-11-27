"""3D viewer (pyqtgraph OpenGL) with streamed terrain and a QPainter HUD.

Physics is NED (x north, y east, z down); the GL scene is (x east,
y north, z up). Everything is drawn relative to a floating origin that
re-centres as the boat moves, so float32 GL precision never degrades
and the world has no edge: terrain tiles stream in around the boat,
the sea surface, grid and range rings travel with it.
"""

from __future__ import annotations

import math
from collections import OrderedDict

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5 import QtCore, QtGui

from submarine_toolbox.dynamics.rigid_body import rotation_body_to_ned
from submarine_toolbox.gui import hud

_P = np.array([[0.0, 1.0, 0.0],
               [1.0, 0.0, 0.0],
               [0.0, 0.0, -1.0]])

CONTACT_COLORS = {
    "merchant": (1.0, 0.62, 0.2, 1.0),
    "warship": (1.0, 0.3, 0.3, 1.0),
    "fishing": (0.95, 0.9, 0.35, 1.0),
    "submarine": (0.55, 0.65, 1.0, 1.0),
}

CAMERA_MODES = ("Chase", "Orbit", "Top", "Free")

def _gl(ne_d):
    """NED -> GL axes (rows or single vector)."""
    a = np.asarray(ne_d, dtype=float)
    return np.stack([a[..., 1], a[..., 0], -a[..., 2]], axis=-1)

# ---------------------------------------------------------------- meshes
def _revolve(xs, rs, n_theta=48):
    th = np.linspace(0, 2 * np.pi, n_theta, endpoint=False)
    xs, rs = np.asarray(xs), np.asarray(rs)
    v = np.stack([np.repeat(xs, n_theta),
                  np.outer(rs, np.cos(th)).ravel(),
                  np.outer(rs, np.sin(th)).ravel()], axis=1)
    i, j = np.meshgrid(np.arange(len(xs) - 1), np.arange(n_theta),
                       indexing="ij")
    a = (i * n_theta + j).ravel()
    b = (i * n_theta + (j + 1) % n_theta).ravel()
    c, d = a + n_theta, b + n_theta
    f = np.concatenate([np.stack([a, b, c], 1), np.stack([b, d, c], 1)])
    return v, f

def _hull_profile(L, R, n=60):
    Lt, Ln = 0.24 * L, 0.14 * L
    s = np.linspace(0, 1, n // 2)
    xt = -L / 2 + Lt * s
    rt = R * np.sqrt(np.clip(1 - (1 - s) ** 2.2, 0, 1)) * (0.12 + 0.88 * s ** 0.35)
    rt[0] = 0.0
    s2 = np.linspace(0, 1, n // 3)
    xn = L / 2 - Ln + Ln * s2
    rn = R * np.sqrt(np.clip(1 - s2 ** 2.4, 0, 1))
    return (np.concatenate([xt, [L / 2 - Ln - 1e-3], xn]),
            np.concatenate([rt, [R], rn]))

def _prism(outline_xy, z0, z1):
    """Vertical prism from a convex outline (x, y) between z0 and z1."""
    o = np.asarray(outline_xy)
    n = len(o)
    v = np.vstack([np.column_stack([o, np.full(n, z0)]),
                   np.column_stack([o, np.full(n, z1)])])
    k = np.arange(n)
    k1 = (k + 1) % n
    side = np.concatenate([np.stack([k, k1, k + n], 1),
                           np.stack([k1, k1 + n, k + n], 1)])
    cap = np.stack([np.full(n - 2, n), n + k[1:-1], n + k[2:]], 1)
    return v, np.vstack([side, cap])

def _fin(x0, x1, span0, span1, chord_tip, t, axis):
    """Tapered fin; axis 'y' (horizontal) or 'z' (vertical)."""
    pts = np.array([[x0, span0], [x1, span0],
                    [x0 + chord_tip, span1], [x0, span1]])
    v = []
    for off in (-t, t):
        for px, ps in pts:
            v.append([px, ps, off] if axis == "y" else [px, off, ps])
    v = np.array(v)
    f = np.array([[0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6],
                  [0, 4, 5], [0, 5, 1], [1, 5, 6], [1, 6, 2],
                  [2, 6, 7], [2, 7, 3], [3, 7, 4], [3, 4, 0]])
    return v, f

def boat_mesh(L, R):
    parts = []
    v, f = _revolve(*_hull_profile(L, R))
    parts.append((v, f, None))
    th = np.linspace(-np.pi / 2, np.pi / 2, 10)
    hw, sl = 0.9, 8.5
    xs0 = 0.12 * L
    nose = np.column_stack([xs0 + sl / 2 + hw * np.cos(th), hw * np.sin(th)])
    tail = np.column_stack([xs0 - sl / 2 - 2.2 * hw * np.cos(th[::-1]),
                            hw * np.sin(th[::-1])])
    sail = np.vstack([nose, tail])
    parts.append((*_prism(sail, -R + 0.4, -R - 5.0), None))
    for sgn in (-1, 1):
        parts.append((*_fin(xs0 + 0.5, xs0 + 3.2, sgn * hw, sgn * 3.4, 2.0,
                            0.12, "y"), -R - 3.2))
    xf = -L / 2 + 2.0
    for sgn in (-1, 1):
        parts.append((*_fin(xf, xf + 5.5, sgn * 0.8 * R, sgn * 6.0, 2.6,
                            0.15, "y"), None))
        parts.append((*_fin(xf, xf + 5.5, sgn * 0.8 * R, sgn * 5.5, 2.6,
                            0.15, "z"), None))
    hv, hf = _revolve([-L / 2 - 1.6, -L / 2 - 0.9, -L / 2 + 0.4],
                      [0.0, 0.9, 0.5], 18)
    parts.append((hv, hf, None))

    verts, faces, off = [], [], 0
    for v, f, dz in parts:
        v = v.copy()
        if dz is not None:
            v[:, 2] += dz
        verts.append(v)
        faces.append(f + off)
        off += len(v)
    verts = np.vstack(verts)
    faces = np.vstack(faces)
    tri = verts[faces]
    fn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    vn = np.zeros_like(verts)
    for k in range(3):
        np.add.at(vn, faces[:, k], fn)
    vn /= np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-12)
    light = np.array([0.35, -0.45, -0.82])
    light /= np.linalg.norm(light)
    lam = np.clip(vn @ light, 0.0, 1.0)
    rim = np.clip(1.0 - np.abs(vn[:, 2]), 0.0, 1.0) ** 3
    base = np.array([0.30, 0.33, 0.37])
    col = base[None, :] * (0.38 + 0.95 * lam[:, None]) \
        + np.array([0.10, 0.22, 0.30]) * rim[:, None]
    col = np.column_stack([np.clip(col, 0, 1), np.ones(len(col))])
    return verts, faces, col

# --------------------------------------------------------------- terrain
_PAL_D = np.array([60.0, 150.0, 300.0, 450.0, 650.0, 900.0])
_PAL_C = np.array([[0.72, 0.66, 0.46],
                   [0.42, 0.52, 0.44],
                   [0.18, 0.38, 0.46],
                   [0.10, 0.25, 0.38],
                   [0.06, 0.15, 0.27],
                   [0.04, 0.09, 0.17]])
_LIGHT = np.array([-0.45, 0.35, 0.82])
_LIGHT /= np.linalg.norm(_LIGHT)

def terrain_colors(depth, nx, ny, relief=5.0):
    """Depth palette x baked hillshade (normals exaggerated by `relief`)."""
    c = np.stack([np.interp(depth, _PAL_D, _PAL_C[:, k]) for k in range(3)], -1)
    nx, ny = relief * nx, relief * ny
    nz = 1.0 / np.sqrt(1.0 + nx * nx + ny * ny)
    lam = (nx * _LIGHT[0] + ny * _LIGHT[1] + _LIGHT[2]) * nz
    shade = 0.30 + 0.95 * np.clip(lam, 0.0, 1.0)
    rgb = np.clip(c * shade[..., None], 0, 1)
    return np.concatenate([rgb, np.ones(depth.shape + (1,))], -1)

class TerrainTiles:
    """Streams square terrain tiles around the boat (LRU mesh cache)."""

    def __init__(self, view, floor_fn, tile=1000.0, res=36, radius=3,
                 cache=160):
        self.view = view
        self.floor_fn = floor_fn
        self.tile = tile
        self.res = res
        self.radius = radius
        self.items: dict = {}
        self.cache: OrderedDict = OrderedDict()
        self.cache_max = cache
        r = res
        i, j = np.meshgrid(np.arange(r), np.arange(r), indexing="ij")
        a = (i * (r + 1) + j).ravel()
        b, c = a + 1, a + (r + 1)
        d = c + 1
        self.faces = np.concatenate([np.stack([a, c, b], 1),
                                     np.stack([b, c, d], 1)]).astype(np.uint32)
        self.origin = None

    def _mesh(self, key):
        md = self.cache.pop(key, None)
        if md is None:
            ti, tj = key
            T, r = self.tile, self.res
            h = T / r
            g = np.arange(-1, r + 2) * h
            n = ti * T + g
            e = tj * T + g
            N, E = np.meshgrid(n, e, indexing="ij")
            D = self.floor_fn(N, E)
            dn = (D[2:, 1:-1] - D[:-2, 1:-1]) / (2 * h)
            de = (D[1:-1, 2:] - D[1:-1, :-2]) / (2 * h)
            Dc = D[1:-1, 1:-1]
            col = terrain_colors(Dc, de, dn).reshape(-1, 4)
            gi = np.arange(r + 1) * h
            Ny, Ex = np.meshgrid(gi, gi, indexing="ij")
            verts = np.column_stack([Ex.ravel(), Ny.ravel(), -Dc.ravel()])
            md = gl.MeshData(vertexes=verts, faces=self.faces,
                             vertexColors=col)
        self.cache[key] = md
        while len(self.cache) > self.cache_max:
            self.cache.popitem(last=False)
        return md

    def _place(self, key, item):
        ti, tj = key
        item.resetTransform()
        item.translate(tj * self.tile - self.origin[1],
                       ti * self.tile - self.origin[0], 0.0)

    def clear(self):
        for it in self.items.values():
            self.view.removeItem(it)
        self.items.clear()
        self.cache.clear()

    def update(self, own_ne, origin, budget=4):
        rebased = self.origin is None or origin != self.origin
        self.origin = origin
        T = self.tile
        ci, cj = int(math.floor(own_ne[0] / T)), int(math.floor(own_ne[1] / T))
        R = self.radius
        want = {(ci + a, cj + b) for a in range(-R, R + 1)
                for b in range(-R, R + 1) if a * a + b * b <= (R + 0.5) ** 2}
        for k in [k for k in self.items if k not in want]:
            self.view.removeItem(self.items.pop(k))
        if rebased:
            for k, it in self.items.items():
                self._place(k, it)
        missing = sorted((k for k in want if k not in self.items),
                         key=lambda k: (k[0] - ci) ** 2 + (k[1] - cj) ** 2)
        for k in missing[:budget]:
            it = gl.GLMeshItem(meshdata=self._mesh(k), smooth=True,
                               shader=None, computeNormals=False,
                               glOptions="opaque")
            self._place(k, it)
            self.view.addItem(it)
            self.items[k] = it
        return len(missing) - min(len(missing), budget)

# ---------------------------------------------------------------- viewer
class Viewer3D(gl.GLViewWidget):
    TRAIL_LEN = 2000
    TRAIL_STEP = 3.0
    REBASE = 2000.0

    def __init__(self, sim, parent=None):
        super().__init__(parent)
        self.sim = sim
        self.params = sim.params
        self.env = sim.get_subsystem("environment")
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.camera_mode = "Chase"
        self._az_offset = 0.0
        self.hud_state: dict = {}
        self.show_help = False
        self.show_labels = True
        self.toast = ("", 0.0)
        self.setCameraPosition(distance=190, elevation=12, azimuth=-90)
        self.opts["fov"] = 55

        self.origin = (0.0, 0.0)
        floor_fn = (self.env.floor_at if self.env is not None
                    else lambda n, e: np.full(np.shape(n), 400.0))
        self.terrain = TerrainTiles(self, floor_fn)

        self.water = gl.GLMeshItem(
            vertexes=np.array([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0]],
                              float) * 9000.0,
            faces=np.array([[0, 1, 2], [0, 2, 3]]),
            color=(0.35, 0.65, 0.80, 0.13), smooth=False, shader=None,
            computeNormals=False, glOptions="translucent")
        self.water.setDepthValue(10)
        self.addItem(self.water)

        self.grid = gl.GLGridItem()
        self.grid.setSize(6000, 6000)
        self.grid.setSpacing(250, 250)
        self.grid.setColor((110, 170, 220, 45))
        self.grid.setDepthValue(11)
        self.addItem(self.grid)

        th = np.linspace(0, 2 * np.pi, 121)
        ring = []
        for rr in (1000.0, 2000.0, 5000.0):
            p = np.column_stack([rr * np.cos(th), rr * np.sin(th), np.zeros_like(th)])
            ring.append(np.repeat(p, 2, axis=0)[1:-1])
        self._ring_pts = np.vstack(ring)
        self.rings = gl.GLLinePlotItem(pos=self._ring_pts, mode="lines",
                                       color=(0.55, 0.8, 1.0, 0.25), width=1)
        self.rings.setDepthValue(12)
        self.addItem(self.rings)

        v, f, c = boat_mesh(self.params.L, self.params.R)
        md = gl.MeshData(vertexes=v, faces=f, vertexColors=c)
        self.boat = gl.GLMeshItem(meshdata=md, smooth=True, shader=None,
                                  computeNormals=False, glOptions="opaque")
        self.addItem(self.boat)

        self.trail = np.zeros((0, 3))
        self.trail_item = gl.GLLinePlotItem(width=2, antialias=True)
        self.addItem(self.trail_item)
        self.drop = gl.GLLinePlotItem(width=1.5, antialias=True, mode="lines")
        self.addItem(self.drop)
        self.course = gl.GLLinePlotItem(width=2, antialias=True, mode="lines")
        self.addItem(self.course)
        self.contacts = gl.GLScatterPlotItem(size=11, pxMode=True)
        self.contacts.setGLOptions("translucent")
        self.addItem(self.contacts)
        self._contact_ne = np.zeros((0, 3))
        self._contact_meta: list = []

    # --------------------------------------------------------- camera
    def cycle_camera(self):
        i = CAMERA_MODES.index(self.camera_mode)
        self.set_camera_mode(CAMERA_MODES[(i + 1) % len(CAMERA_MODES)])
        return self.camera_mode

    def set_camera_mode(self, mode):
        self.camera_mode = mode
        if mode == "Top":
            self.setCameraPosition(elevation=89.9, azimuth=-90,
                                   distance=max(self.opts["distance"], 900))
        elif mode == "Chase":
            self._az_offset = 0.0
            self.setCameraPosition(elevation=12,
                                   distance=min(self.opts["distance"], 400))

    def reset_camera(self):
        self._az_offset = 0.0
        self.setCameraPosition(distance=190, elevation=12)
        if self.camera_mode == "Top":
            self.setCameraPosition(elevation=89.9, azimuth=-90, distance=900)

    def _chase_azimuth(self, psi):
        return math.degrees(math.atan2(-math.cos(psi), -math.sin(psi)))

    def mouseMoveEvent(self, ev):
        super().mouseMoveEvent(ev)
        if self.camera_mode == "Chase":
            psi = float(self.sim.eta[5])
            d = self.opts["azimuth"] - self._chase_azimuth(psi)
            self._az_offset = (d + 180.0) % 360.0 - 180.0
        elif self.camera_mode == "Top":
            self.opts["elevation"] = 89.9

    def wheelEvent(self, ev):
        super().wheelEvent(ev)
        self.opts["distance"] = float(np.clip(self.opts["distance"], 25.0, 20000.0))

    # ---------------------------------------------------------- frame
    def _rebase(self, own):
        o = self.origin
        if abs(own[0] - o[0]) > self.REBASE or abs(own[1] - o[1]) > self.REBASE:
            T = self.terrain.tile
            self.origin = (round(own[0] / T) * T, round(own[1] / T) * T)

    def _local(self, ned):
        a = np.array(ned, dtype=float)
        a[..., 0] -= self.origin[0]
        a[..., 1] -= self.origin[1]
        return _gl(a)

    def frame(self, hud_state: dict):
        self.hud_state = hud_state
        sim, bus = self.sim, self.sim.bus
        eta = sim.eta
        self._rebase(eta)
        self.terrain.update(eta[:2], self.origin)
        pos = self._local(eta[:3])

        M = np.eye(4)
        M[:3, :3] = _P @ rotation_body_to_ned(eta[3], eta[4], eta[5])
        M[:3, 3] = pos
        self.boat.setTransform(pg.Transform3D(*M.flatten()))

        for it in (self.water, self.rings):
            it.resetTransform()
            it.translate(pos[0], pos[1], 0.0)
        sp = 250.0
        self.grid.resetTransform()
        self.grid.translate(round(pos[0] / sp) * sp, round(pos[1] / sp) * sp, 0.0)

        self._update_trail(eta)
        floor = bus.get("env.seafloor_depth", 400.0)
        clear = floor - eta[2]
        c = (1.0, 0.35, 0.3, 0.9) if clear < 40 else \
            (1.0, 0.8, 0.3, 0.7) if clear < 100 else (0.6, 0.85, 1.0, 0.45)
        self.drop.setData(pos=np.array([pos, [pos[0], pos[1], -floor]]), color=c)

        if bus.get("autopilot.enabled", True):
            h = bus.get("order.heading", 0.0)
            seg = np.linspace(0.0, 600.0, 13)
            pts = np.column_stack([pos[0] + seg * math.sin(h),
                                   pos[1] + seg * math.cos(h),
                                   np.full_like(seg, -bus.get("order.depth", eta[2]))])
            self.course.setData(pos=pts[:12], color=(0.3, 0.95, 1.0, 0.55))
            self.course.setVisible(True)
        else:
            self.course.setVisible(False)

        truth = bus.get("contacts.truth") or []
        if truth:
            ned = np.array([t["pos"] for t in truth])
            self._contact_ne = ned
            self._contact_meta = [(t["cid"], t["ctype"]) for t in truth]
            col = np.array([CONTACT_COLORS.get(t["ctype"], (1, 1, 1, 1))
                            for t in truth])
            self.contacts.setData(pos=self._local(ned), color=col)

        self._update_camera(pos, eta[5])
        self._update_background(pos)
        self.update()

    def _update_trail(self, eta):
        p = np.asarray(eta[:3], float)
        if not len(self.trail) or np.linalg.norm(self.trail[-1] - p) > self.TRAIL_STEP:
            self.trail = np.vstack([self.trail[-(self.TRAIL_LEN - 1):], p])
        if len(self.trail) > 1:
            pts = self._local(np.vstack([self.trail, p]))
            a = np.linspace(0.0, 0.85, len(pts)) ** 1.5
            col = np.column_stack([0.25 + 0 * a, 0.95 + 0 * a, 0.7 + 0 * a, a])
            self.trail_item.setData(pos=pts, color=col)

    def _update_camera(self, pos, psi):
        mode = self.camera_mode
        if mode == "Free":
            return
        self.opts["center"] = pg.Vector(*pos)
        if mode == "Chase":
            target = self._chase_azimuth(psi) + self._az_offset
            cur = self.opts["azimuth"]
            d = (target - cur + 180.0) % 360.0 - 180.0
            self.opts["azimuth"] = cur + 0.25 * d
        if mode in ("Chase", "Orbit") and pos[2] < -12.0:
            s = (-pos[2] - 4.0) / self.opts["distance"]
            if s < 1.0:
                lim = math.degrees(math.asin(s))
                self.opts["elevation"] = min(self.opts["elevation"], lim)

    def _update_background(self, pos):
        cam = self.cameraPosition()
        z = -cam.z()
        if z < 0:
            c = (16, 26, 40)
        else:
            k = min(z / 300.0, 1.0)
            c = tuple(int(a + (b - a) * k) for a, b in ((18, 4), (52, 10), (78, 20)))
        self.setBackgroundColor(c)

    # ------------------------------------------------------------ HUD
    def project(self, ned):
        """NED points -> (screen xy, in_front mask)."""
        if len(ned) == 0:
            return np.zeros((0, 2)), np.zeros(0, bool)
        vp = self.getViewport()
        m = self.projectionMatrix(vp, vp) * self.viewMatrix()
        M = np.array(m.data()).reshape(4, 4).T
        p = self._local(ned)
        h = np.column_stack([p, np.ones(len(p))]) @ M.T
        w = h[:, 3]
        ok = w > 1e-6
        ndc = h[:, :2] / np.where(ok, w, 1.0)[:, None]
        sx = (ndc[:, 0] + 1) * 0.5 * self.width()
        sy = (1 - ndc[:, 1]) * 0.5 * self.height()
        return np.column_stack([sx, sy]), ok

    def paintGL(self):
        super().paintGL()
        p = QtGui.QPainter(self)
        try:
            hud.draw(p, self)
        finally:
            p.end()
