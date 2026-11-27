"""QPainter overlay for the 3D view: nav block, compass tape, depth
gauge, alerts, contact labels, toasts and the help card."""

from __future__ import annotations

import math
import time

from PyQt5 import QtCore, QtGui

from submarine_toolbox.gui.controls import help_rows

KN = 0.5144
FG = QtGui.QColor(210, 225, 240)
DIM = QtGui.QColor(130, 150, 175)
CYAN = QtGui.QColor(90, 230, 255)
AMBER = QtGui.QColor(255, 190, 60)
RED = QtGui.QColor(255, 85, 80)
GREEN = QtGui.QColor(110, 230, 140)
BG = QtGui.QColor(6, 12, 22, 175)
EDGE = QtGui.QColor(120, 170, 220, 70)

_fonts: dict = {}

def font(size, bold=False):
    k = (size, bold)
    if k not in _fonts:
        f = QtGui.QFont("Consolas")
        f.setStyleHint(QtGui.QFont.Monospace)
        f.setPointSizeF(size)
        f.setBold(bold)
        _fonts[k] = f
    return _fonts[k]

def panel(p, rect, radius=6):
    p.setPen(QtGui.QPen(EDGE, 1))
    p.setBrush(BG)
    p.drawRoundedRect(rect, radius, radius)

def text(p, x, y, s, color=FG, f=None):
    if f is not None:
        p.setFont(f)
    p.setPen(color)
    p.drawText(QtCore.QPointF(x, y), s)

def _hms(t):
    t = int(t)
    return f"{t // 3600:02d}:{t // 60 % 60:02d}:{t % 60:02d}"

def alerts(bus):
    out = []
    depth = bus.get("state.depth", 0.0)
    clear = bus.get("env.seafloor_depth", 1e9) - depth
    ahead = bus.get("env.min_floor_ahead", 1e9) - max(depth, bus.get("order.depth", depth))
    hull = bus.get("damage.hull_state", "OK")
    if bus.get("state.grounded"):
        out.append((RED, "GROUNDED"))
    if hull == "FAILING":
        out.append((RED, "HULL FAILING — BEYOND CRUSH DEPTH"))
    elif hull != "OK":
        out.append((AMBER, hull))
    if clear < 40:
        out.append((RED, f"SHOAL — {clear:.0f} m UNDER KEEL"))
    elif ahead < 60:
        out.append((AMBER, f"TERRAIN AHEAD — {max(ahead, 0):.0f} m margin"))
    if bus.get("ballast.blowing"):
        out.append((AMBER, "BLOWING MAIN BALLAST"))
    if bus.get("propulsion.cavitating"):
        out.append((AMBER, "CAVITATING"))
    if bus.get("damage.any_fire"):
        out.append((RED, "FIRE"))
    if any(c["breach"] for c in bus.get("damage.status", [])):
        out.append((RED, "FLOODING"))
    if bus.get("power.dead"):
        out.append((RED, "BATTERY DEPLETED"))
    elif bus.get("power.shed"):
        out.append((AMBER, "LOW BATTERY — LOAD SHED"))
    st = bus.get("atmo.status", "NORMAL")
    if st != "NORMAL":
        out.append((RED if st == "DANGER" else AMBER, f"ATMOSPHERE {st}"))
    if bus.get("hydraulics.leaking"):
        out.append((AMBER, "HYDRAULIC LEAK"))
    if bus.get("radar.counter_detected_by"):
        out.append((AMBER, "RADAR INTERCEPTED BY ESM"))
    return out

def draw(p, view):
    p.setRenderHint(QtGui.QPainter.Antialiasing)
    p.setRenderHint(QtGui.QPainter.TextAntialiasing)
    bus = view.sim.bus
    st = view.hud_state
    W, H = view.width(), view.height()
    if view.show_labels:
        _labels(p, view)
    _nav(p, bus, st)
    _compass(p, bus, W)
    _depth_gauge(p, view, bus, W, H)
    _footer(p, view, st, W, H)
    msg, until = view.toast
    if msg and time.monotonic() < until:
        p.setFont(font(12, True))
        fm = QtGui.QFontMetrics(p.font())
        w = fm.horizontalAdvance(msg) + 28
        r = QtCore.QRectF((W - w) / 2, H - 92, w, 34)
        panel(p, r, 8)
        p.setPen(FG)
        p.drawText(r, QtCore.Qt.AlignCenter, msg)
    if view.show_help:
        _help(p, W, H)

def _nav(p, bus, st):
    auto = bus.get("autopilot.enabled", True)
    hdg = math.degrees(bus.get("state.heading", 0.0)) % 360
    z = bus.get("state.depth", 0.0)
    u = bus.get("state.u", 0.0) / KN
    floor = bus.get("env.seafloor_depth", 0.0)
    rows = [
        ("HDG", f"{hdg:05.1f}°", f"{math.degrees(bus.get('order.heading', 0)) % 360:05.1f}°"),
        ("DEPTH", f"{z:6.1f} m", f"{bus.get('order.depth', 0):5.0f} m"),
        ("SPEED", f"{u:5.1f} kn", f"{bus.get('order.speed', 0) / KN:5.1f} kn"),
    ]
    if not auto:
        rows = [
            ("HDG", f"{hdg:05.1f}°", f"rud {math.degrees(bus.get('act.rudder', 0)):+4.0f}°"),
            ("DEPTH", f"{z:6.1f} m", f"pln {math.degrees(bus.get('act.planes', 0)):+4.0f}°"),
            ("SPEED", f"{u:5.1f} kn", f"{bus.get('order.rpm', 0):+4.0f} rpm"),
        ]
    al = alerts(bus)
    h = 150 + 20 * len(al)
    panel(p, QtCore.QRectF(12, 12, 268, h))
    text(p, 24, 36, "AUTOPILOT" if auto else "MANUAL", GREEN if auto else AMBER,
         font(12, True))
    text(p, 140, 36, "order" if auto else "actuator", DIM, font(9))
    p.setFont(font(11))
    y = 60
    for lab, a, b in rows:
        text(p, 24, y, lab, DIM)
        text(p, 84, y, a, FG)
        text(p, 182, y, b, CYAN)
        y += 21
    keel = floor - z
    text(p, 24, y, "KEEL", DIM)
    text(p, 84, y, f"{keel:6.0f} m", RED if keel < 40 else AMBER if keel < 100 else FG)
    text(p, 182, y, f"pitch {math.degrees(bus.get('state.pitch', 0)):+3.0f}°", DIM, font(9))
    y += 21
    t = bus.get("time", 0.0)
    scale = st.get("time_scale", 1)
    text(p, 24, y, _hms(t), FG, font(11))
    tag = "PAUSED" if st.get("paused") else f"×{scale}"
    text(p, 120, y, tag, AMBER if st.get("paused") or st.get("lagging") else DIM)
    if st.get("lagging") and not st.get("paused"):
        text(p, 170, y, "sim lagging", AMBER, font(9))
    y += 8
    for col, msg in al:
        y += 20
        blink = col is not RED or int(time.monotonic() * 2.5) % 2 == 0
        text(p, 24, y, ("▲ " if blink else "   ") + msg, col, font(10, True))

def _compass(p, bus, W):
    h = 44
    w = max(min(460, W - 292 - 120), 200)
    x0 = max((W - w) / 2, 292)
    r = QtCore.QRectF(x0, 10, w, h)
    panel(p, r)
    hdg = math.degrees(bus.get("state.heading", 0.0)) % 360
    span = 120.0
    ppd = w / span
    p.save()
    p.setClipRect(r.adjusted(4, 0, -4, 0))
    p.setFont(font(9))
    lo = int(math.floor((hdg - span / 2) / 5.0)) * 5
    for d in range(lo, lo + int(span) + 10, 5):
        x = x0 + w / 2 + (d - hdg) * ppd
        dd = d % 360
        major = dd % 30 == 0
        p.setPen(QtGui.QPen(FG if major else DIM, 1))
        p.drawLine(QtCore.QPointF(x, 10 + h - (14 if major else 7)),
                   QtCore.QPointF(x, 10 + h - 2))
        if major and r.left() + 18 < x < r.right() - 18:
            lab = {0: "N", 90: "E", 180: "S", 270: "W"}.get(dd, f"{dd:03d}")
            p.setPen(CYAN if len(lab) == 1 else FG)
            p.drawText(QtCore.QRectF(x - 20, 12, 40, 16), QtCore.Qt.AlignCenter, lab)
    if bus.get("autopilot.enabled", True):
        od = math.degrees(bus.get("order.heading", 0.0)) % 360
        dx = (od - hdg + 180) % 360 - 180
        x = x0 + w / 2 + max(-span / 2, min(span / 2, dx)) * ppd
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(CYAN)
        p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(x - 6, 10 + h),
                                       QtCore.QPointF(x + 6, 10 + h),
                                       QtCore.QPointF(x, 10 + h - 9)]))
    p.restore()
    p.setPen(QtGui.QPen(AMBER, 2))
    p.drawLine(QtCore.QPointF(x0 + w / 2, 8), QtCore.QPointF(x0 + w / 2, 10 + h + 4))
    text(p, x0 + w / 2 - 22, 10 + h + 18, f"{hdg:05.1f}°", AMBER, font(10, True))

def _depth_gauge(p, view, bus, W, H):
    prm = view.params
    floor = bus.get("env.seafloor_depth", 400.0)
    zmax = max(floor, prm.crush_depth) * 1.08
    x, y0 = W - 92, 70
    h = max(H * 0.55, 200)
    panel(p, QtCore.QRectF(x - 12, y0 - 26, 92, h + 46))
    text(p, x - 2, y0 - 8, "DEPTH", DIM, font(9))

    def Y(z):
        return y0 + h * min(max(z, 0.0), zmax) / zmax

    p.setPen(QtCore.Qt.NoPen)
    grad = QtGui.QLinearGradient(0, y0, 0, y0 + h)
    grad.setColorAt(0, QtGui.QColor(40, 110, 160, 160))
    grad.setColorAt(1, QtGui.QColor(5, 20, 45, 160))
    p.setBrush(grad)
    p.drawRect(QtCore.QRectF(x, y0, 18, h))
    p.setBrush(QtGui.QColor(150, 120, 80, 200))
    p.drawRect(QtCore.QRectF(x, Y(floor), 18, y0 + h - Y(floor)))
    p.setFont(font(8))
    for z, col, lab in ((bus.get("env.layer_depth", 60.0), DIM, "layer"),
                        (prm.test_depth, AMBER, "test"),
                        (prm.crush_depth, RED, "crush")):
        p.setPen(QtGui.QPen(col, 1, QtCore.Qt.DashLine))
        p.drawLine(QtCore.QPointF(x - 4, Y(z)), QtCore.QPointF(x + 22, Y(z)))
        p.setPen(col)
        p.drawText(QtCore.QPointF(x + 25, Y(z) + 3), lab)
    p.setPen(QtGui.QColor(200, 170, 120))
    p.drawText(QtCore.QPointF(x + 25, min(Y(floor) + 12, y0 + h + 12)), f"{floor:.0f}")
    if bus.get("autopilot.enabled", True):
        zo = Y(bus.get("order.depth", 0.0))
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(CYAN)
        p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(x - 10, zo - 5),
                                       QtCore.QPointF(x - 10, zo + 5),
                                       QtCore.QPointF(x - 2, zo)]))
    zb = Y(bus.get("state.depth", 0.0))
    p.setPen(QtGui.QPen(AMBER, 3))
    p.drawLine(QtCore.QPointF(x - 2, zb), QtCore.QPointF(x + 20, zb))

def _labels(p, view):
    ned = view._contact_ne
    if not len(ned):
        return
    xy, ok = view.project(ned)
    own = view.sim.eta
    p.setFont(font(8))
    held = {c["cid"] for c in view.sim.bus.get("sonar.contacts", [])}
    for (sx, sy), good, (cid, ctype), q in zip(xy, ok, view._contact_meta, ned):
        if not good or not (0 <= sx <= view.width() and 80 <= sy <= view.height()):
            continue
        rng = math.hypot(q[0] - own[0], q[1] - own[1]) / 1e3
        col = CYAN if cid in held else DIM
        p.setPen(col)
        p.drawText(QtCore.QPointF(sx + 8, sy - 6), f"#{cid} {ctype} {rng:.1f} km")

def _footer(p, view, st, W, H):
    s = f"H help  ·  C camera: {view.camera_mode}  ·  F2 panels"
    p.setFont(font(9))
    fm = QtGui.QFontMetrics(p.font())
    w = fm.horizontalAdvance(s) + 20
    r = QtCore.QRectF(W - w - 12, H - 34, w, 24)
    panel(p, r, 5)
    p.setPen(DIM)
    p.drawText(r, QtCore.Qt.AlignCenter, s)
    ne = view.sim.eta
    text(p, 16, H - 16, f"N {ne[0] / 1e3:+9.2f} km   E {ne[1] / 1e3:+9.2f} km", DIM, font(9))

def _help(p, W, H):
    groups = list(help_rows().items())
    colw, rowh = 370, 21
    if W >= colw * len(groups) + 60:
        cols = [[g] for g in groups]
    else:
        cols = [groups[:-1], groups[-1:]]
        colw = min(colw, (W - 60) / 2)
    nrows = max(sum(len(v) + 2 for _, v in c) for c in cols)
    w = colw * len(cols) + 40
    h = rowh * nrows + 60
    r = QtCore.QRectF((W - w) / 2, max((H - h) / 2, 8), w, h)
    p.setPen(QtGui.QPen(EDGE, 1))
    p.setBrush(QtGui.QColor(4, 10, 20, 230))
    p.drawRoundedRect(r, 10, 10)
    text(p, r.x() + 22, r.y() + 34, "CONTROLS", CYAN, font(14, True))
    text(p, r.x() + 150, r.y() + 34,
         "Esc closes · click the 3D view if keys stop responding", DIM, font(9))
    for i, col in enumerate(cols):
        x = r.x() + 22 + i * colw
        y = r.y() + 50
        for g, items in col:
            y += rowh + 6
            text(p, x, y, g.upper(), AMBER, font(10, True))
            for k, d in items:
                y += rowh
                text(p, x, y, k, FG, font(10, True))
                text(p, x + 76, y, d, DIM, font(9))
            y += 4
