"""Ship-control panels: helm/autopilot, ballast, propulsion, power, ship state.

Each panel holds a reference to the Simulation, writes orders into
sim.bus on user input, and refreshes displayed telemetry from sim.bus
on a GUI timer (refresh(sim) called by MainWindow).
"""

from __future__ import annotations

from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

DEG = np.pi / 180.0
KN = 0.5144

def _sync(w, v):
    if w.hasFocus() or (isinstance(w, QtWidgets.QSlider) and w.isSliderDown()):
        return
    if abs(w.value() - v) > 1e-6:
        w.blockSignals(True)
        w.setValue(v)
        w.blockSignals(False)

class History:
    """Fixed-length sample history keyed on sim time."""

    def __init__(self, n, min_dt=0.1):
        self.t = deque(maxlen=n)
        self.v = deque(maxlen=n)
        self.min_dt = min_dt

    def add(self, t, v):
        if not self.t or t - self.t[-1] >= self.min_dt or t < self.t[-1]:
            self.t.append(t)
            self.v.append(v)

    def arrays(self):
        return np.fromiter(self.t, float), np.fromiter(self.v, float)

def _plot(title, pen, invert=False):
    w = pg.PlotWidget(title=title)
    w.setMaximumHeight(160)
    w.showGrid(x=True, y=True, alpha=0.15)
    w.setMouseEnabled(x=False, y=True)
    if invert:
        w.invertY(True)
    return w, w.plot(pen=pg.mkPen(pen, width=1.6))

def _spin(lo, hi, val, step, suffix=""):
    s = QtWidgets.QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setValue(val)
    s.setSingleStep(step)
    s.setSuffix(suffix)
    return s

class HelmPanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        self.sim = sim
        form = QtWidgets.QFormLayout(self)

        self.ap = QtWidgets.QCheckBox("Autopilot engaged")
        self.ap.setChecked(True)
        self.ap.toggled.connect(lambda v: self._put("autopilot.enabled", v))
        form.addRow(self.ap)

        self.hdg = _spin(0, 359.9, 0, 5, " °")
        self.dep = _spin(0, 400, 15, 10, " m")
        self.spd = _spin(0, 18, 5, 1, " kn")
        self.hdg.valueChanged.connect(
            lambda v: self._put("order.heading", np.radians(v)))
        self.dep.valueChanged.connect(lambda v: self._put("order.depth", v))
        self.spd.valueChanged.connect(
            lambda v: self._put("order.speed", v * KN))
        form.addRow("Ordered heading", self.hdg)
        form.addRow("Ordered depth", self.dep)
        form.addRow("Ordered speed", self.spd)

        form.addRow(QtWidgets.QLabel("— Manual (autopilot off) —"))
        self.rud = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.rud.setRange(-30, 30)
        self.pln = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.pln.setRange(-25, 25)
        self.rpm = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.rpm.setRange(-200, 200)
        self.rud.valueChanged.connect(
            lambda v: self._put("order.rudder", v * DEG))
        self.pln.valueChanged.connect(
            lambda v: self._put("order.planes", v * DEG))
        self.rpm.valueChanged.connect(lambda v: self._put("order.rpm", float(v)))
        form.addRow("Rudder [°]", self.rud)
        form.addRow("Planes [°]", self.pln)
        form.addRow("Shaft RPM", self.rpm)

        self.act = QtWidgets.QLabel("")
        form.addRow(self.act)
        keys = QtWidgets.QLabel("Keys: W/S speed · A/D heading · R/F depth · "
                                "M manual · H help")
        keys.setWordWrap(True)
        keys.setStyleSheet("color:#8aa0b8")
        form.addRow(keys)

    def _put(self, k, v):
        self.sim.bus[k] = v

    def refresh(self, sim):
        b = sim.bus
        ap = bool(b.get("autopilot.enabled", True))
        if self.ap.isChecked() != ap:
            self.ap.blockSignals(True)
            self.ap.setChecked(ap)
            self.ap.blockSignals(False)
        _sync(self.hdg, np.degrees(b.get("order.heading", 0.0)) % 360.0)
        _sync(self.dep, b.get("order.depth", 0.0))
        _sync(self.spd, b.get("order.speed", 0.0) / KN)
        _sync(self.rud, int(round(np.degrees(b.get("order.rudder", 0.0)))))
        _sync(self.pln, int(round(np.degrees(b.get("order.planes", 0.0)))))
        _sync(self.rpm, int(round(b.get("order.rpm", 0.0))))
        for w in (self.hdg, self.dep, self.spd):
            w.setEnabled(ap)
        for w in (self.rud, self.pln, self.rpm):
            w.setEnabled(not ap)
        self.act.setText(
            f"rudder {np.degrees(b.get('act.rudder', 0)):+5.1f}°   "
            f"planes {np.degrees(b.get('act.planes', 0)):+5.1f}°")

class BallastPanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        self.sim = sim
        self.bal = sim.get_subsystem("ballast")
        p = sim.params
        form = QtWidgets.QFormLayout(self)

        self.mbt = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.mbt.setRange(0, 100)
        self.mbt.setValue(100)
        self.mbt.valueChanged.connect(
            lambda v: setattr(self.bal, "mbt_target", v / 100.0))
        form.addRow("MBT fill [%]", self.mbt)

        self.var = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.var.setRange(0, int(p.var_capacity))
        self.var.setValue(int(0.5 * p.var_capacity))
        self.var.valueChanged.connect(
            lambda v: setattr(self.bal, "var_target", float(v)))
        form.addRow("Variable ballast [kg]", self.var)

        self.trim = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.trim.setRange(-100, 100)
        self.trim.valueChanged.connect(
            lambda v: setattr(self.bal, "trim_target", v / 100.0))
        form.addRow("Trim (aft ↔ fwd)", self.trim)

        blow = QtWidgets.QPushButton("EMERGENCY BLOW")
        blow.setStyleSheet(
            "background:#a01818;color:white;font-weight:bold;padding:6px")
        blow.clicked.connect(self.bal.emergency_blow)
        form.addRow(blow)

        self.air = QtWidgets.QProgressBar()
        form.addRow("HP air bank", self.air)
        self.status = QtWidgets.QLabel("")
        form.addRow(self.status)

    def refresh(self, sim):
        b = sim.bus
        _sync(self.mbt, int(round(100 * self.bal.mbt_target)))
        _sync(self.var, int(round(self.bal.var_target)))
        _sync(self.trim, int(round(100 * self.bal.trim_target)))
        self.air.setValue(int(100 * b.get("ballast.air", 1)))
        self.status.setText(
            f"MBT {100 * b.get('ballast.mbt', 0):5.1f}%   "
            f"var {b.get('ballast.var', 0):6.0f} kg"
            + ("   ⚠ BLOWING" if b.get("ballast.blowing") else ""))

class PropulsionPanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        lay = QtWidgets.QVBoxLayout(self)
        self.lbl = QtWidgets.QLabel("")
        self.cav = QtWidgets.QLabel("")
        lay.addWidget(self.lbl)
        lay.addWidget(self.cav)
        self.plot, self.curve = _plot("Shaft RPM", "#f0c040")
        self._hist = History(900)
        lay.addWidget(self.plot)
        lay.addStretch()

    def record(self, sim):
        self._hist.add(sim.time, sim.bus.get("propulsion.rpm", 0.0))

    def refresh(self, sim):
        self.record(sim)
        b = sim.bus
        self.lbl.setText(
            f"RPM {b.get('propulsion.rpm', 0):6.1f}   "
            f"thrust {b.get('propulsion.thrust', 0) / 1e3:7.1f} kN\n"
            f"torque {b.get('propulsion.torque', 0) / 1e3:6.1f} kNm   "
            f"power {b.get('propulsion.power', 0) / 1e3:7.0f} kW")
        cav = b.get("propulsion.cavitating", False)
        self.cav.setText("⚠ CAVITATING" if cav else "flow quiet")
        self.cav.setStyleSheet(
            "color:#ff5050;font-weight:bold" if cav else "color:#50c050")
        self.curve.setData(*self._hist.arrays())

class PowerPanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        form = QtWidgets.QFormLayout(self)
        self.soc = QtWidgets.QProgressBar()
        form.addRow("Battery SOC", self.soc)
        self.lbl = QtWidgets.QLabel("")
        self.lbl.setStyleSheet("font-family:monospace")
        form.addRow(self.lbl)

    def refresh(self, sim):
        b = sim.bus
        self.soc.setValue(int(100 * b.get("power.soc", 1)))
        txt = (f"bus  {b.get('power.voltage', 0):5.0f} V\n"
               f"load {b.get('power.load_kw', 0):7.0f} kW   "
               f"gen {b.get('power.gen_kw', 0):6.0f} kW")
        for k, v in b.get("power.loads_detail", {}).items():
            if v > 0.05:
                txt += f"\n  {k:<12s}{v:8.1f} kW"
        if b.get("power.dead"):
            txt += "\n⚠ BATTERY DEPLETED"
        elif b.get("power.shed"):
            txt += "\n⚠ LOAD SHEDDING (non-vital off)"
        elif b.get("power.snorkeling"):
            txt += "\ncharging (snorkeling)"
        self.lbl.setText(txt)

class StatePanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        lay = QtWidgets.QVBoxLayout(self)
        self.lbl = QtWidgets.QLabel("")
        self.lbl.setStyleSheet("font-family:monospace")
        lay.addWidget(self.lbl)
        self._z = History(1200)
        self._f = History(1200)
        self._u = History(1200)
        self.zplot, self.zc = _plot("Depth / seafloor [m]", "#40d0ff", invert=True)
        self.fc = self.zplot.plot(pen=pg.mkPen("#b08850", width=1.4))
        self.uplot, self.uc = _plot("Speed [kn]", "#d070ff")
        for w in (self.zplot, self.uplot):
            lay.addWidget(w)
        lay.addStretch()

    def record(self, sim):
        b = sim.bus
        self._z.add(sim.time, b.get("state.depth", 0.0))
        self._f.add(sim.time, b.get("env.seafloor_depth", 0.0))
        self._u.add(sim.time, b.get("state.u", 0.0) / KN)

    def refresh(self, sim):
        self.record(sim)
        b, e = sim.bus, sim.eta
        hdg = (np.degrees(b.get("state.heading", 0)) + 360) % 360
        self.lbl.setText(
            f"pos  N {e[0]:9.1f}  E {e[1]:9.1f}\n"
            f"depth {b.get('state.depth', 0):7.1f} m   "
            f"hdg {hdg:6.1f}°\n"
            f"pitch {np.degrees(b.get('state.pitch', 0)):+6.1f}°  "
            f"roll {np.degrees(b.get('state.roll', 0)):+5.1f}°\n"
            f"speed {b.get('state.u', 0) / KN:5.1f} kn   "
            f"t = {b.get('time', 0):7.1f} s\n"
            f"keel  {b.get('env.seafloor_depth', 0) - b.get('state.depth', 0):7.1f} m   "
            f"floor {b.get('env.seafloor_depth', 0):6.0f} m")
        self.zc.setData(*self._z.arrays())
        self.fc.setData(*self._f.arrays())
        self.uc.setData(*self._u.arrays())
