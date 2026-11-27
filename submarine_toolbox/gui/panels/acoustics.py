"""Acoustics panels: passive-sonar waterfall + contact table + active sonar
controls, own-ship signature, environment."""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

from submarine_toolbox.gui.panels.ship_control import History, _plot

DEG = np.pi / 180.0

class SonarPanel(QtWidgets.QWidget):
    """Bearing-time waterfall (broadband), contact table, active ping."""

    N_BEARING = 360
    N_ROWS = 240
    ROW_PERIOD = 1.0

    def __init__(self, sim):
        super().__init__()
        self.sim = sim
        self.psonar = sim.get_subsystem("passive_sonar")
        self.asonar = sim.get_subsystem("active_sonar")
        self._wf = np.zeros((self.N_ROWS, self.N_BEARING))
        self._last_row_t = -1e9
        self._bins = np.arange(self.N_BEARING)
        self._dirty = False

        lay = QtWidgets.QHBoxLayout(self)

        left = QtWidgets.QVBoxLayout()
        glw = pg.GraphicsLayoutWidget()
        vb = glw.addPlot(title="Broadband bearing-time (deg true)")
        vb.setLabel("bottom", "bearing")
        self.img = pg.ImageItem(axisOrder="row-major")
        cmap = pg.colormap.get("inferno")
        self.img.setLookupTable(cmap.getLookupTable(0.0, 1.0, 256))
        vb.addItem(self.img)
        vb.setXRange(0, 360)
        vb.getViewBox().invertY(True)
        left.addWidget(glw)
        lay.addLayout(left, stretch=3)

        right = QtWidgets.QVBoxLayout()
        self.towed = QtWidgets.QCheckBox("Towed array deployed")
        self.towed.toggled.connect(
            lambda v: setattr(self.psonar, "towed_deployed", v))
        right.addWidget(self.towed)

        self.table = QtWidgets.QTableWidget(0, 4)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setHorizontalHeaderLabels(
            ["ID", "Brg °", "SNR dB", "Class"])
        self.table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch)
        right.addWidget(self.table)

        ping = QtWidgets.QPushButton("ACTIVE PING")
        ping.setStyleSheet(
            "background:#a06010;color:white;font-weight:bold;padding:5px")
        ping.clicked.connect(self.asonar.ping)
        right.addWidget(ping)
        self.echoes = QtWidgets.QLabel("no echoes")
        self.echoes.setStyleSheet("font-family:monospace")
        self.echoes.setWordWrap(True)
        right.addWidget(self.echoes)
        self.nl = QtWidgets.QLabel("")
        right.addWidget(self.nl)
        lay.addLayout(right, stretch=2)

    def record(self, sim):
        t = sim.time
        if t < self._last_row_t:
            self._last_row_t = t - self.ROW_PERIOD
        k = int((t - self._last_row_t) // self.ROW_PERIOD)
        if k <= 0:
            return
        self._last_row_t += k * self.ROW_PERIOD
        k = min(k, self.N_ROWS)
        contacts = sim.bus.get("sonar.contacts", [])
        rows = np.abs(np.random.normal(0.0, 0.35, (k, self.N_BEARING)))
        if contacts:
            bd = (np.degrees([c["bearing"] for c in contacts]) + 360.0) % 360.0
            amp = np.minimum([c["snr"] for c in contacts], 25.0) / 8.0
            d = np.abs(self._bins[None, :] - bd[:, None])
            d = np.minimum(d, 360.0 - d)
            rows += amp @ np.exp(-0.5 * (d / 2.0) ** 2)
        self._wf = np.vstack([rows[::-1], self._wf[:-k]])
        self._dirty = True

    def refresh(self, sim):
        self.record(sim)
        b = sim.bus
        contacts = b.get("sonar.contacts", [])
        if self._dirty:
            self._dirty = False
            self.img.setImage(self._wf, autoLevels=False,
                              levels=(0.0, 3.5),
                              rect=QtCore.QRectF(0, 0, 360, self.N_ROWS))
        if self.towed.isChecked() != self.psonar.towed_deployed:
            self.towed.blockSignals(True)
            self.towed.setChecked(self.psonar.towed_deployed)
            self.towed.blockSignals(False)

        self.table.setRowCount(len(contacts))
        for i, c in enumerate(sorted(contacts, key=lambda x: -x["snr"])):
            bdeg = (np.degrees(c["bearing"]) + 360.0) % 360.0
            for j, val in enumerate([str(c["cid"]), f"{bdeg:5.1f}",
                                     f"{c['snr']:5.1f}", c["classified"]]):
                self.table.setItem(i, j, QtWidgets.QTableWidgetItem(val))

        self.nl.setText(f"noise level {b.get('sonar.noise_level', 0):5.1f} dB")

        echoes = b.get("sonar.echoes")
        if echoes is not None and b.get("sonar.ping_time", -1) >= 0:
            if echoes:
                lines = [f"#{e['cid']}: {e['range'] / 1e3:6.2f} km  "
                         f"{(np.degrees(e['bearing']) + 360) % 360:5.1f}°  "
                         f"snr {e['snr']:4.1f}" for e in echoes]
            else:
                lines = ["ping: no echoes"]
            lines.append("⚠ ping counter-detectable to "
                         f"{b.get('sonar.ping_counter_detection_m', 0) / 1e3:.0f} km")
            self.echoes.setText("\n".join(lines))

class SignaturePanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        lay = QtWidgets.QVBoxLayout(self)
        self.lbl = QtWidgets.QLabel("")
        self.lbl.setStyleSheet("font-family:monospace")
        lay.addWidget(self.lbl)
        self.plot, self.curve = _plot("Radiated SL [dB re 1 µPa @ 1 m]",
                                      "#ff6060")
        self._hist = History(900)
        lay.addWidget(self.plot)
        lay.addStretch()

    def record(self, sim):
        self._hist.add(sim.time, sim.bus.get("signature.sl", 0.0))

    def refresh(self, sim):
        self.record(sim)
        b = sim.bus
        sl = b.get("signature.sl", 0.0)
        comps = b.get("signature.components", {})
        rcd = b.get("signature.counter_detection_m", 0.0)
        txt = f"SL total {sl:6.1f} dB\n"
        txt += "\n".join(f"  {k:<16s}{v:6.1f}" for k, v in comps.items())
        txt += f"\ncounter-detection ~{rcd / 1e3:6.2f} km"
        self.lbl.setText(txt)
        self.curve.setData(*self._hist.arrays())

class EnvironmentPanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        self.sim = sim
        self.env = sim.get_subsystem("environment")
        form = QtWidgets.QFormLayout(self)

        self.ss = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.ss.setRange(0, 6)
        self.ss.setValue(self.env.sea_state)
        self.ss.valueChanged.connect(
            lambda v: setattr(self.env, "sea_state", v))
        form.addRow("Sea state", self.ss)

        self.cs = QtWidgets.QDoubleSpinBox()
        self.cs.setRange(0.0, 3.0)
        self.cs.setSingleStep(0.1)
        self.cs.setValue(self.env.current_speed)
        self.cs.valueChanged.connect(
            lambda v: setattr(self.env, "current_speed", v))
        form.addRow("Current [m/s]", self.cs)

        self.cd = QtWidgets.QDoubleSpinBox()
        self.cd.setRange(0.0, 359.9)
        self.cd.setValue(np.degrees(self.env.current_dir))
        self.cd.valueChanged.connect(
            lambda v: setattr(self.env, "current_dir", np.radians(v)))
        form.addRow("Current dir [°]", self.cd)

        self.ld = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.ld.setRange(20, 200)
        self.ld.setValue(int(self.env.layer_depth))
        self.ld.valueChanged.connect(self._layer_changed)
        form.addRow("Layer depth [m]", self.ld)

        self.proc = QtWidgets.QCheckBox("Procedural bathymetry")
        self.proc.setChecked(self.env.procedural)
        self.proc.toggled.connect(lambda v: setattr(self.env, "procedural", v))
        form.addRow(self.proc)
        self.floor_lbl = QtWidgets.QLabel("")
        self.floor_lbl.setStyleSheet("font-family:monospace")
        form.addRow(self.floor_lbl)

        self.svp = pg.PlotWidget(title="SVP  c(z)")
        self.svp.showGrid(x=True, y=True, alpha=0.15)
        self.svp.setMaximumHeight(220)
        self.svp.invertY(True)
        self.svp.setLabel("left", "depth m")
        self.svp_curve = self.svp.plot(pen=pg.mkPen("#40d0ff", width=1.6))
        self.floor_line = pg.InfiniteLine(angle=0, pen=pg.mkPen("#b08850", width=1.5))
        self.svp.addItem(self.floor_line)
        self.own_marker = self.svp.plot(
            symbol="o", symbolBrush="y", symbolSize=8, pen=None)
        form.addRow(self.svp)
        self._redraw_svp()

    def _layer_changed(self, v):
        self.env.layer_depth = float(v)
        self._redraw_svp()

    def _redraw_svp(self, zmax=None):
        zmax = zmax or max(self.env.seafloor_depth, 400.0)
        self._svp_zmax = zmax
        z = np.linspace(0, zmax, 200)
        self.svp_curve.setData(self.env.sound_speed(z), z)

    def refresh(self, sim):
        b = sim.bus
        z = b.get("state.depth", 0.0)
        floor = b.get("env.seafloor_depth", 400.0)
        if abs(max(floor, 400.0) - self._svp_zmax) > 25.0:
            self._redraw_svp(max(floor, 400.0))
        self.floor_line.setValue(floor)
        self.own_marker.setData([float(self.env.sound_speed(z))], [z])
        ahead = b.get("env.floor_ahead")
        txt = f"floor here {floor:6.0f} m"
        if ahead is not None:
            txt += f"\nmin ahead (2 km) {np.min(ahead):5.0f} m"
        self.floor_lbl.setText(txt)
