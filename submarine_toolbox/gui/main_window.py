"""Main window: 3D viewer center, subsystem panels docked, keymap,
wall-clock-paced sim loop."""

from __future__ import annotations

import time

from PyQt5 import QtCore, QtGui, QtWidgets

from submarine_toolbox.core.simulation import Simulation
from submarine_toolbox.subsystems import build_default_boat
from submarine_toolbox.gui.controls import BINDINGS, TIME_SCALES, Controller
from submarine_toolbox.gui.viewer3d import CAMERA_MODES, Viewer3D
from submarine_toolbox.gui.panels.ship_control import (
    HelmPanel, BallastPanel, PropulsionPanel, PowerPanel, StatePanel)
from submarine_toolbox.gui.panels.acoustics import (
    SonarPanel, SignaturePanel, EnvironmentPanel)
from submarine_toolbox.gui.panels.tactical import (
    MastsPanel, NavPanel, TMAPanel, CommsPanel)
from submarine_toolbox.gui.panels.ship_systems import (
    AtmospherePanel, DamagePanel, HydraulicsPanel, EndurancePanel)

SIM_DT = 0.02
SIM_PERIOD_MS = 10
GUI_PERIOD_MS = 33
PANEL_EVERY = 3
STEP_BUDGET_S = 0.012

L, R, B = (QtCore.Qt.LeftDockWidgetArea, QtCore.Qt.RightDockWidgetArea,
           QtCore.Qt.BottomDockWidgetArea)
DOCKS = [
    ("Helm / Autopilot", HelmPanel, R),
    ("Ballast & Trim", BallastPanel, R),
    ("Power", PowerPanel, R),
    ("Masts / Radar / ESM", MastsPanel, R),
    ("Atmosphere", AtmospherePanel, R),
    ("Hydraulics", HydraulicsPanel, R),
    ("Environment", EnvironmentPanel, R),
    ("Ship State", StatePanel, L),
    ("Navigation", NavPanel, L),
    ("TMA", TMAPanel, L),
    ("Propulsion", PropulsionPanel, L),
    ("Signature", SignaturePanel, L),
    ("Comms", CommsPanel, L),
    ("Endurance", EndurancePanel, L),
    ("Sonar", SonarPanel, B),
    ("Damage Control", DamagePanel, B),
]

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Submarine Toolbox")
        self.resize(1600, 950)
        self.setDockOptions(QtWidgets.QMainWindow.AnimatedDocks
                            | QtWidgets.QMainWindow.AllowTabbedDocks)

        self.sim = Simulation(dt=SIM_DT)
        build_default_boat(self.sim)
        self.ctl = Controller(self.sim)

        self.viewer = Viewer3D(self.sim)
        self.setCentralWidget(self.viewer)

        self.panels, self.docks = [], []
        first = {}
        for title, cls, area in DOCKS:
            w = cls(self.sim)
            d = QtWidgets.QDockWidget(title, self)
            d.setObjectName(title)
            d.setWidget(w)
            self.addDockWidget(area, d)
            if area in first:
                self.tabifyDockWidget(first[area], d)
            else:
                first[area] = d
            self.panels.append(w)
            self.docks.append(d)
        for d in first.values():
            d.raise_()
        self.setTabPosition(L, QtWidgets.QTabWidget.West)
        self.setTabPosition(R, QtWidgets.QTabWidget.East)
        self.resizeDocks([first[L], first[R]], [340, 340], QtCore.Qt.Horizontal)
        self.resizeDocks([first[B]], [240], QtCore.Qt.Vertical)
        self._tame_focus()

        self.time_scale = 1
        self.paused = False
        self.lagging = False
        self._acc = 0.0
        self._last = time.perf_counter()
        self._step_cost = 1e-4
        self._gui_n = 0

        self._toolbar()
        self._menus()
        self._status = QtWidgets.QLabel("")
        self.statusBar().addPermanentWidget(self._status)

        self.sim_timer = QtCore.QTimer(self)
        self.sim_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.sim_timer.timeout.connect(self._sim_tick)
        self.sim_timer.start(SIM_PERIOD_MS)
        self.gui_timer = QtCore.QTimer(self)
        self.gui_timer.timeout.connect(self._gui_tick)
        self.gui_timer.start(GUI_PERIOD_MS)
        self.viewer.setFocus()
        self.toast("Press H for controls")

    # ---------------------------------------------------------- setup
    def _tame_focus(self):
        """Keep keyboard focus on the 3D view unless the user is typing."""
        for w in self.findChildren(QtWidgets.QWidget):
            if isinstance(w, (QtWidgets.QAbstractButton, QtWidgets.QSlider,
                              QtWidgets.QComboBox, QtWidgets.QTabBar,
                              QtWidgets.QAbstractItemView,
                              QtWidgets.QGraphicsView)):
                w.setFocusPolicy(QtCore.Qt.NoFocus)
            elif isinstance(w, QtWidgets.QAbstractSpinBox):
                w.setFocusPolicy(QtCore.Qt.ClickFocus)
                w.editingFinished.connect(self.viewer.setFocus)

    def _toolbar(self):
        tb = self.addToolBar("Sim")
        tb.setObjectName("sim_toolbar")
        tb.setMovable(False)
        self.pause_act = tb.addAction("Pause")
        self.pause_act.setCheckable(True)
        self.pause_act.toggled.connect(self._set_paused)
        tb.addSeparator()
        tb.addWidget(QtWidgets.QLabel(" time × "))
        self.ts_combo = QtWidgets.QComboBox()
        self.ts_combo.addItems([str(s) for s in TIME_SCALES])
        self.ts_combo.currentTextChanged.connect(
            lambda t: setattr(self, "time_scale", int(t)))
        tb.addWidget(self.ts_combo)
        tb.addSeparator()
        tb.addWidget(QtWidgets.QLabel(" camera "))
        self.cam_combo = QtWidgets.QComboBox()
        self.cam_combo.addItems(CAMERA_MODES)
        self.cam_combo.currentTextChanged.connect(self.viewer.set_camera_mode)
        tb.addWidget(self.cam_combo)
        tb.addSeparator()
        h = tb.addAction("Help (H)")
        h.triggered.connect(lambda: self.run("help"))
        for w in tb.findChildren(QtWidgets.QWidget):
            w.setFocusPolicy(QtCore.Qt.NoFocus)

    def _menus(self):
        mb = self.menuBar()
        view = mb.addMenu("&View")
        for d in self.docks:
            view.addAction(d.toggleViewAction())
        view.addSeparator()
        view.addAction("Show all panels", lambda: self._show_panels(True))
        view.addAction("Hide all panels", lambda: self._show_panels(False))
        ctl = mb.addMenu("&Controls")
        group = None
        for b in BINDINGS:
            if not b.cmd or not b.keys:
                continue
            if b.group != group:
                if group is not None:
                    ctl.addSeparator()
                group = b.group
            name = b.desc or f"{b.cmd} {b.arg}"
            act = QtWidgets.QAction(name, self)
            act.setShortcuts([QtGui.QKeySequence(k) for k in b.keys])
            act.setShortcutContext(QtCore.Qt.WindowShortcut)
            act.triggered.connect(lambda _=False, b=b: self.run(b.cmd, b.arg))
            self.addAction(act)
            if b.help:
                ctl.addAction(act)

    # -------------------------------------------------------- commands
    def toast(self, msg, secs=1.8):
        self.viewer.toast = (msg, time.monotonic() + secs)
        self.viewer.update()

    def run(self, cmd, arg=None):
        fn = getattr(self, "cmd_" + cmd, None)
        if fn is not None:
            msg = fn() if arg is None else fn(arg)
        else:
            f = getattr(self.ctl, cmd)
            msg = f() if arg is None else f(arg)
        if msg:
            self.toast(msg)

    def cmd_pause(self):
        self.pause_act.setChecked(not self.paused)
        return "PAUSED" if self.paused else "Running"

    def cmd_time_scale(self, d):
        i = TIME_SCALES.index(self.time_scale) if self.time_scale in TIME_SCALES else 0
        i = max(0, min(len(TIME_SCALES) - 1, i + d))
        self.ts_combo.setCurrentText(str(TIME_SCALES[i]))
        return f"Time ×{self.time_scale}"

    def cmd_camera(self):
        m = self.viewer.cycle_camera()
        self.cam_combo.blockSignals(True)
        self.cam_combo.setCurrentText(m)
        self.cam_combo.blockSignals(False)
        return f"Camera: {m}"

    def cmd_reset_camera(self):
        self.viewer.reset_camera()
        return "Camera reset"

    def cmd_labels(self):
        self.viewer.show_labels = not self.viewer.show_labels
        return "Contact labels " + ("on" if self.viewer.show_labels else "off")

    def cmd_panels(self):
        show = not any(d.isVisible() for d in self.docks)
        self._show_panels(show)
        return "Panels shown" if show else "Panels hidden (F2 to restore)"

    def cmd_fullscreen(self):
        self.showNormal() if self.isFullScreen() else self.showFullScreen()
        return ""

    def cmd_help(self):
        self.viewer.show_help = not self.viewer.show_help
        self.viewer.update()
        return ""

    def cmd_escape(self):
        self.viewer.show_help = False
        self.viewer.setFocus()
        self.viewer.update()
        return ""

    def _show_panels(self, show):
        for d in self.docks:
            d.setVisible(show)

    def _set_paused(self, v):
        self.paused = v
        self._last = time.perf_counter()
        self._acc = 0.0

    # ------------------------------------------------------------ loop
    def _sim_tick(self):
        now = time.perf_counter()
        el, self._last = now - self._last, now
        if self.paused:
            return
        self._acc += min(el, 0.25) * self.time_scale
        dt = self.sim.dt
        n = int(self._acc / dt)
        cap = max(1, int(STEP_BUDGET_S / self._step_cost))
        self.lagging = n > cap
        if n > cap:
            n = cap
            self._acc = 0.0
        else:
            self._acc -= n * dt
        if n:
            t0 = time.perf_counter()
            self.sim.step(n)
            c = (time.perf_counter() - t0) / n
            self._step_cost += 0.1 * (c - self._step_cost)

    def _gui_tick(self):
        self.viewer.frame(dict(time_scale=self.time_scale, paused=self.paused,
                               lagging=self.lagging))
        self._gui_n += 1
        draw = self._gui_n % PANEL_EVERY == 0
        for p in self.panels:
            if draw and p.isVisible():
                p.refresh(self.sim)
            elif hasattr(p, "record"):
                p.record(self.sim)
        if draw:
            cap = self.sim.dt / self._step_cost
            self._status.setText(
                f"step {self._step_cost * 1e6:5.0f} µs  ·  capacity ≈{cap:4.0f}× real time"
                f"  ·  origin {self.viewer.origin[0] / 1e3:+.0f}/{self.viewer.origin[1] / 1e3:+.0f} km"
                f"  ·  tiles {len(self.viewer.terrain.items)}")
