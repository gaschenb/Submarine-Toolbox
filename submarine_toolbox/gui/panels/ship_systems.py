"""Ship-systems panels: atmosphere, damage control, hydraulics, endurance."""

from __future__ import annotations

from PyQt5 import QtCore, QtWidgets

from submarine_toolbox.subsystems.damage import COMPARTMENT_SPEC

def _bar(lo, hi):
    b = QtWidgets.QProgressBar()
    b.setRange(lo, hi)
    return b

class AtmospherePanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        self.atmo = sim.get_subsystem("atmosphere")
        form = QtWidgets.QFormLayout(self)
        self.o2 = _bar(150, 220)
        self.co2 = _bar(0, 300)
        form.addRow("O₂ [%]", self.o2)
        form.addRow("CO₂ [%]", self.co2)
        self.gen = QtWidgets.QCheckBox("O₂ generator")
        self.gen.setChecked(True)
        self.gen.toggled.connect(lambda v: setattr(self.atmo, "o2_gen_on", v))
        self.scrub = QtWidgets.QCheckBox("CO₂ scrubbers")
        self.scrub.setChecked(True)
        self.scrub.toggled.connect(
            lambda v: setattr(self.atmo, "scrubber_on", v))
        self.bleed = QtWidgets.QCheckBox("Bleed O₂ bank")
        self.bleed.toggled.connect(
            lambda v: setattr(self.atmo, "bleed_bank", v))
        for w in (self.gen, self.scrub, self.bleed):
            form.addRow(w)
        self.status = QtWidgets.QLabel("")
        form.addRow(self.status)

    def refresh(self, sim):
        b = sim.bus
        o2, co2 = b.get("atmo.o2", 20.9), b.get("atmo.co2", 0.04)
        self.o2.setValue(int(o2 * 10))
        self.o2.setFormat(f"{o2:.2f}")
        self.co2.setValue(int(co2 * 100))
        self.co2.setFormat(f"{co2:.3f}")
        st = b.get("atmo.status", "NORMAL")
        color = {"NORMAL": "#50c050", "CAUTION": "#e0a020",
                 "DANGER": "#ff4040"}[st]
        self.status.setText(
            f"status: {st}   O₂ bank {b.get('atmo.o2_bank', 0):.0f} m³")
        self.status.setStyleSheet(f"color:{color};font-weight:bold")

class DamagePanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        self.dc = sim.get_subsystem("damage")
        lay = QtWidgets.QVBoxLayout(self)

        self.table = QtWidgets.QTableWidget(len(COMPARTMENT_SPEC), 5)
        self.table.setHorizontalHeaderLabels(
            ["Compartment", "Flood %", "Fire", "Damage", "Pump"])
        self.table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        lay.addWidget(self.table)

        row = QtWidgets.QHBoxLayout()
        self.combo = QtWidgets.QComboBox()
        self.combo.addItems([n for n, _, _ in COMPARTMENT_SPEC])
        row.addWidget(self.combo)
        for label, fn in [
                ("Breach S", lambda: self.dc.breach(self._sel(), "small")),
                ("Breach L", lambda: self.dc.breach(self._sel(), "large")),
                ("Patch", lambda: self.dc.patch(self._sel())),
                ("Ignite", lambda: self.dc.ignite(self._sel()))]:
            btn = QtWidgets.QPushButton(label)
            btn.clicked.connect(fn)
            row.addWidget(btn)
        lay.addLayout(row)

        row2 = QtWidgets.QHBoxLayout()
        sup = QtWidgets.QPushButton("Suppress fire")
        sup.clicked.connect(self._toggle_suppress)
        row2.addWidget(sup)
        pump = QtWidgets.QPushButton("Toggle pump")
        pump.clicked.connect(self._toggle_pump)
        row2.addWidget(pump)
        self.doors = QtWidgets.QCheckBox("WT doors shut")
        self.doors.setChecked(True)
        self.doors.toggled.connect(
            lambda v: setattr(self.dc, "doors_shut", v))
        row2.addWidget(self.doors)
        lay.addLayout(row2)

        self.hull = QtWidgets.QLabel("")
        lay.addWidget(self.hull)

    def _sel(self):
        return self.combo.currentText()

    def _toggle_suppress(self):
        c = self.dc.by_name(self._sel())
        c.suppressing = not c.suppressing

    def _toggle_pump(self):
        c = self.dc.by_name(self._sel())
        c.pump_on = not c.pump_on

    def refresh(self, sim):
        b = sim.bus
        for i, c in enumerate(b.get("damage.status", [])):
            fire_txt = ("SUPPRESSING" if c["suppressing"]
                        else f"{c['fire']:.2f}" if c["fire"] > 0 else "—")
            vals = [c["name"] + (" ⚠" if c["breach"] else ""),
                    f"{c['flood_pct']:5.1f}", fire_txt,
                    f"{c['damage']:.0f}", "on" if c["pump"] else "OFF"]
            for j, v in enumerate(vals):
                it = QtWidgets.QTableWidgetItem(v)
                if c["fire"] > 0 and j == 2:
                    it.setForeground(QtCore.Qt.red)
                self.table.setItem(i, j, it)
        hull = b.get("damage.hull_state", "OK")
        col = {"OK": "#50c050", "OVER TEST DEPTH": "#e0a020",
               "FAILING": "#ff4040"}.get(hull, "#e0a020")
        extra = "  ⚠ GROUNDED" if b.get("state.grounded") else ""
        self.hull.setText(f"hull: {hull}{extra}")
        self.hull.setStyleSheet(f"color:{col};font-weight:bold")

class HydraulicsPanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        self.hyd = sim.get_subsystem("hydraulics")
        form = QtWidgets.QFormLayout(self)
        self.press = QtWidgets.QProgressBar()
        self.press.setRange(0, 207)
        form.addRow("Pressure [bar]", self.press)
        self.fluid = QtWidgets.QProgressBar()
        form.addRow("Fluid", self.fluid)
        self.pump = QtWidgets.QCheckBox("Pump on")
        self.pump.setChecked(True)
        self.pump.toggled.connect(
            lambda v: setattr(self.hyd, "pump_on", v))
        form.addRow(self.pump)
        row = QtWidgets.QHBoxLayout()
        for label, fn in [("Sim leak", self.hyd.start_leak),
                          ("Isolate", self.hyd.isolate_leak),
                          ("Refill (reserve)",
                           self.hyd.refill_from_reserve)]:
            btn = QtWidgets.QPushButton(label)
            btn.clicked.connect(fn)
            row.addWidget(btn)
        form.addRow(row)
        self.lbl = QtWidgets.QLabel("")
        form.addRow(self.lbl)

    def refresh(self, sim):
        b = sim.bus
        self.press.setValue(int(b.get("hydraulics.pressure", 0)))
        self.fluid.setValue(int(100 * b.get("hydraulics.fluid", 1)))
        txt = f"actuator effectiveness {b.get('hydraulics.effectiveness', 1):.2f}"
        if b.get("hydraulics.leaking"):
            txt += "\n⚠ LEAKING"
        if not b.get("hydraulics.reserve", True):
            txt += "\nreserve expended"
        self.lbl.setText(txt)

class EndurancePanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        self.end = sim.get_subsystem("endurance")
        form = QtWidgets.QFormLayout(self)
        self.fuel = QtWidgets.QProgressBar()
        form.addRow("Diesel fuel", self.fuel)
        self.water = QtWidgets.QProgressBar()
        form.addRow("Fresh water", self.water)
        self.dist = QtWidgets.QCheckBox("Distiller on")
        self.dist.setChecked(True)
        self.dist.toggled.connect(
            lambda v: setattr(self.end, "distiller_on", v))
        form.addRow(self.dist)
        self.lbl = QtWidgets.QLabel("")
        self.lbl.setStyleSheet("font-family:monospace")
        form.addRow(self.lbl)

    def refresh(self, sim):
        b = sim.bus
        self.fuel.setValue(int(b.get("endurance.fuel_pct", 100)))
        cap = self.end.water_cap
        self.water.setValue(int(100 * b.get("endurance.water_t", 0) / cap))
        self.lbl.setText(
            f"fuel  {b.get('endurance.fuel_t', 0):6.1f} t\n"
            f"water {b.get('endurance.water_t', 0):6.1f} t "
            f"({b.get('endurance.water_days', 0):.1f} d unreplenished)\n"
            f"food  {b.get('endurance.food_days', 0):6.1f} days")
