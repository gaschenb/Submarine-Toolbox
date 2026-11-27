"""Tactical panels: masts + radar/ESM, navigation, TMA, communications."""

from __future__ import annotations

import numpy as np
from PyQt5 import QtWidgets

class MastsPanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        self.masts = sim.get_subsystem("masts")
        self.checks = {}
        form = QtWidgets.QFormLayout(self)
        for attr, label in [("periscope", "Periscope"), ("esm", "ESM mast"),
                            ("radar", "Radar mast"), ("snorkel", "Snorkel")]:
            cb = QtWidgets.QCheckBox(label)
            cb.toggled.connect(
                lambda v, a=attr: setattr(self.masts, a, v))
            self.checks[attr] = cb
            form.addRow(cb)
        self.tx = QtWidgets.QCheckBox("RADAR TRANSMIT")
        self.tx.setStyleSheet("font-weight:bold;color:#e05030")
        self.tx.toggled.connect(
            lambda v: setattr(self.masts, "radar_tx", v))
        form.addRow(self.tx)
        self.warn = QtWidgets.QLabel("")
        self.warn.setWordWrap(True)
        form.addRow(self.warn)

        self.radar_lbl = QtWidgets.QLabel("radar: —")
        self.esm_lbl = QtWidgets.QLabel("esm: —")
        for w in (self.radar_lbl, self.esm_lbl):
            w.setStyleSheet("font-family:monospace")
            w.setWordWrap(True)
            form.addRow(w)

    def refresh(self, sim):
        b = sim.bus
        deep = b.get("state.depth", 0.0) > 18.0
        for attr, cb in self.checks.items():
            actual = getattr(self.masts, attr)
            if cb.isChecked() != actual:
                cb.blockSignals(True)
                cb.setChecked(actual)
                cb.blockSignals(False)
        if self.tx.isChecked() != self.masts.radar_tx:
            self.tx.blockSignals(True)
            self.tx.setChecked(self.masts.radar_tx)
            self.tx.blockSignals(False)
        self.warn.setText("⚠ too deep for masts (>18 m)" if deep else "")

        rc = b.get("radar.contacts", [])
        self.radar_lbl.setText("radar: " + (", ".join(
            f"#{c['cid']} {c['range'] / 1e3:.1f}km "
            f"{(np.degrees(c['bearing']) + 360) % 360:.0f}°" for c in rc)
            if rc else "—"))
        cd = b.get("radar.counter_detected_by", [])
        if cd:
            self.radar_lbl.setText(self.radar_lbl.text()
                                   + f"\n⚠ ESM-detected by {cd}")
        ic = b.get("esm.intercepts", [])
        self.esm_lbl.setText("esm: " + (", ".join(
            f"#{i['cid']} {i['emitter']} "
            f"{(np.degrees(i['bearing']) + 360) % 360:.0f}° "
            f"{i['strength']:.0f}dB" for i in ic) if ic else "—"))

class NavPanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        self.nav = sim.get_subsystem("navigation")
        form = QtWidgets.QFormLayout(self)
        self.lbl = QtWidgets.QLabel("")
        self.lbl.setStyleSheet("font-family:monospace")
        form.addRow(self.lbl)
        self.auto = QtWidgets.QCheckBox("Auto GPS fix at PD")
        self.auto.setChecked(True)
        self.auto.toggled.connect(
            lambda v: setattr(self.nav, "auto_fix", v))
        form.addRow(self.auto)
        fix = QtWidgets.QPushButton("Manual GPS fix")
        fix.clicked.connect(self._manual_fix)
        form.addRow(fix)
        self._sim = sim

    def _manual_fix(self):
        if self._sim.bus.get("nav.gps_available", False):
            self.nav.gps_fix()

    def refresh(self, sim):
        b = sim.bus
        est = b.get("nav.est_ne", sim.eta[:2])
        self.lbl.setText(
            f"INS est   N {est[0]:9.1f}  E {est[1]:9.1f}\n"
            f"true err  {b.get('nav.err_true_m', 0):7.1f} m\n"
            f"last fix  {b.get('nav.since_fix', 0):7.0f} s ago\n"
            f"GPS       {'AVAILABLE' if b.get('nav.gps_available') else 'no antenna'}")

class TMAPanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        self.sim = sim
        self.tma = sim.get_subsystem("tma")
        form = QtWidgets.QFormLayout(self)
        self.combo = QtWidgets.QComboBox()
        self.combo.currentTextChanged.connect(self._select)
        form.addRow("Track contact", self.combo)
        self.lbl = QtWidgets.QLabel("no solution")
        self.lbl.setStyleSheet("font-family:monospace")
        form.addRow(self.lbl)
        hint = QtWidgets.QLabel(
            "Bearing-only: maneuver own ship between legs\n"
            "for range observability.")
        hint.setStyleSheet("color:#909090")
        form.addRow(hint)

    def _select(self, txt):
        try:
            self.tma.set_track(int(txt))
        except ValueError:
            self.tma.set_track(None)

    def refresh(self, sim):
        b = sim.bus
        held = sorted(c["cid"] for c in b.get("sonar.contacts", []))
        items = ["—"] + [str(c) for c in held]
        current = [self.combo.itemText(i) for i in range(self.combo.count())]
        if items != current:
            sel = self.combo.currentText()
            self.combo.blockSignals(True)
            self.combo.clear()
            self.combo.addItems(items)
            if sel in items:
                self.combo.setCurrentText(sel)
            self.combo.blockSignals(False)

        sol = b.get("tma.solution")
        if not sol:
            self.lbl.setText("no solution")
            return
        self.lbl.setText(
            f"contact #{sol['cid']}\n"
            f"range   {sol['range'] / 1e3:6.2f} km\n"
            f"bearing {(np.degrees(sol['bearing']) + 360) % 360:6.1f}°\n"
            f"course  {(np.degrees(sol['course']) + 360) % 360:6.1f}°\n"
            f"speed   {sol['speed']:6.2f} m/s\n"
            f"rms     {sol['rms_deg']:6.2f}°  "
            f"({sol['n_samples']} obs / {sol['leg_span_s']:.0f} s)")

class CommsPanel(QtWidgets.QWidget):
    def __init__(self, sim):
        super().__init__()
        self.cm = sim.get_subsystem("comms")
        self.masts = sim.get_subsystem("masts")
        form = QtWidgets.QFormLayout(self)
        self.wire = QtWidgets.QCheckBox("Floating wire (VLF rx)")
        self.wire.toggled.connect(
            lambda v: setattr(self.cm, "wire_deployed", v))
        form.addRow(self.wire)
        self.mast = QtWidgets.QCheckBox("Comms mast (SATCOM/HF)")
        self.mast.toggled.connect(
            lambda v: setattr(self.masts, "comms", v))
        form.addRow(self.mast)
        send = QtWidgets.QPushButton("Queue outgoing message")
        send.clicked.connect(self.cm.queue_message)
        form.addRow(send)
        self.lbl = QtWidgets.QLabel("")
        self.lbl.setStyleSheet("font-family:monospace")
        form.addRow(self.lbl)

    def refresh(self, sim):
        b = sim.bus
        if self.mast.isChecked() != self.masts.comms:
            self.mast.blockSignals(True)
            self.mast.setChecked(self.masts.comms)
            self.mast.blockSignals(False)
        missed = b.get("comms.missed", 0)
        warn = "  ⚠" if missed > 1 else ""
        self.lbl.setText(
            f"path     {b.get('comms.path', 'none')}\n"
            f"missed   {missed} broadcasts{warn}\n"
            f"outbox   {b.get('comms.outbox', 0)}   "
            f"sent {b.get('comms.sent', 0)}")
