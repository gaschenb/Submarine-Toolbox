"""Keymap and the commands it drives.

BINDINGS is the single source of truth: the window registers shortcuts
from it, the Controls menu lists it, and the help overlay renders it.
Helm keys act on autopilot orders when the autopilot is engaged and on
the raw actuators (rudder / planes / shaft rpm) in manual.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

KN = 0.5144
DEG = math.pi / 180.0
TIME_SCALES = (1, 2, 5, 10, 20, 50, 100)
DEPTH_PRESETS = {1: 15.0, 2: 40.0, 3: 100.0, 4: 200.0}
MAX_DEPTH_ORDER = 400.0
MAX_SPEED_KN = 18.0
MAX_RPM = 200.0

@dataclass(frozen=True)
class Binding:
    keys: tuple
    label: str
    desc: str
    group: str
    cmd: str
    arg: object = None
    help: bool = True

def _b(keys, label, desc, group, cmd, arg=None, help=True):
    return Binding(tuple(keys), label, desc, group, cmd, arg, help)

BINDINGS = [
    _b(["W"], "W / S", "speed ±1 kn  (manual: rpm ±10)", "Helm", "speed", 1),
    _b(["S"], "", "", "Helm", "speed", -1, False),
    _b(["Shift+W"], "", "", "Helm", "speed", 5, False),
    _b(["Shift+S"], "", "", "Helm", "speed", -5, False),
    _b(["A"], "A / D", "heading ∓5°  (manual: rudder)", "Helm", "turn", -5),
    _b(["D"], "", "", "Helm", "turn", 5, False),
    _b(["Shift+A"], "", "", "Helm", "turn", -30, False),
    _b(["Shift+D"], "", "", "Helm", "turn", 30, False),
    _b(["R"], "R / F", "rise / fall 10 m  (manual: planes)", "Helm", "dive", -10),
    _b(["F"], "", "", "Helm", "dive", 10, False),
    _b(["Shift+R"], "", "", "Helm", "dive", -50, False),
    _b(["Shift+F"], "", "", "Helm", "dive", 50, False),
    _b([], "Shift", "×5 step on W S A D R F", "Helm", "", None),
    _b(["1"], "1 – 4", "depth preset 15 / 40 / 100 / 200 m", "Helm", "depth_preset", 1),
    _b(["2"], "", "", "Helm", "depth_preset", 2, False),
    _b(["3"], "", "", "Helm", "depth_preset", 3, False),
    _b(["4"], "", "", "Helm", "depth_preset", 4, False),
    _b(["X"], "X", "all stop", "Helm", "all_stop"),
    _b(["L"], "L", "hold current course & depth", "Helm", "hold"),
    _b(["M"], "M", "autopilot ⇄ manual (bumpless)", "Helm", "toggle_autopilot"),

    _b(["P"], "P", "active sonar ping", "Systems", "ping"),
    _b(["T"], "T", "towed array deploy / recover", "Systems", "towed"),
    _b(["U"], "U", "periscope up / down", "Systems", "periscope"),
    _b(["N"], "N", "snorkel (charge batteries)", "Systems", "snorkel"),
    _b(["G"], "G", "GPS fix (needs a mast at PD)", "Systems", "gps"),
    _b(["Ctrl+B"], "Ctrl+B", "EMERGENCY BLOW", "Systems", "blow"),

    _b(["Space"], "Space", "pause / resume", "View & sim", "pause"),
    _b(["]"], "[ / ]", "time compression down / up", "View & sim", "time_scale", 1),
    _b(["["], "", "", "View & sim", "time_scale", -1, False),
    _b(["C"], "C", "camera: chase / orbit / top / free", "View & sim", "camera"),
    _b(["V"], "V", "reset camera", "View & sim", "reset_camera"),
    _b(["K"], "K", "contact labels on / off", "View & sim", "labels"),
    _b(["F2"], "F2", "hide / show panels", "View & sim", "panels"),
    _b(["F11"], "F11", "fullscreen", "View & sim", "fullscreen"),
    _b(["H", "F1"], "H / F1", "this help", "View & sim", "help"),
    _b(["Esc"], "Esc", "close help, return keys to 3D view", "View & sim", "escape"),
    _b([], "Mouse", "drag orbit · wheel zoom · Ctrl-drag pan", "View & sim", ""),
]

def help_rows():
    groups: dict = {}
    for b in BINDINGS:
        if b.help and b.label:
            groups.setdefault(b.group, []).append((b.label, b.desc))
    return groups

def _clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

class Controller:
    """Executes commands against the simulation; returns a toast string."""

    def __init__(self, sim):
        self.sim = sim

    @property
    def bus(self):
        return self.sim.bus

    @property
    def auto(self):
        return bool(self.bus.get("autopilot.enabled", True))

    def _sub(self, name):
        return self.sim.get_subsystem(name)

    def speed(self, d):
        b = self.bus
        if self.auto:
            v = _clip(b.get("order.speed", 0.0) / KN + d, 0.0, MAX_SPEED_KN)
            b["order.speed"] = v * KN
            return f"Speed order {v:.0f} kn"
        r = _clip(b.get("order.rpm", 0.0) + 10.0 * d, -MAX_RPM, MAX_RPM)
        b["order.rpm"] = r
        return f"Shaft {r:+.0f} rpm"

    def turn(self, d):
        b = self.bus
        if self.auto:
            h = (math.degrees(b.get("order.heading", 0.0)) + d) % 360.0
            b["order.heading"] = math.radians(h)
            return f"Heading order {h:05.1f}°"
        r = _clip(math.degrees(b.get("order.rudder", 0.0)) + d, -30.0, 30.0)
        b["order.rudder"] = r * DEG
        return f"Rudder {r:+.0f}°"

    def dive(self, d):
        b = self.bus
        if self.auto:
            z = _clip(b.get("order.depth", 0.0) + d, 0.0, MAX_DEPTH_ORDER)
            b["order.depth"] = z
            return f"Depth order {z:.0f} m"
        p = _clip(math.degrees(b.get("order.planes", 0.0)) - 0.5 * d, -25.0, 25.0)
        b["order.planes"] = p * DEG
        return f"Planes {p:+.0f}°"

    def depth_preset(self, k):
        z = DEPTH_PRESETS[k]
        self.bus["order.depth"] = z
        if not self.auto:
            self.toggle_autopilot()
        return f"Depth order {z:.0f} m"

    def all_stop(self):
        if self.auto:
            self.bus["order.speed"] = 0.0
        else:
            self.bus["order.rpm"] = 0.0
        return "ALL STOP"

    def hold(self):
        b = self.bus
        if self.auto:
            b["order.heading"] = b.get("state.heading", 0.0) % (2 * math.pi)
            b["order.depth"] = max(b.get("state.depth", 0.0), 0.0)
            return "Holding course & depth"
        b["order.rudder"] = 0.0
        b["order.planes"] = 0.0
        return "Rudder & planes amidships"

    def toggle_autopilot(self):
        b = self.bus
        if self.auto:
            b["order.rpm"] = b.get("propulsion.rpm", 0.0)
            b["order.rudder"] = b.get("act.rudder", 0.0)
            b["order.planes"] = b.get("act.planes", 0.0)
            b["autopilot.enabled"] = False
            return "MANUAL control"
        b["order.heading"] = b.get("state.heading", 0.0) % (2 * math.pi)
        b["order.depth"] = max(b.get("state.depth", 0.0), 0.0)
        b["order.speed"] = max(b.get("state.u", 0.0), 0.0)
        b["autopilot.enabled"] = True
        return "AUTOPILOT engaged"

    def ping(self):
        self._sub("active_sonar").ping()
        return "Active ping — you are counter-detectable"

    def towed(self):
        s = self._sub("passive_sonar")
        s.towed_deployed = not s.towed_deployed
        return "Towed array " + ("deployed" if s.towed_deployed else "recovered")

    def _mast(self, attr, name):
        m = self._sub("masts")
        if not getattr(m, attr) and self.bus.get("state.depth", 0.0) > 18.0:
            return f"Too deep for {name} (> 18 m)"
        setattr(m, attr, not getattr(m, attr))
        return f"{name.capitalize()} " + ("raised" if getattr(m, attr) else "lowered")

    def periscope(self):
        return self._mast("periscope", "periscope")

    def snorkel(self):
        return self._mast("snorkel", "snorkel")

    def gps(self):
        if not self.bus.get("nav.gps_available", False):
            return "No GPS: raise a mast at periscope depth"
        self._sub("navigation").gps_fix()
        return "GPS fix taken"

    def blow(self):
        self._sub("ballast").emergency_blow()
        return "EMERGENCY BLOW"
