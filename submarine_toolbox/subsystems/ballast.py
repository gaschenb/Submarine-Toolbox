"""Ballast: main ballast tanks (MBT), variable ballast, trim transfer,
HP air bank, emergency blow.

Publishes current weight and CG offsets consumed by the dynamics.
Convention: MBT fill 1.0 = flooded (submerged trim); 0.0 = blown
(surfaced, ~150 t positively buoyant). Neutral: MBT=1.0, var=0.5.
"""

from __future__ import annotations

from submarine_toolbox.core.subsystem import Subsystem

class Ballast(Subsystem):
    name = "ballast"

    MBT_RATE = 0.03
    BLOW_RATE = 0.25
    VAR_RATE = 50.0
    TRIM_RATE = 0.05
    AIR_BLOW_DRAIN = 0.15
    AIR_RECHARGE = 0.005

    def __init__(self, sim):
        super().__init__(sim)
        p = sim.params
        self.mbt = 1.0
        self.mbt_target = 1.0
        self.var = 0.5 * p.var_capacity
        self.var_target = self.var
        self.trim = 0.0
        self.trim_target = 0.0
        self.air = 1.0
        self.blowing = False

    def emergency_blow(self):
        if self.air > 0.0:
            self.blowing = True
            self.mbt_target = 0.0

    def update(self, dt: float) -> None:
        p = self.sim.params

        if self.blowing:
            if self.air <= 0.0 or self.mbt <= 1e-3:
                self.blowing = False
            else:
                self.mbt = max(self.mbt - self.BLOW_RATE * dt, 0.0)
                self.air = max(self.air - self.AIR_BLOW_DRAIN * dt, 0.0)
        else:
            err = self.mbt_target - self.mbt
            step = min(abs(err), self.MBT_RATE * dt)
            self.mbt += step * (1 if err > 0 else -1)

        tgt = min(max(self.var_target + self.get("cmd.var_assist", 0.0), 0.0),
                  p.var_capacity)
        err = tgt - self.var
        self.var += min(abs(err), self.VAR_RATE * dt) * (1 if err > 0 else -1)
        err = self.trim_target - self.trim
        self.trim += min(abs(err), self.TRIM_RATE * dt) * (1 if err > 0 else -1)

        if self.get("state.depth", 99.0) < 5.0:
            self.air = min(self.air + self.AIR_RECHARGE * dt, 1.0)

        mass = p.hull_mass + self.mbt * p.mbt_capacity + self.var
        xg = self.trim * p.trim_arm
        self.put("ballast.weight", mass * p.g)
        self.put("ballast.xg", xg)
        self.put("ballast.zg", p.zg)
        self.put("ballast.mbt", self.mbt)
        self.put("ballast.var", self.var)
        self.put("ballast.trim", self.trim)
        self.put("ballast.air", self.air)
        self.put("ballast.blowing", self.blowing)
