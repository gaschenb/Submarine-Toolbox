"""Autopilot: heading, depth and speed hold.

Heading:  heading error -> yaw-rate command -> rudder.
Depth:    depth error -> vertical-rate command -> pitch command -> planes,
          plus variable-ballast assist at low speed where planes lose lift.
Speed:    thrust = drag feedforward + P -> rpm.

The rate loops are scheduled on dynamic pressure: they compute the moment
needed for a fixed closed-loop response and invert the surface lift
(~ u^2) to a deflection, so behaviour is the same at 3 kn and 18 kn.
Integrators use conditional integration (anti-windup).

When disabled, manual orders (order.rudder / order.planes / order.rpm)
pass straight through to the actuators.
"""

from __future__ import annotations

import math

from submarine_toolbox.core.subsystem import Subsystem

DEG = math.pi / 180.0

def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi

def _clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

class Autopilot(Subsystem):
    name = "autopilot"

    K_PSI = 0.06            # heading error -> yaw-rate command [1/s]
    W_R = 0.25              # yaw-rate loop bandwidth [rad/s]
    R_MAX = 2.0 * DEG
    K_Z = 0.03              # depth error -> vertical-rate command [1/s]
    KI_Z = 2e-4
    VZ_MAX = 1.5
    MAX_PITCH = 20 * DEG
    W_TH, Z_TH = 0.15, 0.9  # closed-loop pitch natural freq / damping
    K_VZ = 0.3              # vertical-rate error -> extra pitch
    AUTH = 0.6              # share of surface authority usable in steady state
    U_MIN = 0.3
    ASSIST_U = (1.5, 3.0)   # full ballast assist below, none above [m/s]
    VZ_BAL = 0.25           # vertical rate achievable on ballast alone [m/s]
    K_BAL = 15000.0         # kg per (m/s) vertical-rate error
    BAL_MAX = 8000.0
    K_ZB = 0.015            # depth gain while on ballast (slow actuator)
    KP_SPD, KI_SPD = 40.0, 0.4

    def __init__(self, sim):
        super().__init__(sim)
        self._z_prev = None
        self._z_int = 0.0
        self._u_int = 0.0

    def _lift_per_rad(self, area: float, u: float) -> float:
        cs = self.sim.get_subsystem("control_surfaces")
        cl = cs.cl if cs else 3.0
        return 0.5 * self.sim.params.rho * u * u * area * cl

    def _rpm_feedforward(self, u_ord: float) -> float:
        """Invert the thrust curve: rpm such that thrust == hull drag."""
        p = self.sim.params
        prop = self.sim.get_subsystem("propulsion")
        if prop is None or u_ord <= 0.0:
            return 0.0
        drag = p.d_lin[0] * u_ord + p.d_quad[0] * u_ord ** 2
        k = (1.0 - prop.t_ded) * prop.Kt * p.rho * prop.D ** 4
        return 60.0 * math.sqrt(drag / k)

    def update(self, dt: float) -> None:
        if not self.get("autopilot.enabled", True):
            self.put("cmd.rudder", self.get("order.rudder", 0.0))
            self.put("cmd.planes", self.get("order.planes", 0.0))
            self.put("cmd.rpm", self.get("order.rpm", 0.0))
            self.put("cmd.var_assist", 0.0)
            self._z_prev = None
            self._z_int = 0.0
            self._u_int = 0.0
            return

        sim, p = self.sim, self.sim.params
        nu = sim.nu
        Md = sim.dynamics.Md
        cs = sim.get_subsystem("control_surfaces")
        Ar, Ap = (cs.Ar, cs.Ap) if cs else (6.0, 8.0)
        xr, xp = (abs(cs.xr), abs(cs.xp)) if cs else (25.0, 22.0)
        max_r, max_p = (cs.max_r, cs.max_p) if cs else (30 * DEG, 25 * DEG)
        u = self.get("state.u", 0.0)
        ue = max(u, self.U_MIN)

        # heading: rate limit set by available rudder moment
        r = float(nu[5])
        kN = xr * self._lift_per_rad(Ar, ue)
        r_lim = min(self.R_MAX, self.AUTH * kN * max_r / p.d_lin[5])
        e_psi = wrap_pi(self.get("order.heading", 0.0) - self.get("state.heading", 0.0))
        r_cmd = _clip(self.K_PSI * e_psi, -r_lim, r_lim)
        N_cmd = Md[5] * self.W_R * (r_cmd - r) \
            + p.d_lin[5] * r + p.d_quad[5] * abs(r) * r \
            + (Md[1] - Md[0]) * u * float(nu[1])
        rudder = _clip(N_cmd / kN, -max_r, max_r)

        # depth: pitch envelope set by available plane moment vs BG stiffness
        W = self.get("ballast.weight", p.m * p.g) + self.get("damage.flood_weight", 0.0)
        B = p.rho * p.g * p.volume
        zg = self.get("ballast.zg", p.zg)
        Kh = max(zg * W - p.zb * B, 1.0)
        kM = xp * self._lift_per_rad(Ap, ue)
        s_max = min(math.sin(self.MAX_PITCH), self.AUTH * kM * max_p / Kh)

        z = self.get("state.depth", 0.0)
        if self._z_prev is None:
            self._z_prev = z
        z_dot = (z - self._z_prev) / dt
        self._z_prev = z
        z_ord = self.get("order.depth", z)
        if self.get("ballast.mbt", 1.0) > 0.5:
            z_ord = max(z_ord, p.R + 1.0)   # dived trim cannot hold the surface
        e_z = z_ord - z
        lo, hi = self.ASSIST_U
        wb = _clip((hi - u) / (hi - lo), 0.0, 1.0)
        vz_pl = min(self.VZ_MAX, 0.8 * ue * s_max)
        vz_lim = max(vz_pl, wb * self.VZ_BAL, 0.05)
        kz = self.K_Z + wb * (self.K_ZB - self.K_Z)
        ki = self.KI_Z * (kz / self.K_Z) ** 2
        vz_p = kz * e_z
        vz_cmd = _clip(vz_p + ki * self._z_int, -vz_lim, vz_lim)
        if abs(e_z) < 5.0 and abs(vz_p + ki * self._z_int) < vz_lim:
            self._z_int = _clip(self._z_int + e_z * dt, -2000.0, 2000.0)
        s = (_clip(vz_cmd, -vz_pl, vz_pl)
             + (1.0 - wb) * self.K_VZ * (vz_cmd - z_dot)) / ue
        theta_ord = -math.asin(_clip(s, -s_max, s_max))

        theta = self.get("state.pitch", 0.0)
        q = float(nu[4])
        Mi = Md[4]
        a_ = max(self.W_TH ** 2 - Kh / Mi, 0.0)
        b_ = max(2.0 * self.Z_TH * self.W_TH - p.d_lin[4] / Mi, 0.0)
        xg = (self.get("ballast.xg", 0.0) * self.get("ballast.weight", W)
              + self.get("damage.flood_moment_x", 0.0)) / W
        M_cmd = Mi * (a_ * (theta_ord - theta) - b_ * q) \
            + Kh * math.sin(theta_ord) \
            + (xg * W - p.xb * B) * math.cos(theta) \
            - (Md[2] - Md[0]) * u * float(nu[2])
        planes = _clip(M_cmd / kM, -max_p, max_p)

        # ballast carries the vertical rate the planes cannot
        vz_b = vz_cmd - _clip(vz_cmd, -vz_pl, vz_pl)
        ff = (p.d_lin[2] * vz_b + p.d_quad[2] * abs(vz_b) * vz_b
              - self._lift_per_rad(Ap, u) * self.get("act.planes", 0.0)) / p.g
        var_assist = wb * _clip(ff + self.K_BAL * (vz_cmd - z_dot),
                                -self.BAL_MAX, self.BAL_MAX)

        # speed
        prop = sim.get_subsystem("propulsion")
        rpm_max = (prop.max_rpm if prop else 200.0) * self.get("damage.propulsion_limit", 1.0)
        u_ord = self.get("order.speed", 0.0)
        e_u = u_ord - u
        rpm_u = self._rpm_feedforward(u_ord) + self.KP_SPD * e_u
        rpm = _clip(rpm_u + self.KI_SPD * self._u_int, 0.0, rpm_max)
        if u_ord > 0.0 and abs(e_u) < 0.3 and 0.0 < rpm < rpm_max:
            self._u_int = _clip(self._u_int + e_u * dt, -200.0, 200.0)

        self.put("cmd.rudder", rudder)
        self.put("cmd.planes", planes)
        self.put("cmd.rpm", rpm)
        self.put("cmd.var_assist", var_assist)
        self.put("autopilot.theta_ord", theta_ord)
        self.put("autopilot.vz_cmd", vz_cmd)
