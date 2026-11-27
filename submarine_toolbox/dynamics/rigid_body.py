"""6-DOF equations of motion, Fossen form:

    M nu_dot + C(nu) nu + D(nu) nu + g(eta) = tau

State: eta = [x, y, z, phi, theta, psi] (NED, z down / depth positive)
       nu  = [u, v, w, p, q, r]         (body frame)

M is diagonal, so the RHS is evaluated in scalar form (no small-array
numpy overhead in the integrator hot loop).
"""

from __future__ import annotations

import math

import numpy as np

from submarine_toolbox.dynamics.vessel import VesselParams

def rotation_body_to_ned(phi: float, theta: float, psi: float) -> np.ndarray:
    cf, sf = math.cos(phi), math.sin(phi)
    ct, st = math.cos(theta), math.sin(theta)
    cp, sp = math.cos(psi), math.sin(psi)
    return np.array([
        [cp * ct, -sp * cf + cp * st * sf, sp * sf + cp * cf * st],
        [sp * ct, cp * cf + sf * st * sp, -cp * sf + st * sp * cf],
        [-st, ct * sf, ct * cf],
    ])

class RigidBodyDynamics:
    def __init__(self, params: VesselParams):
        self.p = params
        self.Md = tuple(float(x) for x in np.diag(params.M))
        self.Mi = tuple(1.0 / x for x in self.Md)
        self.dl = tuple(float(x) for x in params.d_lin)
        self.dq = tuple(float(x) for x in params.d_quad)

    def rhs(self, s, tau, W, xg, zg):
        """s = [x,y,z,phi,theta,psi,u,v,w,p,q,r] -> ds/dt (list)."""
        p = self.p
        _, _, z, phi, th, psi, u, v, w, pp, q, r = s
        cf, sf = math.cos(phi), math.sin(phi)
        ct, st = math.cos(th), math.sin(th)
        cp, sp = math.cos(psi), math.sin(psi)
        tt = math.tan(th)
        cts = ct if abs(ct) > 1e-6 else math.copysign(1e-6, ct or 1.0)

        m1, m2, m3, m4, m5, m6 = self.Md
        a0, a1, a2 = m1 * u, m2 * v, m3 * w
        b0, b1, b2 = m4 * pp, m5 * q, m6 * r
        c0 = -(a1 * r - a2 * q)
        c1 = -(a2 * pp - a0 * r)
        c2 = -(a0 * q - a1 * pp)
        c3 = -(a1 * w - a2 * v) - (b1 * r - b2 * q)
        c4 = -(a2 * u - a0 * w) - (b2 * pp - b0 * r)
        c5 = -(a0 * v - a1 * u) - (b0 * q - b1 * pp)

        dl, dq = self.dl, self.dq
        d0 = dl[0] * u + dq[0] * abs(u) * u
        d1 = dl[1] * v + dq[1] * abs(v) * v
        d2 = dl[2] * w + dq[2] * abs(w) * w
        d3 = dl[3] * pp + dq[3] * abs(pp) * pp
        d4 = dl[4] * q + dq[4] * abs(q) * q
        d5 = dl[5] * r + dq[5] * abs(r) * r

        f = min(max((z + p.R) / (2.0 * p.R), 0.15), 1.0)
        B = p.rho * p.g * p.volume * f
        WB = W - B
        mz = zg * W - p.zb * B
        mx = xg * W - p.xb * B
        g0 = WB * st
        g1 = -WB * ct * sf
        g2 = -WB * ct * cf
        g3 = mz * ct * sf
        g4 = mz * st + mx * ct * cf
        g5 = -mx * ct * sf

        Mi = self.Mi
        return [
            cp * ct * u + (-sp * cf + cp * st * sf) * v + (sp * sf + cp * cf * st) * w,
            sp * ct * u + (cp * cf + sf * st * sp) * v + (-cp * sf + st * sp * cf) * w,
            -st * u + ct * sf * v + ct * cf * w,
            pp + sf * tt * q + cf * tt * r,
            cf * q - sf * r,
            (sf * q + cf * r) / cts,
            (tau[0] - c0 - d0 - g0) * Mi[0],
            (tau[1] - c1 - d1 - g1) * Mi[1],
            (tau[2] - c2 - d2 - g2) * Mi[2],
            (tau[3] - c3 - d3 - g3) * Mi[3],
            (tau[4] - c4 - d4 - g4) * Mi[4],
            (tau[5] - c5 - d5 - g5) * Mi[5],
        ]

    def derivatives(self, eta, nu, tau, W, xg, zg):
        d = self.rhs(list(eta) + list(nu), list(tau), W, xg, zg)
        return np.array(d[:6]), np.array(d[6:])
