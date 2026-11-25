"""Offline 6-DOF simulator of Int-Ball2 driven through /ctl/wrench.

Copies the parts of JAXA's simulator that matter for a custom controller:
  - fsm allocation + saturation rule, fan force cap 0.06 N, 0.01 s fan lag, 20 % fan noise (fans.py)
  - rigid body: mass 3.216 kg, inertia from ib2.urdf, gyroscopic term
  - navigation: 0.1 s late, position bias about 2 mm + 0.85 mm noise, velocity bias about 7 mm/s
  - a constant unknown disturbance force and torque (stand-in for airflow / the web gravity module)
  - wrench held between commands (fsm only republishes when a new wrench arrives)
"""
import math
import random
from collections import deque

import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import fans          # scripts/fans.py: the one copy, shipped with the program
import geometry as g

MASS = 3.216
INERTIA = [0.0136245, 0.0139967, 0.0130568]


class IB2Sim(object):
    DT = 0.002

    # Web gravity module (probe of 3 web bags, 2026-11-24): Gazebo gravity x re-set every second,
    # baseline about +0.0022 m/s2 with pulses; in ISS coordinates it pushes along -y (Gazebo x = ISS -y).
    WEB_PULSES = [(5.0, -0.0395), (3.0, 0.0054), (5.0, 0.0441), (1.0, -0.0069), (2.0, -0.0335)]
    WEB_GAPS = [9.0, 19.0, 23.0, 0.0]          # seconds between the pulses above, as seen in the logs

    def __init__(self, seed=0, nav_delay=0.1, dist_force=0.002, dist_torque=0.0002, fan_noise=True,
                 web_gravity=False, gravity_start=None, station=None):
        self.rng = random.Random(seed)
        self.t = 0.0
        self.p = [0.2, 0.0, 0.0]
        self.v = [0.0, 0.0, 0.0]
        self.q = list(g.IDENTITY)
        self.w = [0.0, 0.0, 0.0]
        self.cmd_f = [0.0] * 8          # fan force targets after fsm (held)
        self.fan = [0.0] * 8            # actual fan force (lagged)
        self.fan_noise = fan_noise
        self.nav_delay = nav_delay
        self.hist = deque()
        u = [self.rng.gauss(0, 1) for _ in range(3)]
        n = math.sqrt(sum(x * x for x in u)) or 1.0
        self.dist_f = [dist_force * x / n for x in u]          # world frame
        u = [self.rng.gauss(0, 1) for _ in range(3)]
        n = math.sqrt(sum(x * x for x in u)) or 1.0
        self.dist_t = [dist_torque * x / n for x in u]          # body frame
        self.pos_bias = [-0.0021, -0.0018, -0.00007]
        self.vel_bias = [-0.0056, -0.0043, 0.00024]
        self.saturations = 0
        self.gravity = []                        # (t_from, t_to, g_x) in Gazebo x
        if web_gravity:
            t = gravity_start if gravity_start is not None else self.rng.uniform(20.0, 120.0)
            for k, (dur, gx) in enumerate(self.WEB_PULSES):
                self.gravity.append((t, t + dur, gx))
                t += dur + (self.WEB_GAPS[k] if k < len(self.WEB_GAPS) else 0.0)
        self.web_gravity = web_gravity
        self.real_g = None
        if web_gravity == "real":                # v2.5 sim: the 15 recorded web gravity series, chained
            import json
            with open(os.path.join(HERE, "data", "web_gravity.json")) as fh:
                runs = json.load(fh)
            self.real_g = [row[1:] for r in runs for row in r["series"]]
            self.real_off = gravity_start if gravity_start is not None else self.rng.uniform(0.0, len(self.real_g))
        # station contact (sim/station.py): sphere of radius 0.1 m against the JPM mesh
        self.station = station
        self.contacts = []                      # (t, depth m, normal speed m/s, spin after rad/s)
        self.mu = 0.5
        self._near = None                       # cached (position, distance, normal) near a wall
        self._k = 0

    def gravity_push(self, t):
        """World (ISS) force from the web gravity module at time t."""
        if not self.web_gravity:
            return [0.0, 0.0, 0.0]
        sc = getattr(self, 'gravity_scale', 1.0)
        if self.real_g is not None:
            gx, gy, gz = self.real_g[int(t + self.real_off) % len(self.real_g)]
            return [-gy * MASS * sc, -gx * MASS * sc, -gz * MASS * sc]     # Gazebo (x, y, z) -> ours (-y, -x, -z)
        gx = 0.0022
        for a, b, v in self.gravity:
            if a <= t < b:
                gx = v
        return [0.0, -gx * MASS * getattr(self, 'gravity_scale', 1.0), 0.0]

    def command(self, F_body, T_body):
        f = fans.allocate(F_body, T_body)
        if sum(1 for x in f if x > fans.FMAX) >= fans.N_SAT:
            self.saturations += 1
        self.cmd_f = fans.saturate(f)

    def step(self, duration):
        end = self.t + duration
        k = 1.0 - math.exp(-self.DT / 0.01)
        while self.t < end - 1e-12:
            for i in range(8):
                self.fan[i] += k * (self.cmd_f[i] - self.fan[i])
            ff = [getattr(self, 'fan_k', 1.0) * max(0.0, f + (self.rng.gauss(0, 0.2 * f) if self.fan_noise and f > 0 else 0.0))
                  for f in self.fan]
            Fb, Tb = fans.wrench_of(ff)
            Fw = g.q_rotate(self.q, Fb)
            gp = self.gravity_push(self.t)
            mk = getattr(self, 'mass_k', 1.0)
            a = [(Fw[i] + self.dist_f[i]) / (MASS * mk) + gp[i] / MASS for i in range(3)]
            Iw = [INERTIA[i] * self.w[i] for i in range(3)]
            gyro = g.v_cross(self.w, Iw)
            wd = [(Tb[i] + self.dist_t[i] - mk * gyro[i]) / (INERTIA[i] * mk) for i in range(3)]
            self.v = [self.v[i] + a[i] * self.DT for i in range(3)]
            self.p = [self.p[i] + self.v[i] * self.DT for i in range(3)]
            self.w = [self.w[i] + wd[i] * self.DT for i in range(3)]
            wn = math.sqrt(sum(x * x for x in self.w))
            if wn > 1e-12:
                dq = g.q_from_axis_angle(self.w, wn * self.DT)
                self.q = g.q_normalize(g.q_mul(self.q, dq))
            self._k += 1
            if self.station is not None and self._k % 5 == 0:
                self._contact()
            self.t += self.DT
            self.hist.append((self.t, list(self.p), list(self.v), list(self.q), list(self.w)))
            while self.hist and self.hist[0][0] < self.t - self.nav_delay - 0.05:
                self.hist.popleft()

    def _contact(self):
        """Simple contact with the station: no penetration, inelastic normal, Coulomb friction (spins the ball)."""
        import numpy as np
        st = self.station
        if st.coarse(self.p) > 0.1 + 0.08:
            self._near = None
            return
        c = self._near
        if c is None or g.distance(c[0], self.p) > 0.004:
            d, n = st.exact(self.p)
            c = self._near = (list(self.p), d, [float(x) for x in n])
        d = c[1] + g.v_dot(g.v_sub(self.p, c[0]), c[2])                   # first-order update
        depth = 0.1 - d
        if depth <= 0.0:
            return
        n = c[2]
        self.p = [self.p[i] + depth * n[i] for i in range(3)]                   # push out
        vn = g.v_dot(self.v, n)
        if vn >= 0.0:
            return
        Jn = -MASS * vn                                                         # stop the motion into the wall
        self.v = [self.v[i] - vn * n[i] for i in range(3)]
        r_w = [-0.1 * x for x in n]                                             # contact point, world
        w_w = g.q_rotate(self.q, self.w)
        vc = g.v_add(self.v, g.v_cross(w_w, r_w))
        vt = g.v_sub(vc, g.v_scale(n, g.v_dot(vc, n)))
        vts = g.v_norm(vt)
        if vts > 1e-6:
            # impulse that would stop the slip (sphere: effective mass m/(1 + m r^2 / I) along the slip)
            I = sum(INERTIA) / 3.0
            m_eff = MASS / (1.0 + MASS * 0.01 / I)
            Jt_mag = min(self.mu * Jn, m_eff * vts)
            Jt = g.v_scale(vt, -Jt_mag / vts)
            self.v = [self.v[i] + Jt[i] / MASS for i in range(3)]
            dL_w = g.v_cross(r_w, Jt)
            dL_b = g.q_rotate(g.q_conj(self.q), dL_w)
            self.w = [self.w[i] + dL_b[i] / INERTIA[i] for i in range(3)]
        self.contacts.append((self.t, depth, -vn, g.v_norm(self.w)))

    def nav(self):
        """What /sensor_fusion/navigation reports now: the state nav_delay ago, with bias and noise."""
        target = self.t - self.nav_delay
        rec = self.hist[0]
        for r in self.hist:
            if r[0] <= target:
                rec = r
            else:
                break
        _, p, v, q, w = rec
        p = [p[i] + self.pos_bias[i] + self.rng.gauss(0, 0.00085) for i in range(3)]
        v = [v[i] + self.vel_bias[i] + self.rng.gauss(0, 0.0015) for i in range(3)]
        w = [w[i] + self.rng.gauss(0, math.radians(0.015)) for i in range(3)]
        return p, v, q, w

    def truth(self):
        return list(self.p), list(self.v), list(self.q), list(self.w)
