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

    def __init__(self, seed=0, nav_delay=0.1, dist_force=0.002, dist_torque=0.0002, fan_noise=True):
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
            ff = [max(0.0, f + (self.rng.gauss(0, 0.2 * f) if self.fan_noise and f > 0 else 0.0)) for f in self.fan]
            Fb, Tb = fans.wrench_of(ff)
            Fw = g.q_rotate(self.q, Fb)
            a = [(Fw[i] + self.dist_f[i]) / MASS for i in range(3)]
            Iw = [INERTIA[i] * self.w[i] for i in range(3)]
            gyro = g.v_cross(self.w, Iw)
            wd = [(Tb[i] + self.dist_t[i] - gyro[i]) / INERTIA[i] for i in range(3)]
            self.v = [self.v[i] + a[i] * self.DT for i in range(3)]
            self.p = [self.p[i] + self.v[i] * self.DT for i in range(3)]
            self.w = [self.w[i] + wd[i] * self.DT for i in range(3)]
            wn = math.sqrt(sum(x * x for x in self.w))
            if wn > 1e-12:
                dq = g.q_from_axis_angle(self.w, wn * self.DT)
                self.q = g.q_normalize(g.q_mul(self.q, dq))
            self.t += self.DT
            self.hist.append((self.t, list(self.p), list(self.v), list(self.q), list(self.w)))
            while self.hist and self.hist[0][0] < self.t - self.nav_delay - 0.05:
                self.hist.popleft()

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
