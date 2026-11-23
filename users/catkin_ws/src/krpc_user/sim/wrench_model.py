"""Leg-time model for our own fan controller (fly and turn at the same time)."""
import math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import geometry as g
import fans

MASS = 3.216
INERTIA = [0.0136245, 0.0139967, 0.0130568]


def force_cap(u_body):
    l = fans.load(u_body, [0.0, 0.0, 0.0])
    return 1.0 / l if l > 1e-12 else 1.0


def torque_cap(axis_body):
    l = fans.load([0.0, 0.0, 0.0], axis_body)
    return 1.0 / l if l > 1e-12 else 1.0


def profile_time(d, vmax, a):
    if d <= 1e-9:
        return 0.0
    if d > vmax * vmax / a:
        return d / vmax + vmax / a
    return 2.0 * math.sqrt(d / a)


class WrenchModel(object):
    """Translation and rotation run in parallel; the leg takes the longer of the two.
    Acceleration = eta_f x the weakest push along the travel direction over the attitudes
    the robot passes through (slerp q0 -> q1); angular acceleration = eta_t x torque cap."""

    def __init__(self, eta_f=0.6, eta_t=0.35, v_max=0.25, w_max=0.5, stop=3.5, samples=9):
        self.eta_f, self.eta_t, self.v_max, self.w_max = eta_f, eta_t, v_max, w_max
        self.stop, self.samples = stop, samples

    def leg_accel(self, p0, q0, p1, q1):
        d = g.distance(p0, p1)
        if d < 1e-6:
            return 1.0, d
        u = g.v_scale(g.v_sub(p1, p0), 1.0 / d)
        cap = min(force_cap(g.q_rotate(g.q_conj(g.q_slerp(q0, q1, k / float(self.samples - 1))), u))
                  for k in range(self.samples))
        return self.eta_f * cap / MASS, d

    def leg_alpha(self, q0, q1):
        dq = g.q_mul(g.q_conj(q0), q1)          # rotation in the start body frame
        s = math.sqrt(dq[0] ** 2 + dq[1] ** 2 + dq[2] ** 2)
        if s < 1e-9:
            return 1.0, 0.0
        axis = [dq[0] / s, dq[1] / s, dq[2] / s]
        ang = 2.0 * math.atan2(s, abs(dq[3]))
        inertia = sum(INERTIA[k] * axis[k] ** 2 for k in range(3))
        return self.eta_t * torque_cap(axis) / inertia, ang

    def move_time(self, p0, q0, p1, q1, include_settle=True):
        a, d = self.leg_accel(p0, q0, p1, q1)
        al, ang = self.leg_alpha(q0, q1)
        t = max(profile_time(d, self.v_max, a), profile_time(ang, self.w_max, al))
        return t + (self.stop if include_settle else 0.0)
