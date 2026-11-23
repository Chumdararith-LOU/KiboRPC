"""Our own Int-Ball2 controller: fly and turn at the same time, through /ctl/wrench.

Pure Python 3.6 (no numpy) so it runs inside the krpc_user node.

One leg = a straight line from p0 to p1 and a single-axis turn from q0 to q1, both starting
at once, each with a rest-to-rest bang-coast-bang profile. The acceleration of the line is set
per leg from what the fans can push along that line in every attitude the robot passes through
(fans.py), times ETA_F, so the fans keep spare power for corrections and for turning.

Tracking: PD + feed-forward on position (world frame) and attitude (body frame), on a state that
is predicted 0.1 s ahead (the navigation data is 0.1 s old). Before sending, the wrench is
scaled so no fan is asked for more than LOAD_MAX of its maximum, which keeps JAXA's fan
saturation rule (4 fans over the top -> the rest switch off) from ever triggering.
"""
import math
from collections import deque

import geometry as g
import fans

MASS = 3.216
INERTIA = [0.0136245, 0.0139967, 0.0130568]
ETA_F = 0.6            # share of the fans' push used for the planned acceleration
ETA_T = 0.35           # share of the fans' torque used for the planned turn
V_MAX = 0.25           # m/s
W_MAX = 0.5            # rad/s
LOAD_MAX = 0.95        # never ask a fan for more than 95 % of 0.06 N
NAV_DELAY = 0.1        # s, sim.yaml nav_parameter.delay
POS_WN, POS_ZETA = 1.2, 0.9
ATT_WN, ATT_ZETA = 2.5, 0.9
SAMPLES = 9
PLAN_LOAD = 0.8        # v2.1: a leg's planned push + twist together use at most 80 % of any fan,
                       # leaving room for corrections (web gravity module: pulses up to 0.14 N)
VC_MAX = 0.06         # v2.1: the position correction asks for at most 6 cm/s of approach speed and
WC_MAX = 0.2          # the attitude correction for 0.2 rad/s (far off course, damping must dominate)
KI = 0.4               # v2.1: slow correction for a steady outside push (web baseline about 7 mN)
I_ZONE_M = 0.05        # ... and only learned within 5 cm of the reference
I_MAX = 0.02           # m/s2 cap on that correction (about 64 mN)


def _profile(d, vmax, a):
    """Rest-to-rest bang-coast-bang: returns (t_acc, t_coast, v_peak, total)."""
    if d <= 1e-9:
        return 0.0, 0.0, 0.0, 0.0
    if d > vmax * vmax / a:
        ta = vmax / a
        tc = (d - vmax * vmax / a) / vmax
        return ta, tc, vmax, 2.0 * ta + tc
    ta = math.sqrt(d / a)
    return ta, 0.0, a * ta, 2.0 * ta


def _sample(t, a, prof):
    """(s, s_dot, s_ddot) of a profile at time t."""
    ta, tc, vp, tot = prof
    if tot <= 0.0:
        return 0.0, 0.0, 0.0
    t = max(0.0, min(t, tot))
    if t < ta:
        return 0.5 * a * t * t, a * t, a
    s1 = 0.5 * a * ta * ta
    if t < ta + tc:
        return s1 + vp * (t - ta), vp, 0.0
    tr = tot - t
    total_s = 2.0 * s1 + vp * tc
    if tr <= 0.0:
        return total_s, 0.0, 0.0
    return total_s - 0.5 * a * tr * tr, a * tr, -a


def force_cap(u_body):
    l = fans.load(u_body, [0.0, 0.0, 0.0])
    return 1.0 / l if l > 1e-12 else 1.0


def torque_cap(axis_body):
    l = fans.load([0.0, 0.0, 0.0], axis_body)
    return 1.0 / l if l > 1e-12 else 1.0


def rotvec(q):
    """Rotation vector (axis * angle) of a quaternion, shortest way."""
    if q[3] < 0.0:
        q = [-x for x in q]
    s = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2)
    if s < 1e-12:
        return [0.0, 0.0, 0.0]
    ang = 2.0 * math.atan2(s, q[3])
    return [q[0] / s * ang, q[1] / s * ang, q[2] / s * ang]


class Leg(object):
    """Reference trajectory for one leg."""

    def __init__(self, p0, q0, p1, q1, eta_f=ETA_F, eta_t=ETA_T, v_max=V_MAX, w_max=W_MAX):
        self.p0, self.q0, self.p1, self.q1 = list(p0), g.q_normalize(q0), list(p1), g.q_normalize(q1)
        self.d = g.distance(p0, p1)
        self.u = g.v_scale(g.v_sub(p1, p0), 1.0 / self.d) if self.d > 1e-6 else [1.0, 0.0, 0.0]
        cap = min(force_cap(g.q_rotate(g.q_conj(g.q_slerp(self.q0, self.q1, k / float(SAMPLES - 1))), self.u))
                  for k in range(SAMPLES)) if self.d > 1e-6 else 0.1
        self.acc = eta_f * cap / MASS
        self.prof_p = _profile(self.d, v_max, self.acc)
        dq = g.q_mul(g.q_conj(self.q0), self.q1)
        if dq[3] < 0.0:
            dq = [-x for x in dq]
        s = math.sqrt(dq[0] ** 2 + dq[1] ** 2 + dq[2] ** 2)
        self.ang = 2.0 * math.atan2(s, dq[3])
        self.axis = [dq[0] / s, dq[1] / s, dq[2] / s] if s > 1e-9 else [1.0, 0.0, 0.0]
        inertia = sum(INERTIA[k] * self.axis[k] ** 2 for k in range(3))
        self.alpha = eta_t * torque_cap(self.axis) / inertia
        # v2.1: push and twist share the same fans; scale both until their peaks fit PLAN_LOAD.
        k = self._joint_scale()
        self.acc *= k
        self.alpha *= k
        self.prof_p = _profile(self.d, v_max, self.acc)
        self.prof_r = _profile(self.ang, w_max, self.alpha)
        self.duration = max(self.prof_p[3], self.prof_r[3])

    def _peak_load(self, k):
        worst = 0.0
        T = [INERTIA[i] * self.axis[i] * self.alpha * k for i in range(3)]
        for j in range(SAMPLES):
            q = g.q_slerp(self.q0, self.q1, j / float(SAMPLES - 1))
            F = g.q_rotate(g.q_conj(q), g.v_scale(self.u, MASS * self.acc * k))
            for sf in (1.0, -1.0):
                for st in (1.0, -1.0):
                    worst = max(worst, fans.load([x * sf for x in F], [x * st for x in T]))
        return worst

    def _joint_scale(self):
        if self.d <= 1e-6 and self.ang <= 1e-6:
            return 1.0
        if self._peak_load(1.0) <= PLAN_LOAD:
            return 1.0
        lo, hi = 0.0, 1.0
        for _ in range(14):
            mid = 0.5 * (lo + hi)
            if self._peak_load(mid) <= PLAN_LOAD:
                lo = mid
            else:
                hi = mid
        return max(lo, 0.05)

    def s_at(self, t):
        """Distance along the line the reference has covered at time t."""
        return _sample(t, self.acc, self.prof_p)[0]

    def ref(self, t):
        s, sd, sdd = _sample(t, self.acc, self.prof_p)
        p = g.v_add(self.p0, g.v_scale(self.u, s))
        v = g.v_scale(self.u, sd)
        a = g.v_scale(self.u, sdd)
        th, thd, thdd = _sample(t, self.alpha, self.prof_r)
        q = g.q_normalize(g.q_mul(self.q0, g.q_from_axis_angle(self.axis, th)))
        w = g.v_scale(self.axis, thd)          # body frame (axis is fixed in the body)
        al = g.v_scale(self.axis, thdd)
        return p, v, a, q, w, al


class FanController(object):
    def __init__(self):
        self.kp = POS_WN * POS_WN
        self.kd = 2.0 * POS_ZETA * POS_WN
        self.kpa = ATT_WN * ATT_WN
        self.kda = 2.0 * ATT_ZETA * ATT_WN
        self.cmd_hist = deque()     # (time, world accel, body angular accel) actually commanded
        self.last_load = 0.0
        self.scaled = 0
        self.integ = [0.0, 0.0, 0.0]      # v2.1: integral of the position error (world frame)
        self.t_last = None
        self.limited = False
        self.vel_bias = [0.0, 0.0, 0.0]   # nav velocity bias (sim.yaml: about 7 mm/s), measured at rest
        self._bias_sum = [0.0, 0.0, 0.0]
        self._bias_n = 0

    def learn_bias(self, nav_v):
        """Feed navigation velocities while the robot is known to be still (docked, before the start)."""
        self._bias_n += 1
        self._bias_sum = [self._bias_sum[k] + nav_v[k] for k in range(3)]
        self.vel_bias = [x / self._bias_n for x in self._bias_sum]

    def _predict(self, now, p, v, q, w):
        """Push the 0.1 s old navigation state forward with what we commanded since."""
        t_nav = now - NAV_DELAY
        while self.cmd_hist and self.cmd_hist[0][0] < t_nav - 0.2:
            self.cmd_hist.popleft()
        p, v, w = list(p), list(v), list(w)
        q = list(q)
        hist = list(self.cmd_hist)
        for i, (tc, a_w, al_b) in enumerate(hist):
            t_end = hist[i + 1][0] if i + 1 < len(hist) else now
            t_beg = max(tc, t_nav)
            if t_end <= t_beg:
                continue
            dt = t_end - t_beg
            p = [p[k] + v[k] * dt + 0.5 * a_w[k] * dt * dt for k in range(3)]
            v = [v[k] + a_w[k] * dt for k in range(3)]
            wn = math.sqrt(sum(x * x for x in w))
            if wn > 1e-12:
                q = g.q_normalize(g.q_mul(q, g.q_from_axis_angle(w, wn * dt)))
            w = [w[k] + al_b[k] * dt for k in range(3)]
        return p, v, q, w

    def wrench(self, now, nav, ref):
        """nav = (p, v, q, w) as reported; ref = (p, v, a, q, w, alpha). Returns body (F, T)."""
        p0, v0, q0, w0 = nav
        v0 = [v0[k] - self.vel_bias[k] for k in range(3)]
        p, v, q, w = self._predict(now, p0, v0, q0, w0)
        pr, vr, ar, qr, wr, alr = ref
        dt = 0.0 if self.t_last is None else max(0.0, min(0.1, now - self.t_last))
        self.t_last = now
        if not self.limited and g.distance(pr, p) < I_ZONE_M:   # anti-windup: only near the reference
            self.integ = [self.integ[k] + (pr[k] - p[k]) * dt for k in range(3)]  # and while the fans have room
        corr = [KI * x for x in self.integ]
        n = math.sqrt(sum(x * x for x in corr))
        if n > I_MAX:
            self.integ = [x * I_MAX / n for x in self.integ]
            corr = [x * I_MAX / n for x in corr]
        vc = [self.kp / self.kd * (pr[k] - p[k]) for k in range(3)]
        n = math.sqrt(sum(x * x for x in vc))
        if n > VC_MAX:
            vc = [x * VC_MAX / n for x in vc]
        a_cmd = [ar[k] + self.kd * (vr[k] + vc[k] - v[k]) + corr[k] for k in range(3)]
        e = rotvec(g.q_mul(g.q_conj(q), qr))          # attitude error, body frame
        wr_b = g.q_rotate(g.q_conj(q), g.q_rotate(qr, wr))   # reference rate in the current body frame
        wc = [self.kpa / self.kda * x for x in e]
        n = math.sqrt(sum(x * x for x in wc))
        if n > WC_MAX:
            wc = [x * WC_MAX / n for x in wc]
        al_cmd = [alr[k] + self.kda * (wr_b[k] + wc[k] - w[k]) for k in range(3)]
        Iw = [INERTIA[k] * w[k] for k in range(3)]
        gyro = g.v_cross(w, Iw)
        T = [INERTIA[k] * al_cmd[k] + gyro[k] for k in range(3)]
        F_w = [MASS * a_cmd[k] for k in range(3)]
        F = g.q_rotate(g.q_conj(q), F_w)
        F, T = self._limit(F, T)
        # remember what was really commanded (after limiting) for the predictor
        a_real = g.v_scale(g.q_rotate(q, F), 1.0 / MASS)
        al_real = [(T[k] - gyro[k]) / INERTIA[k] for k in range(3)]
        self.cmd_hist.append((now, a_real, al_real))
        return F, T

    def _limit(self, F, T):
        load = fans.load(F, T)
        self.last_load = load
        self.limited = load > LOAD_MAX
        if load <= LOAD_MAX:
            return F, T
        self.scaled += 1
        if fans.load([0.0, 0.0, 0.0], T) > LOAD_MAX * 0.6:        # keep some force authority
            k = LOAD_MAX * 0.6 / fans.load([0.0, 0.0, 0.0], T)
            T = [x * k for x in T]
        lo, hi = 0.0, 1.0
        for _ in range(20):
            mid = 0.5 * (lo + hi)
            if fans.load([x * mid for x in F], T) <= LOAD_MAX:
                lo = mid
            else:
                hi = mid
        return [x * lo for x in F], T

    def brake(self, now, nav):
        """v2.1: bring the robot to rest where it is (before handing back to JAXA's controller)."""
        p0, v0, q0, w0 = nav
        v0 = [v0[k] - self.vel_bias[k] for k in range(3)]
        p, v, q, w = self._predict(now, p0, v0, q0, w0)
        a_cmd = [-self.kd * v[k] for k in range(3)]
        al_cmd = [-self.kda * w[k] for k in range(3)]
        Iw = [INERTIA[k] * w[k] for k in range(3)]
        gyro = g.v_cross(w, Iw)
        T = [INERTIA[k] * al_cmd[k] + gyro[k] for k in range(3)]
        F = g.q_rotate(g.q_conj(q), [MASS * x for x in a_cmd])
        F, T = self._limit(F, T)
        a_real = g.v_scale(g.q_rotate(q, F), 1.0 / MASS)
        al_real = [(T[k] - gyro[k]) / INERTIA[k] for k in range(3)]
        self.cmd_hist.append((now, a_real, al_real))
        return F, T, math.sqrt(sum(x * x for x in v))

    def stop(self):
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
