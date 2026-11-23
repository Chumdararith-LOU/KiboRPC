"""Int-Ball2 fan system, copied from JAXA's simulator (ctl.yaml `fan`, sim.yaml `thr_parameter`,
fsm.cpp, thrust_allocator.cpp, thr.cpp). Pure Python, no numpy, so it can also run in the robot node.

Pipeline in the simulator:  /ctl/wrench (body F, T)
  -> fsm: f = Wp*y+ + Wm*y- ; f -= min(f) ; pwm = kj*sqrt(|f|) ; saturation(n=4, PWMmax=1)
  -> prop -> thr plugin: fan force = min((pwm/Kprop)^2, 0.06), 0.01 s lag, noise N(0, 0.2*f)
  -> body force = sum f_i * vec_i ; body torque = sum f_i * ((pos_i - cg) x vec_i)
"""
FMAX = 0.06            # N per fan
KJ = 4.082482905       # pwm = KJ * sqrt(force); PWMmax 1.0  <=>  0.06 N
N_SAT = 4
CG = [0.001489, 0.001363, 0.000249]
# Allocation weights per fan: (Fx, Fy, Fz, Tx, Ty, Tz) for positive (WP) and negative (WM) commands
WP = [(0.0, 0.0, 0.0, 0.0, 0.0, 7.3257), (0.0, 0.6027, 0.0, 19.8198, 0.0, 0.0),
      (0.0, 0.6027, 0.4911, 0.0, 13.1935, 0.0), (0.0, 0.0, 0.4911, 19.8198, 13.1935, 7.3257),
      (0.3315, 0.0, 0.4911, 19.8198, 0.0, 0.0), (0.3315, 0.0, 0.0, 0.0, 13.1935, 0.0),
      (0.3315, 0.6027, 0.0, 19.8198, 13.1935, 7.3257), (0.3315, 0.6027, 0.4911, 0.0, 0.0, 7.3257)]
WM = [(0.3315, 0.6027, 0.4911, 19.8198, 13.1935, 0.0), (0.3315, 0.0, 0.4911, 0.0, 13.1935, 7.3257),
      (0.3315, 0.0, 0.0, 19.8198, 0.0, 7.3257), (0.3315, 0.6027, 0.0, 0.0, 0.0, 0.0),
      (0.0, 0.6027, 0.0, 0.0, 13.1935, 7.3257), (0.0, 0.6027, 0.4911, 19.8198, 0.0, 7.3257),
      (0.0, 0.0, 0.4911, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 19.8198, 13.1935, 0.0)]
POS = [[0.045, 0.07, 0.0555], [0.045, -0.07, 0.0555], [0.045, -0.07, -0.0555], [0.045, 0.07, -0.0555],
       [-0.045, 0.07, -0.0555], [-0.045, 0.07, 0.0555], [-0.045, -0.07, 0.0555], [-0.045, -0.07, -0.0555]]
_a, _b, _c = 0.754180198, 0.414799109, 0.509071634
VEC = [[-_a, -_b, -_c], [-_a, _b, -_c], [-_a, _b, _c], [-_a, -_b, _c],
       [_a, -_b, _c], [_a, -_b, -_c], [_a, _b, -_c], [_a, _b, _c]]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


ARM = [_cross([p[i] - CG[i] for i in range(3)], v) for p, v in zip(POS, VEC)]


def allocate(F, T):
    """Fan forces the simulator's fsm asks for (before saturation)."""
    y = list(F) + list(T)
    f = []
    for wp, wm in zip(WP, WM):
        f.append(sum(wp[k] * y[k] if y[k] > 0 else wm[k] * -y[k] for k in range(6)))
    m = min(f)
    return [x - m for x in f]


def load(F, T):
    """Highest fan force / FMAX for this wrench (>1 means the simulator saturates)."""
    return max(allocate(F, T)) / FMAX


def saturate(f):
    """fsm.cpp saturation on forces (pwm>1 <=> f>FMAX): 4+ fans over -> top 4 full, rest OFF."""
    over = sum(1 for x in f if x > FMAX)
    if over >= N_SAT:
        idx = sorted(range(len(f)), key=lambda i: -f[i])
        out = [0.0] * len(f)
        for i in idx[:N_SAT]:
            out[i] = FMAX
        return out
    return [min(x, FMAX) for x in f]


def wrench_of(fan_forces):
    F = [sum(f * v[k] for f, v in zip(fan_forces, VEC)) for k in range(3)]
    T = [sum(f * a[k] for f, a in zip(fan_forces, ARM)) for k in range(3)]
    return F, T
