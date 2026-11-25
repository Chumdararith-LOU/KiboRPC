"""Estimate: fly each leg nose-first (body +-x along the path, the strong push axis) instead of slerping the attitude.
Rough model: turn to align (t1), push along the strong axis (tp), turn to the checkpoint attitude (t2); the turns
overlap half with the push. Compared with the v2.4/2.5 balanced leg time."""
import os, sys, math
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts")); sys.path.insert(0, os.path.join(HERE, "..", "tests"))
sys.argv = [sys.argv[0], sys.argv[1]]
import test_offline as T
import planner, geometry as g, fanctl, fans


class FanModel(object):
    def move_time(self, p0, q0, p1, q1, include_settle=True):
        return fanctl.Leg(p0, q0, p1, q1).duration + (1.6 if include_settle else 0.0)


def legs_of(level, seed):
    _, payload = T.make_course(level, seed)
    cps = payload["checkpoints"]; gid = payload["goalpoint_id"]
    stops = [(c["checkpoint_id"], c["position"], c["attitude"]["quaternion"]) for c in cps if c["checkpoint_id"] != gid]
    gc = [c for c in cps if c["checkpoint_id"] == gid][0]
    order, est, score = planner.plan_route_score(([0.2, 0, 0], list(g.IDENTITY)), stops,
                        (gid, gc["position"], gc["attitude"]["quaternion"]), FanModel(), 300, 6, 1.05, 1.0)
    pose = dict((c["checkpoint_id"], (c["position"], c["attitude"]["quaternion"])) for c in cps)
    path = [([0.2, 0, 0], list(g.IDENTITY))] + [pose[i] for i in order]
    return list(zip(path[:-1], path[1:]))

def turn_time(q_from, q_to):
    dq = g.q_mul(g.q_conj(q_from), q_to)
    if dq[3] < 0: dq = [-x for x in dq]
    s = math.sqrt(sum(x * x for x in dq[:3])); ang = 2 * math.atan2(s, dq[3])
    if ang < 1e-6: return 0.0
    ax = [x / s for x in dq[:3]]
    inertia = sum(fanctl.INERTIA[k] * ax[k] ** 2 for k in range(3))
    al = 0.8 * fanctl.torque_cap(ax) / inertia
    return fanctl._profile(ang, fanctl.W_MAX, al)[3]

def align(q0, u):
    """Attitude closest to q0 whose body +x or -x points along u."""
    best = None
    for sgn in (1.0, -1.0):
        bx = g.q_rotate(q0, [sgn, 0, 0])
        c = max(-1.0, min(1.0, g.v_dot(bx, u))); ang = math.acos(c)
        axis = g.v_cross(bx, u); n = g.v_norm(axis)
        q = q0 if n < 1e-9 else g.q_mul(g.q_from_axis_angle([x / n for x in axis], ang), q0)
        if best is None or ang < best[0]: best = (ang, q)
    return best[1]

for level in ("easy", "normal", "hard", "veryhard"):
    old = new = 0.0
    for seed in range(1, 31):
        for (p0, q0), (p1, q1) in legs_of(level, seed):
            L = fanctl.Leg(p0, q0, p1, q1); old += L.duration
            d = g.distance(p0, p1)
            if d < 0.3:
                new += L.duration; continue
            u = g.v_scale(g.v_sub(p1, p0), 1 / d)
            qa = align(q0, u)
            a = 0.8 * fanctl.force_cap(g.q_rotate(g.q_conj(qa), u)) / fanctl.MASS
            tp = fanctl._profile(d, fanctl.V_MAX, a)[3]
            t = tp + 0.5 * (turn_time(q0, qa) + turn_time(qa, q1))
            new += min(t, L.duration)
    print("%-8s flight per course: today %.1f s, nose-first estimate %.1f s (saves %.1f s, ~%.1f pt)" % (level, old / 30, new / 30, (old - new) / 30, (old - new) / 30 * 0.55))
