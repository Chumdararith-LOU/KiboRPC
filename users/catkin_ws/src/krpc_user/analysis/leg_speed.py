"""v2.4 study: what limits our fan legs, and how much do faster-leg settings save? (planned routes, 30 courses/level)"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
sys.path.insert(0, os.path.join(HERE, "..", "tests"))
sys.argv = [sys.argv[0], sys.argv[1]]
import test_offline as T
import planner, geometry as g, fanctl


class FanModel(object):          # = fanmode.FanModel (fanmode needs rospy)
    def move_time(self, p0, q0, p1, q1, include_settle=True):
        return fanctl.Leg(p0, q0, p1, q1).duration + (1.6 if include_settle else 0.0)

VARIANTS = [("v2.3", {}), ("load 0.9", dict(PLAN_LOAD=0.9)), ("vmax 0.30", dict(V_MAX=0.30)),
            ("balance", dict(BALANCE=True)), ("balance+vmax0.30", dict(BALANCE=True, V_MAX=0.30)),
            ("balance+load0.9", dict(BALANCE=True, PLAN_LOAD=0.9)),
            ("v2.1 rule, eta_f 0.8", dict(ETA_F=0.8)), ("v2.1 rule, eta 0.8/0.8", dict(ETA_F=0.8, ETA_T=0.8)), ("balance, caps 0.6/0.35", dict(BALANCE=True, ETA_F_MAX=0.6, ETA_T_MAX=0.35)),
            ("balance+load0.75", dict(BALANCE=True, PLAN_LOAD=0.75))]
base = dict((k, getattr(fanctl, k)) for k in ("PLAN_LOAD", "V_MAX", "BALANCE", "ETA_F", "ETA_T", "ETA_F_MAX", "ETA_T_MAX"))

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

stats = {}
for level in ("easy", "normal", "hard", "veryhard"):
    legs = [l for seed in range(1, 31) for l in legs_of(level, seed)]
    for name, over in VARIANTS:
        for k, v in base.items(): setattr(fanctl, k, v)
        for k, v in over.items(): setattr(fanctl, k, v)
        tot, turn_bound, vcap, loadcap = 0.0, 0, 0, 0
        for (p0, q0), (p1, q1) in legs:
            L = fanctl.Leg(p0, q0, p1, q1)
            tot += L.duration
            turn_bound += L.prof_r[3] > L.prof_p[3] + 0.05
            vcap += L.prof_p[1] > 0.0
            loadcap += L._load_of(L.acc, L.alpha) > fanctl.PLAN_LOAD - 0.01
        stats[(level, name)] = tot / 30.0
        if name == "v2.3":
            print("%-8s %d legs: turn decides the time on %d%%, top speed reached on %d%%, at the load cap on %d%%" % (
                level, len(legs), 100 * turn_bound // len(legs), 100 * vcap // len(legs), 100 * loadcap // len(legs)))
for k, v in base.items(): setattr(fanctl, k, v)
print("\nflight time per course (s), mean of 30:")
print("%-24s" % "" + "".join("%10s" % l for l in ("easy", "normal", "hard", "veryhard")))
for name, _ in VARIANTS:
    print("%-24s" % name + "".join("%10.1f" % stats[(l, name)] for l in ("easy", "normal", "hard", "veryhard")))
