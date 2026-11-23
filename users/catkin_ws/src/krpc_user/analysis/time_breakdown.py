"""Where does the time go on the best all-checkpoint route? (stock controller model)"""
import math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts")); sys.path.insert(0, os.path.join(HERE, "..", "tests"))
sys.argv = [sys.argv[0], sys.argv[1]]
import test_offline as T, planner, geometry as g
M = planner.MotionModel(accel=0.010, v_max=0.20, ang_accel=0.18, w_max=0.4363, settle=3.0)
WEB = 1.0    # whole web missions came in at 0.95-1.03x the planner estimate
for level in ("hard", "veryhard"):
    acc = dict(trans=0, gap=0, turn=0, stop=0, dist=0, deg=0, n=0)
    N = 30
    for seed in range(1, N + 1):
        _, pl = T.make_course(level, seed)
        cps = pl["checkpoints"]; gid = pl["goalpoint_id"]
        stops = [(c["checkpoint_id"], c["position"], c["attitude"]["quaternion"]) for c in cps if c["checkpoint_id"] != gid]
        gc = [c for c in cps if c["checkpoint_id"] == gid][0]
        order, _ = planner.plan_route(([0.2, 0, 0], list(g.IDENTITY)), stops, (gid, gc["position"], gc["attitude"]["quaternion"]), M)
        by = dict((c["checkpoint_id"], c) for c in cps)
        p, q = [0.2, 0, 0], list(g.IDENTITY)
        for cid in order:
            c = by[cid]; p1, q1 = c["position"], c["attitude"]["quaternion"]
            d = g.distance(p, p1); a = g.q_angle_deg(q, q1)
            acc["trans"] += M.profile_time(d, M.v_max, M.accel) * WEB
            acc["turn"] += M.profile_time(math.radians(a), M.w_max, M.ang_accel) * WEB
            acc["gap"] += 0
            acc["stop"] += M.settle + 1.0     # settle/report + 1 s judge hold
            acc["dist"] += d; acc["deg"] += a; acc["n"] += 1
            p, q = p1, q1
    k = 1.0 / N
    tot = (acc["trans"] + acc["gap"] + acc["turn"] + acc["stop"]) * k
    print("%-8s stops %d | flying %.0f s (%.1f m) | pause before turn %.0f s | turning %.0f s (%.0f deg) | stop+report %.0f s | TOTAL %.0f s -> over by %.0f s"
          % (level, acc["n"] / N, acc["trans"] * k, acc["dist"] * k, acc["gap"] * k, acc["turn"] * k, acc["deg"] * k, acc["stop"] * k, tot, tot - 300))
