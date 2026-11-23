import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts")); sys.path.insert(0, os.path.join(HERE, "..", "tests"))
sys.argv = [sys.argv[0], sys.argv[1]]
import test_offline as T
import planner, geometry as g
from wrench_model import WrenchModel
cases = [("eta 0.45", WrenchModel(eta_f=0.45, eta_t=0.3)), ("eta 0.6", WrenchModel(eta_f=0.6, eta_t=0.35)),
         ("eta 0.75", WrenchModel(eta_f=0.75, eta_t=0.4, v_max=0.3))]
for level in ("hard", "veryhard"):
    for name, m in cases:
        ts = []
        for seed in range(1, 21):
            _, payload = T.make_course(level, seed)
            cps = payload["checkpoints"]; gid = payload["goalpoint_id"]
            stops = [(c["checkpoint_id"], c["position"], c["attitude"]["quaternion"]) for c in cps if c["checkpoint_id"] != gid]
            gc = [c for c in cps if c["checkpoint_id"] == gid][0]
            order, est = planner.plan_route(([0.2, 0, 0], list(g.IDENTITY)), stops, (gid, gc["position"], gc["attitude"]["quaternion"]), m)
            ts.append(est)
        ts.sort()
        print("%-9s %-9s all CPs: median %3.0f s (%3.0f-%3.0f), fit 270 s: %2d/20" % (level, name, ts[10], ts[0], ts[-1], sum(t <= 270 for t in ts)))
