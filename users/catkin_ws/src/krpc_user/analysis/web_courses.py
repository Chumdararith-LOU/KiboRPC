"""Replay the web-sim courses (from our logs) through the planner."""
import json, os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import planner, geometry as g
MODEL = planner.MotionModel(accel=0.010, v_max=0.20, ang_accel=0.18, w_max=0.4363, settle=3.0)
for path in sys.argv[1:]:
    txt = open(path).read()
    m = re.search(r"get_checkpoints[^{]*(\{.*?\"goalpoint_id\": \d+\})", txt)
    data = json.loads(m.group(1))
    cps = data["checkpoints"]; gid = data["goalpoint_id"]
    stops = [(c["checkpoint_id"], c["position"], c["attitude"]["quaternion"]) for c in cps if c["checkpoint_id"] != gid]
    gc = [c for c in cps if c["checkpoint_id"] == gid][0]
    order, est = planner.plan_route(([0.2, 0, 0], list(g.IDENTITY)), stops, (gid, gc["position"], gc["attitude"]["quaternion"]), MODEL)
    print("%s: %d CPs, all-CP route %s est %.0f s -> x0.96 + holds = %.0f s"
          % (os.path.basename(os.path.dirname(os.path.dirname(path))) or path, len(cps), order, est, 0.96*est + len(order) + 1))
