"""How long does the best route through ALL checkpoints take? (sim v1.2 goal rule)"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
sys.path.insert(0, os.path.join(HERE, "..", "tests"))
sys.argv = [sys.argv[0], sys.argv[1]]
import test_offline as T
import planner, geometry as g

MODEL = planner.MotionModel(accel=0.010, v_max=0.20, ang_accel=0.18, w_max=0.4363, settle=3.0)
HOLD = 1.0       # new: judge holds each report for 1 s (also the goal at mission_finished)
PRESTART = 10.0  # start signal -> Mission Start, seen 9-12 s locally
REAL = 0.96      # measured real/estimate over 39 legs

for level in ("easy", "normal", "hard", "veryhard"):
    times = []
    for seed in range(1, 31):
        _, payload = T.make_course(level, seed)
        cps = payload["checkpoints"]; gid = payload["goalpoint_id"]
        stops = [(c["checkpoint_id"], c["position"], c["attitude"]["quaternion"]) for c in cps if c["checkpoint_id"] != gid]
        gc = [c for c in cps if c["checkpoint_id"] == gid][0]
        order, est = planner.plan_route(([0.2, 0, 0], list(g.IDENTITY)), stops,
                                        (gid, gc["position"], gc["attitude"]["quaternion"]), MODEL)
        n = len(order)
        t = REAL * est + HOLD * (n + 1)
        times.append(t)
    times.sort()
    fit_mission = sum(1 for t in times if t <= 300)
    fit_signal = sum(1 for t in times if t + PRESTART <= 300)
    print("%-9s all %2d CPs: median %3.0f s, range %3.0f-%3.0f s | fits 300 s from Mission Start: %2d/30, from start signal: %2d/30"
          % (level, len(order), times[len(times)//2], times[0], times[-1], fit_mission, fit_signal))
