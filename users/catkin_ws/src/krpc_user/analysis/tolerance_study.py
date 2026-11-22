"""How much faster is the full route if we stop at the edge of the allowed zone
instead of the exact checkpoint centre?  (Estimate: arrival distance shrinks by
the position slack, turn angle shrinks by the angle slack.)"""
import math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts")); sys.path.insert(0, os.path.join(HERE, "..", "tests"))
sys.argv = [sys.argv[0], sys.argv[1]]
import test_offline as T
import planner, geometry as g

class EdgeModel(planner.MotionModel):
    def __init__(self, pos_slack, ang_slack_deg, **kw):
        planner.MotionModel.__init__(self, **kw)
        self.s, self.d = pos_slack, math.radians(ang_slack_deg)
    def move_time(self, p0, q0, p1, q1, include_settle=True):
        t = self.profile_time(max(0.0, g.distance(p0, p1) - self.s), self.v_max, self.accel)
        t += self.profile_time(max(0.0, math.radians(g.q_angle_deg(q0, q1)) - self.d), self.w_max, self.ang_accel)
        return t + (self.settle if include_settle else 0.0) + self.overhead

KW = dict(accel=0.010, v_max=0.20, ang_accel=0.18, w_max=0.4363, settle=3.0)
CASES = [("exact centre (today)", 0.00, 0.0),
         ("Lv3 edge: 5 cm, 6 deg", 0.05, 6.0),
         ("Lv2 edge: 15 cm, 16 deg", 0.15, 16.0),
         ("Lv1 edge: 25 cm, 26 deg", 0.25, 26.0)]
WEB = 1.03   # web whole-mission actual/estimate (W2: 0.95-1.03); use the pessimistic end
for level in ("normal", "hard", "veryhard"):
    print(level)
    for name, s, d in CASES:
        m = EdgeModel(s, d, **KW)
        ts = []
        for seed in range(1, 31):
            _, payload = T.make_course(level, seed)
            cps = payload["checkpoints"]; gid = payload["goalpoint_id"]
            stops = [(c["checkpoint_id"], c["position"], c["attitude"]["quaternion"]) for c in cps if c["checkpoint_id"] != gid]
            gc = [c for c in cps if c["checkpoint_id"] == gid][0]
            order, est = planner.plan_route(([0.2, 0, 0], list(g.IDENTITY)), stops, (gid, gc["position"], gc["attitude"]["quaternion"]), m)
            ts.append(WEB * est + 1.0 * len(order) + 1.0)     # + 1 s judge hold per report and at finish
        ts.sort()
        print("  %-26s all CPs: median %3.0f s (%3.0f-%3.0f)  fits 290 s: %2d/30" % (name, ts[15], ts[0], ts[-1], sum(t <= 290 for t in ts)))
