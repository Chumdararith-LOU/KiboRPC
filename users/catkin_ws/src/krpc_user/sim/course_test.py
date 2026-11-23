"""Fly whole courses with our controller in the offline simulator, judged like sim 1.2."""
import math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "..", "scripts")); sys.path.insert(0, os.path.join(HERE, "..", "tests"))
import geometry as g, planner, scorer, fanctl
from ib2sim import IB2Sim

CTL_HZ = 50.0
STOP_EST = 1.6          # planner: seconds per stop (settle check + 1 s judge hold)


class FanModel(object):
    def move_time(self, p0, q0, p1, q1, include_settle=True):
        return fanctl.Leg(p0, q0, p1, q1).duration + (STOP_EST if include_settle else 0.0)


def safe_lv3(ev, spec):
    return ev["level"] == 3 and ev["distance"] <= 0.05 and (ev["centering_deg"] or 0) <= 6 and (ev["roll_deg"] or 0) <= 6


def run(payload, seed, dist_force=0.002, nav_delay=0.1, log=False):
    sim = IB2Sim(seed=seed, dist_force=dist_force, nav_delay=nav_delay)
    ctl = fanctl.FanController()
    for _ in range(50):
        sim.step(0.02); ctl.learn_bias(sim.nav()[1])
    cps = payload["checkpoints"]; gid = payload["goalpoint_id"]
    specs = dict((c["checkpoint_id"], scorer.CheckpointSpec(c)) for c in cps)
    stops = [(c["checkpoint_id"], c["position"], c["attitude"]["quaternion"]) for c in cps if c["checkpoint_id"] != gid]
    gc = specs[gid]
    order, est = planner.plan_route(([0.2, 0, 0], list(g.IDENTITY)), stops, (gid, gc.position, gc.target_q), FanModel())
    t_start = sim.t
    levels = {}
    hold_ref = None
    for cid in order:
        spec = specs[cid]
        p0, v0, q0, w0 = sim.nav()
        leg = fanctl.Leg(p0, q0, spec.position, spec.target_q)
        t0 = sim.t
        still_since = None
        while True:
            ref = leg.ref(sim.t - t0)
            sim.command(*ctl.wrench(sim.t, sim.nav(), ref))
            sim.step(1.0 / CTL_HZ)
            if sim.t - t0 < leg.duration - 0.5:
                continue
            p, v, q, w = sim.nav()
            ev = scorer.evaluate(p, q, spec)
            slow = math.sqrt(sum((v[k] - ctl.vel_bias[k]) ** 2 for k in range(3))) < 0.01
            if safe_lv3(ev, spec) and slow:
                still_since = still_since if still_since is not None else sim.t
                if sim.t - still_since >= 0.3:
                    break
            else:
                still_since = None
            if sim.t - t0 > leg.duration + 20:
                break
        # judge: 1 s hold, lowest level of nav samples at about 30 Hz
        worst = 3
        t_h = sim.t
        while sim.t - t_h < 1.0:
            sim.command(*ctl.wrench(sim.t, sim.nav(), leg.ref(sim.t - t0)))
            sim.step(1.0 / 30.0)
            p, v, q, w = sim.nav()
            worst = min(worst, scorer.evaluate(p, q, spec)["level"])
        levels[cid] = worst
        if log:
            print("  cp %2d: leg %.1f s (plan %.1f), level %d" % (cid, sim.t - t0, leg.duration, worst))
    T = sim.t - t_start
    all_ok = all(levels.get(c["checkpoint_id"], 0) > 0 for c in cps)
    score = (10 * sum(1 for l in levels.values() if l == 3) + (30 if all_ok else 0)) * (1 + (300 - T) / 300.0)
    return T, est, levels, all_ok, score, sim.saturations


if __name__ == "__main__":
    sys.argv = [sys.argv[0], sys.argv[1]] + sys.argv[2:]
    import test_offline as T_
    level = sys.argv[2] if len(sys.argv) > 2 else "veryhard"
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 10
    dist = float(sys.argv[4]) if len(sys.argv) > 4 else 0.002
    for seed in range(1, n + 1):
        _, payload = T_.make_course(level, seed)
        Tm, est, levels, ok, score, sats = run(payload, seed, dist_force=dist)
        lv3 = sum(1 for l in levels.values() if l == 3)
        print("%s seed %2d: %2d/%2d Lv3, goal %s, mission %5.1f s (planned %5.1f), score %5.1f, fan overloads %d"
              % (level, seed, lv3, len(payload["checkpoints"]), ok, Tm, est, score, sats))
