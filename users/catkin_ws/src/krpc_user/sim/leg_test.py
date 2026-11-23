"""Fly single legs in the offline simulator with our controller; report timing and accuracy."""
import math, os, sys, random
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import geometry as g
from ib2sim import IB2Sim
import fanctl

CTL_HZ = 50.0


def fly(sim, ctl, p1, q1, hold=8.0, log=False):
    p0, v0, q0, w0 = sim.nav()
    leg = fanctl.Leg(p0, q0, p1, q1)
    t0 = sim.t
    worst_over, max_load = 0.0, 0.0
    settled_at = None
    while sim.t - t0 < leg.duration + hold:
        ref = leg.ref(sim.t - t0)
        F, T = ctl.wrench(sim.t, sim.nav(), ref)
        max_load = max(max_load, ctl.last_load)
        sim.command(F, T)
        sim.step(1.0 / CTL_HZ)
        p, v, q, w = sim.truth()
        err = g.distance(p, p1)
        aerr = g.q_angle_deg(q, q1)
        speed = math.sqrt(sum(x * x for x in v))
        if sim.t - t0 > leg.duration:
            worst_over = max(worst_over, err)
            if settled_at is None and err < 0.03 and aerr < 3.0 and speed < 0.01:
                settled_at = sim.t - t0
    p, v, q, w = sim.truth()
    return dict(d=leg.d, ang=math.degrees(leg.ang), acc=leg.acc, plan=leg.duration,
                settled=settled_at, err=g.distance(p, p1), aerr=g.q_angle_deg(q, q1),
                max_err_after=worst_over, load=max_load, sat=sim.saturations, scaled=ctl.scaled)


if __name__ == "__main__":
    rng = random.Random(3)
    sim = IB2Sim(seed=1)
    ctl = fanctl.FanController()
    sim.step(0.3)
    p = [0.2, 0.0, 0.0]
    for i in range(8):
        d = [0.5, 1.0, 2.0, 3.0, 5.0, 1.5, 0.3, 4.0][i]
        u = [rng.gauss(0, 1) for _ in range(3)]; u = g.v_unit(u)
        p1 = g.v_add(p, g.v_scale(u, d))
        q1 = g.q_normalize([rng.gauss(0, 1) for _ in range(4)])
        r = fly(sim, ctl, p1, q1)
        print("leg %d: %.1f m, turn %3.0f deg, acc %.4f m/s2 | plan %5.1f s, settled +%s s | final err %.1f mm %.2f deg | max err after plan %.1f mm | peak fan load %.2f | sim saturations %d"
              % (i, r["d"], r["ang"], r["acc"], r["plan"],
                 "%.1f" % (r["settled"] - r["plan"]) if r["settled"] is not None else "never",
                 r["err"] * 1000, r["aerr"], r["max_err_after"] * 1000, r["load"], r["sat"]))
        p = p1
