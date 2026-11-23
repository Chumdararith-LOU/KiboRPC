import os, sys, random
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "..", "scripts")); sys.path.insert(0, os.path.join(HERE, "..", "tests"))
sys.argv = [sys.argv[0], sys.argv[1]]
import test_offline as T_
import course_test as C, ib2sim
cases = {
  "baseline": {},
  "push 6 mN": dict(dist_force=0.006),
  "push 12 mN": dict(dist_force=0.012),
  "nav delay 0.2 s": dict(nav_delay=0.2),
  "control 20 Hz": "hz20",
  "mass +8%": "mass",
}
for name, kw in cases.items():
    if kw == "hz20":
        C.CTL_HZ = 20.0; kw = {}
    else:
        C.CTL_HZ = 50.0
    if kw == "mass":
        ib2sim.MASS = 3.216 * 1.08; kw = {}
    else:
        ib2sim.MASS = 3.216
    res = []
    for seed in (1, 4, 7):
        _, payload = T_.make_course("veryhard", seed)
        res.append(C.run(payload, seed, **kw))
    print("%-16s Lv3 %s | goal %s | mission %s | overloads %s" % (name, [sum(1 for l in r[2].values() if l == 3) for r in res],
          [r[3] for r in res], ["%.0f" % r[0] for r in res], [r[5] for r in res]))
