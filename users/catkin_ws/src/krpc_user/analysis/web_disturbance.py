"""Estimate the outside push on the web: actual acceleration minus what our controller commanded."""
import json, math, re, sys, os
sys.path.insert(0, '/home/claude/kibo/scripts')
import geometry as g, fanctl, fans
run = sys.argv[1]
loc = [json.loads(l) for l in open(os.path.join(run, 'judge_module_logs/localization_history.txt'))]
ib2 = [(float(m.group(1)), [float(m.group(2)), float(m.group(3)), float(m.group(4))])
       for m in re.finditer(r"t=([\d.]+) x:([-\d.]+) y:([-\d.]+) z:([-\d.]+)", open(os.path.join(run, 'flight_recorder_logs/ib2.log')).read())]
def find(p, start):
    best, bi = 1e9, start
    for i in range(start, min(len(loc), start + 4000)):
        d = sum((loc[i]['position'][k] - p[k]) ** 2 for k in range(3))
        if d < best: best, bi = d, i
    return bi
anchors, idx = [], 0
for t, p in ib2:
    idx = find(p, idx); anchors.append((t, idx))
rate = (anchors[-1][1] - anchors[0][1]) / (anchors[-1][0] - anchors[0][0])
t_first = anchors[0][0] - anchors[0][1] / rate
def at(t):
    i = int(round((t - t_first) * rate)); i = max(0, min(len(loc) - 1, i)); return loc[i]
def pos(t): return at(t)['position']
H = 0.5
def vel(t): return [(pos(t + H)[k] - pos(t - H)[k]) / (2 * H) for k in range(3)]
def acc(t): return [(pos(t + H)[k] - 2 * pos(t)[k] + pos(t - H)[k]) / (H * H) for k in range(3)]
# legs from the user log: "[ x s] fan -> checkpoint N ..." with mission-clock times; judge Mission Start time offsets them
log = open(os.path.join(run, 'user_programs_logs/user_program_output.log')).read()
judge = open(os.path.join(run, 'judge_module_logs/judge.log')).read()
t_ms = float(re.search(r"\[([\d.]+)\]: status arrived: Mission Start", judge).group(1))
course = json.loads(re.search(r"course json: (\{.*\})", log).group(1))
cp = dict((c['checkpoint_id'], c) for c in course['checkpoints'])
legs = [(float(m.group(1)), int(m.group(3))) for m in re.finditer(r"\[\s*([\d.]+) s\] fan -> (checkpoint|goal) (\d+)", log)]
stop = re.search(r"\[\s*([\d.]+) s\] fan mode stopped", log)
t_stop = float(stop.group(1)) + t_ms if stop else 1e9
bias = [float(x) / 1000 for x in re.search(r"nav velocity bias \['([-\d.]+)', '([-\d.]+)', '([-\d.]+)'\]", log).groups()]
ctl = fanctl.FanController(); ctl.vel_bias = [0.0, 0.0, 0.0]
print("Mission Start at %.2f s; fan mode stopped at %s" % (t_ms, "%.1f" % t_stop if stop else "never"))
for n, (t_leg, cid) in enumerate(legs):
    t0 = t_ms + t_leg + 0.15
    t_end = min(t_ms + legs[n + 1][0] if n + 1 < len(legs) else t0 + 25, t_stop)
    p0, q0 = pos(t0), at(t0)['orientation']
    leg = fanctl.Leg(p0, q0, cp[cid]['position'], cp[cid]['attitude']['quaternion'])
    rows = []
    t = t0 + 1.0
    while t < t_end - 0.6:
        nav_p, nav_q = pos(t - 0.1), at(t - 0.1)['orientation']     # nav is 0.1 s old
        F, T = ctl.wrench(t, (nav_p, vel(t - 0.1), nav_q, [0.0, 0.0, 0.0]), leg.ref(t - t0))
        a_cmd = g.v_scale(g.q_rotate(at(t)['orientation'], F), 1.0 / fanctl.MASS)
        a_real = acc(t)
        d = g.v_sub(a_real, a_cmd)
        rows.append((t - t_ms, g.v_norm(d) * fanctl.MASS, d, ctl.last_load))
        t += 0.5
    if not rows: continue
    worst = max(rows, key=lambda r: r[1])
    mean = sum(r[1] for r in rows) / len(rows)
    print("leg -> %2d (mission %5.1f s): outside push mean %5.1f mN, max %5.1f mN at %5.1f s, dir %s, fan load up to %.2f" % (
        cid, t_leg, mean * 1000, worst[1] * 1000, worst[0], [round(x / g.v_norm(worst[2]), 2) for x in worst[2]], max(r[3] for r in rows)))
