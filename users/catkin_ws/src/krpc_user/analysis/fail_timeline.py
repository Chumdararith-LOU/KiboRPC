import json, math, re, sys, os
sys.path.insert(0, '/home/claude/kibo/scripts')
import geometry as g, fanctl
run, cid_to = sys.argv[1], int(sys.argv[2])
L = [json.loads(l) for l in open(os.path.join(run, 'judge_module_logs/localization_history.txt'))]
ib2 = [(float(m.group(1)), [float(m.group(2)), float(m.group(3)), float(m.group(4))])
       for m in re.finditer(r"t=([\d.]+) x:([-\d.]+) y:([-\d.]+) z:([-\d.]+)", open(os.path.join(run, 'flight_recorder_logs/ib2.log')).read())]
def find(p, start):
    best, bi = 1e9, start
    for i in range(start, min(len(L), start + 4000)):
        d = sum((L[i]['position'][k] - p[k]) ** 2 for k in range(3))
        if d < best: best, bi = d, i
    return bi
anchors, idx = [], 0
for t, p in ib2:
    idx = find(p, idx); anchors.append((t, idx))
rate = (anchors[-1][1] - anchors[0][1]) / (anchors[-1][0] - anchors[0][0]); t_first = anchors[0][0] - anchors[0][1] / rate
at = lambda t: L[max(0, min(len(L) - 1, int(round((t - t_first) * rate))))]
def acc(t, h=0.25):
    pm, p0, pp = at(t - h)['position'], at(t)['position'], at(t + h)['position']
    return [(pp[k] - 2 * p0[k] + pm[k]) / (h * h) for k in range(3)]
def accs(t):
    a = [acc(t + d) for d in (-0.2, -0.1, 0.0, 0.1, 0.2)]
    return [sum(x[k] for x in a) / 5 for k in range(3)]
log = open(os.path.join(run, 'user_programs_logs/user_program_output.log')).read()
judge = open(os.path.join(run, 'judge_module_logs/judge.log')).read()
t_ms = float(re.search(r"\[([\d.]+)\]: status arrived: Mission Start", judge).group(1))
course = json.loads(re.search(r"course json: (\{.*\})", log).group(1)); cp = dict((c['checkpoint_id'], c) for c in course['checkpoints'])
t_leg = float(re.search(r"\[\s*([\d.]+) s\] fan -> (checkpoint|goal) %d:" % cid_to, log).group(1)) + t_ms + 0.15
stop = re.search(r"\[\s*([\d.]+) s\] fan mode stopped", log); t_stop = float(stop.group(1)) + t_ms
p0, q0 = at(t_leg)['position'], at(t_leg)['orientation']
leg = fanctl.Leg(p0, q0, cp[cid_to]['position'], cp[cid_to]['attitude']['quaternion'])
print("leg -> %d (%.2f m, turn %.0f deg), hand-back after %.1f s" % (cid_to, leg.d, math.degrees(leg.ang), t_stop - t_leg))
t = 0.5
while t < t_stop - t_leg + 0.5:
    a = accs(t_leg + t); r = leg.ref(t)
    ex = g.v_sub(a, r[2]); q = at(t_leg + t)['orientation']
    eb = g.q_rotate(g.q_conj(q), ex); n = g.v_norm(ex)
    err = g.distance(at(t_leg + t)['position'], r[0])
    print("  t=%4.1f s  track err %5.3f m | extra accel %.4f m/s2 = %4.0f mN, body dir [%5.2f %5.2f %5.2f]" % (t, err, n, n * 3216, eb[0] / n, eb[1] / n, eb[2] / n))
    t += 0.5
