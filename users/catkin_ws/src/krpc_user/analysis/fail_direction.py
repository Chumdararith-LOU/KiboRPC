import json, math, re, sys, os
sys.path.insert(0, '/home/claude/kibo/scripts')
import geometry as g, fanctl, fans
run, cid_from, cid_to = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
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
log = open(os.path.join(run, 'user_programs_logs/user_program_output.log')).read()
judge = open(os.path.join(run, 'judge_module_logs/judge.log')).read()
t_ms = float(re.search(r"\[([\d.]+)\]: status arrived: Mission Start", judge).group(1))
course = json.loads(re.search(r"course json: (\{.*\})", log).group(1)); cp = dict((c['checkpoint_id'], c) for c in course['checkpoints'])
t_leg = float(re.search(r"\[\s*([\d.]+) s\] fan -> (checkpoint|goal) %d:" % cid_to, log).group(1)) + t_ms + 0.15
p0, q0 = at(t_leg)['position'], at(t_leg)['orientation']
leg = fanctl.Leg(p0, q0, cp[cid_to]['position'], cp[cid_to]['attitude']['quaternion'])
print("leg -> %d: %.2f m, turn %.0f deg, ref accel %.4f along %s (body %s)" % (cid_to, leg.d, math.degrees(leg.ang), leg.acc,
      [round(x, 2) for x in leg.u], [round(x, 2) for x in g.q_rotate(g.q_conj(q0), leg.u)]))
for t in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0):
    p = at(t_leg + t)['position']; q = at(t_leg + t)['orientation']
    a = g.v_scale(g.v_sub(p, p0), 2.0 / (t * t))           # mean accel from rest
    r = leg.ref(t); ar = g.v_scale(g.v_sub(r[0], p0), 2.0 / (t * t))
    extra = g.v_sub(a, ar)
    qm = at(t_leg + t / 2)['orientation']
    eb = g.q_rotate(g.q_conj(qm), extra)
    print("  0-%.0f s: actual mean accel %.4f m/s2, reference %.4f, unexplained %.4f m/s2 = %.0f mN, body dir %s, turned %.0f deg" % (
        t, g.v_norm(a), g.v_norm(ar), g.v_norm(extra), g.v_norm(extra) * 3216, [round(x / (g.v_norm(eb) or 1), 2) for x in eb], g.q_angle_deg(q0, q)))
