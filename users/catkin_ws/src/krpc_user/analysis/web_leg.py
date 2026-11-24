"""Rebuild a fan-mode leg from web logs: reference vs actual, and the push needed to explain the gap."""
import json, math, re, sys, os
sys.path.insert(0, '/home/claude/kibo/scripts')
import geometry as g, fanctl
run = sys.argv[1]
leg_from, leg_to = int(sys.argv[2]), int(sys.argv[3])     # checkpoint ids
loc = [json.loads(l) for l in open(os.path.join(run, 'judge_module_logs/localization_history.txt'))]
ib2 = [(float(m.group(1)), [float(m.group(2)), float(m.group(3)), float(m.group(4))])
       for m in re.finditer(r"t=([\d.]+) x:([-\d.]+) y:([-\d.]+) z:([-\d.]+)", open(os.path.join(run, 'flight_recorder_logs/ib2.log')).read())]
# time of each localization line: match ib2 samples (4-decimal positions) to line indices
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
print("localization: %d lines, %.1f Hz, t0 %.2f s" % (len(loc), rate, t_first))
tm = lambda i: t_first + i / rate
judge = open(os.path.join(run, 'judge_module_logs/judge.log')).read()
m = re.search(r"'status': 'CheckPointPassed', 'timestamp': '19700101 (\d{2})(\d{2})(\d{2})(\d{3})', 'checkpoint_id': %d" % leg_from, judge)
t_acc = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + int(m.group(3)) + int(m.group(4)) / 1000.0
cs = json.load(open(os.path.join(run, 'cheat_sheet.json')))
log = open(os.path.join(run, 'user_programs_logs/user_program_output.log')).read()
course = json.loads(re.search(r"course json: (\{.*\})", log).group(1))
cp = dict((c['checkpoint_id'], c) for c in course['checkpoints'])
i0 = int((t_acc + 0.01 - t_first) * rate)
p0, q0 = loc[i0]['position'], loc[i0]['orientation']
leg = fanctl.Leg(p0, q0, cp[leg_to]['position'], cp[leg_to]['attitude']['quaternion'])
print("leg %d -> %d starts t=%.2f: %.2f m, turn %.0f deg, planned accel %.4f m/s2, plan %.1f s" % (leg_from, leg_to, tm(i0), leg.d, math.degrees(leg.ang), leg.acc, leg.duration))
prev = None
for k in range(0, int(min(leg.duration + 3, 40) * 10)):
    t = k / 10.0
    i = i0 + int(t * rate)
    if i >= len(loc): break
    p, q = loc[i]['position'], loc[i]['orientation']
    r = leg.ref(t)
    e = g.v_sub(p, r[0])
    if k % 10 == 0:
        along = g.v_dot(e, leg.u)
        print("t=%5.1f  err %5.3f m (along path %+.3f, across %.3f)  att err %4.1f deg   ref speed %.3f m/s" % (
            t, g.v_norm(e), along, math.sqrt(max(0, g.v_norm(e) ** 2 - along ** 2)), g.q_angle_deg(q, r[3]), g.v_norm(r[1])))
