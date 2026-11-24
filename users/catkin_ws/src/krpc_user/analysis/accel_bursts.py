"""How hard is the robot accelerated, second by second? Quadratic fit of position over 1 s windows."""
import json, math, re, sys, os
run = sys.argv[1]
loc = [json.loads(l)['position'] for l in open(os.path.join(run, 'judge_module_logs/localization_history.txt'))]
ib2 = [(float(m.group(1)), [float(m.group(2)), float(m.group(3)), float(m.group(4))])
       for m in re.finditer(r"t=([\d.]+) x:([-\d.]+) y:([-\d.]+) z:([-\d.]+)", open(os.path.join(run, 'flight_recorder_logs/ib2.log')).read())]
def find(p, start):
    best, bi = 1e9, start
    for i in range(start, min(len(loc), start + 4000)):
        d = sum((loc[i][k] - p[k]) ** 2 for k in range(3))
        if d < best: best, bi = d, i
    return bi
anchors, idx = [], 0
for t, p in ib2:
    idx = find(p, idx); anchors.append((t, idx))
rate = (anchors[-1][1] - anchors[0][1]) / (anchors[-1][0] - anchors[0][0])
t_first = anchors[0][0] - anchors[0][1] / rate
W = int(rate * 1.0)
acc = []
for c in range(W, len(loc) - W, int(rate * 0.5)):
    # least-squares quadratic per axis over [c-W, c+W]
    ts = [(i - c) / rate for i in range(c - W, c + W + 1, 4)]
    a = []
    for k in range(3):
        ys = [loc[i][k] for i in range(c - W, c + W + 1, 4)]
        n = len(ts); s0 = n; s1 = sum(ts); s2 = sum(t * t for t in ts); s3 = sum(t ** 3 for t in ts); s4 = sum(t ** 4 for t in ts)
        y0 = sum(ys); y1 = sum(t * y for t, y in zip(ts, ys)); y2 = sum(t * t * y for t, y in zip(ts, ys))
        # solve [s4 s3 s2; s3 s2 s1; s2 s1 s0] [A B C] = [y2 y1 y0]
        import itertools
        M = [[s4, s3, s2, y2], [s3, s2, s1, y1], [s2, s1, s0, y0]]
        for i in range(3):
            piv = M[i][i]
            M[i] = [x / piv for x in M[i]]
            for j in range(3):
                if j != i:
                    f = M[j][i]; M[j] = [M[j][q] - f * M[i][q] for q in range(4)]
        a.append(2 * M[0][3])
    acc.append((t_first + c / rate, math.sqrt(sum(x * x for x in a))))
vals = sorted(x[1] for x in acc)
pct = lambda p: vals[int(p * (len(vals) - 1))]
big = [x for x in acc if x[1] > 0.03]
print("%-40s |a| median %.4f, 90%% %.4f, 99%% %.4f, max %.4f m/s2 | seconds above 0.03: %.1f" % (
    os.path.basename(run.rstrip('/'))[-40:], pct(0.5), pct(0.9), pct(0.99), vals[-1], len(big) * 0.5))
if '--times' in sys.argv:
    print("   bursts at:", ["%.0f" % t for t, a in big][:40])
