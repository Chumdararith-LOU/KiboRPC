# -*- coding: utf-8 -*-
"""Offline tests: our scorer vs the simulator's real judge, the aim fix,
and the route planner vs brute force.

Run:  python3 -I tests/test_offline.py <path to krpc7th-simulator repo>
"""
import contextlib
import io
import itertools
import json
import math
import os
import random
import runpy
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import geometry as g  # noqa: E402
import planner  # noqa: E402
import scorer  # noqa: E402

REPO = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/chumdararith-lou/kiborpc")
GEN_DIR = os.path.join(REPO, "rootfs/krpc/random_env_generator")
JUDGE = os.path.join(REPO, "rootfs/krpc/judge_module/judge.py")
LOADER_DIR = os.path.join(REPO, "users/catkin_ws/src/krpc_api_server/scripts")
WORK = "/tmp/kibo_offline_courses"


# ---------------------------------------------------------------------------
# Load the real judge with ROS stubbed out
# ---------------------------------------------------------------------------
def load_referee_class():
    for name in ("rospy", "tf", "requests", "geometry_msgs", "geometry_msgs.msg",
                 "std_msgs", "std_msgs.msg", "ib2_msgs", "ib2_msgs.msg",
                 "krpc_msgs", "krpc_msgs.srv"):
        sys.modules.setdefault(name, types.ModuleType(name))
    for mod, attrs in (("geometry_msgs.msg", ["Pose"]), ("std_msgs.msg", ["Bool", "String"]),
                       ("ib2_msgs.msg", ["Navigation"]),
                       ("krpc_msgs.srv", ["EvaluateCheckpoint", "EvaluateCheckpointResponse"])):
        for a in attrs:
            setattr(sys.modules[mod], a, type(a, (), {}))
    class _T(object):
        @staticmethod
        def now():
            o = _T()
            o.to_sec = lambda: 0.0
            return o
    sys.modules["rospy"].Time = _T
    sys.modules["rospy"].logwarn = lambda *a, **k: None
    ns = runpy.run_path(JUDGE, run_name="judge_under_test")
    return ns["Referee"]


class _P(object):
    pass


def judge_verdict(Referee, cheat, cp_id, pos, q):
    r = object.__new__(Referee)
    r.cheat_data = cheat
    r.sim_mode = "regular"
    r._checkpoint_rules, r._checkpoint_defaults = r.load_checkpoint_rules()
    pose = _P()
    pose.position, pose.orientation = _P(), _P()
    pose.position.x, pose.position.y, pose.position.z = pos
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    r.ib2_pose = pose
    ok, reason, detail = r.evaluate_checkpoint_navigation(cp_id)
    return detail


# ---------------------------------------------------------------------------
# Courses from the simulator's own generator
# ---------------------------------------------------------------------------
def make_course(level, seed):
    out = os.path.join(WORK, "%s_%d" % (level, seed))
    if not os.path.isdir(out) or not os.listdir(out):
        os.makedirs(out, exist_ok=True)
        sys.path.insert(0, GEN_DIR)
        random.seed(seed)
        argv, cwd = sys.argv, os.getcwd()
        sys.argv = ["generate_objects.py", "-l", level, "-o", out]
        os.chdir(GEN_DIR)
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                runpy.run_path("generate_objects.py", run_name="__main__")
        finally:
            sys.argv = argv
            os.chdir(cwd)
    path = os.path.join(out, sorted(os.listdir(out))[0])
    with open(path) as f:
        cheat = json.load(f)
    sys.path.insert(0, LOADER_DIR)
    from cheat_sheet_loader import CheatSheetLoader
    payload = json.loads(json.dumps(CheatSheetLoader(cheat, path).checkpoint_payload()))
    return cheat, payload


def random_rotation(max_deg, rng):
    axis = [rng.gauss(0, 1) for _ in range(3)]
    return g.q_from_axis_angle(axis, math.radians(rng.uniform(0, max_deg)))


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def test_scorer_matches_judge(Referee, n_courses=12, poses_per_cp=60):
    rng = random.Random(1)
    checked = mismatched = 0
    worst = 0.0
    for i, level in enumerate(["easy", "normal", "hard", "veryhard"] * (n_courses // 4)):
        cheat, payload = make_course(level, 100 + i)
        for cp in payload["checkpoints"]:
            spec = scorer.CheckpointSpec(cp)
            for _ in range(poses_per_cp):
                off = [rng.uniform(-0.35, 0.35) for _ in range(3)]
                pos = g.v_add(spec.position, off)
                if rng.random() < 0.2:
                    q = g.q_normalize([rng.gauss(0, 1) for _ in range(4)])
                else:
                    q = g.q_normalize(g.q_mul(random_rotation(45, rng), spec.target_q))
                ours = scorer.evaluate(pos, q, spec)
                theirs = judge_verdict(Referee, cheat, spec.id, pos, q)
                checked += 1
                diffs = [abs(ours["distance"] - theirs["distance"]),
                         abs(ours["centering_deg"] - theirs["centering_angle_deg"]),
                         abs(ours["roll_deg"] - theirs["roll_angle_deg"])]
                worst = max(worst, max(diffs))
                if ours["level"] != theirs["selected_band"]:
                    mismatched += 1
    assert mismatched == 0, "%d of %d verdicts differ" % (mismatched, checked)
    assert worst < 1e-6, "largest numeric difference %g" % worst
    print("scorer vs judge: %d poses, 0 level mismatches, max numeric diff %.1e" % (checked, worst))


def test_aim_fix(n=2000):
    rng = random.Random(2)
    _, payload = make_course("veryhard", 7)
    worst_cen = worst_roll = 0.0
    upgraded = 0
    total = 0
    for _ in range(n):
        cp = rng.choice(payload["checkpoints"])
        spec = scorer.CheckpointSpec(cp)
        pos = g.v_add(spec.position, [rng.uniform(-0.09, 0.09) for _ in range(3)])
        if g.distance(pos, spec.position) > 0.1:
            continue
        total += 1
        before = scorer.evaluate(pos, spec.target_q, spec)
        q = g.aim_from_here(pos, spec.marker_position, spec.target_q)
        after = scorer.evaluate(pos, q, spec)
        worst_cen = max(worst_cen, after["centering_deg"])
        worst_roll = max(worst_roll, after["roll_deg"])
        assert after["level"] == 3
        if before["level"] < 3:
            upgraded += 1
    assert worst_cen < 1e-4 and worst_roll < 1e-4  # degrees; acos is imprecise near 0
    print("aim fix: %d poses within 10 cm all Lv3 after the fix; %d were below Lv3 with the "
          "plain target attitude" % (total, upgraded))


def test_planner_is_optimal(trials=40):
    rng = random.Random(3)
    model = planner.MotionModel()
    for _ in range(trials):
        n = rng.randint(1, 6)
        pts = [([rng.uniform(0.2, 7), rng.uniform(-0.8, 0.85), rng.uniform(0, 1.68)],
                g.q_normalize([rng.gauss(0, 1) for _ in range(4)])) for _ in range(n + 1)]
        start = ([0.2, 0.0, 0.0], list(g.IDENTITY))
        stops = [(k, p, q) for k, (p, q) in enumerate(pts[:-1])]
        goal = ("goal", pts[-1][0], pts[-1][1])
        order, t = planner.plan_route(start, stops, goal, model)
        brute = min(planner.route_time(start, [pts[k] for k in perm] + [pts[-1]], model)
                    for perm in itertools.permutations(range(n)))
        legs = [pts[k] for k in order[:-1]] + [pts[-1]]
        assert abs(planner.route_time(start, legs, model) - t) < 1e-9
        assert abs(t - brute) < 1e-9, (t, brute)
    print("planner: Held-Karp equals brute force on %d random courses" % trials)


def test_planner_speed():
    import time
    _, payload = make_course("veryhard", 11)
    cps = payload["checkpoints"]
    goal_id = payload["goalpoint_id"]
    stops = [(c["checkpoint_id"], c["position"], c["attitude"]["quaternion"])
             for c in cps if c["checkpoint_id"] != goal_id]
    gcp = [c for c in cps if c["checkpoint_id"] == goal_id][0]
    t0 = time.time()
    order, est = planner.plan_route(([0.2, 0, 0], list(g.IDENTITY)), stops,
                                    (goal_id, gcp["position"], gcp["attitude"]["quaternion"]),
                                    planner.MotionModel())
    dt = time.time() - t0
    assert order[-1] == goal_id and len(order) == len(cps)
    print("planner: 11 stops + goal planned in %.2f s (estimated mission %.0f s)" % (dt, est))


def test_score_planner_is_optimal(trials=60):
    """v1.8: plan_route_score equals brute force over every subset and order."""
    import itertools
    rng = random.Random(7)
    model = planner.MotionModel(accel=0.010, v_max=0.20, ang_accel=0.18, w_max=0.4363, settle=3.0)
    for trial in range(trials):
        n = rng.randint(1, 6)
        pt = lambda: [rng.uniform(0.2, 7.0), rng.uniform(-0.8, 0.85), rng.uniform(0.0, 1.68)]
        stops = [(k + 1, pt(), random_rotation(180, rng)) for k in range(n)]
        goal = (12, pt(), random_rotation(180, rng))
        start = ([0.2, 0, 0], list(g.IDENTITY))
        limit, reserve, pad, gf = rng.uniform(60, 220), 6.0, 1.03, 1.15
        order, fin, score = planner.plan_route_score(start, stops, goal, model, limit, reserve, pad, gf)
        def mt(a, b):
            return model.move_time(a[0], a[1], b[0], b[1])
        gnode = (goal[1], goal[2])
        best = planner.mission_score(1, n == 0, gf * mt(start, gnode))
        for r in range(1, n + 1):
            for sub in itertools.permutations(stops, r):
                t, cur, ok = 0.0, start, True
                for s in sub:
                    node = (s[1], s[2])
                    if t + pad * (mt(cur, node) + gf * mt(node, gnode)) + reserve > limit:
                        ok = False
                        break
                    t += mt(cur, node)
                    cur = node
                fin2 = t + gf * mt(cur, gnode)
                if ok and pad * fin2 + reserve <= limit:
                    best = max(best, planner.mission_score(r + 1, r == n, fin2))
        assert abs(best - score) < 1e-6, (trial, best, score)
    print("score planner: equals brute force on %d random courses" % trials)


if __name__ == "__main__":
    Referee = load_referee_class()
    test_scorer_matches_judge(Referee)
    test_aim_fix()
    test_planner_is_optimal()
    test_planner_speed()
    test_score_planner_is_optimal()
    print("ALL TESTS PASSED")
