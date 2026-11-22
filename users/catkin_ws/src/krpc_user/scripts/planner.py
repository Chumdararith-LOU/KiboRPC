# -*- coding: utf-8 -*-
"""Time model and route planner (course Modules 3 and 10).

Pure Python, no ROS. The model parameters are estimates from JAXA's ctl.yaml
and are meant to be replaced by values measured in Gazebo (experiments E2-E4).
"""
import math

import geometry as g


class MotionModel(object):
    """Estimated time for one absolute move: translate, then turn, then settle.

    Defaults: a = 0.4 * 0.181 N / 3.216 kg, v_max = 0.20 m/s,
    alpha = 0.4 * 8.1904e-3 N m / 0.0136 kg m^2, w_max = 0.436 rad/s.
    """

    def __init__(self, accel=0.0225, v_max=0.20, ang_accel=0.2409,
                 w_max=0.4363, settle=3.0, overhead=0.0):
        self.accel = accel
        self.v_max = v_max
        self.ang_accel = ang_accel
        self.w_max = w_max
        self.settle = settle
        self.overhead = overhead

    @staticmethod
    def profile_time(dist, v_max, accel):
        """Bang-coast-bang time to cover `dist` from rest to rest."""
        if dist <= 1e-9:
            return 0.0
        if dist > v_max * v_max / accel:
            return dist / v_max + v_max / accel
        return 2.0 * math.sqrt(dist / accel)

    def move_time(self, p0, q0, p1, q1, include_settle=True):
        t = self.profile_time(g.distance(p0, p1), self.v_max, self.accel)
        t += self.profile_time(math.radians(g.q_angle_deg(q0, q1)), self.w_max, self.ang_accel)
        if include_settle:
            t += self.settle
        return t + self.overhead


def plan_route(start, stops, goal, model):
    """Fastest order to visit every stop, then the goal (Held-Karp, Module 10.4).

    start: (position, quaternion)
    stops: list of (key, position, quaternion), visited in any order
    goal:  (key, position, quaternion), always last
    Returns (ordered list of keys ending with the goal key, estimated seconds).
    """
    n = len(stops)
    nodes = [start] + [(p, q) for _, p, q in stops] + [(goal[1], goal[2])]
    goal_idx = n + 1

    def cost(i, j):
        return model.move_time(nodes[i][0], nodes[i][1], nodes[j][0], nodes[j][1])

    if n == 0:
        return [goal[0]], cost(0, goal_idx)

    c = [[cost(i, j) for j in range(n + 2)] for i in range(n + 2)]
    full = (1 << n) - 1
    inf = float("inf")
    best = [[inf] * n for _ in range(1 << n)]
    parent = [[-1] * n for _ in range(1 << n)]
    for k in range(n):
        best[1 << k][k] = c[0][k + 1]
    for mask in range(1, 1 << n):
        row = best[mask]
        for last in range(n):
            base = row[last]
            if base == inf:
                continue
            for nxt in range(n):
                bit = 1 << nxt
                if mask & bit:
                    continue
                val = base + c[last + 1][nxt + 1]
                if val < best[mask | bit][nxt]:
                    best[mask | bit][nxt] = val
                    parent[mask | bit][nxt] = last
    total, last = min((best[full][k] + c[k + 1][goal_idx], k) for k in range(n))
    order = []
    mask = full
    while last != -1:
        order.append(last)
        prev = parent[mask][last]
        mask ^= 1 << last
        last = prev
    order.reverse()
    return [stops[k][0] for k in order] + [goal[0]], total


def route_time(start, legs, model):
    """Estimated time to fly through `legs` [(position, quaternion), ...]."""
    t = 0.0
    cur = start
    for leg in legs:
        t += model.move_time(cur[0], cur[1], leg[0], leg[1])
        cur = leg
    return t


def plan_route_budget(start, stops, goal, model, time_limit, reserve, pad, t_start=0.0):
    """Best subset + order that fits the time budget (orienteering, Module 10/11).

    A leg to checkpoint c is allowed only if, when leaving for it at time t,
        t + pad * (time(here -> c) + time(c -> goal)) + reserve <= time_limit
    which is exactly the mission's live "go home" rule, so the plan and the
    rule never disagree. Among plans that fit, it maximises the number of
    checkpoints, then minimises the finish time.
    Returns (ordered keys ending with the goal key, estimated finish time).
    """
    n = len(stops)
    nodes = [start] + [(p, q) for _, p, q in stops] + [(goal[1], goal[2])]
    gi = n + 1
    c = [[model.move_time(nodes[i][0], nodes[i][1], nodes[j][0], nodes[j][1])
          for j in range(n + 2)] for i in range(n + 2)]

    def ok(t, i, j):
        return t + pad * (c[i][j] + c[j][gi]) + reserve <= time_limit

    best = {}
    for k in range(n):
        if ok(t_start, 0, k + 1):
            best[(1 << k, k)] = (t_start + c[0][k + 1], -1)
    for mask in range(1, 1 << n):
        for last in range(n):
            entry = best.get((mask, last))
            if entry is None:
                continue
            t = entry[0]
            for nxt in range(n):
                if mask >> nxt & 1 or not ok(t, last + 1, nxt + 1):
                    continue
                key = (mask | 1 << nxt, nxt)
                val = t + c[last + 1][nxt + 1]
                if key not in best or val < best[key][0]:
                    best[key] = (val, last)
    # Pick: most checkpoints, then earliest finish at the goal.
    choice = (0, -(t_start + c[0][gi]), 0, -1)
    for (mask, last), (t, _) in best.items():
        cand = (bin(mask).count("1"), -(t + c[last + 1][gi]), mask, last)
        if cand[:2] > choice[:2]:
            choice = cand
    count, neg_finish, mask, last = choice
    order = []
    while last != -1:
        order.append(last)
        prev = best[(mask, last)][1]
        mask ^= 1 << last
        last = prev
    order.reverse()
    return [stops[k][0] for k in order] + [goal[0]], -neg_finish


def mission_score(n_passed, all_passed, finish_s, cp_points=10.0, goal_bonus=30.0, limit=300.0):
    """Official web score (decoded 2026-11-22 from three dashboard results):
    (10 per Lv3 checkpoint + 30 if every checkpoint passed) x (1 + (300 - T) / 300),
    with T measured from Mission Start."""
    return (cp_points * n_passed + (goal_bonus if all_passed else 0.0)) * (1.0 + (limit - finish_s) / limit)


def plan_route_score(start, stops, goal, model, time_limit, reserve, pad, goal_factor=1.0, t_start=0.0):
    """v1.8: the subset + order with the highest expected official score.

    Feasibility is the same live "go home" rule as plan_route_budget: a leg to
    checkpoint c is allowed only if, leaving at time t,
        t + pad * (time(here -> c) + goal_factor * time(c -> goal)) + reserve <= time_limit
    and the whole plan must also finish within the limit with that padding.
    Among feasible plans it maximises mission_score(), counting the goal
    checkpoint itself (10 pt) and the goal bonus when every stop is in the plan;
    the expected finish uses the goal-leg factor but no padding.
    Times are seconds since Mission Start.
    Returns (ordered keys ending with the goal key, expected finish, expected score).
    """
    n = len(stops)
    nodes = [start] + [(p, q) for _, p, q in stops] + [(goal[1], goal[2])]
    gi = n + 1
    c = [[model.move_time(nodes[i][0], nodes[i][1], nodes[j][0], nodes[j][1])
          for j in range(n + 2)] for i in range(n + 2)]
    home = [goal_factor * c[i][gi] for i in range(n + 2)]

    def ok(t, i, j):
        return t + pad * (c[i][j] + home[j]) + reserve <= time_limit

    best = {}
    for k in range(n):
        if ok(t_start, 0, k + 1):
            best[(1 << k, k)] = (t_start + c[0][k + 1], -1)
    for mask in range(1, 1 << n):
        for last in range(n):
            entry = best.get((mask, last))
            if entry is None:
                continue
            t = entry[0]
            for nxt in range(n):
                if mask >> nxt & 1 or not ok(t, last + 1, nxt + 1):
                    continue
                key = (mask | 1 << nxt, nxt)
                val = t + c[last + 1][nxt + 1]
                if key not in best or val < best[key][0]:
                    best[key] = (val, last)
    # Straight home is always allowed (the goal must be reported anyway).
    fin0 = t_start + home[0]
    choice = (mission_score(1, n == 0, fin0), fin0, 0, -1)
    for (mask, last), (t, _) in best.items():
        fin = t + home[last + 1]
        if t_start + pad * (fin - t_start) + reserve > time_limit:
            continue
        cnt = bin(mask).count("1")
        sc = mission_score(cnt + 1, cnt == n, fin)
        if sc > choice[0] + 1e-9 or (abs(sc - choice[0]) <= 1e-9 and fin < choice[1]):
            choice = (sc, fin, mask, last)
    score, finish, mask, last = choice
    order = []
    while last != -1:
        order.append(last)
        prev = best[(mask, last)][1]
        mask ^= 1 << last
        last = prev
    order.reverse()
    return [stops[k][0] for k in order] + [goal[0]], finish, score
