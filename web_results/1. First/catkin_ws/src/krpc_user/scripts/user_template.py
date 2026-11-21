#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
KongVong - 7th Kibo-RPC mission program (Int-Ball2)

Target environment: Python 3.6.9, ROS Melodic (do not use 3.7+ features).

Flow:
  /ib2_user/start  ->  mission_start  ->  get_checkpoints
  ->  plan shortest route (Held-Karp, goal checkpoint last)
  ->  for each checkpoint: move -> wait stable -> checkpoint_passed (1 retry)
  ->  goal checkpoint  ->  mission_finished

The three mandatory interfaces (/ib2_user/start subscriber, /ib2_user/stop
service, /ib2_user/ready parameter) are kept exactly as the template requires.
"""

import json
import math
import threading

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped

from krpc_ib2_msgs.srv import CheckPoints, MissionStart, MissionFinished
from krpc_ib2_msgs.msg import CheckpointPassedAction, CheckpointPassedGoal
from ib2_msgs.msg import (CtlCommandAction, CtlCommandGoal, CtlCommandResult,
                          CtlStatusType, Navigation)
from platform_msgs.msg import UserLogic
from platform_msgs.srv import StopProcessingUserNode, StopProcessingUserNodeResponse


# ============================================================================
# Tunable settings (change these while testing, one at a time)
# ============================================================================
APP_VERSION = 'kongvong-0.1'

MISSION_TIME_LIMIT = 300.0     # [s] rulebook: 5 minutes incl. completion report
TIME_SAFETY_MARGIN = 15.0      # [s] keep this much spare at the end
START_POSITION = (0.2, 0.0, 0.0)   # docking station (rulebook table 2.2.2-3)

FRAME_ID = 'iss_body'          # frame for MOVE_TO_ABSOLUTE_TARGET
MOVE_TIMEOUT = 45.0            # [s] max time for one /ctl/command move
SERVICE_TIMEOUT = 10.0         # [s] wait for services / action servers
REPORT_TIMEOUT = 10.0          # [s] max wait for checkpoint_passed result

STABLE_POS_TOL = 0.03          # [m]   position error considered "arrived"
STABLE_LIN_VEL = 0.02          # [m/s] linear speed considered "stopped"
STABLE_ANG_VEL = 0.05          # [rad/s]
STABLE_HOLD = 0.5              # [s] must stay stable this long
STABLE_TIMEOUT = 6.0           # [s] give up waiting and report anyway
PRE_REPORT_HOLD = 0.0          # [s] extra hold before reporting (try 1.0 if passes fail)

AIM_AT_MARKER = True           # re-aim camera (body +X) at marker if needed
AIM_CORRECTION_DEG = 2.0       # only correct if given attitude is off by more
MAX_ATTEMPTS = 2               # tries per checkpoint (move + report)


# ============================================================================
# Pure math helpers (no ROS) - quaternions are (x, y, z, w)
# ============================================================================
def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def v_norm(a):
    return math.sqrt(v_dot(a, a))


def v_unit(a):
    n = v_norm(a)
    if n < 1e-12:
        return (1.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def dist(a, b):
    return v_norm(v_sub(a, b))


def q_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_rotate(q, v):
    """Rotate vector v (body -> world) by unit quaternion q."""
    x, y, z, w = q
    p = q_mul(q_mul(q, (v[0], v[1], v[2], 0.0)), (-x, -y, -z, w))
    return (p[0], p[1], p[2])


def q_shortest_arc(u, v):
    """Smallest rotation taking unit vector u onto unit vector v."""
    d = v_dot(u, v)
    if d < -0.999999:
        axis = v_cross((1.0, 0.0, 0.0), u)
        if v_norm(axis) < 1e-6:
            axis = v_cross((0.0, 1.0, 0.0), u)
        axis = v_unit(axis)
        return (axis[0], axis[1], axis[2], 0.0)
    c = v_cross(u, v)
    return q_normalize((c[0], c[1], c[2], 1.0 + d))


def angle_deg(u, v):
    d = max(-1.0, min(1.0, v_dot(v_unit(u), v_unit(v))))
    return math.degrees(math.acos(d))


def aim_attitude(q_target, position, marker_position):
    """
    The camera looks along body +X. If the given target attitude does not
    point the camera at the marker, apply the smallest extra rotation that
    does. A minimal ("swing") rotation keeps the roll about the line of
    sight as close as possible to the given attitude.
    Returns (quaternion, error_before_deg).
    """
    q_target = q_normalize(q_target)
    forward = q_rotate(q_target, (1.0, 0.0, 0.0))
    to_marker = v_unit(v_sub(marker_position, position))
    err = angle_deg(forward, to_marker)
    if err <= AIM_CORRECTION_DEG:
        return q_target, err
    swing = q_shortest_arc(v_unit(forward), to_marker)
    return q_normalize(q_mul(swing, q_target)), err


def plan_route(start, points, goal=None):
    """
    Exact shortest order (Held-Karp DP).
    start: (x,y,z); points: list of (x,y,z) to visit in any order;
    goal: (x,y,z) visited last, or None.
    Returns (order as list of indices into points, total length).
    12 checkpoints -> 11 free points -> ~250k steps, well under a second.
    """
    n = len(points)
    if n == 0:
        return [], (dist(start, goal) if goal is not None else 0.0)
    if n == 1:
        total = dist(start, points[0]) + (dist(points[0], goal) if goal is not None else 0.0)
        return [0], total

    inf = float('inf')
    size = 1 << n
    d = [[dist(points[i], points[j]) for j in range(n)] for i in range(n)]
    cost = [[inf] * n for _ in range(size)]
    parent = [[-1] * n for _ in range(size)]
    for i in range(n):
        cost[1 << i][i] = dist(start, points[i])

    for mask in range(1, size):
        row = cost[mask]
        for last in range(n):
            c = row[last]
            if c == inf or not (mask >> last) & 1:
                continue
            dl = d[last]
            for nxt in range(n):
                if (mask >> nxt) & 1:
                    continue
                nmask = mask | (1 << nxt)
                nc = c + dl[nxt]
                if nc < cost[nmask][nxt]:
                    cost[nmask][nxt] = nc
                    parent[nmask][nxt] = last

    full = size - 1
    best_last, best_cost = -1, inf
    for last in range(n):
        c = cost[full][last]
        if goal is not None:
            c += dist(points[last], goal)
        if c < best_cost:
            best_cost, best_last = c, last

    order = []
    mask, cur = full, best_last
    while cur != -1:
        order.append(cur)
        prev = parent[mask][cur]
        mask &= ~(1 << cur)
        cur = prev
    order.reverse()
    return order, best_cost


def parse_checkpoints(message):
    """
    Parse the JSON from /mission/get_checkpoints.
    Returns (list of checkpoint dicts, goalpoint_id or None).
    Each dict: id, position, quaternion, marker_position, raw.
    """
    data = json.loads(message)
    result = []
    for cp in data.get('checkpoints', []):
        att = cp.get('attitude') or {}
        marker = cp.get('marker_pose') or {}
        result.append({
            'id': int(cp['checkpoint_id']),
            'position': tuple(float(v) for v in cp['position']),
            'quaternion': tuple(float(v) for v in att.get('quaternion', (0.0, 0.0, 0.0, 1.0))),
            'marker_position': (tuple(float(v) for v in marker['position'])
                                if 'position' in marker else None),
            'raw': cp,
        })
    goal_id = data.get('goalpoint_id')
    return result, (int(goal_id) if goal_id is not None else None)


# ============================================================================
# ROS node
# ============================================================================
class UserTemplate(object):

    def __init__(self):
        rospy.init_node('krpc_user')   # same node name as the official template

        # Action clients
        self.__command = actionlib.SimpleActionClient('/ctl/command', CtlCommandAction)
        self.__checkpoint_client = actionlib.SimpleActionClient(
            '/event/checkpoint_passed', CheckpointPassedAction)

        # Service clients
        self.__mission_start = rospy.ServiceProxy('/event/mission_start', MissionStart)
        self.__mission_finished = rospy.ServiceProxy('/event/mission_finished', MissionFinished)
        self.__checkpoints = rospy.ServiceProxy('/mission/get_checkpoints', CheckPoints)

        # Self-position
        self.__nav_lock = threading.Lock()
        self.__nav = None
        self.__nav_time = None
        self.__subscriber_nav = rospy.Subscriber(
            '/sensor_fusion/navigation', Navigation, self.__callback_nav)

        # Mission state
        self.__t0 = None
        self.__finished_sent = False
        self.__move_active = False
        self.__report_active = False

        # Subscriber
        self.__subscriber_start = rospy.Subscriber(
            '/ib2_user/start', UserLogic, self.__callback_start)    # DO NOT REMOVE THIS LINE

        # Service server
        self.__stop_processing_server = rospy.Service(
            '/ib2_user/stop', StopProcessingUserNode, self.__stop_processing)  # DO NOT REMOVE THIS LINE

        self.__started = False
        rospy.set_param('/ib2_user/ready', True)  # DO NOT REMOVE THIS LINE
        rospy.loginfo('[%s] ready', APP_VERSION)

    # ------------------------------------------------------------------ callbacks
    def __callback_nav(self, msg):
        with self.__nav_lock:
            self.__nav = msg
            self.__nav_time = rospy.get_time()

    def __callback_start(self, msg):
        if self.__started:
            rospy.logwarn('start callback is already processed')
            return
        self.__started = True
        rospy.loginfo('start %s', msg)
        self.__run_user_flow(msg)

    def __stop_processing(self, request):
        try:
            if self.__move_active:
                self.__command.cancel_goal()
            if self.__report_active:
                self.__checkpoint_client.cancel_goal()
            rospy.loginfo('finish processing')
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)
        except Exception as exc:
            rospy.logerr('stop processing failed: %s', exc)
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.ERROR)

    # ------------------------------------------------------------------ time
    def __elapsed(self):
        return rospy.get_time() - self.__t0 if self.__t0 is not None else 0.0

    def __remaining(self):
        return MISSION_TIME_LIMIT - self.__elapsed()

    # ------------------------------------------------------------------ main flow
    def __run_user_flow(self, msg):
        try:
            self.__t0 = rospy.get_time()   # conservative: undock already started the clock
            if not self.__notify_mission_start():
                return

            checkpoints, goal_id = self.__get_checkpoints()
            if not checkpoints:
                rospy.logerr('no checkpoints received; finishing')
                self.__notify_mission_finished()
                return

            # Prepare targets (re-aim camera at marker if needed)
            for cp in checkpoints:
                q = cp['quaternion']
                if AIM_AT_MARKER and cp['marker_position'] is not None:
                    q, err = aim_attitude(q, cp['position'], cp['marker_position'])
                    rospy.loginfo('cp %d: camera-to-marker error before aim %.1f deg',
                                  cp['id'], err)
                cp['target_q'] = q

            goal_cp = None
            free = []
            for cp in checkpoints:
                if goal_id is not None and cp['id'] == goal_id:
                    goal_cp = cp
                else:
                    free.append(cp)
            if goal_id is not None and goal_cp is None:
                rospy.logwarn('goalpoint_id %s is not in the checkpoint list', goal_id)

            start = self.__current_position() or START_POSITION
            order, length = plan_route(start, [cp['position'] for cp in free],
                                       goal_cp['position'] if goal_cp else None)
            route = [free[i] for i in order]
            rospy.loginfo('plan: %s -> goal %s, length %.2f m',
                          [cp['id'] for cp in route],
                          goal_cp['id'] if goal_cp else None, length)

            # Visit checkpoints with a time guard
            durations = []
            for idx, cp in enumerate(route):
                if rospy.is_shutdown():
                    return
                avg = (sum(durations) / len(durations)) if durations else 25.0
                need = avg + (avg if goal_cp else 0.0) + TIME_SAFETY_MARGIN
                if self.__remaining() < need:
                    rospy.logwarn('time guard: %.1fs left, skipping %d remaining checkpoints',
                                  self.__remaining(), len(route) - idx)
                    break
                t = rospy.get_time()
                self.__visit(cp)
                durations.append(rospy.get_time() - t)
                rospy.loginfo('elapsed %.1fs, remaining %.1fs', self.__elapsed(), self.__remaining())

            # Goal checkpoint last
            if goal_cp is not None and not rospy.is_shutdown():
                if self.__remaining() > TIME_SAFETY_MARGIN:
                    self.__visit(goal_cp)
                else:
                    rospy.logwarn('no time left for goal; reporting completion now')

            if not rospy.is_shutdown():
                self.__notify_mission_finished()
        except Exception as exc:
            rospy.logerr('user flow failed: %s', exc)
            # Always try to send the completion report: it carries the time bonus
            try:
                if not rospy.is_shutdown():
                    self.__notify_mission_finished()
            except Exception as exc2:
                rospy.logerr('mission_finished after failure also failed: %s', exc2)

    def __visit(self, cp):
        """Move to one checkpoint and report it. Returns True if passed."""
        for attempt in range(1, MAX_ATTEMPTS + 1):
            if rospy.is_shutdown() or self.__remaining() < TIME_SAFETY_MARGIN:
                return False
            rospy.loginfo('cp %d attempt %d: target %s', cp['id'], attempt,
                          ['%.3f' % v for v in cp['position']])
            moved = self.__move_to(cp['position'], cp['target_q'])
            if not moved:
                rospy.logwarn('cp %d: move failed', cp['id'])
                continue
            self.__wait_stable(cp['position'])
            if PRE_REPORT_HOLD > 0.0:
                rospy.sleep(PRE_REPORT_HOLD)
            ok, message = self.__notify_checkpoint_passed(cp['id'])
            rospy.loginfo('cp %d: passed=%s message=%s', cp['id'], ok, message)
            if ok:
                return True
        return False

    # ------------------------------------------------------------------ helpers
    def __current_position(self):
        with self.__nav_lock:
            nav = self.__nav
        if nav is None:
            return None
        p = nav.pose.pose.position
        return (p.x, p.y, p.z)

    def __wait_stable(self, target):
        """Wait until close to target and nearly still (bounded by STABLE_TIMEOUT)."""
        t_start = rospy.get_time()
        stable_since = None
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.get_time() - t_start < STABLE_TIMEOUT:
            with self.__nav_lock:
                nav, nav_time = self.__nav, self.__nav_time
            now = rospy.get_time()
            if nav is not None and nav_time is not None and now - nav_time < 0.5:
                p = nav.pose.pose.position
                lin = nav.twist.linear
                ang = nav.twist.angular
                err = dist((p.x, p.y, p.z), target)
                v = v_norm((lin.x, lin.y, lin.z))
                w = v_norm((ang.x, ang.y, ang.z))
                if err < STABLE_POS_TOL and v < STABLE_LIN_VEL and w < STABLE_ANG_VEL:
                    if stable_since is None:
                        stable_since = now
                    if now - stable_since >= STABLE_HOLD:
                        rospy.loginfo('stable: err %.3fm v %.3fm/s w %.3frad/s', err, v, w)
                        return True
                else:
                    stable_since = None
            rate.sleep()
        rospy.logwarn('not fully stable after %.1fs; reporting anyway', STABLE_TIMEOUT)
        return False

    def __get_checkpoints(self):
        for attempt in range(3):
            try:
                rospy.wait_for_service('/mission/get_checkpoints', SERVICE_TIMEOUT)
                res = self.__checkpoints()
                rospy.loginfo('get_checkpoints: success=%s message=%s', res.success, res.message)
                if res.success:
                    return parse_checkpoints(res.message)
            except Exception as exc:
                rospy.logwarn('get_checkpoints attempt %d failed: %s', attempt + 1, exc)
            rospy.sleep(0.5)
        return [], None

    def __move_to(self, position, quaternion):
        if not self.__command.wait_for_server(rospy.Duration(SERVICE_TIMEOUT)):
            rospy.logerr('/ctl/command server not available')
            return False
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = FRAME_ID
        target.pose.position.x, target.pose.position.y, target.pose.position.z = position
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = quaternion

        command_type = CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET)
        goal = CtlCommandGoal(target=target, type=command_type)
        t_start = rospy.get_time()
        self.__move_active = True
        try:
            self.__command.send_goal(goal)
            while not rospy.is_shutdown():
                if self.__command.wait_for_result(rospy.Duration(0.2)):
                    result = self.__command.get_result()
                    rospy.loginfo('move result type=%s (%.1fs)',
                                  result.type if result else None, rospy.get_time() - t_start)
                    return result is not None and result.type == CtlCommandResult.TERMINATE_SUCCESS
                if rospy.get_time() - t_start > MOVE_TIMEOUT or \
                        self.__remaining() < TIME_SAFETY_MARGIN / 2.0:
                    rospy.logwarn('move timeout; cancelling')
                    self.__command.cancel_goal()
                    self.__command.wait_for_result(rospy.Duration(5.0))
                    return False
            self.__command.cancel_goal()
            rospy.logwarn('move command cancelled by node shutdown')
            return False
        finally:
            self.__move_active = False

    def __notify_mission_start(self):
        try:
            rospy.wait_for_service('/event/mission_start', SERVICE_TIMEOUT)
            res = self.__mission_start()
            rospy.loginfo('mission_start: success=%s message=%s', res.success, res.message)
            return res.success
        except Exception as exc:
            rospy.logerr('mission_start failed: %s', exc)
            return False

    def __notify_checkpoint_passed(self, checkpoint_id):
        if not self.__checkpoint_client.wait_for_server(rospy.Duration(SERVICE_TIMEOUT)):
            return False, 'checkpoint_passed server not available'
        goal = CheckpointPassedGoal()
        goal.checkpoint_id = checkpoint_id
        t_start = rospy.get_time()
        self.__report_active = True
        try:
            self.__checkpoint_client.send_goal(goal)
            while not rospy.is_shutdown():
                if self.__checkpoint_client.wait_for_result(rospy.Duration(0.2)):
                    result = self.__checkpoint_client.get_result()
                    if result is None:
                        return False, 'no result'
                    return result.success, result.message
                if rospy.get_time() - t_start > REPORT_TIMEOUT:
                    self.__checkpoint_client.cancel_goal()
                    return False, 'report timeout'
            self.__checkpoint_client.cancel_goal()
            return False, 'cancelled by node shutdown'
        finally:
            self.__report_active = False

    def __notify_mission_finished(self):
        if self.__finished_sent:
            return True
        self.__finished_sent = True
        rospy.wait_for_service('/event/mission_finished', SERVICE_TIMEOUT)
        res = self.__mission_finished()
        rospy.loginfo('mission_finished: success=%s message=%s (elapsed %.1fs)',
                      res.success, res.message, self.__elapsed())
        return res.success


def main():
    UserTemplate()
    rospy.spin()


if __name__ == '__main__':
    main()
