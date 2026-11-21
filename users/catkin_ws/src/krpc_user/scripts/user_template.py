#!/usr/bin/python3
# -*- coding:utf-8 -*-
"""Kibo-RPC 7th, team program v1.

Design: course Module 13. One rule above all: never report a checkpoint until
our own copy of the judge (scorer.py) says Lv3 and the robot has held still
for HOLD_S seconds. Around it: a Held-Karp route (planner.py), a mission clock
with a "go home" rule, bounded waits everywhere, and a safe stop.

Helper modules geometry.py, scorer.py and planner.py live next to this file.
"""
import json
import os
import sys
import threading
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rospy  # noqa: E402
import actionlib  # noqa: E402
from platform_msgs.msg import UserLogic  # noqa: E402
from platform_msgs.srv import StopProcessingUserNode, StopProcessingUserNodeResponse  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from krpc_ib2_msgs.srv import MissionStart, MissionFinished, CheckPoints  # noqa: E402
from krpc_ib2_msgs.msg import CheckpointPassedAction, CheckpointPassedGoal  # noqa: E402
from ib2_msgs.msg import CtlCommandAction, CtlCommandGoal, CtlCommandResult, CtlStatusType  # noqa: E402
from ib2_msgs.msg import Navigation  # noqa: E402

import geometry as g  # noqa: E402
import judge_link  # noqa: E402
import planner  # noqa: E402
import startup_probe  # noqa: E402
import scorer  # noqa: E402

VERSION = "v1.4.1 2026-11-22"

# ---------------------------------------------------------------------------
# Tuning (seconds are ROS/sim time). Measured values: experiment log, E1 + E2 (v1 Normal run).
# ---------------------------------------------------------------------------
TIME_LIMIT_S = 300.0        # official mission limit
GO_HOME_RESERVE_S = 15.0    # margin kept when deciding to visit one more checkpoint
GO_HOME_FACTOR = 1.05       # E1-E5: 39 legs, real/estimate = 0.96 overall, worst leg 1.05
HARD_STOP_S = 285.0         # past this, stop everything and report mission finished
HOLD_S = 1.0                # Lv3 + still must hold this long before reporting
STILL_WINDOW_S = 0.5        # stillness is measured from pose change over this window,
STILL_SPEED = 0.03          # m/s    not from nav twist (sim adds ~7 mm/s velocity bias)
STILL_TURN_DEG = 8.0        # deg/s  (E2: 0.01 / 2.0 waited ~1.6 s per checkpoint for nothing)
DIST_MARGIN = 0.8           # report only if distance <= 0.8 x the Lv3 radius ...
ANGLE_MARGIN_DEG = 2.0      # ... and centering, roll <= Lv3 angle - 2 deg
MOVE_TIMEOUT_MIN_S = 25.0
MOVE_TIMEOUT_MAX_S = 100.0  # E2: the 5.3 m goal leg took 57 s; v1's 60 s cap was too tight
EXTRA_SETTLE_S = 6.0        # extra watching after the controller reports success
MAX_AIM_FIXES = 1
WAIT_SERVICE_S = 10.0
WAIT_ACTION_S = 10.0
CANCEL_WAIT_S = 10.0
NAV_STALE_S = 1.0
LOOP_HZ = 20.0
JUDGE_WAIT_S = 45.0         # E6: never announce Mission Start before the judge listens
NO_JUDGE_WAIT_S = 5.0       # if no judge is visible at all, do not stall the run

MODEL = planner.MotionModel(accel=0.010, v_max=0.20, ang_accel=0.18, w_max=0.4363,
                            settle=3.0)


class MissionAbort(Exception):
    """Raised to leave the checkpoint loop (stop request, hard stop, shutdown)."""


class UserTemplate:

    def __init__(self):
        rospy.init_node('krpc_user')
        # v1.4.1: logging only. Times when the judge connects to /gs/data.
        self.__probe = startup_probe.StartupProbe(rospy.get_name())

        # Action clients
        self.__command = actionlib.SimpleActionClient('/ctl/command', CtlCommandAction)
        self.__checkpoint_client = actionlib.SimpleActionClient('/event/checkpoint_passed', CheckpointPassedAction)

        # Service clients
        self.__mission_start = rospy.ServiceProxy('/event/mission_start', MissionStart)
        self.__mission_finished = rospy.ServiceProxy('/event/mission_finished', MissionFinished)
        self.__checkpoints = rospy.ServiceProxy('/mission/get_checkpoints', CheckPoints)

        # Navigation (our pose, the same topic the judge reads)
        self.__nav_lock = threading.Lock()
        self.__nav = None
        self.__nav_stamp = None
        self.__nav_hist = deque()
        self.__subscriber_nav = rospy.Subscriber('/sensor_fusion/navigation', Navigation, self.__callback_nav)

        # Subscriber
        self.__subscriber_start = rospy.Subscriber('/ib2_user/start', UserLogic, self.__callback_start)  # DO NOT REMOVE THIS LINE

        # Service server
        self.__stop_processing_server = rospy.Service('/ib2_user/stop', StopProcessingUserNode, self.__stop_processing) # DO NOT REMOVE THIS LINE

        self.__started = False
        self.__action_active = False
        self.__stop_requested = False
        self.__t0 = None
        self.__finished_sent = False
        rospy.set_param('/ib2_user/ready', True) # DO NOT REMOVE THIS LINE
        rospy.loginfo('krpc_user %s ready', VERSION)

    # ------------------------------------------------------------------
    # Small helpers
    # ------------------------------------------------------------------
    def __callback_nav(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        v = msg.twist.linear
        w = msg.twist.angular
        now = rospy.get_time()
        with self.__nav_lock:
            self.__nav = ([p.x, p.y, p.z], [o.x, o.y, o.z, o.w],
                          [v.x, v.y, v.z], [w.x, w.y, w.z])
            self.__nav_stamp = now
            self.__nav_hist.append((now, self.__nav[0], self.__nav[1]))
            while self.__nav_hist and now - self.__nav_hist[0][0] > 2.0:
                self.__nav_hist.popleft()

    def __pose(self):
        """Latest (position, quaternion, velocity, angular velocity), or None if stale."""
        with self.__nav_lock:
            nav, stamp = self.__nav, self.__nav_stamp
        if nav is None or stamp is None or rospy.get_time() - stamp > NAV_STALE_S:
            return None
        return nav

    def __is_still(self):
        """True if position and attitude barely changed over the last STILL_WINDOW_S."""
        with self.__nav_lock:
            hist = list(self.__nav_hist)
        if len(hist) < 2:
            return False
        t1, p1, q1 = hist[-1]
        old = [h for h in hist if t1 - h[0] >= STILL_WINDOW_S]
        if not old:
            return False
        t0, p0, q0 = old[-1]
        dt = t1 - t0
        return (g.distance(p0, p1) / dt < STILL_SPEED
                and g.q_angle_deg(q0, q1) / dt < STILL_TURN_DEG)

    def __elapsed(self):
        return rospy.get_time() - self.__t0 if self.__t0 is not None else 0.0

    def __check_abort(self):
        if rospy.is_shutdown():
            raise MissionAbort('ROS shutdown')
        if self.__stop_requested:
            raise MissionAbort('stop requested')
        if self.__elapsed() > HARD_STOP_S:
            raise MissionAbort('hard stop at %.1f s' % self.__elapsed())

    def __call_service(self, name, proxy, retries=3):
        for attempt in range(1, retries + 1):
            try:
                rospy.wait_for_service(name, timeout=WAIT_SERVICE_S)
                res = proxy()
                rospy.loginfo('%s -> success=%s message=%s', name, res.success,
                              res.message if len(res.message) < 200 else res.message[:200] + '...')
                return res
            except Exception as exc:  # rospy.ROSException, ServiceException
                rospy.logwarn('%s attempt %d/%d failed: %s', name, attempt, retries, exc)
                if rospy.is_shutdown():
                    break
        return None

    # ------------------------------------------------------------------
    # Moving
    # ------------------------------------------------------------------
    def __send_move(self, position, quaternion):
        target = PoseStamped()
        target.header.frame_id = 'iss_body'
        target.header.stamp = rospy.Time.now()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = position
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = quaternion
        goal = CtlCommandGoal(target=target, type=CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET))
        self.__command.send_goal(goal)
        self.__action_active = True

    def __action_done(self):
        """Poll the move action briefly. Returns its result type, or None while running."""
        if not self.__action_active:
            return CtlCommandResult.TERMINATE_SUCCESS
        if self.__command.wait_for_result(rospy.Duration(0.01)):
            self.__action_active = False
            res = self.__command.get_result()
            return res.type if res is not None else CtlCommandResult.TERMINATE_ABORTED
        return None

    def __cancel_move(self):
        """Cancel the running move and WAIT until it has really ended (lesson from E1)."""
        if not self.__action_active:
            return True
        self.__command.cancel_goal()
        ok = self.__command.wait_for_result(rospy.Duration(CANCEL_WAIT_S))
        self.__action_active = False
        if not ok:
            rospy.logwarn('cancel did not complete within %.0f s', CANCEL_WAIT_S)
        return ok

    def __wait_move_end(self, deadline):
        """After reporting, let the running move finish before the next goal."""
        rate = rospy.Rate(LOOP_HZ)
        while self.__action_active and rospy.get_time() < deadline:
            if rospy.is_shutdown() or self.__stop_requested:
                break
            if self.__action_done() is not None:
                return
            rate.sleep()
        self.__cancel_move()

    # ------------------------------------------------------------------
    # Fly to one checkpoint, self-check, and report
    # ------------------------------------------------------------------
    def __visit(self, spec, is_goal=False):
        """Returns the level reported (0 if not reported)."""
        tag = 'goal' if is_goal else 'checkpoint'
        nav = self.__pose()
        est = MODEL.move_time(nav[0], nav[1], spec.position, spec.target_q, include_settle=False) if nav else 40.0
        timeout = max(MOVE_TIMEOUT_MIN_S, min(MOVE_TIMEOUT_MAX_S, 1.5 * est + 15.0))
        rospy.loginfo('[%6.1f s] -> %s %d: est %.1f s, timeout %.1f s', self.__elapsed(), tag, spec.id, est, timeout)

        self.__send_move(spec.position, spec.target_q)
        t_start = rospy.get_time()
        move_deadline = t_start + timeout
        settle_deadline = None
        hold_since = None
        best = None
        aim_fixes = 0
        rate = rospy.Rate(LOOP_HZ)

        while True:
            self.__check_abort()
            now = rospy.get_time()
            nav = self.__pose()
            if nav is not None:
                ev = scorer.evaluate(nav[0], nav[1], spec)
                still = self.__is_still()
                if best is None or ev['level'] > best['level']:
                    best = ev
                if self.__safe_lv3(ev, spec) and still:
                    hold_since = hold_since if hold_since is not None else now
                    if now - hold_since >= HOLD_S:
                        self.__log_eval(spec, ev, 'Lv3 held')
                        self.__report(spec)
                        self.__wait_move_end(now + EXTRA_SETTLE_S)
                        return 3
                else:
                    hold_since = None
            else:
                hold_since = None

            result = self.__action_done()
            if result is not None and settle_deadline is None:
                rospy.loginfo('[%6.1f s] move result %d after %.1f s', self.__elapsed(), result, now - t_start)
                settle_deadline = now + EXTRA_SETTLE_S

            if settle_deadline is not None and now > settle_deadline:
                # Controller finished but our check never held Lv3. Try one aim fix.
                if nav is not None and aim_fixes < MAX_AIM_FIXES and ev['distance_level'] >= 1:
                    aim_fixes += 1
                    q_fix = g.aim_from_here(nav[0], spec.marker_position, spec.target_q)
                    self.__log_eval(spec, ev, 'aim fix %d' % aim_fixes)
                    self.__send_move(spec.position, q_fix)
                    settle_deadline = None
                    move_deadline = now + MOVE_TIMEOUT_MIN_S
                    continue
                break

            if now > move_deadline:
                rospy.logwarn('[%6.1f s] move to %s %d timed out', self.__elapsed(), tag, spec.id)
                self.__cancel_move()
                break
            rate.sleep()

        # Could not hold Lv3: report the best we have now if it scores at all.
        self.__cancel_move()
        nav = self.__pose()
        if nav is not None:
            ev = scorer.evaluate(nav[0], nav[1], spec)
            if ev['level'] >= 1:
                self.__log_eval(spec, ev, 'fallback')
                self.__report(spec)
                return ev['level']
            self.__log_eval(spec, ev, 'NOT reported')
        return 0

    @staticmethod
    def __safe_lv3(ev, spec):
        """Lv3 with a margin, so a small wobble at report time cannot drop a level."""
        if ev['level'] != 3:
            return False
        r3 = min(t for t, lvl in spec.distance_bands if lvl == 3) if any(
            lvl == 3 for _, lvl in spec.distance_bands) else 0.1
        if ev['distance'] > DIST_MARGIN * r3:
            return False
        for value, bands in ((ev['centering_deg'], spec.centering_bands), (ev['roll_deg'], spec.roll_bands)):
            if value is None or not bands:
                continue
            a3 = min([t for t, lvl in bands if lvl == 3] or [10.0])
            if value > a3 - ANGLE_MARGIN_DEG:
                return False
        return True

    def __log_eval(self, spec, ev, why):
        rospy.loginfo('[%6.1f s] cp %d %s: level %d | dist %.3f m | centering %s | roll %s',
                      self.__elapsed(), spec.id, why, ev['level'], ev['distance'],
                      '%.1f deg' % ev['centering_deg'] if ev['centering_deg'] is not None else '-',
                      '%.1f deg' % ev['roll_deg'] if ev['roll_deg'] is not None else '-')

    def __report(self, spec):
        try:
            if not self.__checkpoint_client.wait_for_server(rospy.Duration(WAIT_ACTION_S)):
                rospy.logerr('checkpoint_passed server not available')
                return False
            self.__checkpoint_client.send_goal(CheckpointPassedGoal(checkpoint_id=spec.id))
            if self.__checkpoint_client.wait_for_result(rospy.Duration(WAIT_ACTION_S)):
                rospy.loginfo('[%6.1f s] reported cp %d: %s', self.__elapsed(), spec.id,
                              self.__checkpoint_client.get_result())
                return True
            self.__checkpoint_client.cancel_goal()
            rospy.logwarn('checkpoint_passed for %d did not answer', spec.id)
        except Exception as exc:
            rospy.logerr('report failed: %s', exc)
        return False

    # ------------------------------------------------------------------
    # Mission flow
    # ------------------------------------------------------------------
    def __run_user_flow(self, msg):
        # The mission clock starts at the start signal, so any waiting below is
        # paid from our own 5-minute budget (the safe assumption).
        self.__t0 = rospy.get_time()
        self.__probe.mark('start signal')
        started = False
        try:
            # v1.4: get everything ready BEFORE announcing Mission Start (E6).
            if not self.__command.wait_for_server(rospy.Duration(WAIT_ACTION_S)):
                rospy.logerr('/ctl/command server not available')
            if not self.__checkpoint_client.wait_for_server(rospy.Duration(WAIT_ACTION_S)):
                rospy.logwarn('/event/checkpoint_passed server not available yet')
            self.__probe.mark('servers ready')
            course = self.__load_course()
            self.__probe.mark('course loaded')
            self.__wait_for_judge()
            self.__probe.mark('gate passed')
            plan = self.__plan(course) if course is not None else None
            self.__probe.mark('mission_start call')
            res = self.__call_service('/event/mission_start', self.__mission_start)
            self.__probe.mark('mission_start returned')
            started = True
            rospy.loginfo('[%6.1f s] %s', self.__elapsed(), self.__probe.summary())
            self.__probe.stop()
            if res is None:
                rospy.logerr('mission_start failed; continuing anyway')
            if plan is None:
                return
            self.__fly_course(*plan)
        except MissionAbort as exc:
            rospy.logwarn('[%6.1f s] mission aborted: %s', self.__elapsed(), exc)
        except Exception as exc:
            rospy.logerr('user flow failed: %s', exc)
        finally:
            self.__cancel_move()
            if not started:
                self.__call_service('/event/mission_start', self.__mission_start)
            self.__finish()

    def __wait_for_judge(self):
        """Wait (bounded) until the judge is connected to /gs/data."""
        t0 = rospy.get_time()
        detail = ''
        rate = rospy.Rate(5.0)
        while True:
            self.__check_abort()
            try:
                ready, judge_seen, detail = judge_link.judge_listening(rospy.get_name())
            except Exception as exc:  # introspection unavailable: do not block forever
                ready, judge_seen, detail = False, False, 'check failed: %s' % exc
            if judge_seen:
                self.__probe.mark('judge subscribed')
            if ready:
                self.__probe.mark('judge connected')
                rospy.loginfo('[%6.1f s] %s', self.__elapsed(), detail)
                return True
            limit = JUDGE_WAIT_S if judge_seen else NO_JUDGE_WAIT_S
            if rospy.get_time() - t0 >= limit:
                break
            rate.sleep()
        rospy.logwarn('[%6.1f s] judge link not confirmed (%s); starting anyway',
                      self.__elapsed(), detail)
        return False

    def __load_course(self):
        res = self.__call_service('/mission/get_checkpoints', self.__checkpoints)
        if res is None or not res.success:
            rospy.logerr('no checkpoints')
            return None
        data = json.loads(res.message)
        cps = data.get('checkpoints', data.get('Checkpoints', []))
        specs = [scorer.CheckpointSpec(cp) for cp in cps]
        goal_id = data.get('goalpoint_id')
        goal = [s for s in specs if s.id == goal_id]
        if not goal:
            rospy.logerr('goal %s not among checkpoints', goal_id)
            return None
        stops = [s for s in specs if s.id != goal_id]
        rospy.loginfo('course: %d checkpoints + goal %d', len(stops), goal_id)
        return stops, goal[0]

    def __plan(self, course):
        stops, goal = course
        nav = self.__wait_for_nav()
        start = (nav[0], nav[1])
        # v1.2: plan the best subset that fits the budget, using the same rule as "go home".
        order, est = planner.plan_route_budget(
            start, [(s.id, s.position, s.target_q) for s in stops],
            (goal.id, goal.position, goal.target_q), MODEL,
            TIME_LIMIT_S, GO_HOME_RESERVE_S, GO_HOME_FACTOR, t_start=self.__elapsed())
        skipped = [s.id for s in stops if s.id not in order]
        rospy.loginfo('[%6.1f s] route %s, estimated finish %.0f s, planned skips %s',
                      self.__elapsed(), order, est, skipped)
        return stops, goal, order

    def __fly_course(self, stops, goal, order):
        by_id = dict((s.id, s) for s in stops)
        levels = {}
        for cp_id in order[:-1]:
            self.__check_abort()
            spec = by_id[cp_id]
            nav = self.__wait_for_nav()
            need = (MODEL.move_time(nav[0], nav[1], spec.position, spec.target_q)
                    + MODEL.move_time(spec.position, spec.target_q, goal.position, goal.target_q))
            if self.__elapsed() + GO_HOME_FACTOR * need + GO_HOME_RESERVE_S > TIME_LIMIT_S:
                rospy.logwarn('[%6.1f s] go home: cp %d needs %.0f s more', self.__elapsed(), cp_id, need)
                break
            levels[cp_id] = self.__visit(spec)
        levels[goal.id] = self.__visit(goal, is_goal=True)
        rospy.loginfo('[%6.1f s] levels %s', self.__elapsed(), levels)

    def __wait_for_nav(self):
        rate = rospy.Rate(LOOP_HZ)
        t_end = rospy.get_time() + 5.0
        while rospy.get_time() < t_end:
            nav = self.__pose()
            if nav is not None:
                return nav
            self.__check_abort()
            rate.sleep()
        raise MissionAbort('no navigation data')

    def __finish(self):
        if self.__finished_sent or rospy.is_shutdown():
            return
        self.__finished_sent = True
        self.__call_service('/event/mission_finished', self.__mission_finished)
        rospy.loginfo('[%6.1f s] mission finished sent', self.__elapsed())

    def __callback_start(self, msg):
        if self.__started:
            rospy.logwarn('start callback is already processed')
            return
        self.__started = True

        rospy.loginfo('start %s', msg)
        self.__run_user_flow(msg)

    def __stop_processing(self, request):
        try:
            self.__stop_requested = True
            if self.__action_active:
                self.__command.cancel_goal()
                self.__command.wait_for_result(rospy.Duration(CANCEL_WAIT_S))
                self.__action_active = False
            rospy.loginfo('finish processing')
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)
        except Exception as exc:
            rospy.logerr('stop processing failed: %s', exc)
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.ERROR)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    UserTemplate().run()
