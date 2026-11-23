"""v2.0: fly with our own controller (fanctl.py) through /ctl/wrench.

How control is handed over (from JAXA's ctl.cpp and fsm.cpp):
  - A /ctl/command goal of type STAND_BY puts JAXA's controller on standby: it publishes one
    zero wrench and then stays silent, so only our /ctl/wrench reaches the fans (via fsm).
  - The fans keep the LAST wrench until a new one arrives, so we always end with a zero wrench
    and a KEEP_POSE goal (JAXA's controller then holds the robot where it is).
  - /ctl/status is watched the whole time; if anything (for example the web simulator's
    keep_pose_release node) wakes JAXA's controller up again, standby is sent again.
Anything unexpected (stale navigation, tracking error, a leg that never settles, standby that
will not stick) raises FanAbort; the caller then hands back to JAXA's controller for the rest.
"""
import math
import re
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from ib2_msgs.msg import CtlCommandGoal, CtlStatus, CtlStatusType

import geometry as g
import fanctl
import scorer

CTL_HZ = 50.0
STANDBY_STABLE_S = 0.5      # standby must hold this long before we start pushing
ENGAGE_TRIES = 4
MAX_REENGAGE = 8            # standby re-sent during flight more often than this -> give up
REENGAGE_GAP_S = 0.5
TRACK_POS_FAIL_M = 0.6      # further than this from the reference -> hand back
TRACK_ATT_FAIL_DEG = 60.0
LEG_EXTRA_S = 15.0          # a leg that has not settled this long after its plan -> hand back
REPORT_WAIT_S = 6.0
STILL_S = 0.3               # safe Lv3 and slow for this long -> report
SAFE_DIST_M = 0.05          # report margins (Lv3 is 0.10 m, 10 deg)
SAFE_ANG_DEG = 6.0
SLOW_MPS = 0.01
NAV_STALE_S = 0.5
FINISH_HOLD_S = 2.0         # keep holding after mission_finished (the judge's 1 s goal hold)
STOP_PER_CP_S = 1.6         # planner: settle check + the judge's 1 s hold
REPORT_RETRIES = 1          # judge said "failed": the judge allows another report


class FanAbort(Exception):
    """Our controller cannot continue safely; hand back to JAXA's controller."""


class FanModel(object):
    """Leg times for the planner when flying with our controller."""

    def move_time(self, p0, q0, p1, q1, include_settle=True):
        return fanctl.Leg(p0, q0, p1, q1).duration + (STOP_PER_CP_S if include_settle else 0.0)


class FanPilot(object):

    def __init__(self, command_client, checkpoint_client, get_nav):
        """get_nav() -> (position, quaternion, velocity, angular velocity, stamp) or None."""
        self.cmd = command_client
        self.cpc = checkpoint_client
        self.get_nav = get_nav
        self.pub = rospy.Publisher('/ctl/wrench', WrenchStamped, queue_size=1)
        self.status_sub = rospy.Subscriber('/ctl/status', CtlStatus, self._on_status)
        self.status = None
        self.status_t = None
        self.ctl = fanctl.FanController()
        self.active = False
        self.reengages = 0
        self.last_reengage = -1e9
        self.leg = None
        self.leg_t0 = None
        self.vel_samples = deque()

    # ------------------------------------------------------------------ helpers
    def _on_status(self, msg):
        self.status = msg.type.type
        self.status_t = rospy.get_time()

    def note_velocity(self, v):
        """Navigation velocity while docked and still, before the start (learns the nav bias)."""
        self.vel_samples.append(list(v))
        while len(self.vel_samples) > 200:
            self.vel_samples.popleft()

    def _nav(self):
        n = self.get_nav()
        if n is None or rospy.get_time() - n[4] > NAV_STALE_S:
            raise FanAbort('navigation stale')
        p, q, v, w = n[0], n[1], n[2], n[3]
        return p, v, q, w

    def _publish(self, F, T):
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'body'
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = F
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = T
        self.pub.publish(msg)

    def _ctl_goal(self, kind):
        n = self.get_nav()
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'iss_body'
        if n is not None:
            target.pose.position.x, target.pose.position.y, target.pose.position.z = n[0]
            (target.pose.orientation.x, target.pose.orientation.y,
             target.pose.orientation.z, target.pose.orientation.w) = n[1]
        else:
            target.pose.orientation.w = 1.0
        self.cmd.send_goal(CtlCommandGoal(target=target, type=CtlStatusType(type=kind)))

    # ------------------------------------------------------------------ handover
    def engage(self):
        """Put JAXA's controller on standby; True when it stays there."""
        if len(self.vel_samples) >= 10:
            for v in list(self.vel_samples)[-100:]:
                self.ctl.learn_bias(v)
        rospy.loginfo('fan mode: nav velocity bias %s mm/s', ['%.1f' % (x * 1000) for x in self.ctl.vel_bias])
        rate = rospy.Rate(CTL_HZ)
        for attempt in range(1, ENGAGE_TRIES + 1):
            self._ctl_goal(CtlStatusType.STAND_BY)
            since = None
            t_end = rospy.get_time() + 2.0 + STANDBY_STABLE_S
            while rospy.get_time() < t_end and not rospy.is_shutdown():
                if self.status == CtlStatusType.STAND_BY:
                    since = since if since is not None else rospy.get_time()
                    if rospy.get_time() - since >= STANDBY_STABLE_S:
                        self.active = True
                        rospy.loginfo('fan mode: JAXA controller on standby (attempt %d)', attempt)
                        return True
                else:
                    since = None
                rate.sleep()
            rospy.logwarn('fan mode: standby not confirmed (status %s), attempt %d', self.status, attempt)
        return False

    def _watch_status(self, now):
        if self.status is None or self.status == CtlStatusType.STAND_BY:
            return
        if now - self.last_reengage < REENGAGE_GAP_S:
            return
        self.reengages += 1
        self.last_reengage = now
        rospy.logwarn('fan mode: JAXA controller woke up (status %d); standby again (%d)', self.status, self.reengages)
        if self.reengages > MAX_REENGAGE:
            raise FanAbort('standby keeps being overridden')
        self._ctl_goal(CtlStatusType.STAND_BY)

    def release(self, keep_pose=True):
        """Stop pushing (zero wrench) and, by default, let JAXA's controller hold the pose."""
        for _ in range(3):
            self._publish([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        if self.active and keep_pose:
            try:
                self._ctl_goal(CtlStatusType.KEEP_POSE)
            except Exception as exc:
                rospy.logwarn('fan mode: keep-pose goal failed: %s', exc)
        self.active = False

    # ------------------------------------------------------------------ flying
    def _step(self, check_abort, now):
        check_abort()
        nav = self._nav()
        ref = self.leg.ref(now - self.leg_t0)
        F, T = self.ctl.wrench(now, nav, ref)
        self._publish(F, T)
        self._watch_status(now)
        dp = g.distance(nav[0], ref[0])
        da = g.q_angle_deg(nav[2], ref[3])
        if dp > TRACK_POS_FAIL_M or da > TRACK_ATT_FAIL_DEG:
            raise FanAbort('tracking error %.2f m / %.0f deg' % (dp, da))
        return nav

    def _safe(self, ev):
        return (ev['level'] == 3 and ev['distance'] <= SAFE_DIST_M
                and (ev['centering_deg'] or 0.0) <= SAFE_ANG_DEG and (ev['roll_deg'] or 0.0) <= SAFE_ANG_DEG)

    def visit(self, spec, check_abort, elapsed, tag='checkpoint'):
        """Fly to spec, settle, report and keep holding until the judge answers.
        Returns (level reported, judge message). Raises FanAbort to hand back."""
        p, v, q, w = self._nav()
        self.leg = fanctl.Leg(p, q, spec.position, spec.target_q)
        self.leg_t0 = rospy.get_time()
        rospy.loginfo('[%6.1f s] fan -> %s %d: %.2f m, turn %.0f deg, plan %.1f s',
                      elapsed(), tag, spec.id, self.leg.d, math.degrees(self.leg.ang), self.leg.duration)
        rate = rospy.Rate(CTL_HZ)
        still_since = None
        report_t = None
        reports = 0
        while True:
            now = rospy.get_time()
            nav = self._step(check_abort, now)
            t_leg = now - self.leg_t0
            if report_t is None:
                if t_leg >= self.leg.duration - 0.5:
                    ev = scorer.evaluate(nav[0], nav[2], spec)
                    vb = self.ctl.vel_bias
                    slow = math.sqrt(sum((nav[1][k] - vb[k]) ** 2 for k in range(3))) < SLOW_MPS
                    if self._safe(ev) and slow:
                        still_since = still_since if still_since is not None else now
                        if now - still_since >= STILL_S:
                            rospy.loginfo('[%6.1f s] cp %d Lv3 held (fan, leg %.1f s): dist %.3f m | centering %.1f deg | roll %.1f deg',
                                          elapsed(), spec.id, t_leg, ev['distance'],
                                          ev['centering_deg'] or 0.0, ev['roll_deg'] or 0.0)
                            self.cpc.send_goal(self._cp_goal(spec.id))
                            report_t = now
                            reports += 1
                    else:
                        still_since = None
                if t_leg > self.leg.duration + LEG_EXTRA_S:
                    raise FanAbort('%s %d did not settle in time' % (tag, spec.id))
            else:
                if self.cpc.wait_for_result(rospy.Duration(0.001)):
                    res = self.cpc.get_result()
                    ok = bool(res is not None and res.success)
                    msg = res.message if res is not None else ''
                    rospy.loginfo('[%6.1f s] reported cp %d: %s %s', elapsed(), spec.id, ok, msg)
                    if ok:
                        m = re.search(r'band=(\d)', msg)
                        return (int(m.group(1)) if m else 3), msg
                    if reports <= REPORT_RETRIES and 'already' not in msg and 'not started' not in msg:
                        report_t = None          # the judge allows a failed checkpoint to be reported again
                        still_since = None
                    else:
                        return 0, msg
                elif now - report_t > REPORT_WAIT_S:
                    rospy.logwarn('[%6.1f s] judge did not answer for cp %d', elapsed(), spec.id)
                    return 0, 'no answer'
            rate.sleep()

    def hold(self, seconds, check_abort):
        """Keep holding the last target (used around mission_finished)."""
        rate = rospy.Rate(CTL_HZ)
        t_end = rospy.get_time() + seconds
        while rospy.get_time() < t_end:
            self._step(check_abort, rospy.get_time())
            rate.sleep()

    @staticmethod
    def _cp_goal(cp_id):
        from krpc_ib2_msgs.msg import CheckpointPassedGoal
        return CheckpointPassedGoal(checkpoint_id=cp_id)
