# -*- coding: utf-8 -*-
"""Dry run of user_template.py without ROS or Gazebo.

Fake rospy / actionlib / message modules plus a simple robot that moves with
the timings measured in experiment E1 (translate, pause, turn, 3 s success
dwell). The fake judge uses scorer.py, which matches judge.py exactly.

Usage: python3 -I tests/dry_run.py <repo> [level] [seed] [--quiet]
"""
import json
import math
import os
import random
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, "..", "scripts")
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, HERE)
import geometry as g  # noqa: E402
import planner  # noqa: E402
import scorer  # noqa: E402

QUIET = "--quiet" in sys.argv


# ---------------------------------------------------------------------------
# Generic message stub
# ---------------------------------------------------------------------------
class Msg(object):
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)

    def __repr__(self):
        return "Msg(%s)" % ", ".join("%s=%r" % kv for kv in sorted(self.__dict__.items()))


def pose_stamped():
    m = Msg(header=Msg(frame_id="", stamp=None), pose=Msg(position=Msg(x=0.0, y=0.0, z=0.0),
                                                           orientation=Msg(x=0.0, y=0.0, z=0.0, w=1.0)))
    return m


# ---------------------------------------------------------------------------
# The fake world: clock, robot, controller, judge
# ---------------------------------------------------------------------------
class World(object):
    DT = 0.01

    def __init__(self, payload, seed):
        self.rng = random.Random(seed)
        self.t = 0.0
        self.pos = [0.2, 0.0, 0.0]
        self.q = list(g.IDENTITY)
        self.vel = [0.0, 0.0, 0.0]
        self.wvel = [0.0, 0.0, 0.0]
        self.goal = None          # dict describing the running move
        self.result = None        # finished move result type
        self.subs = {}
        self.services = {}
        self.next_nav = 0.0
        self.payload = payload
        self.specs = dict((c["checkpoint_id"], scorer.CheckpointSpec(c)) for c in payload["checkpoints"])
        self.reports = {}
        self.mission_start = None
        self.mission_finish = None
        self.shutdown = False
        self.max_t = 600.0

    # controller ---------------------------------------------------------
    def start_move(self, p1, q1):
        d = g.distance(self.pos, p1)
        ang = math.radians(g.q_angle_deg(self.q, q1))
        tt = planner.MotionModel.profile_time(d, 0.20, 0.0142)
        tr = planner.MotionModel.profile_time(ang, 0.436, 0.15)
        self.goal = dict(p0=list(self.pos), q0=list(self.q), p1=list(p1), q1=g.q_normalize(q1),
                         t0=self.t, tt=tt, gap=1.7 if ang > 0.01 else 0.0, tr=tr, inside_since=None)
        self.result = None

    def cancel_move(self):
        if self.goal is not None:
            self.goal = None
            if self.result is None:
                self.result = 1  # TERMINATE_ABORTED

    def step(self, dt):
        end = self.t + dt
        while self.t < end - 1e-12:
            self._tick()
            if self.t >= self.max_t:
                self.shutdown = True
                return

    def _tick(self):
        old_p, old_q = list(self.pos), list(self.q)
        gl = self.goal
        if gl is not None:
            s = self.t - gl["t0"]
            if s < gl["tt"]:
                f = s / gl["tt"]
                f = f * f * (3 - 2 * f)
                self.pos = g.v_add(gl["p0"], g.v_scale(g.v_sub(gl["p1"], gl["p0"]), f))
            else:
                self.pos = list(gl["p1"])
                r = s - gl["tt"] - gl["gap"]
                if r <= 0:
                    self.q = list(gl["q0"])
                elif r < gl["tr"]:
                    f = r / gl["tr"]
                    f = f * f * (3 - 2 * f)
                    self.q = slerp(gl["q0"], gl["q1"], f)
                else:
                    self.q = list(gl["q1"])
            # hover wobble
            self.pos = g.v_add(self.pos, [self.rng.gauss(0, 0.0004) for _ in range(3)])
            close = g.distance(self.pos, gl["p1"]) < 0.10 and math.radians(g.q_angle_deg(self.q, gl["q1"])) < 0.5
            if close:
                gl["inside_since"] = gl["inside_since"] if gl["inside_since"] is not None else self.t
                if self.t - gl["inside_since"] >= 3.0 and self.result is None:
                    self.result = 0  # TERMINATE_SUCCESS; the robot keeps holding (KEEP_POSE)
            else:
                gl["inside_since"] = None
        self.t += self.DT
        # Reported velocity: motion + the sim's nav bias (sim.yaml) + noise.
        self.vel = g.v_add(g.v_scale(g.v_sub(self.pos, old_p), 1.0 / self.DT),
                           [-0.0056 + self.rng.gauss(0, 0.0018), -0.0043 + self.rng.gauss(0, 0.0013), 0.0])
        dq = g.q_angle_deg(old_q, self.q)
        self.wvel = [math.radians(dq) / self.DT, 0.0, 0.0]
        if self.t >= self.next_nav:
            self.next_nav = self.t + 0.05
            cb = self.subs.get("/sensor_fusion/navigation")
            if cb:
                o = self.q
                cb(Msg(pose=Msg(pose=Msg(position=Msg(x=self.pos[0], y=self.pos[1], z=self.pos[2]),
                                         orientation=Msg(x=o[0], y=o[1], z=o[2], w=o[3]))),
                       twist=Msg(linear=Msg(x=self.vel[0], y=self.vel[1], z=self.vel[2]),
                                 angular=Msg(x=self.wvel[0], y=self.wvel[1], z=self.wvel[2]))))

    # judge -------------------------------------------------------------
    def judge_report(self, cp_id):
        if self.mission_start is None or self.mission_finish is not None:
            return
        if cp_id in self.reports:
            return
        spec = self.specs.get(cp_id)
        if spec is None:
            return
        ev = scorer.evaluate(self.pos, self.q, spec)
        self.reports[cp_id] = (self.t - self.mission_start, ev["level"])


def slerp(a, b, f):
    a, b = g.q_normalize(a), g.q_normalize(b)
    d = sum(x * y for x, y in zip(a, b))
    if d < 0:
        b = [-x for x in b]
        d = -d
    if d > 0.9995:
        return g.q_normalize([x + f * (y - x) for x, y in zip(a, b)])
    th = math.acos(d)
    s0 = math.sin((1 - f) * th) / math.sin(th)
    s1 = math.sin(f * th) / math.sin(th)
    return [s0 * x + s1 * y for x, y in zip(a, b)]


WORLD = None


# ---------------------------------------------------------------------------
# Fake modules
# ---------------------------------------------------------------------------
def install_fakes():
    rospy = types.ModuleType("rospy")

    class ROSException(Exception):
        pass

    class Duration(object):
        def __init__(self, s):
            self.s = float(s)

        def to_sec(self):
            return self.s

    class Time(object):
        @staticmethod
        def now():
            return WORLD.t

    class Rate(object):
        def __init__(self, hz):
            self.dt = 1.0 / hz

        def sleep(self):
            WORLD.step(self.dt)

    def log(level):
        def f(fmt, *args):
            if QUIET and level == "INFO":
                return
            print("[%7.2f %s] %s" % (WORLD.t, level, (fmt % args) if args else fmt))
        return f

    def service_proxy(name, _type):
        def call(*a, **k):
            return WORLD.services[name]()
        return call

    rospy.ROSException = ROSException
    rospy.Duration = Duration
    rospy.Time = Time
    rospy.Rate = Rate
    rospy.init_node = lambda *a, **k: None
    rospy.loginfo, rospy.logwarn, rospy.logerr = log("INFO"), log("WARN"), log("ERROR")
    rospy.get_time = lambda: WORLD.t
    rospy.is_shutdown = lambda: WORLD.shutdown
    rospy.wait_for_service = lambda name, timeout=None: None
    rospy.ServiceProxy = service_proxy
    rospy.Service = lambda name, typ, cb: WORLD.services.__setitem__("server:" + name, cb)
    rospy.Subscriber = lambda topic, typ, cb: WORLD.subs.__setitem__(topic, cb)
    rospy.set_param = lambda *a: None

    def spin():
        WORLD.step(1.0)
        WORLD.subs["/ib2_user/start"](Msg(id=0))
    rospy.spin = spin
    sys.modules["rospy"] = rospy

    actionlib = types.ModuleType("actionlib")

    class SimpleActionClient(object):
        def __init__(self, name, _type):
            self.name = name
            self.cp_result = None

        def wait_for_server(self, timeout=None):
            return True

        def send_goal(self, goal):
            if self.name == "/ctl/command":
                p = goal.target.pose.position
                o = goal.target.pose.orientation
                WORLD.start_move([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])
            else:
                WORLD.judge_report(goal.checkpoint_id)
                self.cp_result = Msg(success=True, message="accepted")

        def _done(self):
            return WORLD.result is not None if self.name == "/ctl/command" else True

        def wait_for_result(self, timeout=None):
            if self._done():
                return True
            end = WORLD.t + (timeout.to_sec() if timeout is not None else 1e9)
            while WORLD.t < end and not WORLD.shutdown:
                WORLD.step(World.DT)
                if self._done():
                    return True
            return self._done()

        def get_result(self):
            if self.name == "/ctl/command":
                return Msg(type=WORLD.result)
            return self.cp_result

        def cancel_goal(self):
            if self.name == "/ctl/command":
                WORLD.cancel_move()

    actionlib.SimpleActionClient = SimpleActionClient
    sys.modules["actionlib"] = actionlib

    def pkg(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    class StopResp(Msg):
        SUCCESS = 0
        ERROR = 255

    class CtlStatusType(Msg):
        MOVE_TO_ABSOLUTE_TARGET = 40
        MOVE_TO_RELATIVE_TARGET = 30
        STOP_MOVING = 20

    class CtlCommandResult(Msg):
        TERMINATE_SUCCESS = 0
        TERMINATE_ABORTED = 1
        TERMINATE_TIME_OUT = 2
        TERMINATE_INVALID_NAV = 3
        TERMINATE_INVALID_CMD = 4

    for n in ("platform_msgs", "geometry_msgs", "krpc_ib2_msgs", "ib2_msgs"):
        pkg(n)
    pkg("platform_msgs.msg", UserLogic=Msg)
    pkg("platform_msgs.srv", StopProcessingUserNode=Msg, StopProcessingUserNodeResponse=StopResp)
    pkg("geometry_msgs.msg", PoseStamped=pose_stamped)
    pkg("krpc_ib2_msgs.srv", MissionStart=Msg, MissionFinished=Msg, CheckPoints=Msg)
    pkg("krpc_ib2_msgs.msg", CheckpointPassedAction=Msg, CheckpointPassedGoal=Msg)
    pkg("ib2_msgs.msg", CtlCommandAction=Msg, CtlCommandGoal=Msg, CtlCommandResult=CtlCommandResult,
        CtlStatusType=CtlStatusType, Navigation=Msg)


def install_services():
    def start():
        WORLD.mission_start = WORLD.t
        return Msg(success=True, message="accepted")

    def finish():
        WORLD.mission_finish = WORLD.t
        gid = WORLD.payload["goalpoint_id"]
        if gid not in WORLD.reports:
            ev = scorer.evaluate(WORLD.pos, WORLD.q, WORLD.specs[gid])
            if ev["level"] > 0:
                WORLD.reports[gid] = (WORLD.t - WORLD.mission_start, ev["level"])
        return Msg(success=True, message="accepted")

    WORLD.services["/event/mission_start"] = start
    WORLD.services["/event/mission_finished"] = finish
    WORLD.services["/mission/get_checkpoints"] = lambda: Msg(success=True, message=json.dumps(WORLD.payload))


def run(repo, level, seed):
    global WORLD
    sys.argv = [sys.argv[0], repo]
    import test_offline as T
    _, payload = T.make_course(level, seed)
    WORLD = World(payload, seed)
    install_fakes()
    install_services()
    for mod in ("user_template",):
        sys.modules.pop(mod, None)
    import user_template
    node = user_template.UserTemplate()
    node.run()
    dur = (WORLD.mission_finish - WORLD.mission_start) if WORLD.mission_finish else None
    levels = [WORLD.reports.get(c["checkpoint_id"], (None, 0))[1] for c in payload["checkpoints"]]
    goal_ok = payload["goalpoint_id"] in WORLD.reports
    return dur, levels, goal_ok


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    repo = args[0] if args else os.path.expanduser("~/chumdararith-lou/kiborpc")
    level = args[1] if len(args) > 1 else "normal"
    seed = int(args[2]) if len(args) > 2 else 1
    dur, levels, goal_ok = run(repo, level, seed)
    print("RESULT level=%s seed=%d duration=%s levels=%s goal=%s" % (
        level, seed, "%.1f" % dur if dur else None, levels, goal_ok))
