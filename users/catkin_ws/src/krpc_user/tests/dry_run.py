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
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, "..", "scripts")
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
import geometry as g  # noqa: E402
import planner  # noqa: E402
import scorer  # noqa: E402
from ib2sim import IB2Sim  # noqa: E402  (v2.0: physics for /ctl/wrench flight)

QUIET = "--quiet" in sys.argv
ARRIVAL_ERR = "--err" in sys.argv
GOAL_SLOW = 1.0                # --goal-slow=1.15: every leg to the goal takes longer (web W2/W3)
NO_FAN = "--no-fan" in sys.argv
TRACE_ARGS = list(sys.argv)
WEB_GRAVITY = "--gravity" in sys.argv
GRAVITY_START = next((float(x.split("=")[1]) for x in sys.argv if x.startswith("--gstart=")), None)   # pulse phase override   # ("--nog" = explicit no-op)   # play the web gravity module's pulses (probe 2026-11-24)
JUDGE_FAIL = set(int(x) for a in sys.argv if a.startswith("--judge-fail=") for x in a.split("=")[1].split(","))
                               # --judge-fail=5,8: the judge rejects the first report of these checkpoints
WEBLAG = "--weblag" in sys.argv   # web-like judge timing: hold starts 0.3 s after the report (first report
                                   # +2.7 s, W2/L1), the answer comes 0.2 s after the hold ends
KPR = "--kpr" in sys.argv      # copy the web's keep_pose_release: 8 ms after our first command
                               # after Mission Start, JAXA's controller is woken up (KEEP_POSE)
DIST_F = 0.002                 # --push=0.01: constant disturbance force [N] in fan (physics) mode
for _a in sys.argv:
    if _a.startswith("--goal-slow="):
        GOAL_SLOW = float(_a.split("=", 1)[1])
    if _a.startswith("--push="):
        DIST_F = float(_a.split("=", 1)[1])


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
        self.judge_connect_at = 0.0   # E6 race: the judge hears /gs/data only after this
        self.lost_start = False
        self.moves = 0                # number of /ctl/command goals so far
        self.abort_plan = {}          # move number (1-based) -> abort after this many s (web W1)
        self.slow_plan = {}           # move number -> time factor (web W2: goal legs 1.10-1.20x)
        self.arrival_err = False      # --err: the controller misses like on the web (~1 cm, <= 3.5 deg)
        self.hold_s = 1.0             # sim v1.2: the judge holds every report for 1 s
        self.goal_result = None       # (success, reason) from the v1.2 goal rule
        self.aborts = 0
        # v2.0: physics for /ctl/wrench flight, JAXA controller state, async judge
        self.phys = IB2Sim(seed=seed, dist_force=DIST_F, web_gravity=WEB_GRAVITY, gravity_start=GRAVITY_START)
        self.mode = "kin"             # "kin": JAXA controller moves/holds the robot; "wrench": our pushes
        self.ctl_type = 10            # KEEP_POSE until told otherwise
        self.kpr_at = None
        self.kpr_done = False
        self.ignored_wrench = 0
        self.next_status = 0.0
        self.eval = None              # pending checkpoint hold: dict
        self.hold_stats = []          # (max distance m, max angle deg) seen by the judge in each hold
        self.finish_eval = None       # pending goal hold at mission finish
        self.last_nav = ([0.2, 0.0, 0.0], list(g.IDENTITY))

    # controller ---------------------------------------------------------
    def ctl_goal(self, goal):
        kind = goal.type.type
        if self.kpr and not self.kpr_done and self.mission_start is not None and self.kpr_at is None:
            self.kpr_at = self.t + 0.008
        if kind == 0:                                   # STAND_BY: JAXA controller goes quiet
            self.goal = None
            self.result = 0
            self.ctl_type = 0
            if self.mode != "wrench":
                self.mode = "wrench"
                self.phys.command([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            return
        if kind < 20:                                   # KEEP_POSE: JAXA holds where it is
            self._to_kin()
            self.goal = None
            self.result = 0
            self.ctl_type = 10
            return
        self._to_kin()
        self.ctl_type = kind
        p = goal.target.pose.position
        o = goal.target.pose.orientation
        self.start_move([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])

    def _to_kin(self):
        if self.mode == "wrench":
            p, v, q, w = self.phys.truth()
            self.pos, self.q = p, q
            self.mode = "kin"

    def on_wrench(self, F, T):
        if self.mode == "wrench":
            self.phys.command(F, T)
        else:
            self.ignored_wrench += 1

    @property
    def kpr(self):
        return KPR

    def start_move(self, p1, q1):
        if self.arrival_err:
            e = [max(-0.02, min(0.02, self.rng.gauss(0, 0.008))) for _ in range(3)]
            p1 = g.v_add(p1, e)
            axis = [self.rng.gauss(0, 1) for _ in range(3)]
            q1 = g.q_mul(g.q_from_axis_angle(axis, math.radians(self.rng.uniform(0, 3.5))), q1)
        d = g.distance(self.pos, p1)
        ang = math.radians(g.q_angle_deg(self.q, q1))
        tt = planner.MotionModel.profile_time(d, 0.20, 0.010)
        tr = planner.MotionModel.profile_time(ang, 0.436, 0.18)
        self.goal = dict(p0=list(self.pos), q0=list(self.q), p1=list(p1), q1=g.q_normalize(q1),
                         t0=self.t, tt=tt, gap=1.7 if ang > 0.01 else 0.0, tr=tr, inside_since=None)
        self.result = None
        self.moves += 1
        self.goal["abort_after"] = self.abort_plan.get(self.moves)
        f = self.slow_plan.get(self.moves, 1.0)
        self.goal["tt"] *= f
        self.goal["tr"] *= f
        gpos = self.specs[self.payload["goalpoint_id"]].position
        if g.distance(p1, gpos) < 0.12:
            self.goal["tt"] *= GOAL_SLOW     # web goal legs: the long flight runs slow, not the turn

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
        if self.kpr_at is not None and not self.kpr_done and self.t >= self.kpr_at:
            self.kpr_done = True
            self._to_kin()
            self.ctl_type = 10
            if self.goal is not None:
                self.goal = None
                self.result = 1
        if self.t >= self.next_status:
            self.next_status = self.t + 0.1
            cb = self.subs.get("/ctl/status")
            if cb:
                cb(Msg(type=Msg(type=self.ctl_type)))
        self._judge_tick()
        if self.mode == "wrench":
            self.phys.step(self.DT)
            self.t = self.phys.t
            p, v, q, w = self.phys.truth()
            self.pos, self.q = p, q
            if self.t >= self.next_nav:
                self.next_nav = self.t + 0.05
                np_, nv, nq, nw = self.phys.nav()
                self.last_nav = (np_, nq)
                cb = self.subs.get("/sensor_fusion/navigation")
                if cb:
                    cb(Msg(pose=Msg(pose=Msg(position=Msg(x=np_[0], y=np_[1], z=np_[2]),
                                             orientation=Msg(x=nq[0], y=nq[1], z=nq[2], w=nq[3]))),
                           twist=Msg(linear=Msg(x=nv[0], y=nv[1], z=nv[2]),
                                     angular=Msg(x=nw[0], y=nw[1], z=nw[2]))))
            return
        self._kin_tick()
        # keep the physics model on the kinematic robot (so a later hand-over starts from here)
        self.phys.p, self.phys.q = list(self.pos), list(self.q)
        self.phys.v, self.phys.w = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        self.phys.t = self.t
        self.phys.hist.append((self.t, list(self.pos), [0.0, 0.0, 0.0], list(self.q), [0.0, 0.0, 0.0]))
        while self.phys.hist and self.phys.hist[0][0] < self.t - 0.3:
            self.phys.hist.popleft()
        self.last_nav = (list(self.pos), list(self.q))

    def _judge_tick(self):
        for ev in (self.eval, self.finish_eval):
            if ev is None or ev.get("done"):
                continue
            if self.t >= ev["nxt"]:
                ev["nxt"] = self.t + 1.0 / 30.0
                _e = scorer.evaluate(self.last_nav[0], self.last_nav[1], ev["spec"])
                ev["worst"] = min(ev["worst"], _e["level"])
                ev["md"] = max(ev.get("md", 0.0), _e["distance"])            # closest approach to the Lv3 limits
                ev["ma"] = max(ev.get("ma", 0.0), _e["centering_deg"] or 0.0, _e["roll_deg"] or 0.0)
            if self.t >= ev["end"]:
                ev["nxt"] = 1e18                       # hold over: stop sampling
                if self.t >= ev.get("fire", ev["end"]):
                    ev["done"] = True
                    self.hold_stats.append((ev.get("md", 0.0), ev.get("ma", 0.0)))
                    ev["finish"](ev["worst"])

    def _kin_tick(self):
        old_p, old_q = list(self.pos), list(self.q)
        gl = self.goal
        if gl is not None and gl.get("abort_after") is not None and self.t - gl["t0"] >= gl["abort_after"]:
            self.goal = None
            self.result = 1          # TERMINATE_ABORTED (web W1: collision detector)
            self.aborts += 1
            gl = None
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
    def hold_level(self, spec):
        """Sim v1.2: sample the pose for hold_s (about 30 Hz); the lowest level counts."""
        end = self.t + self.hold_s
        worst = 3
        while self.t < end and not self.shutdown:
            worst = min(worst, scorer.evaluate(self.pos, self.q, spec)["level"])
            self.step(1.0 / 30.0)
        return worst

    def judge_report(self, cp_id):
        """Returns (success, message) like sim v1.2's evaluate_checkpoint."""
        if self.mission_start is None:
            return False, "mission not started"
        if self.mission_finish is not None:
            return False, "mission already finished"
        if cp_id in self.reports:
            return False, "checkpoint already passed: %d" % cp_id
        spec = self.specs.get(cp_id)
        if spec is None:
            return False, "unknown checkpoint_id: %d" % cp_id
        return None, spec          # v2.0: evaluated over the next hold_s while the node keeps flying

    def start_eval(self, cp_id, spec, client):
        def finish(level):
            if cp_id in JUDGE_FAIL:
                JUDGE_FAIL.discard(cp_id)
                level = 0
            if level <= 0:
                client.done(False, "checkpoint left outermost margin")
            else:
                self.reports[cp_id] = (self.t - self.mission_start, level)
                client.done(True, "checkpoint accepted: band=%d" % level)
        lag0, lag1 = (0.3 + (2.7 if not self.reports and not getattr(self, "_lag_first_done", False) else 0.0), 0.2) if WEBLAG else (0.0, 0.0)
        self._lag_first_done = True
        self.eval = dict(spec=spec, end=self.t + lag0 + self.hold_s, nxt=self.t + lag0, worst=3, finish=finish,
                         fire=self.t + lag0 + self.hold_s + lag1)


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

    class Publisher(object):
        def __init__(self, topic, _type, queue_size=1):
            self.topic = topic

        def publish(self, msg):
            if self.topic == "/ctl/wrench":
                f, t = msg.wrench.force, msg.wrench.torque
                WORLD.on_wrench([f.x, f.y, f.z], [t.x, t.y, t.z])
    rospy.Publisher = Publisher
    class ROSInterruptException(Exception):
        pass
    rospy.ROSInterruptException = ROSInterruptException

    def ros_sleep(_s):
        # Background threads (startup probe) poll in real time; they never
        # advance the fake clock, which only the main flow steps.
        if WORLD.shutdown:
            raise ROSInterruptException()
        time.sleep(0.002)
    rospy.sleep = ros_sleep
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
                WORLD.ctl_goal(goal)
            else:
                ok, message = WORLD.judge_report(goal.checkpoint_id)
                if ok is None:
                    self.pending = True
                    WORLD.start_eval(goal.checkpoint_id, message, self)
                else:
                    self.pending = False
                    self.cp_result = Msg(success=ok, message=message)

        def done(self, ok, message):
            self.pending = False
            self.cp_result = Msg(success=ok, message=message)

        def _done(self):
            return WORLD.result is not None if self.name == "/ctl/command" else not getattr(self, "pending", False)

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

    rosgraph = types.ModuleType("rosgraph")

    class Master(object):
        def __init__(self, caller):
            pass

        def getSystemState(self):
            return ([], [["/gs/data", ["/krpc_flight_recorder", "/kibo_rpc_judge"]]], [])

        def lookupNode(self, name):
            return "http://fake-api:1"
    rosgraph.Master = Master
    sys.modules["rosgraph"] = rosgraph

    import xmlrpc.client

    class FakeProxy(object):
        def __init__(self, uri):
            pass

        def getBusInfo(self, caller):
            bus = [[1, "/krpc_flight_recorder", "o", "TCPROS", "/gs/data", True]]
            if WORLD.t >= WORLD.judge_connect_at:
                bus.append([2, "/kibo_rpc_judge", "o", "TCPROS", "/gs/data", True])
            return 1, "bus info", bus
    xmlrpc.client.ServerProxy = FakeProxy
    rospy.get_name = lambda: "/krpc_user"
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
        STAND_BY = 0
        KEEP_POSE = 10
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
    def wrench_stamped():
        return Msg(header=Msg(frame_id="", stamp=None),
                   wrench=Msg(force=Msg(x=0.0, y=0.0, z=0.0), torque=Msg(x=0.0, y=0.0, z=0.0)))
    pkg("geometry_msgs.msg", PoseStamped=pose_stamped, WrenchStamped=wrench_stamped)
    pkg("krpc_ib2_msgs.srv", MissionStart=Msg, MissionFinished=Msg, CheckPoints=Msg)
    pkg("krpc_ib2_msgs.msg", CheckpointPassedAction=Msg, CheckpointPassedGoal=Msg)
    pkg("ib2_msgs.msg", CtlCommandAction=Msg, CtlCommandGoal=Msg, CtlCommandResult=CtlCommandResult,
        CtlStatusType=CtlStatusType, Navigation=Msg, CtlStatus=Msg)


def install_services():
    def start():
        if WORLD.t < WORLD.judge_connect_at:
            WORLD.lost_start = True      # published before the judge connected: lost
        elif WORLD.mission_start is None:
            WORLD.mission_start = WORLD.t
        return Msg(success=True, message="accepted")

    def finish():
        if WORLD.mission_start is None:
            return Msg(success=True, message="accepted")   # judge ignores it
        WORLD.mission_finish = WORLD.t
        gid = WORLD.payload["goalpoint_id"]

        def finish_goal(level):                          # sim v1.2: 1 s hold at finish
            missing = [c["checkpoint_id"] for c in WORLD.payload["checkpoints"]
                       if c["checkpoint_id"] not in WORLD.reports]
            if level <= 0:
                WORLD.goal_result = (False, "checkpoint left outermost margin")
            elif missing:
                WORLD.goal_result = (False, "checkpoints not passed: %s" % ",".join(str(m) for m in missing))
            else:
                WORLD.goal_result = (True, "checkpoint validated")
        WORLD.finish_eval = dict(spec=WORLD.specs[gid], end=WORLD.t + WORLD.hold_s, nxt=WORLD.t, worst=3,
                                 finish=finish_goal)
        return Msg(success=True, message="accepted")

    WORLD.services["/event/mission_start"] = start
    WORLD.services["/event/mission_finished"] = finish
    WORLD.services["/mission/get_checkpoints"] = lambda: Msg(success=True, message=json.dumps(WORLD.payload))


def run(repo, level, seed, judge_delay=0.0, abort_plan=None, slow_plan=None):
    global WORLD
    sys.argv = [sys.argv[0], repo]
    import test_offline as T
    _, payload = T.make_course(level, seed)
    WORLD = World(payload, seed)
    WORLD.judge_connect_at = 1.0 + judge_delay   # the start signal arrives at t = 1 s
    WORLD.abort_plan = dict(abort_plan or {})
    WORLD.slow_plan = dict(slow_plan or {})
    WORLD.arrival_err = ARRIVAL_ERR
    install_fakes()
    install_services()
    for mod in ("user_template", "judge_link"):
        sys.modules.pop(mod, None)
    import user_template
    if NO_FAN:
        user_template.FAN_MODE = False
    if "--trace" in TRACE_ARGS:
        import fanmode
        fanmode.TRACE = True
    node = user_template.UserTemplate()
    node.run()
    t_end = WORLD.t + 2.0
    while WORLD.finish_eval is not None and not WORLD.finish_eval.get("done") and WORLD.t < t_end:
        WORLD.step(World.DT)
    dur = (WORLD.mission_finish - WORLD.mission_start) if WORLD.mission_finish else None
    levels = [WORLD.reports.get(c["checkpoint_id"], (None, 0))[1] for c in payload["checkpoints"]]
    goal_ok = WORLD.goal_result if WORLD.goal_result else (False, "no mission_finished")
    if WORLD.lost_start and WORLD.mission_start is None:
        print("JUDGE MISSED MISSION START: run scores nothing")
    return dur, levels, goal_ok


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    repo = args[0] if args else os.path.expanduser("~/chumdararith-lou/kiborpc")
    level = args[1] if len(args) > 1 else "normal"
    seed = int(args[2]) if len(args) > 2 else 1
    delay = float(args[3]) if len(args) > 3 else 0.0
    # --abort=1:0,7:20  -> abort move 1 at once and move 7 after 20 s (web W1 pattern)
    # --slow=8:1.3      -> move 8 takes 1.3x longer (web W2 goal legs)
    plan, slow = {}, {}
    for a in sys.argv[1:]:
        for flag, target in (("--abort=", plan), ("--slow=", slow)):
            if a.startswith(flag):
                for item in a[len(flag):].split(","):
                    k, v = item.split(":")
                    target[int(k)] = float(v)
    dur, levels, goal_ok = run(repo, level, seed, delay, plan, slow)
    score = None
    if dur:
        score = (10 * sum(1 for x in levels if x == 3) + (30 if goal_ok[0] else 0)) * (1 + (300 - dur) / 300.0)
    print("RESULT level=%s seed=%d duration=%s levels=%s goal=%s aborts=%d score=%s" % (
        level, seed, "%.1f" % dur if dur else None, levels, goal_ok, WORLD.aborts,
        "%.1f" % score if score is not None else None))
    if WORLD.hold_stats:
        print("HOLDS worst distance %.3f m, worst angle %.1f deg over %d holds" % (
            max(x[0] for x in WORLD.hold_stats), max(x[1] for x in WORLD.hold_stats), len(WORLD.hold_stats)))
