"""Start-up timing probe (v1.4.1, logging only: changes no behaviour).

Runs in a background thread from the moment our node starts. It watches when
the judge subscribes to /gs/data and when krpc_api_node actually connects to
it, and records the main start-up steps of our own node. One summary line is
logged after Mission Start, with every time in ROS seconds (the same clock
as the [x s] stamps in our log) relative to the /ib2_user/start signal
(negative = before the signal). The wall/ROS ratio is logged as a check.

This lets the Web simulator's user program log answer: is the judge
connected before the start signal, and by how much?
"""
import threading
import time

import rospy

import judge_link

POLL_S = 0.2
GIVE_UP_S = 180.0


class StartupProbe(object):

    def __init__(self, caller_id):
        self.__caller_id = caller_id
        self.__lock = threading.Lock()
        self.__t_launch = (rospy.get_time(), time.time())
        self.__events = []          # (ros time, wall time, label), first only
        self.__labels = set()
        self.__first_detail = None
        self.__last_detail = None
        self.__polls = 0
        self.__errors = 0
        self.__done = False
        self.__thread = threading.Thread(target=self.__run, name='startup_probe')
        self.__thread.daemon = True
        self.__thread.start()

    def mark(self, label):
        """Record a step of our own flow (first occurrence only)."""
        with self.__lock:
            if label not in self.__labels:
                self.__labels.add(label)
                self.__events.append((rospy.get_time(), time.time(), label))

    def __run(self):
        while not self.__done and not rospy.is_shutdown():
            if time.time() - self.__t_launch[1] > GIVE_UP_S:
                break
            try:
                ready, seen, detail = judge_link.judge_listening(self.__caller_id)
            except Exception as exc:
                ready, seen, detail = False, False, 'check failed: %s' % exc
                self.__errors += 1
            self.__polls += 1
            with self.__lock:
                if self.__first_detail is None:
                    self.__first_detail = detail
                self.__last_detail = detail
            if seen:
                self.mark('judge subscribed')
            if ready:
                self.mark('judge connected')
                with self.__lock:
                    if 'start signal' in self.__labels:
                        break       # everything we need is recorded
            try:
                rospy.sleep(POLL_S)
            except rospy.ROSInterruptException:
                break

    def summary(self):
        """One log line: the start-up timeline relative to the start signal."""
        with self.__lock:
            events = list(self.__events)
            first_check, last_check = self.__first_detail, self.__last_detail
        ref = self.__t_launch
        for e in events:
            if e[2] == 'start signal':
                ref = e[:2]
        parts = ['node launch %+.1f' % (self.__t_launch[0] - ref[0])]
        for r, _w, label in sorted(events):
            parts.append('%s %+.1f' % (label, r - ref[0]))
        last = events[-1] if events else ref
        ros_span = last[0] - self.__t_launch[0]
        wall_span = last[1] - self.__t_launch[1]
        ratio = wall_span / ros_span if ros_span > 0.5 else float('nan')
        missing = [x for x in ('judge subscribed', 'judge connected')
                   if x not in self.__labels]
        text = 'startup timeline (s from start signal): ' + ', '.join(parts)
        text += ' | wall/ros %.2f' % ratio
        if missing:
            text += ' | never seen: %s' % ', '.join(missing)
        text += ' | polls %d, errors %d | first check: %s | last check: %s' % (
            self.__polls, self.__errors, first_check, last_check)
        return text

    def stop(self):
        self.__done = True
