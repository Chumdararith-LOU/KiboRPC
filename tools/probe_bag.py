# -*- coding: utf-8 -*-
"""Pull the fan-control evidence out of a simulation.bag (Python 2 or 3, needs ROS rosbag).

Run inside the simulator image (see probe_bag.sh). Writes small text files to OUT_DIR:
  publishers.txt   every topic in the bag with its publishing nodes and message counts
  wrench.csv       every /ctl/wrench message: time, publisher, force xyz [N], torque xyz [N m]
  duty.csv         every /ctl/duty message: time, publisher, 8 fan duties (1.0 = full power)
  ctl_status.csv   /ctl/status type changes: time, type
  ctl_goals.csv    /ctl/command goals: time, publisher, type
  rosout.txt       log lines from nodes whose name mentions gravity, keep_pose, fsm, ctl, prop or thr
"""
from __future__ import print_function

import collections
import os
import sys

import rosbag

bag_path, out = sys.argv[1], sys.argv[2]
if not os.path.isdir(out):
    os.makedirs(out)
bag = rosbag.Bag(bag_path)

# 1. who publishes what
counts = collections.Counter()
for topic, msg, t, hdr in bag.read_messages(return_connection_header=True, raw=True):
    counts[(topic, hdr.get('callerid', '?'))] += 1
with open(os.path.join(out, 'publishers.txt'), 'w') as f:
    for (topic, caller), n in sorted(counts.items()):
        f.write('%-45s %-40s %8d\n' % (topic, caller, n))

WATCH = ('gravity', 'keep_pose', 'fsm', 'ctl', 'prop', 'thr')
fw = open(os.path.join(out, 'wrench.csv'), 'w')
fd = open(os.path.join(out, 'duty.csv'), 'w')
fs = open(os.path.join(out, 'ctl_status.csv'), 'w')
fg = open(os.path.join(out, 'ctl_goals.csv'), 'w')
fr = open(os.path.join(out, 'rosout.txt'), 'w')
fw.write('t,publisher,fx,fy,fz,tx,ty,tz\n')
fd.write('t,publisher,d1,d2,d3,d4,d5,d6,d7,d8\n')
fs.write('t,type\n')
fg.write('t,publisher,type\n')
last_status = None
topics = ['/ctl/wrench', '/ctl/duty', '/ctl/status', '/ctl/command/goal', '/rosout']
for topic, msg, t, hdr in bag.read_messages(topics=topics, return_connection_header=True):
    ts = t.to_sec()
    who = hdr.get('callerid', '?')
    if topic == '/ctl/wrench':
        f, q = msg.wrench.force, msg.wrench.torque
        fw.write('%.3f,%s,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f\n' % (ts, who, f.x, f.y, f.z, q.x, q.y, q.z))
    elif topic == '/ctl/duty':
        fd.write('%.3f,%s,%s\n' % (ts, who, ','.join('%.3f' % x for x in msg.data)))
    elif topic == '/ctl/status':
        if msg.type.type != last_status:
            last_status = msg.type.type
            fs.write('%.3f,%d\n' % (ts, last_status))
    elif topic == '/ctl/command/goal':
        fg.write('%.3f,%s,%d\n' % (ts, who, msg.goal.type.type))
    elif topic == '/rosout':
        if any(w in msg.name.lower() for w in WATCH):
            fr.write('[%.3f] %s: %s\n' % (msg.header.stamp.to_sec(), msg.name, msg.msg))
for f in (fw, fd, fs, fg, fr):
    f.close()
bag.close()
print('probe written to', out)
