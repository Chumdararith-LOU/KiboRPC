# -*- coding: utf-8 -*-
"""Check that the judge is listening before we announce "Mission Start".

Why (experiment E6, 2026-11-21): krpc_api_node relays our events to the judge
on /gs/data, a plain ROS topic that does not keep old messages. In one run the
judge connected 3.7 s after "Mission Start" was published, lost it, and then
ignored every checkpoint and the finish. The run scored nothing.

How: ask the ROS master which nodes subscribe to /gs/data, then ask
krpc_api_node (standard ROS getBusInfo call) which of them it is actually
connected to. This only reads ROS connection bookkeeping, never simulator truth.
"""
try:
    from xmlrpc.client import ServerProxy  # Python 3
except ImportError:  # pragma: no cover
    from xmlrpclib import ServerProxy  # Python 2

import rosgraph

TOPIC = '/gs/data'
API_NODE = '/krpc_api_node'


def judge_listening(caller_id):
    """Returns (ready, judge_seen, detail).

    ready is True when every /gs/data subscriber whose name contains "judge"
    has a live connection from krpc_api_node. judge_seen is True when such a
    subscriber is registered with the master (so waiting is worth it).
    Raises on ROS/XML-RPC errors.
    """
    master = rosgraph.Master(caller_id)
    _, subs, _ = master.getSystemState()
    listeners = []
    for topic, nodes in subs:
        if topic == TOPIC:
            listeners = list(nodes)
    judges = [n for n in listeners if 'judge' in n.lower()]
    if not judges:
        return False, False, 'no judge subscribed to %s yet (subscribers: %s)' % (TOPIC, listeners)
    code, _msg, bus = ServerProxy(master.lookupNode(API_NODE)).getBusInfo(caller_id)
    if code != 1:
        return False, True, 'getBusInfo failed'
    connected = set()
    for entry in bus:
        # rospy: [connection id, peer node, direction, transport, topic, connected, info]
        if len(entry) > 4 and entry[4] == TOPIC:
            connected.add(entry[1])
    missing = [n for n in judges if n not in connected]
    if missing:
        return False, True, 'judge not connected yet: %s' % missing
    return True, True, 'judge connected: %s' % judges
