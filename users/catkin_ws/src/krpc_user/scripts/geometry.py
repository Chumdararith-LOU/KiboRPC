# -*- coding: utf-8 -*-
"""Vector and quaternion helpers for the Kibo-RPC program.

Pure Python, no ROS, no numpy, so it runs anywhere (Python 3.6+).

Conventions (course Modules 4 and 5):
  * Vectors are lists [x, y, z] in metres.
  * Quaternions are lists [x, y, z, w], scalar LAST, the same order as ROS
    messages and the Kibo-RPC JSON.
  * q_mul(a, b) is the Hamilton product a*b: "do b, then a" in the world frame.
"""
import math

EPS = 1e-12


# ---------------------------------------------------------------------------
# Vectors
# ---------------------------------------------------------------------------
def v_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def v_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def v_scale(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


def v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def v_norm(a):
    return math.sqrt(v_dot(a, a))


def v_unit(a, fallback=None):
    n = v_norm(a)
    if n <= EPS:
        return list(fallback) if fallback is not None else [0.0, 0.0, 0.0]
    return [a[0] / n, a[1] / n, a[2] / n]


def distance(a, b):
    return v_norm(v_sub(a, b))


def angle_between_deg(a, b):
    """Angle between two vectors, in degrees (0..180)."""
    ua, ub = v_unit(a), v_unit(b)
    d = max(-1.0, min(1.0, v_dot(ua, ub)))
    return math.degrees(math.acos(d))


# ---------------------------------------------------------------------------
# Quaternions [x, y, z, w]
# ---------------------------------------------------------------------------
IDENTITY = [0.0, 0.0, 0.0, 1.0]


def q_normalize(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n <= EPS:
        return list(IDENTITY)
    return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]


def q_mul(a, b):
    """Hamilton product a*b."""
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return [w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2]


def q_conj(q):
    return [-q[0], -q[1], -q[2], q[3]]


def q_from_axis_angle(axis, angle_rad):
    u = v_unit(axis, fallback=[1.0, 0.0, 0.0])
    h = 0.5 * angle_rad
    s = math.sin(h)
    return [u[0] * s, u[1] * s, u[2] * s, math.cos(h)]


def q_rotate(q, v):
    """Rotate vector v by quaternion q (q v q*)."""
    q = q_normalize(q)
    r = q_mul(q_mul(q, [v[0], v[1], v[2], 0.0]), q_conj(q))
    return [r[0], r[1], r[2]]


def q_angle_deg(a, b):
    """Smallest rotation angle between two attitudes, in degrees (0..180)."""
    a, b = q_normalize(a), q_normalize(b)
    d = abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3])
    return math.degrees(2.0 * math.acos(min(1.0, d)))


def q_twist_deg(q, axis):
    """Signed twist of q about a unit axis, in degrees (Module 5.8).

    Same formula as the simulator's judge.
    """
    x, y, z, w = q_normalize(q)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    return math.degrees(2.0 * math.atan2(x * axis[0] + y * axis[1] + z * axis[2], w))


def wrap_deg(a):
    """Wrap an angle into [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def q_shortest_arc(u, v):
    """Smallest rotation taking direction u onto direction v."""
    u, v = v_unit(u), v_unit(v)
    d = v_dot(u, v)
    if d < -1.0 + 1e-9:
        # Opposite directions: turn 180 deg about any axis at right angles to u.
        perp = v_cross(u, [1.0, 0.0, 0.0])
        if v_norm(perp) < 1e-6:
            perp = v_cross(u, [0.0, 1.0, 0.0])
        return q_from_axis_angle(perp, math.pi)
    c = v_cross(u, v)
    return q_normalize([c[0], c[1], c[2], 1.0 + d])


BODY_X = [1.0, 0.0, 0.0]


def forward(q):
    """World direction the robot's front (body +x) points to."""
    return q_rotate(q, BODY_X)


def aim_from_here(position, marker_position, target_q):
    """Attitude that points body +x at the marker from `position` while
    keeping the target's twist about the line of sight (Module 9.7).

    With this attitude the judge's centering error is 0 and its roll error
    is 0, whatever small position error the robot has.
    """
    view = v_unit(v_sub(marker_position, position))
    swing = q_shortest_arc(forward(target_q), view)
    q = q_normalize(q_mul(swing, target_q))
    # Left-multiplying by a turn of alpha about `view` adds alpha to the twist.
    delta = wrap_deg(q_twist_deg(q, view) - q_twist_deg(target_q, view))
    fix = q_from_axis_angle(view, -math.radians(delta))
    return q_normalize(q_mul(fix, q))


def q_slerp(a, b, f):
    """Rotation a fraction f of the way from a to b along the shortest arc."""
    a, b = q_normalize(a), q_normalize(b)
    d = sum(x * y for x, y in zip(a, b))
    if d < 0.0:
        b = [-x for x in b]
        d = -d
    if d > 0.9995:
        return q_normalize([x + f * (y - x) for x, y in zip(a, b)])
    th = math.acos(min(1.0, d))
    s0 = math.sin((1.0 - f) * th) / math.sin(th)
    s1 = math.sin(f * th) / math.sin(th)
    return q_normalize([s0 * x + s1 * y for x, y in zip(a, b)])
