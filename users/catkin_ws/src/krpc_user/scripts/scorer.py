# -*- coding: utf-8 -*-
"""Our copy of the simulator's judge (course Module 9).

Mirrors rootfs/krpc/judge_module/judge.py, evaluate_checkpoint_navigation():
  * distance  = |robot centre - checkpoint position|
  * centering = angle(body +x rotated by robot attitude, robot centre -> marker)
  * roll      = |wrap(twist(robot q, view) - twist(target q, view))|
  * level     = min of the three band levels; 0 anywhere means fail.

Works on one checkpoint dict exactly as returned by /mission/get_checkpoints.
Pure Python, no ROS.
"""
import geometry as g

DEFAULT_DISTANCE_BANDS = [(0.1, 3), (0.2, 2), (0.3, 1)]
DEFAULT_ANGLE_BANDS = [(10.0, 3), (20.0, 2), (30.0, 1)]


def _bands(objs, key):
    """[(threshold, level), ...] sorted by threshold, as judge.py builds them."""
    out = []
    if not isinstance(objs, list):
        return out
    for b in objs:
        if not isinstance(b, dict):
            continue
        thr = b.get(key, b.get("margin", b.get("threshold" if key == "angle" else "distance")))
        lvl = b.get("level", b.get("score", b.get("point", b.get("points"))))
        try:
            out.append((float(thr), int(lvl)))
        except (TypeError, ValueError):
            continue
    out.sort(key=lambda t: t[0])
    return out


def classify(value, bands):
    """Same rule as judge.py: first band whose threshold >= value."""
    for thr, lvl in bands:
        if value <= thr:
            return lvl
    return 0


class CheckpointSpec(object):
    """Everything the judge needs about one checkpoint."""

    def __init__(self, cp):
        self.id = int(cp.get("checkpoint_id", cp.get("checkpointid", cp.get("id"))))
        self.position = [float(c) for c in cp["position"]]
        mp = cp.get("marker_pose", {})
        self.marker_position = [float(c) for c in mp.get("position", [0.0, 0.0, 0.0])]
        att = cp.get("attitude") or {}
        q = att.get("quaternion")
        self.target_q = g.q_normalize([float(c) for c in q]) if q else None
        self.distance_bands = _bands(cp.get("score_bands"), "radius") or list(DEFAULT_DISTANCE_BANDS)
        self.centering_bands = _bands(att.get("centering_bands"), "angle")
        self.roll_bands = _bands(att.get("roll_bands"), "angle")


def evaluate(position, q, spec, attitude_scored=True):
    """Predict the judge's verdict for the robot at (position, q).

    Returns a dict: distance, centering_deg, roll_deg, the three levels and
    `level` (0 = fail). `attitude_scored=False` mimics SIM_MODE=beginners.
    """
    d = g.distance(position, spec.position)
    d_lvl = classify(d, spec.distance_bands)
    out = {"distance": d, "distance_level": d_lvl,
           "centering_deg": None, "centering_level": None,
           "roll_deg": None, "roll_level": None}
    levels = [d_lvl]
    if attitude_scored and spec.target_q is not None and (spec.centering_bands or spec.roll_bands):
        view = g.v_unit(g.v_sub(spec.marker_position, position))
        cen = g.angle_between_deg(g.forward(q), view)
        roll = abs(g.wrap_deg(g.q_twist_deg(q, view) - g.q_twist_deg(spec.target_q, view)))
        out["centering_deg"], out["roll_deg"] = cen, roll
        if spec.centering_bands:
            out["centering_level"] = classify(cen, spec.centering_bands)
            levels.append(out["centering_level"])
        if spec.roll_bands:
            out["roll_level"] = classify(roll, spec.roll_bands)
            levels.append(out["roll_level"])
    out["level"] = 0 if any(l == 0 for l in levels) else min(levels)
    return out
