"""Student-editable image geometry and arm-to-UR5 mapping (all angles in radians)."""

import math

import numpy as np

if __package__:
    from . import student_config as config
else:
    import student_config as config


def arm_angles(shoulder, elbow, wrist):
    """Return left elbow interior angle and upper arm angle from image-down.

    Points are (x, y) pixels, not normalized camera coordinates. Shoulder is
    positive toward image-right from downward vertical; elbow is in [0, pi].
    """
    points = np.asarray([shoulder, elbow, wrist], dtype=float)
    if points.shape != (3, 2) or not np.isfinite(points).all():
        raise ValueError("Arm points must be three finite (x, y) pixel coordinates")
    upper = points[1] - points[0]
    forearm = points[2] - points[1]
    if np.linalg.norm(upper) < 1e-6 or np.linalg.norm(forearm) < 1e-6:
        raise ValueError("Shoulder, elbow and wrist must define two nonzero arm segments")
    toward_shoulder = -upper
    cross = toward_shoulder[0] * forearm[1] - toward_shoulder[1] * forearm[0]
    return {
        "elbow": math.atan2(abs(float(cross)), float(toward_shoulder @ forearm)),
        "shoulder": math.atan2(float(upper[0]), float(upper[1])),
    }


def joint_targets(angles, neutral):
    """Map changes from the calibrated human pose onto bounded UR5 targets.

    Joint 2 is the elbow; joint 1 is the optional shoulder. Other joints stay
    at HOME_Q. Edit the gains and shoulder switch in student_config.py.
    """
    try:
        elbow_delta = float(angles["elbow"]) - float(neutral["elbow"])
        shoulder_delta = float(angles["shoulder"]) - float(neutral["shoulder"])
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError("Current and neutral poses need elbow and shoulder radians") from exc
    if not math.isfinite(elbow_delta) or not math.isfinite(shoulder_delta):
        raise ValueError("Current and neutral angles must be finite")
    shoulder_delta = math.atan2(math.sin(shoulder_delta), math.cos(shoulder_delta))
    q = config.HOME_Q.copy()
    q[2] = np.clip(q[2] + config.ELBOW_GAIN * elbow_delta,
                   config.JOINT_MIN[2], config.JOINT_MAX[2])
    if config.CONTROL_SHOULDER:
        q[1] = np.clip(q[1] + config.SHOULDER_GAIN * shoulder_delta,
                       config.JOINT_MIN[1], config.JOINT_MAX[1])
    return q
