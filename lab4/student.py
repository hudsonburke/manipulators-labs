"""Student-editable Lab 4 geometry and arm-to-UR5 mapping (angles in radians)."""

import math
from collections.abc import Mapping

import numpy as np
from numpy.typing import ArrayLike

import student_config as config

# One landmark in pixels: (x, y) as a tuple, list, or numpy row.
Point = ArrayLike


def arm_points(shoulder: Point, elbow: Point, wrist: Point) -> np.ndarray:
    """Validate and return the three (x, y) pixel coordinates of the arm.

    Raises ValueError for non-finite coordinates or a zero-length arm segment.
    """
    points = np.asarray([shoulder, elbow, wrist], dtype=float)
    if points.shape != (3, 2) or not np.isfinite(points).all():
        raise ValueError("Arm points must be three finite (x, y) pixel coordinates")
    if (np.linalg.norm(points[1] - points[0]) < 1e-6
            or np.linalg.norm(points[2] - points[1]) < 1e-6):
        raise ValueError("Shoulder, elbow and wrist must define two nonzero arm segments")
    return points


def elbow_angle(shoulder: Point, elbow: Point, wrist: Point) -> float:
    """Task 2: return the interior elbow angle in radians, within [0, pi].

    Implement this function yourself. The angle is measured at ``elbow``,
    between the two arm segments:

        shoulder --------- elbow --------- wrist
        a = shoulder - elbow        b = wrist - elbow

    Both vectors point *away* from the elbow, so the angle between them is the
    interior joint angle: flat/straight gives pi, a right angle gives pi/2.
    Work in pixels (the caller already scaled the landmarks), so the image
    aspect ratio is handled; the result does not depend on the pixel scale.

    Two standard ways to measure the angle between vectors a and b:

    1. Cosine rule:  arccos( (a . b) / (|a| * |b|) )
       Clip the cosine to [-1, 1] before arccos: floating-point rounding can
       push it slightly outside and arccos then returns NaN. Returns [0, pi].

    2. atan2 form:   atan2( |a_x*b_y - a_y*b_x|, a . b )
       Same answer without clipping, and stable for very small or very large
       vectors. The absolute value of the cross product is what keeps the
       result in [0, pi]; without it the sign flips with the bend direction.

    Self-checks with pixel coordinates (interior angle at the elbow):
        elbow_angle((0, 0), (0, 1), (1, 1))  ->  pi/2   (right angle)
        elbow_angle((0, 0), (1, 0), (2, 0))  ->  pi     (straight arm)
        elbow_angle((0, 0), (1, 0), (2, 1))  ->  3*pi/4 (forearm bent 45 degrees)
    """
    # Steps: validate with arm_points(), form a = shoulder - elbow and
    # b = wrist - elbow, then return the angle between them (formulas above).
    raise NotImplementedError("Implement elbow_angle() in student.py for Lab 4 Task 2.")


def shoulder_angle(shoulder: Point, elbow: Point, wrist: Point) -> float:
    """Signed upper-arm angle from image-down; positive toward image-right.

    Supplied for the optional shoulder extension (see student_config.py).
    """
    points = arm_points(shoulder, elbow, wrist)
    upper = points[1] - points[0]
    return math.atan2(float(upper[0]), float(upper[1]))


def joint_targets(angles: Mapping[str, float], neutral: Mapping[str, float]) -> np.ndarray:
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
