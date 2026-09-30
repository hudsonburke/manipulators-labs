"""Cartesian TCP waypoint data shared by all Lab 2 tasks."""

import numpy as np
import spatialmath as sm

# Replace every placeholder row with a pose recorded in URSim's Base feature.
# Pose format: [x, y, z, rx, ry, rz]
#   x, y, z: position in meters
#   rx, ry, rz: UR rotation vector (axis * angle), in radians -- NOT roll/pitch/yaw
GOALS = np.array(
    [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Replace with waypoint 1
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Replace with waypoint 2
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Replace with waypoint 3
    ],
    dtype=float,
)


def validate_goals(goals: np.ndarray) -> None:
    """Fail early if the starter waypoints have not been replaced."""
    if goals.ndim != 2 or goals.shape[1:] != (6,):
        raise ValueError("GOALS must be an N-by-6 array of UR TCP poses.")
    if len(goals) < 3:
        raise ValueError("Use at least three waypoints for the trajectory.")
    if np.allclose(goals, 0.0):
        raise ValueError(
            "GOALS still contains the all-zero starter values. Record reachable "
            "Base-frame poses in URSim and enter them in goals.py."
        )


def pose_to_se3(pose: np.ndarray) -> sm.SE3:
    """Convert a UR [x, y, z, rx, ry, rz] pose to a Robotics Toolbox SE3 pose."""
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (6,):
        raise ValueError("A UR TCP pose must contain exactly six values.")

    rotation_vector = pose[3:]
    angle = np.linalg.norm(rotation_vector)
    rotation = sm.SO3() if np.isclose(angle, 0.0) else sm.SO3.AngVec(
        angle, rotation_vector / angle
    )
    return sm.SE3.Rt(rotation, pose[:3])
