"""Shared Robotics Toolbox URDF UR5 kinematics for Lab 3."""

from __future__ import annotations

import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
from numpy.typing import ArrayLike, NDArray


def make_robot() -> rtb.Robot:
    """Return one URDF model for kinematics, collision geometry, and Swift."""
    robot = rtb.models.URDF.UR5()
    robot.control_mode = "p"
    return robot


def target_pose(position: ArrayLike, rotation: ArrayLike) -> sm.SE3:
    position_array = np.asarray(position, dtype=float)
    rotation_array = np.asarray(rotation, dtype=float)
    if position_array.shape != (3,) or rotation_array.shape != (3, 3):
        raise ValueError(
            "Target position and rotation must have shapes (3,) and (3, 3)."
        )
    return sm.SE3.Rt(rotation_array, position_array, check=True)


def rotation_error(first: ArrayLike, second: ArrayLike) -> float:
    relative = np.asarray(first, dtype=float).T @ np.asarray(second, dtype=float)
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cosine))


def solve_goal_configuration(
    robot: rtb.Robot,
    target: sm.SE3,
    seed: ArrayLike,
) -> tuple[NDArray[np.float64], float, float]:
    """Solve IK and return joint values plus FK position/orientation errors."""
    solution = robot.ikine_LM(
        target, q0=np.asarray(seed, dtype=float), joint_limits=True
    )
    if not solution.success:
        raise RuntimeError(
            f"IK failed: {solution.reason} (residual {solution.residual})."
        )
    achieved = robot.fkine(solution.q)
    position_error = float(np.linalg.norm(target.t - achieved.t))
    orientation_error = rotation_error(target.R, achieved.R)
    return solution.q.copy(), position_error, orientation_error
