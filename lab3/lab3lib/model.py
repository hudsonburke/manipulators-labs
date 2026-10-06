"""Shared UR5 kinematics and model-consistency checks for Lab 3."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
from numpy.typing import ArrayLike, NDArray

from .world import MujocoWorld


@dataclass(frozen=True)
class ModelAgreement:
    maximum_position_error: float
    maximum_orientation_error: float


def make_robot() -> rtb.DHRobot:
    """Return the same standard-DH UR5 model encoded by the MuJoCo scene."""
    return rtb.models.DH.UR5()


def target_pose(position: ArrayLike, rotation: ArrayLike) -> sm.SE3:
    position_array = np.asarray(position, dtype=float)
    rotation_array = np.asarray(rotation, dtype=float)
    if position_array.shape != (3,) or rotation_array.shape != (3, 3):
        raise ValueError("Target position and rotation must have shapes (3,) and (3, 3).")
    return sm.SE3.Rt(rotation_array, position_array, check=True)


def rotation_error(first: ArrayLike, second: ArrayLike) -> float:
    relative = np.asarray(first, dtype=float).T @ np.asarray(second, dtype=float)
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cosine))


def solve_goal_configuration(
    robot: rtb.DHRobot,
    target: sm.SE3,
    seed: ArrayLike,
) -> tuple[NDArray[np.float64], float, float]:
    """Solve IK and return joint values plus FK position/orientation errors."""
    solution = robot.ikine_LM(target, q0=np.asarray(seed, dtype=float), joint_limits=True)
    if not solution.success:
        raise RuntimeError(
            f"IK failed: {solution.reason} (residual {solution.residual})."
        )
    achieved = robot.fkine(solution.q)
    position_error = float(np.linalg.norm(target.t - achieved.t))
    orientation_error = rotation_error(target.R, achieved.R)
    return solution.q.copy(), position_error, orientation_error


def check_model_agreement(
    robot: rtb.DHRobot,
    world: MujocoWorld,
    configurations: ArrayLike,
    *,
    position_tolerance: float = 1e-9,
    orientation_tolerance: float = 1e-7,
) -> ModelAgreement:
    """Verify that Robotics Toolbox and MuJoCo encode the same FK convention."""
    states = np.asarray(configurations, dtype=float)
    if states.ndim != 2 or states.shape[1] != 6:
        raise ValueError("Model-check configurations must have shape (samples, 6).")

    position_errors: list[float] = []
    orientation_errors: list[float] = []
    for q in states:
        toolbox_pose = robot.fkine(q).A
        mujoco_pose = world.tool_pose(q)
        position_errors.append(
            float(np.linalg.norm(toolbox_pose[:3, 3] - mujoco_pose[:3, 3]))
        )
        orientation_errors.append(
            rotation_error(toolbox_pose[:3, :3], mujoco_pose[:3, :3])
        )

    agreement = ModelAgreement(max(position_errors), max(orientation_errors))
    if agreement.maximum_position_error > position_tolerance:
        raise RuntimeError(
            "Robotics Toolbox and MuJoCo position conventions do not agree: "
            f"{agreement.maximum_position_error:.3e} m."
        )
    if agreement.maximum_orientation_error > orientation_tolerance:
        raise RuntimeError(
            "Robotics Toolbox and MuJoCo orientation conventions do not agree: "
            f"{agreement.maximum_orientation_error:.3e} rad."
        )
    return agreement
