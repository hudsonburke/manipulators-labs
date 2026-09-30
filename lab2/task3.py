"""Task 3: command the Task 2 IK solutions with joint-space motion (moveJ)."""

import os
from pathlib import Path

import numpy as np
import pandas as pd
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

from goals import GOALS, validate_goals

URSIM_HOST = os.environ.get("URSIM_HOST", "localhost")
JOINT_COLUMNS = ["base", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
HOME_Q = np.array(
    [-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0.0]
)
JOINT_SPEED = 0.25  # rad/s
JOINT_ACCELERATION = 0.5  # rad/s^2


def load_ik_solutions() -> np.ndarray:
    """Read the Task 2 output and verify that it matches this trajectory."""
    path = Path("task2_joint_q.csv")
    if not path.exists():
        raise FileNotFoundError("Run Task 2 first to create task2_joint_q.csv.")

    data = pd.read_csv(path)
    if list(data.columns) != JOINT_COLUMNS:
        raise ValueError(
            "task2_joint_q.csv has unexpected columns. Re-run the supplied Task 2 script."
        )
    if len(data) != len(GOALS):
        raise ValueError(
            "task2_joint_q.csv has a different number of rows than GOALS. "
            "Re-run Task 2 after finalizing goals.py."
        )
    return data.to_numpy(dtype=float)


def main() -> None:
    validate_goals(GOALS)
    ik_joint_qs = load_ik_solutions()
    controller = RTDEControl(URSIM_HOST)
    receiver = RTDEReceive(URSIM_HOST)

    actual_poses: list[list[float]] = []
    actual_joint_qs: list[list[float]] = []
    try:
        controller.moveJ(HOME_Q, JOINT_SPEED, JOINT_ACCELERATION)

        for index, (goal, joint_q) in enumerate(zip(GOALS, ik_joint_qs), start=1):
            controller.moveJ(joint_q, JOINT_SPEED, JOINT_ACCELERATION)
            actual_pose = receiver.getActualTCPPose()
            actual_joint_q = receiver.getActualQ()
            actual_poses.append(actual_pose)
            actual_joint_qs.append(actual_joint_q)
            print(f"Waypoint {index}")
            print("  Requested TCP pose: ", goal)
            print("  Commanded joint q:  ", joint_q)
            print("  Actual TCP pose:    ", actual_pose)
            print("  Actual joint q:     ", actual_joint_q)
    finally:
        controller.stopScript()

    results = pd.DataFrame(
        GOALS,
        columns=["target_x", "target_y", "target_z", "target_rx", "target_ry", "target_rz"],
    )
    results[["actual_x", "actual_y", "actual_z", "actual_rx", "actual_ry", "actual_rz"]] = actual_poses
    results[JOINT_COLUMNS] = actual_joint_qs
    results.to_csv("task3_results.csv", index=False)


if __name__ == "__main__":
    main()
