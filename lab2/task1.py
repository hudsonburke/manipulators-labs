"""Task 1: command and record a Cartesian (moveL) trajectory in URSim."""

import os

import numpy as np
import pandas as pd
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

from goals import GOALS, validate_goals

URSIM_HOST = os.environ.get("URSIM_HOST", "localhost")
HOME_Q = np.array(
    [-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0.0]
)
CARTESIAN_SPEED = 0.25  # m/s for moveL
CARTESIAN_ACCELERATION = 0.5  # m/s^2 for moveL
JOINT_SPEED = 0.5  # rad/s for the initial moveJ
JOINT_ACCELERATION = 0.5  # rad/s^2 for the initial moveJ


def main() -> None:
    validate_goals(GOALS)
    controller = RTDEControl(URSIM_HOST)
    receiver = RTDEReceive(URSIM_HOST)

    actual_poses: list[list[float]] = []
    actual_joint_qs: list[list[float]] = []

    try:
        # Begin each trial at the same joint configuration.
        controller.moveJ(HOME_Q, JOINT_SPEED, JOINT_ACCELERATION)

        for index, goal in enumerate(GOALS, start=1):
            controller.moveL(goal, CARTESIAN_SPEED, CARTESIAN_ACCELERATION)
            actual_pose = receiver.getActualTCPPose()
            actual_joint_q = receiver.getActualQ()
            actual_poses.append(actual_pose)
            actual_joint_qs.append(actual_joint_q)
            print(f"Waypoint {index}")
            print("  Requested TCP pose:", goal)
            print("  Actual TCP pose:   ", actual_pose)
            print("  Actual joint q:    ", actual_joint_q)
    finally:
        controller.stopScript()

    pd.DataFrame(GOALS, columns=["x", "y", "z", "rx", "ry", "rz"]).to_csv(
        "goals.csv", index=False
    )
    pd.DataFrame(
        actual_joint_qs,
        columns=["base", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"],
    ).to_csv("task1_joint_q.csv", index=False)

    results = pd.DataFrame(GOALS, columns=["target_x", "target_y", "target_z", "target_rx", "target_ry", "target_rz"])
    results[["actual_x", "actual_y", "actual_z", "actual_rx", "actual_ry", "actual_rz"]] = actual_poses
    results.to_csv("task1_results.csv", index=False)


if __name__ == "__main__":
    main()
