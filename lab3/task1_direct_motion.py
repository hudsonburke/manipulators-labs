"""Task 1: solve IK, try the direct trajectory, and inspect its collision."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import roboticstoolbox as rtb
import student_config as config
from lab3lib.model import (
    check_model_agreement,
    make_robot,
    solve_goal_configuration,
    target_pose,
)
from lab3lib.plotting import sample_configuration_slice, save_configuration_slice
from lab3lib.viewer import Motion
from lab3lib.world import MujocoWorld

LAB_DIR = Path(__file__).resolve().parent
MODEL_PATH = LAB_DIR / "models" / "ur5_scene.xml"
OUTPUT_DIR = LAB_DIR / "outputs"
JOINT_COLUMNS = ["base", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-viewer", action="store_true", help="generate results without starting mjviser")
    parser.add_argument("--skip-slice", action="store_true", help="skip the configuration-space slice")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    OUTPUT_DIR.mkdir(exist_ok=True)

    robot = make_robot()
    world = MujocoWorld(MODEL_PATH, safety_margin=config.SAFETY_MARGIN)
    target = target_pose(config.GOAL_POSITION, config.GOAL_ROTATION)
    q_goal, position_error, orientation_error = solve_goal_configuration(
        robot, target, config.IK_SEED
    )

    agreement = check_model_agreement(
        robot,
        world,
        np.vstack((config.START_Q, q_goal, (config.START_Q + q_goal) / 2)),
    )
    print("Robotics Toolbox / MuJoCo agreement")
    print(f"  maximum position error:    {agreement.maximum_position_error:.3e} m")
    print(f"  maximum orientation error: {agreement.maximum_orientation_error:.3e} rad")
    print("IK result")
    print("  q_goal:", np.array2string(q_goal, precision=4))
    print(f"  FK position error:    {position_error:.3e} m")
    print(f"  FK orientation error: {orientation_error:.3e} rad")
    print("  goal validity:", world.check_configuration(q_goal).summary())

    duration = 4.0
    direct = rtb.jtraj(
        config.START_Q,
        q_goal,
        np.linspace(0.0, duration, 201),
    )
    direct_report = world.check_path(
        direct.q,
        resolution=config.PATH_VALIDATION_DISTANCE,
    )
    print("Direct trajectory")
    print(" ", direct_report.summary())

    pd.DataFrame([q_goal], columns=JOINT_COLUMNS).to_csv(
        OUTPUT_DIR / "task1_goal_q.csv", index=False
    )
    trajectory_data = pd.DataFrame(direct.q, columns=[f"q_{name}" for name in JOINT_COLUMNS])
    trajectory_data.insert(0, "time", direct.t)
    trajectory_data[[f"qd_{name}" for name in JOINT_COLUMNS]] = direct.qd
    trajectory_data.to_csv(OUTPUT_DIR / "task1_direct_trajectory.csv", index=False)

    if not args.skip_slice:
        joint_indices = (0, 2)
        first, second, valid = sample_configuration_slice(
            world,
            config.START_Q,
            joint_indices=joint_indices,
            samples=70,
        )
        save_configuration_slice(
            OUTPUT_DIR / "task1_configuration_slice.png",
            first,
            second,
            valid,
            joint_indices=joint_indices,
            start=config.START_Q,
            goal=q_goal,
        )
        print("Saved task1_configuration_slice.png")

    if not args.no_viewer:
        world.play(
            [Motion.from_samples("Direct trajectory", direct.q, direct.t, direct.qd)],
            port=config.VIEWER_PORT,
        )


if __name__ == "__main__":
    main()
