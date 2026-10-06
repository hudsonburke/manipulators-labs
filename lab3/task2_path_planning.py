"""Task 2: use RRT-Connect and investigate a planning parameter."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import roboticstoolbox as rtb
import student_config as config
from lab3lib.model import make_robot, solve_goal_configuration, target_pose
from lab3lib.planner import JointSpacePlanner
from lab3lib.plotting import save_parameter_plot, save_planner_projection
from lab3lib.trajectory import path_length
from lab3lib.viewer import Motion
from lab3lib.world import MujocoWorld

LAB_DIR = Path(__file__).resolve().parent
MODEL_PATH = LAB_DIR / "models" / "ur5_scene.xml"
OUTPUT_DIR = LAB_DIR / "outputs"
JOINT_COLUMNS = ["base", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-viewer", action="store_true", help="generate results without starting mjviser")
    parser.add_argument("--skip-experiment", action="store_true", help="run only the baseline plan")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    OUTPUT_DIR.mkdir(exist_ok=True)

    robot = make_robot()
    world = MujocoWorld(MODEL_PATH, safety_margin=config.SAFETY_MARGIN)
    target = target_pose(config.GOAL_POSITION, config.GOAL_ROTATION)
    q_goal, _, _ = solve_goal_configuration(robot, target, config.IK_SEED)
    planner = JointSpacePlanner(
        world.joint_limits,
        world.is_collision_free,
        seed=config.PLANNER_SEED,
    )

    result = planner.plan(
        config.START_Q,
        q_goal,
        algorithm="RRTConnect",
        max_connection_distance=config.MAX_CONNECTION_DISTANCE,
        validation_distance=config.PLANNER_VALIDATION_DISTANCE,
        timeout=config.PLANNING_TIMEOUT,
        simplify=True,
    )
    print(result.summary())
    if not result.solved:
        raise RuntimeError("The baseline planner did not find a path. Try another seed.")

    final_report = world.check_path(
        result.waypoints,
        resolution=config.PATH_VALIDATION_DISTANCE,
    )
    print("Independent final validation:", final_report.summary())
    if not final_report.valid:
        raise RuntimeError("The planner output failed independent path validation.")

    joint_ranges = np.ptp(world.joint_limits, axis=0)
    print(f"Raw normalized path length:        {path_length(result.raw_waypoints, joint_ranges):.3f}")
    print(f"Simplified normalized path length: {path_length(result.waypoints, joint_ranges):.3f}")

    pd.DataFrame(result.raw_waypoints, columns=JOINT_COLUMNS).to_csv(
        OUTPUT_DIR / "task2_raw_path.csv", index=False
    )
    pd.DataFrame(result.waypoints, columns=JOINT_COLUMNS).to_csv(
        OUTPUT_DIR / "task2_simplified_path.csv", index=False
    )
    save_planner_projection(
        OUTPUT_DIR / "task2_planner_projection.png",
        result,
        joint_indices=(0, 2),
    )

    if not args.skip_experiment:
        rows: list[dict[str, float | int | bool]] = []
        for connection_distance in config.CONNECTION_DISTANCE_EXPERIMENT:
            for trial_index in range(config.EXPERIMENT_TRIALS):
                trial = planner.plan(
                    config.START_Q,
                    q_goal,
                    algorithm="RRTConnect",
                    max_connection_distance=connection_distance,
                    validation_distance=config.PLANNER_VALIDATION_DISTANCE,
                    timeout=config.PLANNING_TIMEOUT,
                    simplify=False,
                )
                rows.append(
                    {
                        "max_connection_distance": connection_distance,
                        "trial": trial_index + 1,
                        "solved": trial.solved,
                        "planning_time_s": trial.planning_time,
                        "state_checks": trial.state_checks,
                        "waypoints": trial.waypoint_count,
                        "normalized_path_length": (
                            path_length(trial.waypoints, joint_ranges)
                            if trial.solved
                            else np.nan
                        ),
                    }
                )
        experiment = pd.DataFrame(rows)
        experiment.to_csv(OUTPUT_DIR / "task2_parameter_results.csv", index=False)
        save_parameter_plot(
            OUTPUT_DIR / "task2_parameter_results.png",
            experiment,
            parameter="max_connection_distance",
        )
        print("\nConnection-distance experiment")
        print(
            experiment.groupby("max_connection_distance").agg(
                success_rate=("solved", "mean"),
                median_time_s=("planning_time_s", "median"),
                median_state_checks=("state_checks", "median"),
            )
        )

    if not args.no_viewer:
        direct = rtb.jtraj(
            config.START_Q,
            q_goal,
            np.linspace(0.0, 4.0, 201),
        )
        world.play(
            [
                Motion.from_samples("Direct trajectory", direct.q, direct.t, direct.qd),
                Motion.from_samples("Raw planned path", result.raw_waypoints),
                Motion.from_samples("Simplified path", result.waypoints),
            ],
            port=config.VIEWER_PORT,
        )


if __name__ == "__main__":
    main()
