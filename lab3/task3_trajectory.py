"""Task 3: turn a collision-free path into timed joint trajectories."""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd
import roboticstoolbox as rtb
import student_config as config
from lab3lib.plotting import save_joint_plots
from lab3lib.trajectory import add_numerical_derivatives, concatenate_jtraj
from lab3lib.viewer import Motion
from lab3lib.world import MujocoWorld

LAB_DIR = Path(__file__).resolve().parent
MODEL_PATH = LAB_DIR / "models" / "ur5_scene.xml"
OUTPUT_DIR = LAB_DIR / "outputs"
PATH_FILE = OUTPUT_DIR / "task2_simplified_path.csv"
JOINT_COLUMNS = ["base", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-viewer", action="store_true", help="generate results without starting mjviser")
    return parser.parse_args()


def save_trajectory(path: Path, trajectory) -> None:
    data = pd.DataFrame(trajectory.q, columns=[f"q_{name}" for name in JOINT_COLUMNS])
    data.insert(0, "time", trajectory.t)
    data[[f"qd_{name}" for name in JOINT_COLUMNS]] = trajectory.qd
    data[[f"qdd_{name}" for name in JOINT_COLUMNS]] = trajectory.qdd
    data.to_csv(path, index=False)


def main() -> None:
    args = parse_args()
    if not PATH_FILE.exists():
        raise FileNotFoundError("Run Task 2 first to create task2_simplified_path.csv.")

    path_frame = pd.read_csv(PATH_FILE)
    if list(path_frame.columns) != JOINT_COLUMNS:
        raise ValueError("Task 2 path columns are invalid; rerun Task 2.")
    waypoints = path_frame.to_numpy(dtype=float)

    world = MujocoWorld(MODEL_PATH, safety_margin=config.SAFETY_MARGIN)
    waypoint_report = world.check_path(
        waypoints,
        resolution=config.PATH_VALIDATION_DISTANCE,
    )
    print("Input path:", waypoint_report.summary())
    if not waypoint_report.valid:
        raise RuntimeError("The saved Task 2 path is not collision-free.")

    stop_at_waypoints = concatenate_jtraj(
        waypoints,
        dt=config.TRAJECTORY_DT,
        qdmax=config.JOINT_SPEED_LIMITS,
    )
    blended = add_numerical_derivatives(
        rtb.mstraj(
            waypoints,
            dt=config.TRAJECTORY_DT,
            tacc=config.BLEND_TIME,
            qdmax=config.JOINT_SPEED_LIMITS,
        ),
        initial_q=waypoints[0],
        qdmax=config.JOINT_SPEED_LIMITS,
    )

    stop_report = world.check_path(
        stop_at_waypoints.q,
        resolution=config.PATH_VALIDATION_DISTANCE,
    )
    blended_report = world.check_path(
        blended.q,
        resolution=config.PATH_VALIDATION_DISTANCE,
    )
    print("Stop-at-waypoint trajectory:", stop_report.summary())
    print("Blended mstraj trajectory:     ", blended_report.summary())
    print(f"Stop-at-waypoint duration: {stop_at_waypoints.t[-1]:.2f} s")
    print(f"Blended duration:           {blended.t[-1]:.2f} s")

    save_trajectory(OUTPUT_DIR / "task3_stop_at_waypoints.csv", stop_at_waypoints)
    save_trajectory(OUTPUT_DIR / "task3_blended_trajectory.csv", blended)
    save_joint_plots(
        OUTPUT_DIR / "task3_stop_at_waypoints.png",
        stop_at_waypoints.t,
        stop_at_waypoints.q,
        stop_at_waypoints.qd,
        speed_limits=config.JOINT_SPEED_LIMITS,
    )
    save_joint_plots(
        OUTPUT_DIR / "task3_blended_trajectory.png",
        blended.t,
        blended.q,
        blended.qd,
        speed_limits=config.JOINT_SPEED_LIMITS,
    )

    if not blended_report.valid:
        print(
            "WARNING: Blending left the collision-checked path. Reduce BLEND_TIME "
            "or replan with more clearance before executing this trajectory."
        )

    if not args.no_viewer:
        world.play(
            [
                Motion.from_samples("Planned waypoints", waypoints),
                Motion.from_samples(
                    "Stop at waypoints",
                    stop_at_waypoints.q,
                    stop_at_waypoints.t,
                    stop_at_waypoints.qd,
                ),
                Motion.from_samples(
                    "Blended mstraj",
                    blended.q,
                    blended.t,
                    blended.qd,
                ),
            ],
            port=config.VIEWER_PORT,
        )


if __name__ == "__main__":
    main()
