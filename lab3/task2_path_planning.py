"""Task 2: tune UR5 narrow-gap planning and inspect timed attempts in Swift."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import student_config as config
from lab3lib.model import make_robot, solve_goal_configuration, target_pose
from lab3lib.planner import JointSpacePlanner
from lab3lib.plotting import save_joint_position_plot, save_planner_projection
from lab3lib.trajectory import concatenate_jtraj, path_length
from lab3lib.viewer import Motion
from lab3lib.world import CollisionWorld, ConfigurationReport

LAB_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = LAB_DIR / "outputs"
JOINT_COLUMNS = ["base", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-viewer", action="store_true", help="generate results without starting Swift")
    return parser.parse_args()


def timed_attempt(
    world: CollisionWorld, waypoints: np.ndarray, name: str
) -> tuple[Motion, ConfigurationReport]:
    """Run a timed path out to its first fine-checked collision, inclusive."""
    trajectory = concatenate_jtraj(
        waypoints, dt=config.TRAJECTORY_DT, qdmax=config.JOINT_SPEED_LIMITS,
    )
    positions = [trajectory.q[0]]
    times = [float(trajectory.t[0])]
    report = world.check_configuration(positions[0])
    if report.valid:
        for index in range(1, len(trajectory.q)):
            start, end = trajectory.q[index - 1:index + 1]
            count = max(1, int(np.ceil(np.max(np.abs(end - start)) / config.PATH_VALIDATION_DISTANCE)))
            for step in range(1, count + 1):
                fraction = step / count
                q = start + fraction * (end - start)
                report = world.check_configuration(q)
                positions.append(q)
                times.append(float(trajectory.t[index - 1] + fraction * (
                    trajectory.t[index] - trajectory.t[index - 1]
                )))
                if not report.valid:
                    break
            if not report.valid:
                break
    label = f"{name} — {'goal reached' if report.valid else 'stopped at first collision'}"
    return Motion.from_samples(label, np.vstack(positions), times), report


def main() -> None:
    args = parse_args()
    OUTPUT_DIR.mkdir(exist_ok=True)
    robot = make_robot()
    world = CollisionWorld(robot, safety_margin=config.SAFETY_MARGIN)
    target = target_pose(config.GOAL_POSITION, config.GOAL_ROTATION)
    q_goal, _, _ = solve_goal_configuration(robot, target, config.IK_SEED)
    planner = JointSpacePlanner(world.joint_limits, world.is_collision_free, seed=config.PLANNER_SEED)
    result = planner.plan(
        config.START_Q, q_goal, algorithm="RRTConnect",
        max_connection_distance=config.MAX_CONNECTION_DISTANCE,
        validation_distance=config.PLANNER_VALIDATION_DISTANCE,
        timeout=config.PLANNING_TIMEOUT, simplify=True,
    )
    print(result.summary())
    for filename, path in (
        ("task2_raw_path.csv", result.raw_waypoints),
        ("task2_simplified_path.csv", result.waypoints),
        ("task2_closest_partial_path.csv", result.closest_waypoints),
    ):
        pd.DataFrame(path, columns=JOINT_COLUMNS).to_csv(OUTPUT_DIR / filename, index=False)
    save_planner_projection(OUTPUT_DIR / "task2_planner_projection.png", result, joint_indices=(0, 2))

    if result.solved:
        ranges = np.ptp(world.joint_limits, axis=0)
        print(f"Raw normalized path length:        {path_length(result.raw_waypoints, ranges):.3f}")
        print(f"Simplified normalized path length: {path_length(result.waypoints, ranges):.3f}")
        report = world.check_path(result.waypoints, resolution=config.PATH_VALIDATION_DISTANCE)
        print("Independent final validation:", report.summary())
        if not report.valid:
            print("Reduce PLANNER_VALIDATION_DISTANCE; the returned candidate is not a valid solution.")
        attempt = result.waypoints
        name = "Returned candidate trajectory"
    else:
        closest = result.closest_waypoints
        distance = np.linalg.norm((closest[-1] - q_goal) / np.ptp(world.joint_limits, axis=0))
        print(f"Closest start-connected branch: {len(closest)} vertices; normalized goal distance {distance:.3f}.")
        print("Replay that branch, then attempt a straight joint-space extension to the goal. "
              "The extension is diagnostic, NOT an RRT solution.")
        attempt = np.vstack([closest, q_goal])
        name = "Closest partial path + UNPLANNED goal attempt"

    motion, collision_report = timed_attempt(world, attempt, name)
    print(f"Trajectory playback: {len(motion.q)} samples, {motion.t[-1]:.2f} s; {collision_report.summary()}")
    if not collision_report.valid:
        print("Swift stops and holds the first colliding configuration; no later samples are played.")
    elif not result.solved:
        print("The diagnostic extension reached the goal, but is not a planner-reported solution.")
    frame = pd.DataFrame(motion.q, columns=JOINT_COLUMNS)
    frame.insert(0, "time", motion.t)
    frame["collision"] = False
    frame.loc[len(frame) - 1, "collision"] = not collision_report.valid
    frame.to_csv(OUTPUT_DIR / "task2_attempt_trajectory.csv", index=False)
    save_joint_position_plot(
        OUTPUT_DIR / "task2_joint_position_over_time.png",
        motion.t, motion.q, title=motion.name,
    )
    if not args.no_viewer:
        world.play([motion], port=config.VIEWER_PORT, websocket_port=config.VIEWER_WEBSOCKET_PORT)


if __name__ == "__main__":
    main()
