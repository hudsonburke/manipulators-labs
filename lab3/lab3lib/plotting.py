"""Generated figures for Lab 3 reports."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import matplotlib
import numpy as np
import pandas as pd
from numpy.typing import ArrayLike, NDArray

matplotlib.use("Agg")
import matplotlib.pyplot as plt

if TYPE_CHECKING:
    from .planner import PlanningResult
    from .world import MujocoWorld

JOINT_LABELS = (
    "base",
    "shoulder lift",
    "elbow",
    "wrist 1",
    "wrist 2",
    "wrist 3",
)


def sample_configuration_slice(
    world: MujocoWorld,
    reference_q: ArrayLike,
    *,
    joint_indices: tuple[int, int] = (0, 2),
    samples: int = 70,
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.bool_]]:
    """Sample a two-joint slice of the six-dimensional free space."""
    reference = np.asarray(reference_q, dtype=float)
    if reference.shape != (6,):
        raise ValueError("reference_q must contain six joints.")
    if samples < 2:
        raise ValueError("At least two samples per slice axis are required.")
    first, second = joint_indices
    if first == second or not (0 <= first < 6 and 0 <= second < 6):
        raise ValueError("joint_indices must select two different UR5 joints.")

    first_values = np.linspace(
        world.joint_limits[0, first], world.joint_limits[1, first], samples
    )
    second_values = np.linspace(
        world.joint_limits[0, second], world.joint_limits[1, second], samples
    )
    valid = np.empty((samples, samples), dtype=bool)
    q = reference.copy()
    for row, second_value in enumerate(second_values):
        q[second] = second_value
        for column, first_value in enumerate(first_values):
            q[first] = first_value
            valid[row, column] = world.is_collision_free(q)
    return first_values, second_values, valid


def save_configuration_slice(
    path: str | Path,
    first_values: NDArray[np.float64],
    second_values: NDArray[np.float64],
    valid: NDArray[np.bool_],
    *,
    joint_indices: tuple[int, int],
    start: ArrayLike,
    goal: ArrayLike,
) -> None:
    """Plot a sampled configuration-space slice and direct segment."""
    start_q = np.asarray(start, dtype=float)
    goal_q = np.asarray(goal, dtype=float)
    first, second = joint_indices

    figure, axis = plt.subplots(figsize=(7, 6))
    axis.imshow(
        valid,
        origin="lower",
        extent=(
            first_values[0],
            first_values[-1],
            second_values[0],
            second_values[-1],
        ),
        aspect="auto",
        cmap=matplotlib.colors.ListedColormap(["#b83a3a", "#e9edf2"]),
        interpolation="nearest",
    )
    axis.plot(
        [start_q[first], goal_q[first]],
        [start_q[second], goal_q[second]],
        color="#f2aa00",
        linewidth=2.5,
        label="direct segment",
    )
    axis.scatter(start_q[first], start_q[second], color="#2274a5", s=60, label="start")
    axis.scatter(goal_q[first], goal_q[second], color="#2e8b57", s=60, label="goal")
    axis.set_xlabel(f"{JOINT_LABELS[first]} joint (rad)")
    axis.set_ylabel(f"{JOINT_LABELS[second]} joint (rad)")
    axis.set_title("Two-dimensional slice of the UR5 configuration space")
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def save_planner_projection(
    path: str | Path,
    result: PlanningResult,
    *,
    joint_indices: tuple[int, int] = (0, 2),
) -> None:
    """Plot the planner tree and paths projected onto two joints."""
    first, second = joint_indices
    figure, axis = plt.subplots(figsize=(7, 6))
    if len(result.tree_states):
        axis.scatter(
            result.tree_states[:, first],
            result.tree_states[:, second],
            s=8,
            alpha=0.25,
            color="#52677d",
            label="sampled tree states",
        )
    if len(result.raw_waypoints):
        axis.plot(
            result.raw_waypoints[:, first],
            result.raw_waypoints[:, second],
            "o-",
            markersize=3,
            linewidth=1,
            color="#d47b00",
            label="raw path",
        )
    if len(result.waypoints):
        axis.plot(
            result.waypoints[:, first],
            result.waypoints[:, second],
            "o-",
            markersize=4,
            linewidth=2,
            color="#247a3c",
            label="simplified path",
        )
        axis.scatter(
            result.waypoints[0, first],
            result.waypoints[0, second],
            color="#2274a5",
            s=55,
        )
        axis.scatter(
            result.waypoints[-1, first],
            result.waypoints[-1, second],
            color="#2e8b57",
            s=55,
        )
    axis.set_xlabel(f"{JOINT_LABELS[first]} joint (rad)")
    axis.set_ylabel(f"{JOINT_LABELS[second]} joint (rad)")
    axis.set_title("Projection of the six-dimensional planning problem")
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def save_joint_plots(
    path: str | Path,
    t: ArrayLike,
    q: ArrayLike,
    qd: ArrayLike,
    *,
    speed_limits: ArrayLike | None = None,
) -> None:
    """Save joint position and velocity plots for a timed trajectory."""
    time_values = np.asarray(t, dtype=float)
    positions = np.asarray(q, dtype=float)
    velocities = np.asarray(qd, dtype=float)
    if positions.shape != velocities.shape or positions.shape != (len(time_values), 6):
        raise ValueError("Trajectory arrays must have shapes (samples,), (samples, 6), (samples, 6).")

    figure, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for joint, label in enumerate(JOINT_LABELS):
        axes[0].plot(time_values, positions[:, joint], label=label)
        axes[1].plot(time_values, velocities[:, joint], label=label)
    axes[0].set_ylabel("joint position (rad)")
    axes[1].set_ylabel("joint velocity (rad/s)")
    axes[1].set_xlabel("time (s)")
    axes[0].grid(alpha=0.25)
    axes[1].grid(alpha=0.25)
    axes[0].legend(ncol=3, fontsize="small")

    if speed_limits is not None:
        maximum = float(np.max(np.asarray(speed_limits, dtype=float)))
        axes[1].axhline(maximum, color="black", linestyle="--", linewidth=1)
        axes[1].axhline(-maximum, color="black", linestyle="--", linewidth=1)

    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def save_parameter_plot(
    path: str | Path,
    results: pd.DataFrame,
    *,
    parameter: str,
) -> None:
    """Plot success, planning time, and collision checks for a parameter sweep."""
    grouped = results.groupby(parameter, sort=True)
    summary = grouped.agg(
        success_rate=("solved", "mean"),
        median_time=("planning_time_s", "median"),
        median_checks=("state_checks", "median"),
    )

    figure, axes = plt.subplots(1, 3, figsize=(12, 3.8))
    summary["success_rate"].plot.bar(ax=axes[0], color="#2878b5")
    summary["median_time"].plot.bar(ax=axes[1], color="#d07c22")
    summary["median_checks"].plot.bar(ax=axes[2], color="#3c9b5f")
    axes[0].set_ylabel("success fraction")
    axes[1].set_ylabel("median time (s)")
    axes[2].set_ylabel("median state checks")
    for axis in axes:
        axis.set_xlabel(parameter.replace("_", " "))
        axis.tick_params(axis="x", rotation=0)
        axis.grid(axis="y", alpha=0.2)
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)
