"""Generated figures for Lab 3 reports."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import matplotlib
import numpy as np
from numpy.typing import ArrayLike

matplotlib.use("Agg")
import matplotlib.pyplot as plt

if TYPE_CHECKING:
    from .planner import PlanningResult

JOINT_LABELS = (
    "base",
    "shoulder lift",
    "elbow",
    "wrist 1",
    "wrist 2",
    "wrist 3",
)


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
    if not result.solved:
        axis.plot(
            result.closest_waypoints[:, first],
            result.closest_waypoints[:, second],
            "o-", markersize=3, color="#d47b00",
            label="closest start-connected partial path",
        )
    if len(result.raw_waypoints):
        axis.plot(
            result.raw_waypoints[:, first],
            result.raw_waypoints[:, second],
            "o-",
            markersize=3,
            linewidth=1,
            color="#d47b00",
            label="raw candidate",
        )
    if len(result.waypoints):
        axis.plot(
            result.waypoints[:, first],
            result.waypoints[:, second],
            "o-",
            markersize=4,
            linewidth=2,
            color="#247a3c",
            label="simplified candidate (see validation)",
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


def save_joint_position_plot(
    path: str | Path, t: ArrayLike, q: ArrayLike, *, title: str
) -> None:
    """Save the six joint positions as a function of trajectory time."""
    time_values = np.asarray(t, dtype=float)
    positions = np.asarray(q, dtype=float)
    if positions.shape != (len(time_values), 6):
        raise ValueError("Joint positions must have shape (samples, 6).")

    figure, axis = plt.subplots(figsize=(10, 5))
    for joint, label in enumerate(JOINT_LABELS):
        axis.plot(time_values, positions[:, joint], label=label)
    axis.set_xlabel("time (s)")
    axis.set_ylabel("joint position (rad)")
    axis.set_title(title)
    axis.grid(alpha=0.25)
    axis.legend(ncol=3, fontsize="small")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)

