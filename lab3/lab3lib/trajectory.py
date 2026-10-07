"""Trajectory helpers built around Robotics Toolbox for Python."""

from __future__ import annotations

from itertools import pairwise

import numpy as np
import roboticstoolbox as rtb
from numpy.typing import ArrayLike, NDArray
from roboticstoolbox.tools.trajectory import Trajectory


def path_length(path: ArrayLike, joint_ranges: ArrayLike | None = None) -> float:
    """Return piecewise-linear joint-space path length.

    If ``joint_ranges`` is supplied, each joint difference is divided by its
    range before the Euclidean norm is computed.
    """
    states = np.asarray(path, dtype=float)
    if states.ndim != 2 or states.shape[1] != 6:
        raise ValueError("Path must have shape (samples, 6).")
    if len(states) < 2:
        return 0.0

    differences = np.diff(states, axis=0)
    if joint_ranges is not None:
        ranges = np.asarray(joint_ranges, dtype=float)
        if ranges.shape != (6,) or np.any(ranges <= 0):
            raise ValueError("Joint ranges must be six positive values.")
        differences = differences / ranges
    return float(np.linalg.norm(differences, axis=1).sum())


def concatenate_jtraj(
    waypoints: ArrayLike,
    *,
    dt: float = 0.02,
    qdmax: ArrayLike | float = 0.5,
    minimum_segment_time: float = 0.5,
) -> Trajectory:
    """Join stop-at-waypoint quintic ``jtraj`` segments.

    Every segment remains on the planner's straight joint-space edge. The
    quintic profile has zero velocity and acceleration at each waypoint.
    """
    points = np.asarray(waypoints, dtype=float)
    if points.ndim != 2 or points.shape[1] != 6 or len(points) < 2:
        raise ValueError("At least two six-joint waypoints are required.")
    if dt <= 0 or minimum_segment_time <= 0:
        raise ValueError("Trajectory time values must be positive.")

    speeds = np.broadcast_to(np.asarray(qdmax, dtype=float), (6,))
    if np.any(speeds <= 0):
        raise ValueError("Joint speed limits must be positive.")

    times: list[NDArray[np.float64]] = []
    positions: list[NDArray[np.float64]] = []
    velocities: list[NDArray[np.float64]] = []
    accelerations: list[NDArray[np.float64]] = []
    elapsed = 0.0

    for segment_index, (q_start, q_end) in enumerate(pairwise(points)):
        # A quintic blend's peak scalar speed is 1.875 / duration.
        duration = max(
            minimum_segment_time,
            float(np.max(1.875 * np.abs(q_end - q_start) / speeds)),
        )
        sample_count = max(2, int(np.ceil(duration / dt)) + 1)
        segment_time = np.linspace(0.0, duration, sample_count)
        segment = rtb.jtraj(q_start, q_end, segment_time)

        first = 0 if segment_index == 0 else 1
        times.append(segment_time[first:] + elapsed)
        positions.append(segment.q[first:])
        velocities.append(segment.qd[first:])
        accelerations.append(segment.qdd[first:])
        elapsed += duration

    return Trajectory(
        "piecewise jtraj",
        np.concatenate(times),
        np.vstack(positions),
        np.vstack(velocities),
        np.vstack(accelerations),
        istime=True,
    )

