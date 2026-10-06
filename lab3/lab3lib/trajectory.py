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


def add_numerical_derivatives(
    trajectory: Trajectory,
    *,
    initial_q: ArrayLike | None = None,
    qdmax: ArrayLike | float | None = None,
) -> Trajectory:
    """Complete an ``mstraj`` result and estimate its derivatives.

    Robotics Toolbox's ``mstraj`` currently omits derivative fields and labels
    its first post-start sample as time zero. Supplying ``initial_q`` restores
    the missing initial sample. If ``qdmax`` is supplied, the time axis is
    stretched until the measured joint velocities respect those limits.
    With ``initial_q``, this lab's rest-to-rest ``mstraj`` endpoints are fixed
    at zero velocity after the finite differences are computed.
    """
    positions = np.asarray(trajectory.q, dtype=float)
    times = np.asarray(trajectory.t, dtype=float)
    if len(times) < 3:
        raise ValueError("At least three trajectory samples are required.")
    if np.any(np.diff(times) <= 0):
        raise ValueError("Trajectory time values must be strictly increasing.")

    if initial_q is not None:
        initial = np.asarray(initial_q, dtype=float)
        if initial.shape != (positions.shape[1],):
            raise ValueError("Initial configuration has the wrong shape.")
        if not np.allclose(positions[0], initial):
            sample_time = float(np.median(np.diff(times)))
            positions = np.vstack((initial, positions))
            times = np.concatenate(([0.0], times + sample_time))
        else:
            times = times - times[0]

    velocity = np.gradient(positions, times, axis=0, edge_order=2)
    if initial_q is not None:
        velocity[0] = 0.0
        velocity[-1] = 0.0
    if qdmax is not None:
        limits = np.broadcast_to(
            np.asarray(qdmax, dtype=float), (positions.shape[1],)
        )
        if np.any(limits <= 0):
            raise ValueError("Joint speed limits must be positive.")
        speed_ratio = float(np.max(np.abs(velocity) / limits))
        if speed_ratio > 1.0:
            times *= speed_ratio * (1.0 + 1e-9)
            velocity = np.gradient(positions, times, axis=0, edge_order=2)
            if initial_q is not None:
                velocity[0] = 0.0
                velocity[-1] = 0.0

    acceleration = np.gradient(velocity, times, axis=0, edge_order=2)
    return Trajectory(
        trajectory.name,
        times,
        positions,
        velocity,
        acceleration,
        istime=True,
    )
