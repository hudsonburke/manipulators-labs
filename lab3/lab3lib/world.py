"""Robotics Toolbox configuration and path validation for the Lab 3 UR5."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from itertools import combinations, pairwise
from typing import TYPE_CHECKING

import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
from numpy.typing import ArrayLike, NDArray
from spatialmath import SE3

if TYPE_CHECKING:
    from .viewer import Motion


@dataclass(frozen=True)
class CollisionPair:
    """Robot links or obstacles that penetrate or violate clearance."""

    geom1: str
    geom2: str
    distance: float


@dataclass(frozen=True)
class ConfigurationReport:
    """Validation result for one joint configuration."""

    valid: bool
    q: NDArray[np.float64]
    collisions: tuple[CollisionPair, ...] = ()
    reason: str | None = None

    def summary(self) -> str:
        if self.valid:
            return "configuration is collision-free"
        if self.reason is not None:
            return self.reason
        pairs = ", ".join(
            f"{pair.geom1} ↔ {pair.geom2} ({pair.distance:.4f} m)"
            for pair in self.collisions
        )
        return f"collision: {pairs}"


@dataclass(frozen=True)
class PathReport:
    """Validation result for a piecewise-linear joint-space path."""

    valid: bool
    checked_configurations: int
    segment_index: int | None = None
    fraction: float | None = None
    configuration: ConfigurationReport | None = None

    def summary(self) -> str:
        if self.valid:
            return f"path is collision-free ({self.checked_configurations} checks)"
        assert self.segment_index is not None
        assert self.fraction is not None
        assert self.configuration is not None
        return (
            f"path is invalid on segment {self.segment_index + 1} at "
            f"{self.fraction:.1%}: {self.configuration.summary()}"
        )


class CollisionWorld:
    """Share one RTB URDF robot between planning queries and Swift playback.

    Robot-obstacle distances use the supplied collision geometry. Self checks
    compare nonadjacent collision-bearing links; adjacent links are excluded
    because their geometry intentionally meets at the joint.
    """

    def __init__(
        self,
        robot: rtb.Robot,
        *,
        safety_margin: float = 0.0,
        obstacles: Mapping[str, sg.Shape] | None = None,
    ) -> None:
        self.robot = robot
        self.safety_margin = float(safety_margin)
        if not np.isfinite(self.safety_margin) or self.safety_margin < 0:
            raise ValueError("Safety margin must be finite and nonnegative.")
        self.joint_limits = np.asarray(robot.qlim, dtype=float).copy()
        self.obstacles = dict(obstacles) if obstacles is not None else {
            "floor": sg.Cuboid(
                [2.8, 2.8, 0.10], pose=SE3(0, 0, -0.17),
                color=[0.82, 0.84, 0.87, 1],
            ),
            # Two wall sections leave a physical 0.18 m-high aperture:
            # x in [-1.10, -0.23], y in [-0.03, 0.03], z in (0.42, 0.60).
            # The distal arm must lower to cross; a straight start-goal sweep
            # strikes the upper section. The inner x edge clears the shoulder.
            "gap_lower": sg.Cuboid(
                [0.87, 0.06, 0.54], pose=SE3(-0.665, 0, 0.15),
                color=[0.72, 0.24, 0.18, 1],
            ),
            "gap_upper": sg.Cuboid(
                [0.87, 0.06, 0.70], pose=SE3(-0.665, 0, 0.95),
                color=[0.72, 0.24, 0.18, 1],
            ),
        }
        for obstacle in self.obstacles.values():
            obstacle.update()
        self._collision_links = tuple(link for link in robot.links if len(link.collision))
        self._self_pairs = tuple(
            (first, second)
            for first, second in combinations(self._collision_links, 2)
            if first.parent is not second and second.parent is not first
        )

    @property
    def dimension(self) -> int:
        return self.robot.n

    def _as_configuration(self, q: ArrayLike) -> NDArray[np.float64]:
        # RTB's C transform updater reads a contiguous joint vector. Pandas
        # path rows can be strided views despite having the correct shape.
        configuration = np.ascontiguousarray(q, dtype=float)
        if configuration.shape != (self.dimension,):
            raise ValueError(
                f"Expected a {self.dimension}-element joint configuration; "
                f"received shape {configuration.shape}."
            )
        return configuration

    def _collision_pairs(self, configuration: NDArray[np.float64], margin: float):
        self.robot.q = configuration
        # Update once so all collision queries share the same scene transforms.
        self.robot._update_link_tf(configuration)
        self.robot.update()
        for name, obstacle in self.obstacles.items():
            obstacle.update()
            for link in self._collision_links:
                distance, _, _ = link.closest_point(obstacle, margin, skip=True)
                if distance is not None and distance <= margin:
                    yield CollisionPair(link.name, name, float(distance))
        for first, second in self._self_pairs:
            closest = float("inf")
            for first_shape in first.collision:
                for second_shape in second.collision:
                    distance, _, _ = first_shape.closest_point(second_shape, margin)
                    if distance is not None:
                        closest = min(closest, float(distance))
            if closest <= margin:
                yield CollisionPair(first.name, second.name, closest)

    def check_configuration(
        self, q: ArrayLike, *, safety_margin: float | None = None
    ) -> ConfigurationReport:
        """Check finiteness, joint limits, obstacles, and nonadjacent links."""
        configuration = self._as_configuration(q)
        stored_q = configuration.copy()
        if not np.all(np.isfinite(configuration)):
            return ConfigurationReport(False, stored_q, reason="configuration is not finite")
        lower, upper = self.joint_limits
        if np.any(configuration < lower) or np.any(configuration > upper):
            return ConfigurationReport(
                False, stored_q, reason="configuration violates a joint limit",
            )
        margin = self.safety_margin if safety_margin is None else float(safety_margin)
        if not np.isfinite(margin) or margin < 0:
            raise ValueError("Safety margin must be finite and nonnegative.")
        collisions = tuple(self._collision_pairs(configuration, margin))
        return ConfigurationReport(not collisions, stored_q, collisions)

    def is_collision_free(self, q: ArrayLike) -> bool:
        """Short-circuit collision queries for the planner's validity callback."""
        configuration = self._as_configuration(q)
        lower, upper = self.joint_limits
        if (
            not np.all(np.isfinite(configuration))
            or np.any(configuration < lower)
            or np.any(configuration > upper)
        ):
            return False
        return next(self._collision_pairs(configuration, self.safety_margin), None) is None

    def check_path(
        self,
        path: ArrayLike,
        *,
        resolution: float = 0.03,
        safety_margin: float | None = None,
    ) -> PathReport:
        """Densely validate every linear segment of a joint-space path.

        ``resolution`` is the maximum change, in radians, by any one joint
        between adjacent collision checks.
        """
        states = np.asarray(path, dtype=float)
        if states.ndim != 2 or states.shape[1] != self.dimension:
            raise ValueError(
                f"Expected a path with shape (samples, {self.dimension}); "
                f"received {states.shape}."
            )
        if len(states) == 0:
            raise ValueError("A path must contain at least one configuration.")
        if not np.isfinite(resolution) or resolution <= 0:
            raise ValueError("Path validation resolution must be positive and finite.")

        checked = 0
        first = self.check_configuration(states[0], safety_margin=safety_margin)
        checked += 1
        if not first.valid:
            return PathReport(False, checked, 0, 0.0, first)

        for segment_index, (q_start, q_end) in enumerate(pairwise(states)):
            subdivisions = max(
                1,
                int(np.ceil(np.max(np.abs(q_end - q_start)) / resolution)),
            )
            for step in range(1, subdivisions + 1):
                fraction = step / subdivisions
                q = q_start + fraction * (q_end - q_start)
                report = self.check_configuration(q, safety_margin=safety_margin)
                checked += 1
                if not report.valid:
                    return PathReport(
                        False,
                        checked,
                        segment_index,
                        fraction,
                        report,
                    )

        return PathReport(True, checked)

    def play(
        self,
        motions: Mapping[str, Motion] | Sequence[Motion],
        *,
        port: int = 52000,
        websocket_port: int = 53000,
    ) -> None:
        """Launch the browser trajectory viewer and block until Ctrl+C."""
        from .viewer import run_motion_viewer

        run_motion_viewer(self, motions, port=port, websocket_port=websocket_port)
