"""MuJoCo-backed configuration and path validation for the Lab 3 UR5."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path
from typing import TYPE_CHECKING

import mujoco
import numpy as np
from numpy.typing import ArrayLike, NDArray

if TYPE_CHECKING:
    from .viewer import Motion

JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


@dataclass(frozen=True)
class CollisionPair:
    """A pair of penetrating or insufficiently separated MuJoCo geoms."""

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


class MujocoWorld:
    """Own the Lab 3 MuJoCo model and answer collision queries.

    Planning queries use a private ``MjData`` instance. Viewer playback creates a
    separate instance so thousands of planner queries never alter visible state.
    """

    def __init__(self, model_path: str | Path, *, safety_margin: float = 0.0):
        self.model_path = Path(model_path).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self._collision_data = mujoco.MjData(self.model)
        self.safety_margin = float(safety_margin)

        joint_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in JOINT_NAMES
        ]
        if any(joint_id < 0 for joint_id in joint_ids):
            raise ValueError("The MuJoCo model does not contain the expected UR5 joints.")

        self.joint_ids = np.asarray(joint_ids, dtype=int)
        self.qpos_indices = self.model.jnt_qposadr[self.joint_ids].astype(int)
        self.qvel_indices = self.model.jnt_dofadr[self.joint_ids].astype(int)
        self.joint_limits = self.model.jnt_range[self.joint_ids].T.copy()

        self._tool_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "tool0"
        )
        if self._tool_site_id < 0:
            raise ValueError("The MuJoCo model must define a site named 'tool0'.")

    @property
    def dimension(self) -> int:
        return len(JOINT_NAMES)

    def keyframe(self, name: str) -> NDArray[np.float64]:
        """Return the six UR5 joint values stored in a named MuJoCo keyframe."""
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, name)
        if key_id < 0:
            raise KeyError(f"Unknown MuJoCo keyframe: {name}")
        return self.model.key_qpos[key_id, self.qpos_indices].copy()

    def _as_configuration(self, q: ArrayLike) -> NDArray[np.float64]:
        configuration = np.asarray(q, dtype=float)
        if configuration.shape != (self.dimension,):
            raise ValueError(
                f"Expected a {self.dimension}-element joint configuration; "
                f"received shape {configuration.shape}."
            )
        return configuration

    def _write_configuration(
        self,
        data: mujoco.MjData,
        q: ArrayLike,
        qd: ArrayLike | None = None,
    ) -> NDArray[np.float64]:
        configuration = self._as_configuration(q)
        data.qpos[self.qpos_indices] = configuration
        data.qvel[:] = 0.0
        if qd is not None:
            velocity = np.asarray(qd, dtype=float)
            if velocity.shape != (self.dimension,):
                raise ValueError(
                    f"Expected a {self.dimension}-element joint velocity; "
                    f"received shape {velocity.shape}."
                )
            data.qvel[self.qvel_indices] = velocity
        mujoco.mj_forward(self.model, data)
        return configuration

    def _contact_pairs(
        self, data: mujoco.MjData, safety_margin: float
    ) -> tuple[CollisionPair, ...]:
        closest_by_pair: dict[tuple[str, str], float] = {}
        for contact in data.contact[: data.ncon]:
            distance = float(contact.dist)
            if distance > safety_margin:
                continue
            names = tuple(
                sorted(
                    (
                        self.model.geom(int(contact.geom[0])).name,
                        self.model.geom(int(contact.geom[1])).name,
                    )
                )
            )
            closest_by_pair[names] = min(
                distance, closest_by_pair.get(names, float("inf"))
            )
        return tuple(
            CollisionPair(geom1, geom2, distance)
            for (geom1, geom2), distance in closest_by_pair.items()
        )

    def check_configuration(
        self, q: ArrayLike, *, safety_margin: float | None = None
    ) -> ConfigurationReport:
        """Check finiteness, joint limits, and MuJoCo contacts for ``q``."""
        configuration = self._as_configuration(q)
        stored_q = configuration.copy()

        if not np.all(np.isfinite(configuration)):
            return ConfigurationReport(False, stored_q, reason="configuration is not finite")

        lower, upper = self.joint_limits
        if np.any(configuration < lower) or np.any(configuration > upper):
            return ConfigurationReport(
                False,
                stored_q,
                reason="configuration violates a joint limit",
            )

        margin = self.safety_margin if safety_margin is None else float(safety_margin)
        self._write_configuration(self._collision_data, configuration)
        collisions = self._contact_pairs(self._collision_data, margin)
        return ConfigurationReport(not collisions, stored_q, collisions)

    def is_collision_free(self, q: ArrayLike) -> bool:
        """Return whether ``q`` is a finite, in-bounds, collision-free state."""
        return self.check_configuration(q).valid

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

    def tool_pose(self, q: ArrayLike) -> NDArray[np.float64]:
        """Return the MuJoCo ``tool0`` pose as a 4×4 homogeneous matrix."""
        self._write_configuration(self._collision_data, q)
        pose = np.eye(4)
        pose[:3, :3] = self._collision_data.site_xmat[self._tool_site_id].reshape(3, 3)
        pose[:3, 3] = self._collision_data.site_xpos[self._tool_site_id]
        return pose

    def play(
        self,
        motions: Mapping[str, Motion] | Sequence[Motion],
        *,
        port: int = 8080,
    ) -> None:
        """Launch the browser trajectory viewer and block until Ctrl+C."""
        from .viewer import run_motion_viewer

        run_motion_viewer(self, motions, port=port)
