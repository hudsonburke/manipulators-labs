"""Small student-facing wrapper around OMPL manipulator planners."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from time import perf_counter

import numpy as np
from numpy.typing import ArrayLike, NDArray
from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou


@dataclass(frozen=True)
class PlanningResult:
    """Observable outputs from one randomized planning run."""

    solved: bool
    waypoints: NDArray[np.float64]
    raw_waypoints: NDArray[np.float64]
    tree_states: NDArray[np.float64]
    planning_time: float
    simplification_time: float
    state_checks: int
    algorithm: str
    seed: int
    message: str

    @property
    def raw_waypoint_count(self) -> int:
        return len(self.raw_waypoints)

    @property
    def waypoint_count(self) -> int:
        return len(self.waypoints)

    def summary(self) -> str:
        if not self.solved:
            return (
                f"{self.algorithm} did not find a path in "
                f"{self.planning_time:.3f} s ({self.state_checks} state checks)."
            )
        return (
            f"{self.algorithm} found a path in {self.planning_time:.3f} s; "
            f"{self.raw_waypoint_count} raw → {self.waypoint_count} simplified "
            f"waypoints; {self.state_checks} state checks."
        )


class JointSpacePlanner:
    """Plan in a bounded real-vector joint space using a collision callback."""

    def __init__(
        self,
        joint_limits: ArrayLike,
        state_is_valid: Callable[[NDArray[np.float64]], bool],
        *,
        seed: int = 1,
        verbose: bool = False,
    ) -> None:
        limits = np.asarray(joint_limits, dtype=float)
        if limits.shape != (2, 6):
            raise ValueError("Joint limits must have shape (2, 6).")
        if np.any(limits[0] >= limits[1]):
            raise ValueError("Every lower joint limit must be below its upper limit.")
        if seed < 1:
            raise ValueError("OMPL requires a positive random seed.")
        self.joint_limits = limits.copy()
        self.state_is_valid = state_is_valid
        self.seed = int(seed)
        ou.RNG.setSeed(self.seed)
        if not verbose:
            ou.setLogLevel(ou.LOG_WARN)

    @staticmethod
    def _state_to_array(state: ob.State, dimension: int = 6) -> NDArray[np.float64]:
        return np.fromiter((state[index] for index in range(dimension)), dtype=float)

    @classmethod
    def _path_to_array(cls, path: og.PathGeometric) -> NDArray[np.float64]:
        return np.vstack([cls._state_to_array(state) for state in path.getStates()])

    @classmethod
    def _planner_states(cls, setup: og.SimpleSetup) -> NDArray[np.float64]:
        data = ob.PlannerData(setup.getSpaceInformation())
        setup.getPlannerData(data)
        if data.numVertices() == 0:
            return np.empty((0, 6), dtype=float)
        return np.vstack(
            [
                cls._state_to_array(data.getVertex(index).getState())
                for index in range(data.numVertices())
            ]
        )

    def plan(
        self,
        start: ArrayLike,
        goal: ArrayLike,
        *,
        algorithm: str = "RRTConnect",
        max_connection_distance: float = 0.35,
        validation_distance: float = 0.04,
        goal_bias: float = 0.10,
        timeout: float = 3.0,
        simplify: bool = True,
        simplification_time: float = 0.25,
    ) -> PlanningResult:
        """Plan between two six-joint configurations.

        ``validation_distance`` bounds OMPL's Euclidean spacing between state
        checks. The final path must still be independently checked by
        :meth:`MujocoWorld.check_path`.
        """
        start_q = np.asarray(start, dtype=float)
        goal_q = np.asarray(goal, dtype=float)
        if start_q.shape != (6,) or goal_q.shape != (6,):
            raise ValueError("Start and goal must each contain six joint values.")
        if not self.state_is_valid(start_q):
            raise ValueError("The planning start configuration is invalid.")
        if not self.state_is_valid(goal_q):
            raise ValueError("The planning goal configuration is invalid.")
        if max_connection_distance <= 0 or validation_distance <= 0 or timeout <= 0:
            raise ValueError("Planner distances and timeout must be positive.")
        if not 0 <= goal_bias <= 1:
            raise ValueError("Goal bias must lie between 0 and 1.")

        space = ob.RealVectorStateSpace(6)
        bounds = ob.RealVectorBounds(6)
        for index in range(6):
            bounds.setLow(index, float(self.joint_limits[0, index]))
            bounds.setHigh(index, float(self.joint_limits[1, index]))
        space.setBounds(bounds)

        setup = og.SimpleSetup(space)
        state_checks = [0]

        def valid(state: ob.State) -> bool:
            state_checks[0] += 1
            return self.state_is_valid(self._state_to_array(state))

        setup.setStateValidityChecker(valid)
        information = setup.getSpaceInformation()
        resolution_fraction = validation_distance / space.getMaximumExtent()
        information.setStateValidityCheckingResolution(resolution_fraction)

        start_state = space.allocState()
        goal_state = space.allocState()
        for index in range(6):
            start_state[index] = float(start_q[index])
            goal_state[index] = float(goal_q[index])
        setup.setStartAndGoalStates(start_state, goal_state)

        algorithm_key = algorithm.casefold()
        if algorithm_key == "rrtconnect":
            planner = og.RRTConnect(information)
        elif algorithm_key == "rrt":
            planner = og.RRT(information)
            planner.setGoalBias(goal_bias)
        else:
            raise ValueError("algorithm must be 'RRTConnect' or 'RRT'.")
        planner.setRange(float(max_connection_distance))
        setup.setPlanner(planner)
        setup.setup()

        started = perf_counter()
        status = setup.solve(float(timeout))
        planning_time = perf_counter() - started
        tree_states = self._planner_states(setup)

        if not bool(status):
            empty = np.empty((0, 6), dtype=float)
            return PlanningResult(
                False,
                empty,
                empty,
                tree_states,
                planning_time,
                0.0,
                state_checks[0],
                algorithm,
                self.seed,
                str(status),
            )

        raw_waypoints = self._path_to_array(setup.getSolutionPath()).copy()
        elapsed_simplification = 0.0
        if simplify:
            simplify_started = perf_counter()
            setup.simplifySolution(float(simplification_time))
            elapsed_simplification = perf_counter() - simplify_started
        waypoints = self._path_to_array(setup.getSolutionPath()).copy()

        return PlanningResult(
            True,
            waypoints,
            raw_waypoints,
            tree_states,
            planning_time,
            elapsed_simplification,
            state_checks[0],
            algorithm,
            self.seed,
            str(status),
        )
