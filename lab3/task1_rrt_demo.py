"""Task 1: tune a plot-only RRT through a physical two-obstacle gap."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd
import student_config as config

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Rectangle

LAB_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = LAB_DIR / "outputs"
LINK_LENGTH = 1.0
LINK_RADIUS = 0.045
# Wall slabs span the arm's vertical reach, leaving a 0.32 m physical opening.
OBSTACLES = {
    "lower wall": (1.25, 1.35, -2.10, -0.16),
    "upper wall": (1.25, 1.35, 0.16, 2.10),
}
# At q2 = 0, every permitted q1 sends the straight arm through the wall.
# Switching elbow branches therefore requires the opening, not a wall-end detour.
JOINT_LIMITS = np.array([[-0.88, -2.40], [0.88, 2.40]])
FINE_VALIDATION_DISTANCE = 0.002  # independent of the student's edge spacing


@dataclass(frozen=True)
class PlanarReport:
    valid: bool
    q: np.ndarray
    message: str

    def summary(self) -> str:
        return self.message


def arm_points(q: np.ndarray) -> np.ndarray:
    """Base, elbow and tip for two unit links; no robot/viewer dependency."""
    angles = np.array([q[0], q[0] + q[1]])
    displacements = LINK_LENGTH * np.column_stack((np.cos(angles), np.sin(angles)))
    return np.vstack((np.zeros(2), np.cumsum(displacements, axis=0)))


def segment_hits_rectangle(
    start: np.ndarray, end: np.ndarray, bounds: tuple[float, float, float, float]
) -> bool:
    """Exact capsule/rectangle collision, including contact at the link radius."""
    xmin, xmax, ymin, ymax = bounds
    direction = end - start
    enter, leave = 0.0, 1.0
    for coordinate, delta, lower, upper in zip(start, direction, (xmin, ymin), (xmax, ymax)):
        if delta == 0.0:
            if coordinate < lower or coordinate > upper:
                break
        else:
            first, second = sorted(((lower - coordinate) / delta, (upper - coordinate) / delta))
            enter, leave = max(enter, first), min(leave, second)
            if enter > leave:
                break
    else:
        return True  # the link centerline intersects the rectangle

    radius_squared = LINK_RADIUS**2
    for point in (start, end):
        dx = max(xmin - point[0], 0.0, point[0] - xmax)
        dy = max(ymin - point[1], 0.0, point[1] - ymax)
        if dx * dx + dy * dy <= radius_squared:
            return True
    length_squared = float(direction @ direction)
    for corner in ((xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax)):
        offset = np.asarray(corner) - start
        fraction = np.clip(float(offset @ direction) / length_squared, 0.0, 1.0)
        distance = offset - fraction * direction
        if float(distance @ distance) <= radius_squared:
            return True
    return False


class PlanarWorld:
    """Bounded 2R joint space and two rectangular workspace obstacles."""

    def check_configuration(self, q: np.ndarray) -> PlanarReport:
        configuration = np.asarray(q, dtype=float)
        if configuration.shape != (2,) or not np.all(np.isfinite(configuration)):
            return PlanarReport(False, configuration.copy(), "invalid joint configuration")
        if np.any(configuration < JOINT_LIMITS[0]) or np.any(configuration > JOINT_LIMITS[1]):
            return PlanarReport(False, configuration.copy(), "configuration violates joint limits")
        points = arm_points(configuration)
        for link_index, (start, end) in enumerate(pairwise(points), start=1):
            for name, bounds in OBSTACLES.items():
                if segment_hits_rectangle(start, end, bounds):
                    return PlanarReport(False, configuration.copy(), f"collision: link {link_index} / {name}")
        return PlanarReport(True, configuration.copy(), "configuration is collision-free")

    def is_collision_free(self, q: np.ndarray) -> bool:
        return self.check_configuration(q).valid


def plan_rrt(world: PlanarWorld) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    if not np.isfinite(config.RRT2D_STEP_SIZE) or config.RRT2D_STEP_SIZE <= 0:
        raise ValueError("RRT2D_STEP_SIZE must be finite and positive.")
    if not np.isfinite(config.RRT2D_VALIDATION_DISTANCE) or config.RRT2D_VALIDATION_DISTANCE <= 0:
        raise ValueError("RRT2D_VALIDATION_DISTANCE must be finite and positive.")
    if not 0.0 <= config.RRT2D_GOAL_BIAS <= 1.0:
        raise ValueError("RRT2D_GOAL_BIAS must be between 0 and 1.")
    if config.RRT2D_MAX_ITERATIONS < 1:
        raise ValueError("RRT2D_MAX_ITERATIONS must be at least 1.")
    rng = np.random.default_rng(config.RRT2D_SEED)
    nodes = [config.RRT2D_START_Q.copy()]
    parents = [-1]
    goal_index: int | None = None

    def edge_is_free(first: np.ndarray, second: np.ndarray) -> bool:
        distance = float(np.linalg.norm(second - first))
        count = max(1, int(np.ceil(distance / config.RRT2D_VALIDATION_DISTANCE)))
        return all(
            world.is_collision_free(first + (second - first) * (step / count))
            for step in range(1, count + 1)
        )

    for iteration in range(1, config.RRT2D_MAX_ITERATIONS + 1):
        sample = (
            config.RRT2D_GOAL_Q
            if rng.random() < config.RRT2D_GOAL_BIAS
            else rng.uniform(JOINT_LIMITS[0], JOINT_LIMITS[1])
        )
        nearest = int(np.argmin(np.linalg.norm(np.asarray(nodes) - sample, axis=1)))
        direction = sample - nodes[nearest]
        distance = float(np.linalg.norm(direction))
        if distance == 0:
            continue
        candidate = nodes[nearest] + direction * min(config.RRT2D_STEP_SIZE, distance) / distance
        if not edge_is_free(nodes[nearest], candidate):
            continue
        nodes.append(candidate)
        parents.append(nearest)
        new_index = len(nodes) - 1
        if np.linalg.norm(candidate - config.RRT2D_GOAL_Q) <= config.RRT2D_STEP_SIZE and edge_is_free(candidate, config.RRT2D_GOAL_Q):
            nodes.append(config.RRT2D_GOAL_Q.copy())
            parents.append(new_index)
            goal_index = len(nodes) - 1
            break

    tree = np.vstack(nodes)
    edges = np.asarray([(parent, index) for index, parent in enumerate(parents) if parent >= 0], dtype=int).reshape(-1, 2)
    if goal_index is None:
        return tree, edges, np.empty((0, 2)), iteration
    path_indices = [goal_index]
    while parents[path_indices[-1]] >= 0:
        path_indices.append(parents[path_indices[-1]])
    path_indices.reverse()
    return tree, edges, tree[path_indices], iteration


def interpolate_path(path: np.ndarray, spacing: float) -> np.ndarray:
    samples = [path[0]]
    for first, second in pairwise(path):
        count = max(1, int(np.ceil(np.linalg.norm(second - first) / spacing)))
        samples.extend(first + (second - first) * (step / count) for step in range(1, count + 1))
    return np.asarray(samples)


def validate_path(world: PlanarWorld, path: np.ndarray) -> PlanarReport:
    """Validate every candidate independently, never trust the RRT edge checks."""
    for index, q in enumerate(interpolate_path(path, FINE_VALIDATION_DISTANCE)):
        report = world.check_configuration(q)
        if not report.valid:
            return PlanarReport(False, report.q, f"fine sample {index}: {report.message}")
    return PlanarReport(True, path[-1].copy(), "fine-checked path is collision-free")


def save_tree_plot(
    world: PlanarWorld, tree: np.ndarray, edges: np.ndarray, path: np.ndarray,
    report: PlanarReport | None,
) -> None:
    q1_values = np.linspace(JOINT_LIMITS[0, 0], JOINT_LIMITS[1, 0], 201)
    q2_values = np.linspace(JOINT_LIMITS[0, 1], JOINT_LIMITS[1, 1], 241)
    free = np.array([[world.is_collision_free(np.array([q1, q2])) for q1 in q1_values] for q2 in q2_values])
    figure, (workspace, jointspace) = plt.subplots(1, 2, figsize=(13, 7))
    for name, (xmin, xmax, ymin, ymax) in OBSTACLES.items():
        workspace.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, color="#bd4b4b", alpha=0.8))
        workspace.text((xmin + xmax) / 2, (ymin + ymax) / 2, name, rotation=90, ha="center", va="center", color="white")
    workspace.annotate("0.32 m gap\n0.23 m horizontal link-center clearance", xy=(1.30, 0), xytext=(0.00, 1.65), arrowprops={"arrowstyle": "->"}, fontsize=9)
    if len(path):
        dense_path = interpolate_path(path, 0.015)
        tips = np.asarray([arm_points(q)[-1] for q in dense_path])
        accepted = report is not None and report.valid
        color = "#198754" if accepted else "#b83030"
        label = "fine-validated solution" if accepted else "rejected candidate"
        workspace.plot(tips[:, 0], tips[:, 1], color=color, linestyle="-" if accepted else "--", label=f"tip: {label}")
        for index in np.linspace(0, len(dense_path) - 1, min(13, len(dense_path)), dtype=int):
            points = arm_points(dense_path[index])
            workspace.plot(points[:, 0], points[:, 1], color=color, alpha=0.20, linewidth=2)
    for q, color, label in ((config.RRT2D_START_Q, "#2878b5", "start arm"), (config.RRT2D_GOAL_Q, "#e39a24", "goal arm")):
        points = arm_points(q)
        workspace.plot(points[:, 0], points[:, 1], "o-", color=color, linewidth=3, markersize=5, label=label)
    workspace.scatter(0, 0, color="black", s=35, zorder=5)
    workspace.set(xlim=(-0.25, 2.2), ylim=(-2.2, 2.2), xlabel="x (m)", ylabel="y (m)", title="Workspace: two obstacles and the narrow gap")
    workspace.set_aspect("equal")
    workspace.legend(loc="lower left", fontsize=8)

    extent = (JOINT_LIMITS[0, 0], JOINT_LIMITS[1, 0], JOINT_LIMITS[0, 1], JOINT_LIMITS[1, 1])
    jointspace.imshow(free, origin="lower", extent=extent, aspect="auto", cmap=matplotlib.colors.ListedColormap(["#e5a1a1", "#eef2f5"]), interpolation="nearest")
    if len(edges):
        jointspace.add_collection(LineCollection(tree[edges], colors="#78889a", alpha=0.5, linewidths=0.7))
    jointspace.scatter(tree[:, 0], tree[:, 1], color="#78889a", s=3, alpha=0.5)
    if len(path):
        jointspace.plot(path[:, 0], path[:, 1], "o-" if report.valid else "o--", color="#198754" if report.valid else "#b83030", linewidth=2, markersize=3, label="validated solution" if report.valid else "rejected candidate")
        if not report.valid:
            jointspace.scatter(*report.q, marker="X", color="black", s=85, zorder=6, label="first fine-check collision")
            points = arm_points(report.q)
            workspace.plot(points[:, 0], points[:, 1], "x-", color="black", linewidth=2, label="first collision")
            workspace.legend(loc="lower left", fontsize=8)
    jointspace.scatter(*config.RRT2D_START_Q, color="#2878b5", s=65, zorder=5, label="start")
    jointspace.scatter(*config.RRT2D_GOAL_Q, color="#e39a24", s=65, zorder=5, label="goal")
    jointspace.set(xlim=extent[:2], ylim=extent[2:], xlabel="joint 1 (rad)", ylabel="joint 2 (rad)", title="Full bounded configuration space: occupancy / tree / path")
    jointspace.legend(loc="upper left", fontsize=8)
    for axis in (workspace, jointspace):
        axis.grid(alpha=0.2)
    status = "No candidate found" if report is None else ("SUCCESS: independently validated solution" if report.valid else "COLLISION: candidate rejected by fine validation")
    figure.suptitle(f"{status}\nstep={config.RRT2D_STEP_SIZE:g}, bias={config.RRT2D_GOAL_BIAS:g}, edge spacing={config.RRT2D_VALIDATION_DISTANCE:g} rad")
    figure.tight_layout()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    figure.savefig(OUTPUT_DIR / "task1_rrt_tree.png", dpi=180)
    plt.close(figure)


def main() -> None:
    world = PlanarWorld()
    start_report = world.check_configuration(config.RRT2D_START_Q)
    goal_report = world.check_configuration(config.RRT2D_GOAL_Q)
    print("Start:", start_report.summary())
    print("Goal: ", goal_report.summary())
    if not start_report.valid or not goal_report.valid:
        raise RuntimeError("The configured planar start and goal must both be collision-free.")
    tree, edges, path, iterations = plan_rrt(world)
    print(f"RRT {'found a candidate' if len(path) else 'did not find a candidate'} after {iterations} samples; {len(tree)} tree vertices.")
    print(f"Step={config.RRT2D_STEP_SIZE:g} rad, goal bias={config.RRT2D_GOAL_BIAS:g}, edge spacing={config.RRT2D_VALIDATION_DISTANCE:g} rad")
    report = validate_path(world, path) if len(path) else None
    if report is not None:
        print(f"Independent validation ({FINE_VALIDATION_DISTANCE:g} rad): {'SUCCESS' if report.valid else 'COLLISION'}; {report.summary()}")
        if not report.valid:
            print("Rejected candidate is NOT a solution; first invalid q:", report.q)
    else:
        print("No solution path; inspect the explored tree and tune the search.")
    save_tree_plot(world, tree, edges, path, report)
    pd.DataFrame(tree, columns=["q1", "q2"]).assign(parent=np.r_[-1, edges[:, 0]] if len(edges) else -1).to_csv(OUTPUT_DIR / "task1_rrt_tree.csv", index=False)
    pd.DataFrame(path, columns=["q1", "q2"]).assign(fine_validated=bool(report is not None and report.valid)).to_csv(OUTPUT_DIR / "task1_rrt_path.csv", index=False)
    print("Saved task1_rrt_tree.png, task1_rrt_tree.csv and task1_rrt_path.csv in", OUTPUT_DIR)


if __name__ == "__main__":
    main()
