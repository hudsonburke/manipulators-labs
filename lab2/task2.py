"""Task 2: calculate and verify numerical inverse-kinematics solutions."""

import numpy as np
import pandas as pd
import spatialmath as sm
from roboticstoolbox.models.DH import UR5

from goals import GOALS, pose_to_se3, validate_goals

JOINT_COLUMNS = ["base", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
HOME_Q = np.array(
    [-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0.0]
)

# Change this temporarily for the required initial-guess exploration in the README.
# Restore HOME_Q before generating the joint angles used by Task 3.
INITIAL_GUESS = HOME_Q.copy()


def pose_errors(target, actual) -> tuple[float, float]:
    """Return position error (m) and orientation error (rad) between two SE3 poses."""
    position_error = np.linalg.norm(target.t - actual.t)
    orientation_difference = sm.SO3(target.R).inv() * sm.SO3(actual.R)
    orientation_error, _ = orientation_difference.angvec()
    return float(position_error), abs(float(orientation_error))


def main() -> None:
    validate_goals(GOALS)
    ur5 = UR5()
    print("UR5 model used by Robotics Toolbox:")
    print(ur5)

    joint_qs: list[np.ndarray] = []
    diagnostics: list[dict[str, float | int | str | bool]] = []
    seed_q = INITIAL_GUESS.copy()

    for index, goal in enumerate(GOALS, start=1):
        target = pose_to_se3(goal)
        solution = ur5.ikine_LM(target, q0=seed_q)
        if not solution.success:
            raise RuntimeError(
                f"IK failed at waypoint {index}: {solution.reason} "
                f"(residual {solution.residual})."
            )

        q = solution.q
        achieved = ur5.fkine(q)
        position_error, orientation_error = pose_errors(target, achieved)
        joint_qs.append(q)
        diagnostics.append(
            {
                "waypoint": index,
                "success": solution.success,
                "iterations": solution.iterations,
                "residual": solution.residual,
                "position_error_m": position_error,
                "orientation_error_rad": orientation_error,
            }
        )

        print(f"Waypoint {index}")
        print("  Target UR pose:         ", goal)
        print("  IK joint angles (rad):  ", q)
        print(f"  FK position error (m):   {position_error:.3e}")
        print(f"  FK orientation error (rad): {orientation_error:.3e}")

        # Seeding the next solve with this answer tends to keep adjacent poses on
        # the same IK branch. It does not make the solution unique.
        seed_q = q

    pd.DataFrame(joint_qs, columns=JOINT_COLUMNS).to_csv(
        "task2_joint_q.csv", index=False
    )
    pd.DataFrame(diagnostics).to_csv("task2_ik_diagnostics.csv", index=False)


if __name__ == "__main__":
    main()
