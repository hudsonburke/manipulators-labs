"""Values students may change while completing Lab 3."""

import numpy as np

# Task definition -----------------------------------------------------------------
# The target is in the default RTB URDF UR5 end-effector frame (wrist_3_link).
# Position is in meters; rotation is a 3x3 rotation matrix.
# Start and goal lie on opposite sides of the default world's y=0 aperture.
START_Q = np.deg2rad([-60.0, -90.0, -80.0, -100.0, 90.0, 0.0])
GOAL_POSITION = np.array([-0.334997093383395, -0.361931986127936, 0.499972497687035])
GOAL_ROTATION = np.array(
    [[0.0, np.sqrt(3) / 2, -0.5], [0.0, -0.5, -np.sqrt(3) / 2], [-1.0, 0.0, 0.0]]
)
IK_SEED = np.deg2rad([60.0, -90.0, -80.0, -100.0, 90.0, 0.0])

# Collision and planning -----------------------------------------------------------
SAFETY_MARGIN = 0.005  # minimum collision-geometry clearance, meters
PATH_VALIDATION_DISTANCE = 0.005  # maximum joint change between checks, radians
PLANNER_VALIDATION_DISTANCE = 0.30  # deliberately coarse; tune for the narrow aperture
MAX_CONNECTION_DISTANCE = 0.35  # maximum RRT extension, radians
PLANNING_TIMEOUT = 5.0  # seconds
PLANNER_SEED = 4

# Task 1 planar 2R RRT --------------------------------------------------------------
RRT2D_START_Q = np.array([-0.70, 2.00])  # folded below the gap
RRT2D_GOAL_Q = np.array([0.70, -2.00])  # folded above the gap
RRT2D_STEP_SIZE = 0.45  # maximum joint-space extension, radians
RRT2D_VALIDATION_DISTANCE = 0.45  # deliberately coarse; tune independently
RRT2D_GOAL_BIAS = 0.10  # probability of sampling the goal directly
RRT2D_MAX_ITERATIONS = 2500
RRT2D_SEED = 17

# Trajectory generation ------------------------------------------------------------
TRAJECTORY_DT = 0.02  # seconds
JOINT_SPEED_LIMITS = np.full(6, 0.60)  # radians/second

# Browser viewer -------------------------------------------------------------------
VIEWER_PORT = 52000
VIEWER_WEBSOCKET_PORT = 53000
