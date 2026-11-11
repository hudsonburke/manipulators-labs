"""Edit these values while completing Lab 4; all robot angles are radians."""

import numpy as np

# Use the same simulated UR5 starting configuration as Lab 2.
HOME_Q = np.deg2rad([-90.0, -90.0, -90.0, -90.0, 90.0, 0.0])
ELBOW_GAIN = 0.5
# Optional extension after Task 3: also follow your upper arm with URSim joint 1.
CONTROL_SHOULDER = False
SHOULDER_GAIN = -0.5
JOINT_MIN = HOME_Q - np.array([0.0, 0.35, 0.35, 0.0, 0.0, 0.0])
JOINT_MAX = HOME_Q + np.array([0.0, 0.35, 0.35, 0.0, 0.0, 0.0])
MAX_JOINT_SPEED = 0.25  # radians/second
SMOOTHING_TIME_CONSTANT = 0.15  # seconds; use 0 to compare unfiltered targets

# Keep the control safeguards fixed for required tasks.
TARGET_MAX_AGE = 0.75  # seconds without a valid observation before disabling motion
MIN_LANDMARK_CONFIDENCE = 0.6
CONTROL_FREQUENCY = 125.0  # CB3 UR5; this is not a hard-real-time guarantee
