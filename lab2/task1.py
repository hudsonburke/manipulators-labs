import os
import numpy as np
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

URSIM_HOST = os.environ.get("URSIM_HOST", "localhost")
controller = RTDEControl(URSIM_HOST)
receiver = RTDEReceive(URSIM_HOST)

speed = 0.25  # rad/s
acceleration = 0.5  # rad/s^2

# Move to a safe home position using moveJ (joint space)
# Arguments: joint positions [rad], speed [rad/s], acceleration [rad/s^2]
home_q = [-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0.0]
controller.moveJ(home_q, 0.5, 0.5)

## EDIT HERE: Define the points for the robot to move to ##
# Pose: [x, y, z, rx, ry, rz] in meters and axis-angle [rad]
waypoints = [
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
]

for point in waypoints:
    controller.moveL(point, speed, acceleration)

controller.stopScript()
