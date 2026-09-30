import os
import numpy as np
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from goals import GOALS


URSIM_HOST = os.environ.get("URSIM_HOST", "localhost")
controller = RTDEControl(URSIM_HOST)
receiver = RTDEReceive(URSIM_HOST)

speed = 0.25  # rad/s
acceleration = 0.5  # rad/s^2

# Move to a safe home position using moveJ (joint space)
# Arguments: joint positions [rad], speed [rad/s], acceleration [rad/s^2]
home_q = [-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0.0]
controller.moveJ(home_q, 0.5, 0.5)


joint_qs = []
for goal in GOALS:
    controller.moveL(goal, speed, acceleration)
    # Get the current joint positions [rad]
    joint_q = receiver.getActualQ()
    joint_qs.append(joint_q)
    print("Joint positions (rad):", joint_q)

# Maybe output csv?
print(joint_qs)

controller.stopScript()
