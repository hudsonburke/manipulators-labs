import os
import numpy as np
import spatialmath as sm
from roboticstoolbox.models.DH import UR5
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_control import RTDEControlInterface as RTDEControl
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

ur5 = UR5()

for goal in GOALS:
    se3 = sm.SE3.Rt(sm.SO3.RPY(goal[3:]), goal[:3])
    joint_q = ur5.ikine_LM(se3)
    controller.moveJ(joint_q, speed, acceleration)
    print("Target Pose:", goal)
    print("Actual TCP Pose:", receiver.getActualTCPPose())
    print("Target Joint Angles (rad):", joint_q)
    print("Actual Joint Angles (rad):", receiver.getActualQ())


controller.stopScript()
