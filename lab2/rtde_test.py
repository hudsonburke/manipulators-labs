import numpy as np
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import os
URSIM_HOST = os.getenv("URSIM_HOST", "localhost")
rtde_c = RTDEControl(URSIM_HOST)
rtde_r = RTDEReceive(URSIM_HOST)

# Move to a safe home position using moveJ (joint space)
# Arguments: joint positions [rad], speed [rad/s], acceleration [rad/s^2]
home_q = [-np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, 0.0]
rtde_c.moveJ(home_q, 0.5, 0.5)

# Read the TCP pose after arriving at home so moveL targets are always valid
# Pose: [x, y, z, rx, ry, rz] in meters and axis-angle [rad]
tcp_pose = rtde_r.getActualTCPPose()
print("TCP pose at home:", tcp_pose[:3])

# Move 15 cm down in Z using moveL (Cartesian linear move)
# Arguments: pose, speed [m/s], acceleration [m/s^2]
pose_down = tcp_pose[:]
pose_down[2] -= 0.15
rtde_c.moveL(pose_down, 0.25, 0.5)

# Move 10 cm in X
pose_side = pose_down[:]
pose_side[0] += 0.10
rtde_c.moveL(pose_side, 0.25, 0.5)

# Return to home TCP pose
rtde_c.moveL(tcp_pose, 0.25, 0.5)

# Return to home joint configuration
rtde_c.moveJ(home_q, 0.5, 0.5)

rtde_c.stopScript()