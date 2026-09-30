import numpy as np
from roboticstoolbox.models.DH import UR5
import spatialmath as sm
from goals import GOALS

ur5 = UR5()
# Display DH parameters and defined poses
print(ur5)

for goal in GOALS:
    se3 = sm.SE3.Rt(sm.SO3.RPY(goal[3:]), goal[:3])
    joint_angles = ur5.ikine_LM(se3).q
    print(f"Target Pose: {goal}")
    print(f"Joint Angles: {joint_angles}")
