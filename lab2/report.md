# Lab 2 Report — Forward and Inverse Kinematics

**Name:**

## Task 1 — Cartesian trajectory (`moveL`)

1. Describe your trajectory. State the number of waypoints and the intended shape.

2. Did the TCP visit the requested points? Briefly describe the path between points.

## Task 2 — Numerical inverse kinematics

1. In your own words, distinguish forward kinematics from inverse kinematics. What are the input and output of each for the UR5?

2. Compare `task1_joint_q.csv` with `task2_joint_q.csv`. Are the joint vectors identical? If not, give a reason why different joint configurations can still be reasonable for the same requested TCP pose.

## Task 3 — Joint-space test (`moveJ`)

1. Compare the target and measured final TCP poses in `task3_results.csv`. Did the end effector reach each requested position and orientation closely enough? Identify the largest position discrepancy you observed and give a plausible explanation for it.

2. Did the robot follow the same *path* as in Task 1? Explain your observation using the difference between `moveL` (Cartesian-space interpolation) and `moveJ` (joint-space interpolation).

3. Suppose two different joint vectors produce the same TCP pose. What additional criterion might a real robot application use to choose between them? Give an example.

