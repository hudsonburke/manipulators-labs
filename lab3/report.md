# Lab 3 Report — Collision Checking and Motion Planning

**Name:**

## Task 1 — IK and direct motion

1. Report the goal joint configuration returned by inverse kinematics. Was it collision-free as a single configuration?

2. Was the direct `jtraj` collision-free? Identify the first colliding robot link and environment object reported by the program.

3. Explain why checking only the start and goal configurations is insufficient.

4. Insert `task1_configuration_slice.png`. Which quantities vary in this figure, and which four joint values are held fixed? Why is this not the complete UR5 configuration space?

## Task 2 — Sampling-based path planning

1. Insert `task2_planner_projection.png`. Explain what the tree points and two plotted paths represent. Why can this two-joint projection hide differences between configurations?

2. Compare the raw and simplified path waypoint counts and normalized lengths. How can simplification remove waypoints without introducing a collision?

3. Insert `task2_parameter_results.png` and complete the table.

| Maximum connection distance | Success rate | Median planning time (s) | Median state checks |
|---:|---:|---:|---:|
| 0.15 | | | |
| 0.35 | | | |
| 0.70 | | | |

4. Describe one tradeoff you observed when changing the maximum connection distance. Do not claim that one short experiment proves the parameter is universally better.

5. Why does the program independently validate the returned path instead of trusting the planner's success status alone?

## Task 3 — Timed trajectories

1. Insert `task3_stop_at_waypoints.png` and `task3_blended_trajectory.png`.

2. Report the duration and final collision status of each trajectory.

| Trajectory | Duration (s) | Collision-free? |
|---|---:|:---:|
| Stop at waypoints | | |
| Blended `mstraj` | | |

3. Compare the joint-position and joint-velocity curves. What changes at the intermediate waypoints?

4. Distinguish a geometric path from a timed trajectory. Which tool generated each one in this lab?

5. Why can trajectory blending invalidate a previously valid geometric path?

## Design extension

1. State the single value you changed and why you selected it.

2. Describe the resulting motion and include one quantitative result or generated figure.

3. Did the modified pipeline succeed? If not, identify the stage that failed—IK, endpoint validation, planning, trajectory generation, or final validation—and explain what you would try next.

## Toolchain reflection

For each tool, state its role and one output that another component consumes.

| Tool | Role in this lab | Output passed to another component |
|---|---|---|
| Robotics Toolbox for Python | | |
| MuJoCo | | |
| OMPL | | |
| mjviser | | |
