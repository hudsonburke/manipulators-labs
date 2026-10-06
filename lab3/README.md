# Lab 3 — Collision checking and motion planning

## Purpose

In Lab 2, you used forward and inverse kinematics to relate a UR5 joint
configuration \(q\) to a tool pose \(T\). In this lab, you will use a software
planning stack to answer the next question:

> How can the robot move from one valid configuration to another without
> colliding with its environment?

You will use:

- **Robotics Toolbox for Python** for the UR5 kinematic model, inverse
  kinematics, and timed joint trajectories;
- **MuJoCo** for the obstacle environment and collision checking;
- **OMPL** for RRT and RRT-Connect path planning; and
- **mjviser** for browser-based MuJoCo visualization.

The planning and simulator infrastructure is supplied. Your work is to use the
tools, vary meaningful parameters, inspect their outputs, and explain what you
observe.

By the end of the lab, you should be able to:

- distinguish a tool pose, joint configuration, geometric path, and timed
  trajectory;
- explain why valid start and goal configurations do not imply a valid motion;
- use collision checking to validate configurations and path segments;
- use and tune a sampling-based planner;
- generate and inspect joint position and velocity trajectories; and
- explain why a trajectory must be checked again after smoothing.

## Safety

Run all Lab 3 programs in the supplied simulation environment. A
collision-free simulated geometric path is **not** automatically safe to run on
a physical robot. Physical execution additionally requires verified robot and
tool geometry, coordinate transforms, safety margins, velocity and
acceleration limits, controller integration, and instructor review.

## Before you begin

From the repository root, install the locked dependencies:

```sh
uv sync
```

Run the tasks from the repository root:

```sh
uv run python lab3/task1_direct_motion.py
uv run python lab3/task2_path_planning.py
uv run python lab3/task3_trajectory.py
```

Each task starts a browser visualizer and continues running until you press
`Ctrl+C` in its terminal. To generate files without starting the visualizer,
append `--no-viewer`.

The viewer uses port `8080`. In Codespaces, open the forwarded port named
**Lab 3 MuJoCo viewer**. Keep that port private. When running locally, open
<http://localhost:8080>.

## The software pipeline

The lab passes data through several tools:

```text
desired tool pose
    ↓ inverse kinematics (Robotics Toolbox)
goal joint configuration
    ↓ state and edge validity (MuJoCo)
collision-aware path planning (OMPL)
    ↓ path waypoints
trajectory generation (Robotics Toolbox)
    ↓ final collision validation (MuJoCo)
browser playback (mjviser)
```

The supplied MuJoCo model uses the same standard-DH dimensions, joint order,
joint directions, base frame, and tool frame as
`roboticstoolbox.models.DH.UR5()`. Task 1 checks this agreement before planning.

## Background

### Configuration space

The UR5 state used in this lab is

$$
q = [q_1,q_2,q_3,q_4,q_5,q_6]^T.
$$

Its configuration space is six-dimensional. We do not explicitly construct
the complete collision region. Instead, the planner repeatedly asks:

```python
world.is_collision_free(q)
```

The planner must also determine whether the motion between two configurations
is valid. It approximates this by checking interpolated configurations at a
chosen resolution.

### Path versus trajectory

A **path** specifies configurations in order but does not say when the robot
reaches them:

$$
q_0 \rightarrow q_1 \rightarrow \cdots \rightarrow q_n.
$$

A **trajectory** supplies time, position, and velocity:

$$
q(t), \qquad \dot q(t).
$$

OMPL produces a path. Robotics Toolbox converts that path into a trajectory.

### Randomized planning

RRT and RRT-Connect use random samples. Different runs can produce different
valid paths. Important parameters include:

- `MAX_CONNECTION_DISTANCE`: maximum extension of the search tree;
- `PLANNER_VALIDATION_DISTANCE`: spacing used by OMPL's motion validator;
- `PLANNING_TIMEOUT`: maximum search time; and
- `PLANNER_SEED`: initial random-number seed for a repeatable complete script.

The final path is independently checked at the finer
`PATH_VALIDATION_DISTANCE`.

## Student configuration

Values intended for modification are collected in `student_config.py`.
Complete the supplied scene before changing them. The main groups are:

- start configuration and Cartesian goal;
- IK seed;
- collision and planning parameters;
- trajectory speed and blend settings; and
- viewer port.

Do not modify `lab3lib/` for the required tasks.

## Task 1 — IK and the direct trajectory

Run:

```sh
uv run python lab3/task1_direct_motion.py
```

The script:

1. creates the Robotics Toolbox UR5;
2. loads the matching MuJoCo scene;
3. solves IK for the supplied target pose;
4. checks the resulting goal configuration;
5. creates a direct `jtraj` from start to goal;
6. checks the entire direct trajectory for collision; and
7. generates a two-joint configuration-space slice.

In the viewer:

1. pause the direct trajectory;
2. use the frame slider to find the first collision;
3. identify the colliding robot link and obstacle; and
4. enable or disable contact visualization in the **Visualization** tab.

The configuration-space figure holds four joints fixed and samples the base
and elbow joints. It is a two-dimensional **slice**, not the robot's complete
configuration space.

Generated evidence:

```text
outputs/task1_goal_q.csv
outputs/task1_direct_trajectory.csv
outputs/task1_configuration_slice.png
```

## Task 2 — Plan around the obstacle

Run:

```sh
uv run python lab3/task2_path_planning.py
```

The script calls the supplied OMPL `RRTConnect` wrapper. It saves both the raw
planner path and OMPL's simplified path, then checks the simplified result
again using MuJoCo.

The browser menu contains:

- **Direct trajectory** — expected to collide;
- **Raw planned path** — the first path returned by RRT-Connect; and
- **Simplified path** — collision-checked shortcuts through the raw path.

The tree plot shows a two-joint projection of a six-dimensional planning
problem. Points that overlap in this projection may differ in the other four
joints.

The script also repeats planning for three connection distances. Use the CSV
and plot to compare:

- success rate;
- median planning time;
- median number of validity checks;
- waypoint count; and
- normalized path length.

Generated evidence:

```text
outputs/task2_raw_path.csv
outputs/task2_simplified_path.csv
outputs/task2_planner_projection.png
outputs/task2_parameter_results.csv
outputs/task2_parameter_results.png
```

## Task 3 — Generate timed trajectories

Run Task 2 first, then:

```sh
uv run python lab3/task3_trajectory.py
```

The script creates two trajectories through the simplified path:

1. **Stop at waypoints** — concatenated `jtraj` segments. The robot follows
   each collision-checked straight joint-space edge and stops at every
   waypoint.
2. **Blended `mstraj`** — a shorter, smoother multi-segment trajectory with
   polynomial blends.

`mstraj` can round a corner instead of exactly reaching a via point. This can
move the trajectory away from the collision-checked path, so the script checks
both final trajectories again.

The supplied wrapper restores `mstraj`'s omitted initial sample. This call uses
`mstraj`'s default zero endpoint velocities, so the blended motion starts and
ends at rest. If the sampled blend exceeds `JOINT_SPEED_LIMITS`, the wrapper
stretches its time axis until the measured joint velocities satisfy those
limits.

Inspect the generated plots and viewer playback. Compare:

- total duration;
- whether the robot stops at intermediate waypoints;
- position and velocity smoothness; and
- final collision status.

Generated evidence:

```text
outputs/task3_stop_at_waypoints.csv
outputs/task3_blended_trajectory.csv
outputs/task3_stop_at_waypoints.png
outputs/task3_blended_trajectory.png
```

## Small design extension

After completing the supplied problem, make **one** controlled change in
`student_config.py`:

- change the Cartesian goal position by a small amount;
- try a substantially different IK seed;
- change the planner connection distance;
- add a nonzero safety margin; or
- change trajectory speed or blend time.

Run the affected tasks again. If planning fails or the smoothed trajectory
collides, that is a valid result when it is reported and explained. Restore
the supplied configuration before creating the required baseline evidence.

## Deliverables

Submit:

- `student_config.py`;
- `report.md`;
- the generated CSV files;
- the generated PNG figures; and
- any additional figure from your design extension.

Do not submit installed packages, MuJoCo caches, or a running viewer process.

## Tool references

- [Robotics Toolbox for Python](https://petercorke.github.io/robotics-toolbox-python/)
- [Robotics Toolbox trajectory functions](https://petercorke.github.io/robotics-toolbox-python/arm_trajectory.html)
- [MuJoCo Python bindings](https://mujoco.readthedocs.io/en/stable/python.html)
- [OMPL](https://ompl.kavrakilab.org/)
- [mjviser](https://github.com/mujocolab/mjviser)
