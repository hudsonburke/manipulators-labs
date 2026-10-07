# Lab 3 — Collision checking and motion planning

## Purpose

In Lab 2, you used forward and inverse kinematics to relate a UR5 joint
configuration \(q\) to a tool pose \(T\). In this lab, you will use a software
planning stack to answer the next question:

> How can the robot move from one valid configuration to another without
> colliding with its environment?

You will use:

- **Robotics Toolbox for Python** for one URDF UR5 model shared by inverse
  kinematics, collision checking, and timed joint trajectories;
- **OMPL (Open Motion Planning Library)** for RRT (Rapidly-exploring Random Tree) and RRT-Connect path planning; and
- **Swift** for browser-based playback of that same robot and scene.

The planning and visualization infrastructure is supplied. Your work is to use the
tools, vary meaningful parameters, inspect their outputs, and explain what you
observe.

By the end of the lab, you should be able to:

- explain how RRT sampling, nearest-vertex selection, bounded extension, and
  collision checks grow a tree toward a goal;
- distinguish a planner's edge-validation resolution from the finer independent
  path check;
- explain why valid start and goal configurations do not imply a valid motion;
- tune a sampling-based planner and justify the chosen resolution;
- generate a joint-position-versus-time graph and inspect a timed trajectory.

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
uv run python lab3/task1_rrt_demo.py
uv run python lab3/task2_path_planning.py
```

Task 1 saves plots and CSV files, then exits; it does not use Swift.
Task 2 opens Swift and continues running until you press `Ctrl+C` in
its terminal. Append `--no-viewer` for batch generation.
Task 2 opens Swift even when planning fails or fine validation rejects a path.

The supplied Swift playback panel has:

- **Motion**: choose one of the task's paths or trajectories;
- **Frame**: select a zero-based sample index;
- **Pause/Play**: stop or resume playback;
- **Speed**: choose `0.25x`, `0.5x`, `1x`, `2x`, or `4x`.

Playback stops and holds at the trajectory's endpoint; it does not loop.

The status text shows the current time/frame and configuration report,
including collision pairs and the clearance threshold. These reports describe
the displayed sample; the terminal path report also checks interpolated edges.


## The software pipeline

The lab passes data through several tools:

```text
desired tool pose
    ↓ inverse kinematics (Robotics Toolbox)
goal joint configuration
    ↓ state and edge validity (spatialgeometry collision geometry)
collision-aware path planning (OMPL)
    ↓ path waypoints
trajectory generation (Robotics Toolbox)
    ↓ final collision validation (supplied CollisionWorld helper)
browser playback (Swift)
```

`make_robot()` constructs `roboticstoolbox.models.URDF.UR5()`. The same robot
instance is passed to `CollisionWorld` for collision checks and Swift playback.
Its joint limits, link transforms, visual meshes, and collision geometry come
from that URDF model. 

The target pose uses the URDF robot's default Robotics Toolbox end-effector
frame, `wrist_3_link`. It is not the physical UR controller's configured TCP.
Forward and inverse kinematics use this same default frame throughout the lab.

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

The supplied `CollisionWorld` helper checks joint limits, robot–obstacle
collisions, and explicitly checks nonadjacent robot link pairs for
self-collision. Adjacent links are excluded from the self checks because their
geometry meets at a joint. The supplied `SAFETY_MARGIN` requires 5 mm clearance.

The planner must also determine whether the motion between two configurations
is valid. It approximates this by checking interpolated configurations at a
chosen resolution. Sampling is not continuous collision detection: narrow
collisions can fall between samples. The final validation uses finer spacing
than planning, and timed trajectories are checked again at their own samples.

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
- `PLANNER_SEED`: initial random seed for OMPL; time-limited runs can still vary
  with execution timing.

The final path is independently checked at the finer
`PATH_VALIDATION_DISTANCE`.

## Student configuration

`student_config.py` collects the RRT and trajectory parameters. Task 1 uses
`RRT2D_STEP_SIZE`, `RRT2D_GOAL_BIAS`, `RRT2D_VALIDATION_DISTANCE`,
`RRT2D_MAX_ITERATIONS`, and `RRT2D_SEED`. Task 2 starts with deliberately coarse
`PLANNER_VALIDATION_DISTANCE`; start by tuning it alone, then change another
parameter if needed. Do not modify `lab3lib/` for required tasks.


## Task 1 — Tune RRT for a planar 2R manipulator

Run:

```sh
uv run python lab3/task1_rrt_demo.py
```

The two-joint planar arm must pass through a narrow gap between two workspace
obstacles. Use this small problem to build intuition before the UR5 task:
RRT samples joint configurations, selects the nearest existing vertex, and
extends by at most `RRT2D_STEP_SIZE`. Candidate edges are checked at
`RRT2D_VALIDATION_DISTANCE`; `RRT2D_GOAL_BIAS` controls goal-directed sampling.

The opening is 0.32 m tall; the arm links have 0.045 m radius.
Fixed joint bounds force the elbow-branch transition through the gap instead
of allowing a detour around a wall end. Keep the geometry and joint bounds
fixed while tuning. The independent Task 1 check uses 0.002 rad spacing.

Change one parameter at a time and find settings that produce a path passing
the independent fine check. Inspect `outputs/task1_rrt_tree.png`: one panel
shows the physical gap and arm configurations; the other shows the full
two-joint configuration space, explored tree, and candidate path. A coarse
edge check can incorrectly accept an edge crossing an obstacle; that is not
a successful solve. Record settings, search iterations, fine-check status,
and what changed in the plots. Task 1 is plot-only, including failed runs.

Generated evidence:

```text
outputs/task1_rrt_tree.png
outputs/task1_rrt_tree.csv
outputs/task1_rrt_path.csv
```

## Task 2 — Apply RRT tuning to a UR5 narrow gap

Run:

```sh
uv run python lab3/task2_path_planning.py
```

Apply the Task 1 lessons to a six-joint UR5 passing through a physical narrow
gap. OMPL `RRTConnect` uses `MAX_CONNECTION_DISTANCE`,
`PLANNER_VALIDATION_DISTANCE`, `PLANNING_TIMEOUT`, and `PLANNER_SEED`.
The UR5 starts and ends on opposite sides of a wall at `y = 0`; the lower and
upper wall sections leave a 0.18 m-high aperture. The starter edge spacing
is deliberately coarse (`0.30` rad).
Start by changing only validation spacing, then adjust another parameter if
needed. Explain the tradeoff between search effort and collision detection.
Do not change the obstacles, safety margin, or independent fine-check spacing
to manufacture a successful result.

The returned candidate is independently checked at
`PATH_VALIDATION_DISTANCE`, then converted to a timed stop-at-waypoints
trajectory. Each trajectory interval is also checked at that fine spacing.
Swift plays the timed trajectory, stopping and holding at the **first
colliding configuration** if one is found. No subsequent samples are played.

When RRT finds no complete path, the program reconstructs the start-connected
tree branch whose endpoint is closest to the goal, measured by Euclidean
joint distance normalized by joint ranges. It replays that branch and then
attempts a straight joint-space extension to the goal. This extension is
clearly labeled **UNPLANNED**: it explains where an unsuccessful attempt hits
an obstacle, not a planner-generated solution. Even if the diagnostic
extension reaches the goal, it is not reported as an RRT solve.

The projection plot shows only two of the six joints; overlapping points
can differ in the other four. The viewer reports the current configuration's
collision pairs and clearance threshold. Compare these observations with the
terminal fine-validation result, then tune and rerun until the planned path
and trajectory both pass.

Generated evidence:

```text
outputs/task2_raw_path.csv
outputs/task2_simplified_path.csv
outputs/task2_closest_partial_path.csv
outputs/task2_planner_projection.png
outputs/task2_attempt_trajectory.csv
outputs/task2_joint_position_over_time.png
```

The attempt CSV includes time, six joint positions, and a collision flag;
only its final row can be colliding. The required joint-position-over-time graph
plots those same six joint positions against the same time samples used by Swift.
For failed attempts, the graph ends at the first collision; for successful
attempts, it ends at the goal. Its title states which outcome occurred.

Each run overwrites its task's figures and CSVs. Existing outputs may reflect
different parameters from the current starter configuration; use the printed
validation result rather than treating a saved candidate as a successful solve.
Regenerate all evidence after changing your parameters.

## Optional parameter exploration

After completing Task 2, test one additional RRT parameter, such as
`MAX_CONNECTION_DISTANCE`, `PLANNING_TIMEOUT`, or `PLANNER_SEED`. Change only
that value while keeping the validated resolution fixed. Compare the observed
planning time, state-check count, path length, and success; randomized planner
results vary between runs, so do not claim one trial establishes a universal
best setting.

## Deliverables

After running both tasks with your final settings and completing the report,
run the provided submission script from the `lab3` directory:

```sh
cd lab3
./submit.sh
```

The script checks that all required files exist and packages them into
`lab3/submission.zip`, preserving the `outputs/` directory. It does not rerun
the tasks or verify that the saved outputs match your current parameters.
Right-click `submission.zip` in the file explorer and choose **Download**.
Upload the ZIP to the **Lab 3 Assignment on Canvas**.

## Tool references

- [Robotics Toolbox for Python](https://petercorke.github.io/robotics-toolbox-python/)
- [Robotics Toolbox trajectory functions](https://petercorke.github.io/robotics-toolbox-python/arm_trajectory.html)
- [spatialgeometry](https://github.com/petercorke/spatialgeometry)
- [OMPL](https://ompl.kavrakilab.org/)
- [Swift simulator](https://github.com/petercorke/swift)
