# Lab 4 Report — Vision-based URSim control

**Name:**

## Task 1 — Arm landmarks (`task1_landmarks.py`)

![Tracked left arm](outputs/task1_preview.jpg)

| Landmark | MediaPipe index | Why that index |
|---|---:|---|
| Left shoulder | | |
| Left elbow | | |
| Left wrist | | |


## Task 2 — Elbow joint angle (`task2_elbow_angle.py`)

![Elbow angle](outputs/task2_preview.jpg)

| Human pose | Expected angle | Observed (deg) | Observed (rad) | Explanation of difference |
|---|---:|---:|---:|---|
| Arm straight | ≈ 180° | | | |
| Forearm at a right angle | ≈ 90° | | | |
| Elbow sharply bent | ≈ 45–60° | | | |


1. Explain your formula and why the result is always within `[0, π]`.
2. What happens to the measured angle when your arm leaves the image plane (points toward or away from the camera)? Test it, then explain why a 2D measurement behaves this way.

## Task 3 — URSim follows the elbow angle (`task3_control.py`)

![URSim control evidence](outputs/task3_preview.jpg)

| Trial | Elbow gain | Neutral human angle (deg) | Current human angle (deg) | Robot elbow target (rad) | Observation |
|---|---:|---:|---:|---:|---|
| Initial | | | | | |
| Smaller gain | | | | | |
| Reversed sign | | | | | |

| Trial | Smoothing time constant (s) | Jitter | Lag/responsiveness |
|---|---:|---|---|
| No smoothing | 0 | | |
| Smoothed | 0.15 | | |

1. Explain the neutral offset, mapping direction, and target clipping. Which robot joint index is the elbow?
2. What parts of this simulation-only setup would need validation before any physical robot use?

## Optional extension — shoulder joint

| Shoulder gain | Observed behaviour | Why that sign |
|---|---:|---|
