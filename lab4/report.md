# Lab 4 Report — Vision-based URSim control

**Name:**

## Task 1 — Landmarks and angles

![Tracked left arm](outputs/task1_preview.jpg)

| Human pose | Observed elbow angle (deg) | Expected angle (deg) | Explanation of difference |
|---|---:|---:|---|
| Approximately straight | | | |
| Approximately right angle | | | |
| Bent | | | |

1. Explain how `arm_angles()` calculates the elbow and signed shoulder angles.
2. Why use pixel coordinates rather than unscaled normalized coordinates? What changes when your arm leaves the image plane?
3. What did the app display when a landmark became hidden or unreliable?

## Task 2 — Mapping and calibration

![Bounded target preview](outputs/task2_preview.jpg)

| Trial | Elbow gain | Neutral human angle (deg) | Current human angle (deg) | Robot elbow target (rad) | Observation |
|---|---:|---:|---:|---:|---|
| Initial | | | | | |
| Smaller gain | | | | | |
| Reversed sign | | | | | |

1. Explain the neutral offset, mapping direction, and target clipping. Which robot joint index is the elbow?
2. How did you confirm the other joints stayed fixed? What can joint bounds not guarantee?

## Task 3 — URSim control and stopping

![URSim control evidence](outputs/task3_preview.jpg)

| Trial | Shoulder enabled? | Smoothing time constant (s) | Jitter | Lag/responsiveness |
|---|---|---:|---|---|
| Elbow only | No | | | |
| Both joints, no smoothing | Yes | 0 | | |
| Both joints, smoothed | Yes | 0.15 | | |

1. Explain your final mapping and gains. Did actual joints follow the intended direction and stay near the targets?
2. Record the observed outcome for each: hidden wrist, Stop camera, Stop robot, and returning to a visible pose. Did control require explicit re-enabling?
3. Inspect `outputs/task3_control.csv`. Identify an interval with enabled motion and an interval after tracking loss; report target and actual elbow values from each. Explain blank entries and the difference between observation rate and servo rate.
4. Record typical displayed processing and round-trip times. Why do they not measure complete camera-to-robot latency?
5. What parts of this setup would need validation before any physical robot use?
