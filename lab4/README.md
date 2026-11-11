# Lab 4 — Control URSim with your arm

## Purpose

Use OpenCV and MediaPipe to find your arm in a camera image, compute your elbow
angle, and move the simulated UR5 with it. The robot follows one joint from a
calibrated neutral pose; this is two-joint gesture control, not human pose
reconstruction or motion planning.

## Safety and privacy

**Students: simulation only. Do not connect this program to a physical robot.**
Joint bounds and rate limits do not make motion physically safe, and the
keyboard stop is not an emergency stop. The only hardware use is the instructor
demo described under Setup, which selects a real controller with `--host`.

Python reads your camera directly. Images stay on your computer, are not
uploaded, and are never recorded automatically. Pressing the current task's
number key saves one annotated image; Task 3 also saves a CSV. Check the
background of anything you save before submitting. Quitting releases the camera.

## What you run

Start the script for the task you are working on. Each opens one camera window;
only Task 3 connects a robot.

| File | Purpose | Edit? |
|---|---|---|
| `task1_landmarks.py` | Task 1: landmarks in the preview | run |
| `task2_elbow_angle.py` | Task 2: elbow angle in the preview | run |
| `task3_control.py` | Task 3: calibrate and drive URSim (or a UR controller with `--host`) | run |
| `student.py` | `elbow_angle()` — your Task 2 implementation | **yes** |
| `student_config.py` | gains, bounds, smoothing, shoulder switch | **yes** |
| `report.md` | report template | **yes** |
| `submit.py` | packages the submission | no |
| `lab4lib/` | supplied helpers: `camera`, `vision`, `controller`, `session`, `overlay`, `evidence` | no |
| `compose.yaml` | URSim container for Task 3 | no |

## Setup

Use **VS Code on your computer**, with the repository open as a normal folder —
not a Dev Container or Codespace — and run the camera from a native terminal (on
Windows, PowerShell rather than WSL). Install
[uv](https://docs.astral.sh/uv/getting-started/installation/).

From the repository root:

```sh
cd lab4
uv python install 3.12
uv sync --locked
uv run python task1_landmarks.py
```

uv supplies Python 3.12; do not run this lab from the repository-root
environment. If the VS Code Python extension asks for an interpreter, pick
`lab4/.venv/`.

`--camera 0` is the default device; add `--camera 1` (or another index) if the
wrong camera opens. Allow camera access for your terminal in the operating
system's privacy settings, and close other apps using the camera. The preview is
**not mirrored**: MediaPipe's *left* is your anatomical left.

The first run downloads the MediaPipe Pose Landmarker Lite model (about 6 MB)
into `models/`; later runs use the cache. Delete that file if the app reports it
is corrupt. The model is not submitted.

### URSim for Task 3

If you don't have Docker installed, follow the instructions at:

- Mac: <https://docs.docker.com/desktop/setup/install/mac-install/>
- Windows: <https://docs.docker.com/desktop/setup/install/windows-install/>

Docker runs **only URSim**. From `lab4`:

```sh
docker compose up -d
```

Open <http://localhost:6080/vnc.html>, accept the safety prompts, power on,
release the brakes, and open PolyScope's Move tab. `task3_control.py` connects to
`localhost` by default; `--host <name or IP>` selects another controller and
`URSIM_HOST` changes the default, for example `--host 192.168.1.12`. Pass a host
name or IP address, not a URL, and run only one controller against the robot. The
simulator and control ports are bound to localhost — leave them private. The
image is `linux/amd64`, so Apple Silicon depends on Docker emulation, which has
**not been verified** for this lab.

When finished:

```sh
docker compose down
```

If Docker reports a port is already allocated, stop the Labs 2–3 devcontainer
stack first; both publish the same simulator ports.

## Task 1 — Get the arm landmarks with MediaPipe

Run `uv run python task1_landmarks.py`.

1. No robot is connected for this task.
2. MediaPipe Pose returns 33 body landmarks in normalized `[0, 1]` image
   coordinates. The supplied `lab4lib/vision.py` selects the anatomical left
   shoulder, elbow, and wrist — indices 11, 13, and 15 in
   `lab4lib.vision.LEFT_ARM` — and scales them to pixels using the frame size.
3. Check that the three labelled points and connecting lines follow your left
   arm. Record those landmark indices; Task 2 uses the same three points.
4. Exercise the failure cases and record the overlay message for each: no person
   in view, a low-visibility landmark (turn away or leave the frame), an arm
   partly outside the image, and two landmarks collapsing onto each other. The
   app must report the problem instead of showing invented coordinates.
5. Press **1** with a clear tracked pose.

Generated file: `outputs/task1_preview.jpg`.

## Task 2 — Calculate the elbow joint angle

Run `uv run python task2_elbow_angle.py` after editing `student.py`.

1. `student.elbow_angle(shoulder, elbow, wrist)` is yours to implement; the
   supplied version raises `NotImplementedError`. Until it works, the overlay
   shows the landmarks plus `Landmarks tracked. Implement elbow_angle()…`, and
   Task 3 cannot move the robot.
2. Return the interior angle at the elbow in radians, within `[0, π]`; the
   overlay converts it to degrees for display. Each argument is one pixel point
   — an `(x, y)` tuple, list, or numpy row. `arm_points()` validates the input
   and returns the three pixel coordinates; use those, not the normalized
   landmarks, because normalized coordinates distort angles unless the image is
   square.
3. Measure at the **elbow**, between the two segments pointing away from it:

   ```
   a = shoulder - elbow        b = wrist - elbow
   ```

   A straight arm is π (180°); a right angle is π/2 (90°). The result does not
   depend on how close you stand to the camera — only on the arm's shape.
4. Either formula works:
   - cosine rule: `angle = arccos((a·b) / (|a|·|b|))`, clipping the cosine to
     `[-1, 1]` first so rounding cannot turn a straight arm into NaN;
   - `atan2` form: `angle = atan2(abs(a_x*b_y - a_y*b_x), a·b)`, which needs no
     clipping and stays accurate for very small or very large vectors. The
     **absolute** cross product is what keeps the result in `[0, π]`; without
     it the sign flips whenever you bend the other way.
5. Check the math before pointing the camera at yourself:

   ```sh
   uv run python -c "import math; from student import elbow_angle as e; print(round(math.degrees(e((0,0),(0,1),(1,1)))))"
   ```

   This should print `90.0` (right angle). Try `((0,0),(1,0),(2,0))` for a
   straight arm (`180.0`) and `((0,0),(1,0),(2,1))` for a 45° bend (`135.0`).
6. Verify against the poses below, then press **2** with a clearly tracked pose.

Common mistakes: using the normalized landmark values instead of pixels; calling
`arccos` without clipping and getting `nan`; using a signed cross product and
getting negative angles; measuring at the shoulder, or using
`wrist - shoulder`; returning degrees when the caller expects radians.

Generated file: `outputs/task2_preview.jpg`.

## Task 3 — Move the UR5 using that angle

Run `uv run python task3_control.py` with URSim started as described above.

1. `student.joint_targets()` maps the change from a calibrated neutral pose onto
   bounded UR5 joints. Joint 2 is the elbow: home elbow angle plus `ELBOW_GAIN`
   times the change in your elbow angle, clipped to `JOINT_MIN`/`JOINT_MAX`. All
   other joints stay at `HOME_Q`.
2. Hold a comfortable pose and press **C** while stopped; that pose maps to
   `HOME_Q`. Move your elbow without enabling URSim and confirm the six target
   values respond. Change only `ELBOW_GAIN` (smaller magnitude, then the
   opposite sign), restarting and recalibrating each time. Explain the direction,
   sensitivity, and clipping; use a positive gain for your first live test.
3. Press **I** once to move explicitly to `HOME_Q` at limited speed. **Space** or
   **S** cancels initialization. Then press **C**, then **E**, and move slowly
   while comparing target and actual joint positions with PolyScope.
4. Verify fail-closed behavior: hide your wrist. The controller must disable, and
   must not resume when tracking returns; press **E** again explicitly. Also test
   **Space** or **S**, and close the preview to check cleanup. A camera read
   failure must likewise stop control; if it is safe to disconnect your camera,
   observe it, otherwise state in the report that this case was not exercised.
   Restart, initialize, and recalibrate after quitting or a camera failure.
5. Compare `SMOOTHING_TIME_CONSTANT = 0` with the supplied `0.15` seconds, gains
   unchanged. Explain the jitter/lag tradeoff; filtering does not replace the
   motion-rate limit.
6. Run a short trial with enabled motion and a tracking-loss stop, then press
   **3** before quitting.

Generated files: `outputs/task3_preview.jpg` and `outputs/task3_control.csv`.
The CSV contains `time_s`, `tracking`, `enabled`, `human_elbow_rad`,
`human_shoulder_rad`, `target_q1`–`target_q6`, and `actual_q1`–`actual_q6`. Time
is elapsed monotonic time since the app started; angles are radians. Blank
entries mean unavailable data, not zero. The file holds the latest 6000
observations since start, **not** every 125 Hz servo command. Restarting clears
the log, and saving again overwrites the same files.

### Optional extension — also follow your upper arm

After Task 3 works, set `CONTROL_SHOULDER = True` and choose `SHOULDER_GAIN`.
Joint 1 then follows the supplied `student.shoulder_angle()`. Restart,
initialize, and recalibrate, then test slow elbow and shoulder motion with the
other four joints fixed.

## Controls

Keys work only while the OpenCV window has focus. Switching windows does not stop
camera capture or motion, so press **Space** or **S** before switching away.

| Script | Keys |
|---|---|
| `task1_landmarks.py` | **1** save Task 1 evidence · **Q**/**Esc** quit |
| `task2_elbow_angle.py` | **2** save Task 2 evidence · **Q**/**Esc** quit |
| `task3_control.py` | **I** move to HOME · **C** calibrate · **E** enable · **Space**/**S** stop · **3** save Task 3 evidence · **Q**/**Esc** quit |

Closing the preview, a camera read failure, Ctrl+C, and exceptions also disable
control and release the camera, tracker, and controller. Missing or stale pose
data disables motion. A visible pose returning **never automatically re-enables**
control.

## Supplied control behavior

The main thread captures, processes, and displays frames with OpenCV. Task 3's
overlay shows tracking, human angle, targets, actual/controller status,
processing time, key help, and action errors. Displayed processing time is not
complete camera-to-robot latency: capture buffering, display, scheduling,
communication, and robot response also contribute. Frames are capped at
640 x 480; camera buffer size is requested but backends may ignore it, so check
latency rather than assuming the read timestamp is the exposure time.

A separate worker owns the RTDE connection, uses the latest target, smooths
changes, and limits commanded joint speed. The CB3 UR5 control loop targets
125 Hz independently of the camera loop, and a normal desktop OS is **not**
hard-real-time. Loss of confidence, invalid input, or no valid observation
within `TARGET_MAX_AGE` disables control; the worker watchdog can stop motion
even if camera processing stalls. Enabling always requires fresh tracking and an
explicit key press.

The fixed bounds and starting pose suit the supplied empty URSim scene. This lab
does not perform Lab 3 collision checking.

## Submission

Fill in `report.md`, save evidence for all three tasks, then run from `lab4`:

```sh
uv run python submit.py
```

This works in PowerShell as well as macOS/Linux shells. It packages exactly
`student.py`, `student_config.py`, `report.md`, the three images, and
`outputs/task3_control.csv` into `submission.zip`, keeping the `outputs/` paths.
Missing files are listed before an existing archive is touched. Upload the ZIP
to the Lab 4 Canvas assignment. The script does not verify report quality or
that the CSV contains a successful trial, and the model cache is not included.

## Troubleshooting

- **No camera / wrong camera:** check OS camera permissions for your terminal,
  close other camera apps, and try another `--camera` index.
- **No preview window:** use a desktop terminal — not WSL, Codespaces, SSH
  without a desktop, or a Dev Container. Use the Lab 4 environment (GUI OpenCV),
  not a headless OpenCV package.
- **Keys do nothing:** focus the OpenCV window. Focus changes do not stop the
  camera or robot; use PolyScope if you cannot reach the keyboard stop.
- **`NotImplementedError` in Task 2:** implement `student.elbow_angle()`. Task 3
  stays blocked until it works.
- **Model download/import failure:** read the terminal, confirm
  `uv sync --locked` finished in `lab4`, and check internet access.
- **Initialize fails:** make sure Docker and URSim are running with the robot
  powered on and brakes released, stop other Lab 2/4 scripts, and check
  `URSIM_HOST`. Preview and mapping still work without URSim.
- **Tracking keeps disabling:** improve lighting and landmark visibility, check
  processing time and host load. Do not raise the stale timeout to hide a stall.
- **App unresponsive:** do not assume the robot stopped. Check PolyScope, stop
  there if needed, then reinitialize and re-enable after restarting.

## References

- [MediaPipe Pose Landmarker](https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker)
- [OpenCV Python tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [ur_rtde control API](https://sdurobotics.gitlab.io/ur_rtde/pages/reference/api.html)
