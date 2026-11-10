# Lab 4 — Control URSim with your arm

## Purpose

Use OpenCV and MediaPipe to measure a human arm pose, map that measurement to
joint targets, and control the simulated UR5. This is two-joint gesture control,
not reconstruction of a complete human pose or a collision-free motion planner.

By the end, you should be able to explain image-plane angle measurements,
neutral calibration, mapping direction and scale, joint bounds, filtering, and
why the camera frame rate must not set the robot control rate.

## Safety and privacy

**Use only the supplied URSim simulation. Do not connect this program to a
physical robot.** Joint bounds and rate limits do not establish collision-free
or physically safe motion. The browser Stop button is not a hardware emergency
stop. Keep PolyScope accessible and use its stop controls if needed.

The browser requests video only, not audio. Camera images leave your laptop and
are processed in your own Codespace/devcontainer. They are kept in memory, not
recorded automatically. **Save evidence** explicitly writes an annotated image
and, for Task 3, a CSV. Check your background and saved images before submitting.
Stop camera releases the camera. Keep the application port private.

## Setup

Use the repository's Codespace or local devcontainer. Rebuild an existing
container after pulling the Lab 4 configuration so port **8000** is forwarded.
You do not need Python installed on your laptop or USB/camera passthrough.

From the repository root:

```sh
uv sync
uv run python lab4/app.py
```

On its first run, the application downloads the versioned MediaPipe Pose
Landmarker Lite model into `lab4/models/` (about 6 MB). This needs internet
access; later runs use the cached file. If initialization reports a corrupt
model, remove that cached `.task` file and rerun. The model is not submitted.

Open the URL printed by the application, or **Open in Browser** on the port
labeled **Lab 4 camera and URSim control** in the Ports panel. Use a normal
Chrome, Edge, Firefox, or Safari tab, not VS Code's embedded Simple Browser.
In Codespaces use the forwarded **HTTPS** URL and authenticate with GitHub if
prompted. Locally use `http://localhost:8000/`. The page and camera requests use
one port; there is no separate WebSocket authentication step.

Click **Start camera** and allow the browser and operating system to use your
camera. Close other apps using the camera if necessary. The preview is not
mirrored: MediaPipe's **left** means your anatomical left, not image-left.
Keep your torso and left shoulder, elbow, and wrist visible, with your arm
roughly parallel to the camera image plane. Angles measured from a 2D image
change with camera viewpoint; this lab does not estimate anatomical 3D angles.

Tasks 1 and 2 work without a running URSim. For Task 3, open port **6080** at
`/vnc.html`, connect to the URSim desktop, accept its safety prompts, and power
on/release the brakes as in the repository setup instructions. Open PolyScope's
Move tab. Use two side-by-side browser windows: hiding the camera tab disables
tracking. Run only one lab/controller process and one camera tab per Codespace.

Python uses `URSIM_HOST=ursim_cb3` inside the devcontainer. A Python process
running directly on the Docker host uses `URSIM_HOST=localhost`. Do not paste
forwarded HTTPS URLs into the RTDE host setting. Native local Python is not
required for this lab; when using it, also expose URSim's control interfaces as
in the supplied Compose configuration.

## What you edit

- `student.py`: the provided working angle and mapping functions. Read and
  explain them, then modify your mapping as requested.
- `student_config.py`: gains, the shoulder switch, and smoothing time constant.
- `report.md`: record the evidence and observations below.

The supplied `app.py`, `vision.py`, `controller.py`, and `static/` handle browser
capture, model loading, communication, and control timing. Do not modify those
files or the fixed safety settings for required tasks. Restart the Python app
and reload its page after editing Python files; start the camera and recalibrate.

## Task 1 — Observe landmarks and measure angles

1. Start the camera. Keep robot control disabled.
2. Inspect `student.arm_angles()`. The elbow angle is the interior angle
   shoulder → elbow → wrist: a straight arm is approximately 180 degrees.
   The shoulder measurement is the signed upper-arm angle from image-down,
   positive toward image-right. Calculation uses pixel coordinates to respect
   the image aspect ratio; internal values are radians, displayed human angles
   are degrees.
3. Compare a roughly straight elbow, a roughly right-angle elbow, and a bent
   elbow. Explain discrepancies due to camera viewpoint and tracking noise.
4. Cover your wrist or leave the image. Confirm the app reports missing or
   low-confidence tracking instead of inventing a valid angle.
5. Select **Task 1** and **Save evidence** with a clear tracked pose.

Generated file: `lab4/outputs/task1_preview.jpg`.

## Task 2 — Calibrate and map an elbow target

1. Hold a comfortable pose and click **Calibrate neutral pose**. That human
   pose maps to `HOME_Q`, the same starting configuration used in Lab 2.
2. Inspect `student.joint_targets()`. The elbow target is the home elbow angle
   plus `ELBOW_GAIN` times the change in human elbow angle, clipped to the
   supplied bounds. Other joints remain at home; shoulder control is off.
3. Move your elbow and observe the six target values without enabling URSim.
   Change only `ELBOW_GAIN`: compare a smaller magnitude and the opposite sign.
   Restart and recalibrate after each change. Explain direction, sensitivity,
   and what happens at the target bounds. Use a positive gain for initial
   live-control testing.
4. Select **Task 2** and **Save evidence** with a non-neutral bounded target.

Generated file: `lab4/outputs/task2_preview.jpg`.

## Task 3 — Follow your arm in URSim

1. Initialize URSim as above. Click **Initialize URSim (moves robot)** once.
   This explicitly moves the simulated robot to `HOME_Q` at limited speed.
   Wait for initialization to finish. **Stop robot** cancels initialization
   or disables tracking.
2. Hold your neutral human pose and calibrate again. Click **Enable robot
   tracking**. Move slowly and compare the target and actual joint positions
   with PolyScope. No movement is commanded simply by starting the app/camera.
3. Verify fail-closed behavior: hide your wrist or stop the camera. The
   controller must disable, not resume automatically when tracking returns.
   Restore the camera/pose and explicitly enable again. Also test Stop robot.
4. Set `CONTROL_SHOULDER = True`. Restart, initialize, and recalibrate. Test
   slow elbow and shoulder movements with the remaining four joints fixed.
5. Compare `SMOOTHING_TIME_CONSTANT = 0` with the supplied `0.15` seconds.
   Keep gains unchanged for this comparison. Explain the jitter/lag tradeoff;
   filtering does not replace the motion-rate limit.
6. Run a short trial containing actual enabled motion and a tracking-loss stop.
   Select **Task 3** and **Save evidence** before stopping/restarting the camera.

Generated files:

```text
lab4/outputs/task3_preview.jpg
lab4/outputs/task3_control.csv
```

The CSV contains elapsed server time, tracking validity, whether motion was
enabled, human angles, six mapped target joints, and six measured actual joints
(all angles in radians). Blank entries mean unavailable data, not zero. It
contains the latest 6000 processed observations since Start camera, not every
125 Hz servo command. A fresh camera session clears the in-memory log. Saved
evidence is overwritten when saved again for the same task.

## Supplied control behavior

The browser sends at most one frame request at a time and captures the next
frame only after processing. It never queues a video backlog. The preview shows
server processing time and request round-trip time; these are not precise
camera-to-robot latency measurements.

A separate worker owns the RTDE connection, uses the latest target, smooths
changes, and limits commanded joint speed. The CB3 UR5 control loop targets
125 Hz; a Codespace is **not** a hard-real-time system. Loss of confidence,
invalid input, or no valid observations within `TARGET_MAX_AGE` disables control.
It also stops on controller errors. Enable requires fresh tracking and explicit
user action; seeing a person again never re-enables motion by itself.

The fixed joint bounds and starting configuration are for the supplied empty
URSim scene. This application does not perform Lab 3 collision checking.

## Submission

Fill in `report.md`, save evidence for all three tasks, then run:

```sh
cd lab4
bash submit.sh
```

The script packages `student.py`, `student_config.py`, `report.md`, the three
images, and `outputs/task3_control.csv` into `lab4/submission.zip`. Download and
upload the ZIP to the Lab 4 Canvas assignment. File-existence checks do not
verify report completion or that the CSV contains a successful control trial.
Do not include the model cache or unrelated webcam recordings.

## Troubleshooting

- **No camera permission prompt:** open the application in a normal HTTPS tab
  (or localhost), check browser/site and OS camera permissions, and close other
  camera apps. A camera error does not indicate an RTDE connection problem.
- **Old/expired camera session:** close duplicate tabs, then Start camera again.
- **Model download/import failure:** read the server terminal, confirm `uv sync`
  completed and internet access is available, then rerun.
- **Initialize fails:** make sure URSim is powered on with brakes released;
  stop other Lab 2/4 scripts. Check `URSIM_HOST` and the server's controller
  message. Camera preview remains usable without URSim.
- **Tracking keeps disabling:** improve lighting and visibility, keep the app
  visible, inspect processing/round-trip times, and check Codespace load.
  Do not increase the stale timeout to hide a stalled pipeline.
- **Server unreachable:** do not assume the robot stopped. Check PolyScope.
  When changing networks or restarting, explicitly reinitialize/re-enable.

## References

- [MediaPipe Pose Landmarker](https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker)
- [OpenCV Python tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [Browser camera permissions](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia)
- [ur_rtde control API](https://sdurobotics.gitlab.io/ur_rtde/pages/reference/api.html)
