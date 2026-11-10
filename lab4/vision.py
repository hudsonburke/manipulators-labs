"""MediaPipe Tasks left-arm tracking for browser camera frames."""

import math
import os
from pathlib import Path
import tempfile
import time
import urllib.error
import urllib.request

import cv2
import mediapipe as mp
import numpy as np

if __package__:
    from . import student, student_config as config
else:
    import student
    import student_config as config


MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
    "pose_landmarker_lite/float16/1/pose_landmarker_lite.task"
)
MODEL_PATH = Path(__file__).resolve().parent / "models" / "pose_landmarker_lite.task"
LEFT_ARM = (11, 13, 15)  # MediaPipe anatomical left shoulder, elbow, wrist.


def _model_path():
    if MODEL_PATH.is_file() and MODEL_PATH.stat().st_size:
        return MODEL_PATH
    temporary = None
    try:
        MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(MODEL_URL, timeout=30) as response:
            with tempfile.NamedTemporaryFile(dir=MODEL_PATH.parent, suffix=".download", delete=False) as output:
                temporary = Path(output.name)
                deadline = time.monotonic() + 60
                size = 0
                while True:
                    block = response.read(1024 * 1024)
                    if not block:
                        break
                    size += len(block)
                    if size > 32 * 1024 * 1024 or time.monotonic() > deadline:
                        raise RuntimeError("Pose model download exceeded its size or time limit")
                    output.write(block)
                if not size:
                    raise RuntimeError("Pose model download was empty")
        os.replace(temporary, MODEL_PATH)
        return MODEL_PATH
    except (OSError, urllib.error.URLError, RuntimeError) as exc:
        raise RuntimeError(f"Cannot cache pose model from {MODEL_URL}: {exc}") from exc
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


class PoseTracker:
    def __init__(self):
        model = _model_path()
        options = mp.tasks.vision.PoseLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model)),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=config.MIN_LANDMARK_CONFIDENCE,
            min_pose_presence_confidence=config.MIN_LANDMARK_CONFIDENCE,
            min_tracking_confidence=config.MIN_LANDMARK_CONFIDENCE,
        )
        try:
            self._landmarker = mp.tasks.vision.PoseLandmarker.create_from_options(options)
        except Exception as exc:
            raise RuntimeError(f"Cannot initialize MediaPipe pose model {model}: {exc}") from exc
        self._timestamp_ms = -1

    def process(self, frame):
        """Return annotated BGR frame, valid radians or None, and tracking status."""
        if (not isinstance(frame, np.ndarray) or frame.dtype != np.uint8
                or frame.ndim != 3 or frame.shape[2] != 3
                or frame.shape[0] == 0 or frame.shape[1] == 0):
            raise ValueError("Camera frame must be a nonempty uint8 BGR image")
        if self._landmarker is None:
            raise RuntimeError("Pose tracker is closed")
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        self._timestamp_ms = max(self._timestamp_ms + 1, time.monotonic_ns() // 1_000_000)
        result = self._landmarker.detect_for_video(image, self._timestamp_ms)
        annotated = frame.copy()
        angles = None
        message = "No person detected — show your left shoulder, elbow and wrist"
        if result.pose_landmarks:
            landmarks = result.pose_landmarks[0]
            arm = [landmarks[index] for index in LEFT_ARM]
            confidence = [value for point in arm for value in (point.visibility, point.presence)]
            if any(value is None or not math.isfinite(value)
                   or value < config.MIN_LANDMARK_CONFIDENCE for value in confidence):
                message = "Low left-arm confidence — show shoulder, elbow and wrist clearly"
            else:
                height, width = frame.shape[:2]
                coordinates = np.array([(point.x, point.y) for point in arm])
                try:
                    if (not np.isfinite(coordinates).all()
                            or np.any(coordinates < 0) or np.any(coordinates > 1)):
                        raise ValueError("Left arm is outside the camera image")
                    points = coordinates * (width, height)
                    angles = student.arm_angles(*points)
                except ValueError:
                    message = "Invalid left-arm geometry — keep all three joints clearly in view"
                else:
                    message = "Tracking left arm"
                    pixels = [tuple(np.rint(point).astype(int)) for point in points]
                    for first, second in zip(pixels, pixels[1:]):
                        cv2.line(annotated, first, second, (0, 220, 0), 3)
                    for point, label in zip(pixels, ("L shoulder", "L elbow", "L wrist")):
                        cv2.circle(annotated, point, 6, (0, 220, 0), -1)
                        cv2.putText(annotated, label, (point[0] + 8, point[1] - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1, cv2.LINE_AA)
                    for row, key in enumerate(("elbow", "shoulder"), start=2):
                        cv2.putText(annotated, f"{key}: {math.degrees(angles[key]):.1f} deg",
                                    (12, 26 * row), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0, 220, 0), 2, cv2.LINE_AA)
        # OpenCV's built-in font only supports ASCII.
        cv2.putText(annotated, message.replace("—", "-"), (12, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 220, 0) if angles is not None else (0, 180, 255), 1, cv2.LINE_AA)
        return annotated, angles, message

    def close(self):
        if self._landmarker is not None:
            self._landmarker.close()
            self._landmarker = None
