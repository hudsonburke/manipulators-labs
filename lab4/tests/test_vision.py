"""Geometry and tracking contract tests; no model download or inference needed."""

import io
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from lab4 import student, student_config as config, vision


class StudentTests(unittest.TestCase):
    def test_known_pixel_geometry_and_sign(self):
        angles = student.arm_angles((100, 10), (100, 50), (140, 50))
        self.assertAlmostEqual(angles["elbow"], math.pi / 2)
        self.assertEqual(angles["shoulder"], 0)
        self.assertAlmostEqual(student.arm_angles((0, 0), (10, 0), (20, 0))["elbow"], math.pi)
        self.assertAlmostEqual(student.arm_angles((0, 0), (-10, 0), (-20, 0))["shoulder"], -math.pi / 2)

    def test_invalid_geometry_is_rejected(self):
        for points in (((0, 0), (0, 0), (1, 2)),
                       ((0, 0), (1, 2), (1, 2)),
                       ((0, 0), (1, float("nan")), (1, 2)),
                       ((0, 0, 0), (1, 2, 3), (2, 3, 4))):
            with self.subTest(points=points), self.assertRaises(ValueError):
                student.arm_angles(*points)

    def test_neutral_gain_limits_and_fixed_joints(self):
        neutral = {"elbow": 1.5, "shoulder": 0.2}
        np.testing.assert_allclose(student.joint_targets(neutral, neutral), config.HOME_Q)
        with patch.object(config, "CONTROL_SHOULDER", False):
            target = student.joint_targets({"elbow": 1.7, "shoulder": 2.0}, neutral)
        self.assertAlmostEqual(target[2], config.HOME_Q[2] + config.ELBOW_GAIN * 0.2)
        np.testing.assert_array_equal(target[[0, 1, 3, 4, 5]], config.HOME_Q[[0, 1, 3, 4, 5]])
        high = student.joint_targets({"elbow": 100, "shoulder": 0}, neutral)
        low = student.joint_targets({"elbow": -100, "shoulder": 0}, neutral)
        self.assertEqual(high[2], config.JOINT_MAX[2])
        self.assertEqual(low[2], config.JOINT_MIN[2])

    def test_shoulder_delta_wraps_at_pi(self):
        with patch.object(config, "CONTROL_SHOULDER", True):
            target = student.joint_targets({"elbow": 1, "shoulder": -math.pi + 0.1},
                                           {"elbow": 1, "shoulder": math.pi - 0.1})
        self.assertAlmostEqual(target[1], config.HOME_Q[1] + config.SHOULDER_GAIN * 0.2)

    def test_invalid_mapping_inputs_are_rejected(self):
        for angles in (None, {}, {"elbow": float("inf"), "shoulder": 0},
                       {"elbow": 0, "shoulder": float("nan")}):
            with self.subTest(angles=angles), self.assertRaises(ValueError):
                student.joint_targets(angles, {"elbow": 0, "shoulder": 0})


class TrackerTests(unittest.TestCase):
    def setUp(self):
        self.tracker = vision.PoseTracker.__new__(vision.PoseTracker)
        self.tracker._timestamp_ms = -1
        self.driver = Mock()
        self.tracker._landmarker = self.driver
        self.frame = np.zeros((200, 400, 3), dtype=np.uint8)

    def result(self, points=None, confidence=1.0):
        if points is None:
            return SimpleNamespace(pose_landmarks=[])
        landmarks = [None] * 33
        for index, (x, y) in zip(vision.LEFT_ARM, points):
            landmarks[index] = SimpleNamespace(x=x, y=y, visibility=confidence, presence=confidence)
        return SimpleNamespace(pose_landmarks=[landmarks])

    def test_pixel_aspect_ratio_and_annotation(self):
        points = ((0.25, 0.25), (0.5, 0.75), (0.75, 0.75))
        self.driver.detect_for_video.return_value = self.result(points)
        annotated, angles, message = self.tracker.process(self.frame)
        self.assertAlmostEqual(angles["shoulder"], math.pi / 4)
        self.assertAlmostEqual(angles["elbow"], 3 * math.pi / 4)
        self.assertFalse(np.any(self.frame))

    def test_missing_low_confidence_and_degenerate_arms_have_no_angles(self):
        valid = ((0.25, 0.25), (0.5, 0.5), (0.75, 0.5))
        results = (self.result(), self.result(valid, 0.1),
                   self.result(((0.5, 0.5),) * 3),
                   self.result(((float("nan"), 0.5), (0.5, 0.5), (0.8, 0.5))),
                   self.result(((-0.1, 0.5), (0.5, 0.5), (0.8, 0.5))))
        for result in results:
            with self.subTest(result=result):
                self.driver.detect_for_video.return_value = result
                _, angles, message = self.tracker.process(self.frame)
                self.assertIsNone(angles)

    def test_video_timestamps_increase_even_with_same_clock(self):
        self.driver.detect_for_video.return_value = self.result()
        with patch.object(vision.time, "monotonic_ns", return_value=10_000_000):
            self.tracker.process(self.frame)
            self.tracker.process(self.frame)
        timestamps = [call.args[1] for call in self.driver.detect_for_video.call_args_list]
        self.assertLess(timestamps[0], timestamps[1])

    def test_close_is_idempotent_and_invalid_frames_are_rejected(self):
        for frame in (None, np.zeros((0, 2, 3), dtype=np.uint8), np.zeros((2, 2))):
            with self.assertRaises(ValueError):
                self.tracker.process(frame)
        self.tracker.close()
        self.tracker.close()
        with self.assertRaises(RuntimeError):
            self.tracker.process(self.frame)


class ModelCacheTests(unittest.TestCase):
    def test_cached_model_is_available_offline(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pose.task"
            path.write_bytes(b"existing model")
            with patch.object(vision, "MODEL_PATH", path), patch.object(
                    vision.urllib.request, "urlopen", side_effect=OSError("offline")):
                self.assertEqual(vision._model_path(), path)

    def test_failed_download_never_publishes_partial_model(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "models" / "pose.task"
            with patch.object(vision, "MODEL_PATH", path), patch.object(
                    vision.urllib.request, "urlopen", return_value=io.BytesIO(b"")):
                with self.assertRaises(RuntimeError):
                    vision._model_path()
                self.assertFalse(path.exists())
                self.assertEqual(list(path.parent.iterdir()), [])


if __name__ == "__main__":
    unittest.main()
