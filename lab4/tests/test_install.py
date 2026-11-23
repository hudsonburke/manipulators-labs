"""Real native startup imports and API checks; no devices, model, or connections."""

import importlib
from importlib.metadata import version
import unittest


class NativeInstallTests(unittest.TestCase):
    def test_task3_startup_modules_import_without_hardware(self):
        # --help exits before controller's deferred import, so cover it explicitly.
        for name in ('numpy', 'cv2', 'mediapipe', 'rtde_control', 'rtde_receive',
                     'task3_control', 'lab4lib.controller'):
            with self.subTest(module=name):
                module = importlib.import_module(name)
                self.assertIsNotNone(module.__file__)
        # A distribution name is not an import name: do not import "ur_rtde".
        self.assertTrue(version('ur-rtde'))

    def test_mediapipe_tasks_api_used_by_pose_tracker_is_available(self):
        mp = importlib.import_module('mediapipe')
        for api in (mp.Image, mp.tasks.BaseOptions,
                    mp.tasks.vision.PoseLandmarkerOptions,
                    mp.tasks.vision.PoseLandmarker.create_from_options):
            self.assertTrue(callable(api))
        self.assertIsNotNone(mp.ImageFormat.SRGB)
        self.assertIsNotNone(mp.tasks.vision.RunningMode.VIDEO)

    def test_native_rtde_control_and_receive_api_is_available(self):
        control = importlib.import_module('rtde_control').RTDEControlInterface
        receive = importlib.import_module('rtde_receive').RTDEReceiveInterface
        for driver, names in (
            (control, ('moveJ', 'servoJ', 'stopJ', 'servoStop', 'isConnected',
                       'isProgramRunning', 'getAsyncOperationProgress', 'disconnect')),
            (receive, ('getActualQ', 'getActualQd', 'isConnected', 'disconnect')),
        ):
            for name in names:
                with self.subTest(driver=driver.__name__, method=name):
                    self.assertTrue(callable(getattr(driver, name)))
        self.assertIsInstance(control.FLAG_UPLOAD_SCRIPT | control.FLAG_NO_WAIT, int)


if __name__ == '__main__':
    unittest.main()
