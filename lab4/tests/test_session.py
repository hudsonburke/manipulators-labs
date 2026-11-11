"""Task 3 session safety: calibration, motion gating, and logging without robot hardware."""

import time
import unittest

import student_config as config
from lab4lib.session import LOG_COLUMNS, ControlSession


class PreviewRobot:
    def __init__(self, host):
        self.initialized = self.enabled = self.busy = self.closed = False
        self.target = None
        self.reason = 'Disconnected'
        self.received = []

    def snapshot(self):
        return {'initialized': self.initialized, 'enabled': self.enabled, 'busy': self.busy,
                'message': self.reason, 'actual_q': None, 'command_q': None}

    def initialize(self):
        self.initialized = True

    def disable(self, reason='Stopped'):
        self.enabled = False
        self.reason = reason

    def update_target(self, q, observed_at):
        self.target = q.copy()
        self.received.append(observed_at)

    def enable(self):
        self.enabled = True

    def close(self):
        self.disable()
        self.closed = True


class ControlSessionTests(unittest.TestCase):
    def setUp(self):
        self.robot = PreviewRobot('test-only')
        self.session = ControlSession(self.robot)

    def observe(self, angles=None, message='Tracked', observed_at=None):
        self.session.observe(angles, time.monotonic() if observed_at is None
                             else observed_at, message)

    def ready(self):
        self.observe({'elbow': 1.5, 'shoulder': 0.2})
        self.session.initialize()
        self.session.calibrate()

    def test_enable_requires_initialized_fresh_calibration(self):
        with self.assertRaises(ValueError):
            self.session.enable()
        self.observe({'elbow': 1.5, 'shoulder': 0.2})
        self.session.calibrate()
        with self.assertRaises(ValueError):
            self.session.enable()
        self.session.initialize()
        self.assertIsNone(self.session.neutral)
        self.observe({'elbow': 1.5, 'shoulder': 0.2})
        with self.assertRaises(ValueError):
            self.session.enable()
        self.session.calibrate()
        self.session.enable()
        self.assertTrue(self.robot.enabled)
        self.assertEqual(self.session.feedback, 'Enable requested. Space or S stops motion.')

    def test_enable_refuses_a_stale_pose(self):
        self.ready()
        self.session.observed_at = time.monotonic() - config.TARGET_MAX_AGE - 1
        with self.assertRaises(ValueError):
            self.session.enable()
        self.assertFalse(self.robot.enabled)

    def test_calibration_requires_stopped_fresh_pose(self):
        self.ready()
        original = dict(self.session.neutral)
        self.robot.busy = True
        with self.assertRaises(ValueError):
            self.session.calibrate()
        self.robot.busy = False
        self.robot.enabled = True
        with self.assertRaises(ValueError):
            self.session.calibrate()
        self.session.stop()
        self.session.observed_at -= config.TARGET_MAX_AGE + 1
        with self.assertRaises(ValueError):
            self.session.calibrate()
        self.assertEqual(self.session.neutral, original)

    def test_missing_or_stale_pose_stops_motion_and_never_resumes(self):
        self.ready()
        self.session.enable()
        self.observe(None, message='Missing wrist')
        self.assertFalse(self.robot.enabled)
        self.assertIsNone(self.session.target)
        self.assertEqual(self.robot.reason, 'Missing wrist')
        self.observe({'elbow': 1.5, 'shoulder': 0.2})
        self.assertFalse(self.robot.enabled)
        self.assertIsNotNone(self.session.target)
        self.session.observe({'elbow': 1.5, 'shoulder': 0.2}, time.monotonic() - config.TARGET_MAX_AGE - 1,
                             'Tracked')
        self.assertFalse(self.session.tracking())
        self.assertFalse(self.robot.enabled)

    def test_initialize_clears_calibration_and_target(self):
        self.ready()
        self.session.initialize()
        self.assertIsNone(self.session.neutral)
        self.assertIsNone(self.session.target)
        self.assertEqual(self.session.feedback, 'Connecting and moving to HOME.')

    def test_log_rows_keep_blanks_when_data_is_unavailable(self):
        self.observe(None, message='Missing wrist')
        row = dict(zip(LOG_COLUMNS, self.session.log[-1]))
        self.assertFalse(row['tracking'])
        self.assertFalse(row['enabled'])
        self.assertTrue(all(row[name] == '' for name in LOG_COLUMNS
                            if name not in ('time_s', 'tracking', 'enabled')))
        self.ready()
        self.session.enable()
        self.observe({'elbow': 1.5, 'shoulder': 0.2})
        row = dict(zip(LOG_COLUMNS, self.session.log[-1]))
        self.assertTrue(row['enabled'])
        self.assertAlmostEqual(row['human_elbow_rad'], 1.5)

    def test_stop_reports_its_reason(self):
        self.ready()
        self.session.enable()
        self.session.stop('Operator stop.')
        self.assertFalse(self.robot.enabled)
        self.assertEqual(self.session.feedback, 'Operator stop.')


if __name__ == '__main__':
    unittest.main()
