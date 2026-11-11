"""Calibration, target mapping, and motion gating for Task 3.

The camera loop feeds poses in; the controller worker owns all RTDE traffic.
"""

import time
from collections import deque

import student_config as config
from student import joint_targets

LOG_COLUMNS = ['time_s', 'tracking', 'enabled', 'human_elbow_rad', 'human_shoulder_rad',
               *[f'target_q{i}' for i in range(1, 7)], *[f'actual_q{i}' for i in range(1, 7)]]


class ControlSession:
    """Hold the neutral pose and targets; gate motion on fresh tracking."""

    def __init__(self, robot):
        self.robot = robot
        self.neutral = self.angles = self.target = None
        self.observed_at = float('-inf')
        self.feedback = 'Preview only; nothing moves until you enable control.'
        self.started_at = time.monotonic()
        # shortcut: keep the latest 6000 observations; use streaming logs for longer trials.
        self.log = deque(maxlen=6000)

    def tracking(self):
        return (self.angles is not None
                and 0 <= time.monotonic() - self.observed_at <= config.TARGET_MAX_AGE)

    def snapshot(self):
        return self.robot.snapshot()

    def observe(self, angles, observed_at, message):
        """Accept one tracked pose; stale or invalid poses stop motion."""
        self.angles, self.observed_at, self.target = angles, observed_at, None
        if not self.tracking():
            self.robot.disable(message if angles is None
                               else 'Pose is stale; robot control disabled.')
        elif self.neutral is not None:
            self.target = joint_targets(angles, self.neutral)
            self.robot.update_target(self.target, observed_at)
        self.log.append(self._log_row())

    def _log_row(self):
        state = self.robot.snapshot()
        actual = state['actual_q']
        return [time.monotonic() - self.started_at, self.tracking(), state['enabled'],
                self.angles['elbow'] if self.angles else '',
                self.angles['shoulder'] if self.angles else '',
                *(self.target.tolist() if self.target is not None else [''] * 6),
                *(actual if actual is not None else [''] * 6)]

    def initialize(self):
        self.robot.initialize()
        self.neutral = self.target = None
        self.feedback = 'Connecting and moving to HOME.'

    def calibrate(self):
        self._require_pose()
        state = self.robot.snapshot()
        if state['enabled'] or state['busy']:
            raise ValueError('Stop robot motion and wait before calibrating.')
        self.neutral = dict(self.angles)
        self.target = joint_targets(self.angles, self.neutral)
        self.robot.update_target(self.target, self.observed_at)
        self.feedback = 'Neutral pose calibrated. E enables control.'

    def enable(self):
        self._require_pose()
        state = self.robot.snapshot()
        if not state['initialized'] or state['busy']:
            raise ValueError('Initialize successfully and wait for HOME before enabling.')
        if self.neutral is None or self.target is None:
            raise ValueError('Calibrate a neutral pose before enabling robot control.')
        self.robot.update_target(self.target, self.observed_at)
        self.robot.enable()
        self.feedback = 'Enable requested. Space or S stops motion.'

    def stop(self, reason='Stopped by user. Enable again explicitly to resume.'):
        self.robot.disable(reason)
        self.feedback = str(reason)

    def _require_pose(self):
        if not self.tracking():
            self.robot.disable('Missing or stale pose; robot control disabled.')
            raise ValueError('A fresh left shoulder, elbow, and wrist pose is required.')

    def close(self):
        self.robot.disable('Application closing; robot control disabled.')
        self.robot.close()
