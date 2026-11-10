"""Camera-session and HTTP trust-boundary regressions without robot hardware."""

import http.client
import json
from threading import Thread
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from lab4 import app


class PreviewRobot:
    def __init__(self, host):
        self.enabled = False

    def snapshot(self):
        return {'initialized': True, 'enabled': self.enabled, 'busy': False,
                'message': 'Test transport', 'actual_q': None, 'command_q': None}

    def disable(self, reason='Stopped'):
        self.enabled = False

    def update_target(self, q, observed_at):
        pass

    def enable(self):
        self.enabled = True

    def close(self):
        self.disable()


class CameraAppTests(unittest.TestCase):
    def setUp(self):
        self.pose_patch = patch.object(app, 'PoseTracker')
        self.tracker = self.pose_patch.start().return_value
        self.robot_patch = patch.object(app, 'RobotController', PreviewRobot)
        self.robot_patch.start()
        self.addCleanup(self.pose_patch.stop)
        self.addCleanup(self.robot_patch.stop)
        self.camera = app.CameraApp('test-only')
        self.addCleanup(self.camera.close)
        self.frame = np.zeros((120, 160, 3), dtype=np.uint8)
        self.tracker.process.return_value = (self.frame, {'elbow': 1.5, 'shoulder': 0.2}, 'Tracked')
        self.jpeg = cv2.imencode('.jpg', self.frame)[1].tobytes()
        self.session = self.camera.action('session', '')['session']

    def observe_and_calibrate(self):
        self.camera.process(self.jpeg, self.session, time.monotonic())
        self.camera.action('calibrate', self.session)

    def test_enable_requires_fresh_calibration_and_tracking(self):
        with self.assertRaises(ValueError):
            self.camera.action('enable', self.session)
        self.observe_and_calibrate()
        state = self.camera.action('enable', self.session)
        self.assertTrue(state['controller']['enabled'])
        self.camera.action('disable', self.session)
        self.camera.observed_at = time.monotonic() - app.config.TARGET_MAX_AGE - 1
        with self.assertRaises(ValueError):
            self.camera.action('enable', self.session)
        self.assertFalse(self.camera.status()['controller']['enabled'])

    def test_new_session_disarms_and_rejects_old_observations(self):
        self.observe_and_calibrate()
        self.camera.action('enable', self.session)
        new = self.camera.action('session', '')
        self.assertFalse(new['controller']['enabled'])
        self.assertFalse(new['calibrated'])
        with self.assertRaises(ValueError):
            self.camera.process(self.jpeg, self.session, time.monotonic())
        self.assertFalse(self.camera.status()['tracking'])

    def test_tracking_loss_disables_and_recovery_does_not_resume(self):
        self.observe_and_calibrate()
        self.camera.action('enable', self.session)
        self.tracker.process.return_value = (self.frame, None, 'Missing wrist')
        lost = self.camera.process(self.jpeg, self.session, time.monotonic())
        self.assertFalse(lost['tracking'])
        self.assertFalse(lost['controller']['enabled'])
        self.assertIsNone(lost['target_q'])
        self.tracker.process.return_value = (self.frame, {'elbow': 1.5, 'shoulder': 0.2}, 'Tracked')
        recovered = self.camera.process(self.jpeg, self.session, time.monotonic())
        self.assertTrue(recovered['tracking'])
        self.assertFalse(recovered['controller']['enabled'])

    def test_http_authentication_and_upload_limits(self):
        server = app.CameraServer(('127.0.0.1', 0), self.camera)
        thread = Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            def request(path, token, data=b'{}', headers=None):
                connection = http.client.HTTPConnection(*server.server_address, timeout=5)
                connection.request('POST', path, data, {'X-Lab-Token': token, **(headers or {})})
                response = connection.getresponse()
                result = response.status, json.loads(response.read())
                connection.close()
                return result
            self.assertEqual(request('/api/enable', 'wrong-token')[0], 403)
            self.assertFalse(self.camera.status()['controller']['enabled'])
            self.assertEqual(request('/api/frame', self.camera.token,
                                     headers={'Content-Length': str(app.MAX_FRAME_BYTES + 1)})[0], 413)
            self.assertEqual(request('/api/frame', self.camera.token, b'not-a-jpeg',
                                     {'X-Camera-Session': self.session, 'Content-Type': 'image/jpeg'})[0], 400)
            self.assertFalse(self.camera.status()['controller']['enabled'])
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    unittest.main()
