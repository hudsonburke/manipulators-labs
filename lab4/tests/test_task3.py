"""Task 3 host selection: real CLI parsing, no camera or robot required."""

import unittest
from unittest.mock import patch

import task3_control


class HostSelectionTests(unittest.TestCase):
    def run_with(self, argv, robot):
        with patch.object(task3_control, 'PoseTracker'), \
                patch.object(task3_control, 'Camera'), \
                patch.object(task3_control, 'RobotController', robot), \
                patch.object(task3_control.sys, 'stdout'), \
                patch.object(task3_control.sys, 'stderr'):
            return task3_control.main(argv)

    def capture_host(self, argv):
        created = []

        def robot(host):
            created.append(host)
            raise RuntimeError('stop before opening the camera')

        self.assertEqual(self.run_with(argv, robot), 1)
        return created

    def test_host_flag_reaches_the_controller(self):
        self.assertEqual(self.capture_host(['--host', '192.0.2.5']), ['192.0.2.5'])

    def test_default_host_is_the_local_simulator(self):
        with patch.dict(task3_control.os.environ, {}, clear=True):
            self.assertEqual(self.capture_host([]), ['localhost'])
        with patch.dict(task3_control.os.environ, {'URSIM_HOST': 'sim.example'}):
            self.assertEqual(self.capture_host([]), ['sim.example'])

    def test_url_is_rejected_before_talking_to_the_robot(self):
        for value in ('http://192.0.2.5', '192.0.2.5/', ''):
            with self.subTest(value=value), patch.object(task3_control.sys, 'stderr'):
                with self.assertRaises(SystemExit) as error:
                    task3_control.main(['--host', value])
                self.assertEqual(error.exception.code, 2)

    def test_warning_only_for_non_local_targets(self):
        for host in ('localhost', '127.0.0.1', '::1'):
            self.assertIsNone(task3_control.demo_warning(host))
        warning = task3_control.demo_warning('192.0.2.5')
        self.assertIn('emergency stop', warning)
        self.assertIn('192.0.2.5', warning)


if __name__ == '__main__':
    unittest.main()
