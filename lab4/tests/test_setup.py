"""Read-only setup diagnostics, requiring neither ur_rtde nor a simulator."""

import contextlib
import io
import subprocess
import struct
import unittest
from unittest.mock import Mock, patch

import check_setup


class SetupTests(unittest.TestCase):
    def run_cli(self, argv, environment_ok=True):
        output = io.StringIO()
        with contextlib.redirect_stdout(output), \
                patch.object(check_setup, 'check_environment', return_value=environment_ok):
            code = check_setup.main(argv)
        return code, output.getvalue()

    def test_default_never_connects_or_starts_a_probe(self):
        with patch.object(check_setup.socket, 'create_connection') as connect, \
                patch.object(check_setup.subprocess, 'run') as run:
            code, output = self.run_cli([])
        self.assertEqual(code, 0)
        self.assertIn('Robot/camera untouched', output)
        connect.assert_not_called()
        run.assert_not_called()

    def test_robot_probe_uses_platform_default_and_preserves_overrides(self):
        for platform, default in (('win32', '127.0.0.1'), ('darwin', 'localhost'),
                                  ('linux', 'localhost')):
            for environment, argv, expected in (
                ({}, [], default),
                ({'URSIM_HOST': 'sim.example'}, [], 'sim.example'),
                ({'URSIM_HOST': 'sim.example'}, ['--host', '127.0.0.1'], '127.0.0.1'),
            ):
                with self.subTest(platform=platform, environment=environment, argv=argv), \
                        patch.object(check_setup.sys, 'platform', platform), \
                        patch.dict(check_setup.os.environ, environment, clear=True), \
                        patch.object(check_setup.subprocess, 'run', return_value=Mock(returncode=0)) as run:
                    code, _ = self.run_cli(['--robot', *argv])
                self.assertEqual(code, 0)
                command = run.call_args.args[0]
                self.assertEqual(command[command.index('--host') + 1], expected)

    def test_robot_probe_still_runs_when_environment_fails(self):
        with patch.dict(check_setup.os.environ, {'URSIM_HOST': 'sim.example'}), \
                patch.object(check_setup.subprocess, 'run', return_value=Mock(returncode=0)) as run:
            code, _ = self.run_cli(['--robot'], environment_ok=False)
        self.assertEqual(code, 1)
        self.assertIn('sim.example', run.call_args.args[0])
        self.assertGreater(run.call_args.kwargs['timeout'], 0)

    def test_host_override_and_probe_failure(self):
        with patch.dict(check_setup.os.environ, {'URSIM_HOST': 'other.example'}), \
                patch.object(check_setup.subprocess, 'run', return_value=Mock(returncode=1)) as run:
            code, _ = self.run_cli(['--robot', '--host', 'localhost'])
        self.assertEqual(code, 1)
        self.assertIn('localhost', run.call_args.args[0])
        self.assertNotIn('other.example', run.call_args.args[0])

    def test_overall_probe_deadline_reports_all_endpoints(self):
        with patch.object(check_setup.subprocess, 'run',
                          side_effect=subprocess.TimeoutExpired('probe', 22)):
            code, output = self.run_cli(['--robot', '--host', 'slow.example'])
        self.assertEqual(code, 1)
        for port in (29999, 30003, 30004):
            self.assertIn(f'slow.example:{port}', output)

    def test_bad_host_or_timeout_is_rejected(self):
        for argv in (['--host', 'http://localhost'], ['--host', ''], ['--host', 'bad\nhost'],
                     ['--timeout', 'nan'], ['--timeout', '0'], ['--timeout', '31']):
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    check_setup.main(argv)
                self.assertEqual(error.exception.code, 2)

    def test_import_errors_keep_original_loader_diagnostics(self):
        missing = ModuleNotFoundError("No module named 'rtde_control'", name='rtde_control')
        loader = ImportError('DLL load failed: dependent library unavailable')
        output = io.StringIO()
        with contextlib.redirect_stdout(output), \
                patch.object(check_setup.metadata, 'distribution',
                             side_effect=check_setup.metadata.PackageNotFoundError('ur-rtde')), \
                patch.object(check_setup.importlib, 'import_module', side_effect=[missing, loader]) as imports:
            self.assertFalse(check_setup.check_environment())
        self.assertEqual(imports.call_count, 2)
        self.assertIn('module missing', output.getvalue())
        self.assertIn('extension/dependency load failed', output.getvalue())
        self.assertIn('DLL load failed: dependent library unavailable', output.getvalue())
        self.assertIn("No module named 'rtde_control'", output.getvalue())
        self.assertIn('Interpreter:', output.getvalue())

    def test_transitive_missing_module_is_not_mislabeled(self):
        missing = ModuleNotFoundError("No module named 'dependency'", name='dependency')
        output = io.StringIO()
        with contextlib.redirect_stdout(output), \
                patch.object(check_setup.metadata, 'distribution', return_value=Mock(version='1.6.5')), \
                patch.object(check_setup.importlib, 'import_module', side_effect=missing):
            self.assertFalse(check_setup.check_environment())
        self.assertNotIn('(module missing)', output.getvalue())
        self.assertIn("No module named 'dependency'", output.getvalue())


class DashboardTests(unittest.TestCase):
    def socket_with(self, text):
        connection = Mock()
        connection.recv.side_effect = [bytes([byte]) for byte in text.encode('ascii')]
        return connection

    def dashboard(self, robotmode='RUNNING', safetymode='NORMAL', program='STOPPED'):
        return self.socket_with('Connected: Universal Robots Dashboard Server\n'
                                f'Robotmode: {robotmode}\nSafetymode: {safetymode}\n{program}\n')

    def test_commands_are_serial_and_read_only(self):
        connection = self.dashboard()
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(check_setup.check_dashboard(connection, 'localhost', 0.1))
        self.assertEqual([call.args[0] for call in connection.sendall.call_args_list],
                         [b'robotmode\n', b'safetymode\n', b'programState\n'])

    def test_not_ready_states_fail_without_changing_robot(self):
        for mode, safety in (('POWER_OFF', 'NORMAL'), ('IDLE', 'NORMAL'),
                             ('RUNNING', 'PROTECTIVE_STOP')):
            with self.subTest(mode=mode, safety=safety), contextlib.redirect_stdout(io.StringIO()):
                connection = self.dashboard(mode, safety)
                self.assertFalse(check_setup.check_dashboard(connection, 'robot.example', 0.1))
                self.assertEqual(connection.sendall.call_count, 3)

    def test_closed_or_partial_banner_fails(self):
        connection = Mock()
        connection.recv.side_effect = [b'C', b'']
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertFalse(check_setup.check_dashboard(connection, 'robot.example', 0.1))
        self.assertIn('robot.example:29999 banner', output.getvalue())
        connection.sendall.assert_not_called()

    def test_line_limit_prevents_unbounded_replies(self):
        connection = Mock()
        connection.recv.return_value = b'x'
        with self.assertRaisesRegex(ValueError, '4096'):
            check_setup.read_dashboard_line(connection, 1)

    def test_timeout_stops_queries_without_misaligning_replies(self):
        connection = self.socket_with('Connected: Universal Robots Dashboard Server\n')
        connection.recv.side_effect = list(connection.recv.side_effect) + [TimeoutError('timed out')]
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertFalse(check_setup.check_dashboard(connection, 'robot.example', 0.1))
        self.assertIn('robot.example:29999 robotmode', output.getvalue())
        self.assertEqual(connection.sendall.call_count, 1)

    def test_partial_port_availability_reports_exact_failed_endpoint(self):
        dashboard = self.dashboard()
        rtde = Mock()
        rtde.recv.side_effect = TimeoutError('no unsolicited bytes')
        dashboard.__enter__ = Mock(return_value=dashboard)
        dashboard.__exit__ = Mock(return_value=False)
        rtde.__enter__ = Mock(return_value=rtde)
        rtde.__exit__ = Mock(return_value=False)
        output = io.StringIO()
        with patch.object(check_setup.socket, 'create_connection',
                          side_effect=[dashboard, ConnectionRefusedError('refused'), rtde]) as connect, \
                contextlib.redirect_stdout(output):
            self.assertFalse(check_setup.check_robot('robot.example', 0.1))
        self.assertIn('FAIL TCP robot.example:30003', output.getvalue())
        self.assertIn('OK TCP robot.example:30004', output.getvalue())
        self.assertEqual([call.args[0] for call in connect.call_args_list],
                         [('robot.example', 29999), ('robot.example', 30003), ('robot.example', 30004)])
        rtde.sendall.assert_not_called()

    def probe_with_rtde(self, chunks):
        dashboard = self.dashboard()
        upload = Mock()
        rtde = Mock()
        rtde.recv.side_effect = chunks
        for connection in (dashboard, upload, rtde):
            connection.__enter__ = Mock(return_value=connection)
            connection.__exit__ = Mock(return_value=False)
        output = io.StringIO()
        with patch.object(check_setup.socket, 'create_connection',
                          side_effect=[dashboard, upload, rtde]), \
                contextlib.redirect_stdout(output):
            result = check_setup.check_robot('robot.example', 0.1)
        upload.sendall.assert_not_called()
        upload.send.assert_not_called()
        rtde.sendall.assert_not_called()
        rtde.send.assert_not_called()
        self.assertEqual([call.args[0] for call in dashboard.sendall.call_args_list],
                         [b'robotmode\n', b'safetymode\n', b'programState\n'])
        return result, output.getvalue(), rtde

    def test_running_normal_dashboard_does_not_override_unconfirmed_safety(self):
        warning = ('SafetySetup has not been confirmed yet, '
                   'therefore no data will be send yet')
        body = b'\x02' + warning.encode('ascii')
        frame = struct.pack('!HB', len(body) + 3, 77) + body
        result, output, _ = self.probe_with_rtde([bytes([byte]) for byte in frame])
        self.assertFalse(result)
        self.assertIn('Robotmode: RUNNING', output)
        self.assertIn('Safetymode: NORMAL', output)
        self.assertIn(warning, output)

    def test_healthy_silent_rtde_passes_without_sending(self):
        result, output, _ = self.probe_with_rtde([TimeoutError('no unsolicited bytes')])
        self.assertTrue(result)
        self.assertNotIn('FAIL', output)

    def test_other_notice_followed_by_silence_is_not_safety_failure(self):
        body = b'\x02Other startup notice'
        frame = struct.pack('!HB', len(body) + 3, 77) + body
        result, output, _ = self.probe_with_rtde(
            [bytes([byte]) for byte in frame] + [TimeoutError('no further notices')])
        self.assertTrue(result)
        self.assertIn('Other startup notice', output)
        self.assertNotIn('FAIL', output)

    def test_partial_closed_and_malformed_notices_fail_without_sending(self):
        cases = (
            [b'\x00', TimeoutError('partial header')],
            [b''],
            [b'\x00', b'\x08', b'M', b'\x02', b''],
            [b'\x00', b'\x08', b'M', b'\x02', TimeoutError('partial body')],
            [b'\x00', b'\x02', b'M'],
            [b'\xff', b'\xff', b'M'],
            [b'\x00', b'\x03', b'M'],
            [b'\x00', b'\x05', b'M', b'\x02', b'\xff'],
        )
        for chunks in cases:
            with self.subTest(chunks=chunks):
                result, output, rtde = self.probe_with_rtde(chunks)
                self.assertFalse(result)
                self.assertIn('FAIL robot.example:30004 RTDE startup notice', output)
                self.assertLessEqual(rtde.recv.call_count, 5)

    def test_continuous_frames_are_bounded(self):
        # Unknown startup frame kinds do not justify consuming an unbounded stream.
        frame = [b'\x00', b'\x03', b'X']
        result, output, rtde = self.probe_with_rtde(frame * 16)
        self.assertFalse(result)
        self.assertIn('exceeded 16 frames', output)
        self.assertEqual(rtde.recv.call_count, 48)


if __name__ == '__main__':
    unittest.main()
