"""Deterministic safety tests: fake drivers never connect to a robot."""

import json
import math
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from lab4 import controller


class FakeTransport:
    def __init__(self):
        self.calls = []
        self.lock = threading.Lock()
        self.q = controller.config.HOME_Q.copy()
        self.connected = True
        self.program_running = True
        self.finish_home = True
        self.moving = False
        self.reject_home = False
        self.servo_error = False

    def record(self, name, *args):
        with self.lock:
            self.calls.append((name, threading.get_ident(), args))

    def calls_for(self, name):
        with self.lock:
            return [args for method, _, args in self.calls if method == name]

    def control(self, host, frequency, flags):
        self.record("connect_control", host, frequency, flags)
        return FakeControl(self)

    def receive(self, host, frequency):
        self.record("connect_receive", host, frequency)
        return FakeReceive(self)


class FakeControl:
    def __init__(self, transport):
        self.transport = transport

    def isConnected(self):
        self.transport.record("control_connected")
        return self.transport.connected

    def isProgramRunning(self):
        self.transport.record("program_running")
        return self.transport.connected and self.transport.program_running

    def moveJ(self, q, speed, acceleration, asynchronous):
        self.transport.record("moveJ", q, speed, acceleration, asynchronous)
        self.transport.moving = not self.transport.finish_home
        if self.transport.finish_home:
            self.transport.q = np.array(q)
        return not self.transport.reject_home

    def getAsyncOperationProgress(self):
        self.transport.record("async_progress")
        return 0 if self.transport.moving else -1

    def servoJ(self, q, speed, acceleration, period, lookahead, gain):
        self.transport.record("servoJ", q, speed, acceleration, period, lookahead, gain)
        if self.transport.servo_error:
            raise RuntimeError("simulated servo transport failure")
        self.transport.q = np.array(q)
        return True

    def servoStop(self, acceleration):
        self.transport.record("servoStop", acceleration)
        return True

    def stopJ(self, acceleration, asynchronous):
        self.transport.record("stopJ", acceleration, asynchronous)
        self.transport.moving = False

    def stopScript(self):
        self.transport.record("stopScript")

    def disconnect(self):
        self.transport.record("disconnect_control")


class FakeReceive:
    def __init__(self, transport):
        self.transport = transport

    def isConnected(self):
        self.transport.record("receive_connected")
        return self.transport.connected

    def getActualQ(self):
        self.transport.record("actual_q")
        return self.transport.q.tolist()

    def getActualQd(self):
        self.transport.record("actual_qd")
        return [0.0] * 6

    def disconnect(self):
        self.transport.record("disconnect_receive")


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.transport = FakeTransport()
        self.clock_offset = 0.0
        real_monotonic = time.monotonic
        self.patches = [
            patch.object(controller, "RTDEControlInterface", self.transport.control),
            patch.object(controller, "RTDEReceiveInterface", self.transport.receive),
            patch.object(controller.time, "monotonic", lambda: real_monotonic() + self.clock_offset),
        ]
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)
        self.robot = controller.RobotController("ursim")
        self.addCleanup(self.robot.close)

    def until(self, predicate):
        deadline = time.perf_counter() + 2.0
        while not predicate():
            if time.perf_counter() >= deadline:
                self.fail(f"Timed out waiting for controller: {self.robot.snapshot()}")
            time.sleep(0.001)

    def initialize(self):
        self.robot.initialize()
        self.until(lambda: self.robot.snapshot()["initialized"])

    def target(self, elbow=0.3):
        q = controller.config.HOME_Q.copy()
        q[2] += elbow
        self.robot.update_target(q, time.monotonic())
        return q

    def enable(self):
        self.initialize()
        self.target()
        self.robot.enable()
        self.until(lambda: bool(self.transport.calls_for("servoJ")))

    def test_startup_does_not_connect_or_move(self):
        time.sleep(0.02)
        self.assertEqual(self.transport.calls, [])
        self.assertFalse(self.robot.snapshot()["initialized"])
        self.assertFalse(self.robot.snapshot()["enabled"])
        with self.assertRaises(RuntimeError):
            self.robot.enable()
        json.dumps(self.robot.snapshot(), allow_nan=False)

    def test_initialization_is_explicit_and_does_not_enable(self):
        self.initialize()
        move = self.transport.calls_for("moveJ")[0]
        np.testing.assert_allclose(move[0], controller.config.HOME_Q)
        self.assertTrue(move[3])
        self.assertFalse(self.robot.snapshot()["enabled"])
        self.assertEqual(self.transport.calls_for("servoJ"), [])
        with self.assertRaises(ValueError):
            self.robot.enable()
        thread_ids = {thread_id for _, thread_id, _ in self.transport.calls}
        self.assertEqual(thread_ids, {self.robot._worker.ident})
        json.dumps(self.robot.snapshot(), allow_nan=False)

    def test_cancel_initial_move_stops_and_requires_initialization_again(self):
        self.transport.finish_home = False
        self.transport.q[2] += 0.2
        self.robot.initialize()
        self.until(lambda: bool(self.transport.calls_for("moveJ")))
        self.assertTrue(self.robot.snapshot()["busy"])
        self.robot.disable("Cancel initialization")
        self.assertFalse(self.robot.snapshot()["enabled"])
        self.until(lambda: not self.robot.snapshot()["busy"])
        self.assertEqual(self.transport.calls_for("stopJ"), [(2.0, True)])
        self.assertFalse(self.robot.snapshot()["initialized"])
        self.assertTrue(self.transport.calls_for("disconnect_control"))
        with self.assertRaises(RuntimeError):
            self.robot.enable()

    def test_close_cancels_initial_move_and_joins_worker(self):
        self.transport.finish_home = False
        self.transport.q[2] += 0.2
        self.robot.initialize()
        self.until(lambda: bool(self.transport.calls_for("moveJ")))
        self.robot.close()
        self.assertFalse(self.robot._worker.is_alive())
        self.assertTrue(self.transport.calls_for("stopJ"))
        self.assertTrue(self.transport.calls_for("disconnect_receive"))
        self.assertFalse(self.robot.snapshot()["busy"])
        with self.assertRaises(RuntimeError):
            self.robot.initialize()

    def test_program_startup_is_polled_before_any_motion(self):
        self.transport.program_running = False
        self.robot.initialize()
        self.until(lambda: bool(self.transport.calls_for("program_running")))
        self.assertTrue(self.robot.snapshot()["busy"])
        self.assertEqual(self.transport.calls_for("moveJ"), [])
        self.assertEqual(self.transport.calls_for("connect_control")[0][2], controller.CONTROL_FLAGS)
        self.transport.program_running = True
        self.until(lambda: self.robot.snapshot()["initialized"])

    def test_cancel_during_connection_cannot_start_a_late_move(self):
        entered, release = threading.Event(), threading.Event()

        def connect(host, frequency, flags):
            entered.set()
            release.wait(1.0)
            return self.transport.control(host, frequency, flags)

        with patch.object(controller, "RTDEControlInterface", connect):
            self.robot.initialize()
            self.assertTrue(entered.wait(1.0))
            self.robot.disable("Cancel while connecting")
            release.set()
            self.until(lambda: not self.robot.snapshot()["busy"])
        self.assertEqual(self.transport.calls_for("moveJ"), [])
        self.assertFalse(self.robot.snapshot()["initialized"])

    def test_initialization_timeout_is_reported_not_success(self):
        self.transport.finish_home = False
        self.transport.q[2] += 0.2
        self.robot.initialize()
        self.until(lambda: bool(self.transport.calls_for("moveJ")))
        self.clock_offset += controller.INITIALIZATION_TIMEOUT + 1
        self.until(lambda: not self.robot.snapshot()["busy"])
        state = self.robot.snapshot()
        self.assertIn("timed out", state["message"])
        self.assertFalse(state["initialized"])
        self.assertFalse(state["enabled"])
        self.assertTrue(self.transport.calls_for("stopJ"))

    def test_watchdog_stops_without_frames_and_never_auto_resumes(self):
        self.enable()
        self.clock_offset += controller.config.TARGET_MAX_AGE + 1
        self.until(lambda: not self.robot.snapshot()["busy"] and not self.robot.snapshot()["enabled"])
        self.assertTrue(self.transport.calls_for("servoStop"))
        self.assertTrue(self.robot.snapshot()["initialized"])
        self.assertIn("expired", self.robot.snapshot()["message"])
        before = len(self.transport.calls_for("servoJ"))
        self.target()
        time.sleep(3 / controller.config.CONTROL_FREQUENCY)
        self.assertEqual(len(self.transport.calls_for("servoJ")), before)
        self.robot.enable()
        self.until(lambda: len(self.transport.calls_for("servoJ")) > before)

    def test_program_startup_timeout_does_not_send_a_move(self):
        self.transport.program_running = False
        self.robot.initialize()
        self.until(lambda: bool(self.transport.calls_for("program_running")))
        self.clock_offset += controller.INITIALIZATION_TIMEOUT + 1
        self.until(lambda: not self.robot.snapshot()["busy"])
        self.assertIn("timed out", self.robot.snapshot()["message"])
        self.assertEqual(self.transport.calls_for("moveJ"), [])

    def test_commands_are_bounded_fixed_and_speed_limited(self):
        self.initialize()
        self.robot.update_target(np.full(6, 100.0), time.monotonic())
        self.robot.enable()
        self.until(lambda: len(self.transport.calls_for("servoJ")) >= 4)
        self.robot.disable()
        self.until(lambda: not self.robot.snapshot()["busy"])
        commands = np.array([args[0] for args in self.transport.calls_for("servoJ")])
        differences = np.diff(np.vstack([controller.config.HOME_Q, commands]), axis=0)
        self.assertTrue(np.all(np.abs(differences) <= controller.config.MAX_JOINT_SPEED / controller.config.CONTROL_FREQUENCY + 1e-12))
        self.assertTrue(np.all(commands >= controller.config.JOINT_MIN))
        self.assertTrue(np.all(commands <= controller.config.JOINT_MAX))
        fixed = [0, 1, 3, 4, 5]
        np.testing.assert_allclose(commands[:, fixed], np.tile(controller.config.HOME_Q[fixed], (len(commands), 1)))
        self.assertGreater(commands[-1, 2], controller.config.HOME_Q[2])

    def test_shoulder_control_can_be_opted_in_without_unlocking_other_joints(self):
        with patch.object(controller.config, "CONTROL_SHOULDER", True):
            robot = controller.RobotController("ursim")
        self.addCleanup(robot.close)
        home = controller.config.HOME_Q.copy()
        step = robot._step(home, home + 1, 0.01)
        self.assertGreater(step[1], home[1])
        self.assertGreater(step[2], home[2])
        np.testing.assert_allclose(step[[0, 3, 4, 5]], home[[0, 3, 4, 5]])

    def test_optional_smoothing_has_known_time_constant(self):
        home = controller.config.HOME_Q.copy()
        target = home.copy()
        target[2] += 0.001
        dt = 0.01
        expected = 0.001 * (1 - math.exp(-dt / controller.config.SMOOTHING_TIME_CONSTANT))
        smoothed = self.robot._step(home, target, dt)
        self.assertAlmostEqual(smoothed[2] - home[2], expected)
        self.robot._tau = 0.0
        np.testing.assert_allclose(self.robot._step(home, target, dt), target)
        np.testing.assert_allclose(self.robot._step(home, target, 0.0), home)

    def test_invalid_target_or_timestamp_disables_and_clears_target(self):
        self.enable()
        for bad in ([0] * 5, [math.nan] * 6, [math.inf] * 6, [[0] * 6]):
            with self.subTest(target=bad):
                with self.assertRaises(ValueError):
                    self.robot.update_target(bad, time.monotonic())
                self.assertFalse(self.robot.snapshot()["enabled"])
        self.until(lambda: not self.robot.snapshot()["busy"])
        for timestamp in (math.nan, math.inf, time.monotonic() + 100, time.monotonic() - 100, True):
            with self.subTest(timestamp=timestamp):
                with self.assertRaises(ValueError):
                    self.robot.update_target(controller.config.HOME_Q, timestamp)
        with self.assertRaises(ValueError):
            self.robot.enable()
        self.assertTrue(self.transport.calls_for("servoStop"))
        json.dumps(self.robot.snapshot(), allow_nan=False)

    def test_latest_observation_wins_and_inputs_are_copied(self):
        self.initialize()
        old = time.monotonic()
        target = self.target(0.2)
        target[2] += 10
        stale_order = controller.config.HOME_Q.copy()
        stale_order[2] -= 0.2
        self.robot.update_target(stale_order, old)
        with self.robot._condition:
            self.assertAlmostEqual(self.robot._target[2], controller.config.HOME_Q[2] + 0.2)
        snapshot = self.robot.snapshot()
        snapshot["actual_q"][0] = 100
        self.assertNotEqual(self.robot.snapshot()["actual_q"][0], 100)

    def test_transport_failure_stops_and_requires_explicit_reinitialization(self):
        self.initialize()
        self.transport.servo_error = True
        self.target()
        self.robot.enable()
        self.until(lambda: not self.robot.snapshot()["busy"] and "simulated servo transport failure" in self.robot.snapshot()["message"])
        state = self.robot.snapshot()
        self.assertFalse(state["initialized"])
        self.assertFalse(state["enabled"])
        self.assertTrue(self.transport.calls_for("servoStop"))
        self.assertTrue(self.transport.calls_for("disconnect_control"))
        self.target()
        with self.assertRaises(RuntimeError):
            self.robot.enable()

    def test_rejected_home_and_lost_connection_are_reported(self):
        self.transport.reject_home = True
        self.robot.initialize()
        self.until(lambda: not self.robot.snapshot()["busy"] and "rejected" in self.robot.snapshot()["message"])
        self.assertFalse(self.robot.snapshot()["initialized"])
        self.transport.reject_home = False
        self.initialize()
        self.transport.connected = False
        self.until(lambda: not self.robot.snapshot()["busy"] and "connection lost" in self.robot.snapshot()["message"])
        self.assertFalse(self.robot.snapshot()["initialized"])

    def test_invalid_config_is_rejected_before_connecting(self):
        cases = {
            "MAX_JOINT_SPEED": [0, -1, math.inf],
            "SMOOTHING_TIME_CONSTANT": [-1, math.nan],
            "CONTROL_FREQUENCY": [0, 0.1, 500],
            "TARGET_MAX_AGE": [0, math.nan],
            "HOME_Q": [[0] * 5, [math.nan] * 6],
            "JOINT_MIN": [controller.config.HOME_Q + 1],
            "CONTROL_SHOULDER": ["yes"],
        }
        for name, values in cases.items():
            for value in values:
                with self.subTest(name=name, value=value), patch.object(controller.config, name, value):
                    with self.assertRaises(ValueError):
                        controller.RobotController("ursim")
        self.assertEqual(self.transport.calls, [])


if __name__ == "__main__":
    unittest.main()
