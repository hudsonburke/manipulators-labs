"""Opt-in, simulation-only RTDE control; the worker owns every driver call."""

import math
import threading
import time

import numpy as np
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

import student_config as config


INITIALIZATION_TIMEOUT = 60.0
CLOSE_TIMEOUT = 10.0
CONTROL_FLAGS = RTDEControlInterface.FLAG_UPLOAD_SCRIPT | RTDEControlInterface.FLAG_NO_WAIT


def _joints(value, name):
    try:
        q = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain six finite joint angles") from exc
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError(f"{name} must contain six finite joint angles")
    return q.copy()


def _number(value, name, *, allow_zero=False):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be finite and positive") from exc
    if not math.isfinite(number) or number < 0 or (number == 0 and not allow_zero):
        raise ValueError(f"{name} must be finite and {'nonnegative' if allow_zero else 'positive'}")
    return number


class RobotController:
    def __init__(self, host):
        if not isinstance(host, str) or not host.strip() or any(ord(c) < 32 for c in host):
            raise ValueError("A valid URSim hostname is required")
        self._host = host.strip()
        self._home = _joints(config.HOME_Q, "HOME_Q")
        self._lower = _joints(config.JOINT_MIN, "JOINT_MIN")
        self._upper = _joints(config.JOINT_MAX, "JOINT_MAX")
        if np.any(self._lower > self._home) or np.any(self._home > self._upper):
            raise ValueError("Joint bounds must include HOME_Q")
        if not isinstance(config.CONTROL_SHOULDER, (bool, np.bool_)):
            raise ValueError("CONTROL_SHOULDER must be a boolean")
        inactive = [0, 3, 4, 5] if config.CONTROL_SHOULDER else [0, 1, 3, 4, 5]
        self._lower[inactive] = self._upper[inactive] = self._home[inactive]
        self._speed = _number(config.MAX_JOINT_SPEED, "MAX_JOINT_SPEED")
        self._tau = _number(config.SMOOTHING_TIME_CONSTANT, "SMOOTHING_TIME_CONSTANT", allow_zero=True)
        self._max_age = _number(config.TARGET_MAX_AGE, "TARGET_MAX_AGE")
        self._frequency = _number(config.CONTROL_FREQUENCY, "CONTROL_FREQUENCY")
        if not 1 <= self._frequency <= 125:
            raise ValueError("CONTROL_FREQUENCY must be between 1 and 125 Hz for the CB3 UR5")
        self._period = 1.0 / self._frequency
        self._condition = threading.Condition()
        self._closed = self._initialized = self._enabled = self._busy = False
        self._initialize_requested = self._stop_requested = False
        self._epoch = 0
        self._message = "Disconnected. Initialize explicitly to connect and move to HOME."
        self._actual = self._command = self._target = self._target_at = None
        self._last_command_at = None
        self._control = self._receive = None
        self._motion = None
        self._worker = threading.Thread(target=self._run, name="lab4-rtde", daemon=True)
        self._worker.start()

    def initialize(self):
        with self._condition:
            if self._closed:
                raise RuntimeError("Controller is closed")
            if self._busy or self._stop_requested:
                raise RuntimeError("Wait for the current operation to finish")
            if self._enabled:
                raise RuntimeError("Disable control before initializing")
            self._epoch += 1
            self._initialized = False
            self._busy = self._initialize_requested = True
            self._target = self._target_at = None
            self._message = "Connecting to URSim and moving to HOME."
            self._condition.notify_all()

    def enable(self):
        with self._condition:
            if self._closed:
                raise RuntimeError("Controller is closed")
            if not self._initialized or self._busy or self._stop_requested:
                raise RuntimeError("Initialize successfully before enabling control")
            now = time.monotonic()
            if self._target_at is None or not 0 <= now - self._target_at <= self._max_age:
                raise ValueError("A fresh, valid calibrated target is required")
            if self._enabled:
                return
            self._command = np.clip(self._actual, self._lower, self._upper)
            self._last_command_at = now
            self._enabled = True
            self._message = "Control enabled."

    def update_target(self, q, observed_at):
        try:
            target = _joints(q, "Target")
            if isinstance(observed_at, (bool, np.bool_)):
                raise ValueError("Observation time must be a monotonic timestamp")
            observed_at = float(observed_at)
            age = time.monotonic() - observed_at
            if not math.isfinite(observed_at) or not 0 <= age <= self._max_age:
                raise ValueError("Target observation is stale or its timestamp is invalid")
        except (TypeError, ValueError, OverflowError) as exc:
            self.disable(f"Invalid target: {exc}")
            raise ValueError(str(exc)) from exc
        with self._condition:
            if self._closed:
                raise RuntimeError("Controller is closed")
            if self._target_at is None or observed_at >= self._target_at:
                self._target = np.clip(target, self._lower, self._upper)
                self._target_at = observed_at

    def disable(self, reason="Control disabled."):
        with self._condition:
            if self._closed:
                return
            self._epoch += 1
            self._stop_requested |= self._enabled or self._busy
            self._busy |= self._enabled
            self._enabled = self._initialize_requested = False
            self._target = self._target_at = None
            self._message = str(reason)
            self._condition.notify_all()

    def snapshot(self):
        with self._condition:
            return {
                "initialized": self._initialized,
                "enabled": self._enabled,
                "busy": self._busy,
                "message": self._message,
                "actual_q": None if self._actual is None else self._actual.tolist(),
                "command_q": None if self._command is None else self._command.tolist(),
            }

    def close(self):
        with self._condition:
            self._closed = True
            self._enabled = self._initialized = False
            self._target = self._target_at = None
            self._epoch += 1
            self._condition.notify_all()
        self._worker.join(CLOSE_TIMEOUT)
        if self._worker.is_alive():
            raise RuntimeError("RTDE worker did not stop; a driver call is still blocked")

    def _cancelled(self, epoch):
        with self._condition:
            return self._closed or epoch != self._epoch

    def _stop_motion(self):
        motion, self._motion = self._motion, None
        if self._control is not None:
            if motion == "home":
                self._control.stopJ(2.0, True)
            elif motion == "servo" and self._control.servoStop(2.0) is False:
                raise RuntimeError("URSim rejected servoStop")

    def _release(self):
        errors = []
        try:
            self._stop_motion()
        except Exception as exc:
            errors.append(str(exc))
        if self._control is not None:
            for method in (self._control.stopScript, self._control.disconnect):
                try:
                    method()
                except Exception as exc:
                    errors.append(str(exc))
        if self._receive is not None:
            try:
                self._receive.disconnect()
            except Exception as exc:
                errors.append(str(exc))
        self._control = self._receive = None
        return errors

    def _step(self, command, target, dt):
        alpha = 1.0 if self._tau == 0 else -math.expm1(-dt / self._tau)
        delta = np.clip(alpha * (target - command), -self._speed * dt, self._speed * dt)
        return np.clip(command + delta, self._lower, self._upper)

    def _run(self):
        next_tick = time.monotonic()
        home_deadline = None
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(
                        lambda: self._closed or self._stop_requested or self._initialize_requested,
                        timeout=max(0.0, next_tick - time.monotonic()),
                    )
                    if self._closed:
                        break
                    stop, self._stop_requested = self._stop_requested, False
                    initialize, self._initialize_requested = self._initialize_requested, False
                    epoch = self._epoch
                tick = time.monotonic()
                next_tick = tick + self._period
                try:
                    if stop:
                        self._stop_motion()
                        with self._condition:
                            initialized = self._initialized
                        if not initialized:
                            errors = self._release()
                            if errors:
                                raise RuntimeError("; ".join(errors))
                        with self._condition:
                            if epoch == self._epoch:
                                self._busy = False
                    if initialize:
                        home_deadline = time.monotonic() + INITIALIZATION_TIMEOUT
                        self._motion = "starting"
                        if self._receive is None:
                            self._receive = RTDEReceiveInterface(self._host, self._frequency)
                    if self._cancelled(epoch):
                        continue
                    if initialize and self._control is None:
                        self._control = RTDEControlInterface(self._host, self._frequency, CONTROL_FLAGS)
                    if self._receive is None:
                        continue
                    if not self._receive.isConnected():
                        raise RuntimeError("RTDE feedback connection lost")
                    if self._control is None or not self._control.isConnected():
                        raise RuntimeError("RTDE control connection lost")
                    now = time.monotonic()
                    if self._motion in ("starting", "home") and now >= home_deadline:
                        raise RuntimeError("Initialization timed out before reaching HOME")
                    if not self._control.isProgramRunning():
                        if self._motion == "starting":
                            continue
                        raise RuntimeError("URSim control program is not running")
                    if self._motion == "starting":
                        if self._cancelled(epoch):
                            continue
                        self._motion = "home"
                        if self._control.moveJ(self._home.tolist(), self._speed, 0.5, True) is False:
                            raise RuntimeError("URSim rejected the move to HOME")
                    actual = _joints(self._receive.getActualQ(), "Received joints")
                    velocity = _joints(self._receive.getActualQd(), "Received joint speeds")
                    with self._condition:
                        self._actual = actual
                    if self._motion == "home":
                        at_home = np.max(np.abs(actual - self._home)) <= 0.01
                        stationary = np.max(np.abs(velocity)) <= 0.02
                        if at_home and stationary and self._control.getAsyncOperationProgress() < 0:
                            with self._condition:
                                if epoch == self._epoch and not self._closed:
                                    self._motion = None
                                    self._initialized = True
                                    self._busy = False
                                    self._command = self._home.copy()
                                    self._message = "At HOME. Calibrate, then explicitly enable control."
                    with self._condition:
                        if not self._enabled or epoch != self._epoch:
                            continue
                        if self._target_at is None or not 0 <= now - self._target_at <= self._max_age:
                            self.disable("Tracking target expired; control disabled. Enable explicitly to resume.")
                            continue
                        dt = min(max(0.0, now - self._last_command_at), self._period)
                        command = self._step(self._command, self._target, dt)
                    if self._cancelled(epoch):
                        continue
                    self._motion = "servo"
                    if self._control.servoJ(command.tolist(), 0.0, 0.0, self._period, 0.1, 300) is False:
                        raise RuntimeError("URSim rejected a servo target")
                    with self._condition:
                        self._command = command
                        self._last_command_at = now
                except Exception as exc:
                    with self._condition:
                        self._epoch += 1
                        self._enabled = self._initialized = False
                        self._busy = True
                        self._initialize_requested = self._stop_requested = False
                        self._target = self._target_at = None
                        self._message = f"RTDE error: {exc}"
                    errors = self._release()
                    with self._condition:
                        self._busy = False
                        if errors:
                            self._message += f"; cleanup: {'; '.join(errors)}"
        finally:
            errors = self._release()
            with self._condition:
                self._enabled = self._initialized = self._busy = False
                self._message = "Closed." + (f" Cleanup errors: {'; '.join(errors)}" if errors else "")
