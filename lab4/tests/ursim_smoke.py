"""Standalone CI-only motion scenario for the compose URSim, never unittest discovery.

Run in a disposable simulator only:
    LAB4_URSIM_CI=1 uv run python tests/ursim_smoke.py
This powers on/releases brakes and moves the simulated robot. check_setup.py does not.
The CI workflow first confirms the fresh installation through the real PolyScope
safety dialog; Dashboard RUNNING/NORMAL alone does not confirm that installation.
The workflow supplies an outer process deadline for native driver calls.
"""

import os
from pathlib import Path
import socket
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from check_setup import PORTS, read_dashboard_line


HOST = '127.0.0.1'  # Deliberately not configurable or inherited from URSIM_HOST.


class Dashboard:
    def __enter__(self):
        self.connection = socket.create_connection((HOST, 29999), timeout=2)
        try:
            banner = read_dashboard_line(self.connection, 2)
            if 'dashboard' not in banner.lower():
                raise RuntimeError(f'Unexpected dashboard banner: {banner!r}')
        except BaseException:
            self.connection.close()
            raise
        return self

    def command(self, command):
        self.connection.settimeout(2)
        self.connection.sendall((command + '\n').encode('ascii'))
        reply = read_dashboard_line(self.connection, 2)
        print(f'Dashboard {command}: {reply}', flush=True)
        return reply

    def __exit__(self, *args):
        self.connection.close()


def wait_until(label, predicate, timeout=20, tick=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if tick is not None:
            tick()
        if predicate():
            print(f'PASS: {label}', flush=True)
            return
        time.sleep(0.04)
    raise AssertionError(f'Timed out after {timeout:g}s: {label}')


def prepare_simulator():
    deadline = time.monotonic() + 150
    last_error = None
    while time.monotonic() < deadline:
        try:
            for port, _ in PORTS:
                with socket.create_connection((HOST, port), timeout=1):
                    pass
            with Dashboard() as dashboard:
                mode = dashboard.command('robotmode')
                safety = dashboard.command('safetymode')
                if mode not in ('Robotmode: BOOTING', 'Robotmode: NO_CONTROLLER'):
                    break
        except (OSError, ValueError) as exc:
            last_error = exc
        time.sleep(2)
    else:
        raise RuntimeError(f'URSim interfaces did not become ready within 150s: {last_error}')
    if safety != 'Safetymode: NORMAL':
        raise RuntimeError(f'Fresh simulator has unexpected safety state: {safety}; not auto-clearing it')
    with Dashboard() as dashboard:
        if mode == 'Robotmode: POWER_OFF':
            if dashboard.command('power on') != 'Powering on':
                raise RuntimeError('Simulator refused power on')
        wait_until('simulator powered on',
                   lambda: dashboard.command('robotmode') in ('Robotmode: IDLE', 'Robotmode: RUNNING'),
                   timeout=30)
        if dashboard.command('robotmode') != 'Robotmode: RUNNING':
            if dashboard.command('brake release') != 'Brake releasing':
                raise RuntimeError('Simulator refused brake release')
        wait_until('simulator RUNNING with brakes released',
                   lambda: dashboard.command('robotmode') == 'Robotmode: RUNNING', timeout=30)
        if dashboard.command('safetymode') != 'Safetymode: NORMAL':
            raise RuntimeError('Simulator is not in NORMAL safety mode')


def exercise_motion():
    # No student angle mapping, MediaPipe, OpenCV, camera, or ControlSession imports.
    import numpy as np
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface

    import student_config as config
    from lab4lib.controller import RobotController, CONTROL_FLAGS

    receive = seed_control = robot = None
    try:
        receive = RTDEReceiveInterface(HOST, config.CONTROL_FREQUENCY)
        print(f'Initial actual_q: {receive.getActualQ()}', flush=True)
        seed_control = RTDEControlInterface(HOST, config.CONTROL_FREQUENCY, CONTROL_FLAGS)
        wait_until('seed control program running', seed_control.isProgramRunning, timeout=30)
        seed = np.array(config.HOME_Q, dtype=float)
        seed[2] += 0.22
        if seed_control.moveJ(seed.tolist(), 0.25, 0.5, True) is False:
            raise AssertionError('Simulator rejected seed move')

        def arrived(q, tolerance=0.015):
            return (np.max(np.abs(np.array(receive.getActualQ()) - q)) < tolerance
                    and np.max(np.abs(receive.getActualQd())) < 0.02)

        wait_until('real simulator joints reach non-HOME seed', lambda: arrived(seed), timeout=45)
        nonhome = np.array(receive.getActualQ())
        if np.max(np.abs(nonhome - config.HOME_Q)) < 0.18:
            raise AssertionError(f'Seed is not sufficiently different from HOME: {nonhome}')
        seed_control.stopScript()
        seed_control.disconnect()
        seed_control = None

        robot = RobotController(HOST)
        robot.initialize()
        previous_message = None

        def state():
            nonlocal previous_message
            snapshot = robot.snapshot()
            if snapshot['message'] != previous_message:
                print(f'Controller: {snapshot}', flush=True)
                previous_message = snapshot['message']
            if snapshot['message'].startswith('RTDE error:'):
                raise AssertionError(snapshot)
            return snapshot

        wait_until('RobotController reports At HOME after actual movement',
                   lambda: state()['initialized'] and not state()['busy'], timeout=75)
        home_actual = np.array(receive.getActualQ())
        if not arrived(config.HOME_Q) or np.max(np.abs(home_actual - nonhome)) < 0.18:
            raise AssertionError(f'HOME did not move actual joints: {nonhome} -> {home_actual}')
        print(f'PASS: HOME actual_q {nonhome.tolist()} -> {home_actual.tolist()}', flush=True)

        positive = np.array(config.HOME_Q, dtype=float)
        negative = positive.copy()
        positive[2] += 0.18
        negative[2] -= 0.18

        def fresh(target):
            robot.update_target(target, time.monotonic())
            state()

        def follow(target, label):
            wait_until(label, lambda: arrived(target, 0.035), timeout=15,
                       tick=lambda: fresh(target))
            if not state()['enabled']:
                raise AssertionError('Fresh tracking unexpectedly disarmed control')

        def hold_still(label, duration, tick=None):
            baseline = np.array(receive.getActualQ())
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if tick:
                    tick()
                state()
                if np.max(np.abs(np.array(receive.getActualQ()) - baseline)) > 0.025:
                    raise AssertionError(f'{label}: actual joints drifted from {baseline.tolist()} '
                                         f'to {receive.getActualQ()}')
                time.sleep(0.04)
            print(f'PASS: {label}', flush=True)

        fresh(positive)
        robot.enable()
        follow(positive, 'fresh synthetic target moves actual joints')
        wait_until('actual joints moving before missing-pose pause',
                   lambda: receive.getActualQ()[2] < positive[2] - 0.02,
                   tick=lambda: fresh(negative))
        robot.pause_tracking()
        wait_until('missing pose pauses tracking', lambda: 'Tracking paused' in state()['message'])
        if not state()['enabled']:
            raise AssertionError('Tracking pause disarmed control')
        time.sleep(0.2)  # Allow servoStop deceleration before measuring standstill.
        hold_still('tracking pause holds actual joints', 0.6)
        follow(negative, 'fresh target resumes automatically without enable')
        wait_until('actual joints moving before watchdog pause',
                   lambda: receive.getActualQ()[2] > negative[2] + 0.02,
                   tick=lambda: fresh(positive))

        wait_until('stale target watchdog pauses tracking',
                   lambda: 'Tracking paused' in state()['message'], timeout=config.TARGET_MAX_AGE + 3)
        if not state()['enabled']:
            raise AssertionError('Stale tracking disarmed control')
        time.sleep(0.2)
        hold_still('watchdog pause holds actual joints', 0.5)
        follow(positive, 'fresh target resumes after watchdog pause')

        robot.disable('Explicit CI stop; require explicit enable to resume.')
        wait_until('explicit stop disarms and finishes',
                   lambda: not state()['enabled'] and not state()['busy'])
        time.sleep(0.2)
        hold_still('fresh targets cannot rearm explicit stop', 1.0, tick=lambda: fresh(negative))
        if state()['enabled']:
            raise AssertionError('Fresh target rearmed an explicit stop')
        fresh(negative)
        robot.enable()
        follow(negative, 'explicit enable restores following after stop')
        print('PASS: all simulator motion scenarios', flush=True)
    finally:
        try:
            if robot is not None:
                print(f'Final controller snapshot: {robot.snapshot()}', flush=True)
                robot.close()
        finally:
            try:
                if seed_control is not None:
                    seed_control.stopJ(2.0, True)
                    seed_control.stopScript()
                    seed_control.disconnect()
            finally:
                if receive is not None:
                    print(f'Final actual_q: {receive.getActualQ()}', flush=True)
                    receive.disconnect()
                with Dashboard() as dashboard:
                    for command in ('robotmode', 'safetymode', 'programState'):
                        dashboard.command(command)


def main():
    if os.environ.get('LAB4_URSIM_CI') != '1':
        raise SystemExit('Refusing motion. Use only the disposable compose URSim with LAB4_URSIM_CI=1.')
    print(f'Disposable URSim integration only: {HOST}; powering on and moving simulated joints.', flush=True)
    prepare_simulator()
    exercise_motion()


if __name__ == '__main__':
    main()
