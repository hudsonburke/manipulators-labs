"""Task 3: move URSim with your elbow angle.

Run from lab4 with URSim started: uv run python task3_control.py [--camera N]
"""

import argparse
import math
import os
import sys
import time

from lab4lib.camera import Camera
from lab4lib.controller import RobotController
from lab4lib.evidence import save_control_log, save_preview
from lab4lib.overlay import with_status
from lab4lib.session import LOG_COLUMNS, ControlSession
from lab4lib.vision import PoseTracker

KEYS = {ord('i'): 'initialize', ord('c'): 'calibrate', ord('e'): 'enable',
        ord('s'): 'stop', ord(' '): 'stop'}


def status_lines(session, message, processing_ms):
    state = session.snapshot()
    angles = session.angles
    target = session.target

    def joints(values):
        return 'unavailable' if values is None else ', '.join(f'{q:.3f}' for q in values)

    return [
        'Task 3 - URSim control',
        message,
        'Human elbow [deg]: ' + ('unavailable' if angles is None
                                 else f"{math.degrees(angles['elbow']):.1f}"),
        'Target q1..q6 [rad]: ' + ('calibrate to preview' if target is None else joints(target)),
        'Actual q1..q6 [rad]: ' + joints(state['actual_q']),
        f"URSim: initialized={state['initialized']} enabled={state['enabled']} "
        f"busy={state['busy']} | {processing_ms:.0f} ms read + processing",
        state['message'],
        'I: HOME | C: calibrate | E: enable | Space/S: stop',
        '3: save Task 3 evidence | Keys work while this window is focused',
        'Action: ' + session.feedback,
    ]


def demo_warning(host):
    """Warn when the target is not a local simulator, i.e. a real controller demo."""
    if host in ('localhost', '127.0.0.1', '::1'):
        return None
    return (f'Targeting {host}, which is not a local simulator. Confirm the robot is in remote '
            'control mode, its workspace is clear, and the emergency stop is within reach.')


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Task 3: move a UR controller or URSim with your elbow angle.')
    parser.add_argument('--camera', type=int, default=0, help='camera index (default: 0)')
    parser.add_argument('--host', default=os.environ.get('URSIM_HOST', 'localhost'),
                        help='UR controller or URSim host name/IP (default: $URSIM_HOST, else localhost)')
    args = parser.parse_args(argv)
    host = args.host.strip()
    if not host or '://' in host or '/' in host:
        parser.error('--host must be a host name or IP address, not a URL')
    warning = demo_warning(host)
    if warning:
        print(warning, file=sys.stderr, flush=True)
    print(f'Task 3 target controller: {host}', flush=True)
    camera = tracker = session = None
    try:
        tracker = PoseTracker()
        session = ControlSession(RobotController(host))
        camera = Camera(args.camera, 'Lab 4 - Task 3')
        while True:
            observed_at = time.monotonic()
            try:
                annotated, angles, message = tracker.process(camera.read())
            except (ValueError, RuntimeError) as exc:
                session.stop('Frame processing failed; robot control disabled.')
                raise RuntimeError(f'Frame processing failed: {exc}') from exc
            session.observe(angles, observed_at, message)
            shown = with_status(annotated, status_lines(
                session, message, (time.monotonic() - observed_at) * 1000))
            key = camera.show(shown)
            if key in (27, ord('q'), ord('Q')) or camera.closed():
                return 0
            try:
                if key == ord('3'):
                    print('Saved', save_preview(shown, 3),
                          save_control_log(LOG_COLUMNS, session.log), flush=True)
                else:
                    action = KEYS.get(key + 32 if 65 <= key <= 90 else key)
                    if action:
                        getattr(session, action)()
            except (ValueError, RuntimeError, OSError) as exc:
                session.stop('Action failed; robot control disabled. Enable again explicitly to resume.')
                session.feedback = f'ERROR: {exc}'
                print(f'ERROR: {exc}', file=sys.stderr, flush=True)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f'Task 3 error: {exc}', file=sys.stderr, flush=True)
        return 1
    finally:
        if session is not None:
            try:
                session.close()
            except Exception as exc:
                print(f'Task 3 cleanup error: {exc}', file=sys.stderr, flush=True)
        if camera is not None:
            camera.close()
        if tracker is not None:
            tracker.close()


if __name__ == '__main__':
    raise SystemExit(main())
