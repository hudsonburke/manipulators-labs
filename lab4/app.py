"""Serve the Lab 4 browser camera and control application on one private port."""

from __future__ import annotations

import argparse
import base64
from collections import deque
import csv
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
from threading import Lock
import time
from urllib.parse import urlsplit

# Bound decompression before OpenCV allocates an image from an uploaded JPEG.
os.environ["OPENCV_IO_MAX_IMAGE_PIXELS"] = str(1280 * 960)

import cv2
import numpy as np

if __package__:
    from . import student_config as config
    from .controller import RobotController
    from .student import joint_targets
    from .vision import PoseTracker
else:
    import student_config as config
    from controller import RobotController
    from student import joint_targets
    from vision import PoseTracker

LAB_DIR = Path(__file__).resolve().parent
MAX_FRAME_BYTES = 1_000_000


class CameraApp:
    def __init__(self, host: str):
        self.tracker = PoseTracker()
        self.robot = RobotController(host)
        self.token = secrets.token_urlsafe(32)
        self.lock = Lock()
        self.inference_lock = Lock()
        self.session = None
        self.neutral = None
        self.angles = None
        self.observed_at = float('-inf')
        self.target = None
        self.jpeg = None
        self.started_at = time.monotonic()
        # shortcut: retain the latest 6000 frames; use streaming logs for longer trials.
        self.log = deque(maxlen=6000)

    def check_session(self, session):
        if not session or session != self.session:
            raise ValueError('Camera session expired. Start the camera again; use one tab per application.')

    def status(self):
        with self.lock:
            return self._status()

    def _status(self):
        return {
            'tracking': self.angles is not None and time.monotonic() - self.observed_at <= config.TARGET_MAX_AGE,
            'calibrated': self.neutral is not None,
            'angles': self.angles,
            'target_q': self.target.tolist() if self.target is not None else None,
            'controller': self.robot.snapshot(),
        }

    def action(self, action, session):
        if action == 'disable':
            self.robot.disable('Stopped by user. Enable again explicitly to resume.')
            return self.status()
        with self.lock:
            if action == 'session':
                self.robot.disable('New camera session; robot control is disabled.')
                self.session = secrets.token_urlsafe(24)
                self.neutral = self.angles = self.target = self.jpeg = None
                self.observed_at = float('-inf')
                self.log.clear()
                self.started_at = time.monotonic()
                return {'session': self.session, **self._status()}
            self.check_session(session)
            if action == 'initialize':
                self.neutral = None
                self.target = None
                self.robot.initialize()
            elif action == 'calibrate':
                state = self._status()
                if state['controller']['enabled'] or state['controller']['busy']:
                    raise ValueError('Stop robot motion before calibrating.')
                if not state['tracking']:
                    raise ValueError('Show your left shoulder, elbow, and wrist clearly before calibrating.')
                self.neutral = dict(self.angles)
                self.target = joint_targets(self.angles, self.neutral)
                self.robot.update_target(self.target, self.observed_at)
            elif action == 'enable':
                state = self._status()
                if not state['tracking'] or not state['calibrated']:
                    raise ValueError('A calibrated, fresh pose is required to enable robot control.')
                self.robot.update_target(self.target, self.observed_at)
                self.robot.enable()
            else:
                raise ValueError('Unknown action.')
            return self._status()

    def process(self, data: bytes, session: str, observed_at: float):
        with self.lock:
            self.check_session(session)
        with self.inference_lock:
            frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None or frame.shape[0] > 960 or frame.shape[1] > 1280:
                raise ValueError('Send a JPEG frame no larger than 1280 x 960.')
            annotated, angles, message = self.tracker.process(frame)
            with self.lock:
                self.check_session(session)
                self.angles = angles
                self.observed_at = observed_at
                self.target = None
                if angles is None:
                    self.robot.disable(message)
                elif self.neutral is not None:
                    self.target = joint_targets(angles, self.neutral)
                    self.robot.update_target(self.target, observed_at)
                state = self._status()
                target_text = 'Calibrate to preview robot targets.' if self.target is None else (
                    'Target [rad]: ' + ', '.join(f'{q:.2f}' for q in self.target)
                )
                annotated = cv2.copyMakeBorder(annotated, 0, 62, 0, 0, cv2.BORDER_CONSTANT, value=(25, 25, 25))
                cv2.putText(annotated, target_text, (10, annotated.shape[0] - 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
                cv2.putText(annotated, 'URSim: ' + state['controller']['message'][:90], (10, annotated.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                ok, encoded = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    raise RuntimeError('Could not encode the processed camera image.')
                self.jpeg = encoded.tobytes()
                actual = state['controller']['actual_q']
                self.log.append([
                    time.monotonic() - self.started_at, state['tracking'], state['controller']['enabled'],
                    angles['elbow'] if angles else '', angles['shoulder'] if angles else '',
                    *(self.target.tolist() if self.target is not None else [''] * 6),
                    *(actual if actual is not None else [''] * 6),
                ])
                return {
                    **state, 'message': message,
                    'processing_ms': round((time.monotonic() - observed_at) * 1000),
                    'image': 'data:image/jpeg;base64,' + base64.b64encode(self.jpeg).decode('ascii'),
                }

    def save_evidence(self, task: str, session: str):
        with self.lock:
            self.check_session(session)
            if task not in ('1', '2', '3') or self.jpeg is None:
                raise ValueError('Choose Task 1, 2, or 3 after processing a camera frame.')
            output = LAB_DIR / 'outputs'
            output.mkdir(exist_ok=True)
            filename = f'task{task}_preview.jpg'
            (output / filename).write_bytes(self.jpeg)
            if task == '3':
                with (output / 'task3_control.csv').open('w', newline='') as stream:
                    writer = csv.writer(stream)
                    writer.writerow(['time_s', 'tracking', 'enabled', 'human_elbow_rad', 'human_shoulder_rad',
                                     *[f'target_q{i}' for i in range(1, 7)], *[f'actual_q{i}' for i in range(1, 7)]])
                    writer.writerows(self.log)
            return {'message': f'Saved lab4/outputs/{filename}' + (' and task3_control.csv.' if task == '3' else '.')}

    def close(self):
        self.robot.close()
        with self.inference_lock:
            self.tracker.close()


class CameraServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, app):
        self.app = app
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(5)

    def log_message(self, format, *args):
        pass

    def reply(self, status, body, content_type='application/json'):
        if content_type == 'application/json':
            body = json.dumps(body, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.send_header('Permissions-Policy', 'camera=(self), microphone=()')
        self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' data:; media-src 'self' blob:; frame-ancestors 'none'")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def authorized(self):
        return secrets.compare_digest(self.headers.get('X-Lab-Token', ''), self.server.app.token)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == '/api/status':
            if not self.authorized():
                self.reply(403, {'error': 'Reload the application to obtain an access token.'})
            else:
                self.reply(200, self.server.app.status())
        elif path in ('/', '/app.js', '/style.css'):
            filename = 'index.html' if path == '/' else path[1:]
            body = (LAB_DIR / 'static' / filename).read_bytes()
            if path == '/':
                body = body.replace(b'__LAB_TOKEN__', self.server.app.token.encode())
            content_type = {'/': 'text/html; charset=utf-8', '/app.js': 'text/javascript; charset=utf-8', '/style.css': 'text/css; charset=utf-8'}[path]
            self.reply(200, body, content_type)
        else:
            self.reply(404, {'error': 'Not found.'})

    def do_POST(self):
        app = self.server.app
        if not self.authorized():
            self.reply(403, {'error': 'Reload the application to obtain an access token.'})
            return
        observed_at = time.monotonic()
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 <= length <= MAX_FRAME_BYTES:
                self.reply(413, {'error': 'Request exceeds the 1 MB frame limit.'})
                return
            data = self.rfile.read(length)
            if len(data) != length:
                raise ValueError('Incomplete request body.')
            session = self.headers.get('X-Camera-Session', '')
            path = urlsplit(self.path).path
            if path == '/api/frame':
                if self.headers.get_content_type() != 'image/jpeg':
                    raise ValueError('Camera frames must use image/jpeg.')
                result = app.process(data, session, observed_at)
            elif path == '/api/evidence':
                body = json.loads(data or b'{}')
                if not isinstance(body, dict):
                    raise ValueError('Expected a JSON object.')
                result = app.save_evidence(str(body.get('task', '')), session)
            elif path in ('/api/session', '/api/initialize', '/api/calibrate', '/api/enable', '/api/disable'):
                result = app.action(path.rsplit('/', 1)[1], session)
            else:
                self.reply(404, {'error': 'Not found.'})
                return
            self.reply(200, result)
        except (ValueError, RuntimeError, cv2.error) as exc:
            if urlsplit(self.path).path == '/api/frame':
                app.robot.disable('Frame processing failed; robot control disabled.')
            self.reply(400, {'error': str(exc)})
        except (OSError, TimeoutError) as exc:
            app.robot.disable('Camera connection or evidence write failed.')
            self.reply(503, {'error': str(exc)})
        except Exception as exc:
            app.robot.disable('Unexpected processing error; robot control disabled.')
            print(f'Lab 4 request failed: {exc}', flush=True)
            self.reply(500, {'error': 'Processing failed. Check the server terminal.'})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8000)
    args = parser.parse_args()
    app = CameraApp(os.environ.get('URSIM_HOST', 'localhost'))
    try:
        server = CameraServer(('0.0.0.0', args.port), app)
        name = os.environ.get('CODESPACE_NAME')
        domain = os.environ.get('GITHUB_CODESPACES_PORT_FORWARDING_DOMAIN')
        url = f'https://{name}-{args.port}.{domain}' if name and domain else f'http://localhost:{args.port}'
        print(f'Open Lab 4 in a normal browser tab: {url}/', flush=True)
        print('Robot control is disabled. Camera frames are not saved unless you choose Save evidence.', flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print('\nStopping Lab 4.', flush=True)
        finally:
            server.server_close()
    finally:
        app.close()


if __name__ == '__main__':
    main()
