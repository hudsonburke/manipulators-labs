"""Optional same-origin browser camera transport; all lab computations stay in Python.

Run from lab4: uv run --extra browser python web_lab.py --host ursim_cb3
Use a private HTTPS forwarded port, never a public port or a physical controller.
"""

import argparse
import asyncio
from concurrent.futures import ThreadPoolExecutor
from contextlib import suppress
from io import BytesIO
import json
import logging
import math
import os
from pathlib import Path
import time
from urllib.parse import urlsplit
import warnings

from aiohttp import WSMsgType, web
import cv2
import numpy as np
from PIL import Image

import student_config as config
from lab4lib.evidence import OUTPUT, save_control_log, save_preview
from lab4lib.overlay import with_status
from lab4lib.session import LOG_COLUMNS, ControlSession
from task3_control import status_lines

MAX_JPEG_BYTES = 256 * 1024
MAX_WIDTH, MAX_HEIGHT = 640, 480
SIMULATOR_HOSTS = frozenset(('localhost', '127.0.0.1', '::1', 'ursim_cb3'))
EVIDENCE_NAMES = frozenset(('task1_preview.jpg', 'task2_preview.jpg',
                            'task3_preview.jpg', 'task3_control.csv'))
WEB_ROOT = Path(__file__).resolve().parent / 'web'
LOGGER = logging.getLogger('lab4.browser')
SECURITY_HEADERS = {
    'Content-Security-Policy': "default-src 'none'; script-src 'self'; style-src 'self'; "
                               "img-src 'self' blob:; connect-src 'self'; "
                               "base-uri 'none'; form-action 'none'; frame-ancestors 'none'",
    'Permissions-Policy': 'camera=(self), microphone=(), geolocation=()',
    'X-Content-Type-Options': 'nosniff',
    'X-Frame-Options': 'DENY',
    'Referrer-Policy': 'no-referrer',
    'Cross-Origin-Opener-Policy': 'same-origin',
    'Cross-Origin-Resource-Policy': 'same-origin',
    'Cache-Control': 'no-store',
}


def _simulator_host(host):
    if not isinstance(host, str) or host not in SIMULATOR_HOSTS:
        raise ValueError('Browser control accepts only localhost, 127.0.0.1, ::1, or ursim_cb3; '
                         'physical robot addresses are forbidden.')
    return host


def _origin_authority(origin):
    try:
        parsed = urlsplit(origin)
        parsed.port  # Reject malformed and out-of-range ports.
        if (parsed.scheme in ('http', 'https') and parsed.hostname
                and parsed.username is None and parsed.password is None
                and not parsed.path and not parsed.query and not parsed.fragment
                and not any(char.isspace() for char in origin)):
            return parsed.netloc.lower()
    except ValueError:
        pass
    return None


def _trusted_host(request, forwarded_authority):
    hosts = request.headers.getall('Host', [])
    authority = _origin_authority('http://' + hosts[0]) if len(hosts) == 1 else None
    return authority is not None and (
        urlsplit('//' + authority).hostname in ('localhost', '127.0.0.1', '::1')
        or authority == forwarded_authority)


def _same_origin(request, forwarded_origin):
    """Trust only loopback origins or the configured Codespaces HTTPS origin.

    The tunnel may preserve Host or rewrite it to loopback. Forwarded-host
    headers are not trusted, and matching attacker-controlled Origin/Host
    values must not bypass the loopback boundary through DNS rebinding.
    """
    origins = request.headers.getall('Origin', [])
    if len(origins) != 1:
        return False
    authority = _origin_authority(origins[0])
    return authority is not None and (
        origins[0] == forwarded_origin
        or (authority == request.headers['Host'].lower()
            and urlsplit('//' + authority).hostname in ('localhost', '127.0.0.1', '::1')))


def _decode_jpeg(data):
    """Inspect a bounded JPEG header before any full-size pixel allocation."""
    if not data or len(data) > MAX_JPEG_BYTES:
        raise ValueError('JPEG must be nonempty and at most 256 KiB.')
    try:
        with warnings.catch_warnings():
            warnings.simplefilter('error', Image.DecompressionBombWarning)
            with Image.open(BytesIO(data)) as image:
                if image.format != 'JPEG':
                    raise ValueError('Only binary JPEG camera frames are accepted.')
                width, height = image.size
                if not 0 < width <= MAX_WIDTH or not 0 < height <= MAX_HEIGHT:
                    raise ValueError('JPEG dimensions must be at most 640x480.')
                rgb = np.asarray(image.convert('RGB'))
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    except (OSError, Image.DecompressionBombWarning, Image.DecompressionBombError) as exc:
        raise ValueError(f'Invalid JPEG camera frame: {exc}') from exc


def _tracker_factory():
    from lab4lib.vision import PoseTracker
    return PoseTracker()


def _controller_factory(host):
    from lab4lib.controller import RobotController
    return RobotController(host)


class _Owner:
    """One websocket owns one disarmed session and one serialized vision worker."""

    def __init__(self, server, ws):
        self.server, self.ws = server, ws
        self.session = None
        self.tracker = None
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='lab4-browser-vision')
        self.active = True
        self.cleanup_failed = False
        self.generation = 0
        self.task = 1
        self.message = 'Start with Task 1. No robot motion is enabled.'
        self.processing_ms = 0.0
        self.shown = None
        self.frame_pending = False
        self.granted_at = None
        self.grant_generation = 0
        self.frame_task = self.action_task = self.ticker = self.cleanup_task = None
        self.last_activity = self.server.clock()

    async def _worker(self, function, *args):
        return await asyncio.get_running_loop().run_in_executor(self.executor, function, *args)

    def _infer(self, data):
        frame = _decode_jpeg(data)
        if self.tracker is None:
            self.tracker = self.server.tracker_factory()
        return self.tracker.process(frame)

    @staticmethod
    def _render(annotated, lines):
        shown = with_status(annotated, lines)
        success, jpeg = cv2.imencode('.jpg', shown, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not success:
            raise RuntimeError('Could not encode the annotated camera preview.')
        return shown, jpeg.tobytes()

    def _lines(self):
        if self.task == 3:
            return status_lines(self.session, self.message, self.processing_ms)
        lines = [f'Task {self.task} - ' + ('arm landmarks' if self.task == 1 else 'elbow angle'),
                 self.message]
        if self.task == 2:
            angles = self.session.angles
            lines.append('Elbow [deg, rad]: ' + ('not implemented yet' if angles is None else
                         f"{math.degrees(angles['elbow']):.1f}, {angles['elbow']:.3f}"))
        return [*lines, 'Save evidence: save the current task preview | Stop camera: disconnect']

    async def state(self):
        if not self.active or self.ws.closed or self.session is None:
            return
        session = self.session
        await self.ws.send_json({
            'type': 'state', 'task': self.task, 'tracking': session.tracking(),
            'angles': session.angles,
            'target': None if session.target is None else session.target.tolist(),
            'calibrated': session.neutral is not None, 'robot': session.snapshot(),
            'feedback': session.feedback, 'message': self.message,
            'processing_ms': self.processing_ms,
        })

    async def ready(self):
        if self.active and not self.ws.closed:
            # Clear pending BEFORE handing the browser its next single frame credit.
            self.frame_pending = False
            self.granted_at = self.server.clock()
            self.grant_generation = self.generation
            await self.ws.send_json({'type': 'ready'})

    def disarm(self, reason):
        if self.session is not None:
            self.session.stop(reason)

    async def error(self, message):
        self.disarm('Action failed; robot control disabled. Enable again explicitly to resume.')
        if self.session is not None:
            self.session.feedback = f'ERROR: {message}'
        if self.active and not self.ws.closed:
            await self.ws.send_json({'type': 'error', 'message': str(message)})

    async def fail(self, message):
        self.disarm(message)
        self.active = False
        self.generation += 1
        if not self.ws.closed:
            with suppress(ConnectionError, RuntimeError):
                await self.ws.send_json({'type': 'error', 'message': str(message)})
                await self.ws.close(code=1008, message=str(message).encode('utf-8')[:120])

    async def _tick(self):
        while self.active:
            await asyncio.sleep(0.2)
            if self.server.clock() - self.last_activity > self.server.idle_timeout:
                await self.fail('Camera session timed out; robot control disabled. Reconnect explicitly.')
                return
            await self.state()

    async def _process_frame(self, data, observed_at, generation):
        started = time.monotonic()
        try:
            if generation != self.generation:
                return
            if not 0 <= self.server.clock() - observed_at <= config.TARGET_MAX_AGE:
                self.message = 'Stale camera credit; capture a new frame. Tracking paused.'
                self.session.observe(None, observed_at, self.message)
                return
            annotated, angles, message = await self._worker(self._infer, data)
            if not self.active or generation != self.generation:
                return
            self.processing_ms = (time.monotonic() - started) * 1000
            self.message = str(message)
            # The server's credit time, not receipt or inference completion time,
            # conservatively bounds camera/network/inference latency.
            self.session.observe(angles, observed_at, self.message)
            shown, jpeg = await self._worker(self._render, annotated, self._lines())
            if not self.active or generation != self.generation:
                return
            self.shown = shown
            await self.ws.send_bytes(jpeg)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            if self.active and generation == self.generation:
                LOGGER.exception('Browser frame processing failed')
                await self.fail(f'Frame processing failed: {exc}')
        finally:
            self.frame_pending = False
            if self.active:
                await self.ready()

    async def frame(self, data):
        if self.frame_pending or self.granted_at is None:
            await self.fail('A camera frame arrived without a ready credit; robot control disabled.')
            return
        observed_at, generation = self.granted_at, self.grant_generation
        self.granted_at = None
        if len(data) > MAX_JPEG_BYTES:
            await self.fail('JPEG exceeds the 256 KiB limit; robot control disabled.')
            return
        self.frame_pending = True
        self.frame_task = asyncio.create_task(self._process_frame(data, observed_at, generation))

    def _save(self, shown, task, rows):
        paths = [save_preview(shown, task, output_dir=self.server.output_dir)]
        if task == 3:
            paths.append(save_control_log(LOG_COLUMNS, rows, output_dir=self.server.output_dir))
        return [{'name': path.name, 'url': f'/outputs/{path.name}'} for path in paths]

    async def _save_evidence(self, shown, task, rows, generation):
        try:
            files = await self._worker(self._save, shown, task, rows)
            if self.active and generation == self.generation:
                await self.ws.send_json({'type': 'saved', 'files': files})
        except Exception as exc:
            if self.active and generation == self.generation:
                await self.error(exc)

    async def text(self, text):
        try:
            if len(text) > 4096:
                raise ValueError('Control messages must be at most 4096 characters.')
            message = json.loads(text)
            if not isinstance(message, dict):
                raise ValueError('Control messages must be JSON objects.')
            kind = message.get('type')
            if kind == 'task' and set(message) == {'type', 'task'}:
                task = message['task']
                if type(task) is not int or task not in (1, 2, 3):
                    raise ValueError('Select Task 1, 2, or 3.')
                if task != self.task:
                    self.disarm('Task changed; robot control disabled. Initialize and calibrate as needed.')
                    self.generation += 1
                    self.task = task
                    self.session = ControlSession(self.session.robot)
                    self.shown = None
                    self.processing_ms = 0.0
                    self.message = f'Task {task} selected; waiting for a new camera frame.'
                await self.state()
                return
            if kind != 'action' or set(message) != {'type', 'action'}:
                raise ValueError('Unknown control message.')
            action = message['action']
            if action == 'camera_stop':
                self.disarm('Camera stopped; robot control disabled.')
                self.active = False
                self.generation += 1
                await self.ws.close()
                return
            if action == 'stop':
                self.session.stop()
            elif action == 'save':
                if self.shown is None:
                    raise ValueError('Wait for a current-task camera preview before saving evidence.')
                if self.action_task is not None and not self.action_task.done():
                    raise ValueError('An evidence save is already pending.')
                self.action_task = asyncio.create_task(self._save_evidence(
                    self.shown, self.task, list(self.session.log), self.generation))
            elif action in ('initialize', 'calibrate', 'enable'):
                if self.task != 3:
                    raise ValueError('Robot actions are available only in Task 3.')
                getattr(self.session, action)()
            else:
                raise ValueError('Unknown action.')
        except (ValueError, RuntimeError, OSError, TypeError) as exc:
            await self.error(exc)
        await self.state()

    async def run(self):
        try:
            self.session = ControlSession(self.server.controller_factory(self.server.robot_host))
            await self.state()
            await self.ready()
            self.ticker = asyncio.create_task(self._tick())
            async for message in self.ws:
                if not self.active:
                    break
                self.last_activity = self.server.clock()
                if message.type == WSMsgType.BINARY:
                    await self.frame(message.data)
                elif message.type == WSMsgType.TEXT:
                    await self.text(message.data)
                elif message.type == WSMsgType.ERROR:
                    break
        except Exception as exc:
            LOGGER.exception('Browser owner session failed')
            await self.fail(f'Browser session failed: {exc}')
        finally:
            await self.shutdown()

    async def _cleanup(self):
        # Close RTDE concurrently with, never after, potentially slow inference.
        robot_close = (asyncio.create_task(asyncio.to_thread(self.session.close))
                       if self.session is not None else None)
        if self.ticker is not None and self.ticker is not asyncio.current_task():
            self.ticker.cancel()
            await asyncio.gather(self.ticker, return_exceptions=True)
        pending = [task for task in (self.frame_task, self.action_task) if task is not None]
        if pending:
            await asyncio.gather(*pending, return_exceptions=True)
        try:
            if self.tracker is not None:
                await self._worker(self.tracker.close)
        except Exception:
            self.cleanup_failed = True
            LOGGER.exception('Browser tracker cleanup failed; ownership remains locked')
        finally:
            self.executor.shutdown(wait=True)
        if robot_close is not None:
            try:
                await robot_close
            except Exception:
                self.cleanup_failed = True
                LOGGER.exception('Browser RTDE cleanup failed; ownership remains locked')
        if not self.ws.closed:
            await self.ws.close()

    async def shutdown(self):
        if self.cleanup_task is None:
            # No awaits precede disarming or invalidating in-flight results.
            self.disarm('Browser disconnected; robot control disabled.')
            self.active = False
            self.generation += 1
            self.cleanup_task = asyncio.create_task(self._cleanup())
        await asyncio.shield(self.cleanup_task)


class _Server:
    def __init__(self, robot_host, tracker_factory, controller_factory, output_dir, clock,
                 idle_timeout, forwarded_origin):
        self.robot_host = robot_host
        self.tracker_factory = tracker_factory
        self.controller_factory = controller_factory
        self.output_dir = Path(output_dir).resolve()
        self.clock, self.idle_timeout = clock, idle_timeout
        self.forwarded_origin = forwarded_origin
        self.owner = None

    async def websocket(self, request):
        if not _same_origin(request, self.forwarded_origin):
            raise web.HTTPForbidden(text='A matching same-origin browser Origin is required.')
        ws = web.WebSocketResponse(max_msg_size=MAX_JPEG_BYTES + 1, heartbeat=2.0, timeout=2.0)
        if self.owner is not None:
            await ws.prepare(request)
            message = ('Cleanup failed; browser control remains locked. Check URSim has stopped, '
                       'then restart the Python server before reconnecting.'
                       if self.owner.cleanup_failed else
                       'Another browser owns this session, or its cleanup is still running. '
                       'Stop that camera before reconnecting.')
            await ws.send_json({'type': 'error', 'message': message})
            await ws.close(code=1008, message=b'One controlling browser at a time')
            return ws
        owner = _Owner(self, ws)
        self.owner = owner  # Reserve before the first await; GET does not claim ownership.
        try:
            await ws.prepare(request)
            await owner.run()
        finally:
            await owner.shutdown()
            if not owner.cleanup_failed:
                self.owner = None
        return ws

    async def output(self, request):
        name = request.match_info['name']
        if name not in EVIDENCE_NAMES:
            raise web.HTTPNotFound()
        path = self.output_dir / name
        if path.is_symlink() or not path.is_file() or path.resolve().parent != self.output_dir:
            raise web.HTTPNotFound()
        return web.FileResponse(path, headers={'Content-Disposition': f'attachment; filename="{name}"'})

    async def shutdown(self, app):
        if self.owner is not None:
            await self.owner.shutdown()


async def _security_headers(request, response):
    response.headers.update(SECURITY_HEADERS)


def create_app(robot_host='127.0.0.1', *, tracker_factory=None, controller_factory=None,
               output_dir=None, clock=time.monotonic, idle_timeout=30.0, forwarded_origin=None):
    """Create an idle app; injectable Python factories support isolated safety checks.

    The real CLI uses only the shared PoseTracker and RobotController. No browser
    route can change factories, robot host, exercise functions, or output directory.
    """
    _simulator_host(robot_host)
    if isinstance(idle_timeout, bool) or not math.isfinite(idle_timeout) or idle_timeout <= 0:
        raise ValueError('idle_timeout must be finite and positive.')
    forwarded_authority = None
    if forwarded_origin is not None:
        forwarded_authority = _origin_authority(forwarded_origin)
        if forwarded_authority is None or not forwarded_origin.startswith('https://'):
            raise ValueError('The forwarded origin must be an HTTPS origin without a path.')
        forwarded_origin = 'https://' + forwarded_authority
    server = _Server(robot_host, tracker_factory or _tracker_factory,
                     controller_factory or _controller_factory,
                     OUTPUT if output_dir is None else output_dir, clock, idle_timeout,
                     forwarded_origin)

    @web.middleware
    async def trusted_host(request, handler):
        if not _trusted_host(request, forwarded_authority):
            raise web.HTTPForbidden(text='Use the loopback URL or this Codespace forwarded URL.')
        return await handler(request)

    app = web.Application(client_max_size=MAX_JPEG_BYTES + 1, middlewares=[trusted_host])
    for route, name in (('/', 'index.html'), ('/app.js', 'app.js'), ('/style.css', 'style.css')):
        async def static(request, name=name):
            return web.FileResponse(WEB_ROOT / name)
        app.router.add_get(route, static)
    app.router.add_get('/ws', server.websocket)
    app.router.add_get('/outputs/{name}', server.output)
    app.on_response_prepare.append(_security_headers)
    app.on_shutdown.append(server.shutdown)
    return app


def main(argv=None):
    parser = argparse.ArgumentParser(description='Optional private browser camera UI for URSim only.')
    parser.add_argument('--host', default='ursim_cb3', choices=sorted(SIMULATOR_HOSTS),
                        help='allowlisted simulator host (default: ursim_cb3; never a physical robot)')
    parser.add_argument('--port', type=int, default=8000, help='loopback HTTP port (default: 8000)')
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    print('Browser fallback is simulation-only. Keep the forwarded port PRIVATE and open its '
          'top-level HTTPS URL; native scripts remain the primary workflow.', flush=True)
    name = os.environ.get('CODESPACE_NAME')
    domain = os.environ.get('GITHUB_CODESPACES_PORT_FORWARDING_DOMAIN', 'app.github.dev')
    forwarded_origin = f'https://{name}-{args.port}.{domain}' if name else None
    web.run_app(create_app(args.host, forwarded_origin=forwarded_origin),
                host='127.0.0.1', port=args.port)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
