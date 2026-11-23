"""Optional transport safety checks, deliberately outside native test discovery.

From lab4: uv run --extra browser python -m unittest discover -s tests -p browser_checks.py -v
These isolated tests use explicit Python factories, not production debug routes.
The separate browser_smoke.py checks real camera/MediaPipe/URSim integration.
"""

import asyncio
import csv
from io import StringIO
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock

from aiohttp import ClientSession, WSServerHandshakeError, WSMsgType
from aiohttp.test_utils import TestClient, TestServer
import cv2
import numpy as np

import student_config as config
from lab4lib.session import LOG_COLUMNS
from web_lab import MAX_JPEG_BYTES, create_app


class PreviewRobot:
    def __init__(self, host):
        self.host = host
        self.initialized = self.enabled = self.busy = self.closed = False
        self.reason = 'Disconnected. Initialize explicitly.'
        self.target = None
        self.received = []
        self.pauses = 0
        self.disarmed = threading.Event()
        self.close_called = threading.Event()
        self.block_close = None

    def snapshot(self):
        return {'initialized': self.initialized, 'enabled': self.enabled, 'busy': self.busy,
                'message': self.reason, 'actual_q': None, 'command_q': None}

    def initialize(self):
        self.initialized = True
        self.reason = 'At HOME.'

    def disable(self, reason='Stopped'):
        self.enabled = False
        self.target = None
        self.reason = reason
        self.disarmed.set()

    def pause_tracking(self):
        self.target = None
        self.pauses += 1

    def update_target(self, q, observed_at):
        self.target = q.copy()
        self.received.append(observed_at)

    def enable(self):
        self.enabled = True
        self.disarmed.clear()

    def close(self):
        self.close_called.set()
        if self.block_close is not None:
            self.block_close.wait(5)
        self.disable('Closed.')
        self.initialized = False
        self.closed = True


class Tracker:
    def __init__(self, harness):
        self.harness = harness
        self.created_thread = threading.get_ident()
        self.process_threads = []
        self.closed_thread = None

    def process(self, frame):
        self.process_threads.append(threading.get_ident())
        self.harness.processing.set()
        if self.harness.release is not None:
            if not self.harness.release.wait(5):
                raise RuntimeError('Test did not release its blocked inference.')
        if self.harness.failure is not None:
            raise self.harness.failure
        return frame, self.harness.angles, ('Tracking left arm' if self.harness.angles else 'Missing wrist')

    def close(self):
        self.closed_thread = threading.get_ident()


class Harness:
    def __init__(self):
        self.robots = []
        self.trackers = []
        self.angles = {'elbow': 1.5, 'shoulder': 0.2}
        self.processing = threading.Event()
        self.release = None
        self.failure = None
        self.clock_offset = 0.0

    def robot(self, host):
        robot = PreviewRobot(host)
        self.robots.append(robot)
        return robot

    def tracker(self):
        tracker = Tracker(self)
        self.trackers.append(tracker)
        return tracker

    def clock(self):
        return time.monotonic() + self.clock_offset


def jpeg():
    success, image = cv2.imencode('.jpg', np.zeros((48, 64, 3), dtype=np.uint8))
    if not success:
        raise RuntimeError('Cannot create bounded test JPEG.')
    return image.tobytes()


class BrowserChecks(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.harness = Harness()
        self.output = tempfile.TemporaryDirectory()
        self.addCleanup(self.output.cleanup)
        self.client = await self.client_for()
        self.sockets = []

    async def client_for(self, **options):
        app = create_app(tracker_factory=self.harness.tracker,
                         controller_factory=self.harness.robot,
                         output_dir=Path(self.output.name), clock=self.harness.clock, **options)
        client = TestClient(TestServer(app))
        await client.start_server()
        self.addAsyncCleanup(client.close)
        return client

    async def asyncTearDown(self):
        if self.harness.release is not None:
            self.harness.release.set()
        for robot in self.harness.robots:
            if robot.block_close is not None:
                robot.block_close.set()
        for ws in self.sockets:
            await ws.close()

    async def connect(self, client=None, *, origin=None):
        client = self.client if client is None else client
        origin = str(client.make_url('/')).rstrip('/') if origin is None else origin
        ws = await client.ws_connect('/ws', headers={'Origin': origin})
        self.sockets.append(ws)
        return ws

    async def replacement_owner(self):
        async with asyncio.timeout(4):
            while True:
                ws = await self.connect()
                first = await self.message(ws)
                self.assertEqual(first.type, WSMsgType.TEXT)
                if first.json()['type'] == 'state':
                    await self.event(ws, 'ready')
                    return ws
                self.assertEqual(first.json()['type'], 'error')
                await ws.close()
                await asyncio.sleep(0.01)

    async def message(self, ws):
        message = await asyncio.wait_for(ws.receive(), 4)
        self.assertNotEqual(message.type, WSMsgType.ERROR, str(ws.exception()))
        return message

    async def event(self, ws, kind, predicate=lambda message: True):
        async with asyncio.timeout(4):
            while True:
                message = await self.message(ws)
                self.assertNotIn(message.type, (WSMsgType.CLOSE, WSMsgType.CLOSED), message.data)
                if message.type == WSMsgType.TEXT:
                    value = message.json()
                    if value.get('type') == kind and predicate(value):
                        return value

    async def owner(self, client=None):
        ws = await self.connect(client)
        await self.event(ws, 'ready')
        return ws

    async def select(self, ws, task):
        await ws.send_json({'type': 'task', 'task': task})
        return await self.event(ws, 'state', lambda state: state['task'] == task)

    async def action(self, ws, action):
        await ws.send_json({'type': 'action', 'action': action})

    async def capture(self, ws):
        # A task switch invalidates its outstanding old-stage credit. That frame
        # is intentionally dropped; the next grant captures the current stage.
        for _ in range(3):
            await ws.send_bytes(jpeg())
            preview = None
            async with asyncio.timeout(4):
                while True:
                    message = await self.message(ws)
                    self.assertNotIn(message.type, (WSMsgType.CLOSE, WSMsgType.CLOSED), message.data)
                    if message.type == WSMsgType.BINARY:
                        preview = message.data
                    elif message.type == WSMsgType.TEXT:
                        value = message.json()
                        self.assertNotEqual(value['type'], 'error', value)
                        if value['type'] == 'ready':
                            break
            if preview is not None:
                self.assertTrue(preview.startswith(b'\xff\xd8'))
                return preview
        self.fail('No preview after three fresh camera grants.')

    async def arm(self, ws):
        await self.select(ws, 3)
        await self.capture(ws)
        await self.action(ws, 'initialize')
        await self.event(ws, 'state', lambda state: state['robot']['initialized'])
        await self.action(ws, 'calibrate')
        await self.event(ws, 'state', lambda state: state['calibrated'])
        await self.action(ws, 'enable')
        return await self.event(ws, 'state', lambda state: state['robot']['enabled'])

    async def wait_thread_event(self, event):
        self.assertTrue(await asyncio.to_thread(event.wait, 3), 'Worker event did not arrive.')

    async def test_get_does_not_acquire_owner_or_open_devices(self):
        response = await self.client.get('/app.js')
        self.assertEqual(response.status, 200)
        self.assertEqual(response.headers['X-Frame-Options'], 'DENY')
        self.assertIn("frame-ancestors 'none'", response.headers['Content-Security-Policy'])
        self.assertEqual(response.headers['Cache-Control'], 'no-store')
        self.assertEqual(self.harness.robots, [])
        self.assertEqual(self.harness.trackers, [])
        await self.owner()
        self.assertEqual(len(self.harness.robots), 1)
        self.assertEqual(self.harness.trackers, [])
        self.assertFalse(self.harness.robots[0].enabled)

    async def test_allowlist_rejects_physical_or_malformed_targets(self):
        for host in ('192.168.0.2', 'ur5', '127.0.0.2', 'localhost:30004',
                     'http://localhost', 'localhost\n', '', None):
            with self.subTest(host=host), self.assertRaises(ValueError):
                create_app(host)

    async def test_strict_origin_and_https_proxy_authority(self):
        async with ClientSession() as session:
            url = self.client.make_url('/ws')
            for headers in ({}, {'Origin': 'null'}, {'Origin': 'https://evil.example'},
                            {'Origin': str(self.client.make_url('/path'))},
                            {'Origin': 'https://evil.example', 'X-Forwarded-Host': 'evil.example'}):
                with self.subTest(headers=headers), self.assertRaises(WSServerHandshakeError) as error:
                    await session.ws_connect(url, headers=headers)
                self.assertEqual(error.exception.status, 403)
        authority = self.client.make_url('/').authority
        ws = await self.connect(origin=f'https://{authority}')
        await self.event(ws, 'ready')
        self.assertEqual(len(self.harness.robots), 1)

    async def test_matching_untrusted_origin_and_host_cannot_read_evidence_or_control(self):
        (Path(self.output.name) / 'task1_preview.jpg').write_bytes(jpeg())
        headers = {'Host': 'evil.example', 'Origin': 'http://evil.example',
                   'X-Forwarded-Host': 'localhost'}
        for path in ('/', '/app.js', '/outputs/task1_preview.jpg'):
            response = await self.client.get(path, headers=headers)
            self.assertEqual(response.status, 403)
        with self.assertRaises(WSServerHandshakeError) as error:
            await self.client.ws_connect('/ws', headers=headers)
        self.assertEqual(error.exception.status, 403)
        self.assertEqual(self.harness.robots, [])

    async def test_configured_codespaces_origin_handles_preserved_and_rewritten_host(self):
        origin = 'https://example-space-8000.app.github.dev'
        for preserve_host in (False, True):
            client = await self.client_for(forwarded_origin=origin)
            headers = {'Origin': origin}
            if preserve_host:
                headers['Host'] = 'example-space-8000.app.github.dev'
            ws = await client.ws_connect('/ws', headers=headers)
            self.sockets.append(ws)
            await self.event(ws, 'ready')
            await ws.send_json({'type': 'task', 'task': 3})
            await self.event(ws, 'state', lambda state: state['task'] == 3)
            await ws.close()

    async def test_second_owner_cannot_mutate_first_session(self):
        first = await self.owner()
        await self.arm(first)
        robot = self.harness.robots[0]
        second = await self.connect()
        error = await self.event(second, 'error')
        self.assertIn('Another browser', error['message'])
        self.assertEqual(len(self.harness.robots), 1)
        self.assertTrue(robot.enabled)
        await self.action(first, 'stop')
        await self.event(first, 'state', lambda state: not state['robot']['enabled'])
        self.assertFalse(robot.enabled)

    async def test_stale_grant_drops_before_decode_and_new_credit_recovers(self):
        self.harness.clock_offset = -(config.TARGET_MAX_AGE + 1)
        ws = await self.owner()
        self.harness.clock_offset = 0
        await ws.send_bytes(jpeg())
        await self.event(ws, 'ready')
        self.assertEqual(self.harness.trackers, [])
        self.assertEqual(self.harness.robots[0].pauses, 1)
        stale = await self.event(ws, 'state', lambda state: 'Stale camera credit' in state['message'])
        self.assertFalse(stale['tracking'])
        await self.capture(ws)
        fresh = await self.event(ws, 'state', lambda state: state['tracking'])
        self.assertIsNotNone(fresh['angles'])
        self.assertEqual(len(self.harness.trackers), 1)

    async def test_disconnect_during_inference_disarms_before_wait_and_reconnects_cleanly(self):
        ws = await self.owner()
        await self.arm(ws)
        robot = self.harness.robots[0]
        prior_targets = len(robot.received)
        self.harness.processing.clear()
        self.harness.release = threading.Event()
        await ws.send_bytes(jpeg())
        await self.wait_thread_event(self.harness.processing)
        await ws.close()
        await self.wait_thread_event(robot.disarmed)
        await self.wait_thread_event(robot.close_called)
        self.assertFalse(robot.enabled)
        second = await self.connect()
        self.assertIn('cleanup', (await self.event(second, 'error'))['message'])
        self.assertEqual(len(self.harness.robots), 1)
        self.harness.release.set()
        tracker = self.harness.trackers[0]
        async with asyncio.timeout(4):
            while tracker.closed_thread is None:
                await asyncio.sleep(0.01)
        self.assertEqual(len(robot.received), prior_targets, 'Late inference refreshed a disconnected target.')
        self.assertEqual(tracker.created_thread, tracker.closed_thread)
        self.assertTrue(all(thread == tracker.created_thread for thread in tracker.process_threads))
        self.assertNotEqual(tracker.created_thread, threading.get_ident())
        replacement = await self.replacement_owner()
        state = await self.event(replacement, 'state')
        self.assertFalse(state['robot']['enabled'])
        self.assertFalse(state['robot']['initialized'])
        self.assertFalse(state['calibrated'])

    async def test_ownership_waits_for_controller_cleanup_too(self):
        ws = await self.owner()
        robot = self.harness.robots[0]
        robot.block_close = threading.Event()
        await ws.close()
        await self.wait_thread_event(robot.close_called)
        second = await self.connect()
        self.assertIn('cleanup', (await self.event(second, 'error'))['message'])
        self.assertEqual(len(self.harness.robots), 1)
        robot.block_close.set()
        replacement = await self.replacement_owner()
        state = await self.event(replacement, 'state')
        self.assertFalse(state['robot']['enabled'])

    async def test_inference_completion_does_not_replace_credit_timestamp(self):
        ws = await self.owner()
        await self.arm(ws)
        robot = self.harness.robots[0]
        prior_targets = len(robot.received)
        self.harness.processing.clear()
        self.harness.release = threading.Event()
        await ws.send_bytes(jpeg())
        await self.wait_thread_event(self.harness.processing)
        with mock.patch('student_config.TARGET_MAX_AGE', 0.01):
            await asyncio.sleep(0.03)
            self.harness.release.set()
            await self.event(ws, 'ready')
            stale = await self.event(ws, 'state', lambda state: not state['tracking'])
        self.assertTrue(stale['robot']['enabled'])
        self.assertIsNone(robot.target)
        self.assertEqual(len(robot.received), prior_targets)

    async def test_task_change_invalidates_inflight_pose_and_disarms(self):
        ws = await self.owner()
        await self.arm(ws)
        robot = self.harness.robots[0]
        prior_targets = len(robot.received)
        self.harness.processing.clear()
        self.harness.release = threading.Event()
        await ws.send_bytes(jpeg())
        await self.wait_thread_event(self.harness.processing)
        await self.select(ws, 1)
        self.assertFalse(robot.enabled)
        self.harness.release.set()
        await self.event(ws, 'ready')
        self.assertEqual(len(robot.received), prior_targets)
        state = await self.event(ws, 'state', lambda value: value['task'] == 1)
        self.assertIsNone(state['angles'])
        self.assertIsNone(state['target'])
        await self.action(ws, 'save')
        self.assertIn('preview', (await self.event(ws, 'error'))['message'])

    async def test_late_old_task_error_does_not_close_new_stage(self):
        ws = await self.owner()
        await self.capture(ws)
        self.harness.processing.clear()
        self.harness.release = threading.Event()
        await ws.send_bytes(jpeg())
        await self.wait_thread_event(self.harness.processing)
        await self.select(ws, 2)
        self.harness.failure = ValueError('Obsolete task inference failed.')
        self.harness.release.set()
        await self.event(ws, 'ready')
        self.harness.failure = None
        await self.capture(ws)
        state = await self.event(ws, 'state', lambda value: value['task'] == 2 and value['tracking'])
        self.assertFalse(state['robot']['enabled'])

    async def test_missing_pose_pauses_and_fresh_pose_resumes_without_rearming_stop(self):
        ws = await self.owner()
        await self.arm(ws)
        robot = self.harness.robots[0]
        self.harness.angles = None
        await self.capture(ws)
        missing = await self.event(ws, 'state', lambda state: not state['tracking'])
        self.assertTrue(missing['robot']['enabled'])
        self.assertIsNone(robot.target)
        self.harness.angles = {'elbow': 1.7, 'shoulder': 0.2}
        await self.capture(ws)
        await self.event(ws, 'state', lambda state: state['tracking'])
        self.assertTrue(robot.enabled)
        self.assertIsNotNone(robot.target)
        await self.action(ws, 'stop')
        await self.event(ws, 'state', lambda state: not state['robot']['enabled'])
        await self.capture(ws)
        self.assertFalse(robot.enabled)

    async def test_double_frame_without_credit_is_fatal_and_disarms(self):
        ws = await self.owner()
        await self.arm(ws)
        self.harness.processing.clear()
        self.harness.release = threading.Event()
        await ws.send_bytes(jpeg())
        await self.wait_thread_event(self.harness.processing)
        await ws.send_bytes(jpeg())
        error = await self.event(ws, 'error')
        self.assertIn('without a ready credit', error['message'])
        self.assertFalse(self.harness.robots[0].enabled)
        self.harness.release.set()

    async def test_oversized_jpeg_header_is_rejected_without_full_pixel_decode(self):
        ws = await self.owner()
        image = bytearray(jpeg())
        marker = image.index(b'\xff\xc0')
        image[marker + 5:marker + 7] = (4096).to_bytes(2, 'big')
        image[marker + 7:marker + 9] = (4096).to_bytes(2, 'big')
        with mock.patch('web_lab.Image.Image.convert', side_effect=AssertionError('Huge pixel decode')) as decode:
            await ws.send_bytes(image)
            error = await self.event(ws, 'error')
        self.assertIn('640x480', error['message'])
        decode.assert_not_called()
        self.assertEqual(self.harness.trackers, [])
        self.assertTrue(self.harness.robots[0].disarmed.is_set())

    async def test_oversized_wire_payload_rejected_before_decode(self):
        ws = await self.owner()
        with mock.patch('web_lab._decode_jpeg', side_effect=AssertionError('Oversized decode')) as decode:
            await ws.send_bytes(b'x' * (MAX_JPEG_BYTES + 1))
            closed = await self.message(ws)
        self.assertEqual(closed.type, WSMsgType.CLOSE)
        self.assertEqual(closed.data, 1009)  # aiohttp rejects at the wire limit.
        decode.assert_not_called()
        await self.wait_thread_event(self.harness.robots[0].disarmed)

    async def test_evidence_fixed_downloads_and_native_csv_columns(self):
        ws = await self.owner()
        await self.action(ws, 'save')
        self.assertIn('preview', (await self.event(ws, 'error'))['message'])
        for task in (1, 2, 3):
            await self.select(ws, task)
            await self.capture(ws)
            await self.action(ws, 'save')
            saved = await self.event(ws, 'saved')
            expected = [f'task{task}_preview.jpg'] + (['task3_control.csv'] if task == 3 else [])
            self.assertEqual([item['name'] for item in saved['files']], expected)
            for item in saved['files']:
                self.assertEqual(item['url'], f"/outputs/{item['name']}")
                response = await self.client.get(item['url'])
                self.assertEqual(response.status, 200)
                self.assertIn('attachment;', response.headers['Content-Disposition'])
                data = await response.read()
                if item['name'].endswith('.jpg'):
                    self.assertTrue(data.startswith(b'\xff\xd8'))
                else:
                    rows = list(csv.reader(StringIO(data.decode('utf-8'))))
                    self.assertEqual(rows[0], LOG_COLUMNS)
                    self.assertGreater(len(rows), 1)
        (Path(self.output.name) / 'private.txt').write_text('not downloadable')
        for path in ('/outputs/private.txt', '/outputs/../student.py', '/outputs/task4_preview.jpg'):
            self.assertEqual((await self.client.get(path)).status, 404)
        allowed = Path(self.output.name) / 'task1_preview.jpg'
        allowed.unlink()
        allowed.symlink_to(Path(self.output.name) / 'private.txt')
        self.assertEqual((await self.client.get('/outputs/task1_preview.jpg')).status, 404)

    async def test_actions_outside_task3_fail_without_motion(self):
        ws = await self.owner()
        await self.action(ws, 'initialize')
        error = await self.event(ws, 'error')
        self.assertIn('only in Task 3', error['message'])
        self.assertFalse(self.harness.robots[0].initialized)
        await ws.send_json({'type': 'task', 'task': True})
        self.assertIn('Select Task', (await self.event(ws, 'error'))['message'])

    async def test_idle_timeout_disarms_and_closes_controller(self):
        client = await self.client_for(idle_timeout=0.3)
        ws = await self.owner(client)
        robot = self.harness.robots[0]
        error = await self.event(ws, 'error')
        self.assertIn('timed out', error['message'])
        await self.wait_thread_event(robot.close_called)
        self.assertTrue(robot.disarmed.is_set())

    async def test_status_keeps_updating_without_camera_during_home(self):
        ws = await self.owner()
        await self.select(ws, 3)
        await self.action(ws, 'initialize')
        await self.event(ws, 'state', lambda state: state['robot']['initialized'])
        robot = self.harness.robots[0]
        robot.busy = True
        robot.reason = 'Moving to HOME.'
        busy = await self.event(ws, 'state', lambda state: state['robot']['busy'])
        self.assertIn('HOME', busy['robot']['message'])
        robot.busy = False
        robot.reason = 'At HOME.'
        idle = await self.event(ws, 'state', lambda state: not state['robot']['busy'])
        self.assertEqual(idle['robot']['message'], 'At HOME.')
        self.assertFalse(idle['tracking'])
        self.assertEqual(self.harness.trackers, [])


if __name__ == '__main__':
    unittest.main()
