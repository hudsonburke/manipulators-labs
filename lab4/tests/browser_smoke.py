"""Chromium camera -> MediaPipe -> aiohttp -> actual URSim motion proof.

Run after ursim_smoke.py, with LAB4_URSIM_CI=1 inside GitHub Actions.
Use --camera-only locally for real camera/inference/UI checks without robot I/O.
No student geometry solution: first prove the unchanged exercise rejects a real
pose, then replace only its in-process output with a configurable constant.
The real tracker must still find and validate landmarks for every valid target.
"""

import argparse
import asyncio
import csv
import hashlib
import io
import json
import logging
import math
import os
from pathlib import Path
import sys
import time
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

HOST = '127.0.0.1'  # Never configurable, and never inherited from URSIM_HOST.
ARTIFACTS = Path('ci-artifacts/browser')
FIXTURE_URL = 'https://storage.googleapis.com/mediapipe-assets/pose.jpg'
FIXTURE_SHA256 = 'c8a830ed683c0276d713dd5aeda28f415f10cd6291972084a40d0d8b934ed62b'
FPS = 10
VISIBLE_SECONDS = 90
BLANK_SECONDS = 12


def download(url, path, maximum):
    """Bound both network reads and wall time; retain provenance with SHA-256."""
    deadline = time.monotonic() + 60
    size = 0
    digest = hashlib.sha256()
    with urllib.request.urlopen(url, timeout=15) as response, path.open('wb') as output:
        while block := response.read(65536):
            size += len(block)
            if size > maximum or time.monotonic() > deadline:
                raise RuntimeError(f'Download exceeded size/time limit: {url}')
            digest.update(block)
            output.write(block)
    if not size:
        raise RuntimeError(f'Empty download: {url}')
    return {'url': url, 'bytes': size, 'sha256': digest.hexdigest()}


def prepare_camera():
    import cv2
    import numpy as np
    from lab4lib.vision import MODEL_PATH, MODEL_URL

    source = ARTIFACTS / 'pose.jpg'
    provenance = {'fixture': download(FIXTURE_URL, source, 8 * 1024 * 1024)}
    if provenance['fixture']['sha256'] != FIXTURE_SHA256:
        raise RuntimeError('Public MediaPipe pose fixture checksum changed; refusing camera scenario')
    # Download even when a developer cache exists: CI records the actual model.
    MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    provenance['model'] = download(MODEL_URL, MODEL_PATH, 32 * 1024 * 1024)
    image = cv2.imread(str(source))
    if image is None:
        raise RuntimeError('Public MediaPipe fixture is not a decodable image')
    height, width = image.shape[:2]
    scale = min(640 / width, 480 / height)
    resized = cv2.resize(image, (round(width * scale), round(height * scale)))
    visible = np.full((480, 640, 3), 255, dtype=np.uint8)
    y, x = (480 - resized.shape[0]) // 2, (640 - resized.shape[1]) // 2
    visible[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    blank = np.full_like(visible, 255)
    frames = [cv2.cvtColor(frame, cv2.COLOR_BGR2YUV_I420).tobytes()
              for frame in (visible, blank)]
    camera = ARTIFACTS / 'visible-blank-visible.y4m'
    phases = [(VISIBLE_SECONDS, 0), (BLANK_SECONDS, 1), (VISIBLE_SECONDS, 0)]
    with camera.open('wb') as output:
        output.write(f'YUV4MPEG2 W640 H480 F{FPS}:1 Ip A1:1 C420jpeg\n'.encode())
        for seconds, index in phases:
            for _ in range(seconds * FPS):
                output.write(b'FRAME\n')
                output.write(frames[index])
    provenance['camera'] = {'width': 640, 'height': 480, 'fps': FPS,
                            'phases': [['visible', VISIBLE_SECONDS],
                                       ['blank', BLANK_SECONDS],
                                       ['visible', VISIBLE_SECONDS]]}
    (ARTIFACTS / 'sources.json').write_text(json.dumps(provenance, indent=2))
    print(f'Fixture/model provenance: {json.dumps(provenance)}', flush=True)
    return camera.resolve()


class Observation:
    """Observe actual WebSocket traffic without modifying the page or app."""

    def __init__(self, name, log):
        self.name, self.log = name, log
        self.state = None
        self.messages = []
        self.preview_count = 0
        self.last_state_at = 0

    def attach(self, page):
        page.on('websocket', lambda socket: socket.on('framereceived', self.receive))
        page.on('console', lambda message: self.record('console',
                                                     f'{message.type}: {message.text}'))
        page.on('pageerror', lambda error: self.record('pageerror', str(error)))
        page.on('requestfailed', lambda request: self.record('requestfailed',
                                                             f'{request.url}: {request.failure}'))

    def record(self, kind, payload):
        self.log.write(json.dumps({'time': time.monotonic(), 'page': self.name,
                                   'kind': kind, 'payload': payload}) + '\n')
        self.log.flush()

    def receive(self, payload):
        if isinstance(payload, bytes):
            self.preview_count += 1
            return
        message = json.loads(payload)
        self.record('websocket', message)
        self.messages.append(message)
        if message.get('type') == 'state':
            self.state = message
            self.last_state_at = time.monotonic()

    def robot(self):
        return (self.state or {}).get('robot', {})


async def until(label, predicate, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            print(f'PASS: {label}', flush=True)
            return
        await asyncio.sleep(0.05)
    raise AssertionError(f'Timed out after {timeout:g}s: {label}')


async def scenario(camera, *, camera_only=False):
    import cv2
    import numpy as np
    from aiohttp import web
    from playwright.async_api import Error as PlaywrightError, async_playwright
    from rtde_receive import RTDEReceiveInterface

    import student
    import student_config as config
    from lab4lib.session import LOG_COLUMNS
    from web_lab import create_app

    original = student.elbow_angle
    # Direct rejection is supplemental; the real camera rejection is below.
    try:
        original((0, 0), (0, 1), (1, 1))
    except NotImplementedError:
        pass
    else:
        raise AssertionError('CI requires the original student exercise to remain unimplemented')
    synthetic = {'angle': math.pi / 2, 'calls': 0}

    def constant_exercise_output(*points):
        # Deliberately no geometry. The real PoseTracker validated these points.
        synthetic['calls'] += 1
        return synthetic['angle']

    runner = web.AppRunner(create_app(robot_host=HOST, output_dir=ARTIFACTS / 'outputs'))
    receive = browser = context = page = playwright = None
    page_errors = []
    wire_log = (ARTIFACTS / 'browser.jsonl').open('w')
    joint_log = (ARTIFACTS / 'actual-joints.jsonl').open('w')
    try:
        await runner.setup()
        site = web.TCPSite(runner, HOST, 0)
        await site.start()
        port = runner.addresses[0][1]
        origin = f'http://{HOST}:{port}'
        if not camera_only:
            receive = await asyncio.to_thread(RTDEReceiveInterface, HOST, config.CONTROL_FREQUENCY)

        def actual():
            q = np.array(receive.getActualQ())
            joint_log.write(json.dumps({'time': time.monotonic(), 'q': q.tolist(),
                                        'qd': receive.getActualQd()}) + '\n')
            joint_log.flush()
            return q

        def arrived(target, tolerance=0.035):
            return (np.max(np.abs(actual() - target)) < tolerance
                    and np.max(np.abs(receive.getActualQd())) < 0.02)

        async def stationary(label, duration=1.0):
            await asyncio.sleep(0.3)  # Allow actual servoStop deceleration.
            baseline = actual()
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if np.max(np.abs(actual() - baseline)) > 0.025:
                    raise AssertionError(f'{label}: actual joints moved from {baseline}')
                await asyncio.sleep(0.05)
            print(f'PASS: {label} {baseline.tolist()}', flush=True)

        async def save_evidence(*names):
            await page.get_by_role('button', name='Save evidence', exact=True).click()
            for name in names:
                link = page.locator('#downloads a').filter(has_text=name)
                await link.wait_for()
                async with page.expect_download() as event:
                    await link.click()
                saved = await event.value
                await saved.save_as(str(ARTIFACTS / name))
                if name.endswith('.jpg') and cv2.imread(str(ARTIFACTS / name)) is None:
                    raise AssertionError(f'Downloaded {name} is not a real image')

        async def reject_second_tab():
            # A real second tab cannot replace or mutate the active owner.
            other = await context.new_page()
            other.set_default_timeout(15000)
            contender = Observation('second-tab', wire_log)
            contender.attach(other)
            await other.goto(origin)
            before = owner.preview_count
            enabled = owner.robot()['enabled']
            await other.get_by_role('button', name='Start camera', exact=True).click()
            await until('second controlling tab receives a clear ownership error',
                        lambda: any(message.get('type') == 'error' for message in contender.messages))
            if not (await other.locator('#error').inner_text()).strip():
                raise AssertionError('Ownership rejection is not visible to the second tab')
            await until('single owner keeps processing after rejected tab',
                        lambda: owner.preview_count > before + 2)
            if owner.robot()['enabled'] != enabled:
                raise AssertionError('Rejected second tab changed owner control state')
            await other.close()

        async with asyncio.timeout(270):
            playwright = await async_playwright().start()
            browser = await playwright.chromium.launch(headless=True, args=[
                '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream',
                f'--use-file-for-fake-video-capture={camera}', '--autoplay-policy=no-user-gesture-required',
            ])
            context = await browser.new_context(permissions=['camera'], accept_downloads=True)
            await context.tracing.start(screenshots=True, snapshots=True, sources=True)
            page = await context.new_page()
            page.set_default_timeout(15000)
            page.on('pageerror', lambda error: page_errors.append(str(error)))
            owner = Observation('owner', wire_log)
            owner.attach(page)
            await page.goto(origin)
            await page.get_by_role('button', name='Start camera', exact=True).click()
            camera_started = time.monotonic()
            await until('actual camera produces server JPEG previews', lambda: owner.preview_count >= 3, 30)
            await page.locator('#preview').wait_for(state='visible')
            if not await page.locator('#preview').evaluate('(image) => image.naturalWidth > 0 && image.naturalHeight > 0'):
                raise AssertionError('Annotated camera preview did not decode in Chromium')
            if owner.state['task'] != 1:
                raise AssertionError('Task 1 must be the default')
            for width, height in ((1280, 720), (390, 844)):
                await page.set_viewport_size({'width': width, 'height': height})
                stop = await page.get_by_role('button', name='Stop', exact=True).bounding_box()
                if (stop is None or stop['x'] < 0 or stop['y'] < 0
                        or stop['x'] + stop['width'] > width
                        or stop['y'] + stop['height'] > height):
                    raise AssertionError('Stop must stay in view alongside the camera on desktop/mobile')
            await page.set_viewport_size({'width': 1280, 'height': 720})
            await save_evidence('task1_preview.jpg')

            await page.locator('#task').select_option('2')
            await until('real pose reaches original Task 2 exercise rejection',
                        lambda: owner.state['task'] == 2 and owner.state['angles'] is None
                        and not owner.state['tracking'], 20)
            if owner.robot()['enabled']:
                raise AssertionError('Unimplemented exercise enabled robot control')
            await page.screenshot(path=str(ARTIFACTS / 'exercise-blocked.png'), full_page=True)
            student.elbow_angle = constant_exercise_output
            print('CI LIMITATION: configurable constant exercise output; no student angle solution.', flush=True)
            await until('real MediaPipe landmarks reach CI-only constant exercise boundary',
                        lambda: synthetic['calls'] >= 3 and owner.state['angles'] is not None)
            await save_evidence('task2_preview.jpg')
            if camera_only:
                preview_before = owner.preview_count
                await asyncio.sleep(1.0)
                await until('off-DOM camera video continues producing fresh server previews',
                            lambda: owner.preview_count >= preview_before + 5, 10)
                await reject_second_tab()
                await page.screenshot(path=str(ARTIFACTS / 'camera-only.png'), full_page=True)
                await page.get_by_role('button', name='Stop camera', exact=True).click()
                if page_errors:
                    raise AssertionError(f'Browser uncaught errors: {page_errors}')
                print('PASS: camera-only real Chromium / MediaPipe / stub / evidence / ownership; no robot connections.', flush=True)
                return

            await page.locator('#task').select_option('3')
            await until('Task 3 selected and disarmed', lambda: owner.state['task'] == 3
                        and not owner.robot()['enabled'])
            before_home = actual()
            if np.max(np.abs(before_home - config.HOME_Q)) < 0.12:
                raise AssertionError('Native scenario must leave actual joints away from HOME')
            await page.get_by_role('button', name='HOME', exact=True).click()
            await until('browser HOME completes real joint movement',
                        lambda: owner.robot().get('initialized') and not owner.robot()['busy']
                        and arrived(config.HOME_Q, 0.015), 75)
            if np.max(np.abs(actual() - before_home)) < 0.12:
                raise AssertionError('Browser HOME did not move actual joints')
            await until('fresh visible pose before calibration', lambda: owner.state['tracking'])
            await page.get_by_role('button', name='Calibrate', exact=True).click()
            await until('browser calibration acknowledged', lambda: owner.state['calibrated'])
            await page.get_by_role('button', name='Enable', exact=True).click()
            await until('browser explicitly enables control', lambda: owner.robot()['enabled'])
            synthetic['angle'] = math.pi / 2 + 0.36
            positive = np.array(config.HOME_Q, dtype=float)
            positive[2] += config.ELBOW_GAIN * 0.36
            negative = np.array(config.HOME_Q, dtype=float)
            negative[2] -= config.ELBOW_GAIN * 0.36
            await until('browser frames cause actual servo motion', lambda: arrived(positive), 20)
            await reject_second_tab()

            # The camera is an actual fixed Y4M capture device, not injected frames.
            # A long first visible phase leaves bounded time for real HOME setup.
            remaining = VISIBLE_SECONDS - (time.monotonic() - camera_started)
            if remaining < 3:
                raise AssertionError('Browser setup overran the first visible camera phase')
            await asyncio.sleep(max(0, remaining - 1.5))
            synthetic['angle'] = math.pi / 2 - 0.36
            await until('actual joints move before blank camera phase',
                        lambda: actual()[2] < positive[2] - 0.025, 5)
            await until('blank camera pauses without disarming',
                        lambda: not owner.state['tracking'] and owner.robot()['enabled']
                        and 'Tracking paused' in owner.robot()['message'], 15)
            await stationary('blank camera holds actual joints', 1.2)
            paused = actual()
            # Change the exercise constant while blank; only a newly detected
            # real pose can propagate this target back through the shared session.
            synthetic['angle'] = math.pi / 2 + 0.36
            await until('returned visible pose auto-resumes without another Enable',
                        lambda: owner.state['tracking'] and owner.robot()['enabled']
                        and arrived(positive), BLANK_SECONDS + 20)
            if np.max(np.abs(actual() - paused)) < 0.025:
                raise AssertionError('Auto-resume did not move actual joints')
            await save_evidence('task3_preview.jpg', 'task3_control.csv')
            rows = list(csv.reader(io.StringIO((ARTIFACTS / 'task3_control.csv').read_text())))
            if not rows or rows[0] != LOG_COLUMNS or len(rows) < 4:
                raise AssertionError('Downloaded CSV lacks real shared-session observations')
            if not any(row[2] == 'True' and all(row[-6:]) for row in rows[1:]):
                raise AssertionError('CSV does not contain enabled actual robot joints')

            await page.get_by_role('button', name='Stop', exact=True).click()
            await until('Stop explicitly disarms', lambda: not owner.robot()['enabled']
                        and not owner.robot()['busy'])
            synthetic['angle'] = math.pi / 2 - 0.36
            await until('fresh changed target still arrives after Stop',
                        lambda: owner.state['target'] is not None
                        and abs(owner.state['target'][2] - negative[2]) < 0.005)
            await stationary('Stop never automatically rearms on fresh frames', 1.2)
            if owner.robot()['enabled']:
                raise AssertionError('Fresh frames rearmed Stop')
            await page.get_by_role('button', name='Enable', exact=True).click()
            await until('explicit Enable restores actual servo motion', lambda: arrived(negative), 20)
            synthetic['angle'] = math.pi / 2 + 0.36
            await until('actual joints moving before browser disconnect',
                        lambda: actual()[2] > negative[2] + 0.025)
            await page.screenshot(path=str(ARTIFACTS / 'enabled.png'), full_page=True)
            # Crash the renderer: unlike page.close(), this cannot run pagehide's
            # graceful camera_stop action. Cleanup must handle transport loss.
            crashed_page, page = page, None
            cdp = await context.new_cdp_session(crashed_page)
            async with crashed_page.expect_event('crash', timeout=5000):
                try:
                    await asyncio.wait_for(cdp.send('Page.crash'), 5)
                except (PlaywrightError, TimeoutError):
                    pass  # The observed crash event, not this exception, is the proof.
            await until('page disconnect stops actual robot velocity',
                        lambda: np.max(np.abs(receive.getActualQd())) < 0.02, 10)
            await stationary('page disconnect holds actual joints', 1.0)

            page = await context.new_page()
            page.set_default_timeout(15000)
            page.on('pageerror', lambda error: page_errors.append(str(error)))
            reconnected = Observation('reconnected', wire_log)
            reconnected.attach(page)
            await page.goto(origin)
            await page.get_by_role('button', name='Start camera', exact=True).click()
            await until('new browser owner obtains real camera preview',
                        lambda: reconnected.preview_count >= 3, 30)
            if (reconnected.robot().get('enabled') or reconnected.state['calibrated']
                    or reconnected.robot().get('initialized')):
                raise AssertionError('Reconnect restored old controller/calibration state')
            await stationary('reconnect remains disarmed with fresh camera frames', 1.0)
            await page.get_by_role('button', name='Stop camera', exact=True).click()
            await page.screenshot(path=str(ARTIFACTS / 'reconnected-disarmed.png'), full_page=True)
            if page_errors:
                raise AssertionError(f'Browser uncaught errors: {page_errors}')
            print('PASS: real browser camera / MediaPipe / URSim motion scenarios', flush=True)
    finally:
        student.elbow_angle = original
        (ARTIFACTS / 'student-boundary.json').write_text(json.dumps({
            'original_stub_rejected': True,
            'ci_constant_calls': synthetic['calls'], 'limitation': 'No student angle math tested',
        }, indent=2))
        try:
            if page is not None and not page.is_closed():
                await page.screenshot(path=str(ARTIFACTS / 'final.png'), full_page=True, timeout=5000)
        finally:
            try:
                if context is not None:
                    await context.tracing.stop(path=str(ARTIFACTS / 'trace.zip'))
                    await context.close()
                if browser is not None:
                    await browser.close()
                if playwright is not None:
                    await playwright.stop()
            finally:
                try:
                    await runner.cleanup()
                finally:
                    if receive is not None:
                        receive.disconnect()
                    wire_log.close()
                    joint_log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-only', action='store_true',
                        help='Run real browser/MediaPipe checks without opening any robot connection')
    args = parser.parse_args()
    if not args.camera_only:
        if os.environ.get('LAB4_URSIM_CI') != '1' or os.environ.get('GITHUB_ACTIONS') != 'true':
            raise SystemExit('Refusing browser robot motion outside disposable GitHub URSim CI.')
        if os.environ.get('URSIM_HOST', HOST) != HOST:
            raise SystemExit('Refusing non-loopback URSIM_HOST; browser CI only uses 127.0.0.1.')
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(level=logging.INFO, handlers=[
        logging.StreamHandler(), logging.FileHandler(ARTIFACTS / 'server.log'),
    ])
    camera = prepare_camera()
    try:
        asyncio.run(asyncio.wait_for(scenario(camera, camera_only=args.camera_only), timeout=300))
    finally:
        # The generated camera is large; fixture + hash + phases reproduce it.
        camera.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
