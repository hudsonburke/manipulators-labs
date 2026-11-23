"""CI-only: accept the fresh disposable URSim's safety dialog via its real VNC UI.

Dependencies live in the workflow's isolated environment, not the student project.
No safety configuration, controller state, or program files are written here.
"""

import argparse
import csv
import io
import ipaddress
import os
from pathlib import Path
import socket
import subprocess
import time

from PIL import Image
from vncdotool import api


ARTIFACTS = Path('ci-artifacts')
TITLE = 'confirmation of applied safety configuration'
BUTTON = 'confirm safety configuration'


def normalized(text):
    return ' '.join(''.join(char if char.isalnum() else ' ' for char in text.casefold()).split())


def capture(client, name):
    """Keep the unmodified framebuffer and OCR coordinates for every observation."""
    path = ARTIFACTS / f'{name}.png'
    client.captureScreen(str(path))
    with Image.open(path) as image:
        if image.size != (800, 600):
            raise RuntimeError(f'Unexpected URSim framebuffer {image.size}; inspect {path}; no click sent')
        # Enlarge small Swing text for OCR, but retain the original screenshot.
        enlarged = image.resize((1600, 1200))
        pixels = io.BytesIO()
        enlarged.save(pixels, format='PNG')
    result = subprocess.run(
        ['tesseract', 'stdin', 'stdout', '--psm', '11', 'tsv'],
        input=pixels.getvalue(), capture_output=True, check=True, timeout=10)
    text = result.stdout.decode('utf-8')
    (ARTIFACTS / f'{name}.tsv').write_text(text)
    groups = {}
    for word in csv.DictReader(io.StringIO(text), delimiter='\t'):
        if word['level'] != '5' or not word['text'].strip():
            continue
        key = tuple(word[field] for field in ('page_num', 'block_num', 'par_num', 'line_num'))
        groups.setdefault(key, []).append(word)
    lines = []
    for words in groups.values():
        label = normalized(' '.join(word['text'] for word in words))
        left = min(int(word['left']) for word in words)
        top = min(int(word['top']) for word in words)
        right = max(int(word['left']) + int(word['width']) for word in words)
        bottom = max(int(word['top']) + int(word['height']) for word in words)
        lines.append((label, (left, top, right, bottom)))
    print(f'GUI {name}: {[label for label, _ in lines]}', flush=True)
    return lines


def confirm(client):
    # Capture before any input. Startup polling never clicks a guessed coordinate.
    lines = capture(client, 'safety-initial')
    deadline = time.monotonic() + 90
    observation = 0
    while True:
        titles = [box for label, box in lines if label == TITLE]
        buttons = [box for label, box in lines if label == BUTTON]
        if len(titles) == len(buttons) == 1:
            break
        if time.monotonic() >= deadline:
            raise RuntimeError('Expected initial safety confirmation dialog not recognized within 90s; '
                               'inspect safety-*.png/tsv; no click sent')
        time.sleep(2)
        observation += 1
        lines = capture(client, f'safety-wait-{observation:02d}')

    title, button = titles[0], buttons[0]
    # OCR boxes are in the enlarged image. The real dialog's centered button is
    # below its title; refuse an unexpected layout rather than clicking elsewhere.
    if not (title[3] < button[1] and button[1] > 600
            and abs((title[0] + title[2]) - (button[0] + button[2])) < 160):
        raise RuntimeError(f'Unexpected safety dialog layout: title={title}, button={button}; no click sent')
    x, y = (button[0] + button[2]) // 4, (button[1] + button[3]) // 4
    print(f'Clicking observed "Confirm Safety Configuration" at ({x}, {y})', flush=True)
    client.mouseMove(x, y)
    client.mousePress(1)

    # One click only. Require a recognized subsequent PolyScope screen, not just
    # missing OCR text. A power-off prompt may overlay the main menu; Dashboard
    # initialization and real RTDE motion remain separate assertions.
    deadline = time.monotonic() + 20
    observation = 0
    while True:
        lines = capture(client, f'safety-after-{observation:02d}')
        labels = {label for label, _ in lines}
        if TITLE not in labels and BUTTON not in labels and {'run program', 'program robot'} <= labels:
            print('PASS: actual safety dialog dismissed; RTDE/motion readiness still unverified', flush=True)
            return
        if time.monotonic() >= deadline:
            raise RuntimeError('Safety confirmation click did not return to the expected main menu within 20s; '
                               'inspect safety-after-*.png/tsv; refusing motion')
        time.sleep(1)
        observation += 1


def main():
    if os.environ.get('LAB4_URSIM_CI') != '1':
        raise SystemExit('Refusing GUI input outside disposable URSim CI (LAB4_URSIM_CI=1 required).')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--container-ip', required=True)
    args = parser.parse_args()
    address = ipaddress.IPv4Address(args.container_ip)
    private_docker = any(address in ipaddress.IPv4Network(network)
                         for network in ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16'))
    if not private_docker:
        raise SystemExit('Refusing non-private Docker address; VNC must never be publicly exposed.')
    ARTIFACTS.mkdir(exist_ok=True)
    # Official image documents Docker-IP:5900. The captured image's x11vnc
    # invocation has no password/auth flags; use no password, not an invented default.
    deadline = time.monotonic() + 90
    while True:
        try:
            with socket.create_connection((str(address), 5900), timeout=2):
                break
        except OSError:
            if time.monotonic() >= deadline:
                raise RuntimeError('Disposable simulator VNC did not listen within 90s')
            time.sleep(1)
    try:
        with api.connect(f'{address}::5900', password=None, timeout=10) as client:
            confirm(client)
    finally:
        api.shutdown()


if __name__ == '__main__':
    main()
