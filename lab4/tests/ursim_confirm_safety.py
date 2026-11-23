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
import sys
import time

from PIL import Image
from vncdotool import api

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from check_setup import check_rtde_notices
from ursim_smoke import prepare_simulator


ARTIFACTS = Path('ci-artifacts')
TITLE = 'confirmation of applied safety configuration'
BUTTON = 'confirm safety configuration'
LABELS = (TITLE, BUTTON, 'not now', 'power off', 'program robot', 'run program',
          'go to initialization screen', 'initialize robot', 'ok')


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
        ['tesseract', 'stdin', 'stdout', '--psm', '3', 'tsv'],
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
        selections = [words]
        for phrase in LABELS:
            count = len(phrase.split())
            for start in range(len(words) - count + 1):
                selected = words[start:start + count]
                if normalized(' '.join(word['text'] for word in selected)) == phrase:
                    selections.append(selected)
        for selected in selections:
            label = normalized(' '.join(word['text'] for word in selected))
            left = min(int(word['left']) for word in selected)
            top = min(int(word['top']) for word in selected)
            right = max(int(word['left']) + int(word['width']) for word in selected)
            bottom = max(int(word['top']) + int(word['height']) for word in selected)
            if (label, (left, top, right, bottom)) not in lines:
                lines.append((label, (left, top, right, bottom)))
    print(f'GUI {name}: {[label for label, _ in lines]}', flush=True)
    return lines


def confirm(client, address):
    """Load the installation through the observed UI and confirm if requested."""
    deadline = time.monotonic() + 120
    dismissed_power_prompt = opened_program = clicked_confirmation = False
    opened_initialization = finished_initialization = False

    def click(lines, label):
        boxes = [box for text, box in lines if text == label]
        if len(boxes) != 1:
            raise RuntimeError(f'Expected one visible {label!r} button, got {boxes}; no click sent')
        left, top, right, bottom = boxes[0]
        x, y = (left + right) // 4, (top + bottom) // 4
        print(f'Clicking observed {label!r} at ({x}, {y})', flush=True)
        client.mouseMove(x, y)
        client.mousePress(1)

    observation = 0
    while time.monotonic() < deadline:
        name = 'safety-initial' if observation == 0 else f'safety-step-{observation:02d}'
        lines = capture(client, name)
        labels = {text for text, _ in lines}
        if TITLE in labels and BUTTON in labels and not clicked_confirmation:
            click(lines, BUTTON)
            clicked_confirmation = True
        elif {'not now', 'power off'} <= labels and not dismissed_power_prompt:
            # The fresh image may show this before loading any installation.
            # Dismissing it does not power on or bypass the safety confirmation.
            click(lines, 'not now')
            dismissed_power_prompt = True
        elif {'run program', 'program robot'} <= labels and not opened_program:
            click(lines, 'program robot')
            opened_program = True
        elif {'go to initialization screen', 'power off'} <= labels and not opened_initialization:
            click(lines, 'go to initialization screen')
            opened_initialization = True
            # This is the same simulator-only power/brake preparation used by
            # the motion scenario; opening its real UI also loads installation.
            prepare_simulator()
        elif {'initialize robot', 'ok'} <= labels and opened_initialization and not finished_initialization:
            click(lines, 'ok')
            finished_initialization = True
        elif opened_program or clicked_confirmation:
            # Verify the real backend prerequisite, not just disappearing OCR text.
            # This listener sends no RTDE packets and changes no controller state.
            with socket.create_connection((str(address), 30004), timeout=2) as connection:
                if check_rtde_notices(connection, str(address), 0.5):
                    print('PASS: PolyScope installation loaded; RTDE no longer reports '
                          'unconfirmed safety configuration', flush=True)
                    return
        observation += 1
        time.sleep(1)
    raise RuntimeError('PolyScope did not complete installation/safety setup within 120s; '
                       'inspect safety-*.png/tsv; refusing motion')


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
            confirm(client, address)
    finally:
        api.shutdown()


if __name__ == '__main__':
    main()
