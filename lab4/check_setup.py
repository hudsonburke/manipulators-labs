"""Diagnose the native Lab 4 environment; robot checks are opt-in and read-only."""

import argparse
import importlib
from importlib import metadata
import math
import os
from pathlib import Path
import platform
import socket
import struct
import subprocess
import sys
import time
import traceback


PROJECT = Path(__file__).resolve().parent
PORTS = ((29999, 'Dashboard'), (30003, 'URScript upload'), (30004, 'RTDE'))
DASHBOARD_QUERIES = ('robotmode', 'safetymode', 'programState')


def check_environment():
    """Import extensions without constructing drivers, opening a camera, or connecting."""
    expected = Path(os.environ.get('UV_PROJECT_ENVIRONMENT', PROJECT / '.venv'))
    if not expected.is_absolute():
        expected = PROJECT / expected
    print(f'Interpreter: {sys.executable}')
    print(f'Python: {platform.python_version()} ({platform.python_implementation()})')
    print(f'Platform: {platform.system()} {platform.release()} / {platform.machine()} / '
          f'{struct.calcsize("P") * 8}-bit')
    print(f'Project: {PROJECT}')
    print(f'Environment: {sys.prefix}; expected project environment: {expected}')
    print(f'VIRTUAL_ENV: {os.environ.get("VIRTUAL_ENV", "not set (normal with uv run)")}')
    ok = True
    if sys.version_info[:2] != (3, 12):
        print('FAIL: Lab 4 requires Python 3.12. Run uv sync --locked --python 3.12 in lab4.')
        ok = False
    if Path(sys.prefix).resolve() != expected.resolve():
        print('FAIL: This is not the project interpreter. In lab4 run: uv run python check_setup.py')
        ok = False
    try:
        dist = metadata.distribution('ur-rtde')
        print(f'ur-rtde distribution: {dist.version}; location: {dist.locate_file("")}')
    except metadata.PackageNotFoundError:
        print('FAIL: ur-rtde distribution metadata is missing in this interpreter.')
        ok = False
    for name in ('rtde_control', 'rtde_receive'):
        try:
            module = importlib.import_module(name)
            print(f'OK import {name}: {getattr(module, "__file__", "built-in")}')
        except Exception as exc:
            missing = isinstance(exc, ModuleNotFoundError) and exc.name == name
            kind = 'module missing' if missing else 'extension/dependency load failed'
            print(f'FAIL import {name} ({kind}):')
            print(''.join(traceback.format_exception(exc)), end='')
            ok = False
    if not ok:
        print('Next: cd to lab4; uv sync --locked; uv run python check_setup.py. '
              'For DLL/dlopen errors retain the original traceback above and check '
              'the OS/architecture-specific installation instructions; reinstalling '
              'into a different Python will not fix this interpreter.')
    return ok


def read_dashboard_line(connection, timeout):
    """Read one bounded ASCII protocol line, not an unbounded socket file readline."""
    deadline = time.monotonic() + timeout
    line = bytearray()
    while len(line) < 4096:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('dashboard line deadline exceeded')
        connection.settimeout(remaining)
        char = connection.recv(1)
        if not char:
            raise ConnectionError('dashboard closed before a complete newline-terminated reply')
        if char == b'\n':
            return line.decode('ascii').strip()
        line.extend(char)
    raise ValueError('dashboard reply exceeded 4096 bytes')


def check_dashboard(connection, host, timeout):
    endpoint = f'{host}:29999'
    try:
        banner = read_dashboard_line(connection, timeout)
        if 'dashboard' not in banner.lower():
            raise ValueError(f'unexpected dashboard banner: {banner!r}')
        print(f'{endpoint} banner: {banner}')
    except (OSError, ConnectionError, ValueError) as exc:
        print(f'FAIL {endpoint} banner: {type(exc).__name__}: {exc}')
        return False
    ok = True
    for command in DASHBOARD_QUERIES:
        try:
            connection.settimeout(timeout)
            connection.sendall((command + '\n').encode('ascii'))
            reply = read_dashboard_line(connection, timeout)
            print(f'{endpoint} {command}: {reply}')
            if command == 'robotmode':
                if not reply.startswith('Robotmode:'):
                    raise ValueError('expected Robotmode: <mode>')
                if reply.split(':', 1)[1].strip() != 'RUNNING':
                    print('FAIL: Robot is not RUNNING. In PolyScope inspect safety dialogs, '
                          'power on, and release brakes manually; this diagnostic changes nothing.')
                    ok = False
            elif command == 'safetymode':
                if not reply.startswith('Safetymode:'):
                    raise ValueError('expected Safetymode: <mode>')
                if reply.split(':', 1)[1].strip() != 'NORMAL':
                    print('FAIL: Lab 4 expects NORMAL safety mode. Inspect PolyScope; '
                          'do not automatically clear a protective/emergency stop.')
                    ok = False
            elif not reply or reply.split()[0] not in ('STOPPED', 'PLAYING', 'PAUSED'):
                raise ValueError('expected STOPPED, PLAYING, or PAUSED program state')
        except (OSError, ConnectionError, ValueError) as exc:
            print(f'FAIL {endpoint} {command}: {type(exc).__name__}: {exc}')
            return False  # Do not confuse a delayed reply with the next command's reply.
    return ok


def check_rtde_notices(connection, host, timeout):
    """Only listen for bounded startup notices; never negotiate RTDE or send data."""
    endpoint = f'{host}:30004'
    deadline = time.monotonic() + timeout
    consumed = 0

    def read_exact(size, allow_silence=False):
        data = bytearray()
        while len(data) < size:
            remaining = deadline - time.monotonic()
            try:
                if remaining <= 0:
                    raise TimeoutError('RTDE notice deadline exceeded')
                connection.settimeout(remaining)
                chunk = connection.recv(size - len(data))
            except TimeoutError:
                if allow_silence and not data:
                    return None
                raise
            if not chunk:
                raise ConnectionError('RTDE closed before a complete startup frame')
            data.extend(chunk)
        return data

    try:
        for _ in range(16):
            header = read_exact(3, allow_silence=True)
            if header is None:
                return True  # A healthy RTDE server need not send unsolicited bytes.
            length, kind = struct.unpack('!HB', header)
            if length < 3 or consumed + length > 4096:
                raise ValueError('RTDE startup frames have invalid length or exceed 4096 bytes')
            consumed += length
            body = read_exact(length - 3)
            if kind == 77:  # Legacy unsolicited TEXT_MESSAGE: level byte, then ASCII.
                if len(body) < 2:
                    raise ValueError('RTDE TEXT_MESSAGE has no level/text')
                text = body[1:].decode('ascii')
                print(f'{endpoint} RTDE notice (level {body[0]}): {text}')
                if text == ('SafetySetup has not been confirmed yet, '
                            'therefore no data will be send yet'):
                    print("FAIL: In PolyScope, load/reload the intended installation and "
                          "review 'Confirm Safety Configuration' if prompted. Dashboard "
                          'RUNNING/NORMAL does not establish this prerequisite; '
                          'this diagnostic changes nothing.')
                    return False
        raise ValueError('RTDE startup notices exceeded 16 frames')
    except (OSError, ConnectionError, ValueError) as exc:
        print(f'FAIL {endpoint} RTDE startup notice: {type(exc).__name__}: {exc}')
        return False


def check_robot(host, timeout):
    print(f'Read-only robot checks: {host}; timeout {timeout:g}s per connect/line.')
    ok = True
    for port, purpose in PORTS:
        try:
            with socket.create_connection((host, port), timeout=timeout) as connection:
                print(f'OK TCP {host}:{port} ({purpose})')
                if port == 29999:
                    ok = check_dashboard(connection, host, timeout) and ok
                elif port == 30004:
                    ok = check_rtde_notices(connection, host, timeout) and ok
        except OSError as exc:
            print(f'FAIL TCP {host}:{port} ({purpose}): {type(exc).__name__}: {exc}')
            ok = False
    print('TCP acceptance alone does not prove motion readiness. STOPPED before Task 3 '
          'initialization is normal; ur_rtde uploads its own program. No power, brake, '
          'program, safety, camera, or motion commands were sent.')
    if not ok:
        print('Next: check docker compose ps and docker compose logs; verify all three '
              'published ports and the exact --host above. Native host Python uses '
              'localhost for local Docker; another container/remote Docker has a '
              'different localhost. Inspect PolyScope at http://localhost:6080/vnc.html '
              'for a local simulator. --host overrides URSIM_HOST.')
    return ok


def positive_timeout(value):
    number = float(value)
    if not math.isfinite(number) or not 0 < number <= 30:
        raise argparse.ArgumentTypeError('timeout must be finite and in (0, 30] seconds')
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', action='store_true', help='also check robot TCP ports and query dashboard state (no motion)')
    parser.add_argument('--host', default=os.environ.get('URSIM_HOST', 'localhost'),
                        help='robot hostname/IP (default: $URSIM_HOST, else localhost)')
    parser.add_argument('--timeout', type=positive_timeout, default=2.0,
                        help='seconds per robot connection/reply (default: 2; max: 30)')
    parser.add_argument('--_robot-probe', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    host = args.host.strip()
    if not host or '://' in host or '/' in host or any(c.isspace() or ord(c) < 32 for c in host):
        parser.error('--host must be a hostname or IP address, not a URL')
    if args._robot_probe:
        return 0 if check_robot(host, args.timeout) else 1
    ok = check_environment()
    if args.robot:
        # Also bound DNS resolution and multiple address attempts, which socket's
        # individual connect timeout does not cover. A hung probe cannot hang the CLI.
        budget = 10 * args.timeout + 2
        sys.stdout.flush()
        try:
            result = subprocess.run(
                [sys.executable, '-u', str(Path(__file__).resolve()), '--_robot-probe',
                 '--host', host, '--timeout', str(args.timeout)],
                stdin=subprocess.DEVNULL, timeout=budget, check=False)
            ok = result.returncode == 0 and ok
        except subprocess.TimeoutExpired:
            print(f'FAIL robot checks for {host}:29999, {host}:30003, {host}:30004 '
                  f'exceeded {budget:g}s total deadline (including DNS).')
            ok = False
    else:
        print('Robot/camera untouched. Add --robot --host localhost for read-only network/state checks.')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
