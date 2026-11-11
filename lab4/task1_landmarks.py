"""Task 1: show the left-arm MediaPipe landmarks.

Run from lab4: uv run python task1_landmarks.py [--camera N]
"""

import argparse
import sys

from lab4lib.camera import Camera
from lab4lib.evidence import save_preview
from lab4lib.overlay import with_status
from lab4lib.vision import PoseTracker


def main(argv=None):
    parser = argparse.ArgumentParser(description='Task 1: show the left-arm MediaPipe landmarks.')
    parser.add_argument('--camera', type=int, default=0, help='camera index (default: 0)')
    args = parser.parse_args(argv)
    camera = tracker = None
    try:
        tracker = PoseTracker()
        camera = Camera(args.camera, 'Lab 4 - Task 1')
        while True:
            annotated, _, message = tracker.process(camera.read())
            shown = with_status(annotated, ['Task 1 - arm landmarks', message,
                                            '1: save Task 1 evidence | Q or Esc: quit'])
            key = camera.show(shown)
            if key in (27, ord('q'), ord('Q')) or camera.closed():
                return 0
            if key == ord('1'):
                print('Saved', save_preview(shown, 1), flush=True)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f'Task 1 error: {exc}', file=sys.stderr, flush=True)
        return 1
    finally:
        if camera is not None:
            camera.close()
        if tracker is not None:
            tracker.close()


if __name__ == '__main__':
    raise SystemExit(main())
