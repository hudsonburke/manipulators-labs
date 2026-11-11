"""Task 2: show the elbow angle calculated from your landmarks.

Run from lab4: uv run python task2_elbow_angle.py [--camera N]
"""

import argparse
import math
import sys

from lab4lib.camera import Camera
from lab4lib.evidence import save_preview
from lab4lib.overlay import with_status
from lab4lib.vision import PoseTracker


def main(argv=None):
    parser = argparse.ArgumentParser(description='Task 2: show the elbow angle from your landmarks.')
    parser.add_argument('--camera', type=int, default=0, help='camera index (default: 0)')
    args = parser.parse_args(argv)
    camera = tracker = None
    try:
        tracker = PoseTracker()
        camera = Camera(args.camera, 'Lab 4 - Task 2')
        while True:
            annotated, angles, message = tracker.process(camera.read())
            shown = with_status(annotated, [
                'Task 2 - elbow angle', message,
                'Elbow [deg, rad]: ' + ('not implemented yet' if angles is None
                                        else f"{math.degrees(angles['elbow']):.1f}, "
                                             f"{angles['elbow']:.3f}"),
                '2: save Task 2 evidence | Q or Esc: quit'])
            key = camera.show(shown)
            if key in (27, ord('q'), ord('Q')) or camera.closed():
                return 0
            if key == ord('2'):
                print('Saved', save_preview(shown, 2), flush=True)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f'Task 2 error: {exc}', file=sys.stderr, flush=True)
        return 1
    finally:
        if camera is not None:
            camera.close()
        if tracker is not None:
            tracker.close()


if __name__ == '__main__':
    raise SystemExit(main())
