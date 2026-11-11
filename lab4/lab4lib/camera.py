"""Camera capture, the preview window, and its keys."""

import cv2

FRAME_WIDTH, FRAME_HEIGHT = 640, 480


class Camera:
    """A native camera scaled to a 640x480 preview, plus its window."""

    def __init__(self, index=0, title='Lab 4'):
        self.title = title
        self._capture = cv2.VideoCapture(index)
        if not self._capture.isOpened():
            raise RuntimeError(f'Cannot open camera {index}. Check OS camera permissions and --camera.')
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        # shortcut: backends may ignore buffer size; check latency on each camera.
        self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(title, 960, 700)

    def read(self):
        """Return the next BGR frame, scaled down to at most 640x480."""
        ok, frame = self._capture.read()
        if not ok or frame is None or frame.size == 0:
            raise RuntimeError('Camera read failed. Reconnect the camera and restart the task.')
        height, width = frame.shape[:2]
        scale = min(FRAME_WIDTH / width, FRAME_HEIGHT / height, 1.0)
        if scale < 1.0:
            frame = cv2.resize(frame, (max(1, round(width * scale)), max(1, round(height * scale))),
                               interpolation=cv2.INTER_AREA)
        return frame

    def show(self, image):
        """Display one frame and return the pressed key code (or -1)."""
        cv2.imshow(self.title, image)
        return cv2.waitKey(1) & 0xff

    def closed(self):
        return cv2.getWindowProperty(self.title, cv2.WND_PROP_VISIBLE) < 1

    def close(self):
        self._capture.release()
        cv2.destroyAllWindows()
