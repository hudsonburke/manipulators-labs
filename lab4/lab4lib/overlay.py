"""Status panel drawn under the camera preview."""

import textwrap

import cv2

LINE_HEIGHT = 22
MIN_WIDTH = 960


def with_status(frame, lines):
    """Return a copy of ``frame`` with the wrapped ``lines`` appended below it."""
    width = max(MIN_WIDTH, frame.shape[1])
    wrapped = [part for line in lines
               for part in textwrap.wrap(line, width=max(40, (width - 20) // 8))]
    panel = cv2.copyMakeBorder(frame, 0, LINE_HEIGHT * len(wrapped) + 12, 0,
                               width - frame.shape[1],
                               cv2.BORDER_CONSTANT, value=(25, 25, 25))
    for index, line in enumerate(wrapped):
        cv2.putText(panel, line, (10, frame.shape[0] + LINE_HEIGHT * (index + 1)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return panel
