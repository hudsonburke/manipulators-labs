"""Explicit evidence files written by the task scripts."""

import csv
from pathlib import Path

import cv2

OUTPUT = Path(__file__).resolve().parent.parent / 'outputs'


def save_preview(image, task):
    """Write ``outputs/task<task>_preview.jpg`` and return its path."""
    OUTPUT.mkdir(parents=True, exist_ok=True)
    path = OUTPUT / f'task{task}_preview.jpg'
    if not cv2.imwrite(str(path), image):
        raise OSError(f'Could not save {path.name}.')
    return path


def save_control_log(columns, rows):
    """Write ``outputs/task3_control.csv`` and return its path."""
    OUTPUT.mkdir(parents=True, exist_ok=True)
    path = OUTPUT / 'task3_control.csv'
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(rows)
    return path
