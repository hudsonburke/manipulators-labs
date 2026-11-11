"""Package the seven required Lab 4 files using only the Python standard library."""

from pathlib import Path
import sys
from zipfile import ZIP_DEFLATED, ZipFile


REQUIRED = (
    "student.py",
    "student_config.py",
    "report.md",
    "outputs/task1_preview.jpg",
    "outputs/task2_preview.jpg",
    "outputs/task3_preview.jpg",
    "outputs/task3_control.csv",
)


def main():
    directory = Path(__file__).resolve().parent
    missing = [name for name in REQUIRED if not (directory / name).is_file()]
    if missing:
        print("Missing required submission files:", file=sys.stderr)
        for name in missing:
            print(f"  {directory / name}", file=sys.stderr)
        print(
            "Run task1_landmarks.py, task2_elbow_angle.py, and task3_control.py, press "
            "1, 2, and 3 to save each task's evidence, and complete report.md before "
            "retrying. Any existing submission.zip was left unchanged.",
            file=sys.stderr,
        )
        return 1

    archive = directory / "submission.zip"
    with ZipFile(archive, "w", compression=ZIP_DEFLATED) as package:
        for name in REQUIRED:
            package.write(directory / name, arcname=name)
    print(f"Created {archive}. Upload this ZIP to the Lab 4 Canvas assignment.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
