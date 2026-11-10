#!/usr/bin/env bash
set -euo pipefail

required=(
  student.py
  student_config.py
  report.md
  outputs/task1_preview.jpg
  outputs/task2_preview.jpg
  outputs/task3_preview.jpg
  outputs/task3_control.csv
)

missing=()
for file in "${required[@]}"; do
  [[ -f "$file" ]] || missing+=("$file")
done

if (( ${#missing[@]} )); then
  printf 'Missing required submission files:\n' >&2
  printf '  %s\n' "${missing[@]}" >&2
  printf 'Save evidence for Tasks 1–3 and complete report.md before submitting.\n' >&2
  exit 1
fi

rm -f submission.zip
zip -r submission.zip "${required[@]}"
echo "Created submission.zip. Download it from the file explorer and upload it to Canvas."
