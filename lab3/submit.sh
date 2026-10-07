#!/usr/bin/env bash
set -euo pipefail

required=(
  student_config.py
  report.md
  outputs/task1_rrt_tree.png
  outputs/task1_rrt_tree.csv
  outputs/task1_rrt_path.csv
  outputs/task2_raw_path.csv
  outputs/task2_simplified_path.csv
  outputs/task2_closest_partial_path.csv
  outputs/task2_planner_projection.png
  outputs/task2_attempt_trajectory.csv
  outputs/task2_joint_position_over_time.png
)

missing=()
for file in "${required[@]}"; do
  [[ -f "$file" ]] || missing+=("$file")
done

if (( ${#missing[@]} )); then
  printf 'Missing required submission files:\n' >&2
  printf '  %s\n' "${missing[@]}" >&2
  printf 'Run Tasks 1–2 and complete the report before submitting.\n' >&2
  exit 1
fi

rm -f submission.zip
zip -r submission.zip "${required[@]}"
echo "Created submission.zip. Right-click it in the file explorer and choose 'Download'."
