#!/usr/bin/env bash
set -euo pipefail

required=(
  goals.py
  task1.py
  task2.py
  task3.py
  report.md
  goals.csv
  task1_joint_q.csv
  task1_results.csv
  task2_joint_q.csv
  task2_ik_diagnostics.csv
  task3_results.csv
)

missing=()
for file in "${required[@]}"; do
  [[ -f "$file" ]] || missing+=("$file")
done

if (( ${#missing[@]} )); then
  printf 'Missing required submission files:\n' >&2
  printf '  %s\n' "${missing[@]}" >&2
  printf 'Run Tasks 1–3 and complete the report before submitting.\n' >&2
  exit 1
fi

rm -f submission.zip
zip -r submission.zip "${required[@]}"
echo "Created submission.zip. Right-click it in the file explorer and choose 'Download'."
