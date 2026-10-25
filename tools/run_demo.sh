#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_dir"
python_bin=${PYTHON:-"$repo_dir/.venv/bin/python"}
mode=${1:-offline}

if [[ ! -x "$python_bin" ]]; then
  echo "Python environment not found at $python_bin; follow docs/demo.md first." >&2
  exit 2
fi

case "$mode" in
  offline)
    mkdir -p benchmark-results
    "$python_bin" tools/pre_codex_gate.py --json-out benchmark-results/final-gate.json
    "$python_bin" benchmarks/video/run.py --frames 100 --json-out benchmark-results/video.json
    "$python_bin" benchmarks/faults/run.py --csv-out benchmark-results/fault-campaign.csv
    "$python_bin" benchmarks/resource_profile.py --frames 100 --json-out benchmark-results/resources.json
    "$python_bin" tools/verify_sitl_evidence.py
    ;;
  evidence)
    "$python_bin" tools/verify_sitl_evidence.py
    ;;
  mission)
    "$python_bin" tools/run_gps_denied_mission.py
    ;;
  failsafe)
    "$python_bin" tools/run_estimator_loss_failsafe.py
    ;;
  *)
    echo "usage: $0 {offline|evidence|mission|failsafe}" >&2
    exit 2
    ;;
esac
