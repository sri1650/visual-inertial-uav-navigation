#!/usr/bin/env bash
set -euo pipefail
: "${PX4_DIR:?Set PX4_DIR to a PX4-Autopilot checkout}"
cd "$PX4_DIR"
echo "Launching PX4 SITL with Gazebo. Exact vehicle/world selection is finalised during M6-T01."
make px4_sitl gz_x500
