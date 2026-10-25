#!/usr/bin/env bash
set -euo pipefail

: "${PX4_DIR:?Set PX4_DIR to a PX4-Autopilot checkout}"

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
world=${PX4_GZ_WORLD:-default}
model=${PX4_SIM_MODEL:-gz_x500}
gz_prefix=${PX4_GZ_ENV_PREFIX:-"$PX4_DIR/Tools/setup/macos/.pixi/envs/default"}
log_dir=${PX4_EVIDENCE_DIR:-"$script_dir/evidence"}

if [[ ! -x "$gz_prefix/bin/gz" ]]; then
  echo "Gazebo was not found at $gz_prefix/bin/gz" >&2
  echo "Set PX4_GZ_ENV_PREFIX to the locked PX4 Gazebo environment." >&2
  exit 2
fi

export CONDA_PREFIX="$gz_prefix"
export CMAKE_PREFIX_PATH="$gz_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export PATH="$gz_prefix/bin:$PATH"
for activation in "$gz_prefix"/etc/conda/activate.d/*.sh; do
  [[ -f "$activation" ]] && source "$activation"
done

export PX4_GZ_MODELS="$PX4_DIR/Tools/simulation/gz/models"
export PX4_GZ_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"
export PX4_GZ_PLUGINS="$PX4_DIR/build/px4_sitl_default/src/modules/simulation/gz_plugins"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:$PX4_GZ_MODELS:$PX4_GZ_WORLDS"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:$PX4_GZ_PLUGINS"
export GZ_SIM_SERVER_CONFIG_PATH="$script_dir/server-headless.config"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export HEADLESS=1

mkdir -p "$log_dir"
gz_log="$log_dir/gazebo-server.log"
px4_log="$log_dir/px4-sitl.log"

"$gz_prefix/bin/gz" sim --verbose="${GZ_VERBOSE:-1}" -r -s \
  "$PX4_GZ_WORLDS/$world.sdf" >"$gz_log" 2>&1 &
gz_pid=$!

cleanup() {
  kill "$gz_pid" 2>/dev/null || true
  wait "$gz_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

for _ in {1..90}; do
  if "$gz_prefix/bin/gz" service -i --service "/world/$world/scene/info" 2>&1 \
    | grep -q "Service providers"; then
    break
  fi
  if ! kill -0 "$gz_pid" 2>/dev/null; then
    echo "Gazebo exited before the world became ready; see $gz_log" >&2
    exit 1
  fi
  sleep 1
done

if ! "$gz_prefix/bin/gz" service -i --service "/world/$world/scene/info" 2>&1 \
  | grep -q "Service providers"; then
  echo "Timed out waiting for Gazebo; see $gz_log" >&2
  exit 1
fi

echo "Gazebo world '$world' is ready; launching PX4 model '$model'."
cd "$PX4_DIR"
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$world" PX4_SIM_MODEL="$model" \
  make "PYTHON_EXECUTABLE=${PX4_PYTHON:-/usr/bin/python3}" px4_sitl "$model" \
  2>&1 | tee "$px4_log"
