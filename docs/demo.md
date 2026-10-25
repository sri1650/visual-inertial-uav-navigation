# Reproducible demo

The offline demo runs every project-owned deterministic check and benchmark. The live demo additionally requires the pinned PX4/Gazebo environment. Both paths are simulation or workstation validation only.

## Clean macOS setup

```bash
git clone https://github.com/sri1650/visual-inertial-uav-navigation.git
cd visual-inertial-uav-navigation
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e . pytest
brew install cmake eigen
./tools/run_demo.sh offline
```

The offline command writes machine-readable outputs to the ignored `benchmark-results/` directory and verifies the retained SITL acceptance evidence.

## Pinned PX4/Gazebo setup

```bash
git clone https://github.com/PX4/PX4-Autopilot.git ../PX4-Autopilot
git -C ../PX4-Autopilot checkout 22b15b91fba5e79f01248d1c644ef942382af517
../PX4-Autopilot/Tools/setup/macos.sh --sim-tools
PX4_DIR="$PWD/../PX4-Autopilot" ./simulation/px4/run_sitl.sh
```

Leave that terminal running. In a second terminal, activate this repository's environment and the PX4 locked Gazebo environment:

```bash
export PX4_DIR="$PWD/../PX4-Autopilot"
export CONDA_PREFIX="$PX4_DIR/Tools/setup/macos/.pixi/envs/default"
for f in "$CONDA_PREFIX"/etc/conda/activate.d/*.sh; do . "$f"; done
export PYTHONPATH="$CONDA_PREFIX/lib/python3.13/site-packages"
export GZ_IP=127.0.0.1
./tools/run_demo.sh mission
./tools/run_demo.sh failsafe
```

The mission command disables GNSS fusion, supplies live Gazebo pose through the MAVLink external-vision boundary, flies the retained five-waypoint route, lands and disarms. The failsafe command establishes a new hover, stops only external vision, and verifies PX4's Descend/blind-land action and disarm.

## Evidence-only review

```bash
./tools/run_demo.sh evidence
```

This checks and summarizes the retained mission and failsafe JSON without starting the simulator.
