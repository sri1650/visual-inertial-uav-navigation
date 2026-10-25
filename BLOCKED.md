# Blocked work

## Resolved: M6-T03 PX4 GPS-denied mission arming gate

- Date: 2026-10-25
- Task: `M6-T03`
- Exact command: `CONDA_PREFIX=/Users/sricharankumba/Documents/Codex/dependencies/PX4-main-setup/Tools/setup/macos/.pixi/envs/default PYTHONPATH=$CONDA_PREFIX/lib/python3.13/site-packages GZ_IP=127.0.0.1 .venv/bin/python tools/run_gps_denied_mission.py`
- Error: `RuntimeError: PX4 did not arm in Offboard mode`; PX4 reports `Arming denied: Resolve system health failures first` and `Preflight check: FAILED`.
- Files involved: `tools/run_gps_denied_mission.py`, `src/vionav/mission.py`, `simulation/px4/gps_denied_mission.json`, and `simulation/px4/evidence/px4-sitl.log`.
- Observed state: the GCS link is recognized, GPS fusion is disabled, estimator heading is aligned, and PX4's health report retains arming errors for the system, local-position-estimate, and global-position-estimate components (`arming_check_error_flags=17956864`).
- Root cause: MAVLink integer parameters use byte-wise encoding in the float transport field. Sending numeric `15.0` with type `INT32` stored its IEEE-754 bits (`1097859072`) as `EKF2_EV_CTRL`, silently disabling vision fusion.
- Attempts:
  1. Changed the mission link from onboard-controller identity to a proper GCS/mission-planner identity, clearing the persistent `No connection to the GCS` condition.
  2. Enabled the external-vision yaw fusion bit (`EKF2_EV_CTRL=15`) in addition to position, height, and velocity fusion; PX4 still rejected arming.
- Escalation and resolution: a live read-only estimator snapshot exposed the corrupted parameter. Encoding and decoding the integer through its four wire bytes restored `EKF2_EV_CTRL=15`; the five-waypoint mission then completed and landed cleanly.

## Resolved: PX4 v1.17.0 native Gazebo build (superseded path)

- Date: 2026-10-25
- Task: `M6-T01`
- Exact command: `CONDA_PREFIX=/Users/sricharankumba/Documents/Codex/dependencies/PX4-main-setup/Tools/setup/macos/.pixi/envs/default CMAKE_PREFIX_PATH=$CONDA_PREFIX CCACHE_DISABLE=1 make PYTHON_EXECUTABLE=/usr/bin/python3 px4_sitl`
- Error: the locked current Gazebo Harmonic environment requires C++17 and its protobuf headers trigger PX4's warnings-as-errors. After the C++17 migration and generated-message suppression, `GZMixingInterfaceServo.cpp`, `GZGimbal.cpp`, `GZMixingInterfaceWheel.cpp`, and `GZMixingInterfaceESC.cpp` still fail on `-Wdouble-promotion` and deprecated `RepeatedField::Resize`.
- Files involved: external checkout `PX4-Autopilot-v1.17.0/CMakeLists.txt`, `src/modules/simulation/gz_msgs/CMakeLists.txt`, and `src/modules/simulation/gz_bridge/*`.
- Suspected cause: the v1.17.0 source predates compatibility changes required by the current official locked Gazebo/protobuf environment.
- Attempts:
  1. Applied PX4 upstream's C++14-to-C++17 build migration.
  2. Applied PX4 upstream's `-Wno-double-promotion` suppression for generated protobuf code.
- Resolution path: use the pinned PX4 upstream commit that owns the locked Gazebo environment, rather than adding further unbounded patches to the stable-release checkout.
- Resolution: pinned commit `22b15b91fba5e79f01248d1c644ef942382af517` built and launched successfully with Gazebo 8.10.0. This is no longer an active task blocker.
