# Blocked work

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
