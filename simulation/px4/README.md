# PX4 SITL integration fixture

This directory contains the reproducible PX4/Gazebo integration and its retained acceptance evidence.

Verified:
- headless Gazebo 8.10.0 x500 launch against pinned PX4 commit `22b15b91fba5e79f01248d1c644ef942382af517`
- MAVLink 2 external-vision `ODOMETRY` received by PX4
- EKF2 external position, height and velocity fusion active in SITL

Prepared but not yet accepted:
- GPS-denied waypoint mission
- estimator-loss failsafe policy

Launch SITL with:

```bash
PX4_DIR=/path/to/PX4-Autopilot simulation/px4/run_sitl.sh
```

In a second terminal, stream the deterministic acceptance pose with:

```bash
.venv/bin/python tools/send_external_vision.py --duration 20 --rate-hz 30
```

No physical-flight, hardware or safety-validation claim is made.
