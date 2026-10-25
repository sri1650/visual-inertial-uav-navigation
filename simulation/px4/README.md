# PX4 SITL integration fixture

This directory contains the reproducible PX4/Gazebo integration and its retained acceptance evidence.

Verified:
- headless Gazebo 8.10.0 x500 launch against pinned PX4 commit `22b15b91fba5e79f01248d1c644ef942382af517`
- MAVLink 2 external-vision `ODOMETRY` received by PX4
- EKF2 external position, height and velocity fusion active in SITL
- five-waypoint GPS-denied Offboard mission, followed by landing and disarm
- estimator-stream loss triggering PX4 Descend/blind-land failsafe and disarm

Retained structured evidence is under `simulation/px4/evidence/`.

Launch SITL with:

```bash
PX4_DIR=/path/to/PX4-Autopilot simulation/px4/run_sitl.sh
```

In a second terminal, stream the deterministic acceptance pose with:

```bash
.venv/bin/python tools/send_external_vision.py --duration 20 --rate-hz 30
```

Run the GPS-denied mission or estimator-loss acceptance with:

```bash
.venv/bin/python tools/run_gps_denied_mission.py
.venv/bin/python tools/run_estimator_loss_failsafe.py
```

No physical-flight, hardware or safety-validation claim is made.
