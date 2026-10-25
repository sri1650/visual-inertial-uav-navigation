# Recruiter evidence map

## Strongest demonstrated evidence

### Estimation and C++

- Fifteen-state multiplicative error-state EKF: pose, velocity, gyro bias and accelerometer bias.
- IMU propagation, calibrated visual body-pose updates, covariance propagation and innovation diagnostics.
- Deterministic accelerated-trajectory convergence tests.
- Real estimator health and bounded reset cycle: TRACKING → DEGRADED → LOST → RESETTING → INITIALISING → TRACKING.

### Vision and timing

- Camera/IMU calibration and rigid-transform validation.
- Timestamp association and deterministic offset recovery.
- Feature detection, pyramidal optical flow, forward/backward validation and essential-matrix RANSAC.
- Retained camera-delay, frame-loss, IMU-gap and weak-geometry fault campaign.

### UAV integration

- Headless PX4/Gazebo x500 at pinned PX4 commit `22b15b91fba5e79f01248d1c644ef942382af517`.
- MAVLink 2 ODOMETRY with coordinate, quaternion, covariance, timestamp, quality and reset handling.
- Five-waypoint GPS-denied simulation completed; maximum settled error 0.883 m; landed and disarmed.
- Injected external-estimator loss activated Descend/blind-land failsafe in 0.658 s; landed and disarmed.

### Reproducibility

- 57 Python and 9 C++ tests.
- Clean-machine setup and one-command offline demo.
- Machine-readable benchmark and SITL evidence.
- Explicit separation between measured evidence and unvalidated claims.

## Truthful CV bullets

- Built a middleware-independent modern C++ visual-inertial estimation core with a 15-state error-state EKF, IMU propagation, bias estimation, covariance/innovation diagnostics, and bounded reset recovery.
- Implemented calibrated geometric-vision and timing tooling using pyramidal optical flow, forward/backward validation, essential-matrix RANSAC, camera/IMU association, and deterministic sensor-fault campaigns.
- Integrated a quality-gated MAVLink external-vision adapter with PX4/Gazebo and verified a five-waypoint GPS-denied SITL mission plus a 0.658 s estimator-loss-to-Descend failsafe transition.
- Created reproducible Python/C++ verification and benchmark workflows covering 66 automated tests, frontend latency, process resources, trajectory metrics and retained SITL evidence.

## Claims not supported

Do not claim EuRoC estimator accuracy, physical flight, flight-worthiness, real-time ARM performance, production deployment or safety certification. The PX4 waypoint errors describe simulator control around a ground-truth-derived external-vision feed, not visual-inertial odometry accuracy.
