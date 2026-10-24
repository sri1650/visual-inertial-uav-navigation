# Recruiter / Interview Evidence Map

This file distinguishes implemented evidence from planned capability.

## Implemented and tested now

### Camera/IMU timing and calibration
Evidence:
- deterministic nearest timestamp association in Python and C++
- synthetic time-offset recovery
- calibration parsing/validation
- orthonormal/right-handed transform checks
- calibration-quality report

Interview angle: explain why a 10–20 ms camera/IMU timing error can masquerade as geometric or estimator error.

### IMU pipeline foundation
Evidence:
- strictly ordered IMU buffer
- interpolation
- gyro/accelerometer bias state
- midpoint-style preintegration foundation
- stationary gravity initialisation

Interview angle: explain why biases are part of the estimator state and why raw IMU cannot simply be integrated indefinitely.

### Geometric computer vision frontend
Evidence:
- deterministic grayscale/contrast preprocessing
- feature detection
- pyramidal Lucas–Kanade tracking
- forward/backward consistency gating
- essential-matrix RANSAC outlier rejection
- keyframe selection from feature count, parallax and maximum interval

Interview angle: explain feature loss, RANSAC, epipolar geometry and why low parallax is a poor keyframe trigger.

### Estimator health state
Evidence:
- INITIALISING / TRACKING / DEGRADED / LOST / RESETTING state machine
- thresholds for sensor timeout, feature collapse and uncertainty growth

Interview angle: the estimator must not silently report healthy tracking after observability collapses.

### Evaluation
Evidence:
- EuRoC camera/IMU/ground-truth loaders
- SE(3) trajectory alignment
- ATE/RPE computation
- timestamp-offset sensitivity tooling
- drop/shift fault injection
- machine-readable experiment matrix

## Planned / not yet completed
- nonlinear/factor-graph or filtering VIO backend
- measured EuRoC estimator accuracy
- loop closure and pose-graph correction
- PX4 SITL execution
- live embedded camera pipeline
- geo-localisation
- physical camera/IMU test-rig validation
- physical UAV flight

Do not describe planned items as completed in a CV or interview.

## CV bullets available now
- Developed a C++/Python visual-inertial navigation foundation with typed camera/IMU interfaces, calibration validation, timestamp synchronisation and deterministic sensor-fusion test tooling.
- Implemented a geometric computer-vision frontend using feature detection, pyramidal optical flow, forward/backward track validation, essential-matrix RANSAC and keyframe-selection policies.
- Built IMU buffering, interpolation, bias-state and preintegration foundations in modern C++, with deterministic unit tests for synthetic motion and gravity initialisation.
- Created EuRoC-compatible evaluation tooling covering trajectory alignment, ATE/RPE metrics, timestamp-offset experiments and reproducible sensor-fault injection.
