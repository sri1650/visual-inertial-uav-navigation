# Interview pack

## 90-second explanation

I built a GPS-denied UAV navigation stack around three boundaries. First, a middleware-independent modern C++ core handles calibrated IMU propagation, visual pose updates, covariance and health-aware recovery. Second, Python supplies geometric-vision, dataset, benchmark and simulation orchestration. Third, a quality-gated MAVLink adapter converts ENU/FLU estimates to PX4's NED/FRD convention. I verified the core with deterministic tests and fault campaigns, then used headless PX4/Gazebo to complete a five-waypoint mission with GNSS fusion disabled. Stopping external vision caused PX4 to enter Descend/blind-land failsafe in 0.658 seconds and disarm after landing. The important limitation is that the SITL vision source is independent Gazebo truth, so I do not claim EuRoC or physical-flight VIO accuracy.

## Design decisions to explain

1. **Middleware-independent core:** numerical code is testable without ROS, PX4 or a simulator.
2. **Explicit frames:** ENU/FLU and NED/FRD conversions are typed and unit-tested because a silent frame error can destabilise flight control.
3. **Error-state EKF:** the nominal pose evolves on the manifold while a small 15-state error and covariance are propagated and corrected.
4. **Quality before transport:** stale, low-quality or future-dated estimates are suppressed rather than forwarded as healthy navigation data.
5. **Bounded recovery:** LOST triggers a real state/covariance reset and reinitialisation, with a retained reset counter.
6. **Independent acceptance truth:** Gazebo truth drives the simulated external-vision boundary; PX4's own estimate is never fed back as its measurement.

## Likely technical questions

### Estimation

- Why estimate gyro and accelerometer biases?
- What is the difference between nominal and error state?
- How are covariance and innovation used to reject unsafe updates?
- What becomes unobservable during low parallax or estimator-aiding loss?
- Why is a ground-truth-fed SITL mission not a VIO accuracy benchmark?

### Vision and timing

- Why use forward/backward optical-flow validation and RANSAC together?
- How can a 10–20 ms camera/IMU offset resemble calibration or motion error?
- What causes essential-matrix estimation to fail?
- How should blur, feature collapse and IMU gaps affect estimator health?

### PX4 and safety boundary

- How are ENU/FLU position and attitude converted to NED/FRD?
- Which covariance and reset metadata are sent in MAVLink ODOMETRY?
- Why disable GNSS fusion rather than merely hide a UI indicator?
- What evidence proves the failsafe was estimator-loss driven?
- What additional steps are required before physical flight?

## Measured numbers to remember

- 57 Python tests, 9 C++ tests.
- 5/5 GPS-denied waypoints; 0.883 m maximum settled error; 32.474 s mission.
- 0.658 s estimator-loss-to-Descend transition; blind land and disarm.
- Synthetic frontend: 1.387 ms mean, 1.445 ms p95 over 100 640×480 frames.
- Process maximum RSS: 78.34 MiB on the measured arm64 Mac run.

## Honest next steps

- Wire the camera frontend and C++ estimator into a timestamped EuRoC runner and report ATE/RPE.
- Replace the translation-only pose-graph prototype with a production 6-DoF backend.
- Profile and tune on an actual ARM flight computer.
- Validate camera/IMU calibration and timing on a physical rig before any restrained hardware flight-test programme.
