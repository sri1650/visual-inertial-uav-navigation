# Current State

## Verified pre-Codex baseline

- Formal task graph: **32/39 complete**
- Python tests: **50 passing**
- C++ tests: **9/9 passing**
- Pre-Codex gate: **PASS**
- Physical UAV validation: **none**
- PX4 SITL validation: **not yet completed**

## Completed before Codex

### Timing / calibration
- camera/IMU typed contracts
- rigid transform validation
- camera intrinsics validation
- nearest timestamp association
- deterministic camera/IMU time-offset estimation
- calibration quality report

### Geometric vision
- preprocessing / CLAHE
- feature detection
- pyramidal LK tracking
- forward/backward validation
- essential-matrix RANSAC
- deterministic keyframe policy

### IMU
- C++ IMU ring buffer/interpolation
- bias model
- preintegration foundation
- stationary gravity initialisation

### Estimator health boundary
- explicit INITIALISING / TRACKING / DEGRADED / LOST / RESETTING state model
- C++ and Python quality scoring
- bounded reset/recovery supervisor
- timing/frame/IMU-gap fault campaign

The M4-T02 nonlinear error-state EKF now uses these foundations; quality and recovery integration remain M4-T03/M4-T04.

### SLAM boundary
- Python + C++ keyframe database
- deterministic loop-closure candidate retrieval
- translation pose-graph correction prototype
- loop fixture proves drift correction

The final 6-DoF SLAM/loop backend may be upgraded during integration, but the architecture/interface task is complete.

### PX4 boundary preparation
- ENU <-> NED conversion
- typed external-vision sample contract
- GPS-denied waypoint fixture
- estimator-loss failsafe policy fixture
- SITL launch wrapper/spec

Actual MAVLink/MAVSDK transport and PX4 SITL acceptance remain incomplete.

### Embedded / runtime
- ARM64 cross-build Docker/toolchain profile
- embedded Linux process split and deployment documentation
- real-time synthetic video preprocessing/feature benchmark
- watchdog/failsafe supervisor

### Geo-localisation
- typed map reference/result interface
- ORB coarse retrieval + homography geometric verification
- confidence gating
- offline fixture localisation and low-texture rejection

### Benchmarking / faults
- EuRoC camera/IMU/ground-truth loaders
- rigid SE(3) trajectory alignment
- ATE/RPE benchmark tooling
- timestamp offset sensitivity tools
- dropped-frame and IMU-gap injection helpers
- retained tracking-quality fault campaign
- video latency + process resource benchmark

## Measured local pre-backend evidence

From 100 synthetic 640x480 frames on this runtime:
- mean preprocessing/feature latency: ~4.25 ms/frame
- p50: ~3.91 ms
- p95: ~5.74 ms
- max: ~22.03 ms
- mean detected features: ~158

Process profile over 100 frames:
- mean wall time: ~3.69 ms/frame
- max RSS observed: ~153 MB

These numbers are **frontend/local-runtime evidence only**, not full VIO, ARM hardware, PX4 or production performance.

Retained quality-state campaign:
- nominal -> TRACKING
- 15 ms camera/IMU residual -> DEGRADED
- 40 ms residual -> LOST
- moderate frame loss -> DEGRADED
- severe frame loss -> LOST
- 50 ms IMU gap -> DEGRADED
- 150 ms IMU gap -> LOST

These transitions validate the health policy around the future estimator; they are not trajectory-accuracy results.

## Deliberately reserved for Codex / Work

Remaining formal tasks:
M6-T01, M6-T02, M6-T03, M6-T04, M10-T01, M10-T02, M10-T03

### Hard technical core
1. **M6-T01** — install/launch PX4 SITL + Gazebo and verify environment
2. **M6-T02** — real MAVLink/MAVSDK external vision transport accepted by PX4
3. **M6-T03** — GPS-denied waypoint mission completes in SITL
4. **M6-T04** — estimator-loss failsafe proven in SITL

### Final evidence only after integration
8. M10-T01 — final clean-machine demo script
9. M10-T02 — final integrated benchmark report/architecture diagrams
10. M10-T03 — final recruiter README/interview pack with integrated evidence

## First Work command

```bash
python tools/pre_codex_gate.py
```

If it passes, do not refactor the completed pre-Codex modules unless a deterministic integration failure requires it.

## Codex milestones

- M4-T02 complete: a middleware-independent 15-state multiplicative error-state EKF fuses calibrated IMU propagation with visual body-pose updates, estimates velocity/gyro bias/accelerometer bias, propagates covariance, rejects invalid timing/numerics, and converges on a deterministic accelerated trajectory. Eigen is the only new C++ dependency; 50 Python and 7 C++ tests pass.
- M4-T03 complete: real EKF covariance trace, 6-DoF normalized innovation, feature/inlier statistics, calibrated camera timing residual and measured IMU gap now drive TRACKING/DEGRADED/LOST decisions. Deterministic nominal, weak-geometry, innovation-outlier, stale-camera and IMU-gap cases pass in 8 C++ tests.
- M4-T04 complete: the existing bounded recovery supervisor now owns real EKF reset timing. The verified cycle is TRACKING -> DEGRADED -> LOST -> RESETTING -> INITIALISING -> TRACKING; reset clears pose, velocity, biases and covariance before a new origin and bias initialisation, while preserving the reset counter. Nine C++ tests pass.
