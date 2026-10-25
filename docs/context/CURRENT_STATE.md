# Current State

## Verified pre-Codex baseline

- Formal task graph: **38/39 complete**
- Python tests: **57 passing**
- C++ tests: **9/9 passing**
- Pre-Codex gate: **PASS**
- Physical UAV validation: **none**
- PX4 SITL validation: **Gazebo 8.10.0 x500 launch verified on macOS**

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

The M4-T02 nonlinear error-state EKF uses these foundations, and M4-T03/M4-T04 now integrate live quality diagnostics and bounded recovery.

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

PX4 SITL acceptance, MAVLink external-vision transport, the GPS-denied waypoint mission and the estimator-loss failsafe are complete.

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
M10-T03

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
- M6-T01 complete: pinned PX4 commit `22b15b91fba5e79f01248d1c644ef942382af517` builds against its locked Gazebo Harmonic environment. The project wrapper launches Gazebo 8.10.0 headlessly, spawns x500, connects IMU/GPS/barometer/magnetometer streams, opens MAVLink ports and reaches a successful PX4 startup. Evidence is retained under `simulation/px4/evidence/`; 50 Python and 9 C++ tests pass.
- M6-T02 complete: the MAVLink 2 adapter publishes quality-gated, timestamp-checked `ODOMETRY` in local NED with ENU/FLU pose conversion, covariance, reset counter and VIO estimator type. Live PX4 received the exact sample on `vehicle_visual_odometry`; with `EKF2_EV_CTRL=7`, EKF2 reported `cs_ev_pos`, `cs_ev_hgt` and `cs_ev_vel` true with zero fault-status changes. Evidence is in `simulation/px4/evidence/m6-t02-acceptance.md`; 53 Python and 9 C++ tests pass.
- M6-T03 complete: PX4 flew all five retained NED waypoints with GNSS fusion disabled and live Gazebo ground truth supplied only through the external-vision MAVLink boundary. Every waypoint settled within the 1 m acceptance radius, 840 vision messages were sent, and PX4 landed and disarmed in 32.474 s. Evidence is in `simulation/px4/evidence/m6-t03-mission.json`; 56 Python and 9 C++ tests pass.
- M6-T04 complete: after 399 live vision updates and a stable Offboard hover, the verifier stopped only the external-vision stream while retaining GCS heartbeats and setpoints. PX4 invalidated local position, activated its Descend/blind-land failsafe in 0.658 s, detected landing and disarmed. Evidence is in `simulation/px4/evidence/m6-t04-failsafe.json`; 57 Python and 9 C++ tests pass.
- M10-T01 complete: `tools/run_demo.sh` provides deterministic offline, evidence-only, GPS-denied mission and estimator-loss modes. `docs/demo.md` contains clean macOS and pinned PX4/Gazebo setup commands. The offline demo passed all 57 Python tests, all 9 C++ tests, compilation, frontend/resource benchmarks, the fault campaign and retained SITL evidence validation.
- M10-T02 complete: `docs/benchmark.md` publishes measured local frontend/resource, fault-campaign and PX4/Gazebo results; `docs/architecture.md` documents the implemented data path and estimator-loss sequence. The README links both. EuRoC ATE/RPE remains explicitly unmeasured because no end-to-end EuRoC VIO runner exists.
