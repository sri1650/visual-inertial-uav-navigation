# Roadmap

## M0 — Foundation
- repository structure
- coordinate/frame conventions
- dataset metrics
- timestamp pairing
- CI

## M1 — Sensor contracts & calibration
- camera model
- IMU model
- extrinsic transforms
- calibration-file validation
- time-offset estimation tooling

## M2 — Geometric vision frontend
- image preprocessing
- feature detection/tracking
- outlier rejection
- essential-matrix / epipolar validation
- keyframe policy

## M3 — IMU pipeline
- IMU buffering
- bias model
- preintegration
- gravity initialisation
- covariance propagation

## M4 — VIO estimator
- estimator state
- visual + inertial residuals
- nonlinear optimisation / filter backend
- marginalisation strategy
- estimator reset/recovery

## M5 — Visual SLAM
- keyframe database
- place recognition boundary
- loop closure
- pose-graph correction
- map persistence

## M6 — PX4 simulation integration
- PX4 SITL
- Gazebo sensor bridge
- MAVLink/MAVSDK interface
- pose/velocity injection
- mission and failsafe scenarios

## M7 — Embedded Linux
- ARM/Jetson/Raspberry Pi deployment
- cross-build/container
- performance profiling
- watchdog / process supervision
- real-time video pipeline

## M8 — Geo-localisation
- map/image retrieval interface
- coarse-to-fine localisation
- confidence gating
- degraded-mode behaviour

## M9 — Verification
- EuRoC benchmark
- timestamp-offset fault injection
- dropped-frame / IMU-gap tests
- visual degradation tests
- latency and resource report
- deterministic replay

## M10 — Portfolio finish
- demo video
- architecture diagram
- benchmark report
- recruiter README
- interview pack
