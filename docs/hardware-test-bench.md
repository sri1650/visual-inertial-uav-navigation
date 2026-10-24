# Physical UAV / Sensor Test Bench Plan

Do not buy this until simulation and dataset milestones are stable.

## Phase 1 — desk sensor rig
Goal: validate camera/IMU timing, calibration and live pipeline without flight.

Suggested categories:
- global-shutter USB/MIPI camera
- IMU with stable timestamp access
- Raspberry Pi 5 / Jetson-class Linux computer
- rigid camera/IMU mount
- printed AprilTag/Charuco calibration targets

Evidence:
- camera intrinsics
- camera↔IMU extrinsics
- timestamp residual
- live estimator rate/latency
- controlled hand-motion trajectory tests

## Phase 2 — autopilot integration bench
- PX4-compatible flight controller
- companion computer
- camera + IMU
- MAVLink connection
- bench power supply / protected battery setup

Evidence:
- external vision injection
- estimator health
- mode transitions
- watchdog/failsafe tests with motors disabled

## Phase 3 — supervised flight
Only after appropriate safety/risk checks and legal compliance.

Evidence:
- short controlled GPS-enabled baseline
- GPS-denied test in a safe environment
- recovery/failsafe logs
- timing/latency under real vibration and lighting

No project documentation may claim physical flight validation before these tests are actually performed.
