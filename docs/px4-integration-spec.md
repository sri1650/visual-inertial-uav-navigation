# PX4 External-Vision Integration Specification

## Boundary
The visual-inertial estimator remains middleware-independent. A dedicated adapter converts `EstimatorOutput` into PX4-compatible external vision / odometry messages.

## Data flow

```text
Camera + IMU
    |
    v
C++ VIO core
    |
    | pose, velocity, covariance, quality, timestamp
    v
PX4 adapter
    |
    | MAVLink / MAVSDK external odometry
    v
PX4 EKF2
    |
    v
Position / velocity controller
```

## Required adapter responsibilities
- frame conversion into the configured PX4 local frame
- quaternion convention conversion
- timestamp mapping
- covariance propagation or conservative defaults
- update-rate monitoring
- stale-data rejection
- tracking-quality gating
- reset-counter propagation when estimator origin/state resets

## Health policy

### TRACKING
Publish pose/velocity normally.

### DEGRADED
Continue only if configured quality thresholds are satisfied; mark conservative covariance and emit health warning.

### LOST
Stop publishing trusted external vision measurements and trigger the configured PX4 failsafe path. Do not fabricate pose continuity.

### RESETTING
Withhold normal navigation validity until a new estimator origin is established and reset signalling has been propagated.

## Initial SITL scenarios
1. nominal hover with external vision
2. waypoint mission with GPS disabled
3. injected 20 ms camera/IMU timing error
4. camera blackout
5. IMU data gap
6. estimator reset during hover
7. delayed external-vision messages

## Verification evidence
- PX4 estimator health/logs
- VIO tracking-state timeline
- commanded versus estimated trajectory
- failsafe transition timing
- deterministic scenario configuration

This document is an integration specification, not evidence of successful PX4 integration or flight validation.
