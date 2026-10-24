# Verification Strategy

## Unit
- timestamp pairing
- frame/transform validation
- projection geometry
- feature-track lifecycle
- IMU preintegration math
- state transitions
- metric calculations

## Dataset
EuRoC MAV:
- ATE
- RPE
- tracking loss
- runtime
- time offset sensitivity

## Fault injection
- camera delay
- IMU delay
- frame drop
- IMU packet loss
- blurred images
- overexposure
- low texture
- estimator reset

## Simulation
PX4 SITL + Gazebo:
- hover
- waypoint mission
- GPS denied
- visual degradation
- delayed telemetry
- estimator restart

## Hardware
Not complete until actual hardware exists.
