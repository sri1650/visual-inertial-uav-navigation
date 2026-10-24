# Interfaces

## CameraSample
- timestamp_ns
- frame_id
- image metadata
- optional image handle

## ImuSample
- timestamp_ns
- angular_velocity_rad_s[3]
- linear_acceleration_m_s2[3]

## Calibration
- camera intrinsics
- distortion model
- T_body_camera
- T_body_imu
- estimated camera/IMU time offset

## EstimatorOutput
- timestamp_ns
- position
- orientation
- velocity
- accel bias
- gyro bias
- covariance / quality
- tracking state

## Tracking states
- INITIALISING
- TRACKING
- DEGRADED
- LOST
- RESETTING
