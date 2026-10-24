# Architecture

## Core boundary
The estimator core is C++ and must not depend directly on PX4, ROS 2 or a specific camera driver.

## Main modules
- sensor contracts
- timestamp synchroniser
- calibration
- vision frontend
- IMU preintegration
- estimator backend
- SLAM/loop-closure adapter
- navigation bridge
- telemetry/evaluation

## Integration principle
Middleware adapters translate external messages into typed core-domain inputs. The estimation library remains reusable and testable with recorded datasets.

## Frame convention
Use explicit named frames and document every transform. Never rely on implicit axis assumptions.

Suggested frames:
- `world`
- `body`
- `imu`
- `camera`
- `camera_optical`
