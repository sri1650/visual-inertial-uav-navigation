# Architecture

The estimator core is independent of PX4 and transport middleware. Python owns dataset, vision, benchmark and simulator orchestration; modern C++ owns the timing, IMU, estimator, health and recovery core.

## Data and control flow

```mermaid
flowchart LR
  CAM[Camera frames] --> SYNC[Time association]
  IMU[IMU samples] --> BUF[C++ ordered buffer]
  CAL[Calibration] --> SYNC
  CAL --> EKF
  SYNC --> FE[Vision frontend]
  BUF --> PRE[IMU propagation]
  FE --> EKF[15-state error-state EKF]
  PRE --> EKF
  EKF --> HEALTH[Quality and recovery]
  HEALTH --> GATE{Healthy output?}
  GATE -->|yes| BRIDGE[ENU/FLU to NED/FRD MAVLink]
  GATE -->|no| SUPPRESS[Suppress external vision]
  BRIDGE --> PX4[PX4 EKF2 and control]
  SUPPRESS --> PX4
  EKF --> KF[Keyframe and loop boundary]
  EKF --> EVAL[Replay, faults and metrics]
```

## GPS-denied SITL sequence

```mermaid
sequenceDiagram
  participant GZ as Gazebo x500
  participant V as External-vision adapter
  participant E as PX4 EKF2
  participant C as PX4 Commander
  GZ->>V: independent ENU/FLU pose
  V->>E: MAVLink ODOMETRY in NED/FRD
  E->>C: valid local position
  C->>GZ: Offboard waypoint setpoints
  Note over C,GZ: five waypoints, GNSS fusion disabled
  V--xE: injected estimator-stream loss
  E->>C: local position invalid
  C->>GZ: Descend / blind land
  GZ->>C: landing detected, disarm
```

## Validation boundaries

- Deterministic tests cover contracts, estimation, health, reset and recovery.
- PX4/Gazebo covers simulated transport, fusion, waypoint control and failsafe response.
- EuRoC loaders and metrics exist, but full-dataset VIO accuracy has not been measured.
- No physical camera/IMU rig, flight hardware, production deployment or safety certification is claimed.
