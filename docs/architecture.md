# Detailed Architecture

The project is intentionally split into a middleware-independent estimation core and adapters.

```mermaid
flowchart LR
  CAM[Camera] --> SYNC[Time Synchroniser]
  IMU[IMU] --> BUF[IMU Buffer]
  BUF --> PRE[IMU Preintegration]
  CAM --> FE[Vision Frontend]
  CAL[Calibration] --> FE
  CAL --> PRE
  FE --> EST[VIO Estimator]
  PRE --> EST
  EST --> SLAM[SLAM / Loop Closure]
  EST --> NAV[PX4 Navigation Bridge]
  EST --> LOG[Telemetry / Evaluation]
  SLAM --> NAV
```

## Design goals
- deterministic replay from recorded data
- measurable estimator quality
- explicit degraded/lost states
- portable core C++ library
- testable without flight hardware
