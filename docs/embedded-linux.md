# Embedded Linux / ARM64 deployment profile

The project now includes a reproducible ARM64 cross-build boundary under `deploy/arm64/`.

## Intended targets
- Jetson Orin-class companion computers
- Raspberry Pi 5 / ARM64 development rigs
- generic aarch64 Linux targets

## Build

```bash
docker build -f deploy/arm64/Dockerfile -t vionav-arm64 .
```

The image installs the aarch64 GNU cross toolchain and builds the middleware-independent C++ core in Release mode.

## Runtime process split

1. camera capture / preprocessing
2. VIO estimator
3. PX4/MAVLink bridge
4. watchdog / health supervisor
5. telemetry/logger

The estimator core remains independent from the transport middleware.

## Performance budget

Initial engineering target only—not a measured hardware claim:

- capture/decode: <= 5 ms
- feature tracking: <= 10 ms
- IMU/preintegration: <= 2 ms
- estimator update: <= 8 ms
- total camera-to-estimate: <= 30 ms at target frame rate

Real ARM hardware profiling remains required.
