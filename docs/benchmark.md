# Measured benchmark report

Measured on 2026-10-25 on an arm64 Mac running macOS 27.0.1, Python 3.13.12, OpenCV 5.0.0, NumPy 2.5.4 and Apple Clang 21.0.0. Results are single local runs and are not cross-device performance claims.

## Verification

| Check | Result |
|---|---:|
| Python tests | 57 passed |
| C++ tests | 9 passed |
| Python compilation | passed |
| Fresh C++ configure/build | passed |
| Retained fault cases | 8 passed |

## Synthetic frontend latency

Input: 100 deterministic synthetic 640×480 frames.

| Metric | Result |
|---|---:|
| Mean | 1.387 ms/frame |
| p50 | 1.370 ms/frame |
| p95 | 1.445 ms/frame |
| Maximum | 2.651 ms/frame |
| Mean features | 154.76/frame |

This measures preprocessing and feature detection, not full VIO update latency.

## Process resources

| Metric | Result |
|---|---:|
| Mean wall time | 2.186 ms/frame |
| CPU time for 100 frames | 0.396 s |
| RSS before | 53.22 MiB |
| RSS after / maximum | 78.34 MiB |

The macOS `ru_maxrss` byte convention is handled explicitly. These are workstation-process results, not ARM flight-computer measurements.

## Fault campaign

| Injection | Expected state | Observed score |
|---|---|---:|
| Nominal | TRACKING | 100 |
| 15 ms camera delay | DEGRADED | 80 |
| 40 ms camera delay | LOST | 40 |
| Moderate frame loss | DEGRADED | 75 |
| Severe frame loss | LOST | 0 |
| 50 ms IMU gap | DEGRADED | 80 |
| 150 ms IMU gap | LOST | 30 |
| Blur / low geometry | DEGRADED | 35 |

## PX4/Gazebo integration

PX4 commit `22b15b91fba5e79f01248d1c644ef942382af517`, Gazebo 8.10.0, headless x500:

| Scenario | Result |
|---|---:|
| GPS-denied waypoints | 5/5 completed |
| Maximum settled waypoint error | 0.883 m |
| Mission duration | 32.474 s |
| External-vision messages | 840 |
| Mission landing/disarm | passed |
| Estimator-loss action | Descend / blind land |
| Loss-to-failsafe transition | 0.658 s |
| Failsafe landing/disarm | passed |

The mission uses independent Gazebo ground truth as the simulated external-vision source. It validates the PX4 transport, frame conversion, fusion configuration, control and failsafe boundary; it is not evidence of camera-derived VIO accuracy.

## EuRoC ATE/RPE status

ATE and RPE tooling and EuRoC loaders are tested, but an end-to-end camera/IMU EuRoC estimator runner is not present. Therefore no EuRoC ATE or RPE value is claimed. Generating numbers from ground truth, injected perturbations or simulator truth would not be an estimator benchmark.

## Evidence

- Machine-readable summary: [`docs/generated/final-benchmark.json`](generated/final-benchmark.json)
- GPS-denied mission: [`simulation/px4/evidence/m6-t03-mission.json`](../simulation/px4/evidence/m6-t03-mission.json)
- Estimator-loss failsafe: [`simulation/px4/evidence/m6-t04-failsafe.json`](../simulation/px4/evidence/m6-t04-failsafe.json)
- Reproduction commands: [`docs/demo.md`](demo.md)
