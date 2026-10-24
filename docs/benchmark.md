# Benchmark Plan

## Dataset
Primary: EuRoC MAV.

## Metrics
- ATE RMSE
- RPE translation/rotation
- estimator update latency
- tracking-loss count
- successful relocalisation count
- camera/IMU residual time offset

## Acceptance targets
Initial targets are engineering goals, not claimed results:
- deterministic benchmark execution
- no silent estimator divergence
- explicit LOST state on unrecoverable tracking failure
- measurable degradation when time offset is injected
