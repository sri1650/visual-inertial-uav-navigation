# VIO Fault Laboratory

The benchmark should deliberately degrade sensor input rather than reporting only nominal accuracy.

## Experiments

### Camera/IMU timing offset
Inject 0, 5, 10, 20 and 40 ms camera delay. Measure ATE/RPE, feature count, covariance/quality and state transitions.

### Camera frame loss
Drop 0%, 5%, 10%, 20% and 40% of frames using deterministic seeds.

### IMU gaps
Inject 10, 50 and 100 ms gaps while preserving timestamp monotonicity.

### Visual degradation
Use reproducible transforms for motion blur, low light, low texture and overexposure.

## Expected estimator behaviour
The system must never silently continue reporting high-quality TRACKING after evidence quality collapses. It should transition through DEGRADED and eventually LOST, then require an explicit recovery/reset path.

## Evidence
Every run should emit machine-readable JSON containing:
- fault configuration
- ATE/RPE
- feature count
- estimator state timeline
- tracking-loss count
- recovery time
- mean/p95 latency
