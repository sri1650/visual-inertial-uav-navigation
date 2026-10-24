# Work Handoff — Visual-Inertial UAV Navigation

Continue the attached `visual-inertial-uav-navigation` project.

The project has been aggressively pre-built outside Work to minimise Codex/GPT usage. Treat `tasks/TASKS.json`, `docs/context/CURRENT_STATE.md`, `AGENTS.md`, and this handoff as source of truth.

## Mandatory first step

Run:

```bash
python tools/pre_codex_gate.py
```

Expected handoff baseline:
- 50 Python tests passing
- 6/6 C++ tests passing
- formal task graph 29/39 complete
- no physical-flight claims
- no PX4 SITL completion claim

Do not redesign completed modules or spend tokens recreating timing, calibration, vision frontend, IMU preintegration foundation, quality/recovery policy, keyframe DB, pose-graph boundary, geo-localisation, ARM profile, video benchmark, watchdog, fault campaign or benchmark tooling.

## Remaining mission

### 1. M4-T02 — real VIO estimator backend

This is the main hard task.

Use the existing:
- C++ camera/IMU contracts
- calibration types
- IMU buffer/preintegration
- vision frontend contracts
- estimator state
- quality/recovery types
- frame conventions

Implement a defensible visual-inertial estimator backend in modern C++.

Requirements:
- explicit estimator state: pose, velocity, gyro bias, accel bias
- visual + inertial measurement fusion
- initialisation strategy
- covariance/uncertainty or equivalent backend quality output
- deterministic dataset interface
- no hidden dependency on ROS/PX4 message types
- tests on synthetic trajectories before EuRoC

Choose EKF/MSCKF/factor-graph style deliberately. If adding GTSAM/Ceres/Eigen, document the engineering reason and keep dependencies minimal.

Do not claim convergence until deterministic tests and dataset evidence support it.

### 2. M4-T03 — real quality integration

Wire the real estimator uncertainty/residuals into the already implemented quality layer.

Replace proxy-only quality inputs with measured:
- tracked features/inlier ratio
- estimator covariance or equivalent uncertainty
- camera/IMU timing residual
- IMU gap
- innovation/residual information if available

Prove TRACKING -> DEGRADED -> LOST under retained fault cases.

### 3. M4-T04 — real recovery integration

Use the prepared recovery supervisor, but integrate it with the actual estimator:
- bounded LOST detection
- reset/reinitialisation
- reset counter
- no stale-state reuse
- successful recovery test after temporary degradation

### 4. M6-T01 — PX4 SITL + Gazebo

Use the prepared `simulation/px4/` assets.

Install/use a compatible PX4-Autopilot checkout and prove:
- SITL launches
- Gazebo vehicle launches
- logs are retained
- external vision path can be exercised

### 5. M6-T02 — MAVLink/MAVSDK bridge

Use the existing ENU/NED conversion and external-vision contract.

Implement real transport:
VIO output -> MAVLink/MAVSDK -> PX4 external vision / odometry input.

Verify PX4 accepts the feed. Record exact topic/message/rate and coordinate-frame assumptions.

### 6. M6-T03 — GPS-denied mission

Run the retained mission in `simulation/px4/gps_denied_mission.json`.

Acceptance:
- GPS disabled/unavailable
- external visual-inertial state drives navigation
- waypoint mission completes
- measured position/trajectory error is reported
- no unsupported claim if the estimator is not robust enough

### 7. M6-T04 — estimator-loss failsafe

Use the retained failsafe policy.

Inject estimator loss/timing failure and prove:
- external vision is suppressed/invalidated
- watchdog detects failure
- PX4 transitions to the configured safe behaviour
- logs show the causal sequence

### 8. Final benchmark evidence

After the backend exists:
- run real EuRoC sequences through the estimator
- calculate ATE/RPE using existing tooling
- rerun timing/frame/IMU-gap fault campaigns against actual estimator outputs
- rerun video/resource benchmark on integrated pipeline
- distinguish desktop runtime, SITL, and any future ARM hardware measurements

### 9. Final portfolio finish

Only after the integrated evidence exists:
- clean-machine demo commands
- final README
- architecture diagrams
- benchmark table
- interview pack
- explicit limitations

Never claim physical flight, safety certification or ARM hardware validation unless actually performed.

## Low-credit policy

- One bounded task at a time.
- Use existing interfaces; do not rewrite them for taste.
- Compiler/tests/benchmark data before model judgement.
- Maximum two repair attempts before a blocker report.
- Strongest model only for estimator mathematics, numerical instability, hard C++ integration, PX4 frame/timing issues, or final architecture review.
- Commit after verified milestones.

## Definition of done

The project is done only when:
- nonlinear VIO backend is tested;
- real estimator quality/recovery is integrated;
- EuRoC results are reproducible;
- PX4 SITL accepts external vision;
- GPS-denied mission completes or limitations are honestly documented;
- estimator-loss failsafe is verified;
- final benchmarks use measured data;
- final README distinguishes dataset, simulation and hardware validation.

At completion report:
- repository URL
- final commit SHA
- Python/C++ test summary
- EuRoC ATE/RPE summary
- PX4 mission/failsafe result
- demo commands
- all remaining unvalidated hardware assumptions
