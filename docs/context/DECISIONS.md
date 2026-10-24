# Decisions

1. C++ owns runtime estimation and latency-sensitive perception.
2. Python owns dataset tooling, metrics, reports and experimentation.
3. ROS 2/PX4 are integration adapters, not estimator dependencies.
4. Public-dataset validation precedes simulator integration.
5. Every estimator result must carry a quality/tracking state.
6. Time synchronisation is treated as a first-class engineering problem.
7. No flight-readiness claims without physical validation.
