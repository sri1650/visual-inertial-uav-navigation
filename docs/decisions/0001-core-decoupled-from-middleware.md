# ADR 0001 — Estimator core is middleware-independent

## Decision
The C++ estimation library will not depend directly on ROS 2, PX4 or camera-driver message types.

## Why
This keeps algorithms testable against public datasets, allows deterministic replay, reduces integration coupling, and makes embedded deployment easier.
