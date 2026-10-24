# PX4 SITL integration fixture

This directory intentionally prepares the **contract and acceptance assets** without claiming PX4 has been run in this environment.

Prepared:
- GPS-denied waypoint mission
- estimator-loss failsafe policy
- ENU -> NED external-vision conversion code
- watchdog thresholds
- SITL launch wrapper

Codex/Work must:
1. install/use a compatible PX4-Autopilot checkout;
2. launch SITL + Gazebo;
3. implement MAVLink/MAVSDK external vision transport;
4. verify the external vision feed is accepted;
5. run the retained GPS-denied mission;
6. induce estimator loss and prove the failsafe transition;
7. capture logs and measured trajectory evidence.

No simulation-complete claim is permitted before those checks pass.
