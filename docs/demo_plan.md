# Demo Plan

## Pre-Codex offline demo
1. show camera/IMU calibration + time-offset recovery;
2. run geometric feature tracking/RANSAC;
3. show quality transition under camera delay / IMU gap;
4. demonstrate loop-candidate retrieval and translation pose-graph correction;
5. demonstrate offline geo-localisation fixture;
6. show PX4 external-vision ENU/NED contract;
7. show resource/video benchmark report.

## Final Codex/PX4 demo
1. start PX4 SITL + Gazebo;
2. start VIO estimator;
3. show external-vision feed accepted by PX4;
4. disable GPS;
5. fly retained waypoint square;
6. inject timing/frame degradation;
7. show TRACKING -> DEGRADED;
8. inject estimator loss;
9. show external-vision suppression + failsafe;
10. replay logs and present EuRoC/SITL benchmark table.

No physical-flight footage may be implied unless hardware flight testing actually occurs.
