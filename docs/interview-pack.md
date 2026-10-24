# Interview Pack — Visual-Inertial UAV Navigation

## 90-second project explanation
I built this project to demonstrate the software and estimation problems behind GPS-denied UAV autonomy. The architecture deliberately separates a middleware-independent C++ estimation core from PX4/ROS adapters. Camera and IMU timing, calibration and coordinate frames are first-class interfaces. The validation plan starts with deterministic synthetic tests, then public EuRoC MAV datasets, then PX4 SITL/Gazebo, and only later physical hardware. The important engineering evidence is measured trajectory error, timing sensitivity, tracking-loss behaviour, recovery, latency and resource use.

## Questions to be ready for

### C++ / systems
1. Why keep the estimator core independent from ROS/PX4?
2. Which parts belong in C++ versus Python?
3. How would you avoid allocations in a real-time-ish estimation loop?
4. How would you structure ownership of image/IMU buffers?
5. How would you profile a dropped-frame latency spike?

### State estimation
6. What is the estimator state in a VIO system?
7. Why are accelerometer and gyro biases estimated?
8. What makes visual scale observable or unobservable?
9. What is IMU preintegration and why is it useful?
10. What failure modes cause VIO divergence?

### Geometry / vision
11. Difference between essential matrix and homography?
12. How do you reject feature outliers?
13. What happens in low-texture scenes?
14. How does motion blur affect tracking?
15. Why are camera intrinsics and distortion important?

### Time synchronisation
16. What happens if camera and IMU timestamps are offset by 10–20 ms?
17. How can an offset be estimated?
18. Why can time offset look like spatial calibration error?

### UAV integration
19. How would you feed external pose/velocity into PX4?
20. What should happen when VIO transitions from TRACKING to LOST?
21. How would you test failsafe behaviour without flight hardware?

### Validation
22. ATE vs RPE?
23. Why EuRoC?
24. What evidence would make you trust a VIO improvement?
25. What still remains unproven after SITL?

## CV-ready truthful bullets after completion
- Developed a middleware-independent C++ visual-inertial navigation stack with explicit camera/IMU calibration, timestamp synchronisation and state-quality interfaces.
- Built reproducible EuRoC MAV evaluation tooling reporting ATE/RPE, tracking loss, timing sensitivity and estimator latency.
- Integrated the estimator with PX4 SITL/Gazebo for GPS-denied navigation and estimator-loss failsafe testing.
- Implemented deterministic fault-injection scenarios covering camera delay, frame loss, IMU gaps and degraded visual conditions.
