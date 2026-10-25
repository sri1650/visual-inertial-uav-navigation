# M6-T02 live PX4 external-vision acceptance

- Date: 2026-10-25
- PX4 commit: `22b15b91fba5e79f01248d1c644ef942382af517`
- Simulator: Gazebo 8.10.0, headless x500
- Transport: MAVLink 2 `ODOMETRY`, `MAV_FRAME_LOCAL_NED`, `MAV_ESTIMATOR_TYPE_VIO`
- Publisher command: `.venv/bin/python tools/send_external_vision.py --duration 20 --rate-hz 30`
- First stream: 523 messages sent; second EKF-fusion stream: 387 messages sent

PX4 `vehicle_visual_odometry` received the converted sample with position `[0, 0, 0]`, quaternion `[0.70711, 0, 0, 0.70711]`, NED pose and velocity frames, position variance `[0.01, 0.01, 0.02]`, velocity variance `[0.04, 0.04, 0.06]`, reset counter `0`, and quality `90`.

After setting `EKF2_EV_CTRL=7`, `estimator_status_flags` reported:

- `cs_ev_pos: True`
- `cs_ev_hgt: True`
- `cs_ev_vel: True`
- `fault_status_changes: 0`

This is SITL evidence only. It is not physical-flight or hardware validation.
