from __future__ import annotations
import math
import random


def synthetic_motion_signals(
    *,
    duration_s: float = 8.0,
    camera_hz: float = 20.0,
    imu_hz: float = 200.0,
    candidate_offset_ns: int = 12_000_000,
    noise_std: float = 0.0,
    seed: int = 7,
):
    """Generate deterministic camera-like and IMU-like angular signals.

    The IMU timestamps are shifted by `candidate_offset_ns`.
    """
    if duration_s <= 0 or camera_hz <= 0 or imu_hz <= 0:
        raise ValueError("duration and rates must be positive")
    rng = random.Random(seed)

    def signal(t):
        return (
            0.8 * math.sin(2.0 * math.pi * 0.7 * t)
            + 0.35 * math.sin(2.0 * math.pi * 1.9 * t + 0.3)
            + 0.12 * math.cos(2.0 * math.pi * 3.1 * t)
        )

    cam_times, cam_vals = [], []
    n_cam = int(duration_s * camera_hz)
    for i in range(n_cam):
        t = i / camera_hz
        cam_times.append(int(round(t * 1e9)))
        cam_vals.append(signal(t) + rng.gauss(0.0, noise_std))

    imu_times, imu_vals = [], []
    n_imu = int(duration_s * imu_hz)
    for i in range(n_imu):
        t = i / imu_hz
        imu_times.append(int(round(t * 1e9)) + int(candidate_offset_ns))
        imu_vals.append(signal(t) + rng.gauss(0.0, noise_std))

    return {
        "camera_times_ns": cam_times,
        "camera_signal": cam_vals,
        "imu_times_ns": imu_times,
        "imu_signal": imu_vals,
        "true_offset_ns": int(candidate_offset_ns),
    }
