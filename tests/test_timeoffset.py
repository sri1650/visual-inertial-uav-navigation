import pytest
from vionav.synthetic import synthetic_motion_signals
from vionav.timeoffset import estimate_time_offset_ns

def test_recovers_known_offset():
    d = synthetic_motion_signals(candidate_offset_ns=12_000_000, noise_std=0.0)
    r = estimate_time_offset_ns(
        d["camera_times_ns"], d["camera_signal"],
        d["imu_times_ns"], d["imu_signal"],
        search_min_ns=-30_000_000,
        search_max_ns=30_000_000,
        step_ns=1_000_000,
        min_pairs=50,
    )
    assert abs(r["offset_ns"] - 12_000_000) <= 1_000_000

def test_noise_tolerant():
    d = synthetic_motion_signals(candidate_offset_ns=-8_000_000, noise_std=0.01)
    r = estimate_time_offset_ns(
        d["camera_times_ns"], d["camera_signal"],
        d["imu_times_ns"], d["imu_signal"],
        search_min_ns=-20_000_000,
        search_max_ns=20_000_000,
        step_ns=1_000_000,
        min_pairs=50,
    )
    assert abs(r["offset_ns"] + 8_000_000) <= 2_000_000

def test_reject_bad_step():
    with pytest.raises(ValueError):
        estimate_time_offset_ns([0],[0.0],[0,1],[0.0,0.0],search_min_ns=0,search_max_ns=1,step_ns=0)
