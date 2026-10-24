from vionav.fault_campaign import run_retained_fault_campaign
from vionav.quality import QualityState

def test_retained_campaign_contains_expected_transitions():
    by={r.name:r for r in run_retained_fault_campaign()}
    assert by["nominal"].state is QualityState.TRACKING
    assert by["camera_delay_15ms"].state is QualityState.DEGRADED
    assert by["camera_delay_40ms"].state is QualityState.LOST
    assert by["imu_gap_150ms"].state is QualityState.LOST
