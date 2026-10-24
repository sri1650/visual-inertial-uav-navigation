from vionav.quality import *

def good():
    return QualityInputs(120,.9,1.0,1.0,2.0)

def test_good_tracking():
    r=assess_quality(good())
    assert r.state is QualityState.TRACKING and r.score==100

def test_timing_failure_is_lost():
    r=assess_quality(QualityInputs(120,.9,1,40,2))
    assert r.state is QualityState.LOST
    assert "time_sync_failure" in r.reasons

def test_recovery_supervisor_resets_after_bounded_loss():
    s=RecoverySupervisor(lost_frames_before_reset=2,good_frames_to_recover=2)
    lost=assess_quality(QualityInputs(5,.1,200,50,150))
    assert s.observe(lost)=="HOLD_LOST"
    assert s.observe(lost)=="RESET"
    assert s.reset_count==1
    ok=assess_quality(good())
    assert s.observe(ok)=="RECOVERING"
    assert s.observe(ok)=="RECOVERED"
