from vionav.watchdog import *

def test_healthy():
    r=evaluate_watchdog(WatchdogInputs(1_000_000_000,950_000_000,990_000_000,980_000_000,True))
    assert r.health is Health.HEALTHY and r.allow_external_vision

def test_camera_degraded():
    r=evaluate_watchdog(WatchdogInputs(1_000_000_000,800_000_000,990_000_000,980_000_000,True))
    assert r.health is Health.DEGRADED
    assert "camera_stale" in r.reasons

def test_imu_timeout_requests_failsafe():
    r=evaluate_watchdog(WatchdogInputs(1_000_000_000,990_000_000,800_000_000,980_000_000,True))
    assert r.health is Health.FAILSAFE
    assert r.request_failsafe
