from __future__ import annotations
from dataclasses import dataclass
from enum import Enum


class Health(str,Enum):
    HEALTHY="HEALTHY"
    DEGRADED="DEGRADED"
    FAILSAFE="FAILSAFE"


@dataclass(frozen=True)
class WatchdogInputs:
    now_ns:int
    last_camera_ns:int
    last_imu_ns:int
    last_estimator_ns:int
    estimator_tracking:bool


@dataclass(frozen=True)
class WatchdogResult:
    health:Health
    reasons:tuple[str,...]
    allow_external_vision:bool
    request_failsafe:bool


def evaluate_watchdog(
    x:WatchdogInputs,
    *,
    camera_warn_ms:float=100,
    camera_fail_ms:float=500,
    imu_warn_ms:float=30,
    imu_fail_ms:float=100,
    estimator_fail_ms:float=250,
) -> WatchdogResult:
    if x.now_ns < 0:
        raise ValueError("now_ns cannot be negative")
    reasons=[]
    camera_age=max(0,x.now_ns-x.last_camera_ns)/1e6
    imu_age=max(0,x.now_ns-x.last_imu_ns)/1e6
    est_age=max(0,x.now_ns-x.last_estimator_ns)/1e6

    hard=False
    degraded=False
    if camera_age>camera_fail_ms: reasons.append("camera_timeout"); hard=True
    elif camera_age>camera_warn_ms: reasons.append("camera_stale"); degraded=True
    if imu_age>imu_fail_ms: reasons.append("imu_timeout"); hard=True
    elif imu_age>imu_warn_ms: reasons.append("imu_stale"); degraded=True
    if est_age>estimator_fail_ms: reasons.append("estimator_timeout"); hard=True
    if not x.estimator_tracking: reasons.append("estimator_not_tracking"); hard=True

    health=Health.FAILSAFE if hard else Health.DEGRADED if degraded else Health.HEALTHY
    return WatchdogResult(health,tuple(reasons),health is not Health.FAILSAFE,health is Health.FAILSAFE)
