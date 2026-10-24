from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import math


class QualityState(str, Enum):
    TRACKING="TRACKING"
    DEGRADED="DEGRADED"
    LOST="LOST"


@dataclass(frozen=True)
class QualityInputs:
    tracked_features: int
    inlier_ratio: float
    covariance_trace: float
    camera_imu_residual_ms: float
    imu_gap_ms: float


@dataclass(frozen=True)
class QualityResult:
    state: QualityState
    score: float
    reasons: tuple[str,...]


def assess_quality(x: QualityInputs) -> QualityResult:
    vals=(x.inlier_ratio,x.covariance_trace,x.camera_imu_residual_ms,x.imu_gap_ms)
    if not all(math.isfinite(float(v)) for v in vals):
        return QualityResult(QualityState.LOST,0.0,("non_finite_metric",))
    reasons=[]
    score=100.0

    if x.tracked_features < 15:
        reasons.append("feature_collapse"); score-=65
    elif x.tracked_features < 50:
        reasons.append("low_feature_count"); score-=25

    if x.inlier_ratio < .2:
        reasons.append("geometric_outliers"); score-=55
    elif x.inlier_ratio < .5:
        reasons.append("weak_geometry"); score-=20

    if x.covariance_trace > 100:
        reasons.append("covariance_diverged"); score-=70
    elif x.covariance_trace > 10:
        reasons.append("covariance_high"); score-=20

    if abs(x.camera_imu_residual_ms) > 30:
        reasons.append("time_sync_failure"); score-=60
    elif abs(x.camera_imu_residual_ms) > 10:
        reasons.append("time_sync_degraded"); score-=20

    if x.imu_gap_ms > 100:
        reasons.append("imu_gap"); score-=70
    elif x.imu_gap_ms > 20:
        reasons.append("imu_gap_degraded"); score-=20

    score=max(0.0,score)
    if score < 35 or any(r in reasons for r in ("feature_collapse","covariance_diverged","time_sync_failure","imu_gap")):
        state=QualityState.LOST
    elif score < 80 or reasons:
        state=QualityState.DEGRADED
    else:
        state=QualityState.TRACKING
    return QualityResult(state,score,tuple(reasons))


@dataclass
class RecoverySupervisor:
    lost_frames_before_reset: int = 3
    good_frames_to_recover: int = 5
    lost_streak: int = 0
    good_streak: int = 0
    reset_count: int = 0

    def observe(self, result: QualityResult) -> str:
        if result.state is QualityState.LOST:
            self.lost_streak += 1
            self.good_streak = 0
            if self.lost_streak >= self.lost_frames_before_reset:
                self.reset_count += 1
                self.lost_streak = 0
                return "RESET"
            return "HOLD_LOST"
        self.lost_streak = 0
        if result.state is QualityState.TRACKING:
            self.good_streak += 1
            if self.good_streak >= self.good_frames_to_recover:
                return "RECOVERED"
            return "RECOVERING"
        self.good_streak = 0
        return "DEGRADED"
