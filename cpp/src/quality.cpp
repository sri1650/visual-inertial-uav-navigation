#include "vionav/quality.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vionav {

QualityAssessment assess_quality(const QualityObservation& x) {
  if (!std::isfinite(x.inlier_ratio) || !std::isfinite(x.covariance_trace) ||
      !std::isfinite(x.camera_imu_residual_ms) || !std::isfinite(x.imu_gap_ms) ||
      !std::isfinite(x.visual_innovation_nis))
    return {TrackingState::LOST,0.0,{"non_finite_metric"}};
  if (x.inlier_ratio < 0.0 || x.inlier_ratio > 1.0 || x.covariance_trace < 0.0 ||
      x.imu_gap_ms < 0.0 || x.visual_innovation_nis < 0.0)
    throw std::invalid_argument("invalid quality observation");

  double score=100.0;
  std::vector<std::string> reasons;

  if (x.tracked_features < 15) { reasons.push_back("feature_collapse"); score-=65; }
  else if (x.tracked_features < 50) { reasons.push_back("low_feature_count"); score-=25; }

  if (x.inlier_ratio < .2) { reasons.push_back("geometric_outliers"); score-=55; }
  else if (x.inlier_ratio < .5) { reasons.push_back("weak_geometry"); score-=20; }

  if (x.covariance_trace > 100) { reasons.push_back("covariance_diverged"); score-=70; }
  else if (x.covariance_trace > 10) { reasons.push_back("covariance_high"); score-=20; }

  if (std::abs(x.camera_imu_residual_ms) > 30) { reasons.push_back("time_sync_failure"); score-=60; }
  else if (std::abs(x.camera_imu_residual_ms) > 10) { reasons.push_back("time_sync_degraded"); score-=20; }

  if (x.imu_gap_ms > 100) { reasons.push_back("imu_gap"); score-=70; }
  else if (x.imu_gap_ms > 20) { reasons.push_back("imu_gap_degraded"); score-=20; }

  // Six-degree-of-freedom normalized innovation squared (NIS). The bounds
  // are intentionally conservative around the chi-square tail for 6 DoF.
  if (x.visual_innovation_nis > 25.0) { reasons.push_back("innovation_outlier"); score-=70; }
  else if (x.visual_innovation_nis > 12.6) { reasons.push_back("innovation_high"); score-=20; }

  score=std::max(0.0,score);
  const auto has=[&](const char* s){
    return std::find(reasons.begin(),reasons.end(),std::string{s})!=reasons.end();
  };

  TrackingState state=TrackingState::TRACKING;
  if (score < 35 || has("feature_collapse") || has("covariance_diverged") ||
      has("time_sync_failure") || has("imu_gap"))
    state=TrackingState::LOST;
  else if (score < 80 || !reasons.empty())
    state=TrackingState::DEGRADED;

  return {state,score,reasons};
}

RecoverySupervisor::RecoverySupervisor(std::size_t lost_frames_before_reset,
                                       std::size_t good_frames_to_recover)
    : lost_frames_before_reset_(lost_frames_before_reset),
      good_frames_to_recover_(good_frames_to_recover) {
  if (lost_frames_before_reset_ == 0 || good_frames_to_recover_ == 0)
    throw std::invalid_argument("recovery thresholds must be positive");
}

RecoveryAction RecoverySupervisor::observe(const QualityAssessment& q) {
  if (q.state == TrackingState::LOST) {
    ++lost_streak_; good_streak_=0;
    if (lost_streak_ >= lost_frames_before_reset_) {
      lost_streak_=0; ++reset_count_;
      return RecoveryAction::RESET;
    }
    return RecoveryAction::HOLD_LOST;
  }
  lost_streak_=0;
  if (q.state == TrackingState::TRACKING) {
    ++good_streak_;
    if (good_streak_ >= good_frames_to_recover_)
      return RecoveryAction::RECOVERED;
    return RecoveryAction::RECOVERING;
  }
  good_streak_=0;
  return RecoveryAction::DEGRADED;
}

} // namespace vionav
