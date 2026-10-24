#pragma once
#include "vionav/types.hpp"
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace vionav {

struct QualityObservation {
  std::size_t tracked_features{};
  double inlier_ratio{1.0};
  double covariance_trace{};
  double camera_imu_residual_ms{};
  double imu_gap_ms{};
};

struct QualityAssessment {
  TrackingState state{TrackingState::INITIALISING};
  double score{};
  std::vector<std::string> reasons;
};

QualityAssessment assess_quality(const QualityObservation& observation);

enum class RecoveryAction {
  NONE,
  DEGRADED,
  HOLD_LOST,
  RESET,
  RECOVERING,
  RECOVERED
};

class RecoverySupervisor {
 public:
  explicit RecoverySupervisor(std::size_t lost_frames_before_reset=3,
                              std::size_t good_frames_to_recover=5);
  RecoveryAction observe(const QualityAssessment& assessment);
  std::uint64_t reset_count() const { return reset_count_; }

 private:
  std::size_t lost_frames_before_reset_;
  std::size_t good_frames_to_recover_;
  std::size_t lost_streak_{};
  std::size_t good_streak_{};
  std::uint64_t reset_count_{};
};

} // namespace vionav
