#include "vionav/estimator.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>
#include <vector>

namespace {

vionav::SensorCalibration calibration() {
  vionav::SensorCalibration value;
  value.camera = {320.0, 320.0, 320.0, 240.0, 640, 480, "radtan", {0, 0, 0, 0}};
  value.validate();
  return value;
}

std::vector<vionav::ImuSample> stationary() {
  std::vector<vionav::ImuSample> samples;
  for (std::int64_t index = 0; index < 20; ++index)
    samples.push_back({index * 10'000'000, {}, {0.0, 0.0, 9.80665}});
  return samples;
}

bool has_reason(const vionav::QualityAssessment& assessment, const char* reason) {
  return std::find(assessment.reasons.begin(), assessment.reasons.end(), reason) !=
         assessment.reasons.end();
}

}  // namespace

int main() {
  using namespace vionav;
  VisualInertialEkf estimator(calibration());
  const auto initial_imu = stationary();
  estimator.initialise(
      {initial_imu.back().timestamp_ns, {}, {}, 0.03, 0.02, 140, 0.95}, initial_imu);

  const auto initial_quality = estimator.quality_assessment();
  assert(initial_quality.state == TrackingState::TRACKING);
  const auto covariance = estimator.covariance_diagonal();
  const double covariance_trace = std::accumulate(covariance.begin(), covariance.end(), 0.0);
  assert(std::abs(initial_quality.score - 100.0) < 1e-12);
  assert(std::abs(estimator.diagnostics().covariance_trace - covariance_trace) < 1e-12);

  const std::int64_t first = initial_imu.back().timestamp_ns + 10'000'000;
  estimator.predict({first, {}, {0.0, 0.0, 9.80665}});
  estimator.update_visual({first, {}, {}, 0.03, 0.02, 35, 0.40});
  assert(estimator.state().tracking == TrackingState::DEGRADED);
  assert(has_reason(estimator.quality_assessment(), "low_feature_count"));
  assert(has_reason(estimator.quality_assessment(), "weak_geometry"));

  const std::int64_t second = first + 10'000'000;
  estimator.predict({second, {}, {0.0, 0.0, 9.80665}});
  estimator.update_visual({second, {2.0, 0.0, 0.0}, {}, 0.01, 0.01, 140, 0.95});
  assert(estimator.state().tracking == TrackingState::LOST);
  assert(estimator.diagnostics().visual_innovation_nis > 25.0);
  assert(has_reason(estimator.quality_assessment(), "innovation_outlier"));

  VisualInertialEkf delayed(calibration());
  delayed.initialise(
      {initial_imu.back().timestamp_ns, {}, {}, 0.03, 0.02, 140, 0.95}, initial_imu);
  for (int index = 1; index <= 4; ++index) {
    delayed.predict({initial_imu.back().timestamp_ns + index * 10'000'000,
                     {},
                     {0.0, 0.0, 9.80665}});
  }
  delayed.update_visual({initial_imu.back().timestamp_ns + 5'000'000,
                         {},
                         {},
                         0.03,
                         0.02,
                         140,
                         0.95});
  assert(delayed.state().tracking == TrackingState::LOST);
  assert(delayed.diagnostics().camera_imu_residual_ms == 35.0);
  assert(has_reason(delayed.quality_assessment(), "time_sync_failure"));
}
