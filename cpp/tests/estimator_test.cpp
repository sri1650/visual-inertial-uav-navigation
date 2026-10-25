#include "vionav/estimator.hpp"

#include <cassert>
#include <cmath>
#include <vector>

namespace {

vionav::SensorCalibration calibration() {
  vionav::SensorCalibration value;
  value.camera = {320.0, 320.0, 320.0, 240.0, 640, 480, "radtan", {0, 0, 0, 0}};
  value.validate();
  return value;
}

std::vector<vionav::ImuSample> stationary_samples() {
  std::vector<vionav::ImuSample> samples;
  for (std::int64_t index = 0; index < 20; ++index) {
    samples.push_back({index * 10'000'000, {0.01, -0.02, 0.005}, {0.10, -0.05, 9.82665}});
  }
  return samples;
}

}  // namespace

int main() {
  using namespace vionav;
  VisualInertialEkf estimator(calibration());
  const auto stationary = stationary_samples();
  estimator.initialise(
      {stationary.back().timestamp_ns, {}, {}, 0.03, 0.02, 150, 0.95}, stationary);
  assert(estimator.initialised());
  assert(std::abs(estimator.state().bias.gyro_rad_s.x - 0.01) < 1e-12);
  assert(std::abs(estimator.state().bias.accel_m_s2.x - 0.10) < 1e-12);

  const std::int64_t start = stationary.back().timestamp_ns;
  for (int index = 1; index <= 200; ++index) {
    const double elapsed = index * 0.01;
    const std::int64_t timestamp = start + index * 10'000'000;
    estimator.predict({timestamp, {0.01, -0.02, 0.005}, {1.10, -0.05, 9.82665}});
    if (index % 10 == 0) {
      estimator.update_visual({timestamp,
                               {0.5 * elapsed * elapsed, 0.0, 0.0},
                               {},
                               0.03,
                               0.02,
                               140,
                               0.92});
    }
  }

  assert(std::abs(estimator.state().position_m.x - 2.0) < 0.08);
  assert(std::abs(estimator.state().velocity_m_s.x - 2.0) < 0.10);
  assert(estimator.state().position_m.y * estimator.state().position_m.y < 1e-4);
  assert(estimator.diagnostics().position_innovation_norm_m < 0.08);
  for (double variance : estimator.covariance_diagonal()) {
    assert(std::isfinite(variance));
    assert(variance >= 0.0);
  }

  bool rejected_gap = false;
  try {
    estimator.predict({start + 4'000'000'000, {}, {0.0, 0.0, 9.80665}});
  } catch (const std::invalid_argument&) {
    rejected_gap = true;
  }
  assert(rejected_gap);
}
