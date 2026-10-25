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

std::vector<vionav::ImuSample> stationary(std::int64_t start_ns,
                                           const vionav::Vec3& gyro_bias = {}) {
  std::vector<vionav::ImuSample> samples;
  for (std::int64_t index = 0; index < 20; ++index)
    samples.push_back(
        {start_ns + index * 10'000'000, gyro_bias, {0.0, 0.0, 9.80665}});
  return samples;
}

}  // namespace

int main() {
  using namespace vionav;
  EstimatorConfig config;
  config.lost_frames_before_reset = 2;
  config.good_frames_to_recover = 2;
  VisualInertialEkf estimator(calibration(), config);
  const auto first_stationary = stationary(0);
  estimator.initialise(
      {first_stationary.back().timestamp_ns, {4.0, 1.0, 0.0}, {}, 0.03, 0.02, 140, 0.95},
      first_stationary);
  assert(estimator.state().tracking == TrackingState::TRACKING);

  std::int64_t timestamp = first_stationary.back().timestamp_ns + 10'000'000;
  estimator.predict({timestamp, {}, {1.0, 0.0, 9.80665}});
  estimator.update_visual({timestamp, {4.0, 1.0, 0.0}, {}, 0.03, 0.02, 35, 0.40});
  assert(estimator.state().tracking == TrackingState::DEGRADED);
  assert(estimator.supervise_quality() == RecoveryAction::DEGRADED);

  timestamp += 10'000'000;
  estimator.predict({timestamp, {}, {1.0, 0.0, 9.80665}});
  estimator.update_visual({timestamp, {4.0, 1.0, 0.0}, {}, 0.03, 0.02, 5, 0.10});
  assert(estimator.state().tracking == TrackingState::LOST);
  assert(estimator.supervise_quality() == RecoveryAction::HOLD_LOST);

  timestamp += 10'000'000;
  estimator.predict({timestamp, {}, {1.0, 0.0, 9.80665}});
  estimator.update_visual({timestamp, {4.0, 1.0, 0.0}, {}, 0.03, 0.02, 5, 0.10});
  assert(estimator.supervise_quality() == RecoveryAction::RESET);
  assert(estimator.state().tracking == TrackingState::RESETTING);
  assert(!estimator.initialised());
  assert(estimator.state().reset_count == 1);
  assert(estimator.state().position_m.norm() == 0.0);
  assert(estimator.state().velocity_m_s.norm() == 0.0);
  assert(estimator.state().bias.gyro_rad_s.norm() == 0.0);

  estimator.begin_reinitialisation();
  assert(estimator.state().tracking == TrackingState::INITIALISING);
  const auto second_stationary = stationary(1'000'000'000, {0.02, 0.0, 0.0});
  estimator.initialise({second_stationary.back().timestamp_ns,
                        {10.0, -2.0, 1.0},
                        {},
                        0.03,
                        0.02,
                        160,
                        0.98},
                       second_stationary);
  assert(estimator.state().tracking == TrackingState::TRACKING);
  assert(estimator.state().reset_count == 1);
  assert(std::abs(estimator.state().position_m.x - 10.0) < 1e-12);
  assert(std::abs(estimator.state().position_m.y + 2.0) < 1e-12);
  assert(estimator.state().velocity_m_s.norm() == 0.0);
  assert(std::abs(estimator.state().bias.gyro_rad_s.x - 0.02) < 1e-12);
  assert(estimator.supervise_quality() == RecoveryAction::RECOVERING);

  timestamp = second_stationary.back().timestamp_ns + 10'000'000;
  estimator.predict({timestamp, {0.02, 0.0, 0.0}, {0.0, 0.0, 9.80665}});
  estimator.update_visual(
      {timestamp, {10.0, -2.0, 1.0}, {}, 0.03, 0.02, 160, 0.98});
  assert(estimator.supervise_quality() == RecoveryAction::RECOVERED);
  assert(estimator.state().tracking == TrackingState::TRACKING);
}
