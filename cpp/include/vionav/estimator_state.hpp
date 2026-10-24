#pragma once
#include "vionav/types.hpp"
#include <cstdint>
#include <stdexcept>

namespace vionav {

struct EstimatorQuality {
  double score{0.0};
  std::size_t tracked_features{};
  double covariance_trace{};
  std::int64_t last_camera_timestamp_ns{};
  std::int64_t last_imu_timestamp_ns{};
};

struct EstimatorState {
  TrackingState tracking{TrackingState::INITIALISING};
  Vec3 position_m{};
  Quaternion orientation{};
  Vec3 velocity_m_s{};
  ImuBias bias{};
  EstimatorQuality quality{};
  std::uint64_t reset_count{};
};

class TrackingStateMachine {
 public:
  TrackingState state() const { return state_; }
  TrackingState update(bool initialised,
                       std::size_t tracked_features,
                       double covariance_trace,
                       bool sensor_timeout,
                       bool hard_failure);
  void begin_reset();
  void reset_complete(bool initialised);

 private:
  TrackingState state_{TrackingState::INITIALISING};
};

} // namespace vionav
