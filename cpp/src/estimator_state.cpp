#include "vionav/estimator_state.hpp"
#include <cmath>

namespace vionav {

TrackingState TrackingStateMachine::update(
    bool initialised,
    std::size_t tracked_features,
    double covariance_trace,
    bool sensor_timeout,
    bool hard_failure) {
  if (!std::isfinite(covariance_trace) || covariance_trace < 0.0)
    throw std::invalid_argument("invalid covariance trace");
  if (state_ == TrackingState::RESETTING) return state_;
  if (!initialised) {
    state_ = TrackingState::INITIALISING;
    return state_;
  }
  if (hard_failure || sensor_timeout || tracked_features < 15 || covariance_trace > 100.0) {
    state_ = TrackingState::LOST;
    return state_;
  }
  if (tracked_features < 50 || covariance_trace > 10.0) {
    state_ = TrackingState::DEGRADED;
    return state_;
  }
  state_ = TrackingState::TRACKING;
  return state_;
}

void TrackingStateMachine::begin_reset() {
  state_ = TrackingState::RESETTING;
}

void TrackingStateMachine::reset_complete(bool initialised) {
  state_ = initialised ? TrackingState::TRACKING : TrackingState::INITIALISING;
}

} // namespace vionav
