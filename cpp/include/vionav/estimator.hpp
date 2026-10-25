#pragma once

#include "vionav/calibration.hpp"
#include "vionav/estimator_state.hpp"
#include "vionav/quality.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace vionav {

// Pose of the body frame in the world frame. Camera-front-end adapters must
// apply T_body_camera before constructing this middleware-independent input.
struct VisualPoseMeasurement {
  std::int64_t timestamp_ns{};
  Vec3 position_world_m{};
  Quaternion orientation_world_body{};
  double position_std_m{0.05};
  double orientation_std_rad{0.03};
  std::size_t tracked_features{};
  double inlier_ratio{1.0};
};

struct EstimatorConfig {
  Vec3 gravity_world_m_s2{0.0, 0.0, -9.80665};
  double gyro_noise_rad_s{0.004};
  double accel_noise_m_s2{0.08};
  double gyro_bias_random_walk{0.0002};
  double accel_bias_random_walk{0.004};
  double max_imu_dt_s{0.05};
  std::size_t minimum_initialisation_samples{20};
  std::size_t lost_frames_before_reset{3};
  std::size_t good_frames_to_recover{5};
};

struct EstimatorDiagnostics {
  double covariance_trace{};
  double position_innovation_norm_m{};
  double orientation_innovation_norm_rad{};
  double visual_innovation_nis{};
  double camera_imu_residual_ms{};
  double last_imu_gap_ms{};
  std::size_t tracked_features{};
  double inlier_ratio{1.0};
  std::int64_t last_camera_timestamp_ns{};
  std::int64_t last_imu_timestamp_ns{};
};

// 15-state multiplicative error-state EKF:
// [position, velocity, orientation error, gyro bias, accelerometer bias].
class VisualInertialEkf {
 public:
  explicit VisualInertialEkf(SensorCalibration calibration,
                             EstimatorConfig config = {});
  ~VisualInertialEkf();
  VisualInertialEkf(VisualInertialEkf&&) noexcept;
  VisualInertialEkf& operator=(VisualInertialEkf&&) noexcept;
  VisualInertialEkf(const VisualInertialEkf&) = delete;
  VisualInertialEkf& operator=(const VisualInertialEkf&) = delete;

  void initialise(const VisualPoseMeasurement& initial_pose,
                  const std::vector<ImuSample>& stationary_imu);
  void predict(const ImuSample& sample);
  void update_visual(const VisualPoseMeasurement& measurement);
  void reset();
  void begin_reinitialisation();
  RecoveryAction supervise_quality();

  bool initialised() const;
  const EstimatorState& state() const;
  const EstimatorDiagnostics& diagnostics() const;
  QualityAssessment quality_assessment() const;
  std::array<double, 15> covariance_diagonal() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vionav
