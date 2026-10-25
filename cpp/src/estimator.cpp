#include "vionav/estimator.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vionav {
namespace {

using Matrix15 = Eigen::Matrix<double, 15, 15>;
using Vector15 = Eigen::Matrix<double, 15, 1>;

Eigen::Vector3d eigen(const Vec3& value) { return {value.x, value.y, value.z}; }
Vec3 domain(const Eigen::Vector3d& value) { return {value.x(), value.y(), value.z()}; }

Eigen::Quaterniond eigen(const Quaternion& value) {
  const auto normalised = value.normalized();
  return {normalised.w, normalised.x, normalised.y, normalised.z};
}

Quaternion domain(const Eigen::Quaterniond& value) {
  const auto normalised = value.normalized();
  return {normalised.w(), normalised.x(), normalised.y(), normalised.z()};
}

Eigen::Matrix3d skew(const Eigen::Vector3d& value) {
  Eigen::Matrix3d result;
  result << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(), -value.y(),
      value.x(), 0.0;
  return result;
}

Eigen::Vector3d quaternion_residual(Eigen::Quaterniond expected,
                                    Eigen::Quaterniond measured) {
  Eigen::Quaterniond error = expected.conjugate() * measured;
  if (error.w() < 0.0) error.coeffs() *= -1.0;
  error.normalize();
  const Eigen::AngleAxisd angle_axis(error);
  if (!std::isfinite(angle_axis.angle())) throw std::invalid_argument("invalid visual orientation");
  return angle_axis.axis() * angle_axis.angle();
}

Eigen::Matrix3d calibration_rotation(const RigidTransform& transform) {
  Eigen::Matrix3d result;
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      result(row, column) = transform.rotation[row][column];
  return result;
}

void require_finite(const Eigen::Vector3d& value, const char* label) {
  if (!value.allFinite()) throw std::invalid_argument(label);
}

}  // namespace

struct VisualInertialEkf::Impl {
  SensorCalibration calibration;
  EstimatorConfig config;
  EstimatorState state;
  EstimatorDiagnostics diagnostics;
  Matrix15 covariance{Matrix15::Identity()};
  Eigen::Matrix3d rotation_body_imu{Eigen::Matrix3d::Identity()};
  std::int64_t last_imu_timestamp_ns{};
  bool is_initialised{};

  explicit Impl(SensorCalibration calibration_value, EstimatorConfig config_value)
      : calibration(std::move(calibration_value)), config(config_value) {
    calibration.validate();
    if (!(config.gyro_noise_rad_s > 0.0 && config.accel_noise_m_s2 > 0.0 &&
          config.gyro_bias_random_walk > 0.0 && config.accel_bias_random_walk > 0.0 &&
          config.max_imu_dt_s > 0.0 && config.minimum_initialisation_samples >= 2))
      throw std::invalid_argument("invalid estimator configuration");
    rotation_body_imu = calibration_rotation(calibration.T_body_imu);
    reset_covariance();
  }

  void reset_covariance() {
    covariance.setZero();
    covariance.diagonal().segment<3>(0).setConstant(0.05 * 0.05);
    covariance.diagonal().segment<3>(3).setConstant(0.20 * 0.20);
    covariance.diagonal().segment<3>(6).setConstant(0.05 * 0.05);
    covariance.diagonal().segment<3>(9).setConstant(0.01 * 0.01);
    covariance.diagonal().segment<3>(12).setConstant(0.10 * 0.10);
  }

  void update_quality() {
    diagnostics.covariance_trace = covariance.trace();
    state.quality.covariance_trace = diagnostics.covariance_trace;
    state.quality.tracked_features = diagnostics.tracked_features;
    state.quality.last_camera_timestamp_ns = diagnostics.last_camera_timestamp_ns;
    state.quality.last_imu_timestamp_ns = diagnostics.last_imu_timestamp_ns;
  }

  QualityAssessment assess() const {
    return assess_quality({diagnostics.tracked_features,
                           diagnostics.inlier_ratio,
                           diagnostics.covariance_trace,
                           diagnostics.camera_imu_residual_ms,
                           diagnostics.last_imu_gap_ms,
                           diagnostics.visual_innovation_nis});
  }
};

VisualInertialEkf::VisualInertialEkf(SensorCalibration calibration, EstimatorConfig config)
    : impl_(std::make_unique<Impl>(std::move(calibration), config)) {}
VisualInertialEkf::~VisualInertialEkf() = default;
VisualInertialEkf::VisualInertialEkf(VisualInertialEkf&&) noexcept = default;
VisualInertialEkf& VisualInertialEkf::operator=(VisualInertialEkf&&) noexcept = default;

void VisualInertialEkf::initialise(const VisualPoseMeasurement& pose,
                                   const std::vector<ImuSample>& stationary_imu) {
  if (pose.timestamp_ns < 0) throw std::invalid_argument("negative initial timestamp");
  if (stationary_imu.size() < impl_->config.minimum_initialisation_samples)
    throw std::invalid_argument("insufficient stationary IMU samples");

  Eigen::Vector3d mean_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_accel = Eigen::Vector3d::Zero();
  std::int64_t previous_timestamp = -1;
  for (const auto& sample : stationary_imu) {
    if (sample.timestamp_ns <= previous_timestamp)
      throw std::invalid_argument("initialisation IMU timestamps must increase");
    previous_timestamp = sample.timestamp_ns;
    const Eigen::Vector3d gyro = impl_->rotation_body_imu * eigen(sample.angular_velocity_rad_s);
    const Eigen::Vector3d accel = impl_->rotation_body_imu * eigen(sample.linear_acceleration_m_s2);
    require_finite(gyro, "non-finite gyroscope sample");
    require_finite(accel, "non-finite accelerometer sample");
    mean_gyro += gyro;
    mean_accel += accel;
  }
  mean_gyro /= static_cast<double>(stationary_imu.size());
  mean_accel /= static_cast<double>(stationary_imu.size());
  if (mean_gyro.norm() > 0.5) throw std::invalid_argument("platform is not stationary");

  const Eigen::Quaterniond orientation = eigen(pose.orientation_world_body);
  const Eigen::Vector3d expected_specific_force =
      orientation.conjugate() * (-eigen(impl_->config.gravity_world_m_s2));
  if (std::abs(mean_accel.norm() - expected_specific_force.norm()) > 2.0)
    throw std::invalid_argument("stationary acceleration is inconsistent with gravity");

  impl_->state = {};
  impl_->state.tracking = TrackingState::TRACKING;
  impl_->state.position_m = pose.position_world_m;
  impl_->state.orientation = domain(orientation);
  impl_->state.bias.gyro_rad_s = domain(mean_gyro);
  impl_->state.bias.accel_m_s2 = domain(mean_accel - expected_specific_force);
  impl_->diagnostics = {};
  impl_->diagnostics.tracked_features = pose.tracked_features;
  impl_->diagnostics.inlier_ratio = pose.inlier_ratio;
  impl_->diagnostics.last_camera_timestamp_ns = pose.timestamp_ns;
  impl_->diagnostics.last_imu_timestamp_ns = stationary_imu.back().timestamp_ns;
  impl_->last_imu_timestamp_ns = stationary_imu.back().timestamp_ns;
  impl_->reset_covariance();
  impl_->is_initialised = true;
  impl_->update_quality();
}

void VisualInertialEkf::predict(const ImuSample& sample) {
  if (!impl_->is_initialised) throw std::logic_error("estimator is not initialised");
  if (sample.timestamp_ns <= impl_->last_imu_timestamp_ns)
    throw std::invalid_argument("IMU timestamps must increase");
  const double dt = static_cast<double>(sample.timestamp_ns - impl_->last_imu_timestamp_ns) * 1e-9;
  impl_->diagnostics.last_imu_gap_ms = dt * 1e3;
  if (!(dt > 0.0) || dt > impl_->config.max_imu_dt_s) {
    impl_->state.tracking = TrackingState::LOST;
    throw std::invalid_argument("IMU gap exceeds configured limit");
  }

  const Eigen::Vector3d gyro = impl_->rotation_body_imu * eigen(sample.angular_velocity_rad_s) -
                               eigen(impl_->state.bias.gyro_rad_s);
  const Eigen::Vector3d accel = impl_->rotation_body_imu * eigen(sample.linear_acceleration_m_s2) -
                               eigen(impl_->state.bias.accel_m_s2);
  require_finite(gyro, "non-finite gyroscope sample");
  require_finite(accel, "non-finite accelerometer sample");

  Eigen::Quaterniond orientation = eigen(impl_->state.orientation);
  const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
  const Eigen::Vector3d acceleration_world = rotation * accel + eigen(impl_->config.gravity_world_m_s2);
  Eigen::Vector3d position = eigen(impl_->state.position_m);
  Eigen::Vector3d velocity = eigen(impl_->state.velocity_m_s);
  position += velocity * dt + 0.5 * acceleration_world * dt * dt;
  velocity += acceleration_world * dt;
  const double angle = gyro.norm() * dt;
  const Eigen::Quaterniond delta = angle < 1e-12
      ? Eigen::Quaterniond(1.0, gyro.x() * dt * 0.5, gyro.y() * dt * 0.5,
                           gyro.z() * dt * 0.5)
      : Eigen::Quaterniond(Eigen::AngleAxisd(angle, gyro.normalized()));
  orientation = (orientation * delta).normalized();

  Matrix15 transition = Matrix15::Identity();
  transition.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  transition.block<3, 3>(3, 6) = -rotation * skew(accel) * dt;
  transition.block<3, 3>(3, 12) = -rotation * dt;
  transition.block<3, 3>(6, 6) -= skew(gyro) * dt;
  transition.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;

  Matrix15 process_noise = Matrix15::Zero();
  const double gyro_variance = impl_->config.gyro_noise_rad_s * impl_->config.gyro_noise_rad_s;
  const double accel_variance = impl_->config.accel_noise_m_s2 * impl_->config.accel_noise_m_s2;
  process_noise.diagonal().segment<3>(0).setConstant(0.25 * accel_variance * dt * dt * dt * dt);
  process_noise.diagonal().segment<3>(3).setConstant(accel_variance * dt * dt);
  process_noise.diagonal().segment<3>(6).setConstant(gyro_variance * dt * dt);
  process_noise.diagonal().segment<3>(9).setConstant(
      impl_->config.gyro_bias_random_walk * impl_->config.gyro_bias_random_walk * dt);
  process_noise.diagonal().segment<3>(12).setConstant(
      impl_->config.accel_bias_random_walk * impl_->config.accel_bias_random_walk * dt);
  impl_->covariance = transition * impl_->covariance * transition.transpose() + process_noise;
  impl_->covariance = 0.5 * (impl_->covariance + impl_->covariance.transpose());

  impl_->state.position_m = domain(position);
  impl_->state.velocity_m_s = domain(velocity);
  impl_->state.orientation = domain(orientation);
  impl_->last_imu_timestamp_ns = sample.timestamp_ns;
  impl_->diagnostics.last_imu_timestamp_ns = sample.timestamp_ns;
  impl_->update_quality();
}

void VisualInertialEkf::update_visual(const VisualPoseMeasurement& measurement) {
  if (!impl_->is_initialised) throw std::logic_error("estimator is not initialised");
  const std::int64_t corrected_timestamp =
      measurement.timestamp_ns + impl_->calibration.camera_imu_time_offset_ns;
  if (corrected_timestamp < impl_->diagnostics.last_camera_timestamp_ns ||
      corrected_timestamp > impl_->last_imu_timestamp_ns)
    throw std::invalid_argument("visual timestamp is outside the propagated estimator interval");
  if (!(measurement.position_std_m > 0.0 && measurement.orientation_std_rad > 0.0) ||
      !std::isfinite(measurement.position_std_m) ||
      !std::isfinite(measurement.orientation_std_rad) ||
      !(measurement.inlier_ratio >= 0.0 && measurement.inlier_ratio <= 1.0))
    throw std::invalid_argument("invalid visual measurement uncertainty");

  Eigen::Matrix<double, 6, 1> innovation;
  innovation.head<3>() = eigen(measurement.position_world_m) - eigen(impl_->state.position_m);
  innovation.tail<3>() = quaternion_residual(
      eigen(impl_->state.orientation), eigen(measurement.orientation_world_body));
  if (!innovation.allFinite()) throw std::invalid_argument("non-finite visual innovation");

  Eigen::Matrix<double, 6, 15> observation = Eigen::Matrix<double, 6, 15>::Zero();
  observation.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  observation.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 6, 6> measurement_noise = Eigen::Matrix<double, 6, 6>::Zero();
  measurement_noise.diagonal().head<3>().setConstant(
      measurement.position_std_m * measurement.position_std_m);
  measurement_noise.diagonal().tail<3>().setConstant(
      measurement.orientation_std_rad * measurement.orientation_std_rad);
  const Eigen::Matrix<double, 6, 6> innovation_covariance =
      observation * impl_->covariance * observation.transpose() + measurement_noise;
  const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> decomposition(innovation_covariance);
  if (decomposition.info() != Eigen::Success || !decomposition.isPositive())
    throw std::runtime_error("visual innovation covariance is not positive definite");
  const Eigen::Matrix<double, 15, 6> gain =
      impl_->covariance * observation.transpose() *
      decomposition.solve(Eigen::Matrix<double, 6, 6>::Identity());
  const Vector15 correction = gain * innovation;
  const double innovation_nis = innovation.dot(decomposition.solve(innovation));

  impl_->state.position_m = domain(eigen(impl_->state.position_m) + correction.segment<3>(0));
  impl_->state.velocity_m_s = domain(eigen(impl_->state.velocity_m_s) + correction.segment<3>(3));
  const Eigen::Vector3d rotation_correction = correction.segment<3>(6);
  const double rotation_angle = rotation_correction.norm();
  const Eigen::Quaterniond delta = rotation_angle < 1e-12
      ? Eigen::Quaterniond(1.0, rotation_correction.x() * 0.5,
                           rotation_correction.y() * 0.5, rotation_correction.z() * 0.5)
      : Eigen::Quaterniond(Eigen::AngleAxisd(rotation_angle, rotation_correction.normalized()));
  impl_->state.orientation = domain((eigen(impl_->state.orientation) * delta).normalized());
  impl_->state.bias.gyro_rad_s =
      domain(eigen(impl_->state.bias.gyro_rad_s) + correction.segment<3>(9));
  impl_->state.bias.accel_m_s2 =
      domain(eigen(impl_->state.bias.accel_m_s2) + correction.segment<3>(12));

  const Matrix15 identity = Matrix15::Identity();
  const Matrix15 residual = identity - gain * observation;
  impl_->covariance = residual * impl_->covariance * residual.transpose() +
                      gain * measurement_noise * gain.transpose();
  impl_->covariance = 0.5 * (impl_->covariance + impl_->covariance.transpose());
  impl_->diagnostics.position_innovation_norm_m = innovation.head<3>().norm();
  impl_->diagnostics.orientation_innovation_norm_rad = innovation.tail<3>().norm();
  impl_->diagnostics.visual_innovation_nis = innovation_nis;
  impl_->diagnostics.tracked_features = measurement.tracked_features;
  impl_->diagnostics.inlier_ratio = measurement.inlier_ratio;
  impl_->diagnostics.last_camera_timestamp_ns = corrected_timestamp;
  impl_->diagnostics.camera_imu_residual_ms =
      static_cast<double>(impl_->last_imu_timestamp_ns - corrected_timestamp) * 1e-6;
  impl_->update_quality();
  impl_->state.tracking = impl_->assess().state;
}

void VisualInertialEkf::reset() {
  const std::uint64_t reset_count = impl_->state.reset_count + 1;
  impl_->state = {};
  impl_->state.tracking = TrackingState::RESETTING;
  impl_->state.reset_count = reset_count;
  impl_->diagnostics = {};
  impl_->last_imu_timestamp_ns = 0;
  impl_->is_initialised = false;
  impl_->reset_covariance();
  impl_->update_quality();
}

bool VisualInertialEkf::initialised() const { return impl_->is_initialised; }
const EstimatorState& VisualInertialEkf::state() const { return impl_->state; }
const EstimatorDiagnostics& VisualInertialEkf::diagnostics() const { return impl_->diagnostics; }
QualityAssessment VisualInertialEkf::quality_assessment() const { return impl_->assess(); }

std::array<double, 15> VisualInertialEkf::covariance_diagonal() const {
  std::array<double, 15> result{};
  for (std::size_t index = 0; index < result.size(); ++index)
    result[index] = impl_->covariance(static_cast<Eigen::Index>(index),
                                      static_cast<Eigen::Index>(index));
  return result;
}

}  // namespace vionav
