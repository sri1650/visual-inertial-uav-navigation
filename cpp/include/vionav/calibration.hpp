#pragma once
#include "vionav/types.hpp"
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace vionav {

struct CameraIntrinsics {
  double fx{}, fy{}, cx{}, cy{};
  int width{}, height{};
  std::string distortion_model{"radtan"};
  std::vector<double> distortion;

  void validate() const;
};

struct RigidTransform {
  std::array<std::array<double,3>,3> rotation{{
    {{1,0,0}}, {{0,1,0}}, {{0,0,1}}
  }};
  Vec3 translation_m{};

  void validate(double tol=1e-3) const;
};

struct SensorCalibration {
  CameraIntrinsics camera;
  RigidTransform T_body_camera;
  RigidTransform T_body_imu;
  std::int64_t camera_imu_time_offset_ns{};

  void validate() const;
};

} // namespace vionav
