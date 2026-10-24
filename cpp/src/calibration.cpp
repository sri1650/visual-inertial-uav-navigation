#include "vionav/calibration.hpp"
#include <cmath>
#include <stdexcept>

namespace vionav {

void CameraIntrinsics::validate() const {
  if (!(std::isfinite(fx)&&std::isfinite(fy)&&std::isfinite(cx)&&std::isfinite(cy)))
    throw std::invalid_argument("non-finite camera intrinsics");
  if (fx <= 0 || fy <= 0) throw std::invalid_argument("focal lengths must be positive");
  if (width <= 0 || height <= 0) throw std::invalid_argument("image dimensions must be positive");
  if (cx < 0 || cx >= width || cy < 0 || cy >= height)
    throw std::invalid_argument("principal point outside image");
}

void RigidTransform::validate(double tol) const {
  for (int i=0;i<3;++i) {
    double n=0.0;
    for (int k=0;k<3;++k) {
      if (!std::isfinite(rotation[i][k])) throw std::invalid_argument("non-finite rotation");
      n += rotation[i][k]*rotation[i][k];
    }
    if (std::abs(std::sqrt(n)-1.0) > tol)
      throw std::invalid_argument("rotation row not unit");
  }
  for (int i=0;i<3;++i)
    for (int j=i+1;j<3;++j) {
      double d=0.0;
      for (int k=0;k<3;++k) d += rotation[i][k]*rotation[j][k];
      if (std::abs(d)>tol) throw std::invalid_argument("rotation rows not orthogonal");
    }
  const auto& r=rotation;
  const double det =
      r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])
    - r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])
    + r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]);
  if (std::abs(det-1.0)>5*tol) throw std::invalid_argument("rotation determinant must be +1");
  if (!(std::isfinite(translation_m.x)&&std::isfinite(translation_m.y)&&std::isfinite(translation_m.z)))
    throw std::invalid_argument("non-finite translation");
}

void SensorCalibration::validate() const {
  camera.validate();
  T_body_camera.validate();
  T_body_imu.validate();
}

} // namespace vionav
