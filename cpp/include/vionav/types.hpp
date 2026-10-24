#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace vionav {

struct Vec3 {
  double x{}, y{}, z{};

  Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
  Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
  Vec3 operator/(double s) const {
    if (s == 0.0) throw std::invalid_argument("division by zero");
    return {x/s, y/s, z/s};
  }
  Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
  double norm() const { return std::sqrt(x*x + y*y + z*z); }
};

inline double dot(const Vec3& a, const Vec3& b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

struct Quaternion {
  double w{1.0}, x{}, y{}, z{};

  Quaternion normalized() const {
    const double n = std::sqrt(w*w+x*x+y*y+z*z);
    if (n <= 0.0) throw std::invalid_argument("zero quaternion");
    return {w/n,x/n,y/n,z/n};
  }

  Quaternion conjugate() const { return {w,-x,-y,-z}; }

  Quaternion operator*(const Quaternion& q) const {
    return {
      w*q.w - x*q.x - y*q.y - z*q.z,
      w*q.x + x*q.w + y*q.z - z*q.y,
      w*q.y - x*q.z + y*q.w + z*q.x,
      w*q.z + x*q.y - y*q.x + z*q.w
    };
  }

  Vec3 rotate(const Vec3& v) const {
    const Quaternion qn = normalized();
    Quaternion p{0.0,v.x,v.y,v.z};
    Quaternion r = qn * p * qn.conjugate();
    return {r.x,r.y,r.z};
  }

  static Quaternion from_rotation_vector(const Vec3& rv) {
    const double a = rv.norm();
    if (a < 1e-12) {
      return Quaternion{1.0, rv.x*0.5, rv.y*0.5, rv.z*0.5}.normalized();
    }
    const double h = 0.5*a;
    const double s = std::sin(h)/a;
    return Quaternion{std::cos(h), rv.x*s, rv.y*s, rv.z*s};
  }
};

enum class TrackingState {
  INITIALISING,
  TRACKING,
  DEGRADED,
  LOST,
  RESETTING
};

struct CameraSample {
  std::int64_t timestamp_ns{};
  std::string frame_id;
  int width{};
  int height{};
};

struct ImuSample {
  std::int64_t timestamp_ns{};
  Vec3 angular_velocity_rad_s;
  Vec3 linear_acceleration_m_s2;
};

struct ImuBias {
  Vec3 gyro_rad_s;
  Vec3 accel_m_s2;
};

}  // namespace vionav
